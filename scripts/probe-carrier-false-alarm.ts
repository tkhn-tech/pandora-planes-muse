// probe-carrier-false-alarm.ts — CARRIER DOWN false alarm at round
// start (pass 5, item 4).
//
// The v4 play-test: a "CARRIER DOWN" banner fires within seconds of
// EVERY round start (00:01-00:09), 0 enemies, while the carrier is
// visibly still flying. The only paths that toast it are the
// carrier's onCarrierImpact / onCarrierDown callbacks (instant
// terrain-death in update(), or the dying sequence). This probe
// replicates the live startRound + animate-loop order with the real
// HelicarrierController (3 carriers built the way main.ts does,
// count=1, reset + deactivate, update() at 60 Hz, a round restart
// at t=7 s) and requires that NEITHER callback fires in the first
// 15 s and that the primary carrier stays alive and flying.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-false-alarm.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-carrier-false-alarm.mjs
//   && node /tmp/probe-carrier-false-alarm.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  HelicarrierController,
  carrierLoop,
} from '../src/entities/HelicarrierController';
// Same values as main.ts (not exported from the controller).
const CARRIER_LOOP_SCALES = [1.0, 0.75, 1.25];
const CARRIER_START_INDEXES = [0, 2, 4];

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const origin = new FloatingOrigin();
  const hAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const events: string[] = [];
  const carriers = [0, 1, 2].map((i) => {
    const loop = carrierLoop(CARRIER_LOOP_SCALES[i]!);
    return new HelicarrierController(scene, world, {
      modelGlbUri: undefined,
      heightAt: hAt,
      waypoints: loop.waypoints,
      startIndex: CARRIER_START_INDEXES[i]!,
      onCarrierImpact: () => events.push(`impact@${i}`),
      onCarrierDown: () => events.push(`down@${i}`),
    });
  });
  await Promise.all(
    carriers.map(async (c) => {
      await c.build();
      c.registerWithFloatingOrigin(origin);
    }),
  );
  const startRound = (count: number): void => {
    carriers.forEach((c, i) => {
      if (i < count) {
        const loop = carrierLoop(CARRIER_LOOP_SCALES[i]!);
        const wp0 = loop.waypoints[CARRIER_START_INDEXES[i]!]!;
        c.reset(
          loop.startPos.clone(),
          Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
        );
      } else {
        c.deactivate();
      }
    });
  };
  startRound(1);
  const primary = carriers[0]!;
  // 15 s at 60 Hz, with a round restart (Apply & Restart) at t=7 s.
  for (let f = 0; f < 900; f++) {
    if (f === 420) startRound(1);
    for (const c of carriers) c.update(1 / 60);
  }
  check(
    'no CARRIER DOWN callback fires in the first 15 s (incl. a round restart at t=7 s)',
    events.length === 0,
    events.length ? `fired: ${events.join(', ')}` : 'no callbacks fired',
  );
  check(
    'primary carrier stays alive and flying after 15 s',
    primary.alive && !primary.dying && primary.pos.y > 300,
    `alive=${primary.alive} dying=${primary.dying} y=${primary.pos.y.toFixed(1)}`,
  );
  check(
    'extra carriers stay deactivated (no false alarm from a non-primary carrier)',
    !carriers[1]!.active && !carriers[2]!.active,
    `active1=${carriers[1]!.active} active2=${carriers[2]!.active}`,
  );

  // ---- the two round-start guards (pass 5, item 4) ----
  // (a) A carrier whose model has NOT built yet has an empty
  // podList — that used to read as lift 0 ("all pods dead") and
  // entered dying on the first update(). It must stay alive.
  {
    const unbuilt = new HelicarrierController(scene, world, {
      heightAt: hAt,
      waypoints: carrierLoop(1.0).waypoints,
      startIndex: 0,
      onCarrierDown: () => events.push('down@unbuilt'),
      onCarrierImpact: () => events.push('impact@unbuilt'),
    });
    unbuilt.reset();
    for (let f = 0; f < 120; f++) unbuilt.update(1 / 60);
    check(
      'unbuilt carrier (empty podList) does not enter dying',
      unbuilt.alive && !unbuilt.dying,
      `alive=${unbuilt.alive} dying=${unbuilt.dying}`,
    );
  }
  // (b) A carrier reset over very high ground (constant 1000 m)
  // must survive the 5 s grace window (terrain-following needs
  // time to climb) and the safety net must still fire afterwards
  // if it is still below ground+10.
  {
    const high = new HelicarrierController(scene, world, {
      heightAt: () => 1000,
      waypoints: carrierLoop(1.0).waypoints,
      startIndex: 0,
      onCarrierDown: () => events.push('down@high'),
      onCarrierImpact: () => events.push('impact@high'),
    });
    await high.build();
    high.reset();
    for (let f = 0; f < 180; f++) high.update(1 / 60); // 3 s
    const aliveAt3s = high.alive;
    for (let f = 0; f < 240; f++) high.update(1 / 60); // to 7 s
    check(
      'carrier over high ground survives the 5 s round-start grace window',
      aliveAt3s,
      `alive at 3 s=${aliveAt3s} y=${high.pos.y.toFixed(1)}`,
    );
    check(
      'safety net still fires after the grace window if still below ground+10',
      !high.alive,
      `alive at 7 s=${high.alive} y=${high.pos.y.toFixed(1)} ground+10=1010`,
    );
  }
  console.log(failures === 0 ? '\nALL CARRIER-FALSE-ALARM CHECKS PASSED' : `\n${failures} CARRIER-FALSE-ALARM CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
