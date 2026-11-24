// probe-carrier-live.ts — pass 8, item 3: CARRIER DOWN false alarm
// (~40 s into EVERY round, 0 enemies, carrier visibly flying).
//
// probe-carrier-loop-dt (primary, dt=1/60, no shift) passes with
// min clearance 302.7 m. This probe adds the live elements it
// misses: the frame-1 origin shift (driven by a parked Alpha player),
// the live variable-dt pattern (16–250 ms frames), and ALL THREE
// loop slots 0 / 1 / 2 (shared loop, phase-offset ~1/3 lap,
// exactly as main.ts builds them). It logs per-carrier pos.y vs
// groundY vs groundAhead(2500) every 10 s and the minimum clearance
// per carrier, and reports any terrain-net death with the carrier
// index and the numbers at the moment of death.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-live.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-carrier-live.mjs
//   && node /tmp/probe-carrier-live.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  HelicarrierController,
  carrierLoop,
  CARRIER_ALT,
} from '../src/entities/HelicarrierController';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const heightAtHf = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const scene = new THREE.Scene();
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  const origin = new FloatingOrigin(2000);
  // Shift-aware heightAt, exactly like worldGen.heightfieldHeightAt.
  const heightAtLive = (x: number, z: number): number =>
    heightAtHf(x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const carriers: HelicarrierController[] = [];
  const deaths: string[] = [];
  const minClear: number[] = [Infinity, Infinity, Infinity];
  const minClearAt: string[] = ['', '', ''];
  const playerPos = new THREE.Vector3(-2800, 52, -1800); // parked Alpha player
  // Register it so applyShift moves it (like the live player body):
  // otherwise shiftIfNeeded would fire the same shift every frame.
  origin.registerVector(playerPos);
  for (let i = 0; i < 3; i++) {
    const loop = carrierLoop(i);
    const c = new HelicarrierController(scene, world, {
      heightAt: heightAtLive,
      waypoints: loop.waypoints,
      startIndex: loop.startIndex,
      onCarrierDown: () => {
        deaths.push(
          `carrier ${i} (slot ${i}) died: pos=(${c.pos.x.toFixed(0)}, ${c.pos.y.toFixed(0)}, ${c.pos.z.toFixed(0)}) ground=${heightAtLive(c.pos.x, c.pos.z).toFixed(0)} groundAhead(2500)=${c.lookAheadGround.toFixed(0)}`,
        );
      },
    });
    // Build (loads the SHIELD GLB, creates the deck body) and register
    // with the origin exactly as main.ts does, then the per-round reset.
    await c.build();
    c.registerWithFloatingOrigin(origin);
    const wp0 = loop.waypoints[loop.startIndex]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    carriers.push(c);
  }
  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  for (let f = 0; f < 180 * 60; f++) {
    const t = f / 60;
    const rawDt = dtPattern[f % dtPattern.length]!;
    const sub = Math.max(1, Math.ceil(rawDt / 0.05));
    const dt = rawDt / sub;
    for (let s = 0; s < sub; s++) {
      for (const c of carriers) c.update(dt);
    }
    // End of frame: the origin shift, driven by the parked player
    // (fires at frame 0, like the live game).
    const shift = origin.shiftIfNeeded(playerPos);
    if (shift) origin.applyShift(shift);
    for (let i = 0; i < 3; i++) {
      const c = carriers[i]!;
      const g = heightAtLive(c.pos.x, c.pos.z);
      const clearance = c.pos.y - g;
      if (clearance < minClear[i]!) {
        minClear[i] = clearance;
        minClearAt[i] = `t=${t.toFixed(1)}s pos=(${c.pos.x.toFixed(0)},${c.pos.y.toFixed(0)},${c.pos.z.toFixed(0)}) ground=${g.toFixed(0)} ahead=${c.lookAheadGround.toFixed(0)}`;
      }
    }
    if (f % 600 === 0) {
      console.log(
        `  [t=${t.toFixed(0)}s] ` +
          carriers
            .map(
              (c, i) =>
                `c${i} y=${c.pos.y.toFixed(0)} gnd=${heightAtLive(c.pos.x, c.pos.z).toFixed(0)} ahead=${c.lookAheadGround.toFixed(0)} alive=${c.alive}`,
            )
            .join(' '),
      );
    }
    if (deaths.length > 0) break;
  }
  for (let i = 0; i < 3; i++) {
    console.log(`  carrier ${i} (slot ${i}): min clearance ${minClear[i]!.toFixed(1)} m at ${minClearAt[i]}`);
  }
  for (const d of deaths) console.log(`  DEATH: ${d}`);
  check('no carrier dies in 180 s (all 3 loops, live dt + origin shift)', deaths.length === 0, deaths.join('; '));
  for (let i = 0; i < 3; i++) {
    check(`carrier ${i} keeps >= 100 m clearance`, minClear[i]! >= 100, `min ${minClear[i]!.toFixed(1)} m at ${minClearAt[i]}`);
  }
  console.log(failures === 0 ? 'ALL CARRIER-LIVE CHECKS PASSED' : `${failures} CARRIER-LIVE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
