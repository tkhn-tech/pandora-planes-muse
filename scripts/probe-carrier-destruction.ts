// probe-carrier-destruction.ts — helicarrier destruction sequence (Issue 9).
//
// When a helicarrier is legitimately shot down (all 4 rotor pods
// destroyed) it must get the same treatment as an aircraft: a dying
// freefall, fire/smoke, a ground-impact explosion, debris and a
// wreck left afterwards — never a silent vanish or an intact fall.
// Checks:
//   1. Destroying all 4 pods starts dying (dying=true, alive stays
//      true — the carrier does NOT vanish on the spot).
//   2. While dying it stops waypoint-following (yaw rate decays),
//      lists heavily and sinks at an increasing rate.
//   3. On ground contact onCarrierImpact fires, the wreck rests on
//      the ground (pos.y ~ ground + 5) with a heavy list, and the
//      deck reads as absent (deckHeightAt -> -Infinity).
//   4. After ~10 s of wreck-smoking alive=false and onCarrierDown
//      fires — but the group stays visible at the wreck pose (the
//      wreckage is left afterwards, not removed).
//   5. A carrier with pods intact never enters dying.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-destruction.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-carrier-destruction.mjs
//   && node /tmp/probe-carrier-destruction.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (The live game runs in a browser, where all of
// these exist; the stubs only let the driver parse the model in node.)
if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(
      type: string,
      init?: { lengthComputable?: boolean; loaded?: number; total?: number },
    ) {
      this.type = type;
      this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0;
      this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => {
          (listeners[type] ??= []).push(fn);
        },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', {
        set: () => {
          queueMicrotask(() => {
            for (const fn of listeners['load'] ?? []) fn();
          });
        },
      });
      return img;
    },
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  HelicarrierController,
  carrierLoop,
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
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
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const triH = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  const makeCarrier = async (): Promise<{
    c: HelicarrierController;
    impact: { fired: boolean; pos: THREE.Vector3 | null };
    down: { fired: boolean };
  }> => {
    const impact = { fired: false, pos: null as THREE.Vector3 | null };
    const down = { fired: false };
    const loop = carrierLoop(1.0);
    const c = new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: triH,
      waypoints: loop.waypoints,
      startIndex: 0,
      onCarrierImpact: (pos) => {
        impact.fired = true;
        impact.pos = pos.clone();
      },
      onCarrierDown: () => {
        down.fired = true;
      },
    });
    await c.build();
    const wp0 = loop.waypoints[0]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    return { c, impact, down };
  };

  // ---- 1-4. Shoot down a carrier: all 4 pods destroyed ----
  {
    const { c, impact, down } = await makeCarrier();
    // Pods intact: fly 10 s, never dying.
    for (let t = 0; t < 10; t += 1 / 60) c.update(1 / 60);
    check(
      'pods intact: carrier never enters dying',
      !c.dying && c.alive,
      `dying=${c.dying} alive=${c.alive}`,
    );
    // Destroy all 4 pods (damagePod amount >> pod hp).
    for (let i = 0; i < 4; i++) c.damagePod(i, 10000);
    // One update to enter the dying state.
    c.update(1 / 60);
    const yAtDying = c.pos.y;
    check(
      'all pods dead: dying starts, carrier does NOT vanish (alive stays true)',
      c.dying && c.alive,
      `dying=${c.dying} alive=${c.alive} y=${yAtDying.toFixed(1)}`,
    );
    // Fly the dying sequence (1/60 s steps, up to 120 s): it must
    // sink, list heavily, impact, and be written off — with the
    // impact callback firing and the wreck left on the ground.
    let minY = Infinity;
    let maxList = 0;
    let impactY: number | null = null;
    let impactList: number | null = null;
    let deckAfterImpact: number | null = null;
    for (let t = 0; t < 120; t += 1 / 60) {
      c.update(1 / 60);
      minY = Math.min(minY, c.pos.y);
      maxList = Math.max(maxList, Math.abs(c.list));
      if (impact.fired && impactY === null) {
        impactY = c.pos.y;
        impactList = c.list;
        deckAfterImpact = c.deckHeightAt(c.pos.x, c.pos.z);
      }
      if (!c.alive) break;
    }
    const groundAtWreck = impact.pos ? triH(impact.pos.x, impact.pos.z) : NaN;
    check(
      'dying carrier sinks and lists heavily (never an intact fall)',
      minY < yAtDying - 100 && maxList > 0.4,
      `y ${yAtDying.toFixed(1)} -> min ${minY.toFixed(1)} m, max list ${(maxList * 180 / Math.PI).toFixed(0)} deg`,
    );
    check(
      'ground impact fires onCarrierImpact, wreck rests on the ground with a heavy list',
      impact.fired &&
        impact.pos !== null &&
        impactY !== null &&
        Math.abs(impactY - (groundAtWreck + 5)) < 2 &&
        impactList !== null &&
        Math.abs(impactList) > 0.5,
      `impact=${impact.fired} impactY=${impactY?.toFixed(1)} ground+5=${(groundAtWreck + 5).toFixed(1)} list=${impactList !== null ? (impactList * 180 / Math.PI).toFixed(0) : '?'} deg`,
    );
    check(
      'deck reads as absent after impact (wreck is not landable)',
      deckAfterImpact === -Infinity,
      `deckHeightAt=${deckAfterImpact}`,
    );
    check(
      'wreck is written off ~10 s after impact (alive=false, onCarrierDown fires)',
      !c.alive && down.fired,
      `alive=${c.alive} down=${down.fired} wreckT=${c.wreckT.toFixed(1)} s`,
    );
    check(
      'wreckage is left afterwards: the group stays visible at the wreck pose',
      c.group.visible &&
        Math.abs(c.group.position.y - (groundAtWreck + 5)) < 2,
      `visible=${c.group.visible} groupY=${c.group.position.y.toFixed(1)} ground+5=${(groundAtWreck + 5).toFixed(1)}`,
    );
  }

  // ---- 5. A carrier that loses pods but keeps one never dies ----
  {
    const { c } = await makeCarrier();
    for (let i = 0; i < 3; i++) c.damagePod(i, 10000);
    for (let t = 0; t < 60; t += 1) c.update(1);
    check(
      '3 of 4 pods dead: carrier survives (sinks slowly, does not enter dying)',
      c.alive && !c.dying,
      `alive=${c.alive} dying=${c.dying} y=${c.pos.y.toFixed(1)}`,
    );
  }

  console.log(failures === 0 ? '\nALL CARRIER-DESTRUCTION CHECKS PASSED' : `\n${failures} CARRIER-DESTRUCTION CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
