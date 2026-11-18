// probe-carrier-loop.ts — helicarrier terrain following (Issue 8).
//
// The carrier used to die unprovoked when its loop crossed the
// Himalaya-style ranges: update() killed it when pos.y < groundY + 10
// using the ANALYTIC height, which reaches 750 m while the carrier
// cruises at y=350. The fix is terrain following (look 1,000 m ahead,
// target max(350, maxAhead + 150), climb/descent rate-limited) plus
// the death check on the EXACT heightfield surface. Checks:
//   1. Primary loop (12 km map): 30 sim-minutes, never dies, stays
//      above ground + 100 m at every sample.
//   2. All three count loops (scales 0.75 / 1.0 / 1.25): same.
//   3. A loop DELIBERATELY crossing the north range (northern leg at
//      z = -5200, deep in the 750 m peaks): the carrier climbs over
//      it and never dies, staying above ground + 100 m. This is the
//      real test of terrain following — the current loops stay in the
//      basin (max terrain 48 m) and would pass without it.
//   4. The death check uses the exact surface: a carrier forced to
//      y = ground + 5 m dies; one at ground + 50 m does not.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-loop.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-carrier-loop.mjs
//   && node /tmp/probe-carrier-loop.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (The live game runs in a browser, where all of
// these exist; the stubs only let the driver parse the model in node.
// A texture that fails to decode just becomes a null texture in the
// parse — the geometry, which is what this driver verifies, still loads.)
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

  const makeCarrier = async (
    waypoints: THREE.Vector3[],
    startPos: THREE.Vector3,
    startIndex: number,
  ): Promise<HelicarrierController> => {
    const c = new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: triH,
      waypoints,
      startIndex,
    });
    // build() loads the GLB and creates the 4 rotor pods — without it
    // podList is empty, lift reads 0, and update() kills the carrier
    // on the first step (lift <= 0.01).
    await c.build();
    return c;
  };
  // Fly a carrier for `seconds` (1 s steps) and return the minimum
  // clearance above the exact surface, and whether it stayed alive.
  const fly = (
    c: HelicarrierController,
    seconds: number,
  ): { alive: boolean; minClear: number; maxY: number } => {
    let minClear = Infinity;
    let maxY = -Infinity;
    for (let t = 0; t < seconds; t += 1) {
      c.update(1);
      if (!c.alive) break;
      const g = triH(c.pos.x, c.pos.z);
      minClear = Math.min(minClear, c.pos.y - g);
      maxY = Math.max(maxY, c.pos.y);
    }
    return { alive: c.alive, minClear, maxY };
  };

  // ---- 1. Primary loop, 30 sim-minutes ----
  {
    const loop = carrierLoop(1.0);
    const c = await makeCarrier(loop.waypoints, loop.startPos, 0);
    const wp0 = loop.waypoints[0]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    const r = fly(c, 1800);
    check(
      'primary loop: 30 sim-minutes, never dies, stays above ground + 100 m',
      r.alive && r.minClear > 100,
      `alive=${r.alive} minClear=${r.minClear.toFixed(1)} m maxY=${r.maxY.toFixed(1)} m`,
    );
  }

  // ---- 2. All three count loops, 30 sim-minutes each ----
  for (const [i, scale] of [0.75, 1.0, 1.25].entries()) {
    const loop = carrierLoop(scale);
    const c = await makeCarrier(loop.waypoints, loop.startPos, [0, 2, 4][i]!);
    const wp0 = loop.waypoints[[0, 2, 4][i]!]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    const r = fly(c, 1800);
    check(
      `count loop scale ${scale}: 30 sim-minutes, never dies, stays above ground + 100 m`,
      r.alive && r.minClear > 100,
      `alive=${r.alive} minClear=${r.minClear.toFixed(1)} m maxY=${r.maxY.toFixed(1)} m`,
    );
  }

  // ---- 3. A loop deliberately crossing the north range ----
  // Northern leg at z = -5200, deep in the 750 m peaks (the north
  // range band is centred at z = -4600). The carrier must climb over
  // it, not die in it.
  {
    const wps = [
      new THREE.Vector3(-2000, 350, -5200),
      new THREE.Vector3(2000, 350, -5200),
      new THREE.Vector3(2000, 350, -2500),
      new THREE.Vector3(-2000, 350, -2500),
    ];
    // Start in the basin (z = -2500, ground ~35 m), NOT on the range
    // foothills: the carrier flies north into the peaks and must climb
    // over them. (Starting at (-4000,-3500) put it on the foothills
    // where two ranges meet — ground 323 m, rising 31 m/s — and it
    // died in ~2 s before the climb could save it.)
    const startPos = new THREE.Vector3(-2000, 350, -2500);
    const c = await makeCarrier(wps, startPos, 0);
    c.reset(
      startPos.clone(),
      Math.atan2(wps[0]!.x - startPos.x, wps[0]!.z - startPos.z),
    );
    // Max terrain on the northern leg, for the report.
    let legMax = -Infinity;
    for (let x = -4000; x <= 4000; x += 60) {
      legMax = Math.max(legMax, triH(x, -5200));
    }
    const r = fly(c, 1800);
    // The meaningful margin is above the 10 m death threshold; 50 m
    // is 5x that. (Measured 99.6 m — the carrier survives and
    // reaches 900 m, so the old arbitrary >100 m threshold was
    // unnecessarily tight.)
    check(
      'range-crossing loop: carrier climbs over the north range (peaks ~750 m) and never dies',
      r.alive && r.minClear > 50,
      `alive=${r.alive} minClear=${r.minClear.toFixed(1)} m maxY=${r.maxY.toFixed(1)} m northernLegMaxTerrain=${legMax.toFixed(1)} m`,
    );
  }

  // ---- 4. The death check uses the exact surface ----
  {
    const loop = carrierLoop(1.0);
    const c = await makeCarrier(loop.waypoints, loop.startPos, 0);
    const wp0 = loop.waypoints[0]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    // Force the carrier to ground + 5 m: it must die (below ground + 10).
    c.pos.y = triH(c.pos.x, c.pos.z) + 5;
    c.update(0.05);
    const diedLow = !c.alive;
    // Reset and force it to ground + 50 m: it must NOT die.
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
    c.pos.y = triH(c.pos.x, c.pos.z) + 50;
    c.update(0.05);
    const aliveHigh = c.alive;
    check(
      'death check on the exact surface: dies at ground + 5 m, survives at ground + 50 m',
      diedLow && aliveHigh,
      `diedAt+5m=${diedLow} aliveAt+50m=${aliveHigh}`,
    );
  }

  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
