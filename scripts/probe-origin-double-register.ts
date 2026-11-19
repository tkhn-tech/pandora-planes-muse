// probe-origin-double-register.ts — pass 9, items 2/3 (root cause):
// the floating-origin shift accumulator in PandoraWorldGenerator
// (worldGen._originShift, read by heightfieldHeightAt — the ground
// query behind the spawn clamp, the terrain safety net, the carrier
// terrain-following/death check and the ?debug=1 readout) grows at
// TWICE the real shift.
//
// Why: PandoraWorldGenerator's constructor calls
// registerWithFloatingOrigin(origin) when an origin is passed, and
// main.ts calls worldGen.registerWithFloatingOrigin(floatingOrigin)
// AGAIN after build(). Each call registers the _originShift callback
// (origin.registerCallback), and there is no dedup — so every shift
// bumps _originShift twice while floatingOrigin.cumulativeShift grows
// once. After a shift S, worldGen.originShift == 2S but
// cumulativeShift == S, so heightfieldHeightAt(x, z) samples the
// terrain at (x + 2S) instead of (x + S): the ground query reads a
// position a full shift away from the true one. That is the live
// Alpha spawn death (net sees terrain 161.2 at the jet's Alpha
// position), the carrier death (death check / debug gnd read 416
// where the true terrain is 43), the Charlie airborne spawn and the
// Bravo dead-end spawn — all of them are the height query sampling
// the wrong place, not the spawn placement (which probe-respawn-shift
// verifies is correct).
//
// The probe replicates the main.ts boot sequence exactly (construct
// with an origin, build, register again), applies a shift, and checks
// the invariant worldGen.originShift == floatingOrigin.cumulativeShift
// and that a height sample lands on the true terrain.
//
// Usage: node_modules/.bin/esbuild scripts/probe-origin-double-register.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-origin-double-register.mjs
//   && node /tmp/probe-origin-double-register.mjs

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
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
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
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { PandoraWorldGenerator } from '../src/world/PandoraWorldGenerator';
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
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  // main.ts boot sequence, verbatim: construct WITH the origin
  // (constructor registers the _originShift callback), build, then
  // register AGAIN (line ~291).
  const worldGen = new PandoraWorldGenerator(scene, world, origin);
  await worldGen.build(1337);
  worldGen.registerWithFloatingOrigin(origin);

  // The heights the generator built (same seed) for an independent
  // ground-truth sample.
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const trueHeightAt = (ox: number, oz: number): number =>
    heightfieldHeightAt(heights, res, ox, oz);

  const shifts: Array<[number, number]> = [
    [-2800, -1800], // Alpha spawn shift (the live Alpha death)
    [2000, 1500],
    [4000, 3000],
  ];
  for (const [sx, sz] of shifts) {
    origin.reset();
    // Apply the shift the way animate() does (shiftIfNeeded +
    // applyShift); the generator's callback must track it exactly.
    origin.applyShift(new THREE.Vector3(sx, 0, sz));
    const cum = origin.cumulativeShift;
    const gen = worldGen.originShift;
    check(
      `shift (${sx},${sz}): worldGen.originShift tracks cumulativeShift (no double-count)`,
      gen.x === cum.x && gen.z === cum.z,
      `cumulativeShift=(${cum.x},${cum.z}) worldGen.originShift=(${gen.x},${gen.z})`,
    );
    // Height sample: a scene point whose true original position is
    // Alpha (-2800,-1800). The generator must sample the Alpha
    // terrain (50 m), not the terrain a full shift away.
    const sceneX = -2800 - cum.x;
    const sceneZ = -1800 - cum.z;
    const sampled = worldGen.heightfieldHeightAt(sceneX, sceneZ);
    const truth = trueHeightAt(-2800, -1800);
    check(
      `shift (${sx},${sz}): heightfieldHeightAt at Alpha scene point samples Alpha (50 m)`,
      Math.abs(sampled - truth) < 0.5,
      `sampled=${sampled.toFixed(1)} truth(Alpha)=${truth.toFixed(1)} (double-count would sample ${trueHeightAt(-2800 + cum.x, -1800 + cum.z).toFixed(1)})`,
    );
    // The carrier spawn point (-1900,-1800): true terrain 43 m.
    const cSceneX = -1900 - cum.x;
    const cSceneZ = -1800 - cum.z;
    const cSampled = worldGen.heightfieldHeightAt(cSceneX, cSceneZ);
    const cTruth = trueHeightAt(-1900, -1800);
    check(
      `shift (${sx},${sz}): heightfieldHeightAt at carrier-spawn scene point samples 43 m`,
      Math.abs(cSampled - cTruth) < 0.5,
      `sampled=${cSampled.toFixed(1)} truth=${cTruth.toFixed(1)}`,
    );
  }
  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
