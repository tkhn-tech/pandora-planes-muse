// probe-deck-measure.ts — measure the SHIELD deck rects + deckHeightAt
// at the live 120 m aft deck spawn point, through the real model build.
// Usage: esbuild scripts/probe-deck-measure.ts --bundle --platform=node
//   --format=esm --outfile=/tmp/probe-deck-measure.mjs && node /tmp/probe-deck-measure.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(type: string, init?: { lengthComputable?: boolean; loaded?: number; total?: number }) {
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
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import {
  generateHeights,
  heightfieldHeightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

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
  const scene = new THREE.Scene();
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
  });
  await carrier.build();
  console.log('deckLocalY=' + carrier.deckLocalY.toFixed(3));
  console.log('deckTopY=' + carrier.deckTopY.toFixed(3));
  console.log('deckRects=' + JSON.stringify(carrier.deckRects));
  console.log(
    'pos x=' + carrier.pos.x.toFixed(1) + ' y=' + carrier.pos.y.toFixed(1) +
    ' z=' + carrier.pos.z.toFixed(1) + ' yaw=' + carrier.yaw.toFixed(3),
  );
  const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
  const sx = carrier.pos.x - fwd.x * 120;
  const sz = carrier.pos.z - fwd.z * 120;
  const local = carrier.carrierLocal(new THREE.Vector3(sx, 0, sz));
  console.log('spawn120 local x=' + local.x.toFixed(1) + ' z=' + local.z.toFixed(1));
  const dh = carrier.deckHeightAt(sx, sz);
  console.log('deckHeightAt(spawn120)=' + dh.toFixed(2) + ' deckTopY=' + carrier.deckTopY.toFixed(2));
  if (dh === -Infinity) {
    console.log('FAIL deckHeightAt is -Infinity at the live 120 m aft deck spawn — the jet spawns OFF the deck');
    process.exit(1);
  }
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (let lz = -200; lz <= 250; lz += 5) {
    const p = new THREE.Vector3(
      carrier.pos.x + Math.sin(carrier.yaw) * lz, 0,
      carrier.pos.z + Math.cos(carrier.yaw) * lz,
    );
    const h = carrier.deckHeightAt(p.x, p.z);
    if (h > -Infinity) {
      if (lz < minZ) minZ = lz;
      if (lz > maxZ) maxZ = lz;
    }
  }
  console.log('deck centreline live range: local z ' + minZ + ' .. ' + maxZ);
  console.log('PASS deck measurement');
}

void main();
