// probe-carrier-loop-dt.ts — pass 6, item 4: the existing
// probe-carrier-loop drives the carrier with dt=1 (1 s steps). The live
// game uses dt<=0.05 substeps. Re-run the primary loop at live dt
// (1/60) for 180 s and report any death / minimum clearance.
// Usage: esbuild scripts/probe-carrier-loop-dt.ts --bundle --platform=node
//   --format=esm --outfile=/tmp/probe-carrier-loop-dt.mjs && node /tmp/probe-carrier-loop-dt.mjs
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  HelicarrierController,
  carrierLoop,
  CARRIER_ALT,
} from '../src/entities/HelicarrierController';
import {
  generateHeights,
  heightfieldHeightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({ seed: 1337, chunkX: 0, chunkZ: 0, size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1 });
  const res = TERRAIN_SEGMENTS + 1;
  const heightAt = (x: number, z: number): number => heightfieldHeightAt(heights, res, x, z);
  const scene = new THREE.Scene();
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  const loop = carrierLoop(0);
  let downAt = -1;
  let downDetail = '';
  let t = 0;
  const carrier = new HelicarrierController(scene, world, {
    heightAt,
    waypoints: loop.waypoints,
    startIndex: 0,
    onCarrierDown: () => {
      if (downAt < 0) {
        downAt = t;
        downDetail = `pos=(${carrier.pos.x.toFixed(0)}, ${carrier.pos.y.toFixed(0)}, ${carrier.pos.z.toFixed(0)}) ground=${heightAt(carrier.pos.x, carrier.pos.z).toFixed(0)}`;
      }
    },
  });
  carrier.reset(new THREE.Vector3(-1900, CARRIER_ALT, -1800), Math.PI / 2);
  let minClearance = Infinity;
  let minClearAt = '';
  for (let i = 0; i < 180 * 60; i++) {
    t = i / 60;
    carrier.update(1 / 60);
    const g = heightAt(carrier.pos.x, carrier.pos.z);
    const clearance = carrier.pos.y - g;
    if (clearance < minClearance) {
      minClearance = clearance;
      minClearAt = `t=${t.toFixed(1)}s pos=(${carrier.pos.x.toFixed(0)},${carrier.pos.y.toFixed(0)},${carrier.pos.z.toFixed(0)}) ground=${g.toFixed(0)}`;
    }
    if (!carrier.alive) break;
  }
  console.log(`min clearance over 180 s at dt=1/60: ${minClearance.toFixed(1)} m at ${minClearAt}`);
  if (downAt >= 0) {
    console.log(`FAIL  carrier died (CARRIER DOWN) at t=${downAt.toFixed(1)}s — ${downDetail}`);
    process.exit(1);
  }
  console.log('PASS  carrier completes 180 s at live dt without dying');
}

void main();
