// probe-infinite-flight.ts — pass 11 verification: the map is infinite
// and procedurally generating.
//
// Drives TerrainStreamer DIRECTLY (not through main.ts): a kinematic
// flyer goes 40 km east from Alpha at 150 m/s, y=500, then 21 km back
// west and 21 km east again (82 km total), through origin shifts and
// chunk boundaries, with the REAL FloatingOrigin (shift check at frame
// start, exactly like main.ts animate()) and a REAL Rapier world
// (raycasts prove the colliders are really under the jet).
//
// Asserts:
//  1. No visible position snap: original-frame position is continuous
//     across every shift (max per-step delta == v*dt), scene position
//     stays bounded near the origin.
//  2. Continuous terrain the whole way: heightAt finite and in range,
//     the chunk under the player always has geometry (or is a central
//     chunk owned by the generator), and a Rapier raycast down agrees
//     with heightAt (collider == query surface).
//  3. No crash / edge behaviour: no NaN, no missing chunk, no exception
//     over 82 km; geometry count stays within the window
//     ((2*STREAM_RADIUS+1)^2 chunks).
//  4. The streamer agrees with the old world where they overlap:
//     heightAt == PandoraWorldGenerator.heightfieldHeightAt inside the
//     central 12 km block, and no cliff at the old map edge (seam).
//  5. Determinism: a chunk evicted by flying away regenerates
//     IDENTICALLY when re-entered (exact Float32Array equality).
//
// Usage: node_modules/.bin/esbuild scripts/probe-infinite-flight.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-infinite-flight.mjs
//   && node /tmp/probe-infinite-flight.mjs

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
import { TerrainStreamer, STREAM_CHUNK_SIZE, STREAM_RADIUS } from '../src/world/TerrainStreamer';
import { generateChunkHeights, heightAt, RUNWAY_TABLE } from '../src/world/PandoraTerrainWorker';

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

  // Old world, for the central-block agreement check only (its mesh +
  // heightfield still own the central 12 km; the streamer never builds
  // geometry there).
  const worldGen = new PandoraWorldGenerator(scene, world, origin);
  await worldGen.build(1337);
  worldGen.registerWithFloatingOrigin(origin);

  const added: number[] = [];
  const removed: number[] = [];
  const streamer = new TerrainStreamer(scene, world, origin, {
    onColliderAdded: (h) => added.push(h),
    onColliderRemoved: (h) => removed.push(h),
  });

  // ---------- 1. Determinism of chunk generation ----------
  const hA = generateChunkHeights(7, -1, STREAM_CHUNK_SIZE, 51);
  const hB = generateChunkHeights(7, -1, STREAM_CHUNK_SIZE, 51);
  let detSame = hA.length === hB.length;
  for (let i = 0; detSame && i < hA.length; i++) detSame = hA[i] === hB[i];
  check('chunk generation is deterministic (same chunk twice, exact float equality)', detSame, `${hA.length} samples`);

  // ---------- 2. Central-block agreement (before any flying: scene == original) ----------
  let maxCentralDiff = 0;
  let worstCentral = '';
  const sampleCentral = (x: number, z: number): void => {
    const d = Math.abs(streamer.heightAt(x, z) - worldGen.heightfieldHeightAt(x, z));
    if (d > maxCentralDiff) {
      maxCentralDiff = d;
      worstCentral = `(${x.toFixed(0)},${z.toFixed(0)})`;
    }
  };
  for (let x = -6000; x <= 6000; x += 120) {
    for (let z = -6000; z <= 6000; z += 120) sampleCentral(x, z);
  }
  // Random points + runway strips/plateau edges (worst case for sampling).
  let seed = 12345;
  const rnd = (): number => {
    seed = (seed * 16807) % 2147483647;
    return seed / 2147483647;
  };
  for (let i = 0; i < 500; i++) sampleCentral((rnd() - 0.5) * 12000, (rnd() - 0.5) * 12000);
  for (const r of RUNWAY_TABLE) {
    const sh = Math.sin((r.hdg * Math.PI) / 180);
    const ch = Math.cos((r.hdg * Math.PI) / 180);
    for (let lx = -(r.len / 2 + 200); lx <= r.len / 2 + 200; lx += 10) {
      for (const lz of [-r.width / 2 - 80, 0, r.width / 2 + 80]) {
        sampleCentral(r.x + lx * sh + lz * ch, r.z + lx * ch - lz * sh);
      }
    }
  }
  check(
    'streamer heightAt agrees with generator heightfieldHeightAt inside the central 12 km block',
    maxCentralDiff < 0.05,
    `max |diff| = ${maxCentralDiff.toExponential(2)} m at ${worstCentral} (10,201 grid + 500 random + runway points)`,
  );

  // ---------- 3. Seam at the old map edge (x = +/-6000, z = +/-6000) ----------
  // (a) The two surfaces must meet EXACTLY along the whole seam: the
  //     streamer and the generator sample the same heightAt at the same
  //     60 m nodes with the same triangle diagonal, so the shared edge
  //     nodes agree and the surface is C0 across the seam by
  //     construction. Sweep every edge at the grid step.
  // (c) Sampled-vs-analytic deviation just outside vs just inside:
  //     the 60 m grid undersamples ridged noise on BOTH sides (up to
  //     ~26 m in the steep north ranges), so compare the sweep MAXIMA,
  //     not single points — a single z compares different terrain
  //     features (a crest outside vs a flank inside).
  let edgeWorst = 0;
  let edgeWorstAt = '';
  const edgePts: Array<[number, number, string]> = [];
  for (let t = -6000; t <= 6000; t += 60) {
    edgePts.push([6000, t, `east z=${t}`]);
    edgePts.push([-6000, t, `west z=${t}`]);
    edgePts.push([t, -6000, `north x=${t}`]);
    edgePts.push([t, 6000, `south x=${t}`]);
  }
  for (const [x, z, label] of edgePts) {
    const d = Math.abs(streamer.heightAt(x, z) - worldGen.heightfieldHeightAt(x, z));
    if (d > edgeWorst) {
      edgeWorst = d;
      edgeWorstAt = label;
    }
  }
  let devOutMax = 0;
  let devInMax = 0;
  let devOutAt = '';
  let devInAt = '';
  for (let z = -6000; z <= 6000; z += 60) {
    const devOut = Math.abs(streamer.heightAt(6010, z) - heightAt(6010, z));
    if (devOut > devOutMax) {
      devOutMax = devOut;
      devOutAt = `z=${z}`;
    }
    const devIn = Math.abs(worldGen.heightfieldHeightAt(5990, z) - heightAt(5990, z));
    if (devIn > devInMax) {
      devInMax = devIn;
      devInAt = `z=${z}`;
    }
  }
  check(
    'seam (a): streamer meets the generator exactly along the whole old map edge',
    edgeWorst < 0.05,
    `worst |diff| = ${edgeWorst.toExponential(2)} m at ${edgeWorstAt} (${edgePts.length} edge points, 4 edges)`,
  );
  check(
    'seam (c): sampled-vs-analytic deviation just outside the edge is ~ the same as just inside (same 60 m grid)',
    Math.abs(devOutMax - devInMax) < 10 && devOutMax < 40,
    `max outside=${devOutMax.toFixed(1)} m at ${devOutAt}, max inside=${devInMax.toFixed(1)} m at ${devInAt}`,
  );

  // ---------- 4. The flight: 40 km east, 18 km west, 18 km east ----------
  const pos = new THREE.Vector3(-2800, 500, -1800); // scene coords, Alpha
  origin.registerVector(pos); // live behaviour: the player body is registered
  const ray = new RAPIER.Ray({ x: 0, y: 2000, z: 0 }, { x: 0, y: -1, z: 0 });
  // +0.5 m offset: Rapier heightfield raycasts MISS when the ray passes
  // exactly through a grid line / vertex (measured in a focused diag:
  // exact vertex -> NONE even after 3 steps, +0.5 m in x -> hits at
  // heightAt +/- 0.01). No src/ code uses castRay, so this is a
  // probe-measurement quirk, not a gameplay issue (the controller uses
  // the analytic triangle interpolation, never a raycast).
  const raycastGround = (sx: number, sz: number): number | null => {
    world.step();
    ray.origin.x = sx + 0.5;
    ray.origin.z = sz + 0.5;
    const hit = world.castRay(ray, 4000, true);
    return hit ? 2000 - hit.timeOfImpact : null;
  };

  const dt = 1 / 60;
  const speed = 150; // m/s
  const stepLen = speed * dt; // 2.5 m per frame
  let shifts = 0;
  let chunkCrossings = 0;
  let maxStepDelta = 0;
  let maxSceneAbs = 0;
  let minH = Infinity;
  let maxH = -Infinity;
  let maxRayDiff = 0;
  let worstRayAt = '';
  let nanCount = 0;
  let holeCount = 0;
  let voidRayCount = 0;
  let maxGeom = 0;
  let totalDist = 0;
  let lastChunk = streamer.chunkCoordsAt(pos.x, pos.z);
  let lastOrigX = pos.x;
  let lastOrigZ = pos.z;

  // Snapshot chunk (12,-1) heights at the end of the east leg, to prove
  // deterministic regeneration after eviction + re-entry.
  let snapshot: Float32Array | null = null;
  let createdBeforeReturn = 0;
  let retiredBeforeReturn = 0;

  const flyLeg = (label: string, dirX: number, distM: number): void => {
    const startOrigX = pos.x + origin.cumulativeShift.x;
    const targetOrigX = startOrigX + dirX * distM;
    let nextLogKm = Math.floor(startOrigX / 2000) * 2000 + (dirX > 0 ? 2000 : 0);
    let nextRayAt = startOrigX + dirX * 100;
    let legDist = 0;
    for (let f = 0; f < 60 * 600; f++) {
      // Frame-start shift, exactly like main.ts animate().
      const shift = origin.shiftIfNeeded(pos);
      if (shift) {
        shifts++;
        origin.applyShift(shift);
      }
      // Streaming right after the shift (the wiring order).
      streamer.update(pos);
      // Fly.
      pos.x += dirX * stepLen;
      legDist += stepLen;
      totalDist += stepLen;
      const origX = pos.x + origin.cumulativeShift.x;
      const origZ = pos.z + origin.cumulativeShift.z;
      const dStep = Math.hypot(origX - lastOrigX, origZ - lastOrigZ);
      maxStepDelta = Math.max(maxStepDelta, dStep);
      lastOrigX = origX;
      lastOrigZ = origZ;
      maxSceneAbs = Math.max(maxSceneAbs, Math.abs(pos.x), Math.abs(pos.z));
      const cur = streamer.chunkCoordsAt(origX, origZ);
      if (cur.cx !== lastChunk.cx || cur.cz !== lastChunk.cz) {
        chunkCrossings++;
        lastChunk = cur;
      }
      const h = streamer.heightAt(pos.x, pos.z);
      if (!Number.isFinite(h)) nanCount++;
      else {
        minH = Math.min(minH, h);
        maxH = Math.max(maxH, h);
      }
      const central =
        cur.cx >= -2 && cur.cx <= 1 && cur.cz >= -2 && cur.cz <= 1;
      if (!central && !streamer.hasGeometryAt(origX, origZ)) holeCount++;
      maxGeom = Math.max(maxGeom, streamer.loadedGeometryCount);
      // Raycast every 100 m of original-frame travel.
      if ((dirX > 0 && origX >= nextRayAt) || (dirX < 0 && origX <= nextRayAt)) {
        const rc = raycastGround(pos.x, pos.z);
        if (rc === null) {
          voidRayCount++;
          console.log(`  VOID raycast at orig=(${origX.toFixed(0)},${origZ.toFixed(0)})`);
        } else {
          // Compare against heightAt at the SAME offset point the ray
          // was cast from (raycastGround adds +0.5 m in x and z).
          const hRay = streamer.heightAt(pos.x + 0.5, pos.z + 0.5);
          const d = Math.abs(rc - hRay);
          if (d > maxRayDiff) {
            maxRayDiff = d;
            worstRayAt = `orig=(${origX.toFixed(0)},${origZ.toFixed(0)}) raycast=${rc.toFixed(2)} heightAt=${hRay.toFixed(2)}`;
          }
        }
        nextRayAt += dirX * 100;
      }
      if ((dirX > 0 && origX >= nextLogKm) || (dirX < 0 && origX <= nextLogKm)) {
        console.log(
          `  ${label} origX=${origX.toFixed(0)} sceneX=${pos.x.toFixed(0)} cumShift=(${origin.cumulativeShift.x.toFixed(0)},${origin.cumulativeShift.z.toFixed(0)}) ` +
            `h=${h.toFixed(1)} geom=${streamer.loadedGeometryCount} shifts=${shifts}`,
        );
        nextLogKm += dirX * 2000;
      }
      if ((dirX > 0 && origX >= targetOrigX) || (dirX < 0 && origX <= targetOrigX)) break;
    }
    console.log(`  ${label} done: leg ${legDist.toFixed(0)} m, total ${totalDist.toFixed(0)} m`);
  };

  flyLeg('EAST', +1, 40000);
  const eastEndOrigX = pos.x + origin.cumulativeShift.x;
  const snapH = streamer.heightsAt(37000, -1800);
  snapshot = snapH ? snapH.slice() : null;
  createdBeforeReturn = streamer.createdChunks;
  retiredBeforeReturn = streamer.retiredChunks;
  console.log(
    `  east leg end: origX=${eastEndOrigX.toFixed(0)} chunk=${JSON.stringify(streamer.chunkCoordsAt(eastEndOrigX, -1800))} ` +
      `snapshot chunk (12,-1) heights ${snapshot ? 'captured (' + snapshot.length + ' samples)' : 'MISSING'} ` +
      `created=${createdBeforeReturn} retired=${retiredBeforeReturn}`,
  );

  flyLeg('WEST', -1, 21000); // chunk (12,-1) is 7 chunks behind -> evicted (evict radius STREAM_RADIUS+3 = 6). Pass 12: STREAM_RADIUS went 2 -> 3 to close the no-mesh band inside the 11 km fog far, so the west leg went 18 -> 21 km — at radius 2 the chunk was 6 behind (evict radius 5) and evicted; at radius 3 it is still 6 behind (evict radius 6) and NOT evicted.
  const westEndOrigX = pos.x + origin.cumulativeShift.x;
  const evicted = streamer.heightsAt(37000, -1800) === null;
  console.log(
    `  west leg end: origX=${westEndOrigX.toFixed(0)} chunk (12,-1) heights cached? ${!evicted} (evicted=${evicted}) ` +
      `created=${streamer.createdChunks} retired=${streamer.retiredChunks}`,
  );

  flyLeg('EAST-RETURN', +1, 21000); // re-enter chunk (12,-1): must regenerate
  const retEndOrigX = pos.x + origin.cumulativeShift.x;
  const regen = streamer.heightsAt(37000, -1800);
  let regenSame = false;
  if (snapshot && regen && regen.length === snapshot.length) {
    regenSame = true;
    for (let i = 0; i < snapshot.length; i++) {
      if (regen[i] !== snapshot[i]) {
        regenSame = false;
        break;
      }
    }
  }

  console.log('--- flight summary ---');
  console.log(`  distance flown: ${(totalDist / 1000).toFixed(1)} km (east 40 + west 21 + east-return 21)`);
  console.log(`  origin shifts crossed: ${shifts}`);
  console.log(`  chunk boundaries crossed: ${chunkCrossings}`);
  console.log(`  max per-step original-frame position delta: ${maxStepDelta.toFixed(3)} m (expected v*dt = ${stepLen.toFixed(3)} m)`);
  console.log(`  max |scene| position (must stay near origin): ${maxSceneAbs.toFixed(1)} m`);
  console.log(`  terrain heightAt range: ${minH.toFixed(1)} .. ${maxH.toFixed(1)} m (analytic range is -25..750)`);
  console.log(`  max |raycast - heightAt|: ${maxRayDiff.toFixed(3)} m at ${worstRayAt}`);
  console.log(`  chunks created: ${streamer.createdChunks} (before return leg: ${createdBeforeReturn}), retired: ${streamer.retiredChunks} (before: ${retiredBeforeReturn})`);
  console.log(`  max geometry chunks loaded at once: ${maxGeom} (window is ${(2 * STREAM_RADIUS + 1) ** 2}, central block excluded)`);
  console.log(`  collider handles: added=${added.length} removed=${removed.length} live=${streamer.terrainColliderHandles().length}`);
  console.log(`  return-leg end: origX=${retEndOrigX.toFixed(0)}`);

  check('no position snap: max per-step original-frame delta == v*dt', maxStepDelta <= stepLen + 0.01, `max=${maxStepDelta.toFixed(3)} m vs ${stepLen.toFixed(3)} m`);
  check('scene position stays bounded near the origin across all shifts', maxSceneAbs < 2100, `max |scene| = ${maxSceneAbs.toFixed(1)} m`);
  check('continuous terrain: heightAt finite everywhere, in the analytic range', nanCount === 0 && minH >= -25 && maxH <= 750, `NaNs=${nanCount} range=${minH.toFixed(1)}..${maxH.toFixed(1)}`);
  check('no holes: chunk under the player always has geometry (outside the central block)', holeCount === 0, `geometry holes=${holeCount}`);
  check('no void: Rapier raycast down hits terrain at every sample', voidRayCount === 0, `void raycasts=${voidRayCount}`);
  check('Rapier collider agrees with the height query everywhere', maxRayDiff < 0.5, `max |raycast - heightAt| = ${maxRayDiff.toFixed(3)} m at ${worstRayAt}`);
  // Pass 12: the window bound is (2*STREAM_RADIUS+1)^2 — 25 at radius
  // 2, 49 at radius 3. STREAM_RADIUS went 2 -> 3 to close the
  // no-mesh band inside the 11 km fog far (Worker F: 41% of the
  // ground disc within 11 km of the carrier had no mesh).
  check('window bounded: never more than (2*STREAM_RADIUS+1)^2 geometry chunks loaded', maxGeom <= (2 * STREAM_RADIUS + 1) ** 2, `max=${maxGeom} window=${(2 * STREAM_RADIUS + 1) ** 2}`);
  check('collider lifecycle: every added handle is either live or removed', added.length === streamer.terrainColliderHandles().length + removed.length, `added=${added.length} live=${streamer.terrainColliderHandles().length} removed=${removed.length}`);
  check('chunk (12,-1) was evicted while flying away', evicted, `heights cached after west leg: ${!evicted}`);
  check('evicted chunk regenerates IDENTICALLY on re-entry (exact float equality)', regenSame, snapshot ? `${snapshot.length} samples compared` : 'no snapshot');
  check('re-entered chunk was rebuilt (created counter grew on the return leg)', streamer.createdChunks > createdBeforeReturn, `created ${createdBeforeReturn} -> ${streamer.createdChunks}`);
  check('long flight: at least 30 km flown, through several shifts and chunk boundaries', totalDist >= 30000 && shifts >= 5 && chunkCrossings >= 5, `${(totalDist / 1000).toFixed(1)} km, ${shifts} shifts, ${chunkCrossings} chunk crossings`);

  console.log(failures === 0 ? 'ALL INFINITE-FLIGHT CHECKS PASSED' : `${failures} INFINITE-FLIGHT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
