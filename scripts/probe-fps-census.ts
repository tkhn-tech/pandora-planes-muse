// probe-fps-census.ts — pass 14, worker E: FPS LOW EVERYWHERE.
//
// The v4.8 live check (24 Nov, footer v4.8, 5 rounds, all from Alpha)
// found FPS 4-9 on the deck/ground AND 8-9 at altitude — low
// everywhere, only marginally better at altitude. Pass 13 exonerated
// the simulation (live-path substep ~0.59-1.04 ms; scene census
// terrain 245,000 tris / 34 draw calls = 99.6% of probe-scene tris)
// and blamed the ~200 ms/frame near the ground on render-side cost
// (GPU fill/draw on the phone — 245k terrain tris, DPR <= 2 + MSAA +
// ACES, unmeasurable from Node).
//
// This probe measures, in the closest-to-live conditions Node can
// construct, everything about the submitted frame EXCEPT the GPU
// itself:
//   1. Full live-scene census (real worldGen + TerrainStreamer +
//      RunwayPlacer + 1 active helicarrier + player F/A-18 + 4 enemies
//      + 2 traffic, all real GLBs): object count, draw calls and
//      triangles per top-level contributor, geometries / textures /
//      materials — the same numbers renderer.info.render reports live
//      (calls / triangles / geometries / textures / programs, minus
//      programs, which only exist on a real GL context).
//   2. Frustum-visible census from the two live-check viewpoints
//      (Alpha deck chase cam; 3000 m altitude looking at the horizon):
//      what the renderer actually submits after culling.
//   3. CPU cost of the per-frame work that feeds the renderer:
//      scene.updateMatrixWorld, worldGen.update (waterfalls + clouds),
//      TerrainStreamer.update, frustum-cull traversal — timed over
//      many iterations, live variable dt, live wind.
//   4. Fragment-fill arithmetic for the DPR/MSAA/ACES setup: drawing
//      buffer pixels and MSAA samples per frame at DPR 1 / 1.5 / 2,
//      for a generic phone CSS viewport (ratios, not a device claim).
//
// What this probe CANNOT measure (stated plainly, not faked): GPU
// vertex/fragment time on the phone, MSAA resolve cost, and the
// effect of any DPR/antialias change on live FPS. That is exactly
// what the new renderer.info readout in the ?debug=1 panel is for —
// the live browser check reads it and decides.
//
// Usage: node_modules/.bin/esbuild scripts/probe-fps-census.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-fps-census.mjs
//   && node /tmp/probe-fps-census.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type = '';
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
// GLTFLoader decodes embedded textures with `new Image()` — the
// task brief's Image stub: a src setter that fires load async.
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class ImageStub {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1;
    height = 1;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void {
      (this.listeners[type] ??= []).push(fn);
    }
    removeEventListener(): void {}
    set src(_v: string) {
      queueMicrotask(() => {
        for (const fn of this.listeners['load'] ?? []) fn();
        this.onload?.();
      });
    }
    get src(): string {
      return '';
    }
  };
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
import { TerrainStreamer, STREAM_RADIUS } from '../src/world/TerrainStreamer';
import { RunwayPlacer } from '../src/world/RunwayPlacer';
import { RUNWAY_TABLE } from '../src/world/PandoraTerrainWorker';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { HelicarrierController } from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

interface Bucket {
  calls: number;
  tris: number;
  objects: number;
}
function bucketOf(name: string): string {
  if (name === 'terrain') return 'terrain (central 12km)';
  if (name.startsWith('terrain-chunk-')) return 'terrain chunks (streamer)';
  if (name.startsWith('island-')) return 'floating islands';
  if (name.startsWith('runway')) return 'runway strips';
  if (name.startsWith('ford') || name.startsWith('heli') || name.includes('carrier'))
    return 'helicarrier';
  return 'other';
}

function trisOf(o: THREE.Object3D): number {
  const mesh = o as THREE.Mesh;
  const geo = mesh.geometry as THREE.BufferGeometry | undefined;
  if (!geo) return 0;
  const pos = geo.attributes.position as THREE.BufferAttribute | undefined;
  if (!pos) return 0;
  const idx = geo.index;
  const base = idx ? idx.count / 3 : pos.count / 3;
  const inst = (mesh as THREE.InstancedMesh).count;
  return Math.round(base * (typeof inst === 'number' ? inst : 1));
}

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x87b5d9);
  scene.fog = new THREE.Fog(0x9db8cc, 900, 11000);
  const origin = new FloatingOrigin(2000);

  const worldGen = new PandoraWorldGenerator(scene, world, origin);
  await worldGen.build(1337);
  worldGen.registerWithFloatingOrigin(origin);
  const streamer = new TerrainStreamer(scene, world, origin);
  const placer = new RunwayPlacer(scene);
  placer.place(worldGen.runways);

  // Live default round: player F/A-18 on Alpha, 4 enemies (2 MiG-29 +
  // 2 Su-34), 2 traffic (F-18 + MiG-29), 1 active helicarrier.
  // main.ts builds THREE carriers and hides two; only the active one
  // is built here (the hidden two add scene-graph nodes, not draws).
  const alpha = RUNWAY_TABLE[0]!;
  const player = new Aircraft({
    scene, world, spec: SPECS['fa18']!, isPlayer: true, modelKey: SPECS['fa18']!.modelKey,
  });
  await player.attachModel();
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
  console.log(`player F/A-18: modelStatus=${player.modelStatus} tris=${player.modelTris} err=${player.modelError ?? 'none'}`);

  const jets: Aircraft[] = [player];
  for (let i = 0; i < 4; i++) {
    const key = (i % 2 === 0 ? 'mig29' : 'su34') as 'mig29' | 'su34';
    const e = new Aircraft({
      scene, world, spec: SPECS[key]!, isPlayer: false, modelKey: SPECS[key]!.modelKey,
    });
    await e.attachModel();
    const fwd = player.fwd();
    e.resetAt(
      player.pos.x + fwd.x * (450 + i * 150),
      player.pos.y + 120 + i * 60,
      player.pos.z + fwd.z * (450 + i * 150),
      0,
    );
    e.setAirspeed(150);
    jets.push(e);
    console.log(`enemy ${key}: modelStatus=${e.modelStatus} tris=${e.modelTris} err=${e.modelError ?? 'none'}`);
  }
  for (let i = 0; i < 2; i++) {
    const key = (i % 2 === 0 ? 'fa18' : 'mig29') as 'fa18' | 'mig29';
    const t = new Aircraft({
      scene, world, spec: SPECS[key]!, isPlayer: false, modelKey: SPECS[key]!.modelKey,
    });
    await t.attachModel();
    const r = worldGen.runways[i % worldGen.runways.length]!;
    t.resetAt(r.x, r.elev + 220, r.z, r.hdg);
    t.setAirspeed(130);
    jets.push(t);
  }

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => worldGen.heightfieldHeightAt(x, z),
  });
  await carrier.build();
  carrier.registerWithFloatingOrigin(origin);

  // Fill the streamer window around Alpha (radius 3), like the live
  // loop does over its first frames (budget 2/frame + sync player
  // chunk). Loop until the loaded count stops growing.
  const alphaPos = new THREE.Vector3(alpha.x, alpha.elev, alpha.z);
  let prev = -1;
  for (let i = 0; i < 40 && streamer.loadedGeometryCount !== prev; i++) {
    prev = streamer.loadedGeometryCount;
    streamer.update(alphaPos);
  }
  console.log(
    `streamer: radius=${STREAM_RADIUS} loadedGeometry=${streamer.loadedGeometryCount} ` +
      `cached=${streamer.cachedChunkCount} created=${streamer.createdChunks}`,
  );
  const windowChunks = (2 * STREAM_RADIUS + 1) ** 2 - 16; // 7x7 minus central 4x4
  check(
    'streamer window full around Alpha (radius 3)',
    streamer.loadedGeometryCount === windowChunks,
    `loaded=${streamer.loadedGeometryCount} expected=${windowChunks}`,
  );

  scene.updateMatrixWorld(true);

  // ---------- 1. Full-scene census (renderer.info.render equivalent) ----------
  const buckets = new Map<string, Bucket>();
  const materials = new Set<unknown>();
  const geometries = new Set<unknown>();
  const textures = new Set<unknown>();
  let totalObjects = 0;
  let totalCalls = 0;
  let totalTris = 0;
  let jetCalls = 0;
  let jetTris = 0;
  scene.traverse((o) => {
    totalObjects++;
    const isDraw =
      (o as THREE.Mesh).isMesh ||
      (o as THREE.Sprite).isSprite ||
      (o as THREE.Points).isPoints ||
      (o as THREE.Line).isLine;
    if (!isDraw || !o.visible) return;
    const mesh = o as THREE.Mesh;
    const geo = mesh.geometry as THREE.BufferGeometry | undefined;
    if (geo) geometries.add(geo);
    const mat = (mesh as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach((m) => { materials.add(m); if ((m as unknown as { map?: unknown }).map) textures.add((m as unknown as { map: unknown }).map); });
    else if (mat) { materials.add(mat); if ((mat as unknown as { map?: unknown }).map) textures.add((mat as unknown as { map: unknown }).map); }
    const t = trisOf(o);
    // Attribute jets to their aircraft group (group name is the spec key).
    let b = bucketOf(o.name || '');
    let parent: THREE.Object3D | null = o.parent;
    while (parent) {
      if (parent.name && (parent.name.startsWith('jet-') || jets.some((j) => j.group === parent))) { b = 'jets (7 aircraft, GLBs)'; break; }
      parent = parent.parent;
    }
    const rec = buckets.get(b) ?? { calls: 0, tris: 0, objects: 0 };
    rec.calls++;
    rec.tris += t;
    rec.objects++;
    buckets.set(b, rec);
    totalCalls++;
    totalTris += t;
    if (b === 'jets (7 aircraft, GLBs)') { jetCalls++; jetTris += t; }
  });
  console.log(`\n--- full-scene census (all visible drawables, no frustum culling) ---`);
  console.log(`objects in scene graph: ${totalObjects}`);
  for (const [name, b] of [...buckets.entries()].sort((a, b) => b[1].tris - a[1].tris)) {
    console.log(`  ${name}: ${b.calls} calls, ${b.tris.toLocaleString()} tris (${b.objects} objects)`);
  }
  console.log(`TOTAL: ${totalCalls} draw calls, ${totalTris.toLocaleString()} tris`);
  // Per top-level scene child: who owns the "other" 575 calls?
  console.log(`--- per top-level scene child ---`);
  for (const child of scene.children) {
    let c = 0;
    let t = 0;
    let objs = 0;
    child.traverse((o) => {
      objs++;
      const isDraw =
        (o as THREE.Mesh).isMesh ||
        (o as THREE.Sprite).isSprite ||
        (o as THREE.Points).isPoints;
      if (!isDraw || !o.visible) return;
      c++;
      t += trisOf(o);
    });
    if (c > 0) console.log(`  ${child.name || child.type}: ${c} calls, ${t.toLocaleString()} tris (${objs} objects)`);
  }
  // Per aircraft group: meshes + materials (attachModel clones every
  // material per aircraft — see FlightPhysicsController.attachModel).
  console.log(`--- per aircraft ---`);
  for (const j of jets) {
    let c = 0;
    let t = 0;
    const mats = new Set<unknown>();
    j.group.traverse((o) => {
      const isDraw = (o as THREE.Mesh).isMesh;
      if (!isDraw || !o.visible) return;
      c++;
      t += trisOf(o);
      const m = (o as THREE.Mesh).material;
      if (Array.isArray(m)) m.forEach((x) => mats.add(x));
      else if (m) mats.add(m);
    });
    console.log(`  ${j.spec.key} (${j.modelStatus}): ${c} meshes/calls, ${t.toLocaleString()} tris, ${mats.size} material instances`);
  }
  console.log(`unique geometries: ${geometries.size}, unique materials: ${materials.size}, unique textures: ${textures.size}`);
  const terrainTris =
    (buckets.get('terrain (central 12km)')?.tris ?? 0) +
    (buckets.get('terrain chunks (streamer)')?.tris ?? 0);
  console.log(
    `terrain share: ${terrainTris.toLocaleString()} / ${totalTris.toLocaleString()} tris = ${((terrainTris / Math.max(1, totalTris)) * 100).toFixed(1)}%`,
  );
  check(
    'census reproduces pass-13 terrain figure (245k tris / 34 calls)',
    terrainTris === 245000 &&
      (buckets.get('terrain (central 12km)')?.calls ?? 0) +
        (buckets.get('terrain chunks (streamer)')?.calls ?? 0) ===
        34,
    `terrain=${terrainTris.toLocaleString()} tris, ` +
      `calls=${(buckets.get('terrain (central 12km)')?.calls ?? 0) + (buckets.get('terrain chunks (streamer)')?.calls ?? 0)}`,
  );

  // ---------- 2. Frustum-visible census, live viewpoints ----------
  const camera = new THREE.PerspectiveCamera(62, 1080 / 2400, 0.5, 20000);
  const frustum = new THREE.Frustum();
  const projView = new THREE.Matrix4();
  function visibleCensus(label: string, camPos: THREE.Vector3, lookAt: THREE.Vector3): void {
    camera.position.copy(camPos);
    camera.lookAt(lookAt);
    camera.updateMatrixWorld(true);
    projView.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projView);
    let calls = 0;
    let tris = 0;
    let terrainCalls = 0;
    let terrainTrisV = 0;
    scene.traverse((o) => {
      const isDraw =
        (o as THREE.Mesh).isMesh ||
        (o as THREE.Sprite).isSprite ||
        (o as THREE.Points).isPoints;
      if (!isDraw || !o.visible) return;
      if (!frustum.intersectsObject(o)) return;
      const t = trisOf(o);
      calls++;
      tris += t;
      if (o.name === 'terrain' || o.name.startsWith('terrain-chunk-')) {
        terrainCalls++;
        terrainTrisV += t;
      }
    });
    console.log(
      `${label}: visible ${calls} calls / ${tris.toLocaleString()} tris ` +
        `(terrain ${terrainCalls} calls / ${terrainTrisV.toLocaleString()} tris = ${((terrainTrisV / Math.max(1, tris)) * 100).toFixed(1)}%)`,
    );
  }
  console.log(`\n--- frustum-visible census (what renderer.render actually submits) ---`);
  // Alpha deck chase cam: ~12 m behind / 4 m above the jet, looking
  // along the runway heading (the v4.8 live-check viewpoint).
  const hdgRad = (alpha.hdg * Math.PI) / 180;
  const fwdV = new THREE.Vector3(Math.sin(hdgRad), 0, Math.cos(hdgRad));
  const deckCam = new THREE.Vector3(
    alpha.x - fwdV.x * 12, alpha.elev + 4, alpha.z - fwdV.z * 12,
  );
  visibleCensus('deck chase cam (Alpha)', deckCam, new THREE.Vector3(alpha.x + fwdV.x * 500, alpha.elev + 50, alpha.z + fwdV.z * 500));
  // Altitude: 3000 m over Alpha, looking at the horizon ahead.
  visibleCensus(
    'altitude 3000 m',
    new THREE.Vector3(alpha.x, 3000, alpha.z),
    new THREE.Vector3(alpha.x + fwdV.x * 8000, 1500, alpha.z + fwdV.z * 8000),
  );
  // Altitude looking straight down (max terrain fill).
  visibleCensus(
    'altitude 3000 m, looking down',
    new THREE.Vector3(alpha.x, 3000, alpha.z),
    new THREE.Vector3(alpha.x, 0, alpha.z),
  );

  // ---------- 3. CPU cost of the per-frame render-feeding work ----------
  console.log(`\n--- per-frame CPU cost on this VM (200 iterations, ms/frame) ---`);
  function timeIt(label: string, fn: () => void, n = 200): number {
    fn();
    const t0 = performance.now();
    for (let i = 0; i < n; i++) fn();
    const ms = (performance.now() - t0) / n;
    console.log(`  ${label}: ${ms.toFixed(3)} ms/frame`);
    return ms;
  }
  const wind = new THREE.Vector3(3, 0, 0);
  const dt = 1 / 60;
  const matrixMs = timeIt('scene.updateMatrixWorld', () => scene.updateMatrixWorld(true));
  const worldGenMs = timeIt('worldGen.update(dt, wind) [waterfalls+clouds]', () => worldGen.update(dt, wind));
  const streamerMs = timeIt('terrainStreamer.update (window full, steady state)', () => streamer.update(alphaPos));
  const cullMs = timeIt('frustum-cull traversal (deck cam)', () => {
    camera.position.copy(deckCam);
    camera.lookAt(new THREE.Vector3(alpha.x + fwdV.x * 500, alpha.elev + 50, alpha.z + fwdV.z * 500));
    camera.updateMatrixWorld(true);
    projView.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projView);
    scene.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && o.visible) frustum.intersectsObject(o);
    });
  });
  // updateSky's per-frame allocations (main.ts updateSky, copied
  // verbatim in shape): 4x `new THREE.Color()` per frame.
  const skyMs = timeIt('updateSky-shaped work (4x new Color + setHSL + lerp)', () => {
    const skyDay = new THREE.Color(0x87b5d9);
    const skyDusk = new THREE.Color(0xe8905a);
    const skyNight = new THREE.Color(0x0a1226);
    const sky = new THREE.Color();
    sky.copy(skyDay).lerp(skyDusk, 0.5);
    void skyNight;
  });
  const renderFeedMs = matrixMs + worldGenMs + streamerMs + cullMs;
  console.log(`  render-feeding CPU subtotal: ${renderFeedMs.toFixed(3)} ms/frame (excludes sim + renderer.render itself)`);
  check(
    'render-feeding CPU work is small vs a 110-250 ms live frame (sim already exonerated, pass 13)',
    renderFeedMs < 5,
    `${renderFeedMs.toFixed(2)} ms/frame on probe VM`,
  );
  void skyMs;

  // ---------- 4. Fragment-fill arithmetic (DPR / MSAA / ACES) ----------
  console.log(`\n--- fragment-fill arithmetic (ratios, not a device claim) ---`);
  console.log(`renderer setup in main.ts: antialias=true (MSAA 4x on WebGL2), DPR=min(devicePixelRatio, 2), ACESFilmicToneMapping, fog 900-11000 m, camera far 20000`);
  for (const [w, h, label] of [
    [390, 844, '390x844 CSS (small phone)'],
    [1080, 2400, '1080x2400 CSS (large phone)'],
  ] as const) {
    const row = [1, 1.5, 2].map((dpr) => {
      const px = w * dpr * (h * dpr);
      const samples = px * 4; // MSAA 4x
      return `DPR ${dpr}: ${(px / 1e6).toFixed(1)}M px, ${(samples / 1e6).toFixed(1)}M MSAA samples`;
    });
    console.log(`  ${label}: ${row.join('  |  ')}`);
  }
  console.log(`  DPR 2 vs DPR 1 = 4.0x fragment samples per frame; DPR 1.5 vs DPR 2 = 0.56x.`);
  console.log(`  At the live 5 fps, DPR 2 on a 1080x2400 CSS viewport = ~52M px/s, ~207M MSAA samples/s before overdraw (terrain + sky + transparent clouds/sprites).`);
  console.log(`  UNMEASURED FROM NODE: GPU vertex/fragment time, MSAA resolve time, and the live FPS effect of any DPR/antialias change. The ?debug=1 renderer.info readout is the instrument for that.`);

  console.log(failures === 0 ? '\nALL FPS-CENSUS CHECKS PASSED' : `\n${failures} FPS-CENSUS CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
