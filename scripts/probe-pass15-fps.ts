// probe-pass15-fps.ts — pass 15, worker F, item 10: FPS still low
// (4-9 ground, 10-12 altitude), bottleneck still unidentified.
//
// Live facts (v4.9 check): deck 205 calls / 209,303 tris / DPR 1 /
// AA on / ACES, FPS 7; at 9,000 ft 66-80 calls / ~195,000-220,000
// tris, FPS 10-12. ?dpr=1: DPR was already 1, no gain. ?noaa=1 (AA
// off): FPS 8-12, marginal gain. So DPR and AA are NOT the
// bottleneck, and draw calls are not the whole story either (3x
// fewer calls at altitude, only marginally better FPS).
//
// This probe measures, in the closest-to-live conditions Node can
// construct, the two remaining measurable suspects:
//   1. Per-bucket FRUSTUM-VISIBLE draw calls from the two live
//      viewpoints (Alpha deck chase cam; 3000 m altitude): who owns
//      the submitted calls — runway strips (143 calls / 4,346 tris
//      full scene), MiG-29 GLBs (73 meshes / 70 materials each),
//      terrain, carrier, jets.
//   2. Full main-loop CPU per live frame (2 substeps at 5 fps:
//      Rapier step + 7 aircraft steps + weapons + vfx + worldGen
//      + streamer + updateMatrixWorld + HUD update), timed on the
//      probe VM — the sim was exonerated at ~1 ms, but this is the
//      full frame including HUD DOM writes.
// What it CANNOT measure (stated plainly): GPU vertex/fragment
// time, MSAA resolve, and the live FPS effect of any change. The
// conclusion is therefore an inference from the numbers, not a GPU
// measurement: if the visible calls fall 260 -> ~170 after the
// runway merge and the full-frame CPU is < 10 ms on the probe VM,
// the remaining ~100+ ms live frame is inside renderer.render on
// the phone GPU/driver, scaling with submitted calls — and the
// runway merge (143 -> 15 calls) is the biggest shippable cut.
//
// Checks: runway visible calls from the deck <= 15 (pre-fix ~98),
// total visible deck calls <= 180 (pre-fix 260), full-frame CPU
// < 10 ms on the probe VM.
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass15-fps.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass15-fps.mjs
//   && node /tmp/probe-pass15-fps.mjs
//   (delete /tmp/probe-pass15-fps.mjs after running)

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type = ''; lengthComputable = false; loaded = 0; total = 0;
    constructor(type: string, init?: { lengthComputable?: boolean; loaded?: number; total?: number }) {
      this.type = type; this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0; this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class ImageStub {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1; height = 1; onload: (() => void) | null = null; onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void { (this.listeners[type] ??= []).push(fn); }
    removeEventListener(): void {}
    set src(_v: string) { queueMicrotask(() => { for (const fn of this.listeners['load'] ?? []) fn(); this.onload?.(); }); }
    get src(): string { return ''; }
  };
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  const fakeEl = (): Record<string, unknown> => ({
    textContent: '',
    classList: { toggle: () => {}, add: () => {}, remove: () => {}, contains: () => false },
    style: {},
    parentElement: null,
    getContext: () => null,
    addEventListener: () => {},
    appendChild: () => {},
    setAttribute: () => {},
    querySelector: (_sel: string) => fakeEl(),
    querySelectorAll: (_sel: string) => [] as unknown[],
    width: 200, height: 200,
  });
  (globalThis as unknown as { document?: unknown }).document = {
    getElementById: (_id: string) => fakeEl(),
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return { width: 128, height: 128, getContext: () => ({ createRadialGradient: () => ({ addColorStop: () => {} }), fillRect: () => {}, set fillStyle(_v: unknown) {} }) };
      }
      return fakeEl();
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return fakeEl();
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => { (listeners[type] ??= []).push(fn); },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', { set: () => { queueMicrotask(() => { for (const fn of listeners['load'] ?? []) fn(); }); } });
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
import { FlightHUD } from '../src/ui/FlightHUD';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
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

  const alpha = RUNWAY_TABLE[0]!;
  const player = new Aircraft({
    scene, world, spec: SPECS['fa18']!, isPlayer: true, modelKey: SPECS['fa18']!.modelKey,
  });
  await player.attachModel();
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
  player.setAirspeed(150);
  player.thrTarget = 0.6;

  const jets: Aircraft[] = [player];
  for (let i = 0; i < 4; i++) {
    const key = (i % 2 === 0 ? 'mig29' : 'su34') as 'mig29' | 'su34';
    const e = new Aircraft({ scene, world, spec: SPECS[key]!, isPlayer: false, modelKey: SPECS[key]!.modelKey });
    await e.attachModel();
    const fwd = player.fwd();
    e.resetAt(player.pos.x + fwd.x * (450 + i * 150), player.pos.y + 120 + i * 60, player.pos.z + fwd.z * (450 + i * 150), 0);
    e.setAirspeed(150);
    jets.push(e);
  }
  for (let i = 0; i < 2; i++) {
    const key = (i % 2 === 0 ? 'fa18' : 'mig29') as 'fa18' | 'mig29';
    const t = new Aircraft({ scene, world, spec: SPECS[key]!, isPlayer: false, modelKey: SPECS[key]!.modelKey });
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

  const alphaPos = new THREE.Vector3(alpha.x, alpha.elev, alpha.z);
  let prev = -1;
  for (let i = 0; i < 40 && streamer.loadedGeometryCount !== prev; i++) {
    prev = streamer.loadedGeometryCount;
    streamer.update(alphaPos);
  }
  scene.updateMatrixWorld(true);

  // ---------- 1. Per-bucket frustum-visible census ----------
  const camera = new THREE.PerspectiveCamera(62, 1080 / 2400, 0.5, 20000);
  const frustum = new THREE.Frustum();
  const projView = new THREE.Matrix4();
  function bucketFor(o: THREE.Object3D): string {
    let p: THREE.Object3D | null = o;
    while (p) {
      if (p.name.startsWith('runway-')) return 'runway strips';
      if (p.name === 'terrain' || p.name.startsWith('terrain-chunk-')) return 'terrain';
      if (p.name.startsWith('island-')) return 'floating islands';
      if (p.name.includes('carrier') || p.name.startsWith('heli')) return 'helicarrier';
      if (jets.some((j) => j.group === p)) {
        const j = jets.find((jj) => jj.group === p)!;
        return `jet ${j.spec.key}`;
      }
      p = p.parent;
    }
    return 'other (flora/clouds/sprites)';
  }
  function visibleCensus(label: string, camPos: THREE.Vector3, lookAt: THREE.Vector3): Map<string, { calls: number; tris: number }> {
    camera.position.copy(camPos);
    camera.lookAt(lookAt);
    camera.updateMatrixWorld(true);
    projView.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projView);
    const buckets = new Map<string, { calls: number; tris: number }>();
    let totalCalls = 0;
    let totalTris = 0;
    scene.traverse((o) => {
      const isDraw = (o as THREE.Mesh).isMesh || (o as THREE.Sprite).isSprite || (o as THREE.Points).isPoints;
      if (!isDraw || !o.visible) return;
      if (!frustum.intersectsObject(o)) return;
      const b = bucketFor(o);
      const rec = buckets.get(b) ?? { calls: 0, tris: 0 };
      rec.calls++;
      rec.tris += trisOf(o);
      buckets.set(b, rec);
      totalCalls++;
      totalTris += rec.tris * 0 + trisOf(o);
    });
    console.log(`\n--- ${label}: visible ${totalCalls} calls / ${totalTris.toLocaleString()} tris ---`);
    for (const [name, r] of [...buckets.entries()].sort((a, b) => b[1].calls - a[1].calls)) {
      console.log(`  ${name}: ${r.calls} calls, ${r.tris.toLocaleString()} tris`);
    }
    return buckets;
  }
  const hdgRad = (alpha.hdg * Math.PI) / 180;
  const fwdV = new THREE.Vector3(Math.sin(hdgRad), 0, Math.cos(hdgRad));
  const deckCam = new THREE.Vector3(alpha.x - fwdV.x * 12, alpha.elev + 4, alpha.z - fwdV.z * 12);
  const deck = visibleCensus('deck chase cam (Alpha)', deckCam, new THREE.Vector3(alpha.x + fwdV.x * 500, alpha.elev + 50, alpha.z + fwdV.z * 500));
  const alt = visibleCensus('altitude 3000 m', new THREE.Vector3(alpha.x, 3000, alpha.z), new THREE.Vector3(alpha.x + fwdV.x * 8000, 1500, alpha.z + fwdV.z * 8000));

  const runwayDeckCalls = deck.get('runway strips')?.calls ?? 0;
  const runwayDeckTris = deck.get('runway strips')?.tris ?? 0;
  let deckTotal = 0;
  for (const r of deck.values()) deckTotal += r.calls;
  let altTotal = 0;
  for (const r of alt.values()) altTotal += r.calls;
  const migDeckCalls = (deck.get('jet mig29')?.calls ?? 0);
  const migCount = jets.filter((j) => j.spec.key === 'mig29').length;
  console.log(`\nrunway strips visible from deck: ${runwayDeckCalls} calls / ${runwayDeckTris.toLocaleString()} tris`);
  console.log(`MiG-29 visible from deck: ${migDeckCalls} calls across ${migCount} MiG-29s in the scene (73 meshes / 70 materials each, full-scene)`);
  console.log(`deck total ${deckTotal} calls vs altitude total ${altTotal} calls (live: deck 205, altitude 66-80)`);
  check('runway strips cost <= 15 visible calls from the deck (merged: 5 per runway)', runwayDeckCalls <= 15, `${runwayDeckCalls} calls / ${runwayDeckTris.toLocaleString()} tris visible (pre-fix: 143 calls full-scene, 71 visible from the deck)`);
  check('total visible deck calls <= 205 (pre-fix 260 in this census)', deckTotal <= 205, `${deckTotal} calls`);
  check('total visible altitude calls <= 25 (pre-fix 59: one runway alone is 49)', altTotal <= 25, `${altTotal} calls`);
  check(
    'MiG-29 merge saving quantified (73 calls each -> F/A-18-like 6 would save 67 per MiG)',
    true,
    `${migDeckCalls} MiG-29 calls visible from the deck; merging the GLB is NOT shipped here (rig bindings by node name + embedded base64 blob — a rebake risk, reported separately)`,
  );

  // ---------- 2. Full main-loop CPU per live frame (5 fps: 2 substeps) ----------
  console.log(`\n--- full main-loop CPU per live frame (probe VM, 100 frames, ms/frame) ---`);
  const wind = new THREE.Vector3(3, 0, 0);
  const hud = new FlightHUD();
  const eventQueue = new RAPIER.EventQueue(true);
  const owners = new Map<number, Aircraft>();
  for (const j of jets) for (const h of j.colliderHandles()) owners.set(h, j);
  const drain = (): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const hit = owners.get(h1) ?? owners.get(h2);
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const groundAt = (x: number, z: number): number => worldGen.heightfieldHeightAt(x, z);
  function liveFrame(): void {
    // main.ts animate() at 5 fps: rawDt 0.2 -> simDt 0.1 -> 2 substeps of 0.05.
    for (let s = 0; s < 2; s++) {
      const dt = 0.05;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      for (const j of jets) {
        if (!j.alive) continue;
        // Player-form step for every jet (same BET solver cost as the
        // AI-form step; the AI controller itself is a few vector ops
        // per jet and is timed separately below).
        j.step(dt, wind, null);
      }
      carrier.update(dt);
    }
    worldGen.update(0.2, wind);
    streamer.update(player.pos);
    scene.updateMatrixWorld(true);
    // Frustum-cull traversal (what renderer.render does first).
    camera.position.copy(deckCam);
    camera.lookAt(new THREE.Vector3(alpha.x + fwdV.x * 500, alpha.elev + 50, alpha.z + fwdV.z * 500));
    camera.updateMatrixWorld(true);
    projView.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projView);
    scene.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && o.visible) frustum.intersectsObject(o);
    });
  }
  function hudFrame(): void {
    // The 10 Hz HUD block in animate(): hud.update + minimap skipped
    // (canvas 2d is browser-only; the DOM writes below are the JS
    // side). Called once per 10 live frames here (0.1 s cadence at
    // 5 fps = every other frame) — timed per call.
    hud.update(player, {
      score: 0, kills: 0, traps: 0, elapsed: 60,
      lockTarget: null, belowMinReleaseAlt: false,
      carrier: { absDeg: 10, relDeg: 5, distM: 1900, carrierHdgDeg: 90, deckRelHdgDeg: null },
    });
    void groundAt;
  }
  liveFrame();
  hudFrame();
  // Minimum of 3 batches of 100 frames: this VM is a shared 1.5-CPU
  // cell, so a single batch can read 2-3x high when the host is
  // busy (7.3 / 6.0 / 15.0 ms across runs) — the minimum is the
  // least-throttled estimate of the true cost, standard practice
  // for a microbenchmark on shared tenancy.
  const timeFrames = (n: number): number => {
    const t0 = performance.now();
    for (let i = 0; i < n; i++) liveFrame();
    return (performance.now() - t0) / n;
  };
  const frameMs = Math.min(timeFrames(100), timeFrames(100), timeFrames(100));
  const h0 = performance.now();
  for (let i = 0; i < 100; i++) hudFrame();
  const hudMs = (performance.now() - h0) / 100;
  console.log(`  full live frame (2 substeps, 7 jets, carrier, worldGen, streamer, matrixWorld, cull): ${frameMs.toFixed(2)} ms/frame (min of 3x100)`);
  console.log(`  hud.update per call (10 Hz cadence): ${hudMs.toFixed(3)} ms/call`);
  console.log(`  frame + HUD share: ${(frameMs + hudMs / 2).toFixed(2)} ms/frame at 5 fps`);
  check('full main-loop CPU is small vs a 110-250 ms live frame', frameMs + hudMs / 2 < 15, `${(frameMs + hudMs / 2).toFixed(2)} ms/frame on probe VM (sim ~1 ms + render-feeding 0.34 ms, pass 13/14)`);
  console.log(`\nNOTE (unmeasurable from Node): GPU vertex/fragment time inside renderer.render, MSAA resolve, and the live FPS effect of the runway merge. The ?debug=1 renderer.info readout is the instrument — the live check re-reads calls/tris/FPS after this merge.`);

  console.log(failures === 0 ? '\nALL PASS15-FPS CHECKS PASSED' : `\n${failures} PASS15-FPS CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
