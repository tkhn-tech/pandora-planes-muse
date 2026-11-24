// probe-pass15-slabs.ts — pass 15, worker F, item 8: black/grey slabs
// at crash sites.
//
// The v4.9 live check found large flat black/dark-grey rectangular
// slabs at every crash site, and three black slabs on the grass ahead
// of the jet before the F-22/F-15E crashes (the repeat crash point —
// leftover debris from the previous round). Pass 13/14 charred the
// debris and varied its size, but the pieces are still BoxGeometry
// boxes: the generic pieces are 1.8-3.4 m wide x 0.12-0.32 m thick
// x 3-5.5 m long, and the torn wing/tail fallbacks are 6.1 x 0.3 x
// 4.2 m / 4.8 x 0.3 x 2.8 m flat panels. A Rapier cuboid settles on
// its largest face, so every piece lies flat and presents a full
// rectangle to the chase camera above — the slab read is the GEOMETRY,
// not the colour.
//
// Method: stage an F-22 fast hard impact through the LIVE path
// (same staging as probe-crash-vfx scenario C2n: variable rawDt
// split into <=0.05 s substeps, live wind 3 m/s, real GLB, the
// handleDeaths poll — no onCrash event), let the debris settle to
// +10 s, then measure the SHAPE of every debris piece from its
// geometry (not a state flag):
//   - geometry type: a BoxGeometry IS a rectangular slab (FAIL);
//   - top-face planarity: the range of local Y among the top-face
//     vertices — 0.00 m for a box (perfectly flat), > 0.05 m for
//     torn wreckage;
//   - footprint taper: convex-hull area of the vertices projected
//     onto the ground, over the bounding-rectangle area — 1.00 for
//     a rectangle, < 0.97 for a tapered panel;
//   - rest tilt: angle between the piece's thin axis and world up —
//     ~0 deg for a flat-lying box;
//   - scorch: CircleGeometry with an irregular texture (already
//     non-rectangular — informational, must stay that way).
// A piece counts as slab-like when it is a box AND flat AND
// rectangular in footprint. The probe FAILS pre-fix (6/6 pieces
// slab-like) and PASSES after (0 slab-like).
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass15-slabs.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass15-slabs.mjs
//   && node /tmp/probe-pass15-slabs.mjs
//   (delete /tmp/probe-pass15-slabs.mjs after running)

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string; lengthComputable = false; loaded = 0; total = 0;
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
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1; height = 1; onload: (() => void) | null = null; onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void { (this.listeners[type] ??= []).push(fn); }
    removeEventListener(): void {}
    set src(_v: string) { queueMicrotask(() => { for (const fn of this.listeners['load'] ?? []) fn(); this.onload?.(); }); }
  };
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return { width: 128, height: 128, getContext: () => ({ createRadialGradient: () => ({ addColorStop: () => {} }), fillRect: () => {}, beginPath: () => {}, moveTo: () => {}, lineTo: () => {}, closePath: () => {}, fill: () => {}, arc: () => {}, set fillStyle(_v: unknown) {} }) };
      }
      return {};
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
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
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { CrashVFXManager } from '../src/combat/CrashVFXManager';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(3, 0, 0);
const RAW_DT_PATTERN = [0.25, 0.07, 0.16, 0.09, 0.2, 0.12];
let rawIdx = 0;
function liveFrame(stepSub: (dt: number) => void): void {
  const rawDt = RAW_DT_PATTERN[rawIdx++ % RAW_DT_PATTERN.length]!;
  const sub = Math.max(1, Math.ceil(rawDt / 0.05));
  const dt = rawDt / sub;
  for (let s = 0; s < sub; s++) stepSub(dt);
}

type BoomRec = { sprite: THREE.Sprite; t: number; scale: number };
type PlumeRec = { group: THREE.Group; life: number; maxLife: number; parent: THREE.Object3D | null; baseOpacity: number };
type DebrisRec = { mesh: THREE.Object3D; body: RAPIER.RigidBody; life: number; age: number };
type ScorchRec = { mesh: THREE.Mesh; age: number };

/** Convex-hull area (monotone chain) of 2D points, in m^2. */
function hullArea(pts: Array<{ x: number; z: number }>): number {
  const p = [...pts].sort((a, b) => (a.x - b.x) || (a.z - b.z));
  const cross = (o: { x: number; z: number }, a: { x: number; z: number }, b: { x: number; z: number }): number =>
    (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
  const lower: Array<{ x: number; z: number }> = [];
  const upper: Array<{ x: number; z: number }> = [];
  for (const q of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2]!, lower[lower.length - 1]!, q) <= 0) lower.pop();
    lower.push(q);
  }
  for (let i = p.length - 1; i >= 0; i--) {
    const q = p[i]!;
    while (upper.length >= 2 && cross(upper[upper.length - 2]!, upper[upper.length - 1]!, q) <= 0) upper.pop();
    upper.push(q);
  }
  lower.pop();
  upper.pop();
  const hull = [...lower, ...upper];
  let a = 0;
  for (let i = 0; i < hull.length; i++) {
    const q0 = hull[i]!;
    const q1 = hull[(i + 1) % hull.length]!;
    a += q0.x * q1.z - q1.x * q0.z;
  }
  return Math.abs(a) / 2;
}

interface PieceShape {
  label: string;
  geoType: string;
  dims: [number, number, number];
  flatness: number;
  topRange: number;
  footprintRatio: number;
  restTiltDeg: number;
  slabLike: boolean;
}

/** Measure one debris piece's shape from its geometry + world pose.
 *  All shape metrics are in the piece's LOCAL geometry space (the
 *  position attribute), so the rest pose (yaw/tilt on the ground)
 *  cannot mask the shape: a box rotated 30 deg still has a
 *  rectangular local footprint, and a tapered panel still tapers. */
function measurePiece(mesh: THREE.Object3D, label: string): PieceShape | null {
  let target: THREE.Mesh | null = null;
  mesh.traverse((o) => {
    if (!target && (o as THREE.Mesh).isMesh) target = o as THREE.Mesh;
  });
  if (!target) return null;
  const m = target as THREE.Mesh;
  const geo = m.geometry as THREE.BufferGeometry;
  const posAttr = geo.getAttribute('position') as THREE.BufferAttribute;
  if (!posAttr) return null;
  // Local bounding box (dims + flatness) from the raw vertices.
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (let i = 0; i < posAttr.count; i++) {
    const x = posAttr.getX(i), y = posAttr.getY(i), z = posAttr.getZ(i);
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
  }
  const size = new THREE.Vector3(maxX - minX, maxY - minY, maxZ - minZ);
  const dims: [number, number, number] = [size.x, size.y, size.z];
  const maxD = Math.max(size.x, size.y, size.z);
  const minD = Math.min(size.x, size.y, size.z);
  const flatness = maxD > 0 ? minD / maxD : 1;
  // Top-face planarity: the range of local Y over the TOP (+y)
  // face's vertices — BoxGeometry's groups are ordered px, nx, py,
  // ny, pz, nz, so group 2 is the top face and its index range
  // still identifies those vertices after the deformation (the
  // fold bends the top face up, so its vertices span ~0.5-1.4 m
  // in Y; a flat box's top face is a single Y, range 0). A narrow
  // "near maxY" band does NOT work here: the jitter spreads the
  // highest vertices over ~0.25 m, so the band catches a single
  // vertex and reads 0. Fall back to the band method for
  // non-BoxGeometry-derived pieces (real GLB segments).
  let topRange: number;
  const idx = geo.index;
  const py = geo.groups?.[2];
  if (idx && py && py.count > 0) {
    let tMin = Infinity;
    let tMax = -Infinity;
    for (let k = py.start; k < py.start + py.count; k++) {
      const y = posAttr.getY(idx.getX(k));
      tMin = Math.min(tMin, y);
      tMax = Math.max(tMax, y);
    }
    topRange = tMax - tMin;
  } else {
    let topMin = Infinity;
    let topMax = -Infinity;
    for (let i = 0; i < posAttr.count; i++) {
      const y = posAttr.getY(i);
      if (y > maxY - Math.max(0.05, (maxY - minY) * 0.05)) {
        topMin = Math.min(topMin, y);
        topMax = Math.max(topMax, y);
      }
    }
    topRange = topMax === -Infinity ? 0 : topMax - topMin;
    if (topMax === -Infinity) {
      const ys: number[] = [];
      for (let i = 0; i < posAttr.count; i++) ys.push(posAttr.getY(i));
      ys.sort((a, b) => a - b);
      const top = ys.slice(Math.floor(ys.length * 0.8));
      topRange = top[top.length - 1]! - top[0]!;
    }
  }
  // Footprint taper in LOCAL space: convex-hull area of the (x,z)
  // vertices over the local bounding-rectangle area. A box is
  // exactly its w x l rectangle (ratio 1.00); a tapered panel is
  // a trapezoid (ratio ~0.8).
  const pts: Array<{ x: number; z: number }> = [];
  for (let i = 0; i < posAttr.count; i++) pts.push({ x: posAttr.getX(i), z: posAttr.getZ(i) });
  const rectArea = Math.max((maxX - minX) * (maxZ - minZ), 1e-6);
  const footprintRatio = hullArea(pts) / rectArea;
  // Rest tilt: angle between the piece's LOCAL thin axis (smallest
  // local dimension) and world up, via the mesh world quaternion.
  const q = new THREE.Quaternion();
  m.getWorldQuaternion(q);
  const axes = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
  const dimArr = [size.x, size.y, size.z];
  let thinIdx = 0;
  for (let i = 1; i < 3; i++) if (dimArr[i]! < dimArr[thinIdx]!) thinIdx = i;
  const thinWorld = axes[thinIdx]!.clone().applyQuaternion(q);
  const restTiltDeg = (Math.acos(Math.min(1, Math.abs(thinWorld.y))) * 180) / Math.PI;
  const isBox = geo.type === 'BoxGeometry';
  const slabLike = isBox && flatness < 0.15 && footprintRatio > 0.98 && topRange < 0.02;
  return { label, geoType: geo.type, dims, flatness, topRange, footprintRatio, restTiltDeg, slabLike };
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({ seed: 1337, chunkX: 0, chunkZ: 0, size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1 });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) for (let ix = 0; ix < res; ix++) hfHeights[ix * res + iz] = heights[iz * res + ix]!;
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights, new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE))
      .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };

  // Flat base 300 m downhill of a 40-deg slope face (same site search
  // as probe-crash-vfx scenario B).
  let face = { x: 0, z: 0, g: 0, gx: 0, gz: 0 };
  for (let x = -5000; x <= 5000; x += 120) {
    for (let z = -5000; z <= 5000; z += 120) {
      const gx = (heightAt(x + 60, z) - heightAt(x - 60, z)) / 120;
      const gz = (heightAt(x, z + 60) - heightAt(x, z - 60)) / 120;
      const g = Math.hypot(gx, gz);
      if (g >= 0.45 && g <= 0.85 && g > face.g) face = { x, z, g, gx, gz };
    }
  }
  const fgl = Math.hypot(face.gx, face.gz) || 1;
  const fux = face.gx / fgl, fuz = face.gz / fgl;
  const fhdg = Math.atan2(fux, fuz) * 180 / Math.PI;
  const baseX = face.x - fux * 300, baseZ = face.z - fuz * 300;
  console.log(`flat base at (${baseX.toFixed(0)}, ${baseZ.toFixed(0)}) ground=${heightAt(baseX, baseZ).toFixed(1)}`);

  const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
  const spec = SPECS['f22']!;
  const attachedPlumes: THREE.Group[] = [];
  const prevAlive = new Map<Aircraft, boolean>();
  const handleDeaths = (all: Aircraft[]): void => {
    for (const a of all) {
      const was = prevAlive.get(a) ?? true;
      if (was && !a.alive) {
        vfx.spawnExplosion(a.pos.clone(), 1 + a.fuel);
        vfx.scorchDecal(a.pos.clone());
        if (!a.crashedFromDying) {
          attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0xff7722, a.group));
          attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0x1a1a1a, a.group));
        }
        for (let i = 0; i < 4; i++) {
          const ang = Math.random() * Math.PI * 2;
          const sp = 10 + Math.random() * 20;
          vfx.spawnDebris(null, world, a.pos.clone(), new THREE.Vector3(Math.cos(ang) * sp, 5 + Math.random() * 10, Math.sin(ang) * sp));
        }
      }
      prevAlive.set(a, a.alive);
    }
  };
  const detachEvents: Array<{ seg: string | null }> = [];
  const player = new Aircraft({
    scene, world, spec, isPlayer: true, modelKey: spec.modelKey, deck,
    events: {
      onDetachSegment: (_a, segment, pos, vel) => {
        detachEvents.push({ seg: segment ? segment.name || 'unnamed-node' : null });
        if (segment) vfx.spawnDebris(segment, world, pos, vel);
      },
    },
  });
  player.terrainHeightAt = heightAt;
  origin.registerBody(player.body);
  const owners = new Map<number, Aircraft>();
  for (const h of player.colliderHandles()) owners.set(h, player);
  const drain = (): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const hit = owners.get(h1) ?? owners.get(h2);
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  await player.attachModel();
  console.log(`F-22 modelStatus=${player.modelStatus} tris=${player.modelTris}`);

  // Stage the fast hard impact (probe-crash-vfx B/C2n staging).
  const by = heightAt(baseX, baseZ) + 10;
  player.resetAt(baseX, by, baseZ, fhdg);
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((8 * Math.PI) / 180, (fhdg * Math.PI) / 180, 0, 'YXZ'));
  player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
  const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
  player.body.setLinvel({ x: f.x * 85, y: -16, z: f.z * 85 }, true);
  player.vel.set(f.x * 85, -16, f.z * 85);
  player.gear = true;
  player.grounded = false;
  player.thrTarget = 0.6;
  player.throttle = 0.6;
  player.pitchIn = 0;
  let simT = 0;
  let crashedAt = -1;
  while (simT < 6) {
    liveFrame((dt) => {
      simT += dt;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      player.step(dt, WIND, null);
      vfx.update(dt);
      handleDeaths([player]);
      if (!player.alive && crashedAt < 0) crashedAt = simT;
    });
    if (crashedAt >= 0) break;
  }
  check('F-22 fast hard impact crashes through the LIVE path', crashedAt >= 0 && player.lastCrashReason === 'hard landing', `crashedAt=${crashedAt < 0 ? 'never' : crashedAt.toFixed(2) + 's'} reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(1)}m/s`);
  check('detach events fired with real segments', detachEvents.length === 2 && detachEvents.every((e) => e.seg !== null), `events=${JSON.stringify(detachEvents)}`);

  // Let the debris settle to +10 s after impact (live dt).
  let settleT = 0;
  while (settleT < 10) {
    liveFrame((dt) => {
      settleT += dt;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      vfx.update(dt);
    });
  }

  const V = vfx as unknown as { debris: DebrisRec[]; scorches: ScorchRec[] };
  console.log(`\ndebris pieces at +10 s: ${V.debris.length} (torn among them: ${detachEvents.map((e) => e.seg).join(', ')})`);
  const shapes: PieceShape[] = [];
  for (const d of V.debris) {
    let label = 'generic';
    d.mesh.traverse((o) => {
      if (o.name === 'torn_wing' || o.name === 'torn_tail') label = o.name;
    });
    const s = measurePiece(d.mesh, label);
    if (s) shapes.push(s);
  }
  for (const s of shapes) {
    console.log(
      `  ${s.label}: ${s.geoType} ${s.dims.map((d) => d.toFixed(2)).join('x')}m ` +
        `flat=${s.flatness.toFixed(2)} topRange=${(s.topRange * 100).toFixed(1)}cm ` +
        `footprint=${s.footprintRatio.toFixed(2)} tilt=${s.restTiltDeg.toFixed(1)}deg ${s.slabLike ? 'SLAB-LIKE' : 'torn'}`,
    );
  }
  const slabCount = shapes.filter((s) => s.slabLike).length;
  const boxCount = shapes.filter((s) => s.geoType === 'BoxGeometry').length;
  check('no debris piece is a BoxGeometry (a box IS a rectangular slab)', boxCount === 0, `${boxCount}/${shapes.length} pieces are BoxGeometry`);
  check('no debris piece is slab-like (flat box, flat top, rectangular footprint)', slabCount === 0, `${slabCount}/${shapes.length} pieces slab-like`);
  check(
    'every piece has a non-planar top face (top-face vertex range > 5 cm — torn, not flat)',
    shapes.length > 0 && shapes.every((s) => s.topRange > 0.05),
    `min topRange=${shapes.length ? (Math.min(...shapes.map((s) => s.topRange)) * 100).toFixed(1) : 'n/a'}cm`,
  );
  check(
    'every piece has a tapered footprint (hull/bbox < 0.97 — not a rectangle from above)',
    shapes.length > 0 && shapes.every((s) => s.footprintRatio < 0.97),
    `max footprint=${shapes.length ? Math.max(...shapes.map((s) => s.footprintRatio)).toFixed(2) : 'n/a'}`,
  );
  check('debris field still present at +10 s (6 pieces)', V.debris.length >= 6, `debris=${V.debris.length}`);
  {
    const scorch = V.scorches[0];
    const isCircle = scorch !== undefined && (scorch.mesh.geometry as THREE.BufferGeometry).type === 'CircleGeometry';
    const hasMap = scorch !== undefined && (scorch.mesh.material as THREE.MeshBasicMaterial).map !== null;
    check('scorch is an irregular blotch, not a rectangle (CircleGeometry + irregular texture)', isCircle && hasMap, `geo=${scorch ? (scorch.mesh.geometry as THREE.BufferGeometry).type : 'none'} map=${hasMap ? 'yes' : 'none'}`);
  }

  console.log(failures === 0 ? '\nALL SLAB CHECKS PASSED' : `\n${failures} SLAB CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
