// probe-crash-vfx.ts — pass 11 (stall-impact wreck VFX): verify the
// crash VFX sequence through the LIVE crash path for a hard fast stall
// impact, for fa18 and f22 with REAL GLBs loaded.
//
// Two scenarios, both through Aircraft.step() + the main.ts wiring
// (onDetachSegment -> spawnDebris, onCrash -> handleDeaths replica:
// explosion + scorch + attached fire/smoke + 4 debris boxes):
//
// A. STALL IMPACT (live path, flown): throttle 0, slight nose-down
//    from 165 kt at a 71-deg cliff — the jet stalls, tumbles and
//    crashes 'hard landing' onto the slope (~10 s). Asserts the stall
//    happened and the full VFX sequence follows.
// B. FAST HARD IMPACT (live signature: 165 kt, vs -11.65): staged
//    initial conditions — 10 m above flat ground, nose-down 8 deg,
//    85 m/s forward with -16 m/s vertical, gear down — then step()
//    takes it from there; the crash decision (onSurface gate, vs
//    check, crash('hard landing')) runs through the real code. The
//    staging is documented, not hidden: a free-flown fast impact is
//    not reliably reachable (the post-stall mush + terrain-following
//    level the jet out, and a gear-up face strike bounces/beaches
//    instead of crashing — the onSurface gate reads the
//    post-resolution velocity, so a hard bounce flips relVy positive
//    and the vs check never runs; reported separately, the gate is
//    outside this workstream's files). Gear down for B because the
//    gear-up gate (1.35 m) misses the gear-up resting height.
//
// What each scenario asserts at sampled times after impact:
// - explosion fired, bright at birth, STILL bright at +0.9 s (the old
//   0.9 s sprite was already gone), ended by +2.6 s
// - fire attached to the wreck, still burning at +3 s and +10 s (the
//   old 6 s plumes were gone by +6 s), tracking the wreck
// - debris field present at +1/+3/+10 s, torn wing + torn tail pieces
//   among it, resting on the ground (scenario B), still present at
//   +29 s and NOT expired by +31 s (pass 14: 90 s life — the old
//   30 s life expired the field before the player looked at +75 s
//   real; expiry at +91 s is owned by probe-wreck-watch)
// - scorch present, and conforming to the slope (old flat disc was
//   buried uphill / floating downhill on a mountainside)
// - wreck visible, wing + tail detached
//
// Usage: node_modules/.bin/esbuild scripts/probe-crash-vfx.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-crash-vfx.mjs
//   && node /tmp/probe-crash-vfx.mjs

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
const SAMPLE_TIMES = [0, 0.25, 0.5, 0.9, 1.5, 2.0, 2.6, 3, 6, 10, 12, 29, 31];

interface VfxSample {
  t: number;
  boomVisible: boolean;
  boomOpacity: number;
  boomScale: number;
  plumes: number;
  attachedPlumes: number;
  plumeMaxOpacity: number;
  plumeWorldDist: number; // max distance of an attached plume group from the wreck
  debris: number;
  tornPieces: number;
  debrisResting: number; // pieces within 1 m above local ground and |vy| < 2
  debrisBelowGround: number;
  /** Least-settled piece at this sample (highest centre above the
   *  ground / fastest): the flake diagnostic — when the resting
   *  check fails, this says WHICH piece and why. Format
   *  `h=<centre-gh>m vy=<m/s> spd=<m/s> ang=<rad/s>`. */
  worstPiece: string;
  scorch: number;
  scorchOpacity: number;
  scorchRimMax: number; // max terrain-minus-decal over the decal rim
  scorchRimMin: number;
}

type BoomRec = { sprite: THREE.Sprite; t: number; scale: number };
type PlumeRec = { group: THREE.Group; life: number; maxLife: number; parent: THREE.Object3D | null; baseOpacity: number };
type DebrisRec = { mesh: THREE.Object3D; body: RAPIER.RigidBody; life: number; age: number };
type ScorchRec = { mesh: THREE.Mesh; age: number };

function meshHasName(mesh: THREE.Object3D, name: string): boolean {
  let found = false;
  mesh.traverse((o) => { if (o.name === name) found = true; });
  return found;
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
  // Live parity: the live terrain is shifted with the world (the
  // streamer registers its chunks with the origin), so the probe's
  // heightfield body is registered too — an origin shift during a
  // scenario must move the ground under the jet, not leave the jet
  // flying over a stale collider.
  // (terrainBody is registered above, next to its creation.)
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };

  // Steepest slope (71-deg cliff) for the stall flight, and a flat
  // base 300 m downhill of a 40-deg slope face for the fast impact.
  // Factored: scenario C shifts the origin first, so the sites must
  // be re-searched in the shifted frame (the terrain under a scene
  // point changes with the cumulative shift).
  function findSites(): {
    cliff: { x: number; z: number; g: number; gx: number; gz: number };
    face: { x: number; z: number; g: number; gx: number; gz: number };
    baseX: number;
    baseZ: number;
  } {
    let cliff = { x: 0, z: 0, g: 0, gx: 0, gz: 0 };
    let face = { x: 0, z: 0, g: 0, gx: 0, gz: 0 };
    for (let x = -5000; x <= 5000; x += 120) {
      for (let z = -5000; z <= 5000; z += 120) {
        const gx = (heightAt(x + 60, z) - heightAt(x - 60, z)) / 120;
        const gz = (heightAt(x, z + 60) - heightAt(x, z - 60)) / 120;
        const g = Math.hypot(gx, gz);
        if (g > cliff.g) cliff = { x, z, g, gx, gz };
        if (g >= 0.45 && g <= 0.85 && g > face.g) face = { x, z, g, gx, gz };
      }
    }
    const fgl = Math.hypot(face.gx, face.gz) || 1;
    const fux = face.gx / fgl, fuz = face.gz / fgl;
    return { cliff, face, baseX: face.x - fux * 300, baseZ: face.z - fuz * 300 };
  }
  const sites = findSites();
  const { cliff, face } = sites;
  const { baseX, baseZ } = sites;
  const cgl = Math.hypot(cliff.gx, cliff.gz) || 1;
  const cux = cliff.gx / cgl, cuz = cliff.gz / cgl;
  const chdg = Math.atan2(cux, cuz) * 180 / Math.PI;
  const fgl = Math.hypot(face.gx, face.gz) || 1;
  const fux = face.gx / fgl, fuz = face.gz / fgl;
  const fhdg = Math.atan2(fux, fuz) * 180 / Math.PI;
  console.log(`cliff at (${cliff.x},${cliff.z}) grad=${cliff.g.toFixed(2)}; face at (${face.x},${face.z}) grad=${face.g.toFixed(2)}; flat base at (${baseX.toFixed(0)},${baseZ.toFixed(0)}) grad=${Math.hypot((heightAt(baseX + 60, baseZ) - heightAt(baseX - 60, baseZ)) / 120, (heightAt(baseX, baseZ + 60) - heightAt(baseX, baseZ - 60)) / 120).toFixed(3)}`);

  // ---- Live frame driver (main.ts animate()): raw frame times at
  // browser FPS 4-14 (rawDt 0.07-0.25 s), split into substeps of
  // at most 0.05 s — the Rapier timestep AND the controller/VFX dt
  // are the substep dt, not 1/60. The fixed-dt harness hides every
  // failure that only shows up at these step sizes.
  const RAW_DT_PATTERN = [0.25, 0.07, 0.16, 0.09, 0.2, 0.12];
  let rawIdx = 0;
  function liveFrame(stepSub: (dt: number) => void): void {
    const rawDt = RAW_DT_PATTERN[rawIdx++ % RAW_DT_PATTERN.length]!;
    const sub = Math.max(1, Math.ceil(rawDt / 0.05));
    const dt = rawDt / sub;
    for (let s = 0; s < sub; s++) stepSub(dt);
  }

  /** Live handleDeaths replica (main.ts): the live game does NOT wire
   *  AircraftEvents.onCrash at all — makeAircraft wires only
   *  onDetachSegment/onFlares — and detects deaths by polling
   *  prevAlive once per substep, AFTER vfx.update(dt). The VFX calls
   *  and their order are copied verbatim (explosion 1+fuel, scorch,
   *  attached fire+smoke for an instant crash, 4 debris boxes). */
  function makeLiveDeaths(vfx: CrashVFXManager, attachedPlumes: THREE.Group[]): {
    handleDeaths: (all: Aircraft[]) => void;
    prevAlive: Map<Aircraft, boolean>;
  } {
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
            vfx.spawnDebris(
              null,
              world,
              a.pos.clone(),
              new THREE.Vector3(Math.cos(ang) * sp, 5 + Math.random() * 10, Math.sin(ang) * sp),
            );
          }
        }
        prevAlive.set(a, a.alive);
      }
    };
    return { handleDeaths, prevAlive };
  }

  /** Build an aircraft wired exactly like main.ts makeAircraft +
   *  handleDeaths (same calls, same order). Returns the aircraft and
   *  the per-run VFX record. `live` switches the wiring to the LIVE
   *  one: NO onCrash event (main.ts never wires it) — deaths are
   *  detected by the handleDeaths poll returned here, called once
   *  per substep after vfx.update(dt). */
  function makeRun(key: 'fa18' | 'f22', vfx: CrashVFXManager, live = false): {
    player: Aircraft;
    drain: () => void;
    detachEvents: Array<{ seg: string | null }>;
    attachedPlumes: THREE.Group[];
    handleDeaths: (all: Aircraft[]) => void;
  } {
    const spec = SPECS[key]!;
    const detachEvents: Array<{ seg: string | null }> = [];
    const attachedPlumes: THREE.Group[] = [];
    const { handleDeaths } = makeLiveDeaths(vfx, attachedPlumes);
    const player = new Aircraft({
      scene, world, spec, isPlayer: true, modelKey: spec.modelKey, deck,
      events: {
        onDetachSegment: (_a, segment, pos, vel) => {
          detachEvents.push({ seg: segment ? segment.name || 'unnamed-node' : null });
          if (segment) vfx.spawnDebris(segment, world, pos, vel);
        },
        // Fixed-dt scenarios A/B keep the onCrash replica (the old
        // wiring). Live scenarios pass live=true and rely on the
        // handleDeaths poll instead — exactly like main.ts.
        ...(!live ? {
          onCrash: (a, _reason) => {
            // handleDeaths replica (main.ts): explosion scaled by fuel,
            // scorch, attached fire + smoke for an instant crash, and
            // 4 debris boxes — in that order.
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
          },
        } : {}),
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
    return { player, drain, detachEvents, attachedPlumes, handleDeaths };
  }

  /** Remove one scenario's VFX residue (debris bodies, scorches,
   *  plumes, booms) from the shared world/scene/origin so later
   *  scenarios fly a clean world. Each scenario builds a fresh
   *  CrashVFXManager, but the old manager's debris BODIES stay in
   *  the Rapier world (nothing ever removes them) — a later jet can
   *  clip a leftover box and crash somewhere else entirely (the
   *  f22 A scorch flake: fa18's D3/E debris moved the crash site
   *  after the probe grew more crash scenarios). Called at the end
   *  of every scenario, after the player is disposed. */
  function cleanupVfx(vfx: CrashVFXManager): void {
    const V = vfx as unknown as { booms: BoomRec[]; plumes: PlumeRec[]; debris: DebrisRec[]; scorches: ScorchRec[] };
    for (const d of V.debris) {
      scene.remove(d.mesh);
      origin.unregisterBody(d.body);
      world.removeRigidBody(d.body);
    }
    V.debris.length = 0;
    for (const s of V.scorches) {
      scene.remove(s.mesh);
      origin.unregisterObject(s.mesh);
      s.mesh.geometry.dispose();
      (s.mesh.material as THREE.Material).dispose();
    }
    V.scorches.length = 0;
    for (const p of V.plumes) {
      if (p.parent) p.parent.remove(p.group);
      else {
        scene.remove(p.group);
        origin.unregisterObject(p.group);
      }
    }
    V.plumes.length = 0;
    for (const b of V.booms) {
      scene.remove(b.sprite);
      origin.unregisterObject(b.sprite);
    }
    V.booms.length = 0;
  }

  /** Sample the VFX state at time t after impact. */
  function sample(vfx: CrashVFXManager, player: Aircraft, attachedPlumes: THREE.Group[], t: number): VfxSample {
    const V = vfx as unknown as { booms: BoomRec[]; plumes: PlumeRec[]; debris: DebrisRec[]; scorches: ScorchRec[] };
    const boom = V.booms[0];
    let plumeMaxOpacity = 0;
    let attachedCount = 0;
    for (const p of V.plumes) {
      if (p.parent) attachedCount++;
      for (const c of p.group.children) {
        const s = c as THREE.Sprite;
        plumeMaxOpacity = Math.max(plumeMaxOpacity, (s.material as THREE.SpriteMaterial).opacity);
      }
    }
    let plumeWorldDist = 0;
    const wp = new THREE.Vector3();
    for (const g of attachedPlumes) {
      if (!g.parent) continue;
      g.getWorldPosition(wp);
      plumeWorldDist = Math.max(plumeWorldDist, wp.distanceTo(player.pos));
    }
    let tornPieces = 0;
    let debrisResting = 0;
    let debrisBelowGround = 0;
    let worstScore = -Infinity;
    let worstPiece = 'none';
    for (const d of V.debris) {
      if (meshHasName(d.mesh, 'torn_wing') || meshHasName(d.mesh, 'torn_tail')) tornPieces++;
      const p = d.body.translation();
      const gh = heightAt(p.x, p.z);
      if (p.y < gh - 0.5) debrisBelowGround++;
      const lv = d.body.linvel();
      const av = d.body.angvel();
      const spd = Math.hypot(lv.x, lv.y, lv.z);
      const ang = Math.hypot(av.x, av.y, av.z);
      // Settled (pass 12): a piece counts as resting when it is
      // near the ground AND effectively still — a long torn panel
      // may legitimately rest propped on a corner (centre up to
      // ~2.5 m), which the old strict 1.0 m centre-height window
      // failed even though the piece had stopped (the 5/6 flake:
      // worst piece h=2.01 vy=-0.17 spd=1.44 ang=0.02). The
      // CrashVFXManager fix (angular damping 2.0 + friction 0.8)
      // makes pieces lie flat and stop sliding, so a settled piece
      // should now also be low — the 2.5 m window is a tolerance,
      // not the norm.
      if (p.y - gh >= -0.5 && p.y - gh <= 2.5 && Math.abs(lv.y) < 2 && spd < 2 && ang < 0.5) debrisResting++;
      // Least-settled score: centre height above the 2.5 m resting
      // window, plus vertical and total speed, plus spin — a long
      // torn panel propped on its end scores high on height, a
      // still-bouncing piece on speed.
      const score = Math.max(0, p.y - gh - 2.5) + Math.abs(lv.y) + spd * 0.1 + ang * 0.2;
      if (score > worstScore) {
        worstScore = score;
        worstPiece = `h=${(p.y - gh).toFixed(2)} vy=${lv.y.toFixed(2)} spd=${spd.toFixed(2)} ang=${ang.toFixed(2)}`;
      }
    }
    let scorchOpacity = 0;
    let rimMax = -Infinity;
    let rimMin = Infinity;
    for (const s of V.scorches) {
      scorchOpacity = Math.max(scorchOpacity, (s.mesh.material as THREE.MeshBasicMaterial).opacity);
      // Pass 13: the decal's vertices are baked + terrain-snapped
      // (mesh quaternion identity), so read the actual vertex
      // heights instead of reconstructing a tilted disc from the
      // quaternion — the old reconstruction assumed a rigid flat
      // disc and could not see the snap.
      const posAttr = s.mesh.geometry.getAttribute('position') as THREE.BufferAttribute;
      for (let vi = 0; vi < posAttr.count; vi++) {
        const wx = s.mesh.position.x + posAttr.getX(vi);
        const wy = s.mesh.position.y + posAttr.getY(vi);
        const wz = s.mesh.position.z + posAttr.getZ(vi);
        const delta = heightAt(wx, wz) - wy;
        rimMax = Math.max(rimMax, delta);
        rimMin = Math.min(rimMin, delta);
      }
    }
    return {
      t,
      boomVisible: boom ? boom.sprite.visible : false,
      boomOpacity: boom ? boom.sprite.material.opacity : 0,
      boomScale: boom ? boom.sprite.scale.x : 0,
      plumes: V.plumes.length,
      attachedPlumes: attachedCount,
      plumeMaxOpacity,
      plumeWorldDist,
      debris: V.debris.length,
      tornPieces,
      debrisResting,
      debrisBelowGround,
      worstPiece,
      scorch: V.scorches.length,
      scorchOpacity,
      scorchRimMax: rimMax === -Infinity ? 0 : rimMax,
      scorchRimMin: rimMin === Infinity ? 0 : rimMin,
    };
  }

  /** Step the world + VFX for 31 s after impact, sampling at
   *  SAMPLE_TIMES. Returns the samples keyed by time. `liveDt`
   *  drives it with the live frame pattern (variable rawDt split
   *  into ≤0.05 s substeps) instead of fixed 1/60. */
  function sampleAfterImpact(vfx: CrashVFXManager, player: Aircraft, attachedPlumes: THREE.Group[], liveDt = false): Map<number, VfxSample> {
    const out = new Map<number, VfxSample>();
    out.set(0, sample(vfx, player, attachedPlumes, 0));
    let next = 1;
    let simT = 0;
    const stepOnce = (dt: number): void => {
      world.timestep = dt;
      world.step(eventQueue);
      vfx.update(dt);
      simT += dt;
      // Epsilon: 1860 accumulated 1/60 steps sum to 30.999999999999996,
      // not 31 — without it the +31 s sample is never taken.
      if (next < SAMPLE_TIMES.length && simT >= SAMPLE_TIMES[next]! - 1e-9) {
        out.set(SAMPLE_TIMES[next]!, sample(vfx, player, attachedPlumes, SAMPLE_TIMES[next]!));
        next++;
      }
    };
    if (!liveDt) {
      for (let i = 1; i <= 31 * 60; i++) stepOnce(1 / 60);
    } else {
      while (simT < 31) liveFrame(stepOnce);
    }
    return out;
  }

  /** The shared VFX assertions, run for one crashed jet. */
  function verify(key: string, tag: string, player: Aircraft, detachEvents: Array<{ seg: string | null }>, samples: Map<number, VfxSample>, flatGround: boolean): void {
    const at = (t: number): VfxSample => samples.get(t)!;
    console.log(`  [${tag}] samples: ` + [0, 0.9, 3, 10, 29, 31].map((t) => {
      const s = at(t);
      return `t+${t}s boom=${s.boomVisible ? 'Y' : '-'}op${s.boomOpacity.toFixed(2)}/${s.boomScale.toFixed(0)}m plume=${s.plumes}(att${s.attachedPlumes})op${s.plumeMaxOpacity.toFixed(2)} debris=${s.debris}(torn${s.tornPieces}) rest=${s.debrisResting}/${s.debris} worst[${s.worstPiece}] scorch=${s.scorch}op${s.scorchOpacity.toFixed(2)}`;
    }).join(' | '));
    check(`${key} ${tag}: explosion fires at impact, bright and jet-sized`, at(0).boomVisible && at(0).boomOpacity >= 0.9 && at(0.25).boomScale >= 10, `vis=${at(0).boomVisible} op=${at(0).boomOpacity.toFixed(2)} scale@0s=${at(0).boomScale.toFixed(0)}m scale@0.25s=${at(0.25).boomScale.toFixed(0)}m (the t+0 scale is racy: a boom spawned by the poll after the last update of its frame has not been sized yet — the +0.25 s scale is the jet-sized read)`);
    check(`${key} ${tag}: explosion is STILL bright at +0.9 s (old 0.9 s sprite was already gone)`, at(0.9).boomVisible && at(0.9).boomOpacity > 0.5, `vis=${at(0.9).boomVisible} op=${at(0.9).boomOpacity.toFixed(2)} scale=${at(0.9).boomScale.toFixed(0)}m`);
    check(`${key} ${tag}: explosion has ended by +2.6 s`, !at(2.6).boomVisible, `vis=${at(2.6).boomVisible} op=${at(2.6).boomOpacity.toFixed(2)}`);
    check(`${key} ${tag}: fire attached to the wreck at +1 s`, at(1.5).attachedPlumes >= 2 && at(1.5).plumeMaxOpacity >= 0.4, `attached=${at(1.5).attachedPlumes} maxOp=${at(1.5).plumeMaxOpacity.toFixed(2)}`);
    check(`${key} ${tag}: attached fire tracks the wreck (within 2 m)`, at(1.5).plumeWorldDist <= 2, `dist=${at(1.5).plumeWorldDist.toFixed(2)}m`);
    check(`${key} ${tag}: fire still burning at +3 s`, at(3).attachedPlumes >= 1 && at(3).plumeMaxOpacity >= 0.4, `attached=${at(3).attachedPlumes} maxOp=${at(3).plumeMaxOpacity.toFixed(2)}`);
    check(`${key} ${tag}: fire still burning at +10 s (old 6 s plumes were gone by +6 s)`, at(10).attachedPlumes >= 1, `attached=${at(10).attachedPlumes} plumes=${at(10).plumes}`);
    check(`${key} ${tag}: debris field present at +1 s (6 pieces: 4 boxes + wing + tail)`, at(1.5).debris >= 6, `debris=${at(1.5).debris}`);
    check(`${key} ${tag}: debris field still present at +3 s (old rule had 0-2 left)`, at(3).debris >= 6, `debris=${at(3).debris}`);
    check(`${key} ${tag}: torn wing + torn tail pieces in the debris field`, at(3).tornPieces >= 2, `torn=${at(3).tornPieces} detachEvents=${JSON.stringify(detachEvents)}`);
    check(`${key} ${tag}: debris field still present at +10 s`, at(10).debris >= 4, `debris=${at(10).debris}`);
    check(`${key} ${tag}: debris field still present at +29 s (90 s life)`, at(29).debris >= 1, `debris=${at(29).debris}`);
    check(`${key} ${tag}: debris NOT expired by +31 s (90 s life — expiry at +91 s is owned by probe-wreck-watch)`, at(31).debris >= 1, `debris=${at(31).debris}`);
    check(`${key} ${tag}: no debris piece fell through the terrain`, at(10).debrisBelowGround === 0, `below=${at(10).debrisBelowGround}`);
    if (flatGround) {
      check(`${key} ${tag}: debris resting on the ground at +10 s`, at(10).debrisResting === at(10).debris && at(10).debris > 0, `resting=${at(10).debrisResting}/${at(10).debris}`);
    }
    check(`${key} ${tag}: scorch mark present and readable at +1 s`, at(1.5).scorch >= 1 && at(1.5).scorchOpacity >= 0.38, `scorch=${at(1.5).scorch} op=${at(1.5).scorchOpacity.toFixed(2)}`);
    check(`${key} ${tag}: scorch conforms to the slope (rim within 0.4 m above / 0.6 m below the surface; old flat disc was +0.7 buried / -1.0 floating)`, at(1.5).scorchRimMax < 0.4 && at(1.5).scorchRimMin > -0.6, `rimMax=${at(1.5).scorchRimMax.toFixed(2)} rimMin=${at(1.5).scorchRimMin.toFixed(2)}`);
    check(`${key} ${tag}: wreck stays visible, wing + tail detached`, player.group.visible === true && player.wingDetached && player.tailDetached, `visible=${player.group.visible} wing=${player.wingDetached} tail=${player.tailDetached}`);
  }

  for (const key of ['fa18', 'f22'] as const) {
    // ---------- Scenario A: stall impact, flown live ----------
    {
      origin.reset();
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, detachEvents, attachedPlumes } = makeRun(key, vfx);
      await player.attachModel();
      const rig = (player as unknown as { rig: { bound: Record<string, unknown>; missing: string[] } | null }).rig;
      console.log(`\n=== ${key} A (stall impact) modelStatus=${player.modelStatus} rig bound=[${rig ? Object.keys(rig.bound).join(',') : 'NO-RIG'}] wing/tail bound=${rig ? !!rig.bound['wing_L'] && !!rig.bound['tail_section'] : false} ===`);
      const sx = cliff.x - cux * 700, sz = cliff.z - cuz * 700;
      const sy = heightAt(sx, sz) + 55;
      player.resetAt(sx, sy, sz, chdg);
      player.setAirspeed(85); // 165 kt
      player.gear = false;
      player.grounded = false;
      player.thrTarget = 0;
      player.throttle = 0;
      // Pass 12: full pull to force a genuine stall. The old 0.15
      // relied on the geometric-trim deficiency to stall the jet,
      // but the hands-off 1 g auto-trim (BET) now holds 1 g with
      // |pitchIn| < 0.25, so the jet mushed into the terrain at high
      // AoA without the stall flag ever tripping. A full pull is
      // outside the trim gate and drives AoA past the peak.
      // pitchIn NEGATIVE pulls up (v2 convention).
      player.pitchIn = -1.0;
      let impactedAt = -1;
      let sawStall = false;
      let impactSpeed = 0;
      for (let i = 0; i < 3600; i++) {
        world.timestep = 1 / 60;
        world.step(eventQueue);
        drain();
        player.step(1 / 60, WIND, null);
        vfx.update(1 / 60);
        if (player.stalled) sawStall = true;
        if (!player.alive) { impactedAt = i; impactSpeed = player.vel.length(); break; }
      }
      check(`${key} A: jet stalls during the flight`, sawStall, `sawStall=${sawStall}`);
      check(`${key} A: stall impact crashes through the live path`, impactedAt >= 0 && (player.lastCrashReason === 'hard landing' || player.lastCrashReason === 'terrain'), `impact at t=${(impactedAt / 60).toFixed(2)}s reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(2)}m/s bank=${player.lastCrashBankDeg.toFixed(1)}deg speed=${impactSpeed.toFixed(1)}m/s fuel=${player.fuel.toFixed(2)}`);
      check(`${key} A: detach events fired with real segments (not null)`, detachEvents.length === 2 && detachEvents.every((e) => e.seg !== null), `events=${JSON.stringify(detachEvents)}`);
      const samples = sampleAfterImpact(vfx, player, attachedPlumes);
      verify(key, 'A stall', player, detachEvents, samples, false);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }

    // ---------- Scenario B: fast hard impact, staged conditions ----------
    {
      origin.reset();
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, detachEvents, attachedPlumes } = makeRun(key, vfx);
      await player.attachModel();
      console.log(`\n=== ${key} B (fast impact) modelStatus=${player.modelStatus} ===`);
      const by = heightAt(baseX, baseZ) + 10;
      player.resetAt(baseX, by, baseZ, fhdg);
      const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((8 * Math.PI) / 180, (fhdg * Math.PI) / 180, 0, 'YXZ'));
      player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
      const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
      player.body.setLinvel({ x: f.x * 85, y: -16, z: f.z * 85 }, true);
      player.vel.set(f.x * 85, -16, f.z * 85); // ac.vel is what drain reports as impactVy — respawn() zeroed it, so stage it too
      player.gear = true;
      player.grounded = false;
      player.thrTarget = 0.6;
      player.throttle = 0.6;
      player.pitchIn = 0;
      let impactedAt = -1;
      let impactSpeed = 0;
      for (let i = 0; i < 360; i++) {
        world.timestep = 1 / 60;
        world.step(eventQueue);
        drain();
        player.step(1 / 60, WIND, null);
        vfx.update(1 / 60);
        if (!player.alive) { impactedAt = i; impactSpeed = player.vel.length(); break; }
      }
      check(`${key} B: fast hard impact crashes 'hard landing' with vs <= -9 m/s`, impactedAt >= 0 && player.lastCrashReason === 'hard landing' && player.lastCrashVs <= -9, `impact at t=${(impactedAt / 60).toFixed(2)}s reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(2)}m/s bank=${player.lastCrashBankDeg.toFixed(1)}deg speed=${impactSpeed.toFixed(1)}m/s (${(impactSpeed * 1.944).toFixed(0)}kt)`);
      check(`${key} B: impact speed is fast (>= 50 m/s)`, impactSpeed >= 50, `speed=${impactSpeed.toFixed(1)}m/s`);
      check(`${key} B: detach events fired with real segments (not null)`, detachEvents.length === 2 && detachEvents.every((e) => e.seg !== null), `events=${JSON.stringify(detachEvents)}`);
      const samples = sampleAfterImpact(vfx, player, attachedPlumes);
      verify(key, 'B fast', player, detachEvents, samples, true);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }

    // ---------- Scenario C: stall impact through the LIVE path ----------
    // Same stall flight as A, but driven exactly like main.ts
    // animate(): variable rawDt (FPS 4-14) split into <=0.05 s
    // substeps, an origin shift applied before the flight (the jet
    // flies in the shifted frame), NO onCrash event (main.ts never
    // wires it) — deaths are found by the handleDeaths poll once
    // per substep after vfx.update(dt). The VFX must land on the
    // wreck in the shifted frame, and the debris must still settle.
    {
      origin.reset();
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, detachEvents, attachedPlumes, handleDeaths } = makeRun(key, vfx, true);
      await player.attachModel();
      // Origin shift BEFORE the flight (worldGen.shiftOriginIfNeeded
      // fires on the player's position): everything registered moves
      // by -shift, cumulativeShift grows, heightAt re-frames.
      origin.applyShift(new THREE.Vector3(2500, 0, -1200));
      const cSites = findSites();
      const ccgl = Math.hypot(cSites.cliff.gx, cSites.cliff.gz) || 1;
      const ccux = cSites.cliff.gx / ccgl, ccuz = cSites.cliff.gz / ccgl;
      const cchdg = Math.atan2(ccux, ccuz) * 180 / Math.PI;
      console.log(`\n=== ${key} C (stall, LIVE path) modelStatus=${player.modelStatus} shift=${origin.cumulativeShift.x},${origin.cumulativeShift.z} cliff grad=${cSites.cliff.g.toFixed(2)} ===`);
      const sx = cSites.cliff.x - ccux * 700, sz = cSites.cliff.z - ccuz * 700;
      const sy = heightAt(sx, sz) + 55;
      player.resetAt(sx, sy, sz, cchdg);
      player.setAirspeed(85); // 165 kt
      player.gear = false;
      player.grounded = false;
      player.thrTarget = 0;
      player.throttle = 0;
      // Pass 12: full pull to force a genuine stall (see scenario A
      // — the 0.15 pull is inside the auto-trim gate, so the jet
      // mushed into the terrain without stalling). pitchIn NEGATIVE
      // pulls up (v2 convention).
      player.pitchIn = -1.0;
      let impactedAt = -1;
      let sawStall = false;
      let impactSpeed = 0;
      let simT = 0;
      let firstImpactVy: number | null = null;
      const trace: string[] = [];
      while (simT < 60) {
        liveFrame((dt) => {
          simT += dt;
          const sh = origin.shiftIfNeeded(player.pos);
          if (sh) origin.applyShift(sh);
          world.timestep = dt;
          world.step(eventQueue);
          drain();
          if (firstImpactVy === null) {
            const pc = (player as unknown as { pendingContact: { impactVy: number } | null }).pendingContact;
            if (pc) firstImpactVy = pc.impactVy;
          }
          player.step(dt, WIND, null);
          vfx.update(dt);
          handleDeaths([player]);
          if (player.stalled) sawStall = true;
          // 1 s resolution (capped at 40 rows): the impact/bounce
          // happens in the first seconds — the 5 s sampling missed
          // it, and the fa18 C failure (tumble at 2-12 m AGL for
          // 60 s, no crash) needs the impact moment captured.
          if (Math.abs(simT % 1) < dt && trace.length < 40) {
            trace.push(`t=${simT.toFixed(0)} agl=${(player.pos.y - heightAt(player.pos.x, player.pos.z)).toFixed(0)}m vy=${player.vel.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt aoa=${player.aoaDeg.toFixed(1)} pitch=${(new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x * 180 / Math.PI).toFixed(1)} stalled=${player.stalled} alive=${player.alive}`);
          }
          if (!player.alive && impactedAt < 0) { impactedAt = simT; impactSpeed = player.vel.length(); }
        });
        if (impactedAt >= 0) break;
      }
      console.log(`  [C stall-live] trace: ` + trace.join(' | '));
      check(`${key} C: jet stalls during the live-path flight`, sawStall, `sawStall=${sawStall}`);
      check(`${key} C: stall impact crashes through the LIVE path (poll, variable dt, shifted frame)`, impactedAt >= 0 && (player.lastCrashReason === 'hard landing' || player.lastCrashReason === 'terrain'), `impact at t=${impactedAt.toFixed(2)}s reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(2)}m/s bank=${player.lastCrashBankDeg.toFixed(1)}deg speed=${impactSpeed.toFixed(1)}m/s firstImpactVy=${firstImpactVy === null ? 'none' : firstImpactVy.toFixed(1)}m/s`);
      check(`${key} C: detach events fired with real segments (not null)`, detachEvents.length === 2 && detachEvents.every((e) => e.seg !== null), `events=${JSON.stringify(detachEvents)}`);
      // Frame consistency: the explosion, the scorch and the debris
      // must be AT the wreck in the shifted frame, not left 2 km
      // behind at the pre-shift coordinates. The scorch marks the
      // IMPACT point, and the wreck can slide/tumble away from it
      // after the crash is declared on first contact (pass 12) —
      // so the scorch limit is 20 m, not 5 m: a 2 km shift error
      // still fails by two orders of magnitude, while a 12–14 m
      // slide after an 85 m/s impact is legitimate.
      {
        const V = vfx as unknown as { booms: BoomRec[]; debris: DebrisRec[]; scorches: ScorchRec[] };
        const boomD = V.booms.length ? V.booms[0]!.sprite.position.distanceTo(player.pos) : Infinity;
        const scorchD = V.scorches.length ? V.scorches[0]!.mesh.position.distanceTo(player.pos) : Infinity;
        let debrisMeanD = Infinity;
        if (V.debris.length) {
          const c = new THREE.Vector3();
          for (const d of V.debris) { const p = d.body.translation(); c.x += p.x; c.y += p.y; c.z += p.z; }
          c.divideScalar(V.debris.length);
          debrisMeanD = c.distanceTo(player.pos);
        }
        check(`${key} C: VFX is at the wreck in the shifted frame (boom < 5 m, scorch < 20 m from the impact point, debris mean < 40 m)`, boomD < 5 && scorchD < 20 && debrisMeanD < 40, `boom=${boomD.toFixed(1)}m scorch=${scorchD.toFixed(1)}m debrisMean=${debrisMeanD.toFixed(1)}m wreck=(${player.pos.x.toFixed(0)},${player.pos.y.toFixed(0)},${player.pos.z.toFixed(0)}) shift=(${origin.cumulativeShift.x.toFixed(0)},${origin.cumulativeShift.z.toFixed(0)})`);
      }
      const samples = sampleAfterImpact(vfx, player, attachedPlumes, true);
      verify(key, 'C stall-live', player, detachEvents, samples, false);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }

    // ---------- Scenario C2: fast hard impact, LIVE path, flat ground ----------
    // Same staging as B, live wiring + live dt. Run TWICE — with and
    // without the pre-flight origin shift — ON THE SAME TERRAIN: the
    // sites are found in the unshifted frame (origin.reset() first,
    // before any shift) inside each iteration, and the shift variant
    // stages at scene coords (base - shift), which map to the same
    // original-frame ground (heightAt adds the cumulative shift).
    // If the shift machinery is consistent, the two runs must behave
    // identically; any difference is the shift breaking the ground
    // (collider vs analytic height). This is the flake repro too:
    // the debris-resting check runs on flat ground under live step
    // sizes (Rapier timestep up to 0.05 s).
    for (const useShift of [true, false]) {
      origin.reset();
      const c2Sites = findSites();
      const c2fgl = Math.hypot(c2Sites.face.gx, c2Sites.face.gz) || 1;
      const c2fux = c2Sites.face.gx / c2fgl, c2fuz = c2Sites.face.gz / c2fgl;
      const c2fhdg = Math.atan2(c2fux, c2fuz) * 180 / Math.PI;
      const c2BaseX = c2Sites.face.x - c2fux * 300, c2BaseZ = c2Sites.face.z - c2fuz * 300;
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, detachEvents, attachedPlumes, handleDeaths } = makeRun(key, vfx, true);
      await player.attachModel();
      // Same-terrain staging: the shift variant stages at scene
      // (base - shift) so heightAt (which adds the cumulative
      // shift) reads the same original-frame ground as the
      // no-shift run.
      const sx = useShift ? c2BaseX - 2500 : c2BaseX;
      const sz = useShift ? c2BaseZ + 1200 : c2BaseZ;
      if (useShift) origin.applyShift(new THREE.Vector3(2500, 0, -1200));
      const c2tag = useShift ? 'C2s fast-live-shift' : 'C2n fast-live-noshift';
      console.log(`\n=== ${key} ${c2tag} modelStatus=${player.modelStatus} shift=${origin.cumulativeShift.x},${origin.cumulativeShift.z} stage=(${sx.toFixed(0)},${sz.toFixed(0)}) ground=${heightAt(sx, sz).toFixed(1)} ===`);
      const by = heightAt(sx, sz) + 10;
      player.resetAt(sx, by, sz, c2fhdg);
      const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((8 * Math.PI) / 180, (c2fhdg * Math.PI) / 180, 0, 'YXZ'));
      player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
      const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
      player.body.setLinvel({ x: f.x * 85, y: -16, z: f.z * 85 }, true);
      player.vel.set(f.x * 85, -16, f.z * 85);
      player.gear = true;
      player.grounded = false;
      player.thrTarget = 0.6;
      player.throttle = 0.6;
      player.pitchIn = 0;
      let impactedAt = -1;
      let impactSpeed = 0;
      let simT = 0;
      const c2trace: string[] = [];
      while (simT < 6) {
        liveFrame((dt) => {
          simT += dt;
          world.timestep = dt;
          world.step(eventQueue);
          drain();
          player.step(dt, WIND, null);
          vfx.update(dt);
          handleDeaths([player]);
          if (!player.alive && impactedAt < 0) { impactedAt = simT; impactSpeed = player.vel.length(); }
        });
        if (simT < 2.5 && c2trace.length < 12) {
          c2trace.push(`t=${simT.toFixed(2)} agl=${(player.pos.y - heightAt(player.pos.x, player.pos.z)).toFixed(1)}m vy=${player.vel.y.toFixed(1)} ias=${(player.ias * 1.944).toFixed(0)}kt alive=${player.alive} surf=${player.onSurface} gnd=${player.grounded}`);
        }
        if (impactedAt >= 0) break;
      }
      console.log(`  [${c2tag}] trace: ` + c2trace.join(' | '));
      check(`${key} ${c2tag}: fast hard impact crashes 'hard landing' through the LIVE path`, impactedAt >= 0 && player.lastCrashReason === 'hard landing' && player.lastCrashVs <= -9, `impact at t=${impactedAt.toFixed(2)}s reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(2)}m/s bank=${player.lastCrashBankDeg.toFixed(1)}deg speed=${impactSpeed.toFixed(1)}m/s (${(impactSpeed * 1.944).toFixed(0)}kt)`);
      check(`${key} ${c2tag}: detach events fired with real segments (not null)`, detachEvents.length === 2 && detachEvents.every((e) => e.seg !== null), `events=${JSON.stringify(detachEvents)}`);
      const samples = sampleAfterImpact(vfx, player, attachedPlumes, true);
      verify(key, c2tag, player, detachEvents, samples, true);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }

    // ---------- Scenario D: steep dive must crash, not auto-recover ----------
    // The live F-18 cases: 441 kt at -26,976 fpm from 276 ft, and
    // 746 kt at -66,874 fpm from 35,842 ft — hands-off (neutral
    // stick, no pull), live dt. A hard fast dive into the ground
    // must crash; levelling off, bouncing off the terrain, or
    // climbing away is the auto-recovery failure. D1 stages the
    // low case exactly (84 m AGL, 227 m/s, 37 deg dive); D2 stages
    // the high case's dive angle and speed at 3000 m AGL (the same
    // mechanism — trim pull-out or Rapier bounce — with a shorter
    // fall; altitude only changes time-to-impact).
    for (const dive of [
      { tag: 'D1 low', agl: 84, speed: 227, pitchDownDeg: 37, thr: 0.6, maxT: 5, expectT: 3 },
      { tag: 'D2 high', agl: 3000, speed: 384, pitchDownDeg: 62, thr: 1.0, maxT: 30, expectT: 25 },
      // Pass 13 D3: SHALLOW dives — the pass-12 trim (TRIM_K=2.0)
      // holds the steep/fast dives (D1/D2) but the F-22 pulls out
      // of every 30-deg dive (measured: 2-3 g at AoA ~0 deg, vy
      // decaying to 0 over 14-22 s) and the F-18 pulls out of a
      // 45-deg dive given altitude (from 10,000 m: level at 54 s,
      // min AGL 1924 m) — the live F-18 case (-34,286 fpm at
      // ~7,000 ft to +8,419 fpm) dove from ~35,842 ft, so the slow
      // pull-out had the time to complete. A hard fast dive into
      // the ground must crash at ANY dive angle and altitude.
      { tag: 'D3a shallow-150', agl: 2100, speed: 150, pitchDownDeg: 30, thr: 0.6, maxT: 30, expectT: 25 },
      { tag: 'D3b shallow-250', agl: 2100, speed: 250, pitchDownDeg: 30, thr: 0.6, maxT: 30, expectT: 25 },
      { tag: 'D3c shallow-350', agl: 2100, speed: 350, pitchDownDeg: 30, thr: 0.6, maxT: 30, expectT: 25 },
      { tag: 'D3d high-alt', agl: 10000, speed: 250, pitchDownDeg: 45, thr: 0.6, maxT: 60, expectT: 58 },
    ] as const) {
      origin.reset();
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, detachEvents, attachedPlumes, handleDeaths } = makeRun(key, vfx, true);
      await player.attachModel();
      console.log(`\n=== ${key} ${dive.tag} (steep dive, LIVE path) ===`);
      const dy = heightAt(baseX, baseZ) + dive.agl;
      player.resetAt(baseX, dy, baseZ, fhdg);
      const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((dive.pitchDownDeg * Math.PI) / 180, (fhdg * Math.PI) / 180, 0, 'YXZ'));
      player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
      const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
      player.body.setLinvel({ x: f.x * dive.speed, y: f.y * dive.speed, z: f.z * dive.speed }, true);
      player.vel.copy(new THREE.Vector3(f.x * dive.speed, f.y * dive.speed, f.z * dive.speed));
      player.gear = false;
      player.grounded = false;
      player.thrTarget = dive.thr;
      player.throttle = dive.thr;
      player.pitchIn = 0;
      player.rollIn = 0;
      player.yawIn = 0;
      let simT = 0;
      let crashedAt = -1;
      let bounced = false;
      let sawHardDescent = false;
      let levelledOff = false;
      let levelT = 0;
      let minAgl = Infinity;
      let lastVy = f.y * dive.speed;
      // First-contact impactVy: the patch-spec premise — the
      // collision event fires at first touch, so the impactVy the
      // controller reads (pendingContact) is the PRE-resolution
      // impact speed, not the post-bounce climb. Record the first
      // non-null pendingContact after drain(), before step()
      // consumes it.
      let firstImpactVy: number | null = null;
      const trace: string[] = [];
      while (simT < dive.maxT) {
        liveFrame((dt) => {
          simT += dt;
          world.timestep = dt;
          world.step(eventQueue);
          drain();
          if (firstImpactVy === null) {
            const pc = (player as unknown as { pendingContact: { impactVy: number } | null }).pendingContact;
            if (pc) firstImpactVy = pc.impactVy;
          }
          player.step(dt, WIND, null);
          vfx.update(dt);
          handleDeaths([player]);
          const agl = player.pos.y - heightAt(player.pos.x, player.pos.z);
          minAgl = Math.min(minAgl, agl);
          lastVy = player.vel.y;
          // Bounce (state-based — the flip spans several substeps as
          // Rapier resolves the penetration, so a single-substep
          // vy-flip test misses it): a hard descent low, then a
          // climb while still low and alive — the contact solver
          // ejecting the jet instead of the crash logic.
          if (player.alive && agl < 100 && lastVy < -50) sawHardDescent = true;
          if (player.alive && sawHardDescent && agl < 100 && lastVy > 5) bounced = true;
          // Level-off: sustained near-zero sink while high and
          // alive — the trim/aero pull-out, hands-off. Gated on
          // the jet NEVER having been low (minAgl > 500): a
          // post-bounce climb back above 500 m is a bounce
          // artefact, not a level-off, and must not count.
          if (player.alive && agl > 500 && minAgl > 500 && lastVy > -20) levelT += dt;
          else levelT = 0;
          if (levelT > 2) levelledOff = true;
          if (!player.alive && crashedAt < 0) crashedAt = simT;
        });
        if (Math.abs(simT % 1) < 0.2 && trace.length < 40) {
          trace.push(`t=${simT.toFixed(1)} agl=${(player.pos.y - heightAt(player.pos.x, player.pos.z)).toFixed(0)}m vy=${lastVy.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt aoa=${player.aoaDeg.toFixed(1)} pitch=${(new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x * 180 / Math.PI).toFixed(1)} alive=${player.alive}`);
        }
        if (crashedAt >= 0 || levelledOff) break;
      }
      console.log(`  [${dive.tag}] trace: ` + trace.join(' | '));
      check(`${key} ${dive.tag}: steep dive crashes within ${dive.expectT} s (no auto-recovery)`, crashedAt >= 0 && crashedAt <= dive.expectT, `crashedAt=${crashedAt < 0 ? 'never' : crashedAt.toFixed(2) + 's'} reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(1)}m/s minAgl=${minAgl.toFixed(1)}m finalVy=${lastVy.toFixed(1)}m/s finalAgl=${(player.pos.y - heightAt(player.pos.x, player.pos.z)).toFixed(0)}m firstImpactVy=${firstImpactVy === null ? 'none' : firstImpactVy.toFixed(1)}m/s`);
      check(`${key} ${dive.tag}: no Rapier bounce off the terrain (vy never flips to a climb while low and alive)`, !bounced, `bounced=${bounced} firstImpactVy=${firstImpactVy === null ? 'none' : firstImpactVy.toFixed(1)}m/s`);
      check(`${key} ${dive.tag}: no hands-off level-off at altitude`, !levelledOff, `levelledOff=${levelledOff}`);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }

    // ---------- Scenario E: the wreck must READ as a wreck, persistently ----------
    // Pass 13 (task 1): the crash lines prove crash() + handleDeaths
    // run live (the vs values ARE the debug log), and the v4.3 live
    // note records a "brief orange glow faded ~15 s" — so the
    // explosion and the fire DO fire; what fails live is the
    // PERSISTENT state. By the time anyone looks at the wreck the
    // 2.5 s explosion and the 12 s fire are gone, and what remains
    // does not read as a crash: the donor GLB is a single-mesh
    // airframe (fa18 'ground', f22 'F-22-airframe') that sits level
    // and factory-clean, the 4 debris boxes are identical light-grey
    // 4 x 0.5 x 2.5 m slabs (0x8a97a3), and the scorch is a perfect
    // dark circle that reads as a shadow. The live report — "sat
    // whole on the grass with only a dark shadow beneath it" plus
    // "grey rectangular slabs near every crash site" — is exactly
    // that state. E stages a fast hard impact through the live
    // path, then asserts the PERSISTENT wreck state at +15/+20 s:
    // charred airframe, missiles off the rails, wreck tilted onto
    // a wingtip, fire still burning, charred irregular debris, an
    // irregular scorch — and that the explosion sprite survives a
    // mid-life origin shift at the wreck (it was the one VFX object
    // never registered with the origin).
    {
      origin.reset();
      const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
      const { player, drain, handleDeaths } = makeRun(key, vfx, true);
      await player.attachModel();
      console.log(`\n=== ${key} E (persistent wreck state) ===`);
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
      let shiftedAt: number | null = null;
      let boomDistAfterShift = Infinity;
      // Sample the persistent wreck state at +3/+15/+20 s.
      interface ESample { attached: number; plumeOp: number; }
      const eSamples = new Map<number, ESample>();
      const takeE = (t: number): void => {
        const V = vfx as unknown as { plumes: PlumeRec[] };
        let attached = 0;
        let plumeOp = 0;
        for (const p of V.plumes) {
          if (p.parent) {
            attached++;
            for (const c of p.group.children) plumeOp = Math.max(plumeOp, (c as THREE.Sprite).material.opacity);
          }
        }
        eSamples.set(t, { attached, plumeOp });
      };
      while (simT < 30) {
        liveFrame((dt) => {
          simT += dt;
          world.timestep = dt;
          world.step(eventQueue);
          drain();
          if (player.alive) player.step(dt, WIND, null);
          vfx.update(dt);
          handleDeaths([player]);
          if (!player.alive && crashedAt < 0) crashedAt = simT;
          if (crashedAt >= 0) {
            const since = simT - crashedAt;
            // Mid-boom-life origin shift (+1.0 s): the explosion
            // sprite must move with the world, not stay behind.
            if (shiftedAt === null && since >= 1.0) {
              shiftedAt = simT;
              origin.applyShift(new THREE.Vector3(2500, 0, -1200));
              const V = vfx as unknown as { booms: BoomRec[] };
              const bt = player.body.translation();
              if (V.booms.length) {
                boomDistAfterShift = V.booms[0]!.sprite.position.distanceTo(new THREE.Vector3(bt.x, bt.y, bt.z));
              }
            }
            for (const st of [3, 15, 20]) {
              if (!eSamples.has(st) && since >= st) takeE(st);
            }
          }
        });
        if (crashedAt >= 0 && simT - crashedAt >= 20.5) break;
      }
      check(`${key} E: fast hard impact crashes through the LIVE path`, crashedAt >= 0 && player.lastCrashReason === 'hard landing', `crashedAt=${crashedAt < 0 ? 'never' : crashedAt.toFixed(2) + 's'} reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(1)}m/s`);
      // Wreck char: the largest mesh of the model is the airframe
      // (fa18 'ground' 18.79 m, f22 'F-22-airframe' 19.00 m); its
      // material colour starts at #ffffff and must be charred dark.
      {
        const model = (player as unknown as { model: THREE.Object3D | null }).model;
        let biggest: THREE.Mesh | null = null;
        let biggestLen = 0;
        model?.traverse((o) => {
          if ((o as THREE.Mesh).isMesh) {
            const box = new THREE.Box3().setFromObject(o);
            const size = new THREE.Vector3();
            box.getSize(size);
            const len = Math.max(size.x, size.y, size.z);
            if (len > biggestLen) { biggestLen = len; biggest = o as THREE.Mesh; }
          }
        });
        let maxChan = 1;
        if (biggest) {
          const mats = Array.isArray(biggest.material) ? biggest.material : [biggest.material];
          for (const m of mats) {
            const c = (m as THREE.MeshStandardMaterial).color;
            if (c) maxChan = Math.min(maxChan, Math.max(c.r, c.g, c.b));
          }
        }
        check(`${key} E: wreck airframe is charred (largest mesh material dark, not factory #ffffff)`, biggest !== null && maxChan <= 0.35, `mesh=${biggest?.name ?? 'none'} len=${biggestLen.toFixed(1)}m maxChan=${maxChan.toFixed(2)}`);
        if (key === 'fa18') {
          const pack = model?.getObjectByName('Weapon_pack');
          check(`${key} E: missiles off the rails (Weapon_pack hidden on the wreck)`, pack !== undefined && pack.visible === false, `Weapon_pack visible=${pack?.visible}`);
        }
        const rollDeg = Math.abs(new THREE.Euler().setFromQuaternion(player.group.quaternion, 'YXZ').z) * 180 / Math.PI;
        check(`${key} E: wreck rests tilted on a wingtip (roll 8-30 deg, not level like a parked jet)`, rollDeg >= 8 && rollDeg <= 30, `roll=${rollDeg.toFixed(1)}deg`);
      }
      {
        const s15 = eSamples.get(15)!;
        const s20 = eSamples.get(20)!;
        check(`${key} E: wreck fire still burning at +15 s (old 12 s plumes were gone)`, s15.attached >= 1 && s15.plumeOp >= 0.3, `attached=${s15.attached} maxOp=${s15.plumeOp.toFixed(2)}`);
        check(`${key} E: wreck fire still present at +20 s`, s20.attached >= 1, `attached=${s20.attached}`);
      }
      {
        // Generic debris pieces: the 4 null-segment pieces are plain
        // Meshes (torn wing/tail arrive as Groups holding the
        // segment). They must be charred dark, not light grey, and
        // not all the same slab size. Pass 15 (worker F, item 8):
        // the pieces are torn panels, not BoxGeometry boxes, so
        // sizes come from each mesh's bounding box, not from
        // geometry parameters (which no longer exist).
        const V = vfx as unknown as { debris: DebrisRec[]; scorches: ScorchRec[] };
        const boxes = V.debris.filter((d) => d.mesh instanceof THREE.Mesh);
        let maxChan = 0;
        const sizes = new Set<string>();
        for (const d of boxes) {
          const m = d.mesh as THREE.Mesh;
          const mat = m.material as THREE.MeshLambertMaterial;
          if (mat.color) maxChan = Math.max(maxChan, Math.max(mat.color.r, mat.color.g, mat.color.b));
          const bb = new THREE.Box3().setFromObject(m);
          const sz = new THREE.Vector3();
          bb.getSize(sz);
          sizes.add(`${sz.x.toFixed(1)}x${sz.y.toFixed(1)}x${sz.z.toFixed(1)}`);
        }
        check(`${key} E: debris pieces are charred wreckage, not light-grey slabs`, boxes.length >= 4 && maxChan < 0.35, `pieces=${boxes.length} maxChan=${maxChan.toFixed(2)} (old 0x8a97a3 = 0.64)`);
        check(`${key} E: debris pieces vary in size (not 4 identical slabs)`, sizes.size >= 2, `distinctSizes=${sizes.size} [${[...sizes].join(', ')}]`);
        // Pass 15 (worker F, item 8): the pieces must be torn
        // panels, not boxes — see probe-pass15-slabs for the shape
        // measurement; this is the cheap type guard here.
        const boxCount = boxes.filter((d) => ((d.mesh as THREE.Mesh).geometry as THREE.BufferGeometry).type === 'BoxGeometry').length;
        check(`${key} E: debris pieces are torn panels, not BoxGeometry slabs`, boxCount === 0, `boxGeometries=${boxCount}/${boxes.length}`);
        const scorch = V.scorches[0];
        check(`${key} E: scorch is an irregular blotch, not a perfect disc (textured, not a flat circle)`, scorch !== undefined && (scorch.mesh.material as THREE.MeshBasicMaterial).map !== null, `map=${(scorch?.mesh.material as THREE.MeshBasicMaterial)?.map ? 'yes' : 'none'}`);
      }
      check(`${key} E: explosion sprite stays at the wreck across a mid-life origin shift (< 5 m from the wreck body)`, boomDistAfterShift < 5, `boomDist=${boomDistAfterShift === Infinity ? 'no-boom' : boomDistAfterShift.toFixed(1) + 'm'} shift=${origin.cumulativeShift.x.toFixed(0)},${origin.cumulativeShift.z.toFixed(0)}`);
      origin.unregisterBody(player.body);
      player.dispose();
      cleanupVfx(vfx);
    }
  }

  console.log(failures === 0 ? '\nALL CRASH-VFX CHECKS PASSED' : `\n${failures} CRASH-VFX CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
