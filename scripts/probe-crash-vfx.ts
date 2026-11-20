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
//   +29 s and expired by +31 s (old rule deleted pieces on first
//   ground contact, so the field was gone within ~3 s)
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
        return { width: 128, height: 128, getContext: () => ({ createRadialGradient: () => ({ addColorStop: () => {} }), fillRect: () => {}, set fillStyle(_v: unknown) {} }) };
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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };

  // Steepest slope (71-deg cliff) for the stall flight, and a flat
  // base 300 m downhill of a 40-deg slope face for the fast impact.
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
  const cgl = Math.hypot(cliff.gx, cliff.gz) || 1;
  const cux = cliff.gx / cgl, cuz = cliff.gz / cgl;
  const chdg = Math.atan2(cux, cuz) * 180 / Math.PI;
  const fgl = Math.hypot(face.gx, face.gz) || 1;
  const fux = face.gx / fgl, fuz = face.gz / fgl;
  const fhdg = Math.atan2(fux, fuz) * 180 / Math.PI;
  const baseX = face.x - fux * 300, baseZ = face.z - fuz * 300;
  console.log(`cliff at (${cliff.x},${cliff.z}) grad=${cliff.g.toFixed(2)}; face at (${face.x},${face.z}) grad=${face.g.toFixed(2)}; flat base at (${baseX.toFixed(0)},${baseZ.toFixed(0)}) grad=${Math.hypot((heightAt(baseX + 60, baseZ) - heightAt(baseX - 60, baseZ)) / 120, (heightAt(baseX, baseZ + 60) - heightAt(baseX, baseZ - 60)) / 120).toFixed(3)}`);

  /** Build an aircraft wired exactly like main.ts makeAircraft +
   *  handleDeaths (same calls, same order). Returns the aircraft and
   *  the per-run VFX record. */
  function makeRun(key: 'fa18' | 'f22', vfx: CrashVFXManager): {
    player: Aircraft;
    drain: () => void;
    detachEvents: Array<{ seg: string | null }>;
    attachedPlumes: THREE.Group[];
  } {
    const spec = SPECS[key]!;
    const detachEvents: Array<{ seg: string | null }> = [];
    const attachedPlumes: THREE.Group[] = [];
    const player = new Aircraft({
      scene, world, spec, isPlayer: true, modelKey: spec.modelKey, deck,
      events: {
        onDetachSegment: (_a, segment, pos, vel) => {
          detachEvents.push({ seg: segment ? segment.name || 'unnamed-node' : null });
          if (segment) vfx.spawnDebris(segment, world, pos, vel);
        },
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
    return { player, drain, detachEvents, attachedPlumes };
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
    for (const d of V.debris) {
      if (meshHasName(d.mesh, 'torn_wing') || meshHasName(d.mesh, 'torn_tail')) tornPieces++;
      const p = d.body.translation();
      const gh = heightAt(p.x, p.z);
      if (p.y < gh - 0.5) debrisBelowGround++;
      if (p.y - gh >= -0.5 && p.y - gh <= 1.0 && Math.abs(d.body.linvel().y) < 2) debrisResting++;
    }
    let scorchOpacity = 0;
    let rimMax = -Infinity;
    let rimMin = Infinity;
    for (const s of V.scorches) {
      scorchOpacity = Math.max(scorchOpacity, (s.mesh.material as THREE.MeshBasicMaterial).opacity);
      // Rim points in the decal's own plane (it is tilted to the
      // slope since pass 11): centre + right*cos + up*sin, r = 5.
      const q = s.mesh.quaternion;
      const right = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
      const up2 = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
      const c = s.mesh.position;
      for (let k = 0; k < 8; k++) {
        const a = (k / 8) * Math.PI * 2;
        const rx = c.x + (right.x * Math.cos(a) + up2.x * Math.sin(a)) * 5;
        const rz = c.z + (right.z * Math.cos(a) + up2.z * Math.sin(a)) * 5;
        const ry = c.y + (right.y * Math.cos(a) + up2.y * Math.sin(a)) * 5;
        const delta = heightAt(rx, rz) - ry;
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
      scorch: V.scorches.length,
      scorchOpacity,
      scorchRimMax: rimMax === -Infinity ? 0 : rimMax,
      scorchRimMin: rimMin === Infinity ? 0 : rimMin,
    };
  }

  /** Step the world + VFX for 31 s after impact, sampling at
   *  SAMPLE_TIMES. Returns the samples keyed by time. */
  function sampleAfterImpact(vfx: CrashVFXManager, player: Aircraft, attachedPlumes: THREE.Group[]): Map<number, VfxSample> {
    const out = new Map<number, VfxSample>();
    out.set(0, sample(vfx, player, attachedPlumes, 0));
    let next = 1;
    for (let i = 1; i <= 31 * 60; i++) {
      const t = i / 60;
      world.timestep = 1 / 60;
      world.step(eventQueue);
      vfx.update(1 / 60);
      if (next < SAMPLE_TIMES.length && t >= SAMPLE_TIMES[next]!) {
        out.set(SAMPLE_TIMES[next]!, sample(vfx, player, attachedPlumes, SAMPLE_TIMES[next]!));
        next++;
      }
    }
    return out;
  }

  /** The shared VFX assertions, run for one crashed jet. */
  function verify(key: string, tag: string, player: Aircraft, detachEvents: Array<{ seg: string | null }>, samples: Map<number, VfxSample>, flatGround: boolean): void {
    const at = (t: number): VfxSample => samples.get(t)!;
    console.log(`  [${tag}] samples: ` + [0, 0.9, 3, 10, 29, 31].map((t) => {
      const s = at(t);
      return `t+${t}s boom=${s.boomVisible ? 'Y' : '-'}op${s.boomOpacity.toFixed(2)}/${s.boomScale.toFixed(0)}m plume=${s.plumes}(att${s.attachedPlumes})op${s.plumeMaxOpacity.toFixed(2)} debris=${s.debris}(torn${s.tornPieces}) scorch=${s.scorch}op${s.scorchOpacity.toFixed(2)}`;
    }).join(' | '));
    check(`${key} ${tag}: explosion fires at impact, bright and jet-sized`, at(0).boomVisible && at(0).boomOpacity >= 0.9 && at(0).boomScale >= 10, `vis=${at(0).boomVisible} op=${at(0).boomOpacity.toFixed(2)} scale=${at(0).boomScale.toFixed(0)}m`);
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
    check(`${key} ${tag}: debris field still present at +29 s (30 s life)`, at(29).debris >= 1, `debris=${at(29).debris}`);
    check(`${key} ${tag}: debris expired by +31 s`, at(31).debris === 0, `debris=${at(31).debris}`);
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
      player.pitchIn = 0.15;
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
    }
  }

  console.log(failures === 0 ? '\nALL CRASH-VFX CHECKS PASSED' : `\n${failures} CRASH-VFX CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
