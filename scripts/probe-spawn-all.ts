// probe-spawn-all.ts — spawn survival for ALL SIX aircraft (pass 5).
//
// Live v4 report (Nicholas, 2026-11-18 ~12:50 SGT): a jet "appears and
// immediately explodes" on a RUNWAY spawn (F-35A AND F-22 — not
// F-35A-specific), and on a HELICARRIER spawn it appears FLOATING
// (not seated on the deck) and collides with a disc (a rotor pod
// disc). This driver replicates the live startRound + main-loop order
// as closely as a node driver can — real 12 km terrain + runway boxes,
// FloatingOrigin with the world/carrier/player registered and
// shiftOriginIfNeeded each frame (Alpha is 3,330 m from the origin,
// past the 2,000 m threshold, so the FIRST frame shifts), the SHIELD
// carrier built + reset, carrier.update() before world.step(), the
// live 3 m/s wind, drainCollisionEvents -> notifyContact — and checks
// each of the six jets stays alive and seated, on the runway and on
// the deck, parked and on a takeoff roll, at round start AND on a
// mid-loop deck respawn.
//
// Usage: node_modules/.bin/esbuild scripts/probe-spawn-all.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-spawn-all.mjs
//   && node /tmp/probe-spawn-all.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (Same stubs as probe-carrier-deck.ts.)
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
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(3, 0, 0); // live default windVec
const R2D = 180 / Math.PI;
const KEYS = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const;

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
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix]!;
    }
  }
  const terrainCollider = world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  // NOTE: heightAt must be shift-aware like the live
  // heightfieldGroundHeight (scene coords + cumulativeShift = original
  // frame) — after the first origin shift the aircraft's scene coords
  // are near the origin while the terrain it sits over is Alpha's.
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );

  // Floating origin (live: main.ts registers terrain body, carrier and
  // player; worldGen.shiftOriginIfNeeded runs once per frame).
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);

  // SHIELD carrier, as in main.ts startRound.
  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt,
  });
  await carrier.build();
  const deckBody = carrier.deckBodyRef;
  if (deckBody) {
    for (let i = 0; i < deckBody.numColliders(); i++) {
      owners.set(deckBody.collider(i).handle, 'deck');
    }
  }
  carrier.registerWithFloatingOrigin(origin);
  // Same wrapper as main.ts: point velocity = linear + yawRate x r.
  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.active && carrier.alive;
    },
    deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
    velocity: () => carrier.vel.clone(),
    velocityAt: (x, z) => {
      const rx = x - carrier.pos.x;
      const rz = z - carrier.pos.z;
      const w = carrier.yawRate;
      return new THREE.Vector3(carrier.vel.x + w * rz, carrier.vel.y, carrier.vel.z - w * rx);
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: (_x, _z) => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };
  const resetCarrier = (): void => {
    carrier.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
  };
  resetCarrier();

  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = heightAt;
    origin.registerBody(ac.body);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return ac;
  };
  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      const other = hit === a ? b : a;
      hit.notifyContact(hit.vel.y, other === 'deck');
    });
  };
  // One main-loop substep in live order: carrier moves first, then
  // Rapier integrates, contacts dispatch, then the aircraft steps.
  // After the frame's substeps the origin shift runs (here: every
  // step, since each step is one frame at 60 fps in this driver).
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, WIND, null);
    const shift = origin.shiftIfNeeded(ac.pos);
    if (shift) origin.applyShift(shift);
  };
  const dispose = (ac: Aircraft): void => {
    origin.unregisterBody(ac.body);
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  };

  // Fresh origin state per scenario, exactly like main.ts startRound
  // (worldGen.resetOrigin() before the player spawns): the previous
  // scenario's shift must NOT leak into the next spawn's frame, or
  // the jet spawns at original-frame coords in a shifted world —
  // i.e. on a mountainside — and dies of 'terrain' on the spot.
  const freshOrigin = (): void => {
    origin.reset();
  };

  const alpha = RUNWAY_TABLE[0]!;
  const deckSpawnPoint = (): { sx: number; sz: number; yawDeg: number } => {
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    return {
      sx: carrier.pos.x - fwd.x * 120,
      sz: carrier.pos.z - fwd.z * 120,
      yawDeg: carrier.yaw * R2D,
    };
  };
  // Distance from the jet to the nearest rotor pod disc centre
  // (world xz) — the "disc" Nicholas collided with on deck spawn.
  const nearestPodDist = (ac: Aircraft): number => {
    let best = Infinity;
    for (const p of carrier.podObjects) {
      const wp = p.getWorldPosition(new THREE.Vector3());
      best = Math.min(best, Math.hypot(ac.pos.x - wp.x, ac.pos.z - wp.z));
    }
    return best;
  };

  // ---- A. Runway spawn: parked 5 s, then takeoff roll 15 s ----
  // (15 s, not 10: the F-18 / F-35A / Su-34 need the full spool-up
  // + acceleration to Vr — they lift at 299/384/384 m in the
  // held-pull probe, but a 10 s window cut them off mid-roll while
  // still alive on the runway. The property is "lifts off without
  // exploding", not "lifts in 10 s".)
  for (const key of KEYS) {
    freshOrigin();
    const ac = makeAircraft(key);
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    let diedAt = -1;
    for (let i = 0; i < 300; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) { diedAt = i; break; }
    }
    const parked = ac.alive;
    const parkDetail = `alive=${ac.alive} reason=${ac.lastCrashReason ?? '-'} y=${ac.pos.y.toFixed(1)} gnd=${ac.grounded} surf=${ac.onSurface} diedAt=${diedAt}`;
    check(`${key} runway spawn: alive + grounded after 5 s parked`, parked && ac.grounded, parkDetail);
    if (parked) {
      ac.thrTarget = 1;
      ac.pitchIn = -1;
      let lifted = false;
      for (let i = 0; i < 900; i++) {
        substep(ac, 1 / 60);
        if (key === 'fa18' && i % 120 === 0) {
          console.log(`  [diag ${key} t=${(i / 60).toFixed(0)}s] x=${ac.pos.x.toFixed(0)} ias=${(ac.ias * 1.944).toFixed(0)} kt thr=${ac.throttle.toFixed(2)} gnd=${ac.grounded} aoa=${ac.aoaDeg.toFixed(1)}`);
        }
        if (!ac.alive) break;
        if (ac.pos.y > heightAt(ac.pos.x, ac.pos.z) + 4 && !ac.grounded) { lifted = true; break; }
      }
      check(
        `${key} runway spawn: takeoff roll survives (no explosion)`,
        ac.alive && lifted,
        `alive=${ac.alive} reason=${ac.lastCrashReason ?? '-'} lifted=${lifted} y=${ac.pos.y.toFixed(1)}`,
      );
    }
    dispose(ac);
  }

  // ---- B. Deck spawn at round start: parked 10 s ----
  for (const key of KEYS) {
    freshOrigin();
    resetCarrier();
    const ac = makeAircraft(key);
    const { sx, sz, yawDeg } = deckSpawnPoint();
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, yawDeg);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    const gap0 = ac.pos.y - carrier.deckTopY;
    let diedAt = -1;
    let minGap = Infinity;
    let maxGap = -Infinity;
    let minPod = Infinity;
    for (let i = 0; i < 600; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) { diedAt = i; break; }
      const gap = ac.pos.y - carrier.deckTopY;
      minGap = Math.min(minGap, gap);
      maxGap = Math.max(maxGap, gap);
      minPod = Math.min(minPod, nearestPodDist(ac));
    }
    check(
      `${key} deck spawn (round start): alive + onDeck + seated 10 s`,
      ac.alive && ac.onDeck && minGap > 1.0 && maxGap < 4.0,
      `alive=${ac.alive} reason=${ac.lastCrashReason ?? '-'} onDeck=${ac.onDeck} gap0=${gap0.toFixed(2)} gap=[${minGap.toFixed(2)},${maxGap.toFixed(2)}] minPodDist=${minPod.toFixed(1)} m diedAt=${diedAt}`,
    );
    dispose(ac);
  }

  // ---- C. Deck RESPAWN mid-loop (carrier turning, 60 s in) ----
  freshOrigin();
  resetCarrier();
  for (let i = 0; i < 3600; i++) carrier.update(1 / 60);
  for (const key of KEYS) {
    const ac = makeAircraft(key);
    const { sx, sz, yawDeg } = deckSpawnPoint();
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, yawDeg);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    let diedAt = -1;
    let minGap = Infinity;
    let maxGap = -Infinity;
    let minPod = Infinity;
    for (let i = 0; i < 600; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) { diedAt = i; break; }
      const gap = ac.pos.y - carrier.deckTopY;
      minGap = Math.min(minGap, gap);
      maxGap = Math.max(maxGap, gap);
      minPod = Math.min(minPod, nearestPodDist(ac));
    }
    check(
      `${key} deck respawn (mid-loop): alive + onDeck + seated 10 s`,
      ac.alive && ac.onDeck && minGap > 1.0 && maxGap < 4.0,
      `alive=${ac.alive} reason=${ac.lastCrashReason ?? '-'} onDeck=${ac.onDeck} gap=[${minGap.toFixed(2)},${maxGap.toFixed(2)}] minPodDist=${minPod.toFixed(1)} m diedAt=${diedAt}`,
    );
    dispose(ac);
  }

  // ---- D. Runway spawn at LOW frame rate (dt = 0.05 s substeps,
  // a slow phone: rawDt 0.25 s -> 5 substeps). The gear skids spawn
  // 0.25 m inside the runway box; at a big dt the depenetration
  // impulse is bigger, and a spurious contact velocity can read as
  // a hard landing.
  for (const key of ['f22', 'f35a'] as const) {
    freshOrigin();
    const ac = makeAircraft(key);
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    let diedAt = -1;
    for (let i = 0; i < 200; i++) {
      substep(ac, 0.05);
      if (!ac.alive) { diedAt = i; break; }
    }
    check(
      `${key} runway spawn at dt=0.05: alive + grounded after 10 s`,
      ac.alive && ac.grounded,
      `alive=${ac.alive} reason=${ac.lastCrashReason ?? '-'} y=${ac.pos.y.toFixed(1)} diedAt=${diedAt}`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL SPAWN CHECKS PASSED' : `\n${failures} SPAWN CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
