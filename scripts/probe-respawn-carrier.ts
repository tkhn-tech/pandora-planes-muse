// probe-respawn-carrier.ts — Issue 3: respawn-immediate-explosion when
// the spawn-at-helicarrier option is active.
//
// Root cause (found by reading the code, confirmed here): the deck
// velocity the controller transfers to a jet on the deck was the
// carrier's LINEAR velocity only (DeckProvider.velocity()). A jet
// spawns 120 m aft of the deck centre; when the carrier yaws at up to
// 0.12 rad/s the deck point under the jet moves sideways at up to
// 0.12 * 120 = 14.4 m/s relative to the linear velocity, and Rapier
// friction is 0 on all colliders, so nothing holds the jet laterally
// — it drifts off the deck, deckHeightAt returns -Infinity, the jet
// is airborne at ~345 m and falls, and the crash (explosion) follows.
// The live v3 test saw exactly this: the F-35A fell off the deck
// shortly after spawning and crashed. The fix is DeckProvider.
// velocityAt(x, z) — linear vel + yawRate x r — used by the
// controller's ground block and by the deck spawn / respawn.
//
// Checks:
//   1. Deck ride through a waypoint turn: the carrier turns ~120 deg
//      at waypoint 0; the jet rides the deck for 30 s with no input,
//      stays alive and onDeck, and its deck-point velocity tracks the
//      point velocity (not the linear velocity) — the rotation term
//      reaches > 5 m/s during the turn, proving the turn was exercised.
//   2. Death + respawn on the deck mid-turn: the jet is killed, then
//      respawned on the deck at the carrier's CURRENT position/yaw
//      with the point velocity (the issue-4 picker respawn path), and
//      rides the deck for 30 s — alive, onDeck, no crash, no
//      respawn-explosion.
//
// Usage: node_modules/.bin/esbuild scripts/probe-respawn-carrier.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-respawn-carrier.mjs
//   && node /tmp/probe-respawn-carrier.mjs

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
import {
  HelicarrierController,
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
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
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
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
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({
          x: 0,
          y: Math.sin(hdg / 2),
          z: 0,
          w: Math.cos(hdg / 2),
        })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  // SHIELD helicarrier (real GLB), as in main.ts.
  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
  });
  await carrier.build();
  const dc = carrier.deckColliderRef;
  if (dc) owners.set(dc.handle, 'deck');
  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.alive;
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
    yawRateAt: () => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };

  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    let crashReason: string | null = null;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
      events: {
        onCrash: (_a, reason) => {
          crashReason = reason;
        },
      },
    });
    (ac as unknown as { __crash: () => string | null }).__crash = () => crashReason;
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
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
      hit.notifyContact(hit.vel.y, false);
    });
  };

  /** One main.ts-style substep (carrier first, then physics, then the jet). */
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };

  /** Deck spawn exactly as main.ts startRound does it (issue-3 fix:
   *  the point velocity at the spawn point, not the linear cruise). */
  const deckSpawn = (ac: Aircraft): void => {
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * 120;
    const sz = carrier.pos.z - fwd.z * 120;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
  };

  // ---- 1. Deck ride through a waypoint turn ----
  // Park the carrier 300 m before waypoint 0, heading east toward it:
  // it reaches the waypoint in ~7.5 s and turns ~120 deg toward
  // waypoint 1 at up to 0.12 rad/s. The jet rides the deck for 30 s.
  {
    carrier.reset(
      new THREE.Vector3(
        CARRIER_WAYPOINTS[0]!.x - 300,
        CARRIER_START_POS.y,
        CARRIER_WAYPOINTS[0]!.z,
      ),
      Math.PI / 2,
    );
    const ac = makeAircraft('f35a');
    deckSpawn(ac);
    // Stage-by-stage velocity trace for the first 3 substeps: pin down
    // where the spawn velocity (40 m/s) is lost.
    for (let s = 0; s < 3; s++) {
      const b0 = ac.body.linvel();
      carrier.update(1 / 60);
      const b1 = ac.body.linvel();
      world.timestep = 1 / 60;
      world.step(eventQueue);
      const b2 = ac.body.linvel();
      drain(ac);
      ac.step(1 / 60, wind, null);
      const b3 = ac.body.linvel();
      console.log(
        `  stage s=${s}: spawn/body=${b0.x.toFixed(1)},${b0.z.toFixed(1)} afterCarrier=${b1.x.toFixed(1)},${b1.z.toFixed(1)} afterWorld=${b2.x.toFixed(1)},${b2.z.toFixed(1)} afterStep=${b3.x.toFixed(1)},${b3.z.toFixed(1)} ac.vel=(${ac.vel.x.toFixed(1)},${ac.vel.z.toFixed(1)}) onDeck=${ac.onDeck}`,
      );
    }
    let maxRotTerm = 0;
    let maxYawRate = 0;
    let minDeckH = Infinity;
    let crashReason: string | null = null;
    const crashOf = (a: Aircraft): string | null =>
      (a as unknown as { __crash: () => string | null }).__crash();
    let offDeckAt = -1;
    for (let i = 0; i < 30 * 60; i++) {
      substep(ac, 1 / 60);
      const pv = deckProvider.velocityAt(ac.pos.x, ac.pos.z);
      const lv = deckProvider.velocity();
      if (ac.onDeck) {
        maxRotTerm = Math.max(maxRotTerm, Math.hypot(pv.x - lv.x, pv.z - lv.z));
      }
      maxYawRate = Math.max(maxYawRate, Math.abs(carrier.yawRate));
      const dh = carrier.deckHeightAt(ac.pos.x, ac.pos.z);
      if (dh > -Infinity) minDeckH = Math.min(minDeckH, dh);
      if (i % 60 === 0) {
        const local = carrier.localPosition(ac.pos);
        console.log(
          `  t=${(i / 60).toFixed(0)}s onDeck=${ac.onDeck} onSurf=${ac.onSurface} gnd=${ac.grounded} ` +
            `local=${local ? `(${local.x.toFixed(1)},${local.z.toFixed(1)})` : 'off-deck'} ` +
            `y=${ac.pos.y.toFixed(2)} deckTop=${carrier.deckTopY.toFixed(2)} ` +
            `vel=(${ac.vel.x.toFixed(1)},${ac.vel.z.toFixed(1)}) yawRate=${carrier.yawRate.toFixed(3)} carrierYaw=${(carrier.yaw * 180 / Math.PI).toFixed(0)}deg`,
        );
      }
      if (!ac.onDeck && offDeckAt < 0) {
        offDeckAt = i / 60;
        const local = carrier.localPosition(ac.pos);
        console.log(
          `  OFF-DECK at t=${offDeckAt.toFixed(2)}s local=${local ? `(${local.x.toFixed(1)},${local.z.toFixed(1)})` : 'off-deck'} ` +
            `y=${ac.pos.y.toFixed(2)} deckTop=${carrier.deckTopY.toFixed(2)} vel=(${ac.vel.x.toFixed(1)},${ac.vel.z.toFixed(1)}) yawRate=${carrier.yawRate.toFixed(3)}`,
        );
      }
      if (!ac.alive) {
        crashReason = crashOf(ac);
        break;
      }
    }
    const local = carrier.localPosition(ac.pos);
    check(
      'deck ride through a waypoint turn: alive + onDeck after 30 s',
      ac.alive && ac.onDeck,
      `alive=${ac.alive} onDeck=${ac.onDeck} crash=${crashReason ?? 'none'} local=${local ? `(${local.x.toFixed(1)},${local.z.toFixed(1)})` : 'off-deck'} y=${ac.pos.y.toFixed(1)} (deck ${carrier.deckTopY.toFixed(1)})`,
    );
    check(
      'the turn actually exercised the rotation term',
      maxYawRate > 0.1 && maxRotTerm > 5,
      `max |yawRate|=${maxYawRate.toFixed(3)} rad/s, max |pointVel - linearVel|=${maxRotTerm.toFixed(1)} m/s (up to 14.4 m/s at 120 m aft)`,
    );
    ac.dispose();
  }

  // ---- 2. Death + respawn on the deck mid-turn ----
  // Kill the jet, then respawn it on the deck at the carrier's CURRENT
  // position/yaw with the point velocity (the issue-4 picker respawn
  // path), while the carrier is turning at its max yaw rate. It must
  // ride the deck for 30 s — alive, onDeck, no crash, no explosion.
  {
    // Drive the carrier into its turn first: step it alone until the
    // yaw rate peaks (the waypoint-0 turn).
    carrier.reset(
      new THREE.Vector3(
        CARRIER_WAYPOINTS[0]!.x - 300,
        CARRIER_START_POS.y,
        CARRIER_WAYPOINTS[0]!.z,
      ),
      Math.PI / 2,
    );
    let turned = false;
    for (let i = 0; i < 60 * 30 && !turned; i++) {
      carrier.update(1 / 60);
      if (Math.abs(carrier.yawRate) > 0.11) turned = true;
    }
    const ac = makeAircraft('fa18');
    // Kill it first (the death that precedes the respawn).
    ac.crash('test kill');
    const respawnRotTerm = (() => {
      const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
      const sx = carrier.pos.x - fwd.x * 120;
      const sz = carrier.pos.z - fwd.z * 120;
      const pv = deckProvider.velocityAt(sx, sz);
      const lv = deckProvider.velocity();
      return Math.hypot(pv.x - lv.x, pv.z - lv.z);
    })();
    // Respawn on the deck (issue-4 path): resetAt + point velocity.
    deckSpawn(ac);
    let crashReason: string | null = null;
    const crashOf = (a: Aircraft): string | null =>
      (a as unknown as { __crash: () => string | null }).__crash();
    for (let i = 0; i < 30 * 60; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) {
        crashReason = crashOf(ac);
        break;
      }
    }
    const local = carrier.localPosition(ac.pos);
    check(
      'respawn on the deck mid-turn: alive + onDeck after 30 s (no respawn explosion)',
      ac.alive && ac.onDeck,
      `alive=${ac.alive} onDeck=${ac.onDeck} crash=${crashReason ?? 'none'} local=${local ? `(${local.x.toFixed(1)},${local.z.toFixed(1)})` : 'off-deck'} respawn rotation term=${respawnRotTerm.toFixed(1)} m/s yawRate=${carrier.yawRate.toFixed(3)} rad/s`,
    );
    check(
      'respawn happened while the carrier was turning',
      turned && respawnRotTerm > 5,
      `turned=${turned} rotation term at respawn=${respawnRotTerm.toFixed(1)} m/s`,
    );
    ac.dispose();
  }

  // ---- 3. Deck hold with a velocity-transfer residual (pass 5,
  // item 5) ----
  // A parked jet on the deck through a waypoint turn: the velocity
  // transfer leaves a ~0.5-1.5 m/s deck-relative residual, which used
  // to exceed the old deckHold gate (sp < 1) and disengage the hold
  // — the weak ground friction (0.3) could not hold it against the
  // deck's motion, so it walked off the stern in 12-35 s (v4
  // play-test). The gate is now sp < 3 (the throttle gate already
  // distinguishes a parked jet from a taxiing one). Give the jet a
  // 2 m/s sideways deck-relative drift at spawn and require it to
  // stay on the deck for 30 s through the turn.
  {
    carrier.reset(
      new THREE.Vector3(
        CARRIER_WAYPOINTS[0]!.x - 300,
        CARRIER_START_POS.y,
        CARRIER_WAYPOINTS[0]!.z,
      ),
      Math.PI / 2,
    );
    const ac = makeAircraft('fa18');
    deckSpawn(ac);
    // 2 m/s sideways (to the carrier's starboard) on top of the deck
    // point velocity — the transfer residual.
    const right = new THREE.Vector3(Math.cos(carrier.yaw), 0, -Math.sin(carrier.yaw));
    const dv = ac.vel.clone().addScaledVector(right, 2);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    const startLocal = carrier.localPosition(ac.pos);
    let crashReason: string | null = null;
    const crashOf = (a: Aircraft): string | null =>
      (a as unknown as { __crash: () => string | null }).__crash();
    for (let i = 0; i < 30 * 60; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) {
        crashReason = crashOf(ac);
        break;
      }
    }
    const endLocal = carrier.localPosition(ac.pos);
    const drift = startLocal && endLocal
      ? Math.hypot(endLocal.x - startLocal.x, endLocal.z - startLocal.z)
      : Infinity;
    check(
      'deck hold: parked jet with a 2 m/s transfer residual stays on the deck for 30 s through a turn',
      ac.alive && ac.onDeck && drift < 5,
      `alive=${ac.alive} onDeck=${ac.onDeck} crash=${crashReason ?? 'none'} drift=${drift.toFixed(1)} m (limit 5 m)`,
    );
    ac.dispose();
  }

  console.log(failures === 0 ? '\nALL RESPAWN-CARRIER CHECKS PASSED' : `\n${failures} RESPAWN-CARRIER CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
