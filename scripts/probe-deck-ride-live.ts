// probe-deck-ride-live.ts — pass 6, items 5/9: the deck ride failed
// LIVE (7 attempts, the jet fell off in 11–45 s, brake latched) while
// probe-respawn-carrier passes 5/5. That probe uses WIND=0, fixed
// dt=1/60, and a carrier parked 300 m before a waypoint. The live
// game has wind 3 m/s, variable frame dt (jank), and the carrier on
// its REAL loop from its real start position with terrain-following.
// This probe reproduces those live conditions: wind 3 m/s, the
// live-like variable-dt pattern, the carrier starting at
// CARRIER_START_POS on its real loop, brake latched, 60 s. It checks
// the jet stays onDeck the whole time (no fall-off, no crash) and
// that onDeck stays true (so the score gate — which ticks while the
// jet is NOT onDeck — would not tick while parked on the deck).
//
// Usage: node_modules/.bin/esbuild scripts/probe-deck-ride-live.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-deck-ride-live.mjs
//   && node /tmp/probe-deck-ride-live.mjs

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

/** The live frame-dt pattern (same as probe-live-spawn's `var` mode):
 *  mostly 16–50 ms with occasional 100–250 ms jank frames. */
const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];

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
  // Live default wind: 3 m/s from the west (main.ts SandboxConfig).
  const wind = new THREE.Vector3(3, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
    waypoints: CARRIER_WAYPOINTS,
    startIndex: 0,
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
      // Pass 10: the contact's other side decides isDeck (main.ts
      // pattern). Hardcoding false made a deck spawn on a descending
      // carrier read as a 10 m/s hard landing — the jet inherits the
      // deck's descent rate, so the relative impact is ~0, not 10.
      const other = hit === a ? b : a;
      hit.notifyContact(hit.vel.y, other === 'deck');
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

  const deckSpawn = (ac: Aircraft): void => {
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * 120;
    const sz = carrier.pos.z - fwd.z * 120;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
  };

  // Carrier on its REAL loop from its REAL start position (the live
  // startRound reset), heading east toward waypoint 0. It crosses the
  // basin and turns at the waypoint ~2 min in; 60 s of deck riding
  // covers the straight leg plus the start of the turn.
  carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
  const ac = makeAircraft('f35a');
  deckSpawn(ac);
  ac.brake = true; // the brake button is latched (live: it stays highlighted)
  ac.thrTarget = 0;
  const crashOf = (a: Aircraft): string | null =>
    (a as unknown as { __crash: () => string | null }).__crash();

  let offDeckAt = -1;
  let maxDeckDrift = 0;
  let minOnDeckT = Infinity;
  let scoreTickSeconds = 0;
  let t = 0;
  let pi = 0;
  // 60 s of simulated time at the live variable-dt pattern.
  while (t < 60) {
    const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
    pi++;
    const sub = Math.ceil(rawDt / 0.05);
    const dt = rawDt / sub;
    for (let s = 0; s < sub; s++) substep(ac, dt);
    t += rawDt;
    if (!ac.alive) break;
    // Deck-local position (the spawn is at local z = -120, 8.8 m from
    // the stern edge at local z = -128.8).
    const local = deckProvider.local(ac.pos.x, ac.pos.z);
    if (local) {
      maxDeckDrift = Math.max(maxDeckDrift, Math.abs(local.z + 120));
    }
    if (ac.onDeck) {
      minOnDeckT = Math.min(minOnDeckT, t);
    } else if (offDeckAt < 0) {
      offDeckAt = t;
    }
    // The pass-5 score gate: +1/s only when NOT grounded, NOT onDeck,
    // AGL > 10 m and moving > 20 m/s. While parked on the deck it
    // must not tick — count the seconds where it WOULD tick.
    const agl = ac.pos.y - deckProvider.deckHeightAt(ac.pos.x, ac.pos.z);
    if (!ac.onDeck && !ac.onSurface && agl > 10 && ac.vel.length() > 20) scoreTickSeconds++;
  }
  const localEnd = deckProvider.local(ac.pos.x, ac.pos.z);
  check(
    'deck ride (live conditions): jet stays alive 60 s on the deck',
    ac.alive,
    ac.alive
      ? `alive at t=${t.toFixed(1)}s, local z=${localEnd ? localEnd.z.toFixed(1) : 'off-deck'}`
      : `died at t=${t.toFixed(1)}s reason=${crashOf(ac) ?? '?'}`,
  );
  check(
    'deck ride (live conditions): jet never leaves the deck',
    offDeckAt < 0,
    offDeckAt < 0
      ? `onDeck the whole 60 s (max drift from spawn ${maxDeckDrift.toFixed(1)} m)`
      : `left the deck at t=${offDeckAt.toFixed(1)}s (local z=${localEnd ? localEnd.z.toFixed(1) : 'off-deck'})`,
  );
  check(
    'deck ride (live conditions): score does not tick while parked on the deck',
    scoreTickSeconds === 0,
    `score-tick seconds=${scoreTickSeconds}`,
  );

  console.log(failures === 0 ? '\nALL DECK-RIDE-LIVE CHECKS PASSED' : `\n${failures} DECK-RIDE-LIVE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
