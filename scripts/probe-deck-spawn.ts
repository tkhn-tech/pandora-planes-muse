// probe-deck-spawn.ts — pass 9, item 4: the deck spawn put the jet on
// the grass UNDER the carrier (live trace: y=384.1 on the terrain, 25 m
// below the carrier at y=409, sliding at 78 kt for 90+ s with the brake
// latched, never on the deck). Two contributors: the spawn was only
// 8.8 m from the stern edge (120 m aft; the aft deck rect reaches
// local z = -128.8), so the jet could fall off the stern, and the
// double-registered origin shift (probe-origin-double-register) made
// the ground/deck detection read the wrong terrain, so a jet on the
// ground read as airborne and ground friction never applied — hence
// the 78 kt slide at the inherited deck velocity. This probe uses the
// fixed 85 m aft spawn (16 m inside the stern) on the carrier's real
// loop with live wind + variable dt, and checks the jet spawns ON the
// deck (deckHeightAt = deck top, onDeck, not on the terrain), stays on
// the deck 30 s with the brake latched at 0% throttle, and takes off
// from the deck at 100% throttle. It also checks the slide directly:
// a jet placed on the terrain at 40 m/s with the brake latched and 0%
// throttle must decelerate to a stop, not slide at 78 kt.
//
// Usage: node_modules/.bin/esbuild scripts/probe-deck-spawn.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-deck-spawn.mjs
//   && node /tmp/probe-deck-spawn.mjs

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

const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
const WIND = new THREE.Vector3(3, 0, 0);
// Fixed deck-spawn offset (main.ts DECK_SPAWN_AFT_M).
const DECK_SPAWN_AFT_M = 85;

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
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
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
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, WIND, null);
  };
  const stepFor = (ac: Aircraft, seconds: number): void => {
    let t = 0;
    let pi = 0;
    while (t < seconds) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
    }
  };
  // The respawnPlayer/startRound deck branch with the fixed offset.
  const deckSpawn = (ac: Aircraft): void => {
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
  };

  carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
  for (const key of ['fa18', 'f22', 'f35a', 'f15e'] as const) {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x, z) => heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    deckSpawn(ac);
    ac.brake = true;
    ac.thrTarget = 0;
    // At spawn: on the deck (not the terrain below it).
    const local0 = deckProvider.local(ac.pos.x, ac.pos.z);
    const deckY0 = carrier.deckHeightAt(ac.pos.x, ac.pos.z);
    const groundY0 = heightfieldHeightAt(heights, res, ac.pos.x, ac.pos.z);
    check(
      `${key}: deck spawn lands ON the deck (85 m aft)`,
      local0 !== null &&
        deckY0 > -1e8 &&
        Math.abs(ac.pos.y - (deckY0 + 2.2)) < 1.0 &&
        ac.pos.y - groundY0 > 100,
      `local z=${local0 ? local0.z.toFixed(1) : 'off-deck'} deckY=${deckY0.toFixed(1)} y=${ac.pos.y.toFixed(1)} ground=${groundY0.toFixed(1)}`,
    );
    // 30 s parked on the deck, brake latched, 0% throttle: stays on
    // the deck (no fall to the terrain, no slide off the stern).
    stepFor(ac, 30);
    const local30 = deckProvider.local(ac.pos.x, ac.pos.z);
    check(
      `${key}: stays on the deck 30 s parked (brake latched, 0% throttle)`,
      ac.alive && ac.onDeck && local30 !== null,
      `alive=${ac.alive} onDeck=${ac.onDeck} local z=${local30 ? local30.z.toFixed(1) : 'off-deck'} y=${ac.pos.y.toFixed(1)} deckY=${carrier.deckHeightAt(ac.pos.x, ac.pos.z).toFixed(1)}`,
    );
    // Take off from the deck at 100% throttle (brake off): lifts off
    // and climbs. Pull at Vr while on the deck, then hold a gentle
    // climb (pitchIn=-0.3) until 15 m above the deck — releasing the
    // pull the instant the jet leaves the surface makes the F-18 sink
    // ~60 m before it recovers (diagnostic: it lifts off at ias~98
    // but the wing at 0 deg pitch does not hold it until the speed
    // builds).
    ac.brake = false;
    ac.thrTarget = 1;
    const startY = ac.pos.y;
    let tOff = -1;
    let t = 0;
    let pi = 0;
    while (t < 20 && tOff < 0) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
      if (ac.onSurface && ac.ias > 55) ac.pitchIn = -1;
      else if (!ac.onSurface && ac.pos.y < startY + 15) ac.pitchIn = -0.3;
      else ac.pitchIn = 0;
      if (!ac.onSurface && ac.pos.y > startY + 10) tOff = t;
      if (!ac.alive) break;
    }
    check(
      `${key}: takes off from the deck at 100% throttle`,
      ac.alive && tOff >= 0,
      `tOff=${tOff < 0 ? 'never' : tOff.toFixed(1) + 's'} alive=${ac.alive}`,
    );
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  }

  // The slide, directly: a jet on the flat Alpha runway at 40 m/s
  // (the deck velocity it inherits) with the brake latched and 0%
  // throttle must decelerate to a stop — not slide at 78 kt for 90 s.
  // (The live slide read as airborne because the double-shifted height
  // query put the floor a full shift away, so onSurface stayed false
  // and ground friction never ran. The runway is used, not rough
  // terrain, so the jet stays in contact and the brake can act.)
  {
    const spec = SPECS['fa18']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x, z) => heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    const r = RUNWAY_TABLE[0]!;
    ac.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
    ac.body.setLinvel({ x: 40, y: 0, z: 0 }, true);
    ac.vel.set(40, 0, 0);
    ac.brake = true;
    ac.thrTarget = 0;
    stepFor(ac, 15);
    const speed = Math.hypot(ac.vel.x, ac.vel.z);
    check(
      'terrain slide: jet at 40 m/s on a runway, brake latched, 0% throttle stops (no 78 kt slide)',
      speed < 5,
      `speed after 15 s=${speed.toFixed(1)} m/s (${(speed * 1.944).toFixed(0)} kt) onSurf=${ac.onSurface} y=${ac.pos.y.toFixed(1)} runway elev=${r.elev}`,
    );
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  }

  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
