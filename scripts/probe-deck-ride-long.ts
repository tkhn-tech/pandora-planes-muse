// probe-deck-ride-long.ts — pass 10, item 1: Nicholas spawns on the
// helicarrier deck and "after awhile the plane seems to clip through
// it, and it does not properly travel on the surface before taking
// off".
//
// probe-deck-ride-live only covers 60 s on the FIRST leg (flat
// terrain, no climb, no origin shift — it has no FloatingOrigin at
// all). The live game is different after ~60 s: the carrier turns at
// its first waypoint (~2 min in), crosses higher terrain, and the
// floating origin shifts every ~50 s of carrier travel (40 m/s /
// 2000 m threshold). This probe reproduces the FULL live frame loop
// (frame-start shift, carrier update, world.step, drain, aircraft
// step, live variable-dt pattern) for 180 s parked on the deck with
// the brake latched, then a full-throttle takeoff roll along the
// deck. It samples the jet's height above the deck every second and
// logs the exact conditions (shift? turn? climb?) whenever the jet
// clips below the deck surface or leaves the deck.
//
// Usage: node_modules/.bin/esbuild scripts/probe-deck-ride-long.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-deck-ride-long.mjs
//   && node /tmp/probe-deck-ride-long.mjs

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
import { FloatingOrigin } from '../src/world/FloatingOrigin';
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
const GEAR_H = 2.2;
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
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  // Scene coords + cumulativeShift = original-frame coords (the
  // heightfield query the live game uses).
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: heightAtLive,
    waypoints: CARRIER_WAYPOINTS,
    startIndex: 0,
  });
  await carrier.build();
  carrier.registerWithFloatingOrigin(origin);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');
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
    ac.terrainHeightAt = heightAtLive;
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

  const deckSpawn = (ac: Aircraft): void => {
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, carrier.deckTopY + GEAR_H, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
  };

  for (const key of ['f35a', 'fa18'] as const) {
    // Fresh origin + carrier per jet (a shift is a one-way trip; the
    // second jet must start from the same state as the first).
    origin.reset();
    carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
    const ac = makeAircraft(key);
    await ac.attachModel();
    origin.registerBody(ac.body);
    deckSpawn(ac);
    ac.brake = true;
    ac.thrTarget = 0;

    const crashOf = (): string | null =>
      (ac as unknown as { __crash: () => string | null }).__crash();
    let t = 0;
    let pi = 0;
    let minGap = Infinity;
    let maxGap = -Infinity;
    let minGapAt = -1;
    let clipAt = -1;
    let offDeckAt = -1;
    let offDeckLocal: { x: number; z: number } | null = null;
    let shifts = 0;
    let lastShiftMag = 0;
    const shiftTimes: number[] = [];
    let maxDeckBodyVel = 0;
    let maxDeckBodyVelAt = -1;
    let lastCarrierYaw = carrier.yaw;
    let maxYawRate = 0;
    let minCarrierClearance = Infinity;
    const trace: string[] = [];
    // Detailed per-frame log around the clip window (t 80–115 s):
    // gap, jet vertical velocity, carrier vertical velocity, contact
    // flags, and the deck body's derived linvel (the shift-teleport
    // hypothesis: a shift should show up as shift/dt here).
    const detail: string[] = [];

    // Phase 1: 180 s parked on the deck, brake latched, live dt.
    while (t < 180) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.max(1, Math.ceil(rawDt / 0.05));
      const dt = rawDt / sub;
      const sh = origin.shiftIfNeeded(ac.pos);
      if (sh) {
        origin.applyShift(sh);
        shifts++;
        lastShiftMag = Math.hypot(sh.x, sh.z);
        shiftTimes.push(t);
      }
      let shiftDbVel = -1;
      for (let s = 0; s < sub; s++) {
        carrier.update(dt);
        world.timestep = dt;
        world.step(eventQueue);
        drain(ac);
        ac.step(dt, WIND, null);
        // The kinematic deck body's derived velocity right after the
        // step: a shift teleport shows up here as shift/dt.
        const db = carrier.deckBodyRef;
        if (db) {
          const lv = db.linvel();
          const m = Math.hypot(lv.x, lv.y, lv.z);
          if (sh && shiftDbVel < 0) shiftDbVel = m;
          if (m > maxDeckBodyVel) {
            maxDeckBodyVel = m;
            maxDeckBodyVelAt = t;
          }
        }
      }
      t += rawDt;
      if (!ac.alive) break;
      maxYawRate = Math.max(maxYawRate, Math.abs(carrier.yawRate));
      lastCarrierYaw = carrier.yaw;
      const deckTop = carrier.deckTopY;
      const gap = ac.pos.y - (deckTop + GEAR_H);
      const terr = heightAtLive(ac.pos.x, ac.pos.z);
      minCarrierClearance = Math.min(minCarrierClearance, carrier.pos.y - terr);
      if (gap < minGap) {
        minGap = gap;
        minGapAt = t;
      }
      if (gap > maxGap) maxGap = gap;
      if (gap < -0.25 && clipAt < 0) clipAt = t;
      if (t >= 80 && t <= 115 && (Math.abs(gap) > 0.05 || sh || carrier.vel.y !== 0)) {
        detail.push(
          `t=${t.toFixed(2)} gap=${gap.toFixed(2)} velY=${ac.vel.y.toFixed(2)} ` +
            `cVy=${carrier.vel.y.toFixed(1)} onS=${ac.onSurface ? 1 : 0} onD=${ac.onDeck ? 1 : 0} ` +
            `${sh ? `SHIFT ${lastShiftMag.toFixed(0)}m dbVel=${shiftDbVel.toFixed(0)}` : ''}`,
        );
      }
      const local = deckProvider.local(ac.pos.x, ac.pos.z);
      if (!ac.onDeck && offDeckAt < 0) {
        offDeckAt = t;
        offDeckLocal = local;
      }
      if (Math.abs(t - Math.round(t)) < 0.06) {
        trace.push(
          `t=${t.toFixed(0)} gap=${gap.toFixed(2)} onD=${ac.onDeck ? 1 : 0} ` +
            `cY=${carrier.pos.y.toFixed(0)} cVy=${carrier.vel.y.toFixed(1)} ` +
            `yawR=${carrier.yawRate.toFixed(3)} terr=${terr.toFixed(0)} ` +
            `loc=${local ? `${local.x.toFixed(0)},${local.z.toFixed(0)}` : 'off'} ` +
            `shift=${origin.cumulativeShift.x.toFixed(0)},${origin.cumulativeShift.z.toFixed(0)}`,
        );
      }
    }
    console.log(`--- ${key} phase 1 (180 s parked, brake latched) ---`);
    console.log(trace.filter((_l, i) => i % 10 === 0).join('\n'));
    console.log(
      `shifts=${shifts} at t=[${shiftTimes.map((s) => s.toFixed(1)).join(',')}] ` +
        `lastShiftMag=${lastShiftMag.toFixed(0)}m ` +
        `maxDeckBodyVel=${maxDeckBodyVel.toFixed(0)}m/s@${maxDeckBodyVelAt.toFixed(1)}s ` +
        `maxYawRate=${maxYawRate.toFixed(3)} minCarrierClearance=${minCarrierClearance.toFixed(0)}m`,
    );
    console.log(`--- ${key} detail t=80-115 (|gap|>0.05 or shift or climbing) ---`);
    console.log(detail.slice(0, 120).join('\n'));
    check(
      `${key}: alive after 180 s parked on the deck`,
      ac.alive,
      ac.alive ? `t=${t.toFixed(1)}s` : `died at t=${t.toFixed(1)}s reason=${crashOf() ?? '?'}`,
    );
    check(
      `${key}: never clips below the deck surface (gap >= -0.25 m)`,
      clipAt < 0,
      clipAt < 0
        ? `min gap ${minGap.toFixed(2)} m at t=${minGapAt.toFixed(1)}s, max gap ${maxGap.toFixed(2)} m`
        : `CLIPPED at t=${clipAt.toFixed(1)}s (min gap ${minGap.toFixed(2)} m at t=${minGapAt.toFixed(1)}s)`,
    );
    check(
      `${key}: stays onDeck the whole 180 s`,
      offDeckAt < 0,
      offDeckAt < 0
        ? `onDeck throughout`
        : `left the deck at t=${offDeckAt.toFixed(1)}s local=${offDeckLocal ? `${offDeckLocal.x.toFixed(0)},${offDeckLocal.z.toFixed(0)}` : 'off-deck'}`,
    );

    // Phase 2: takeoff roll along the deck (same round, same carrier).
    if (ac.alive && ac.onDeck) {
      ac.brake = false;
      ac.thrTarget = 1;
      let rollT = 0;
      let rollMinGap = Infinity;
      let rollClipAt = -1;
      let liftedAt = -1;
      let liftSpeed = 0;
      let maxRollGap = -Infinity;
      while (rollT < 30 && ac.alive) {
        const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
        pi++;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        const sh = origin.shiftIfNeeded(ac.pos);
        if (sh) origin.applyShift(sh);
        for (let s = 0; s < sub; s++) {
          carrier.update(dt);
          world.timestep = dt;
          world.step(eventQueue);
          drain(ac);
          ac.step(dt, WIND, null);
        }
        rollT += rawDt;
        if (ac.rotateCue) ac.pitchIn = -1;
        const gap = ac.pos.y - (carrier.deckTopY + GEAR_H);
        if (ac.onSurface) {
          rollMinGap = Math.min(rollMinGap, gap);
          maxRollGap = Math.max(maxRollGap, gap);
          if (gap < -0.25 && rollClipAt < 0) rollClipAt = rollT;
        } else if (liftedAt < 0 && ac.pos.y > carrier.deckTopY + GEAR_H + 5) {
          liftedAt = rollT;
          liftSpeed = ac.ias * 1.94384;
        }
        if (liftedAt >= 0) break;
      }
      console.log(
        `phase 2 roll: minGap=${rollMinGap.toFixed(2)} maxGap=${maxRollGap.toFixed(2)} ` +
          `liftedAt=${liftedAt >= 0 ? liftedAt.toFixed(1) + 's' : 'never'} ` +
          `liftSpeed=${liftSpeed.toFixed(0)}kt alive=${ac.alive}`,
      );
      check(
        `${key}: takeoff roll stays on the deck surface (no clip)`,
        rollClipAt < 0,
        rollClipAt < 0
          ? `roll gap ${rollMinGap.toFixed(2)}..${maxRollGap.toFixed(2)} m`
          : `CLIPPED during the roll at t=${rollClipAt.toFixed(1)}s (min gap ${rollMinGap.toFixed(2)} m)`,
      );
      check(
        `${key}: lifts off from the deck`,
        liftedAt >= 0 && ac.alive,
        liftedAt >= 0
          ? `airborne at ${liftedAt.toFixed(1)}s at ${liftSpeed.toFixed(0)} kt`
          : `never lifted off in 30 s (alive=${ac.alive}, onSurface=${ac.onSurface})`,
      );
    }

    origin.unregisterBody(ac.body);
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
    void lastCarrierYaw;
  }

  console.log(failures === 0 ? '\nALL DECK-RIDE-LONG CHECKS PASSED' : `\n${failures} DECK-RIDE-LONG CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
