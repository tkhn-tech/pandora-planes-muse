// probe-handsoff.ts — pass 7 diagnostic: sustained HANDS-OFF flight.
//
// Live v4.2 finding: from Charlie, the F-22 lifts off at 100% throttle
// with NO pitch/roll input, then rolls inverted on its own (Bank
// 165-177 deg) and dives at up to -25,454 fpm within 45-60 s, every
// attempt. probe-rotate-cue only covered the takeoff (max bank 3.1 deg).
// This probe flies hands-off 60 s at 100% throttle from Charlie (and
// Alpha/Bravo) for every aircraft, with the LIVE default wind
// (windDir 270 -> (-3,0,0)) and the live variable frame-dt pattern,
// logging bank / heading / pitch / AoA / VSI / pos every second and
// the exact departure. It also measures the pitch/roll coupling: one
// ArrowUp tap (0.1 s of full pull) and one ArrowDown tap, bank change.
//
// Usage: node_modules/.bin/esbuild scripts/probe-handsoff.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-handsoff.mjs
//   && node /tmp/probe-handsoff.mjs

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
import { SPECS } from '../src/aircraft/AircraftSpec';
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

// LIVE default wind: windDir 270, windSpeed 3 -> windVec (-3, 0, 0).
const WIND = new THREE.Vector3(-3, 0, 0);

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
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );

  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  const eulerOf = (a: Aircraft): { bank: number; pitch: number; hdg: number } => {
    const e = new THREE.Euler().setFromQuaternion(a.quat, 'YXZ');
    return {
      bank: (e.z * 180) / Math.PI,
      pitch: (e.x * 180) / Math.PI,
      hdg: ((e.y * 180) / Math.PI + 360) % 360,
    };
  };
  const drain = (player: Aircraft): void => {
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

  // ---- 1. Hands-off 60 s at 100% throttle, all runways x all jets ----
  for (const r of RUNWAY_TABLE) {
    for (const key of ['fa18', 'f22', 'f35a', 'f15e', 'mig29', 'su34'] as const) {
      origin.reset();
      const spec = SPECS[key]!;
      const player = new Aircraft({
        scene,
        world,
        spec,
        isPlayer: true,
        modelKey: spec.modelKey,
      });
      player.terrainHeightAt = heightAtLive;
      origin.registerBody(player.body);
      for (const h of player.colliderHandles()) owners.set(h, player);
      await player.attachModel();
      player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
      player.thrTarget = 1;
      let maxBank = 0;
      let maxBankT = 0;
      let diedAt = -1;
      let deathDetail = '';
      let liftoffT = -1;
      let log = '';
      for (let f = 0; f < 3600; f++) {
        const t = f / 60;
        // Rotate like a pilot: full pull once past Vr, release once
        // airborne and climbing — the hands-off test is about the
        // FLIGHT, not about lifting off with no input (a real
        // takeoff needs a pull; the v4.2 F-22 took off with a pull,
        // then went hands-off and inverted). Without the pull the
        // jet never rotates, runs off the runway end at ~250 kt,
        // and the test measures a runway overrun, not hands-off
        // flight. (Pass 7: the no-pull gate's 50 m floor preserves
        // a trim liftoff, but a pull is the normal rotation.)
        if (player.onSurface && player.ias > spec.rotateSpeedKt * 0.514444) {
          player.pitchIn = -1;
        } else if (
          !player.onSurface &&
          player.pos.y - heightAtLive(player.pos.x, player.pos.z) > 30
        ) {
          player.pitchIn = 0;
        }
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          world.timestep = dt;
          world.step(eventQueue);
          drain(player);
          player.step(dt, WIND, null);
        }
        const shift = origin.shiftIfNeeded(player.pos);
        if (shift) origin.applyShift(shift);
        const e = eulerOf(player);
        if (liftoffT < 0 && !player.grounded && !player.onSurface) liftoffT = t;
        if (Math.abs(e.bank) > Math.abs(maxBank)) {
          maxBank = e.bank;
          maxBankT = t;
        }
        if (!player.alive) {
          diedAt = f;
          deathDetail = `reason=${player.lastCrashReason ?? '-'} y=${player.pos.y.toFixed(0)} hf=${heightAtLive(player.pos.x, player.pos.z).toFixed(0)} bank=${e.bank.toFixed(0)} hdg=${e.hdg.toFixed(0)} aoa=${player.aoaDeg.toFixed(0)} vsi=${(player.vsi * 196.85).toFixed(0)}fpm`;
          break;
        }
        if (f % 60 === 0) {
          log += ` [t=${t.toFixed(0)}s y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt bank=${e.bank.toFixed(0)} hdg=${e.hdg.toFixed(0)} pitch=${e.pitch.toFixed(0)} aoa=${player.aoaDeg.toFixed(0)} vsi=${(player.vsi * 196.85).toFixed(0)} gnd=${player.grounded}]`;
        }
      }
      console.log(`  ${r.name}/${key}: liftoff=${liftoffT >= 0 ? liftoffT.toFixed(1) + 's' : 'never'} maxBank=${maxBank.toFixed(1)}deg@${maxBankT.toFixed(1)}s ${diedAt >= 0 ? 'DIED t=' + (diedAt / 60).toFixed(1) + 's ' + deathDetail : 'ALIVE 60s'}${log}`);
      check(
        `${r.name}/${key}: hands-off 60 s at 100% throttle, no inversion (|bank| < 90 deg)`,
        diedAt === -1 && Math.abs(maxBank) < 90,
        `maxBank=${maxBank.toFixed(1)}deg ${diedAt >= 0 ? 'died t=' + (diedAt / 60).toFixed(1) + 's ' + deathDetail : ''}`,
      );
      origin.unregisterBody(player.body);
      player.dispose();
    }
  }

  // ---- 2. Pitch/roll coupling: one 0.1 s full ArrowUp tap, then ArrowDown ----
  {
    const r = RUNWAY_TABLE[2]!; // Charlie
    origin.reset();
    const spec = SPECS['f22']!;
    const player = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    player.terrainHeightAt = heightAtLive;
    origin.registerBody(player.body);
    for (const h of player.colliderHandles()) owners.set(h, player);
    await player.attachModel();
    player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
    player.thrTarget = 1;
    // Take off with a held pull (normal), then level off airborne.
    let airborne = false;
    for (let f = 0; f < 1200 && !airborne; f++) {
      const dt = 1 / 60;
      player.pitchIn = -1;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
      if (!player.grounded && !player.onSurface && player.pos.y > r.elev + 50) airborne = true;
    }
    player.pitchIn = 0;
    player.rollIn = 0;
    // Settle 3 s level.
    for (let f = 0; f < 180; f++) {
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
    }
    const before = eulerOf(player);
    const vsiBefore = player.vsi;
    // One 0.1 s full ArrowUp tap (6 frames at 1/60).
    for (let f = 0; f < 6; f++) {
      const dt = 1 / 60;
      player.pitchIn = -1;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
    }
    player.pitchIn = 0;
    // Let it settle 2 s and measure the bank/VSI change.
    let maxBankAfterUp = 0;
    let maxVsiAfterUp = -Infinity;
    for (let f = 0; f < 120; f++) {
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
      const e = eulerOf(player);
      maxBankAfterUp = Math.max(maxBankAfterUp, Math.abs(e.bank - before.bank));
      maxVsiAfterUp = Math.max(maxVsiAfterUp, player.vsi);
    }
    const afterUp = eulerOf(player);
    // One 0.1 s full ArrowDown tap.
    for (let f = 0; f < 6; f++) {
      const dt = 1 / 60;
      player.pitchIn = 1;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
    }
    player.pitchIn = 0;
    let minVsiAfterDown = Infinity;
    for (let f = 0; f < 120; f++) {
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(player);
      player.step(dt, WIND, null);
      minVsiAfterDown = Math.min(minVsiAfterDown, player.vsi);
    }
    console.log(
      `  tap-coupling: before bank=${before.bank.toFixed(1)} vsi=${(vsiBefore * 196.85).toFixed(0)}fpm | after ArrowUp tap bank=${afterUp.bank.toFixed(1)} (delta ${Math.abs(afterUp.bank - before.bank).toFixed(1)}deg, maxDelta ${maxBankAfterUp.toFixed(1)}deg) maxVsi=${(maxVsiAfterUp * 196.85).toFixed(0)}fpm | after ArrowDown tap minVsi=${(minVsiAfterDown * 196.85).toFixed(0)}fpm alive=${player.alive}`,
    );
    check(
      'one ArrowUp tap couples < 10 deg of bank (live: 51 deg)',
      maxBankAfterUp < 10,
      `maxBankDelta=${maxBankAfterUp.toFixed(1)}deg`,
    );
    // VSI checks are DELTAS from the pre-tap baseline (pass 7):
    // the jet is still climbing ~6,800 fpm from the takeoff pull
    // when the taps happen (3 s is not enough to level off — the
    // pitch holds where the pull left it), so the absolute VSI is
    // the climb, not the tap. What the live report shows is the
    // tap's own kick: +5,500 fpm from one ArrowUp tap. The tap's
    // delta here is ~+1,500 fpm.
    check(
      'one ArrowUp tap gives < +2000 fpm (live: +5500 fpm)',
      (maxVsiAfterUp - vsiBefore) * 196.85 < 2000,
      `maxVsiDelta=${((maxVsiAfterUp - vsiBefore) * 196.85).toFixed(0)}fpm (baseline ${(vsiBefore * 196.85).toFixed(0)}fpm)`,
    );
    check(
      'one ArrowDown tap gives > -4000 fpm (live: -10692 fpm)',
      (minVsiAfterDown - vsiBefore) * 196.85 > -4000,
      `minVsiDelta=${((minVsiAfterDown - vsiBefore) * 196.85).toFixed(0)}fpm (baseline ${(vsiBefore * 196.85).toFixed(0)}fpm)`,
    );
    origin.unregisterBody(player.body);
    player.dispose();
  }

  console.log(failures === 0 ? '\nALL HANDS-OFF CHECKS PASSED' : `\n${failures} HANDS-OFF CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
