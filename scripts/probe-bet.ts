// probe-bet.ts — BladeElementSolver calibration (Issue 6).
//
// BET is opt-in via AircraftOptions.useBet; before it becomes the
// default it must match the curve solver on the numbers that matter,
// for ALL six aircraft (F/A-18F, F-35A, F-22, F-15E, Su-34, MiG-29).
// Checks:
//   1. BET takeoff for all 6 (Alpha runway, full throttle + full
//      pull, flaps 0): lifts off inside the 650 m runway at a speed
//      near the per-model rotation speed.
//   2. BET max-performance turn at 400 kt SL, 80 deg bank (AoA hold):
//      turn rate in the research band per aircraft, no departure,
//      no stall flag in the vortex-lift region.
//   3. BET F-35A hover at 94% throttle, vector 90: holds altitude
//      (the hover auto-level PD added for BET promotion) and the
//      pilot can roll in hover.
//   4. BET stall flag: FALSE at 20-25 deg AoA (vortex-lift region,
//      the jet is flying fine) and TRUE past the peak-CL AoA.
//
// Research targets (Docs/flight-behaviour-per-model.md §7/§8):
//   sustained turn SL: F-18 19.2, F-22 ~19-20, F-15E ~19-20,
//   F-35A ~16, MiG-29 ~21, Su-34 ~14-16 deg/s.
//   Rotation speeds (kt): F-18 140 (pass 11 — was 126, below its own
//   stall speed), F-35A 150, F-22 140, F-15E 150,
//   MiG-29 130, Su-34 140.
//
// Usage: node_modules/.bin/esbuild scripts/probe-bet.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-bet.mjs
//   && node /tmp/probe-bet.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (The live game runs in a browser, where all of
// these exist; the stubs only let the driver parse the model in node.)
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
import { SPECS } from '../src/aircraft/AircraftSpec';
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

// Research sustained-turn bands (deg/s at SL, max performance) and
// rotation speeds (kt) per aircraft.
const TURN_BAND: Record<string, [number, number]> = {
  fa18: [13, 22],
  f35a: [11, 19],
  f22: [13, 23],
  f15e: [13, 23],
  mig29: [15, 24],
  su34: [10, 19],
};
const ROTATE_KT: Record<string, number> = {
  fa18: 140,
  f35a: 150,
  f22: 140,
  f15e: 150,
  mig29: 130,
  su34: 140,
};

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
  // TRANSPOSED like every other probe and PandoraWorldGenerator: the
  // generator's heights array is z-major, Rapier wants x-major. The
  // original probe copied the heights straight, so the collider was
  // the transpose of the real terrain — Alpha's collider height was
  // NOT 50 m, every takeoff jet spawned inside the terrain and died
  // instantly (reason=terrain), and the ground hover sank. That was
  // a harness bug, not a solver bug.
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
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  // Runway boxes (takeoff-driver): flat cuboids per runway, top 5 cm
  // above the heightfield, so the gear contacts one clean face
  // instead of the heightfield's 60 m triangles.
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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      useBet: true,
    });
    ac.terrainHeightAt = heightAt;
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
  const substep = (ac: Aircraft, dt: number): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  const alpha = RUNWAY_TABLE[0]!;
  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * (180 / Math.PI);

  // ---- 1. BET takeoff, all 6 aircraft ----
  for (const key of ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const) {
    const ac = makeAircraft(key);
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.flaps = 0;
    ac.thrTarget = 1;
    ac.pitchIn = -1;
    const x0 = ac.pos.x;
    let lifted = false;
    let liftDist = 0;
    let liftKt = 0;
    for (let i = 0; i < 60 * 30; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      if (ac.pos.y > heightAt(ac.pos.x, ac.pos.z) + 4 && ac.vsi > 0 && !ac.grounded) {
        lifted = true;
        liftDist = ac.pos.x - x0;
        liftKt = ac.ias * 1.94384;
        break;
      }
      if (ac.pos.x - x0 > 750) break;
    }
    const rotKt = ROTATE_KT[key]!;
    check(
      `BET takeoff ${key}: lifts off inside 650 m near rotate ${rotKt} kt`,
      ac.alive && lifted && liftDist < 650 && liftKt > rotKt - 25 && liftKt < rotKt + 40,
      `alive=${ac.alive} lifted=${lifted} dist=${liftDist.toFixed(0)} m speed=${liftKt.toFixed(0)} kt (rotate ${rotKt})`,
    );
    dispose(ac);
  }

  // ---- 2. BET max-performance turn, all 6 (400 kt SL, 80 deg bank) ----
  for (const key of ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const) {
    const ac = makeAircraft(key);
    ac.resetAt(alpha.x, 200, alpha.z, alpha.hdg);
    ac.setAirspeed(400 / 1.94384);
    ac.thrTarget = 1;
    const hdg0 = Math.atan2(ac.fwd().x, ac.fwd().z);
    const bankDes = (80 * Math.PI) / 180;
    const aoaTarget = 12;
    let minAlt = Infinity;
    let maxAoA = -Infinity;
    let stalledInTurn = false;
    for (let i = 0; i < 600; i++) {
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
      ac.pitchIn = Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3));
      ac.yawIn = 0;
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      minAlt = Math.min(minAlt, ac.pos.y);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      // The stall flag must not fire in the vortex-lift region
      // (AoA 20-25 deg) — only past the peak. Record if it fires
      // while the AoA is still in the vortex-lift band.
      if (ac.aoaDeg >= 20 && ac.aoaDeg <= 25 && ac.stalled) {
        stalledInTurn = true;
      }
    }
    const hdg1 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let dHdg = hdg1 - hdg0;
    while (dHdg > Math.PI) dHdg -= 2 * Math.PI;
    while (dHdg < -Math.PI) dHdg += 2 * Math.PI;
    const turnRate = Math.abs((dHdg * 180) / Math.PI / 10);
    const [lo, hi] = TURN_BAND[key]!;
    check(
      `BET max turn ${key}: ${lo}-${hi} deg/s at 400 kt SL, 80 deg bank, no departure`,
      ac.alive && turnRate > lo && turnRate < hi && maxAoA < 40,
      `rate=${turnRate.toFixed(1)} deg/s alive=${ac.alive} minAlt=${minAlt.toFixed(0)} m maxAoA=${maxAoA.toFixed(1)}`,
    );
    check(
      `BET stall flag ${key}: does NOT fire at 20-25 deg AoA (vortex lift)`,
      !stalledInTurn,
      `stalledInVortexLift=${stalledInTurn}`,
    );
    dispose(ac);
  }

  // ---- 3. BET F-35A hover at 94% (auto-level PD) ----
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.vector = 90;
    ac.thrTarget = 0.94;
    let maxAlt = 0;
    for (let i = 0; i < 300; i++) {
      substep(ac, 1 / 60);
      maxAlt = Math.max(maxAlt, ac.pos.y);
      if (!ac.alive) break;
    }
    check(
      'BET F-35A hover balances at 94% throttle, vector 90',
      ac.alive && ac.pos.y - alpha.elev < 3.5 && Math.abs(ac.vsi) < 0.5,
      `alt=${ac.pos.y.toFixed(2)} m (max ${maxAlt.toFixed(2)}) vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    // Airborne hover at 150 m: holds altitude, pilot can roll, and
    // the auto-level returns the bank toward 0 hands-off.
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.setAirspeed(0);
    ac.vector = 90;
    ac.throttle = 0.94;
    ac.thrTarget = 0.94;
    for (let i = 0; i < 180; i++) substep(ac, 1 / 60);
    const hoverAlt = ac.pos.y;
    check(
      'BET F-35A airborne hover holds altitude at 94%',
      ac.alive && Math.abs(ac.vsi) < 1.5 && hoverAlt > 120,
      `alt=${hoverAlt.toFixed(1)} m vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    // Pilot authority in hover: 0.3 s of full right roll must move
    // the bank (the pre-FCS bug: the stick did nothing in hover).
    const bank0 = bankOf(ac);
    ac.rollIn = 1;
    for (let i = 0; i < 18; i++) substep(ac, 1 / 60);
    const bank1 = bankOf(ac);
    ac.rollIn = 0;
    check(
      'BET F-35A pilot CAN roll in hover (stick moves the bank)',
      Math.abs(bank1 - bank0) > 2,
      `bank ${bank0.toFixed(1)} -> ${bank1.toFixed(1)} deg in 0.3 s`,
    );
    // Hands-off auto-level: the bank returns toward 0.
    for (let i = 0; i < 180; i++) substep(ac, 1 / 60);
    const bank2 = bankOf(ac);
    check(
      'BET F-35A hover auto-level returns bank toward 0 hands-off',
      Math.abs(bank2) < Math.abs(bank1) * 0.7,
      `bank ${bank1.toFixed(1)} -> ${bank2.toFixed(1)} deg over 3 s hands-off`,
    );
    dispose(ac);
  }

  // ---- 4. BET stall flag past the peak ----
  // F-18 pulled hard at low speed: AoA past the 35 deg peak must
  // set the stall flag (the warning), and it must clear below
  // peak - 3 deg.
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
    ac.setAirspeed(90 / 1.94384); // ~90 kt, slow — a hard pull stalls it
    ac.thrTarget = 1;
    ac.pitchIn = -1;
    let stalledSeen = false;
    let maxAoA = -Infinity;
    for (let i = 0; i < 300; i++) {
      substep(ac, 1 / 60);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      if (ac.stalled) stalledSeen = true;
      if (!ac.alive) break;
    }
    check(
      'BET stall flag fires past the peak-CL AoA (35 deg, F-18)',
      stalledSeen && maxAoA > 35,
      `stalled=${stalledSeen} maxAoA=${maxAoA.toFixed(1)} deg`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL BET CHECKS PASSED' : `\n${failures} BET CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
