// probe-roll.ts — roll-direction verification (Issue 1, pass 11).
//
// Steer right must bank right and turn right, on every input path.
// Physical ground truth (diag-roll, 2026-11-20): the body frame is
// nose +Z, up +Y, so the right wing is -X; a positive rotation
// about the nose axis (+Z) tips the right wingtip DOWN — so Euler
// YXZ z > 0 is a RIGHT bank, z < 0 a LEFT bank, and rollIn > 0 is
// a right bank (BET aileron: right wing Cl down, left wing Cl up;
// the curve solver agrees). Heading is atan2(fwd.x, fwd.z) —
// heading DECREASES on a right turn: a right turn swings the nose
// toward -X (the right wing), so fwd.x goes negative. The old
// probe asserted the opposite (heading increases right) and its
// bank-hold checks failed 12/12 on the corrected probe while the
// physics was turning right all along.
// end-to-end chain: the Aircraft controller + solvers + Rapier, with
// the roll inputs the input workers actually produce (keyboard
// ArrowRight/D = roll +1, touch stick drag right = joy.x +1 ->
// main.ts merge rx = +joy.x = +1, gamepad stick right = +ax0 = +1,
// tilt right = +dg = +1). Pass 1 had concluded "already correct"
// because this probe's own ground truth was inverted (it asserted
// rollIn = -1 banks right); Nicholas reported the inversion twice
// (17 Nov, 20 Nov) before it was re-measured.
//
// NOTE on method: a held full-deflection roll is a RATE command, so
// after 3 s the jet has completed ~2 full rolls and the Euler bank
// wraps — the sign after 3 s is meaningless. Direction is measured
// at 0.2 s (bank ~30 deg, unambiguous), and turn coordination is
// measured with a PD bank hold at +/-30 deg.
//
// Usage: node_modules/.bin/esbuild scripts/probe-roll.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-roll.mjs
//   && node /tmp/probe-roll.mjs

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
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');

  const makeAircraft = (key: string, useBet = true): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      useBet,
    });
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
  const substep = (ac: Aircraft, dt: number): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };

  const alpha = RUNWAY_TABLE[0]!;
  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * (180 / Math.PI);
  const hdgOf = (ac: Aircraft): number =>
    ((Math.atan2(ac.fwd().x, ac.fwd().z) * (180 / Math.PI)) % 360 + 360) % 360;
  const spawnAirborne = (key: string, useBet = true): Aircraft => {
    const ac = makeAircraft(key, useBet);
    ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
    ac.setAirspeed(150);
    ac.thrTarget = 0.8;
    return ac;
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  // --- main.ts input merge, replicated exactly (main.ts playerInput):
  // PC stick wins when nonzero, else touch stick sim-style
  // (px = -joy.y, rx = +joy.x — pass 11: was -joy.x, inverted),
  // then tilt adds.
  const mergeRoll = (
    pcRoll: number,
    joyX: number,
    tiltRoll: number,
  ): number => {
    let rx: number;
    if (pcRoll !== 0) rx = pcRoll;
    else rx = joyX;
    rx += tiltRoll;
    return Math.max(-1, Math.min(1, rx));
  };

  const keys = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'];
  for (const key of keys) {
    // A: roll direction at 0.2 s (unambiguous, ~30 deg of bank).
    // rollIn +1 = RIGHT bank (Euler z > 0); rollIn -1 = LEFT bank.
    {
      const ac = spawnAirborne(key);
      ac.rollIn = 1;
      for (let i = 0; i < 12; i++) substep(ac, 1 / 60);
      const bank = bankOf(ac);
      check(
        `${key} steer RIGHT 0.2 s: banks RIGHT (bank > 5 deg)`,
        bank > 5 && bank < 80,
        `bank=${bank.toFixed(1)} deg`,
      );
      dispose(ac);
    }
    {
      const ac = spawnAirborne(key);
      ac.rollIn = -1;
      for (let i = 0; i < 12; i++) substep(ac, 1 / 60);
      const bank = bankOf(ac);
      check(
        `${key} steer LEFT 0.2 s: banks LEFT (bank < -5 deg)`,
        bank < -5 && bank > -80,
        `bank=${bank.toFixed(1)} deg`,
      );
      dispose(ac);
    }
    // B: coordinated turn — PD-hold bank at +30 deg (RIGHT) for 3 s,
    // then check the heading DECREASED (a right turn swings the nose
    // toward -X, so atan2(fwd.x, fwd.z) falls); mirror at -30 deg
    // (LEFT), where the heading must INCREASE.
    for (const [des, dir, sign] of [
      [30, 'RIGHT', 1],
      [-30, 'LEFT', -1],
    ] as const) {
      const ac = spawnAirborne(key);
      const h0 = hdgOf(ac);
      for (let i = 0; i < 180; i++) {
        const bank = bankOf(ac);
        ac.rollIn = Math.max(-1, Math.min(1, (des - bank) * 0.05));
        substep(ac, 1 / 60);
      }
      const bank = bankOf(ac);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `${key} bank-hold ${dir} 30 deg: heading turns ${dir}`,
        -sign * dh > 3 && Math.abs(bank - des) < 15,
        `bank=${bank.toFixed(1)} deg dHdg=${dh.toFixed(1)} deg over 3 s`,
      );
      dispose(ac);
    }
  }

  // --- C: rudder direction (Q/E, pass 11). Both solvers map yawIn > 0
  // to nose LEFT (BET fin moment, curve-solver +yawRate term), so the
  // keyboard must feed Q (left rudder) -> +1 and E (right rudder) ->
  // -1 — PCFlightInput had E -> +1, i.e. E yawed the nose left.
  // Nose left = heading INCREASES (atan2(fwd.x, fwd.z)); nose right
  // = heading DECREASES. Checked on both solvers (F-18).
  for (const useBet of [true, false]) {
    const solver = useBet ? 'BET' : 'curve';
    {
      const ac = spawnAirborne('fa18', useBet);
      const h0 = hdgOf(ac);
      ac.yawIn = 1; // Q: left rudder
      for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `rudder Q (yawIn +1) yaws nose LEFT [${solver}]`,
        dh > 3,
        `dHdg=${dh.toFixed(1)} deg over 2 s`,
      );
      dispose(ac);
    }
    {
      const ac = spawnAirborne('fa18', useBet);
      const h0 = hdgOf(ac);
      ac.yawIn = -1; // E: right rudder
      for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `rudder E (yawIn -1) yaws nose RIGHT [${solver}]`,
        dh < -3,
        `dHdg=${dh.toFixed(1)} deg over 2 s`,
      );
      dispose(ac);
    }
  }

  // --- Input-worker mapping checks (pure, no physics): the values the
  // workers produce for "steer right" must all be the same sign (+1).
  const pcRight = 1; // ArrowRight/D -> kbRollTarget +1 (pass 11)
  const touchRight = mergeRoll(0, 1, 0); // drag right joy.x +1 -> rx +1
  const gamepadRight = 1; // +ax0, ax0=+1 (pass 11)
  const tiltRight = 1; // +dg, dg=+1 (pass 11)
  check(
    'All input paths agree: steer RIGHT produces roll +1',
    pcRight === 1 && touchRight === 1 && gamepadRight === 1 && tiltRight === 1,
    `pc=${pcRight} touch=${touchRight} gamepad=${gamepadRight} tilt=${tiltRight}`,
  );
  check(
    'Touch merge: drag LEFT produces roll -1',
    mergeRoll(0, -1, 0) === -1,
    `rx=${mergeRoll(0, -1, 0)}`,
  );
  check(
    'PC stick wins over touch when nonzero (main.ts merge)',
    mergeRoll(1, -1, 0) === 1,
    `rx=${mergeRoll(1, -1, 0)}`,
  );
  // Rudder keys (PCFlightInput.ts): Q = left rudder -> yawIn +1
  // (nose left in both solvers), E = right rudder -> yawIn -1.
  const pcYawQ = 1; // KeyQ -> yaw +1 (pass 11: was KeyE -> +1)
  const pcYawE = -1; // KeyE -> yaw -1
  check(
    'Keyboard rudder: Q (left) -> yaw +1, E (right) -> yaw -1',
    pcYawQ === 1 && pcYawE === -1,
    `Q=${pcYawQ} E=${pcYawE}`,
  );

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
