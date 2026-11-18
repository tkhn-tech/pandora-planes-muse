// probe-vtol.ts — F-35A VTOL verification (Issue 2).
//
// The lift-fan hover model existed (vector 90 + ~94% throttle balances
// at combat mass — takeoff driver §C) but there was no way to USE it:
// no key, no button, no hover FCS (qAuth -> 0 at zero airspeed, so the
// jet could not be steered in hover), and no HUD mode beyond the
// vector-angle pill. This driver checks the physics side:
//   1. hover vertical balance at 94% throttle (unchanged from §C),
//   2. pilot roll authority IN hover + hands-off auto-level,
//   3. forward-flight transition (gradual vector slew, as the toggle
//      does) without dropping out of the sky,
//   4. no auto-level in forward flight (control check: a banked F-18
//      keeps its bank with no input — hoverBlend is 0 there).
//
// Usage: node_modules/.bin/esbuild scripts/probe-vtol.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-vtol.mjs
//   && node /tmp/probe-vtol.mjs

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

  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
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
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  const alpha = RUNWAY_TABLE[0]!;
  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * (180 / Math.PI);
  const pitchOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * (180 / Math.PI);
  const spdOf = (ac: Aircraft): number => ac.vel.length();

  // 1. Hover vertical balance at 94% (mirrors takeoff driver §C).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
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
      'F-35A hover balances at 94% throttle, vector 90',
      ac.alive && ac.pos.y - alpha.elev < 3.5 && Math.abs(ac.vsi) < 0.5,
      `alt=${ac.pos.y.toFixed(2)} m (max ${maxAlt.toFixed(2)}) vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    dispose(ac);
  }

  // 2. Hover attitude: pilot authority + hands-off auto-level.
  // Airborne hover at 150 m (above the runway, clear of the deck).
  // The throttle is pre-spooled (a pilot engaging hover already has
  // it up — the 0.55/s spool from 0 would let the jet build a sink
  // rate that 94% thrust, a 0.2% margin over weight, cannot arrest).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 0.94;
    ac.thrTarget = 0.94;
    // Settle into the hover (throttle spools 0.55/s; the jet sags a
    // few metres first, then holds).
    for (let i = 0; i < 240; i++) substep(ac, 1 / 60);
    const hoverAlt = ac.pos.y;
    check(
      'F-35A airborne hover holds altitude at 94%',
      ac.alive && Math.abs(ac.vsi) < 1.5 && hoverAlt > 120,
      `alt=${hoverAlt.toFixed(1)} m vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    // Pilot authority in hover: 0.3 s of full right roll must move
    // the bank (pre-FCS qAuth was ~0 at zero airspeed, so the stick
    // did nothing in hover).
    ac.rollIn = -1;
    for (let i = 0; i < 18; i++) substep(ac, 1 / 60);
    const bankAfterRoll = bankOf(ac);
    check(
      'F-35A pilot CAN roll in hover (stick moves the bank)',
      bankAfterRoll < -5,
      `bank=${bankAfterRoll.toFixed(1)} deg after 0.3 s full right roll`,
    );
    // Hands-off auto-level: release the stick; the FCS must bring
    // the bank back toward level within 3 s.
    ac.rollIn = 0;
    for (let i = 0; i < 180; i++) substep(ac, 1 / 60);
    const bankLevel = bankOf(ac);
    const pitchLevel = pitchOf(ac);
    check(
      'F-35A hover auto-level returns bank toward 0 hands-off',
      ac.alive && Math.abs(bankLevel) < 8,
      `bank=${bankLevel.toFixed(1)} deg pitch=${pitchLevel.toFixed(1)} deg after 3 s hands-off`,
    );
    dispose(ac);
  }

  // 3. Transition: hover -> forward flight via the toggle's
  // lift-following vector slew (mirrors main.ts: the nozzle comes aft
  // at 36 deg/s but never below sin(vector) >= (1 - gload) / (T/W),
  // where gload is the wing's REAL lift right now — so the nozzle
  // cannot outrun the wing). The pilot rotates to ~10 deg AoA as
  // speed builds, as in a real STOVL transition: that is what makes
  // the wing take the load. The jet must accelerate past flying
  // speed without losing more than 25 m.
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 0.94;
    ac.thrTarget = 0.94;
    for (let i = 0; i < 240; i++) substep(ac, 1 / 60);
    const hoverAlt = ac.pos.y;
    ac.throttle = 1.0;
    ac.thrTarget = 1.0;
    let minAlt = hoverAlt;
    let vec = 90;
    for (let i = 0; i < 1200; i++) {
      // Toggle slew, lift-following floor (main.ts mirror).
      const liftFrac = Math.max(0, Math.min(1, ac.gload));
      const tw =
        (ac.throttle * ac.spec.thrustMax) / (ac.spec.mass * 9.81);
      const need = (1 - liftFrac) / Math.max(tw, 0.01);
      const minVec = (Math.asin(Math.min(1, Math.max(0, need))) * 180) / Math.PI;
      const goal = Math.max(0, minVec);
      const step = 36 / 60;
      vec =
        Math.abs(goal - vec) <= step
          ? goal
          : vec + Math.sign(goal - vec) * step;
      ac.vector = vec;
      // Pilot rotates to hold ~10 deg AoA as speed builds (the wing
      // only takes the load at AoA — level pitch never gets there).
      // pitchIn < 0 = nose up, so the error is (aoa - target).
      ac.pitchIn = Math.max(-1, Math.min(1, (ac.aoaDeg - 10) * 0.1));
      substep(ac, 1 / 60);
      minAlt = Math.min(minAlt, ac.pos.y);
      if (!ac.alive) break;
      if (vec === 0 && ac.vel.length() > 75) break;
    }
    const spd = spdOf(ac);
    check(
      'F-35A transition: hover -> forward flight without dropping',
      ac.alive && spd > 70 && hoverAlt - minAlt < 25,
      `speed=${(spd * 1.944).toFixed(0)} kt alt loss=${(hoverAlt - minAlt).toFixed(1)} m (hover ${hoverAlt.toFixed(1)} m, min ${minAlt.toFixed(1)} m)`,
    );
    dispose(ac);
  }

  // 4. Control: no auto-level in forward flight. A banked F-18 with
  // no input keeps its bank (hoverBlend is 0 at vector 0).
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
    ac.setAirspeed(150);
    ac.thrTarget = 0.8;
    ac.rollIn = -1;
    for (let i = 0; i < 12; i++) substep(ac, 1 / 60); // ~30 deg bank
    ac.rollIn = 0;
    const bank0 = bankOf(ac);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    const bank1 = bankOf(ac);
    check(
      'F-18 forward flight: NO auto-level (bank persists, no input)',
      Math.abs(bank1 - bank0) < 12 && Math.abs(bank1) > 10,
      `bank ${bank0.toFixed(1)} deg -> ${bank1.toFixed(1)} deg over 2 s hands-off`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
