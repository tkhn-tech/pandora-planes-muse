// Probe: AI two-loop steering + RTB pattern, in node.
//
// Spawns a MiG-29 (enemy, veteran) in the air near an airborne player
// F-18 and steps the AI + physics. Checks the AI turns toward the
// player (heading error shrinks, distance closes), banks the right way,
// and stays alive. Then damages it (hp < 20) so it RTBs, and checks it
// descends along the pattern toward the runway instead of circling.
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';
import {
  generateHeights,
  heightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

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
  const eq = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
    }
  }
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
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
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const groundH = (x: number, z: number): number => heightAt(x, z, heights);
  const carrier = new HelicarrierController(scene, world, {});
  await carrier.build();
  carrier.reset(CARRIER_START_POS.clone(), Math.atan2(CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x, CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z));

  // Player F-18 airborne, flying east at 100 m/s, 500 m.
  const player = new Aircraft({
    scene,
    world,
    spec: SPECS.fa18,
    isPlayer: true,
    modelKey: 'fa18',
    events: {},
  });
  player.terrainHeightAt = groundH;
  player.resetAt(-1000, 500, -900, 90);
  player.setAirspeed(100);
  player.thrTarget = 0.6;

  // Enemy MiG-29, 1000 m EAST of the player (head-on), heading west.
  const enemy = new Aircraft({
    scene,
    world,
    spec: SPECS.mig29,
    isPlayer: false,
    modelKey: 'mig29',
    events: {},
  });
  enemy.terrainHeightAt = groundH;
  enemy.resetAt(0, 600, -900, 270);
  enemy.setAirspeed(150);
  enemy.thrTarget = 0.8;
  const ai = new AircraftAIController(enemy, 'veteran', 'enemy', {
    groundHeight: groundH,
    originShift: new THREE.Vector3(0, 0, 0),
  });

  const wind = new THREE.Vector3(0, 0, 0);
  const dt = 1 / 60;
  let time = 0;
  let lastCtrl: { rollIn?: number; yawIn?: number; pitchIn?: number } = {};
  const substep = (): void => {
    world.timestep = dt;
    world.step(eq);
    eq.drainCollisionEvents(() => {});
    // Player flies straight and level.
    player.pitchIn = 0;
    player.rollIn = 0;
    player.yawIn = 0;
    player.step(dt, wind, null);
    const ctrl = ai.update(dt, player, carrier, time);
    lastCtrl = ctrl;
    enemy.step(dt, ctrl, wind, groundH, null);
    time += dt;
  };
  const hdgErr = (): number => {
    const toP = player.pos.clone().sub(enemy.pos);
    const des = Math.atan2(toP.x, toP.z);
    const cur = Math.atan2(enemy.fwd().x, enemy.fwd().z);
    let e = des - cur;
    while (e > Math.PI) e -= 2 * Math.PI;
    while (e < -Math.PI) e += 2 * Math.PI;
    return (e * 180) / Math.PI;
  };
  const bank = (): number =>
    (new THREE.Euler().setFromQuaternion(enemy.quat, 'YXZ').z * 180) / Math.PI;

  console.log('--- engage: MiG-29 should turn toward the player and close ---');
  for (let i = 0; i < 1800; i++) {
    substep();
    // High-frequency trace around the departure (t=2-5 s).
    if (time >= 2 && time <= 5 && i % 15 === 0) {
      console.log(
        `TRACE t=${time.toFixed(2)}s pitchIn=${(lastCtrl.pitchIn ?? 0).toFixed(2)} rollIn=${(lastCtrl.rollIn ?? 0).toFixed(2)} yawIn=${(lastCtrl.yawIn ?? 0).toFixed(2)} ` +
          `spd=${(enemy.ias * 1.944).toFixed(0)}kt aoa=${enemy.aoaDeg.toFixed(1)} bank=${bank().toFixed(1)} ` +
          `vsi=${enemy.vsi.toFixed(1)} thr=${enemy.throttle.toFixed(2)} stalled=${enemy.stalled} spin=${enemy.spin.toFixed(2)}`,
      );
    }
    if (i % 120 === 0) {
      console.log(
        `t=${time.toFixed(0)}s state=${ai.state} dist=${enemy.pos.distanceTo(player.pos).toFixed(0)}m ` +
          `hdgErr=${hdgErr().toFixed(1)}deg bank=${bank().toFixed(1)}deg ` +
          `spd=${(enemy.ias * 1.944).toFixed(0)}kt alt=${enemy.pos.y.toFixed(0)}m ` +
          `aoa=${enemy.aoaDeg.toFixed(1)} stalled=${enemy.stalled} spin=${enemy.spin.toFixed(2)} ` +
          `thr=${enemy.throttle.toFixed(2)} rollIn=${(lastCtrl.rollIn ?? 0).toFixed(2)} yawIn=${(lastCtrl.yawIn ?? 0).toFixed(2)} alive=${enemy.alive}`,
      );
    }
    if (!enemy.alive) break;
  }
  const d0 = enemy.pos.distanceTo(player.pos);
  console.log(`engage end: dist=${d0.toFixed(0)}m alive=${enemy.alive} state=${ai.state}`);

  console.log('--- RTB: damage the MiG-29, it should descend to a runway ---');
  enemy.hp = 15;
  for (let i = 0; i < 18000; i++) {
    substep();
    if (i % 300 === 0) {
      // RTB goal is the final-approach fix (-3605, 122, -900) — print
      // the distance and heading error to it so the convergence is
      // visible. (The probe measures to the fix; the jet's actual
      // goal advances approach -> final -> rollout.)
      const toGoal = new THREE.Vector3(-3605, 122, -900).sub(enemy.pos);
      const desHdg = Math.atan2(toGoal.x, toGoal.z);
      const curHdg = Math.atan2(enemy.fwd().x, enemy.fwd().z);
      let hdgErrDeg = ((desHdg - curHdg) * 180) / Math.PI;
      while (hdgErrDeg > 180) hdgErrDeg -= 360;
      while (hdgErrDeg < -180) hdgErrDeg += 360;
      console.log(
        `t=${time.toFixed(0)}s state=${ai.state} alt=${enemy.pos.y.toFixed(0)}m ` +
          `spd=${(enemy.ias * 1.944).toFixed(0)}kt pos=(${enemy.pos.x.toFixed(0)},${enemy.pos.z.toFixed(0)}) ` +
          `distGoal=${toGoal.length().toFixed(0)}m hdgErrGoal=${hdgErrDeg.toFixed(0)}deg ` +
          `bank=${bank().toFixed(1)}deg rollIn=${(lastCtrl.rollIn ?? 0).toFixed(2)} ` +
          `thr=${enemy.throttle.toFixed(2)} grounded=${enemy.grounded} alive=${enemy.alive}`,
      );
    }
    if (!enemy.alive) break;
    if (enemy.grounded && enemy.ias < 10) break;
  }
  console.log(
    `RTB end: state=${ai.state} alt=${enemy.pos.y.toFixed(0)}m spd=${(enemy.ias * 1.944).toFixed(0)}kt ` +
      `grounded=${enemy.grounded} alive=${enemy.alive}`,
  );
  console.log('done');
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
