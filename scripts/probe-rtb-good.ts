// Scratch probe: RTB from a GOOD position — jet already east of Alpha,
// at 300 m, 80 m/s, heading east (aligned with the runway). Damage it
// and check it flies the straight-in and lands (grounded, slow).
// If this lands, the RTB mechanism works and the probe-ai worst case
// (far west after a dogfight) is the edge case needing track-following.
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
    seed: 1337, chunkX: 0, chunkZ: 0,
    size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eq = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++)
    for (let ix = 0; ix < res; ix++)
      hfHeights[ix * res + iz] = heights[iz * res + ix];
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    ).setFriction(0)
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
  // Dummy player (the AI needs one; it flies straight and level).
  const player = new Aircraft({
    scene, world, spec: SPECS.fa18!, isPlayer: true, modelKey: 'fa18', events: {},
  });
  player.terrainHeightAt = groundH;
  player.resetAt(-1000, 500, -900, 90);
  player.setAirspeed(100);
  // Two scenarios (issue 4, third pass): (1) good position — at the
  // final-approach fix, on the glide slope; (2) HIGH arrival — at the
  // fix but 300 m altitude, ~128 m above the slope (the live failure:
  // the damaged AI arrived 59-94 m high and flew past). Both must
  // land (grounded, slow) on the Alpha runway.
  const scenarios = [
    { name: 'good position (on the glide slope)', y: 172 },
    { name: 'high arrival (128 m above the slope)', y: 300 },
  ];
  let failures = 0;
  for (const sc of scenarios) {
    // Enemy MiG-29 at the final-approach fix — 2 km west of the Alpha
    // touchdown ON the extended centreline, heading EAST (90 deg),
    // 80 m/s. Alpha is (-2800, -1800) elev 50, hdg 90, len 650:
    // touchdown = Alpha - 205 m along the landing direction =
    // (-3005, 50, -1800); fix = touchdown - 2000 m = (-5005, 172,
    // -1800). (The old start (-3605, 122, -900) was 900 m NORTH of
    // the centreline and 600 m PAST the touchdown heading away — not
    // a good position at all, which is why the jet circled.)
    const enemy = new Aircraft({
      scene, world, spec: SPECS.mig29!, isPlayer: false, modelKey: 'mig29', events: {},
    });
    enemy.terrainHeightAt = groundH;
    enemy.resetAt(-5005, sc.y, -1800, 90);
    enemy.setAirspeed(80);
    enemy.thrTarget = 0.5;
    const ai = new AircraftAIController(enemy, 'veteran', 'enemy', {
      groundHeight: groundH,
      originShift: new THREE.Vector3(0, 0, 0),
    });
    const wind = new THREE.Vector3(0, 0, 0);
    const dt = 1 / 60;
    let time = 0;
    enemy.hp = 15; // damaged -> RTB immediately
    let landed = false;
    let landTime = 0;
    for (let i = 0; i < 60 * 180; i++) {
      world.timestep = dt;
      world.step(eq);
      eq.drainCollisionEvents(() => {});
      player.pitchIn = 0; player.rollIn = 0; player.yawIn = 0;
      player.step(dt, wind, null);
      const ctrl = ai.update(dt, player, carrier, time);
      enemy.step(dt, ctrl, wind, groundH, null);
      time += dt;
      if (enemy.grounded && enemy.ias < 10) { landed = true; landTime = time; break; }
      if (!enemy.alive) break;
    }
    console.log(
      landed
        ? `PASS  RTB ${sc.name}: LANDED (grounded, slow) at t=${landTime.toFixed(0)} s, pos=(${enemy.pos.x.toFixed(0)},${enemy.pos.z.toFixed(0)})`
        : `FAIL  RTB ${sc.name}: did NOT land — alt=${enemy.pos.y.toFixed(0)}m spd=${(enemy.ias * 1.944).toFixed(0)}kt grounded=${enemy.grounded} alive=${enemy.alive}`,
    );
    if (!landed) failures++;
    enemy.dispose();
    // Reset the player so the next scenario starts clean.
    player.resetAt(-1000, 500, -900, 90);
    player.setAirspeed(100);
  }
  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}
main().catch((e) => { console.error(e); process.exit(2); });
