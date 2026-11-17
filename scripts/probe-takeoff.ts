// Probe: F-35A flaps 0 takeoff — trace speed / AoA / pitch / rates.
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
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
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
    }
  }
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const tc = world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  void tc;
  // Runway boxes (same as the driver / generator) — without these the
  // nose ski crossing a heightfield cell boundary explodes the contact.
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
  const spec = SPECS.f35a;
  const ac = new Aircraft({
    scene,
    world,
    spec,
    isPlayer: true,
    modelKey: 'f35a',
    events: {},
  });
  ac.terrainHeightAt = (x, z) => heightAt(x, z, heights);
  ac.resetAt(-1400, 0, -900, 90);
  for (let i = 0; i < 120; i++) {
    world.timestep = 1 / 60;
    world.step(eq);
    ac.step(1 / 60, new THREE.Vector3(0, 0, 0), null);
  }
  ac.flaps = 0;
  ac.thrTarget = 1;
  ac.pitchIn = -1;
  const wind = new THREE.Vector3(0, 0, 0);
  for (let i = 0; i < 900; i++) {
    const preVel = ac.body.linvel();
    world.timestep = 1 / 60;
    world.step(eq);
    const postVel = ac.body.linvel();
    // Drain collision events into notifyContact (driver does this).
    eq.drainCollisionEvents((h1, h2, started) => {
      void h1;
      void h2;
      void started;
    });
    ac.step(1 / 60, wind, null);
    const dvx = postVel.x - preVel.x;
    const dvy = postVel.y - preVel.y;
    const dvz = postVel.z - preVel.z;
    const kick = Math.hypot(dvx, dvy, dvz);
    if (kick > 0.5) {
      console.log(
        `KICK t=${(i / 60).toFixed(2)}s x=${ac.pos.x.toFixed(1)} pre=(${preVel.x.toFixed(2)},${preVel.y.toFixed(2)},${preVel.z.toFixed(2)}) post=(${postVel.x.toFixed(2)},${postVel.y.toFixed(2)},${postVel.z.toFixed(2)}) dv=(${dvx.toFixed(2)},${dvy.toFixed(2)},${dvz.toFixed(2)}) |dv|=${kick.toFixed(2)}`,
      );
    }
    if (i % 30 === 0) {
      const e = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
      console.log(
        `t=${(i / 60).toFixed(1)}s x=${ac.pos.x.toFixed(0)} y=${ac.pos.y.toFixed(1)} ` +
          `spd=${(ac.ias * 1.944).toFixed(0)}kt aoa=${ac.aoaDeg.toFixed(1)} ` +
          `pitch=${((e.x * 180) / Math.PI).toFixed(1)} roll=${((e.z * 180) / Math.PI).toFixed(1)} ` +
          `g=${ac.gload.toFixed(2)} stalled=${ac.stalled} spin=${ac.spin.toFixed(2)} ` +
          `grounded=${ac.grounded} onSurf=${ac.onSurface} alive=${ac.alive}`,
      );
    }
    if (!ac.alive) break;
  }
  ac.dispose();
  console.log('done');
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
