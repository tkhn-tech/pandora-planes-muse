// Probe: F-35A hover on the real terrain (driver setup), trace every 0.5 s.
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import {
  generateHeights,
  heightAt,
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
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min),
    terrainBody,
  );
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
  ac.vector = 90;
  ac.thrTarget = 1.0;
  const wind = new THREE.Vector3(0, 0, 0);
  for (let i = 0; i < 300; i++) {
    world.timestep = 1 / 60;
    world.step(eq);
    ac.step(1 / 60, wind, null);
    if (i % 30 === 0) {
      const e = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
      console.log(
        `t=${(i / 60).toFixed(1)}s thr=${ac.throttle.toFixed(2)} vec=${ac.vectorDeg} ` +
          `alt=${ac.pos.y.toFixed(2)} vsi=${ac.vel.y.toFixed(2)} ias=${ac.ias.toFixed(1)} ` +
          `aoa=${ac.aoaDeg.toFixed(1)} pitch=${((e.x * 180) / Math.PI).toFixed(1)} ` +
          `roll=${((e.z * 180) / Math.PI).toFixed(1)} g=${ac.gload.toFixed(2)} ` +
          `grounded=${ac.grounded} alive=${ac.alive}`,
      );
    }
  }
  ac.dispose();
  console.log('done');
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
