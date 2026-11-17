// Probe: is the MiG-29 aero stable on its own? Straight and level,
// neutral stick, no AI. Then a gentle 30 deg bank hold. If it departs
// with no input, the aero is broken; if it is stable, the AI is.
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
      .setRestitution(0),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const groundH = (x: number, z: number): number => heightAt(x, z, heights);
  const ac = new Aircraft({
    scene,
    world,
    spec: SPECS.mig29,
    isPlayer: false,
    modelKey: 'mig29',
    events: {},
  });
  ac.terrainHeightAt = groundH;
  ac.resetAt(-1000, 600, 0, 90);
  ac.setAirspeed(150);
  ac.thrTarget = 0.8;
  ac.gear = false;
  ac.grounded = false;
  const wind = new THREE.Vector3(0, 0, 0);
  const dt = 1 / 60;
  const bank = (): number =>
    (new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * 180) / Math.PI;
  console.log('--- straight and level, neutral stick, 10 s ---');
  for (let i = 0; i < 600; i++) {
    world.timestep = dt;
    world.step(eq);
    eq.drainCollisionEvents(() => {});
    ac.pitchIn = 0;
    ac.rollIn = 0;
    ac.yawIn = 0;
    ac.step(dt, wind, null);
    if (i % 120 === 0) {
      console.log(
        `t=${(i / 60).toFixed(0)}s spd=${(ac.ias * 1.944).toFixed(0)}kt alt=${ac.pos.y.toFixed(0)}m ` +
          `aoa=${ac.aoaDeg.toFixed(1)} bank=${bank().toFixed(1)} stalled=${ac.stalled} spin=${ac.spin.toFixed(2)} alive=${ac.alive}`,
      );
    }
    if (!ac.alive) break;
  }
  console.log('--- hard turn: rollIn -1.0 + pitchIn -0.5 (full right roll, moderate pull), 10 s ---');
  for (let i = 0; i < 600; i++) {
    world.timestep = dt;
    world.step(eq);
    eq.drainCollisionEvents(() => {});
    ac.pitchIn = -0.5;
    ac.rollIn = -1.0;
    ac.yawIn = 0;
    ac.step(dt, wind, null);
    if (i % 120 === 0) {
      console.log(
        `t=${(10 + i / 60).toFixed(0)}s spd=${(ac.ias * 1.944).toFixed(0)}kt alt=${ac.pos.y.toFixed(0)}m ` +
          `aoa=${ac.aoaDeg.toFixed(1)} bank=${bank().toFixed(1)} stalled=${ac.stalled} spin=${ac.spin.toFixed(2)} alive=${ac.alive}`,
      );
    }
    if (!ac.alive) break;
  }
  console.log('done');
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
