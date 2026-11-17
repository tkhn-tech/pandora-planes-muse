// Scratch probe: trace the F-18 80-deg max-performance turn.
// Prints bank, AoA, throttle, ias, gload every 0.5 s to see why the
// speed bleeds 400 -> 223 kt (the §0 calibration says drag ~205 kN
// vs thrust 196 kN, a slow bleed, not 9 m/s^2).
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
  const wind = new THREE.Vector3(0, 0, 0);
  const ac = new Aircraft({
    scene, world, spec: SPECS.fa18!, isPlayer: true, modelKey: 'fa18', events: {},
  });
  ac.terrainHeightAt = groundH;
  ac.resetAt(-1400, 200, -900, 90);
  ac.setAirspeed(400 / 1.94384);
  ac.thrTarget = 1;
  const bankDes = (80 * Math.PI) / 180;
  const aoaTarget = 12;
  console.log('t    bank  aoa   thr   ias(kt) g    alt');
  for (let i = 0; i < 600; i++) {
    const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
    ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
    ac.pitchIn = Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3));
    ac.yawIn = 0;
    world.timestep = 1 / 60;
    world.step(eq);
    eq.drainCollisionEvents(() => undefined);
    ac.step(1 / 60, wind, null);
    if (i % 30 === 0) {
      console.log(
        `${(i / 60).toFixed(1).padStart(4)} ` +
        `${((bankNow * 180) / Math.PI).toFixed(0).padStart(5)} ` +
        `${ac.aoaDeg.toFixed(1).padStart(5)} ` +
        `${ac.throttle.toFixed(2).padStart(5)} ` +
        `${(ac.ias * 1.94384).toFixed(0).padStart(6)} ` +
        `${ac.gload.toFixed(1).padStart(4)} ` +
        `${ac.pos.y.toFixed(0).padStart(5)}`,
      );
    }
    if (!ac.alive) break;
  }
}
main().catch((e) => { console.error(e); process.exit(2); });
