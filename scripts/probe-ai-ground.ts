// probe-ai-ground.ts — AI ground height (Issue 6).
//
// The AI's terrain avoidance (hard floor: ground + 60 m, and the
// 260 m-ahead pull-up) reads the groundHeight function it is given.
// The analytic heightAt() sits up to ~107 m BELOW the real Rapier
// heightfield surface at steep spots, so with the analytic height
// the AI's floor could sit BELOW the real terrain — the jet would
// fly into a hill it thought it was 60 m above. main.ts now passes
// the exact heightfieldHeightAt (the same surface the player
// controller, VFX and weapons already use). Checks:
//   1. The analytic height really does underestimate somewhere by
//      > 60 m (the fix matters — otherwise this is a no-op).
//   2. AI with the EXACT heightfield survives a low pass over the
//      worst spot (pulls up, stays alive).
//   3. INFO only: the same pass with the ANALYTIC height (the old
//      wiring) — expected to crash into the terrain, demonstrating
//      the bug the fix addresses.
//
// Usage: node_modules/.bin/esbuild scripts/probe-ai-ground.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-ai-ground.mjs
//   && node /tmp/probe-ai-ground.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';
import {
  generateHeights,
  heightAt,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail: string): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name} — ${detail}`);
  if (!ok) failures++;
}

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
  const terrainCollider = world.createCollider(
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
  const analyticH = (x: number, z: number): number => heightAt(x, z, heights);
  const exactH = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const carrier = new HelicarrierController(scene, world, {});
  await carrier.build();
  carrier.reset(CARRIER_START_POS.clone(), Math.atan2(CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x, CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z));

  // Worst analytic underestimate (exact - analytic) on the map.
  let worst = { x: 0, z: 0, gap: 0, exact: 0, analytic: 0 };
  for (let z = -5900; z <= 5900; z += 60) {
    for (let x = -5900; x <= 5900; x += 60) {
      const e = exactH(x, z);
      const a = analyticH(x, z);
      if (e - a > worst.gap) worst = { x, z, gap: e - a, exact: e, analytic: a };
    }
  }
  check(
    'AI ground height: analytic height underestimates the real surface by > 60 m somewhere (the fix matters)',
    worst.gap > 60,
    `worst gap ${worst.gap.toFixed(1)} m at (${worst.x},${worst.z}) exact=${worst.exact.toFixed(1)} m analytic=${worst.analytic.toFixed(1)} m`,
  );

  // Dummy player (the AI needs one; it flies straight and level, far
  // from the test jet so the AI stays in patrol, not engage).
  const player = new Aircraft({
    scene, world, spec: SPECS.fa18!, isPlayer: true, modelKey: 'fa18', events: {},
  });
  player.terrainHeightAt = exactH;
  player.resetAt(5000, 800, 5000, 90);
  player.setAirspeed(100);
  const wind = new THREE.Vector3(0, 0, 0);
  const dt = 1 / 60;

  // Fly the AI jet low toward the worst spot: start 800 m west of it,
  // at an altitude BETWEEN the analytic floor (analytic + 60) and the
  // real surface — so the analytic AI thinks it is safe (above its
  // floor) while the exact AI sees the hill and pulls up. Heading
  // east (90 deg) straight at the spot, 80 m/s.
  const runPass = (useExact: boolean): { alive: boolean; minClear: number } => {
    const jet = new Aircraft({
      scene, world, spec: SPECS.mig29!, isPlayer: false, modelKey: 'mig29', events: {},
    });
    jet.terrainHeightAt = exactH;
    const startAlt = worst.analytic + 60 + (worst.gap - 60) / 2; // between analytic floor and real surface
    jet.resetAt(worst.x - 800, startAlt, worst.z, 90);
    jet.setAirspeed(80);
    jet.thrTarget = 0.6;
    const ai = new AircraftAIController(jet, 'veteran', 'enemy', {
      groundHeight: useExact ? exactH : analyticH,
      originShift: new THREE.Vector3(0, 0, 0),
    });
    let minClear = Infinity;
    for (let i = 0; i < 60 * 30; i++) {
      world.timestep = dt;
      world.step(eq);
      eq.drainCollisionEvents(() => {});
      player.pitchIn = 0; player.rollIn = 0; player.yawIn = 0;
      player.step(dt, wind, null);
      const ctrl = ai.update(dt, player, carrier, i * dt);
      jet.step(dt, ctrl, wind, exactH, null);
      minClear = Math.min(minClear, jet.pos.y - exactH(jet.pos.x, jet.pos.z));
      if (!jet.alive) break;
      if (jet.pos.x > worst.x + 800) break; // flew past the spot
    }
    const alive = jet.alive;
    jet.dispose();
    return { alive, minClear };
  };

  const exact = runPass(true);
  check(
    'AI ground height: AI with the EXACT heightfield survives a low pass over the worst analytic-underestimate spot',
    exact.alive && exact.minClear > 0,
    `alive=${exact.alive} min clearance above the real surface=${exact.minClear.toFixed(1)} m`,
  );
  const analytic = runPass(false);
  console.log(
    `INFO  AI ground height: same pass with the ANALYTIC height (old wiring) — alive=${analytic.alive} min clearance=${analytic.minClear.toFixed(1)} m (a crash here is the bug the fix addresses; the exact-height pass above is the check).`,
  );

  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => { console.error(e); process.exit(2); });
