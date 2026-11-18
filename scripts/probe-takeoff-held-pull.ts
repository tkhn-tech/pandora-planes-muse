// probe-takeoff-held-pull.ts — held full pull on takeoff (pass 5).
//
// A real player holds full pull through the takeoff. Before the
// pass-5 AoA limiter, the BET's elevator moment overwhelmed the
// pitch damping and a held pull ran the F-22 to AoA 158 deg (deep
// stall, fall back onto the runway) and the other jets to a 43-51
// deg wallow — probe-bet never caught it because it modulates the
// pull to an AoA target after liftoff. This probe holds full pull
// for all six jets and requires a clean takeoff: alive, lifts off
// inside the 650 m runway, max AoA <= 50 deg, and climbing at 15 s.
//
// Usage: node_modules/.bin/esbuild scripts/probe-takeoff-held-pull.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-takeoff-held-pull.mjs
//   && node /tmp/probe-takeoff-held-pull.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
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

const WIND = new THREE.Vector3(3, 0, 0);

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
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix]!;
    }
  }
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const alpha = RUNWAY_TABLE[0]!;
  const along = alpha.len / 2 + 100;
  const across = alpha.width / 2 + 2;
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(across, 0.5, along)
      .setTranslation(alpha.x, alpha.elev - 0.45, alpha.z)
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0),
    terrainBody,
  );
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const deck: DeckProvider = {
    alive: false,
    deckHeightAt: () => -Infinity,
    velocity: () => new THREE.Vector3(),
    velocityAt: () => new THREE.Vector3(),
    yawRate: () => 0,
    yawRateAt: () => 0,
    local: () => null,
  };

  for (const key of ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const) {
    origin.reset();
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck,
    });
    ac.terrainHeightAt = heightAt;
    origin.registerBody(ac.body);
    const owners = new Map<number, Aircraft>();
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    const drain = (): void => {
      eventQueue.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const hit = owners.get(h1) ?? owners.get(h2);
        if (!hit || !hit.alive) return;
        hit.notifyContact(hit.vel.y, false);
      });
    };
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.thrTarget = 1;
    ac.pitchIn = -1; // held full pull, the whole takeoff
    let maxAoA = 0;
    let liftoffDist: number | null = null;
    let liftoffT = -1;
    for (let i = 0; i < 900; i++) {
      const t = i / 60;
      world.timestep = 1 / 60;
      world.step(eventQueue);
      drain();
      ac.step(1 / 60, WIND, null);
      const shift = origin.shiftIfNeeded(ac.pos);
      if (shift) origin.applyShift(shift);
      if (ac.aoaDeg > maxAoA) maxAoA = ac.aoaDeg;
      if (liftoffDist === null && !ac.onSurface && ac.pos.y > alpha.elev + 1) {
        // pos is in the shifted frame — add the cumulative shift back
        // to measure against the original-frame runway centre.
        liftoffDist = ac.pos.x + origin.cumulativeShift.x - alpha.x;
        liftoffT = t;
      }
      if (!ac.alive) break;
    }
    const dist = liftoffDist === null ? Infinity : liftoffDist;
    check(
      `${key}: survives a held full-pull takeoff`,
      ac.alive,
      ac.alive ? '' : `crashed (${ac.lastCrashReason ?? '-'})`,
    );
    check(
      `${key}: max AoA stays under 50 deg (no backflip / wallow)`,
      maxAoA <= 50,
      `maxAoA=${maxAoA.toFixed(1)} deg`,
    );
    check(
      `${key}: lifts off inside 650 m of the spawn (probe-bet criterion)`,
      liftoffDist !== null && dist < 650,
      liftoffDist === null
        ? 'never lifted off'
        : `liftoff at ${dist.toFixed(0)} m from centre, t=${liftoffT.toFixed(1)}s (flat plateau runs to 405 m)`,
    );
    check(
      `${key}: climbing at 15 s (no stall-fall back onto the runway)`,
      ac.alive && ac.pos.y > alpha.elev + 20 && ac.ias > 50,
      `y=${ac.pos.y.toFixed(0)} ias=${(ac.ias * 1.944).toFixed(0)} kt`,
    );
    origin.unregisterBody(ac.body);
    ac.dispose();
  }

  console.log(failures === 0 ? '\nALL HELD-PULL TAKEOFF CHECKS PASSED' : `\n${failures} HELD-PULL TAKEOFF CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
