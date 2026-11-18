// probe-runway-sink.ts — reproduce the v4 play-test runway sink
// (pass 5 expansion, item 1).
//
// Play-test (18 Nov, 30 min, footer v4.0): Alpha AND Bravo spawns
// are broken for ALL aircraft, 0 enemies, no input — the jet slowly
// sinks at ~-88 fpm THROUGH the runway, then registers as crashed
// within ~10-27 s. Charlie works. This probe parks each jet on each
// runway with no input for 30 s, at dt=1/60 and at dt=0.05 (slow
// phone), and logs y / onSurface / grounded / heightfield height so
// the spawn/terrain mismatch shows up directly.
//
// Usage: node_modules/.bin/esbuild scripts/probe-runway-sink.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-runway-sink.mjs
//   && node /tmp/probe-runway-sink.mjs

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
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  const scene = new THREE.Scene();
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
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

  // Heightfield elevation at each runway centre (original frame) —
  // the plateau flat top should equal the runway elev.
  for (const r of RUNWAY_TABLE) {
    const hf = heightfieldHeightAt(heights, res, r.x, r.z);
    console.log(
      `${r.name}: runway elev=${r.elev} heightfield at centre=${hf.toFixed(2)} (diff ${(hf - r.elev).toFixed(2)} m) box top=${(r.elev + 0.05).toFixed(2)}`,
    );
  }

  for (const dt of [1 / 60, 0.05]) {
    for (const r of RUNWAY_TABLE) {
      origin.reset();
      const spec = SPECS['fa18']!;
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
      ac.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
      const y0 = ac.pos.y;
      let crashedAt = -1;
      let reason = '';
      const steps = Math.round(30 / dt);
      for (let i = 0; i < steps; i++) {
        const t = i * dt;
        world.timestep = dt;
        world.step(eventQueue);
        drain();
        ac.step(dt, WIND, null);
        const shift = origin.shiftIfNeeded(ac.pos);
        if (shift) origin.applyShift(shift);
        if (i % Math.round(5 / dt) === 0) {
          console.log(
            `  [dt=${dt.toFixed(3)} ${r.name} t=${t.toFixed(0)}s] y=${ac.pos.y.toFixed(2)} hf=${heightAt(ac.pos.x, ac.pos.z).toFixed(2)} onSurf=${ac.onSurface} gnd=${ac.grounded} alive=${ac.alive}`,
          );
        }
        if (!ac.alive) {
          crashedAt = i;
          reason = ac.lastCrashReason ?? '-';
          break;
        }
      }
      const sank = y0 - ac.pos.y;
      // Rest height on the box top is elev + 2.25 (gear feet on the
      // box top, elev + 0.05); the jet must hold within 0.5 m of it
      // for 30 s parked, not sink through the runway (live: -88 fpm,
      // crash in 10-27 s on Alpha/Bravo).
      const restY = r.elev + 2.25;
      check(
        `dt=${dt.toFixed(3)} ${r.name}: parked jet survives 30 s (no sink-through crash)`,
        ac.alive,
        ac.alive ? '' : `crashed at t=${(crashedAt * dt).toFixed(1)}s reason=${reason}`,
      );
      check(
        `dt=${dt.toFixed(3)} ${r.name}: holds rest height (no slow sink)`,
        Math.abs(ac.pos.y - restY) < 0.5,
        `y ${y0.toFixed(2)} -> ${ac.pos.y.toFixed(2)} (rest ${restY.toFixed(2)}, sank ${sank.toFixed(2)} m)`,
      );
      origin.unregisterBody(ac.body);
      ac.dispose();
    }
  }

  console.log(failures === 0 ? '\nALL RUNWAY-SINK CHECKS PASSED' : `\n${failures} RUNWAY-SINK CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
