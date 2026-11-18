// probe-live-spawn.ts — pass 6 diagnostic: replicate the LIVE startRound
// sequence exactly, including the phase the other probes skip: animate()
// stepping the player BEFORE its first spawn (startRound sets started=true
// before `await player.attachModel()`; the body is disabled at (0,0,0)
// and alive defaults true, so step() runs on an unspawned jet).
// Then spawn at each runway and run the live substep loop with variable
// frame dt (slow-device pattern) + shiftOriginIfNeeded, logging alive /
// pos / grounded / onSurface / vsi every 0.5 s and the exact death.
//
// Usage: node_modules/.bin/esbuild scripts/probe-live-spawn.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-live-spawn.mjs
//   && node /tmp/probe-live-spawn.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(
      type: string,
      init?: { lengthComputable?: boolean; loaded?: number; total?: number },
    ) {
      this.type = type;
      this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0;
      this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => {
          (listeners[type] ??= []).push(fn);
        },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', {
        set: () => {
          queueMicrotask(() => {
            for (const fn of listeners['load'] ?? []) fn();
          });
        },
      });
      return img;
    },
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  heightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(Number(process.env['WIND_MS'] ?? 3), 0, 0);

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
  const heightAtHf = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  // ---- terrain truth at the runways + the origin (pre-spawn position) ----
  console.log('--- terrain truth (exact heightfield / analytic) ---');
  console.log(
    `origin (0,0): hf=${heightAtHf(0, 0).toFixed(1)} analytic=${heightAt(0, 0).toFixed(1)}`,
  );
  for (const r of RUNWAY_TABLE) {
    let mn = Infinity;
    let mx = -Infinity;
    for (let dx = -400; dx <= 400; dx += 40) {
      for (let dz = -100; dz <= 100; dz += 20) {
        const h = heightAtHf(r.x + dx, r.z + dz);
        mn = Math.min(mn, h);
        mx = Math.max(mx, h);
      }
    }
    console.log(
      `${r.name} centre=(${r.x},${r.z}) elev=${r.elev} hdg=${r.hdg}: hf=${heightAtHf(r.x, r.z).toFixed(2)} analytic=${heightAt(r.x, r.z).toFixed(2)} footprint hf min=${mn.toFixed(1)} max=${mx.toFixed(1)} boxTop=${(r.elev + 0.05).toFixed(2)}`,
    );
  }

  // ---- world (same manual setup as the other probes) ----
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix]!;
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
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );

  // Variable frame dt, slow-device pattern (a phone / loaded tab):
  // 16 ms, 33 ms, 50 ms frames and occasional 100-250 ms hitches.
  // DT_MODE=fixed60 runs every frame at 1/60 (the old probe harness);
  // DT_MODE=slow runs every frame at 0.25 s (worst-case load jank).
  const dtMode = process.env['DT_MODE'] ?? 'var';
  const dtPattern =
    dtMode === 'fixed60'
      ? [1 / 60]
      : dtMode === 'slow'
        ? [0.25]
        : [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  const bankOf = (a: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(a.quat, 'YXZ').z * (180 / Math.PI);
  const pitchOf = (a: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(a.quat, 'YXZ').x * (180 / Math.PI);
  const drain = (player: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      const other = hit === a ? b : a;
      hit.notifyContact(hit.vel.y, other === 'deck');
    });
  };

  for (const r of RUNWAY_TABLE) {
    for (const key of ['fa18', 'f22', 'f35a', 'f15e', 'mig29', 'su34'] as const) {
      origin.reset();
      const spec = SPECS[key]!;
      const player = new Aircraft({
        scene,
        world,
        spec,
        isPlayer: true,
        modelKey: spec.modelKey,
      });
      player.terrainHeightAt = heightAtLive;
      origin.registerBody(player.body);
      for (const h of player.colliderHandles()) owners.set(h, player);
      await player.attachModel();

      // Phase A: pre-spawn — started=true, attachModel "loading":
      // 60 frames stepping the unspawned jet (body disabled at origin).
      let preSpawnCrash = false;
      let preSpawnReason = '';
      for (let f = 0; f < 60; f++) {
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          world.timestep = dt;
          world.step(eventQueue);
          drain(player);
          player.step(dt, WIND, null);
        }
        if (!player.alive && !preSpawnCrash) {
          preSpawnCrash = true;
          preSpawnReason = player.lastCrashReason ?? '-';
        }
      }
      // Phase B: spawn at the runway (startRound path) and run 30 s.
      player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
      let diedAt = -1;
      let deathDetail = '';
      let minY = Infinity;
      let contacts = 0;
      const origNotify = player.notifyContact.bind(player);
      // count contacts by wrapping: notifyContact is called by drain
      void origNotify;
      for (let f = 0; f < 1800; f++) {
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          world.timestep = dt;
          world.step(eventQueue);
          drain(player);
          player.step(dt, WIND, null);
        }
        const shift = origin.shiftIfNeeded(player.pos);
        if (shift) origin.applyShift(shift);
        minY = Math.min(minY, player.pos.y);
        if (!player.alive) {
          diedAt = f;
          deathDetail = `reason=${player.lastCrashReason ?? '-'} pos=(${player.pos.x.toFixed(0)},${player.pos.y.toFixed(1)},${player.pos.z.toFixed(0)}) hf=${heightAtLive(player.pos.x, player.pos.z).toFixed(1)} gnd=${player.grounded} vsi=${player.vsi.toFixed(1)}`;
          break;
        }
        if (f % 300 === 0) {
          console.log(
            `  [${r.name}/${key} t=${(f / 60).toFixed(0)}s] y=${player.pos.y.toFixed(2)} vsi=${player.vsi.toFixed(2)} gnd=${player.grounded} onSurf=${player.onSurface} ias=${(player.ias * 1.944).toFixed(0)} bank=${bankOf(player).toFixed(1)} pitch=${pitchOf(player).toFixed(1)}`,
          );
        }
      }
      void contacts;
      check(
        `${r.name}/${key}: pre-spawn phase clean + survives 30 s parked (variable dt)`,
        !preSpawnCrash && diedAt === -1,
        `preSpawnCrash=${preSpawnCrash}(${preSpawnReason}) diedAt=${diedAt >= 0 ? (diedAt / 60).toFixed(1) + 's ' + deathDetail : 'n/a'} minY=${minY.toFixed(1)}`,
      );
      origin.unregisterBody(player.body);
      player.dispose();
    }
  }

  console.log(failures === 0 ? '\nALL LIVE-SPAWN CHECKS PASSED' : `\n${failures} LIVE-SPAWN CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
