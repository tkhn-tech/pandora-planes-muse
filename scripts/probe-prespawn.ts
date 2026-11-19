// probe-prespawn.ts — pass 8, item 1: the PRE-SPAWN crash.
//
// A fresh Aircraft is born alive=true with its Rapier body DISABLED at
// (0,0,0) ("respawn() enables + places the aircraft"). In main.ts,
// startRound sets started=true BEFORE the player is created, and then
// awaits player.attachModel() — during that await the animate loop runs
// with started=true and steps the unplaced player. At (0,0,0) the
// terrain is ~40 m up, so the controller's terrain safety net fires
// crash('terrain') on the first frame — BEFORE resetAt ever places the
// jet on Alpha. resetAt then revives it (alive=true), but handleDeaths
// already fired: the "WENT DOWN — terrain" banner + a 3 s respawn
// timer that pops the picker over a live, intact jet sitting at 0 kt.
// That is the live "Alpha/Bravo spawn kills the jet in 1-3 s" — the
// jet never died post-spawn; it died pre-spawn at the world origin.
// It repeats on every jet change (a new Aircraft per jet) and on
// every fresh load, which is why Alpha/Bravo "die" and Charlie
// (reached via a picker respawn after a REAL death) "works".
//
// Cases:
//   A. Step a fresh Aircraft 30 frames WITHOUT resetAt (the live
//      attachModel await): must NOT crash — the fix (stepInternal
//      early-returns while the body is disabled) makes this a no-op.
//   B. No steps before resetAt, then Alpha 10 s: alive (the fixed flow).
//   C. Jet-change restart: a placed player, then a NEW Aircraft stepped
//      30 frames without resetAt: must NOT crash either.
//
// Usage: node_modules/.bin/esbuild scripts/probe-prespawn.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-prespawn.mjs
//   && node /tmp/probe-prespawn.mjs

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
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

// Live default wind: windDir 270, windSpeed 3 -> (-3, 0, 0).
const WIND = new THREE.Vector3(-3, 0, 0);

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  console.log(`  terrain at world origin (0,0): ${heightAt(0, 0).toFixed(1)} m`);

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
  const drain = (): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const stepWorld = (dt: number): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain();
  };
  const mkPlayer = (key: 'fa18' | 'f22' | 'f35a'): Aircraft => {
    const p = new Aircraft({
      scene,
      world,
      spec: SPECS[key]!,
      isPlayer: true,
      modelKey: SPECS[key]!.modelKey,
    });
    p.terrainHeightAt = heightAt;
    for (const h of p.colliderHandles()) owners.set(h, p);
    return p;
  };

  // ---- A. Fresh aircraft stepped 30 frames before any resetAt ----
  {
    const p = mkPlayer('fa18');
    let crashedAt = -1;
    for (let f = 0; f < 30; f++) {
      stepWorld(1 / 60);
      p.step(1 / 60, WIND, null);
      if (!p.alive && crashedAt < 0) crashedAt = f;
    }
    check(
      'A: fresh aircraft stepped before resetAt does NOT crash',
      p.alive,
      p.alive
        ? 'alive after 30 unplaced frames (body disabled, step is a no-op)'
        : `CRASHED at frame ${crashedAt}: reason=${p.lastCrashReason ?? '-'}`,
    );
    p.dispose();
  }

  // ---- B. Fixed flow: no steps before resetAt, Alpha 10 s ----
  {
    const p = mkPlayer('f22');
    const alpha = RUNWAY_TABLE[0]!;
    p.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    let diedAt = -1;
    for (let f = 0; f < 600; f++) {
      stepWorld(1 / 60);
      p.step(1 / 60, WIND, null);
      if (!p.alive && diedAt < 0) diedAt = f;
    }
    check(
      'B: Alpha spawn (no pre-spawn steps) survives 10 s',
      p.alive,
      p.alive ? `y=${p.pos.y.toFixed(2)}` : `died frame ${diedAt}: ${p.lastCrashReason ?? '-'}`,
    );
    p.dispose();
  }

  // ---- C. Jet-change restart: placed player, then a NEW aircraft
  //      stepped before its resetAt ----
  {
    const old = mkPlayer('fa18');
    const alpha = RUNWAY_TABLE[0]!;
    old.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let f = 0; f < 300; f++) {
      stepWorld(1 / 60);
      old.step(1 / 60, WIND, null);
    }
    const fresh = mkPlayer('f22');
    let crashedAt = -1;
    for (let f = 0; f < 30; f++) {
      stepWorld(1 / 60);
      old.step(1 / 60, WIND, null);
      fresh.step(1 / 60, WIND, null);
      if (!fresh.alive && crashedAt < 0) crashedAt = f;
    }
    check(
      'C: new aircraft (jet change) stepped before resetAt does NOT crash',
      fresh.alive,
      fresh.alive
        ? 'alive after 30 unplaced frames'
        : `CRASHED at frame ${crashedAt}: reason=${fresh.lastCrashReason ?? '-'}`,
    );
    check(
      'C: old placed player still alive during the swap',
      old.alive,
      old.alive ? '' : `old player died: ${old.lastCrashReason ?? '-'}`,
    );
    old.dispose();
    fresh.dispose();
  }

  console.log(failures === 0 ? 'ALL PRE-SPAWN CHECKS PASSED' : `${failures} PRE-SPAWN CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
