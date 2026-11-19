// probe-respawn-shift.ts — pass 9, item 2: Alpha spawn death (instant,
// vs=0.00, bank=0.0, jet sitting on the runway; 4 of 7 Alpha spawns
// died, 3 survived) and the Charlie airborne spawn (69 m above the
// sampled floor) + Bravo dead-end spawn.
//
// Root-cause hypothesis under test: startRound's runway spawn passes
// ORIGINAL-frame runway coords straight to resetAt() (scene coords)
// WITHOUT subtracting the floating-origin shift, while respawnPlayer
// DOES subtract it. On round 1 (shift 0) both agree; on round 2+
// (shift != 0, after the player has flown > 2,000 m) startRound
// spawns the jet at original position r0 + shift — up to kilometres
// from the runway, buried in high terrain — where the position-based
// safety net (pos.y < terrainH - 0.5) kills it instantly with vs=0
// and bank=0. The intermittency is the shift: whether r0 + shift
// lands on low or high terrain. The carrier reset() has the same
// shape of bug (see probe-carrier-reset.ts).
//
// The probe simulates a previous round (spawn at Alpha, fly to a far
// original position P so cumulativeShift == P), then runs the
// startRound path (NO shift subtraction) and the respawnPlayer path
// (WITH shift subtraction) for each runway x jet, and checks the jet
// lands ON its runway (alive, onSurface, y ~= elev + 2.25, floor ~=
// elev, original-frame position inside the strip).
//
// Usage: node_modules/.bin/esbuild scripts/probe-respawn-shift.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-respawn-shift.mjs
//   && node /tmp/probe-respawn-shift.mjs

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
// Plausible death positions (original frame) from a previous round —
// the cumulative shift after the player flew there and died.
const DEATH_POS: Array<[number, number]> = [
  [2000, 1500],
  [4000, 3000],
  [-1000, -3000],
  [5000, 0],
  [0, -5000],
  [3000, 3000],
];

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
  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  // Step the live substep loop for `seconds`, applying the frame-start
  // origin shift exactly as animate() does (pass 8).
  const stepFor = (player: Aircraft, seconds: number): void => {
    const frames = Math.round(seconds * 60);
    for (let f = 0; f < frames; f++) {
      const rawDt = dtPattern[f % dtPattern.length]!;
      const sub = Math.max(1, Math.ceil(rawDt / 0.05));
      const dt = rawDt / sub;
      // Frame-start shift (pass 8): player.pos is fresh from the
      // previous frame's last step.
      const sh = origin.shiftIfNeeded(player.pos);
      if (sh) origin.applyShift(sh);
      for (let s = 0; s < sub; s++) {
        world.timestep = dt;
        world.step(eventQueue);
        drain(player);
        player.step(dt, WIND, null);
      }
    }
  };

  console.log('--- startRound path (NO shift subtraction) vs respawnPlayer path (WITH) ---');
  for (const [px, pz] of DEATH_POS) {
    for (const key of ['fa18', 'f22', 'f35a'] as const) {
      for (const mode of ['startRound', 'respawnPlayer'] as const) {
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
        // Previous round: spawn at Alpha, then fly to (px, pz) and die
        // there, so cumulativeShift == (px, pz).
        const r0 = RUNWAY_TABLE[0]!;
        player.resetAt(r0.x, r0.elev + 2.25, r0.z, r0.hdg);
        stepFor(player, 0.5);
        // Teleport to the death position (scene = original - shift).
        const sh0 = origin.cumulativeShift;
        player.resetAt(px - sh0.x, 400, pz - sh0.z, 90);
        stepFor(player, 0.5);
        const shiftAtDeath = origin.cumulativeShift.clone();
        // Now respawn at Alpha via the path under test.
        if (mode === 'startRound') {
          // main.ts startRound: resetOrigin() FIRST (line ~1218) — the
          // shift is zeroed and everything moves back to the original
          // frame — THEN the runway spawn uses the original coords
          // straight in (line ~1329). The probe must include the reset:
          // omitting it makes the spawn wrong by the whole shift.
          origin.reset();
          player.resetAt(r0.x, r0.elev + 2.25, r0.z, r0.hdg);
        } else {
          // main.ts respawnPlayer: original minus the shift.
          const sh = origin.cumulativeShift;
          player.resetAt(r0.x - sh.x, r0.elev + 2.25, r0.z - sh.z, r0.hdg);
        }
        player.thrTarget = 0;
        stepFor(player, 5);
        const floorY = heightAtLive(player.pos.x, player.pos.z);
        const origX = player.pos.x + origin.cumulativeShift.x;
        const origZ = player.pos.z + origin.cumulativeShift.z;
        const onRunway =
          Math.abs(origX - r0.x) < r0.len / 2 + 100 &&
          Math.abs(origZ - r0.z) < r0.width / 2 + 2;
        const ok =
          player.alive &&
          player.onSurface &&
          Math.abs(player.pos.y - (r0.elev + 2.25)) < 1.5 &&
          Math.abs(floorY - r0.elev) < 1.5;
        check(
          `${mode}/Alpha/${key} death@(${px},${pz}) shift=(${shiftAtDeath.x.toFixed(0)},${shiftAtDeath.z.toFixed(0)}): survives 5 s ON the runway`,
          ok,
          `alive=${player.alive} reason=${player.lastCrashReason ?? '-'} y=${player.pos.y.toFixed(1)} floor=${floorY.toFixed(1)} orig=(${origX.toFixed(0)},${origZ.toFixed(0)}) onRunway=${onRunway} onSurf=${player.onSurface} gnd=${player.grounded}`,
        );
        origin.unregisterBody(player.body);
        for (const h of player.colliderHandles()) owners.delete(h);
        player.dispose();
      }
    }
  }
  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
