// probe-spawn-shift.ts — pass 8, item 1, suspect (c): the FIRST-FRAME
// origin shift.
//
// probe-spawn-firstframes shows the spawn stable in the isolated
// physics path — but it NEVER fires the floating-origin shift. Live,
// the player spawns at Alpha (-2800,-1800), 3340 m from the origin,
// so worldGen.shiftOriginIfNeeded fires at the END of frame 1 and
// teleports the terrain body, the player body, the carrier and every
// registered vector by +2800/+1800. This probe runs the exact live
// sequence (resetAt, then per-frame substeps, then the shift check at
// the end of each frame) for every runway x aircraft, 10 s each, and
// reports any death with the numbers.
//
// Usage: node_modules/.bin/esbuild scripts/probe-spawn-shift.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-spawn-shift.mjs
//   && node /tmp/probe-spawn-shift.mjs

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
  const heightAtHf = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

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
  // Live heightAt: scene coords + cumulative shift = original frame.
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];

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

      player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
      let diedAt = -1;
      let diedReason = '';
      let diedDetail = '';
      let shiftFiredAt = -1;
      let minClearance = Infinity;
      for (let f = 0; f < 600; f++) {
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          world.timestep = dt;
          world.step(eventQueue);
          drain();
          player.step(dt, WIND, null);
          if (player.alive) {
            // Measure clearance HERE (fresh pos, synced from the body
            // at the start of step) — NOT after the shift below, where
            // player.pos is stale (pre-shift) until the next step.
            minClearance = Math.min(
              minClearance,
              player.pos.y - heightAtLive(player.pos.x, player.pos.z),
            );
          }
          if (!player.alive && diedAt < 0) {
            diedAt = f;
            diedReason = player.lastCrashReason ?? '-';
            diedDetail = `y=${player.pos.y.toFixed(2)} hf=${heightAtLive(player.pos.x, player.pos.z).toFixed(2)} aoa=${player.aoaDeg.toFixed(1)} vsi=${player.vsi.toFixed(2)} onSurf=${player.onSurface} gnd=${player.grounded}`;
          }
        }
        // End of frame, exactly like animate(): shift the origin if
        // the player is past the 2 km threshold.
        const shift = origin.shiftIfNeeded(player.pos);
        if (shift) {
          if (shiftFiredAt < 0) shiftFiredAt = f;
          origin.applyShift(shift);
        }
        if (!player.alive) break;
      }
      check(
        `${r.name}/${key}: spawn survives 10 s with the live frame-1 origin shift`,
        diedAt < 0,
        diedAt >= 0
          ? `DIED f=${diedAt} reason=${diedReason} shiftFiredAt f=${shiftFiredAt} — ${diedDetail}`
          : `shift fired at frame ${shiftFiredAt}, min clearance above hf ${minClearance.toFixed(2)} m`,
      );
      origin.unregisterBody(player.body);
      player.dispose();
    }
  }
  console.log(failures === 0 ? 'ALL SPAWN-SHIFT CHECKS PASSED' : `${failures} SPAWN-SHIFT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
