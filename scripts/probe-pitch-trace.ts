// probe-pitch-trace.ts — pass 7 diagnostic: trace the hands-off pitch-up
// for Charlie/f22 at 10 Hz with the APPLIED body-frame angular velocity
// (angVelBody), AoA, pitch, bank, VSI, IAS, so the pitch-up's source can
// be identified (solver rate vs controller clamp vs something else).
//
// Usage: node_modules/.bin/esbuild scripts/probe-pitch-trace.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-pitch-trace.mjs
//   && node /tmp/probe-pitch-trace.mjs

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

const WIND = new THREE.Vector3(-3, 0, 0);

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
  const owners = new Map<number, Aircraft | 'terrain'>();
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
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

  const r = RUNWAY_TABLE[2]!; // Charlie
  const spec = SPECS['f22']!;
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
  player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
  player.thrTarget = 1;
  console.log('t pitch bank hdg aoa vsi ias y agl wPitch wRoll wYaw thr gnd onSurf');
  for (let f = 0; f < 3600; f++) {
    const t = f / 60;
    const dt = 1 / 60;
    world.timestep = dt;
    world.step(eventQueue);
    drain(player);
    player.step(dt, WIND, null);
    const shift = origin.shiftIfNeeded(player.pos);
    if (shift) origin.applyShift(shift);
    if (f % 6 === 0) {
      const e = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ');
      const av = player.angVelBody;
      console.log(
        `${t.toFixed(1)} ${(e.x * 57.3).toFixed(1)} ${(e.z * 57.3).toFixed(1)} ${(((e.y * 57.3) % 360) + 360) % 360 | 0} ${player.aoaDeg.toFixed(1)} ${(player.vsi * 196.85).toFixed(0)} ${(player.ias * 1.944).toFixed(0)} ${player.pos.y.toFixed(0)} ${(player.pos.y - heightAtLive(player.pos.x, player.pos.z)).toFixed(0)} ${(av.x * 57.3).toFixed(2)} ${(av.z * 57.3).toFixed(2)} ${(av.y * 57.3).toFixed(2)} ${player.throttle.toFixed(2)} ${player.grounded} ${player.onSurface}`,
      );
    }
    if (!player.alive) {
      console.log(`DIED t=${t.toFixed(1)} reason=${player.lastCrashReason}`);
      break;
    }
  }
  console.log('DONE');
}

main();
