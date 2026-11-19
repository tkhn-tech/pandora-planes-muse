// probe-spawn-firstframes.ts — pass 7 diagnostic: instrument the FIRST
// FRAMES after a live-path runway spawn, per runway x aircraft, with the
// LIVE default wind (windDir 270 -> windVec (-3,0,0)) and the live variable
// frame-dt pattern. Logs aoaDeg / vsi / vel / pos.y / onSurface / body
// linvel after world.step (before the controller step) so a Rapier-side
// drop or a controller-side drop can be told apart.
//
// Usage: node_modules/.bin/esbuild scripts/probe-spawn-firstframes.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-spawn-firstframes.mjs
//   && node /tmp/probe-spawn-firstframes.mjs

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

// LIVE default wind: windDir 270, windSpeed 3 -> windVec (-3, 0, 0).
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

  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  const bankOf = (a: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(a.quat, 'YXZ').z * (180 / Math.PI);

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

      const contacts: string[] = [];
      const drain = (): void => {
        eventQueue.drainCollisionEvents((h1, h2, started) => {
          if (!started) return;
          const a = owners.get(h1);
          const b = owners.get(h2);
          const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
          if (!hit || !hit.alive) return;
          const other = hit === a ? b : a;
          contacts.push(`vy=${hit.vel.y.toFixed(2)} other=${other === 'deck' ? 'deck' : 'terrain'}`);
          hit.notifyContact(hit.vel.y, other === 'deck');
        });
      };

      player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
      console.log(
        `--- ${r.name}/${key} spawn y=${player.pos.y.toFixed(2)} hf=${heightAtHf(r.x, r.z).toFixed(2)} boxTop=${(r.elev + 0.05).toFixed(2)} ---`,
      );
      let diedAt = -1;
      let maxAoA = -Infinity;
      let minVsi = Infinity;
      for (let f = 0; f < 600; f++) {
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          world.timestep = dt;
          world.step(eventQueue);
          const bt = player.body.translation();
          const bl = player.body.linvel();
          drain();
          player.step(dt, WIND, null);
          if (f < 12) {
            console.log(
              `  f${f}s${s} dt=${dt.toFixed(3)} bodyAfterStep y=${bt.y.toFixed(3)} linvel=(${bl.x.toFixed(2)},${bl.y.toFixed(2)},${bl.z.toFixed(2)}) | afterCtrl y=${player.pos.y.toFixed(3)} vel=(${player.vel.x.toFixed(2)},${player.vel.y.toFixed(2)},${player.vel.z.toFixed(2)}) aoa=${player.aoaDeg.toFixed(1)} vsi=${player.vsi.toFixed(2)} onSurf=${player.onSurface} gnd=${player.grounded} stalled=${player.stalled} alive=${player.alive} contacts=${contacts.length}`,
            );
          }
          maxAoA = Math.max(maxAoA, player.aoaDeg);
          minVsi = Math.min(minVsi, player.vsi);
          if (!player.alive && diedAt < 0) {
            diedAt = f;
            console.log(
              `  DIED f=${f} reason=${player.lastCrashReason ?? '-'} y=${player.pos.y.toFixed(2)} hf=${heightAtLive(player.pos.x, player.pos.z).toFixed(2)} aoa=${player.aoaDeg.toFixed(1)} vsi=${player.vsi.toFixed(2)} contacts=[${contacts.join(' | ')}]`,
            );
          }
        }
        if (!player.alive) break;
        if (f % 120 === 0 && f > 0) {
          console.log(
            `  t=${(f / 60).toFixed(0)}s y=${player.pos.y.toFixed(2)} x=${player.pos.x.toFixed(1)} z=${player.pos.z.toFixed(1)} bank=${bankOf(player).toFixed(1)} ias=${(player.ias * 1.944).toFixed(0)}`,
          );
        }
      }
      console.log(
        `  => ${r.name}/${key}: ${diedAt >= 0 ? 'DIED f=' + diedAt : 'ALIVE 10s'} maxAoA=${maxAoA.toFixed(1)} minVsi=${minVsi.toFixed(2)} finalY=${player.pos.y.toFixed(2)} finalX=${player.pos.x.toFixed(1)} finalZ=${player.pos.z.toFixed(1)}`,
      );
      origin.unregisterBody(player.body);
      player.dispose();
    }
  }
  console.log('DONE');
}

main();
