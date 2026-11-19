// probe-dying-lowspeed.ts — pass 9, item 5: the F-35A was shot down
// LIVE at low speed (22 kt) and high AoA (60°) at 340 ft, and the
// wreck HUNG frozen in mid-air for 30+ s (HUD frozen at 22 kt /
// 340 ft / -3132 fpm / AoA 60°, altitude never changing, no fire, no
// impact, no explosion). probe-going-down only covers the dying fall
// from 1000 m at 150 kt, fixed dt=1/60 — a fast, level shoot-down.
// This probe reproduces the live condition: F-35A at 340 ft AGL over
// Alpha, 22 kt, pitched up to AoA ~60 deg, shot down (startDying),
// stepped with the LIVE variable-dt pattern (0.016–0.25 s) and wind
// 3 m/s. The wreck must fall and impact the ground (crashedFromDying)
// within 15 s, and its altitude must decrease steadily — no 30 s hang.
//
// Usage: node_modules/.bin/esbuild scripts/probe-dying-lowspeed.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-dying-lowspeed.mjs
//   && node /tmp/probe-dying-lowspeed.mjs

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

const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
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
  const alpha = RUNWAY_TABLE[0]!;

  for (const key of ['f35a', 'fa18', 'f22'] as const) {
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
    // 340 ft AGL over Alpha (runway elev 50 m), 22 kt, pitched up to
    // AoA ~60 deg: nose 60 deg above the horizon, moving horizontally
    // at 11 m/s, so the body-frame airflow comes from below-front and
    // atan2(-vBody.y, vBody.z) ~= 60 deg.
    const groundY = heightAt(alpha.x, alpha.z);
    ac.resetAt(alpha.x, groundY + 104, alpha.z, alpha.hdg);
    const pitchUp = THREE.MathUtils.degToRad(60);
    const yaw = THREE.MathUtils.degToRad(alpha.hdg);
    const q = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(-pitchUp, yaw, 0, 'YXZ'),
    );
    ac.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    ac.body.setLinvel({ x: 11, y: 0, z: 0 }, true);
    ac.vel.set(11, 0, 0);
    ac.thrTarget = 0.5;
    // One step to let the solver read the attitude and compute AoA.
    world.timestep = 1 / 60;
    world.step(eventQueue);
    drain();
    ac.step(1 / 60, WIND, null);
    const aoa0 = ac.aoaDeg;
    check(
      `${key}: setup reaches the live AoA (~60 deg at 22 kt)`,
      Math.abs(aoa0 - 60) < 15,
      `aoa=${aoa0.toFixed(1)} deg ias=${(ac.ias * 1.944).toFixed(0)} kt y=${ac.pos.y.toFixed(1)}`,
    );
    // Shot down, exactly as the live kill (damage -> startDying).
    ac.startDying('shot down');
    const y0 = ac.pos.y;
    let impactedAt = -1;
    let minY = y0;
    let maxStall = 0;
    let t = 0;
    let pi = 0;
    while (t < 15) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) {
        world.timestep = dt;
        world.step(eventQueue);
        drain();
        ac.step(dt, WIND, null);
      }
      t += rawDt;
      minY = Math.min(minY, ac.pos.y);
      maxStall = Math.max(maxStall, Math.abs(ac.pos.y - y0));
      if (!ac.alive) {
        impactedAt = t;
        break;
      }
    }
    check(
      `${key}: low-speed high-AoA wreck falls and impacts within 15 s (no 30 s hang)`,
      impactedAt >= 0,
      impactedAt >= 0
        ? `impact at t=${impactedAt.toFixed(1)}s y=${ac.pos.y.toFixed(1)} crashedFromDying=${ac.crashedFromDying}`
        : `still airborne at t=15s y=${ac.pos.y.toFixed(1)} (started ${y0.toFixed(1)}, min ${minY.toFixed(1)}) vsi=${ac.vsi.toFixed(1)} ias=${(ac.ias * 1.944).toFixed(0)} kt`,
    );
    if (impactedAt >= 0) {
      check(
        `${key}: impact is a dying crash (wreckage path)`,
        ac.crashedFromDying === true,
        `crashedFromDying=${ac.crashedFromDying} reason=${ac.lastCrashReason ?? '-'}`,
      );
    }
    origin.unregisterBody(ac.body);
    ac.dispose();
  }

  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
