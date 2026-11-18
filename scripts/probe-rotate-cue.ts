// probe-rotate-cue.ts — pass 6, item 6: the v4.1 live test reported
// "ROTATE — HOLD ↑ / PULL BACK" staying on screen while airborne at
// 100–250 kt in every flight. The ROTATE cue is shown when
// ac.rotateCue (alive && onSurface && ias >= Vr) — so it should clear
// the moment the jet leaves the surface. This probe drives a real
// takeoff (throttle 100%, full pull at Vr) and checks the cue is ON
// on the runway at/above Vr and OFF once airborne, for the F-22 and
// the F/A-18F. It also checks the takeoff does NOT invert the jet
// (item 10: the live F-22 veered right then pitched into an inverted
// climb, Bank 180°, heading 011° -> 197°).
//
// Usage: node_modules/.bin/esbuild scripts/probe-rotate-cue.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-rotate-cue.mjs
//   && node /tmp/probe-rotate-cue.mjs

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
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
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
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({
          x: 0,
          y: Math.sin(hdg / 2),
          z: 0,
          w: Math.cos(hdg / 2),
        })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(3, 0, 0);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  for (const key of ['f22', 'fa18'] as const) {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAt;
    // Charlie runway (the one the live test used): centre, heading 20.
    const rwy = RUNWAY_TABLE.find((r) => r.name === 'Charlie')!;
    ac.resetAt(rwy.x, rwy.elev + 2.2, rwy.z, rwy.hdg);
    // Release the brake, full throttle, roll straight down the runway.
    ac.brake = false;
    ac.thrTarget = 1;
    let cueOnAtVr = false;
    let cueOffAfterLiftoff = false;
    let liftedOffAt = -1;
    let maxBank = 0;
    let maxBankAt = '';
    let headingAtLiftoff = 0;
    let headingAtEnd = 0;
    let inverted = false;
    let t = 0;
    for (let i = 0; i < 40 * 60; i++) {
      // Full pull once at/above Vr (no roll input — the item-10
      // question is whether full pull ALONE inverts the jet).
      if (ac.ias >= spec.rotateSpeedKt * 0.514444) {
        ac.pitchIn = -1; // full pull (nose up)
      }
      ac.rollIn = 0;
      ac.yawIn = 0;
      world.timestep = 1 / 60;
      world.step(eventQueue);
      ac.step(1 / 60, wind, null);
      t = i / 60;
      // Bank from the quaternion (Euler YXZ z), as probe-roll does —
      // the Aircraft does not expose a `bank` field.
      const euler = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
      const bankDeg = Math.abs(euler.z * 180 / Math.PI);
      // Heading: respawn sets quat from Euler(0, +hdg, 0, 'YXZ'), so
      // the Euler Y IS the heading (a negated read here reported the
      // spawn heading 20° as 340°, a phantom 40° "veer").
      const headingDeg = ((euler.y * 180 / Math.PI) % 360 + 360) % 360;
      if (bankDeg > maxBank) {
        maxBank = bankDeg;
        maxBankAt = `t=${t.toFixed(1)}s ias=${(ac.ias * 1.944).toFixed(0)}kt`;
      }
      if (bankDeg > 90) inverted = true;
      if (!ac.alive) break;
      if (ac.onSurface && ac.ias >= spec.rotateSpeedKt * 0.514444 && ac.rotateCue) {
        cueOnAtVr = true;
      }
      if (liftedOffAt < 0 && !ac.onSurface && ac.ias > 10) {
        liftedOffAt = t;
        headingAtLiftoff = headingDeg;
      }
      if (liftedOffAt >= 0 && t > liftedOffAt + 1 && !ac.rotateCue) {
        cueOffAfterLiftoff = true;
      }
      if (liftedOffAt >= 0 && t > liftedOffAt + 5) {
        headingAtEnd = headingDeg;
        break;
      }
    }
    check(
      `${key}: ROTATE cue is ON on the runway at/above Vr`,
      cueOnAtVr,
      `Vr=${spec.rotateSpeedKt}kt`,
    );
    check(
      `${key}: ROTATE cue is OFF 1 s after liftoff`,
      cueOffAfterLiftoff,
      liftedOffAt >= 0
        ? `liftoff at t=${liftedOffAt.toFixed(1)}s`
        : 'never lifted off',
    );
    check(
      `${key}: full-pull takeoff does NOT invert the jet`,
      !inverted && maxBank < 60,
      `max bank=${maxBank.toFixed(1)}° at ${maxBankAt}; heading ${headingAtLiftoff.toFixed(0)}° -> ${headingAtEnd.toFixed(0)}°`,
    );
    ac.dispose();
  }

  console.log(failures === 0 ? '\nALL ROTATE-CUE CHECKS PASSED' : `\n${failures} ROTATE-CUE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
