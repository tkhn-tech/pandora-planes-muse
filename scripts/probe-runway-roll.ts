// probe-runway-roll.ts — pass 9, item 1: conventional jets will NOT roll
// at 100% throttle live (F-18 4+ min on Bravo, 0 kt, brake off, FUEL
// UNCHANGED; F-22 30 s, 0 kt). In v4.2 the F-22 DID roll and take off
// from Charlie, so this is a v4.3 regression. Reproduce through the LIVE
// input merge, not the direct-thrTarget driver path that probe-takeoff
// uses: the on-screen slider sets ac.thrTarget AND pcInput.setThrottle,
// then every substep the playerInput merge runs (getControls() ->
// throttle null keeps thrTarget) before ac.step(). Spawn via the
// respawnPlayer path (resetAt with the origin shift subtracted), with
// the live variable-dt pattern and the floating-origin shift. A jet at
// 100% throttle, brake off, must reach rotation speed and lift off
// inside the runway. Logs thrTarget / throttle / fuel / ias / vel /
// onSurface every 2 s so a lost throttle shows up as fuel-not-burning.
//
// Usage: node_modules/.bin/esbuild scripts/probe-runway-roll.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-runway-roll.mjs
//   && node /tmp/probe-runway-roll.mjs
//   DT_MODE=fixed60 node /tmp/probe-runway-roll.mjs

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
// PCFlightInput needs window + navigator (keyboard/gamepad listeners).
if (typeof (globalThis as unknown as { window?: unknown }).window === 'undefined') {
  (globalThis as unknown as { window?: unknown }).window = {
    addEventListener: () => {},
    removeEventListener: () => {},
  };
}
if (typeof (globalThis as unknown as { navigator?: unknown }).navigator === 'undefined') {
  (globalThis as unknown as { navigator?: unknown }).navigator = {
    getGamepads: () => [],
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { PCFlightInput } from '../src/input/PCFlightInput';
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
  const dtMode = process.env['DT_MODE'] ?? 'var';
  const dtPattern =
    dtMode === 'fixed60'
      ? [1 / 60]
      : [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  console.log(`--- runway roll via the LIVE slider merge (dt=${dtMode}) ---`);

  // Fake canvas for PCFlightInput (constructor only adds listeners).
  const fakeCanvas = {
    addEventListener: () => {},
    removeEventListener: () => {},
  } as unknown as HTMLCanvasElement;

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
      const pcInput = new PCFlightInput(fakeCanvas);

      // Spawn via the respawnPlayer path (original-frame runway coords
      // minus the current origin shift, gear feet on the box top).
      const shift = origin.cumulativeShift;
      player.resetAt(r.x - shift.x, r.elev + 2.25, r.z - shift.z, r.hdg);
      player.brake = false;
      // Settle 1 s parked (the tester sits, then moves the slider).
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
        const sh = origin.shiftIfNeeded(player.pos);
        if (sh) origin.applyShift(sh);
      }
      // The on-screen slider: sets thrTarget AND pcInput.setThrottle.
      player.thrTarget = 1;
      pcInput.setThrottle(1);
      const fuel0 = player.fuel;
      let t60kt = -1;
      let tOff = -1;
      let minThrTarget = 1;
      let maxThr = 0;
      let diedAt = -1;
      let deathDetail = '';
      const startY = player.pos.y;
      // Run 45 s: per substep, the playerInput throttle merge, then step.
      // The run ends 3 s after liftoff — the roll and liftoff are the
      // checks; a level climb-out after that would hit the ranges.
      let endAt = -1;
      for (let f = 0; f < 45 * 60; f++) {
        const rawDt = dtPattern[f % dtPattern.length]!;
        const sub = Math.max(1, Math.ceil(rawDt / 0.05));
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          // Live playerInput merge (main.ts ~1835): only overwrite
          // thrTarget when the PC input reports a non-null throttle.
          const pc = pcInput.getControls();
          if (pc.throttle != null) player.thrTarget = pc.throttle;
          world.timestep = dt;
          world.step(eventQueue);
          drain(player);
          player.step(dt, WIND, null);
          minThrTarget = Math.min(minThrTarget, player.thrTarget);
          maxThr = Math.max(maxThr, player.throttle);
        }
        const sh = origin.shiftIfNeeded(player.pos);
        if (sh) origin.applyShift(sh);
        // Rotation pull at Vr (55 m/s), then a gentle climb so the
        // climb-out clears the ranges; release once well above ground.
        if (player.ias > 55 && player.onSurface) player.pitchIn = -1;
        if (!player.onSurface && player.pos.y < startY + 200) player.pitchIn = -0.3;
        if (!player.onSurface && player.pos.y >= startY + 200) player.pitchIn = 0;
        const t = f / 60;
        if (t60kt < 0 && player.ias * 1.944 >= 60) t60kt = t;
        if (tOff < 0 && !player.onSurface && player.pos.y > startY + 10) {
          tOff = t;
          endAt = t + 3;
        }
        if (endAt >= 0 && t >= endAt) break;
        if (!player.alive) {
          diedAt = t;
          deathDetail = `reason=${player.lastCrashReason ?? '-'} pos=(${player.pos.x.toFixed(0)},${player.pos.y.toFixed(1)},${player.pos.z.toFixed(0)})`;
          break;
        }
        if (f % 120 === 0) {
          console.log(
            `  [${r.name}/${key} t=${t.toFixed(0)}s] thrTarget=${player.thrTarget.toFixed(2)} thr=${player.throttle.toFixed(2)} fuel=${player.fuel.toFixed(1)} ias=${(player.ias * 1.944).toFixed(0)}kt vel=(${player.vel.x.toFixed(1)},${player.vel.y.toFixed(1)},${player.vel.z.toFixed(1)}) y=${player.pos.y.toFixed(1)} onSurf=${player.onSurface} alive=${player.alive}`,
          );
        }
      }
      const fuelDrop = fuel0 - player.fuel;
      check(
        `${r.name}/${key}: rolls — reaches 60 kt in 20 s at 100% throttle, brake off`,
        diedAt < 0 && t60kt >= 0 && t60kt <= 20,
        `t60kt=${t60kt < 0 ? 'never' : t60kt.toFixed(1) + 's'} diedAt=${diedAt >= 0 ? diedAt.toFixed(1) + 's ' + deathDetail : 'n/a'} minThrTarget=${minThrTarget.toFixed(2)} maxThr=${maxThr.toFixed(2)} fuelDrop=${fuelDrop.toFixed(2)}`,
      );
      check(
        `${r.name}/${key}: lifts off inside the runway (45 s, pull at Vr)`,
        diedAt < 0 && tOff >= 0 && tOff <= 45,
        `tOff=${tOff < 0 ? 'never' : tOff.toFixed(1) + 's'} diedAt=${diedAt >= 0 ? diedAt.toFixed(1) + 's ' + deathDetail : 'n/a'}`,
      );
      // Fuel is a 0..1 fraction; at full throttle it burns ~0.0012/s
      // (0.008 + 0.11 per s, /100) — ~0.007 over the ~6 s roll+climb
      // to liftoff+3 s — vs ~0.0005 at idle, so >0.004 proves the
      // throttle is actually at 100% and thrust reaches the engine.
      check(
        `${r.name}/${key}: fuel burns at 100% throttle (thrust reaches the engine)`,
        fuelDrop > 0.004,
        `fuel ${fuel0.toFixed(2)} -> ${player.fuel.toFixed(2)} (drop ${fuelDrop.toFixed(3)})`,
      );
      origin.unregisterBody(player.body);
      for (const h of player.colliderHandles()) owners.delete(h);
      player.dispose();
    }
  }
  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
