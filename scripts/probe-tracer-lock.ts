// probe-tracer-lock.ts — pass 9, item 9: the v4.3 live kill happened
// during a 20 s observation gap and no tracers or lock indicator were
// seen. Parked jets are ignored BY DESIGN (v1.1 rule), so this does
// not test that — it verifies the two things the player SHOULD see
// when an airborne kill happens: (1) an enemy gun pass draws a
// visible tracer line in the scene (WeaponSystem.aiGunShot ->
// showTracer, a pooled THREE.Line, 0.09 s life, at most 400 m), and
// (2) the player's missile lock builds while an enemy is held inside
// the 15 deg cone under 2500 m (main.ts lock scan: lockT += dt,
// lockTarget set; full lock at lockT > 1.2, the HUD's "LOCKED"
// state) and breaks when the target leaves the cone.
//
// Usage: node_modules/.bin/esbuild scripts/probe-tracer-lock.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-tracer-lock.mjs
//   && node /tmp/probe-tracer-lock.mjs

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
import { WeaponSystem } from '../src/combat/WeaponSystem';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const R2D = 180 / Math.PI;

async function main(): Promise<void> {
  await RAPIER.init();
  const scene = new THREE.Scene();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const weapons = new WeaponSystem(scene, world, {
    groundHeight: () => 0,
    deckHeight: () => -Infinity,
  });

  const player = new Aircraft({
    scene,
    world,
    spec: SPECS['f35a']!,
    isPlayer: true,
    modelKey: SPECS['f35a']!.modelKey,
  });
  player.terrainHeightAt = () => 0;
  const enemy = new Aircraft({
    scene,
    world,
    spec: SPECS['mig29']!,
    isPlayer: false,
    modelKey: SPECS['mig29']!.modelKey,
  });
  enemy.terrainHeightAt = () => 0;
  await player.attachModel();
  await enemy.attachModel();

  // The pooled tracer lines the WeaponSystem added to the scene.
  const tracerLines: THREE.Line[] = [];
  scene.traverse((o) => {
    if (o instanceof THREE.Line) tracerLines.push(o);
  });
  const visibleTracers = (): THREE.Line[] => tracerLines.filter((l) => l.visible);

  // ---- 1. Enemy tracer ------------------------------------------------
  // Player flying east at 1000 m; enemy 500 m behind and above,
  // nose on the player.
  player.resetAt(0, 1000, 0, 90);
  player.setAirspeed(100);
  enemy.resetAt(-500, 1100, 0, 90);
  enemy.setAirspeed(120);
  // Aim the enemy at the player (its fwd() must point at the player
  // for aiGunShot's ray; resetAt sets heading only, so set the
  // quaternion to look at the player).
  const lookAt = (ac: Aircraft, target: THREE.Vector3): void => {
    const m = new THREE.Matrix4().lookAt(
      ac.pos,
      target,
      new THREE.Vector3(0, 1, 0),
    );
    const q = new THREE.Quaternion().setFromRotationMatrix(m);
    ac.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    ac.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    ac.vel.set(0, 0, 0);
  };
  lookAt(enemy, player.pos);
  check(
    'tracer pool exists in the scene (10 lines, all hidden at rest)',
    tracerLines.length === 10 && visibleTracers().length === 0,
    `lines=${tracerLines.length} visible=${visibleTracers().length}`,
  );
  weapons.aiGunShot(enemy, player, 0);
  const vis = visibleTracers();
  let tracerSpan = 0;
  let tracerNearShooter = false;
  if (vis.length > 0) {
    const p = (vis[0]!.geometry.attributes.position as THREE.BufferAttribute);
    const a = new THREE.Vector3(p.getX(0), p.getY(0), p.getZ(0));
    const b = new THREE.Vector3(p.getX(1), p.getY(1), p.getZ(1));
    tracerSpan = a.distanceTo(b);
    tracerNearShooter = a.distanceTo(enemy.pos) < 50;
  }
  check(
    'enemy gun pass draws a visible tracer line',
    vis.length === 1 && tracerSpan > 100 && tracerNearShooter,
    `visible=${vis.length} span=${tracerSpan.toFixed(0)} m (max 400) starts near shooter=${tracerNearShooter}`,
  );
  // The tracer lives 0.09 s: weapons.update hides it after its life.
  weapons.update(0.2, [enemy, player]);
  check(
    'tracer hides after its 0.09 s life (no stale lines)',
    visibleTracers().length === 0,
    `visible=${visibleTracers().length}`,
  );

  // ---- 2. Player missile lock ----------------------------------------
  // Enemy 1000 m ahead of the player, dead in the nose cone.
  player.resetAt(0, 1000, 0, 90);
  player.setAirspeed(100);
  enemy.resetAt(1000, 1000, 0, 90);
  enemy.setAirspeed(100);
  player.lockT = 0;
  player.lockTarget = null;
  // The main.ts lock scan, replicated: hold the target inside the
  // 15 deg cone, < 2500 m.
  const lockScan = (dt: number): void => {
    let best: Aircraft | null = null;
    let bd = 2500;
    for (const e of [enemy]) {
      if (!e.alive || e.dying) continue;
      const to = e.pos.clone().sub(player.pos);
      const d = to.length();
      if (d < bd && player.fwd().angleTo(to.normalize()) * R2D < 15) {
        bd = d;
        best = e;
      }
    }
    if (best) {
      player.lockTarget = best;
      player.lockT += dt;
    } else {
      player.lockTarget = null;
      player.lockT = Math.max(0, player.lockT - dt * 2);
    }
  };
  for (let i = 0; i < 120; i++) lockScan(1 / 60);
  check(
    'player lock builds to full lock (>1.2 s) with the enemy in the cone',
    player.lockTarget === enemy && player.lockT > 1.2,
    `lockT=${player.lockT.toFixed(2)}s target=${player.lockTarget === enemy ? 'enemy' : 'none'}`,
  );
  // Target leaves the cone (turns away / dies): the lock breaks.
  enemy.resetAt(0, 1000, 1500, 270);
  for (let i = 0; i < 120; i++) lockScan(1 / 60);
  check(
    'lock breaks when the target leaves the 15 deg cone',
    player.lockTarget === null && player.lockT === 0,
    `lockT=${player.lockT.toFixed(2)}s target=${player.lockTarget === null ? 'none' : 'enemy'}`,
  );

  player.dispose();
  enemy.dispose();
  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
