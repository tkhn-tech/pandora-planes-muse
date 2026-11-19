// probe-ai-live.ts — pass 8, item 2: enemies never engage, through the
// LIVE menu/enemy-spawn path.
//
// probe-ai-hover (hovering player, enemies placed 1500 m away) passes
// 4/4 — but live, 3 Veterans never engaged in 2.5 min. This probe
// replicates startRound's enemy spawn EXACTLY (first two 450/600 m
// ahead +120/-150 m side, +120/180 m up, RANDOM headings; third at a
// random bearing at enemySpawnDistance 1200 m; setAirspeed(150),
// thrTarget 0.8) and a scripted player takeoff from Alpha (full
// throttle, pull at rotation, climb to 300 m, fly level east), with
// the live wind (-3,0,0), the live variable-dt pattern and the
// frame-1 origin shift. It logs AI state / distance / fire flags so
// a non-engagement can be diagnosed (pFair components, distances,
// spawn headings), instead of just failing.
//
// Usage: node_modules/.bin/esbuild scripts/probe-ai-live.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-ai-live.mjs
//   && node /tmp/probe-ai-live.mjs

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
import { AircraftAIController } from '../src/entities/AircraftAIController';
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
// Seeded RNG so the "random" spawn headings are reproducible.
const rngState = { s: 123456789 };
const rng = (): number => {
  rngState.s = (rngState.s * 1103515245 + 12345) & 0x7fffffff;
  return rngState.s / 0x7fffffff;
};

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
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
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

  const alpha = RUNWAY_TABLE[0]!;
  const player = new Aircraft({
    scene,
    world,
    spec: SPECS['f22']!,
    isPlayer: true,
    modelKey: SPECS['f22']!.modelKey,
  });
  player.terrainHeightAt = heightAtLive;
  origin.registerBody(player.body);
  for (const h of player.colliderHandles()) owners.set(h, player);
  await player.attachModel();
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);

  // ---- startRound's enemy spawn, verbatim (3 Veterans) ----
  const carrierStub = {
    deckHeightAt: () => -Infinity,
  } as unknown as Parameters<AircraftAIController['update']>[2];
  const enemies: Aircraft[] = [];
  const ais: AircraftAIController[] = [];
  const spawnHeadings: number[] = [];
  for (let i = 0; i < 3; i++) {
    const key = i % 2 === 0 ? 'mig29' : 'su34';
    const e = new Aircraft({
      scene,
      world,
      spec: SPECS[key]!,
      isPlayer: false,
      modelKey: SPECS[key]!.modelKey,
    });
    e.terrainHeightAt = heightAtLive;
    origin.registerBody(e.body);
    for (const h of e.colliderHandles()) owners.set(h, e);
    await e.attachModel();
    let ex: number;
    let ey: number;
    let ez: number;
    if (i < 2) {
      const fwd = player.fwd();
      const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
      const along = 450 + i * 150;
      const side = i === 0 ? 120 : -150;
      const up = 120 + i * 60;
      ex = player.pos.x + fwd.x * along + rgt.x * side;
      ey = player.pos.y + up;
      ez = player.pos.z + fwd.z * along + rgt.z * side;
    } else {
      const a = rng() * Math.PI * 2;
      const d = 1200 * (0.8 + rng() * 0.5);
      ex = player.pos.x + Math.cos(a) * d;
      ey = 650 + rng() * 350;
      ez = player.pos.z + Math.sin(a) * d;
    }
    const hdg = rng() * 360;
    spawnHeadings.push(hdg);
    e.resetAt(ex, ey, ez, hdg);
    e.setAirspeed(150);
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    const ai = new AircraftAIController(e, 'veteran', 'enemy', {
      groundHeight: heightAtLive,
      originShift: origin.cumulativeShift,
      rng,
    });
    origin.registerVector(ai.navTarget);
    ais.push(ai);
    enemies.push(e);
  }
  console.log(
    `  enemy spawn headings (deg): ${spawnHeadings.map((h) => h.toFixed(0)).join(', ')} (player hdg ${alpha.hdg})`,
  );
  console.log(
    `  enemy spawn dists: ${enemies.map((e) => e.pos.distanceTo(player.pos).toFixed(0)).join(', ')} m`,
  );

  // ---- live loop, 150 s: scripted player takeoff + level flight ----
  // Climb target from the CLI (default 1000 m): 1000 m is safely
  // above all terrain; 300 m flies the player low over the rising
  // ground east of Alpha — the run that killed e0 at 35.3 s and e1
  // at ~100 s before the pass-8 speed-scaled look-ahead.
  const climbTarget = Number(process.argv[2] ?? 1000) || 1000;
  console.log(`  scenario: scripted climb to ${climbTarget} m`);
  const dtPattern = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
  const vrMs = SPECS['f22']!.rotateSpeedKt * 0.514444;
  let everEngage = false;
  let firstEngageAt = -1;
  let everFire = false;
  let firstFireAt = -1;
  let minDist = Infinity;
  let enemyDeathAt = -1;
  let enemyDeathReasons: string[] = [];
  let playerDeathAt = -1;
  let frames = 0;
  const SIM_SECONDS = 150;
  for (let f = 0; f < SIM_SECONDS * 60; f++) {
    const t = f / 60;
    const rawDt = dtPattern[f % dtPattern.length]!;
    const sub = Math.max(1, Math.ceil(rawDt / 0.05));
    const dt = rawDt / sub;
    for (let s = 0; s < sub; s++) {
      frames++;
      // Scripted player: full throttle; pull at Vr; climb gently to
      // the scenario target and fly level east. A target >= 500 is an
      // absolute altitude (1000 m is safely above all terrain); a
      // target < 500 is terrain-relative (AGL), so the low run follows
      // the rising ground east of Alpha instead of flying a fixed
      // absolute altitude into a mountainside.
      // The climb demand is capped at 15 m/s — an uncapped
      // vs demand commands full pull at low speed and stalls the jet.
      player.thrTarget = 1;
      if (player.onSurface && player.ias >= vrMs) player.pitchIn = -1;
      else if (!player.onSurface) {
        const altTarget =
          climbTarget < 500
            ? heightAtLive(player.pos.x, player.pos.z) + climbTarget
            : climbTarget;
        const vsDes = Math.max(-10, Math.min(15, (altTarget - player.pos.y) * 0.08));
        player.pitchIn = Math.max(-0.4, Math.min(0.4, (player.vsi - vsDes) * 0.06));
        player.rollIn = 0;
      } else {
        player.pitchIn = 0;
        player.rollIn = 0;
      }
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      player.step(dt, WIND, null);
      for (let j = 0; j < enemies.length; j++) {
        const e = enemies[j]!;
        const ai = ais[j]!;
        if (!e.alive) {
          if (enemyDeathAt < 0) enemyDeathAt = t;
          continue;
        }
        const ctrl = ai.update(dt, player, carrierStub, t);
        if (ai.state === 'engage' && !everEngage) {
          everEngage = true;
          firstEngageAt = t;
        }
        if ((ai.fireGun || ai.fireMissile) && !everFire) {
          everFire = true;
          firstFireAt = t;
        }
        minDist = Math.min(minDist, e.pos.distanceTo(player.pos));
        e.step(dt, ctrl, WIND, heightAtLive, null);
        if (!e.alive && enemyDeathAt < 0) {
          enemyDeathAt = t;
        }
        if (!e.alive && enemyDeathReasons.length < 3) {
          enemyDeathReasons.push(
            `e${j} t=${t.toFixed(1)}s reason=${e.lastCrashReason ?? '-'} y=${e.pos.y.toFixed(0)} hf=${heightAtLive(e.pos.x, e.pos.z).toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt state=${ai.state}`,
          );
        }
      }
      if (!player.alive && playerDeathAt < 0) playerDeathAt = t;
    }
    const shift = origin.shiftIfNeeded(player.pos);
    if (shift) origin.applyShift(shift);
    if (f % 300 === 0) {
      console.log(
        `  [t=${t.toFixed(0)}s] player y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt gnd=${player.grounded} spawnProt=${player.spawnProt.toFixed(1)} alive=${player.alive} ` +
          enemies
            .map(
              (e, j) =>
                `e${j} ${e.alive ? 'alive' : 'DEAD'} ${ais[j]!.state} d=${e.pos.distanceTo(player.pos).toFixed(0)}m y=${e.pos.y.toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt`,
            )
            .join(' '),
      );
    }
    if (!player.alive) break;
  }
  console.log(`  frames=${frames} firstEngageAt=${firstEngageAt >= 0 ? firstEngageAt.toFixed(1) + 's' : 'never'} firstFireAt=${firstFireAt >= 0 ? firstFireAt.toFixed(1) + 's' : 'never'} minDist=${minDist.toFixed(0)}m enemyDeathAt=${enemyDeathAt >= 0 ? enemyDeathAt.toFixed(1) + 's' : 'none'} playerDeathAt=${playerDeathAt >= 0 ? playerDeathAt.toFixed(1) + 's' : 'none'}`);
  for (const r of enemyDeathReasons) console.log(`  ENEMY DEATH: ${r}`);
  check('AI engages the taking-off player (state=engage)', everEngage, everEngage ? `first at ${firstEngageAt.toFixed(1)}s` : 'never left patrol/bomb in 150 s');
  check('AI fires at the player (gun or missile)', everFire, everFire ? `first at ${firstFireAt.toFixed(1)}s` : `closest approach ${minDist.toFixed(0)} m`);
  check('enemies survive 150 s (no terrain death)', enemyDeathAt < 0, enemyDeathAt >= 0 ? `first enemy died at t=${enemyDeathAt.toFixed(1)}s` : 'all alive');
  check('scripted player survives the 150 s flight', player.alive, player.alive ? `y=${player.pos.y.toFixed(0)}` : `died at ${playerDeathAt.toFixed(1)}s: ${player.lastCrashReason ?? '-'}`);

  for (const e of enemies) e.dispose();
  player.dispose();
  console.log(failures === 0 ? 'ALL AI-LIVE CHECKS PASSED' : `${failures} AI-LIVE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
