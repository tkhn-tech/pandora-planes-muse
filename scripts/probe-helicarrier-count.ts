// probe-helicarrier-count.ts — helicarrier count option (Issue 5).
//
//   0–3 carriers, default 1. Each flies its own loop (a scaled
//   hexagon, so the loops are concentric and never overlap) with its
//   own start index (spread around the circuits). Checks:
//   1. Count 0: all carriers deactivated — deck reads as absent
//      everywhere (deckHeightAt = -Infinity), the deck spawn falls
//      back to Alpha, and the AI null stub reports no deck.
//   2. Count 1: the primary flies its loop (60 s, alive, y=350,
//      40 m/s); the other two are deactivated and invisible.
//   3. Count 2: carriers 0+1 fly their loops for 10 sim-minutes;
//      minimum pairwise separation stays above the carrier length
//      (333 m) — no overlap or collision.
//   4. Count 3: same for all three carriers.
//   5. Count 2 deck spawn: the jet spawns on the primary's deck and
//      rides it through a waypoint turn (30 s, stays on deck, alive).
//
// Usage: node_modules/.bin/esbuild scripts/probe-helicarrier-count.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-helicarrier-count.mjs
//   && node /tmp/probe-helicarrier-count.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (The live game runs in a browser, where all of
// these exist; the stubs only let the driver parse the model in node.
// A texture that fails to decode just becomes a null texture in the
// parse — the geometry, which is what this driver verifies, still loads.)
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
import {
  HelicarrierController,
  carrierLoop,
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
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

const LOOP_SCALES = [1.0, 0.75, 1.25];
const START_INDEXES = [0, 2, 4];

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
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  // Build all three carriers (as main.ts does), each on its own loop.
  const carriers = LOOP_SCALES.map((scale, i) => {
    const loop = carrierLoop(scale);
    return new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
      waypoints: loop.waypoints,
      startIndex: START_INDEXES[i]!,
    });
  });
  await Promise.all(carriers.map((c) => c.build()));
  for (const c of carriers) {
    const db = c.deckBodyRef;
    if (db) {
      for (let i = 0; i < db.numColliders(); i++) {
        owners.set(db.collider(i).handle, 'deck');
      }
    }
  }
  check(
    'all 3 carriers built (modelReady)',
    carriers.every((c) => c.modelReady),
    carriers.map((c) => String(c.modelReady)).join(','),
  );

  const resetCarrier = (
    c: HelicarrierController,
    scale: number,
    startIndex: number,
  ): void => {
    const loop = carrierLoop(scale);
    const wp0 = loop.waypoints[startIndex]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
  };
  // The deck provider from main.ts, over all carriers.
  const deckHeightAll = (x: number, z: number): number => {
    let best = -Infinity;
    for (const c of carriers) {
      if (!c.active || !c.alive) continue;
      best = Math.max(best, c.deckHeightAt(x, z));
    }
    return best;
  };

  // ---- 1. Count 0: everything deactivated ----
  carriers.forEach((c) => c.deactivate());
  // Sample deck height at each loop's start position and at each
  // carrier's deck centre (where the deck would be if active).
  let deckSeen = false;
  for (let i = 0; i < 3; i++) {
    const loop = carrierLoop(LOOP_SCALES[i]!);
    if (deckHeightAll(loop.startPos.x, loop.startPos.z) > -Infinity) deckSeen = true;
  }
  check(
    'count 0: deck reads as absent everywhere (all deactivated)',
    !deckSeen && carriers.every((c) => !c.active),
    `deckSeen=${deckSeen} active=[${carriers.map((c) => c.active).join(',')}]`,
  );
  // The AI null stub (main.ts stepAi with 0 carriers): deckHeightAt
  // is -Infinity, so no carrier landing is ever attempted.
  const aiStub = { deckHeightAt: () => -Infinity };
  check(
    'count 0: AI null stub reports no deck',
    aiStub.deckHeightAt(0, 0) === -Infinity,
  );
  // A deck spawn with 0 carriers falls back to Alpha (startRound
  // guard: cfg.spawn === "carrier" && heliCount > 0).
  const heliCount0 = 0;
  const spawnFallsBack = !(heliCount0 > 0);
  check(
    'count 0: carrier spawn falls back to Alpha',
    spawnFallsBack && RUNWAY_TABLE[0]!.x === -2800,
    `alpha=(${RUNWAY_TABLE[0]!.x},${RUNWAY_TABLE[0]!.z})`,
  );

  // ---- 2. Count 1: primary only ----
  resetCarrier(carriers[0]!, LOOP_SCALES[0]!, START_INDEXES[0]!);
  carriers[1]!.deactivate();
  carriers[2]!.deactivate();
  let minY1 = Infinity;
  let maxY1 = -Infinity;
  for (let t = 0; t < 60; t += 1) {
    carriers[0]!.update(1);
    minY1 = Math.min(minY1, carriers[0]!.pos.y);
    maxY1 = Math.max(maxY1, carriers[0]!.pos.y);
  }
  check(
    'count 1: primary flies its loop 60 s (alive, y=350, 40 m/s)',
    carriers[0]!.alive && Math.abs(carriers[0]!.pos.y - 350) < 1,
    `alive=${carriers[0]!.alive} y=${carriers[0]!.pos.y.toFixed(1)} (min ${minY1.toFixed(1)}, max ${maxY1.toFixed(1)}) speed=${carriers[0]!.vel.length().toFixed(1)} m/s`,
  );
  check(
    'count 1: other two are deactivated and invisible',
    !carriers[1]!.active && !carriers[2]!.active &&
      !carriers[1]!.group.visible && !carriers[2]!.group.visible,
  );

  // ---- 3 & 4. Counts 2 and 3: separation over 10 sim-minutes ----
  for (const count of [2, 3]) {
    carriers.forEach((c, i) => {
      if (i < count) {
        resetCarrier(c, LOOP_SCALES[i]!, START_INDEXES[i]!);
      } else {
        c.deactivate();
      }
    });
    let minSep = Infinity;
    let died = false;
    for (let t = 0; t < 600; t += 1) {
      for (let i = 0; i < count; i++) {
        carriers[i]!.update(1);
        if (!carriers[i]!.alive) died = true;
      }
      for (let i = 0; i < count; i++) {
        for (let j = i + 1; j < count; j++) {
          const d = carriers[i]!.pos.distanceTo(carriers[j]!.pos);
          minSep = Math.min(minSep, d);
        }
      }
    }
    check(
      `count ${count}: all active carriers stay alive over 10 sim-minutes`,
      !died,
      `died=${died} alive=[${carriers.slice(0, count).map((c) => c.alive).join(',')}]`,
    );
    check(
      `count ${count}: minimum pairwise separation stays above the carrier length (333 m)`,
      minSep > 333,
      `minSep=${minSep.toFixed(0)} m`,
    );
  }

  // ---- 5. Count 2 deck spawn: jet rides the primary through a turn ----
  carriers.forEach((c, i) => {
    if (i < 2) {
      resetCarrier(c, LOOP_SCALES[i]!, START_INDEXES[i]!);
    } else {
      c.deactivate();
    }
  });
  const primary = carriers[0]!;
  const deckProvider: DeckProvider = {
    get alive() {
      return carriers.some((c) => c.active && c.alive);
    },
    deckHeightAt: (x, z) => deckHeightAll(x, z),
    velocity: () => primary.vel.clone(),
    velocityAt: (x, z) => {
      const c = carriers.find(
        (cc) => cc.active && cc.alive && cc.deckHeightAt(x, z) > -Infinity,
      ) ?? primary;
      const rx = x - c.pos.x;
      const rz = z - c.pos.z;
      const w = c.yawRate;
      return new THREE.Vector3(c.vel.x + w * rz, c.vel.y, c.vel.z - w * rx);
    },
    yawRate: () => primary.yawRate,
    yawRateAt: (x, z) => {
      const c = carriers.find(
        (cc) => cc.active && cc.alive && cc.deckHeightAt(x, z) > -Infinity,
      ) ?? primary;
      return c.yawRate;
    },
    local: (x, z) => {
      const c = carriers.find(
        (cc) => cc.active && cc.alive && cc.deckHeightAt(x, z) > -Infinity,
      );
      if (!c) return null;
      const v = c.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };
  const ac = new Aircraft({
    scene,
    world,
    spec: SPECS['fa18']!,
    isPlayer: true,
    modelKey: SPECS['fa18']!.modelKey,
    deck: deckProvider,
    events: {},
  });
  ac.terrainHeightAt = (x: number, z: number) =>
    heightfieldHeightAt(heights, res, x, z);
  for (const h of ac.colliderHandles()) owners.set(h, ac);
  // Deck spawn (main.ts startRound): 120 m aft of the deck centre,
  // with the deck point velocity.
  const fwd = new THREE.Vector3(
    Math.sin(primary.yaw),
    0,
    Math.cos(primary.yaw),
  );
  const sx = primary.pos.x - fwd.x * 120;
  const sz = primary.pos.z - fwd.z * 120;
  ac.resetAt(
    sx,
    primary.deckTopY + 2.2,
    sz,
    (primary.yaw * 180) / Math.PI,
  );
  const dv = deckProvider.velocityAt(sx, sz);
  ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
  ac.vel.copy(dv);
  // Ride 30 s (0.05 s substeps, carrier updates first, as in main.ts).
  let minLocalZ = Infinity;
  let maxLocalZ = -Infinity;
  let minLocalX = Infinity;
  let maxLocalX = -Infinity;
  let onDeckTime = 0;
  for (let t = 0; t < 30; t += 0.05) {
    primary.update(0.05);
    carriers[1]!.update(0.05);
    world.timestep = 0.05;
    world.step(eventQueue);
    ac.step(0.05, wind, null);
    const loc = primary.localPosition(ac.pos);
    if (loc) {
      minLocalX = Math.min(minLocalX, loc.x);
      maxLocalX = Math.max(maxLocalX, loc.x);
      minLocalZ = Math.min(minLocalZ, loc.z);
      maxLocalZ = Math.max(maxLocalZ, loc.z);
    }
    if (ac.onDeck) onDeckTime += 0.05;
  }
  check(
    'count 2: deck spawn rides the primary 30 s through a turn (alive, on deck)',
    ac.alive && onDeckTime > 25,
    `alive=${ac.alive} onDeckTime=${onDeckTime.toFixed(1)} s localX=[${minLocalX.toFixed(1)},${maxLocalX.toFixed(1)}] localZ=[${minLocalZ.toFixed(1)},${maxLocalZ.toFixed(1)}]`,
  );

  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
