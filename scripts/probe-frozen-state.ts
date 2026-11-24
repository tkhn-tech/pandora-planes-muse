// probe-frozen-state.ts — Pass 15, worker E, item 6 (FROZEN STATE).
//
// v4.9 live (24 Nov, 18 rounds): a Helicarrier respawn froze the F-35A
// mid-air at 78 kt, 1,138 ft, 0 fpm, G 0.0, surf=0, floor 30.4, carrier
// receding 818 m -> 1.5 km; throttle 100% did not move it and R did not
// respawn a live jet. The same surf=0 / G 0.0 frozen state occurred once
// on an Alpha spawn (jet stayed at 0 kt at 100% throttle until restart).
//
// The numbers pin the state down: 78 kt is the carrier cruise velocity
// (40 m/s = 77.7 kt) — the STALE deck velocity the respawn wrote into
// vel — and 1,138 ft is deck altitude, so the jet is hanging in mid-air
// exactly where the deck was when the carrier moved away. A jet whose
// step() runs cannot hang: gravity alone drops it ~5 m in the first
// second and full throttle accelerates it ~10 m/s^2. So the probe hunts
// for the state where a live, enabled jet stops being stepped or stops
// being integrated, through the LIVE path only:
//   * real Aircraft / BET, real Rapier world + runway boxes + real
//     SHIELD carrier GLBs (all three, carrierLoop slots, as in main.ts)
//   * main.ts's exact frame pacing: rawDt 0.11-0.25 s (4-9 fps), simDt
//     capped at 0.10 s, substeps dt <= 0.05 s
//   * live wind (-3, 0, 0) (menu default windDir 270, 3 m/s)
//   * floating-origin shifts (threshold 2000 m) every frame
//   * kill -> picker-respawn on the deck mid-loop (respawnPlayer's exact
//     deck branch: 85 m aft, deckTopY + 2.2, deck-point velocity), and
//     Alpha respawn, with throttle ramped to 100% after the spawn
//   * the R state machine from main.ts (spectating / respawnTimer /
//     pickerOpen) to check whether a dead/frozen jet can come back
//
// A run FAILS if any scenario produces, while the jet is alive:
//   (a) FROZEN: position moves < 0.05 m over a trailing 2 sim-s window
//       while throttle > 0.3 or airborne, or
//   (b) MID-AIR CRASH: the jet dies (alive=false) with its wreck more
//       than 10 m above the floor within 5 sim-s of a deck respawn
//       (the wreck then hangs frozen at deck height — the live picture),
//   (c) R DEAD-END: after a crash the R state machine cannot reach a
//       live jet (picker never opens, or every re-respawn re-crashes).
// It also reports body.isSleeping() / isEnabled() at every sample so a
// Rapier-sleep freeze is distinguished from a step() early-return.
//
// Usage: node_modules/.bin/esbuild scripts/probe-frozen-state.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-frozen-state.mjs
//   && node /tmp/probe-frozen-state.mjs
//   (delete /tmp/probe-frozen-state.mjs after running)

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
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1;
    height = 1;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void {
      (this.listeners[type] ??= []).push(fn);
    }
    removeEventListener(): void {}
    set src(_v: string) {
      queueMicrotask(() => {
        for (const fn of this.listeners['load'] ?? []) fn();
        this.onload?.();
      });
    }
  };
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: () => ({}),
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
import { readFileSync } from 'node:fs';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const MS2KT = 1.94384;
const D2R = Math.PI / 180;
// Live wind: menu default windDir 270 deg, 3 m/s -> (-3, 0, 0).
const WIND = new THREE.Vector3(-3, 0, 0);
// Live frame pacing at 4-9 fps: rawDt per frame, simDt capped at 0.10 s
// (main.ts pass-13 spiral breaker), substeps <= 0.05 s.
const RAW_DT_PATTERN = [0.25, 0.14, 0.2, 0.11, 0.17, 0.22, 0.13, 0.19, 0.25, 0.12, 0.16, 0.21];
const DECK_SPAWN_AFT_M = 85;

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
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
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
    owners.set(box.handle, 'terrain');
  }

  // Three carriers on their phase-offset loops, exactly as main.ts.
  const carriers: HelicarrierController[] = [0, 1, 2].map((slot) => {
    const loop = carrierLoop(slot);
    return new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt,
      waypoints: loop.waypoints,
      startIndex: loop.startIndex,
    });
  });
  await Promise.all(
    carriers.map(async (c) => {
      await c.build();
      c.registerWithFloatingOrigin(origin);
      const dc = c.deckColliderRef;
      if (dc) owners.set(dc.handle, 'deck');
    }),
  );
  const carrier = carriers[0]!;
  function carrierAt(x: number, z: number): HelicarrierController | null {
    for (const c of carriers) {
      if (c.active && c.alive && c.deckHeightAt(x, z) > -Infinity) return c;
    }
    return null;
  }
  const deckProvider: DeckProvider = {
    get alive() {
      return carriers.some((c) => c.active && c.alive);
    },
    get deckTopY() {
      let best = -Infinity;
      for (const c of carriers) {
        if (!c.active || !c.alive) continue;
        best = Math.max(best, c.deckTopY);
      }
      return best;
    },
    deckHeightAt: (x, z) => {
      let best = -Infinity;
      for (const c of carriers) {
        if (!c.active || !c.alive) continue;
        best = Math.max(best, c.deckHeightAt(x, z));
      }
      return best;
    },
    velocity: () => carrier.vel.clone(),
    velocityAt: (x, z) => {
      const c = carrierAt(x, z) ?? carrier;
      const rx = x - c.pos.x;
      const rz = z - c.pos.z;
      const w = c.yawRate;
      return new THREE.Vector3(c.vel.x + w * rz, c.vel.y, c.vel.z - w * rx);
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: (x, z) => (carrierAt(x, z) ?? carrier).yawRate,
    local: (x, z) => {
      const c = carrierAt(x, z);
      if (!c) return null;
      const v = c.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };
  const floorAt = (x: number, z: number): number =>
    Math.max(heightAt(x, z), deckProvider.deckHeightAt(x, z));

  // Reset all carriers to their loop starts (main.ts startRound).
  const resetCarriers = (): void => {
    carriers.forEach((c, i) => {
      const loop = carrierLoop(i);
      const wp0 = loop.waypoints[loop.startIndex]!;
      c.reset(
        loop.startPos.clone(),
        Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
      );
    });
  };

  const makeJet = async (key: string): Promise<Aircraft> => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = heightAt;
    origin.registerBody(ac.body);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    return ac;
  };
  const drain = (ac: Aircraft): void => {
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

  const disposeJet = (ac: Aircraft): void => {
    origin.unregisterBody(ac.body);
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  };
  // ---- live frame loop (main.ts animate, minus render/HUD/weapons) ----
  // Returns a driver object holding the mutable frame state.
  let dtIdx = 0;
  const frame = (ac: Aircraft, seconds: number, throttleFn: (t: number) => void): {
    simT: number;
    frozenAt: number;
    frozenDetail: string;
    crashedAt: number;
    crashReason: string;
    crashY: number;
    crashFloor: number;
    maxDv: number;
    maxDvAt: number;
    maxDvEarly: number;
    maxDvEarlyAt: number;
    maxSleep: boolean;
    steps: number;
  } => {
    let simT = 0;
    let frozenAt = -1;
    let frozenDetail = '';
    let crashedAt = -1;
    let crashReason = '';
    let crashY = 0;
    let crashFloor = 0;
    let maxDv = 0;
    let maxDvAt = -1;
    let maxDvEarly = 0;
    let maxDvEarlyAt = -1;
    let maxSleep = false;
    let steps = 0;
    // Trailing 2 sim-s position window for the freeze detector.
    const trail: Array<{ t: number; x: number; y: number; z: number }> = [];
    let prevVel = ac.vel.clone();
    while (simT < seconds) {
      const rawDt = RAW_DT_PATTERN[dtIdx % RAW_DT_PATTERN.length]!;
      dtIdx++;
      // Floating-origin shift at frame start (main.ts).
      const shift = origin.shiftIfNeeded(ac.pos);
      if (shift) origin.applyShift(shift);
      const simDt = Math.min(rawDt, 0.1);
      const sub = Math.max(1, Math.ceil(simDt / 0.05));
      const dt = simDt / sub;
      for (let s = 0; s < sub; s++) {
        for (const c of carriers) c.update(dt);
        world.timestep = dt;
        // Rapier velocity jump across this world.step (the dv>20 guard's
        // input): compare the body linvel before/after the step.
        const bv0 = ac.body.linvel();
        world.step(eventQueue);
        const bv1 = ac.body.linvel();
        const dvStep = Math.hypot(
          bv1.x - bv0.x,
          bv1.y - bv0.y,
          bv1.z - bv0.z,
        );
        if (ac.alive && dvStep > maxDv) {
          maxDv = dvStep;
          maxDvAt = simT;
        }
        if (ac.alive && simT < 5 && dvStep > maxDvEarly) {
          maxDvEarly = dvStep;
          maxDvEarlyAt = simT;
        }
        drain(ac);
        const wasAlive = ac.alive;
        const wasVel = ac.vel.clone();
        ac.step(dt, WIND, null);
        steps++;
        if (ac.body.isSleeping()) maxSleep = true;
        // The controller's own dv-guard input, for the record.
        void wasVel;
        void prevVel;
        prevVel = ac.vel.clone();
        if (wasAlive && !ac.alive && crashedAt < 0) {
          crashedAt = simT;
          crashReason = ac.lastCrashReason ?? '-';
          crashY = ac.pos.y;
          crashFloor = floorAt(ac.pos.x, ac.pos.z);
        }
      }
      simT += simDt;
      throttleFn(simT);
      trail.push({ t: simT, x: ac.pos.x, y: ac.pos.y, z: ac.pos.z });
      while (trail.length && trail[0]!.t < simT - 2) trail.shift();
      if (frozenAt < 0 && ac.alive && trail.length > 1) {
        const first = trail[0]!;
        const moved = Math.hypot(
          ac.pos.x - first.x,
          ac.pos.y - first.y,
          ac.pos.z - first.z,
        );
        if (moved < 0.05 && (ac.thrTarget > 0.3 || !ac.onSurface)) {
          frozenAt = simT;
          frozenDetail =
            `moved ${moved.toFixed(3)} m in 2 sim-s at t=${simT.toFixed(1)}s ` +
            `pos=(${ac.pos.x.toFixed(1)},${ac.pos.y.toFixed(1)},${ac.pos.z.toFixed(1)}) ` +
            `vel=(${ac.vel.x.toFixed(2)},${ac.vel.y.toFixed(2)},${ac.vel.z.toFixed(2)}) ` +
            `ias=${(ac.ias * MS2KT).toFixed(1)}kt vsi=${ac.vsi.toFixed(2)} ` +
            `gload=${ac.gload.toFixed(2)} surf=${ac.onSurface ? 1 : 0} ` +
            `gnd=${ac.grounded ? 1 : 0} onDeck=${ac.onDeck} ` +
            `thr=${ac.throttle.toFixed(2)}/${ac.thrTarget.toFixed(2)} ` +
            `bodyEnabled=${ac.body.isEnabled()} sleeping=${ac.body.isSleeping()} ` +
            `floor=${floorAt(ac.pos.x, ac.pos.z).toFixed(1)}`;
        }
      }
    }
    return { simT, frozenAt, frozenDetail, crashedAt, crashReason, crashY, crashFloor, maxDv, maxDvAt, maxDvEarly, maxDvEarlyAt, maxSleep, steps };
  };

  /** respawnPlayer's deck branch, verbatim (main.ts). */
  const deckRespawn = (ac: Aircraft): { spawnedOnDeck: boolean; carrierIdx: number } => {
    const liveCarrier = carriers.find((c) => c.active && c.alive) ?? null;
    if (!liveCarrier) return { spawnedOnDeck: false, carrierIdx: -1 };
    const c = liveCarrier;
    const fwd = new THREE.Vector3(Math.sin(c.yaw), 0, Math.cos(c.yaw));
    const sx = c.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = c.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, c.deckTopY + 2.2, sz, (c.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    ac.thrTarget = 0;
    return { spawnedOnDeck: true, carrierIdx: carriers.indexOf(c) };
  };
  /** respawnPlayer's runway branch, verbatim (main.ts). */
  const runwayRespawn = (ac: Aircraft, idx: 0 | 1 | 2): void => {
    const shift = origin.cumulativeShift;
    const r = RUNWAY_TABLE[idx]!;
    ac.resetAt(r.x - shift.x, r.elev + 2.25, r.z - shift.z, r.hdg);
    ac.thrTarget = 0;
  };

  // Drive the carriers alone for `seconds` sim-s (loop phase advance).
  const driveCarriers = (seconds: number): void => {
    let t = 0;
    let i = 0;
    while (t < seconds) {
      const rawDt = RAW_DT_PATTERN[i++ % RAW_DT_PATTERN.length]!;
      const simDt = Math.min(rawDt, 0.1);
      const sub = Math.max(1, Math.ceil(simDt / 0.05));
      const dt = simDt / sub;
      for (let s = 0; s < sub; s++) for (const c of carriers) c.update(dt);
      t += simDt;
    }
  };

  // ---- S1: kill -> deck respawn mid-loop, throttle to 100% ----
  console.log('--- S1. Kill -> deck respawn mid-loop, throttle 100% (F-35A, the live jet) ---');
  {
    resetCarriers();
    driveCarriers(90); // carrier mid-loop, possibly turning / climbing
    const ac = await makeJet('f35a');
    ac.crash('test kill');
    const spawn = deckRespawn(ac);
    console.log(
      `  respawned on carrier ${spawn.carrierIdx} at (${ac.pos.x.toFixed(0)}, ${ac.pos.y.toFixed(1)}, ${ac.pos.z.toFixed(0)}) ` +
        `deckTop=${carriers[spawn.carrierIdx]!.deckTopY.toFixed(1)} yawRate=${carriers[spawn.carrierIdx]!.yawRate.toFixed(3)} ` +
        `deckVy=${carriers[spawn.carrierIdx]!.vel.y.toFixed(1)} vel=(${ac.vel.x.toFixed(1)},${ac.vel.z.toFixed(1)})`,
    );
    const r = frame(ac, 60, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
      ac.pitchIn = 0;
      ac.rollIn = 0;
    });
    console.log(
      `  after 60 sim-s: alive=${ac.alive} onDeck=${ac.onDeck} surf=${ac.onSurface ? 1 : 0} ` +
        `pos=(${ac.pos.x.toFixed(0)},${ac.pos.y.toFixed(1)},${ac.pos.z.toFixed(0)}) ` +
        `ias=${(ac.ias * MS2KT).toFixed(0)}kt gload=${ac.gload.toFixed(2)} ` +
        `maxDvAcrossWorldStep=${r.maxDv.toFixed(1)} m/s at ${r.maxDvAt.toFixed(1)}s ` +
        `(first 5 s: ${r.maxDvEarly.toFixed(1)} m/s at ${r.maxDvEarlyAt.toFixed(1)}s) ` +
        `sleptAtAnyPoint=${r.maxSleep} steps=${r.steps}`,
    );
    if (r.frozenAt >= 0) console.log(`  FROZEN at ${r.frozenAt.toFixed(1)}s: ${r.frozenDetail}`);
    if (r.crashedAt >= 0) {
      console.log(
        `  CRASHED at ${r.crashedAt.toFixed(1)}s reason=${r.crashReason} y=${r.crashY.toFixed(1)} floor=${r.crashFloor.toFixed(1)} (wreck ${(r.crashY - r.crashFloor).toFixed(1)} m above floor)`,
      );
    }
    check(
      'S1: no frozen state in 60 s after a mid-loop deck respawn at 100% throttle',
      r.frozenAt < 0,
      r.frozenDetail,
    );
    check(
      'S1: no mid-air crash within 5 s of the deck respawn',
      !(r.crashedAt >= 0 && r.crashedAt < 5 && r.crashY - r.crashFloor > 10),
      r.crashedAt >= 0 ? `crashed at ${r.crashedAt.toFixed(1)}s reason=${r.crashReason} ${(r.crashY - r.crashFloor).toFixed(1)} m above floor` : '',
    );
    // NOTE: survival is NOT the item-6 criterion — a hands-off jet that
    // rolls off the bow and flies into a mountainside is the auto-liftoff
    // area (another worker), not a freeze. The criterion is that IF it
    // dies, it dies on a real impact (near the floor), not hanging
    // mid-air right after the spawn.
    check(
      'S1: any crash is a real impact (wreck within 15 m of the floor), not a mid-air hang',
      r.crashedAt < 0 || r.crashY - r.crashFloor < 15,
      r.crashedAt >= 0 ? `crashed at ${r.crashedAt.toFixed(1)}s ${(r.crashY - r.crashFloor).toFixed(1)} m above floor` : '',
    );
    disposeJet(ac);
  }

  // ---- S2: deck respawn while the carrier is turning hard + climbing ----
  console.log('--- S2. Deck respawn at max yaw rate (turn) and at max climb ---');
  {
    for (const mode of ['turn', 'climb'] as const) {
      resetCarriers();
      // Drive carrier 0 until the wanted regime (or 240 s of loop).
      let t = 0;
      let i = 0;
      let hit = false;
      let hitVal = 0;
      while (t < 240 && !hit) {
        const rawDt = RAW_DT_PATTERN[i++ % RAW_DT_PATTERN.length]!;
        const simDt = Math.min(rawDt, 0.1);
        const sub = Math.max(1, Math.ceil(simDt / 0.05));
        const dt = simDt / sub;
        for (let s = 0; s < sub; s++) for (const c of carriers) c.update(dt);
        t += simDt;
        if (mode === 'turn' && Math.abs(carrier.yawRate) > 0.11) {
          hit = true;
          hitVal = carrier.yawRate;
        }
        if (mode === 'climb' && carrier.vel.y > 8) {
          hit = true;
          hitVal = carrier.vel.y;
        }
      }
      const ac = await makeJet('f35a');
      ac.crash('test kill');
      const spawn = deckRespawn(ac);
      const r = frame(ac, 45, (tt) => {
        ac.thrTarget = Math.min(1, tt / 2);
      });
      console.log(
        `  [${mode}] regimeReached=${hit} (${mode === 'turn' ? `yawRate=${hitVal.toFixed(3)}` : `deckVy=${hitVal.toFixed(1)}`}) ` +
          `spawnCarrier=${spawn.carrierIdx} alive=${ac.alive} onDeck=${ac.onDeck} ` +
          `crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
          `frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) + 's' : 'never'} maxDv=${r.maxDv.toFixed(1)} slept=${r.maxSleep}`,
      );
      if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
      check(
        `S2[${mode}]: no frozen state after deck respawn in the ${mode} regime`,
        r.frozenAt < 0,
        r.frozenDetail,
      );
      check(
        `S2[${mode}]: no mid-air crash within 5 s of the deck respawn`,
        !(r.crashedAt >= 0 && r.crashedAt < 5 && r.crashY - r.crashFloor > 10),
        r.crashedAt >= 0 ? `crashed at ${r.crashedAt.toFixed(1)}s reason=${r.crashReason}` : '',
      );
      disposeJet(ac);
    }
  }

  // ---- S3: Alpha spawn, park 30 s at 0 throttle, then 100% ----
  console.log('--- S3. Alpha spawn: park 30 s at 0%, then 100% (the second live case) ---');
  {
    resetCarriers();
    origin.reset();
    const ac = await makeJet('f35a');
    runwayRespawn(ac, 0);
    const r1 = frame(ac, 30, () => {
      ac.thrTarget = 0;
    });
    console.log(
      `  parked 30 s: alive=${ac.alive} surf=${ac.onSurface ? 1 : 0} gnd=${ac.grounded ? 1 : 0} ` +
        `gload=${ac.gload.toFixed(2)} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
        `pos moved check frozenAt=${r1.frozenAt >= 0 ? r1.frozenAt.toFixed(1) : 'never'} slept=${r1.maxSleep}`,
    );
    const r2 = frame(ac, 30, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
    });
    console.log(
      `  then 100% for 30 s: alive=${ac.alive} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
        `pos=(${ac.pos.x.toFixed(0)},${ac.pos.y.toFixed(1)},${ac.pos.z.toFixed(0)}) ` +
        `crashedAt=${r2.crashedAt >= 0 ? r2.crashedAt.toFixed(1) + 's/' + r2.crashReason : 'never'} frozenAt=${r2.frozenAt >= 0 ? r2.frozenAt.toFixed(1) : 'never'}`,
    );
    check(
      'S3: parked jet does not freeze while parked (0% throttle, 30 s)',
      r1.frozenAt < 0,
      r1.frozenDetail,
    );
    check(
      'S3: jet moves / takes off at 100% throttle after parking (no 0 kt freeze)',
      r2.frozenAt < 0 && ac.ias * MS2KT > 20,
      r2.frozenDetail || `ias=${(ac.ias * MS2KT).toFixed(1)}kt`,
    );
    disposeJet(ac);
  }

  // ---- S4: repeated death -> deck respawn cycles (crash loop?) ----
  console.log('--- S4. Five death -> deck-respawn cycles (does R keep giving a live jet?) ---');
  {
    resetCarriers();
    driveCarriers(45);
    const ac = await makeJet('f35a');
    let liveAfter = 0;
    let midAirCrashes = 0;
    for (let cycle = 0; cycle < 5; cycle++) {
      ac.crash('test kill');
      const spawn = deckRespawn(ac);
      // The R state machine (main.ts): death -> spectating + 3 s timer
      // -> picker opens -> R picks the highlighted default (carrier)
      // -> respawnPlayer. Here the respawn above IS that pick.
      const r = frame(ac, 20, (t) => {
        ac.thrTarget = Math.min(1, t / 2);
      });
      const midAir =
        r.crashedAt >= 0 && r.crashedAt < 5 && r.crashY - r.crashFloor > 10;
      if (midAir) midAirCrashes++;
      if (ac.alive && r.frozenAt < 0) liveAfter++;
      console.log(
        `  cycle ${cycle}: spawnCarrier=${spawn.carrierIdx} aliveAfter20s=${ac.alive} ` +
          `onDeck=${ac.onDeck} crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
          `midAir=${midAir} frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) : 'never'} maxDv=${r.maxDv.toFixed(1)}`,
      );
      if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
      // Advance the loop phase between cycles so each respawn is new.
      driveCarriers(25);
    }
    check(
      'S4: every death -> deck-respawn cycle yields a live jet after 20 s',
      liveAfter === 5,
      `live after 20 s in ${liveAfter}/5 cycles`,
    );
    check(
      'S4: no mid-air crash within 5 s of any deck respawn',
      midAirCrashes === 0,
      `${midAirCrashes} mid-air crash(es)`,
    );
    disposeJet(ac);
  }

  // ---- S5: all four jets, deck respawn mid-loop ----
  console.log('--- S5. All four jets: deck respawn mid-loop, 100% throttle, 45 s ---');
  {
    for (const key of ['fa18', 'f22', 'f15e', 'f35a'] as const) {
      resetCarriers();
      driveCarriers(120);
      const ac = await makeJet(key);
      ac.crash('test kill');
      deckRespawn(ac);
      const r = frame(ac, 45, (t) => {
        ac.thrTarget = Math.min(1, t / 2);
      });
      console.log(
        `  ${key}: alive=${ac.alive} onDeck=${ac.onDeck} ` +
          `crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
          `frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) : 'never'} maxDv=${r.maxDv.toFixed(1)} slept=${r.maxSleep}`,
      );
      if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
      check(
        `S5[${key}]: no frozen state after mid-loop deck respawn`,
        r.frozenAt < 0,
        r.frozenDetail,
      );
      check(
        `S5[${key}]: no mid-air crash within 5 s of the deck respawn`,
        !(r.crashedAt >= 0 && r.crashedAt < 5 && r.crashY - r.crashFloor > 10),
        r.crashedAt >= 0 ? `crashed at ${r.crashedAt.toFixed(1)}s reason=${r.crashReason}` : '',
      );
      disposeJet(ac);
    }
  }

  // ---- S6: Rapier sleep, isolated ----
  // A parked jet (Alpha, 0% throttle) for 60 sim-s: does the body sleep,
  // and does the controller's setLinvel(wakeUp=true) still move it when
  // the throttle goes to 100%? A sleeping body that never wakes is the
  // textbook freeze — measure it directly instead of assuming.
  console.log('--- S6. Rapier sleep, isolated: park 60 s, then 100% throttle ---');
  {
    resetCarriers();
    origin.reset();
    const ac = await makeJet('fa18');
    runwayRespawn(ac, 0);
    let sleptSeconds = 0;
    let samples = 0;
    const r1 = frame(ac, 60, () => {
      ac.thrTarget = 0;
      samples++;
      if (ac.body.isSleeping()) sleptSeconds++;
    });
    // Note: samples count frames, not seconds; report the raw counts.
    console.log(
      `  parked 60 sim-s: sleepingFrames=${sleptSeconds}/${samples} alive=${ac.alive} ` +
        `gload=${ac.gload.toFixed(2)} surf=${ac.onSurface ? 1 : 0}`,
    );
    const ias0 = ac.ias;
    const r2 = frame(ac, 10, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
    });
    console.log(
      `  100% for 10 s: ias ${((ias0 * MS2KT)).toFixed(1)} -> ${(ac.ias * MS2KT).toFixed(1)} kt ` +
        `alive=${ac.alive} frozenAt=${r2.frozenAt >= 0 ? r2.frozenAt.toFixed(1) : 'never'} sleepingNow=${ac.body.isSleeping()}`,
    );
    check(
      'S6: a parked-then-throttled jet accelerates (sleep does not freeze it)',
      ac.ias * MS2KT > 20 && r2.frozenAt < 0,
      `ias=${(ac.ias * MS2KT).toFixed(1)}kt frozenAt=${r2.frozenAt}`,
    );
    void r1;
    disposeJet(ac);
  }

  // ---- S7: the R recovery for a frozen jet (pass 15, worker E, item 6) ----
  // The physics loop (S1-S6) never reproduces the freeze, so the
  // trigger itself is still unproven — but the RECOVERY dead-end is
  // demonstrated and fixed here. The old stuck detector gates on
  // `vel < 1` and `!grounded`, and a frozen jet's `vel` is STALE:
  // deck velocity ~40 m/s on a deck respawn (so `vel < 1` never
  // fires), or 0 kt with `grounded` still true from respawn (so
  // `!grounded` never fires). R can therefore never open the picker
  // for exactly the live frozen state. The pass-15 fix adds a second
  // gate that measures MOVEMENT, not velocity: alive, not dying, not
  // on a surface, throttle past half, vector < 70, and the position
  // moved < 0.1 m in the frame. This section drives the detector,
  // verbatim from main.ts's animate(), over frozen and healthy
  // states and shows the old gate never firing for a frozen jet
  // while the new gate fires — and that it does NOT fire for a
  // healthy jet, a VTOL hover, or a parked/braked jet.
  console.log('--- S7. R recovery: the stuck detector on a frozen jet vs healthy states ---');
  {
    type DetState = {
      alive: boolean;
      dying: boolean;
      grounded: boolean;
      onDeck: boolean;
      onSurface: boolean;
      thrTarget: number;
      vector: number;
      vel: THREE.Vector3;
      pos: THREE.Vector3;
      agl: number;
    };
    const runDetector = (
      frames: number,
      stepFn: () => void,
      stateFn: () => DetState,
    ): { oldFiredAt: number; newFiredAt: number; rAt: number; stuckTime: number } => {
      let stuckTime = 0;
      let oldFiredAt = -1;
      let newFiredAt = -1;
      let rAt = -1;
      let prev: THREE.Vector3 | null = null;
      for (let f = 0; f < frames; f++) {
        stepFn();
        const s = stateFn();
        const moved = prev ? s.pos.distanceTo(prev) : Infinity;
        prev = s.pos.clone();
        // Verbatim from main.ts (animate): the old beached gate and
        // the pass-15 unresponsive gate. animate adds one substep
        // dt (<= 0.05 s) per frame, so stuckTime advances 0.05/frame
        // here and R (stuckTime > 2) is available after 41 frames.
        const oldBeached =
          s.alive && !s.dying && !s.grounded && !s.onDeck && s.vel.length() < 1 && s.agl < 10;
        const unresponsive =
          s.alive &&
          !s.dying &&
          !s.onSurface &&
          s.thrTarget > 0.5 &&
          s.vector < 70 &&
          moved < 0.1;
        if (oldBeached && oldFiredAt < 0) oldFiredAt = f;
        if (unresponsive && newFiredAt < 0) newFiredAt = f;
        if (oldBeached || unresponsive) {
          stuckTime += 0.05;
          if (stuckTime > 2 && rAt < 0) rAt = f;
        } else {
          stuckTime = 0;
        }
      }
      return { oldFiredAt, newFiredAt, rAt, stuckTime };
    };
    const stateOf = (ac: Aircraft): DetState => ({
      alive: ac.alive,
      dying: ac.dying,
      grounded: ac.grounded,
      onDeck: ac.onDeck,
      onSurface: ac.onSurface,
      thrTarget: ac.thrTarget,
      vector: ac.vector,
      vel: ac.vel,
      pos: ac.pos,
      agl: ac.pos.y - heightAt(ac.pos.x, ac.pos.z),
    });
    const noop = (): void => {};

    // S7a: mid-air frozen jet, the live case — deck respawn, then
    // held static (step() not running) with the stale deck velocity.
    {
      const ac = await makeJet('f35a');
      deckRespawn(ac);
      ac.grounded = false; // airborne: a step ran and cleared it
      ac.onSurface = false;
      ac.onDeck = false;
      ac.thrTarget = 1;
      // vel is the stale deck velocity (~40 m/s); pos is static.
      console.log(
        `  S7a frozen mid-air: vel=(${ac.vel.x.toFixed(1)},${ac.vel.y.toFixed(1)},${ac.vel.z.toFixed(1)}) ` +
          `agl=${(ac.pos.y - heightAt(ac.pos.x, ac.pos.z)).toFixed(0)} m thrTarget=${ac.thrTarget} surf=${ac.onSurface ? 1 : 0}`,
      );
      const r = runDetector(60, noop, () => stateOf(ac));
      console.log(
        `  S7a: old gate fired at frame ${r.oldFiredAt} (never = -1); new gate fired at frame ${r.newFiredAt}; R available at frame ${r.rAt} (stuckTime=${r.stuckTime.toFixed(2)})`,
      );
      check(
        'S7a: the OLD vel-based gate NEVER fires for the frozen mid-air jet (vel stale at ~40 m/s)',
        r.oldFiredAt < 0,
        `oldFiredAt=${r.oldFiredAt}`,
      );
      check(
        'S7a: the NEW movement-based gate fires for the frozen mid-air jet',
        r.newFiredAt >= 0,
        `newFiredAt=${r.newFiredAt}`,
      );
      check(
        'S7a: R opens the picker for the frozen mid-air jet (stuckTime > 2 within 60 frames)',
        r.rAt >= 0 && r.rAt <= 45,
        `rAt=${r.rAt}`,
      );
      disposeJet(ac);
    }

    // S7b: Alpha frozen jet — vel stale at 0, grounded stale true
    // (respawn set it, step() never ran to clear it).
    {
      const ac = await makeJet('f35a');
      runwayRespawn(ac, 0);
      ac.vel.set(0, 0, 0);
      ac.grounded = true;
      ac.onSurface = false;
      ac.onDeck = false;
      ac.thrTarget = 1;
      console.log(
        `  S7b frozen on Alpha: vel=0 kt grounded=${ac.grounded} surf=${ac.onSurface ? 1 : 0} thrTarget=${ac.thrTarget}`,
      );
      const r = runDetector(60, noop, () => stateOf(ac));
      console.log(
        `  S7b: old gate fired at frame ${r.oldFiredAt}; new gate fired at frame ${r.newFiredAt}; R available at frame ${r.rAt}`,
      );
      check(
        'S7b: the OLD gate NEVER fires for the frozen Alpha jet (grounded stale true)',
        r.oldFiredAt < 0,
        `oldFiredAt=${r.oldFiredAt}`,
      );
      check(
        'S7b: the NEW gate fires for the frozen Alpha jet and R is available',
        r.newFiredAt >= 0 && r.rAt >= 0 && r.rAt <= 45,
        `newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      disposeJet(ac);
    }

    // S7c: healthy control — a real jet accelerating from 0 at 100%
    // throttle on Alpha. It moves > 0.1 m/frame from the second
    // frame on, so the new gate must never accumulate 2 s.
    {
      const ac = await makeJet('f35a');
      runwayRespawn(ac, 0);
      ac.thrTarget = 1;
      const stepReal = (): void => {
        world.timestep = 0.05;
        world.step(eventQueue);
        drain(ac);
        ac.step(0.05, WIND, null);
      };
      const r = runDetector(60, stepReal, () => stateOf(ac));
      console.log(
        `  S7c healthy takeoff: ias=${(ac.ias * MS2KT).toFixed(1)} kt pos moved; stuckTime=${r.stuckTime.toFixed(2)} oldFiredAt=${r.oldFiredAt} newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      check(
        'S7c: the NEW gate does NOT fire for a healthy jet accelerating at 100% throttle (no false R)',
        r.rAt < 0,
        `rAt=${r.rAt} stuckTime=${r.stuckTime.toFixed(2)}`,
      );
      disposeJet(ac);
    }

    // S7d: VTOL hover control — static, vector 90, throttle 1. The
    // vector >= 70 exclusion must keep the gate off (the pass-5
    // hover false positive must not come back).
    {
      const ac = await makeJet('f35a');
      const r0 = RUNWAY_TABLE[0]!;
      // Shift-corrected coords, as runwayRespawn does: the origin
      // has shifted by S7, so raw table coords would land the jet
      // in the ranges (terrain high, agl < 10) instead of over Alpha.
      const shift = origin.cumulativeShift;
      ac.resetAt(r0.x - shift.x, r0.elev + 100, r0.z - shift.z, r0.hdg);
      ac.setAirspeed(0);
      ac.vector = 90;
      ac.thrTarget = 1;
      ac.onSurface = false;
      ac.grounded = false;
      const r = runDetector(60, noop, () => stateOf(ac));
      console.log(
        `  S7d hover (vector 90, static): oldFiredAt=${r.oldFiredAt} newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      check(
        'S7d: the NEW gate does NOT fire for a VTOL hover (vector >= 70 excluded)',
        r.newFiredAt < 0 && r.rAt < 0,
        `newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      disposeJet(ac);
    }

    // S7e: parked/braked control — on a surface at 100% throttle,
    // static. onSurface must keep the gate off.
    {
      const ac = await makeJet('f35a');
      runwayRespawn(ac, 0);
      ac.thrTarget = 1;
      ac.onSurface = true;
      ac.grounded = true;
      const r = runDetector(60, noop, () => stateOf(ac));
      console.log(
        `  S7e parked (onSurface, throttle 1): oldFiredAt=${r.oldFiredAt} newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      check(
        'S7e: the NEW gate does NOT fire for a parked jet at 100% throttle (onSurface excluded)',
        r.newFiredAt < 0 && r.rAt < 0,
        `newFiredAt=${r.newFiredAt} rAt=${r.rAt}`,
      );
      disposeJet(ac);
    }

    // S7f: wiring in main.ts (source checks).
    const mainSrc = readFileSync('src/main.ts', 'utf8');
    check(
      'S7f: main.ts has the unresponsive gate (movement-based, not vel-based)',
      mainSrc.includes('unresponsive') &&
        mainSrc.includes('moved < 0.1') &&
        mainSrc.includes('thrTarget > 0.5'),
      '',
    );
    check(
      'S7f: the unresponsive gate excludes VTOL hover (vector < 70) and surfaces (!onSurface)',
      mainSrc.includes('player.vector < 70') && mainSrc.includes('!player.onSurface'),
      '',
    );
    check(
      'S7f: R opens the picker once the detector has fired (stuckTime > 2)',
      mainSrc.includes('stuckTime > 2') && mainSrc.includes('showRespawnPicker(true)'),
      '',
    );
  }

  console.log(failures === 0 ? '\nALL FROZEN-STATE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
