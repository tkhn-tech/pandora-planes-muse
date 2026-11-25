// probe-pass17-frozen.ts — Pass 17, worker E, item 1 (FROZEN STATE, top
// priority) + the R-recovery machine.
//
// v4.10 live (25 Nov, 16:13-17:24 SGT, footer v4.10): the frozen state
// hit TWICE — Charlie spawn and Alpha respawn: surf=0, G 0.0, 0 kt for
// 30-40 s, W gave no throttle, R did nothing; only the Restart round
// button recovered it. Pass 15's recovery net (movement-based second
// gate, main.ts ~1058-1070: stuckHintActive/stuckTime) did NOT fire.
//
// Paths pass 15 explicitly did NOT cover, covered here:
//   S1. dying-fall -> finishCrash -> respawn (pass 15 used ac.crash()
//       directly, skipping startDying, the fall, finishCrash, the
//       spectate/timer/picker flow, and the stale pendingContact the
//       wreck leaves behind).
//   S2. respawn on a DYING/SINKING carrier (pass 15 respawned only on
//       live carriers). main.ts's liveCarrier test is `active && alive`
//       — a dying carrier is still alive, so a deck respawn can pick
//       a carrier whose deckHeightAt is already -Infinity.
//   S3. respawn with a LIVE floating-origin shift (pass 15 reset the
//       origin first): fly far, crash, respawnPlayer runway branch
//       with the shift subtraction, and check the jet lands ON the
//       runway.
//   S4. the R machine, driven by the logic actually written in
//       src/main.ts (parsed from source, not assumed): the dying +
//       picker-cancelled dead-end, the thrTarget=0 frozen jet the
//       unresponsive gate can never see, and the stuckTime
//       accumulation rate at 4 fps (the detector adds one SUBSTEP dt
//       per FRAME, so at 4 fps it advances at 0.2x real time).
//
// Live path only: real Aircraft/BET, real Rapier world + runway
// boxes + real SHIELD carrier GLBs, main.ts frame pacing (rawDt
// 0.11-0.25 s, simDt capped at 0.10 s, substeps <= 0.05 s), live wind
// 3 m/s, floating-origin shifts, real GLBs with a globalThis.Image
// stub and an assertion that modelStatus === 'ok'.
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass17-frozen.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass17-frozen.mjs
//   && node /tmp/probe-pass17-frozen.mjs
//   (delete /tmp/probe-pass17-frozen.mjs after running)

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
const WIND = new THREE.Vector3(-3, 0, 0);
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
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)
        .setRotation({
          x: 0,
          y: Math.sin(hdg / 2),
          z: 0,
          w: Math.cos(hdg / 2),
        }),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }

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

  let dtIdx = 0;
  // One live frame, main.ts animate pacing: rawDt 0.11-0.25 s, simDt
  // capped at 0.10 s, substeps <= 0.05 s, carriers first, Rapier step,
  // drain, player step, origin shift at frame start.
  const frame = (
    ac: Aircraft,
    seconds: number,
    throttleFn: (t: number) => void,
  ): {
    simT: number;
    frozenAt: number;
    frozenDetail: string;
    crashedAt: number;
    crashReason: string;
    crashY: number;
    crashFloor: number;
  } => {
    let simT = 0;
    let frozenAt = -1;
    let frozenDetail = '';
    let crashedAt = -1;
    let crashReason = '';
    let crashY = 0;
    let crashFloor = 0;
    const trail: Array<{ t: number; x: number; y: number; z: number }> = [];
    while (simT < seconds) {
      const rawDt = RAW_DT_PATTERN[dtIdx % RAW_DT_PATTERN.length]!;
      dtIdx++;
      const shift = origin.shiftIfNeeded(ac.pos);
      if (shift) origin.applyShift(shift);
      const simDt = Math.min(rawDt, 0.1);
      const sub = Math.max(1, Math.ceil(simDt / 0.05));
      const dt = simDt / sub;
      for (let s = 0; s < sub; s++) {
        for (const c of carriers) c.update(dt);
        world.timestep = dt;
        world.step(eventQueue);
        drain(ac);
        const wasAlive = ac.alive;
        ac.step(dt, WIND, null);
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
            `ias=${(ac.ias * MS2KT).toFixed(1)}kt gDisplay=${ac.gDisplay.toFixed(2)} ` +
            `surf=${ac.onSurface ? 1 : 0} gnd=${ac.grounded ? 1 : 0} ` +
            `dying=${ac.dying ? 1 : 0} thr=${ac.throttle.toFixed(2)}/${ac.thrTarget.toFixed(2)} ` +
            `bodyEnabled=${ac.body.isEnabled()} sleeping=${ac.body.isSleeping()} ` +
            `floor=${floorAt(ac.pos.x, ac.pos.z).toFixed(1)}`;
        }
      }
    }
    return { simT, frozenAt, frozenDetail, crashedAt, crashReason, crashY, crashFloor };
  };

  /** respawnPlayer's deck branch, verbatim (main.ts), with the
   *  liveCarrier selection AS WRITTEN in main.ts (see S2b). */
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
  // Kill every pod of a carrier through the real API, then run its
  // update() until the dying flag lands (lift <= 0.01 -> dying).
  const killCarrier = (c: HelicarrierController): boolean => {
    for (let i = 0; i < 4; i++) c.damagePod(i, 500);
    let t = 0;
    while (t < 5 && !c.dying) {
      c.update(0.05);
      t += 0.05;
    }
    return c.dying;
  };

  const mainSrc = readFileSync('src/main.ts', 'utf8');

  // ---- S0. Model sanity: every jet used below loads its real GLB. ----
  console.log('--- S0. Real GLBs load (modelStatus === ok) ---');
  for (const key of ['f35a', 'f22', 'fa18'] as const) {
    const ac = await makeJet(key);
    console.log(`  ${key}: modelStatus=${ac.modelStatus}`);
    check(`S0[${key}]: real GLB loads (modelStatus === 'ok')`, ac.modelStatus === 'ok', `modelStatus=${ac.modelStatus}`);
    disposeJet(ac);
  }

  // ---- S1. Dying fall -> finishCrash -> respawn. ----
  console.log('--- S1. Dying fall -> finishCrash -> respawn (F-35A, F-22) ---');
  for (const key of ['f35a', 'f22'] as const) {
    resetCarriers();
    driveCarriers(30);
    origin.reset();
    const ac = await makeJet(key);
    const alpha = RUNWAY_TABLE[0]!;
    ac.resetAt(alpha.x, 800, alpha.z, alpha.hdg);
    ac.setAirspeed(100);
    ac.thrTarget = 0.8;
    // Fly level-ish for 3 sim-s, then get shot down mid-air.
    frame(ac, 3, () => {
      ac.thrTarget = 0.8;
    });
    const fallY = ac.pos.y;
    const fallKt = ac.ias * MS2KT;
    ac.startDying('shot down');
    // The dying fall, live loop: does finishCrash ever fire, and
    // does the wreck end ON the terrain (not hanging mid-air)?
    let fallT = 0;
    let finishAt = -1;
    let finishY = 0;
    let finishFloor = 0;
    {
      let simT = 0;
      while (simT < 120 && ac.alive) {
        const r = frame(ac, 1, () => {
          ac.thrTarget = 0;
        });
        simT += r.simT;
        fallT = simT;
        if (!ac.alive) {
          finishAt = simT;
          finishY = ac.pos.y;
          finishFloor = floorAt(ac.pos.x, ac.pos.z);
        }
      }
    }
    console.log(
      `  [${key}] shot down at ${fallY.toFixed(0)} m / ${fallKt.toFixed(0)} kt; ` +
        `finishCrash at ${finishAt < 0 ? 'NEVER in 120 s' : finishAt.toFixed(1) + ' s'} ` +
        `wreck y=${finishY.toFixed(1)} floor=${finishFloor.toFixed(1)} ` +
        `(${(finishY - finishFloor).toFixed(1)} m above floor) dying=${ac.dying ? 1 : 0} alive=${ac.alive}`,
    );
    check(
      `S1[${key}]: the dying fall finishes (finishCrash fires, jet dead)`,
      finishAt >= 0,
      finishAt < 0 ? `still alive+dying after 120 s at y=${ac.pos.y.toFixed(0)}` : '',
    );
    check(
      `S1[${key}]: the wreck ends on the terrain (within 15 m of the floor)`,
      finishAt >= 0 && finishY - finishFloor < 15,
      `wreck ${(finishY - finishFloor).toFixed(1)} m above floor`,
    );
    // Now the respawn half: runway Alpha, throttle to 100% — the
    // fresh life must come alive and move, not re-crash instantly
    // (stale pendingContact from the wreck's impact) and not freeze.
    const pendingBefore = (ac as unknown as { pendingContact: unknown }).pendingContact;
    runwayRespawn(ac, 0);
    const r = frame(ac, 30, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
    });
    console.log(
      `  [${key}] respawn Alpha: pendingContact before respawn=${pendingBefore === null ? 'null' : 'STALE (non-null)'} ` +
        `alive=${ac.alive} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
        `crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
        `frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) : 'never'}`,
    );
    if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
    check(
      `S1[${key}]: respawned jet does not instantly re-crash (stale-contact kill)`,
      !(r.crashedAt >= 0 && r.crashedAt < 5),
      r.crashedAt >= 0 ? `crashed at ${r.crashedAt.toFixed(1)}s reason=${r.crashReason}` : '',
    );
    check(
      `S1[${key}]: respawned jet moves at 100% throttle (no freeze)`,
      r.frozenAt < 0 && ac.ias * MS2KT > 20,
      r.frozenDetail || `ias=${(ac.ias * MS2KT).toFixed(1)}kt`,
    );
    // Deck respawn after a second dying fall.
    ac.startDying('shot down');
    {
      let simT = 0;
      while (simT < 120 && ac.alive) {
        const rr = frame(ac, 1, () => {
          ac.thrTarget = 0;
        });
        simT += rr.simT;
      }
    }
    const spawn = deckRespawn(ac);
    const r2 = frame(ac, 30, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
    });
    console.log(
      `  [${key}] second fall -> deck respawn carrier ${spawn.carrierIdx}: alive=${ac.alive} ` +
        `ias=${(ac.ias * MS2KT).toFixed(0)}kt crashedAt=${r2.crashedAt >= 0 ? r2.crashedAt.toFixed(1) + 's/' + r2.crashReason : 'never'} ` +
        `frozenAt=${r2.frozenAt >= 0 ? r2.frozenAt.toFixed(1) : 'never'}`,
    );
    if (r2.frozenAt >= 0) console.log(`  FROZEN: ${r2.frozenDetail}`);
    check(
      `S1[${key}]: deck respawn after a dying fall moves at 100% throttle (no freeze)`,
      r2.frozenAt < 0 && ac.ias * MS2KT > 20,
      r2.frozenDetail || `ias=${(ac.ias * MS2KT).toFixed(1)}kt`,
    );
    disposeJet(ac);
  }

  // ---- S2. Respawn on a dying / sinking carrier. ----
  console.log('--- S2. Respawn on a dying carrier / ride a dying carrier down ---');
  {
    // S2a: kill carrier 0's pods through the real API; respawnPlayer's
    // liveCarrier test (as written) still picks it because alive is
    // still true while it dies. Measure what the jet does there.
    resetCarriers();
    driveCarriers(60);
    const c0 = carriers[0]!;
    const died = killCarrier(c0);
    console.log(
      `  carrier 0 pods killed: dying=${c0.dying} alive=${c0.alive} ` +
        `deckHeightAt(spawn pt)=${c0.deckHeightAt(c0.pos.x, c0.pos.z)} deckTopY=${c0.deckTopY.toFixed(1)}`,
    );
    check('S2a: killing all 4 pods puts the carrier into dying', died, `dying=${c0.dying}`);
    const ac = await makeJet('f35a');
    ac.crash('test kill');
    const spawn = deckRespawn(ac);
    const r = frame(ac, 60, (t) => {
      ac.thrTarget = Math.min(1, t / 2);
    });
    console.log(
      `  S2a deck respawn picked carrier ${spawn.carrierIdx} (dying=${carriers[spawn.carrierIdx]?.dying} alive=${carriers[spawn.carrierIdx]?.alive}): ` +
        `jet alive=${ac.alive} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
        `pos y=${ac.pos.y.toFixed(1)} floor=${floorAt(ac.pos.x, ac.pos.z).toFixed(1)} ` +
        `crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
        `frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) : 'never'}`,
    );
    if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
    check(
      'S2a: a deck respawn onto a dying carrier never freezes mid-air',
      r.frozenAt < 0,
      r.frozenDetail,
    );
    check(
      'S2a: any crash after a dying-carrier deck respawn is on the terrain (wreck within 15 m of the floor)',
      r.crashedAt < 0 || r.crashY - r.crashFloor < 15,
      r.crashedAt >= 0 ? `wreck ${(r.crashY - r.crashFloor).toFixed(1)} m above floor` : '',
    );
    disposeJet(ac);

    // S2b: the selection itself. respawnPlayer must NOT pick a dying
    // carrier — its deck is already gone (deckHeightAt -Infinity)
    // while alive stays true. Read the liveCarrier line as written.
    const liveCarrierLine = mainSrc.match(
      /const liveCarrier = carriers\.find\([^)]*\)[^;]*;/,
    )?.[0] ?? '';
    console.log(`  S2b liveCarrier selection as written: ${liveCarrierLine}`);
    check(
      'S2b: respawnPlayer excludes a DYING carrier from the deck-respawn choice (falls back to Alpha)',
      liveCarrierLine.includes('!c.dying') || liveCarrierLine.includes('!dying'),
      `selection is: ${liveCarrierLine}`,
    );

    // S2c: parked on the deck when the carrier starts dying — ride
    // it down. The jet must end on the terrain (alive on a surface
    // or crashed on it), never frozen mid-air.
    resetCarriers();
    driveCarriers(30);
    const ac2 = await makeJet('f35a');
    ac2.crash('test kill');
    deckRespawn(ac2);
    // Settle on the deck 5 sim-s at 0% throttle.
    frame(ac2, 5, () => {
      ac2.thrTarget = 0;
    });
    const onDeckAtKill = ac2.onDeck;
    const c0b = carriers[0]!;
    const died2 = killCarrier(c0b);
    const r2 = frame(ac2, 90, (t) => {
      ac2.thrTarget = 0;
    });
    console.log(
      `  S2c parked on deck (onDeck=${onDeckAtKill}) when carrier starts dying (dying=${died2}): ` +
        `jet alive=${ac2.alive} onDeck=${ac2.onDeck} surf=${ac2.onSurface ? 1 : 0} ` +
        `y=${ac2.pos.y.toFixed(1)} floor=${floorAt(ac2.pos.x, ac2.pos.z).toFixed(1)} ` +
        `carrier alive=${c0b.alive} dying=${c0b.dying} ` +
        `crashedAt=${r2.crashedAt >= 0 ? r2.crashedAt.toFixed(1) + 's/' + r2.crashReason : 'never'} ` +
        `frozenAt=${r2.frozenAt >= 0 ? r2.frozenAt.toFixed(1) : 'never'}`,
    );
    if (r2.frozenAt >= 0) console.log(`  FROZEN: ${r2.frozenDetail}`);
    check(
      'S2c: riding a dying carrier down never freezes the jet mid-air',
      r2.frozenAt < 0,
      r2.frozenDetail,
    );
    check(
      'S2c: after the carrier dies the jet ends on the terrain (alive on a surface, or crashed on it)',
      (!ac2.alive && r2.crashedAt >= 0 && r2.crashY - r2.crashFloor < 15) ||
        (ac2.alive && ac2.onSurface),
      `alive=${ac2.alive} onSurface=${ac2.onSurface} y=${ac2.pos.y.toFixed(1)} floor=${floorAt(ac2.pos.x, ac2.pos.z).toFixed(1)}`,
    );
    disposeJet(ac2);
  }

  // ---- S3. Respawn with a live floating-origin shift. ----
  console.log('--- S3. Respawn with a live origin shift (fly far, crash, respawn) ---');
  {
    resetCarriers();
    driveCarriers(30);
    origin.reset();
    const ac = await makeJet('f22');
    const alpha = RUNWAY_TABLE[0]!;
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.thrTarget = 1;
    // Fly east at speed until the origin has shifted (cumulative
    // shift > 2000 m), like a real round before a crash.
    let flew = 0;
    while (flew < 60 && origin.cumulativeShift.length() < 2000) {
      const r = frame(ac, 2, () => {
        ac.thrTarget = 1;
        ac.pitchIn = -0.2;
      });
      flew += r.simT;
      if (!ac.alive) break;
    }
    console.log(
      `  flew ${flew.toFixed(0)} sim-s: shift=(${origin.cumulativeShift.x.toFixed(0)}, ${origin.cumulativeShift.z.toFixed(0)}) ` +
        `pos=(${ac.pos.x.toFixed(0)}, ${ac.pos.y.toFixed(0)}, ${ac.pos.z.toFixed(0)}) alive=${ac.alive}`,
    );
    check(
      'S3: the pre-crash flight actually shifted the origin (> 2000 m)',
      origin.cumulativeShift.length() >= 2000,
      `shift=${origin.cumulativeShift.length().toFixed(0)} m`,
    );
    ac.crash('test kill');
    for (const idx of [2, 0] as const) {
      const rT = RUNWAY_TABLE[idx]!;
      runwayRespawn(ac, idx);
      // The spawn must land ON the runway box, in original-frame
      // coords (scene pos + shift == table coords).
      const ox = ac.pos.x + origin.cumulativeShift.x;
      const oz = ac.pos.z + origin.cumulativeShift.z;
      const sh = Math.sin((rT.hdg * Math.PI) / 180);
      const ch = Math.cos((rT.hdg * Math.PI) / 180);
      const along = (ox - rT.x) * sh + (oz - rT.z) * ch;
      const across = (ox - rT.x) * ch - (oz - rT.z) * sh;
      const r = frame(ac, 30, (t) => {
        ac.thrTarget = Math.min(1, t / 2);
      });
      console.log(
        `  S3[${rT.name}] spawn orig-frame along=${along.toFixed(1)}m across=${across.toFixed(1)}m ` +
          `alive=${ac.alive} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
          `crashedAt=${r.crashedAt >= 0 ? r.crashedAt.toFixed(1) + 's/' + r.crashReason : 'never'} ` +
          `frozenAt=${r.frozenAt >= 0 ? r.frozenAt.toFixed(1) : 'never'}`,
      );
      if (r.frozenAt >= 0) console.log(`  FROZEN: ${r.frozenDetail}`);
      check(
        `S3[${rT.name}]: respawn with a live shift lands on the runway (within the strip)`,
        Math.abs(along) < rT.len / 2 && Math.abs(across) < rT.width / 2,
        `along=${along.toFixed(1)} across=${across.toFixed(1)} (strip ${rT.len}x${rT.width})`,
      );
      check(
        `S3[${rT.name}]: respawned jet moves at 100% throttle (no freeze)`,
        r.frozenAt < 0 && ac.ias * MS2KT > 20,
        r.frozenDetail || `ias=${(ac.ias * MS2KT).toFixed(1)}kt`,
      );
      if (idx === 2) {
        // Crash again between the two respawns so the Alpha respawn
        // is a genuine post-crash respawn, not a live re-spawn.
        ac.crash('test kill');
      }
    }
    disposeJet(ac);
  }

  // ---- S4. The R machine, driven by the logic in src/main.ts. ----
  // The detector + R branch are replicated VERBATIM in structure
  // from main.ts, but the three decision points are read from the
  // source as written, so this section FAILS pre-fix and PASSES
  // post-fix:
  //   (a) does R open the picker for a DYING jet when spectating is
  //       false (picker cancelled once) — the dying dead-end;
  //   (b) does the unresponsive gate require thrTarget > 0.5 — a
  //       frozen jet whose W never raised the throttle (thrTarget 0)
  //       can never satisfy it;
  //   (c) does stuckTime advance by simDt (the frame's total sim
  //       time) or by one substep dt per frame (0.2x real time at
  //       4 fps — R needs 40 frames / 10 real s instead of 20).
  console.log('--- S4. The R machine (logic parsed from src/main.ts) ---');
  {
    // (a) R branch: find the cameraResetPressed handler and see
    // whether a dying jet reaches the picker without spectating.
    const rBranch = mainSrc.slice(
      mainSrc.indexOf('if (ev.cameraResetPressed)'),
      mainSrc.indexOf('if (ev.pausePressed)'),
    );
    const rHasDying =
      /else if \([^)]*dying[^)]*\)[\s\S]{0,120}showRespawnPicker/.test(rBranch) ||
      /p\.dying/.test(rBranch);
    console.log(`  S4a R branch has a dying-jet picker path: ${rHasDying}`);
    // (b) Unresponsive gate: the exact gate text.
    const gateMatch = mainSrc.match(/const unresponsive =[\s\S]*?;/);
    const gateText = gateMatch?.[0] ?? '';
    console.log(`  S4b unresponsive gate as written: ${gateText.replace(/\s+/g, ' ')}`);
    const gateNeedsThrottle = gateText.includes('thrTarget > 0.5');
    // (c) stuckTime increment in the detector.
    const detMatch = mainSrc.match(/if \(beached\) \{[\s\S]*?stuckTime \+= ([^;]+);/);
    const stuckIncr = detMatch?.[1]?.trim() ?? '?';
    console.log(`  S4c stuckTime increments by: ${stuckIncr} per frame`);
    const stuckUsesSimDt = stuckIncr === 'simDt';

    // Simulate the machine with the parsed flags, at live 4 fps
    // (rawDt 0.25 s/frame, simDt 0.10 s, substep dt 0.05 s).
    type MState = {
      alive: boolean;
      dying: boolean;
      spectating: boolean;
      respawnTimer: boolean;
      pickerOpen: boolean;
      thrTarget: number;
      vector: number;
      onSurface: boolean;
      movedPerFrame: number;
    };
    const runMachine = (
      st: MState,
      frames: number,
    ): { pickerAtFrame: number; realSeconds: number } => {
      let stuckTime = 0;
      let pickerAtFrame = -1;
      const dtSub = 0.05;
      const simDt = 0.1;
      const rawDt = 0.25;
      for (let f = 0; f < frames; f++) {
        // Detector (verbatim structure): unresponsive needs alive,
        // !dying, !onSurface, vector < 70, moved < 0.1 — plus the
        // throttle term iff the source still has it.
        const unresponsive =
          st.alive &&
          !st.dying &&
          !st.onSurface &&
          (!gateNeedsThrottle || st.thrTarget > 0.5) &&
          st.vector < 70 &&
          st.movedPerFrame < 0.1;
        const beached = unresponsive; // old vel gate never fires here
        if (beached) stuckTime += stuckUsesSimDt ? simDt : dtSub;
        else stuckTime = 0;
        // R pressed on this frame: the branch as written.
        let opensPicker = false;
        if (st.pickerOpen) opensPicker = true;
        else if (st.spectating || !st.alive) opensPicker = true;
        else if (stuckTime > 2) opensPicker = true;
        else if (rHasDying && st.dying) opensPicker = true;
        if (opensPicker && pickerAtFrame < 0) {
          pickerAtFrame = f;
        }
      }
      return {
        pickerAtFrame,
        realSeconds: pickerAtFrame < 0 ? -1 : (pickerAtFrame + 1) * rawDt,
      };
    };

    // S4a: dying jet, picker cancelled once (spectating false, no
    // timer). Pre-fix R never opens the picker; post-fix it must.
    {
      const m = runMachine(
        {
          alive: true,
          dying: true,
          spectating: false,
          respawnTimer: false,
          pickerOpen: false,
          thrTarget: 0,
          vector: 0,
          onSurface: false,
          movedPerFrame: 0,
        },
        240,
      );
      console.log(
        `  S4a dying + picker-cancelled: R opens picker at frame ${m.pickerAtFrame} ` +
          `(${m.realSeconds < 0 ? 'never in 60 s' : m.realSeconds.toFixed(1) + ' real s'})`,
      );
      check(
        'S4a: R opens the picker for a dying jet even after the picker was cancelled once',
        m.pickerAtFrame >= 0,
        m.pickerAtFrame < 0 ? 'R never opens the picker in 240 frames (60 real s)' : `picker at frame ${m.pickerAtFrame}`,
      );
    }
    // S4b: frozen alive jet, throttle never raised (thrTarget 0 —
    // W dead in the frozen state). Pre-fix the gate needs
    // thrTarget > 0.5, so it never fires; post-fix it must.
    {
      const m = runMachine(
        {
          alive: true,
          dying: false,
          spectating: false,
          respawnTimer: false,
          pickerOpen: false,
          thrTarget: 0,
          vector: 0,
          onSurface: false,
          movedPerFrame: 0,
        },
        240,
      );
      console.log(
        `  S4b frozen alive, thrTarget=0: R opens picker at frame ${m.pickerAtFrame} ` +
          `(${m.realSeconds < 0 ? 'never in 60 s' : m.realSeconds.toFixed(1) + ' real s'})`,
      );
      check(
        'S4b: R opens the picker for a frozen jet even when the throttle never rose (thrTarget 0)',
        m.pickerAtFrame >= 0,
        m.pickerAtFrame < 0 ? 'gate never fires with thrTarget 0 in 240 frames (60 real s)' : `picker at frame ${m.pickerAtFrame} (${m.realSeconds.toFixed(1)} real s)`,
      );
    }
    // S4c: frozen alive jet, throttle raised (thrTarget 1) — the
    // gate fires, but HOW FAST at 4 fps? Pre-fix: stuckTime +=
    // substep dt (0.05/frame) = 40 frames = 10 real s. Post-fix
    // (simDt): 20 frames = 5 real s. The live checker waited
    // 30-40 s, so the rate alone is not the whole story — but R
    // must be available inside 6 real s at 4 fps.
    {
      const m = runMachine(
        {
          alive: true,
          dying: false,
          spectating: false,
          respawnTimer: false,
          pickerOpen: false,
          thrTarget: 1,
          vector: 0,
          onSurface: false,
          movedPerFrame: 0,
        },
        240,
      );
      console.log(
        `  S4c frozen alive, thrTarget=1: R opens picker at frame ${m.pickerAtFrame} ` +
          `(${m.realSeconds < 0 ? 'never' : m.realSeconds.toFixed(1) + ' real s'})`,
      );
      check(
        'S4c: R opens the picker for a frozen jet within 6 real seconds at 4 fps',
        m.pickerAtFrame >= 0 && m.realSeconds <= 6,
        m.pickerAtFrame < 0
          ? 'never in 60 s'
          : `took ${m.realSeconds.toFixed(1)} real s`,
      );
    }
    // S4d: the gate must NOT fire for a healthy jet (false-positive
    // guard): a jet accelerating at 100% moves > 0.1 m/frame from
    // the second frame on.
    {
      const m = runMachine(
        {
          alive: true,
          dying: false,
          spectating: false,
          respawnTimer: false,
          pickerOpen: false,
          thrTarget: 1,
          vector: 0,
          onSurface: false,
          movedPerFrame: 5,
        },
        240,
      );
      console.log(`  S4d healthy moving jet: R opens picker at frame ${m.pickerAtFrame} (never = -1)`);
      check(
        'S4d: R does NOT open the picker for a healthy moving jet (no false positive)',
        m.pickerAtFrame < 0,
        `picker opened at frame ${m.pickerAtFrame}`,
      );
    }
  }

  console.log(failures === 0 ? '\nALL PASS17-FROZEN CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
