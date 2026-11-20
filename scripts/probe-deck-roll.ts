// probe-deck-roll.ts — pass 11, item 1 (deck wall): the jet must be
// able to travel the FULL deck length for takeoff AND landing.
// Live report (v4.5): "when i try to take off from the heli carrier i
// hit a wall and cannot move further" — all four jets (the earlier
// "F-18 works" report was withdrawn by Nicholas).
//
// Takeoff: the REAL startRound deck spawn (85 m aft of deck centre,
// facing the bow, inheriting deck-point velocity), brake off, 100%
// throttle, immediate roll — no 180 s park first (probe-deck-ride-long
// parks first, so it never tests the live spawn-then-roll path). A
// rotation pull at the rotation gate. Asserts liftoff before the bow
// end, and no wall-stop anywhere along the run. A second no-pull
// variant rolls to the bow end: it must roll OFF the end (or lift),
// never stop dead ON the deck — a stop on the deck is the wall.
//
// Landing: scripted final onto the deck (hook down), touchdown in the
// wires, trap + brakes to a stop ON the deck — no wall, no fall-through.
//
// Frame loop: the full live loop (carrier update, world.step, drain,
// aircraft step, live variable-dt pattern, live wind, origin shifts).
//
// Usage: node_modules/.bin/esbuild scripts/probe-deck-roll.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-deck-roll.mjs
//   && node /tmp/probe-deck-roll.mjs

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
  HelicarrierController,
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
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

const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
const WIND = new THREE.Vector3(3, 0, 0);
const DECK_SPAWN_AFT_M = 85;
const clamp = (v: number, lo: number, hi: number): number =>
  Math.max(lo, Math.min(hi, v));

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
  const floatingOrigin = new FloatingOrigin(2000);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
    waypoints: CARRIER_WAYPOINTS,
    startIndex: 0,
  });
  await carrier.build();
  const dc = carrier.deckColliderRef;
  if (dc) owners.set(dc.handle, 'deck');
  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.alive;
    },
    // Pass 11: the deck-top Y, so the auto-gear can measure AGL
    // above the deck (see FlightPhysicsController.autoGear).
    get deckTopY() {
      return carrier.deckTopY;
    },
    deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
    velocity: () => carrier.vel.clone(),
    velocityAt: (x, z) => {
      const rx = x - carrier.pos.x;
      const rz = z - carrier.pos.z;
      const w = carrier.yawRate;
      return new THREE.Vector3(carrier.vel.x + w * rz, carrier.vel.y, carrier.vel.z - w * rx);
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: () => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };

  // Manual world -> carrier-local (carrier.localPosition returns null
  // off the deck footprint). R_y(yaw): world = R*local + pos.
  const localOfPos = (x: number, z: number): { x: number; z: number } => {
    const dx = x - carrier.pos.x;
    const dz = z - carrier.pos.z;
    const c = Math.cos(carrier.yaw);
    const s = Math.sin(carrier.yaw);
    return { x: c * dx - s * dz, z: s * dx + c * dz };
  };
  // Contact log: which collider the jet touches, and whether it was
  // on the deck surface at the time (a deck contact while NOT on the
  // deck surface = hitting a deck side/end face = the wall).
  let contacts: Array<{ t: number; other: string; onDeck: boolean; lz: number; lx: number }> = [];
  let simT = 0;
  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || hit !== ac) return;
      const other = a === 'deck' || b === 'deck' ? 'deck' : a === 'terrain' || b === 'terrain' ? 'terrain' : 'other';
      const loc = localOfPos(ac.pos.x, ac.pos.z);
      contacts.push({ t: simT, other, onDeck: ac.onDeck, lz: loc.z, lx: loc.x });
      hit.notifyContact(hit.vel.y, other === 'deck');
    });
  };
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, WIND, null);
  };
  const stepFor = (ac: Aircraft, seconds: number, perStep?: (t: number) => void): void => {
    let t = 0;
    let pi = 0;
    while (t < seconds) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) {
        substep(ac, dt);
        simT += dt;
        perStep?.(simT);
      }
      t += rawDt;
    }
  };

  console.log(
    `deck rects: ${carrier.deckRects.map((r) => `x[${(r.cx - r.hx).toFixed(1)},${(r.cx + r.hx).toFixed(1)}] z[${(r.cz - r.hz).toFixed(1)},${(r.cz + r.hz).toFixed(1)}]`).join(' | ')} deckLocalY=${carrier.deckLocalY.toFixed(2)}`,
  );

  const spawnOnDeck = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
    ac.deck = deckProvider;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
    // startRound deck spawn, verbatim (main.ts).
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    return ac;
  };
  const localOf = (ac: Aircraft): { x: number; z: number } =>
    localOfPos(ac.pos.x, ac.pos.z);
  const deckRelSpeed = (ac: Aircraft): number => {
    const dv = deckProvider.velocityAt(ac.pos.x, ac.pos.z);
    return Math.hypot(ac.vel.x - dv.x, ac.vel.z - dv.z);
  };

  for (const key of ['fa18', 'f22', 'f35a', 'f15e'] as const) {
    const spec = SPECS[key]!;
    // --- (A) takeoff with a rotation pull at the gate ---
    {
      contacts = [];
      simT = 0;
      const ac = spawnOnDeck(key);
      console.log(`\n--- ${key} takeoff (modelStatus=${ac.modelStatus}) spawn local=${JSON.stringify(localOf(ac))} deckTopY=${carrier.deckTopY.toFixed(1)} carrierYaw=${((carrier.yaw * 180) / Math.PI).toFixed(1)} ---`);
      ac.brake = false;
      ac.thrTarget = 1;
      ac.gear = true;
      let maxLz = -Infinity;
      let liftoffLz: number | null = null;
      let liftoffT: number | null = null;
      let wallStop: { t: number; lx: number; lz: number } | null = null;
      let slowSince: number | null = null;
      let wasRolling = false;
      let logT = 0;
      stepFor(ac, 30, (t) => {
        const loc = localOf(ac);
        maxLz = Math.max(maxLz, loc.z);
        const sp = deckRelSpeed(ac);
        if (sp > 10) wasRolling = true;
        if (t - logT >= 1) {
          logT = t;
          console.log(
            `  t=${t.toFixed(1)} local=(${loc.x.toFixed(1)}, ${loc.z.toFixed(1)}) sp=${sp.toFixed(1)}m/s gs=${Math.hypot(ac.vel.x, ac.vel.z).toFixed(1)} agl=${(ac.pos.y - ac.floorY).toFixed(1)} onDeck=${ac.onDeck} alive=${ac.alive}`,
          );
        }
        // Rotation pull at the gate (stallSpeed x 0.85), like the player.
        if (sp > spec.stallSpeed * 0.85 && liftoffLz === null) ac.pitchIn = -1;
        // Wheels-off: the jet is no longer touching the surface
        // (onSurface false = risen past the 0.5 m deck tolerance),
        // not the old agl > 10 m lag, which read ~2 m past the bow.
        if (liftoffLz === null && !ac.grounded && !ac.onSurface) {
          liftoffLz = loc.z;
          liftoffT = t;
        }
        // Wall signature: the jet was rolling, then its
        // deck-relative speed collapses to ~0 while on the deck,
        // throttle 100%, alive, not airborne, for > 1 s.
        if (wasRolling && ac.onDeck && ac.alive && sp < 2 && liftoffLz === null) {
          if (slowSince === null) slowSince = t;
          if (t - slowSince > 1 && !wallStop) wallStop = { t, lx: loc.x, lz: loc.z };
        } else {
          slowSince = null;
        }
        if (liftoffLz !== null || wallStop || !ac.alive) return;
      });
      // Stop stepping once lifted off / walled / dead — stepFor ran the
      // full 30 s; report from the recorded values instead.
      const deckContactsOffSurface = contacts.filter((c) => c.other === 'deck' && !c.onDeck);
      console.log(
        `  result: maxLocalZ=${maxLz.toFixed(1)} liftoff=${liftoffLz === null ? 'NONE' : `localZ=${liftoffLz.toFixed(1)} t=${liftoffT!.toFixed(1)}s`} ` +
          `wallStop=${wallStop ? `t=${wallStop.t.toFixed(1)} local=(${wallStop.lx.toFixed(1)}, ${wallStop.lz.toFixed(1)})` : 'none'} ` +
          `alive=${ac.alive} deckContactsWhileOffSurface=${deckContactsOffSurface.length}`,
      );
      check(`${key} takeoff: lifts off at/just past the bow end (localZ < 170)`, liftoffLz !== null && liftoffLz < 170, `liftoffLocalZ=${liftoffLz === null ? 'NONE' : liftoffLz.toFixed(1)} maxLocalZ=${maxLz.toFixed(1)}`);
      check(`${key} takeoff: no wall-stop on the deck`, wallStop === null, wallStop ? `stopped at local=(${wallStop.lx.toFixed(1)}, ${wallStop.lz.toFixed(1)})` : '');
      check(`${key} takeoff: no deck side/end-face contact while off the surface`, deckContactsOffSurface.length === 0, `${deckContactsOffSurface.length} contacts, first=${deckContactsOffSurface[0] ? `t=${deckContactsOffSurface[0].t.toFixed(1)} local=(${deckContactsOffSurface[0].lx.toFixed(1)}, ${deckContactsOffSurface[0].lz.toFixed(1)})` : 'none'}`);
      ac.dispose();
      owners.clear();
      owners.set(terrainCollider.handle, 'terrain');
    }
    // --- (B) no-pull roll to the bow end: must roll OFF, never stop ---
    {
      contacts = [];
      simT = 0;
      const ac = spawnOnDeck(key);
      ac.brake = false;
      ac.thrTarget = 1;
      ac.gear = true;
      let maxLz = -Infinity;
      let stoppedOnDeck = false;
      let rolledOff = false;
      let slowSince: number | null = null;
      stepFor(ac, 25, (t) => {
        const loc = localOf(ac);
        maxLz = Math.max(maxLz, loc.z);
        const sp = deckRelSpeed(ac);
        if (loc.z > 152.5) rolledOff = true;
        if (ac.onDeck && ac.alive && sp < 2 && !rolledOff) {
          if (slowSince === null) slowSince = t;
          if (t - slowSince > 1) stoppedOnDeck = true;
        } else {
          slowSince = null;
        }
      });
      console.log(`  ${key} no-pull: maxLocalZ=${maxLz.toFixed(1)} rolledOffBow=${rolledOff} stoppedOnDeck=${stoppedOnDeck} alive=${ac.alive}`);
      check(`${key} no-pull roll: never stops dead ON the deck (rolls off the bow or lifts)`, !stoppedOnDeck, `maxLocalZ=${maxLz.toFixed(1)}`);
      ac.dispose();
      owners.clear();
      owners.set(terrainCollider.handle, 'terrain');
    }
  }

  // --- Landing: scripted final, hook down, trap + brakes to a stop ---
  for (const key of ['fa18', 'f22'] as const) {
    const spec = SPECS[key]!;
    contacts = [];
    simT = 0;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
    ac.deck = deckProvider;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
    // Final: 500 m astern of the carrier, ON the 3.5 deg slope
    // (deckTop+27.9 at 500 m out — starting on the slope, not 5 m
    // above it, so there is no initial dive transient), 42 m/s
    // deck-relative, heading = carrier yaw, hook + gear down,
    // flaps 30.
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * 500;
    const sz = carrier.pos.z - fwd.z * 500;
    const dv0 = deckProvider.velocityAt(sx, sz);
    // Per-jet approach speed (deck-relative): 0.8x stall speed,
    // floored at 46 m/s — the F-18 at 46 is only 1.3x its 66 m/s
    // stall, so a pitch-up bled it to 9.3 m/s (below stall) onto
    // the stern edge. F-18 52.8 m/s, F-22 46 m/s.
    const vApp = Math.max(46, spec.stallSpeed * 0.8);
    ac.resetAt(sx, carrier.deckTopY + 32, sz, (carrier.yaw * 180) / Math.PI);
    ac.setAirspeed(Math.hypot(dv0.x, dv0.z) + vApp);
    ac.body.setLinvel({ x: dv0.x + fwd.x * vApp, y: 0, z: dv0.z + fwd.z * vApp }, true);
    ac.vel.set(dv0.x + fwd.x * vApp, 0, dv0.z + fwd.z * vApp);
    ac.thrTarget = 0.65;
    ac.gear = true;
    ac.hook = true;
    ac.flaps = 30;
    console.log(`\n--- ${key} landing: final 500m astern, deckTop+32, 46m/s deck-relative, hook down ---`);
    let touchedDown = false;
    let touchdownLz: number | null = null;
    let pitchInt = 0;
    let prevT = 0;
    // Trim AoA for level flight at the approach airspeed
    // (deck velocity + vApp — the F-18's vApp 52.8 gives 92.8 m/s,
    // not the 86 the first version hardcoded; the stale 5.5 deg
    // trim made it float 8.6 m over the deck and never touch
    // down), from the spec: CL_needed = 2W/(rho V^2 S), then the
    // linear CL region (CL_0 0.15, slope 0.08/deg, plus the flap
    // lift increment FLAP_CL 0.35 at flaps 30). NOT read from the
    // live state on step 1 — the jet is placed untrimmed (pitch
    // 0, AoA ~0) and would read ~0 deg. F-18 ~3.9 deg, F-22 ~2.4 deg.
    const vAirApp = Math.hypot(dv0.x, dv0.z) + vApp;
    const clNeed =
      (2 * spec.mass * 9.81) / (1.225 * vAirApp * vAirApp * spec.wingArea);
    const aoaTrim = Math.max(
      0,
      Math.min(15, (clNeed - 0.15 - 0.35) / 0.08),
    );
    let stopped = false;
    let stoppedLz: number | null = null;
    let fellThrough = false;
    let minClearance = Infinity;
    let logT = 0;
    stepFor(ac, 60, (t) => {
      const loc = localOf(ac);
      const deckY = carrier.deckHeightAt(ac.pos.x, ac.pos.z);
      if (deckY > -Infinity) minClearance = Math.min(minClearance, ac.pos.y - deckY);
      if (t - logT >= 2) {
        logT = t;
        const dvLog = deckProvider.velocityAt(ac.pos.x, ac.pos.z);
        const eul = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
        console.log(
          `  t=${t.toFixed(1)} local=(${loc.x.toFixed(1)}, ${loc.z.toFixed(1)}) alt=${(ac.pos.y - carrier.deckTopY).toFixed(1)} deckY=${carrier.deckTopY.toFixed(0)} dvY=${dvLog.y.toFixed(1)} sp=${deckRelSpeed(ac).toFixed(1)} pitch=${(eul.x * 57.3).toFixed(1)} aoa=${ac.aoa.toFixed(1)} vsi=${ac.vsi.toFixed(1)} stalled=${ac.stalled} onDeck=${ac.onDeck} alive=${ac.alive}`,
        );
      }
      if (!touchedDown && ac.onDeck) {
        touchedDown = true;
        touchdownLz = loc.z;
        console.log(`  TOUCHDOWN t=${t.toFixed(1)} local=(${loc.x.toFixed(1)}, ${loc.z.toFixed(1)}) sp=${deckRelSpeed(ac).toFixed(1)}`);
      }
      if (touchedDown && !stopped) {
        ac.brake = true;
        ac.thrTarget = 0;
        ac.pitchIn = 0;
        if (deckRelSpeed(ac) < 1) {
          stopped = true;
          stoppedLz = loc.z;
        }
      } else if (!touchedDown) {
        // Approach: 46 m/s deck-relative (~86 m/s ground, ~1.3x
        // the F-18's 66 m/s stall speed) with a 0.5 throttle
        // floor — a 42 m/s target with a 0.3 floor let the
        // throttle cut while the jet was 8 m/s fast, and flaps-30
        // + gear drag then bled the speed to 3.8 m/s in 2 s
        // (below stall) and the jet sank under the deck. The
        // vertical loop is a two-term PD: a small altitude-error
        // term on the commanded vertical speed (capped descent
        // 3.5 m/s) and a stronger gain on the vertical-speed
        // error, so a descent is arrested before it overshoots
        // the slope. (pitchIn > 0 = nose down.)
        // Glideslope: 3.5 deg to local z=-75 (wire zone centre) at
        // deckTop+2. The slope RISES astern of the wires: at a
        // distance d astern, altitude = deckTop + 2 + d * tan(3.5).
        // (An earlier version had the sign backwards — (loc.z+75)
        // — which made the target a flat deckTop+2 for the whole
        // approach astern, so the jet dived 30 m below the deck
        // before the wires.)
        const slopeAlt = carrier.deckTopY + 2 + Math.max(0, (-75 - loc.z) * 0.061);
        const altErr = slopeAlt - ac.pos.y;
        // The deck itself climbs/descends with the terrain
        // (terrain-following, up to +15/-10 m/s): the commanded
        // vertical speed must include the deck's own vertical
        // velocity (feed-forward), or a climbing deck outruns the
        // jet — the earlier cap of +1.0 m/s climb let a climbing
        // deck leave the jet 30-50 m below it mid-approach.
        const dvY = deckProvider.velocityAt(ac.pos.x, ac.pos.z).y;
        const vsDes = clamp(-vApp * 0.061 + altErr * 0.35 + dvY, -4.5, 15.0);
        // Pitch-attitude hold (not a P loop on vertical speed —
        // that went UNSTABLE: a violent phugoid, pitch swinging
        // -25 deg to +10 deg, AoA -9 deg to +29 deg, vsi to
        // -25 m/s, and the jet dived under the deck). Desired
        // flight-path angle from the altitude error and the deck's
        // vertical velocity; desired pitch = that angle + the
        // jet's trim AoA (measured in level flight at the approach
        // speed on the first step); pitch command proportional to
        // the pitch error — a first-order loop, no overshoot.
        // (pitchIn < 0 = nose up; Euler YXZ x > 0 = nose down.)
        const airspeed = Math.max(20, Math.hypot(ac.vel.x, ac.vel.z));
        const gammaDes = Math.atan2(vsDes, airspeed);
        // NO flare: a carrier landing is flown onto the deck at
        // the glideslope rate (-2.6 m/s), not flared like a runway
        // landing. The earlier flare (below 4 m, -1.2 m/s) floated
        // the F-22 86 m past the wires (touchdown +11.4), past the
        // trap, and the brake alone could not stop it in the
        // remaining deck — it ran off the bow and died. The jet
        // must touch down IN the wire zone (-45..-105) for the
        // trap to catch it.
        const gammaFlare = gammaDes;
        // Euler YXZ x > 0 is nose DOWN, so a nose-up attitude is a
        // NEGATIVE Euler x: pitchDes = -(gamma + trimAoA). And the
        // loop drives TOWARD the target: pitchIn > 0 is nose down,
        // so pitchIn = (pitchDes - pitchNow) * gain. (Both signs
        // were backwards in the first version of this hold, and the
        // jet climbed 286 m over the deck instead of landing.)
        const pitchDes = -(gammaFlare + (aoaTrim * Math.PI) / 180);
        const pitchNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x;
        // PI attitude hold: a P-only loop has a steady-state trim
        // error — the BET jet sits at its natural trim pitch (the
        // F-22 froze at -5.0 deg nose-up, climbing at 3.1 m/s) and
        // the small P output cannot hold a non-trim attitude. The
        // integral term (clamped) eliminates that error.
        const pitchErr = pitchDes - pitchNow;
        const dtStep = t - prevT;
        prevT = t;
        pitchInt = clamp(pitchInt + pitchErr * dtStep * 0.8, -0.35, 0.35);
        ac.pitchIn = clamp(pitchErr * 4.0 + pitchInt, -0.7, 0.7);
        // Centreline: bank toward local x=0 (rollIn > 0 = right bank).
        const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
        ac.rollIn = clamp((0 - loc.x) * 0.03 - bankNow * 1.0, -0.7, 0.7);
        // Speed: hold vApp deck-relative. The throttle floor
        // is 0.15, not 0.5: at 0.5 the F-18 accelerates even at
        // minimum command (thrust 98 kN vs ~36 kN drag at 86 m/s)
        // and the jet ran away to 84 m/s deck-relative while
        // climbing. Stronger gain so the steady-state error is
        // ~3 m/s, not ~5.
        ac.thrTarget = clamp(0.5 + (vApp - deckRelSpeed(ac)) * 0.12, 0.15, 0.9);
      }
      if (deckY > -Infinity && ac.pos.y < deckY - 1) fellThrough = true;
      if (stopped || !ac.alive) return;
    });
    const deckContactsOffSurface = contacts.filter((c) => c.other === 'deck' && !c.onDeck);
    console.log(
      `  result: touchedDown=${touchedDown} touchdownLz=${touchdownLz === null ? 'NONE' : touchdownLz.toFixed(1)} ` +
        `stopped=${stopped} stoppedLz=${stoppedLz === null ? 'NONE' : stoppedLz.toFixed(1)} fellThrough=${fellThrough} ` +
        `minClearanceOverDeck=${minClearance === Infinity ? 'n/a' : minClearance.toFixed(2)} deckContactsWhileOffSurface=${deckContactsOffSurface.length} alive=${ac.alive}`,
    );
    check(`${key} landing: touches down on the deck`, touchedDown && touchdownLz !== null && touchdownLz > -128 && touchdownLz < 152, `touchdownLz=${touchdownLz === null ? 'NONE' : touchdownLz.toFixed(1)} (wire zone -105..-45)`);
    check(`${key} landing: rolls out to a stop ON the deck`, stopped && stoppedLz !== null && stoppedLz > -128 && stoppedLz < 152, `stoppedLz=${stoppedLz === null ? 'NONE' : stoppedLz.toFixed(1)}`);
    check(`${key} landing: never falls through the deck`, !fellThrough, `minClearance=${minClearance === Infinity ? 'n/a' : minClearance.toFixed(2)}`);
    check(`${key} landing: no deck side/end-face contact while off the surface`, deckContactsOffSurface.length === 0, `${deckContactsOffSurface.length} contacts`);
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
