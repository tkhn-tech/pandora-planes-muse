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
// Pass 12, Worker F additions:
// (C) PARKED HEADING: a jet parked (brake, 0% throttle) on the deck for
// 180 s through the waypoint-0 turn must keep its DECK-RELATIVE heading
// (jet heading minus carrier yaw) — the live v4.6.1 check saw the HUD
// heading swing 090 -> 081 -> 341 while parked, which is the carrier
// itself turning 90 -> 341; the question is whether the jet follows it.
// (D) RETURN + TRAP: takeoff, fly 3 km out, navigate back to the
// moving carrier on bearing/distance alone (the HUD values the pass-12
// patch adds), and land in the wires — the full live return path, with
// a floating-origin shift during the transit.
// (E) CARRIER CAMERA: the live chase (30 m behind, +10.2 m) and
// free-orbit (30 m, pitch 0.32) sightlines at a deck spawn, raycast
// against the REAL carrier GLB meshes — the v4.6.1 live check saw the
// jet invisible at deck spawn, the frame filled by carrier structure.
// (F) CARRIER BEARING: the bearing/distance values the HUD patch will
// show, computed from live state at spawn / mid-transit / on final —
// bearing error vs the true jet->carrier direction, distance error vs
// the true range.
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
  orbitCameraPositionDeckAware,
  deckChaseCameraPosition,
  deckOrbitYaw,
} from '../src/camera/OrbitCamera';
import {
  generateHeights,
  heightfieldHeightAt,
  fbm,
  plateauWeight,
  runwayBlend,
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

  // --- (C) Parked heading through the waypoint-0 turn ---
  // The carrier starts heading east (yaw 90 deg) toward waypoint 0,
  // 4,900 m away: it reaches it at ~122 s, then turns ~120 deg toward
  // waypoint 1 (yaw 90 -> 341 deg) at up to 0.12 rad/s. A parked jet
  // must ride the turn with the deck: its deck-relative heading
  // (jet heading minus carrier yaw, wrapped) must stay ~0. The live
  // v4.6.1 check read the WORLD heading swinging 090 -> 081 -> 341
  // while parked — that swing is the carrier's own turn, and is only
  // a bug if the deck-relative heading moves with it.
  const wrapDeg = (d: number): number => {
    let w = d % 360;
    if (w > 180) w -= 360;
    if (w < -180) w += 360;
    return w;
  };
  const jetHdgDeg = (ac: Aircraft): number =>
    ((Math.atan2(ac.fwd().x, ac.fwd().z) * 180) / Math.PI + 360) % 360;
  for (const key of ['fa18', 'f22'] as const) {
    contacts = [];
    simT = 0;
    const ac = spawnOnDeck(key);
    ac.brake = true;
    ac.thrTarget = 0;
    ac.throttle = 0;
    console.log(`\n--- ${key} parked heading: 180 s on deck through the waypoint-0 turn ---`);
    let maxRelHdg = 0;
    let maxRelHdgAt = 0;
    let offDeckAt: number | null = null;
    let logT = 0;
    stepFor(ac, 180, (t) => {
      const rel = wrapDeg(jetHdgDeg(ac) - (carrier.yaw * 180) / Math.PI);
      if (Math.abs(rel) > maxRelHdg) {
        maxRelHdg = Math.abs(rel);
        maxRelHdgAt = t;
      }
      if (!ac.onDeck && offDeckAt === null) offDeckAt = t;
      if (t - logT >= 10) {
        logT = t;
        console.log(
          `  t=${t.toFixed(0)} jetHdg=${jetHdgDeg(ac).toFixed(1)} carrierYaw=${(((((carrier.yaw * 180) / Math.PI) % 360) + 360) % 360).toFixed(1)} relHdg=${rel.toFixed(2)} onDeck=${ac.onDeck} onSurf=${ac.onSurface} local=${JSON.stringify(localOf(ac))} alive=${ac.alive}`,
        );
      }
    });
    console.log(
      `  result: max |deck-relative heading|=${maxRelHdg.toFixed(2)} deg at t=${maxRelHdgAt.toFixed(1)}s offDeckAt=${offDeckAt === null ? 'never' : offDeckAt.toFixed(1) + 's'} alive=${ac.alive} onDeck=${ac.onDeck}`,
    );
    check(
      `${key} parked: deck-relative heading holds through the carrier turn (< 5 deg)`,
      maxRelHdg < 5,
      `max |relHdg|=${maxRelHdg.toFixed(2)} deg at t=${maxRelHdgAt.toFixed(1)}s`,
    );
    check(
      `${key} parked: stays on the deck for the whole 180 s`,
      offDeckAt === null && ac.alive,
      offDeckAt === null ? 'onDeck throughout' : `left deck at t=${offDeckAt.toFixed(1)}s`,
    );
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  }

  // --- (E) Carrier camera sightlines at a deck spawn (legacy formulas) ---
  // Raycast (real triangle hits, not bboxes — the carrier is rotated
  // 90 deg, so world AABBs overstate every structure) from the camera
  // position the live code computes to the jet, against the REAL
  // carrier GLB group. The live v4.6.1 chase is want = jet - fwd*30 +
  // up*10.2, lookAt(jet); the live free orbit is orbitCameraPosition
  // (jet, yaw 0, pitch 0.32, dist 30) — replicated here verbatim from
  // main.ts updateCamera so the check tests the LIVE framing, not a
  // re-implementation.
  const raycaster = new THREE.Raycaster();
  const sightlineBlockedBy = (from: THREE.Vector3, to: THREE.Vector3): string | null => {
    const dir = to.clone().sub(from);
    const len = dir.length();
    dir.normalize();
    raycaster.set(from, dir);
    raycaster.far = len - 0.5;
    const hits = raycaster.intersectObject(carrier.group, true);
    return hits.length > 0 ? (hits[0]!.object.name || '(unnamed)') + ` d=${hits[0]!.distance.toFixed(1)}m` : null;
  };
  {
    carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
    scene.updateMatrixWorld(true);
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const jet = new THREE.Vector3(
      carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M,
      carrier.deckTopY + 2.2,
      carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M,
    );
    const chase = jet.clone().addScaledVector(fwd, -30).add(new THREE.Vector3(0, 30 * 0.34, 0));
    const orbit0 = new THREE.Vector3(
      jet.x + Math.sin(0) * Math.cos(0.32) * 30,
      jet.y + Math.sin(0.32) * 30,
      jet.z + Math.cos(0) * Math.cos(0.32) * 30,
    );
    const chaseBlock = sightlineBlockedBy(chase, jet);
    const orbitBlock = sightlineBlockedBy(orbit0, jet);
    console.log(
      `\n--- carrier camera (legacy): jet=(${jet.x.toFixed(0)},${jet.y.toFixed(1)},${jet.z.toFixed(0)}) deckTopY=${carrier.deckTopY.toFixed(1)} ---`,
    );
    console.log(`  chase cam=(${chase.x.toFixed(0)},${chase.y.toFixed(1)},${chase.z.toFixed(0)}) blockedBy=${chaseBlock ?? 'none'}`);
    console.log(`  orbit cam=(${orbit0.x.toFixed(0)},${orbit0.y.toFixed(1)},${orbit0.z.toFixed(0)}) blockedBy=${orbitBlock ?? 'none'}`);
    // The two legacy checks are written as expect-BLOCKED: they
    // document the live v4.6.1 bug (jet invisible at deck spawn). If
    // the model ever changes so the legacy framing clears, these fail
    // on purpose — the bug is gone another way and the expectations
    // need updating. The FIXED checks below are the ones that must
    // stay green.
    check('legacy chase sightline at deck spawn is BLOCKED (documents the live bug)', chaseBlock !== null, chaseBlock ?? 'clear (bug gone?)');
    check('legacy free-orbit sightline at deck spawn is BLOCKED (documents the live bug)', orbitBlock !== null, orbitBlock ?? 'clear (bug gone?)');
    // Pass 12 (fixed): the same two sightlines through the new
    // OrbitCamera helpers — the deck chase capped under the detail_0
    // overhead sheet, the free orbit through orbitCameraPositionDeckAware
    // (terrain floor + deck ceiling, target = the jet). Both must be
    // clear, or the fix did not hold. A full yaw sweep at the default
    // pitch/dist is logged: angles blocked by the island tower / side
    // structures are expected (the ship has real superstructure there)
    // — the fix guarantees the DEFAULT framing, not every angle.
    const groundAt = (x: number, z: number): number =>
      heightfieldHeightAt(heights, res, x, z);
    const fixedChase = deckChaseCameraPosition(jet, fwd, new THREE.Vector3(0, 1, 0), 30, carrier.deckTopY);
    // The fixed default orbit sits ASTERN on the deck centreline
    // (deckOrbitYaw = carrier yaw + pi), not at the generic yaw 0
    // (world-south = the port side here, over detail_16).
    const fixedOrbit = orbitCameraPositionDeckAware(
      jet,
      { yaw: deckOrbitYaw(carrier.yaw), pitch: 0.32 },
      30,
      groundAt,
      carrier.deckTopY,
    );
    const fixedChaseBlock = sightlineBlockedBy(fixedChase, jet);
    const fixedOrbitBlock = sightlineBlockedBy(fixedOrbit, jet);
    console.log(`  FIXED chase cam=(${fixedChase.x.toFixed(0)},${fixedChase.y.toFixed(1)},${fixedChase.z.toFixed(0)}) blockedBy=${fixedChaseBlock ?? 'none'}`);
    console.log(`  FIXED orbit cam=(${fixedOrbit.x.toFixed(0)},${fixedOrbit.y.toFixed(1)},${fixedOrbit.z.toFixed(0)}) blockedBy=${fixedOrbitBlock ?? 'none'}`);
    check('fixed deck chase sightline at deck spawn is clear (jet visible)', fixedChaseBlock === null, fixedChaseBlock ?? 'clear');
    check('fixed free-orbit sightline at deck spawn is clear (jet visible)', fixedOrbitBlock === null, fixedOrbitBlock ?? 'clear');
    const sweep: string[] = [];
    let sweepClear = 0;
    for (let k = 0; k < 16; k++) {
      const yaw = (k / 16) * Math.PI * 2;
      const cam = orbitCameraPositionDeckAware(jet, { yaw, pitch: 0.32 }, 30, groundAt, carrier.deckTopY);
      const b = sightlineBlockedBy(cam, jet);
      if (b === null) sweepClear++;
      else sweep.push(`yaw=${yaw.toFixed(2)} blocked by ${b}`);
    }
    console.log(`  FIXED orbit yaw sweep (pitch 0.32, dist 30): clear ${sweepClear}/16; blocked: ${sweep.length ? sweep.join('; ') : 'none'}`);
  }

  // --- (F) Carrier bearing / distance from live state ---
  // The HUD patch (FlightHUD + main.ts, patch spec in the pass-12
  // report) shows: absolute bearing to the carrier (deg, 0 = north /
  // +Z, clockwise), bearing RELATIVE to the jet's nose (deg, + = right),
  // and slant range (m). Verified here against the true geometry at
  // three live moments: deck spawn, mid-transit (carrier 3 km away
  // after the takeoff), and on final. The formula under test is the
  // one the patch spec gives main.ts/FlightHUD — a wrong sign or axis
  // here is exactly the "carrier cannot be found" live failure.
  const carrierBearing = (
    from: THREE.Vector3,
    jetFwd: THREE.Vector3,
  ): { absDeg: number; relDeg: number; distM: number } => {
    const dx = carrier.pos.x - from.x;
    const dz = carrier.pos.z - from.z;
    const absDeg = ((Math.atan2(dx, dz) * 180) / Math.PI + 360) % 360;
    const jetHdg = ((Math.atan2(jetFwd.x, jetFwd.z) * 180) / Math.PI + 360) % 360;
    return { absDeg, relDeg: wrapDeg(absDeg - jetHdg), distM: Math.hypot(dx, dz) };
  };
  {
    carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const jet = new THREE.Vector3(
      carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M,
      carrier.deckTopY + 2.2,
      carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M,
    );
    const b = carrierBearing(jet, fwd);
    const trueDist = Math.hypot(carrier.pos.x - jet.x, carrier.pos.z - jet.z);
    console.log(
      `\n--- carrier bearing at deck spawn: abs=${b.absDeg.toFixed(1)} rel=${b.relDeg.toFixed(1)} dist=${b.distM.toFixed(1)}m (true ${trueDist.toFixed(1)}m) ---`,
    );
    // At the spawn the carrier centre is dead ahead, 85 m away.
    check('bearing at deck spawn: carrier dead ahead (rel ~0 deg)', Math.abs(b.relDeg) < 1, `rel=${b.relDeg.toFixed(2)} deg`);
    check('distance at deck spawn: 85 m', Math.abs(b.distM - 85) < 1, `dist=${b.distM.toFixed(1)}m`);
    // 3 km east of the carrier, heading east: the carrier is dead
    // astern (rel ~180 deg), 3,000 m away.
    const away = new THREE.Vector3(carrier.pos.x + 3000, carrier.deckTopY + 100, carrier.pos.z);
    const b2 = carrierBearing(away, fwd);
    console.log(`  3 km east, heading east: abs=${b2.absDeg.toFixed(1)} rel=${b2.relDeg.toFixed(1)} dist=${b2.distM.toFixed(0)}m`);
    check('bearing 3 km out: carrier dead astern (rel ~180 deg)', Math.abs(Math.abs(b2.relDeg) - 180) < 1, `rel=${b2.relDeg.toFixed(2)} deg`);
    check('distance 3 km out: 3,000 m', Math.abs(b2.distM - 3000) < 1, `dist=${b2.distM.toFixed(1)}m`);
  }

  // --- (G) Free-orbit ground view: does the legacy camera go under the terrain? ---
  // The live free-orbit branch (main.ts updateCamera, camMode 2) sets
  // the camera from orbit angles/dist and clamps ONLY at the camera's
  // own (x, z): Math.max(cy, groundHeight(cx, cz) + 3). It never checks
  // the ground under the MIDPOINT to the target, or along the
  // sightline — unlike the chase branch and unlike OrbitCamera's
  // applyCameraFloor (camera + midpoint + target + 2 m). On steep
  // ground a low, far orbit can put the camera (or its sightline)
  // UNDER a rise between camera and jet: the terrain mesh there is
  // single-sided (FrontSide), so from inside/under it the ground
  // between camera and jet is backface-culled — the lower frame shows
  // void instead of terrain (the v4.6.1 live "black terrain below the
  // horizon"). Measured here: legacy positions vs applyCameraFloor
  // positions, over a full yaw sweep at the lowest pitch (-0.1) and
  // the default pitch (0.32), at dist 30 / 60 / 120, around a jet
  // parked on the STEEPEST ground in the central block (worst case)
  // and around the deck spawn (the live case). A second measurement
  // replicates the EXACT vertex-colour recipe (PandoraWorldGenerator
  // buildTerrain / TerrainStreamer buildMesh) and the live Lambert
  // lighting at todH 10 (sun + hemisphere, main.ts updateSky) for the
  // terrain in the free-orbit ground view from the deck spawn, to
  // test the other candidate: the terrain itself rendering near-black.
  {
    // Steepest ground: max |dh| over a 60 m step in the central block.
    let steepX = 0;
    let steepZ = 0;
    let steepGrad = 0;
    for (let ix = 20; ix < res - 20; ix += 4) {
      for (let iz = 20; iz < res - 20; iz += 4) {
        const x = (ix / (res - 1) - 0.5) * WORLD_SIZE;
        const z = (iz / (res - 1) - 0.5) * WORLD_SIZE;
        if (Math.abs(x) > 4000 || Math.abs(z) > 4000) continue;
        const h = heightfieldHeightAt(heights, res, x, z);
        const hx = heightfieldHeightAt(heights, res, x + 60, z);
        const hz = heightfieldHeightAt(heights, res, x, z + 60);
        const g = Math.abs(hx - h) + Math.abs(hz - h);
        if (g > steepGrad) {
          steepGrad = g;
          steepX = x;
          steepZ = z;
        }
      }
    }
    const groundAt = (x: number, z: number): number =>
      heightfieldHeightAt(heights, res, x, z);
    // Import-free re-implementation of the LEGACY inline orbit (main.ts)
    // and of OrbitCamera.applyCameraFloor, so the comparison is
    // like-for-like on positions, not on imports.
    const legacyOrbit = (t: THREE.Vector3, yaw: number, pitch: number, dist: number): THREE.Vector3 => {
      const cx = t.x + Math.sin(yaw) * Math.cos(pitch) * dist;
      const cz = t.z + Math.cos(yaw) * Math.cos(pitch) * dist;
      const cy = t.y + Math.sin(pitch) * dist;
      return new THREE.Vector3(cx, Math.max(cy, groundAt(cx, cz) + 3), cz);
    };
    const segUnderTerrain = (from: THREE.Vector3, to: THREE.Vector3): number => {
      // Worst penetration (m) of the segment below ground+0.5, sampled
      // every ~10 m; 0 = clear.
      let worst = 0;
      const n = Math.max(2, Math.ceil(from.distanceTo(to) / 10));
      for (let i = 0; i <= n; i++) {
        const p = from.clone().lerp(to, i / n);
        const pen = groundAt(p.x, p.z) + 0.5 - p.y;
        if (pen > worst) worst = pen;
      }
      return worst;
    };
    for (const site of [
      { name: 'steep ground', jet: new THREE.Vector3(steepX, groundAt(steepX, steepZ) + 2.2, steepZ) },
      { name: 'deck spawn', jet: new THREE.Vector3(carrier.pos.x - Math.sin(carrier.yaw) * 85, carrier.deckTopY + 2.2, carrier.pos.z - Math.cos(carrier.yaw) * 85) },
    ] as const) {
      let legacyBad = 0;
      let legacyWorst = 0;
      let total = 0;
      for (const pitch of [-0.1, 0.32]) {
        for (const dist of [30, 60, 120]) {
          for (let k = 0; k < 16; k++) {
            const yaw = (k / 16) * Math.PI * 2;
            const cam = legacyOrbit(site.jet, yaw, pitch, dist);
            const pen = segUnderTerrain(cam, site.jet);
            total++;
            if (pen > 0) legacyBad++;
            if (pen > legacyWorst) legacyWorst = pen;
          }
        }
      }
      console.log(
        `  ${site.name} (jet y=${site.jet.y.toFixed(1)}, steepGrad=${site.name === 'steep ground' ? steepGrad.toFixed(1) + 'm/60m' : 'n/a'}): legacy orbit sightline under terrain in ${legacyBad}/${total} samples, worst penetration ${legacyWorst.toFixed(1)}m`,
      );
      check(
        `${site.name}: legacy free-orbit sightline never goes under the terrain`,
        legacyBad === 0,
        `${legacyBad}/${total} samples under terrain, worst ${legacyWorst.toFixed(1)}m`,
      );
    }
    // Terrain brightness in the free-orbit ground view from the deck
    // spawn: terrain within 3 km of the carrier's ground position
    // (the unfogged near field, fog near 900 m clear weather), exact
    // colour recipe + live Lambert lighting at todH 10. Colours are
    // THREE.Color hexes (sRGB -> linear working space, as three r160
    // does); sun dir from updateSky; hemi sky/ground from main.ts.
    // Reports mean / 1st-percentile sRGB luminance of the outgoing
    // light — < 0.05 sRGB would read as black on a phone.
    {
      const srgbToLin = (c: number): number => {
        const s = c / 255;
        return s <= 0.04045 ? s / 12.92 : Math.pow((s + 0.055) / 1.055, 2.4);
      };
      const hexLin = (hex: number): [number, number, number] => [
        srgbToLin((hex >> 16) & 0xff),
        srgbToLin((hex >> 8) & 0xff),
        srgbToLin(hex & 0xff),
      ];
      const cA = hexLin(0x3d7a44);
      const cB = hexLin(0x8a7a4a);
      const cC = hexLin(0x5a6a72);
      const cD = hexLin(0x2a4a34);
      const cSnow = hexLin(0xe8eef2);
      const sunCol = hexLin(0xffd9a0); // approx HSL(0.09,0.7,0.72) sun
      const hemiSky = hexLin(0xbfd9ff);
      const hemiGnd = hexLin(0x4a5a3a);
      const clamp01 = (v: number): number => Math.max(0, Math.min(1, v));
      const lerp3 = (a: [number, number, number], b: [number, number, number], t: number): [number, number, number] => [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
      ];
      // Sun at todH 10 (updateSky): az 150 deg, el 60 deg; direction
      // from the surface to the sun, normalised (y +0.66).
      const sunDir = new THREE.Vector3(
        Math.cos((150 * Math.PI) / 180) * 800,
        779,
        Math.sin((150 * Math.PI) / 180) * 800,
      ).normalize();
      const sunI = 0.15 + 2.1 * 0.866;
      const hemiI = 0.25 + 0.65 * 0.866;
      const lums: number[] = [];
      let darkFrac = 0;
      let n = 0;
      for (let gx = -3000; gx <= 3000; gx += 120) {
        for (let gz = -3000; gz <= 3000; gz += 120) {
          const x = carrier.pos.x + gx;
          const z = carrier.pos.z + gz;
          if (Math.abs(x) > 5900 || Math.abs(z) > 5900) continue;
          const h = groundAt(x, z);
          const f = fbm(x / 240 + 7, z / 240 - 2);
          let col = lerp3(cA, cB, clamp01(f * 1.4 - 0.3));
          if (plateauWeight(x, z) <= 0.5) {
            if (h > 120) col = lerp3(col, cC, clamp01((h - 120) / 120));
            if (h > 550) col = lerp3(col, cSnow, clamp01((h - 550) / 150));
          }
          if (h < 5) col = lerp3(col, cD, 0.6);
          if (runwayBlend(x, z) > 0.5) col = hexLin(0x3a3a3a);
          // Normal from the heightfield gradient (60 m step).
          const nx = groundAt(x - 60, z) - groundAt(x + 60, z);
          const nz = groundAt(x, z - 60) - groundAt(x, z + 60);
          const normal = new THREE.Vector3(nx, 120, nz).normalize();
          const ndl = Math.max(0, normal.dot(sunDir));
          const hemiMix = normal.y * 0.5 + 0.5;
          const hemi: [number, number, number] = [
            (hemiGnd[0] + (hemiSky[0] - hemiGnd[0]) * hemiMix) * hemiI,
            (hemiGnd[1] + (hemiSky[1] - hemiGnd[1]) * hemiMix) * hemiI,
            (hemiGnd[2] + (hemiSky[2] - hemiGnd[2]) * hemiMix) * hemiI,
          ];
          const out: [number, number, number] = [
            col[0] * (sunCol[0] * sunI * ndl + hemi[0]),
            col[1] * (sunCol[1] * sunI * ndl + hemi[1]),
            col[2] * (sunCol[2] * sunI * ndl + hemi[2]),
          ];
          // Linear -> sRGB, then Rec.709 luminance.
          const toSrgb = (v: number): number =>
            v <= 0.0031308 ? v * 12.92 : 1.055 * Math.pow(v, 1 / 2.4) - 0.055;
          const lum = 0.2126 * toSrgb(out[0]) + 0.7152 * toSrgb(out[1]) + 0.0722 * toSrgb(out[2]);
          lums.push(lum);
          if (lum < 0.05) darkFrac++;
          n++;
        }
      }
      lums.sort((a, b) => a - b);
      const mean = lums.reduce((a, b) => a + b, 0) / Math.max(1, lums.length);
      const p1 = lums[Math.floor(lums.length * 0.01)] ?? 0;
      const p50 = lums[Math.floor(lums.length * 0.5)] ?? 0;
      console.log(
        `  terrain brightness near carrier (n=${n}): mean sRGB luminance=${mean.toFixed(3)}, p1=${p1.toFixed(3)}, median=${p50.toFixed(3)}, fraction < 0.05 (near-black)=${(darkFrac / Math.max(1, n)).toFixed(3)}`,
      );
      check(
        'terrain in the free-orbit ground view does not render near-black (mean luminance >= 0.08)',
        mean >= 0.08,
        `mean=${mean.toFixed(3)} p1=${p1.toFixed(3)} darkFrac=${(darkFrac / Math.max(1, n)).toFixed(3)}`,
      );
    }
    // Ground-mesh coverage in the free-orbit ground view from the deck
    // spawn: the terrain mesh only exists inside the central block
    // (+/-6000 m) plus the streamer's window (Chebyshev radius 2 chunks
    // of 3000 m around the player's chunk). Ground beyond that — out to
    // the clear-weather fog far (11 km) — has NO geometry, so below the
    // horizon in that direction the frame shows flat background colour
    // instead of terrain. Measured: fraction of the ground disc (r =
    // 11 km, centred on the carrier's ground position) with no mesh.
    {
      const pcx = Math.floor(carrier.pos.x / 3000);
      const pcz = Math.floor(carrier.pos.z / 3000);
      const hasMesh = (x: number, z: number): boolean => {
        if (Math.abs(x) <= 6000 && Math.abs(z) <= 6000) return true;
        const cx = Math.floor(x / 3000);
        const cz = Math.floor(z / 3000);
        return Math.max(Math.abs(cx - pcx), Math.abs(cz - pcz)) <= 2;
      };
      let noMesh = 0;
      let total = 0;
      for (let gx = -11000; gx <= 11000; gx += 500) {
        for (let gz = -11000; gz <= 11000; gz += 500) {
          if (Math.hypot(gx, gz) > 11000) continue;
          total++;
          if (!hasMesh(carrier.pos.x + gx, carrier.pos.z + gz)) noMesh++;
        }
      }
      console.log(
        `  ground-mesh coverage within 11 km of the carrier: no mesh on ${(noMesh / total).toFixed(2)} of the disc (${noMesh}/${total} samples) — ground past the streamer window renders as flat background, not terrain`,
      );
    }
  }

  // --- (D) Return + trap: takeoff, fly out, navigate back on bearing,
  // land in the wires — the full live return path ---
  // The v4.6.1 live failure was not the trap (section B traps from a
  // scripted 500 m final) but FINDING the carrier after takeoff: the
  // HUD gives no carrier bearing or distance, so a pilot who flies
  // away cannot come back. This section flies the whole loop with the
  // live loop (variable dt, live wind 3 m/s, origin shifts, real spawn
  // code, real carrier GLB) and steers the transit ONLY on the
  // carrier bearing/distance values the HUD patch adds (same formula
  // as section F) — if those values cannot guide the jet home, the
  // return fails here. The carrier keeps flying its loop throughout
  // (it reaches waypoint 0 and starts its ~120 deg turn mid-return),
  // so the final is onto a turning, terrain-following deck.
  {
    const heightAtLive = (x: number, z: number): number =>
      heightfieldHeightAt(
        heights,
        res,
        x + floatingOrigin.cumulativeShift.x,
        z + floatingOrigin.cumulativeShift.z,
      );
    const carrier2 = new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: heightAtLive,
      waypoints: CARRIER_WAYPOINTS,
      startIndex: 0,
    });
    await carrier2.build();
    carrier2.registerWithFloatingOrigin(floatingOrigin);
    floatingOrigin.registerBody(terrainBody);
    const deck2: DeckProvider = {
      get alive() {
        return carrier2.alive;
      },
      get deckTopY() {
        return carrier2.deckTopY;
      },
      deckHeightAt: (x, z) => carrier2.deckHeightAt(x, z),
      velocity: () => carrier2.vel.clone(),
      velocityAt: (x, z) => {
        const rx = x - carrier2.pos.x;
        const rz = z - carrier2.pos.z;
        const w = carrier2.yawRate;
        return new THREE.Vector3(carrier2.vel.x + w * rz, carrier2.vel.y, carrier2.vel.z - w * rx);
      },
      yawRate: () => carrier2.yawRate,
      yawRateAt: () => carrier2.yawRate,
      local: (x, z) => {
        const v = carrier2.localPosition(new THREE.Vector3(x, 0, z));
        return v ? { x: v.x, z: v.z } : null;
      },
    };
    const spec = SPECS['fa18']!;
    const ac = new Aircraft({ scene, world, spec, isPlayer: true, modelKey: spec.modelKey });
    ac.terrainHeightAt = heightAtLive;
    ac.deck = deck2;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    floatingOrigin.registerBody(ac.body);
    const dc2 = carrier2.deckColliderRef;
    if (dc2) owners.set(dc2.handle, 'deck');
    contacts = [];
    simT = 0;
    // Deck spawn, verbatim startRound (carrier2's current pos/yaw).
    const fwd0 = new THREE.Vector3(Math.sin(carrier2.yaw), 0, Math.cos(carrier2.yaw));
    const sx = carrier2.pos.x - fwd0.x * DECK_SPAWN_AFT_M;
    const sz = carrier2.pos.z - fwd0.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, carrier2.deckTopY + 2.2, sz, (carrier2.yaw * 180) / Math.PI);
    const dv0 = deck2.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv0.x, y: dv0.y, z: dv0.z }, true);
    ac.vel.copy(dv0);
    const takeoffHdg = (carrier2.yaw * 180) / Math.PI;
    console.log(`\n--- fa18 return + trap: deck spawn carrierYaw=${takeoffHdg.toFixed(1)} deckTopY=${carrier2.deckTopY.toFixed(1)} ---`);
    type Phase = 'takeoff' | 'climb' | 'outbound' | 'turn1' | 'inbound' | 'turn2' | 'chase' | 'final' | 'done';
    let phase: Phase = 'takeoff';
    let tSec = 0;
    let shifts = 0;
    let pitchInt = 0;
    let liftoffT: number | null = null;
    let outboundT0 = 0;
    let minInboundDist = Infinity;
    let inboundPassed = false;
    let goArounds = 0;
    let inWindowPrev = false;
    let downwindT0 = 0;
    let touchedDown = false;
    let touchdownLz: number | null = null;
    let stopped = false;
    let stoppedLz: number | null = null;
    let fellThrough = false;
    let finalPitchInt = 0;
    let finalAoaTrim = 0;
    let finalVApp = 0;
    const bearing2 = (): { absDeg: number; relDeg: number; distM: number } => {
      const dx = carrier2.pos.x - ac.pos.x;
      const dz = carrier2.pos.z - ac.pos.z;
      const absDeg = ((Math.atan2(dx, dz) * 180) / Math.PI + 360) % 360;
      return { absDeg, relDeg: wrapDeg(absDeg - jetHdgDeg(ac)), distM: Math.hypot(dx, dz) };
    };
    const local2 = (): { x: number; z: number } | null => deck2.local(ac.pos.x, ac.pos.z);
    // Footprint-free deck-local position (same rotation as section A's
    // localOfPos). deck2.local() is null off the deck footprint
    // (localPosition requires deckContains), so the chase->final gate
    // (450 m astern) can NEVER fire on it — the first version's
    // final phase was unreachable for exactly this reason.
    const manualLocal2 = (): { x: number; z: number } => {
      const dx = ac.pos.x - carrier2.pos.x;
      const dz = ac.pos.z - carrier2.pos.z;
      const c = Math.cos(carrier2.yaw);
      const s = Math.sin(carrier2.yaw);
      return { x: c * dx - s * dz, z: s * dx + c * dz };
    };
    const cYawDeg = (): number => {
      const d = (carrier2.yaw * 180) / Math.PI;
      return ((d % 360) + 360) % 360;
    };
    // Generic fly-to: heading hold via bank, altitude hold via the
    // landing section's PI pitch law (trim AoA for the target speed,
    // flaps 0), speed hold via throttle. (pitchIn < 0 = nose up;
    // Euler YXZ x > 0 = nose down.)
    const flyTo = (desHdgDeg: number, desAltM: number, desSpeedMs: number, dt: number, maxBankRad = 0.45, maxSinkMs = 6): void => {
      // Bank sign (measured, return3): a POSITIVE Euler-z bank makes the
      // heading DECREASE (the jet turns left), so bankDes carries a
      // minus sign against the heading error. The first version had
      // the sign backwards and the jet turned 270 deg the wrong way
      // around in turn1 (hdg 90 -> 320 the long way, never closing).
      const hdgErrRad = (wrapDeg(desHdgDeg - jetHdgDeg(ac)) * Math.PI) / 180;
      const bankDes = clamp(-hdgErrRad * 1.0, -maxBankRad, maxBankRad);
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = clamp((bankDes - bankNow) * 0.6, -0.5, 0.5);
      // Trim AoA from the FASTER of current and target airspeed,
      // capped at 12 deg. The return3 version used the target speed
      // (climb at high speed); return4 used the current speed (the
      // trim blew up to the 15 deg cap as the jet slowed in a turn,
      // commanding 16 deg nose-up at 63 m/s — a positive-feedback
      // balloon that stalled the jet at 32 m/s, AoA 45 deg, in
      // return7). Using max(ias, target) keeps the trim sane in
      // both directions. And when slow, never command a climb:
      // trade altitude for speed instead.
      const vTrim = Math.max(ac.ias, desSpeedMs);
      const clNeed = (2 * spec.mass * 9.81) / (1.225 * vTrim * vTrim * spec.wingArea);
      const aoaTrim = clamp((clNeed - 0.15) / 0.08, 0, 12);
      let vsDes = clamp((desAltM - ac.pos.y) * 0.4, -maxSinkMs, 10);
      if (ac.ias < desSpeedMs - 10) vsDes = Math.min(vsDes, -2);
      const airspeed = Math.max(20, ac.ias);
      const gammaDes = Math.atan2(vsDes, airspeed);
      const pitchDes = -(gammaDes + (aoaTrim * Math.PI) / 180);
      const pitchNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x;
      const pitchErr = pitchDes - pitchNow;
      pitchInt = clamp(pitchInt + pitchErr * dt * 0.8, -0.35, 0.35);
      ac.pitchIn = clamp(pitchErr * 3.0 + pitchInt, -0.6, 0.6);
      // AoA limiter (return8): the pitch law kept commanding nose-up
      // to hold altitude in a 27 deg banked turn while the wing was
      // already at 18.6 deg AoA — the jet stalled, speed collapsed
      // 92 -> 52 m/s at full throttle, and it fell into a 615 m peak
      // ("hard landing"). Above 13 deg AoA, never pull up; above 16
      // deg, push the nose down. Unload first, regain speed, then
      // the altitude loop resumes. (13 deg, not 12: at 80 m/s in a
      // 35 deg bank the wing needs ~12 deg AoA for 1.22 g, and a
      // 12 deg cap starved the turn — return9's chase sank 428 m
      // to y~31. The chase now flies 95 m/s, where 1.22 g needs
      // only ~10 deg AoA.)
      if (ac.aoa > 13) ac.pitchIn = Math.max(ac.pitchIn, 0);
      if (ac.aoa > 16) ac.pitchIn = Math.max(ac.pitchIn, 0.15);
      // Speed: floor 0 (idle), not 0.20 — at 20% throttle the clean
      // F-18 still accelerates past 130 m/s, so a 0.20 floor can never
      // hold 95 m/s. Idle lets drag slow it.
      ac.thrTarget = clamp(0.3 + (desSpeedMs - ac.ias) * 0.1, 0, 1.0);
    };
    // Final-approach law: the section-B landing controller, verbatim
    // laws, parameterised by carrier2/deck2 (3.5 deg slope to the wire
    // zone, deck vertical-velocity feed-forward, PI pitch hold, bank
    // to the centreline, speed hold at vApp, hook + flaps 30).
    const finalStep = (dt: number): void => {
      // Manual local coords, NOT the footprint-gated local2(): the
      // first version returned immediately off the deck footprint
      // (localPosition is null 450 m astern), so the jet flew the
      // whole final on stale chase inputs (return6: 47 s level at
      // y~31, pitchIn/rollIn/thr frozen at -0.17/-0.08/0.26).
      const loc = manualLocal2();
      if (!touchedDown) {
        const slopeAlt = carrier2.deckTopY + 2 + Math.max(0, (-75 - loc.z) * 0.061);
        const altErr = slopeAlt - ac.pos.y;
        const dvY = deck2.velocityAt(ac.pos.x, ac.pos.z).y;
        const vsDes = clamp(-finalVApp * 0.061 + altErr * 0.35 + dvY, -4.5, 15.0);
        const airspeed = Math.max(20, Math.hypot(ac.vel.x, ac.vel.z));
        const gammaDes = Math.atan2(vsDes, airspeed);
        const pitchDes = -(gammaDes + (finalAoaTrim * Math.PI) / 180);
        const pitchNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x;
        const pitchErr = pitchDes - pitchNow;
        finalPitchInt = clamp(finalPitchInt + pitchErr * dt * 0.8, -0.35, 0.35);
        ac.pitchIn = clamp(pitchErr * 4.0 + finalPitchInt, -0.7, 0.7);
        const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
        ac.rollIn = clamp((0 - loc.x) * 0.03 - bankNow * 1.0, -0.7, 0.7);
        ac.thrTarget = clamp(0.5 + (finalVApp - deckRelSpeed2(ac)) * 0.12, 0.15, 0.9);
      } else {
        ac.brake = true;
        ac.thrTarget = 0;
        ac.pitchIn = 0;
        if (deckRelSpeed2(ac) < 1) {
          stopped = true;
          stoppedLz = loc.z;
        }
      }
    };
    function deckRelSpeed2(a: Aircraft): number {
      const dv = deck2.velocityAt(a.pos.x, a.pos.z);
      return Math.hypot(a.vel.x - dv.x, a.vel.z - dv.z);
    }
    let logT = 0;
    const stepD = (seconds: number): void => {
      let t = 0;
      let pi = 0;
      while (t < seconds && phase !== 'done') {
        const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
        pi++;
        const sh = floatingOrigin.shiftIfNeeded(ac.pos);
        if (sh) {
          floatingOrigin.applyShift(sh);
          shifts++;
        }
        const sub = Math.ceil(rawDt / 0.05);
        const dt = rawDt / sub;
        for (let s = 0; s < sub; s++) {
          carrier2.update(dt);
          world.timestep = dt;
          world.step(eventQueue);
          drain(ac);
          ac.step(dt, WIND, null);
          simT += dt;
          tSec += dt;
          const b = bearing2();
          const loc = local2();
          const deckY = carrier2.deckHeightAt(ac.pos.x, ac.pos.z);
          if (deckY > -Infinity && ac.pos.y < deckY - 1) fellThrough = true;
          if (tSec - logT >= 5) {
            logT = tSec;
            const eul = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
            console.log(
              `  t=${tSec.toFixed(0)} phase=${phase} hdg=${jetHdgDeg(ac).toFixed(0)} cYaw=${cYawDeg().toFixed(0)} ` +
                `brg=${b.absDeg.toFixed(0)}/${b.relDeg.toFixed(0)} dist=${b.distM.toFixed(0)}m alt=${(ac.pos.y - carrier2.deckTopY).toFixed(0)} ` +
                `ias=${ac.ias.toFixed(0)} pitch=${(-eul.x * 57.3).toFixed(1)} bank=${(eul.z * 57.3).toFixed(1)} ` +
                `pitchIn=${ac.pitchIn.toFixed(2)} rollIn=${ac.rollIn.toFixed(2)} thr=${ac.throttle.toFixed(2)}/${ac.thrTarget.toFixed(2)} ` +
                `aoa=${ac.aoa.toFixed(1)} stalled=${ac.stalled} loc=${loc ? `(${loc.x.toFixed(0)},${loc.z.toFixed(0)})` : 'off'} onDeck=${ac.onDeck} alive=${ac.alive} shifts=${shifts}`,
            );
          }
          // --- phase machine ---
          if (phase === 'takeoff') {
            ac.brake = false;
            ac.thrTarget = 1;
            ac.gear = true;
            if (deckRelSpeed2(ac) > spec.stallSpeed * 0.85 && liftoffT === null) ac.pitchIn = -1;
            if (liftoffT === null && !ac.grounded && !ac.onSurface) {
              liftoffT = tSec;
              phase = 'climb';
              console.log(`  LIFTOFF t=${tSec.toFixed(1)}s localZ=${loc ? loc.z.toFixed(1) : 'off-deck'}`);
            }
          } else if (phase === 'climb') {
            // Cruise at deck+450: the terrain under the return leg is
            // Himalaya-like (baseHeight clamped to 750 m, ranges to
            // ~700 m); deck+180 = ~525 m flew the jet into a 521 m
            // hillside at t~201 in return3. deck+450 = ~795 m is above
            // the 750 m terrain maximum anywhere, so the transit is
            // provably clear of terrain. Descent happens only while
            // closing on the carrier, whose route stays over terrain
            // < ~130 m (HelicarrierController header).
            flyTo(takeoffHdg, carrier2.deckTopY + 450, 95, dt);
            if (ac.pos.y >= carrier2.deckTopY + 400) {
              phase = 'outbound';
              outboundT0 = tSec;
              console.log(`  CLIMB DONE t=${tSec.toFixed(1)}s alt=${(ac.pos.y - carrier2.deckTopY).toFixed(0)}`);
            }
          } else if (phase === 'outbound') {
            flyTo(takeoffHdg, carrier2.deckTopY + 450, 95, dt);
            if (tSec - outboundT0 > 35) {
              phase = 'turn1';
              console.log(`  OUTBOUND DONE t=${tSec.toFixed(1)}s dist=${b.distM.toFixed(0)}m brgRel=${b.relDeg.toFixed(0)}`);
            }
          } else if (phase === 'turn1') {
            const des = (((carrier2.yaw * 180) / Math.PI + 180) % 360 + 360) % 360;
            flyTo(des, carrier2.deckTopY + 450, 90, dt);
            if (Math.abs(wrapDeg(jetHdgDeg(ac) - des)) < 6) {
              phase = 'base';
              console.log(`  TURN1 DONE t=${tSec.toFixed(1)}s hdg=${jetHdgDeg(ac).toFixed(0)} dist=${b.distM.toFixed(0)}m`);
            }
          } else if (phase === 'base') {
            // Base leg (return14): the overhead-break pattern flung
            // the jet 6.4 km out (two 180 deg turns at 90 m/s have a
            // 1.05 km radius each and the carrier moves 1.5 km per
            // turn). Instead, the jet flies straight to a BASE point
            // 500 m astern and 500 m to the port side, then makes a
            // single 90 deg turn onto the final heading. The approach
            // is along the carrier's loop corridor (terrain <=80 m at
            // 500 m, <=309 m at 1000 m — measured), so the altitude
            // follows a terrain-safe profile: deck+40 at 500 m,
            // rising 0.35 m per m of distance, capped at deck+450
            // (above the 750 m terrain maximum).
            const fwdB = new THREE.Vector3(Math.sin(carrier2.yaw), 0, Math.cos(carrier2.yaw));
            const rightB = new THREE.Vector3(Math.cos(carrier2.yaw), 0, -Math.sin(carrier2.yaw));
            const aimB = carrier2.pos.clone().addScaledVector(fwdB, -500).addScaledVector(rightB, -500);
            const aimBrgB = ((Math.atan2(aimB.x - ac.pos.x, aimB.z - ac.pos.z) * 180) / Math.PI + 360) % 360;
            const distAimB = Math.hypot(aimB.x - ac.pos.x, aimB.z - ac.pos.z);
            const baseAlt = carrier2.deckTopY + 40 + clamp((b.distM - 500) * 0.35, 0, 410);
            flyTo(aimBrgB, baseAlt, 90, dt, 0.61);
            if (distAimB < 400 || b.distM < 900) {
              phase = 'baseTurn';
              console.log(`  BASE DONE t=${tSec.toFixed(1)}s distAim=${distAimB.toFixed(0)}m dist=${b.distM.toFixed(0)}m alt=${(ac.pos.y - carrier2.deckTopY).toFixed(0)}`);
            }
          } else if (phase === 'baseTurn') {
            // Single 90 deg turn onto the carrier's heading (final).
            const des = cYawDeg();
            flyTo(des, carrier2.deckTopY + 150, 85, dt, 0.61);
            if (Math.abs(wrapDeg(jetHdgDeg(ac) - des)) < 10 && Math.abs(b.relDeg) < 60) {
              phase = 'chase';
              console.log(`  BASETURN DONE t=${tSec.toFixed(1)}s hdg=${jetHdgDeg(ac).toFixed(0)} dist=${b.distM.toFixed(0)}m brgRel=${b.relDeg.toFixed(0)}`);
            }
          } else if (phase === 'chase') {
            // Fixed-heading + lateral correction down a 3.5 deg glide
            // path to the gate (deck+40 at 500 m astern). No pursuit
            // of a moving point: the jet flies the carrier's heading
            // and corrects its lateral offset, so it cannot orbit.
            const mLocC = manualLocal2();
            const desC = cYawDeg() + clamp(-mLocC.x * 0.06, -25, 25);
            const chaseAltC = carrier2.deckTopY + 40 + Math.max(0, b.distM - 500) * 0.061;
            flyTo(desC, chaseAltC, 85, dt, 0.45, 12);
            const mLoc = manualLocal2();
            // Entry conditions on top of the -600..-450 m window:
            // aligned within 20 deg of the carrier's heading and at
            // deck+0..+150. Return9 entered at 598 m astern, 36 deg
            // off heading and 98 m BELOW the deck — the final law
            // (built for a 500 m, on-slope, aligned entry) rolled it
            // to 53 deg bank, stalled it at AoA 38 deg, and it hit
            // the terrain 2.4 km out. A bad entry is a go-around.
            const hdgErrF = Math.abs(wrapDeg(jetHdgDeg(ac) - cYawDeg()));
            const altOkF = ac.pos.y > carrier2.deckTopY && ac.pos.y < carrier2.deckTopY + 150;
            const inWindow = mLoc.z < -450 && mLoc.z > -600 && Math.abs(mLoc.x) < 100;
            if (inWindow && hdgErrF < 20 && altOkF) {
              phase = 'final';
              finalVApp = Math.max(46, spec.stallSpeed * 0.8);
              const dvF = deck2.velocityAt(ac.pos.x, ac.pos.z);
              const vAirApp = Math.hypot(dvF.x, dvF.z) + finalVApp;
              const clNeed = (2 * spec.mass * 9.81) / (1.225 * vAirApp * vAirApp * spec.wingArea);
              finalAoaTrim = Math.max(0, Math.min(15, (clNeed - 0.15 - 0.35) / 0.08));
              ac.gear = true;
              ac.hook = true;
              ac.flaps = 30;
              console.log(`  CHASE DONE t=${tSec.toFixed(1)}s local=(${mLoc.x.toFixed(1)},${mLoc.z.toFixed(1)}) vApp=${finalVApp.toFixed(1)} aoaTrim=${finalAoaTrim.toFixed(1)}`);
            } else if (inWindowPrev && mLoc.z > -450) {
              goArounds++;
              console.log(`  GO-AROUND #${goArounds} t=${tSec.toFixed(1)}s hdgErr=${hdgErrF.toFixed(0)} alt=${(ac.pos.y - carrier2.deckTopY).toFixed(0)} loc=(${mLoc.x.toFixed(0)},${mLoc.z.toFixed(0)})`);
            }
            inWindowPrev = inWindow;
          } else if (phase === 'final') {
            finalStep(dt);
            if (!touchedDown && ac.onDeck) {
              touchedDown = true;
              touchdownLz = loc ? loc.z : null;
              console.log(`  TOUCHDOWN t=${tSec.toFixed(1)}s local=(${loc ? loc.x.toFixed(1) : '?'},${loc ? loc.z.toFixed(1) : '?'}) sp=${deckRelSpeed2(ac).toFixed(1)}`);
            }
            if (stopped) {
              phase = 'done';
              console.log(`  TRAPPED + STOPPED t=${tSec.toFixed(1)}s stoppedLz=${stoppedLz === null ? '?' : stoppedLz.toFixed(1)}`);
            }
          }
          if (!ac.alive) break;
        }
        t += rawDt;
      }
    };
    ac.brake = false;
    stepD(900); // budget: climb ~60 s + transit ~250 s + break pattern ~150 s + final ~30 s
    const bEnd = bearing2();
    const locEnd = local2();
    console.log(
      `  result: phase=${phase} t=${tSec.toFixed(1)}s touchedDown=${touchedDown} touchdownLz=${touchdownLz === null ? 'NONE' : touchdownLz.toFixed(1)} ` +
        `stopped=${stopped} stoppedLz=${stoppedLz === null ? 'NONE' : stoppedLz.toFixed(1)} fellThrough=${fellThrough} shifts=${shifts} ` +
        `finalDist=${bEnd.distM.toFixed(0)}m finalLocal=${locEnd ? `(${locEnd.x.toFixed(0)},${locEnd.z.toFixed(0)})` : 'off-deck'} alive=${ac.alive} ` +
        `crashReason=${ac.lastCrashReason ?? 'none'} fuel=${ac.fuel.toFixed(2)} pos=(${ac.pos.x.toFixed(0)},${ac.pos.y.toFixed(0)},${ac.pos.z.toFixed(0)})`,
    );
    check('return: takes off (liftoff recorded)', liftoffT !== null, liftoffT === null ? 'never lifted off' : `t=${liftoffT.toFixed(1)}s`);
    check('return: navigates back to the carrier on bearing (ends on final / trapped)', phase === 'final' || phase === 'done', `ended in phase=${phase} dist=${bEnd.distM.toFixed(0)}m`);
    check('return: touches down on the deck', touchedDown && touchdownLz !== null && touchdownLz > -128 && touchdownLz < 152, `touchdownLz=${touchdownLz === null ? 'NONE' : touchdownLz.toFixed(1)}`);
    check('return: traps and rolls out to a stop ON the deck', stopped && stoppedLz !== null && stoppedLz > -128 && stoppedLz < 152, `stoppedLz=${stoppedLz === null ? 'NONE' : stoppedLz.toFixed(1)}`);
    check('return: never falls through the deck', !fellThrough, fellThrough ? 'fell below deckTop-1' : 'clean');
    check('return: survives at least one origin shift during the transit', shifts >= 1, `shifts=${shifts}`);
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
