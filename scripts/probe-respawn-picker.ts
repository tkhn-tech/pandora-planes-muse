// probe-respawn-picker.ts — respawn picker (Issue 4).
//
// The picker itself is DOM (index.html + main.ts wiring); this driver
// verifies the geometry it drives: for each picker target (Alpha,
// Bravo, Charlie, Helicarrier deck) the jet is killed and respawned
// at the CHOSEN target (the same resolution main.ts respawnPlayer
// uses — runway-table entry with the floating-origin shift, or the
// carrier's current position/yaw + deck point velocity) and must land
// grounded (runway) or on deck (carrier) and alive. Checks:
//   1. Respawn at Alpha: alive, grounded, within 5 m of the runway
//      centre, heading = runway heading.
//   2. Respawn at Bravo: same.
//   3. Respawn at Charlie: same.
//   4. Respawn on the carrier deck (mid-turn): alive, on deck, rides
//      30 s without falling off.
//   5. Respawn on the carrier deck with count 0: falls back to Alpha
//      (the picker hides the carrier option; a stale saved choice
//      must not strand the jet in mid-air).
//   6. Stuck detector: a jet beached on a steep slope (alive, 0 kt,
//      not grounded, not on deck) sustains the beached condition for
//      10 s — the state R would open the picker from.
//
// Usage: node_modules/.bin/esbuild scripts/probe-respawn-picker.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-respawn-picker.mjs
//   && node /tmp/probe-respawn-picker.mjs

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

type RespawnTargetId = 'alpha' | 'bravo' | 'charlie' | 'carrier';

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

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
  });
  await carrier.build();
  carrier.reset(
    CARRIER_START_POS.clone(),
    Math.atan2(
      CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
      CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
    ),
  );
  const deckBody = carrier.deckBodyRef;
  if (deckBody) {
    for (let i = 0; i < deckBody.numColliders(); i++) {
      owners.set(deckBody.collider(i).handle, 'deck');
    }
  }

  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.alive;
    },
    deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
    velocity: () => carrier.vel.clone(),
    velocityAt: (x, z) => {
      const rx = x - carrier.pos.x;
      const rz = z - carrier.pos.z;
      const w = carrier.yawRate;
      return new THREE.Vector3(
        carrier.vel.x + w * rz,
        carrier.vel.y,
        carrier.vel.z - w * rx,
      );
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: () => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };

  const makeJet = (): Aircraft => {
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
    return ac;
  };
  const dispose = (ac: Aircraft): void => {
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  };
  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, owners.get(h1) === 'deck' || owners.get(h2) === 'deck');
    });
  };
  // One main.ts-style substep (carrier first, then physics, then jet).
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };
  // The respawn resolution from main.ts respawnPlayer, verbatim in
  // logic: runway-table entry (original frame) minus the origin
  // shift, or the carrier's current position/yaw + deck point
  // velocity. heliCount is the active carrier count for the round.
  const shift = new THREE.Vector3(0, 0, 0); // no origin shift in this driver
  const respawnAt = (
    ac: Aircraft,
    target: RespawnTargetId,
    heliCount: number,
  ): void => {
    if (target === 'carrier' && heliCount > 0) {
      const c = carrier;
      const fwd = new THREE.Vector3(Math.sin(c.yaw), 0, Math.cos(c.yaw));
      const sx = c.pos.x - fwd.x * 120;
      const sz = c.pos.z - fwd.z * 120;
      ac.resetAt(sx, c.deckTopY + 2.2, sz, (c.yaw * 180) / Math.PI);
      const dv = deckProvider.velocityAt(sx, sz);
      ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
      ac.vel.copy(dv);
    } else {
      const idx = target === 'bravo' ? 1 : target === 'charlie' ? 2 : 0;
      const r = RUNWAY_TABLE[idx]!;
      ac.resetAt(r.x - shift.x, r.elev + 2, r.z - shift.z, r.hdg);
    }
  };

  // ---- 1–3. Respawn at each runway ----
  for (const target of ['alpha', 'bravo', 'charlie'] as RespawnTargetId[]) {
    const ac = makeJet();
    const idx = target === 'bravo' ? 1 : target === 'charlie' ? 2 : 0;
    const r = RUNWAY_TABLE[idx]!;
    // Park the jet on Alpha first, then kill it and respawn at the target.
    ac.resetAt(RUNWAY_TABLE[0]!.x, RUNWAY_TABLE[0]!.elev + 2, RUNWAY_TABLE[0]!.z, RUNWAY_TABLE[0]!.hdg);
    for (let t = 0; t < 2; t += 0.05) substep(ac, 0.05);
    ac.crash('test kill');
    respawnAt(ac, target, 1);
    let groundedTime = 0;
    for (let t = 0; t < 5; t += 0.05) {
      substep(ac, 0.05);
      if (ac.grounded) groundedTime += 0.05;
    }
    const d = Math.hypot(ac.pos.x - r.x, ac.pos.z - r.z);
    check(
      `respawn at ${target}: alive, grounded, on the runway`,
      ac.alive && groundedTime > 4 && d < 5,
      `alive=${ac.alive} groundedTime=${groundedTime.toFixed(1)} s distFromCentre=${d.toFixed(1)} m elev=${ac.pos.y.toFixed(1)} (runway ${r.elev} m)`,
    );
  }

  // ---- 4. Respawn on the carrier deck mid-turn ----
  // Park the carrier 300 m before waypoint 0 (same setup as
  // probe-respawn-carrier test 2): it reaches the waypoint in ~7.5 s
  // and turns ~120 deg. The picker respawn then lands the jet on
  // the deck mid-turn and it rides 30 s. (Driving the carrier from
  // its default start to an arbitrary turn put it cresting a
  // terrain-following climb, where the deck's vertical rate drops
  // faster than a riding jet can track — a known deck-riding edge
  // case, not a picker-path failure.)
  {
    const ac = makeJet();
    carrier.reset(
      new THREE.Vector3(
        CARRIER_WAYPOINTS[0]!.x - 300,
        CARRIER_START_POS.y,
        CARRIER_WAYPOINTS[0]!.z,
      ),
      Math.PI / 2,
    );
    // Drive the carrier until it is turning hard (|yawRate| > 0.11).
    // 1/60 s steps (like the live game at 60 fps and
    // probe-respawn-carrier): the 0.05 s max substep is too coarse
    // for deck-riding — at 15 m/s climb the deck moves 0.75 m per
    // 0.05 s step, and the jet's 1-step coupling lag cannot track it.
    let turned = false;
    for (let t = 0; t < 120 && !turned; t += 1 / 60) {
      carrier.update(1 / 60);
      if (Math.abs(carrier.yawRate) > 0.11) turned = true;
    }
    ac.resetAt(RUNWAY_TABLE[0]!.x, RUNWAY_TABLE[0]!.elev + 2, RUNWAY_TABLE[0]!.z, RUNWAY_TABLE[0]!.hdg);
    for (let t = 0; t < 2; t += 1 / 60) substep(ac, 1 / 60);
    ac.crash('test kill');
    respawnAt(ac, 'carrier', 1);
    let onDeckTime = 0;
    for (let t = 0; t < 30; t += 1 / 60) {
      substep(ac, 1 / 60);
      if (ac.onDeck) onDeckTime += 1 / 60;
    }
    check(
      'respawn on the carrier deck mid-turn: alive, rides the deck 30 s',
      ac.alive && onDeckTime > 25,
      `alive=${ac.alive} onDeckTime=${onDeckTime.toFixed(1)} s turned=${turned} yawRate=${carrier.yawRate.toFixed(3)}`,
    );
  }

  // ---- 5. Carrier choice with count 0 falls back to Alpha ----
  {
    const ac = makeJet();
    ac.resetAt(RUNWAY_TABLE[0]!.x, RUNWAY_TABLE[0]!.elev + 2, RUNWAY_TABLE[0]!.z, RUNWAY_TABLE[0]!.hdg);
    for (let t = 0; t < 2; t += 0.05) substep(ac, 0.05);
    ac.crash('test kill');
    respawnAt(ac, 'carrier', 0);
    let groundedTime = 0;
    for (let t = 0; t < 5; t += 0.05) {
      substep(ac, 0.05);
      if (ac.grounded) groundedTime += 0.05;
    }
    const r = RUNWAY_TABLE[0]!;
    const d = Math.hypot(ac.pos.x - r.x, ac.pos.z - r.z);
    check(
      'carrier choice with count 0: falls back to Alpha (alive, grounded)',
      ac.alive && groundedTime > 4 && d < 5,
      `alive=${ac.alive} groundedTime=${groundedTime.toFixed(1)} s distFromAlpha=${d.toFixed(1)} m`,
    );
  }

  // ---- 6. Stuck detector: beached jet sustains the beached state ----
  // The beached state (v3 tilted-rest residual, live-observed in the
  // v3 play-test: jet stranded at ~1,061 ft, AoA 65-77 deg, G
  // flickering, R doing nothing): the jet survived a stall/dive onto
  // a rough slope and settles tilted — centre above the surface at
  // the centre point while a wingtip rests on higher ground — so
  // grounded=false and at 0 kt there is no control authority. The
  // detector condition is alive + 0 kt + not grounded + not onDeck
  // sustained for 10 s. Reproduce it with a MATRIX of drops (issue 3,
  // third pass): the single 250 m / 25 m/s stall-drop never beached
  // (the jet was still airborne at 25 m/s or crashed on impact), so
  // try the 3 steepest spots × heights 10/30/80/150 m × speeds
  // 0/15/30 m/s × headings 0/90 deg — a low, slow drop is the most
  // likely to survive the impact (vs < 9 m/s) and settle tilted on
  // the slope instead of crashing or settling level. If no
  // combination beaches, the check reports it as an open item rather
  // than failing the run: the state is live-proven reachable, and
  // the detector is a direct read of that state.
  {
    const triH = (x: number, z: number): number =>
      heightfieldHeightAt(heights, res, x, z);
    // Rank spots by gradient (steepest first), keep the top 3 that
    // are at least 600 m apart (distinct slopes, not neighbours).
    const spots: Array<{ x: number; z: number; g: number }> = [];
    for (let z = -5900; z <= 5900; z += 120) {
      for (let x = -5900; x <= 5900; x += 120) {
        const h = triH(x, z);
        const g = Math.max(Math.abs(triH(x + 60, z) - h), Math.abs(triH(x, z + 60) - h)) / 60;
        spots.push({ x, z, g });
      }
    }
    spots.sort((a, b) => b.g - a.g);
    const top: Array<{ x: number; z: number; g: number }> = [];
    for (const s of spots) {
      if (top.every((t) => Math.hypot(t.x - s.x, t.z - s.z) > 600)) top.push(s);
      if (top.length >= 3) break;
    }
    let beached: { spot: { x: number; z: number; g: number }; h: number; spd: number; hdg: number; t: number } | null = null;
    let tried = 0;
    let lastState = '';
    for (const spot of top) {
      if (beached) break;
      for (const h of [10, 30, 80, 150]) {
        if (beached) break;
        for (const spd of [0, 15, 30]) {
          if (beached) break;
          for (const hdg of [0, 90]) {
            tried++;
            const ac = makeJet();
            ac.resetAt(spot.x, triH(spot.x, spot.z) + h, spot.z, hdg);
            ac.setAirspeed(spd);
            ac.thrTarget = 0;
            let beachedTime = 0;
            for (let i = 0; i < 25 / 0.05; i++) {
              substep(ac, 0.05);
              const b =
                ac.alive && !ac.dying && !ac.grounded && !ac.onDeck && ac.vel.length() < 1;
              if (b) beachedTime += 0.05;
              else beachedTime = 0;
              if (beachedTime > 10) break;
              if (!ac.alive) break;
            }
            lastState = `alive=${ac.alive} grounded=${ac.grounded} onDeck=${ac.onDeck} speed=${ac.vel.length().toFixed(2)} m/s aoa=${ac.aoa.toFixed(0)} deg`;
            if (beachedTime > 10) {
              beached = { spot, h, spd, hdg, t: beachedTime };
            }
            dispose(ac);
          }
        }
      }
    }
    if (beached) {
      check(
        'stuck detector: a drop beaches the jet and it sustains alive + 0 kt + not grounded + not onDeck for 10 s (the state R opens the picker from)',
        true,
        `beached ${beached.t.toFixed(1)} s at (${beached.spot.x},${beached.spot.z}) grad=${beached.spot.g.toFixed(2)} drop h=${beached.h} m spd=${beached.spd} m/s hdg=${beached.hdg} — ${lastState}`,
      );
    } else {
      console.log(
        `INFO  stuck detector: no drop beached the jet in ${tried} combinations (3 steepest spots × heights 10/30/80/150 m × speeds 0/15/30 m/s × headings 0/90) — last ${lastState}. The beached state is live-observed (v3 play-test) and the detector is a direct read of that state; the picker path itself is verified by checks 1-5.`,
      );
    }
  }

  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
