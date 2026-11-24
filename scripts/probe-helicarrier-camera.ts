// probe-helicarrier-camera.ts — Pass 15, worker E, item 9
// (RESTART CAMERA BAD CASE).
//
// v4.9 live: one F-35A Helicarrier respawn gave a front-on, below-deck
// camera looking up at the carrier underside, jet floating beside the
// carrier. Most restarts were sane (pass 14 snap).
//
// Measured mechanism: respawnPlayer snaps the camera against the LIVE
// carrier — the first active && alive carrier (`liveCarrier`) — but the
// very next updateCamera frame recomputes the chase want with
// `carrier.deckTopY`, where `carrier` is ALWAYS carriers[0]. When the
// jet is on carrier 1 or 2 (respawn after carriers[0] is dead, or a
// landing on another carrier) and that carrier's deck is at a different
// altitude, the chase ceiling clamps the camera to the WRONG deck's
// deckTop + 4 — hundreds of metres below the jet. The chase helper is
// sound when given the right deck: deck chase clamps y to
// [deckTop+3, deckTop+4] and the deck ceiling is applied last, so the
// final y sits in that band. The bug is the DECK SELECTION in main.ts,
// in updateCamera's chase, free-orbit and spectate branches, and
// respawnPlayer's camOrbit yaw (which also uses carriers[0].yaw instead
// of liveCarrier.yaw).
//
// The fix: resolve the deck under the jet (carrierAt / the deckProvider,
// as respawnPlayer already does for the spawn) in updateCamera, and use
// liveCarrier.yaw for the deck-spawn camOrbit.
//
// This probe runs through the live path (real SHIELD carrier GLBs on
// the carrierLoop, real deckProvider, real deckTopY): it reproduces the
// scenario, measures the snap want vs the (old) updateCamera want, and
// source-checks the wiring in main.ts (same pattern as
// probe-restart-camera — main.ts cannot be imported in node, it needs
// DOM). No FloatingOrigin shift is needed: the camera question is
// about which deck's deckTopY the want is capped to, and a shift
// moves jet and camera together.
//
// Usage: node_modules/.bin/esbuild scripts/probe-helicarrier-camera.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-helicarrier-camera.mjs
//   && node /tmp/probe-helicarrier-camera.mjs
//   (delete /tmp/probe-helicarrier-camera.mjs after running)

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
import { HelicarrierController, carrierLoop } from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import {
  chaseCameraPosition,
  deckOrbitYaw,
  orbitCameraPositionDeckAware,
} from '../src/camera/OrbitCamera';
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

const D2R = Math.PI / 180;
const WIND = new THREE.Vector3(-3, 0, 0);

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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const scene = new THREE.Scene();
  const alpha = RUNWAY_TABLE[0]!;

  // Three real SHIELD carriers on the carrierLoop, as main.ts does.
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
  for (const c of carriers) {
    await c.build();
    // Real-GLB assertion (the carrier has no modelStatus field):
    // SHIELD deckLocalY is (29.78 - 35.06) = -5.28 m, so deckTopY is
    // 350 - 5.28 = 344.72 m at the start position; the procedural
    // fallback hull would give 350 - 9.57 = 340.43 m.
    if (Math.abs(c.deckTopY - 344.72) > 1) {
      throw new Error(`carrier deckTopY ${c.deckTopY.toFixed(2)} — real SHIELD GLB did not load (fallback?)`);
    }
  }
  const carrierAt = (x: number, z: number): HelicarrierController | null => {
    let best: HelicarrierController | null = null;
    let bestY = -Infinity;
    for (const c of carriers) {
      const y = c.deckHeightAt(x, z);
      if (y > bestY) {
        bestY = y;
        best = c;
      }
    }
    return bestY === -Infinity ? null : best;
  };
  const deck: DeckProvider = {
    alive: true,
    deckHeightAt: (x, z) => {
      const c = carrierAt(x, z);
      return c ? c.deckHeightAt(x, z) : -Infinity;
    },
    velocity: () => new THREE.Vector3(),
    velocityAt: (x, z, t) => {
      const c = carrierAt(x, z);
      return c ? c.velocityAt(x, z, t) : new THREE.Vector3();
    },
    yawRate: () => 0,
    yawRateAt: (x, z, t) => {
      const c = carrierAt(x, z);
      return c ? c.yawRateAt(x, z, t) : 0;
    },
    local: (x, z) => {
      const c = carrierAt(x, z);
      return c ? c.local(x, z) : null;
    },
  };
  const makeAircraft = async (key: string): Promise<Aircraft> => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck,
    });
    ac.terrainHeightAt = heightAt;
    await ac.attachModel();
    if (ac.modelStatus !== 'ok') throw new Error(`jet modelStatus ${ac.modelStatus}`);
    return ac;
  };
  // (No FloatingOrigin shift is needed here: the camera question is
  // about which deck's deckTopY the want is capped to, and the shift
  // moves jet and camera together.)

  // Drive the carriers around the loop for a while so they separate
  // (phase offsets + terrain-following) — then measure deck heights.
  for (let i = 0; i < 3; i++) carriers[i]!.update(90);
  console.log('--- carriers after 90 s of loop driving ---');
  carriers.forEach((c, i) => {
    console.log(
      `  carrier ${i}: pos=(${c.pos.x.toFixed(0)}, ${c.pos.y.toFixed(1)}, ${c.pos.z.toFixed(0)}) yaw=${((c.yaw * 180) / Math.PI).toFixed(1)} deckTopY=${c.deckTopY.toFixed(1)} alive=${c.alive}`,
    );
  });

  // The jet that the camera follows: an F-35A placed on a deck, the
  // way respawnPlayer does (85 m aft of the deck centre, deckTopY+2.2).
  const placeOnDeck = (c: HelicarrierController, ac: Aircraft): void => {
    const p = c.pos.clone();
    p.x -= Math.sin(c.yaw) * 85;
    p.z -= Math.cos(c.yaw) * 85;
    ac.resetAt(p.x, c.deckTopY + 2.2, p.z, (c.yaw * 180) / Math.PI);
    // onDeck is only set by step(); set it as a post-step jet would be.
    ac.onDeck = true;
    ac.onSurface = true;
    ac.grounded = false;
  };
  const deckTopUnderJet = (ac: Aircraft): number | null => {
    // The FIXED resolution: the deck under the jet, not carriers[0].
    if (!ac.onDeck) return null;
    const c = carrierAt(ac.pos.x, ac.pos.z);
    return c ? c.deckTopY : null;
  };

  const ac = await makeAircraft('f35a');
  const primary = carriers[0]!;
  const second = carriers[1]!;
  const fwdOf = (a: Aircraft): THREE.Vector3 => {
    const f = new THREE.Vector3(0, 0, 1);
    f.applyQuaternion(a.quat);
    return f;
  };

  // ---- A. Respawn on carrier 1 after carrier 0 is dead ----
  console.log('--- A. Respawn on carrier 1, carrier 0 dead (the live case) ---');
  {
    // Kill carrier 0 the way the game does (pods dead -> updateDying
    // -> after the sequence, alive=false). For the camera question
    // only the resulting alive=false / deckHeightAt=-Infinity matters,
    // plus the wreck's resting height: the dying sequence sinks the
    // carrier to the ground (ground + 5 m) before writing it off, so
    // a dead carrier's deckTopY sits near terrain, not at 350 m.
    (primary as unknown as { alive: boolean }).alive = false;
    primary.pos.y = heightAt(primary.pos.x, primary.pos.z) + 5;
    const liveCarrier = carriers.find((c) => c.active && c.alive)!;
    console.log(
      `  liveCarrier = carrier ${carriers.indexOf(liveCarrier)} (deckTopY=${liveCarrier.deckTopY.toFixed(1)}); carrier 0 deckTopY=${primary.deckTopY.toFixed(1)} (dead, deckHeightAt=-Inf)`,
    );
    placeOnDeck(liveCarrier, ac);
    const deckTopFixed = deckTopUnderJet(ac)!;
    const snapWant = chaseCameraPosition(
      ac.pos, fwdOf(ac), new THREE.Vector3(0, 1, 0), 30, heightAt, deckTopFixed,
    );
    const oldUpdateWant = chaseCameraPosition(
      ac.pos, fwdOf(ac), new THREE.Vector3(0, 1, 0), 30, heightAt, primary.deckTopY,
    );
    console.log(
      `  jet y=${ac.pos.y.toFixed(1)} deckTopY(jet's carrier)=${deckTopFixed.toFixed(1)} snap want y=${snapWant.y.toFixed(1)} (band [${(deckTopFixed + 3).toFixed(1)}, ${(deckTopFixed + 4).toFixed(1)}])`,
    );
    console.log(
      `  OLD updateCamera want y=${oldUpdateWant.y.toFixed(1)} — ${(ac.pos.y - oldUpdateWant.y).toFixed(0)} m BELOW the jet, looking up at the carrier underside`,
    );
    check(
      'A1 snap want sits in the jet-deck band [deckTop+3, deckTop+4]',
      snapWant.y >= deckTopFixed + 3 - 0.01 && snapWant.y <= deckTopFixed + 4 + 0.01,
      `want y=${snapWant.y.toFixed(1)}`,
    );
    check(
      'A2 the OLD updateCamera want (carriers[0] deck) is > 100 m below the jet — the live bad case, reproduced',
      ac.pos.y - oldUpdateWant.y > 100,
      `want y=${oldUpdateWant.y.toFixed(1)}, jet y=${ac.pos.y.toFixed(1)}`,
    );
    const fixedWant = chaseCameraPosition(
      ac.pos, fwdOf(ac), new THREE.Vector3(0, 1, 0), 30, heightAt, deckTopFixed,
    );
    check(
      'A3 the FIXED updateCamera want (deck under the jet) is the same as the snap want',
      Math.abs(fixedWant.y - snapWant.y) < 0.01,
      `fixed want y=${fixedWant.y.toFixed(1)}`,
    );
    // camOrbit yaw: respawnPlayer used carriers[0].yaw; it must use
    // liveCarrier.yaw. Drive carriers apart first if yaws coincide.
    const yawOld = deckOrbitYaw(primary.yaw);
    const yawFixed = deckOrbitYaw(liveCarrier.yaw);
    console.log(
      `  deckOrbitYaw: carriers[0]=${yawOld.toFixed(2)}rad liveCarrier=${yawFixed.toFixed(2)}rad (yaws differ by ${((Math.abs(yawOld - yawFixed) * 180) / Math.PI).toFixed(1)}deg)`,
    );
    // Free-orbit branch (orbitCameraPositionDeckAware) with the wrong
    // deck ceiling: same wrong-deck exposure as chase.
    const orbitOld = orbitCameraPositionDeckAware(
      ac.pos, { yaw: yawFixed, pitch: -0.2 }, 40, heightAt, primary.deckTopY,
    );
    const orbitFixed = orbitCameraPositionDeckAware(
      ac.pos, { yaw: yawFixed, pitch: -0.2 }, 40, heightAt, deckTopFixed,
    );
    console.log(
      `  orbit: OLD deck ceiling -> y=${orbitOld.y.toFixed(1)}; FIXED -> y=${orbitFixed.y.toFixed(1)} (jet y=${ac.pos.y.toFixed(1)})`,
    );
    check(
      'A4 free-orbit with the OLD (carriers[0]) deck ceiling is > 100 m below the jet',
      ac.pos.y - orbitOld.y > 100,
      `orbit y=${orbitOld.y.toFixed(1)}`,
    );
    check(
      'A5 free-orbit with the FIXED deck ceiling sits in the deck band',
      orbitFixed.y >= deckTopFixed + 3 - 0.01 && orbitFixed.y <= deckTopFixed + 4 + 0.01,
      `orbit y=${orbitFixed.y.toFixed(1)}`,
    );
    // Restore carrier 0 for the next section (position and alive).
    (primary as unknown as { alive: boolean }).alive = true;
    primary.pos.y = 350;
  }

  // ---- B. Landing on carrier 1 with carrier 0 alive but lower ----
  // Drive the carriers until their deck heights differ by > 50 m
  // (terrain-following look-ahead into the ranges), or accept the
  // measured difference if it already does.
  console.log('--- B. Jet on carrier 1, carrier 0 alive (altitude split) ---');
  {
    let t = 90;
    let split = Math.abs(carriers[1]!.deckTopY - carriers[0]!.deckTopY);
    while (split < 50 && t < 400) {
      for (const c of carriers) c.update(10);
      t += 10;
      split = Math.abs(carriers[1]!.deckTopY - carriers[0]!.deckTopY);
    }
    console.log(
      `  after ${t}s: carrier0 deckTopY=${carriers[0]!.deckTopY.toFixed(1)} carrier1 deckTopY=${carriers[1]!.deckTopY.toFixed(1)} split=${split.toFixed(1)} m`,
    );
    if (split >= 50) {
      placeOnDeck(carriers[1]!, ac);
      const deckTopFixed = deckTopUnderJet(ac)!;
      const oldWant = chaseCameraPosition(
        ac.pos, fwdOf(ac), new THREE.Vector3(0, 1, 0), 30, heightAt, carriers[0]!.deckTopY,
      );
      console.log(
        `  jet y=${ac.pos.y.toFixed(1)}; OLD updateCamera want y=${oldWant.y.toFixed(1)} — ${(ac.pos.y - oldWant.y).toFixed(0)} m below the jet`,
      );
      check(
        'B1 with a > 50 m deck split, the OLD updateCamera want is > 40 m below the jet (wrong deck leaks in without any carrier death)',
        ac.pos.y - oldWant.y > 40,
        `want y=${oldWant.y.toFixed(1)}`,
      );
    } else {
      console.log(
        '  (carriers never split by > 50 m in 400 s — the wrong-deck leak then needs carrier 0 dead, section A; skipping B1)',
      );
    }
  }

  // ---- C. Wiring in main.ts (source checks) ----
  console.log('--- C. Wiring in src/main.ts (source checks) ---');
  {
    const src = readFileSync('src/main.ts', 'utf8');
    // updateCamera must not resolve the chase/orbit/spectate deck
    // ceiling from carriers[0] (`carrier.deckTopY`); it must resolve
    // the deck under the jet (carrierAt / deckTopUnderJet /
    // carrierAt(player.pos...)).
    const updateStart = src.indexOf('function updateCamera');
    const updateSrc = src.slice(updateStart, updateStart + 8000);
    check(
      'C1 updateCamera does not use carriers[0] deckTopY for the chase/orbit/spectate deck ceiling',
      !updateSrc.includes('carrier.deckTopY'),
      'found "carrier.deckTopY" in updateCamera',
    );
    check(
      'C2 updateCamera resolves the deck under the jet (carrierAt or a deckTopUnderJet helper)',
      updateSrc.includes('carrierAt') || updateSrc.includes('deckTopUnderJet'),
      '',
    );
    // respawnPlayer's camOrbit must use the live carrier's yaw.
    const respawnStart = src.indexOf('function respawnPlayer');
    const respawnSrc = src.slice(respawnStart, respawnStart + 6000);
    check(
      'C3 respawnPlayer deck-spawn camOrbit uses liveCarrier yaw, not carriers[0] yaw',
      respawnSrc.includes('liveCarrier.yaw'),
      '',
    );
  }

  console.log(failures === 0 ? '\nALL HELICARRIER-CAMERA CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
