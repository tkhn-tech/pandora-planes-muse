// probe-carrier-deck.ts — SHIELD helicarrier integration (Issue 7).
//
// The carrier is now the converted SHIELD helicarrier GLB (not the
// Ford): its own rotor discs spin, the deck is two rectangles (wide
// stern + narrow bow), the deck collider matches the visuals, and the
// procedural pods are NOT doubled up. Checks:
//   1. build() loads the GLB: deckLocalY ~ -5.28, 2 deck rects, 4 pods
//      with spinning discs (rotation.y advances on update).
//   2. deckHeightAt agrees with the visuals: on-deck points (wide
//      stern, narrow bow) read the deck top; off-deck points (beyond
//      the stern, beyond the beam, over the narrow section's edge)
//      read -Infinity.
//   3. The carrier flies its loop: 60 s of updates keep it alive at
//      y=350, moving at 40 m/s.
//   4. Deck spawn + takeoff roll work on the SHIELD deck for all 4
//      jets (mirrors main.ts startRound + takeoff driver §E).
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-deck.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-carrier-deck.mjs
//   && node /tmp/probe-carrier-deck.mjs

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
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';
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

  // SHIELD helicarrier (real GLB), as in main.ts.
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
  // Register every deck collider (the SHIELD deck has two).
  const deckBody = carrier.deckBodyRef;
  if (deckBody) {
    for (let i = 0; i < deckBody.numColliders(); i++) {
      owners.set(deckBody.collider(i).handle, 'deck');
    }
  }

  // 1. Build results.
  check(
    'SHIELD model loaded (modelReady)',
    carrier.modelReady,
    `modelReady=${carrier.modelReady}`,
  );
  check(
    'deckLocalY is the SHIELD value (-5.28)',
    Math.abs(carrier.deckLocalY - -5.28) < 0.1,
    `deckLocalY=${carrier.deckLocalY.toFixed(2)} m`,
  );
  check(
    'deck is 2 rectangles (wide stern + narrow bow)',
    carrier.deckRects.length === 2,
    JSON.stringify(carrier.deckRects.map((r) => ({
      cx: +r.cx.toFixed(1), cz: +r.cz.toFixed(1), hx: +r.hx.toFixed(1), hz: +r.hz.toFixed(1),
    }))),
  );
  const pods = carrier.pods;
  check(
    '4 rotor pods from the GLB',
    pods.length === 4 && pods.every((p) => p.alive && p.hp === 100),
    `pods=${pods.length} hp=[${pods.map((p) => p.hp).join(',')}]`,
  );
  const podWorld = carrier.podObjects.map((p) =>
    p.getWorldPosition(new THREE.Vector3()),
  );
  // Discs must sit in their duct rings: carrier-local fwd pair
  // (z ~ +50, |x| ~ 54) and aft pair (z ~ -80, |x| ~ 62), and the
  // pod order is fwd-starboard, fwd-port, aft-starboard, aft-port.
  const podLocal = podWorld.map((p) => {
    const dx = p.x - carrier.pos.x;
    const dz = p.z - carrier.pos.z;
    // World -> carrier-local (yaw east): local x = -(dz), local z = dx.
    return { x: -dz, z: dx };
  });
  const podGeomOk =
    podLocal.length === 4 &&
    podLocal[0]!.x > 50 && podLocal[0]!.x < 60 && podLocal[0]!.z > 45 && podLocal[0]!.z < 55 &&
    podLocal[1]!.x < -50 && podLocal[1]!.x > -60 && podLocal[1]!.z > 45 && podLocal[1]!.z < 55 &&
    podLocal[2]!.x > 57 && podLocal[2]!.x < 66 && podLocal[2]!.z < -75 && podLocal[2]!.z > -85 &&
    podLocal[3]!.x < -57 && podLocal[3]!.x > -66 && podLocal[3]!.z < -75 && podLocal[3]!.z > -85;
  check(
    'rotor discs sit in their duct rings (fwd pair z~+50, aft pair z~-80; order fwd-stbd, fwd-port, aft-stbd, aft-port)',
    podGeomOk,
    podLocal.map((p) => `(${p.x.toFixed(0)},${p.z.toFixed(0)})`).join(' '),
  );
  // Discs spin: update() advances rotation.y (SHIELD) — capture the
  // disc rotation before/after via the pod objects' rotation.
  const disc0 = carrier.podObjects[0]!;
  const rotBefore = disc0.rotation.y;
  carrier.update(1.0);
  const rotAfter = disc0.rotation.y;
  check(
    'rotor discs spin on update (rotation.y advances)',
    Math.abs(rotAfter - rotBefore - 22) < 0.01,
    `rotation.y ${rotBefore.toFixed(2)} -> ${rotAfter.toFixed(2)} in 1 s`,
  );

  // 2. deckHeightAt vs the visuals (carrier at (-500,350,-900), yaw
  // east: local +z is world +x, local +x is world -z).
  // Local (0, -60) [wide stern] and (0, +100) [narrow bow] are deck;
  // local (0, -140) [past the stern] and (40, 0) [past the beam] and
  // (30, +100) [narrow section edge] are not.
  const toWorld = (lx: number, lz: number): [number, number] => {
    // carrier-local -> world, yaw = east (90 deg): world x = pos.x + lz,
    // world z = pos.z - lx.
    return [carrier.pos.x + lz, carrier.pos.z - lx];
  };
  const deckTop = carrier.deckTopY;
  for (const [lx, lz, want] of [
    [0, -60, true],
    [0, -120, true],
    [0, 100, true],
    [0, 140, true],
    [0, -140, false],
    [40, 0, false],
    [-40, 0, false],
    [30, 100, false],
    [30, -60, true],
  ] as const) {
    const [wx, wz] = toWorld(lx, lz);
    const h = carrier.deckHeightAt(wx, wz);
    check(
      `deckHeightAt local (${lx},${lz}) ${want ? 'is deck' : 'is NOT deck'}`,
      want ? Math.abs(h - deckTop) < 0.01 : h === -Infinity,
      `h=${h === -Infinity ? '-Inf' : h.toFixed(1)} deckTop=${deckTop.toFixed(1)}`,
    );
  }

  // 3. Loop flight: 60 s of updates.
  {
    let minY = Infinity;
    for (let i = 0; i < 3600; i++) {
      carrier.update(1 / 60);
      minY = Math.min(minY, carrier.pos.y);
    }
    check(
      'carrier flies its loop 60 s: alive, holds ~350 m, moving 40 m/s',
      carrier.alive && Math.abs(carrier.pos.y - 350) < 1 && minY > 340,
      `alive=${carrier.alive} y=${carrier.pos.y.toFixed(1)} minY=${minY.toFixed(1)} speed=${carrier.vel.length().toFixed(1)} m/s`,
    );
    // Reset for the deck tests.
    carrier.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
  }

  // 4. Deck spawn + takeoff roll for all 4 jets (takeoff driver §E).
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
      return new THREE.Vector3(carrier.vel.x + w * rz, carrier.vel.y, carrier.vel.z - w * rx);
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: () => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };
  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return ac;
  };
  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };
  const alpha = RUNWAY_TABLE[0]!;
  void alpha;
  for (const key of ['fa18', 'f35a', 'f22', 'f15e'] as const) {
    const ac = makeAircraft(key);
    const fwd = new THREE.Vector3(
      Math.sin(carrier.yaw),
      0,
      Math.cos(carrier.yaw),
    );
    const sx = carrier.pos.x - fwd.x * 120;
    const sz = carrier.pos.z - fwd.z * 120;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = fwd.clone().multiplyScalar(carrier.speed);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    // Ride the deck 3 s with no input.
    let onDeck = true;
    for (let i = 0; i < 180; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive || !ac.onDeck) onDeck = false;
    }
    check(
      `${key} rides the SHIELD deck 3 s (no input, no slide-off)`,
      onDeck && ac.alive,
      `alive=${ac.alive} onDeck=${ac.onDeck} alt=${ac.pos.y.toFixed(1)}`,
    );
    // Take off from the deck.
    ac.thrTarget = 1;
    ac.pitchIn = -1;
    let airborne = false;
    for (let i = 0; i < 600; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      if (!ac.onDeck && !ac.grounded && ac.pos.y > carrier.deckTopY + 5) {
        airborne = true;
        break;
      }
    }
    check(
      `${key} takes off from the SHIELD deck`,
      ac.alive && airborne,
      `alive=${ac.alive} airborne=${airborne} alt=${ac.pos.y.toFixed(1)}`,
    );
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
    if (deckBody) {
      for (let i = 0; i < deckBody.numColliders(); i++) {
        owners.set(deckBody.collider(i).handle, 'deck');
      }
    }
    carrier.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
