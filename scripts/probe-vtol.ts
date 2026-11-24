// probe-vtol.ts — F-35A VTOL verification (Issue 2).
//
// The lift-fan hover model existed (vector 90 + ~94% throttle balances
// at combat mass — takeoff driver §C) but there was no way to USE it:
// no key, no button, no hover FCS (qAuth -> 0 at zero airspeed, so the
// jet could not be steered in hover), and no HUD mode beyond the
// vector-angle pill. This driver checks the physics side:
//   1. hover vertical balance at 94% throttle (unchanged from §C),
//   2. pilot roll authority IN hover + hands-off auto-level,
//   3. forward-flight transition (gradual vector slew, as the toggle
//      does) without dropping out of the sky,
//   4. forward flight auto-levels hands-off (pass 7, item 2 —
//      was "no auto-level in forward flight"; the BET spiral
//      divergence made that unplayable, so forward flight now
//      returns the bank toward 0 with no roll input, like hover).
//   5. LIVE PATH (pass 14, item 13): the v4.8 live report — at 100%
//      throttle the F-35A does not climb altitude. Sections 1-2b pass
//      at fixed dt=1/60 with WIND=0, so the failure must be live-only:
//      variable frame dt (4-9 fps) + the 0.10 s sim cap + 3 m/s wind
//      + the real spawn (runway / SHIELD deck) + the real throttle
//      path (slider -> thrTarget setter -> pilotThr). Section 5 runs
//      that loop and checks (a) an airborne deep hover at 100%
//      climbs, (b) a deck-parked jet at vector 90 / 100% lifts off
//      and climbs, (c) the same from the runway (control — no deck
//      coupling there).
//
// Usage: node_modules/.bin/esbuild scripts/probe-vtol.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-vtol.mjs
//   && node /tmp/probe-vtol.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (Same stubs as probe-carrier-deck.ts /
// probe-live-round.ts, plus the global Image THREE's ImageLoader
// needs — without it every textured GLB fails with "Image is not
// defined" and attachModel falls back to the procedural stand-in.)
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
  (globalThis as unknown as { Image?: unknown }).Image = class ImageStub {
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    crossOrigin: string | null = null;
    width = 1;
    height = 1;
    private _src = '';
    get src(): string {
      return this._src;
    }
    set src(v: string) {
      this._src = v;
      queueMicrotask(() => {
        if (this.onload) this.onload();
      });
    }
    addEventListener(type: string, fn: () => void): void {
      if (type === 'load') this.onload = fn;
      if (type === 'error') this.onerror = fn;
    }
    removeEventListener(): void {}
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
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');

  const makeAircraft = (key: string): Aircraft => {
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
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  const alpha = RUNWAY_TABLE[0]!;
  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * (180 / Math.PI);
  const pitchOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * (180 / Math.PI);
  const spdOf = (ac: Aircraft): number => ac.vel.length();

  // 1. Hover vertical balance at 94% (mirrors takeoff driver §C).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.vector = 90;
    ac.thrTarget = 0.94;
    let maxAlt = 0;
    for (let i = 0; i < 300; i++) {
      substep(ac, 1 / 60);
      maxAlt = Math.max(maxAlt, ac.pos.y);
      if (!ac.alive) break;
    }
    check(
      'F-35A hover balances at 94% throttle, vector 90',
      ac.alive && ac.pos.y - alpha.elev < 3.5 && Math.abs(ac.vsi) < 0.5,
      `alt=${ac.pos.y.toFixed(2)} m (max ${maxAlt.toFixed(2)}) vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    dispose(ac);
  }

  // 2. Hover attitude: pilot authority + hands-off auto-level.
  // Airborne hover at 150 m (above the runway, clear of the deck).
  // The throttle is pre-spooled (a pilot engaging hover already has
  // it up — the 0.55/s spool from 0 would let the jet build a sink
  // rate that 94% thrust, a 0.2% margin over weight, cannot arrest).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 0.94;
    ac.thrTarget = 0.94;
    // Settle into the hover (throttle spools 0.55/s; the jet sags a
    // few metres first, then holds).
    for (let i = 0; i < 240; i++) substep(ac, 1 / 60);
    const hoverAlt = ac.pos.y;
    check(
      'F-35A airborne hover holds altitude at 94%',
      ac.alive && Math.abs(ac.vsi) < 1.5 && hoverAlt > 120,
      `alt=${hoverAlt.toFixed(1)} m vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    // Pass 9 (item 8): long hover — the live 40 s hover sank 335 ft
    // -> 321 ft (~20 fpm, 0.1 m/s) with the P loop alone. The
    // integral term must close that steady-state sink: after a
    // further 40 s the altitude drift must be under 3 m (vs ~4 m
    // live) and the final VSI near zero.
    const longStart = ac.pos.y;
    for (let i = 0; i < 2400; i++) substep(ac, 1 / 60);
    const longDrift = ac.pos.y - longStart;
    check(
      'F-35A 40 s hover: integral term closes the steady-state sink',
      ac.alive && Math.abs(longDrift) < 3 && Math.abs(ac.vsi) < 0.5,
      `drift=${longDrift.toFixed(2)} m over 40 s (live: -4.3 m) vsi=${ac.vsi.toFixed(2)} m/s alt=${ac.pos.y.toFixed(1)} m`,
    );
    // Pilot authority in hover: 0.3 s of full right roll must move
    // the bank (pre-FCS qAuth was ~0 at zero airspeed, so the stick
    // did nothing in hover).
    ac.rollIn = -1;
    for (let i = 0; i < 18; i++) substep(ac, 1 / 60);
    const bankAfterRoll = bankOf(ac);
    check(
      'F-35A pilot CAN roll in hover (stick moves the bank)',
      bankAfterRoll < -5,
      `bank=${bankAfterRoll.toFixed(1)} deg after 0.3 s full right roll`,
    );
    // Hands-off auto-level: release the stick; the FCS must bring
    // the bank back toward level within 3 s.
    ac.rollIn = 0;
    for (let i = 0; i < 180; i++) substep(ac, 1 / 60);
    const bankLevel = bankOf(ac);
    const pitchLevel = pitchOf(ac);
    check(
      'F-35A hover auto-level returns bank toward 0 hands-off',
      ac.alive && Math.abs(bankLevel) < 8,
      `bank=${bankLevel.toFixed(1)} deg pitch=${pitchLevel.toFixed(1)} deg after 3 s hands-off`,
    );
    dispose(ac);
  }

  // 2b. Hover altitude-hold (pass 6, item 8): the live F-35A at VEC
  // 90 and 100% throttle lifted off the deck but sank and went down
  // instead of hovering — the pilot cannot hold the ~94% hover point
  // precisely on a slider, so it porpoised and the sink half ended
  // in a hard landing. The throttle is now an altitude command in a
  // deep hover: 100% must climb gently (never sink), and a low
  // throttle must descend gently (a controlled descent, not a fall).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 1.0;
    ac.thrTarget = 1.0;
    const startAlt = ac.pos.y;
    for (let i = 0; i < 900; i++) substep(ac, 1 / 60);
    check(
      'F-35A hover at 100% throttle climbs, does not sink (altitude hold)',
      ac.alive && ac.pos.y >= startAlt - 2 && ac.vsi > -0.5,
      `alt=${ac.pos.y.toFixed(1)} m (start ${startAlt.toFixed(1)}) vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    dispose(ac);
  }
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 0.85;
    ac.thrTarget = 0.85;
    for (let i = 0; i < 600; i++) substep(ac, 1 / 60);
    check(
      'F-35A hover at 85% throttle descends gently, controlled (altitude hold)',
      ac.alive && ac.vsi > -6 && ac.pos.y > 100,
      `alt=${ac.pos.y.toFixed(1)} m vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    dispose(ac);
  }

  // 3. Transition: hover -> forward flight via the toggle's
  // lift-following vector slew (mirrors main.ts: the nozzle comes aft
  // at 36 deg/s but never below sin(vector) >= (1 - gload) / (T/W),
  // where gload is the wing's REAL lift right now — so the nozzle
  // cannot outrun the wing). The pilot rotates to ~10 deg AoA as
  // speed builds, as in a real STOVL transition: that is what makes
  // the wing take the load. The jet must accelerate past flying
  // speed without losing more than 25 m.
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, 150, alpha.z, alpha.hdg);
    ac.vector = 90;
    ac.throttle = 0.94;
    ac.thrTarget = 0.94;
    for (let i = 0; i < 240; i++) substep(ac, 1 / 60);
    const hoverAlt = ac.pos.y;
    ac.throttle = 1.0;
    ac.thrTarget = 1.0;
    let minAlt = hoverAlt;
    let vec = 90;
    for (let i = 0; i < 1200; i++) {
      // Toggle slew, lift-following floor (main.ts mirror).
      const liftFrac = Math.max(0, Math.min(1, ac.gload));
      const tw =
        (ac.throttle * ac.spec.thrustMax) / (ac.spec.mass * 9.81);
      const need = (1 - liftFrac) / Math.max(tw, 0.01);
      const minVec = (Math.asin(Math.min(1, Math.max(0, need))) * 180) / Math.PI;
      const goal = Math.max(0, minVec);
      const step = 36 / 60;
      vec =
        Math.abs(goal - vec) <= step
          ? goal
          : vec + Math.sign(goal - vec) * step;
      ac.vector = vec;
      // Pilot rotates to hold ~10 deg AoA as speed builds (the wing
      // only takes the load at AoA — level pitch never gets there).
      // pitchIn < 0 = nose up, so the error is (aoa - target).
      ac.pitchIn = Math.max(-1, Math.min(1, (ac.aoaDeg - 10) * 0.1));
      substep(ac, 1 / 60);
      minAlt = Math.min(minAlt, ac.pos.y);
      if (!ac.alive) break;
      if (vec === 0 && ac.vel.length() > 75) break;
    }
    const spd = spdOf(ac);
    check(
      'F-35A transition: hover -> forward flight without dropping',
      ac.alive && spd > 70 && hoverAlt - minAlt < 25,
      `speed=${(spd * 1.944).toFixed(0)} kt alt loss=${(hoverAlt - minAlt).toFixed(1)} m (hover ${hoverAlt.toFixed(1)} m, min ${minAlt.toFixed(1)} m)`,
    );
    dispose(ac);
  }

  // 4. Control: forward flight DOES auto-level hands-off (pass 7,
  // item 2). A banked F-18 with no input returns toward level —
  // the BET has no dihedral and only CLP = −2.0 of roll damping,
  // so without the auto-level any bank grows into a spiral dive
  // (v4.2: F-22 rolled inverted hands-off, Bank 165–177 deg).
  // This check used to assert NO auto-level in forward flight
  // (the bank persists); that was the old behaviour and the bug.
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
    ac.setAirspeed(150);
    ac.thrTarget = 0.8;
    ac.rollIn = -1;
    for (let i = 0; i < 12; i++) substep(ac, 1 / 60); // ~30 deg bank
    ac.rollIn = 0;
    const bank0 = bankOf(ac);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    const bank1 = bankOf(ac);
    check(
      'F-18 forward flight: auto-level returns bank toward 0 hands-off (pass 7, item 2)',
      Math.abs(bank1) < Math.abs(bank0) - 5,
      `bank ${bank0.toFixed(1)} deg -> ${bank1.toFixed(1)} deg over 2 s hands-off`,
    );
    dispose(ac);
  }

  // --- 5. LIVE PATH (pass 14, item 13): 100% throttle must climb. ---
  // Same loop the game runs: variable frame dt (4-9 fps), simDt
  // capped at 0.10 s, 3 m/s wind, FloatingOrigin shifts, real GLB
  // attach, real spawn (Alpha runway / SHIELD deck), throttle via
  // the slider path (thrTarget setter -> pilotThr) and the vector
  // via main.ts's 36 deg/s slew. The fixed-dt sections above pass,
  // so any failure here is live-only.
  {
    const WIND_LIVE = new THREE.Vector3(3, 0, 0);
    const DT_PATTERN = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
    const worldL = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    const eventQL = new RAPIER.EventQueue(true);
    const terrainBodyL = worldL.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    const hfL = new Float32Array(heights.length);
    for (let iz = 0; iz < res; iz++) {
      for (let ix = 0; ix < res; ix++) {
        hfL[ix * res + iz] = heights[iz * res + ix]!;
      }
    }
    const terrainColliderL = worldL.createCollider(
      RAPIER.ColliderDesc.heightfield(
        TERRAIN_SEGMENTS,
        TERRAIN_SEGMENTS,
        hfL,
        new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
      )
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBodyL,
    );
    const sceneL = new THREE.Scene();
    const ownersL = new Map<number, Aircraft | 'terrain' | 'deck'>();
    ownersL.set(terrainColliderL.handle, 'terrain');
    for (const r of RUNWAY_TABLE) {
      const box = worldL.createCollider(
        RAPIER.ColliderDesc.cuboid(r.width / 2 + 2, 0.5, r.len / 2 + 100)
          .setTranslation(r.x, r.elev - 0.45, r.z)
          .setFriction(0)
          .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
          .setRestitution(0)
          .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
        terrainBodyL,
      );
      ownersL.set(box.handle, 'terrain');
    }
    const originL = new FloatingOrigin(2000);
    originL.registerBody(terrainBodyL);
    const heightAtL = (x: number, z: number): number =>
      heightfieldHeightAt(
        heights,
        res,
        x + originL.cumulativeShift.x,
        z + originL.cumulativeShift.z,
      );
    const carrierL = new HelicarrierController(sceneL, worldL, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: heightAtL,
    });
    await carrierL.build();
    const deckBodyL = carrierL.deckBodyRef;
    if (deckBodyL) {
      for (let i = 0; i < deckBodyL.numColliders(); i++) {
        ownersL.set(deckBodyL.collider(i).handle, 'deck');
      }
    }
    carrierL.registerWithFloatingOrigin(originL);
    const deckProviderL: DeckProvider = {
      get alive() {
        return carrierL.active && carrierL.alive;
      },
      deckHeightAt: (x, z) => carrierL.deckHeightAt(x, z),
      velocity: () => carrierL.vel.clone(),
      velocityAt: (x, z) => {
        const rx = x - carrierL.pos.x;
        const rz = z - carrierL.pos.z;
        const w = carrierL.yawRate;
        return new THREE.Vector3(
          carrierL.vel.x + w * rz,
          carrierL.vel.y,
          carrierL.vel.z - w * rx,
        );
      },
      yawRate: () => carrierL.yawRate,
      yawRateAt: (_x, _z) => carrierL.yawRate,
      local: (x, z) => {
        const v = carrierL.localPosition(new THREE.Vector3(x, 0, z));
        return v ? { x: v.x, z: v.z } : null;
      },
    };
    console.log(
      `  [live] carrier model: ${carrierL.modelStatus ?? 'built'} deckTopY=${carrierL.deckTopY.toFixed(1)}`,
    );
    const drainL = (): void => {
      eventQL.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const a = ownersL.get(h1);
        const b = ownersL.get(h2);
        const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
        if (!hit || !hit.alive) return;
        const other = hit === a ? b : a;
        hit.notifyContact(hit.vel.y, other === 'deck');
      });
    };
    const hoverThr = (SPECS['f35a']!.mass * 9.81) / SPECS['f35a']!.thrustMax;
    console.log(`  [live] F-35A hoverThr=${hoverThr.toFixed(4)} (mass ${SPECS['f35a']!.mass} kg, thrustMax ${SPECS['f35a']!.thrustMax} N)`);

    // One live session: fresh F-35A, real GLB, the main.ts frame loop
    // (shift -> substeps -> render wall time), vector via the
    // main.ts 36 deg/s slew, throttle via the slider path.
    class LiveVtol {
      player: Aircraft;
      wall = 0;
      sim = 0;
      private pi = 0;
      private vecTarget: number | null = null;
      constructor() {
        const spec = SPECS['f35a']!;
        this.player = new Aircraft({
          scene: sceneL,
          world: worldL,
          spec,
          isPlayer: true,
          modelKey: spec.modelKey,
          deck: deckProviderL,
        });
        this.player.terrainHeightAt = heightAtL;
        originL.registerBody(this.player.body);
        for (const h of this.player.colliderHandles()) ownersL.set(h, this.player);
      }
      async attach(): Promise<void> {
        await this.player.attachModel();
      }
      setVectorTarget(deg: number): void {
        this.vecTarget = deg;
      }
      frame(script?: (s: LiveVtol) => void): void {
        const rawDt = Math.min(DT_PATTERN[this.pi % DT_PATTERN.length]!, 0.25);
        this.pi++;
        if (script) script(this);
        const shift = originL.shiftIfNeeded(this.player.pos);
        if (shift) originL.applyShift(shift);
        const simDt = Math.min(rawDt, 0.1);
        const sub = Math.max(1, Math.ceil(simDt / 0.05));
        const dt = simDt / sub;
        for (let k = 0; k < sub; k++) {
          // main.ts playerInput VTOL slew (36 deg/s toward target).
          if (this.vecTarget !== null) {
            const step = 36 * dt;
            const cur = this.player.vector;
            if (Math.abs(this.vecTarget - cur) <= step) {
              this.player.vector = this.vecTarget;
            } else {
              this.player.vector =
                cur + Math.sign(this.vecTarget - cur) * step;
            }
          }
          carrierL.update(dt);
          worldL.timestep = dt;
          worldL.step(eventQL);
          drainL();
          this.player.step(dt, WIND_LIVE, null);
          this.sim += dt;
        }
        this.wall += rawDt;
      }
      dispose(): void {
        originL.unregisterBody(this.player.body);
        for (const h of this.player.colliderHandles()) ownersL.delete(h);
        this.player.dispose();
      }
    }

    // 5a. Airborne deep hover, 100% throttle — must climb.
    {
      console.log('--- 5a LIVE: airborne hover VEC 90, throttle 100% ---');
      originL.reset();
      carrierL.reset(
        CARRIER_START_POS.clone(),
        Math.atan2(
          CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
          CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
        ),
      );
      const s = new LiveVtol();
      await s.attach();
      console.log(`  [5a] model: ${s.player.modelStatus} tris=${s.player.modelTris}`);
      s.player.resetAt(alpha.x, alpha.elev + 150, alpha.z, alpha.hdg);
      s.player.gear = false;
      // Pre-spooled at the hover point, nozzle ALREADY at 90 — the
      // case is a jet IN a deep hover, not a transition into one.
      // (A 36 deg/s slew from 0 departs forward-down before the
      // nozzle reaches 90: +21 m/s, -9 m/s sink, AoA 30 deg — that
      // is the transition, not the hover hold. The live pilot
      // arrives in hover the same way the fixed-dt §2b case does:
      // vector 90 before stepping.)
      s.player.throttle = hoverThr;
      s.player.thrTarget = hoverThr;
      s.player.vector = 90;
      // Settle 5 s wall into the hover.
      while (s.wall < 5) s.frame();
      console.log(
        `  [5a] hover settled: alt=${s.player.pos.y.toFixed(1)} vsi=${s.player.vsi.toFixed(2)} vel=(${s.player.vel.x.toFixed(1)}, ${s.player.vel.y.toFixed(1)}, ${s.player.vel.z.toFixed(1)}) ias=${(s.player.ias * 1.944).toFixed(1)}kt vec=${s.player.vectorDeg.toFixed(0)} thr=${s.player.throttle.toFixed(3)} thrT=${s.player.thrTarget.toFixed(3)} pilotThr=${s.player.pilotThr.toFixed(3)} onSurf=${s.player.onSurface}`,
      );
      const startAlt = s.player.pos.y;
      const startSim = s.sim;
      // Push the slider to 100% (the slider path: thrTarget setter
      // records pilotThr).
      s.player.thrTarget = 1.0;
      let minAlt = startAlt;
      let maxThr = 0;
      let lastPilotThr = 0;
      let nextLog = s.wall + 1;
      while (s.wall < 20) {
        s.frame();
        minAlt = Math.min(minAlt, s.player.pos.y);
        maxThr = Math.max(maxThr, s.player.throttle);
        lastPilotThr = s.player.pilotThr;
        if (s.wall >= nextLog) {
          nextLog += 1;
          const p = s.player;
          const pitch = new THREE.Euler().setFromQuaternion(p.quat, 'YXZ').x * (180 / Math.PI);
          const bank = new THREE.Euler().setFromQuaternion(p.quat, 'YXZ').z * (180 / Math.PI);
          console.log(
            `    [5a +${(s.wall - 5).toFixed(0)}s] alt=${p.pos.y.toFixed(1)} vsi=${p.vsi.toFixed(2)} pitch=${pitch.toFixed(1)} bank=${bank.toFixed(1)} vel=(${p.vel.x.toFixed(1)}, ${p.vel.y.toFixed(1)}, ${p.vel.z.toFixed(1)}) ias=${(p.ias * 1.944).toFixed(1)}kt aoa=${p.aoaDeg.toFixed(1)} vec=${p.vectorDeg.toFixed(0)} thr=${p.throttle.toFixed(3)} thrT=${p.thrTarget.toFixed(3)} pilotThr=${p.pilotThr.toFixed(3)} onSurf=${p.onSurface}`,
          );
        }
        if (!s.player.alive) break;
      }
      const endAlt = s.player.pos.y;
      const simT = s.sim - startSim;
      const wallT = 15;
      const climbSim = (endAlt - startAlt) / simT;
      const climbWall = (endAlt - startAlt) / wallT;
      console.log(
        `  [5a] alt ${startAlt.toFixed(1)} -> ${endAlt.toFixed(1)} m (min ${minAlt.toFixed(1)}) over ${simT.toFixed(1)} s sim / ${wallT} s wall: climb ${climbSim.toFixed(2)} m/s sim, ${climbWall.toFixed(2)} m/s wall; vsi=${s.player.vsi.toFixed(2)} throttle=${s.player.throttle.toFixed(3)} (max ${maxThr.toFixed(3)}) thrTarget(auto)=${s.player.thrTarget.toFixed(3)} pilotThr=${lastPilotThr.toFixed(3)} vector=${s.player.vectorDeg.toFixed(0)} ias=${(s.player.ias * 1.944).toFixed(1)}kt onSurface=${s.player.onSurface} alive=${s.player.alive}`,
      );
      check(
        '5a LIVE airborne hover at 100% throttle climbs >= +1.0 m/s (sim time)',
        s.player.alive && climbSim >= 1.0,
        `climb=${climbSim.toFixed(2)} m/s sim (${climbWall.toFixed(2)} m/s wall), vsi=${s.player.vsi.toFixed(2)}`,
      );
      s.dispose();
    }

    // 5b. Parked on the SHIELD deck, vector 90, throttle 100% — must
    // lift off and climb. (The deck pre-coupling is the prime
    // suspect: it overwrites newVel.y with the deck-following value
    // while in the band, which erases the thrust-driven climb.)
    {
      console.log('--- 5b LIVE: deck-parked VEC 90, throttle 100% ---');
      originL.reset();
      carrierL.reset(
        CARRIER_START_POS.clone(),
        Math.atan2(
          CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
          CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
        ),
      );
      const s = new LiveVtol();
      await s.attach();
      console.log(`  [5b] model: ${s.player.modelStatus} tris=${s.player.modelTris}`);
      // startRound deck spawn, verbatim: aft of deck centre, facing
      // the bow, inheriting the deck-point velocity.
      const DECK_SPAWN_AFT_M = 120;
      const fwd = new THREE.Vector3(
        Math.sin(carrierL.yaw),
        0,
        Math.cos(carrierL.yaw),
      );
      const sx = carrierL.pos.x - fwd.x * DECK_SPAWN_AFT_M;
      const sz = carrierL.pos.z - fwd.z * DECK_SPAWN_AFT_M;
      s.player.resetAt(sx, carrierL.deckTopY + 2.2, sz, (carrierL.yaw * 180) / Math.PI);
      const dv = deckProviderL.velocityAt(sx, sz);
      s.player.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
      s.player.vel.copy(dv);
      s.player.gear = true;
      const deckTop0 = carrierL.deckTopY;
      console.log(
        `  [5b] spawned on deck: y=${s.player.pos.y.toFixed(1)} deckTop=${deckTop0.toFixed(1)} deckVel=(${dv.x.toFixed(1)}, ${dv.y.toFixed(1)}, ${dv.z.toFixed(1)})`,
      );
      s.setVectorTarget(90);
      // Throttle slider to 100% from idle (spool 0.55/s sim).
      s.player.thrTarget = 1.0;
      let maxAboveDeck = 0;
      let leftDeckAt = -1;
      let maxThr = 0;
      // NOTE: spawn height IS deckTop+2.2 (gear rest height), so the
      // liftoff bar must be well clear of it: > 4 m means the gear
      // is genuinely off the deck (the old > 2 m bar passed at
      // spawn while the jet sat pinned at 2.20 m for 30 s).
      while (s.wall < 30) {
        s.frame();
        const above = s.player.pos.y - carrierL.deckTopY;
        maxAboveDeck = Math.max(maxAboveDeck, above);
        if (leftDeckAt < 0 && above > 4) leftDeckAt = s.wall;
        maxThr = Math.max(maxThr, s.player.throttle);
        if (!s.player.alive) break;
      }
      const endAbove = s.player.pos.y - carrierL.deckTopY;
      console.log(
        `  [5b] max above deck=${maxAboveDeck.toFixed(2)} m (end ${endAbove.toFixed(2)} m) leftDeckAt=${leftDeckAt < 0 ? 'never' : leftDeckAt.toFixed(1) + ' s wall'} throttle max=${maxThr.toFixed(3)} vector=${s.player.vectorDeg.toFixed(0)} pilotThr=${s.player.pilotThr.toFixed(3)} onSurface=${s.player.onSurface} alive=${s.player.alive} reason=${s.player.lastCrashReason ?? '-'}`,
      );
      check(
        '5b LIVE deck liftoff at 100% throttle leaves the deck (> 4 m) and climbs',
        s.player.alive && leftDeckAt >= 0 && endAbove > 3,
        `max above deck=${maxAboveDeck.toFixed(2)} m (live: never lifts off)`,
      );
      s.dispose();
    }

    // 5c. Runway liftoff control: same inputs, Alpha runway (no deck
    // coupling). If this climbs and 5b does not, the deck coupling
    // is the cause, not the hover hold.
    {
      console.log('--- 5c LIVE: runway-parked VEC 90, throttle 100% (control) ---');
      originL.reset();
      carrierL.reset(
        CARRIER_START_POS.clone(),
        Math.atan2(
          CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
          CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
        ),
      );
      const s = new LiveVtol();
      await s.attach();
      s.player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      s.player.gear = true;
      s.setVectorTarget(90);
      s.player.thrTarget = 1.0;
      let maxAbove = 0;
      let leftAt = -1;
      while (s.wall < 20) {
        s.frame();
        const above = s.player.pos.y - alpha.elev;
        maxAbove = Math.max(maxAbove, above);
        if (leftAt < 0 && above > 5) leftAt = s.wall;
        if (!s.player.alive) break;
      }
      console.log(
        `  [5c] max above runway=${maxAbove.toFixed(2)} m leftAt=${leftAt < 0 ? 'never' : leftAt.toFixed(1) + ' s wall'} throttle=${s.player.throttle.toFixed(3)} vector=${s.player.vectorDeg.toFixed(0)} pilotThr=${s.player.pilotThr.toFixed(3)} onSurface=${s.player.onSurface} alive=${s.player.alive}`,
      );
      check(
        '5c LIVE runway liftoff at 100% throttle leaves the ground (> 5 m)',
        s.player.alive && leftAt >= 0,
        `max above runway=${maxAbove.toFixed(2)} m`,
      );
      s.dispose();
    }
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
