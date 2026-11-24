// probe-vtol-live.ts — VTOL live-path diagnostics (Pass 15, Worker D).
//
// probe-vtol §5 passes at the carrier START position (flat terrain,
// deckVy ~ 0), but the v4.9 live report shows the deck liftoff
// sinking and crashing. The carrier flies 40 m/s and terrain-follows
// with deckVy clamped to [-10, +15] m/s (ramped at 5 m/s^2) — a
// VTOL liftoff inherits that vertical velocity, and a level jet on a
// climbing deck reads AoA -20 deg (flight-path angle, not a wing
// stall) with CL -0.8 of wing downforce. This probe measures the
// LIVE deck (after a long carrier run over real terrain) and the
// failure modes the start-position probe cannot see:
//   D1. deckVy census over a long live carrier run (min/max, calm
//       fraction, time below -8 m/s).
//   D2. deck liftoff attempted in a BAD deck phase (deckVy < -8) and
//       in a CALM phase (|deckVy| < 2): does it lift, sink, crash?
//   D3. deep-hover AoA/G with a slow forward drift (7-12 kt, the
//       live numbers): what does the AoA formula read at near-zero
//       airspeed while climbing?
//   D4. VTOL->forward transition with NO pilot pull (the live case):
//       the main.ts vector floor lets the nozzle to 0 deg once
//       ias >= stallSpeed, even though the wing is unloaded at
//       level pitch — does the jet sink out / crash?
//
// Usage: node_modules/.bin/esbuild scripts/probe-vtol-live.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-vtol-live.mjs
//   && node /tmp/probe-vtol-live.mjs

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
import { clFromCurve } from '../src/physics/IAerodynamicsSolver';
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

const WIND_LIVE = new THREE.Vector3(3, 0, 0);
const DT_PATTERN = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];

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
    /** One live frame. script runs BEFORE the substeps (input). */
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
        if (this.vecTarget !== null) {
          const step = 36 * dt;
          const cur = this.player.vector;
          if (Math.abs(this.vecTarget - cur) <= step) {
            this.player.vector = this.vecTarget;
          } else {
            this.player.vector = cur + Math.sign(this.vecTarget - cur) * step;
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

  const alpha = RUNWAY_TABLE[0]!;
  const hoverThr = (SPECS['f35a']!.mass * 9.81) / SPECS['f35a']!.thrustMax;
  console.log(`F-35A hoverThr=${hoverThr.toFixed(4)} mass=${SPECS['f35a']!.mass} thrustMax=${SPECS['f35a']!.thrustMax}`);

  // --- D1. DeckVy census: run the carrier alone for 240 s wall. ---
  {
    console.log('--- D1: deckVy census, 240 s wall live run ---');
    originL.reset();
    carrierL.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
    // A dummy jet far away so the frame loop has a player to shift on;
    // the carrier update is what matters here.
    const s = new LiveVtol();
    await s.attach();
    s.player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    let minVy = Infinity;
    let maxVy = -Infinity;
    let n = 0;
    let nCalm = 0; // |vy| < 3
    let nBad = 0; // vy < -8
    let nClimb = 0; // vy > +8
    let sumAbs = 0;
    let nextLog = 0;
    while (s.wall < 240) {
      s.frame();
      const vy = carrierL.vel.y;
      minVy = Math.min(minVy, vy);
      maxVy = Math.max(maxVy, vy);
      n++;
      sumAbs += Math.abs(vy);
      if (Math.abs(vy) < 3) nCalm++;
      if (vy < -8) nBad++;
      if (vy > 8) nClimb++;
      if (s.wall >= nextLog) {
        nextLog += 30;
        console.log(
          `  [D1 t=${s.wall.toFixed(0)}s wall] deckTopY=${carrierL.deckTopY.toFixed(1)} vy=${vy.toFixed(1)} m/s pos=(${carrierL.pos.x.toFixed(0)}, ${carrierL.pos.z.toFixed(0)}) yaw=${((carrierL.yaw * 180) / Math.PI).toFixed(0)}`,
        );
      }
    }
    console.log(
      `  [D1] deckVy min=${minVy.toFixed(1)} max=${maxVy.toFixed(1)} mean|vy|=${(sumAbs / n).toFixed(1)} m/s; |vy|<3: ${((100 * nCalm) / n).toFixed(0)}% of frames, vy<-8: ${((100 * nBad) / n).toFixed(0)}%, vy>+8: ${((100 * nClimb) / n).toFixed(0)}%`,
    );
    check(
      'D1 deckVy census recorded (informational — fails only if no heave at all)',
      maxVy > 5 && minVy < -5,
      `range [${minVy.toFixed(1)}, ${maxVy.toFixed(1)}] m/s`,
    );
    s.dispose();
  }

  // Helper: deck-parked liftoff attempt. waitFor: 'bad' (vy < -8) or
  // 'calm' (|vy| < 2). Returns the outcome, including the deck
  // vertical velocity at the moment the jet first clears 4 m above
  // the deck (the pass-15 release gate must refuse vy < -3).
  const deckLiftoffAttempt = async (
    label: string,
    waitFor: 'bad' | 'calm',
  ): Promise<{ lifted: boolean; crashed: boolean; maxAbove: number; minRelVs: number; liftDeckVy: number | null }> => {
    console.log(`--- D2${label}: deck liftoff, waiting for ${waitFor} deck phase ---`);
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
    const DECK_SPAWN_AFT_M = 120;
    const spawnOnDeck = (): void => {
      const fwd = new THREE.Vector3(Math.sin(carrierL.yaw), 0, Math.cos(carrierL.yaw));
      const sx = carrierL.pos.x - fwd.x * DECK_SPAWN_AFT_M;
      const sz = carrierL.pos.z - fwd.z * DECK_SPAWN_AFT_M;
      s.player.resetAt(sx, carrierL.deckTopY + 2.2, sz, (carrierL.yaw * 180) / Math.PI);
      const dv = deckProviderL.velocityAt(sx, sz);
      s.player.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
      s.player.vel.copy(dv);
      s.player.gear = true;
    };
    spawnOnDeck();
    // Let the carrier run (jet rides the deck) until the wanted phase.
    let waited = 0;
    let phaseVy = 0;
    while (s.wall < 300) {
      s.frame();
      waited = s.wall;
      const vy = carrierL.vel.y;
      const ok = waitFor === 'bad' ? vy < -8 : Math.abs(vy) < 2;
      if (ok && waited > 5) {
        phaseVy = vy;
        break;
      }
      // Re-spawn on the deck if the jet slid off while waiting (it
      // should ride, but the wait can be long).
      if (!s.player.onSurface && s.player.alive) spawnOnDeck();
      if (!s.player.alive) {
        console.log(`  [D2${label}] jet died while waiting (reason=${s.player.lastCrashReason})`);
        s.dispose();
        return { lifted: false, crashed: true, maxAbove: 0, minRelVs: 0, liftDeckVy: null };
      }
    }
    console.log(
      `  [D2${label}] phase found after ${waited.toFixed(1)} s wall: deckVy=${phaseVy.toFixed(1)} m/s deckTopY=${carrierL.deckTopY.toFixed(1)}`,
    );
    s.setVectorTarget(90);
    s.player.thrTarget = 1.0;
    let maxAbove = 0;
    let minRelVs = Infinity;
    let liftedAt = -1;
    let liftDeckVy: number | null = null;
    let nextLog = s.wall + 1;
    const t0 = s.wall;
    while (s.wall < t0 + 30) {
      s.frame();
      const p = s.player;
      const above = p.pos.y - carrierL.deckTopY;
      maxAbove = Math.max(maxAbove, above);
      const relVs = p.vel.y - carrierL.vel.y;
      if (!p.onSurface) minRelVs = Math.min(minRelVs, relVs);
      if (liftedAt < 0 && above > 4) {
        liftedAt = s.wall - t0;
        liftDeckVy = carrierL.vel.y;
      }
      if (s.wall >= nextLog) {
        nextLog += 1;
        console.log(
          `    [D2${label} +${(s.wall - t0).toFixed(0)}s] aboveDeck=${above.toFixed(1)}m vel.y=${p.vel.y.toFixed(1)} deckVy=${carrierL.vel.y.toFixed(1)} relVs=${relVs.toFixed(1)} ias=${(p.ias * 1.944).toFixed(0)}kt aoa=${p.aoaDeg.toFixed(1)} G=${p.gload.toFixed(2)} vec=${p.vectorDeg.toFixed(0)} thr=${p.throttle.toFixed(3)} onSurf=${p.onSurface} alive=${p.alive}`,
        );
      }
      if (!p.alive) break;
    }
    const endAbove = s.player.pos.y - carrierL.deckTopY;
    console.log(
      `  [D2${label}] maxAbove=${maxAbove.toFixed(1)}m endAbove=${endAbove.toFixed(1)}m liftedAt=${liftedAt < 0 ? 'never' : liftedAt.toFixed(1) + 's'} liftDeckVy=${liftDeckVy === null ? 'n/a' : liftDeckVy.toFixed(1) + ' m/s'} minRelVs=${minRelVs === Infinity ? 'n/a' : minRelVs.toFixed(1) + ' m/s'} alive=${s.player.alive} reason=${s.player.lastCrashReason ?? '-'} lastCrashVs=${s.player.lastCrashVs.toFixed(2)}`,
    );
    const out = {
      lifted: s.player.alive && liftedAt >= 0 && endAbove > 3,
      crashed: !s.player.alive,
      maxAbove,
      minRelVs: minRelVs === Infinity ? 0 : minRelVs,
      liftDeckVy,
    };
    s.dispose();
    return out;
  };

  // D2a: bad phase first (reproduces the live crash, if the phase is
  // the cause). D2b: calm phase (control).
  const bad = await deckLiftoffAttempt('a-bad', 'bad');
  check(
    'D2a deck liftoff in BAD phase (deckVy < -8) lifts and survives',
    bad.lifted && !bad.crashed,
    `lifted=${bad.lifted} crashed=${bad.crashed} maxAbove=${bad.maxAbove.toFixed(1)}m minRelVs=${bad.minRelVs.toFixed(1)} m/s`,
  );
  check(
    'D2a liftoff releases only in a calm deck phase (deckVy >= -3 at liftoff)',
    bad.liftDeckVy !== null && bad.liftDeckVy >= -3,
    `liftDeckVy=${bad.liftDeckVy === null ? 'n/a' : bad.liftDeckVy.toFixed(1) + ' m/s'}`,
  );
  const calm = await deckLiftoffAttempt('b-calm', 'calm');
  check(
    'D2b deck liftoff in CALM phase (|deckVy| < 2) lifts and survives',
    calm.lifted && !calm.crashed,
    `lifted=${calm.lifted} crashed=${calm.crashed} maxAbove=${calm.maxAbove.toFixed(1)}m minRelVs=${calm.minRelVs.toFixed(1)} m/s`,
  );

  // --- D3. Deep-hover AoA/G with a slow forward drift. ---
  {
    console.log('--- D3: deep hover AoA/G, 7-12 kt drift, 100% throttle ---');
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
    s.player.resetAt(alpha.x, alpha.elev + 150, alpha.z, alpha.hdg);
    s.player.gear = false;
    s.player.throttle = hoverThr;
    s.player.thrTarget = hoverThr;
    s.player.vector = 90;
    while (s.wall < 5) s.frame();
    // Impose a slow sideways drift (5 m/s ~ 10 kt, the live speed)
    // on top of the hover, then push the throttle to 100%. The drift
    // is sideways so the air velocity exceeds the 5 m/s AoA threshold
    // while the forward component stays small — the case where the
    // old AoA formula read the climb rate as AoA (live: -18 deg).
    const v0 = s.player.vel.clone();
    s.player.body.setLinvel({ x: v0.x, y: v0.y, z: v0.z + 5 }, true);
    s.player.thrTarget = 1.0;
    let minAoA = Infinity;
    let maxAoA = -Infinity;
    let minG = Infinity;
    let maxG = -Infinity;
    let nextLog = s.wall + 1;
    const startAlt = s.player.pos.y;
    while (s.wall < 20) {
      s.frame();
      const p = s.player;
      minAoA = Math.min(minAoA, p.aoaDeg);
      maxAoA = Math.max(maxAoA, p.aoaDeg);
      minG = Math.min(minG, p.gload);
      maxG = Math.max(maxG, p.gload);
      if (s.wall >= nextLog) {
        nextLog += 1;
        console.log(
          `    [D3 +${(s.wall - 5).toFixed(0)}s] alt=${p.pos.y.toFixed(1)} vsi=${p.vsi.toFixed(2)} vel=(${p.vel.x.toFixed(1)}, ${p.vel.y.toFixed(1)}, ${p.vel.z.toFixed(1)}) ias=${(p.ias * 1.944).toFixed(1)}kt aoa=${p.aoaDeg.toFixed(1)} G=${p.gload.toFixed(2)} thr=${p.throttle.toFixed(3)}`,
        );
      }
      if (!p.alive) break;
    }
    const climbSim = (s.player.pos.y - startAlt) / s.sim;
    console.log(
      `  [D3] AoA range [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, G range [${minG.toFixed(2)}, ${maxG.toFixed(2)}], climb ${(s.player.pos.y - startAlt).toFixed(1)} m over ${(s.sim).toFixed(1)} s sim (${climbSim.toFixed(2)} m/s sim)`,
    );
    check(
      'D3 deep hover AoA stays sane (|AoA| < 5 deg) while climbing at 100%',
      s.player.alive && minAoA > -5 && maxAoA < 5,
      `AoA [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, G [${minG.toFixed(2)}, ${maxG.toFixed(2)}]`,
    );
    check(
      'D3 deep hover at 100% climbs firmly (sustained vsi >= +3.0 m/s)',
      s.player.alive && s.player.vsi >= 3.0,
      `sustained vsi=${s.player.vsi.toFixed(2)} m/s (${(s.player.vsi * 196.85).toFixed(0)} fpm), avg climb=${climbSim.toFixed(2)} m/s sim`,
    );
    s.dispose();
  }

  // --- D4. Transition with NO pilot pull (live case). ---
  // From a settled hover, slew the vector aft with the main.ts floor
  // (pass 15: the wing's potential lift at 10 deg AoA, or the actual
  // gload if pulling — the old "ias >= stallSpeed -> liftFrac 1"
  // floor let the nozzle to 0 while the wing was unloaded at level
  // pitch), hands-off pitch (pitchIn 0), full throttle. The live
  // transition crashed at 134 kt, -6,512 fpm after the vector
  // reached 0; the pre-fix probe reproduced it (141.6 m lost,
  // vsi -53.8 m/s).
  {
    console.log('--- D4: transition, no pilot pull, main.ts vector floor ---');
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
    s.player.resetAt(alpha.x, alpha.elev + 150, alpha.z, alpha.hdg);
    s.player.gear = false;
    s.player.throttle = hoverThr;
    s.player.thrTarget = hoverThr;
    s.player.vector = 90;
    while (s.wall < 5) s.frame();
    const hoverAlt = s.player.pos.y;
    s.player.thrTarget = 1.0;
    s.player.throttle = 1.0;
    let minAlt = hoverAlt;
    let vec = 90;
    let vecZeroAt = -1;
    let nextLog = s.wall + 1;
    const t0 = s.wall;
    while (s.wall < t0 + 40) {
      s.frame();
      // Slew the vector per frame at 36 deg/s of SIM time: each live
      // frame advances at most 0.1 s sim, so 3.6 deg/frame is the
      // main.ts slew (playerInput runs it per substep).
      // main.ts playerInput vector floor, pass-15 mirror: the wing's
      // POTENTIAL lift at 10 deg AoA at the current speed (or the
      // actual gload if the pilot is pulling) — not "1 above stall
      // speed", which let the nozzle to 0 while the wing was
      // unloaded at level pitch (the live transition crash).
      const p = s.player;
      const rho = 1.225 * Math.exp(-Math.max(p.pos.y, 0) / 8500);
      const q = 0.5 * rho * p.ias * p.ias;
      const sLift = p.spec.wingAreaEffective ?? p.spec.wingArea;
      const cl10 = clFromCurve(10, p.flaps / 30, p.spec.clCurve);
      const potential = (q * sLift * cl10) / (p.spec.mass * 9.81);
      const liftFrac = Math.max(
        0,
        Math.min(1, Math.max(potential, p.gload)),
      );
      const tw = (p.throttle * p.spec.thrustMax) / (p.spec.mass * 9.81);
      const need = (1 - liftFrac) / Math.max(tw, 0.01);
      const minVec = (Math.asin(Math.min(1, Math.max(0, need))) * 180) / Math.PI;
      const goal = Math.max(0, minVec);
      const stepDeg = 36 * 0.1;
      vec = Math.abs(goal - vec) <= stepDeg ? goal : vec + Math.sign(goal - vec) * stepDeg;
      p.vector = vec;
      if (vecZeroAt < 0 && vec === 0) vecZeroAt = s.wall - t0;
      p.pitchIn = 0;
      p.rollIn = 0;
      minAlt = Math.min(minAlt, p.pos.y);
      if (s.wall >= nextLog) {
        nextLog += 2;
        const pitch = new THREE.Euler().setFromQuaternion(p.quat, 'YXZ').x * (180 / Math.PI);
        const bank = new THREE.Euler().setFromQuaternion(p.quat, 'YXZ').z * (180 / Math.PI);
        const gamma = (Math.atan2(p.vel.y, Math.hypot(p.vel.x, p.vel.z)) * 180) / Math.PI;
        console.log(
          `    [D4 +${(s.wall - t0).toFixed(0)}s] alt=${p.pos.y.toFixed(1)} (min ${minAlt.toFixed(1)}) vsi=${p.vsi.toFixed(1)} ias=${(p.ias * 1.944).toFixed(0)}kt vec=${p.vectorDeg.toFixed(0)} aoa=${p.aoaDeg.toFixed(1)} pitch=${pitch.toFixed(1)} gamma=${gamma.toFixed(1)} bank=${bank.toFixed(1)} gear=${p.gear} G=${p.gload.toFixed(2)} thr=${p.throttle.toFixed(2)} alive=${p.alive}`,
        );
      }
      if (!p.alive) break;
      if (vec === 0 && p.ias > 75) break;
    }
    const spdKt = s.player.ias * 1.944;
    console.log(
      `  [D4] hoverAlt=${hoverAlt.toFixed(1)} minAlt=${minAlt.toFixed(1)} loss=${(hoverAlt - minAlt).toFixed(1)}m vecZeroAt=${vecZeroAt < 0 ? 'never' : vecZeroAt.toFixed(1) + 's'} final speed=${spdKt.toFixed(0)}kt vsi=${s.player.vsi.toFixed(1)} alive=${s.player.alive} reason=${s.player.lastCrashReason ?? '-'}`,
    );
    check(
      'D4 hands-off transition: no crash, alt loss < 25 m, ends in forward flight',
      s.player.alive && hoverAlt - minAlt < 25 && spdKt > 70,
      `loss=${(hoverAlt - minAlt).toFixed(1)}m speed=${spdKt.toFixed(0)}kt alive=${s.player.alive}`,
    );
    s.dispose();
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
