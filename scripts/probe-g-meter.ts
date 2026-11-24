// probe-g-meter.ts — Pass 15, worker E, item 13 (G METER WRONG).
//
// v4.9 live: the G meter reads 0.0 parked in some spawns (surf=0) versus
// 1.0 in normal spawns, and 0.1-0.3 in steep climbs.
//
// Measured here, through the live path (real Aircraft / BET, real Rapier
// world + runway boxes, live wind (-3, 0, 0), variable dt):
//   * gload is the SOLVER's wing-lift load factor: lift / (m*g), where
//     lift is the wing lift only (BET: q*S*CL from the stations; curve:
//     same, floored at 0.2). It knows nothing about thrust, drag, or the
//     ground. So it reads ~0 whenever the wing is not flying, even when
//     the pilot is being pressed into the seat:
//       - parked, surf=0 (no grounded override): v ~ 0 -> q ~ 0 ->
//         lift ~ 0 -> G ~ 0.0. A normally parked jet reads 1.0 ONLY
//         because the controller overrides gload to 1.0 when grounded.
//         The parked 0.0 is therefore the item-6 frozen state's symptom
//         (surf=0 while parked), not a second G bug.
//       - steep climb: the velocity aligns with the nose (AoA ~ 0-10
//         deg once the jet is pointing up and flying up), so the wing
//         lift collapses toward 0 while the THRUST carries the jet —
//         the lift-only meter reads 0.1-0.3 although the pilot feels
//         ~1 g (thrust) plus the pull.
//   * The quantity a G meter actually measures is the specific force
//     the airframe/pilot feels: the MAGNITUDE of the non-gravitational
//     forces, |lift + drag + thrust| / (m*g). In ANY steady flight
//     (level, climb, dive, hover) the force sum balances weight, so
//     that reads 1.0; in a pull it is 2-4; in free fall it is 0
//     (correct — weightless); in a VTOL hover it is ~0.94-1.0 (the
//     fan carries it). (The component along body-up is NOT it: in a
//     vertical climb the thrust acts along the spine, through the
//     seat back, so the body-up component is ~0 even though the
//     pilot feels ~1 g.) The fix exposes that magnitude as
//     Aircraft.gDisplay and the HUD reads it.
//   * `gload` (lift-only in forward flight) is UNCHANGED — it must
//     not become the display value. main.ts's VTOL vector floor
//     reads p.gload as the wing's real lift share
//     (liftFrac = max(potential, p.gload)); if gload became the
//     specific-force magnitude it would read ~1.0 at any vector, the
//     floor would let the nozzle all the way down at 0 kt, and the
//     jet would drop. (Pass 15 item 5 already changed gload in the
//     vectored regime only, vector >= 70, to the total vertical
//     force — that escape is bounded: below 70 gload reverts to
//     wing-only and the floor catches the nozzle at ~70 deg, where
//     vertical thrust still carries the jet. A magnitude gload would
//     have no such bound.) The over-G check also reads gload as the
//     wing-bending load.
//
// Checks read ac.gDisplay (absent pre-fix -> FAIL, present post-fix).
// The lift-vs-specific comparison is printed in every section so the
// explanation is measured, not asserted.
//
// Usage: node_modules/.bin/esbuild scripts/probe-g-meter.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-g-meter.mjs
//   && node /tmp/probe-g-meter.mjs
//   (delete /tmp/probe-g-meter.mjs after running)

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
import { BladeElementSolver } from '../src/physics/BladeElementSolver';
import {
  ZERO_DAMAGE,
  clFromCurve,
  type AeroState,
} from '../src/physics/IAerodynamicsSolver';
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

const MS2KT = 1.94384;
const KT2MS = 1 / MS2KT;
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G = 9.81;
const WIND = new THREE.Vector3(-3, 0, 0);
const DT_PATTERN = [1 / 60, 1 / 50, 1 / 40, 1 / 30];

/** The display-G definition, computed independently from a solver
 *  result: the specific-force magnitude |lift + drag + thrust| / (m*g).
 *  Used to cross-check ac.gDisplay (the fix must agree with this to
 *  0.05). */
function specificG(
  lift: THREE.Vector3,
  drag: THREE.Vector3,
  thrust: THREE.Vector3,
  _up: THREE.Vector3,
  mass: number,
): number {
  const f = lift.clone().add(drag).add(thrust);
  return f.length() / (mass * G);
}

function pitchUpDeg(ac: Aircraft): number {
  return -new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * R2D;
}
function gammaDeg(ac: Aircraft): number {
  const v = ac.vel.length();
  if (v < 1) return 0;
  return Math.asin(Math.max(-1, Math.min(1, ac.vel.y / v))) * R2D;
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
  const owners = new Map<number, Aircraft | 'terrain'>();
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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const alpha = RUNWAY_TABLE[0]!;
  const deck: DeckProvider = {
    alive: false,
    deckHeightAt: () => -Infinity,
    velocity: () => new THREE.Vector3(),
    velocityAt: () => new THREE.Vector3(),
    yawRate: () => 0,
    yawRateAt: () => 0,
    local: () => null,
  };
  let dtIdx = 0;
  const nextDt = (): number => {
    const dt = DT_PATTERN[dtIdx % DT_PATTERN.length]!;
    dtIdx++;
    return dt;
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
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const substep = (ac: Aircraft, dt: number): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, WIND, null);
  };
  const dispose = (ac: Aircraft): void => {
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  };
  /** ac.gDisplay when the fix exists, NaN when it does not. */
  const gDisplayOf = (ac: Aircraft): number => {
    const v = (ac as unknown as { gDisplay?: unknown }).gDisplay;
    return typeof v === 'number' ? v : NaN;
  };

  // ---- 0. Solver-level: lift-G vs specific-G on constructed states ----
  console.log('--- 0. Solver-level: lift-G vs specific-G (BET, F-18) ---');
  {
    const spec = SPECS['fa18']!;
    const bet = new BladeElementSolver(spec);
    const mkState = (vAir: THREE.Vector3, quat: THREE.Quaternion, throttle: number): AeroState => ({
      velocityAir: vAir,
      quat,
      mass: spec.mass,
      wingArea: spec.wingArea,
      wingAreaLift: spec.wingAreaEffective ?? spec.wingArea,
      span: spec.span,
      length: spec.length,
      throttle,
      thrustMax: spec.thrustMax,
      vectorDeg: 0,
      flapsDeg: 0,
      airbrake: 0,
      gearDown: false,
      damage: { ...ZERO_DAMAGE },
      altitude: 400,
      density: 1.225,
      archetype: spec.archetype,
      controls: { pitch: 0, roll: 0, yaw: 0 },
      controlAuthority: { turn: spec.turn, rollDeg: spec.rollDeg, pitch: spec.pitch },
      inducedK: spec.inducedK,
      clCurve: spec.clCurve,
      angVel: new THREE.Vector3(),
      tvcDeg: spec.tvcDeg,
      rollCapSchedule: spec.rollCapSchedule,
      dt: 1 / 60,
    });
    const up = new THREE.Vector3(0, 1, 0);
    const v129 = new THREE.Vector3(0, 0, 129);
    // Level flight at 250 kt, trimmed: search the nose-up pitch that
    // makes the wing carry the weight (lift-G ~ 1) at throttle 0.2.
    // (A constructed state at AoA 0 is NOT level flight — the wing
    // at AoA 0 makes ~0.4 g and the jet would sink.)
    let rLevel: ReturnType<typeof bet.solve> | null = null;
    let specLevel = NaN;
    for (let pitchDeg = 0; pitchDeg <= 8; pitchDeg += 0.25) {
      const qLevel = new THREE.Quaternion().setFromEuler(
        new THREE.Euler(-pitchDeg * D2R, 0, 0, 'YXZ'),
      );
      const r = bet.solve(mkState(v129, qLevel, 0.2));
      if (r.gLoad >= 0.95 && r.gLoad <= 1.05) {
        rLevel = r;
        specLevel = specificG(r.lift, r.drag, r.thrust, up, spec.mass);
        console.log(
          `  level 250kt (trim pitch ${pitchDeg.toFixed(2)}deg): lift-G=${r.gLoad.toFixed(2)} specific-G=${specLevel.toFixed(2)} (both ~1)`,
        );
        break;
      }
    }
    if (!rLevel) {
      console.log('  level 250kt: no trim pitch found in 0-8 deg');
    }
    // Vertical climb: nose up 90 deg, velocity straight up at 129 m/s,
    // throttle 1.0. The wing sees AoA ~ 0 -> lift ~ 0; the thrust
    // carries the jet. A lift-only meter reads ~0; specific force
    // reads (T - D)/mg ~ 1.
    const qUp90 = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(-90 * D2R, 0, 0, 'YXZ'),
    );
    const vUp = new THREE.Vector3(0, 129, 0);
    const rClimb = bet.solve(mkState(vUp, qUp90, 1.0));
    const upClimb = new THREE.Vector3(0, 0, 1).applyQuaternion(qUp90); // body-up in world = nose dir here
    const specClimb = specificG(rClimb.lift, rClimb.drag, rClimb.thrust, upClimb, spec.mass);
    console.log(
      `  vertical climb 250kt: lift-G=${rClimb.gLoad.toFixed(2)} specific-G=${specClimb.toFixed(2)} (lift ~ 0, thrust carries it)`,
    );
    check(
      'solver level flight: lift-G and specific-G agree (~1.0) when trimmed',
      rLevel !== null && Math.abs(rLevel.gLoad - 1) < 0.1 && Math.abs(specLevel - 1) < 0.3,
      rLevel ? `lift-G=${rLevel.gLoad.toFixed(2)} specific-G=${specLevel.toFixed(2)}` : 'no trim found',
    );
    check(
      'solver vertical climb: lift-G collapses (< 0.5) while specific-G stays sane (0.5-1.5) — the live 0.1-0.3 signature',
      rClimb.gLoad < 0.5 && specClimb > 0.5 && specClimb < 1.5,
      `lift-G=${rClimb.gLoad.toFixed(2)} specific-G=${specClimb.toFixed(2)}`,
    );
  }

  // Level-flight AoA law (probe-aoa-g): hold the AoA level flight needs.
  const rhoAt = (altM: number): number => 1.225 * Math.exp(-Math.max(altM, 0) / 8500);
  const aoaForCL = (key: string, clTarget: number): number => {
    const spec = SPECS[key]!;
    const peak = spec.clCurve.peakAoA;
    if (clTarget <= 0.15) return 0;
    let lo = 0;
    let hi = peak;
    for (let i = 0; i < 40; i++) {
      const mid = (lo + hi) / 2;
      if (clFromCurve(mid, 0, spec.clCurve) < clTarget) lo = mid;
      else hi = mid;
    }
    return (lo + hi) / 2;
  };
  const levelPitchIn = (ac: Aircraft, key: string, alt0: number): number => {
    const spec = SPECS[key]!;
    const sLift = spec.wingAreaEffective ?? spec.wingArea;
    const rho = rhoAt(ac.pos.y);
    const q = 0.5 * rho * Math.max(ac.ias, 1) * Math.max(ac.ias, 1);
    const clReq = (spec.mass * 9.81) / Math.max(q * sLift, 1);
    let aoaTarget = aoaForCL(key, clReq);
    aoaTarget += Math.max(-4, Math.min(4, (alt0 - ac.pos.y) * 0.03));
    aoaTarget = Math.min(aoaTarget, spec.clCurve.peakAoA - 2);
    const raw = Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3));
    return raw < 0 ? Math.min(raw, -0.3) : raw;
  };

  // ---- 1. Level flight F-18 at 250 kt ----
  console.log('--- 1. Level flight F-18 at 250 kt (live path) ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(250 * KT2MS);
    ac.thrTarget = 0.6;
    const alt0 = ac.pos.y;
    let simT = 0;
    let gSum = 0;
    let gdSum = 0;
    let n = 0;
    while (simT < 15) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, 'fa18', alt0);
      ac.thrTarget = Math.max(0, Math.min(1, 0.6 + (250 - ac.ias * MS2KT) * 0.05));
      substep(ac, dt);
      if (!ac.alive) break;
      if (simT > 5) {
        gSum += ac.gload;
        gdSum += gDisplayOf(ac);
        n++;
      }
    }
    const gMean = n ? gSum / n : NaN;
    const gdMean = n ? gdSum / n : NaN;
    console.log(`  level: lift-G mean=${gMean.toFixed(2)}, gDisplay mean=${Number.isNaN(gdMean) ? 'MISSING (pre-fix)' : gdMean.toFixed(2)}`);
    check(
      'level flight: gDisplay reads 1.0 (0.8-1.2)',
      !Number.isNaN(gdMean) && gdMean > 0.8 && gdMean < 1.2,
      `gDisplay mean=${Number.isNaN(gdMean) ? 'MISSING' : gdMean.toFixed(2)}`,
    );
    dispose(ac);
  }

  // ---- 2. Steep climb: F-18 from level 400 kt, pull up and HOLD the climb ----
  // The live 0.1-0.3 is a STEADY steep climb, not a pull: once the jet
  // is pointing up and flying up, the velocity aligns with the nose
  // (AoA ~ 0-5 deg), the wing unloads, and the lift-only meter
  // collapses — while the thrust carries the jet at ~1 g. A full
  // pull reads HIGH on the lift meter (2.8-8.7 g at AoA 30 deg+), so
  // the probe pulls up to ~75 deg pitch and then HOLDS the climb.
  console.log('--- 2. Steep climb: F-18 from level 400 kt, pull up and hold ~75 deg ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(400 * KT2MS);
    ac.thrTarget = 1;
    const alt0 = ac.pos.y;
    let simT = 0;
    while (simT < 2) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, 'fa18', alt0);
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
    }
    // Phase A: full pull until pitch >= 70 deg. Record the peak
    // display-G (a pull must read > 1.5 g).
    let maxDispG = -99;
    let gdSeen = false;
    let tPull = 0;
    while (tPull < 3 && pitchUpDeg(ac) < 70) {
      const dt = nextDt();
      tPull += dt;
      ac.pitchIn = -1;
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      const gd = gDisplayOf(ac);
      if (!Number.isNaN(gd)) {
        gdSeen = true;
        maxDispG = Math.max(maxDispG, gd);
      }
    }
    // Phase B: hold ~75 deg pitch for 5 s and measure the meters.
    let minLiftG = 99;
    let minLiftCtx = '';
    let minDispG = 99;
    let gdSum = 0;
    let gdN = 0;
    const rows: string[] = [];
    let tHold = 0;
    while (tHold < 5) {
      const dt = nextDt();
      tHold += dt;
      const pitchErr = pitchUpDeg(ac) - 75;
      ac.pitchIn = Math.max(-1, Math.min(1, pitchErr * 0.15)); // pitchIn NEGATIVE pulls up
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      const gd = gDisplayOf(ac);
      if (ac.gload < minLiftG) {
        minLiftG = ac.gload;
        minLiftCtx = `t=${tHold.toFixed(2)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)}`;
      }
      if (!Number.isNaN(gd)) {
        gdSeen = true;
        minDispG = Math.min(minDispG, gd);
        // Settled-climb mean: only the last 2 s of the hold, after
        // the pull-in transient (dispG 6.14 -> 4.81 -> 2.17) has
        // decayed and the jet is flying up at ~constant speed.
        if (tHold > 3) {
          gdSum += gd;
          gdN++;
        }
      }
      if (Math.abs(tHold % 0.5) < dt) {
        rows.push(
          `t=${tHold.toFixed(1)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} liftG=${ac.gload.toFixed(2)} dispG=${Number.isNaN(gd) ? 'MISSING' : gd.toFixed(2)}`,
        );
      }
    }
    const gdMean = gdN ? gdSum / gdN : NaN;
    console.log(`  [hold] ${rows.join(' | ')}`);
    console.log(
      `  hold: min lift-G=${minLiftG.toFixed(2)} at ${minLiftCtx}; gDisplay ${gdSeen ? `min=${minDispG.toFixed(2)} mean=${gdMean.toFixed(2)}` : 'MISSING (pre-fix)'}; pull max gDisplay=${gdSeen ? maxDispG.toFixed(2) : 'MISSING'}`,
    );
    check(
      'steep climb (held): the lift-only meter dips to the live band (min < 0.5) — defect reproduced',
      minLiftG < 0.5,
      `min lift-G=${minLiftG.toFixed(2)} at ${minLiftCtx}`,
    );
    check(
      'steep climb (held): gDisplay stays a sane positive value (min > 0.5) through the climb',
      gdSeen && minDispG > 0.5,
      gdSeen ? `min gDisplay=${minDispG.toFixed(2)}` : 'gDisplay MISSING (pre-fix)',
    );
    check(
      'steep climb (held, settled): gDisplay means ~1.0 (0.8-1.2, the thrust carries the jet)',
      gdSeen && gdMean > 0.8 && gdMean < 1.2,
      gdSeen ? `settled mean gDisplay=${gdMean.toFixed(2)}` : 'gDisplay MISSING (pre-fix)',
    );
    check(
      'pull into the climb: gDisplay peaks above 1.5 g (it is a pull, not level flight)',
      gdSeen && maxDispG > 1.5,
      gdSeen ? `max gDisplay=${maxDispG.toFixed(2)}` : 'gDisplay MISSING (pre-fix)',
    );
    dispose(ac);
  }

  // ---- 3. Parked on Alpha (normal) ----
  console.log('--- 3. Parked on the Alpha runway (control) ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    let minGd = NaN;
    let maxGd = NaN;
    for (let i = 0; i < 600; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      const gd = gDisplayOf(ac);
      if (!Number.isNaN(gd)) {
        minGd = Number.isNaN(minGd) ? gd : Math.min(minGd, gd);
        maxGd = Number.isNaN(maxGd) ? gd : Math.max(maxGd, gd);
      }
    }
    console.log(
      `  runway park: surf=${ac.onSurface ? 1 : 0} gnd=${ac.grounded ? 1 : 0} lift-G=${ac.gload.toFixed(2)} gDisplay=${Number.isNaN(minGd) ? 'MISSING' : `[${minGd.toFixed(2)}, ${maxGd.toFixed(2)}]`}`,
    );
    check(
      'parked normal: gDisplay reads 1.0 (grounded override)',
      !Number.isNaN(minGd) && Math.abs(minGd - 1) < 0.05 && Math.abs(maxGd - 1) < 0.05,
      `gDisplay range=${Number.isNaN(minGd) ? 'MISSING' : `[${minGd.toFixed(2)}, ${maxGd.toFixed(2)}]`}`,
    );
    dispose(ac);
  }

  // ---- 4. Stationary in mid-air (the surf=0 parked signature) ----
  // An air-spawned jet at 0 kt, 100 m up: the wing flies nothing and
  // there is no ground, so BOTH meters read ~0 — that is correct for
  // a G meter in free fall (weightless), and it is exactly what the
  // live frozen jet showed. The parked-0.0 bug is therefore the
  // frozen state (surf=0 while parked), not the G formula: a parked
  // jet only reads 1.0 via the grounded override, which needs surf=1.
  console.log('--- 4. Stationary in mid-air (surf=0 signature, first step) ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, alpha.elev + 100, alpha.z, alpha.hdg);
    // One step only: read the meters before the fall develops.
    substep(ac, 1 / 60);
    const gd = gDisplayOf(ac);
    console.log(
      `  air-spawn 0 kt: surf=${ac.onSurface ? 1 : 0} gnd=${ac.grounded ? 1 : 0} lift-G=${ac.gload.toFixed(2)} gDisplay=${Number.isNaN(gd) ? 'MISSING' : gd.toFixed(2)} vsi=${ac.vsi.toFixed(2)}`,
    );
    check(
      'stationary mid-air: lift-G reads ~0 (the live parked-0.0 value) — the parked 0.0 is the surf=0 state, not a second G bug',
      Math.abs(ac.gload) < 0.3,
      `lift-G=${ac.gload.toFixed(2)}`,
    );
    dispose(ac);
  }

  // ---- 5. VTOL hover (F-35A, vector 90, airborne) ----
  console.log('--- 5. VTOL hover: F-35A at vector 90, airborne, holding altitude ---');
  {
    const ac = await makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 100, alpha.z, alpha.hdg);
    ac.setAirspeed(0);
    ac.vector = 90;
    ac.thrTarget = 1;
    let gdSum = 0;
    let gSum = 0;
    let n = 0;
    let simT = 0;
    while (simT < 8) {
      const dt = nextDt();
      simT += dt;
      substep(ac, dt);
      if (!ac.alive) break;
      if (simT > 3) {
        gdSum += gDisplayOf(ac);
        gSum += ac.gload;
        n++;
      }
    }
    const gdMean = n ? gdSum / n : NaN;
    const gMean = n ? gSum / n : NaN;
    console.log(
      `  hover: y=${ac.pos.y.toFixed(1)} vsi=${ac.vsi.toFixed(2)} gload mean=${gMean.toFixed(2)} (item 5: total vertical force at vector >= 70) gDisplay mean=${Number.isNaN(gdMean) ? 'MISSING' : gdMean.toFixed(2)}`,
    );
    check(
      'hover: gDisplay reads ~1.0 (0.7-1.3, the fan carries the jet)',
      !Number.isNaN(gdMean) && gdMean > 0.7 && gdMean < 1.3,
      `gDisplay mean=${Number.isNaN(gdMean) ? 'MISSING' : gdMean.toFixed(2)} gload mean=${gMean.toFixed(2)}`,
    );
    dispose(ac);
  }

  // ---- 6. HUD wiring: FlightHUD must display gDisplay, not gload ----
  console.log('--- 6. HUD wiring (source check, same pattern as probe-restart-camera) ---');
  {
    const { readFileSync } = await import('node:fs');
    const hudSrc = readFileSync('src/ui/FlightHUD.ts', 'utf8');
    check(
      'FlightHUD displays the display-G (gDisplay) for hudG, not the lift-only gload',
      hudSrc.includes('gDisplay'),
      '',
    );
    const ctlSrc = readFileSync('src/physics/FlightPhysicsController.ts', 'utf8');
    check(
      'FlightPhysicsController computes gDisplay from the force sum (lift+drag+thrust magnitude over weight)',
      ctlSrc.includes('gDisplay'),
      '',
    );
    check(
      'gload keeps its base assignment (wing lift) — the fix adds gDisplay instead of changing what gload means',
      ctlSrc.includes('this.gload = result.gLoad'),
      '',
    );
    const mainSrc = readFileSync('src/main.ts', 'utf8');
    check(
      'main.ts VTOL vector floor still reads p.gload (the wing load) — the fix must not feed the floor a display value',
      mainSrc.includes('p.gload'),
      '',
    );
  }

  console.log(failures === 0 ? '\nALL G-METER CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
