// probe-aoa-g.ts — pass 13, worker E: AoA / G sign audit.
//
// The v4.7 live check reported AoA −41° to −54° in steep climbs
// (with G −1.2 to −3.0) and AoA drifting −9.8° to −20° while the
// F-18 sat stationary on the deck. The v1 root cause was a body-frame
// AoA sign (atan2(−vBody.y, vBody.z)); this probe re-derives the live
// numbers through the LIVE path (real Aircraft / BET, real Rapier
// world + runway boxes, main.ts's exact substep order, live wind
// (−3, 0, 0) from windDir 270 / 3 m/s, VARIABLE dt, real GLB) and
// compares the solver's AoA against the independent geometric AoA
// (pitch − flight-path angle) at every sample, so a sign error shows
// up as a mismatch rather than as a plausible-looking number.
//
//   0. Solver-level sanity: the formula itself, on constructed
//      states (level / nose-up 30° with level velocity / velocity
//      tilted up 50° with a level nose).
//   1. Level flight F-18 at 250 kt: AoA small positive, G ≈ 1.
//   2. Steep climb: F-18 from level 400 kt, full pull held 6 s —
//      trace pitch / gamma / AoA / G every 0.5 s; AoA must track
//      pitch − gamma and stay positive while climbing steeply.
//   3. Steep climb with bank (60° banked pull, the live stick
//      usually has roll in it): same trace.
//   4. Parked on the deck while the carrier climbs over the north
//      range (range-crossing loop): the jet is CARRIED up at up to
//      15 m/s with a level nose, so its air velocity tilts up and
//      the formula reads negative AoA — the deck-drift candidate.
//   5. Parked on the Alpha runway (control): AoA ≈ 0, G = 1.
//
// Usage: node_modules/.bin/esbuild scripts/probe-aoa-g.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-aoa-g.mjs
//   && node /tmp/probe-aoa-g.mjs
//   (delete /tmp/probe-aoa-g.mjs after running)

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

const MS2KT = 1.94384;
const KT2MS = 1 / MS2KT;
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

/** Nose-up pitch in degrees (Euler YXZ x > 0 is nose DOWN, so the
 *  pitch-up angle is −x). */
function pitchUpDeg(ac: Aircraft): number {
  return -new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * R2D;
}
function bankDeg(ac: Aircraft): number {
  return new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * R2D;
}
/** Flight-path angle: asin(vy / |v|), degrees. */
function gammaDeg(ac: Aircraft): number {
  const v = ac.vel.length();
  if (v < 1) return 0;
  return (Math.asin(Math.max(-1, Math.min(1, ac.vel.y / v))) * R2D);
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
  // Live wind: menu default windDir 270, windSpeed 3 m/s —
  // windVec = (sin270·3, 0, cos270·3) = (−3, 0, 0).
  const wind = new THREE.Vector3(-3, 0, 0);
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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const alpha = RUNWAY_TABLE[0]!;

  const DT_PATTERN = [1 / 60, 1 / 50, 1 / 40, 1 / 30];
  let dtIdx = 0;
  const nextDt = (): number => {
    const dt = DT_PATTERN[dtIdx % DT_PATTERN.length]!;
    dtIdx++;
    return dt;
  };

  const makeAircraft = async (key: string, useBet = true): Promise<Aircraft> => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      useBet,
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
    ac.step(dt, wind, null);
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
  };

  // ---- 0. Solver-level sanity: the formula on constructed states ----
  console.log('--- 0. Solver-level AoA on constructed states (BET) ---');
  {
    const spec = SPECS['fa18']!;
    const bet = new BladeElementSolver(spec);
    const mkState = (vAir: THREE.Vector3, quat: THREE.Quaternion): AeroState => ({
      velocityAir: vAir,
      quat,
      mass: spec.mass,
      wingArea: spec.wingArea,
      wingAreaLift: spec.wingAreaEffective ?? spec.wingArea,
      span: spec.span,
      length: spec.length,
      throttle: 0.5,
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
    const v77 = 77; // ~150 kt
    // Level: velocity along body +Z, nose level.
    const rLevel = bet.solve(mkState(new THREE.Vector3(0, 0, v77), new THREE.Quaternion()));
    // Nose up 30°, velocity still horizontal (world): body quat is
    // Euler(−30° pitch-up, 0, 0) — x = −30° gives fwd tilted up 30°.
    const qUp30 = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(-30 * D2R, 0, 0, 'YXZ'),
    );
    const rUp30 = bet.solve(mkState(new THREE.Vector3(0, 0, v77), qUp30));
    // Nose level, velocity tilted up 50° (a zoom / crest: the air
    // arrives from above the nose).
    const vUp50 = new THREE.Vector3(0, v77 * Math.sin(50 * D2R), v77 * Math.cos(50 * D2R));
    const rVel50 = bet.solve(mkState(vUp50, new THREE.Quaternion()));
    console.log(
      `  level AoA=${rLevel.aoaDeg.toFixed(1)} (expect 0) | nose-up-30/level-vel AoA=${rUp30.aoaDeg.toFixed(1)} (expect +30) | level-nose/vel-up-50 AoA=${rVel50.aoaDeg.toFixed(1)} (expect −50)`,
    );
    check('solver AoA level = 0', Math.abs(rLevel.aoaDeg) < 0.5, `aoa=${rLevel.aoaDeg.toFixed(2)}`);
    check(
      'solver AoA nose-up 30 deg, level velocity = +30',
      Math.abs(rUp30.aoaDeg - 30) < 0.5,
      `aoa=${rUp30.aoaDeg.toFixed(2)}`,
    );
    check(
      'solver AoA level nose, velocity up 50 deg = −50 (air from above the nose)',
      Math.abs(rVel50.aoaDeg + 50) < 0.5,
      `aoa=${rVel50.aoaDeg.toFixed(2)}`,
    );
  }

  // AoA-target level-flight law (probe-stall-speeds): hold the AoA
  // level flight needs at the current speed.
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
  console.log('--- 1. Level flight F-18 at 250 kt (BET, live path) ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(250 * KT2MS);
    ac.thrTarget = 0.6;
    const alt0 = ac.pos.y;
    let simT = 0;
    let aoaSum = 0;
    let aoaN = 0;
    let gSum = 0;
    let maxAbsAoAErr = 0;
    while (simT < 15) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, 'fa18', alt0);
      ac.thrTarget = Math.max(0, Math.min(1, 0.6 + (250 - ac.ias * MS2KT) * 0.05));
      ac.rollIn = 0;
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) break;
      if (simT > 5) {
        aoaSum += ac.aoaDeg;
        gSum += ac.gload;
        aoaN++;
        const geo = pitchUpDeg(ac) - gammaDeg(ac);
        maxAbsAoAErr = Math.max(maxAbsAoAErr, Math.abs(ac.aoaDeg - geo));
      }
    }
    const aoaMean = aoaN ? aoaSum / aoaN : NaN;
    const gMean = aoaN ? gSum / aoaN : NaN;
    console.log(
      `  settled AoA mean=${aoaMean.toFixed(2)} deg, G mean=${gMean.toFixed(2)}, max |AoA − (pitch−gamma)|=${maxAbsAoAErr.toFixed(2)} deg`,
    );
    check(
      'level flight: AoA small positive (0 to 6 deg)',
      aoaMean > 0 && aoaMean < 6,
      `mean AoA=${aoaMean.toFixed(2)} deg`,
    );
    check('level flight: G ≈ 1 (0.8–1.2)', gMean > 0.8 && gMean < 1.2, `mean G=${gMean.toFixed(2)}`);
    check(
      'level flight: solver AoA tracks pitch−gamma within 2 deg',
      maxAbsAoAErr < 2,
      `max err=${maxAbsAoAErr.toFixed(2)} deg`,
    );
    dispose(ac);
  }

  // ---- 2 & 3. Steep climb traces ----
  const climbTrace = async (
    tag: string,
    banked: boolean,
  ): Promise<{ minAoA: number; minG: number; maxAoAErr: number; rows: string[] }> => {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(400 * KT2MS);
    ac.thrTarget = 1;
    const alt0 = ac.pos.y;
    // 2 s level settle.
    let simT = 0;
    while (simT < 2) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, 'fa18', alt0);
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
    }
    // Full pull 6 s (banked: hold 60° bank too).
    let minAoA = 90;
    let minG = 99;
    let maxAoAErr = 0;
    const rows: string[] = [];
    let tPull = 0;
    while (tPull < 6) {
      const dt = nextDt();
      tPull += dt;
      ac.pitchIn = -1;
      if (banked) {
        const bankNow = bankDeg(ac);
        ac.rollIn = Math.max(-1, Math.min(1, (60 - bankNow) * 0.05));
      } else {
        ac.rollIn = 0;
      }
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      minG = Math.min(minG, ac.gload);
      // pitch−gamma is only the geometric AoA while the jet is
      // upright and below the vertical: past the loop top the Euler
      // pitch wraps (bank flips to 180) and pitch−gamma is
      // meaningless, while the body-frame AoA stays valid.
      if (Math.abs(bankDeg(ac)) < 30 && pitchUpDeg(ac) < 75) {
        const geo = pitchUpDeg(ac) - gammaDeg(ac);
        maxAoAErr = Math.max(maxAoAErr, Math.abs(ac.aoaDeg - geo));
      }
      if (Math.abs(tPull % 0.5) < dt) {
        const geoRow = pitchUpDeg(ac) - gammaDeg(ac);
        // 3D angle between nose and velocity + sideslip: if the
        // velocity is mostly SIDEWAYS to the nose (large |beta|),
        // the pitch-plane AoA is a projection, not the whole story.
        const vHat = ac.vel.clone().normalize();
        const angle3d = Math.acos(
          Math.max(-1, Math.min(1, ac.fwd().dot(vHat))),
        ) * R2D;
        rows.push(
          `t=${tPull.toFixed(1)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} pitch-gamma=${geoRow.toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} beta=${ac.betaDeg.toFixed(1)} 3d=${angle3d.toFixed(1)} G=${ac.gload.toFixed(1)} bank=${bankDeg(ac).toFixed(0)} stalled=${ac.stalled} spin=${ac.spin.toFixed(2)}`,
        );
      }
    }
    console.log(`  [${tag}] ${rows.join(' | ')}`);
    dispose(ac);
    return { minAoA, minG, maxAoAErr, rows };
  };
  console.log('--- 2c. Steep climb at live low-FPS dt (fixed 0.05 s) ---');
  {
    // The v4.7 live check ran at 4–6 FPS near the deck: rawDt
    // 0.17–0.25 s, substeps dt = 0.05 s. Same pull as §2 but every
    // step at dt = 0.05, to check the sign does not flip at the live
    // step size.
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(400 * KT2MS);
    ac.thrTarget = 1;
    const alt0 = ac.pos.y;
    let simT = 0;
    while (simT < 2) {
      simT += 0.05;
      ac.pitchIn = levelPitchIn(ac, 'fa18', alt0);
      ac.thrTarget = 1;
      substep(ac, 0.05);
      if (!ac.alive) break;
    }
    let minAoA = 90;
    let minG = 99;
    let tPull = 0;
    const rows: string[] = [];
    while (tPull < 6) {
      tPull += 0.05;
      ac.pitchIn = -1;
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, 0.05);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      minG = Math.min(minG, ac.gload);
      if (Math.abs(tPull % 1) < 0.05) {
        rows.push(
          `t=${tPull.toFixed(0)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} G=${ac.gload.toFixed(1)} bank=${bankDeg(ac).toFixed(0)}`,
        );
      }
    }
    console.log(`  [dt=0.05 pull] ${rows.join(' | ')}`);
    check(
      'steep climb at dt=0.05: AoA stays positive (min > −5 deg), G positive',
      minAoA > -5 && minG > 0,
      `min AoA=${minAoA.toFixed(1)} deg min G=${minG.toFixed(2)}`,
    );
    dispose(ac);
  }
  console.log('--- 2d. Full pull from 150 kt (stall departure) ---');
  {
    // Low-speed pull: the jet stalls and departs. The spin model
    // is known to flip AoA to −40..−132 in a tumble (BET comments)
    // — record whether a plain stall departure reaches the live
    // −41..−54 band, and with what G.
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(150 * KT2MS);
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
    let minAoA = 90;
    let minG = 99;
    let tPull = 0;
    const rows: string[] = [];
    while (tPull < 12) {
      const dt = nextDt();
      tPull += dt;
      ac.pitchIn = -1;
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      minG = Math.min(minG, ac.gload);
      if (Math.abs(tPull % 1) < dt) {
        rows.push(
          `t=${tPull.toFixed(0)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} G=${ac.gload.toFixed(1)} bank=${bankDeg(ac).toFixed(0)} stalled=${ac.stalled} spin=${ac.spin.toFixed(2)}`,
        );
      }
    }
    console.log(`  [150kt pull] ${rows.join(' | ')}`);
    console.log(
      `  150kt pull: min AoA=${minAoA.toFixed(1)} deg, min G=${minG.toFixed(2)}, alive=${ac.alive}`,
    );
    dispose(ac);
  }
  console.log('--- 2e. Steep climb on the CURVE solver (same pull, for comparison) ---');
  {
    // If the curve solver holds a clean loop-top (small |beta|,
    // AoA/G positive), the §2 departure is BET-specific; if it
    // departs the same way, it is the manoeuvre, not the solver.
    const ac = await makeAircraft('fa18', false);
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
    let minAoA = 90;
    let minG = 99;
    let maxBeta = 0;
    let tPull = 0;
    const rows: string[] = [];
    while (tPull < 6) {
      const dt = nextDt();
      tPull += dt;
      ac.pitchIn = -1;
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      minG = Math.min(minG, ac.gload);
      maxBeta = Math.max(maxBeta, Math.abs(ac.betaDeg));
      if (Math.abs(tPull % 1) < dt) {
        rows.push(
          `t=${tPull.toFixed(0)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} beta=${ac.betaDeg.toFixed(1)} G=${ac.gload.toFixed(1)} bank=${bankDeg(ac).toFixed(0)}`,
        );
      }
    }
    console.log(`  [curve pull] ${rows.join(' | ')}`);
    console.log(
      `  curve pull: min AoA=${minAoA.toFixed(1)} deg, min G=${minG.toFixed(2)}, max |beta|=${maxBeta.toFixed(1)} deg, alive=${ac.alive}`,
    );
    dispose(ac);
  }
  console.log('--- 2f. Steep climb, auto-level spring disabled (rollIn = 0.001) ---');
  {
    // The airborne auto-level spring (controller) fires only when
    // rollIn === 0 exactly, reading the Euler YXZ bank. Near the
    // vertical the Euler bank is a gimbal artefact (172° at the loop
    // top) and the spring commands a max-rate roll about the nose
    // axis — which, nose-vertical, is a world yaw that slices the
    // nose sideways (beta). A 0.001 roll input disables the spring
    // while adding a negligible aileron deflection: if beta stays
    // ~0 and AoA/G stay positive, the spring is the culprit.
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
    let minAoA = 90;
    let minG = 99;
    let maxBeta = 0;
    let tPull = 0;
    const rows: string[] = [];
    while (tPull < 6) {
      const dt = nextDt();
      tPull += dt;
      ac.pitchIn = -1;
      ac.rollIn = 0.001;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      substep(ac, dt);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      minG = Math.min(minG, ac.gload);
      maxBeta = Math.max(maxBeta, Math.abs(ac.betaDeg));
      if (Math.abs(tPull % 1) < dt) {
        rows.push(
          `t=${tPull.toFixed(0)} kt=${(ac.ias * MS2KT).toFixed(0)} pitch=${pitchUpDeg(ac).toFixed(0)} gamma=${gammaDeg(ac).toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} beta=${ac.betaDeg.toFixed(1)} G=${ac.gload.toFixed(1)} bank=${bankDeg(ac).toFixed(0)}`,
        );
      }
    }
    console.log(`  [spring-off pull] ${rows.join(' | ')}`);
    console.log(
      `  spring-off pull: min AoA=${minAoA.toFixed(1)} deg, min G=${minG.toFixed(2)}, max |beta|=${maxBeta.toFixed(1)} deg, alive=${ac.alive}`,
    );
    check(
      'steep climb with the auto-level spring off: no yaw departure (max |beta| < 10 deg)',
      maxBeta < 10,
      `max |beta|=${maxBeta.toFixed(1)} deg`,
    );
    check(
      'steep climb with the auto-level spring off: AoA stays positive (min > −5 deg), G positive',
      minAoA > -5 && minG > 0,
      `min AoA=${minAoA.toFixed(1)} deg min G=${minG.toFixed(2)}`,
    );
    dispose(ac);
  }
  console.log('--- 2. Steep climb: F-18 from level 400 kt, full pull 6 s ---');
  {
    const r = await climbTrace('straight pull', false);
    check(
      'steep climb (straight pull): AoA stays positive while climbing (min > −5 deg)',
      r.minAoA > -5,
      `min AoA=${r.minAoA.toFixed(1)} deg`,
    );
    check(
      'steep climb (straight pull): G stays positive (min > 0)',
      r.minG > 0,
      `min G=${r.minG.toFixed(2)}`,
    );
    check(
      'steep climb (straight pull): solver AoA tracks pitch−gamma within 3 deg',
      r.maxAoAErr < 3,
      `max err=${r.maxAoAErr.toFixed(2)} deg`,
    );
  }
  console.log('--- 3. Steep climb with 60 deg bank: full pull 6 s ---');
  {
    const r = await climbTrace('banked pull', true);
    check(
      'steep climb (banked pull): AoA stays positive while climbing (min > −5 deg)',
      r.minAoA > -5,
      `min AoA=${r.minAoA.toFixed(1)} deg`,
    );
    check(
      'steep climb (banked pull): G stays positive (min > 0)',
      r.minG > 0,
      `min G=${r.minG.toFixed(2)}`,
    );
    check(
      'steep climb (banked pull): solver AoA tracks pitch−gamma within 5 deg',
      r.maxAoAErr < 5,
      `max err=${r.maxAoAErr.toFixed(2)} deg`,
    );
  }

  // ---- 4. Parked on the deck while the carrier climbs the range ----
  console.log('--- 4. Parked on the deck, carrier climbing over the north range ---');
  {
    const loop = carrierLoop(0);
    const carrier = new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt,
      waypoints: [
        new THREE.Vector3(-2000, 350, -5200),
        new THREE.Vector3(2000, 350, -5200),
        new THREE.Vector3(2000, 350, -2500),
        new THREE.Vector3(-2000, 350, -2500),
      ],
      startIndex: 0,
    });
    await carrier.build();
    const startPos = new THREE.Vector3(-2000, 350, -2500);
    carrier.reset(
      startPos.clone(),
      Math.atan2(-2000 - startPos.x, -5200 - startPos.z),
    );
    const deckProvider: DeckProvider = {
      get alive() {
        return carrier.active && carrier.alive;
      },
      deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
      velocity: () => carrier.vel.clone(),
      velocityAt: (x, z) => {
        // HelicarrierController has no velocityAt (main.ts computes
        // it from vel + yawRate x r) — same formula here.
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
    const spec = SPECS['fa18']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      useBet: true,
      deck: deckProvider,
    });
    ac.terrainHeightAt = heightAt;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * 85;
    const sz = carrier.pos.z - fwd.z * 85;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    ac.thrTarget = 0;
    ac.brake = true;
    let minAoA = 90;
    let maxAoA = -90;
    let maxDeckVy = -99;
    let minDeckVy = 99;
    let simT = 0;
    const rows: string[] = [];
    while (simT < 120) {
      const dt = nextDt();
      simT += dt;
      carrier.update(dt);
      world.timestep = dt;
      world.step(eventQueue);
      drain(ac);
      ac.step(dt, wind, null);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      maxDeckVy = Math.max(maxDeckVy, carrier.vel.y);
      minDeckVy = Math.min(minDeckVy, carrier.vel.y);
      if (Math.abs(simT % 10) < dt) {
        rows.push(
          `t=${simT.toFixed(0)}s deckVy=${carrier.vel.y.toFixed(1)} carrierY=${carrier.pos.y.toFixed(0)} aoa=${ac.aoaDeg.toFixed(1)} onDeck=${ac.onDeck} grounded=${ac.grounded} ias=${(ac.ias * MS2KT).toFixed(0)}kt`,
        );
      }
    }
    console.log(`  [deck park] ${rows.join(' | ')}`);
    console.log(
      `  deck park: AoA range [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, deck Vy range [${minDeckVy.toFixed(1)}, ${maxDeckVy.toFixed(1)}] m/s, alive=${ac.alive} onDeck=${ac.onDeck}`,
    );
    check(
      'deck park: AoA ≈ 0 while parked on the deck (within ±3 deg) even as the carrier climbs',
      minAoA > -3 && maxAoA < 3,
      `AoA range [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, deck Vy [${minDeckVy.toFixed(1)}, ${maxDeckVy.toFixed(1)}] m/s`,
    );
    dispose(ac);
  }

  // ---- 5. Parked on the Alpha runway (control) ----
  console.log('--- 5. Parked on the Alpha runway (control) ---');
  {
    const ac = await makeAircraft('fa18');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    ac.brake = true;
    let minAoA = 90;
    let maxAoA = -90;
    for (let i = 0; i < 600; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      minAoA = Math.min(minAoA, ac.aoaDeg);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
    }
    console.log(`  runway park: AoA range [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, G=${ac.gload.toFixed(1)}`);
    check(
      'runway park: AoA ≈ 0 (within ±3 deg), G = 1',
      minAoA > -3 && maxAoA < 3 && Math.abs(ac.gload - 1) < 0.01,
      `AoA range [${minAoA.toFixed(1)}, ${maxAoA.toFixed(1)}] deg, G=${ac.gload.toFixed(2)}`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL AoA/G CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
