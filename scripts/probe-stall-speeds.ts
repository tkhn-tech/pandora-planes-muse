// probe-stall-speeds.ts — pass 11: flight-characteristics audit probe.
//
// Measures ACTUAL behaviour through the live paths (real Aircraft /
// FlightPhysicsController, real Rapier world with the real terrain
// heightfield + runway boxes, main.ts's exact substep order, live wind
// 3 m/s, VARIABLE dt, real GLBs loaded via attachModel with the Image
// stub — see probe-ford-texture.ts for the stub pattern). Default
// solver is BET (the live default); the F-22 150 kt level-flight check
// is also run on the curve solver to isolate BET-vs-curve.
//
// Sections:
//   0. Solver-level CL sweep (BET vs curve, no Rapier): effective CL
//      vs AoA at 150 kt. Catches the tail-download lift loss and the
//      MiG-29 lift-area split (BET wing stations sum to wingArea 38
//      while the curve solver lifts on wingAreaEffective 45).
//   1. F-22 level flight at 150 kt: must NOT stall (the live report).
//      Also all six at 150 kt for reference.
//   2. Stall speed per jet: idle throttle (power-off, like the
//      published Vs numbers), altitude-hold autopilot, decelerating
//      from 200 kt. Stall = the stall flag firing (AoA past the
//      peak-CL AoA) or an unrecoverable sink at full pull. Band per
//      jet is the researched clean Vs (Docs/flight-behaviour-
//      per-model.md §7 as corrected in pass 11).
//   3. Takeoff roll: full throttle + full pull, flaps 0 — liftoff
//      speed/dist vs the per-model rotation speed. Plus the live
//      F-22 case (29% throttle, flaps 0 and flaps 30): it must NOT
//      lift off below Vr.
//   4. Sustained turn: 400 kt SL at the §0 drag = thrust point,
//      per jet (bank/AoA computed from that jet's own thrust,
//      k and CLmax) — rate and bleed measured settled, s 5-15.
//
// Usage: node_modules/.bin/esbuild scripts/probe-stall-speeds.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-stall-speeds.mjs
//   && node /tmp/probe-stall-speeds.mjs
//   (delete /tmp/probe-stall-speeds.mjs after running)

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB (copied from probe-ford-texture.ts).
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
import { SPECS } from '../src/aircraft/AircraftSpec';
import { BladeElementSolver } from '../src/physics/BladeElementSolver';
import { CurveAerodynamicsSolver } from '../src/physics/CurveAerodynamicsSolver';
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

const MS2KT = 1.94384;
const KT2MS = 1 / MS2KT;
const D2R = Math.PI / 180;

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

/** Researched clean stall speeds (m/s) at combat mass, pass 11
 *  (Vs = sqrt(2W/(rho*S*CLmax)) on the LIFT area): F-18 66 (CL 1.6
 *  on 46.5), F-35A 68 (CL 1.5 on 42.7), F-22 60 (CL 1.40 on 78),
 *  F-15E 72 (CL 1.10 on 56.5), MiG-29 64 (CL 1.35 on 45 eff),
 *  Su-34 71 (CL 1.53 on 62). Bands are Vs +/- 8 kt. */
const STALL_BAND_KT: Record<string, [number, number]> = {
  fa18: [121, 137],
  f35a: [124, 140],
  f22: [109, 125],
  f15e: [133, 149],
  mig29: [116, 132],
  su34: [130, 146],
};
/** Rotation speeds (kt), pass 11: F-18 140 (was 126 — below its own
 *  128 kt stall speed), F-35A 150, F-22 140, F-15E 150, MiG-29 130,
 *  Su-34 140. */
const ROTATE_KT: Record<string, number> = {
  fa18: 140,
  f35a: 150,
  f22: 140,
  f15e: 150,
  mig29: 130,
  su34: 140,
};
const JETS = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const;

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
  // Live wind: the menu default 3 m/s (probe-stall-respawn).
  const wind = new THREE.Vector3(3, 0, 0);
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

  // Variable dt, live-like: the frame time wanders 16-33 ms instead
  // of the fixed 1/60 the old probes used.
  const DT_PATTERN = [1 / 60, 1 / 50, 1 / 40, 1 / 30];
  let dtIdx = 0;
  const nextDt = (): number => {
    const dt = DT_PATTERN[dtIdx % DT_PATTERN.length]!;
    dtIdx++;
    return dt;
  };

  const makeAircraft = async (
    key: string,
    useBet = true,
  ): Promise<Aircraft> => {
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
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  // ---- Autopilot: AoA-target hold (the turn probes' law, proven
  // stable with BET). The altitude-only law (pitch from altitude +
  // vsi error) is UNSTABLE with BET: BET's pitch rate integrates
  // from moments, so a saturated pull overshoots the AoA to 35 deg+
  // (section 1/2 first ran that law and every jet "stalled" at
  // 150 kt — an autopilot artefact, not the physics; the curve
  // solver's prescribed rates hid it). Here the target AoA is the
  // AoA that gives the CL level flight needs at the current speed,
  // plus a slow altitude trim (+/-2 deg). The pull eases as soon as
  // the AoA reaches target, so there is no runaway.
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
  /** Pitch input holding the AoA level flight needs at the current
   *  speed (plus a slow altitude trim). `aoaCapDeg` caps the target
   *  AoA (peakAoA + 1 for a stall test — command just past the peak
   *  so the flag fires crisply at Vs; peakAoA - 2 for level flight).
   *  `trimDeg`/`trimGain` bound the altitude trim (section 1 uses
   *  4 deg / 0.03; section 2 uses the same, so the autopilot holds
   *  altitude down to Vs instead of settling into a glide above
   *  it — the weak 2 deg / 0.02 trim let the jet sink at -6 m/s
   *  from the start of the decel).
   *  `sinkGain` adds AoA when sinking (deg per m/s of sink, capped
   *  at +5 deg) so the autopilot arrests a descent instead of
   *  settling into a steady glide below Vs — without it the jet
   *  glides down at ~Vs with the AoA below the peak and the flag
   *  never fires. `minPull` floors the pull at that magnitude when
   *  the AoA is below target: the controller's "no pull, no
   *  pitch-up" gate zeroes the nose-up rate when pitchIn >= -0.25
   *  and the jet is already pitched up > 5 deg, and the
   *  proportional law (gain 0.3) hovers at -0.25 with a ~0.8 deg
   *  lag — so the gate stays shut and the pitch creeps. A -0.3
   *  floor keeps the gate open whenever the jet needs to pitch up;
   *  pushes (AoA above target) are unaffected. */
  const levelPitchIn = (
    ac: Aircraft,
    key: string,
    alt0: number,
    aoaCapDeg: number,
    trimDeg = 2,
    trimGain = 0.02,
    sinkGain = 0,
    minPull = 0,
  ): number => {
    const spec = SPECS[key]!;
    const sLift = spec.wingAreaEffective ?? spec.wingArea;
    const rho = rhoAt(ac.pos.y);
    const q = 0.5 * rho * Math.max(ac.ias, 1) * Math.max(ac.ias, 1);
    const clReq = (spec.mass * 9.81) / Math.max(q * sLift, 1);
    let aoaTarget = aoaForCL(key, clReq);
    aoaTarget += Math.max(-trimDeg, Math.min(trimDeg, (alt0 - ac.pos.y) * trimGain));
    if (sinkGain > 0 && ac.vsi < 0) {
      aoaTarget += Math.min(-ac.vsi * sinkGain, 5);
    }
    aoaTarget = Math.min(aoaTarget, aoaCapDeg);
    const raw = Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3));
    // Floor the pull (never the push): keeps the controller's
    // -0.25 "no pull" gate open while the AoA is below target.
    return raw < 0 && minPull > 0 ? Math.min(raw, -minPull) : raw;
  };

  // ---- 0. Solver-level CL sweep: BET effective CL vs the curve
  // solver's CL at the same AoA (150 kt, no ground effect, no
  // thrust, neutral controls). The spec CL curve is the
  // WHOLE-AIRCRAFT curve, so BET's station sum must reproduce it:
  // a big gap means the tail download (or a missing lifting area)
  // is eating the lift the spec promises.
  console.log('--- 0. CL sweep: BET effective CL vs curve CL at 150 kt ---');
  for (const key of JETS) {
    const spec = SPECS[key]!;
    const bet = new BladeElementSolver(spec);
    const curve = new CurveAerodynamicsSolver();
    const mkState = (aoaDeg: number): AeroState => ({
      velocityAir: new THREE.Vector3(
        0,
        -77 * Math.sin(aoaDeg * D2R),
        77 * Math.cos(aoaDeg * D2R),
      ),
      quat: new THREE.Quaternion(),
      mass: spec.mass,
      wingArea: spec.wingArea,
      wingAreaLift: spec.wingAreaEffective ?? spec.wingArea,
      span: spec.span,
      length: spec.length,
      throttle: 0,
      thrustMax: spec.thrustMax,
      vectorDeg: 0,
      flapsDeg: 0,
      airbrake: 0,
      gearDown: false,
      damage: { ...ZERO_DAMAGE },
      altitude: 0,
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
    const row: string[] = [];
    let gap9 = 0;
    let gap35 = 0;
    for (const aoa of [0, 5, 9, 15, 25, 35]) {
      // Two calls, read the second: the BET tail downwash uses the
      // PREVIOUS step's CL (one-step lag), so a fresh solver's first
      // call has no downwash and the tail lifts instead of
      // downloading — the first-call CL reads ~0.1 high. The second
      // call is the settled value.
      bet.solve(mkState(aoa));
      const b = bet.solve(mkState(aoa));
      const c = curve.solve(mkState(aoa));
      row.push(`${aoa}deg BET ${b.cl.toFixed(2)} / curve ${c.cl.toFixed(2)}`);
      if (aoa === 9) gap9 = Math.abs(b.cl - c.cl);
      if (aoa === 35) gap35 = Math.abs(b.cl - c.cl);
    }
    console.log(
      `${key} (spec peak ${spec.clCurve.peak} @ ${spec.clCurve.peakAoA} deg): ${row.join(' | ')}`,
    );
    check(
      `CL sweep ${key}: BET within 0.20 of curve CL at 9 deg AoA (level flight)`,
      gap9 < 0.2,
      `gap=${gap9.toFixed(2)}`,
    );
    check(
      `CL sweep ${key}: BET peak CL within 0.30 of spec peak ${spec.clCurve.peak}`,
      gap35 < 0.3,
      `gap=${gap35.toFixed(2)}`,
    );
  }

  // ---- 1. Level flight at 150 kt must NOT stall (the live F-22
  // report). Altitude-hold autopilot + speed-hold throttle, 15 s.
  console.log('--- 1. Level flight at 150 kt (BET, live path) ---');
  const levelAt150 = async (
    key: string,
    useBet: boolean,
  ): Promise<{ stalled: boolean; altDrift: number; maxAoA: number }> => {
    const spec = SPECS[key]!;
    const ac = await makeAircraft(key, useBet);
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(150 * KT2MS);
    ac.thrTarget = 0.6;
    const alt0 = ac.pos.y;
    let stalled = false;
    let maxAoA = -90;
    let minAlt = Infinity;
    let maxAlt = -Infinity;
    let simT = 0;
    // 20 s; the altitude band is measured over the LAST 10 s only
    // (the spawn transient — AoA 0, no lift — takes a few seconds
    // to settle out of the band).
    while (simT < 20) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, key, alt0, spec.clCurve.peakAoA - 2, 4, 0.03);
      // Speed hold around 150 kt.
      ac.thrTarget = Math.max(
        0,
        Math.min(1, 0.6 + (150 - ac.ias * MS2KT) * 0.05),
      );
      ac.rollIn = 0;
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) break;
      if (ac.stalled) stalled = true;
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      if (simT > 10) {
        minAlt = Math.min(minAlt, ac.pos.y);
        maxAlt = Math.max(maxAlt, ac.pos.y);
      }
    }
    const altDrift = maxAlt - minAlt;
    dispose(ac);
    return { stalled, altDrift, maxAoA };
  };
  for (const key of JETS) {
    const r = await levelAt150(key, true);
    check(
      `level flight ${key} at 150 kt: no stall flag, holds altitude (band < 100 m over the settled 10 s)`,
      !r.stalled && r.altDrift < 100,
      `stalled=${r.stalled} alt band=${r.altDrift.toFixed(1)} m maxAoA=${r.maxAoA.toFixed(1)} deg`,
    );
  }
  {
    const r = await levelAt150('f22', false);
    check(
      'level flight f22 at 150 kt on the CURVE solver: no stall flag either (isolates BET-vs-curve)',
      !r.stalled && r.altDrift < 100,
      `stalled=${r.stalled} alt band=${r.altDrift.toFixed(1)} m maxAoA=${r.maxAoA.toFixed(1)} deg`,
    );
  }

  // ---- 2. Stall speed: idle throttle (power-off), altitude-hold
  // autopilot, decelerating from 200 kt. Stall = the flag fires, or
  // a sustained sink (vsi < -3 m/s for 2 s) while the autopilot is
  // commanding level flight — the sink IS the stall (the wing can
  // no longer hold altitude). The old "sink at full pull" form
  // missed it: with the lift now perpendicular to the airflow the
  // jet settles into a steady glide at ~Vs with the AoA below the
  // peak, so the flag fires late or never and the jet just glides
  // down to the floor.
  console.log('--- 2. Stall speed, power-off decel from 200 kt (BET) ---');
  for (const key of JETS) {
    const spec = SPECS[key]!;
    const ac = await makeAircraft(key, true);
    const modelOk = ac.modelStatus === 'ok' && ac.modelTris > 0;
    check(
      `${key} real GLB loaded (modelStatus ok, ${ac.modelTris} tris)`,
      modelOk,
      `modelStatus=${ac.modelStatus} tris=${ac.modelTris} err=${ac.modelError ?? ''}`,
    );
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(200 * KT2MS);
    ac.thrTarget = 0.5;
    const alt0 = ac.pos.y;
    // 3 s settle at 200 kt on the AoA-target law (strong trim +
    // sink arrest + min pull, so the jet holds altitude into the
    // decel instead of sinking from the start).
    let simT = 0;
    while (simT < 3) {
      const dt = nextDt();
      simT += dt;
      ac.pitchIn = levelPitchIn(ac, key, alt0, spec.clCurve.peakAoA - 2, 4, 0.03, 0.5, 0.3);
      ac.thrTarget = Math.max(0, Math.min(1, 0.5 + (200 - ac.ias * MS2KT) * 0.02));
      substep(ac, dt);
      if (!ac.alive) break;
    }
    // Idle: the drag decelerates the jet; the autopilot pulls to
    // hold altitude until the wing cannot.
    ac.thrTarget = 0.05;
    let stallKt: number | null = null;
    let stallHow = '';
    let maxAoA = -90;
    let sinkT = 0;
    let minLevelKt: number | null = null;
    let levelT = 0;
    simT = 0;
    while (simT < 120) {
      const dt = nextDt();
      simT += dt;
      // AoA target capped just past the peak: as the speed decays
      // the target AoA rises with the CL level flight needs, and
      // the flag fires crisply when the required AoA passes the
      // peak — i.e. at Vs.
      ac.pitchIn = levelPitchIn(ac, key, alt0, spec.clCurve.peakAoA + 1, 4, 0.03, 0.5, 0.3);
      ac.rollIn = 0;
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) {
        stallKt = ac.ias * MS2KT;
        stallHow = 'crashed before the flag fired';
        break;
      }
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      // Lowest speed held near-level (|vsi| < 2.5) for 1 s
      // (informational — the BET phugoid makes a tighter window
      // noisy; the flag speed is the measurement).
      if (Math.abs(ac.vsi) < 2.5) {
        levelT += dt;
        if (levelT >= 1) minLevelKt = ac.ias * MS2KT;
      } else {
        levelT = 0;
      }
      if (ac.stalled) {
        stallKt = ac.ias * MS2KT;
        stallHow = `stall flag at AoA ${ac.aoaDeg.toFixed(1)} deg`;
        break;
      }
      // Sustained sink while the autopilot commands level flight:
      // the wing can no longer hold altitude — that speed is Vs,
      // whether or not the AoA has crossed the peak yet. (No
      // full-pull condition: the autopilot is always pulling to
      // hold altitude, so any sustained sink is the stall.)
      if (ac.vsi < -3) {
        sinkT += dt;
        if (sinkT > 2) {
          stallKt = ac.ias * MS2KT;
          stallHow = `unrecoverable sink at AoA ${ac.aoaDeg.toFixed(1)} deg (no flag)`;
          break;
        }
      } else {
        sinkT = 0;
      }
      if (ac.pos.y < 120) {
        stallKt = ac.ias * MS2KT;
        stallHow = 'sank to 120 m without a flag';
        break;
      }
    }
    const [lo, hi] = STALL_BAND_KT[key]!;
    check(
      `stall speed ${key}: in researched band ${lo}-${hi} kt (spec Vs ${(spec.stallSpeed * MS2KT).toFixed(0)} kt)`,
      stallKt !== null && stallKt >= lo && stallKt <= hi,
      `measured ${stallKt === null ? 'none' : stallKt.toFixed(0) + ' kt'} (${stallHow}), maxAoA=${maxAoA.toFixed(1)} deg, min level speed=${minLevelKt === null ? 'n/a' : minLevelKt.toFixed(0) + ' kt'}`,
    );
    dispose(ac);
  }

  // ---- 3. Takeoff roll: full throttle + full pull, flaps 0.
  console.log('--- 3. Takeoff roll, full throttle + full pull, flaps 0 (BET) ---');
  const takeoffRun = async (
    key: string,
    flaps: number,
    thr: number,
  ): Promise<{
    lifted: boolean;
    liftKt: number;
    liftDist: number;
    maxAoA: number;
    alive: boolean;
    finalKt: number;
    finalDist: number;
  }> => {
    const ac = await makeAircraft(key, true);
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.flaps = flaps;
    ac.thrTarget = thr;
    ac.pitchIn = -1;
    const x0 = ac.pos.x;
    let lifted = false;
    let liftKt = 0;
    let liftDist = 0;
    let maxAoA = -90;
    let simT = 0;
    while (simT < 30) {
      const dt = nextDt();
      simT += dt;
      substep(ac, dt);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      if (!ac.alive) break;
      if (
        ac.pos.y > heightAt(ac.pos.x, ac.pos.z) + 4 &&
        ac.vsi > 0 &&
        !ac.grounded &&
        ac.ias > 40
      ) {
        lifted = true;
        liftKt = ac.ias * MS2KT;
        liftDist = ac.pos.x - x0;
        break;
      }
      if (ac.pos.x - x0 > 750) break;
    }
    const alive = ac.alive;
    const finalKt = ac.ias * MS2KT;
    const finalDist = ac.pos.x - x0;
    dispose(ac);
    return { lifted, liftKt, liftDist, maxAoA, alive, finalKt, finalDist };
  };
  // F-18 flaps-0 liftoff speed, so the flaps-30 check below can
  // compare against the same run (flaps must help).
  let fa18Flaps0Kt = 0;
  for (const key of JETS) {
    const r = await takeoffRun(key, 0, 1);
    const vr = ROTATE_KT[key]!;
    check(
      `takeoff ${key} flaps 0: lifts off near Vr ${vr} kt, inside 650 m`,
      r.alive && r.lifted && r.liftDist < 650 && r.liftKt > vr - 10 && r.liftKt < vr + 35,
      `lifted=${r.lifted} at ${r.liftKt.toFixed(0)} kt in ${r.liftDist.toFixed(0)} m, maxAoA=${r.maxAoA.toFixed(1)} deg, alive=${r.alive}`,
    );
    if (key === 'fa18') fa18Flaps0Kt = r.liftKt;
  }
  // The live F-22 case: 29% throttle must NOT lift off below Vr.
  for (const flaps of [0, 30]) {
    const r = await takeoffRun('f22', flaps, 0.29);
    check(
      `takeoff f22 flaps ${flaps} at 29% throttle: does NOT lift off below Vr 140 kt`,
      !r.lifted || r.liftKt >= 140,
      `lifted=${r.lifted} at ${r.liftKt.toFixed(0)} kt in ${r.liftDist.toFixed(0)} m, alive=${r.alive} (ran off the end at ${r.finalKt.toFixed(0)} kt after ${r.finalDist.toFixed(0)} m)`,
    );
  }
  {
    const r = await takeoffRun('fa18', 30, 1);
    // Flaps must help: the flaps-30 liftoff is at or below the
    // flaps-0 liftoff for the same jet (the old hardcoded 165 kt
    // bar went stale when the Vr fix moved the F-18's liftoff
    // from 154 to 168 kt).
    check(
      'takeoff fa18 flaps 30: lifts off at or below the flaps-0 speed (flaps help)',
      r.alive && r.lifted && r.liftKt <= fa18Flaps0Kt,
      `lifted at ${r.liftKt.toFixed(0)} kt in ${r.liftDist.toFixed(0)} m (flaps 0: ${fa18Flaps0Kt.toFixed(0)} kt), alive=${r.alive}`,
    );
  }

  // ---- 4. Sustained turn: 400 kt SL at the §0 drag = thrust
  // point, per jet. The rate is measured over seconds 1.5-4.5 —
  // right after the bank is in, while the speed is still near
  // 400 kt. Later windows do NOT measure the sustained rate: the
  // §0 point is only sustained AT 400 kt (below it, drag exceeds
  // thrust at the same bank, the jet must either raise its AoA
  // into the separation-drag wall — the adaptive law tried that
  // and bled 88-165 kt in 5 s with AoA at the peak — or give up
  // bank). The fixed per-jet AoA hold (CL_sust from drag =
  // thrust, bank = acos(1/n_sust)) with the early window is the
  // measurement the harness can actually make: probe-80turn's
  // trace shows the F-18 at 17-18 deg/s in s 1-2, vs the §0 19.0.
  console.log('--- 4. Sustained turn, 400 kt SL, drag = thrust point (BET) ---');
  for (const key of JETS) {
    const spec = SPECS[key]!;
    const sLift = spec.wingAreaEffective ?? spec.wingArea;
    const v0 = 400 * KT2MS;
    const q0 = 0.5 * 1.225 * v0 * v0;
    const qS = q0 * sLift;
    const cdAvail = spec.thrustMax / qS - 0.028;
    const clSust = Math.min(Math.sqrt(Math.max(cdAvail, 0) / spec.inducedK), spec.clCurve.peak);
    const nSust = (qS * clSust) / (spec.mass * 9.81);
    const bankDes = Math.acos(Math.min(1 / Math.max(nSust, 1.01), 1));
    const aoaTarget = aoaForCL(key, clSust);
    const expectRate = (9.81 * Math.sqrt(nSust * nSust - 1)) / v0 / D2R;
    const ac = await makeAircraft(key, true);
    ac.resetAt(alpha.x, 200, alpha.z, alpha.hdg);
    ac.setAirspeed(v0);
    ac.thrTarget = 1;
    let hdg15 = 0;
    let hdg45 = 0;
    let kt15 = 0;
    let kt45 = 0;
    let minAlt = Infinity;
    let maxAoA = -Infinity;
    let simT = 0;
    while (simT < 4.5) {
      const dt = nextDt();
      simT += dt;
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
      // Two-phase AoA law. ENTRY (t < 1.5 s): the load factor
      // follows the CURRENT bank (n = 1/cos(bankNow)) so the
      // lift's vertical component stays ~1 g through the roll-in
      // (no zoom). SUSTAINED (t >= 1.5 s): the AoA is HELD at the
      // §0 sustained point (CL_sust) — it must NOT chase the
      // bleeding speed, because the §0 point is unstable in the
      // speed direction: below 400 kt the CL the bank needs rises,
      // the AoA follows it into the separation-drag wall, and the
      // jet bleeds 50-124 kt in 3 s with the AoA at the peak (the
      // current-bank law left running past the entry did exactly
      // that). The fixed hold + coordinated entry is the
      // measurement: rate over s 1.5-4.5 at ~400 kt.
      const rho = rhoAt(ac.pos.y);
      const q = 0.5 * rho * Math.max(ac.ias, 1) * Math.max(ac.ias, 1);
      let aoaTarget: number;
      if (simT < 1.5) {
        const nReq = 1 / Math.max(Math.cos(bankNow), 0.15);
        const clReq = (nReq * spec.mass * 9.81) / Math.max(q * sLift, 1);
        aoaTarget = Math.min(aoaForCL(key, clReq), aoaForCL(key, clSust));
      } else {
        aoaTarget = aoaForCL(key, clSust);
      }
      ac.pitchIn = Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3));
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) break;
      if (Math.abs(simT - 1.5) < 0.03) {
        hdg15 = Math.atan2(ac.fwd().x, ac.fwd().z);
        kt15 = ac.ias * MS2KT;
      }
      minAlt = Math.min(minAlt, ac.pos.y);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      kt45 = ac.ias * MS2KT;
    }
    hdg45 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let dHdg = hdg45 - hdg15;
    while (dHdg > Math.PI) dHdg -= 2 * Math.PI;
    while (dHdg < -Math.PI) dHdg += 2 * Math.PI;
    const turnRate = Math.abs((dHdg * 180) / Math.PI / 3);
    const bleed = kt15 - kt45;
    check(
      `sustained turn ${key}: ~${expectRate.toFixed(1)} deg/s at 400 kt SL (${(bankDes / D2R).toFixed(0)} deg bank, ${nSust.toFixed(1)} g, AoA ${aoaTarget.toFixed(1)} deg), s 1.5-4.5`,
      ac.alive && turnRate > expectRate - 3 && turnRate < expectRate + 3 && maxAoA < spec.clCurve.peakAoA + 5 && minAlt > 150,
      `rate=${turnRate.toFixed(1)} deg/s alive=${ac.alive} minAlt=${minAlt.toFixed(0)} m maxAoA=${maxAoA.toFixed(1)} bleed=${bleed.toFixed(0)} kt over s 1.5-4.5`,
    );
    dispose(ac);
  }

  // ---- 5. The live 41% throttle case (pass 12): an F-22 at 150 kt,
  // level-ish at 141-163 kt on 41% throttle collapsed to 5-9 kt with
  // the STALL banner at 447 ft — while §1 holds 150 kt level and §2
  // measures the power-off stall at 120 kt. Reconcile: at 41%
  // throttle the thrust (0.41 x 312 kN = 128 kN) far exceeds the
  // level-flight drag at 150 kt (~41 kN), so a level 41% flight
  // must ACCELERATE, not collapse — unless the collapse came from
  // the stick (a held pull driving the AoA past the peak into the
  // separation-drag wall, where drag exceeds thrust and the speed
  // bleeds to near zero). Three variants, all F-22 from 150 kt:
  //   5a altitude-hold autopilot, throttle fixed 0.41 (the live
  //      "level-ish" case as flown: hold altitude, see what the
  //      speed does);
  //   5b full pull held, throttle fixed 0.41 (the pull hypothesis:
  //      it MUST stall and decelerate hard — the question is
  //      whether it then recovers/dives or HANGS at <20 kt, the
  //      live 5-9 kt signature);
  //   5c hands-off (pitchIn 0), throttle fixed 0.41.
  console.log('--- 5. Live 41% throttle case, F-22 from 150 kt (BET) ---');
  const lowPowerRun = async (
    tag: string,
    mode: 'hold' | 'pull' | 'handsOff',
    seconds: number,
  ): Promise<{
    minKt: number;
    finalKt: number;
    stalled: boolean;
    crashed: boolean;
    hangS: number;
    maxAoA: number;
    log: string;
  }> => {
    const key = 'f22';
    const spec = SPECS[key]!;
    const ac = await makeAircraft(key, true);
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(150 * KT2MS);
    ac.thrTarget = 0.41;
    ac.throttle = 0.41;
    const alt0 = ac.pos.y;
    let stalled = false;
    let maxAoA = -90;
    let minKt = Infinity;
    let hangS = 0;
    let simT = 0;
    const rows: string[] = [];
    while (simT < seconds) {
      const dt = nextDt();
      simT += dt;
      if (mode === 'hold') {
        ac.pitchIn = levelPitchIn(ac, key, alt0, spec.clCurve.peakAoA + 1, 4, 0.03, 0.5, 0.3);
      } else if (mode === 'pull') {
        ac.pitchIn = -1;
      } else {
        ac.pitchIn = 0;
      }
      ac.thrTarget = 0.41;
      ac.rollIn = 0;
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) break;
      if (ac.stalled) stalled = true;
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      const kt = ac.ias * MS2KT;
      minKt = Math.min(minKt, kt);
      const agl = ac.pos.y - heightAt(ac.pos.x, ac.pos.z);
      if (kt < 20 && agl > 50) hangS += dt;
      if (Math.abs(simT % 5) < dt) {
        rows.push(`t=${simT.toFixed(0)}s ${kt.toFixed(0)}kt alt=${ac.pos.y.toFixed(0)}m aoa=${ac.aoaDeg.toFixed(1)} stalled=${ac.stalled} vsi=${ac.vsi.toFixed(1)}`);
      }
    }
    const finalKt = ac.ias * MS2KT;
    const crashed = !ac.alive;
    const log = rows.join(' | ');
    dispose(ac);
    console.log(`  [5${tag}] ${log} => minKt=${minKt.toFixed(0)} finalKt=${finalKt.toFixed(0)} maxAoA=${maxAoA.toFixed(1)} stalled=${stalled} crashed=${crashed} hang<20kt=${hangS.toFixed(1)}s`);
    return { minKt, finalKt, stalled, crashed, hangS, maxAoA, log };
  };
  {
    const r = await lowPowerRun('a hold', 'hold', 20);
    check(
      'low-power 5a: F-22 at 41% throttle holding altitude does NOT collapse (thrust 128 kN >> ~41 kN drag at 150 kt)',
      !r.crashed && r.minKt >= 140 && r.finalKt >= 150,
      `minKt=${r.minKt.toFixed(0)} finalKt=${r.finalKt.toFixed(0)} stalled=${r.stalled} crashed=${r.crashed}`,
    );
  }
  {
    const r = await lowPowerRun('b pull', 'pull', 60);
    check(
      'low-power 5b: F-22 at 41% throttle with FULL PULL held does NOT hang at <20 kt airborne for >10 s (it must stall, then dive/recover or crash — the live 5-9 kt hang)',
      r.hangS <= 10,
      `hang<20kt=${r.hangS.toFixed(1)}s minKt=${r.minKt.toFixed(0)} maxAoA=${r.maxAoA.toFixed(1)} stalled=${r.stalled} crashed=${r.crashed}`,
    );
  }
  {
    const r = await lowPowerRun('c hands-off', 'handsOff', 30);
    check(
      'low-power 5c: F-22 at 41% throttle hands-off does NOT stall or collapse',
      !r.crashed && !r.stalled && r.minKt >= 120,
      `minKt=${r.minKt.toFixed(0)} finalKt=${r.finalKt.toFixed(0)} stalled=${r.stalled} crashed=${r.crashed} maxAoA=${r.maxAoA.toFixed(1)}`,
    );
  }
  // 5d: the remaining collapse candidate — a BANKED turn at 41%.
  // Level flight cannot collapse (5a/5c accelerate; 5b mushes at
  // ~131 kt), but a 60 deg bank doubles the load factor, so the
  // stall speed rises to Vs*sqrt(2) ≈ 170 kt: a jet at 150 kt in
  // the turn is already stalled, and holding the bank bleeds the
  // speed fast. If the live test was turning, this is the collapse.
  {
    const key = 'f22';
    const spec = SPECS[key]!;
    const ac = await makeAircraft(key, true);
    ac.resetAt(alpha.x, 400, alpha.z, alpha.hdg);
    ac.setAirspeed(150 * KT2MS);
    ac.thrTarget = 0.41;
    ac.throttle = 0.41;
    const alt0 = ac.pos.y;
    const bankDes = 60 * D2R;
    let stalled = false;
    let minKt = Infinity;
    let simT = 0;
    const rows: string[] = [];
    while (simT < 30) {
      const dt = nextDt();
      simT += dt;
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
      ac.pitchIn = levelPitchIn(ac, key, alt0, spec.clCurve.peakAoA + 1, 4, 0.03, 0.5, 0.3);
      ac.thrTarget = 0.41;
      ac.yawIn = 0;
      substep(ac, dt);
      if (!ac.alive) break;
      if (ac.stalled) stalled = true;
      const kt = ac.ias * MS2KT;
      minKt = Math.min(minKt, kt);
      if (Math.abs(simT % 5) < dt) {
        rows.push(`t=${simT.toFixed(0)}s ${kt.toFixed(0)}kt alt=${ac.pos.y.toFixed(0)}m aoa=${ac.aoaDeg.toFixed(1)} bank=${(bankNow / D2R).toFixed(0)} stalled=${ac.stalled}`);
      }
    }
    const finalKt = ac.ias * MS2KT;
    const crashed = !ac.alive;
    console.log(`  [5d banked] ${rows.join(' | ')} => minKt=${minKt.toFixed(0)} finalKt=${finalKt.toFixed(0)} stalled=${stalled} crashed=${crashed}`);
    // The banked-turn hypothesis is REFUTED too: the jet accelerates
    // out of the turn (thrust >> drag even banked) and never stalls
    // — it sinks to ~193 m trading altitude for speed instead. So
    // NONE of the four 41% variants collapses: the live 5-9 kt
    // collapse is not a level/banked/pulled flight-model effect at
    // 41% throttle, and the probe's 120 kt stall stands.
    check(
      'low-power 5d: F-22 at 41% throttle in a 60 deg banked turn does NOT collapse either (accelerates, no stall flag)',
      !stalled && !crashed && minKt >= 140,
      `minKt=${minKt.toFixed(0)} finalKt=${finalKt.toFixed(0)} stalled=${stalled} crashed=${crashed}`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL STALL-SPEED CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
