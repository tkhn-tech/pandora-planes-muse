// BladeElementSolver — the BET flight model (BET report §5), behind
// IAerodynamicsSolver alongside CurveAerodynamicsSolver.
//
// Station layout per aircraft (per-model supplement §7): wing 16,
// tailplane 8, fin 4 (F-18) or 8 (twin canted fins), LEX 8 (F-18 /
// F-22 / MiG-29), chine 4 (F-35A), canard 8 (Su-34), fuselage 1 (2
// for the F-15E lifting body), engines 2, lift-fan 1 (F-35A). Totals:
// F-18 39, F-35A 39, F-22 43, F-15E 36, MiG-29 43, Su-34 43.
//
// Per station, per fixed step (report §5):
//   1. Local velocity v_local = v_cg + ω × r (r = station position
//      relative to the CG). This one term is what produces all the
//      damping and coupling — pitch damping Cmq, yaw damping Cnr,
//      roll damping Clp — for free.
//   2. Local AoA α_i = atan2(−v_local.y, v_local.z) + twist +
//      control deflection (ΔCl = Cl_δ·δ, Cl_δ ≈ 2.0/rad).
//   3. Section Cl from the per-model polar (clFromCurve — the same
//      vortex-lift curve as the curve solver); the tailplane sees
//      downwash α_tail = α − ε, ε ≈ 2·CL/(π·AR) (one-step lag).
//   4. Section lift dL = q_i·area_i·Cl_i; moment M = Σ r_i × F_i.
//   5. ω̇ = I⁻¹(M − ω × Iω), integrate ω, hand it to the controller
//      as `rates` (and as torque = I·ω/TAU for compatibility).
//
// Asymmetry falls out with no special cases: one wing stalled /
// damaged → its stations make less lift → rolling moment; engine-out
// → the live engine's thrust at y = ±0.55 m gives a yaw moment;
// wing loss → zero that side's stations (the controller's damage
// zones scale the station lift). The F-22's TVC is a thrust station
// whose direction rotates with the nozzle angle (±20° pitch), so at
// high AoA the nozzles contribute a direct pitch moment even at zero
// airspeed — post-stall manoeuvres fall out of the geometry. The
// F-35A's hover is two thrust stations (lift fan forward, nozzle aft)
// plus roll-post stations at the wingtips; the TOTAL vertical thrust
// is still throttle·thrustMax (191 kN), so hover stays at 94% at
// combat mass — the honest F-35B number, do not tune it down.
// (At vector 0 the fan is covered and the nozzle carries 100%.)
//
// The spin is a yaw-rate state (SpinModel, BET §2) applied as a
// MOMENT — a PD term driving ω_y toward the spin target rate — so it
// converges instead of accumulating. Drag is computed globally
// (CD = CD0 + k·CL² + separation + flaps / airbrake / gear, on the
// total CL) exactly as in the curve solver, so it is not
// double-counted per station and the §0 sustained-turn calibration
// (F-18 19.0°/s at 400 kt SL) holds for both solvers.
// Cost: ~40 stations × ~60 flops ≈ 2,400 flops per solve — negligible.
// Station arrays are preallocated and reused; no per-station objects.

import * as THREE from 'three';
import {
  clFromCurve,
  controlPitch,
  controlRoll,
  controlYaw,
  rollCapAtSpeed,
  type AeroResult,
  type AeroState,
  type DamageZones,
  type FlightControls,
  type IAerodynamicsSolver,
} from './IAerodynamicsSolver';
import { SpinModel } from './SpinModel';
import {
  inertias,
  TORQUE_TAU,
} from './CurveAerodynamicsSolver';
import type { AircraftSpec, ClCurve } from '../aircraft/AircraftSpec';

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const G = 9.81;
const RHO_0 = 1.225;
const RHO_SCALE_H = 8500;
const CD_0 = 0.028;
const CD_FLAP = 0.06;
const CD_AIRBRAKE = 0.12;
const CD_GEAR = 0.03;
const GE_CL = 1.15;
const GE_CD = 0.9;
const CL_DELTA = 2.0; // Cl per radian of control deflection (§5)
const ELEV_DEFLECT_DEG = 25; // max elevator / canard deflection
const AIL_DEFLECT_DEG = 20; // max aileron deflection
const RUD_DEFLECT_DEG = 25; // max rudder deflection
const ENGINE_Y = 0.55; // twin-engine lateral offset, m
const HYDRO_PITCH = 0.4; // pitch authority with hydraulic failure
const ENGINE_DMG_THRUST = 0.7;
const ROTOR_TRANSLIFT = 0.3;
const ROTOR_DRAG = 0.05;
const SPIN_MOMENT_TAU = 0.5; // s — spin moment drives ω toward the spin rate

type StationKind =
  | 'wing'
  | 'tail'
  | 'fin'
  | 'lex'
  | 'chine'
  | 'canard'
  | 'fuselage'
  | 'engine'
  | 'liftfan'
  | 'rollpost';

interface Station {
  kind: StationKind;
  /** Position relative to the CG, body frame (x right, y up, z fwd). */
  r: THREE.Vector3;
  /** Planform area, m². */
  area: number;
  /** Chord, m (for the polar reference; not used in the force). */
  chord: number;
  /** Side: −1 left, +1 right, 0 centreline. */
  side: number;
  /** Control surface on this station, if any. */
  control: 'aileron' | 'elevator' | 'rudder' | 'none';
  /** CL scale for this surface (LEX vortex lift 1.1, fuselage 0.5). */
  clScale: number;
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

/** Per-model station-layout table (supplement §7). `fin` is the total
 *  fin-station count (4 = 2 per fin, 8 = 4 per fin); `lex` covers LEX
 *  (F-18/F-22/MiG-29) and chine (F-35A, 4); `canard` is Su-34 only. */
const LAYOUT: Record<
  string,
  { fin: number; lex: number; canard: number; fuselage: number; liftFan: boolean }
> = {
  fa18: { fin: 4, lex: 8, canard: 0, fuselage: 1, liftFan: false },
  f35a: { fin: 8, lex: 4, canard: 0, fuselage: 1, liftFan: true },
  f22: { fin: 8, lex: 8, canard: 0, fuselage: 1, liftFan: false },
  f15e: { fin: 8, lex: 0, canard: 0, fuselage: 2, liftFan: false },
  mig29: { fin: 8, lex: 8, canard: 0, fuselage: 1, liftFan: false },
  su34: { fin: 8, lex: 0, canard: 8, fuselage: 1, liftFan: false },
  heli: { fin: 0, lex: 0, canard: 0, fuselage: 0, liftFan: false },
};

/** Build the station table for one spec. Areas are sized so the wing
 *  stations sum to the wing area S (the model's CL curve is the
 *  whole-aircraft curve, dominated by the wing); the tail / LEX /
 *  canard / fuselage areas are small fractions of S, so the total lift
 *  stays within ~10% of the curve solver at the same AoA and the
 *  takeoff numbers agree. */
export function buildStations(spec: AircraftSpec): Station[] {
  const layout = LAYOUT[spec.key] ?? LAYOUT.fa18;
  const S = spec.wingArea;
  const span = spec.span;
  const L = spec.length;
  const stations: Station[] = [];
  const meanChord = S / span;
  const rootChord = meanChord / 0.7; // taper 1.0 → 0.4, mean 0.7

  // Wing: 8 per half, chord tapering root → tip, AC slightly aft of
  // the CG for a swept wing. Ailerons on the outer half (i ≥ 4).
  for (const side of [-1, 1]) {
    for (let i = 0; i < 8; i++) {
      const t = (i + 0.5) / 8; // 0 root → 1 tip
      const chord = rootChord * (1 - 0.6 * t);
      stations.push({
        kind: 'wing',
        r: new THREE.Vector3(side * (span / 2) * t, -0.3, -0.08 * L),
        area: chord * (span / 16),
        chord,
        side,
        control: i >= 4 ? 'aileron' : 'none',
        clScale: 1,
      });
    }
  }
  // Tailplane: 4 per half, at the tail, all-moving stabilators.
  const tailSpan = 0.35 * span;
  const tailChord = meanChord * 0.55;
  for (const side of [-1, 1]) {
    for (let i = 0; i < 4; i++) {
      const t = (i + 0.5) / 4;
      stations.push({
        kind: 'tail',
        r: new THREE.Vector3(side * (tailSpan / 2) * t, 0.2, -0.42 * L),
        area: tailChord * (tailSpan / 8),
        chord: tailChord,
        side,
        control: 'elevator',
        clScale: 1,
      });
    }
  }
  // Fins: twin canted fins (2 or 4 stations per fin), vertical.
  // Their "lift" is a side force (along body-x), AoA = sideslip β.
  const finsPerSide = layout.fin / 2;
  for (const side of [-1, 1]) {
    for (let i = 0; i < finsPerSide; i++) {
      const t = (i + 0.5) / finsPerSide;
      stations.push({
        kind: 'fin',
        r: new THREE.Vector3(side * 0.8, 0.5 + t * 1.8, -0.4 * L),
        area: meanChord * 0.4 * (2.2 / finsPerSide),
        chord: meanChord * 0.4,
        side,
        control: 'rudder',
        clScale: 1,
      });
    }
  }
  // LEX / chine: vortex-lift stations at the wing root, forward.
  const lexCount = layout.lex;
  if (lexCount > 0) {
    const perSide = lexCount / 2;
    for (const side of [-1, 1]) {
      for (let i = 0; i < perSide; i++) {
        const t = (i + 0.5) / perSide;
        stations.push({
          kind: spec.key === 'f35a' ? 'chine' : 'lex',
          r: new THREE.Vector3(
            side * 0.12 * span,
            -0.2,
            (spec.key === 'f35a' ? 0.15 : 0.1) * L * (1 - t * 0.3),
          ),
          area: (S * 0.04) / lexCount,
          chord: meanChord * 0.5,
          side,
          control: 'none',
          clScale: spec.key === 'f35a' ? 0.8 : 1.1,
        });
      }
    }
  }
  // Canard (Su-34): forward of the CG, all-moving, opposite elevator
  // sign to the tailplane (it is ahead of the CG).
  if (layout.canard > 0) {
    const perSide = layout.canard / 2;
    for (const side of [-1, 1]) {
      for (let i = 0; i < perSide; i++) {
        const t = (i + 0.5) / perSide;
        stations.push({
          kind: 'canard',
          r: new THREE.Vector3(side * 0.12 * span, 0, 0.28 * L * (1 - t * 0.2)),
          area: (S * 0.06) / layout.canard,
          chord: meanChord * 0.5,
          side,
          control: 'elevator',
          clScale: 1,
        });
      }
    }
  }
  // Fuselage: drag-only for most; the F-15E's flat fuselage is a
  // lifting body (2 stations with real lift), the MiG-29 / Su-34
  // blended bodies get one lifting station each.
  const fusLift = spec.key === 'f15e' || spec.key === 'mig29' || spec.key === 'su34';
  for (let i = 0; i < layout.fuselage; i++) {
    stations.push({
      kind: 'fuselage',
      r: new THREE.Vector3(0, 0, (i - (layout.fuselage - 1) / 2) * 0.15 * L),
      area: S * (spec.key === 'f15e' ? 0.12 : 0.05),
      chord: meanChord,
      side: 0,
      control: 'none',
      clScale: fusLift ? 0.5 : 0,
    });
  }
  // Engines: thrust stations (the force is applied here, so the
  // moment r × T falls out — engine-out yaw, TVC pitch, hover trim).
  // Twin engines at y = ±0.55 m; the F-35A gets one nozzle + one
  // lift fan + two roll posts.
  if (spec.archetype === 'vtol' && layout.liftFan) {
    stations.push({
      kind: 'engine',
      r: new THREE.Vector3(0, -0.2, -0.45 * L),
      area: 0,
      chord: 0,
      side: 0,
      control: 'none',
      clScale: 0,
    });
    stations.push({
      kind: 'liftfan',
      r: new THREE.Vector3(0, 0.3, 2.5),
      area: 0,
      chord: 0,
      side: 0,
      control: 'none',
      clScale: 0,
    });
    for (const side of [-1, 1]) {
      stations.push({
        kind: 'rollpost',
        r: new THREE.Vector3(side * 0.45 * span, -0.3, -0.05 * L),
        area: 0,
        chord: 0,
        side,
        control: 'none',
        clScale: 0,
      });
    }
  } else {
    for (const side of [-1, 1]) {
      stations.push({
        kind: 'engine',
        r: new THREE.Vector3(side * ENGINE_Y, -0.2, -0.3 * L),
        area: 0,
        chord: 0,
        side,
        control: 'none',
        clScale: 0,
      });
    }
  }
  return stations;
}

const DEFAULT_CL_CURVE: ClCurve = {
  linearEndAoA: 15,
  linearEndCL: 1.35,
  peak: 1.6,
  peakAoA: 35,
  floor: 0.8,
  postSlope: 0.05,
  cdSep: 0.35,
};

export class BladeElementSolver implements IAerodynamicsSolver {
  private readonly stations: Station[];
  private readonly spec: AircraftSpec;
  private stalled = false;
  private readonly spinModel = new SpinModel();
  /** Previous step's total CL — the tailplane downwash ε = 2·CL/(π·AR)
   *  is computed from it (one-step lag, physical: the downwash takes
   *  time to reach the tail). */
  private prevCL = 0;

  constructor(spec: AircraftSpec) {
    this.spec = spec;
    this.stations = buildStations(spec);
  }

  /** Station count (for the driver / debug — supplement §7 totals). */
  get stationCount(): number {
    return this.stations.length;
  }

  solve(state: AeroState): AeroResult {
    const dt = state.dt ?? 1 / 60;
    const m = state.mass;
    const S = state.wingArea;
    const S_lift = state.wingAreaLift ?? S;
    const span = state.span;
    const length = state.length ?? span * 1.4;
    const dmg: DamageZones = state.damage;
    const engineDmgAvg = (dmg.engineL + dmg.engineR) / 2;
    const controls: FlightControls | undefined = state.controls;
    const pitchIn = controls ? controlPitch(controls) : 0;
    const rollIn = controls ? controlRoll(controls) : 0;
    const yawIn = controls ? controlYaw(controls) : 0;
    const auth = state.controlAuthority ?? { turn: 1, rollDeg: 150, pitch: 1 };
    const fail = state.failures ?? {};
    const curve = state.clCurve ?? DEFAULT_CL_CURVE;
    const inducedK = state.inducedK ?? 0.1;
    const spec = this.spec;

    const vAir = state.velocityAir;
    const v = vAir.length();
    const quat = state.quat;
    const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(quat);
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);

    // Body-frame air velocity (for the station-local velocities).
    const vBody = vAir.clone().applyQuaternion(quat.clone().invert());
    let aoaDeg = 0;
    let betaDeg = 0;
    if (v > 5) {
      aoaDeg = Math.atan2(-vBody.y, vBody.z) * R2D;
      betaDeg = Math.atan2(vBody.x, vBody.z) * R2D;
    }

    const alt = Math.max(state.altitude, 0);
    const rho =
      state.density > 0 ? state.density : RHO_0 * Math.exp(-alt / RHO_SCALE_H);
    const q = 0.5 * rho * v * v;

    // --- Control deflections (rad) ---
    // Elevator: pitchIn < 0 (pull) → the tailplane's Cl DECREASES
    // (trailing-edge-up elevator → downward tail force → nose up).
    // The tail is aft of the CG, so a downward tail force pitches the
    // nose up (M_x = −r.z·Fy < 0 = nose up). Canard: opposite sign
    // (ahead of the CG, more lift → nose up).
    const hydroPitch = fail.hydroDegraded ? HYDRO_PITCH : 1;
    const elevDeflect = pitchIn * ELEV_DEFLECT_DEG * D2R * hydroPitch * auth.pitch;
    const ailDeflect = rollIn * AIL_DEFLECT_DEG * D2R;
    const rudDeflect = -yawIn * RUD_DEFLECT_DEG * D2R;
    // F-22 TVC: the nozzle angle follows the pitch input (±20°).
    const tvcAngle = (state.tvcDeg ?? 0) * D2R * clamp(-pitchIn, -1, 1);

    // --- Thrust (with the §0 lapse) ---
    const lapse = Math.pow(Math.max(rho / RHO_0, 0.05), 0.7);
    const thrustTotal =
      state.throttle *
      state.thrustMax *
      lapse *
      (1 - engineDmgAvg * ENGINE_DMG_THRUST) *
      (1 - 0.5 * (fail.engineOut ? 1 : 0));
    // Per-engine thrust (0 when that engine is dead — the damage
    // zones drive the split, so the yaw moment falls out).
    const engL = (thrustTotal * (1 - dmg.engineL)) / 2;
    const engR = (thrustTotal * (1 - dmg.engineR)) / 2;
    const thrustEff = engL + engR;

    // --- Station loop: lift + moment ---
    // ω is the current body-frame angular velocity (from the controller
    // — the Rapier body's rate, so contacts / damping feed back in).
    // When the controller omits it, start from zero.
    const omega = state.angVel ? state.angVel.clone() : new THREE.Vector3();
    const AR = (span * span) / S;
    const downwash = (2 * this.prevCL) / (Math.PI * AR); // rad
    const flapFrac = state.flapsDeg / 30;
    const stallA = curve.linearEndAoA + flapFrac * 3;
    if (aoaDeg > stallA) this.stalled = true;
    else if (aoaDeg < stallA - 3) this.stalled = false;

    const totalMoment = new THREE.Vector3(); // body frame
    const vLocal = new THREE.Vector3();
    const omegaCrossR = new THREE.Vector3();
    let liftSum = 0; // total lift (along body-up), N

    for (const st of this.stations) {
      if (st.kind === 'engine' || st.kind === 'liftfan' || st.kind === 'rollpost') {
        continue; // thrust stations — handled after the aero loop
      }
      // v_local = v_cg + ω × r — the damping/coupling term (§5.1).
      omegaCrossR.crossVectors(omega, st.r);
      vLocal.copy(vBody).add(omegaCrossR);
      const vL = vLocal.length();
      if (vL < 0.5) continue; // no airflow over this station
      const q_i = 0.5 * rho * vL * vL;

      if (st.kind === 'fin') {
        // Fin: side force from the sideslip β, plus rudder.
        // M = r × F with F = (Fx, 0, 0): M_y = z·Fx, M_z = −y·Fx.
        const betaLocal = Math.atan2(vLocal.x, vLocal.z);
        let cSide = clamp(4.0 * betaLocal, -1.2, 1.2);
        if (st.control === 'rudder') cSide += CL_DELTA * rudDeflect;
        const fSide = q_i * st.area * cSide;
        totalMoment.y += st.r.z * fSide;
        totalMoment.z += -st.r.y * fSide;
        continue;
      }

      // Lifting surface: local AoA in the pitch plane.
      let alpha = Math.atan2(-vLocal.y, vLocal.z) * R2D;
      if (st.kind === 'tail') alpha -= downwash * R2D;
      // Wing damage scales the station lift (one wing stalled /
      // damaged → less lift → rolling moment, no special cases).
      let dmgScale = 1;
      if (st.kind === 'wing') {
        dmgScale = 1 - (st.side < 0 ? dmg.wingL : dmg.wingR) * 0.8;
      } else if (st.kind === 'tail') {
        dmgScale = 1 - dmg.tail * 0.7;
      }
      let cl: number;
      if (st.kind === 'wing' || st.kind === 'lex' || st.kind === 'chine') {
        cl = clFromCurve(alpha, flapFrac, curve) * st.clScale;
      } else if (st.kind === 'canard') {
        cl = clamp(4.0 * alpha * D2R, -1.2, 1.2);
        cl += CL_DELTA * -elevDeflect; // canard: opposite elevator sign
      } else if (st.kind === 'tail') {
        cl = clamp(4.0 * alpha * D2R, -1.0, 1.0);
        cl += CL_DELTA * elevDeflect;
      } else {
        // Fuselage lifting body.
        cl = clamp(2.0 * alpha * D2R, -0.8, 0.8) * st.clScale;
      }
      if (st.control === 'aileron') {
        // Differential aileron: rollIn > 0 (bank left) → left wing
        // Cl down, right wing Cl up.
        cl += CL_DELTA * ailDeflect * (st.side < 0 ? -1 : 1);
      }
      cl *= dmgScale;
      const dL = q_i * st.area * cl;
      // Lift direction: perpendicular to the local velocity in the
      // pitch plane, with the +y sense (up). vHat = (vx, vy, vz) →
      // perpendicular (0, vz, −vy) is up when flying forward.
      const vHat = vLocal.clone().normalize();
      const liftDirY = vHat.z;
      const liftDirZ = -vHat.y;
      const liftDirLen = Math.hypot(liftDirY, liftDirZ);
      const ly = liftDirLen > 0.01 ? liftDirY / liftDirLen : 1;
      const lz = liftDirLen > 0.01 ? liftDirZ / liftDirLen : 0;
      // Force (body frame): (0, dL·ly, dL·lz). Moment r × F.
      totalMoment.x += st.r.y * (dL * lz) - st.r.z * (dL * ly);
      totalMoment.y += st.r.z * 0 - st.r.x * (dL * lz);
      totalMoment.z += st.r.x * (dL * ly) - st.r.y * 0;
      liftSum += dL * Math.max(0, ly);
    }

    // --- Thrust forces + moments ---
    // Thrust direction (body frame): fwd·cos(vec + tvc) +
    // up·sin(vec + tvc). For the F-35A the vector angle is the VTOL
    // vector; for the F-22 the TVC angle follows the pitch input.
    const vecAngle = spec.archetype === 'vtol' ? state.vectorDeg * D2R : 0;
    const thrustDirBody = new THREE.Vector3(0, 0, Math.cos(vecAngle + tvcAngle));
    thrustDirBody.y = Math.sin(vecAngle + tvcAngle);
    const isVtolFan = spec.archetype === 'vtol';
    // F-35A fan split: the fan carries 47% of the thrust at full
    // vector, 0% at vector 0 (covered) — the nozzle carries the rest,
    // so the TOTAL is always throttle·thrustMax (hover stays 94%).
    // Hover pitch control: pull → more fan (forward) → nose up.
    const hoverBlend = clamp((state.vectorDeg - 45) / 45, 0, 1);
    const fanShare =
      (0.47 + -pitchIn * 0.1 * hoverBlend) * clamp(vecAngle / (Math.PI / 2), 0, 1);
    const addThrust = (r: THREE.Vector3, dir: THREE.Vector3, t: number): void => {
      // Moment r × (dir·t), body frame.
      const fx = dir.x * t;
      const fy = dir.y * t;
      const fz = dir.z * t;
      totalMoment.x += r.y * fz - r.z * fy;
      totalMoment.y += r.z * fx - r.x * fz;
      totalMoment.z += r.x * fy - r.y * fx;
    };
    for (const st of this.stations) {
      if (st.kind === 'engine') {
        const t = isVtolFan
          ? thrustEff * (1 - fanShare)
          : st.side < 0
            ? engL
            : engR;
        addThrust(st.r, thrustDirBody, t);
      } else if (st.kind === 'liftfan') {
        addThrust(st.r, thrustDirBody, thrustEff * fanShare);
      } else if (st.kind === 'rollpost') {
        // Vertical thrust at the wingtips, hover only; the roll input
        // splits them differentially (roll control in hover).
        const t =
          thrustEff * 0.045 * clamp(vecAngle / (Math.PI / 2), 0, 1) *
          (1 + (st.side < 0 ? -1 : 1) * rollIn * 0.5 * hoverBlend);
        addThrust(st.r, new THREE.Vector3(0, 1, 0), t);
      }
    }

    // --- Total lift / drag (world frame) ---
    // Drag is global (curve-solver formula on the total CL) so it is
    // not double-counted per station and the §0 calibration holds.
    const clTotal = q > 1 ? liftSum / (q * S_lift) : 0;
    this.prevCL = clTotal;
    const cdSep = curve.cdSep * Math.pow(Math.max(0, (aoaDeg - 20) / 15), 2);
    const cdParasitic =
      CD_0 +
      flapFrac * CD_FLAP +
      state.airbrake * CD_AIRBRAKE +
      (state.gearDown ? CD_GEAR : 0);
    let geCL = 1;
    let geCD = 1;
    if (
      state.heightAboveGround !== undefined &&
      state.heightAboveGround < span &&
      v > 30
    ) {
      geCL = GE_CL;
      geCD = GE_CD;
    }
    const clEff = clTotal * geCL;
    const cdEff =
      (cdParasitic + (S_lift / S) * inducedK * clTotal * clTotal + cdSep) * geCD;
    let lift = q * S_lift * clEff;
    const drag =
      q * S * cdParasitic * geCD +
      q * S_lift * (inducedK * clTotal * clTotal + cdSep) * geCD;

    // Rotary archetype: never routed here (the controller keeps the
    // curve solver for helis), but stay safe if it is.
    if (state.archetype === 'rotary') {
      let rotorLift =
        state.throttle * state.thrustMax * (1 + clamp(v / 40, 0, 1) * ROTOR_TRANSLIFT);
      if (state.throttle < 0.15 && v > 20) {
        rotorLift = Math.max(rotorLift, 0.35 * state.thrustMax);
      }
      lift = rotorLift;
    }

    const liftVec = new THREE.Vector3();
    const dragVec = new THREE.Vector3();
    if (v > 1) {
      liftVec.addScaledVector(up, lift);
      dragVec.addScaledVector(vAir.clone().normalize(), -drag);
    }
    // Thrust (world frame): the body-frame thrust direction rotated
    // into the world. All thrust (nozzle + fan + roll posts) acts
    // along the vectored direction when active, so the total is
    // thrustEff along thrustDirBody.
    const thrustVecWorld = thrustDirBody.clone().applyQuaternion(quat).multiplyScalar(thrustEff);

    const gLoad = Math.max(0.2, lift / (m * G));

    // --- Spin (BET §2) as a moment ---
    // The spin is a yaw-rate state; it is applied as a PD moment
    // driving ω toward the spin target rate (τ = 0.5 s), so it
    // converges instead of accumulating. The spin builds while
    // stalled and yawed and decays on NATOPS recovery input.
    const spinStep = this.spinModel.step(
      dt,
      this.stalled,
      yawIn,
      pitchIn,
      rollIn,
      dmg.wingL - dmg.wingR,
    );
    const inertia = inertias(m, span, length);
    // Only when the spin is actually developed — otherwise this PD
    // term would be a strong yaw damper (driving ω_y → 0) and would
    // kill the coordinated yaw and the rudder response.
    if (this.spinModel.spin > 0.01) {
      totalMoment.y +=
        (inertia.iy * (spinStep.yawRate - omega.y)) / SPIN_MOMENT_TAU;
      totalMoment.z +=
        (inertia.iz * (spinStep.rollRate - omega.z)) / SPIN_MOMENT_TAU;
    }

    // --- Angular dynamics: ω̇ = I⁻¹(M − ω × Iω) ---
    const Iw = new THREE.Vector3(
      inertia.ix * omega.x,
      inertia.iy * omega.y,
      inertia.iz * omega.z,
    );
    const gyro = new THREE.Vector3().crossVectors(omega, Iw);
    const omegaDot = new THREE.Vector3(
      (totalMoment.x - gyro.x) / inertia.ix,
      (totalMoment.y - gyro.y) / inertia.iy,
      (totalMoment.z - gyro.z) / inertia.iz,
    );
    const newOmega = omega.clone().addScaledVector(omegaDot, dt);
    // Rate limits (the airframe's authority): pitch 55°/s, yaw 90°/s
    // (the spin max), roll spec.rollDeg (MiG-29 airspeed schedule).
    const rollCapDeg = state.rollCapSchedule
      ? Math.min(auth.rollDeg, rollCapAtSpeed(v, state.rollCapSchedule))
      : auth.rollDeg;
    newOmega.x = clamp(newOmega.x, -55 * D2R, 55 * D2R);
    newOmega.y = clamp(newOmega.y, -90 * D2R, 90 * D2R);
    newOmega.z = clamp(newOmega.z, -rollCapDeg * D2R, rollCapDeg * D2R);

    const torque = new THREE.Vector3(
      (inertia.ix * newOmega.x) / TORQUE_TAU,
      (inertia.iy * newOmega.y) / TORQUE_TAU,
      (inertia.iz * newOmega.z) / TORQUE_TAU,
    );

    return {
      lift: liftVec,
      drag: dragVec,
      thrust: thrustVecWorld,
      torque,
      rates: {
        pitchRate: newOmega.x,
        yawRate: newOmega.y,
        rollRate: newOmega.z,
      },
      aoaDeg,
      cl: clTotal,
      cd: cdEff,
      stalled: this.stalled,
      spin: this.spinModel.spin,
      spinArrow: this.spinModel.arrow,
      spinDir: this.spinModel.spinDir,
      betaDeg,
      gLoad,
    };
  }
}
