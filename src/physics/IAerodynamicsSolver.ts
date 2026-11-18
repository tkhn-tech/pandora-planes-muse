// IAerodynamicsSolver — the contract between the flight model and the
// physics controller. Ported from the v2 HTML template
// (pandora-template.html ~958–1075, Aircraft.step): the same CL/CD curve,
// stall + spin, ISA density, ground effect, VTOL vector and rotary branch,
// with the v1.1 body-frame AoA fix (atan2(-vBody.y, vBody.z) from the AIR
// velocity — never the old asin flight-path formula).
//
// OWNERSHIP LINE (Rapier vs solver): the solver owns FORCES. It is a pure
// function of the aero state (plus stall/spin hysteresis held on the
// solver instance, one instance per aircraft). It never moves anything.
// The controller (FlightPhysicsController) owns INTEGRATION: it turns the
// solver's forces into a velocity and hands that velocity to a Rapier
// dynamic rigid body, which owns POSITION and COLLISIONS. Rapier gravity
// is switched off on aircraft bodies (gravityScale 0) because gravity is
// already inside the solver's force sum — otherwise it would be counted
// twice.
//
// FlightControls — RECONCILIATION NOTE. Two shapes for this type already
// exist in the project and both must keep compiling:
//   * The input workers (PCFlightInput / MobileFlightInput /
//     IFlightInputProvider, written first) return the MERGED STICK shape:
//     { pitch, roll, yaw, throttle: number | null } where throttle null
//     means "no opinion this frame, keep the last target".
//   * The physics task brief specified the long shape: pitchIn / rollIn /
//     yawIn plus flaps / airbrake / vector / gear / hook / brake / firing.
//   * The AI controller returns { pitchIn, rollIn, yawIn, thrTarget }.
// All fields are therefore optional, and consumers must resolve through
// controlPitch / controlRoll / controlYaw / resolveThrottleTarget.
// The integrator (main.ts) is the place that should one day canonicalise
// this to a single shape; until then, producers may use any naming and
// consumers must use the resolve* helpers, never the raw fields.

import * as THREE from 'three';
import type {
  AircraftArchetype,
  ClCurve,
  RollCapSchedule,
} from '../aircraft/AircraftSpec';

const G = 9.81;

/** Per-model lift coefficient from the piecewise ClCurve (BET report §2
 *  + per-model supplement §1–§6). Anchors: (0°, CL_0) →
 *  (linearEndAoA, linearEndCL) → (peakAoA, peak), then a linear
 *  post-stall fall at postSlope per degree, floored at `floor`. Flaps
 *  add flapFrac·0.35 to the linear/vortex region only (v2 behaviour —
 *  the post-stall branch is flap-independent) and shift the section
 *  stall by flapFrac·3°. Negative AoA mirrors the linear slope about
 *  CL_0 (no flap increment below zero). */
export function clFromCurve(
  aoaDeg: number,
  flapFrac: number,
  curve: ClCurve,
): number {
  const CL_0 = 0.15;
  const FLAP_CL = 0.35;
  const flap = flapFrac * FLAP_CL;
  if (aoaDeg < 0) {
    // Symmetric-ish negative branch: linear slope about CL_0, no flaps.
    const slope = (curve.linearEndCL - CL_0) / Math.max(curve.linearEndAoA, 1);
    return Math.max(-0.8, CL_0 + slope * aoaDeg);
  }
  if (aoaDeg <= curve.linearEndAoA) {
    const slope =
      (curve.linearEndCL - CL_0) / Math.max(curve.linearEndAoA, 1);
    return CL_0 + flap + slope * aoaDeg;
  }
  if (aoaDeg <= curve.peakAoA) {
    const t =
      (aoaDeg - curve.linearEndAoA) /
      Math.max(curve.peakAoA - curve.linearEndAoA, 1);
    return curve.linearEndCL + flap + (curve.peak - curve.linearEndCL) * t;
  }
  return Math.max(
    curve.floor,
    curve.peak - curve.postSlope * (aoaDeg - curve.peakAoA),
  );
}

/** Sustained turn rate (rad/s) at the current speed/density — the §0
 *  calibration, computed not quoted: at the sustained turn drag =
 *  thrust, so CD = T/(qS) = CD0 + k·CL², n = qS·CL/W, and
 *  ω = g·√(n²−1)/V. Returns 0 when the jet cannot sustain any turn
 *  (thrust below parasitic drag). CL is capped at clMax (the airfoil's
 *  peak — without the cap the formula lets CL run to 7+ at low speed,
 *  where thrust dwarfs qS, and reports a turn rate the wing cannot
 *  actually hold; the AI then banks 40°+ at 35 m/s, bleeds energy and
 *  never accelerates). Used by the curve solver's coordinated-yaw cap
 *  and by the AI's two-loop bank command, so both agree on the envelope. */
export function sustainedTurnRate(
  vMs: number,
  rho: number,
  thrustN: number,
  wingArea: number,
  inducedK: number,
  mass: number,
  clMax = 1.6,
): number {
  if (vMs < 5) return 0;
  const q = 0.5 * rho * vMs * vMs;
  const qS = q * wingArea;
  const cdAvail = thrustN / Math.max(qS, 1) - 0.028;
  if (cdAvail <= 0) return 0;
  const cl = Math.min(Math.sqrt(cdAvail / Math.max(inducedK, 0.01)), clMax);
  const n = (qS * cl) / (mass * G);
  if (n <= 1) return 0;
  return (G * Math.sqrt(n * n - 1)) / vMs;
}

/** Roll-rate cap from an airspeed schedule (MiG-29 supplement §5):
 *  linearly interpolate the [speed, maxRate] breakpoints, clamped at
 *  the ends. Returns +Infinity when no schedule is given. */
export function rollCapAtSpeed(
  vMs: number,
  schedule?: RollCapSchedule,
): number {
  if (!schedule || schedule.length === 0) return Infinity;
  if (vMs <= schedule[0][0]) return schedule[0][1];
  for (let i = 1; i < schedule.length; i++) {
    if (vMs <= schedule[i][0]) {
      const [v0, r0] = schedule[i - 1];
      const [v1, r1] = schedule[i];
      const t = (vMs - v0) / Math.max(v1 - v0, 1);
      return r0 + (r1 - r0) * t;
    }
  }
  return schedule[schedule.length - 1][1];
}

/** Per-zone structural damage, 0 = healthy .. 1 = destroyed. The v2
 *  template carried two scalars (wingDmg, engineDmg); the port splits
 *  them per side so asymmetric damage can roll/yaw the jet. `fuel` is
 *  the fuel-tank zone (damage-model §3 — a destroyed tank starts a
 *  fuel fire); `fin` is the vertical fin (damage-model §7 — losing it
 *  alone causes wild yaw/roll without killing the jet outright). The
 *  solvers read wingL/wingR/tail/engineL/engineR; fuel/fin are read
 *  by the controller's fire state machine and the HUD. */
export interface DamageZones {
  wingL: number;
  wingR: number;
  tail: number;
  engineL: number;
  engineR: number;
  fuel: number;
  fin: number;
}

export const ZERO_DAMAGE: DamageZones = {
  wingL: 0,
  wingR: 0,
  tail: 0,
  engineL: 0,
  engineR: 0,
  fuel: 0,
  fin: 0,
};

/** Pilot / AI control input for one fixed step.
 *
 *  ALL FIELDS ARE OPTIONAL — three producer shapes exist in the project
 *  and all three must keep compiling:
 *    * input workers: { pitch, roll, yaw, throttle: number | null }
 *      (throttle null = "no opinion, keep the last target");
 *    * the task-brief long shape: pitchIn / rollIn / yawIn plus flaps /
 *      airbrake / vector / gear / hook / brake / firing;
 *    * the AI controller: { pitchIn, rollIn, yawIn, thrTarget }
 *      (throttle target kept on the AI, returned each update).
 *  Consumers MUST resolve through controlPitch / controlRoll /
 *  controlYaw / resolveThrottleTarget below, never the raw fields.
 *  Missing stick fields default to 0 (no input); missing throttle
 *  fields mean "keep the current target". */
export interface FlightControls {
  /** Stick pitch, -1..1. NEGATIVE pulls up (v2 template convention:
   *  pitchIn < -0.25 is the take-off rotation trigger, and
   *  pitchRate = pitchIn * 55 deg/s). */
  pitch?: number;
  /** Stick roll, -1..1. */
  roll?: number;
  /** Stick yaw / rudder, -1..1. */
  yaw?: number;
  /** Throttle target 0..1, or null = "no opinion, keep the last target"
   *  (the input workers' convention). */
  throttle?: number | null;
  /** Alias for pitch. */
  pitchIn?: number;
  /** Alias for roll. */
  rollIn?: number;
  /** Alias for yaw. */
  yawIn?: number;
  /** Throttle target 0..1 (the AI controller's alias for `throttle`). */
  thrTarget?: number;
  /** Flaps 0..30 deg (0 = up). */
  flaps?: number;
  /** Airbrake 0..1. */
  airbrake?: number;
  /** Thrust-vector angle 0..90 deg (VTOL jets only; clamped to the spec). */
  vector?: number;
  /** Gear down. */
  gear?: boolean;
  /** Tailhook down. */
  hook?: boolean;
  /** Wheel brakes held. */
  brake?: boolean;
  /** Gun trigger held. */
  firing?: boolean;
}

/** Resolve the effective stick pitch from either naming (default 0). */
export function controlPitch(c: FlightControls): number {
  return c.pitchIn ?? c.pitch ?? 0;
}

/** Resolve the effective stick roll from either naming (default 0). */
export function controlRoll(c: FlightControls): number {
  return c.rollIn ?? c.roll ?? 0;
}

/** Resolve the effective stick yaw from either naming (default 0). */
export function controlYaw(c: FlightControls): number {
  return c.yawIn ?? c.yaw ?? 0;
}

/** Resolve the effective throttle target: `throttle` wins, then the
 *  AI's `thrTarget`, then null ("keep the current target"). */
export function resolveThrottleTarget(c: FlightControls): number | null {
  return c.throttle ?? c.thrTarget ?? null;
}

/** Failure flags the G7 failure system can set on an aircraft. The
 *  solver only reads them; it never sets them. */
export interface AeroFailures {
  /** Whole-engine failure (both engines at half thrust in v2). */
  engineOut?: boolean;
  /** Single-engine-out side: -1 = left engine dead, +1 = right dead,
   *  0 = none. Yaws toward the dead engine (v2 template). */
  engineSide?: -1 | 0 | 1;
  /** Hydraulic failure: pitch authority x0.4 (v2). */
  hydroDegraded?: boolean;
}

/** Control-authority multipliers, from the spec (turn / roll / pitch). */
export interface ControlAuthority {
  /** Turn multiplier (v2 spec.turn). */
  turn: number;
  /** Max roll rate, deg/s (v2 spec.roll). */
  rollDeg: number;
  /** Pitch multiplier (v2 spec.pitch). */
  pitch: number;
}

/** Everything the solver needs for one fixed step. */
export interface AeroState {
  /** Airspeed vector (ground velocity MINUS wind), in WORLD frame.
   *  The solver converts it to the body frame itself. */
  velocityAir: THREE.Vector3;
  /** Aircraft attitude. */
  quat: THREE.Quaternion;
  /** Flying mass, kg (spec.mass — the v2 operating mass). */
  mass: number;
  /** Wing area, m^2. */
  wingArea: number;
  /** Lift reference area, m^2 (MiG-29: 45 — the blended fuselage +
   *  LERX generate ~40% of total lift, so lift uses 45 m² while drag
   *  uses wingArea 38 m²). Defaults to wingArea. */
  wingAreaLift?: number;
  /** Wingspan, m (also the ground-effect height threshold). */
  span: number;
  /** Mean chord, m. Defaults to wingArea / span. */
  chord?: number;
  /** Fuselage length, m (roll/pitch inertia model). Defaults to span*1.4. */
  length?: number;
  /** SPOOLED throttle 0..1 — the controller owns the 0.55/s spool
   *  (v2 template), the solver takes the result as-is. */
  throttle: number;
  /** Max thrust, N (spec.thrustMax). */
  thrustMax: number;
  /** Thrust-vector angle, deg (VTOL jets only). */
  vectorDeg: number;
  /** Flaps, deg (0..30). */
  flapsDeg: number;
  /** Airbrake 0..1. */
  airbrake: number;
  /** Gear down. */
  gearDown: boolean;
  /** Per-zone damage, 0..1. */
  damage: DamageZones;
  /** Altitude above sea level, m (ISA density). */
  altitude: number;
  /** Air density, kg/m^3. Pass 1.225 at sea level; <= 0 recomputes ISA. */
  density: number;
  /** Aircraft archetype — REQUIRED: the solver branches on rotary
   *  (rotor-disc lift, collective, autorotation) vs fixed/vtol. */
  archetype: AircraftArchetype;
  /** Pilot / AI input for this step. Needed for the control torques
   *  (rates). Omit for a pure aero query (no control torque). */
  controls?: FlightControls;
  /** Height above the ground / deck, m. Ground effect applies when this
   *  is below one wingspan and airspeed > 30 m/s. Omit to skip it. */
  heightAboveGround?: number;
  /** Control authority (spec.turn / roll / pitch). Defaults to 1 / 150 / 1. */
  controlAuthority?: ControlAuthority;
  /** Active failure flags (G7). */
  failures?: AeroFailures;
  /** Induced-drag factor k = 1/(pi*e*AR) for this aircraft (BET §0).
   *  Defaults to 0.10 (F-18) when omitted. */
  inducedK?: number;
  /** Per-model lift curve (BET §2 + supplement §1–§6). Defaults to the
   *  F-18 curve when omitted. */
  clCurve?: ClCurve;
  /** Body-frame angular velocity, rad/s (x pitch, y yaw, z roll) — the
   *  current Rapier body rate, for the blade-element solver's
   *  v_local = v_cg + ω × r. Omit for the curve solver (it does not
   *  need it). */
  angVel?: THREE.Vector3;
  /** Thrust-vectoring nozzle deflection, deg (F-22 TVC ±20 pitch
   *  only). Absent = no vectoring. */
  tvcDeg?: number;
  /** Airspeed roll-authority cap schedule (MiG-29). Absent = no cap. */
  rollCapSchedule?: RollCapSchedule;
  /** Lift scale, 0..1 (default 1). The controller sets this to 0.15
   *  when the aircraft is DYING (damage-model §6 step 1): the airframe
   *  is breaking up and the wing can no longer hold 1 g, so the jet
   *  freefalls instead of gliding on with zero controls. It scales the
   *  LIFT only, not the drag — a broken wing has high drag. */
  liftScale?: number;
  /** Dying (damage-model §6): the airframe is breaking up. The BET
   *  solver uses this for a nose-down departure moment (a dead jet
   *  with zeroed controls stays trimmed and glides — the live
   *  shoot-down held AoA 10 deg and G 0.8 all the way down — so
   *  the wreck must be driven nose-down into a spiral dive).
   *  The controller also applies a 0.15 rad/s nose-down bias to
   *  the rates; the moment is the solver-side half, fed back
   *  through the rate sync. */
  dying?: boolean;
  /** Fixed-step length, s. Default 1/60. Used only for the stall
   *  nose-drop rate and spin build/decay (v2 applies them per step). */
  dt?: number;
}

/** Solver output for one fixed step. lift / drag / thrust are WORLD-frame
 *  force vectors (N); torque is a BODY-frame control torque (N·m). */
export interface AeroResult {
  /** Lift force, along the body up axis (N). For a rotary archetype this
   *  is rotor-disc lift, not wing lift. */
  lift: THREE.Vector3;
  /** Drag force, opposing the airflow (N). Also carries the stall-spin
   *  lateral drift force (v2 adds it straight to velocity; here it is a
   *  force so the total stays lift + drag + thrust). */
  drag: THREE.Vector3;
  /** Thrust force, along the (possibly vectored) thrust axis (N). At
   *  vector 90 deg on a VTOL jet this is vertical lift, not forward
   *  thrust — exactly as in v2. */
  thrust: THREE.Vector3;
  /** Control torque in the BODY frame (N·m), built from the v2
   *  rate-authority model: torque = I * rate / TAU with TAU = 0.25 s, so
   *  the controller recovers the template's pitch/roll/yaw rates via
   *  torqueToRates(). The v2 template has no aerodynamic moment model —
   *  only rate authority — so this torque is the rate command expressed
   *  as a torque, not a free moment. */
  torque: THREE.Vector3;
  /** Angle of attack, deg (body-frame, v1.1 fix). */
  aoaDeg: number;
  /** Lift coefficient (after ground effect, clamped -0.8..1.5). */
  cl: number;
  /** Drag coefficient. */
  cd: number;
  /** True while stalled (with the v2 3-deg hysteresis). */
  stalled: boolean;
  /** Spin amount 0..1 (builds while stalled and yawed, decays otherwise). */
  spin: number;
  /** Spin command-arrow direction for the HUD (BET §2 / NATOPS): −1 =
   *  push the stick left, +1 = push it right, 0 = no arrow (spin ≤ 0.5
   *  or not spinning). The arrow points the anti-spin lateral-stick
   *  direction — opposite the spin's yaw. */
  spinArrow: -1 | 0 | 1;
  /** Spin direction: +1 = yawing right, −1 = yawing left (the
   *  SpinModel's spinDir). The AI's spin recovery needs it even when
   *  the arrow is 0 (spin ≤ 0.5) — the anti-spin rudder is opposite
   *  this, and without it the spin only decays at the slow neutral
   *  rate. */
  spinDir: 1 | -1;
  /** Sideslip angle β = atan2(vBody.x, vBody.z), deg. Positive = air
   *  arriving from the right (nose left of the velocity). The AI feeds
   *  rudder to centre the ball from this. */
  betaDeg: number;
  /** Direct angular rates (rad/s, body frame) — the blade-element
   *  solver returns these (its rates are integrated from the real
   *  moment model, ω̇ = I⁻¹(M − ω × Iω), so they are emergent, not
   *  prescribed). The controller uses them directly when present and
   *  falls back to torqueToRates(torque) for the curve solver. */
  rates?: { pitchRate: number; yawRate: number; rollRate: number };
  /** G-load from wing/rotor lift (lift / weight), floored at 0.2 (v2). */
  gLoad: number;
}

/** The aerodynamic solver contract. One instance per aircraft: stall
 *  hysteresis and spin state live on the instance. */
export interface IAerodynamicsSolver {
  solve(state: AeroState): AeroResult;
  /** Reset stall/spin hysteresis (respawn — a fresh jet is not
   *  stalled and not spinning, whatever the wreck was doing). */
  reset?(): void;
  /** Sync the solver's integrated angular-rate state to the rates the
   *  controller actually applied (after its clamps / biases). Only
   *  the blade-element solver integrates a rate state — the curve
   *  solver prescribes its rates fresh each step, so it has nothing
   *  to sync and omits this. Without the sync, the BET solver keeps
   *  integrating from a rate the body never flew: the rotation clamp
   *  zeroes the pitch rate on the ground while the thrust moment
   *  winds the solver's pitch rate up, and the jet rotates on its
   *  own above Vr; the dying nose-down bias never reaches the
   *  solver's damping model either. The controller calls this every
   *  step with the final (clamped / biased) solver rates. */
  syncRates?(rates: { pitchRate: number; yawRate: number; rollRate: number }): void;
}
