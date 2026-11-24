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
const AIL_DEFLECT_DEG = 30; // max aileron deflection (was 20: the
// 20 deg moment banked only ~2.4 deg in the first 0.2 s, probe-roll
// needs >5 deg; 30 deg is the upper end of a fighter's aileron
// travel and, with the 0.5 roll-inertia factor, banks ~6-7 deg)
const RUD_DEFLECT_DEG = 25; // max rudder deflection
/** Scheduled max elevator / canard deflection (issue 6 calibration).
 *  A fixed 25 deg deflection is far too much at 400 kt (the tail
 *  moment overwhelms the damping and the jet departs to 180 deg AoA)
 *  and far too little at 90 kt (the tail cannot overcome the wing's
 *  nose-down moment and the jet never reaches its stall). Real FCS
 *  schedules stabilator travel with q; here the max deflection is
 *  δ_max = A/V, clamped to 5..30 deg, so the equilibrium pitch rate
 *  is roughly constant across speeds (a rate-command feel, like the
 *  curve solver's 55 deg/s authority) instead of exploding with q. */
const ELEV_SCHED_A = 75; // m/s·rad: δ_max = A/V (default; per-spec
// elevSchedA overrides — F-22 60, MiG-29 110)
const ELEV_MIN_DEG = 5;
const ELEV_MAX_DEG = 30;
/** Explicit rotary damping (issue 6 calibration): the ω×r term alone
 *  is worth only Cmq ≈ −9 (the tail is small), so the elevator
 *  overpowers it and the pitch rate runs away. These are the standard
 *  derivatives (per rad), applied as moments M = C·q·S·ref·(ω·ref/2V).
 *  The explicit Cmq is small (−5) on purpose: a large Cmq forces the
 *  tail to carry a big download to balance the damping moment, and
 *  that download subtracts from the net lift — the max-turn rates
 *  fall short (MiG-29 14.3°/s at Cmq −60 vs 15.0°/s at Cmq −5).
 *  The geometric ω×r term (≈ −9) plus the explicit −5 gives a total
 *  ≈ −14, which bounds the full-stick pitch rate while keeping the
 *  tail download small. The cost is more pitch overshoot in hard
 *  turns (maxAoA 14–16° vs the 12° target) — the loops are stable
 *  (probe-bet green, no departure) but less damped than at Cmq −60. */
const CMQ = -5; // pitch damping, per rad (see below)
const CLP = -2.0; // roll damping, per rad (see below)
const CNR = -0.2; // yaw damping, per rad
/** Stall nose-down moment coefficient. Past the peak-CL AoA the
 *  wing's AC is near the CG (11% MAC static margin, nearly
 *  neutral), so the BET solver has no emergent nose-down
 *  departure — a stalled jet with neutral controls settles into
 *  a stable deep stall and hangs in mid-air instead of dropping
 *  (probe-stall-respawn 2a: hung at y=157, AoA 0, G 1.0). The
 *  curve solver has an explicit stall nose-drop (STALL_NOSE_DROP);
 *  BET gets the equivalent as a RATE bias in the controller
 *  (see FlightPhysicsController — BET_STALL_NOSE_DROP_RATE).
 *  A moment was tried first (q-scaled, then a fixed
 *  0.03·m·g·cbar): it pitches the nose down, the AoA falls,
 *  the wing regains lift, and the jet recovers into a gentle
 *  descent and lands softly instead of crashing. Only a rate
 *  bias — which the pilot's pull can still overpower, so
 *  probe-bet check 4 (F-18 full pull to maxAoA > 35 deg) still
 *  passes — drives the nose down far enough to kill the lift
 *  and dive. Kept for reference; the moment itself is unused. */
const STALL_NOSE_DOWN_K = 0.03;
/** Hands-off 1g auto-trim gain (pass 12, raised pass 13): per radian
 *  of AoA error above the 1g AoA, radians of elevator trim
 *  (nose-down when the AoA is too high). See the trim block in
 *  solve(): the BET's geometric trim AoA (~1-4 deg, set by the
 *  station layout) is speed-blind, so at 780 kt it holds ~4 deg
 *  AoA and makes ~11 g — a hands-off steep dive pulls itself out.
 *  A real fighter's FCS / trim holds 1 g hands-off; this
 *  proportional trim does the same.
 *  Gain 2.0 (pass 12): the elevator moves the trim AoA ~3.4 deg
 *  per degree of deflection, so the closed-loop trim AoA lands
 *  within ~1 deg of the 1g AoA; the deflection is capped at the
 *  scheduled elevator authority, so the trim can never exceed what
 *  the pilot's own stick commands. That ~1 deg was measured (pass
 *  13 dive sweep) to be too loose: at high speed the wing makes
 *  multi-g lift at AoA ~0 (F-22: 2.5 g at AoA -0.7 deg, 350 m/s),
 *  so a 1 deg steady-state error leaves 1.5-3 g of excess lift
 *  that curves a shallow dive into a pull-out over 10-25 s — the
 *  F-22 pulled out of every 30-deg dive and the F-18 pulled out
 *  of a 45-deg dive given altitude (from 10,000 m: level at 54 s,
 *  min AGL 1924 m), which is the live F-18 case (-34,286 fpm at
 *  ~7,000 ft to +8,419 fpm — it dove from ~35,842 ft, so the slow
 *  pull-out had time to complete). The trim gate is NOT the cause
 *  — it reads ON through the whole dive (hands-off, wings level,
 *  unstalled, gear up at altitude; the gear auto-retracts above
 *  100 m AGL and the bank levels within 2 s, so neither gate can
 *  hold it off). The loop is just too soft — but stiffening the
 *  gain alone trades the dive for level flight: K=8/10 held the
 *  dives yet broke the Su-34 hands-off test (pinned at AoA -1
 *  deg, vsi decaying to -18,000 fpm over 60 s), because the P
 *  term targets the WING polar while the tail download makes the
 *  NET lift short — the soft gain's steady-state error was
 *  accidentally compensating for it. So the gain stays at 6.0
 *  (the error lands ~0.2-0.3 deg) and the integral term below
 *  (TRIM_KI) removes the net-lift shortfall instead. */
const TRIM_K = 6.0;
/** Integral-trim gain (pass 13): radians of elevator trim per
 *  g-second of net-lift error. See the integral term in the trim
 *  block — the P term alone cannot hold the net lift, because it
 *  targets the wing polar while the tail download eats part of
 *  the lift. 0.05 moves the elevator ~3 deg in 10 s at a 0.15 g
 *  shortfall: fast enough to hold a dive ballistic, slow enough
 *  not to fight the P term (the pitch plant answers in ~1 s). */
const TRIM_KI = 0.05;
/** Dying nose-down moment coefficient (damage-model §6). Tried
 *  and found ineffective — the tail's moment in the flat spin
 *  overwhelms a 0.05·q·S·cbar moment, so dying is a rate bias
 *  in the controller (BET_DYING_NOSE_DROP_RATE) instead. Kept
 *  for reference; the moment itself is unused. */
const DYING_NOSE_DOWN_K = 0.05;
/** CLP is high on purpose (issue 6): the bank-hold loop drives the
 *  roll rate with a 0.42 s lag at CLP −0.4, so the bank overshoots
 *  80° to past 90° in ~1 s — and past 90° the pitch input stops
 *  raising the AoA (it turns the nose horizontally), the AoA loop
 *  saturates at full pull, and the jet departs. At CLP −2.0 the
 *  roll-rate time constant is ~0.19 s and the equilibrium roll rate
 *  ~57°/s, so the bank approaches 80° without overshooting past 90°.
 *  The geometric ω×r damping already contributes Clp ≈ −1.0, so
 *  the total is ≈ −3.0 — high for a bare airframe, but this is the
 *  fly-by-wire FCS's rate damping, not the bare airframe. */
const ENGINE_Y = 0.55; // twin-engine lateral offset, m
const HYDRO_PITCH = 0.4; // pitch authority with hydraulic failure
const ENGINE_DMG_THRUST = 0.7;
const ROTOR_TRANSLIFT = 0.3;
const ROTOR_DRAG = 0.05;
const SPIN_MOMENT_TAU = 0.5; // s — spin moment drives ω toward the spin rate
/** F-35A lift-fan / nozzle thrust balance (issue 6 calibration): the
 *  nozzle station sits at z = −2.2 m so the hover thrust centroid
 *  (47% fan at +2.5 m, 53% nozzle) is at the CG. The old −0.45·L
 *  (−7.1 m) put the centroid 2.6 m aft and gave a large nose-down
 *  moment in hover that the FCS could not trim. */
const VTOL_NOZZLE_Z = -2.2;
/** F-35A roll-post authority (issue 6 calibration): each post carries
 *  9% of the hover thrust with a full (±100%) differential, so full
 *  roll in hover gives ~2.3 deg of bank in 0.3 s. The old 4.5% /
 *  ±50% gave 0.4 deg — the stick did nothing in hover. */
const ROLL_POST_FRAC = 0.25;
const ROLL_POST_DIFF = 1.0;

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

/** Stabilator / canard polar (issue 6 calibration). The old tail model
 *  was `cl = clamp(4·α, −1, 1) + CL_DELTA·δ`, so full elevator ADDED
 *  0.87 CL on top of the tail's baseline lift — nearly doubling the
 *  tail's max CL — and the tail could never drive the nose up at high
 *  wing AoA because its own baseline lift ate the elevator authority.
 *  An all-moving stabilator rotates the whole surface, so the
 *  deflection adds to the EFFECTIVE angle (α_eff = α_tail + δ) and the
 *  polar is evaluated once at α_eff. The polar is linear (slope
 *  4.0/rad) to ±14°, peaks at 1.0 at ±16°, and falls to 0.4 at ±40°,
 *  so the tail stalls like a wing and cannot hold the nose up forever
 *  past the wing's peak-CL AoA. Canards use the same polar with the
 *  deflection sign reversed (they lift the nose up on a pull). */
export function tailPolar(alphaDeg: number): number {
  const s = alphaDeg < 0 ? -1 : 1;
  const x = Math.abs(alphaDeg);
  if (x <= 14) return s * 4.0 * x * D2R;
  if (x <= 16) return s * (0.977 + (1.0 - 0.977) * ((x - 14) / 2));
  return s * Math.max(0.4, 1.0 - 0.025 * (x - 16));
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
  // Pass 11: the wing stations sum to wingArea, but the lift
  // reference is wingAreaEffective when set (MiG-29: the blended
  // fuselage + LERX generate ~40% of the lift, so the curve solver
  // lifts on 45 m² while BET's wing stations summed to 38 — BET
  // under-lifted the MiG-29 by ~8% at level AoA and ~19% at the
  // peak, and it sank out of a stall test without ever flagging).
  // Scale the wing-station areas so they sum to the lift area.
  const wingAreaScale = (spec.wingAreaEffective ?? S) / S;

  // Wing: 8 per half, chord tapering root → tip, AC slightly aft of
  // the CG for a swept wing. Ailerons on the outer half (i ≥ 4).
  // The wing AC sits at z = −0.03·L (~16.5% MAC static margin).
  // The old −0.02·L (11% MAC) trimmed the jet to ~5 deg AoA at
  // 220 m/s — 2.4 g of lift — so it climbed ~200 m in the 7 s
  // settle of the dying test and the wreck fell from 1360 m
  // instead of 1000 m (probe-stall-respawn 2b, impactT 27–30 s
  // vs the < 25 s bar). Halving the margin shifts the trim to
  // ~2–3 deg AoA, ~1 g, near-level at high speed. The old
  // −0.08·L put it at 43% MAC (Cessna-like), giving a nose-down
  // moment so strong the elevator could not hold 12° AoA at
  // 65 m/s and the jet could not rotate (issue 6 calibration).
  // 5.5% is still positively stable — a fighter-realistic
  // relaxed margin — and the tail still provides the restoring
  // moment.
  for (const side of [-1, 1]) {
    for (let i = 0; i < 8; i++) {
      const t = (i + 0.5) / 8; // 0 root → 1 tip
      const chord = rootChord * (1 - 0.6 * t);
      stations.push({
        kind: 'wing',
        r: new THREE.Vector3(side * (span / 2) * t, -0.3, -0.03 * L),
        area: chord * (span / 16) * wingAreaScale,
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
    // Nozzle at z = VTOL_NOZZLE_Z (−2.2 m): the hover thrust centroid
    // (47% fan at +2.5 m, 53% nozzle) sits at the CG, so hover trims
    // level. The old −0.45·L (−7.1 m) put the centroid 2.6 m aft and
    // gave a large nose-down moment the FCS could not trim
    // (issue 6 calibration).
    stations.push({
      kind: 'engine',
      r: new THREE.Vector3(0, -0.2, VTOL_NOZZLE_Z),
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
  /** Previous step's NET g-load (lift / weight, all stations —
   *  the tailplane download is in it). The integral trim reads it
   *  (one-step lag, like the downwash). Seeded at 1 g. */
  private prevGLoad = 1;
  /** Integral-trim state (pass 13): elevator radians learned from
   *  the net-lift error, so the trim holds the NET lift at the
   *  flight-path value instead of the wing polar's 1 g. Clamped to
   *  the elevator authority; frozen while the trim gates are off;
   *  reset on respawn. */
  private trimInt = 0;
  /** Integrated angular-velocity state (issue 6 calibration). The
   *  solver integrates ω itself and returns it as the rate; the
   *  controller sets the Rapier body to it. Reading the body's rate
   *  each step broke takeoff — the gear contacts zero the body's
   *  rate, so the pitch rate never accumulated and the jet could not
   *  rotate. Seeded once from the body's rate after a reset. */
  private omega = new THREE.Vector3();
  private omegaSeeded = false;

  constructor(spec: AircraftSpec) {
    this.spec = spec;
    this.stations = buildStations(spec);
  }

  /** Reset stall/spin hysteresis, the tailplane-downwash memory, and
   *  the integrated angular velocity (respawn — a fresh jet is not
   *  stalled, not spinning, and not rotating). */
  reset(): void {
    this.stalled = false;
    this.spinModel.reset();
    this.prevCL = 0;
    this.prevGLoad = 1;
    this.trimInt = 0;
    this.omega.set(0, 0, 0);
    this.omegaSeeded = false;
  }

  /** Sync the integrated angular-velocity state to the rates the
   *  controller actually applied (see IAerodynamicsSolver.syncRates).
   *  The controller's rotation clamp and dying bias override the
   *  solver's rates; without this the solver keeps integrating from
   *  a rate the body never flew. */
  syncRates(rates: { pitchRate: number; yawRate: number; rollRate: number }): void {
    this.omega.set(rates.pitchRate, rates.yawRate, rates.rollRate);
    this.omegaSeeded = true;
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
    // Elevator: pitchIn < 0 (pull) → the tailplane's effective angle
    // DECREASES (all-moving stabilator rotates trailing-edge-up →
    // downward tail force → nose up). The tail is aft of the CG, so a
    // downward tail force pitches the nose up (M_x = −r.z·Fy < 0 =
    // nose up). Canard: opposite sign (ahead of the CG, more lift →
    // nose up). The max deflection is SCHEDULED with airspeed
    // (δ_max = A/V, 5..30 deg): a fixed 25 deg is far too much at
    // 400 kt (the tail moment overwhelms the damping and the jet
    // departs to 180 deg AoA) and far too little at 90 kt (the tail
    // cannot overcome the wing's nose-down moment and the jet never
    // reaches its stall). Scheduling makes the equilibrium pitch rate
    // roughly constant across speeds — a rate-command feel, like the
    // curve solver's 55 deg/s authority (issue 6 calibration).
    const hydroPitch = fail.hydroDegraded ? HYDRO_PITCH : 1;
    const elevMax = clamp(
      (this.spec.elevSchedA ?? ELEV_SCHED_A) / Math.max(v, 1),
      ELEV_MIN_DEG * D2R,
      ELEV_MAX_DEG * D2R,
    );
    let elevDeflect = pitchIn * elevMax * hydroPitch * auth.pitch;
    // --- Hands-off 1g auto-trim (pass 12) ---
    // The BET's geometric trim AoA (~1-4 deg, set by the station
    // layout) is speed-blind: at 780 kt it holds ~4 deg AoA and
    // makes ~11 g, so a hands-off steep dive pulls itself out
    // (probe-crash-vfx D2: F-22 levelled off at 1527 m AGL,
    // F-18 at 2.3 g). A real fighter's FCS trims to 1 g hands-off,
    // so when the stick is neutral, the wings are level, the jet
    // is unstalled and not dying and not in VTOL hover, add a
    // proportional elevator trim driving the AoA toward the 1g
    // AoA — the AoA at which CL = m·g/(q·S_lift), found by
    // bisection on the CL curve and clamped to the peak AoA.
    // At 150 kt the 1g AoA is ~11 deg (holds level); at 780 kt
    // it is ~0.4 deg (the dive stays ballistic and hits the
    // ground). Capped at the scheduled elevator authority, so
    // the trim can never exceed what the stick commands, and
    // degraded with the hydraulics. Off whenever the pilot is
    // pulling (turns/stall tests own the elevator), the jet is
    // banked (a turn needs >1 g), stalled (the nose-drop owns
    // that regime), dying, or slow (AoA undefined below 30 m/s).
    // And off ON THE GROUND (heightAboveGround <= 5 m — the
    // gear rest height is 2.2 m): the trim's nose-up drive
    // during the takeoff roll rotated the F-35A before Vr, it
    // lifted off below flying speed, sank back at -12.4 m/s
    // and crashed "hard landing" on the runway at t=2.8 s
    // (probe-handsoff Alpha/f35a). A trim is a flight trim.
    // And off with the GEAR DOWN (pass 12, coordinator): gear down
    // is takeoff/landing configuration, where the pilot (or the
    // landing autopilot) owns the elevator — on a carrier approach
    // at 55 m/s the 1g AoA is ~15-20 deg, so the trim drove the
    // nose up against the ~3 deg approach law and the jet floated
    // 45 m long past the last wire (probe-deck-roll return3). In
    // clean cruise (gear up) the trim still owns the dive regime.
    if (
      Math.abs(pitchIn) < 0.25 &&
      Math.abs(new THREE.Euler().setFromQuaternion(quat, 'YXZ').z) < 20 * D2R &&
      !this.stalled &&
      !state.dying &&
      state.vectorDeg < 70 &&
      v > 30 &&
      (state.heightAboveGround ?? 0) > 5 &&
      !state.gearDown &&
      state.archetype !== 'rotary'
    ) {
      const clReq = (m * G) / Math.max(q * S_lift, 1e-6);
      // The 1g AoA by bisection on the CL curve over [-20, peak]:
      // CL_0 is 0.15, so at high speed the 1g AoA is NEGATIVE
      // (F-22 at 780 kt: ~-1.7 deg) — a [0, peak] search floors
      // aoaReq at 0, where the wing still makes ~4.7 g and the
      // dive pulls out anyway (D2 f22 pre-fix: AoA settled at
      // 0.6 deg, 5.4 deg/s pull-out). At low speed clReq exceeds
      // the peak and the search saturates at peakAoA (the stall
      // gate then owns the regime).
      let lo = -20;
      let hi = curve.peakAoA;
      for (let i = 0; i < 40; i++) {
        const mid = 0.5 * (lo + hi);
        if (clFromCurve(mid, (state.flapsDeg ?? 0) / 30, curve) < clReq) lo = mid;
        else hi = mid;
      }
      const aoaReq = 0.5 * (lo + hi);
      elevDeflect += clamp((aoaDeg - aoaReq) * D2R * TRIM_K, -elevMax, elevMax) * hydroPitch;
      // Integral trim (pass 13): the P term above targets the WING
      // polar's 1 g CL, but the net lift sums every station — the
      // tailplane's download (needed to balance the wing's nose-up
      // moment) makes the net lift at that AoA less than 1 g, so a
      // hands-off jet slowly sinks (Su-34: pinned at AoA -1 deg,
      // vsi decaying to -18,000 fpm over 60 s at 100% throttle).
      // The P loop's steady-state error used to compensate for it
      // by accident (a soft gain leaves the AoA above the 1 g
      // AoA); stiffening the gain alone removes the compensation
      // and the jet sinks (K=8/10 broke the Su-34 hands-off test
      // while K=6 left the F-22 high-alt dive pulling out). So the
      // trim also integrates the NET g-load error against the
      // flight-path lift (cos γ — 1 g level, 0.71 g in a 45-deg
      // dive), driving the steady-state net lift to the target
      // over ~10 s. Same gates as the P term, so it freezes (never
      // winds up) while the pilot pulls, the jet banks, stalls,
      // or sits on its gear. Sign: a lift shortfall (prevGLoad <
      // gTarget) drives the integral negative, which is nose-up
      // elevator (positive elevDeflect is nose-down).
      const cosGamma = Math.min(1, Math.hypot(vAir.x, vAir.z) / Math.max(v, 1e-6));
      const gTarget = Math.max(0, cosGamma);
      this.trimInt = clamp(
        this.trimInt + (this.prevGLoad - gTarget) * dt * TRIM_KI,
        -elevMax,
        elevMax,
      );
      elevDeflect += this.trimInt * hydroPitch;
    }
    const ailDeflect = rollIn * AIL_DEFLECT_DEG * D2R;
    const rudDeflect = -yawIn * RUD_DEFLECT_DEG * D2R;
    // F-22 TVC: the nozzle angle follows the pitch input (±20°).
    // SIGN (issue 6): thrustDirBody is the thrust FORCE direction, so
    // a pull (pitchIn < 0, nose up) needs the thrust to point DOWN —
    // the downward force at the tail pushes the tail down and the
    // nose up. The old `clamp(-pitchIn, ...)` gave +20° on a pull
    // (thrust UP at the tail), a 540 kN·m nose-DOWN moment that
    // pinned the nose down and let the TVC's upward thrust bounce
    // the jet off the runway at 18 kt ("lifted" at 5 m). Now
    // tvcAngle follows pitchIn directly: pull → −20° → thrust down.
    const tvcAngle = (state.tvcDeg ?? 0) * D2R * clamp(pitchIn, -1, 1);

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
    // ω is the solver's OWN integrated angular-velocity state (issue 6
    // calibration). The old code read the Rapier body's rate each step,
    // and the gear contacts zero that rate every step while the jet is
    // on the ground — so the pitch rate never accumulated and the jet
    // could not rotate at all (pitch stayed 0.0 through the whole
    // takeoff roll, and the jet only lifted off at 200+ kt when wing
    // lift alone sufficed, past the runway end). The curve solver is
    // immune because it PRESCRIBES a rate each step. Integrating ω
    // inside the solver makes BET prescribe its rate the same way:
    // the controller sets the body to the solver's rate each substep,
    // so the two stay in step (the contacts can only fight the rate,
    // never erase the solver's memory of it). Seeded once from the
    // body's rate after construction / reset (the respawned body is
    // at rest, so this is 0 in practice).
    if (!this.omegaSeeded) {
      if (state.angVel) this.omega.copy(state.angVel);
      this.omegaSeeded = true;
    }
    const omega = this.omega;
    const AR = (span * span) / S;
    const downwash = (2 * this.prevCL) / (Math.PI * AR); // rad
    const flapFrac = state.flapsDeg / 30;
    // Stall flag = PEAK-CL stall (issue 6), not the section stall.
    // The section stall (~15-18 deg) is where the wing root first
    // separates, but vortex lift keeps CL rising to the peak
    // (~25-35 deg), so the aircraft flies on past it. The old flag
    // fired the STALL warning in turns at 130-150 kt where the real
    // jet is flying fine.
    const stallA = curve.peakAoA + flapFrac * 3;
    if (aoaDeg > stallA) this.stalled = true;
    // LATCH at low speed: below ~20 m/s the AoA is undefined
    // (aoaDeg stays 0 when v <= 5), so a jet that stalls and
    // bleeds off all its speed would clear the flag and the
    // nose-down moment with it, and hang nose-up on thrust
    // (probe-stall-respawn 2a: hung at y=158, ias 7, AoA 0).
    // The flag only clears once the jet has regained flying
    // speed AND the AoA is back below the peak — a stalled jet
    // stays stalled until it recovers.
    else if (v > 20 && aoaDeg < stallA - 3) this.stalled = false;

    const totalMoment = new THREE.Vector3(); // body frame
    const vLocal = new THREE.Vector3();
    const omegaCrossR = new THREE.Vector3();
    let liftSum = 0; // total lift (body-up component), N — the
    // calibrated magnitude (q·S·CL); see the force application
    // below for why the DIRECTION is not the body-up axis.
    // Summed station lift VECTOR, body frame (0, y, z): the
    // station lifts are perpendicular to their local airflow, so
    // their sum is the true lift direction — perpendicular to the
    // airflow, tilted forward of the body-up axis by the AoA.
    const liftVecBody = new THREE.Vector3();

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
        // SIGN (weathercock — issue 6 root cause): β > 0 means the
        // velocity is to the LEFT of the nose (vBody.x > 0, +X is the
        // LEFT wing), so the tail must push the nose LEFT (positive
        // M_y: +ω.y swings the nose toward +X) to align.
        // M_y = r.z·Fx with r.z < 0 (tail), so Fx must be NEGATIVE
        // for β > 0: cSide = −4·β. The old +4·β was positive
        // feedback — it yawed the nose AWAY from the velocity, the
        // sideslip grew to −78°, the yaw rate ran to the 90°/s clamp,
        // the drag spiked, the jet bled 400 → 41 kt and tumbled to
        // AoA ~180° in every hard turn. (The curve solver had the
        // same sign error and was fixed there; BET's fin never was.)
        // The rudder term is unchanged: yawIn > 0 (LEFT rudder — both
        // solvers map +yawIn to nose left) → rudDeflect < 0 → cSide
        // decreases → Fx negative → nose LEFT, which matches the
        // curve solver. Pass 11: the old comment said β > 0 was
        // velocity-right and the nose goes right; the code was right.
        const betaLocal = Math.atan2(vLocal.x, vLocal.z);
        let cSide = clamp(-4.0 * betaLocal, -1.2, 1.2);
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
        // st.side < 0 is the RIGHT wing (-X), so it scales by
        // wingR; the left wing (+X) by wingL. Pass 11: was swapped
        // (it read wingL for the right wing) and only worked because
        // the hit-zone anchors were mirrored the same way.
        dmgScale = 1 - (st.side < 0 ? dmg.wingR : dmg.wingL) * 0.8;
      } else if (st.kind === 'tail') {
        dmgScale = 1 - dmg.tail * 0.7;
      }
      let cl: number;
      if (st.kind === 'wing' || st.kind === 'lex' || st.kind === 'chine') {
        cl = clFromCurve(alpha, flapFrac, curve) * st.clScale;
      } else if (st.kind === 'canard') {
        // All-moving canard (issue 6): the deflection rotates the
        // whole surface, so it adds to the effective angle with the
        // opposite sign to the tail (ahead of the CG — a pull INCREASES
        // the canard's effective AoA and lifts the nose up).
        cl = tailPolar(alpha - elevDeflect * R2D);
      } else if (st.kind === 'tail') {
        // All-moving stabilator (issue 6): the deflection adds to the
        // effective angle, and the polar is evaluated once at α_eff.
        // The old `clamp(4·α, −1, 1) + CL_DELTA·δ` let full elevator
        // ADD 0.87 CL on top of the tail's baseline lift — nearly
        // doubling the tail's max CL — while at high wing AoA the
        // tail's own baseline lift ate the elevator authority, so the
        // jet could not pitch up at 90 kt at all.
        cl = tailPolar(alpha + elevDeflect * R2D);
      } else {
        // Fuselage lifting body.
        cl = clamp(2.0 * alpha * D2R, -0.8, 0.8) * st.clScale;
      }
      if (st.control === 'aileron') {
        // Differential aileron: rollIn > 0 (bank RIGHT) → right
        // wing Cl down, left wing Cl up (the right wing is -X, so
        // st.side < 0 is the right wing). Pass 11: the old comment
        // said "rollIn > 0 (bank left)" and had the wings swapped —
        // that wrong ground truth is why pass 1 concluded the roll
        // was "already correct" while every input path was inverted.
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
      // The vector sum keeps the forward (body-z) component the
      // scalar drops — same ly > 0 guard as liftSum, so a
      // reversed-flow station contributes to neither.
      if (ly > 0) {
        liftVecBody.y += dL * ly;
        liftVecBody.z += dL * lz;
      }
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
    // The pitch coefficient is 0.25 (probe-vtol 3b). The fan share
    // is NOT scaled by the vector angle: the old
    // clamp(vecAngle/(π/2)) factor dragged the share (and the
    // thrust centroid) aft as the vector dropped (0.47 → 0.37 at
    // 70 deg), and the vertical thrust then pitched the nose down
    // faster than the pull could counter — the transition departed
    // nose-down to 84 deg. With the share held at 0.47 the centroid
    // stays at the CG through the transition and the thrust has no
    // pitch moment; the tail + the pitch term control the pitch.
    // (At vector 0 the fan's thrust points forward like the
    // nozzle's, so the split is moment-neutral there too.)
    const hoverBlend = clamp((state.vectorDeg - 45) / 45, 0, 1);
    const fanShare = 0.47 + -pitchIn * 0.25 * hoverBlend;
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
            ? engR
            : engL;
        addThrust(st.r, thrustDirBody, t);
      } else if (st.kind === 'liftfan') {
        addThrust(st.r, thrustDirBody, thrustEff * fanShare);
      } else if (st.kind === 'rollpost') {
        // Vertical thrust at the wingtips, hover only; the roll input
        // splits them differentially (roll control in hover). Each
        // post carries ROLL_POST_FRAC of the hover thrust with a
        // ROLL_POST_DIFF differential — full roll gives ~2.3 deg of
        // bank in 0.3 s. The old 4.5% / ±50% gave 0.4 deg, so the
        // stick did nothing in hover (issue 6 calibration).
        const t =
          thrustEff * ROLL_POST_FRAC * clamp(vecAngle / (Math.PI / 2), 0, 1) *
          (1 + (st.side < 0 ? -1 : 1) * rollIn * ROLL_POST_DIFF * hoverBlend);
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
    // Dying airframe (damage-model §6 step 1, same as the curve
    // solver): the wing can no longer hold 1 g. The curve solver
    // applies liftScale; this one ignored it, so a BET dying jet
    // glided on at full lift. 0.15 (controller) + the dying pitch
    // bias drive the wreck into the stall and the fall.
    lift *= state.liftScale ?? 1;
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
      // LIFT DIRECTION (pass 11): the summed station lift vector,
      // NOT the body-up axis. The stations' lifts are perpendicular
      // to their local airflow, so their sum is the true lift
      // direction — tilted FORWARD of body-up by the AoA. Applying
      // the lift along body-up put a spurious L·sin(AoA) component
      // BACKWARD along the flight path (the body-up axis is sin(AoA)
      // aft of the perpendicular to the airflow): at the sustained-
      // turn AoA (~12°) that is ~20% of the lift — ~190 kN on the
      // F-18, more than the thrust — so no jet could hold its §0
      // sustained turn rate (measured 13-17 deg/s vs the §0 16-21),
      // every hard turn bled 20-40 kt, and a 150 kt turn bled into
      // the stall flag. The magnitude is unchanged (liftSum, the
      // calibrated q·S·CL); only the direction is corrected, so the
      // whole lift now acts perpendicular to the flight path and the
      // only backward force is the drag, as the §0 calibration
      // assumes. Rotary keeps body-up (rotor thrust is along the
      // mast); a degenerate sum (hover, v ≈ 0) falls back to it.
      let liftDir = up;
      if (state.archetype !== 'rotary' && liftVecBody.lengthSq() > 1) {
        liftDir = liftVecBody.clone().normalize().applyQuaternion(quat);
      }
      // Pass 14: the magnitude must be UNSIGNED. liftVecBody is the
      // SIGNED summed station vector (it points belly-ward at
      // negative AoA) and `lift` is the SIGNED scalar (negative at
      // negative AoA, clFromCurve's negative branch floors at -0.8)
      // — scaling the signed direction by the signed scalar
      // double-counts the sign, so at AoA < 0 the lift force was
      // applied UPWARD: a jet commanding full nose-down at 200+ m/s
      // climbed at +10..+150 m/s for minutes while the loop
      // saturated nose-down (probe-ai-live cruise1v1, AoA -15..-26,
      // pitchIn +0.60 for 180 s, y 3.4 -> 7.1 km; the pass-12
      // player's 20 km zoom-climb at AoA -41 is the same bug). The
      // direction already carries the sign, so the magnitude must
      // not. `lift` itself stays signed (gLoad below, the VTOL
      // vector floor, and the HUD all read the true signed value).
      liftVec.addScaledVector(liftDir, Math.abs(lift));
      dragVec.addScaledVector(vAir.clone().normalize(), -drag);
    }
    // Thrust (world frame): the body-frame thrust direction rotated
    // into the world. All thrust (nozzle + fan + roll posts) acts
    // along the vectored direction when active, so the total is
    // thrustEff along thrustDirBody.
    const thrustVecWorld = thrustDirBody.clone().applyQuaternion(quat).multiplyScalar(thrustEff);

    // Unclamped: the lift fraction can go NEGATIVE (AoA < 0, the wing
    // pushes down), and the lift-following vector floor in main.ts
    // (and the probe) reads ac.gload to decide how far the nozzle
    // may come down — it clamps to [0, 1] itself. The old
    // Math.max(0.2, …) floor reported 0.2 g when the wing was
    // actually at −0.5 g, so the floor let the vector down to 49
    // deg while the jet was sinking, the AoA ran away negative,
    // and the transition fell out of the sky (probe-vtol 3b, 96 m
    // lost). With the true value the floor holds the vector near
    // 69 deg until the wing genuinely takes the load. The HUD
    // displays the raw value (negative G reads negative, which is
    // true); the over-G check only fires on large positive G.
    const gLoad = lift / (m * G);
    // Pass 13: remember the net g-load for the integral trim (the
    // next step reads it as prevGLoad, one-step lag like prevCL).
    this.prevGLoad = gLoad;

    // --- Spin (BET §2) as a moment ---
    // The spin is a yaw-rate state; it is applied as a PD moment
    // driving ω toward the spin target rate (τ = 0.5 s), so it
    // converges instead of accumulating. The spin builds while
    // stalled and yawed and decays on NATOPS recovery input.
    // DYING: the spin is suppressed — asymmetric missile damage
    // builds it while stalled, and spin yaw/roll + the dying
    // nose-down bias tumbles the wreck (AoA flipping −40 to
    // −132) instead of diving it, which slows the fall to 27–30 s
    // (probe-stall-respawn 2b). A dying jet does not recover, so
    // there is no spin to fly out of: it should dive straight
    // down. Passing stalled=false and asymWing=0 lets the spin
    // decay, and the moment is not applied at all while dying.
    const spinStep = this.spinModel.step(
      dt,
      state.dying ? false : this.stalled,
      yawIn,
      pitchIn,
      rollIn,
      state.dying ? 0 : dmg.wingR - dmg.wingL,
    );
    const inertia = inertias(m, span, length);
    // Only when the spin is actually developed — otherwise this PD
    // term would be a strong yaw damper (driving ω_y → 0) and would
    // kill the coordinated yaw and the rudder response.
    if (this.spinModel.spin > 0.01 && !state.dying) {
      totalMoment.y +=
        (inertia.iy * (spinStep.yawRate - omega.y)) / SPIN_MOMENT_TAU;
      totalMoment.z +=
        (inertia.iz * (spinStep.rollRate - omega.z)) / SPIN_MOMENT_TAU;
    }

    // --- Stall nose-down departure + dying departure ---
    // These are RATE biases, applied by the controller after the
    // sync (see FlightPhysicsController) — NOT moments. A moment
    // cannot produce a departure here: it pitches the nose down,
    // the AoA falls, the wing regains lift, and the jet recovers
    // into a gentle descent and lands softly instead of crashing
    // (probe-stall-respawn 2a: the fixed 0.03·m·g·cbar moment got
    // the jet to the ground but it landed, G 1.0, not crashed).
    // The curve solver's stall nose-drop is a rate bias
    // (STALL_NOSE_DROP) for the same reason. The dying moment
    // (0.05·q·S·cbar) was likewise ineffective — the tail's
    // moment in the flat spin overwhelms it — so dying is a
    // rate bias too. The constants live in the controller.

    // --- Explicit rotary damping (issue 6 calibration) ---
    // The ω×r term alone is worth only Cmq ≈ −9 (the tail is small),
    // so the elevator overpowers it and the pitch rate runs away to
    // the 55 deg/s clamp and the jet departs to 180 deg AoA. These
    // standard derivatives bound the equilibrium rate. CMQ is kept
    // small (−5) — a large Cmq forces a big tail download that eats
    // the net lift and costs turn rate (see the constant's comment).
    // The terms vanish at rest (q = 0).
    const cbar = S / span;
    const vDamp = Math.max(v, 20);
    totalMoment.x += CMQ * q * S * cbar * (omega.x * cbar / (2 * vDamp));
    totalMoment.y += CNR * q * S * span * (omega.y * span / (2 * vDamp));
    totalMoment.z += CLP * q * S * span * (omega.z * span / (2 * vDamp));

    // Transition pitch damper (BET promotion, probe-vtol 3b): a
    // hard rate cap alone lets the pitch sit AT the cap (20
    // deg/s) for seconds and still depart — the fan-share pitch
    // moment is near a bifurcation (0.25 departs nose-down,
    // 0.40 departs nose-up, 0.32/0.36 depart nose-down after a
    // brief rotation), so no single coefficient is stable. A
    // rate-proportional damper (moment = −K·Ix·ωx, K = 1.5/s,
    // a ~0.7 s time constant) resists the rate smoothly instead
    // of clamping it, so the nose cannot dwell at a high rate.
    // It is NOT q-scaled — at low speed q ≈ 0 and the aero
    // damping vanishes, which is exactly when it is needed.
    // Active only while any vector is up and the speed is below
    // 40 m/s (the transition zone); the pilot can still rotate
    // to 10 deg AoA, just not run away.
    if (hoverBlend > 0 && v < 40) {
      totalMoment.x += -1.5 * inertia.ix * omega.x;
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

    // Hover auto-level (issue 6 — BET promotion): the curve solver
    // has this (issue 2) but BET did not, so a BET hover had no
    // stable reference and drifted off level. Same PD as the curve
    // solver — in a deep hover (vector >= 70 deg, slow) the lift-fan
    // FCS holds the jet level, capped so the pilot can overpower it.
    // Euler YXZ x > 0 = nose down, z > 0 = right bank; the correction
    // is the opposite sign on both axes. Switches off above 25 m/s,
    // so the transition to forward flight is the pilot's.
    // PITCH YIELDS TO THE PILOT (probe-vtol 3b): the pitch
    // correction is scaled by (1 − |pitchIn|), so a pilot who is
    // pulling to rotate to 10 deg AoA is not fought by the
    // leveler — the old unscaled PD held the pitch at 0 deg
    // while the speed was under 25 m/s, then switched off and
    // the pent-up full pull over-rotated the jet to −88 deg.
    // The roll correction is unscaled (the pilot is not rolling
    // in the transition; hands-off hover still levels the bank).
    if (hoverBlend > 0.5 && v < 25) {
      const euler = new THREE.Euler().setFromQuaternion(quat, 'YXZ');
      const bank = euler.z;
      const pitch = euler.x;
      newOmega.z += clamp(-bank * 0.8, -0.3, 0.3) * hoverBlend;
      newOmega.x +=
        clamp(-pitch * 0.8, -0.3, 0.3) * hoverBlend * (1 - Math.abs(pitchIn));
    }

    // Transition pitch-rate cap (BET promotion, probe-vtol 3b):
    // in the vector 45–90 deg zone the lift-fan thrust (forward
    // of the CG, mostly vertical) pitches the nose up hard, and
    // at low q there is almost no aerodynamic pitch damping to
    // stop it. This caps the pitch rate at ±20 deg/s while any
    // vector is up (hoverBlend > 0) and the speed is below 40 m/s
    // — a RATE cap only, no leveling. Above 40 m/s the wing and
    // tail have authority and the cap lifts.
    if (hoverBlend > 0 && v < 40) {
      newOmega.x = clamp(newOmega.x, -0.35, 0.35);
    }

    // Store the integrated rate (issue 6): the solver's own ω state
    // is what the next step integrates from — NOT the Rapier body's
    // rate, which the gear contacts zero while the jet is on the
    // ground. The stored rate is the clamped, FCS-corrected rate the
    // controller actually sets on the body.
    this.omega.copy(newOmega);

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
