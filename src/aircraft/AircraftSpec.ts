// AircraftSpec — per-aircraft constants for Pandora Planes (Vite port).
// Values are the v2 HTML template's SPECS
// (pandora-template.html ~473–495), sanity-checked against real figures
// 16 Nov 2026, then corrected to COMBAT MASSES 17 Nov 2026 per
// Docs/flight-behaviour-per-model.md §7/§8 (the sim had mixed empty and
// combat masses — F-22 flew at T/W 1.61, F-15E at 1.84, while the F-18
// flew at 0.99). Su-34 thrustMax was a hard bug: 166 kN is ~63% of the
// real AL-31FM1 afterburner total (264 kN) and below even the dry total.
// Stall speeds and rotation gates are re-derived at combat mass; hover
// for the F-35A at combat mass is ~94% throttle, which is the honest
// F-35B number — do not tune it down.
//
// `mass` is the SIM flying mass (the v2 operating mass — the number the
// lift equation must beat). `fuelMass` is the fuel load carried inside
// that mass (F-22: 8,200 kg per the task) and is used only to scale fuel
// burn / endurance; it is NOT added to `mass`.

export type AircraftArchetype = 'fixed-wing' | 'vtol' | 'rotary';

/** GLB-backed model keys. 'heli' is the procedural UH-60 (no GLB module
 *  exists in Drive) — it is in this union so the rotary spec can name a
 *  modelKey; attachModel() builds it procedurally instead of loading. */
export type ModelKey =
  | 'fa18'
  | 'f35a'
  | 'f22'
  | 'f15e'
  | 'mig29'
  | 'su34'
  | 'ford'
  | 'heli';

/** Per-model lift curve (BET report §2 + per-model supplement §1–§6).
 *  Piecewise-linear through three anchors, all in degrees / CL:
 *    (0°, cl0) → (linearEndAoA, linearEndCL) → (peakAoA, peak)
 *  then a linear post-stall fall at `postSlope` per degree, floored at
 *  `floor`. The section stall is at linearEndAoA; the vortex-lift
 *  region (linearEndAoA → peakAoA) is what keeps the Hornet / F-22
 *  flying far past the section stall, and the F-15E's steep postSlope
 *  (0.06/deg) is its sharp stall break — no single shared curve can
 *  reproduce both, so each model carries its own. `cdSep` is the
 *  separation-drag coefficient: CD_sep = cdSep·max(0,(α−20)/15)². */
export interface ClCurve {
  /** AoA (deg) where the linear region ends (wing-section stall). */
  linearEndAoA: number;
  /** CL at the end of the linear region. */
  linearEndCL: number;
  /** Peak CL. */
  peak: number;
  /** AoA (deg) of the peak CL. */
  peakAoA: number;
  /** CL floor at very high AoA. */
  floor: number;
  /** Post-stall CL fall, per degree past the peak. */
  postSlope: number;
  /** Separation-drag coefficient (see above). */
  cdSep: number;
}

/** Airspeed roll-authority cap (MiG-29 supplement §5): the current
 *  q-scaling gives MORE roll authority at more speed, but the Fulcrum's
 *  weak aileron actuators give LESS at high speed / low altitude and
 *  near the stall. Breakpoints are [airspeed m/s, max roll rate deg/s],
 *  linearly interpolated, clamped at the ends. Absent = no cap (the
 *  spec.rollDeg is the only limit). */
export type RollCapSchedule = Array<[number, number]>;

/** Rotary-wing parameters (collective / cyclic / autorotation). Only
 *  used when archetype is 'rotary'. */
export interface RotaryParams {
  /** Rotor diameter, m (== span for the heli). */
  rotorDiameter: number;
  /** Max collective lift multiplier at full throttle. */
  maxCollective: number;
  /** Cyclic authority multiplier (roll/pitch response). */
  cyclicAuthority: number;
  /** Fraction of max thrust kept as lift in autorotation
   *  (idle throttle + 20 m/s keeps ~35% lift in v2). */
  autorotationLift: number;
}

/** Included-gear station volume (aircraft frame, after normalize):
 *  triangles whose centroid is inside [x0, x1] x [z0, z1] and below
 *  capY belong to that gear. capY is the fuselage / wing underside
 *  at the station minus a 0.05 m safety margin, so no airframe
 *  triangle is ever cut into the gear. */
export interface IncludedGearStation {
  slot: 'gear_front' | 'gear_L' | 'gear_R';
  x0: number;
  x1: number;
  z0: number;
  z1: number;
  capY: number;
}

export interface AircraftSpec {
  /** Record key ('fa18', 'f35a', ...). */
  key: string;
  /** Full display name. */
  name: string;
  /** Short HUD name. */
  short: string;
  archetype: AircraftArchetype;
  /** Sim flying mass, kg (v2 operating mass). */
  mass: number;
  /** Fuel load carried inside `mass`, kg. */
  fuelMass: number;
  /** Wing area, m^2 (rotor-disc reference area for the heli). */
  wingArea: number;
  /** Wingspan, m (rotor diameter for the heli). */
  span: number;
  /** Fuselage length, m (collider + inertia model). */
  length: number;
  /** Max thrust, N. */
  thrustMax: number;
  /** Stall speed, m/s (0 for the heli — it has no wing stall). At combat
   *  mass (per-model supplement §7). */
  stallSpeed: number;
  /** Induced-drag factor k = 1/(pi*e*AR), per model (supplement §7).
   *  The curve solver uses this instead of a shared constant. */
  inducedK: number;
  /** Oswald efficiency factor e (supplement §7). */
  e: number;
  /** Rotation speed, kt, at combat mass (supplement §7; main report §3).
   *  The HUD shows a ROTATE cue at this speed. 0 for the heli. */
  rotateSpeedKt: number;
  /** Thrust-vectoring nozzle deflection, deg (F-22 TVC ±20 deg pitch
   *  only). Absent = no vectoring. */
  tvcDeg?: number;
  /** BET elevator-scheduling constant A (m/s·rad): the max stabilator
   *  deflection is δ_max = A/V. Per model — the value that holds the
   *  12° AoA in a max-performance turn without the tail downforce
   *  eating the lift: the F-22 (big tail + TVC) wants 60, the MiG-29
   *  (small tail) wants 90, the rest 75. Absent = 75. Calibrated
   *  against probe-bet (issue 6). */
  elevSchedA?: number;
  /** Effective lifting area, m^2 (MiG-29: 45 — the blended fuselage +
   *  LERX generate ~40% of total lift, so lift uses 45 m^2 while drag
   *  uses wingArea 38 m^2). Absent = wingArea. */
  wingAreaEffective?: number;
  /** Per-model lift curve (BET report §2 + supplement §1–§6). */
  clCurve: ClCurve;
  /** Airspeed roll-authority cap (MiG-29 only — supplement §5). */
  rollCapSchedule?: RollCapSchedule;
  /** Structural G limit. Over-G damage starts at maxG + 1.5 (v2). */
  maxG: number;
  /** Tailhook fitted (carrier traps). */
  tailhook: boolean;
  /** VTOL-capable (lift-fan thrust vectoring). */
  vtol: boolean;
  /** Max thrust-vector angle, deg (0–90 for VTOL). */
  maxVectorDeg: number;
  /** Mesh tint (procedural fallback colour). */
  tint: number;
  /** Model module key (see src/assets/models/*.ts). */
  modelKey: ModelKey;
  /** Turn-rate multiplier (v2 spec.turn). */
  turn: number;
  /** Max roll rate, deg/s (v2 spec.roll). */
  rollDeg: number;
  /** Pitch multiplier (v2 spec.pitch). */
  pitch: number;
  /** Carrier-capable (may launch / trap on the Ford). */
  carrier: boolean;
  /** Gun cyclic rate, rpm (M61 6000, GAU-22 3300, GSh-30-1 1800). */
  gunRpm?: number;
  /** Can carry bombs. */
  bombs?: boolean;
  /** Belly alignment for normalizeAircraft: align the fuselage
   *  belly at rawY (raw GLB frame, metres) to targetY in the
   *  aircraft frame, instead of the bbox min to -0.8 m. Set only
   *  when the GLB's bbox min is NOT the belly — the F-18's bbox min
   *  is its included gear wheels, 1.36 m below the belly, so the
   *  default alignment put the belly at +0.56 m and the wheels at
   *  -0.8 m, 1.4 m short of the physics gear feet (-2.2 m). */
  bellyAlign?: { rawY: number; targetY: number };
  /** Included landing gear, separated from the airframe at load time
   *  (F-18 only — the other player GLBs were verified 2026-11-20 to
   *  have no gear: F-22/F-15E carry only a flat "landingOff" belly
   *  fairing, the F-35A's lowest point is its belly, so they keep
   *  procedural gear). When present, the rig binds these instead of
   *  building procedural legs. */
  includedGear?: IncludedGearStation[];
  /** Rotary parameters (heli only). */
  rotary?: RotaryParams;
}

/** All flyable specs, keyed by spec.key. */
export const SPECS: Record<string, AircraftSpec> = {
  fa18: {
    key: 'fa18',
    name: 'F/A-18F Super Hornet',
    short: 'F-18',
    archetype: 'fixed-wing',
    mass: 20242,
    fuelMass: 6354,
    wingArea: 46.5,
    span: 13.62,
    length: 18.79,
    thrustMax: 196000,
    stallSpeed: 66,
    inducedK: 0.1,
    e: 0.8,
    // Pass 11: Vr 140 kt — the old 126 kt was BELOW the 128 kt
    // stall speed (CL needed at Vr 1.66 > CLmax 1.6), which is why
    // a deck liftoff at ~130 kt dropped at -2114 fpm: it lifted
    // off below a flyable speed.
    rotateSpeedKt: 140,
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.35,
      peak: 1.6,
      peakAoA: 35,
      floor: 0.8,
      postSlope: 0.05,
      cdSep: 0.35,
    },
    maxG: 9,
    tailhook: true,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x8a97a3,
    modelKey: 'fa18',
    // Included landing gear (Nicholas's repeated request, 2026-11-20
    // — the pass-11 "no included gear in any GLB" claim was WRONG:
    // it measured the NORMALIZED model, where normalizeAircraft had
    // already aligned the gear wheel bottoms to -0.8 m, so of course
    // nothing was below -0.8. Measured on the RAW GLB with
    // MeshoptDecoder (diag-glb, 2026-11-20): the "ground" airframe
    // mesh carries its own gear — nose twin wheels at (x +-0.29,
    // z +3.57), mains at (x +-1.55, z -2.26), wheel bottoms at raw
    // y -0.01 (mains) / 0.00 (nose), 1.36 m below the fuselage belly
    // (raw y 1.35, centre strip |x|<0.5, z -1..1). bellyAlign puts
    // the belly at -0.84 m so the wheel bottoms land exactly on the
    // physics gear feet (-2.2 m); the stations below are the gear
    // volumes in the aircraft frame after that shift (nose fuselage
    // underside -0.95 m, mains wing underside -1.20 m, each minus a
    // 0.05 m safety margin — measured by raycast, diag-fa18-caps).
    bellyAlign: { rawY: 1.35, targetY: -0.84 },
    includedGear: [
      { slot: 'gear_front', x0: -0.8, x1: 0.8, z0: 2.8, z1: 4.4, capY: -1.0 },
      { slot: 'gear_L', x0: -2.2, x1: -0.9, z0: -3.1, z1: -1.6, capY: -1.25 },
      { slot: 'gear_R', x0: 0.9, x1: 2.2, z0: -3.1, z1: -1.6, capY: -1.25 },
    ],
    turn: 1.0,
    rollDeg: 150,
    pitch: 1.0,
    carrier: true,
    gunRpm: 6000,
    bombs: true,
  },
  f35a: {
    key: 'f35a',
    name: 'F-35A VTOL',
    short: 'F-35A',
    archetype: 'vtol',
    mass: 18270,
    fuelMass: 8278,
    wingArea: 42.7,
    span: 10.7,
    length: 15.69,
    thrustMax: 191000,
    stallSpeed: 68,
    inducedK: 0.149,
    e: 0.8,
    rotateSpeedKt: 150,
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.35,
      peak: 1.5,
      peakAoA: 30,
      floor: 0.9,
      postSlope: 0.02,
      cdSep: 0.3,
    },
    maxG: 9,
    tailhook: true,
    vtol: true,
    maxVectorDeg: 90,
    tint: 0x6b7a86,
    modelKey: 'f35a',
    turn: 0.9,
    rollDeg: 160,
    pitch: 1.05,
    carrier: true,
    gunRpm: 3300,
    bombs: false,
  },
  f22: {
    key: 'f22',
    name: 'F-22 Raptor',
    short: 'F-22',
    archetype: 'fixed-wing',
    mass: 24883,
    fuelMass: 8200,
    wingArea: 78.0,
    span: 13.56,
    length: 19.0,
    thrustMax: 312000,
    // Pass 11: clean Vs ~117 kt at combat weight (best estimates
    // 108-130 kt; the exact figure is classified), so CLmax 1.40
    // (was 1.75, uncorroborated) and stallSpeed 60 m/s (was 54,
    // 105 kt — ~20 kt too low). Vr 140 kt unchanged.
    stallSpeed: 60,
    inducedK: 0.159,
    e: 0.85,
    rotateSpeedKt: 140,
    tvcDeg: 20,
    elevSchedA: 60, // big tail + TVC: less scheduling needed (issue 6)
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.2,
      peak: 1.4,
      peakAoA: 35,
      floor: 0.85,
      postSlope: 0.015,
      cdSep: 0.3,
    },
    maxG: 9.5,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x7d8a99,
    modelKey: 'f22',
    turn: 1.05,
    rollDeg: 200,
    pitch: 1.15,
    carrier: false,
    gunRpm: 6000,
    bombs: false,
  },
  f15e: {
    key: 'f15e',
    name: 'F-15E Strike Eagle',
    short: 'F-15E',
    archetype: 'fixed-wing',
    mass: 20400,
    fuelMass: 6103,
    wingArea: 56.5,
    span: 13.05,
    length: 19.41,
    thrustMax: 258000,
    // Pass 11: real CLmax ≈ 1.1 (no leading-edge flaps), so
    // Vs ~141 kt — the old CL 1.36 predicted Vs 127 kt, not the
    // spec's own 120 kt. Thrust 258 kN is the F100-PW-229
    // (the -220 is 211 kN total). Vr 150 kt unchanged.
    stallSpeed: 72,
    inducedK: 0.132,
    e: 0.8,
    rotateSpeedKt: 150,
    clCurve: {
      linearEndAoA: 14,
      linearEndCL: 1.05,
      peak: 1.1,
      peakAoA: 25,
      floor: 0.7,
      postSlope: 0.06,
      cdSep: 0.4,
    },
    maxG: 9,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x9aa0a3,
    modelKey: 'f15e',
    turn: 1.0,
    rollDeg: 180,
    pitch: 0.95,
    carrier: false,
    gunRpm: 6000,
    bombs: true,
  },
  mig29: {
    key: 'mig29',
    name: 'MiG-29 Fulcrum',
    short: 'MiG-29',
    archetype: 'fixed-wing',
    mass: 15300,
    fuelMass: 4300,
    wingArea: 38.0,
    wingAreaEffective: 45,
    span: 11.36,
    length: 17.32,
    thrustMax: 163000,
    // Pass 11: CL peak 1.35 on the 45 m² EFFECTIVE (lifting-body)
    // area — the old 1.58 was quoted on the 38 m² wing area, so it
    // predicted Vs 124 kt while the spec said 115 kt. 1.35 on
    // 45 m² ≡ 1.60 on 38 m², Vs ~124 kt. Vr 130 kt unchanged.
    stallSpeed: 64,
    inducedK: 0.11,
    e: 0.85,
    rotateSpeedKt: 130,
    elevSchedA: 110, // small tail: more scheduling needed (issue 6)
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.15,
      peak: 1.35,
      peakAoA: 30,
      floor: 0.9,
      postSlope: 0.02,
      cdSep: 0.3,
    },
    // MiG-29 roll authority is speed-scheduled (supplement §5): 160 deg/s
    // max, but ~20 deg/s just above the stall (~145 kt), 105 deg/s at
    // Mach 0.4, <60 deg/s at high speed / low altitude, 40 deg/s
    // supersonic. The q-scaling alone gets the top end backwards.
    rollCapSchedule: [
      [71, 20],
      [137, 105],
      [200, 90],
      [340, 40],
    ],
    maxG: 9,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x7a7f72,
    modelKey: 'mig29',
    turn: 1.1,
    rollDeg: 155,
    pitch: 1.0,
    carrier: false,
    gunRpm: 1800,
    bombs: false,
  },
  su34: {
    key: 'su34',
    name: 'Su-34 Fullback',
    short: 'Su-34',
    archetype: 'fixed-wing',
    // Pass 11: mass 30,000 kg is the roster's COMBAT mass
    // (like the F-22's 24,883 kg vs its 38,000 kg MTOW) — the real
    // normal takeoff weight is 39,000 kg (MTOW 45,100 kg), at
    // which the same CLmax gives Vs ~157 kt. Kept at 30 t by
    // convention; the docs carry the 39 t figure.
    mass: 30000,
    fuelMass: 12100,
    wingArea: 62.0,
    span: 14.7,
    length: 23.3,
    thrustMax: 264000,
    stallSpeed: 71,
    inducedK: 0.114,
    e: 0.8,
    rotateSpeedKt: 140,
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.35,
      peak: 1.53,
      peakAoA: 30,
      floor: 0.8,
      postSlope: 0.04,
      cdSep: 0.35,
    },
    maxG: 7,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x5f6b62,
    modelKey: 'su34',
    turn: 0.85,
    rollDeg: 120,
    pitch: 0.85,
    carrier: false,
    gunRpm: 1800,
    bombs: true,
  },
  heli: {
    key: 'heli',
    name: 'UH-60 (traffic)',
    short: 'UH-60',
    archetype: 'rotary',
    mass: 8000,
    fuelMass: 1360,
    wingArea: 10.0,
    span: 16.36,
    length: 19.8,
    thrustMax: 110000,
    stallSpeed: 0,
    inducedK: 0.1,
    e: 0.8,
    rotateSpeedKt: 0,
    clCurve: {
      linearEndAoA: 15,
      linearEndCL: 1.35,
      peak: 1.5,
      peakAoA: 15,
      floor: 0,
      postSlope: 0.045,
      cdSep: 0,
    },
    maxG: 3,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x4a5a3a,
    modelKey: 'heli',
    turn: 0.8,
    rollDeg: 90,
    pitch: 0.8,
    carrier: false,
    rotary: {
      rotorDiameter: 16.36,
      maxCollective: 1.3,
      cyclicAuthority: 0.8,
      autorotationLift: 0.35,
    },
  },
};

/** Nose convention fix per model, applied by normalizeAircraft in
 *  FlightPhysicsController (v2 getModel): F-18 / F-35A nose +Z (0),
 *  F-22 / F-15E / Su-34 nose +X (-PI/2), MiG-29 nose -Z (PI). */
export const NOSE_ROT_Y: Record<ModelKey, number> = {
  fa18: 0,
  f35a: 0,
  f22: -Math.PI / 2,
  f15e: -Math.PI / 2,
  mig29: Math.PI,
  su34: -Math.PI / 2,
  ford: 0,
  heli: 0,
};
