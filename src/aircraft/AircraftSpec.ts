// AircraftSpec — per-aircraft constants for Pandora Planes (Vite port).
// Values are the v2 HTML template's SPECS
// (pandora-template.html ~473–495), sanity-checked against real figures
// 16 Nov 2026. Changes vs v1.1, carried over verbatim:
//   F-35A 18200 -> 14500 kg (real empty 13,290 kg; at 14,500 kg T/W is
//   1.32 and a vector-90 hover needs ~74% throttle, as it should).
//   F-22 22000 -> 19700 kg (the task's figure; real empty 19,700 kg).
//   F-15E 22000 -> 14300 kg (real empty 14,379 kg).
//   Stalls: F-35A 60, F-22 65, F-15E 62 m/s (F-18 62, MiG-29 60, Su-34 65).
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
  /** Stall speed, m/s (0 for the heli — it has no wing stall). */
  stallSpeed: number;
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
    stallSpeed: 62,
    maxG: 9,
    tailhook: true,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x8a97a3,
    modelKey: 'fa18',
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
    mass: 14500,
    fuelMass: 8278,
    wingArea: 42.7,
    span: 10.7,
    length: 15.69,
    thrustMax: 191000,
    stallSpeed: 60,
    maxG: 9,
    tailhook: true,
    vtol: true,
    maxVectorDeg: 90,
    tint: 0x6b7a86,
    modelKey: 'f35a',
    turn: 1.05,
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
    mass: 19700,
    fuelMass: 8200,
    wingArea: 78.0,
    span: 13.56,
    length: 19.0,
    thrustMax: 312000,
    stallSpeed: 65,
    maxG: 9.5,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x7d8a99,
    modelKey: 'f22',
    turn: 1.15,
    rollDeg: 180,
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
    mass: 14300,
    fuelMass: 6103,
    wingArea: 56.5,
    span: 13.05,
    length: 19.41,
    thrustMax: 258000,
    stallSpeed: 62,
    maxG: 9,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x9aa0a3,
    modelKey: 'f15e',
    turn: 0.95,
    rollDeg: 140,
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
    mass: 15000,
    fuelMass: 4300,
    wingArea: 38.0,
    span: 11.36,
    length: 17.32,
    thrustMax: 162000,
    stallSpeed: 60,
    maxG: 9,
    tailhook: false,
    vtol: false,
    maxVectorDeg: 0,
    tint: 0x7a7f72,
    modelKey: 'mig29',
    turn: 1.0,
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
    mass: 22000,
    fuelMass: 12100,
    wingArea: 62.0,
    span: 14.7,
    length: 23.3,
    thrustMax: 166000,
    stallSpeed: 65,
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
    bombs: false,
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
