// FlightPhysicsController — Aircraft: one rigid body + one solver.
// Ported from the v2 HTML template (pandora-template.html ~889–1204
// Aircraft.step / groundCheck / damage / crash, ~749–890 model loading,
// ~1204–1330 carrier deck interaction).
//
// OWNERSHIP LINE (solver vs Rapier):
//   * CurveAerodynamicsSolver owns FORCES (lift / drag / thrust / control
//     torque) — a pure function of the aero state.
//   * This controller owns the per-step BOOKKEEPING the template did in
//     Aircraft.step: throttle spool (0.55/s), fuel burn, G-overstress,
//     ground-roll friction, nose-wheel steering, arresting wires, deck
//     velocity transfer, spawn protection, damage zones.
//   * Rapier owns INTEGRATION and COLLISIONS. Each fixed step the
//     controller computes the new velocity from the solver forces
//     (semi-implicit Euler, exactly like the template: vel += acc·dt)
//     and hands it to the body with setLinvel / setAngvel
//     (NOT applyImpulse — the template's rates are kinematic targets,
//     and impulses would fight the body's damping/inertia). Rapier then
//     integrates position and resolves contacts against the terrain
//     heightfield / carrier deck. The body runs with gravityScale 0:
//     gravity is already in the solver's force sum.
//   * Ground contact therefore comes from Rapier: the integrator feeds
//     collision events into notifyContact(), and `grounded` is set when
//     a contact was reported this step AND vertical speed is ~0. The
//     terrainHeightAt callback is used ONLY as a fallback contact
//     detector (and for the terrain-penetration safety net) — the
//     controller NEVER pins pos.y to the ground (no analytic ground
//     pinning); Rapier resolves penetration.
//
// TEMPLATE COMPATIBILITY (main.ts drives the player template-style, so
// the Aircraft also exposes the v2 template's mutable surface):
//   * pitchIn / rollIn / yawIn, thrTarget, flaps, airbrake, vector,
//     brake, gear, hook, gunCd, flareCd are public mutable fields.
//     The 3-argument step(dt, wind, turbAcc) overload reads them.
//   * failEngine / failEngineSide / failFire / failHydro / failGearJam /
//     failFuel are accessor pairs over `failures`.
//   * trapped / lastHitBy / vtolCounted are readable (lastHitBy is a
//     minimal { isPlayer } record — WeaponSystem passes only a boolean,
//     so the shooter object itself is not retained).
//   * resetAt(x, y, z, hdgDeg) is the template-named alias of respawn().
//   The canonical API is still step(dt, controls, wind,
//   terrainHeightAt, turbAcc?) with FlightControls; the template surface
//   exists so the integrator compiles while it migrates to it.
//
// CALL ORDER (integrator / main.ts): world.step() -> for each aircraft:
//   notifyContact() from the collision-event dispatch, then
//   aircraft.step(dt, controls, wind, terrainHeightAt). The controller
//   reads the body AFTER world.step, so pos/vel/quat are fresh.
// RAPIER must be initialised (await RAPIER.init()) before the world
// passed in here is created. Aircraft start DISABLED at the origin;
// respawn(runway) enables and places them.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import {
  controlPitch,
  controlRoll,
  resolveThrottleTarget,
  type FlightControls,
  type IAerodynamicsSolver,
  ZERO_DAMAGE,
  type DamageZones,
} from './IAerodynamicsSolver';
import {
  CurveAerodynamicsSolver,
  torqueToRates,
} from './CurveAerodynamicsSolver';
import { BladeElementSolver } from './BladeElementSolver';
import {
  NOSE_ROT_Y,
  type AircraftSpec,
  type ModelKey,
} from '../aircraft/AircraftSpec';
import { AircraftRigBindings } from '../aircraft/AircraftRigBindings';
import { DamageableEntity } from '../combat/DamageableEntity';

const D2R = Math.PI / 180;
const G = 9.81;

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

/** v2 constants (template Aircraft.step / groundCheck / damage). */
const SPOOL_RATE = 0.55; // throttle units per second
const GEAR_HEIGHT = 2.2; // gear-down ride height above the surface
const BELLY_HEIGHT = 1.2; // gear-up ride height
const HARD_LANDING_VS = 9; // m/s vertical speed that breaks the jet
const HARD_LANDING_BANK = 0.45; // rad of bank that breaks the jet
const GROUND_FRICTION = 0.3; // m/s^2 rolling friction (research §6:
// real rolling resistance is ~0.02-0.03 g; 1.2 ate ~12% of takeoff accel)
const ROTATION_RATE_MAX = 55 * D2R; // rad/s nose-up while the gear is on
// a surface. Was 12 deg/s (BET report §3: rotation needs ~10-15
// deg/s, not the 55 deg/s max authority — the full rate whips the
// nose up and tail-strikes). But 12 deg/s could not overcome the
// SHIELD deck contact resistance (F-18 stuck at AoA 2 deg, rolled
// the deck at 214 kt and never lifted off), and 20 deg/s only
// reached AoA 3.5 deg. The curve solver rotates at its full 55
// deg/s authority and powers through the contact, so the BET cap
// matches it — the jet rotates fast, the lift builds, and it is
// off the deck before a tail strike can develop.
/** BET stall nose-drop rate, rad/s nose-down (probe-stall-respawn
 *  2a). The BET solver has no emergent nose-down departure past
 *  the peak-CL AoA (the wing AC sits near the CG), so a stalled
 *  jet with neutral controls hangs in a deep stall and then
 *  lands softly instead of crashing. A moment cannot fix it —
 *  it pitches the nose down, the AoA falls, the wing regains
 *  lift, and the jet recovers into a gentle descent. A rate
 *  bias (the curve solver's STALL_NOSE_DROP is the same shape)
 *  drives the nose down far enough to kill the lift and dive.
 *  It is ADDED to the solver's rate (not a floor), so a full
 *  pull can still overpower it — probe-bet check 4 (F-18 full
 *  pull to maxAoA > 35 deg) still passes. Applied AFTER the
 *  sync so it never accumulates in the solver's integrated ω,
 *  and only when airborne (onSurface would pitch the jet into
 *  the ground) and only for BET (the curve solver has its own
 *  stall nose-drop; check via syncRates, which only BET
 *  implements). */
const BET_STALL_NOSE_DROP_RATE = 0.3; // rad/s nose-down while stalled
/** VTOL hover altitude-hold gain (pass 6, item 8): throttle units
 *  per m/s of vertical-speed error. 0.05 -> a 2 m/s sink commands
 *  +0.10 throttle (~1 m/s^2 of correction on the F-35A), firm
 *  enough to hold altitude without fighting the 0.55/s spool. */
/** VTOL hover integral gain (pass 9, item 8): the P loop alone leaves
 *  a steady-state sink (~20 fpm live, 335 ft -> 321 ft over 40 s) —
 *  holding a constant vertical speed needs throttle slightly above
 *  hoverThr, which a pure P loop only produces while an error
 *  persists. The integral term accumulates the VS error and closes
 *  that gap. Clamped to +/-0.05 throttle (the windup limit) and
 *  reset whenever the hold is inactive, so it cannot carry a stale
 *  offset into forward flight. 0.01/s per (m/s) of error: a 0.1 m/s
 *  sink adds 0.001/s, reaching the ~0.005 needed in ~5 s. */
const HOVER_VS_KP = 0.05;
const HOVER_VS_KI = 0.01;
const HOVER_VS_INT_MAX = 0.05;
/** BET dying nose-drop rate, rad/s nose-down (probe-stall-respawn
 *  2b). The shared 0.15 rad/s dying bias (before the sync) is
 *  the dive driver; this extra is 0 — it is kept as a tuning
 *  knob. 0.35 was tried and made it WORSE (29.6 s): the 0.5
 *  rad/s total makes the AoA oscillate (−47, −96, −69, −107…)
 *  instead of diving cleanly, and the oscillation's drag slows
 *  the fall. The tumble is the SPIN (suppressed while dying,
 *  see the spin block in the solver) plus the pitch overshoot.
 *  The real fix for 2b is the trim (wing AC −0.01·L, see the
 *  wing station block): the wreck was falling from 1360 m, not
 *  1000 m, because the jet climbed ~200 m in the 7 s settle.
 *  BET only, airborne only. The curve solver is untouched. */
const BET_DYING_NOSE_DROP_RATE = 0; // rad/s nose-down while dying
const BRAKE_DECEL = 6; // m/s^2 with wheel brakes
const STEER_RATE = 0.9; // nose-wheel steering, rad/s at full stick
const TRAP_MAX_SPEED = 72; // m/s — faster snaps the wire
const TRAP_BASE_DECEL = 14; // m/s^2 initial wire decel
const TRAP_RAMP = 30; // m/s^2 extra decel per second of trap
const SPAWN_PROTECTION_S = 5; // v1.1: player spawn protection
const OVER_G_MARGIN = 1.5; // damage starts at maxG + 1.5
const OVER_G_DPS = 8; // hp per second over the limit
const OVER_G_MIN_SPEED = 60; // m/s
const WIRE_LOCAL_Z_MIN = -105; // arresting-wire trigger zone (carrier-local)
const WIRE_LOCAL_Z_MAX = -45;
const WIRE_LOCAL_X = 26;
const FUEL_BURN_BASE = 0.008; // per second, template units (fuel 0..100)
const FUEL_BURN_THR = 0.11; // per second at full throttle
const FUEL_LEAK_EXTRA = 0.25; // per second with a fuel leak
const WING_LOSS_DMG = 0.75; // wing damage that detaches the wing
const WING_DETACH_FATAL = 0.4; // wing damage that detaches on crash
const SMOKE_HP = 60; // smoke below this hp
const FIRE_HP = 35; // fire below this hp
const LOAD_TIMEOUT_MS = 9000; // v2 loadGLB timeout

// ---- Damage model (War Thunder report §3, §7) ----
// Fire: a fuel-tank fire ramps to full intensity in ~12 s (game pace;
// WT's 40-60 s scaled down), an engine fire ramps faster (~4 s) but
// stays localised. Fuel fire 2.0 hp/s at full intensity, engine fire
// 1.2 hp/s. A fuel fire spreads to one adjacent zone per ~8 s; a wing
// fire burning long enough destroys the spar (wing separation).
const FUEL_FIRE_RAMP_S = 12; // seconds to full intensity
const ENGINE_FIRE_RAMP_S = 4;
const FUEL_FIRE_DPS = 2.0; // hp/s at full intensity
const ENGINE_FIRE_DPS = 1.2;
const FIRE_SPREAD_S = 8; // fuel fire spreads to an adjacent zone
// Extinguish: a fuel fire's self-extinguish chance rises with airspeed
// (slipstream) — 2%/s below 150 m/s IAS, 8%/s above 300 m/s. An
// engine fire has a 15%/s chance to go out when that engine's throttle
// is cut to 0 (fuel starvation). Diving fast with the throttle cut is
// the correct player response.
const FUEL_FIRE_EXT_LO = 0.02; // per second, IAS < 150 m/s
const FUEL_FIRE_EXT_HI = 0.08; // per second, IAS > 300 m/s
const ENGINE_FIRE_EXT_THR0 = 0.15; // per second, throttle cut to 0

/** Component HP per zone (damage-model §7, tuned to gun 12/hit,
 *  missile 40, bomb ≤60). `destroyed` = fuselage 0, OR both engines 0,
 *  OR tail 0 (controls gone), OR total 0 — the WT severe-damage rule.
 *  Wing 0 = wing detaches → dying spin. Fuel 0 = fuel fire starts.
 *  F-35A is single-engine (one `engine` zone, not engineL/engineR);
 *  losing it is a severe-damage kill (all engines destroyed). */
function zoneHpFor(specKey: string): Record<string, number> {
  switch (specKey) {
    case 'f35a':
      return {
        fuselage: 40, wingL: 25, wingR: 25, tail: 20, fin: 15,
        engine: 25, fuel: 20,
      };
    case 'f22':
      return {
        fuselage: 45, wingL: 30, wingR: 30, tail: 22, fin: 15,
        engineL: 22, engineR: 22, fuel: 22,
      };
    case 'f15e':
      return {
        fuselage: 42, wingL: 28, wingR: 28, tail: 20, fin: 15,
        engineL: 22, engineR: 22, fuel: 22,
      };
    case 'mig29':
      return {
        fuselage: 38, wingL: 25, wingR: 25, tail: 18, fin: 15,
        engineL: 20, engineR: 20, fuel: 18,
      };
    case 'su34':
      return {
        fuselage: 45, wingL: 30, wingR: 30, tail: 22, fin: 15,
        engineL: 22, engineR: 22, fuel: 25,
      };
    case 'fa18':
    default:
      return {
        fuselage: 40, wingL: 25, wingR: 25, tail: 20, fin: 15,
        engineL: 20, engineR: 20, fuel: 20,
      };
  }
}

/** Damage zone names accepted by applyDamage(). 'wing' / 'engine' are
 *  the v2 template zones (WeaponSystem passes these); the per-side
 *  variants let callers hit one side only. 'fuel' is the fuel tank
 *  (damage-model §3 — a hit can start a fuel fire); 'fin' is the
 *  vertical fin (damage-model §7). */
export type DamageZone =
  | 'wing'
  | 'wingL'
  | 'wingR'
  | 'engine'
  | 'engineL'
  | 'engineR'
  | 'tail'
  | 'fin'
  | 'fuel'
  | 'fuselage';

/** Spawn point: runway centre, elevation and heading (deg). */
export interface RunwaySpawn {
  x: number;
  z: number;
  elev: number;
  hdg: number;
}

/** Failure flags set by the G7 failure system (main.ts). */
export interface AircraftFailures {
  engineOut?: boolean;
  engineSide?: -1 | 0 | 1;
  hydroDegraded?: boolean;
  fuelLeak?: boolean;
  fire?: boolean;
  gearJam?: boolean;
}

/** Minimal carrier-deck surface the controller needs. The carrier
 *  worker owns the carrier itself; this is only the interaction surface
 *  (template carrierDeckY / carrierRelative / carrier.vel / yawRate). */
export interface DeckProvider {
  alive: boolean;
  /** Deck height at world (x, z), or -Infinity when not over the deck. */
  deckHeightAt(x: number, z: number): number;
  /** Deck velocity, m/s (world frame) — the LINEAR velocity of the
   *  deck centre. */
  velocity(): THREE.Vector3;
  /** Velocity of the DECK POINT at world (x, z), m/s — the linear
   *  velocity PLUS the yaw rotation (yawRate x r, where r is the
   *  offset from the deck centre). This is the one to use for anything
   *  at a specific deck point: a jet 120 m aft of the centre sees up
   *  to 14 m/s of sideways deck motion when the carrier turns at its
   *  0.12 rad/s yaw rate, and the linear velocity alone leaves it
   *  behind — the v3 live-test deck fall-off (issue 3). */
  velocityAt(x: number, z: number): THREE.Vector3;
  /** Deck yaw rate, rad/s. */
  yawRate(): number;
  /** Yaw rate of the deck at world (x, z), rad/s — the carrier whose
   *  deck is under that point (issue 5: with several carriers the
   *  single yawRate() cannot say which one the jet is on). */
  yawRateAt(x: number, z: number): number;
  /** Carrier-local coords, or null when the provider cannot convert. */
  local(x: number, z: number): { x: number; z: number } | null;
}

/** Hooks the integrator wires to visuals / audio / score. The
 *  controller never imports CrashVFXManager, the HUD or sfx — main.ts
 *  connects these. */
export interface AircraftEvents {
  /** Aircraft went down (reason: 'shot down' | 'hard landing' |
   *  'terrain' | 'engine fire' | ...). Main plays the explosion. */
  onCrash?: (ac: Aircraft, reason: string) => void;
  /** Aircraft entered the dying fall (damage-model §6 step 1) — shot
   *  down but still falling, not yet dead. Main starts the spectate
   *  camera for the player and shows the "SHOT DOWN" banner. */
  onDying?: (ac: Aircraft, reason: string) => void;
  /** A wing/tail segment was detached by fatal damage. Main hands the
   *  mesh to CrashVFXManager.spawnDebris() as a tumbling Rapier body.
   *  `segment` is null when the model had no such node (box fallback). */
  onDetachSegment?: (
    ac: Aircraft,
    segment: THREE.Object3D | null,
    pos: THREE.Vector3,
    vel: THREE.Vector3,
  ) => void;
  /** Arresting wire caught (hook down, in the wires, under 72 m/s). */
  onTrap?: (ac: Aircraft) => void;
  /** Wire snapped (in the wires at 72 m/s or faster). */
  onWireSnap?: (ac: Aircraft) => void;
  /** Flares were fired (bookkeeping already done by fireFlares()).
   *  Main wires the VISUALS here (6 sprites) — do NOT call
   *  WeaponSystem.fireFlares() from this hook, it would decrement the
   *  flare count a second time. */
  onFlares?: (ac: Aircraft) => void;
}

export interface AircraftOptions {
  scene: THREE.Scene;
  world: RAPIER.World;
  spec: AircraftSpec;
  isPlayer: boolean;
  modelKey: ModelKey;
  /** Pre-built rig (dependency injection for tests). Normally omitted:
   *  attachModel() builds one from the loaded model. */
  rig?: AircraftRigBindings;
  /** Carrier deck surface (optional; no deck interaction without it). */
  deck?: DeckProvider;
  /** Use the blade-element solver (BET §5) instead of the curve
   *  solver. Default false: the curve solver carries all of the BET
   *  §0–§4 corrections (per-model CL curves, coordinated yaw, spin
   *  state, adverse yaw, thrust lapse, qAuth floor, MiG roll cap,
   *  F-22 TVC) and is the verified default. The BET solver is
   *  calibrated (probe-bet 16/16 green since the issue-6 pass) but
   *  NOT yet the default: promoting it regresses 20 checks in 4
   *  integration probes (probe-roll 12, probe-stall-respawn 4,
   *  probe-vtol 3, probe-respawn-carrier 1) — its roll response in
   *  the first 0.2 s, VTOL transition, stall-down and deck-ride
   *  behaviour differ from the curve solver's, and those probes
   *  encode the game's required behaviour. Promote only after those
   *  regressions are fixed. Enable per-aircraft for testing. */
  useBet?: boolean;
  events?: AircraftEvents;
}

/** GLB data-URI loader per model key. The modules are generated by
 *  scripts/embed_models.mjs and export a named `*_GLB` const. */
const MODEL_LOADERS: Record<
  Exclude<ModelKey, 'heli'>,
  () => Promise<string>
> = {
  fa18: async () => (await import('../assets/models/fa18')).FA18_GLB,
  f35a: async () => (await import('../assets/models/f35a')).F35A_GLB,
  f22: async () => (await import('../assets/models/f22')).F22_GLB,
  f15e: async () => (await import('../assets/models/f15e')).F15E_GLB,
  mig29: async () => (await import('../assets/models/mig29')).MIG29_GLB,
  su34: async () => (await import('../assets/models/su34')).SU34_GLB,
  ford: async () => (await import('../assets/models/ford')).FORD_GLB,
};

// One GLTFLoader for all aircraft; the MeshoptDecoder registration is
// the v1.1 fix (the F-18 GLB requires EXT_meshopt_compression and v1
// never registered a decoder, so it always fell back to the blocky jet).
const gltfLoader = new GLTFLoader();
gltfLoader.setMeshoptDecoder(MeshoptDecoder);

function countTris(root: THREE.Object3D): number {
  let t = 0;
  root.traverse((o) => {
    if ((o as THREE.Mesh).isMesh) {
      const g = (o as THREE.Mesh).geometry;
      if (g.index) t += g.index.count / 3;
      else if (g.attributes.position) t += g.attributes.position.count / 3;
    }
  });
  return Math.round(t);
}

/** Centre the loaded scene on its bbox centre and scale it to
 *  targetLen along Z (v2 normalizeAircraft verbatim). */
function normalizeAircraft(
  obj: THREE.Object3D,
  targetLen: number,
  noseRotY: number,
): { group: THREE.Group; scale: number; size: THREE.Vector3 } {
  const inner = new THREE.Group();
  inner.add(obj);
  inner.rotation.y = noseRotY;
  const wrap = new THREE.Group();
  wrap.add(inner);
  const box = new THREE.Box3().setFromObject(wrap);
  const size = new THREE.Vector3();
  box.getSize(size);
  const ctr = new THREE.Vector3();
  box.getCenter(ctr);
  inner.position.sub(ctr);
  const s = targetLen / Math.max(size.z, 0.01);
  wrap.scale.setScalar(s);
  return { group: wrap, scale: s, size: size.clone().multiplyScalar(s) };
}

function loadGLB(uri: string): Promise<THREE.Object3D> {
  return new Promise((res, rej) => {
    let done = false;
    const to = setTimeout(() => {
      if (!done) {
        done = true;
        rej(new Error('timeout'));
      }
    }, LOAD_TIMEOUT_MS);
    gltfLoader.load(
      uri,
      (g) => {
        if (!done) {
          done = true;
          clearTimeout(to);
          res(g.scene);
        }
      },
      undefined,
      (e) => {
        if (!done) {
          done = true;
          clearTimeout(to);
          rej(e);
        }
      },
    );
  });
}

/** Procedural low-poly jet (fallback if a GLB fails; v2 proceduralJet). */
function proceduralJet(tint: number, twin: boolean): THREE.Group {
  const g = new THREE.Group();
  const mat = new THREE.MeshLambertMaterial({ color: tint });
  const dark = new THREE.MeshLambertMaterial({ color: 0x2a2f36 });
  const fuse = new THREE.Mesh(new THREE.ConeGeometry(1.1, 7, 6), mat);
  fuse.rotation.x = Math.PI / 2;
  fuse.position.z = 4.5;
  g.add(fuse);
  const body = new THREE.Mesh(new THREE.CylinderGeometry(1.0, 1.3, 9, 6), mat);
  body.rotation.x = Math.PI / 2;
  body.position.z = -1.5;
  g.add(body);
  const wing = new THREE.Mesh(new THREE.BoxGeometry(13.5, 0.35, 4.2), mat);
  wing.position.set(0, -0.2, -1.5);
  g.add(wing);
  const tail = new THREE.Mesh(new THREE.BoxGeometry(5.5, 0.3, 2.6), mat);
  tail.position.set(0, 0.1, -5.6);
  g.add(tail);
  for (const s of [-1, 1]) {
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.3, 3.4, 2.6), mat);
    fin.position.set(s * 1.3, 1.6, -5.4);
    fin.rotation.z = -s * 0.35;
    g.add(fin);
    const noz = new THREE.Mesh(new THREE.CylinderGeometry(0.75, 0.9, 2.2, 6), dark);
    noz.rotation.x = Math.PI / 2;
    noz.position.set(s * (twin ? 1.1 : 0.7), 0, -6.2);
    g.add(noz);
  }
  const can = new THREE.Mesh(
    new THREE.SphereGeometry(1.0, 8, 6),
    new THREE.MeshLambertMaterial({ color: 0x1a2a3a }),
  );
  can.scale.set(0.9, 0.7, 1.8);
  can.position.set(0, 0.9, 3.2);
  g.add(can);
  return g;
}

/** Procedural low-poly helicopter (v2 proceduralHeli). Nodes are named
 *  so AircraftRigBindings finds the rotors. No heli GLB exists in Drive. */
function proceduralHeli(): THREE.Group {
  const g = new THREE.Group();
  const mat = new THREE.MeshLambertMaterial({ color: 0x4a5a3a });
  const dark = new THREE.MeshLambertMaterial({ color: 0x2a2f36 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(3.4, 3.0, 8.5), mat);
  body.position.z = 1;
  g.add(body);
  const nose = new THREE.Mesh(new THREE.SphereGeometry(1.7, 8, 6), mat);
  nose.scale.set(1, 0.9, 1.2);
  nose.position.set(0, -0.2, 5.2);
  g.add(nose);
  const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 1.1, 8, 6), mat);
  boom.rotation.x = Math.PI / 2;
  boom.position.set(0, 0.6, -6.5);
  g.add(boom);
  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.3, 2.6, 2.2), mat);
  fin.position.set(0, 1.8, -10.2);
  g.add(fin);
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.35, 1.6, 6), dark);
  mast.position.y = 2.2;
  g.add(mast);
  const rotor = new THREE.Group();
  rotor.name = 'main_rotor';
  for (let i = 0; i < 4; i++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(16.4, 0.12, 0.7), dark);
    b.rotation.y = (i * Math.PI) / 4;
    rotor.add(b);
  }
  rotor.position.y = 3.0;
  g.add(rotor);
  const tailRotor = new THREE.Group();
  tailRotor.name = 'tail_rotor';
  for (let i = 0; i < 2; i++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.12, 0.5), dark);
    b.rotation.x = (i * Math.PI) / 2;
    tailRotor.add(b);
  }
  tailRotor.position.set(1.0, 1.2, -10.0);
  tailRotor.rotation.z = Math.PI / 2;
  g.add(tailRotor);
  for (const s of [-1, 1]) {
    const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 6, 5), dark);
    skid.rotation.x = Math.PI / 2;
    skid.position.set(s * 1.5, -1.9, 1);
    g.add(skid);
  }
  return g;
}

export class Aircraft {
  readonly spec: AircraftSpec;
  readonly isPlayer: boolean;
  readonly modelKey: ModelKey;
  readonly key: string;
  /** Visual group (synced from the Rapier body every step). */
  readonly group: THREE.Group;
  /** Rapier dynamic body. Rapier owns position; read it via pos/vel/quat. */
  readonly body: RAPIER.RigidBody;
  /** All colliders on the body (fuselage + 3 gear posts). main.ts
   *  registers every handle in colliderOwners so contacts on the gear
   *  posts dispatch into notifyContact() like fuselage contacts. */
  private readonly colliders: RAPIER.Collider[] = [];
  /** Loaded model (null until attachModel resolves). */
  model: THREE.Group | null = null;
  modelReady = false;
  modelStatus: 'pending' | 'ok' | 'failed' = 'pending';
  modelError?: string;
  modelTris = 0;
  rig: AircraftRigBindings | null = null;

  /** Position / velocity / attitude, synced from the body at the start
   *  of every step() (after the integrator's world.step) and in
   *  respawn(). Read these — never the body directly. */
  readonly pos = new THREE.Vector3();
  readonly vel = new THREE.Vector3();
  readonly quat = new THREE.Quaternion();

  /** Template-style stick inputs (main.ts writes these, then calls the
   *  3-argument step overload). pitchIn NEGATIVE pulls up (v2). */
  pitchIn = 0;
  rollIn = 0;
  yawIn = 0;
  /** Throttle TARGET 0..1 (spooled into `throttle` at 0.55/s).
   *  A getter/setter (pass 6, item 8): the setter records the value
   *  as the PILOT's command (`pilotThr`), so the hover altitude-hold
   *  can read the pilot's throttle without reading back its own
   *  auto-throttle output (which fed back and oscillated the
   *  throttle 1.0 -> 0.71 -> 1.0 on a ~7 s period). The hold writes
   *  the backing field `_thrTarget` directly, bypassing the setter.
   *  External code (main.ts, probes) sets `thrTarget` as before. */
  private _thrTarget = 0;
  get thrTarget(): number {
    return this._thrTarget;
  }
  set thrTarget(v: number) {
    this._thrTarget = clamp(v, 0, 1);
    this.pilotThr = this._thrTarget;
  }
  /** Last throttle commanded by the pilot (slider / keys / an
   *  external `thrTarget` set) — the hover hold's climb/descent
   *  command. Never written by the auto-throttle itself. */
  pilotThr = 0;
  /** VTOL hover integral accumulator (pass 9, item 8): the accumulated
   *  vertical-speed error (m/s·s) for the hover altitude-hold's
   *  integral term. Reset whenever the hold is inactive. */
  hoverVsInt = 0;
  /** VTOL pad-landing credit flag (main.ts sets / clears it). */
  vtolCounted = false;
  /** Terrain height callback used by the 3-argument step overload.
   *  Main MUST assign the real ground height here (e.g.
   *  `ac.terrainHeightAt = (x, z) => worldGen.groundHeight(x, z)`) —
   *  the default is flat 0, which is only correct on the runway. */
  terrainHeightAt: (x: number, z: number) => number = () => 0;

  alive = true;
  grounded = false;
  onDeck = false;
  /** True while the gear is in physical contact with a surface (set
   *  in stepInternal — NOT the `grounded` flag, whose 0.85×stall gate
   *  flips far too early). The rotation clamp and the ROTATE HUD cue
   *  read this. */
  onSurface = false;
  aoaDeg = 0;
  ias = 0;
  gload = 1;
  vsi = 0;
  stalled = false;
  /** Spin amount 0..1 (BET §2 — the solver's spin state). */
  spin = 0;
  /** Spin command-arrow direction for the HUD: −1 = stick left,
   *  +1 = stick right, 0 = no arrow. */
  spinArrow: -1 | 0 | 1 = 0;
  /** Spin direction: +1 = yawing right, −1 = yawing left. The AI's
   *  spin recovery feeds rudder opposite this. */
  spinDir: 1 | -1 = 1;
  /** Sideslip β, deg (BET §1 — the solver's weathercock input). */
  betaDeg = 0;
  /** Fuel 0..1 (fraction). */
  fuel = 1;
  /** Spooled throttle 0..1 (HUD / solver input). */
  throttle = 0;
  flaps = 0;
  airbrake = 0;
  vectorDeg = 0;
  gear = true;
  hook = false;
  brake = false;
  /** Sim seconds; spawn protection ends at spawnProtectedUntil. */
  spawnProtectedUntil = 0;
  lastHitByIsPlayer = false;
  /** Overall hit points 0..100 (v2 template hp). */
  hp = 100;
  /** Per-zone structural damage 0..1. */
  readonly damageZones: DamageZones = { ...ZERO_DAMAGE };
  /** Single source of truth for per-zone HP (damage-model §6 step 2).
   *  Built from the §7 table for this spec; applyDamage routes through
   *  it, and its severe-damage rule (fuselage / both engines / tail /
   *  total) is what starts the dying fall. */
  readonly damageEntity: DamageableEntity;
  /** Fire state machine (damage-model §3). `fireKind` null = no fire.
   *  `fireIntensity` 0..1 ramps up (fuel 12 s, engine 4 s) and drives
   *  the burn rate and the visuals. `fireZone` is the zone burning. */
  fireKind: 'fuel' | 'engine' | null = null;
  fireIntensity = 0;
  fireZone: DamageZone | null = null;
  /** `dying` (damage-model §6 step 1): the jet is shot down and
   *  falling, but not yet dead — alive stays true for physics, the
   *  solver steps degraded (throttle 0, controls zeroed), and the
   *  ground impact becomes finishCrash(). Untargetable while dying. */
  dying = false;
  dyingReason: string | null = null;
  /** True once finishCrash() has run (ground impact after a dying
   *  fall) — handleDeaths uses it to scale the explosion by the
   *  remaining fuel and scatter debris. */
  crashedFromDying = false;
  lockT = 0;
  lockTarget: Aircraft | null = null;
  /** Gun cooldown, s (WeaponSystem sets it; GUN_COOLDOWN 0.075). */
  gunCd = 0;
  /** Missiles / flares remaining. */
  msl = 6;
  flares = 30;
  flareCd = 0;
  bombs: number;
  failures: AircraftFailures = {};
  lastCrashReason: string | null = null;
  // Pass 8 (item 1): the vertical speed and bank at the moment of the
  // crash, so the ?debug=1 spawn trace can report the actual numbers
  // behind a "WENT DOWN — terrain" instead of just the reason string.
  lastCrashVs = 0;
  lastCrashBankDeg = 0;

  private readonly world: RAPIER.World;
  private readonly deck?: DeckProvider;
  private readonly events: AircraftEvents;
  /** Aerodynamics solver (BET §5): the blade-element solver is the
   *  default — its rates are emergent from the station moments (§5
   *  moment model). It is calibrated (probe-bet 16/16 green since
   *  the issue-6 pass) and pass 4 fixed the 20 integration checks
   *  that blocked promotion (roll authority, stall/dying, VTOL
   *  transition, deck ride), so it was promoted to default. The
   *  curve solver (opt-out via AircraftOptions.useBet = false) is
   *  the verified fallback — it carries all of the BET §0–§4
   *  corrections (per-model CL curves, coordinated yaw, spin state,
   *  adverse yaw, thrust lapse, qAuth floor, MiG roll cap, F-22
   *  TVC). The rotary archetype always uses the curve solver (the
   *  report keeps the scalar rotor model until BET lands for helis).
   *  Both implement IAerodynamicsSolver, so the controller treats
   *  them identically. */
  private readonly solver: IAerodynamicsSolver;
  /** True when the solver is the blade-element model (pass 5: the
   *  AoA limiter below is BET-only — the curve solver's pitch rate
   *  is prescribed from the input and its stall nose-drop caps it). */
  private readonly isBet: boolean;
  /** Body-frame angular velocity (rad/s), read from the Rapier body
   *  in syncFromBody and fed to the BET solver as AeroState.angVel. */
  private readonly angVelBody = new THREE.Vector3();
  private simTime = 0;
  private _trapped = false;
  private trapTimer = 0;
  /** Deck point velocity from the previous substep (issue 3): the
   *  ground block subtracts this — not the current deck velocity —
   *  when forming the deck-relative velocity, because newVel was
   *  built from the velocity the jet was carrying. Invalid while
   *  airborne / after a respawn, so the first touchdown uses the
   *  current deck velocity instead. */
  private _prevDeckVel = new THREE.Vector3();
  private _prevDeckValid = false;
  private wingDetached = false;
  private tailDetached = false;
  private fireSpreadTimer = 0;
  private lastHitByRecord: { isPlayer: boolean } | null = null;
  private pendingContact: { impactVy: number; isDeck: boolean } | null = null;
  private static readonly modelCache = new Map<
    ModelKey,
    { group: THREE.Group; tris: number; failed: boolean; error?: string }
  >();

  constructor(opts: AircraftOptions) {
    this.spec = opts.spec;
    this.isPlayer = opts.isPlayer;
    this.modelKey = opts.modelKey;
    this.key = opts.spec.key;
    this.world = opts.world;
    this.deck = opts.deck;
    this.events = opts.events ?? {};
    this.solver =
      opts.useBet !== false && opts.spec.archetype !== 'rotary'
        ? new BladeElementSolver(opts.spec)
        : new CurveAerodynamicsSolver();
    this.isBet =
      opts.useBet !== false && opts.spec.archetype !== 'rotary';
    this.rig = opts.rig ?? null;
    this.bombs = opts.spec.bombs ? 4 : 0;
    this.damageEntity = new DamageableEntity(zoneHpFor(opts.spec.key));

    this.group = new THREE.Group();
    opts.scene.add(this.group);

    // Four colliders, not one big box — the takeoff fix (issues 1 + 2).
    //
    // The old single cuboid spanned the whole fuselage (hy 1.6, centre 0),
    // so its bottom corners sat in the runway and Rapier's contact solver
    // cancelled any pitch rotation: the jet could not rotate at all, and
    // its 0.6 friction (0.75 combined with the 0.9 terrain, Rapier's
    // average combine rule) ate ~149 kN of the 196 kN thrust — the
    // observed ~1.3 m/s^2 instead of ~7-9 m/s^2.
    //
    // Now the fuselage rides CLEAR of the ground (belly -0.8 m, i.e. 1.4 m
    // above the surface when parked) and the jet stands on three gear
    // posts whose feet touch the surface (bottom -2.2 m = gear height).
    // Rotation pivots on the main gear: at 12 deg nose-up the tail belly
    // drops 0.30*length*sin(12 deg) ~ 1.17 m for the F-18 and still clears
    // by ~0.2 m, so pitch authority is no longer fought by the ground.
    // Friction is 0 on every collider with the Min combine rule — the
    // controller owns all ground physics (rolling resistance 0.3 m/s^2,
    // brakes, steering, traps, deck-velocity transfer) and Rapier
    // Coulomb friction must not eat thrust a second time.
    //
    // Density is set so the Rapier mass equals spec.mass — the solver
    // divides forces by the same mass, so the two agree.
    const s = opts.spec;
    const hx = Math.max(s.span * 0.28, 1.2);
    const hzFuse = s.length * 0.45;
    const vFuse = 8 * hx * 1.4 * hzFuse;
    // Gear "skis": each post is a 3 m long skid (hz 1.5) rather than a
    // 1 m stub. The posts are physics-only (invisible — the visual gear
    // is part of the GLB), and a long skid spreads the ground contact
    // over several heightfield triangles instead of balancing the whole
    // jet on three 0.7 x 1.0 m pads. That matters because Rapier's
    // heightfield contact solver produces a spurious up-and-back
    // impulse when a small pad straddles a cell boundary / diagonal:
    // measured in the node driver, one world.step changed the F-35A's
    // Rapier velocity from (4.89, -0.16) to (1.30, +1.12) m/s at
    // t=1.35 s / 4.9 m/s on flat ground (flaps 30), launching it into
    // a tumble (AoA -129 deg); the F-15E did the same at 21 m/s and
    // read as a 40 m/s lift-off. A 3 m skid keeps face contacts
    // dominant so a single edge contact cannot dominate the manifold.
    const vMain = 2 * (0.7 * 0.8 * 3.0); // two main skids
    const vNose = 0.6 * 0.8 * 3.0; // one nose skid
    const density = s.mass / (vFuse + vMain + vNose);
    this.body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0, 0, 0)
        .setLinearDamping(0)
        .setAngularDamping(0.2)
        .setCcdEnabled(true)
        .setGravityScale(0) // gravity is inside the solver's force sum
        .setEnabled(false), // respawn() enables + places the aircraft
    );
    const minRule = RAPIER.CoefficientCombineRule.Min;
    const mk = (
      hx2: number,
      hy2: number,
      hz2: number,
      x: number,
      y: number,
      z: number,
    ): void => {
      this.colliders.push(
        this.world.createCollider(
          RAPIER.ColliderDesc.cuboid(hx2, hy2, hz2)
            .setTranslation(x, y, z)
            .setDensity(density)
            .setFriction(0)
            .setFrictionCombineRule(minRule)
            .setRestitution(0)
            // Collision events feed the integrator's notifyContact() dispatch.
            .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
          this.body,
        ),
      );
    };
    // Fuselage: belly -0.8 m, top +2.0 m (hy 1.4, centre +0.6).
    mk(hx, 1.4, hzFuse, 0, 0.6, 0);
    // Main gear: two skids at z = -0.15*length, feet at -2.2 m.
    mk(0.35, 0.4, 1.5, -1.6, -1.8, -s.length * 0.15);
    mk(0.35, 0.4, 1.5, 1.6, -1.8, -s.length * 0.15);
    // Nose gear: z = +0.35*length, feet at -2.2 m.
    mk(0.3, 0.4, 1.5, 0, -1.8, s.length * 0.35);
  }

  /** Handles of every collider on this aircraft's body, for the
   *  integrator's colliderOwners registry (fuselage + gear posts). */
  colliderHandles(): number[] {
    return this.colliders.map((c) => c.handle);
  }

  /** Remove the Rapier body and the visual group (round restart). */
  dispose(): void {
    try {
      this.world.removeRigidBody(this.body);
    } catch {
      // already removed
    }
    this.group.removeFromParent();
  }

  /** Sim clock, seconds since construction. */
  get time(): number {
    return this.simTime;
  }

  /** ROTATE cue (BET §3): the HUD shows "ROTATE" when the gear is on
   *  a surface and the speed has reached the per-spec rotation speed.
   *  Gated on onSurface (physical contact), not `grounded` — the
   *  flag's 0.85×stall gate flips far too early (F-22: 46 m/s vs its
   *  72 m/s Vr). */
  get rotateCue(): boolean {
    return (
      this.alive &&
      this.onSurface &&
      this.spec.rotateSpeedKt > 0 &&
      this.ias >= this.spec.rotateSpeedKt * 0.514444
    );
  }

  /** v1.1 spawn protection, seconds REMAINING (template spawnProt). */
  get spawnProt(): number {
    return Math.max(0, this.spawnProtectedUntil - this.simTime);
  }

  /** v1.1 spawn protection: 5 s after (player) respawn. */
  isSpawnProtected(): boolean {
    return this.simTime < this.spawnProtectedUntil;
  }

  /** AI controller driving this aircraft, if any (template ac.ai).
   *  Set by the AI controller's constructor. crash() marks it dead. */
  ai: { state: string } | null = null;

  /** HUD alias: AoA in degrees (same value as aoaDeg). */
  get aoa(): number {
    return this.aoaDeg;
  }

  /** HUD alias: thrust-vector angle in degrees (same as vectorDeg).
   *  Settable — the HUD slider and main.ts assign to it. */
  get vector(): number {
    return this.vectorDeg;
  }

  set vector(v: number) {
    this.vectorDeg = clamp(v, 0, this.spec.maxVectorDeg);
  }

  get isTrapped(): boolean {
    return this._trapped;
  }

  /** Template field: true while an arresting wire holds the jet. */
  get trapped(): boolean {
    return this._trapped;
  }

  /** Template kill-credit record (main.ts: lastHitBy?.isPlayer).
   *  Minimal { isPlayer } — WeaponSystem passes only a boolean, so the
   *  shooter object itself is not retained. */
  get lastHitBy(): { isPlayer: boolean } | null {
    return this.lastHitByRecord;
  }

  /** Template failure flags (accessors over `failures`). */
  get failEngine(): boolean {
    return !!this.failures.engineOut;
  }

  set failEngine(v: boolean) {
    this.failures.engineOut = v;
  }

  get failEngineSide(): -1 | 0 | 1 {
    return this.failures.engineSide ?? 0;
  }

  set failEngineSide(v: -1 | 0 | 1) {
    this.failures.engineSide = v;
  }

  get failFire(): boolean {
    return !!this.failures.fire;
  }

  set failFire(v: boolean) {
    this.failures.fire = v;
  }

  get failHydro(): boolean {
    return !!this.failures.hydroDegraded;
  }

  set failHydro(v: boolean) {
    this.failures.hydroDegraded = v;
  }

  get failGearJam(): boolean {
    return !!this.failures.gearJam;
  }

  set failGearJam(v: boolean) {
    this.failures.gearJam = v;
  }

  get failFuel(): boolean {
    return !!this.failures.fuelLeak;
  }

  set failFuel(v: boolean) {
    this.failures.fuelLeak = v;
  }

  get isWingDetached(): boolean {
    return this.wingDetached;
  }

  fwd(): THREE.Vector3 {
    return new THREE.Vector3(0, 0, 1).applyQuaternion(this.quat);
  }

  up(): THREE.Vector3 {
    return new THREE.Vector3(0, 1, 0).applyQuaternion(this.quat);
  }

  right(): THREE.Vector3 {
    return new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat);
  }

  /** Smoke / fire state for the damage visuals (v2: smoke < 60 hp,
   *  fire < 35 hp). The controller never spawns the plumes itself —
   *  main.ts reads this every frame and drives CrashVFXManager.
   *  The fire state machine (damage-model §3) overrides the hp
   *  thresholds: a burning jet shows fire at any hp, with the kind
   *  (fuel = orange flame + black smoke, engine = localised fire)
   *  and the ramped intensity for the visuals. */
  getDamageFx(): {
    smoke: boolean;
    fire: boolean;
    fireKind: 'fuel' | 'engine' | null;
    fireIntensity: number;
  } {
    return {
      smoke: this.alive && (this.hp < SMOKE_HP || this.fireKind !== null),
      fire: this.alive && (this.hp < FIRE_HP || this.fireKind !== null),
      fireKind: this.fireKind,
      fireIntensity: this.fireIntensity,
    };
  }

  /** Collision-event feed from the integrator: call once per Rapier
   *  contact involving this body (before step()). impactVy is the
   *  vertical velocity at contact (negative = descending). Sets the
   *  grounded test for the next step and the hard-landing check. */
  notifyContact(impactVy: number, isDeck: boolean): void {
    this.pendingContact = { impactVy, isDeck };
  }

  /** Load the GLB for this modelKey (MeshoptDecoder registered — the
   *  v1.1 fix) with a procedural fallback on failure. The template is
   *  cached per key and cloned per aircraft (materials cloned too, so
   *  per-jet tinting never mutates the template). Builds the rig. */
  async attachModel(): Promise<void> {
    let template:
      | { group: THREE.Group; tris: number; failed: boolean; error?: string }
      | undefined = Aircraft.modelCache.get(this.modelKey);
    if (!template) {
      try {
        let group: THREE.Group;
        if (this.modelKey === 'heli') {
          group = proceduralHeli();
        } else {
          const uri = await MODEL_LOADERS[this.modelKey]();
          const scene3 = await loadGLB(uri);
          scene3.traverse((o) => {
            if ((o as THREE.Mesh).isMesh) (o as THREE.Mesh).castShadow = false;
          });
          if (this.modelKey === 'fa18') {
            const pilot = scene3.getObjectByName('helmet.002');
            if (pilot) pilot.visible = false;
          }
          const targetLen =
            this.modelKey === 'ford' ? 333 : this.spec.length;
          const norm = normalizeAircraft(
            scene3,
            targetLen,
            NOSE_ROT_Y[this.modelKey],
          );
          group = norm.group;
        }
        template = { group, tris: countTris(group), failed: false };
        Aircraft.modelCache.set(this.modelKey, template);
      } catch (e) {
        const error = String((e as Error)?.message ?? e);
        const proc = proceduralJet(
          this.spec.tint,
          this.modelKey === 'mig29' || this.modelKey === 'su34',
        );
        if (this.modelKey === 'su34') proc.scale.setScalar(23.3 / 19);
        template = { group: proc, tris: countTris(proc), failed: true, error };
        Aircraft.modelCache.set(this.modelKey, template);
      }
    }
    this.modelStatus = template.failed ? 'failed' : 'ok';
    this.modelError = template.error;
    this.modelTris = template.tris;
    this.model = template.group.clone(true);
    this.model.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && (o as THREE.Mesh).material) {
        (o as THREE.Mesh).material = (
          (o as THREE.Mesh).material as THREE.Material
        ).clone();
      }
    });
    this.group.add(this.model);
    this.modelReady = true;
    if (!this.rig) this.rig = new AircraftRigBindings(this.model, this.spec);
  }

  /** One fixed step — canonical form. Call after the integrator's
   *  world.step() (and after feeding that step's contacts via
   *  notifyContact()). turbAcc is an optional gust acceleration; wind
   *  is an AIRSPEED offset (v1.1 — the wind vector is subtracted from
   *  ground velocity, never added as an acceleration). */
  step(
    dt: number,
    controls: FlightControls,
    wind: THREE.Vector3,
    terrainHeightAt: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void;
  /** One fixed step — template form (main.ts player/AI loop). Inputs
   *  come from the aircraft's public fields (pitchIn / rollIn / yawIn /
   *  thrTarget / flaps / airbrake / vector / gear / hook / brake) and
   *  terrain height from `this.terrainHeightAt` — main MUST assign the
   *  real ground-height function there (default is flat 0). */
  step(dt: number, wind: THREE.Vector3, turbAcc?: THREE.Vector3 | null): void;
  step(
    dt: number,
    controlsOrWind: FlightControls | THREE.Vector3,
    windOrTurb?: THREE.Vector3,
    terrainHeightAt?: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void {
    if (controlsOrWind instanceof THREE.Vector3) {
      const controls: FlightControls = {
        pitch: this.pitchIn,
        roll: this.rollIn,
        yaw: this.yawIn,
        throttle: null, // null = keep thrTarget (main sets it directly)
      };
      this.stepInternal(
        dt,
        controls,
        controlsOrWind,
        this.terrainHeightAt,
        windOrTurb,
      );
      return;
    }
    this.stepInternal(
      dt,
      controlsOrWind,
      windOrTurb ?? new THREE.Vector3(),
      terrainHeightAt ?? this.terrainHeightAt,
      turbAcc,
    );
  }

  private stepInternal(
    dt: number,
    controls: FlightControls,
    wind: THREE.Vector3,
    terrainHeightAt: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void {
    if (!this.alive) return;
    // Pass 8 (item 1): a body that is not enabled is not placed yet
    // (a fresh Aircraft before its first resetAt, or a wreck whose
    // body was disabled by crash()/finishCrash()). Rapier does not
    // integrate a disabled body, so syncFromBody would read a stale
    // translation and the solver would integrate forces against a
    // frozen position. Do not step it — this is the defense-in-depth
    // half of the pre-spawn exoneration (probe-prespawn).
    if (!this.body.isEnabled()) return;
    this.simTime += dt;
    if (this.gunCd > 0) this.gunCd -= dt;
    if (this.flareCd > 0) this.flareCd -= dt;
    this.syncFromBody();

    // Dying: the jet is falling, not flying — all controls are zeroed
    // and the throttle is cut (damage-model §6 step 1). The solver
    // steps degraded (stall/spin + drag) so the tumble and the
    // terminal velocity emerge naturally. This override is
    // authoritative: even if main.ts or the AI keeps writing
    // pitchIn/rollIn/yawIn, the solver sees zero.
    if (this.dying) {
      controls = { pitch: 0, roll: 0, yaw: 0, throttle: 0 };
    }

    const pitchIn = controlPitch(controls);
    const rollIn = controlRoll(controls);
    const thr = resolveThrottleTarget(controls);
    if (thr !== null) this.thrTarget = clamp(thr, 0, 1);
    // VTOL hover altitude-hold (pass 6, item 8): the F-35A at VEC 90
    // and 100% throttle lifted off the deck but sank and went down
    // instead of hovering — the pilot cannot hold the ~94% hover
    // point precisely on a slider, so it porpoised (climb at 96%,
    // sink at 92%) and the sink half ended in a hard landing. There
    // was an attitude FCS (hover auto-level) but no altitude hold.
    // In a deep hover (vector >= 70 deg, slow, airborne) the
    // throttle becomes an altitude command: the pilot's throttle
    // sets the desired vertical speed relative to the hover point
    // (100% -> +~2 m/s climb, hover point -> 0, idle -> -5 m/s
    // descent), and a P loop on the vertical-speed error drives the
    // actual throttle to hold it. Gated on !onSurface (previous
    // step): on the deck the pilot's throttle rules, so a parked
    // jet at vector 90 does not lift off by itself. Forward flight
    // (vector < 70 or ias >= 25) is untouched.
    if (
      this.spec.vtol &&
      this.vectorDeg >= 70 &&
      this.ias < 25 &&
      !this.onSurface &&
      this.alive &&
      !this.dying
    ) {
      const hoverThr = (this.spec.mass * G) / this.spec.thrustMax;
      // The pilot's command is `pilotThr` (recorded by the thrTarget
      // setter), NOT thrTarget itself — the hold writes thrTarget,
      // so reading it back would feed the auto-throttle's own output
      // into its command and oscillate (pass 6 diagnosis).
      const vsDes = clamp((this.pilotThr - hoverThr) * 30, -5, 3);
      // Pass 9 (item 8): integral term. The P loop alone leaves a
      // steady-state sink (~20 fpm live) because holding a constant
      // VS needs throttle slightly above hoverThr, which a pure P
      // loop only produces while an error persists. Accumulate the
      // VS error (clamped against windup) and add it.
      const vsErr = vsDes - this.vel.y;
      this.hoverVsInt = clamp(
        this.hoverVsInt + vsErr * dt * HOVER_VS_KI,
        -HOVER_VS_INT_MAX,
        HOVER_VS_INT_MAX,
      );
      const autoThr = clamp(
        hoverThr + vsErr * HOVER_VS_KP + this.hoverVsInt,
        0,
        1,
      );
      // Write the backing field directly: the thrTarget setter would
      // record the auto-throttle output as the pilot's command.
      this._thrTarget = autoThr;
    } else {
      // Hold inactive (forward flight, on the deck, dead, dying):
      // drop the integral so a stale offset cannot carry into the
      // next hover.
      this.hoverVsInt = 0;
    }
    if (controls.flaps !== undefined) this.flaps = clamp(controls.flaps, 0, 30);
    if (controls.airbrake !== undefined) this.airbrake = clamp(controls.airbrake, 0, 1);
    if (controls.vector !== undefined) {
      this.vectorDeg = clamp(controls.vector, 0, this.spec.maxVectorDeg);
    }
    if (controls.gear !== undefined && !this.failures.gearJam) {
      this.gear = controls.gear;
    }
    if (controls.hook !== undefined) this.hook = controls.hook;
    // Brake: only overwrite when the controls carry it. The
    // template-form step (main.ts and the probes) never passes brake
    // — it is set directly on the field by the UI latch — so the old
    // unconditional `controls.brake ?? false` reset it to false every
    // step and the brake NEVER worked live (the button latched red
    // but the jet kept rolling/sliding; pass 9, item 4). Flaps,
    // airbrake, vector, gear and hook above are all conditional for
    // the same reason; brake was the one exception.
    if (controls.brake !== undefined) this.brake = controls.brake;

    // Throttle spool (v2, 0.55/s).
    this.throttle += clamp(
      this.thrTarget - this.throttle,
      -SPOOL_RATE * dt,
      SPOOL_RATE * dt,
    );

    const S = this.spec;
    const m = S.mass;
    const vel = this.vel;
    const vAir = vel.clone().sub(wind);
    this.ias = vAir.length();

    const terrainH = terrainHeightAt(this.pos.x, this.pos.z);
    const deckY =
      this.deck && this.deck.alive
        ? this.deck.deckHeightAt(this.pos.x, this.pos.z)
        : -Infinity;
    const floorY = Math.max(terrainH, deckY);
    const isDeck = deckY > terrainH;

    const result = this.solver.solve({
      velocityAir: vAir,
      quat: this.quat,
      mass: m,
      wingArea: S.wingArea,
      wingAreaLift: S.wingAreaEffective ?? S.wingArea,
      span: S.span,
      chord: S.wingArea / S.span,
      length: S.length,
      throttle: this.throttle,
      thrustMax: S.thrustMax,
      vectorDeg: this.vectorDeg,
      flapsDeg: this.flaps,
      airbrake: this.airbrake,
      gearDown: this.gear,
      damage: this.damageZones,
      altitude: Math.max(this.pos.y, 0),
      density: 1.225 * Math.exp(-Math.max(this.pos.y, 0) / 8500),
      archetype: S.archetype,
      controls,
      heightAboveGround: this.pos.y - floorY,
      controlAuthority: { turn: S.turn, rollDeg: S.rollDeg, pitch: S.pitch },
      failures: {
        engineOut: this.failures.engineOut,
        engineSide: this.failures.engineSide,
        hydroDegraded: this.failures.hydroDegraded,
      },
      // BET per-model data (report §0/§2, supplement §1–§6).
      inducedK: S.inducedK,
      clCurve: S.clCurve,
      angVel: this.angVelBody,
      tvcDeg: S.tvcDeg,
      rollCapSchedule: S.rollCapSchedule,
      // Dying airframe (damage-model §6 step 1): the wing can no
      // longer hold 1 g — lift is cut to 15% so the jet freefalls
      // instead of gliding on with zero controls. 30% was not
      // enough: at 427 kt (220 m/s) 30% lift is still ~1 g, and the
      // live shoot-down glided down gently for 40–60 s instead of
      // falling as a wreck (probe: 44.4 s from 1,000 m). Drag is
      // untouched (the solver keeps the full drag on the unscaled CL).
      liftScale: this.dying ? 0.15 : 1,
      dying: this.dying,
      dt,
    });
    this.aoaDeg = result.aoaDeg;
    this.gload = result.gLoad;
    // Pass 5 (item 6): the STALL warning showed spuriously in VTOL
    // hover. The BET stall flag LATCHES (it only clears once the jet
    // has regained flying speed, v > 20 m/s), so a flag set during
    // the transition into hover never clears while hovering (v < 20)
    // — and in hover the wing is not flying at all (the lift fan
    // carries the jet), so a "stall" is meaningless. Gate the flag
    // on forward flight: thrust vector below 70 deg (hover is 90).
    this.stalled = result.stalled && this.vectorDeg < 70;
    this.spin = result.spin;
    this.spinArrow = result.spinArrow;
    this.spinDir = result.spinDir;
    this.betaDeg = result.betaDeg;

    // Forces -> acceleration -> new velocity (semi-implicit Euler, v2).
    const acc = result.lift
      .clone()
      .add(result.drag)
      .add(result.thrust)
      .divideScalar(m);
    acc.y -= G;
    if (turbAcc) acc.add(turbAcc);
    const newVel = vel.clone().addScaledVector(acc, dt);

    // --- Ground / deck contact (v2 groundCheck, Rapier-owned) ---
    const gearH = this.gear ? GEAR_HEIGHT : BELLY_HEIGHT;
    const contact = this.pendingContact;
    this.pendingContact = null;
    // A DYING jet that contacts anything solid while descending has
    // hit the ground — finish the crash now (fuel-scaled explosion +
    // debris + scorch at the impact position, then the spectate /
    // respawn flow). Without this, a wreck that touches down tilted
    // can bounce on the collider for tens of seconds without ever
    // entering the ground branch below (its centre sits above
    // floorY + gearH, and it is too fast/bouncy for the tilted-rest
    // rule), so the spectate never ends and the respawn never fires
    // — the v2 shoot-down's wreck glided/bounced for 40–60 s and
    // then fell into the stuck respawn state. Gated on descending
    // faster than 3 m/s, so a gentle mid-air bump does not finish a
    // wreck that is still falling.
    if (this.dying && contact && contact.impactVy < -3) {
      this.finishCrash();
      return;
    }
    const deckVel =
      isDeck && this.deck && this.deck.alive
        ? this.deck.velocityAt(this.pos.x, this.pos.z)
        : new THREE.Vector3();
    let steerYawRate = 0;
    let deckYawRate = 0;
    if (isDeck && this.deck && this.deck.alive) {
      deckYawRate = this.deck.yawRateAt(this.pos.x, this.pos.z);
    }
    // Physical ground contact this step — NOT the `grounded` flag. The
    // flag's 0.85xstall rotation gate flips far too early (F-22: 46 m/s
    // vs its 72 m/s Vr), which released the pitch clamp mid-roll and let
    // the full 55 deg/s authority whip the nose into a tail strike.
    let onSurface = false;
    // Ground-relative horizontal velocity (deck-relative on the carrier).
    let relH = new THREE.Vector3();
    // Tolerance 0.15 m (was 0.05): the runway box colliders sit 5 cm
    // above the analytic height (their top is elev + 0.05, so the gear
    // rests at floorY + gearH + 0.05), which put a resting jet EXACTLY
    // on the old 0.05 threshold — floating-point flicker read it as
    // airborne, grounded/onSurface went false, the rotation clamp
    // released, and the full pitch authority whipped every jet nose-up
    // at 30 kt. 0.15 m still means "on or within a hand's breadth of
    // the ground"; a jet 15 cm up is in ground effect, not flying.
    //
    // Tilted rest (v3): a jet that comes to rest TILTED on a slope
    // sits with its centre above floorY + gearH — the contact point
    // (a wingtip, the nose ski) is offset from the centre, and on a
    // slope the terrain there is higher. The 0.15 m tolerance only
    // covers a level rest, so a stalled jet that survived onto a
    // mountainside read as airborne forever (grounded=false, G 0.2,
    // 6 kt, no crash, no respawn — the v2 stall-down hang). A slow,
    // essentially stationary jet within 2 m of the surface is
    // resting, not flying: treat it as on the ground. Gated on low
    // horizontal speed (< 5 m/s) + no descent, so a fast low pass is
    // unaffected, and a VTOL hover (above 2 m) is unaffected. The
    // hard-landing check below still applies — a jet resting banked
    // past 0.45 rad has crashed, it has not landed.
    //
    // Deck-relative vertical velocity (issue 8): a jet resting on a
    // CLIMBING deck rises at the deck's rate (up to 25 m/s), so the
    // absolute newVel.y <= 0.5 gate read it as airborne the moment
    // the carrier started climbing — onSurface went false, onDeck
    // went false, the vertical coupling below never engaged, and the
    // jet was left behind. The gate is on the velocity RELATIVE to
    // the surface (deckVel.y on a deck, 0 on a runway).
    //
    // The position tolerance is wider on a deck (0.5 m vs 0.15 m):
    // a moving deck's climb rate changes step to step, so a jet
    // resting on it jitters a few cm above the nominal rest height
    // (measured 0.16 m — 1 cm over the 0.15 m gate — which flipped
    // onSurface false and broke the coupling). A runway never moves,
    // so it keeps the tight 0.15 m gate.
    // (Pass 7 tried 1.5 m / 0.5 m: the wider deck gate kept a jet
    // "on the deck" too long for the deck-takeoff probe to read it
    // airborne, and the wider runway gate let a single ArrowUp tap
    // lift the jet — both reverted; the deck-ride / score fixes
    // below come from the coupling, not the gate width.)
    const surfTol = isDeck ? 0.5 : 0.15;
    // Pre-coupling (issue 8): apply the vertical deck coupling
    // BEFORE the onSurface velocity gate, based on position only.
    // The gate below reads relVy from the PRE-coupling newVel.y, so
    // when the deck reverses (climb -> descent) the jet's leftover
    // climb momentum (up to +15 m/s vs a -10 m/s deck = relVy +25)
    // rejects the jet before the coupling inside the branch can fix
    // it — the jet leaves the deck, then falls back and crashes.
    // The coupling is applied when the jet is within the surface
    // tolerance on the deck, EXCEPT when rotating for takeoff (deck-
    // relative speed + pull-up), so a takeoff is never pinned down.
    // relVy is recomputed from the coupled velocity for the gate.
    // Deck-relative horizontal speed (0 for a parked jet on the
    // deck; >0.85x stall at rotation). Hoisted so both the
    // pre-coupling and the onSurface gate below can use it.
    const spPre = isDeck
      ? Math.hypot(newVel.x - deckVel.x, newVel.z - deckVel.z)
      : 0;
    if (isDeck && this.pos.y <= floorY + gearH + surfTol) {
      const rotating =
        spPre > S.stallSpeed * 0.85 && this.pitchIn < -0.25;
      if (!rotating) {
        newVel.y = deckVel.y + (floorY + gearH - this.pos.y) * 8.0;
      }
    }
    const relVy = newVel.y - (isDeck ? deckVel.y : 0);
    const spdH0 = Math.hypot(newVel.x, newVel.z);
    const restingTilted =
      spdH0 < 5 &&
      relVy <= 0.5 &&
      relVy >= -0.5 &&
      this.pos.y <= floorY + gearH + 2.0;
    // Whether the jet was on a surface last step — a continuing
    // contact is riding, not a new touchdown (see the vs computation
    // in the onSurface branch). Captured before the branch updates
    // the flag.
    const prevOnSurface = this.onSurface;
    // Pass 8 (item 4) — deck-gate HYSTERESIS. The gate is a position
    // check (deck 0.5 m): when terrain-following crests a climb the
    // deck can drop away from the jet by >1 m in one 0.05 s substep
    // (the jet's stale +15 m/s climb velocity vs the deck reversing
    // to -10 m/s), the gate flips false, the vertical coupling (which
    // only runs inside the branch) stops, the jet never re-engages,
    // and it falls off in 20-60 s. The fix is hysteresis, not a
    // wider gate: a jet ENTERS the deck contact at 0.5 m (unchanged
    // — a uniform 1.5 m widening was tried in pass 7 and reverted,
    // it kept a jet "on the deck" too long for probe-carrier-deck),
    // but once it is riding the deck it only EXITS at 1.5 m, so a
    // brief 1.25 m drop keeps the coupling engaged and the jet is
    // pulled back onto the deck. A genuinely departed jet still
    // reads airborne past deckTopY + 5 (the hysteresis band is 1.5 m,
    // not 5 m). The pre-coupling band above stays at surfTol — only
    // the exit gate widens.
    const wasOnDeck = prevOnSurface && this.onDeck;
    const exitTol = wasOnDeck ? 1.5 : surfTol;
    // On the deck, a slow jet (parked / taxiing, below half stall
    // speed relative to the deck) is on the surface by position
    // alone — the relVy <= 0.5 gate is waived. The deck's vertical
    // speed changes as terrain-following crests a climb, and the
    // jet's velocity (set a step earlier) is briefly stale, so a
    // resting jet reads relVy up to ~+10 and the gate would declare
    // it airborne, stop the coupling and launch it off the deck
    // (issue 8). A fast jet (takeoff roll / rotation) still needs
    // the velocity gate, so it can lift off. Runways keep the gate.
    const onSurfaceVel =
      relVy <= 0.5 || (isDeck && spPre < S.stallSpeed * 0.5);
    if (
      (this.pos.y <= floorY + gearH + exitTol && onSurfaceVel) ||
      restingTilted
    ) {
      // Touching a surface. Rapier resolves the penetration — this
      // block only reads the contact, applies roll physics and the
      // v2 runway/deck rules; it never pins pos.y.
      // A dying jet touching the ground is the impact — finishCrash
      // (fuel-scaled explosion + debris + scorch), not a landing.
      if (this.dying) {
        this.finishCrash();
        return;
      }
      onSurface = true;
      this.onSurface = true;
      this.grounded = true;
      this.onDeck = isDeck;
      // Pass 6 (item 6): the BET stall flag LATCHES — it only
      // clears once the jet has regained flying speed (v > 20) —
      // so a jet that stalled, recovered and landed kept showing
      // STALL while parked (6 kt, AoA 0 deg, throttle 0). A jet on
      // the ground is not stalled: the wing is not flying. Clear
      // it on contact. The stall nose-drop is gated on !onSurface
      // anyway, so this cannot touch a departure.
      this.stalled = false;
      // Hard-landing vertical speed, RELATIVE to the surface (issue
      // 8): impactVy and newVel.y are absolute, but a jet riding a
      // descending deck sinks at the deck's rate — a 10 m/s deck
      // descent is not a 10 m/s impact. Subtract the deck's vertical
      // velocity when the contact is with the deck. And a jet that
      // was ALREADY on the surface last step is riding, not landing:
      // its relative velocity is the deck carrying it (up to 15 m/s
      // on a terrain-following climb), not an impact — so vs is 0
      // for a continuing contact, and only a NEW touchdown (airborne
      // -> onSurface) is judged on impact speed. Without this, a jet
      // riding a climbing deck read as "sinking" at the climb rate
      // and crashed "hard landing" (measured vs=15.46, newVelY=-0.46,
      // deckVelY=15.00, contact=none).
      const vs = prevOnSurface
        ? 0
        : contact
          ? Math.max(
              0,
              -(contact.impactVy - (contact.isDeck ? deckVel.y : 0)),
            )
          : Math.max(0, -relVy);
      const bank = Math.abs(
        new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').z,
      );
      if (vs > HARD_LANDING_VS || bank > HARD_LANDING_BANK) {
        this.crash('hard landing');
        return;
      }
      // Ground roll, relative to the deck when on it: friction +
      // brakes + nose-wheel steering (v2). Deck velocity passes
      // through untouched (subtracted before friction, added back).
      //
      // The relative velocity is computed against the PREVIOUS deck
      // point velocity, not the current one — newVel was built from
      // the velocity the jet was carrying, which contains the previous
      // deck velocity. The deck rotates, so the current deck point
      // velocity has turned away from the one the jet carries;
      // subtracting the current one injects a spurious relative
      // velocity (~|D| * dyaw per step) that the 0.3 m/s^2 ground
      // friction can never kill, and the jet slides off the deck
      // sideways during a turn (issue 3). The relative velocity is
      // then rotated with the deck (it is fixed in the deck frame,
      // not the world frame) before the current deck velocity is
      // added back, so a jet parked on the deck rides it through a
      // 120 deg waypoint turn without drifting.
      const prevD = this._prevDeckValid ? this._prevDeckVel : deckVel;
      relH = new THREE.Vector3(newVel.x - prevD.x, 0, newVel.z - prevD.z);
      const sp = relH.length();
      // Deck hold (issue 8): a parked jet on the deck (idle throttle,
      // low deck-relative speed) is held firmly, so the small
      // velocity-transfer error that accumulates through a turn +
      // climb (measured ~0.4 m/s aft drift) cannot walk it off the
      // stern. A jet taking off (throttle up) uses normal friction,
      // so deck takeoffs are unaffected.
      // Pass 5 (item 5): the deck hold disengaged when the
      // deck-relative speed exceeded 1 m/s, and the velocity transfer
      // through a waypoint turn leaves a ~0.5-1.5 m/s residual — so a
      // parked jet's hold would drop out mid-turn, the weak ground
      // friction (0.3) could not hold it against the deck's motion,
      // and it walked off the stern in 12-35 s (the v4 play-test).
      // The throttle gate already distinguishes a parked jet from a
      // taxiing one, so the speed gate is widened to 3 m/s: a parked
      // jet drifting at up to 3 m/s is still a parked jet and is held.
      const deckHold = isDeck && this.throttle < 0.1 && sp < 3;
      const fr = this.brake
        ? BRAKE_DECEL
        : deckHold
          ? 5.0
          : GROUND_FRICTION;
      // Friction acts at ANY relative speed (no 0.5 m/s floor): a jet
      // parked on the deck faces a 40 m/s headwind from the carrier's
      // own motion, and the aerodynamic drag creeps it aft at ~0.06
      // m/s per second. With the old `sp > 0.5` guard the friction
      // never engaged on that slow creep, so the jet drifted ~8 m aft
      // over a waypoint turn and slid off the stern (issue 3). The
      // 0.3 m/s^2 ground friction comfortably holds it against the
      // ~0.07 m/s^2 drag. (Steering still needs real rolling speed.)
      relH.multiplyScalar(Math.max(0, 1 - (fr * dt) / Math.max(sp, 1e-6)));
      if (sp > 0.5 && Math.abs(rollIn) > 0.1) {
        steerYawRate = rollIn * STEER_RATE * clamp(sp / 20, 0, 1);
      }
      const dyaw = deckYawRate * dt;
      if (dyaw !== 0) {
        const c = Math.cos(dyaw);
        const s = Math.sin(dyaw);
        const rx = relH.x * c + relH.z * s;
        const rz = -relH.x * s + relH.z * c;
        relH.set(rx, 0, rz);
      }
      newVel.x = relH.x + deckVel.x;
      newVel.z = relH.z + deckVel.z;
      // Vertical deck coupling (issue 8): a jet resting on the deck
      // rides it up AND down. A pure velocity match (newVel.y =
      // deckVel.y) was not enough — the jet drifted up ~0.2 m/s
      // relative to the deck (the kinematic deck body is both
      // translated to its new position AND integrated by its linvel
      // each step, so the contact and the controller disagree by a
      // small amount each substep). So the coupling is proportional
      // on the height gap: the deck's vertical velocity plus a
      // correction that drives the jet back to the rest height
      // (floorY + gearH). A 0.5 m gap closes at 1 m/s (gain 2/s) —
      // gentle, and it holds the jet on the deck through climbs,
      // descents and turns. Gated on the deck: a runway never moves
      // vertically, and forcing newVel.y there would pin a rotating
      // jet to the ground.
      if (isDeck) {
        newVel.y = deckVel.y + (floorY + gearH - this.pos.y) * 8.0;
      }
      this._prevDeckVel.copy(deckVel);
      this._prevDeckValid = true;
      // Arresting wires (v2: 3 zones across the stern deck, local
      // z −45..−105, |x| < 26; hook down + under 72 m/s = trap).
      if (this.onDeck && this.hook && !this._trapped && this.deck) {
        const rel = this.deck.local(this.pos.x, this.pos.z);
        const inWires =
          rel !== null &&
          rel.z < WIRE_LOCAL_Z_MAX &&
          rel.z > WIRE_LOCAL_Z_MIN &&
          Math.abs(rel.x) < WIRE_LOCAL_X;
        if (inWires && sp < TRAP_MAX_SPEED) {
          this._trapped = true;
          this.trapTimer = 0;
          this.events.onTrap?.(this);
        } else if (inWires && sp >= TRAP_MAX_SPEED) {
          this.events.onWireSnap?.(this);
          this.hook = false;
        }
      }
      if (this._trapped) {
        this.trapTimer += dt;
        const k = Math.max(
          0,
          1 - ((TRAP_BASE_DECEL + this.trapTimer * TRAP_RAMP) * dt) / Math.max(sp, 1),
        );
        newVel.x = deckVel.x + (newVel.x - deckVel.x) * k;
        newVel.z = deckVel.z + (newVel.z - deckVel.z) * k;
        if (sp < 2) {
          this._trapped = false;
          this.thrTarget = 0;
          this.throttle = 0;
        }
      }
      // Deck velocity transfer: a landed jet rides the moving deck
      // (deckYawRate is already set above, from the current deck).
      // Take-off rotation gate (v2): enough speed + pull lifts off
      // naturally via the aero step; the flag just gates roll physics.
      if (sp > S.stallSpeed * 0.85 && pitchIn < -0.25) this.grounded = false;
      else if (sp < 5) this.grounded = true;
    } else {
      this.grounded = false;
      this.onDeck = false;
      onSurface = false;
      this.onSurface = false;
      this._prevDeckValid = false;
    }
    // Terrain collision safety net: only when actually penetrating.
    // A dying jet penetrating the terrain is the impact — finishCrash.
    if (this.pos.y < terrainH - 0.5) {
      if (this.dying) this.finishCrash();
      else this.crash('terrain');
      return;
    }

    // G-load on the ground is 1.0, not the solver's 0.2 floor (research
    // §6; the play-test saw 0.2 g on a parked jet). The solver's floor
    // stays — this override is the controller's ground truth.
    if (this.grounded) this.gload = 1;

    // G-load + structural limit (v2): over-G damages the airframe.
    if (this.gload > S.maxG + OVER_G_MARGIN && this.ias > OVER_G_MIN_SPEED) {
      this.applyDamage(OVER_G_DPS * dt, 'fuselage', false);
      if (!this.alive) return;
    }

    // Rotation: solver torque -> v2 rates -> Rapier angular velocity.
    // The BET solver returns `rates` directly (integrated from the
    // real moment model, ω̇ = I⁻¹(M − ω × Iω)); the curve solver
    // returns only a torque, recovered here with torqueToRates().
    const rates =
      result.rates ?? torqueToRates(result.torque, m, S.span, S.length);
    // Dying departure (damage-model §6): a dead jet with zeroed
    // controls stays trimmed and glides — the live shoot-down held
    // AoA 10° and G 0.8 all the way down. A wreck does not trim: the
    // nose is driven DOWN into a spiral dive (0.15 rad/s ≈ 9°/s),
    // the wing unloads, and the spin model takes over — the jet
    // falls as a tumbling wreck instead of gliding on. A nose-UP
    // bias was tried first (0.5, 0.25, 0.1): it held the jet at
    // ~90° AoA, maximum separation drag, ~30 m/s terminal — a slow
    // wallow, 42 s from 1,000 m. Nose-down settles it in a steep
    // spiral at ~60–80 m/s (the research's flat-spin band).
    if (this.dying) {
      rates.pitchRate = Math.max(rates.pitchRate, 0.15);
    }
    // Takeoff rotation (BET report §3): while the gear is on a surface,
    // the nose stays down until the per-spec rotation speed, and even
    // then the rotation rate is capped at ~12 deg/s — the full 55 deg/s
    // authority whips the nose up, the fuselage tail strikes the runway,
    // and Rapier resolves the penetration by launching the jet, which
    // read as a 40 m/s "lift-off" (F-18 flaps 30, F-15E flaps 30) or a
    // tumble (F-35A flaps 0, AoA -129 deg). Gated on physical contact
    // (onSurface), not the grounded flag — the flag's 0.85xstall gate
    // flips far too early for this (F-22: 46 m/s vs its 72 m/s Vr).
    // Pitch-down is left alone (the nose gear stops it). Speed is
    // deck-relative on the carrier.
    // A nose-up rate on the ground is only allowed while the pilot is
    // actually pulling (pitchIn < -0.25, the same threshold the
    // grounded flag uses). The BET solver is a moment model: the
    // thrust line sits 0.2 m below the CG, so at full thrust the
    // engines pitch the jet nose-up with a ~39 kN·m moment that
    // winds the solver's pitch rate up behind this clamp — without
    // the pull gate the jet rotated on its own the instant the
    // speed clamp released at Vr, lifting off with no input at all
    // (probe-stall-respawn 2c/2d). The curve solver never has this
    // problem (its pitch rate is prescribed from the input), and a
    // real jet only rotates when the pilot pulls — so the gate
    // encodes the procedure, not a fudge. The rate sync below feeds
    // the gated rate back into the solver, so the windup is reset
    // every step instead of accumulating.
    // The gate also applies while LOW (CG < 10 m above the surface)
    // after leaving it: a jet that runs off the runway end at
    // ~195 kt with no input pitches itself up to its ~5 deg trim
    // AoA (tail downwash + LEX nose-up moment) and flies off the
    // end (2c again, at 660 m). Held level it sinks onto the slope
    // like the curve solver's jet. Above 10 m the pilot has full
    // authority — a real takeoff climbs through 10 m in ~1 s, and
    // the bank-hold / max-turn checks fly at 200 m, so the gate
    // cannot touch them. (A stabilator incidence was tried first
    // to trim the 5 deg out: it fixed the takeoff but reversed
    // the bank-hold turn direction — the extra tail lift pitched
    // the nose down until the AoA went negative and the banked
    // lift turned the jet the wrong way — so the trim stays as
    // modelled and the gate carries the takeoff behaviour.)
    if (onSurface && rates.pitchRate < 0) {
      // Vr is an AIRSPEED, not a deck-relative speed: relH subtracts
      // the carrier's 40 m/s, so on a moving deck the gate read
      // 52 kt when the jet was really at 131 kt and never let it
      // rotate (probe-carrier-deck: fa18/f35a rolled the SHIELD
      // deck at 214 kt with full pull and never lifted off). The
      // ias is the air-relative speed, so it is Vr on a runway and
      // on a deck alike.
      if (this.ias < S.rotateSpeedKt * 0.514444 || pitchIn >= -0.25) rates.pitchRate = 0;
      else rates.pitchRate = Math.max(rates.pitchRate, -ROTATION_RATE_MAX);
    }
    // AoA limiter (BET only, pass 5): a held full pull must not run
    // the AoA past the stall into a backflip. The BET is a moment
    // model — at full elevator the tail moment overwhelms the pitch
    // damping, so the pitch rate stays high while the AoA builds: a
    // held pull on takeoff ran the F-22 to AoA 158 deg (deep stall,
    // fall back onto the runway) and the other jets to a 43-51 deg
    // wallow (probe-takeoff-trace). The curve solver never has this
    // problem (its pitch rate is prescribed from the input and its
    // stall nose-drop caps it). The limiter tapers the nose-up rate
    // to zero as the AoA approaches 50 deg — past the 35 deg
    // peak-CL stall, so the stall flag, the stall nose-drop and the
    // stall-down probe (which needs AoA > 40 deg) all still work,
    // but far below the backflip. Applied before syncRates so the
    // solver's integrated omega is fed the limited rate, not the
    // runaway one. A pilot who releases the pull recovers normally.
    // The taper starts AT the 35 deg peak-CL stall (not 30): a
    // 30 deg taper already cut the pitch rate by 15% at 33 deg and
    // capped the low-speed stall test at maxAoA 33.1 deg — below
    // the 35 deg stall flag — and let the stall-down jet descend
    // gently and land instead of crashing (probe-bet + probe-
    // stall-respawn regressions). At 35 deg the factor is 1.0 (no
    // limiting), so the stall flag and stall-down both work; the
    // taper only bites above the stall, where a backflip lives.
    if (this.isBet && rates.pitchRate < 0 && this.aoaDeg > 35) {
      rates.pitchRate *= clamp((50 - this.aoaDeg) / 15, 0, 1);
    }
    // No pull, no pitch-up (pass 7, item 2). The gate zeroes the
    // (below 10 m AGL — the original gate: a jet that runs off the
    // runway end with no input sinks onto the slope instead of
    // flying off, and a single ArrowUp tap does not run away into
    // a liftoff) OR the pitch is already 5 deg nose-up or more
    // while the wings are near level (|bank| < 45 deg — the
    // altitude loop: hands-off at altitude the jet climbs at its
    // trim AoA, the speed bleeds, and the phugoid grows into a
    // loop — v4.2: F-22 rolled inverted, Bank 165–177 deg, and
    // dived at −25,454 fpm with NO input). The bank condition
    // keeps the gate out of a banked turn: the max-turn AoA hold
    // pitches up at ~12 deg AoA and 80 deg bank to hold altitude,
    // and the gate was zeroing that nose-up rate whenever the
    // hold's pitchIn went >= −0.25, dropping the MiG-29 turn rate
    // to 14.9 deg/s (band 15–24). (An AoA < 5 deg condition was
    // tried first and failed the other way: the F-22's hands-off
    // AoA runs >= 5 deg, so the gate never bit and it inverted on
    // all three runways.) Below 5 deg nose-up and above 10 m the
    // nose-up rate is free, so a jet pushed nose-down can pitch
    // back up to level. With a pull this does NOT apply — a real
    // rotation / turn gets full pitch authority.
    const pitchNow = new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').x;
    const bankNow = new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').z;
    const pitchedUp =
      pitchNow < (-5 * Math.PI) / 180 &&
      Math.abs(bankNow) < (45 * Math.PI) / 180;
    if (
      !onSurface &&
      rates.pitchRate < 0 &&
      pitchIn >= -0.25 &&
      (this.pos.y - floorY < 10 || pitchedUp)
    ) {
      rates.pitchRate = 0;
    }
    // Nose-down on the ground: the nose gear is a hard stop — the
    // jet can derotate from nose-up down to level (landing), but it
    // cannot pitch below level. Without this the BET elevator's
    // nose-down moment pivots the jet on its nose gear, raising
    // the CG and fooling the lift-off detection (probe-stall-
    // respawn 2d: a push "lifted off" at 300 m with the nose 8 deg
    // down and AoA −9 deg — the jet was pivoting, not flying).
    // Clamp the rate so this step lands exactly at level, not past
    // it: rate <= −pitch/dt. Pitch is Euler YXZ x (x > 0 = nose
    // down), so a nose-up attitude is a negative pitch and the cap
    // is positive there; at level or nose-down the cap is 0.
    // The clamp also applies while LOW (CG < 7 m above the
    // surface, gear within ~5 m) even after leaving the surface:
    // a jet that runs off the runway end at ~195 kt and is pushed
    // nose-down drives its nose gear into the falling slope and
    // bounces off it, which reads as a lift-off (2d again, at
    // 680 m). Held level it settles onto the slope like the curve
    // solver's jet instead. Above 7 m the pilot has full
    // nose-down authority (dives, carrier approaches).
    if (rates.pitchRate > 0 && (onSurface || this.pos.y - floorY < 7)) {
      const pitch = new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').x;
      const maxNoseDownRate = Math.max(0, -pitch / dt);
      rates.pitchRate = Math.min(rates.pitchRate, maxNoseDownRate);
    }
    // Parked on a surface (runway or deck): zero the solver's
    // aerodynamic rates (probe-respawn-carrier 4 for the deck;
    // pass 6, item 1 for runways). A parked jet rides the deck
    // through a waypoint turn via deckYawRate (added below), but
    // the BET fin's weathercock yaw moment fights it: the heading
    // lags the carrier's yaw, the sideslip hits ±19 deg, the fin
    // yaws the jet the OTHER way, and the heading error grows to
    // 37 deg while the jet slides aft off the stern. The curve
    // solver has no fin side force, so it never fights the deck.
    // A parked jet is not flying — its heading is the deck's
    // heading and its attitude is the deck's attitude — so ALL
    // three aerodynamic rates are zeroed (before the sync, so
    // the solver's ω does not wind up fighting the deck either).
    // The yaw zeroing alone fixed the heading (37 deg → 0.7 deg)
    // but the fin's side force (the 19.8 deg rotation-term
    // "sideslip" at 120 m aft) still rolled the jet to 15.7 deg
    // bank, which friction cannot resist (it acts on velocity,
    // not attitude), and it still slid off. Zeroing pitch and
    // roll too holds the jet level on the deck. Gated on
    // onSurface + idle throttle: a jet taxiing (steerYawRate,
    // added below) or taking off (throttle up) keeps its
    // aerodynamic rates. BET only — the curve solver's prescribed
    // rates are already zero when parked (no input), so this is
    // a no-op for it, but it is gated on syncRates (BET) to be
    // explicit.
    // Pass 6 (item 1): the same wind-driven roll killed parked
    // jets on RUNWAYS, not just the deck — a 3 m/s wind drives a
    // BET roll moment at ~0 airspeed, the controller prescribes
    // the solver's angular velocity to Rapier (bypassing the gear
    // contact that would hold a real jet level), and the parked
    // jet slowly rolled onto one skid (bank 0 -> 17 deg in ~5 s)
    // until the ground block's bank > 26 deg rule crashed it
    // "hard landing" (probe-live-spawn: 9/18 fail at live-like
    // variable dt, 5/18 even at fixed 1/60 — the old 5 s parked
    // probes never ran long enough to see it; WIND=0 -> 0 fail).
    // So the parked zeroing is no longer deck-only: any surface.
    if (
      onSurface &&
      this.throttle < 0.1 &&
      typeof this.solver.syncRates === 'function'
    ) {
      rates.pitchRate = 0;
      rates.yawRate = 0;
      rates.rollRate = 0;
    }
    // Pass 7 (item 7): a jet on the ground is not stalled — the
    // wing is not flying — so clear the stall flag EVERY step on a
    // surface, not only on a new contact. The pass-6 fix cleared it
    // in notifyContact, which fires only on a contact START; the
    // solver's flag LATCHES (it only clears once v > 20 AND AoA <
    // peak − 3), so a jet that stalled, recovered and landed kept
    // showing STALL while parked (v4.2: F-22 parked on Charlie at
    // 6 kt, AoA 0 deg, STALL showing; F-35A dead on Alpha, STALL
    // showing). The solver re-asserts the flag each step while the
    // latch holds, so the clear must run each step too. The stall
    // nose-drop is gated on !onSurface, so this cannot touch a
    // departure; on liftoff the solver's latch clears once the jet
    // has flying speed and a low AoA.
    if (onSurface) this.stalled = false;
    // Rolling on a surface at any throttle: the gear holds the
    // wings level (pass 6, item 1). The aileron cannot roll a jet
    // on its gear, and the wind moment must not start a roll
    // during the takeoff roll either. Level the roll rate toward
    // bank 0 (a gentle spring, max ~9 deg/s) so a banked
    // touchdown settles level instead of freezing tilted. Pitch
    // stays gated by the rotation clamp above (a jet must pitch
    // to rotate); yaw is steering, added below. BET only.
    if (onSurface && typeof this.solver.syncRates === 'function') {
      const bankNow = new THREE.Euler().setFromQuaternion(
        this.quat,
        'YXZ',
      ).z;
      rates.rollRate = clamp(-bankNow * 2, -0.15, 0.15);
      // Pass 7 (item 1/2): with no steering input the gear holds
      // the HEADING too. The BET fin's weathercock yaw moment
      // otherwise yaws a rolling jet freely — a hands-off takeoff
      // roll veers off the runway side (the v4.2 F-22 veered right
      // of the centreline, and the Charlie jet yawed ~65 deg with
      // no input at 0% throttle), and off the runway the gear
      // hits the heightfield skirt, which is the spawn/takeoff
      // death path. A parked jet is already covered by the idle-
      // throttle zeroing above; this covers the rolling jet at any
      // throttle. Steering (rollIn) still yaws it via
      // steerYawRate below.
      if (Math.abs(rollIn) <= 0.1) rates.yawRate = 0;
    }
    // Airborne auto-level (pass 7, item 2): with no roll input the
    // wings return gently to level. The BET has no dihedral and
    // only CLP = −2.0 of roll damping, so any bank — a gust, a
    // contact kick, the wind — grows into a spiral dive: the
    // hands-off probe showed banks building 22 → 41 → 62 → 109
    // deg and the jet diving, and the v4.2 play-test's F-22 rolled
    // inverted (Bank 165–177 deg) with NO input. A real fighter's
    // FCS holds the bank hands-off; this is the same behaviour as
    // a gentle spring (max ~14 deg/s) added to the solver's roll
    // rate, so the solver's dynamics still contribute but the bank
    // cannot run away. Gated on no roll input (|rollIn| < 0.1) so
    // a commanded bank (turns, the AI, the max-turn checks) is
    // untouched; gated off while dying / stalled / in VTOL hover
    // (the hover FCS has its own auto-level). Gated on rollIn
    // EXACTLY 0 (not < 0.1): the max-turn probe holds its 80 deg
    // bank with a feedback rollIn that goes to ~0 on target, and a
    // < 0.1 gate let the auto-level fight the bank-hold and drop
    // the turn rate below its band (probe-bet: fa18 13.0, mig29
    // 13.6 deg/s). A released stick / centred stick reads exactly
    // 0, so hands-off still gets the auto-level. Also gated on
    // |bank| < 45 deg: it must not fight a deliberate steep bank
    // (the 80 deg bank-hold) even in the instants the feedback
    // rollIn reads exactly 0 — the spring only returns moderate
    // banks to level, and the spiral is slow enough below 45 deg
    // that the spring catches it before it gets there.
    if (
      !onSurface &&
      typeof this.solver.syncRates === 'function' &&
      rollIn === 0 &&
      !this.dying &&
      !this.stalled &&
      this.vectorDeg < 70
    ) {
      const bankNow = new THREE.Euler().setFromQuaternion(
        this.quat,
        'YXZ',
      ).z;
      if (Math.abs(bankNow) < (45 * Math.PI) / 180) {
        rates.rollRate += clamp(-bankNow * 2, -0.25, 0.25);
      }
    }
    this.solver.syncRates?.(rates);
    // BET stall / dying nose-drop (rate biases, AFTER the sync).
    // The BET solver has no emergent nose-down departure past
    // peak-CL AoA, and a moment cannot produce one (it recovers
    // the AoA and the lift with it). These are ADDITIVE biases on
    // the applied rate — a full pull can still overpower the
    // stall one — and they are added after the sync so they never
    // accumulate in the solver's integrated ω (the same reason
    // the steer/deck yaw are added here). BET only (the curve
    // solver has its own stall nose-drop and its own correct
    // dying fall) and airborne only (on a surface they would
    // pitch the jet into the ground past the nose-down clamp).
    if (!onSurface && typeof this.solver.syncRates === 'function') {
      if (this.stalled) rates.pitchRate += BET_STALL_NOSE_DROP_RATE;
      if (this.dying) rates.pitchRate += BET_DYING_NOSE_DROP_RATE;
    }
    const angBody = new THREE.Vector3(
      rates.pitchRate,
      rates.yawRate + steerYawRate + deckYawRate,
      rates.rollRate,
    );
    const angWorld = angBody.applyQuaternion(this.quat);
    this.body.setAngvel({ x: angWorld.x, y: angWorld.y, z: angWorld.z }, true);
    this.body.setLinvel({ x: newVel.x, y: newVel.y, z: newVel.z }, true);
    // VSI reads 0 on a surface (live finding 7): the single-substep
    // gravity increment (-9.81 * dt) left a constant -93/-94 fpm
    // while parked on the ground, because the ground block never
    // zeroes the vertical velocity. A resting jet has no vertical
    // speed.
    this.vsi = onSurface ? 0 : newVel.y;

    // Fuel burn (v2 rates are per second on a 0..100 scale; fuel here
    // is 0..1, so divide by 100 — same ~13 min endurance at full
    // throttle). Empty tanks cut the throttle target.
    this.fuel = Math.max(
      0,
      this.fuel -
        ((FUEL_BURN_BASE +
          this.throttle * FUEL_BURN_THR +
          (this.failures.fuelLeak ? FUEL_LEAK_EXTRA : 0)) *
          dt) /
          100,
    );
    if (this.fuel <= 0 && this.throttle > 0) this.thrTarget = 0;

    // Fire state machine (damage-model §3). The failure-scenario
    // engine fire feeds it (failures.fire starts an engine fire if
    // none is burning); the damage-model fire (fuel/engine, from
    // applyDamage's ignition rolls) is the same machine. A fuel fire
    // ramps to full in ~12 s, an engine fire in ~4 s; fuel burns
    // 2.0 hp/s at full intensity, engine 1.2 hp/s. A fuel fire spreads
    // to one adjacent zone per ~8 s (a wing fire burning long enough
    // destroys the spar). Extinguish: a fuel fire's self-extinguish
    // chance rises with airspeed (2%/s below 150 m/s IAS, 8%/s above
    // 300 m/s); an engine fire has a 15%/s chance to go out when the
    // throttle is cut to 0. A fire that destroys the jet starts the
    // dying fall — it does not vanish in mid-air.
    if (this.failures.fire && !this.fireKind && this.alive && !this.dying) {
      this.startFire('engine', 'engineL');
    }
    if (this.fireKind && this.alive && !this.dying) {
      const ramp = this.fireKind === 'fuel' ? FUEL_FIRE_RAMP_S : ENGINE_FIRE_RAMP_S;
      this.fireIntensity = Math.min(1, this.fireIntensity + dt / ramp);
      const dps =
        this.fireKind === 'fuel' ? FUEL_FIRE_DPS : ENGINE_FIRE_DPS;
      const burn = dps * this.fireIntensity * dt;
      this.hp -= burn;
      // The fire eats the burning zone too (structural damage, so the
      // solver degrades and a wing fire can destroy the spar).
      const dz = this.damageZones;
      const zone = this.fireZone;
      if (zone === 'fuel') dz.fuel = clamp(dz.fuel + burn * 0.004, 0, 1);
      else if (zone === 'engineL' || zone === 'engine') dz.engineL = clamp(dz.engineL + burn * 0.005, 0, 1);
      else if (zone === 'engineR') dz.engineR = clamp(dz.engineR + burn * 0.005, 0, 1);
      // Spread (fuel fire only): one adjacent zone per ~8 s.
      if (this.fireKind === 'fuel') {
        this.fireSpreadTimer += dt;
        if (this.fireSpreadTimer >= FIRE_SPREAD_S) {
          this.fireSpreadTimer = 0;
          const adjacent: DamageZone[] = ['wingL', 'wingR', 'tail', 'engineL', 'engineR'];
          const spreadTo = adjacent[Math.floor(Math.random() * adjacent.length)]!;
          if (spreadTo === 'wingL') dz.wingL = clamp(dz.wingL + 0.15, 0, 1);
          else if (spreadTo === 'wingR') dz.wingR = clamp(dz.wingR + 0.15, 0, 1);
          else if (spreadTo === 'tail') dz.tail = clamp(dz.tail + 0.15, 0, 1);
          else if (spreadTo === 'engineL') dz.engineL = clamp(dz.engineL + 0.15, 0, 1);
          else dz.engineR = clamp(dz.engineR + 0.15, 0, 1);
        }
      }
      // A wing fire burning long enough destroys the spar.
      if ((dz.wingL > WING_LOSS_DMG || dz.wingR > WING_LOSS_DMG) && !this.wingDetached) {
        this.detachWing();
      }
      // Extinguish rolls (per-second probabilities, scaled by dt).
      let extP = 0;
      if (this.fireKind === 'fuel') {
        const t = clamp((this.ias - 150) / 150, 0, 1);
        extP = FUEL_FIRE_EXT_LO + (FUEL_FIRE_EXT_HI - FUEL_FIRE_EXT_LO) * t;
      } else if (this.throttle <= 0.05 && this.thrTarget <= 0.05) {
        extP = ENGINE_FIRE_EXT_THR0;
      }
      if (extP > 0 && Math.random() < extP * dt) {
        this.fireKind = null;
        this.fireIntensity = 0;
        this.fireZone = null;
      }
      if (this.hp <= 0) {
        this.startDying('fire');
        return;
      }
    }

    this.updateRig(dt, controls);
  }

  /** Decoy flares (template Aircraft.fireFlares): 6 sprites, 1.5 s
   *  cooldown, consumes one flare. The AI calls this directly; the
   *  visuals go through events.onFlares (main wires the sprites).
   *  Returns false when there is nothing to fire. NOTE: this is the
   *  bookkeeping path — the player's input path should use
   *  WeaponSystem.fireFlares() instead, never both on one press. */
  fireFlares(): boolean {
    if (!this.alive || this.flares <= 0 || this.flareCd > 0) return false;
    this.flares--;
    this.flareCd = 1.5;
    this.events.onFlares?.(this);
    return true;
  }

  /** Animate bound control surfaces / rotors (v2 updateRig via the rig). */
  updateRig(dt: number, controls: FlightControls): void {
    if (!this.rig || !this.modelReady) return;
    this.rig.update(dt, controls, {
      aoaDeg: this.aoaDeg,
      gear: this.gear,
      hook: this.hook,
      vectorDeg: this.vectorDeg,
      rotorRpm: 4 + this.throttle * 24,
    });
  }

  /** Start a fire (damage-model §3). Fuel fire ramps to full in ~12 s,
   *  engine fire in ~4 s. Called from applyDamage's ignition rolls
   *  and from the engine-fire failure. A fire already burning is not
   *  restarted (its intensity keeps ramping). */
  startFire(kind: 'fuel' | 'engine', zone: DamageZone): void {
    if (!this.alive || this.dying) return;
    if (this.fireKind) return;
    this.fireKind = kind;
    this.fireZone = zone;
    this.fireIntensity = 0;
    this.fireSpreadTimer = 0;
  }

  /** v2 damage(): zone multipliers, per-zone accumulators, wing loss,
   *  smoke/fire thresholds, kill credit. A grounded or spawn-protected
   *  PLAYER cannot be damaged (v1.1 — the AI should not be shooting at
   *  a parked jet); over-G calls with byPlayer=false while airborne
   *  are unaffected.
   *
   *  Damage model (§6 step 2): the hit routes through DamageableEntity
   *  (the single source of truth for per-zone HP, §7 table) AND the
   *  Aircraft's hp/damageZones (which feed the solver's degradation).
   *  A lost zone triggers its effect: fuel → ignition roll (fuel fire),
   *  engine → engine-fire roll, wing → detach, tail → controls dead
   *  (dying), fin → tail authority degraded. Severe damage (WT rule:
   *  fuselage / both engines / tail / total) or hp ≤ 0 starts the
   *  dying fall — the jet does not vanish in mid-air.
   *
   *  `deferKill` (WeaponSystem.applyProximityDamage only): a missile
   *  or bomb hits SEVERAL zones at once, and the kill must be decided
   *  AFTER all of them are applied — otherwise the first zone's kill
   *  (fuselage 40 HP destroyed → startDying) aborts the loop before
   *  the fuel/engine zones are hit and before any ignition roll runs,
   *  and the jet dies with hp exactly 60, no fire, no smoke, tails
   *  intact (the v2 live shoot-down: no visible damage at all). With
   *  deferKill the zone effects still apply (tail detaches, fires
   *  start) but the dying decision is left to checkKill(), which the
   *  caller runs once after the whole proximity burst. */
  applyDamage(
    amount: number,
    zone: DamageZone,
    byPlayer: boolean,
    deferKill = false,
  ): void {
    if (!this.alive || this.dying) return;
    this.lastHitByIsPlayer = byPlayer;
    this.lastHitByRecord = { isPlayer: byPlayer };
    if (this.isPlayer && (this.grounded || this.isSpawnProtected())) return;
    // Route through the entity (zone HP + severe-damage rule). Map the
    // v2 generic zones to per-side entity zones (a 'wing' hit damages
    // both wings; an 'engine' hit damages both engines). Single-engine
    // jets (F-35A) have one `engine` zone in the entity, not
    // engineL/engineR — a generic 'engine' hit routes there, and there
    // is NO second-side hit: the old code sent it to 'engineR', which
    // the entity fell back to the fuselage, so an engine hit destroyed
    // the fuselage instead of the engine (and the HUD ENG chip never
    // moved). Found by the damage-chips probe (issue 5).
    const singleEngine = this.damageEntity.maxHpFor('engine') > 0;
    const entityZone =
      zone === 'wing'
        ? 'wingL'
        : zone === 'engine'
          ? singleEngine
            ? 'engine'
            : 'engineL'
          : zone;
    const result = this.damageEntity.applyDamage(amount, entityZone, byPlayer);
    // A generic 'wing'/'engine' hit damages the other side too (the
    // entity only saw one side above) — check its zone loss as well.
    // Twin-engine only: a single-engine jet has no second engine.
    let lost2: string | null = null;
    if (zone === 'wing') {
      lost2 = this.damageEntity.applyDamage(amount, 'wingR', byPlayer).zoneLost;
    } else if (zone === 'engine' && !singleEngine) {
      lost2 = this.damageEntity.applyDamage(amount, 'engineR', byPlayer).zoneLost;
    }
    let mult = 1;
    if (zone === 'wing' || zone === 'wingL' || zone === 'wingR') mult = 1.2;
    else if (zone === 'engine' || zone === 'engineL' || zone === 'engineR') mult = 1.5;
    this.hp -= amount * mult;
    const dz = this.damageZones;
    if (zone === 'wing') {
      dz.wingL = clamp(dz.wingL + amount * 0.002, 0, 1);
      dz.wingR = clamp(dz.wingR + amount * 0.002, 0, 1);
    } else if (zone === 'wingL') dz.wingL = clamp(dz.wingL + amount * 0.004, 0, 1);
    else if (zone === 'wingR') dz.wingR = clamp(dz.wingR + amount * 0.004, 0, 1);
    else if (zone === 'engine') {
      dz.engineL = clamp(dz.engineL + amount * 0.0025, 0, 1);
      dz.engineR = clamp(dz.engineR + amount * 0.0025, 0, 1);
    } else if (zone === 'engineL') dz.engineL = clamp(dz.engineL + amount * 0.005, 0, 1);
    else if (zone === 'engineR') dz.engineR = clamp(dz.engineR + amount * 0.005, 0, 1);
    else if (zone === 'tail') dz.tail = clamp(dz.tail + amount * 0.004, 0, 1);
    else if (zone === 'fin') {
      dz.fin = clamp(dz.fin + amount * 0.004, 0, 1);
      // Losing the fin degrades directional stability; the solver has
      // no fin input, so it feeds the tail authority (handling
      // degrades without killing the jet outright, per §7).
      dz.tail = clamp(dz.tail + amount * 0.002, 0, 1);
    } else if (zone === 'fuel') {
      dz.fuel = clamp(dz.fuel + amount * 0.004, 0, 1);
      // A fuel hit leaks and raises the ignition chance; an incendiary
      // (gun API mix) hit ignites it ~5% of the time (BRL Memo 484:
      // AP 0-2%, API ~5%). The missile/bomb proximity path rolls its
      // own 25% fuel-explosion chance in WeaponSystem.
      if (Math.random() < 0.05) this.startFire('fuel', 'fuel');
    }
    // Zone-lost effects (first time a zone's HP hits zero).
    const lost = result.zoneLost ?? lost2;
    if (lost === 'fuel') this.startFire('fuel', 'fuel');
    else if (lost === 'engine' || lost === 'engineL' || lost === 'engineR') {
      // An engine destroyed by a hit has a chance to catch fire.
      if (Math.random() < 0.15) {
        this.startFire('engine', lost as DamageZone);
      }
    } else if (lost === 'tail') {
      // Tail (controls) gone — the tail section detaches visibly and
      // the jet is uncontrollable. With deferKill the caller decides
      // the kill after the whole burst (see above); without it, the
      // dying fall starts now.
      this.detachTail();
      if (!deferKill) {
        this.startDying('tail destroyed');
        return;
      }
    } else if (lost === 'fin') {
      dz.tail = Math.max(dz.tail, 0.6);
    }
    if ((dz.wingL > WING_LOSS_DMG || dz.wingR > WING_LOSS_DMG) && !this.wingDetached) {
      this.detachWing();
    }
    // Severe damage (WT rule) or hp exhausted → the dying fall, not an
    // instant vanish. The kill is credited at this moment (see
    // DamageableEntity.isSeverelyDamaged), not when the wreck hits.
    // Deferred when the caller is applying a multi-zone burst.
    if (!deferKill && (this.damageEntity.isSeverelyDamaged() || this.hp <= 0)) {
      this.startDying('shot down');
    }
  }

  /** Kill evaluation for a deferred multi-zone burst (WeaponSystem
   *  proximity damage): start the dying fall if the accumulated
   *  damage is severe (WT rule) or hp is exhausted. A no-op when the
   *  jet is already dying or dead. */
  checkKill(reason: string): void {
    if (!this.alive || this.dying) return;
    if (this.damageEntity.isSeverelyDamaged() || this.hp <= 0) {
      this.startDying(reason);
    }
  }

  /** Detach the worse-damaged wing as a tumbling segment (v2 detachWing:
   *  hide the matching mesh, spawn debris, toast on the player side).
   *  The mesh goes to onDetachSegment; with no wing node the event
   *  still fires with segment=null (CrashVFXManager box fallback). */
  detachWing(): void {
    if (this.wingDetached) return;
    this.wingDetached = true;
    const side =
      this.damageZones.wingL >= this.damageZones.wingR ? 'wing_L' : 'wing_R';
    const segment = this.rig
      ? this.rig.detachSegment(side as 'wing_L' | 'wing_R')
      : null;
    this.events.onDetachSegment?.(this, segment, this.pos.clone(), this.vel.clone());
  }

  /** Detach the tail as a tumbling segment (damage-model §6 step 4 —
   *  the rig already binds `tail_section` per its slot list). Called
   *  when the tail zone is destroyed (spar/fire rules), not only from
   *  a damage threshold. */
  detachTail(): void {
    if (this.tailDetached) return;
    this.tailDetached = true;
    const segment = this.rig ? this.rig.detachSegment('tail_section') : null;
    this.events.onDetachSegment?.(this, segment, this.pos.clone(), this.vel.clone());
  }

  /** Start the dying fall (damage-model §6 step 1). The jet is shot
   *  down: `alive` stays true for physics, but `dying` makes it
   *  untargetable (AI skips it, weapons skip it, lock breaks), zeroes
   *  the throttle, and records the death cause/killer. The aircraft
   *  keeps stepping with the solver running degraded — throttle 0,
   *  controls zeroed — so the stall/spin model, asymmetric roll and
   *  engine-out yaw produce the tumble naturally, and the controller's
   *  gravity + solver drag drive the fall to its terminal velocity.
   *  NOTE: Rapier gravity stays at scale 0 — the controller already
   *  adds `acc.y -= G`, so setting gravityScale = 1 would double-count
   *  gravity (the report's §6 step 1 assumed gravity lived in the
   *  solver's force sum; in this port it lives in the controller).
   *
   *  A combat kill must be VISIBLE (the v2 live shoot-down showed
   *  nothing: no fire, no smoke, tails intact, the jet gliding down
   *  gently). So a dying jet that is not already burning starts a
   *  fire at once — a fuel fire if the tank is lost or damaged, else
   *  an engine fire — at 0.7 intensity so flames show immediately
   *  instead of ramping up from nothing over 12 s. And if the tail
   *  zone is lost, the tail section detaches here too, so the
   *  "tail destroyed" text matches the model. The fire state machine
   *  does not run while dying (it is gated on !dying), so this fire
   *  burns all the way down — the smoke trail the spectate camera
   *  follows. */
  startDying(reason: string): void {
    if (!this.alive || this.dying) return;
    this.dying = true;
    this.dyingReason = reason;
    this.lastCrashReason = reason;
    this.thrTarget = 0;
    this.throttle = 0;
    this.pitchIn = 0;
    this.rollIn = 0;
    this.yawIn = 0;
    if (!this.fireKind) {
      if (
        this.damageEntity.isZoneLost('fuel') ||
        this.damageZones.fuel > 0.3
      ) {
        this.fireKind = 'fuel';
        this.fireZone = 'fuel';
      } else {
        this.fireKind = 'engine';
        // Single-engine jet: the one `engine` zone, not 'engineL'
        // (the old hardcoded 'engineL' put the dying fire on a zone
        // the F-35A does not have, so the HUD ENG chip never showed
        // it as burning).
        this.fireZone =
          this.damageEntity.maxHpFor('engine') > 0 ? 'engine' : 'engineL';
      }
      this.fireIntensity = 0.7;
      this.fireSpreadTimer = 0;
    }
    if (this.damageEntity.isZoneLost('tail')) this.detachTail();
    if (this.ai) this.ai.state = 'dead';
    this.events.onDying?.(this, reason);
  }

  /** Ground impact after a dying fall (damage-model §6 step 5). The
   *  explosion is scaled by the remaining fuel (a wreck with fuel
   *  left fireballs; without fuel it crumples), debris scatters, and
   *  only then does the aircraft become dead — handleDeaths (via
   *  onCrash) does the explosion/scorch at THIS (impact) position,
   *  not the position where the jet was shot down.
   *  Pass 6 (item 3): the wreck stays VISIBLE. The old code hid the
   *  group, so a terrain crash showed nothing — the jet vanished and
   *  the camera showed empty terrain. A wreck is left on the ground
   *  (it becomes the new jet on respawn, which restores it). The
   *  group's pose is synced to the impact position so a jet that flew
   *  into a hillside does not sit buried below the surface. */
  finishCrash(): void {
    if (!this.alive) return;
    this.crashedFromDying = true;
    this.alive = false;
    if (
      (this.damageZones.wingL > WING_DETACH_FATAL ||
        this.damageZones.wingR > WING_DETACH_FATAL) &&
      !this.wingDetached
    ) {
      this.detachWing();
    }
    this.group.position.copy(this.pos);
    this.group.quaternion.copy(this.quat);
    this.body.setEnabled(false);
    this.events.onCrash?.(this, this.dyingReason ?? 'crashed');
  }

  /** Instant vanish (v2 crash): hard landing / terrain impact when the
   *  jet was NOT already dying — it is already on the ground, so there
   *  is no fall to spectate. A dying jet never reaches here; its ground
   *  contact calls finishCrash() instead.
   *  Pass 5 (item 3): the v4 play-test's terrain crash showed NOTHING
   *  — the jet just vanished and the picker opened. Two causes: (a)
   *  the crash position is BELOW the terrain (the jet sank through the
   *  runway / into a hillside), so handleDeaths' explosion spawned
   *  underground and was hidden by the terrain, and the camera's
   *  spectate/chase target was below ground too. The wreck position
   *  is now clamped UP to the exact heightfield surface, so the
   *  explosion, scorch and debris all read at the surface. (b) an
   *  instant crash never engaged the spectate camera — main.ts
   *  handleDeaths now does that for the player. */
  crash(reason: string): void {
    if (!this.alive) return;
    this.alive = false;
    this.lastCrashReason = reason;
    // Pass 8 (item 1): snapshot the vertical speed and bank at the
    // crash moment for the debug spawn trace (vsi is the current
    // vertical speed; bank from the quaternion, YXZ like the
    // hard-landing check).
    this.lastCrashVs = this.vsi;
    this.lastCrashBankDeg =
      Math.abs(new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').z) *
      (180 / Math.PI);
    // Lift a below-surface wreck to the surface (terrain crash / sink
    // through a runway) so the death VFX and the camera target the
    // surface, not a point buried under it.
    const gh = this.terrainHeightAt(this.pos.x, this.pos.z);
    if (this.pos.y < gh) this.pos.y = gh;
    // Pass 8 (item 5): an instant crash tears the airframe apart. A
    // terrain / hard-landing impact arrives with no prior damage zones
    // set, so the old conditional detach (which needed wing damage
    // past the fatal threshold) never fired and the wreck looked
    // intact and undamaged — the live "intact jet, no fire" finding.
    // Detach a wing and the tail unconditionally: the impact does it.
    if (!this.wingDetached) this.detachWing();
    if (!this.tailDetached) this.detachTail();
    // Pass 6 (item 3): the wreck stays VISIBLE (see finishCrash) and
    // its pose is synced to the surface-clamped position, so a jet
    // that sank through a runway or into a hillside reads as wreckage
    // on the ground instead of vanishing.
    this.group.position.copy(this.pos);
    this.group.quaternion.copy(this.quat);
    this.body.setEnabled(false);
    if (this.ai) this.ai.state = 'dead';
    this.events.onCrash?.(this, reason);
  }

  /** Template-named alias of respawn(): place at (x, y, z) with heading
   *  hdgDeg. y is the absolute height (respawn adds gear height to the
   *  runway elevation instead, so this subtracts it back). */
  resetAt(x: number, y: number, z: number, hdgDeg: number): void {
    this.respawn({ x, z, elev: y - GEAR_HEIGHT, hdg: hdgDeg });
  }

  /**
   * Give an air-spawned aircraft a flying speed along its nose. Enemies
   * and traffic spawn in mid-air — a jet there is flying, not hovering
   * at 0 kt. Without this the jet spawns below stall speed, the wing
   * cannot hold 1 g, and no steering law can turn it; the AI bleeds
   * altitude and circles while it accelerates from zero. Call after
   * resetAt (respawn zeroes the body velocity). Ground spawns (runway,
   * deck) do not call this.
   */
  setAirspeed(vMs: number): void {
    const f = this.fwd();
    this.body.setLinvel({ x: f.x * vMs, y: 0, z: f.z * vMs }, true);
    this.vel.set(f.x * vMs, 0, f.z * vMs);
    this.ias = vMs;
  }

  /** Reset to a runway (v2 resetAt): repaired, refuelled, re-armed,
   *  5 s spawn protection for the player. The respawn timer is owned
   *  by main.ts (v1.1: tracked, so Restart cancels a pending respawn). */
  respawn(runway: RunwaySpawn): void {
    // Never spawn inside the terrain: the analytic runway elevation can
    // sit below the sampled mesh/collider height at plateau edges, and a
    // spawn inside the heightfield is what the first-Take-Off crash looked
    // like (jet falling nose-high from 5 ft before any throttle input).
    // Clamp the spawn elevation to the real ground height.
    const ground = this.terrainHeightAt
      ? this.terrainHeightAt(runway.x, runway.z)
      : runway.elev;
    const elev = Math.max(runway.elev, ground);
    this.pos.set(runway.x, elev + GEAR_HEIGHT, runway.z);
    this.vel.set(0, 0, 0);
    this.quat.setFromEuler(new THREE.Euler(0, runway.hdg * D2R, 0, 'YXZ'));
    this.throttle = 0;
    this.thrTarget = 0;
    this.pitchIn = 0;
    this.rollIn = 0;
    this.yawIn = 0;
    this.vtolCounted = false;
    this.fuel = 1;
    this.hp = 100;
    this.damageEntity.repair();
    this.damageZones.wingL = 0;
    this.damageZones.wingR = 0;
    this.damageZones.tail = 0;
    this.damageZones.engineL = 0;
    this.damageZones.engineR = 0;
    this.damageZones.fuel = 0;
    this.damageZones.fin = 0;
    this.fireKind = null;
    this.fireIntensity = 0;
    this.fireZone = null;
    this.fireSpreadTimer = 0;
    this.dying = false;
    this.dyingReason = null;
    this.crashedFromDying = false;
    this.alive = true;
    this.msl = 6;
    this.flares = 30;
    this.gunCd = 0;
    this.flareCd = 0;
    this.grounded = true;
    this.onDeck = false;
    this.onSurface = false;
    this.stalled = false;
    this.spin = 0;
    this.spinArrow = 0;
    this.betaDeg = 0;
    // The solver's own stall/spin hysteresis is NOT on the fields
    // above — reset it too, or a respawned jet inherits the wreck's
    // stall state and the STALL warning survives the restart (the
    // v2 play-test saw it survive three restarts).
    this.solver.reset?.();
    this._trapped = false;
    this.trapTimer = 0;
    this._prevDeckVel.set(0, 0, 0);
    this._prevDeckValid = false;
    this.wingDetached = false;
    this.tailDetached = false;
    this.lastHitByIsPlayer = false;
    this.lastHitByRecord = null;
    this.lastCrashReason = null;
    this.spawnProtectedUntil =
      this.simTime + (this.isPlayer ? SPAWN_PROTECTION_S : 0);
    this.bombs = this.spec.bombs ? 4 : 0;
    this.failures = {};
    this.flaps = 0;
    this.airbrake = 0;
    this.vectorDeg = 0;
    this.gear = true;
    this.hook = false;
    this.brake = false;
    this.aoaDeg = 0;
    this.gload = 1;
    this.vsi = 0;
    this.ias = 0;
    this.lockT = 0;
    this.lockTarget = null;
    this.group.visible = true;
    this.body.setEnabled(true);
    this.body.setTranslation(
      { x: this.pos.x, y: this.pos.y, z: this.pos.z },
      true,
    );
    this.body.setRotation(
      { x: this.quat.x, y: this.quat.y, z: this.quat.z, w: this.quat.w },
      true,
    );
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.syncFromBody();
  }

  /** Read the Rapier body into pos / vel / quat and the visual group.
   *  Also reads the body's angular velocity (world frame) into
   *  angVelBody (body frame) for the BET solver's v_local = v_cg +
   *  ω × r — the solver needs the true rate, including anything
   *  Rapier's contact solver or angular damping did to it. */
  private syncFromBody(): void {
    const t = this.body.translation();
    this.pos.set(t.x, t.y, t.z);
    const v = this.body.linvel();
    this.vel.set(v.x, v.y, v.z);
    const r = this.body.rotation();
    this.quat.set(r.x, r.y, r.z, r.w);
    const av = this.body.angvel();
    this.angVelBody.set(av.x, av.y, av.z).applyQuaternion(this.quat.clone().invert());
    this.group.position.copy(this.pos);
    this.group.quaternion.copy(this.quat);
  }
}
