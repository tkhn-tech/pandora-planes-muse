// AircraftAIController.ts — G5 AI (v2 template lines ~1333-1432).
// Same FlightControls as the player, no cheating: the controller returns
// steering + throttle and the caller feeds them to the physics step — it
// does NOT mutate the aircraft's control fields (v2's AI wrote ac.rollIn /
// ac.pitchIn / ac.thrTarget directly because it shared the Aircraft object;
// the TS port routes those through the returned FlightControls). The only
// direct aircraft call is fireFlares(), which is the aircraft's own method.
//
// Firing is NOT done here: v2 called aiGunShot / launchMissile (G6). This
// controller instead raises `fireGun` / `fireMissile` flags that the caller
// reads after update() and hands to the weapons module (which owns ammo,
// tracers and missiles). Kills/score are NOT decided here — DamageableEntity
// owns lastHitBy.
//
// Skill table, lead aiming, engage gating, hard floor and the traffic
// runway-hopping are all verbatim v2, including the two v1.1 fixes:
//  - never engage a grounded or spawn-protected player (a parked jet is not
//    a dogfight target);
//  - hard floor at ground + 60 m with pull-up (v1 AI flew into the terrain
//    chasing a ground target).
// One addition, flagged as such: the carrier deck is also treated as a floor
// (deckTop + 60 m), since v2's AI only avoided terrain and could fly
// through the carrier. The `carrier` parameter exists for that check.

import * as THREE from 'three';
import type { Aircraft } from '../physics/FlightPhysicsController';
import type { FlightControls } from '../physics/IAerodynamicsSolver';
import { sustainedTurnRate } from '../physics/IAerodynamicsSolver';
import type { HelicarrierController } from './HelicarrierController';
import { RUNWAYS, runwayDirection, type Runway } from '../world/RunwayPlacer';

export const SKILL_TABLE = {
  rookie: { lead: 0.0, react: 1.6, err: 0.35, burst: 0.5 },
  veteran: { lead: 0.5, react: 0.9, err: 0.18, burst: 0.9 },
  ace: { lead: 1.0, react: 0.45, err: 0.07, burst: 1.4 },
} as const;

export type SkillKey = keyof typeof SKILL_TABLE;
export type AiRole = 'enemy' | 'traffic';
export type AiState = 'patrol' | 'engage' | 'evade' | 'rtb' | 'bomb' | 'dead';

export interface AircraftAIOptions {
  /**
   * SCENE-frame ground height (PandoraWorldGenerator.groundHeight — it adds
   * the floating-origin shift). REQUIRED for the hard floor to mean
   * anything: without it the floor defaults to 0 and the AI will fly into
   * hills. The integrator must pass the world's groundHeight.
   */
  groundHeight?: (x: number, z: number) => number;
  /** Shared cumulative floating-origin shift (FloatingOrigin.cumulativeShift)
   *  — runways are stored in original-frame coords. */
  originShift?: THREE.Vector3;
  /** Decision RNG. Defaults to Math.random; AI decisions are not part of
   *  the deterministic world-gen sequence. */
  rng?: () => number;
}

export interface SpawnPose {
  key: string;
  position: THREE.Vector3;
  heading: number; // degrees
}

const R2D = 180 / Math.PI;

export class AircraftAIController {
  readonly aircraft: Aircraft;
  readonly role: AiRole;
  readonly skillKey: SkillKey;
  readonly skill: (typeof SKILL_TABLE)[SkillKey];
  state: AiState = 'patrol';
  navTarget = new THREE.Vector3();
  /** Set by update(): gun in cone + range this frame — caller fires it. */
  fireGun = false;
  /** Set by update(): missile launch this frame — caller fires it. */
  fireMissile = false;
  /** Set by update(): bomb release this frame — caller drops it. */
  dropBomb = false;

  private decideT = 0;
  private fireT = 0;
  private evadeT = 0;
  /** Throttle target is stateful (v2 integrates ac.thrTarget); kept here
   *  since the controller no longer writes the aircraft's field. */
  private thrTarget = 0.7;
  /** Smoothed turn rate (rad/s) for the HF two-loop bank command — the
   *  6/s turn-rate smoothing. */
  private omegaSm = 0;
  /** RTB landing (straight-in approach + 3.5 deg glide slope + rollout).
   *  HISTORY: this was a full HF pattern (entry -> downwind -> base ->
   *  final -> rollout), but the point-to-point steering never converged
   *  from an arbitrary post-dogfight position — the jet circled the
   *  pattern for 290 s without landing (probe). The failure modes,
   *  each fixed in turn and each insufficient alone: the throttle
   *  floor (0.35) let the jet accelerate away to 528 kt; the sink
   *  unload fired on the commanded RTB descent and cut the bank;
   *  the downwind speed (110 m/s) gave only a 4 deg/s turn; the
   *  sustained-turn cap collapsed to ~0 at idle throttle; the entry
   *  point forced a 180 deg reversal the steering could not complete.
   *  A damaged jet doing a straight-in is realistic (it does not fly
   *  a full pattern) and, crucially, it converges: the jet flies to
   *  ONE fix — the final-approach fix, 2 km out on the extended
   *  centreline at glide-slope altitude — from wherever it is, then
   *  descends the slope. No 180 deg reversal, no waypoint overshoot
   *  loop. The glide slope (3.5 deg, ~140 kt) and rollout are the
   *  same real numbers as the pattern's final leg. */
  private rtbPhase: 'approach' | 'final' | 'rollout' = 'approach';
  private rtbRunway: Runway | null = null;
  private readonly rtbApproach = new THREE.Vector3(); // final-approach fix
  private readonly rtbFinal = new THREE.Vector3(); // touchdown point
  private readonly rtbFinalGoal = new THREE.Vector3(); // glide-slope goal
  private readonly rtbRollout = new THREE.Vector3();
  /** Bombing run state (HF popup→dive→drop→pullout, 4 g level pull-out). */
  private bombState: 'none' | 'popup' | 'dive' | 'pullout' = 'none';
  private readonly bombTarget = new THREE.Vector3();
  private readonly bombPopup = new THREE.Vector3();
  private readonly bombGoalV = new THREE.Vector3();
  private readonly groundHeight: (x: number, z: number) => number;
  private readonly originShift: THREE.Vector3;
  private readonly rng: () => number;
  private readonly tmpA = new THREE.Vector3();
  private readonly tmpB = new THREE.Vector3();

  constructor(aircraft: Aircraft, skill: SkillKey, role: AiRole, opts: AircraftAIOptions = {}) {
    this.aircraft = aircraft;
    this.skillKey = skill;
    this.skill = SKILL_TABLE[skill];
    this.role = role;
    this.groundHeight = opts.groundHeight ?? (() => 0);
    this.originShift = opts.originShift ?? new THREE.Vector3();
    this.rng = opts.rng ?? Math.random;
    this.aircraft.ai = this; // v2: AIController constructor sets ac.ai
    this.pickNav();
  }

  pickNav(player?: Aircraft): void {
    if (this.role === 'traffic') {
      const r = RUNWAYS[Math.floor(this.rng() * RUNWAYS.length)];
      const alt = this.aircraft.spec.archetype === 'rotary' ? 120 + this.rng() * 80 : 260 + this.rng() * 150; // helis stay low
      this.navTarget.set(
        r.x - this.originShift.x + (this.rng() - 0.5) * 400,
        r.elev + alt,
        r.z - this.originShift.z + (this.rng() - 0.5) * 400,
      );
    } else {
      const ref = player ?? this.aircraft;
      this.navTarget
        .copy(ref.pos)
        .add(this.tmpA.set((this.rng() - 0.5) * 600, 150 + this.rng() * 300, (this.rng() - 0.5) * 600));
    }
  }

  update(dt: number, player: Aircraft, carrier: HelicarrierController, time: number): FlightControls {
    const ac = this.aircraft;
    this.fireGun = false;
    this.fireMissile = false;
    this.dropBomb = false;
    if (!ac.alive) {
      this.state = 'dead';
      return { pitchIn: 0, rollIn: 0, yawIn: 0, thrTarget: 0 };
    }
    // Dying: the jet is falling, not flying — stop steering and stop
    // firing (damage-model §6 step 7). The controller's stepInternal
    // also zeroes the controls, but the AI must not fire or navigate.
    if (ac.dying) {
      this.state = 'dead';
      return { pitchIn: 0, rollIn: 0, yawIn: 0, thrTarget: 0 };
    }

    // Decisions at ~10 Hz (v2).
    this.decideT -= dt;
    if (this.decideT <= 0) {
      this.decideT = 0.1 + this.rng() * 0.05;
      if (ac.hp < 20 && this.state !== 'rtb') this.state = 'rtb';
      else if (ac.hp < 35 && this.state === 'engage') {
        this.state = 'evade';
        this.evadeT = 3 + this.rng() * 2;
      } else if (this.role === 'enemy' && this.state !== 'evade' && this.state !== 'rtb') {
        // v1.1: never engage a grounded or spawn-protected player, and
        // never engage a dying (falling) one — it is untargetable.
        // Pass 6 (item 11): a slow / hovering target is not going
        // anywhere, so the AI commits from further away — the 2600 m
        // 3D range let a hovering F-35A sit unengaged while the AI
        // patrolled past it (the live test: 3 Veterans, 2.5 min,
        // "NO LOCK" the whole time). And once engaged it STAYS engaged
        // (hysteresis): the AI flies at a slow target at 90+ m/s,
        // overshoots, and is 3+ km out while it turns back — with a
        // single range it dropped to patrol on the overshoot and flew
        // away instead of re-attacking, so it never closed to firing
        // range. Disengage only beyond 5000 m (slow) / 4000 m (fast).
        const pFair =
          player.alive && !player.dying && !player.grounded && player.spawnProt <= 0;
        const slowTarget = player.ias < 50;
        const engageRange = slowTarget ? 3500 : 2600;
        const disengageRange = slowTarget ? 5000 : 4000;
        const dist = ac.pos.distanceTo(player.pos);
        if (this.state === 'engage') {
          this.state = pFair && dist < disengageRange ? 'engage' : 'patrol';
        } else {
          this.state = pFair && dist < engageRange ? 'engage' : 'patrol';
        }
      }
      if (this.state === 'patrol' || this.state === 'rtb') {
        if (this.rng() < 0.3) this.pickNav(player);
      }
      // Bombing run: a jet with bombs (Su-34) in patrol has a chance to
      // attack the player's runway instead of circling. Target is Alpha,
      // the player's home base.
      if (
        this.role === 'enemy' &&
        ac.bombs > 0 &&
        this.state === 'patrol' &&
        this.rng() < 0.25
      ) {
        this.state = 'bomb';
        this.bombState = 'none';
        const alpha = RUNWAYS[0]; // Alpha
        this.bombTarget.set(
          alpha.x - this.originShift.x,
          alpha.elev,
          alpha.z - this.originShift.z,
        );
      }
      // Leaving RTB / bomb resets the pattern / run so re-entering
      // recomputes it from the jet's current position.
      if (this.state !== 'rtb') {
        this.rtbRunway = null;
        this.rtbPhase = 'approach';
      }
      if (this.state !== 'bomb') this.bombState = 'none';
    }
    if (this.state === 'evade') {
      this.evadeT -= dt;
      if (this.evadeT <= 0) this.state = 'engage';
    }

    // Steering goal + wanted speed per state. RTB flies the landing
    // pattern (rtbGoal), the bomb run flies its phases (bombGoal);
    // engage/evade/patrol keep the v2 goals.
    let goal: THREE.Vector3 = this.navTarget;
    let speedWant = 130;
    let gearDown = false;
    let hookDown = false;
    let vsCap = 15; // vertical-speed demand cap (m/s); bomb dive raises it
    let pitchCap = 1; // pitch authority cap; bomb pull-out lowers it (4 g)
    if (this.state === 'bomb') {
      const bg = this.bombGoal(dt);
      goal = bg.goal;
      speedWant = bg.speed;
      vsCap = bg.vsCap;
      pitchCap = bg.pitchCap;
    } else if (this.state === 'rtb') {
      const rg = this.rtbGoal();
      goal = rg.goal;
      speedWant = rg.speed;
      gearDown = rg.gear;
    } else if (this.state === 'engage' && player.alive && !player.dying) {
      const lead = this.skill.lead;
      const tGo = ac.pos.distanceTo(player.pos) / 280;
      goal = this.tmpB.copy(player.pos).addScaledVector(player.vel, lead * tGo);
      speedWant = clamp(player.ias + 25, 90, 240);
    } else if (this.state === 'evade') {
      // v2: ac.pos + (player.pos - ac.pos) * -1 + (0, 120, 0) — away from
      // the player and climbing.
      goal = this.tmpB
        .copy(ac.pos)
        .addScaledVector(this.tmpA.copy(player.pos).sub(ac.pos), -1)
        .add(new THREE.Vector3(0, 120, 0));
      speedWant = 230;
    }
    const toGoal = this.tmpA.copy(goal).sub(ac.pos);
    const dist = toGoal.length();
    if (dist < 120 && this.state === 'patrol') this.pickNav(player);

    // ---- HF two-loop steering (bank from turn rate) ----
    // Loop 1: desired turn rate from the heading error, capped at the
    // sustained-turn envelope — the SAME sustainedTurnRate the curve
    // solver uses for its coordinated-yaw cap, so the AI and the aero
    // agree on what the jet can hold. Loop 2: bank from the turn rate,
    // phi = atan(V*omega/g) (HF coordinated bank, re-anchored to metres
    // and g = 9.81), clamped to +-75 deg, then rollIn from the bank
    // error. This replaces v2's rollIn = -local.x * 2.2, which banked
    // directly from the heading error and over-banked in tight turns.
    // Sign: heading increases to the RIGHT (atan2(fwd.x, fwd.z)), so a
    // positive heading error (goal to the right) needs a RIGHT bank
    // (negative phi, rollIn < 0 = bank right).
    const fwd = ac.fwd();
    const curHdg = Math.atan2(fwd.x, fwd.z);
    const desHdg = Math.atan2(toGoal.x, toGoal.z);
    let hdgErr = desHdg - curHdg;
    while (hdgErr > Math.PI) hdgErr -= 2 * Math.PI;
    while (hdgErr < -Math.PI) hdgErr += 2 * Math.PI;
    const S = ac.spec;
    const rho = 1.225 * Math.exp(-Math.max(ac.pos.y, 0) / 8500);
    // The sustained-turn cap uses the thrust AVAILABLE for the turn,
    // not the momentary throttle: during the RTB slowdown thrTarget
    // is 0 (idle + airbrake), and feeding 0 thrust into
    // sustainedTurnRate collapses omegaMax to the 0.05 floor, capping
    // the commanded turn at ~1.7 deg/s and the bank at ~14 deg — the
    // jet could not reverse onto the pattern (probe: 55 s to turn
    // 120 deg, 6 km out). Floor the thrust at 70% power for the cap:
    // the jet will spool up as the turn bleeds speed (the throttle
    // logic raises thrTarget when ias < want), and the 0.6 factor
    // below keeps the command inside the envelope.
    const omegaMax = Math.max(
      sustainedTurnRate(ac.ias, rho, S.thrustMax * Math.max(this.thrTarget, 0.7), S.wingArea, S.inducedK, S.mass),
      0.05,
    );
    // RTB turns are gentler when nearly aligned (0.35x the sustained
    // rate, gain 1.0), but FULL rate when far off heading: the 0.35x
    // cap alone turns at only ~2.2 deg/s, so an 84 deg alignment takes
    // 38 s and 2.7 km — the jet circles the fix instead of reaching
    // it (probe). When |hdgErr| > 60 deg the jet needs the full rate
    // to get roughly aligned; once within 60 deg the gentle cap
    // prevents the overshoot the 0.6x cap caused (a full 360 deg turn
    // around the downwind waypoint). The gain is 1.0 in RTB (1.5
    // elsewhere) so the commanded rate stays proportional over a
    // wider error range instead of saturating at max above ~3.5 deg.
    const omegaCapFrac =
      this.state === 'rtb'
        ? Math.abs(hdgErr) > (60 * Math.PI) / 180
          ? 0.6
          : 0.35
        : 0.6;
    const omegaGain = this.state === 'rtb' ? 1.0 : 1.5;
    const omegaDes = clamp(hdgErr * omegaGain, -omegaMax * omegaCapFrac, omegaMax * omegaCapFrac);
    // Turn-rate smoothing 6/s (HF lag); the 3/s heading lag is the
    // aircraft's own response (roll-rate limit), not a second filter.
    this.omegaSm += (omegaDes - this.omegaSm) * Math.min(1, dt * 6);
    // Bank cap is state-dependent: a transit (patrol / traffic) turn is
    // gentle (30 deg), a combat / pattern turn may use 45 deg. The
    // sustained rate at 150 m/s is a 75+ deg bank (a 4 g turn) and the
    // AI bled speed and departed commanding it; 60 deg (2 g) also
    // departed in the probe — the vs-pitch pulls to hold altitude in
    // the bank, the AoA overshoots past the stall, and the speed
    // collapses (MiG-29: 297 -> 43 kt, AoA 111 deg). 45 deg (1.41 g)
    // is sustainable with margin, and the 0.6 rate factor above keeps
    // the commanded turn inside the envelope the solver can actually
    // hold (the formula ignores separation drag, so the raw max is
    // optimistic).
    const bankCap =
      this.state === 'patrol' || this.role === 'traffic'
        ? (30 * Math.PI) / 180
        : (45 * Math.PI) / 180;
    let phiDes = clamp(
      -Math.atan((this.omegaSm * Math.max(ac.ias, 1)) / 9.81),
      -bankCap,
      bankCap,
    );
    // Sink unload: in a bank the vertical lift is lift·cos(phi), so a
    // hard-banked turn that cannot hold altitude sinks — and the
    // vs-pitch then pulls to climb, the AoA overshoots past the stall,
    // and the speed collapses (MiG-29 in the probe: 54 deg bank,
    // 293 -> 40 kt in 2 s, AoA 82 deg). When sinking fast, unload
    // (reduce the bank) instead of pulling harder — levelling the
    // wings restores vertical lift and the jet climbs out of the sink.
    // RTB FIX: this must NOT fire on a commanded descent. The RTB
    // downwind leg is 300 m while the jet is at 500+ m, so the
    // vs-pitch commands ~-8.6 m/s — and the old threshold (vsi < -8)
    // cut the bank from 45 deg to ~13 deg on every RTB descent,
    // crippling the turn (the probe showed a MiG-29 taking 55 s to
    // turn 120 deg and flying 6 km past the runway). The unload only
    // fires when the jet is sinking FASTER than commanded (vsi <
    // vsDes - 5 m/s) — i.e. it cannot hold the commanded vertical
    // speed — not when it is simply descending to the pattern as
    // commanded. An altitude threshold (goal 100 m below) was tried
    // and failed: once the jet is within 100 m of pattern altitude
    // the unload fired again on the final part of the descent.
    const vsDesHere = clamp((goal.y - ac.pos.y) * (this.state === 'rtb' ? 0.08 : 0.03), -vsCap, vsCap);
    if (ac.vsi < vsDesHere - 5) phiDes *= 0.3;
    const phiNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
    let rollIn = clamp((phiDes - phiNow) * 2.0, -1, 1);
    // v2 evade weave: sin(performance.now() * 0.004); time here is seconds,
    // so the equivalent angular frequency is 4 rad/s.
    if (this.state === 'evade') rollIn = clamp(rollIn + Math.sin(time * 4) * 0.5, -1, 1);

    // Pitch from vertical-speed error: vsDes from the goal's elevation,
    // pitchIn from (vsi - vsDes). pitchIn < 0 = nose up, so a climb
    // demand (vsDes > vsi) gives a negative pitchIn. Gains are LOW
    // (vsDes 0.03, pitch 0.03, authority capped at +-0.6): the
    // report's 0.08 gain porpoised — full nose-down to descend, then
    // full nose-up to arrest the descent — and tumbled the jet to
    // AoA +154 deg. The AI never needs full pitch authority. The cap
    // also respects pitchCap (the bomb pull-out's 0.3 for a 4 g pull).
    // vsDes gain is 0.03 normally (the 0.08 gain porpoised), but RTB
    // final needs a steeper descent to intercept the glide slope from
    // above: at 0.03 a jet 178 m above the slope only commands
    // -5.3 m/s and arrives at the touchdown ~90 m high, then flies
    // past and climbs away instead of landing (probe). In RTB final
    // the gain is 0.08 — the porpoising risk is lower there because
    // the goal rides the slope smoothly and the speed is low.
    // GLIDE-SLOPE FEEDFORWARD (issue 4, third pass): the goal rides
    // the slope, so it DESCENDS as the jet flies forward — a pure
    // proportional vsDes has a steady-state error of slopeRate/gain
    // (at 72 m/s the slope sinks 4.4 m/s, so at gain 0.08 the jet
    // flies 55 m above the slope and arrives at the touchdown ~90 m
    // high, then flies past). Feed the slope sink rate forward:
    // vsDes = (goal.y - pos.y) * gain - ias * tan(3.5 deg), so the
    // jet descends at the slope rate even when exactly on the slope
    // and the proportional term only corrects small errors.
    const vsGain = this.state === 'rtb' ? 0.08 : 0.03;
    const slopeSink =
      this.state === 'rtb' && this.rtbPhase === 'final'
        ? ac.ias * Math.tan((3.5 * Math.PI) / 180)
        : 0;
    const vsDes = clamp((goal.y - ac.pos.y) * vsGain - slopeSink, -vsCap, vsCap);
    const pitchLim = Math.min(pitchCap, 0.6);
    // Pitch gain 0.06 (was 0.03, pass 4): the 0.03 gain was tuned for
    // the curve solver's prescribed pitch rate (pitchIn * 55 deg/s),
    // where a -0.14 pitchIn gives -7.7 deg/s. Under BET the rate is
    // emergent from the elevator moment, so the same pitchIn gives
    // only ~1 deg/s and the jet cannot arrest a -13 m/s sink on the
    // RTB glide slope (it hit the terrain short of the runway).
    // Doubling the gain restores the pitch authority under BET; the
    // 0.08 gain porpoised under the curve solver, but BET's slower
    // emergent response damps the oscillation.
    let pitchIn = clamp((ac.vsi - vsDes) * 0.06, -pitchLim, pitchLim);

    // Rudder: centre the ball (sideslip beta) + adverse-yaw feedforward
    // when rolling. beta > 0 (velocity right of the nose) needs
    // nose-right rudder (yawIn > 0); rolling left (rollIn > 0) yaws the
    // nose right, so feed rudder left. v2 left yawIn at 0 always — the
    // AI never used the rudder. Rudder only in normal flight (AoA
    // within +-12 deg) and GENTLE (gain 0.02, cap +-0.25): a 10 deg
    // sideslip fed yawIn 0.50 at 315 kt, and the strong yaw departed
    // the jet (bank flipped 105 deg in 2 s, AoA 137 deg). The rudder
    // centres the ball; it is not a primary turn control.
    let yawIn = 0;
    if (!ac.stalled && ac.aoaDeg > -10 && ac.aoaDeg < 12) {
      yawIn = clamp(ac.betaDeg * 0.02 - rollIn * 0.15, -0.25, 0.25);
    }

    // AoA limiter (envelope protection): never pull into the stall, and
    // never dive so steeply the AoA goes past -10 deg. Past 12 deg AoA
    // the nose-up command is capped at level; past the stall the AI
    // pushes the nose down to recover. Without this the vs-based pitch
    // porpoises and stalls the jet into a spin. The limit is tighter
    // (10 deg) when banked > 30 deg — in a turn the optimal AoA for
    // the load factor is lower, and over-pulling there is what
    // departed the MiG-29 in the probe.
    const aoaLim =
      Math.abs(phiNow) > (30 * Math.PI) / 180 ? 10 : 12;
    if (ac.aoaDeg > aoaLim) pitchIn = Math.max(pitchIn, 0);
    if (ac.aoaDeg < -10) pitchIn = Math.min(pitchIn, 0);
    if (ac.stalled) pitchIn = Math.max(pitchIn, 0.5);

    // Throttle: speed management (Ace dives to regain energy — v2).
    const want = speedWant + (this.state === 'engage' && ac.ias < 90 ? 40 : 0);
    // Never throttle back while banked AND slow: a turn's induced drag
    // needs thrust, and throttling to 0.35 in a 70 deg bank bled a
    // MiG-29 from 292 kt to 93 kt in 4 s. But if the jet is well above
    // the wanted speed it may throttle back — holding 0.85 while fast
    // ran it away to 371 kt, where the high-q controls get twitchy and
    // the AoA spikes.
    // RTB FIX: when the jet is above the wanted speed (ias > want),
    // the throttle floor is IDLE (0), not 0.35. At 0.35 the thrust
    // still exceeds the clean drag at high speed, so the jet
    // ACCELERATES and can never slow to pattern speed — the RTB probe
    // showed a MiG-29 running away to 528 kt while trying to land.
    // A +30 m/s dead zone here trapped the jet at want+30 (272 kt):
    // the idle cut out exactly at the threshold and 0.35 thrust held
    // it there. So the floor is 0 whenever above want, and 0.35/0.85
    // only when at or below want (where it needs thrust to hold the
    // turn / speed).
    const bankNow = Math.abs(new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z);
    const aboveWant = ac.ias > want;
    // RTB final-phase floor (pass 4, BET promotion): on the glide
    // slope the jet needs power to stay on it — with the floor at 0
    // the AI cut to idle because it was 5 m/s fast, and under BET
    // the jet sank at -13 m/s (AoA 11-12 deg, high drag) and hit the
    // terrain short of the runway. The curve solver glides at idle;
    // BET does not. So in the RTB final phase the floor is 0.35
    // even when above want (the airbrake still bleeds the excess
    // speed); the 0 floor stays for the approach, where the jet must
    // slow from cruise to pattern speed.
    const rtbFinalFloor = this.state === 'rtb' && this.rtbPhase === 'final';
    const minThr = aboveWant
      ? (rtbFinalFloor ? 0.35 : 0)
      : bankNow > (15 * Math.PI) / 180 && ac.ias < want
        ? 0.85
        : 0.35;
    this.thrTarget = clamp(this.thrTarget + (ac.ias < want ? dt * 0.8 : -dt * 0.8), minThr, 1);
    if (ac.pos.y > 1400) this.thrTarget = Math.min(this.thrTarget, 0.5);
    // Airbrake (RTB fix, same root cause): when above the wanted
    // speed by more than 10 m/s the airbrake deploys — it adds CD 0.12
    // (~112 kN at 200 m/s), so the jet bleeds speed in seconds instead
    // of the ~70 s idle-throttle alone needs. Retracted once within
    // 10 m/s of the wanted speed. This is what lets a damaged jet
    // actually slow to the 140 kt pattern instead of arriving at the
    // runway at 500 kt.
    const airbrake = ac.ias > want + 10 ? 1 : 0;

    // Terrain avoidance: pull up if the ground is close ahead (v1.1:
    // stronger, plus a hard floor — v1 AI flew into the terrain chasing a
    // ground target).
    // RTB FIX (issue 4, third pass): do NOT apply terrain avoidance on
    // the RTB final/rollout — the jet is committed to landing on the
    // flat runway plateau, and the avoidance is what held it at ~130 m
    // and flew it past the touchdown instead of landing. The hard floor
    // (ground + 60 m) fires whenever the jet descends below 110 m over
    // the 50 m runway and commands full nose-up + 0.8 throttle; the
    // ahead check (ground within 80 m below, 260 m ahead) fires on
    // final too, because the runway itself is the ground ahead within
    // 80 m. The glide slope keeps the jet above the runway and the
    // plateau is flat, so there is no terrain to avoid on final.
    // In RTB approach (manoeuvring to the fix over rough terrain) and
    // in every other state the avoidance still applies.
    const landingFinal =
      this.state === 'rtb' && this.rtbPhase !== 'approach';
    if (!landingFinal) {
      const ahead = this.tmpB.copy(ac.pos).addScaledVector(ac.fwd(), 260);
      if (this.groundHeight(ahead.x, ahead.z) > ac.pos.y - 80) pitchIn = -1;
      const deckY = carrier.deckHeightAt(ac.pos.x, ac.pos.z);
      if (
        ac.pos.y < this.groundHeight(ac.pos.x, ac.pos.z) + 60 ||
        (deckY > -Infinity && ac.pos.y < deckY + 60)
      ) {
        pitchIn = -1;
        this.thrTarget = Math.max(this.thrTarget, 0.8);
      }
    }

    // Firing intent (v2 G6 block): gun in cone + range, missile when locked
    // — never at a grounded, spawn-protected or dying player (v1.1 +
    // damage-model §6: a dying jet is untargetable). Sets the
    // fireGun / fireMissile flags; the caller performs the shot.
    this.fireT -= dt;
    if (this.state === 'engage' && player.alive && !player.dying && !player.grounded && player.spawnProt <= 0 && this.fireT <= 0) {
      const toP = this.tmpB.copy(player.pos).sub(ac.pos);
      const d = toP.length();
      const ang = ac.fwd().angleTo(toP.normalize()) * R2D;
      // Pass 6 (item 11): the 12 deg gun / 15 deg missile cones are
      // too narrow for a slow / hovering target — the AI flies past
      // at 90+ m/s and its nose is inside the cone only for a moment,
      // so it never fires (live: "NO LOCK" for 2.5 min). A slow target
      // is not manoeuvring away, so the cones widen for it (gun 20
      // deg, missile 30 deg) and the missile reaches further (3000 m:
      // the AI orbits a slow target at 1.5-3 km and never closes to
      // the 2200 m envelope). A fast, manoeuvring target keeps the
      // tight cones and range.
      const slowTarget = player.ias < 50;
      const gunCone = slowTarget ? 20 : 12;
      const mslCone = slowTarget ? 30 : 15;
      const mslRange = slowTarget ? 3000 : 2200;
      if (d < 900 && ang < gunCone) {
        this.fireT = 1.2 / this.skill.burst;
        this.fireGun = true;
      }
      if (d < mslRange && ang < mslCone && ac.msl > 0 && this.rng() < 0.25) {
        this.fireT = 3;
        this.fireMissile = true;
      }
    }

    // Defensive flares when damaged or when the player is behind (v2).
    if ((ac.hp < 50 && this.rng() < dt * 0.3) || (this.state === 'engage' && playerBehind(ac, player))) {
      ac.fireFlares();
    }

    // Stall / spin recovery (NATOPS): if the jet is stalled or spinning,
    // recover first and do not chase the goal. Nose down to break the
    // stall (pitchIn > 0.1 is also the SpinModel's anti-spin condition),
    // wings level, full rudder opposite the spin (neutral rudder for a
    // stall without a spin). This overrides the terrain pull-up — a
    // stalled jet cannot climb out of the ground anyway, and holding
    // full pull through the stall is what built the spin to 1.0 and
    // kept it there.
    if (ac.stalled || ac.spin > 0.3) {
      pitchIn = 0.6;
      rollIn = clamp((0 - phiNow) * 2.0, -1, 1);
      yawIn = ac.spin > 0.3 ? -ac.spinDir * 1.0 : 0;
    }

    // RTB rollout braking: wheel brakes on the rollout so the jet
    // actually slows to a stop on the runway (issue 4). Idle throttle
    // + airbrake alone take too long to bleed 140 kt to a stop, and
    // the probe's landed criterion is grounded + ias < 10. The
    // controller only applies the brake when grounded, so setting it
    // in rollout is safe in the air.
    const brake = this.state === 'rtb' && this.rtbPhase === 'rollout';
    return {
      pitchIn,
      rollIn,
      yawIn,
      thrTarget: this.thrTarget,
      gear: gearDown,
      hook: hookDown,
      airbrake,
      brake,
    };
  }

  /**
   * RTB landing (straight-in): approach -> final -> rollout. The jet
   * flies to the final-approach fix (2 km out on the extended
   * centreline, at glide-slope altitude), then descends the 3.5 deg
   * glide slope at ~140 kt (72 m/s), gear down from the final leg,
   * rollout along the runway. See the rtbPhase comment for why the
   * full pattern was replaced: the point-to-point pattern steering
   * circled for 290 s without landing from a post-dogfight position.
   */
  private rtbGoal(): { goal: THREE.Vector3; speed: number; gear: boolean } {
    const ac = this.aircraft;
    if (!this.rtbRunway) {
      const r = this.nearestRunway(ac.pos);
      this.rtbRunway = r;
      this.rtbPhase = 'approach';
      const h = runwayDirection(r); // landing direction (unit)
      // Touchdown ~120 m past the threshold.
      this.rtbFinal
        .set(r.x - this.originShift.x, r.elev, r.z - this.originShift.z)
        .addScaledVector(h, -(r.len / 2 - 120));
      // Final-approach fix: 2 km out on the extended centreline, at
      // glide-slope altitude (2000 * tan(3.5 deg) = 122 m AGL). The
      // jet flies here from wherever it is, already aligned with the
      // runway, then descends the slope. 2 km gives room to align
      // and slow to 80 m/s before the descent.
      this.rtbApproach
        .copy(this.rtbFinal)
        .addScaledVector(h, -2000)
        .setY(r.elev + 2000 * Math.tan((3.5 * Math.PI) / 180));
      this.rtbRollout.copy(this.rtbFinal).addScaledVector(h, 500).setY(r.elev + 2);
    }
    const horiz = (a: THREE.Vector3, b: THREE.Vector3): number =>
      Math.hypot(a.x - b.x, a.z - b.z);
    switch (this.rtbPhase) {
      case 'approach':
        if (horiz(ac.pos, this.rtbApproach) < 300) this.rtbPhase = 'final';
        return { goal: this.rtbApproach, speed: 80, gear: false };
      case 'final': {
        // Final: the goal rides the 3.5 deg glide slope — its altitude
        // is the slope height at the jet's distance from the touchdown,
        // so the vs-based pitch holds the slope instead of diving at
        // the runway. Trap speed ~140 kt (72 m/s).
        const d = horiz(ac.pos, this.rtbFinal);
        // FLARE (issue 4, third pass): inside 300 m of the touchdown
        // the goal is the runway itself (touchdown + 2 m), not the
        // slope — a jet that drifted BELOW the slope would otherwise
        // be commanded to CLIMB back to it and float ~1 km past the
        // touchdown before touching down (probe). The flare goal
        // descends it onto the strip instead.
        if (d < 300) {
          this.rtbFinalGoal.set(this.rtbFinal.x, this.rtbFinal.y + 2, this.rtbFinal.z);
        } else {
          this.rtbFinalGoal.set(
            this.rtbFinal.x,
            this.rtbFinal.y + d * Math.tan((3.5 * Math.PI) / 180),
            this.rtbFinal.z,
          );
        }
        if (d < 100 && ac.pos.y < this.rtbFinal.y + 15) this.rtbPhase = 'rollout';
        return { goal: this.rtbFinalGoal, speed: 72, gear: true };
      }
      case 'rollout':
      default:
        // Rollout: the jet has landed and brakes to a STOP on the
        // runway (speed 0, not 30 — a damaged jet does not taxi off
        // at 58 kt; and at speedWant 30 the throttle spooled back up
        // whenever the brake got it below 30 m/s, so it never reached
        // the probe's landed criterion of ias < 10 and rolled off the
        // end of the runway). The brake (set in update()) + idle
        // throttle bring it to a stop on the strip.
        return { goal: this.rtbRollout, speed: 0, gear: true };
    }
  }

  /**
   * Bombing run (HF popup→dive→drop→pullout): popup to 800 m AGL 3 km
   * from the target, dive at the target, release when the ballistic
   * impact point (CCIP) is on the target, then a 4 g level pull-out
   * with wings level. The AI drops via the dropBomb flag (main.ts calls
   * weapons.dropBomb). Only for jets with bombs (Su-34).
   */
  private bombGoal(_dt: number): {
    goal: THREE.Vector3;
    speed: number;
    vsCap: number;
    pitchCap: number;
  } {
    const ac = this.aircraft;
    if (this.bombState === 'none') {
      // Popup: 3 km from the target, 800 m AGL, on the side the jet is
      // already on so it does not have to cross the target.
      const dir = this.tmpA.copy(ac.pos).sub(this.bombTarget).setY(0);
      if (dir.lengthSq() < 1) dir.set(1, 0, 0);
      dir.normalize();
      this.bombPopup
        .copy(this.bombTarget)
        .addScaledVector(dir, 3000)
        .setY(this.bombTarget.y + 800);
      this.bombState = 'popup';
    }
    switch (this.bombState) {
      case 'popup': {
        const d = Math.hypot(ac.pos.x - this.bombPopup.x, ac.pos.z - this.bombPopup.z);
        if (d < 300) this.bombState = 'dive';
        return { goal: this.bombPopup, speed: 150, vsCap: 15, pitchCap: 1 };
      }
      case 'dive': {
        // Dive at the target; the steep vsCap lets the vs-based pitch
        // drive a real dive instead of a 15 m/s descent.
        if (this.bombReleaseNow()) {
          this.dropBomb = true;
          this.bombState = 'pullout';
        }
        return { goal: this.bombTarget, speed: 180, vsCap: 80, pitchCap: 1 };
      }
      case 'pullout': {
        // 4 g level pull-out, wings level: pitch authority capped so
        // the pull is ~4 g, not the full 55 deg/s (which is ~18 g at
        // 180 m/s). Goal is ahead and climbing to 500 m; the generic
        // steering pulls up toward it.
        this.bombGoalV
          .copy(ac.pos)
          .addScaledVector(ac.fwd(), 800)
          .setY(Math.max(ac.pos.y + 100, this.bombTarget.y + 500));
        const pitch = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x;
        if (Math.abs(pitch) < 0.1 && ac.pos.y > this.bombTarget.y + 400) {
          this.bombState = 'none';
          this.state = 'patrol';
        }
        return { goal: this.bombGoalV, speed: 180, vsCap: 15, pitchCap: 0.3 };
      }
      default:
        return { goal: this.bombTarget, speed: 180, vsCap: 80, pitchCap: 1 };
    }
  }

  /**
   * CCIP release cue: simulate the bomb's ballistic fall from the jet's
   * position and velocity; release when the impact point is within
   * 150 m of the target.
   */
  private bombReleaseNow(): boolean {
    const ac = this.aircraft;
    const h = ac.pos.y - this.bombTarget.y;
    if (h <= 0) return true; // at/below target height — release now
    const vy = ac.vel.y;
    // Time to fall h: h = -vy*t + 0.5*g*t^2 -> t = (vy + sqrt(vy^2 + 2gh))/g.
    const t = (vy + Math.sqrt(vy * vy + 2 * 9.81 * h)) / 9.81;
    const ix = ac.pos.x + ac.vel.x * t;
    const iz = ac.pos.z + ac.vel.z * t;
    return Math.hypot(ix - this.bombTarget.x, iz - this.bombTarget.z) < 150;
  }

  private nearestRunway(p: THREE.Vector3): Runway {
    // p is in scene coords; RUNWAYS are original-frame — convert through
    // originShift (v2).
    const wx = p.x + this.originShift.x;
    const wz = p.z + this.originShift.z;
    let best = RUNWAYS[0];
    let bd = Infinity;
    for (const r of RUNWAYS) {
      const d = (wx - r.x) ** 2 + (wz - r.z) ** 2;
      if (d < bd) {
        bd = d;
        best = r;
      }
    }
    return best;
  }

  /**
   * v2 general enemy spawn (startRound, i >= 2): random bearing, distance
   * minDist..maxDist from the player, 650-950 m altitude. The caller
   * constructs the Aircraft + AIController from the returned pose. The
   * first-two-enemies forward-view spawn (450-600 m ahead of Alpha) is a
   * start-of-round placement, not a respawn — the game manager owns it.
   */
  static spawnEnemyNear(player: Aircraft, minDist: number, maxDist: number, rng: () => number = Math.random): SpawnPose {
    const a = rng() * Math.PI * 2;
    const d = minDist + rng() * (maxDist - minDist);
    return {
      key: rng() < 0.5 ? 'mig29' : 'su34',
      position: new THREE.Vector3(player.pos.x + Math.cos(a) * d, 650 + rng() * 350, player.pos.z + Math.sin(a) * d),
      heading: rng() * 360,
    };
  }

  /**
   * v2 traffic spawn (startRound): jets at runway + 220 m, helis at runway +
   * 150 m, heading along the runway. Key cycles fa18 / mig29 / heli (every
   * 3rd traffic is the rotary UH-60).
   */
  static spawnTraffic(index = 0, rng: () => number = Math.random): SpawnPose {
    const key = index % 3 === 2 ? 'heli' : index % 2 === 0 ? 'fa18' : 'mig29';
    const r = RUNWAYS[index % RUNWAYS.length];
    const alt = key === 'heli' ? 150 : 220;
    return { key, position: new THREE.Vector3(r.x, r.elev + alt, r.z), heading: r.hdg };
  }
}

function playerBehind(ac: Aircraft, player: Aircraft): boolean {
  if (!player.alive) return false;
  const toAc = ac.pos.clone().sub(player.pos).normalize();
  return player.fwd().dot(toAc) > 0.85 && player.pos.distanceTo(ac.pos) < 700;
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}
