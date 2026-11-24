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
  /** Last steering outputs, for probes / debugging (read-only). */
  lastPitchIn = 0;
  lastRollIn = 0;
  lastVsDes = 0;
  lastTerrainTurn = 0;

  private decideT = 0;
  /** Gun cooldown (s); the missile has its own (below). Pass 12:
   *  one shared fireT let a missile launch (3 s) suppress the gun
   *  through the whole 900 m head-on crossing — the AI fired its
   *  missile at ~1800 m and coasted through gun range unable to
   *  shoot. The gun and the missile are independent weapons. */
  private fireGunT = 0;
  /** Missile cooldown (s). */
  private fireMslT = 0;
  private evadeT = 0;
  /** Throttle target is stateful (v2 integrates ac.thrTarget); kept here
   *  since the controller no longer writes the aircraft's field. */
  private thrTarget = 0.7;
  /** Smoothed turn rate (rad/s) for the HF two-loop bank command — the
   *  6/s turn-rate smoothing. */
  private omegaSm = 0;
  /** Pass 15: previous frame's AoA (deg) and smoothed AoA rate
   *  (deg/s) for the AoA-rate damper. */
  private prevAoA = 0;
  private aoaRateSm = 0;
  /** Pass 15: SIM-time clock, accumulated from dt. The `time`
   *  argument update() receives is the caller's `elapsed` — REAL
   *  time in the live loop (main.ts advances elapsed by rawDt and
   *  caps sim time at 0.1 s/frame, so at 4 fps elapsed runs 2.5x
   *  sim time). The AI's internal rhythms (loiter orbit, evade
   *  weave) must run on SIM time: on real time the loiter orbit
   *  point moves at 0.08*800 = 64 m/s real = 160 m/s sim, while
   *  the loitering jet flies at 100 m/s sim — it can never catch
   *  its orbit point, drifts 3 km from the player during a slow
   *  human takeoff, and flies into terrain chasing it (delayed60:
   *  orbit lag 2956 m, terrain death at 26 s sim, never engages —
   *  the v4.9 live symptom exactly). Accumulating dt makes the
   *  orbit move at 64 m/s sim, which the 100 m/s loiter holds. */
  private simTime = 0;
  /** Pass 15: previous frame's goal-driven vsDes (m/s), for the
   *  vsDes slew limit below. */
  private prevVsDes = 0;
  /** Pass 15: loiter-orbit anchor. The orbit point is anchored to
   *  the jet's HEADING when it first loiters (see the orbit block):
   *  the point starts 800 m from the player along the jet's line of
   *  flight, and loiterT is the sim time spent loitering (the orbit
   *  advances only while loitering, so a fight does not move it). */
  private loiterAnchorA: number | null = null;
  private loiterT = 0;
  /** Pass 14: latched terrain-avoidance turn side (+0.6 left,
   *  -0.6 right, 0 none) — see the turn block for why it latches. */
  private terrainTurnLatch = 0;
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
      // Pass 14: terrain floor on the patrol point. The point is
      // ref.y + 150-450 m, so when the player is low (50-200 m)
      // the point sits at 200-500 m — but the hills rise to ~550 m,
      // so the jet descends to a point BELOW a ridge and then
      // cannot climb out when the ridge rises (e2 pursuit t=41-48:
      // descending -4..-10 m/s to a ~480 m patrol point over a
      // 365 m ridge, dead at 54.6 s). Keep the point at least
      // 200 m above the terrain at its (x, z) so the patrol leg
      // itself is terrain-safe; the intercept/engage still dive to
      // the player, so the fight is unaffected.
      this.navTarget.y = Math.max(
        this.navTarget.y,
        this.groundHeight(this.navTarget.x, this.navTarget.z) + 200,
      );
    }
  }

  // `time` is the caller's elapsed clock (real time live); it is
  // deliberately unused — the AI's rhythms run on this.simTime
  // (accumulated from dt above). Kept in the signature so callers
  // (main.ts, probes) do not change.
  update(dt: number, player: Aircraft, carrier: HelicarrierController, _time: number): FlightControls {
    const ac = this.aircraft;
    // Pass 15: sim-time clock (see the simTime field note). `time`
    // is the caller's real-time elapsed; the AI's internal rhythms
    // below use this.simTime instead.
    this.simTime += dt;
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
      // Pass 8 (item 2): hoisted to the decide block so the bomb-entry
      // gate below can use it too (it was scoped to the engage branch).
      const pFair =
        player.alive && !player.dying && !player.grounded && player.spawnProt <= 0;
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
        // Pass 12 (Worker E): the fast disengage range is 6000 m,
        // not 4000 m — after a head-on pass the AI needs ~18 s to
        // reverse onto the player's tail, during which the player
        // extends ~2.7 km; at 4000 m it dropped to patrol mid-turn
        // and flew away instead of re-attacking. The engage range
        // is unchanged (2600/3500 m), so the hysteresis is wider,
        // not the trigger.
        const slowTarget = player.ias < 50;
        const engageRange = slowTarget ? 3500 : 2600;
        const disengageRange = slowTarget ? 5000 : 6000;
        const dist = ac.pos.distanceTo(player.pos);
        if (this.state === 'engage') {
          this.state = pFair && dist < disengageRange ? 'engage' : 'patrol';
        } else {
          this.state = pFair && dist < engageRange ? 'engage' : 'patrol';
        }
      }
      if (this.state === 'patrol' || this.state === 'rtb') {
        // Pass 13: the timer re-pick is traffic-only now. For an
        // enemy it made the patrol goal jump +/-300 m around the
        // player every ~0.33 s, so the jet chased a jumping goal and
        // wandered away from the player instead of orbiting him (the
        // delayed1v1 drift: 479 m -> 3,690 m during a 25 s ground
        // hold). An enemy re-picks on ARRIVAL (the dist < 120 rule
        // below) or not at all — in intercept it does not use
        // navTarget anyway.
        if (this.role === 'traffic' && this.rng() < 0.3) this.pickNav(player);
      }
      // Pass 12 (Worker E): a Su-34 in patrol entered a bombing run
      // on a 25% roll per ~0.1 s decide tick — i.e. within ~0.5 s of
      // being in patrol with a fair player — so both Su-34s spent
      // most of a round 10-20 km away bombing Alpha instead of
      // fighting (probe-ai-live baseline: 107 of 960 enemy-seconds
      // in 'bomb', entered while the player was 1-2 km away). A
      // bombing run is only a fallback when there is no dogfight to
      // join: the player must be beyond engage range. And a bomb
      // run ABORTS back to engage when the player closes inside
      // engage range — the run must not preempt a fight.
      const distToPlayer = ac.pos.distanceTo(player.pos);
      const slowTgt = player.ias < 50;
      const engageR = slowTgt ? 3500 : 2600;
      // Pass 13: an intercepting enemy (patrol + fair player) does
      // not leave the fight to bomb — see the header note. A bomb run
      // from 10 km out is the same "enemy never fights" failure.
      const interceptingNow =
        this.role === 'enemy' && this.state === 'patrol' && pFair;
      if (
        this.role === 'enemy' &&
        ac.bombs > 0 &&
        this.state === 'patrol' &&
        pFair &&
        !interceptingNow &&
        distToPlayer > engageR &&
        this.rng() < 0.05
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
      if (this.state === 'bomb' && pFair && distToPlayer < engageR) {
        this.state = 'engage';
        this.bombState = 'none';
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
    // Pass 13 (Worker D): the live 1v1 failure — patrol is a dead end.
//
// Measured in probe-ai-live --scenario cruise1v1 / delayed1v1 (F-35A,
// 1 MiG-29, human cruise, live wind/variable dt/origin shift): the
// enemy NEVER fights. The chain, all measured:
//  1. While the player is on the ground the enemy patrols AWAY: its
//     navTarget is re-picked around the player every ~0.33 s with
//     +/-300 m jitter at 130 m/s, so it wanders off instead of
//     orbiting — in delayed1v1 it goes from 479 m to 3,690 m away
//     during a 25 s ground hold, nose 157 deg away from the player.
//     By takeoff it is beyond the 2600 m engage range.
//  2. Patrol can never re-close: patrol speedWant is 130 m/s vs a
//     150-200 m/s cruising player. The enemy's nose sat 2-10 deg on
//     the player for 240 s straight and the gap STILL grew ~65 m/s
//     to 21-29 km. The player sees NO LOCK for the whole round
//     (maxLockT 0.00 s over 300 s — the live 4:49 symptom exactly).
//  3. The one head-on pass is wasted (min dist 124 m, 0 shots: the
//     pass is over in ~1 s with the nose already 166 deg off) and the
//     engage reversal is too slow to matter (45 deg bank cap beyond
//     1200 m turns at ~4 deg/s; the gap grows to 5.2 km and the AI
//     disengages at 6 km).
// The fix: an enemy in patrol with a fair (airborne) player flies an
// INTERCEPT — lead pursuit at closing speed — instead of a navTarget
// transit; an enemy whose player is not fair yet LOITERS near the
// player at 100 m/s instead of transiting away; and the engage bank
// cap opens to 70 deg for a big heading error (a reversal) at any
// range. Firing stays engage-gated, so patrol still never shoots.
// Side effect, stated plainly: the Su-34 bomb entry is now gated on
// NOT intercepting, and an intercepting enemy is by definition in
// patrol with a fair player — so while the player is airborne a
// Su-34 no longer leaves the fight to bomb Alpha. The bomb run was
// already gated to ~11 s of 960 enemy-seconds by pass 12's abort
// rule; the intercept takes precedence over the remaining sliver,
// because a jet 10 km out that bombs instead of closing is the same
// "enemy never fights" failure from the player's side.

    // Heading now (used by the engage corner-speed rule below and
    // by the steering section; computed once).
    const fwd = ac.fwd();
    const curHdg = Math.atan2(fwd.x, fwd.z);
    // Pass 13: fair-player test (same as pFair in the decide block)
    // and the intercept flag. An enemy in patrol with a fair player
    // is INTERCEPTING — it flies to the player, not to a navTarget.
    const fairPlayer =
      player.alive && !player.dying && !player.grounded && player.spawnProt <= 0;
    const intercepting =
      this.state === 'patrol' && this.role === 'enemy' && fairPlayer;
    // Pass 13: loitering (see the orbit block below) — an enemy in
    // patrol whose player is not fair yet.
    const loitering =
      this.state === 'patrol' && this.role === 'enemy' && !fairPlayer;
    // Pass 15: leaving the loiter resets the orbit anchor, so
    // re-entering it re-anchors to the jet's then-current heading.
    if (!loitering) {
      this.loiterAnchorA = null;
      this.loiterT = 0;
    }
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
    } else if (intercepting) {
      // Pass 13: INTERCEPT transit. Lead pursuit of the player at
      // closing speed (player speed + 60, floor 200, cap 280 — a
      // real intercept, full throttle while far), with the engage
      // corner-speed rule (slow to 120 m/s in a hard turn, where the
      // 70 deg bank below turns at ~22 deg/s instead of ~2 deg/s at
      // 280 m/s). Climb/dive authority matches engage (vsCap 30) so
      // a climbing player cannot out-climb the interceptor. The
      // engage state still triggers at 2600 m and takes over with
      // player speed + 25 from there; firing stays engage-gated, so
      // this transit never shoots — it only delivers the jet to the
      // fight. Without it, a patrolling enemy astern of a cruising
      // player at 130 m/s never closes (probe-ai-live cruise1v1:
      // nose 2-10 deg on the player for 240 s, gap +65 m/s to 21 km).
      const distToP = ac.pos.distanceTo(player.pos);
      const lead = this.skill.lead * clamp(distToP / 2000, 0, 1);
      const tGo = distToP / 280;
      goal = this.tmpB.copy(player.pos).addScaledVector(player.vel, lead * tGo);
      // Pass 14: the 280 m/s cap is far below the airframes (level
      // max from the solver drag, CD0 0.028: MiG-29 ~458 m/s at
      // 163 kN, Su-34 ~497 m/s at 264 kN) and below a fast player —
      // the live F-22 cruised at ~360 m/s (700 kt), so a 280 m/s
      // interceptor could never close a tail chase even flying
      // perfectly. The cap is now 380 m/s: above the fastest
      // player's cruise with margin to close, inside the airframe
      // envelope, and the corner-speed rule below still slows the
      // jet for a hard turn.
      // Pass 15: the +60 m/s closing margin is too slow to re-close
      // after ANY separation. Measured (probe-ai-live15 cruise1v1):
      // after the player is shot down and respawns, the enemy sits
      // 12-17 km behind and closes at ~60 m/s — 200+ s sim (8+ min
      // real at 4 fps) to re-engage, so the fight is over after the
      // first kill. +120 m/s halves that (15 km in ~125 s sim); the
      // corner-speed rule below still caps the speed when the
      // heading error is big, and engage takes over inside 2600 m
      // with player.ias+25, so the merge geometry is unchanged.
      speedWant = clamp(player.ias + 120, 200, 380);
      vsCap = 30;
      const desHdgHere = Math.atan2(goal.x - ac.pos.x, goal.z - ac.pos.z);
      let hdgErrHere = desHdgHere - curHdg;
      while (hdgErrHere > Math.PI) hdgErrHere -= 2 * Math.PI;
      while (hdgErrHere < -Math.PI) hdgErrHere += 2 * Math.PI;
      // Pass 13: range-dependent corner-speed cap (see the engage
      // branch — a flat 120 pinned the interceptor at 120 m/s for a
      // whole tail chase against a faster player).
      if (Math.abs(hdgErrHere) > (30 * Math.PI) / 180) {
        speedWant = Math.min(speedWant, 120 + distToP * 0.03);
      }
    } else if (this.state === 'engage' && player.alive && !player.dying) {
      const distToP = ac.pos.distanceTo(player.pos);
      // Pass 12 (Worker E): the steering lead blends out at close
      // range. At full lead (veteran 0.5, tGo = dist/280) the goal
      // sits ~15 deg off the player in a crossing geometry at EVERY
      // range — the nose tracked the intercept point, never the
      // player, so the 12 deg gun cone never closed even at 133 m
      // (probe-ai-live: nose 60-97 deg off while circling at
      // ~1300 m, chasing a lead point that itself moves at 150 m/s).
      // Beyond 2000 m the full lead stands (a real intercept); inside
      // it the goal becomes the player itself — pure pursuit, nose on
      // the target for the gun. The skill lead still differentiates
      // the long intercept.
      const leadScale = clamp(distToP / 2000, 0, 1);
      const lead = this.skill.lead * leadScale;
      const tGo = distToP / 280;
      goal = this.tmpB.copy(player.pos).addScaledVector(player.vel, lead * tGo);
      // Pass 12 (Worker E): the speed cap was 240 m/s — a fast player
      // (F-22 at full throttle reaches 270+ m/s) outran the AI in a
      // tail chase, so the AI only ever closed head-on. 280 m/s lets
      // it match and close on a fast player; the want is still only
      // player speed + 25 m/s, so it does not overshoot a slow one.
      // Pass 14: 380 m/s cap, as in the intercept branch above — a
      // 280 m/s AI cannot close on a 360 m/s player in a tail chase
      // even in pure pursuit (player.ias + 25 was capped below the
      // player's speed, so the gap grew in engage and the AI
      // disengaged at 6000 m mid-turn).
      speedWant = clamp(player.ias + 25, 90, 380);
      // Pass 12 (Worker E): corner speed. A 70 deg bank at 175 m/s
      // turns at ~9 deg/s, barely above the line-of-sight rate of a
      // crossing player at 1300 m (~7 deg/s) — the AI circled the
      // player for 30+ s with its nose 60-97 deg off, never settling.
      // At 120 m/s the same bank turns at ~13 deg/s and the bearing
      // rate falls too, so the nose catches the goal. When the turn
      // is big (> 60 deg of heading error) the AI slows to corner
      // speed; when aligned it accelerates back to player speed + 25.
      const desHdgHere = Math.atan2(goal.x - ac.pos.x, goal.z - ac.pos.z);
      let hdgErrHere = desHdgHere - curHdg;
      while (hdgErrHere > Math.PI) hdgErrHere -= 2 * Math.PI;
      while (hdgErrHere < -Math.PI) hdgErrHere += 2 * Math.PI;
      // Pass 12 (Worker E): the 30 deg threshold, not 60 — the AI
      // converges on the target's bearing at only ~3 deg/s at
      // 175 m/s, so it spent 20-30 s per pass circling with its nose
      // 30-70 deg off (probe-ai-live PASS trace). Corner speed
      // applies for most of the approach, not just the reversal.
      // Pass 13 (Worker D): the cap is RANGE-dependent (was a flat
      // 120). A flat 120 pinned the jet at 120 m/s for the whole tail
      // chase against a 200 m/s player — the heading error never
      // drops below 30 deg while chasing a faster target from behind,
      // so the gap grew even in engage and the nose never closed
      // (cruise1v1: 102 s in engage, 0 shots, enemy out-climbed to
      // 7284 m). The turn rate the cap buys matters at CLOSE range,
      // where the bearing rate is high; far away a modest turn rate
      // suffices and the jet must keep its speed to close. 120 m/s at
      // the merge, rising 30 m/s per km of range.
      if (Math.abs(hdgErrHere) > (30 * Math.PI) / 180) {
        speedWant = Math.min(speedWant, 120 + distToP * 0.03);
      }
      // Pass 12 (Worker E): climb/dive authority in a dogfight.
      // vsCap 15 m/s let a player climbing at 20-40 m/s (routine for
      // a fighter at full throttle) out-climb the AI forever — the
      // AI's nose sat below the player and the fire cones never
      // closed. 30 m/s follows a climbing/diving player; the hard
      // floor and terrain pull-up still apply.
      vsCap = 30;
    } else if (this.state === 'evade') {
      // v2: ac.pos + (player.pos - ac.pos) * -1 + (0, 120, 0) — away from
      // the player and climbing.
      goal = this.tmpB
        .copy(ac.pos)
        .addScaledVector(this.tmpA.copy(player.pos).sub(ac.pos), -1)
        .add(new THREE.Vector3(0, 120, 0));
      speedWant = 230;
    }
    // Pass 13: LOITER (orbit). An enemy whose player is not a fair
    // target yet (on the ground, spawn-protected, dying) ORBITS the
    // player — a point 800 m out, circling at 0.08 rad/s, 500 m above
    // him — instead of flying to a fixed navTarget. The fixed-point
    // version failed in delayed1v1: the goal sat near the enemy's own
    // spawn, BEHIND its westbound heading, and at the 30 deg patrol
    // bank cap the 180 deg reversal takes ~55 s — the jet flew 3.7 km
    // west into the rim mountains, terrain-avoidance zoom-climbed it
    // to 1842 m, and it stalled to 56 m/s (108 kt). An orbit point
    // moves smoothly, so the jet never needs a reversal: it stays
    // within ~1 km of the player, inside the 2600 m engage range the
    // moment he goes airborne, co-altitude, already turning. Speed
    // 100 m/s (just above 1.5x the MiG-29's stall, so the low-energy
    // bank cap does not fight the orbit turn) and a 55 deg loiter
    // bank cap below (the orbit needs ~39 deg at 100 m/s).
    if (this.state === 'patrol' && this.role === 'enemy' && !fairPlayer) {
      // Orbit altitude player.y + 800 (was +500). The hills around
      // Alpha rise to 600+ m, so a 550 m orbit sits AT the hilltop
      // height — the jet spends the whole ground hold climbing
      // toward an orbit that is itself below the ridges on its path
      // (delayed60: climbed 207 -> 617 m but the ridge ahead was
      // 611 m, dead at 36.8 s). At +800 the orbit is above every
      // local hilltop.
      const orbitY = player.pos.y + 800;
      // ORBIT: anchored, slow, WIDE (radius 1500 m, was 800 m).
      // The old orbit point started at a fixed phase (orbitA =
      // time*0.08, due east of the player) — but an enemy spawns
      // FACING the player, so the point sat ~180 deg behind it, and
      // the 55 deg loiter bank cap at 130 m/s only turns at ~6
      // deg/s: the jet flew a 1.2 km radius circle, its nose
      // 110-168 deg off the player, and drifted 3 km from the
      // player during a 60 s ground hold. The anchor puts the point
      // 1500 m from the player ALONG THE JET'S LINE OF FLIGHT
      // (orbitA = pi/2 - hdg converts the atan2(fwd.x, fwd.z)
      // heading into the orbit's atan2(z, x) angle), so the jet
      // flies straight to it — no 180 deg reversal. The orbit
      // advances at 0.04 rad/s (was 0.08): the point moves at
      // 60 m/s, the jet at 130 m/s. And the radius is 1500 m, not
      // 800 m: at 130 m/s an 800 m orbit needs a 65 deg bank —
      // above the 55 deg cap, so the jet could never hold it and
      // cut across the circle, spiralling outward (a separate
      // climb-to-a-point-above-the-player phase was tried and
      // reverted for the same reason: the jet flies through the
      // point at 130 m/s, the bearing flips 180 deg, and the
      // 29 s reversal carries it 2.4 km away). A 1500 m orbit needs
      // a 49 deg bank at 130 m/s — inside the cap with margin —
      // and 1500 m is inside the 2600 m engage range, so the enemy
      // engages the moment the player goes airborne.
      if (this.loiterAnchorA === null) {
        const hdg = Math.atan2(ac.fwd().x, ac.fwd().z);
        this.loiterAnchorA = Math.PI / 2 - hdg;
        this.loiterT = 0;
      }
      this.loiterT += dt;
      const orbitA = this.loiterAnchorA + this.loiterT * 0.04;
      goal = this.tmpB.set(
        player.pos.x + Math.cos(orbitA) * 1500,
        orbitY,
        player.pos.z + Math.sin(orbitA) * 1500,
      );
      // Pass 15: loiter climb authority 30 m/s (was the patrol
      // default 15). The floor-driven climb demand caps at +20,
      // but vsDes itself was capped at vsCap 15, so the jet could
      // only climb at +15 m/s — not enough to reach an 811 m floor
      // in the 8 s of lookahead warning (delayed60: +11 m/s
      // average, grazed a 611 m ridge at 617 m). At 30 m/s it gains
      // 240 m in 8 s. A transit climb, not a fight manoeuvre.
      vsCap = 30;
      // Pass 15: loiter speed 100 -> 130 m/s. At 100 m/s the
      // MiG-29 cannot hold the 55 deg orbit bank at the AoA limit
      // (it needs 1/cos(55 deg) = 1.75 g, it makes ~1.5 g at AoA
      // 10 deg), so it sinks unrecoverably while commanding a climb
      // (delayed60: bank -55 deg, AoA pinned at 10, vsi +5 -> -19
      // m/s, dead on flat ground at 31 s sim — the AoA limiter
      // zeroes the nose-up command and the loiter sink-unload
      // exemption keeps the bank on). At 130 m/s the dynamic
      // pressure is 1.7x, the jet holds the orbit bank with climb
      // margin, and it closes the orbit lag faster. Still above
      // 1.5x stall (96 m/s), so the low-energy bank cap does not
      // fight the orbit turn.
      speedWant = 130;
    }
    const toGoal = this.tmpA.copy(goal).sub(ac.pos);
    const dist = toGoal.length();
    if (dist < 120 && this.state === 'patrol') this.pickNav(player);

    // Pass 14: TERRAIN CLIMB DEMAND as a vsDes floor + AVOIDANCE
    // TURN, computed here (before the steering) so the turn bias
    // can feed desHdg. The lookahead pull-up used to write pitchIn
    // directly (-0.3..-1.0 nose-up for as long as the terrain ahead
    // was within 120 m) — crossing hilly terrain at 200-300 m that
    // held full nose-up for 5-10 s and zoomed the jet to pitch
    // -81.7 deg, AoA -94.6 (tailslide), 413 -> 77 kt, right after
    // the head-on pass (probe-ai-live cruise1v1). A climb-RATE
    // demand instead lets the pitch loop relax the nose-up command
    // as the climb is achieved, so the jet clears the hill and
    // levels instead of zooming. The hard floor (ground + 60 m)
    // demands a +20 m/s climb; the lookahead demand scales with
    // the shortfall (0 at 120 m clearance, +20 at zero). RTB
    // final/rollout skips terrain entirely, as before (the jet is
    // committed to the flat runway).
    // MULTI-POINT lookahead: the single point at lookAhead (up to
    // 875 m at 350 kt, ~5 s of flight) is blind to a hill closer
    // than that — the crest 300 m ahead kills the jet while the
    // point 875 m out sits past it in a valley. Sample at
    // 0.25/0.5/0.75/1.0 of the lookahead distance and take the
    // MINIMUM clearance, so the nearest hill drives the demand.
    // AVOIDANCE TURN: a climb alone cannot clear a steep ridge —
    // the terrain rises 36-105 m/s at 350 kt while the climb caps
    // at +20 (e1 pursuit t=21: climbing +20 m/s while the hill
    // rose 36 m/s, dead at 12 m clearance). When the terrain ahead
    // is seriously blocked (clearAhead < 60 m) and one side at
    // +/-45 deg is much clearer, bias the desired heading 0.6 rad
    // (~34 deg) toward the clearer side. Heading INCREASES on a
    // left turn, so the left side gets +0.6 and the right -0.6.
    // The +5 m margin (any meaningful improvement over a blocked
    // centre) replaces an earlier +40 m margin that never fired,
    // because the hills around Alpha are high in every direction.
    const landingFinal =
      this.state === 'rtb' && this.rtbPhase !== 'approach';
    let terrainClimb = 0;
    let belowFloor = false;
    let terrainTurn = 0;
    if (!landingFinal) {
      // Pass 14: lookahead follows the VELOCITY (direction of
      // travel), not the nose. In a banked turn the nose leads the
      // velocity by the turn lag, so a nose-based lookahead samples
      // a safe direction while the velocity carries the jet into
      // the ridge (e2 pursuit t=48: descending -10 m/s in patrol,
      // vsDes -12, no terrain demand, while the ridge 500 m ahead
      // on its path rose to 427 m). The travel heading is the
      // horizontal velocity direction, falling back to the nose
      // heading below 10 m/s horizontal speed.
      const velH = Math.hypot(ac.vel.x, ac.vel.z);
      const travelHdg =
        velH > 10 ? Math.atan2(ac.vel.x, ac.vel.z) : curHdg;
      const travelX = Math.sin(travelHdg);
      const travelZ = Math.cos(travelHdg);
      const lookAhead = Math.max(260, ac.ias * 5);
      let clearAhead = Infinity;
      // (Pass 14: eight sample points were tried and reverted — they
      // fired the avoidance on smaller hills, pulling the jets out
      // of the fight (pursuit engage 197 -> 164 s, player lock
      // seconds 9.3 -> 5.2) without fixing e2's ridge death. Four
      // points stand.)
      // (Pass 15: an ARC lookahead — sampling along the jet's
      // turning path (omega = -g*tan(bank)/ias) instead of the
      // straight velocity heading — was tried and REVERTED. It
      // fixed the late turning-path deaths (cruise4 2 deaths at
      // 176.3 s -> 1 at 33.6 s, gun 7 -> 14, msl 8 -> 13) but
      // regressed the pursuit engagement below the pass-14
      // baseline (engage 230.1 -> 141.8 s vs the 197.0 s baseline)
      // — the arc samples terrain to the side, so it fires the
      // terrain demand/climb more often and pulls the AI out of
      // the fight, the same mechanism as the pass-14 eight-point
      // revert. The straight-line four points stand; the remaining
      // turning-path deaths (a fast jet in a sustained bank
      // turning into a ridge the straight samples miss) are the
      // known residual, reported honestly.)
      for (const frac of [0.25, 0.5, 0.75, 1.0]) {
        const gx = ac.pos.x + travelX * lookAhead * frac;
        const gz = ac.pos.z + travelZ * lookAhead * frac;
        const gh = this.groundHeight(gx, gz);
        clearAhead = Math.min(clearAhead, ac.pos.y - gh);
      }
      // Pass 15: INTERCEPT / LOITER GOAL TERRAIN FLOOR. An
      // intercept flies at the player's altitude, which can be far
      // below the ridge tops the jet must cross — the player climbs
      // out over flat Alpha while the jet is kilometres away over
      // 550 m hills — so the jet meets a ridge it cannot out-climb
      // or out-turn (cruise1v1: 252 m/s into a ridge rising 268 m
      // in <250 m of travel, dead at 128 s). A loitering jet has
      // the same problem in miniature: its orbit point sits at
      // player.y + 500, but it climbs toward it at only +6 m/s
      // (the 0.03 vs-gain), so during a 60 s ground hold it never
      // gets above 400 m — below the hilltops — and flies into a
      // ridge (delayed60: y 320 -> 385, ridge 165 -> 380, dead at
      // 29.8 s). Floor the transit goal at the highest terrain on
      // the lookahead + 200 m, so the jet climbs BEFORE the ridge
      // instead of meeting it at the player's altitude. Transit
      // only (intercept + loiter) — the dogfight (engage) still
      // dives to the player, and the floor only ever raises the goal.
      // The floor looks FURTHER ahead than the avoidance (ias*8,
      // min 600 m — ~8 s of flight): the 5 s avoidance lookahead
      // sees a wall-like ridge only when it is too late to climb
      // over it (delayed60: ridge rising 72 m/s, jet climbing
      // +20 m/s, dead at 119.8 s — the crest was 6.8 s out when the
      // avoidance first fired). And the floor drives a climb
      // DEMAND of its own (shortfall * 0.05, cap +20): the gentle
      // 0.03 vs-gain alone only commands +5 m/s for a 160 m
      // shortfall, which does not clear a ridge in 8 s.
      if (intercepting || loitering) {
        const floorLookAhead = Math.max(600, ac.ias * 8);
        let maxGroundFloor = -Infinity;
        for (const frac of [0.25, 0.5, 0.75, 1.0]) {
          maxGroundFloor = Math.max(
            maxGroundFloor,
            this.groundHeight(
              ac.pos.x + travelX * floorLookAhead * frac,
              ac.pos.z + travelZ * floorLookAhead * frac,
            ),
          );
        }
        const floorY = maxGroundFloor + 200;
        goal.y = Math.max(goal.y, floorY);
        if (floorY > ac.pos.y) {
          terrainClimb = Math.max(
            terrainClimb,
            clamp((floorY - ac.pos.y) * 0.05, 0, 20),
          );
        }
      }
      if (clearAhead < 120) {
        terrainClimb = Math.max(
          terrainClimb,
          clamp((120 - clearAhead) * 0.25, 0, 20),
        );
      }
      if (clearAhead < 60) {
        // Sides sampled NEAR only (0.25/0.5 of lookAhead): the turn
        // commits the jet to the terrain 200-400 m to the side, and
        // the far points (875 m) are hills it will never reach on
        // this heading — including them in the min made both sides
        // read blocked and the turn never fired (byte-identical
        // pursuit run with the turn in place).
        let clearL = Infinity;
        let clearR = Infinity;
        for (const frac of [0.25, 0.5]) {
          const dL = lookAhead * frac;
          clearL = Math.min(
            clearL,
            ac.pos.y -
              this.groundHeight(
                ac.pos.x + Math.sin(travelHdg + Math.PI / 4) * dL,
                ac.pos.z + Math.cos(travelHdg + Math.PI / 4) * dL,
              ),
          );
          clearR = Math.min(
            clearR,
            ac.pos.y -
              this.groundHeight(
                ac.pos.x + Math.sin(travelHdg - Math.PI / 4) * dL,
                ac.pos.z + Math.cos(travelHdg - Math.PI / 4) * dL,
              ),
          );
        }
        const best = Math.max(clearL, clearR);
        // LATCH the avoidance side: on a wide ridge the two sides
        // are nearly equal, so the "better side" flips every frame
        // and the bank swings +/-60 deg at ~1 Hz, destroying the
        // climb (e2 pursuit t=49-55: terrTurn -0.60 -> +0.60, bank
        // -1 -> +60 -> -65 deg, climb +15 m/s vs ridge +32 m/s,
        // dead at 10 m clearance). Once a side is chosen, keep it
        // until the centre is clear. There is NO flip condition:
        // the side samples are heading-relative, so as the jet turns
        // they rotate with it and the "other side" can read 50 m
        // better purely because the jet turned — flipping on that
        // re-reads the same ridge from a new angle and reverses the
        // bank mid-manoeuvre (the latch still flipped at t=53 in
        // the run with a 50 m flip margin). The side was chosen
        // from the geometry at the moment the ridge blocked the
        // centre; turning toward it is the way out, and the latch
        // clears only when the centre is clear again.
        // Pass 15: the turn only fires when a side is CLEARLY
        // better (+80 m, was +5 m). At +5 m the jet turned toward a
        // side that was barely better — noise on 500 m hills — and
        // the 60 deg avoidance bank then killed the climb it needed
        // (pursuit e2 t=49-54: terrTurn latched, bank 60 deg, AoA
        // pinned at the limit, vsi -15..0 while the ridge rose
        // faster than it climbed). When the terrain is high in
        // every direction the turn is not an escape at all — the
        // climb is — so the jet climbs wings-level instead (the
        // bank cap below). A side 80 m clearer is a real valley,
        // and the turn still fires for it.
        if (this.terrainTurnLatch !== 0) {
          terrainTurn = this.terrainTurnLatch;
        } else if (best > clearAhead + 80) {
          this.terrainTurnLatch = clearL >= clearR ? 0.6 : -0.6;
          terrainTurn = this.terrainTurnLatch;
        }
        if (terrainTurn !== 0) {
          // Slow for the avoidance turn: at 350 kt a 45 deg bank
          // turns at 3.2 deg/s and the ridge closes in ~2 s; at
          // corner speed (120 m/s) the same bank turns at 4.7 deg/s
          // and the terrain closes half as fast. The throttle
          // section below cuts to idle + airbrake on this want.
          speedWant = Math.min(speedWant, 120);
        }
      } else if (clearAhead >= 120) {
        this.terrainTurnLatch = 0;
      }
      const deckY = carrier.deckHeightAt(ac.pos.x, ac.pos.z);
      const floorY = Math.max(
        this.groundHeight(ac.pos.x, ac.pos.z) + 60,
        deckY > -Infinity ? deckY + 60 : -Infinity,
      );
      if (ac.pos.y < floorY) {
        belowFloor = true;
        terrainClimb = Math.max(terrainClimb, 20);
      }
    }

    // ---- HF two-loop steering (bank from turn rate) ----
    // Loop 1: desired turn rate from the heading error, capped at the
    // sustained-turn envelope — the SAME sustainedTurnRate the curve
    // solver uses for its coordinated-yaw cap, so the AI and the aero
    // agree on what the jet can hold. Loop 2: bank from the turn rate,
    // phi = atan(V*omega/g) (HF coordinated bank, re-anchored to metres
    // and g = 9.81), clamped to +-75 deg, then rollIn from the bank
    // error. This replaces v2's rollIn = -local.x * 2.2, which banked
    // directly from the heading error and over-banked in tight turns.
    // Sign: heading DECREASES on a right turn (atan2(fwd.x, fwd.z) —
    // a right turn swings the nose toward -X, the right wing), so a
    // goal to the right gives a NEGATIVE heading error and a negative
    // omegaDes — and phiDes = -atan(...) is then POSITIVE, a RIGHT
    // bank (Euler YXZ z > 0, rollIn > 0). The minus sign in phiDes
    // is what makes a negative (right-turn) rate bank right. Pass 11:
    // the old comment claimed heading increases right and negative
    // phi is a right bank — both backwards; the code was right and
    // the comment was the trap. (fwd / curHdg are computed above the
    // goal section — the engage corner-speed rule uses them too.)
    const desHdg = Math.atan2(toGoal.x, toGoal.z) + terrainTurn;
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
      // Pass 11: cap the CL at this aircraft's peak (the old
      // default 1.6 overstated the envelope for the F-15E and
      // F-22), and use the LIFT area (MiG-29: 45 m², not the 38 m²
      // wing area) so the cap agrees with the curve solver's.
      sustainedTurnRate(ac.ias, rho, S.thrustMax * Math.max(this.thrTarget, 0.7), S.wingAreaEffective ?? S.wingArea, S.inducedK, S.mass, S.clCurve.peak),
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
    // Pass 12 (coordinator): the combat cap stays 0.6x. Raising it
    // to 0.9x/0.75x was tried and reverted: the gun fired more
    // (5-21 shots) but the AI bled speed in sustained hard turns
    // and the probe outcomes swung chaotically with small changes
    // (player lock 0.00-7.47 s, gun 0-21 across runs), so no
    // higher cap is reliably better. The sustained-rate formula is
    // optimistic (ignores separation drag), so 0.6x is the safe
    // cap; the missiles — the AI's primary weapon — engage
    // reliably at 0.6x (4-13 launched, player shot down).
    // Pass 12 (coordinator): engage/evade use 0.9x, not 0.6x. At
    // corner speed (120 m/s) 0.6x is ~7.4 deg/s — barely above the
    // player's 5.6 deg/s turn, and the bearing rotates away at
    // ~3 deg/s as the player closes from behind, so the heading
    // error GREW at 1.4 deg/s for 35 s (probe-ai-live: e0 banked
    // 57 deg steady, omegaSm -0.13, hdgErr 118 -> 166 deg) and the
    // gun never fired. 0.9x is ~11 deg/s (bank ~67 deg, inside the
    // 70 deg cap) — enough to gain on the error and close it. The
    // sustained-rate formula is optimistic (ignores separation
    // drag), so 1.0x is not used; the AoA limiter + corner-speed
    // throttle guard the departure if the jet bleeds speed.
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
    // Pass 12 (Worker E): the combat bank cap was 45 deg — at the
    // AI's 150-175 m/s that turns at g*tan(45)/V = 3.7-4.4 deg/s,
    // so a 180 deg reversal after a head-on pass took ~45 s while
    // the commanded rate (0.6 * sustained, ~11 deg/s) needed 71 deg
    // of bank. Measured in probe-ai-live: in 54 s of close
    // engagement the nose never got within 32 deg of the player
    // (146 deg at < 900 m) and the AI fired zero times. Combat
    // (engage/evade) may now use 70 deg (~2.9 g): the sustained-turn
    // envelope at 150 m/s is ~4.9 g for the MiG-29, so 2.9 g is
    // inside it, and the AoA limiter (10 deg banked) + sink unload
    // still guard the departure the old 60 deg cap was protecting
    // against. Patrol/traffic stay gentle (30 deg); RTB/bomb keep
    // 45 deg.
    // Pass 13: the combat bank cap (70 deg) now also applies in the
    // intercept transit, and in engage/evade it opens for a BIG
    // heading error at any range — a reversal after a head-on pass at
    // 45 deg bank turns at ~4 deg/s and takes ~45 s, during which a
    // cruising player extends 6-10 km and the AI disengages. At 70 deg
    // and corner speed (120 m/s) the same reversal takes ~8 s. The
    // 45 deg long-range cap stays for small errors, where the pitch
    // must stay free to descend.
    const combatBank =
      this.state === 'engage' || this.state === 'evade' || intercepting;
    const bankCap =
      this.role === 'traffic'
        ? (30 * Math.PI) / 180
        : loitering
          ? // Pass 13: the orbit needs ~39 deg of bank at 100 m/s
            // (atan(V*omega/g) for omega 0.08 rad/s); 30 deg cannot
            // hold the circle and the jet cuts across it.
            (55 * Math.PI) / 180
          : combatBank
            ? // Pass 13 A/B (2nd round, REVERTED): 60 deg for a big
              // heading error at any range was tried to fix the slow
              // reversal (45 deg at 200 m/s turns at ~2.8 deg/s, so
              // the 180 deg reversal after a head-on pass takes ~64 s
              // and the 200 m/s player flies 12.8 km away before the
              // enemy can turn back — cruise1v1 was one head-on pass
              // in 300 s, firstFire never). 60 deg DID let the enemy
              // fight (cruise1v1 firstFire 36.8 s, player shot down
              // 51.9 s) — but it broke the pursuit lock AGAIN
              // (maxLockT 0.03 s, gun 24), the same as 70 deg. The
              // pursuit lock is the binding constraint: any bank above
              // 45 deg at long range lets the AI turn/climb out of
              // the scripted player's 15 deg lock cone faster than it
              // can track, while the AI's 30 deg gun cone still
              // tracks the player fine. So the reversal stays slow
              // (the 1v1 cruise fight after a head-on pass is the
              // known remaining limitation, reported honestly) and
              // the range-gated 45/70 caps — which pass the pursuit
              // probe — are restored.
              // Pass 15: the 70 deg bank returns for the INTERCEPT
              // only (not engage/evade). The pass-13 constraint is
              // the dogfight: in intercept the jet is beyond 2600 m,
              // outside the player's 2500 m lock range, so the player
              // cannot lock it anyway, and once it enters engage the
              // range gate below applies as before. At 45 deg and
              // 217 m/s the turn onto the intercept course takes
              // ~35 s for 90 deg (delayed60: first engage 52 s after
              // airborne despite a 3.2 km gap the +120 m/s closure
              // should cover in ~5 s); at 70 deg it takes ~13 s.
              (intercepting && Math.abs(hdgErr) > (30 * Math.PI) / 180
                ? (70 * Math.PI) / 180
                : ((ac.pos.distanceTo(goal) <= 1200 ? 70 : 45) * Math.PI) /
                  180)
            : (45 * Math.PI) / 180;
    // (pass 12, Worker E: the combat bank cap is range-gated. In a
    // 70 deg bank the pitch input rotates the nose in the roll plane,
    // so the jet cannot descend — it stays 500-2700 m above the
    // player while the vs-law commands a descent it cannot fly, and
    // the player can never lock a target 1000 m above it (the
    // vertical angle alone is 15-20 deg). At long range (>1200 m)
    // the bearing rate is low, so a 45 deg bank (5.6 deg/s) still
    // intercepts while the pitch is free to descend to the player's
    // altitude; inside 1200 m the full 70 deg returns for the
    // dogfight, where the jet is already co-altitude. An
    // altitude-gated cap was tried first and crippled the close-in
    // turn whenever the jet was off-altitude, even in the fight.)
    let phiDes = clamp(
      -Math.atan((this.omegaSm * Math.max(ac.ias, 1)) / 9.81),
      -bankCap,
      bankCap,
    );
    // Pass 12 (coordinator): low-energy bank cap. When the jet is
    // slow (ias < 1.5x stall speed) a hard bank bleeds the remaining
    // speed faster than full throttle can replace it — the induced
    // drag at 70 deg bank exceeds the thrust, so the jet keeps
    // slowing until it stalls and cannot climb out of the ground.
    // Capping the bank at 30 deg (1.15 g) when low on energy lets
    // the jet hold altitude and accelerate back to a safe speed
    // before it turns hard again. At healthy speed the cap does not
    // apply, so the dogfight turn rate (probe-ai-live) is untouched.
    // This is the guard that saves the AI engaging a slow/hovering
    // player at low altitude: the sink unload below only fires when
    // the jet sinks FASTER than commanded, and a jet descending
    // toward a low player is descending AS commanded — so the
    // unload never fired while the MiG-29 bled to 83 kt and died
    // "hard landing" 39 m above terrain (probe-ai-hover, seeded).
    // The threshold is 1.5x (not 1.3x): the bleed is gradual and
    // the stall is sudden, so 1.3x fired only after the jet was
    // already stalling and the death time did not move at all.
    const lowEnergy = ac.ias < 1.5 * S.stallSpeed;
    if (lowEnergy) phiDes = clamp(phiDes, (-30 * Math.PI) / 180, (30 * Math.PI) / 180);
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
    // Pass 12 (Worker E): the unload does NOT fire in combat
    // (engage/evade) at healthy speed. In a hard-banked turn the jet
    // sinks, and the unload cut the bank to 30% — killing the turn
    // rate exactly when the AI needed it, so it circled the player
    // for 20-30 s with its nose 30-70 deg off and never closed to
    // gun range (probe-ai-live PASS trace). The stall guard in
    // combat is the AoA limiter above (pitch capped at level past
    // 10-12 deg), not the unload: the jet sinks in the turn instead
    // of departing, and the hard floor + terrain pull-up still catch
    // it. In patrol/RTB/bomb the unload stays — that is where the
    // sink-into-stall departure was observed.
    // Pass 12 (coordinator): BUT the unload DOES fire in combat
    // when the jet is low on energy (ias < 1.3x stall speed). The
    // AoA limiter alone does not save a jet that has bled below
    // stall in a sustained hard turn: a stalled jet cannot climb,
    // so the hard floor + terrain pull-up cannot catch it either —
    // the MiG-29 bled to 83 kt (vs 124 kt Vs), sank at -10 m/s, and
    // died "hard landing" 39 m above terrain engaging a hovering
    // player (probe-ai-hover, seeded). Levelling the wings restores
    // vertical lift and lets the jet accelerate out of the sink
    // before it stalls. At healthy speed the unload still does not
    // fire in combat, so the turn rate (probe-ai-live) is untouched.
    // (lowEnergy is defined above, at the bank cap — 1.5x stall
    // speed, so it fires early in the bleed, not after the stall.)
    // Pass 13: loitering joins the combat exemption — the orbit turn
    // IS the loiter, and cutting its bank on every sink defeats it.
    // The low-energy unload (below 1.5x stall) still fires, and the
    // AoA limiter + hard floor still guard the departure.
    // (Pass 14: a low-clearance loitering unload — firing when the
    // jet is low and sinking — was tried and reverted: it fixed the
    // delayed1v1 patrol sink (the jet recovered from 27 m to 250 m)
    // but cost the pursuit (engage 197 -> 143 s, player missile
    // damage 349 -> 0), because loitering enemies level their wings
    // whenever low instead of holding the orbit, and the engagement
    // geometry changes. The pursuit is the primary scenario, so
    // the exemption stands as pass 13 left it.)
    const inCombat = this.state === 'engage' || this.state === 'evade' || intercepting || loitering;
    if (ac.vsi < vsDesHere - 5 && (!inCombat || lowEnergy)) phiDes *= 0.3;
    // Pass 14: TERRAIN AVOIDANCE BANK. The heading bias alone
    // (terrainTurn on desHdg) cannot turn the jet away from a ridge
    // — the bank is capped at 45 deg at long range, which turns at
    // only ~3.2 deg/s at 350 kt, so the jet turned 22 deg in the 7 s
    // of warning and still flew into the hill (e1 pursuit t=14-21,
    // bank stuck at 45-46 deg while terrTurn was -0.60). The
    // avoidance turn overrides the bank cap to 60 deg (5.7 deg/s)
    // while the terrain is blocked. It does NOT fire when low on
    // energy (a slow jet cannot hold a 60 deg bank), and it is
    // occasional (only when the terrain ahead is blocked), so it
    // does not raise the everyday long-range bank the way a general
    // 60 deg cap would (that broke the pursuit lock in pass 13).
    // GATED ON AoA HEADROOM (reverted — see below): a 60 deg bank
    // tilts the lift sideways, so at the AoA limit the climb
    // collapses — but gating the avoidance bank to 45 deg when
    // AoA >= 8 deg was NET WORSE (pursuit engage 284 -> 155 s,
    // terrain death back to 23.7 s): the 45 deg turn is too slow to
    // clear the ridge e1 hits at t=14-21, and the 60 deg bank is
    // what saved it. The e0 death (60 deg bank at the AoA limit)
    // is the rarer failure (1 death at 90.6 s vs e1's 21.9 s), so
    // the 60 deg avoidance bank stands ungated.
    // (Pass 14: speed-gating the 60 deg bank was tried at 150 m/s
    // and 120 m/s and reverted both times — the pursuit enemies fly
    // at 120-150 m/s when the avoidance fires (after the slowdown
    // to corner speed), so any threshold near there flickers the
    // bank on/off and collapses the engagement (197 -> 108-173 s)
    // without fixing the deaths. The 60 deg bank stands ungated
    // except for lowEnergy, as in the best pursuit run.)
    if (terrainTurn !== 0 && !lowEnergy) {
      phiDes = ((terrainTurn < 0 ? 1 : -1) * 60 * Math.PI) / 180;
    }
    // Pass 15: BANK CAP WHILE CLIMBING. When the terrain climb
    // demand is active and the jet is NOT turning away (no side is
    // clearly better — the terrain is high in every direction), the
    // climb is the escape, and a steep bank kills it: in a 60 deg
    // bank the vertical lift halves, so the jet commands +20 m/s
    // and sinks instead (pursuit e2 t=42-54: 55-60 deg bank, AoA
    // pinned at the 10 deg limit, vsi -3..-15 while the ridge rose
    // faster than it climbed). Cap the bank at 30 deg while the
    // climb demand is active so the climb has lift. The avoidance
    // turn above is the escape when a side IS clearly better, and
    // it keeps its 60 deg bank. This fires only when the terrain
    // ahead is within 120 m of clearance, so a high dogfight is
    // untouched.
    if (terrainClimb > 0 && terrainTurn === 0) {
      phiDes = clamp(phiDes, (-30 * Math.PI) / 180, (30 * Math.PI) / 180);
    }
    // (Pass 14: a CLIMB UNLOAD here — levelling the wings when the
    // terrain climb was failing during an avoidance turn — was tried
    // and reverted: it fired through most low-level flying (the
    // terrain is blocked often and the jet is rarely climbing at
    // the demanded +20 while manoeuvring), breaking the engagement
    // (pursuit engage 281 -> 107-114 s, player lock 5.29 -> 0.03 s)
    // without fixing the death. The avoidance bank stands as is.)
    const phiNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
    let rollIn = clamp((phiDes - phiNow) * 2.0, -1, 1);
    // v2 evade weave: sin(performance.now() * 0.004); time here is seconds,
    // so the equivalent angular frequency is 4 rad/s. Pass 15: sim time
    // (see the simTime note) — on real time the weave runs 2.5x too
    // fast in sim terms at 4 fps.
    if (this.state === 'evade') rollIn = clamp(rollIn + Math.sin(this.simTime * 4) * 0.5, -1, 1);

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
    const vsGain =
      this.state === 'rtb' || loitering
        ? 0.08
        : this.state === 'engage' || this.state === 'evade'
          ? 0.015
          : 0.03;
    // (Pass 15: loitering joins RTB at 0.08. The 0.03 gain only
    // commands +8 m/s toward an orbit point 300 m above, so a
    // loitering jet takes ~60 s to climb to its orbit altitude and
    // spends the whole ground hold below the hilltops (delayed60:
    // y 207 -> 385 over 24 s sim, dead on a 380 m ridge). At 0.08
    // it commands +15 m/s (the vsCap) and reaches the orbit in
    // ~20 s. A transit goal moves smoothly, so the porpoising
    // risk the 0.08 gain carries in engage does not apply here.)
    // (pass 12, Worker E: engage/evade 0.015, was 0.03. The outer
    // altitude loop saturated — vsDes clamped at +-vsCap for any
    // altitude error over ~1000 m — and the inner pitch loop then
    // saturated too, so the cascade bang-banged between full
    // nose-up and full nose-down: a 100 deg porpoise (pitch -62 to
    // +38 deg, vsi +75 to -70 m/s) that kept the AI 1000-2400 m
    // above the player and the nose never on it. Halving the outer
    // gain widens its proportional band to 2000 m, so the descent
    // is gradual and the pitch stays unsaturated. RTB keeps 0.08 —
    // the glide slope needs it — and patrol/bomb keep 0.03.)
    const slopeSink =
      this.state === 'rtb' && this.rtbPhase === 'final'
        ? ac.ias * Math.tan((3.5 * Math.PI) / 180)
        : 0;
    // (Terrain climb demand + avoidance turn are computed above the
    // steering section — the turn bias feeds desHdg there, and
    // terrainClimb feeds vsDes here.)
    let vsDes = clamp((goal.y - ac.pos.y) * vsGain - slopeSink, -vsCap, vsCap);
    // Pass 15: VSDES SLEW LIMIT. A state transition steps the goal
    // (and so the vsDes) in one frame — patrol climb +30 to engage
    // descent -5 — and the pitch law (pitchIn from the vs error)
    // slams the nose down to kill the climb: AoA -16.4, g -2.1, vsi
    // +26 -> -20 in 2 s, 60 m of altitude gone, and the jet ends up
    // flying low and level at ~330 m over terrain that is rising
    // toward it (cruise1v1 e0 and cruise4 e1: both dead at ~30 s on
    // a ridge spike after exactly this dive). The AoA limiter cannot
    // catch it — it reads the AoA at the START of the step, and the
    // AoA swings 18 deg inside one substep. Slewing the goal-driven
    // vsDes at 20 m/s per second (a full +30 -> -30 swing in 3 s,
    // the engage transition in ~1.8 s) makes the descent gradual
    // instead of a slam: the jet keeps its climb energy and stays
    // above the ridges. The slew applies ONLY to the goal-driven
    // vsDes, BEFORE the terrain-climb max below — a terrain demand
    // (+20) must still jump in immediately, because a ridge 120 m
    // ahead cannot wait 1.8 s.
    vsDes = clamp(
      vsDes,
      this.prevVsDes - 20 * dt,
      this.prevVsDes + 20 * dt,
    );
    this.prevVsDes = vsDes;
    if (terrainClimb > 0) vsDes = Math.max(vsDes, terrainClimb);
    const pitchLim = Math.min(pitchCap, 0.6);
    // Pitch gain, speed-scheduled (pass 12, Worker E): a fixed 0.06
    // saturates on any vs error over ~10 m/s, and at combat speeds
    // (150-175 m/s) the BET elevator is powerful enough that the
    // saturated pitch law bang-bangs between full nose-up and full
    // nose-down — probe-ai-live CTRL trace: pitch swung -62 deg to
    // +38 deg, vsi +75 to -70 m/s, a 100 deg porpoise that kept the
    // AI 1000-2400 m above the player and the nose never on it.
    // Pitch authority scales with dynamic pressure (~V^2), so the
    // gain is ~1/V: 0.06 at RTB speeds (<= 70 m/s, unchanged —
    // the RTB glide-slope fix above needs it) tapering to ~0.025
    // at 170 m/s. The proportional band widens with speed, so the
    // pitch command stays unsaturated through a normal descent.
    const pitchGain = clamp(4.2 / Math.max(ac.ias, 1), 0.025, 0.06);
    let pitchIn = clamp((ac.vsi - vsDes) * pitchGain, -pitchLim, pitchLim);

    // Rudder: centre the ball (sideslip beta) + adverse-yaw feedforward
    // when rolling. beta > 0 (velocity LEFT of the nose, +X) needs
    // nose-LEFT rudder (yawIn > 0 — both solvers map +yawIn to nose
    // left); rolling RIGHT (rollIn > 0) drags the nose LEFT via
    // adverse yaw (away from the turn), so feed rudder RIGHT
    // (yawIn < 0). v2 left yawIn at 0 always — the
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
    if (ac.stalled) pitchIn = Math.max(pitchIn, 0.5);
    // Negative-AoA recovery (pass 12, Worker E): at AoA < -10 deg
    // the nose is below the velocity vector — a zoom climb after a
    // dive or a hard pull-out — and the old veto (min(pitchIn, 0))
    // only removed the nose-down demand, leaving pitchIn = 0: the
    // jet froze at AoA -25..-40 deg, climbing at +40..+70 m/s,
    // ~2000 m above the player, nose 90-120 deg off, for 15-30 s
    // (probe-ai-live CTRL trace). The recovery pulls nose-up toward
    // the velocity vector, PROPORTIONAL to the AoA error and tapering
    // to zero at -10 deg: a fixed -0.4 drove the nose to -62 deg
    // and looped the jet over into a -70 m/s dive. Symmetric to the
    // stall recovery above (nose-down out of a positive stall);
    // this pulls nose-up out of a negative one.
    // Pass 14: AoA-RATE DAMPER. The e2 pursuit death (t=65-67) was
    // a pitch oscillation, not a bad command: in a 45 deg banked
    // turn at 300 kt the AoA swung +12.9/+10.8/+2.4/-0.8/+7.0/-15.1
    // deg at ~1 Hz while pitchIn sat near zero (-0.11..+0.09) —
    // the vsi P-loop's phase lag (the lift needs ~1 s to bend the
    // velocity) lets the AoA swing freely, and near AoA 0 the lift
    // vanishes so the velocity stops following the nose and the
    // swing amplifies past the tail's negative peak (-16 deg),
    // where the vsi loop's full nose-down (+0.60 at vsi +78) drives
    // the AoA to -75 deg and the jet tumbles. A D-term on the AoA
    // opposes the swing itself: pitchIn += aoaRate * 0.01, so a
    // 15 deg/s AoA swing meets a 0.15 counter-command. It is a pure
    // damper (no trim bias — a steady AoA contributes nothing), so
    // a normal pull (AoA rising ~5 deg/s) costs only ~0.05 of
    // command. Applied after the limiters, before the throttle.
    const aoaRateRaw =
      dt > 0 ? (ac.aoaDeg - this.prevAoA) / dt : 0;
    this.prevAoA = ac.aoaDeg;
    // Low-pass the rate (10/s): the substep AoA jitters ~0.5 deg
    // per 0.016 s substep, which is 30 deg/s of raw noise — the
    // unfiltered rate would inject +/-0.3 of noise into pitchIn.
    // The 0.1 s time constant keeps the 1 Hz oscillation (the
    // departure mode) while rejecting the substep jitter.
    this.aoaRateSm += (aoaRateRaw - this.aoaRateSm) * Math.min(1, dt * 10);
    pitchIn = clamp(pitchIn + this.aoaRateSm * 0.01, -pitchLim, pitchLim);

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
    // Pass 11 (lift fix): the pass-4 RTB-final floor (0.35 even when
    // above want) is re-tuned to 0.10, not removed. It existed because
    // under the buggy BET the jet sank at -13 m/s at idle on the slope
    // (the phantom L·sin(AoA) drag); with the lift now perpendicular
    // to the airflow the jet glides at idle like the curve solver, so
    // the floor's rationale changed — but the idle glide is STILL
    // steeper than the 3.5 deg slope (gear down, -7 to -13 m/s at
    // idle), so with the floor at 0 the jet sinks below the slope and
    // hits terrain ~1 km short of the runway (probe). Conversely the
    // 0.35 floor overpowered the drag: 0.35 throttle (~57 kN on the
    // MiG-29) beat the airbrake (~33 kN at 100 m/s) and the jet
    // accelerated down the slope to 203 kt and died on touchdown.
    // 0.10 (~16 kN) is the slope-holding thrust at ~72 m/s (drag
    // ~25 kN minus the 9 kN gravity component down the slope) — it
    // holds the glide without beating the drag, so the jet settles
    // near the 72 m/s final speed instead of running away. The floor
    // is 0.10 in RTB final when above want; 0 elsewhere above want;
    // 0.35/0.85 only when at or below want.
    const rtbFinalFloor = this.state === 'rtb' && this.rtbPhase === 'final';
    const minThr = aboveWant
      ? (rtbFinalFloor ? 0.1 : 0)
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
    // Pass 14: the terrain climb itself is now a vsDes floor (see
    // the pitch section above) — this block only keeps the hard-floor
    // throttle boost, so a jet climbing out of the floor has power.
    if (!landingFinal && belowFloor) {
      this.thrTarget = Math.max(this.thrTarget, 0.8);
    }

    // Firing intent (v2 G6 block): gun in cone + range, missile when locked
    // — never at a grounded, spawn-protected or dying player (v1.1 +
    // damage-model §6: a dying jet is untargetable). Sets the
    // fireGun / fireMissile flags; the caller performs the shot.
    // The gun and missile cooldowns are independent (see fireGunT):
    // a missile launch must not suppress the gun.
    this.fireGunT -= dt;
    this.fireMslT -= dt;
    if (this.state === 'engage' && player.alive && !player.dying && !player.grounded && player.spawnProt <= 0) {
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
      // Pass 12 (Worker E): the gun cone is 30 deg for a fast target
      // (was 20, was 12 before that) — see the note above. The
      // missile cone stays 15 deg: the missile needs a real lock.
      const slowTarget = player.ias < 50;
      const gunCone = 30;
      // (pass 12, Worker E: was 20, was 12 before that. The gun
      // aim is nose-independent — aiGunShot fires a ray at the lead
      // point with the skill error — so the cone is only the
      // trigger decision, not the ballistics. But the nose is ~22
      // deg off during the approach because BOTH jets are turning
      // (the player turns 5.6 deg/s in its 45 deg bank, the AI
      // 10 deg/s), so a 20 deg gate almost never opens at <900 m
      // even though the AI is in the 15 deg missile cone at
      // 1300-2000 m and fires 6-8 missiles a run. 30 deg is "nose
      // roughly on target, pull the trigger"; the hit is still
      // decided by the lead point + skill error.)
      const mslCone = slowTarget ? 30 : 15;
      const mslRange = slowTarget ? 3000 : 2200;
      if (d < 900 && ang < gunCone && this.fireGunT <= 0) {
        this.fireGunT = 1.2 / this.skill.burst;
        this.fireGun = true;
      }
      if (d < mslRange && ang < mslCone && ac.msl > 0 && this.fireMslT <= 0 && this.rng() < 0.25) {
        this.fireMslT = 3;
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
    this.lastPitchIn = pitchIn;
    this.lastRollIn = rollIn;
    this.lastVsDes = vsDes;
    this.lastTerrainTurn = terrainTurn;
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
