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
import type { HelicarrierController } from './HelicarrierController';
import { RUNWAYS, type Runway } from '../world/RunwayPlacer';

export const SKILL_TABLE = {
  rookie: { lead: 0.0, react: 1.6, err: 0.35, burst: 0.5 },
  veteran: { lead: 0.5, react: 0.9, err: 0.18, burst: 0.9 },
  ace: { lead: 1.0, react: 0.45, err: 0.07, burst: 1.4 },
} as const;

export type SkillKey = keyof typeof SKILL_TABLE;
export type AiRole = 'enemy' | 'traffic';
export type AiState = 'patrol' | 'engage' | 'evade' | 'rtb' | 'dead';

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

  private decideT = 0;
  private fireT = 0;
  private evadeT = 0;
  /** Throttle target is stateful (v2 integrates ac.thrTarget); kept here
   *  since the controller no longer writes the aircraft's field. */
  private thrTarget = 0.7;
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
    if (!ac.alive) {
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
        // v1.1: never engage a grounded or spawn-protected player.
        const pFair = player.alive && !player.grounded && player.spawnProt <= 0;
        this.state = pFair && ac.pos.distanceTo(player.pos) < 2600 ? 'engage' : 'patrol';
      }
      if (this.state === 'patrol' || this.state === 'rtb') {
        if (this.rng() < 0.3) this.pickNav(player);
      }
    }
    if (this.state === 'evade') {
      this.evadeT -= dt;
      if (this.evadeT <= 0) this.state = 'engage';
    }

    // Steering goal + wanted speed per state (v2).
    let goal = this.navTarget;
    let speedWant = 130;
    if (this.state === 'engage' && player.alive) {
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
    } else if (this.state === 'rtb') {
      const r = this.nearestRunway(ac.pos);
      goal = this.tmpB.set(r.x - this.originShift.x, r.elev + 200, r.z - this.originShift.z);
      speedWant = 110;
    }
    const toGoal = this.tmpA.copy(goal).sub(ac.pos);
    const dist = toGoal.length();
    if (dist < 120 && (this.state === 'patrol' || this.state === 'rtb')) this.pickNav(player);
    const desired = toGoal.normalize();
    const inv = ac.quat.clone().invert();
    const local = desired.clone().applyQuaternion(inv);
    // Body frame: +y up, +x right, +z forward. rollIn > 0 = bank left,
    // pitchIn < 0 = nose up (v2 comment).
    let rollIn = clamp(-local.x * 2.2 + (this.state === 'evade' ? 1.2 : 0), -1, 1);
    let pitchIn = clamp(-local.y * 2.0, -1, 1);
    // v2 evade weave: sin(performance.now() * 0.004); time here is seconds,
    // so the equivalent angular frequency is 4 rad/s.
    if (this.state === 'evade') rollIn = clamp(rollIn + Math.sin(time * 4) * 0.5, -1, 1);

    // Throttle: speed management (Ace dives to regain energy — v2).
    const want = speedWant + (this.state === 'engage' && ac.ias < 90 ? 40 : 0);
    this.thrTarget = clamp(this.thrTarget + (ac.ias < want ? dt * 0.8 : -dt * 0.8), 0.35, 1);
    if (ac.pos.y > 1400) this.thrTarget = Math.min(this.thrTarget, 0.5);

    // Terrain avoidance: pull up if the ground is close ahead (v1.1:
    // stronger, plus a hard floor — v1 AI flew into the terrain chasing a
    // ground target).
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

    // Firing intent (v2 G6 block): gun in cone + range, missile when locked
    // — never at a grounded or spawn-protected player (v1.1). Sets the
    // fireGun / fireMissile flags; the caller performs the shot.
    this.fireT -= dt;
    if (this.state === 'engage' && player.alive && !player.grounded && player.spawnProt <= 0 && this.fireT <= 0) {
      const toP = this.tmpB.copy(player.pos).sub(ac.pos);
      const d = toP.length();
      const ang = ac.fwd().angleTo(toP.normalize()) * R2D;
      if (d < 900 && ang < 12) {
        this.fireT = 1.2 / this.skill.burst;
        this.fireGun = true;
      }
      if (d < 2200 && ang < 15 && ac.msl > 0 && this.rng() < 0.25) {
        this.fireT = 3;
        this.fireMissile = true;
      }
    }

    // Defensive flares when damaged or when the player is behind (v2).
    if ((ac.hp < 50 && this.rng() < dt * 0.3) || (this.state === 'engage' && playerBehind(ac, player))) {
      ac.fireFlares();
    }

    return { pitchIn, rollIn, yawIn: 0, thrTarget: this.thrTarget };
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
