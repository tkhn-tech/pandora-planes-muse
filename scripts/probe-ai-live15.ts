// probe-ai-live15.ts — pass 15, Worker C: LIVE-FAITHFUL AI probe.
//
// probe-ai-live.ts passes simT as the AI's `time` argument and advances
// sim time by the full rawDt every frame — i.e. sim time == real time.
// The LIVE loop does neither:
//   elapsed += rawDt;                       // real time, uncapped
//   simDt = Math.min(rawDt, 0.1);          // sim time capped at 0.1 s/frame
//   ai.update(dt, player, carrier, elapsed) // AI clock = REAL time
// At 4 fps (live ground FPS) rawDt = 0.25 s, so sim advances 0.4x real
// while the AI's `time` argument advances 1x real. Anything the AI
// drives from `time` — the loiter orbit angle (time*0.08) and the evade
// weave (sin(time*4)) — runs 2.5x too fast in sim terms: the loiter
// orbit point moves at 0.08*800 = 64 m/s real = 160 m/s sim, while the
// loitering jet flies at 100 m/s sim — it can never catch its orbit
// point. This probe replicates the live loop EXACTLY (elapsed vs simT
// split, simDt cap, ground/air fps split) and measures whether the
// enemy stays near the player through a slow human takeoff and then
// closes, engages and fights — in REAL time, as the player experiences
// it. Scenarios: cruise1v1, delayed1v1 (25 s real ground hold),
// delayed60 (60 s real ground hold — a slow human), cruise4 (4
// enemies, cruise player), pursuit (4 enemies, pursuit player).
//
// Usage: node_modules/.bin/esbuild scripts/probe-ai-live15.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-ai-live15.mjs
//   && node /tmp/probe-ai-live15.mjs [--scenario cruise1v1]
//   [--fps-ground 4] [--fps-air 11] [--real 330] [--spd 200]

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
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
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
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { WeaponSystem } from '../src/combat/WeaponSystem';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

// Live default wind: windDir 270, windSpeed 3 -> (-3, 0, 0).
const WIND = new THREE.Vector3(-3, 0, 0);
// Seeded RNG so the "random" spawn bearings are reproducible.
const rngState = { s: 123456789 };
const rng = (): number => {
  rngState.s = (rngState.s * 1103515245 + 12345) & 0x7fffffff;
  return rngState.s / 0x7fffffff;
};
// Deterministic Math.random (pass 12, Worker E): WeaponSystem uses
// Math.random() directly (gun error, flare break, ignition rolls),
// so two runs of the same code diverged wildly (player lock 6.52 s
// vs 0.05 s) from unseeded randomness alone. A probe must be
// reproducible. Seed it from a SEPARATE LCG so the AI/spawn rng
// sequence above is untouched. This is probe scaffolding only — the
// live game keeps the real Math.random.
const rngMathState = { s: 987654321 };
Math.random = (): number => {
  rngMathState.s = (rngMathState.s * 1103515245 + 12345) & 0x7fffffff;
  return rngMathState.s / 0x7fffffff;
};

const R2D = 180 / Math.PI;
// Pass 15, Worker C: scenario selection + LIVE frame-rate model.
// The live loop advances elapsed by rawDt (real time) and sim time by
// simDt = min(rawDt, 0.1). FPS on the ground is 4-9, at altitude
// 10-12, so rawDt is 0.25 s on the ground and ~0.09 s in the air.
// --real is the REAL-time budget (the live check ran ~5.5 min = 330 s
// real); sim time is whatever the fps model yields.
const argv = process.argv.slice(2);
const scenarioArg = (argv[argv.indexOf('--scenario') + 1] ?? 'cruise1v1') as string;
const SCENARIO: 'pursuit' | 'cruise1v1' | 'delayed1v1' | 'delayed60' | 'cruise4' =
  scenarioArg === 'cruise1v1' || scenarioArg === 'delayed1v1' || scenarioArg === 'delayed60' || scenarioArg === 'cruise4' || scenarioArg === 'pursuit'
    ? (scenarioArg as 'pursuit' | 'cruise1v1' | 'delayed1v1' | 'delayed60' | 'cruise4')
    : 'cruise1v1';
const IS_CRUISE = SCENARIO !== 'pursuit';
const IS_1V1 = SCENARIO === 'cruise1v1' || SCENARIO === 'delayed1v1' || SCENARIO === 'delayed60';
const realIdx = argv.indexOf('--real');
const REAL_SECONDS = realIdx >= 0 ? Number(argv[realIdx + 1] ?? 330) : 330;
const fgIdx = argv.indexOf('--fps-ground');
const FPS_GROUND = fgIdx >= 0 ? Number(argv[fgIdx + 1] ?? 4) : 4;
const faIdx = argv.indexOf('--fps-air');
const FPS_AIR = faIdx >= 0 ? Number(argv[faIdx + 1] ?? 11) : 11;
const ENEMY_COUNT = SCENARIO === 'cruise4' ? 4 : IS_1V1 ? 1 : 4; // live default 4, live check 1
const PLAYER_KEY = IS_CRUISE ? 'f35a' : 'f22'; // live jet vs probe jet
const spdIdx = argv.indexOf('--spd');
const CRUISE_SPD = spdIdx >= 0 ? Number(argv[spdIdx + 1] ?? 200) : 200; // m/s, human cruise in the F-35A
// Pass 14 diagnostic: --noflares sets every enemy's flare count to 0,
// isolating whether the AI's flares (missile break 2.2/s while
// flareCd > 0.4) are what defeats the player's missiles.
const NO_FLARES = argv.includes('--noflares');
const TAKEOFF_DELAY_S = SCENARIO === 'delayed1v1' ? 25 : SCENARIO === 'delayed60' ? 60 : 0;
const LOCK_CONE_DEG = 15;
const LOCK_RANGE = 2500;
const LOCK_TIME = 1.2;

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

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
  const heightAtHf = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
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
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const drain = (): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };

  const weapons = new WeaponSystem(scene, world, {
    groundHeight: heightAtLive,
    deckHeight: () => -Infinity,
  });

  const alpha = RUNWAY_TABLE[0]!;
  const player = new Aircraft({
    scene,
    world,
    spec: SPECS[PLAYER_KEY]!,
    isPlayer: true,
    modelKey: SPECS[PLAYER_KEY]!.modelKey,
  });
  player.terrainHeightAt = heightAtLive;
  origin.registerBody(player.body);
  for (const h of player.colliderHandles()) owners.set(h, player);
  await player.attachModel();
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
  // Probe scaffolding (see header): hp 1000 so a working AI's hits are
  // measured as damage over the full run instead of ending it early.
  player.hp = 1000;

  // ---- startRound's enemy spawn, verbatim from main.ts (4 Veterans) ----
  const carrierStub = {
    deckHeightAt: () => -Infinity,
  } as unknown as Parameters<AircraftAIController['update']>[2];
  const enemies: Aircraft[] = [];
  const ais: AircraftAIController[] = [];
  const spawnHeadings: number[] = [];
  for (let i = 0; i < ENEMY_COUNT; i++) {
    const key = i % 2 === 0 ? 'mig29' : 'su34';
    const e = new Aircraft({
      scene,
      world,
      spec: SPECS[key]!,
      isPlayer: false,
      modelKey: SPECS[key]!.modelKey,
    });
    e.terrainHeightAt = heightAtLive;
    origin.registerBody(e.body);
    for (const h of e.colliderHandles()) owners.set(h, e);
    await e.attachModel();
    let ex: number;
    let ey: number;
    let ez: number;
    if (i < 2) {
      // Pass 17 (Worker B): dead-ahead, synced with the main.ts
      // startRound fix — the old offsets (side 120/-150, up
      // 120/180) put both enemies ~21 deg off the nose, OUTSIDE
      // the 15 deg lock cone from frame one, so the 1-enemy cruise
      // lock measured 0.00 s. Dead-ahead (side 0, up 30/60) the
      // same scan locks in 1.70 s (probe-pass17-lock).
      const fwd = player.fwd();
      const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
      const along = 450 + i * 150;
      const side = 0;
      const up = 30 + i * 30;
      ex = player.pos.x + fwd.x * along + rgt.x * side;
      ey = player.pos.y + up;
      ez = player.pos.z + fwd.z * along + rgt.z * side;
    } else {
      const a = rng() * Math.PI * 2;
      const d = 1200 * (0.8 + rng() * 0.5);
      ex = player.pos.x + Math.cos(a) * d;
      ey = 650 + rng() * 350;
      ez = player.pos.z + Math.sin(a) * d;
    }
    // Pass 8 (item 2): enemies spawn FACING the player (head-on
    // intercept), not on a random heading.
    const hdgToPlayer =
      (Math.atan2(player.pos.x - ex, player.pos.z - ez) * 180) / Math.PI;
    spawnHeadings.push(hdgToPlayer);
    e.resetAt(ex, ey, ez, hdgToPlayer);
    e.setAirspeed(150);
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    if (NO_FLARES) e.flares = 0;
    const ai = new AircraftAIController(e, 'veteran', 'enemy', {
      groundHeight: heightAtLive,
      originShift: origin.cumulativeShift,
      rng,
    });
    origin.registerVector(ai.navTarget);
    ais.push(ai);
    enemies.push(e);
  }
  console.log(
    `  enemy spawn headings (deg, all facing player): ${spawnHeadings.map((h) => h.toFixed(0)).join(', ')} (player hdg ${alpha.hdg})`,
  );
  console.log(
    `  enemy spawn dists: ${enemies.map((e) => e.pos.distanceTo(player.pos).toFixed(0)).join(', ')} m`,
  );

  // ---- live loop, REAL_SECONDS of real time (live-faithful) ----
  const vrMs = SPECS[PLAYER_KEY]!.rotateSpeedKt * 0.514444;
  const ALT_TARGET = 1000;
  const SPD_TARGET = IS_CRUISE ? CRUISE_SPD : 150; // m/s, a human cruise
  let simT = 0;
  // Pass 15: LIVE clock split. elapsed is REAL time (rawDt per frame,
  // uncapped) — the clock main.ts passes to ai.update as `time` and
  // advances for failure timers / the HUD. simT is SIM time (capped
  // at 0.1 s of sim per frame). At 4 fps they diverge 2.5x.
  let elapsed = 0;
  let frame = 0;
  let airborneAt = -1;
  let airborneAtReal = -1;
  let firstEngageAt = -1;
  let firstEngageAtReal = -1;
  let engageSeconds = 0;
  let engageSecondsReal = 0; // real-time engagement (player experience)
  // Pass 15: loiter-drift diagnostics — how far the enemy wanders
  // from the player while the player is still on the ground.
  let enemyDistAtAirborne = -1;
  let maxEnemyDistDuringHold = 0;
  let orbitLagAtAirborne = -1; // enemy distance to its loiter orbit point
  let firstFireAt = -1;
  let aiGunShots = 0;
  let aiMslLaunched = 0;
  let aiBombsDropped = 0;
  let minDist = Infinity;
  let timeWithin2200 = 0;
  let timeWithin900 = 0;
  let maxLockT = 0;
  let lockSeconds = 0;
  let playerMslFired = 0;
  let firedForLock = false; // one missile per lock acquisition
  let playerMslStart = player.msl;
  let enemyDeathAt = -1;
  let enemyDeathReasons: string[] = [];
  // Terrain death only (pass 12, coordinator): an enemy "shot down"
  // / "fire" / "tail destroyed" is a legitimate player kill (the
  // desired outcome — the player's missiles doing damage), NOT a
  // failure. Only a terrain death ('hard landing' / 'terrain') fails
  // the survive check.
  let terrainDeathAt = -1;
  let playerDeathAt = -1;
  let playerDeaths = 0;
  let playerDeathReasons: string[] = [];
  let playerHpLostTotal = 0; // cumulative across lives
  let maxPlayerAlt = 0;
  let maxPlayerVsi = 0;
  let maxEnemyG = 0;
  let maxPlayerG = 0;
  // Pass 14: rolling per-enemy context (last ~20 s at 1 s cadence) so a
  // terrain/hard-landing death can be diagnosed from what the jet was
  // doing, not just where it died.
  const deathCtx: Record<number, string[]> = {};
  const enemyHpStart = enemies.map((e) => e.hp);
  let playerTarget: Aircraft | null = null; // sticky pursuit target
  let playerOmegaSm = 0; // smoothed turn rate for the two-loop heading (below)
  let playerHpLast = 1000; // pass 13: damage-triggered break (cruise only)
  let breakUntil = -1;
  let breakTarget: Aircraft | null = null;
  // Pass 15: lock-geometry diagnostics (why is 1v1 cruise lock 0.00 s?).
  let minLockAngle = Infinity; // min nose angle to any enemy while < 2500 m
  let timeWithin25 = 0; // sim s with nearest enemy < 2500 m and < 25 deg
  let timeWithin15 = 0; // sim s with nearest enemy < 2500 m and < 15 deg
  let lastBreakLog: number | null = null;
  const stateSeconds: Record<string, number> = {};
  let lastLogSim = -10;
  let lastTraceSim: number | null = null; // pass 13: 1v1 enemy trace clock

  // Scripted player autopilot — PITCH-ATTITUDE control, not vertical
  // speed: a vs-controller saturates nose-down (+0.4..+0.5) whenever
  // the climb rate overshoots, drives the AoA to -40 deg, and the jet
  // gets stuck in a deep-negative-AoA zoom climb (vsi +50..+100 m/s
  // to 10 km — the old probe's false pass). Attitude hold keeps the
  // inputs small and the AoA sane: +10 deg nose-up in the climb, then
  // level with a small altitude trim. Rotation is a moderate -0.4
  // pull, released the moment the jet is airborne.
  const flyPlayer = (dt: number): void => {
    if (!player.alive) return;
    // Pass 13 cruise scenarios: a human who does NOT pursue. Delayed
    // takeoff holds on the runway (throttle idle) for TAKEOFF_DELAY_S.
    // Pass 15: the ground hold is REAL time — a human sits on the
    // runway for TAKEOFF_DELAY_S real seconds, not sim seconds.
    if (IS_CRUISE && player.onSurface && elapsed < TAKEOFF_DELAY_S) {
      player.thrTarget = 0;
      player.pitchIn = 0;
      player.rollIn = 0;
      return;
    }
    if (IS_CRUISE && !player.onSurface) {
      // Pass 15: break detection FIRST (before the throttle law) so
      // the throttle can slow the jet to corner speed during a
      // break (see below).
      if (player.hp < playerHpLast - 1) {
        breakUntil = elapsed + 12;
        let best: Aircraft | null = null;
        let bd = Infinity;
        for (const e of enemies) {
          if (!e.alive || e.dying) continue;
          const d = e.pos.distanceTo(player.pos);
          if (d < bd) {
            bd = d;
            best = e;
          }
        }
        breakTarget = best;
      }
      playerHpLast = player.hp;
      const breaking = elapsed < breakUntil && breakTarget !== null && breakTarget.alive && !breakTarget.dying;
      // Pass 15: CORNER SPEED IN A BREAK. A human breaking into an
      // attacker pulls power to turn tighter: at 193 m/s (375 kt)
      // a 70 deg bank only turns at ~8 deg/s, so the nose closed
      // just 4 deg/s (174 -> 130 deg in 11 s) and the 80 deg bank
      // killed the vertical lift — the player sank 1063 -> 296 m
      // while the enemy climbed away (break trace, cruise1v1).
      // At 120 m/s the same bank turns at ~13 deg/s and the lift
      // holds altitude. So the speed target is 120 m/s in a break.
      const spdTarget = breaking ? 120 : SPD_TARGET;
      player.thrTarget = clamp(
        player.thrTarget + (player.ias < spdTarget ? dt * 0.5 : -dt * 0.5),
        0,
        1,
      );
      // Pass 13: progressive power shed when high (a human pulls
      // power to stop a climb). Level flight at 200 m/s needs only
      // ~0.26 throttle on the F-35A, so power above that while high
      // is climb power: shed it progressively with altitude excess
      // (0.6 at 1100 m, 0.3 at 2500 m). Without it the throttle law —
      // which only watches airspeed — saturates at 1.0 while the jet
      // zoom-climbs (ias never reaches 200 while climbing), and the
      // full-thrust nose-up moment fights the nose-down elevator
      // until the "cruise" is at 4000+ m.
      const altExcess = Math.max(0, player.pos.y - ALT_TARGET);
      player.thrTarget = Math.min(
        player.thrTarget,
        0.6 - Math.min(0.3, altExcess * 0.0002),
      );
      // Cruise east with a gentle +/-15 deg weave; never turns toward
      // the enemy. Pitch is the AI's own vs-cascade (verbatim gains):
      // vsDes from the altitude error, pitchIn from the vs error with
      // a speed-scheduled gain, AoA limiter at 12 deg. The ATTITUDE
      // law tried before could not climb off the runway: its pitchIn
      // chattered around the solver's -0.25 nose-up gate (pitchIn
      // -0.12..-0.37, mostly above the gate, so the pitch rate was
      // quartered), the nose never reached its -10 deg target, and the
      // jet skimmed at 3 m for 14 s — where the weave's ~59 deg bank
      // (a 10.7 deg/s turn at 87 m/s needs that much bank) dipped a
      // wingtip into the runway and crashed it "hard landing" every
      // ~11 s. The vs-law keeps commanding nose-up until the vsi
      // actually rises, so it pushes through the gate and climbs; the
      // progressive throttle cap above is what stops the zoom-climb
      // the vs-law had on a saturated throttle (4600 m). The weave is
      // also gated on altitude (straight east below 300 m) so the jet
      // never banks hard near the ground.
      // Pass 15: 3D LEAD-PURSUIT BREAK (probe scaffolding — tests the
      // pass-14 premise that the 1v1 cruise lock of 0.00 s is the
      // scripted player, not the game). The old break steered HEADING
      // only: the pitch law still targeted ALT_TARGET (1000 m) while
      // the enemy was 332 m below, and the AoA limiter (12 deg)
      // zeroed every nose-up command in the 37-50 deg break bank, so
      // the player spiral-dived 798 -> 127 m — through and past the
      // enemy's altitude — and closed the nose angle only ~3 deg/s
      // (143 -> 113 deg in 11 s) before being shot down (break
      // trace, cruise1v1). A competent human breaking into an
      // attacker pulls to the attacker's ALTITUDE as well as its
      // heading, and pulls hard enough in the bank to hold altitude
      // (AoA up to ~18 deg, inside the F-35A's envelope). So during
      // the break the pitch target is the break target's altitude
      // and the AoA limiter relaxes to 18 deg. The break detection
      // moves BEFORE the pitch law so the pitch can use it. If a
      // lock still does not build with a 3D break, the premise is
      // disproved and the lock itself (15 deg cone / 1.2 s in
      // src/main.ts) is the problem.
      const vsDes = clamp(
        ((breaking && breakTarget ? breakTarget.pos.y : ALT_TARGET) - player.pos.y) * 0.03,
        -15,
        15,
      );
      const pitchGain = clamp(4.2 / Math.max(player.ias, 1), 0.025, 0.06);
      player.pitchIn = clamp((player.vsi - vsDes) * pitchGain, -0.6, 0.6);
      // AoA limiter: 12 deg normally, 18 deg in a break (see above).
      if (player.aoaDeg > (breaking ? 18 : 12)) player.pitchIn = Math.max(player.pitchIn, 0);
      if (player.stalled) player.pitchIn = Math.max(player.pitchIn, 0.5);
      if (player.aoaDeg < -10) {
        player.pitchIn = Math.min(player.pitchIn, clamp((player.aoaDeg + 10) * 0.03, -0.4, 0));
      }
      // Pass 13: defensive break. A human under fire turns INTO the
      // attacker — a straight-line cruise profile never presents the
      // enemy in the 15 deg lock cone (it attacks from astern), so the
      // player could never lock even in a working fight (maxLockT 0.00
      // across every 1v1 run). On taking damage, break toward the
      // nearest enemy for up to 12 s: the head-on pass that follows is
      // the lock opportunity a real pilot gets. Damage-triggered only
      // (no damage, no break), so the cruise profile is unchanged
      // until the fight reaches the player.
      // Pass 13: the weave is gated on altitude — straight east below
      // 300 m, so the jet never banks hard near the ground (see the
      // pitch comment above: a 59 deg weave bank at 3 m dips a wingtip
      // into the runway). Above 300 m the weave resumes.
      let desHdg = (90 * Math.PI) / 180;
      if (player.pos.y > 300) {
        desHdg += (15 * Math.PI) / 180 * Math.sin(simT * 0.05);
      }
      if (breaking && breakTarget) {
        // Pass 13: the break is a LEAD pursuit, not a pure pursuit.
        // A pure pursuit aims at where the enemy IS; by the time the
        // 180 deg reversal completes the enemy has moved 200 m/s *
        // ~10 s = 2 km and the nose points at empty sky. Leading the
        // target (same 0.5 lead the AI uses) puts the enemy in the
        // 15 deg lock cone during the head-on pass that follows.
        const dBreak = breakTarget.pos.distanceTo(player.pos);
        const leadPt = breakTarget.pos
          .clone()
          .addScaledVector(breakTarget.vel, (dBreak / 280) * 0.5);
        desHdg = Math.atan2(leadPt.x - player.pos.x, leadPt.z - player.pos.z);
      }
      const curHdg = Math.atan2(player.fwd().x, player.fwd().z);
      let hdgErr = desHdg - curHdg;
      while (hdgErr > Math.PI) hdgErr -= 2 * Math.PI;
      while (hdgErr < -Math.PI) hdgErr += 2 * Math.PI;
      // Pass 13: the break is a MAX-PERFORMANCE turn (full sustained
      // rate, ~18 deg/s), not the 0.6x cruise rate (~11 deg/s). At
      // 0.6x an 8 s break turned the jet only ~90 deg and the enemy —
      // itself turning at up to 22 deg/s — was never in the 15 deg
      // lock cone (maxLockT 0.00-0.03 s). A human breaking into an
      // attacker pulls hard; the full rate completes the 180 deg
      // reversal inside the 12 s break and the head-on pass that
      // follows is the lock opportunity.
      const rateCap = breaking ? 0.31 : 0.31 * 0.6;
      const omegaDes = clamp(hdgErr * 1.5, -rateCap, rateCap);
      playerOmegaSm += (omegaDes - playerOmegaSm) * Math.min(1, dt * 6);
      const bankDes = clamp(
        -Math.atan((playerOmegaSm * Math.max(player.ias, 1)) / 9.81),
        (-70 * Math.PI) / 180,
        (70 * Math.PI) / 180,
      );
      const bankNow = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').z;
      player.rollIn = clamp((bankDes - bankNow) * 2.0, -1, 1);
      return;
    }
    player.thrTarget = clamp(
      player.thrTarget + (player.ias < SPD_TARGET ? dt * 0.5 : -dt * 0.5),
      0,
      1,
    );
    if (player.onSurface && player.ias >= vrMs) {
      player.pitchIn = -0.4;
      player.rollIn = 0;
      return;
    }
    if (player.onSurface) {
      player.pitchIn = 0;
      player.rollIn = 0;
      return;
    }
    // Airborne: full-lead pursuit of a STICKY target (pass 12,
    // Worker E) — the best lock so far (0.40 s) and the best gun
    // (16). The target is sticky (a human picks a bandit and stays
    // on it): re-acquiring the nearest enemy every frame made the
    // nose oscillate between bandits and never settle in the cone.
    // Below 300 m hold east to finish the climb clear of terrain.
    // (The production-AI-controller player was tried and was worse:
    // it flew at 647 m and got shot down 3x — the AI's pursuit is
    // for closing, not for holding a 1.2 s lock.)
    // Pass 14: the 600 m gate starved the lock — with the AI now
    // lethal at low altitude the player is shot down ~3x per run
    // while climbing out and never reaches 600 m, so it spends the
    // run flying east as a fleeing target and never tracks (lock
    // 0.05 s). A human who sees red diamonds behind him turns to
    // face them at once; the gate is 300 m (clear of the terrain
    // the player spawns over) and the climb is 15 deg so the
    // player gets to fighting altitude faster.
    const pitchNow = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x;
    let desHdg = (90 * Math.PI) / 180;
    if (player.pos.y > 300) {
      if (!playerTarget || !playerTarget.alive || playerTarget.dying || playerTarget.pos.distanceTo(player.pos) > 4000) {
        // Most nearly AHEAD (pass 12, Worker E), not nearest: the
        // nearest enemy is usually BEHIND the player (it passed and
        // is chasing from astern), so a nearest-target player spends
        // the fight in a 180 deg turn / tail chase and the nose is
        // never in the 15 deg cone for the 1.2 s a lock needs
        // (maxLockT 0.03-0.05 s deterministically). With 4 enemies,
        // one is almost always approaching head-on; picking the
        // smallest nose angle works that head-on geometry, where the
        // bearing is near-constant and the lock builds. Sticky, as
        // before — re-acquire only when the current target is
        // dead/dying/out of range.
        let best: Aircraft | null = null;
        let bestAng = Infinity;
        for (const e of enemies) {
          if (!e.alive || e.dying) continue;
          const d = e.pos.distanceTo(player.pos);
          if (d > 4000) continue;
          const ang = player.fwd().angleTo(e.pos.clone().sub(player.pos).normalize()) * 180 / Math.PI;
          if (ang < bestAng) {
            bestAng = ang;
            best = e;
          }
        }
        playerTarget = best;
      }
      if (playerTarget) {
        desHdg = Math.atan2(
          playerTarget.pos.x - player.pos.x,
          playerTarget.pos.z - player.pos.z,
        );
      }
    }
    // 3D lead pursuit for the pitch: the nose must point at the
    // intercept point's altitude, not just match the heading — the
    // lock cone is 3D, and the AI flies 1000-3000 m while the player
    // levelled at 950 m, so the vertical angle alone kept the target
    // out of the 15 deg cone (maxLockT 0.00-0.05 s across every run
    // before the lead pursuit). Without a target, climb to 950 m
    // and level as before.
    let pitchTarget: number;
    if (playerTarget && player.pos.y > 300) {
      const dx = playerTarget.pos.x - player.pos.x;
      const dz = playerTarget.pos.z - player.pos.z;
      const elev = Math.atan2(
        playerTarget.pos.y - player.pos.y,
        Math.hypot(dx, dz),
      );
      pitchTarget = clamp(-elev, (-15 * Math.PI) / 180, (15 * Math.PI) / 180); // +x = nose down
      if (player.pos.y < 300) pitchTarget = Math.min(pitchTarget, 0); // never dive into the terrain
    } else if (player.pos.y < 950) {
      pitchTarget = (-15 * Math.PI) / 180; // climb at 15 deg nose-up (pass 14: faster climb-out)
    } else {
      pitchTarget = clamp((player.pos.y - ALT_TARGET) * 0.0004, (-3.5 * Math.PI) / 180, (3.5 * Math.PI) / 180); // above target -> nose down (+x), below -> nose up
    }
    player.pitchIn = clamp((pitchTarget - pitchNow) * 3.0, -0.5, 0.5);
    // Negative-AoA guard (pass 12, Worker E): the same trap the AI
    // had — holding a fixed pitch attitude while the velocity vector
    // is far above the nose leaves AoA -40..-60 deg and the jet
    // zoom-climbs at +100 m/s to 3000 m (max|vsi| 109-119 m/s).
    // Pull the nose up toward the velocity vector, tapered.
    if (player.aoaDeg < -10) {
      player.pitchIn = Math.min(
        player.pitchIn,
        clamp((player.aoaDeg + 10) * 0.03, -0.4, 0),
      );
    }
    const curHdg = Math.atan2(player.fwd().x, player.fwd().z);
    let hdgErr = desHdg - curHdg;
    while (hdgErr > Math.PI) hdgErr -= 2 * Math.PI;
    while (hdgErr < -Math.PI) hdgErr += 2 * Math.PI;
    // Two-loop heading (pass 12, Worker E) — the AI's own steering,
    // not the P-loop bankDes = hdgErr * 1.0. The P-loop banked
    // directly from the heading error and oscillated +-20 deg around
    // the bearing, so the nose dipped into the 15 deg cone for a
    // single frame at a time (1 frame out of 4169) and lockT never
    // built past 0.05 s. Loop 1 commands a turn rate from the
    // heading error (gain 1.5, capped at 0.6x the ~18 deg/s
    // sustained rate); loop 2 smooths it at 6/s and derives the
    // bank from the rate (phi = atan(V*omega/g)), which is the
    // coordinated bank for that rate and does not overshoot.
    const omegaDes = clamp(hdgErr * 1.5, -0.31 * 0.6, 0.31 * 0.6);
    playerOmegaSm += (omegaDes - playerOmegaSm) * Math.min(1, dt * 6);
    const bankDes = clamp(
      -Math.atan((playerOmegaSm * Math.max(player.ias, 1)) / 9.81),
      (-70 * Math.PI) / 180,
      (70 * Math.PI) / 180,
    );
    const bankNow = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').z;
    player.rollIn = clamp((bankDes - bankNow) * 2.0, -1, 1);
  };

  // Player missile lock scan, verbatim from main.ts playerInput().
  const lockScan = (dt: number): void => {
    if (!player.alive) return;
    let best: Aircraft | null = null;
    let bd = LOCK_RANGE;
    for (const e of enemies) {
      if (!e.alive || e.dying) continue;
      const to = e.pos.clone().sub(player.pos);
      const d = to.length();
      if (d < bd && player.fwd().angleTo(to.normalize()) * R2D < LOCK_CONE_DEG) {
        bd = d;
        best = e;
      }
    }
    if (best) {
      player.lockTarget = best;
      player.lockT += dt;
      lockSeconds += dt;
      // Fire ONCE per lock, inside the missile's effective envelope
      // (pass 12, Worker E): the old scan fired a missile EVERY
      // frame while locked (no cooldown, no msl decrement — main.ts
      // fires on a button press with msl--), spamming ~50 missiles
      // at a fleeing target at 2000 m. A 280 m/s missile closes on
      // a 150 m/s fleeing target at only ~130 m/s, so 2000 m takes
      // ~15 s — longer than the 12 s missile life — and every one
      // expired. Effective range vs a fleeing target is ~1500 m;
      // head-on or crossing targets close faster and can be engaged
      // further out, but 1500 m is the safe envelope for the probe.
      if (
        player.lockT > LOCK_TIME &&
        player.msl > 0 &&
        !firedForLock &&
        bd < 1500
      ) {
        // Pass 14: log the shot geometry — the enemy dumps flares
        // while the player is behind it (flareCd 1.5 s, missile
        // break 2.2/s while flareCd > 0.4), so a tail-chase missile
        // is almost always decoyed and only a head-on shot (where
        // the enemy is not flaring) can hit. The log shows which
        // geometry each shot was and whether the target was flaring.
        const tgt = best as Aircraft & { flareCd: number };
        console.log(
          `  PLAYER MSL: t=${simT.toFixed(1)}s range=${bd.toFixed(0)}m target=${enemies.indexOf(best)} flareCd=${tgt.flareCd.toFixed(2)} targetHp=${best.hp.toFixed(0)}`,
        );
        weapons.launchMissile(player, best);
        playerMslFired++;
        firedForLock = true;
      }
    } else {
      player.lockTarget = null;
      player.lockT = Math.max(0, player.lockT - dt * 2);
      firedForLock = false;
    }
    maxLockT = Math.max(maxLockT, player.lockT);
  };

  // Pass 15: LIVE frame loop, verbatim from main.ts animate():
  //   rawDt = min(clock.getDelta(), 0.25)   (here: 1/fps, ground vs air)
  //   elapsed += rawDt                       (REAL time)
  //   simDt = min(rawDt, 0.1); sub = ceil(simDt/0.05); dt = simDt/sub
  //   ai.update(dt, player, carrier, elapsed)  (AI clock = elapsed)
  // The old probe advanced sim by the full rawDt and passed simT as
  // the AI's time — sim == real — which hides the low-fps divergence.
  while (elapsed < REAL_SECONDS) {
    const rawDt = Math.min(player.onSurface ? 1 / FPS_GROUND : 1 / FPS_AIR, 0.25);
    elapsed += rawDt;
    const simDt = Math.min(rawDt, 0.1);
    const sub = Math.max(1, Math.ceil(simDt / 0.05));
    const dt = simDt / sub;
    for (let s = 0; s < sub; s++) {
      simT += dt;
      frame++;
      flyPlayer(dt);
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      player.step(dt, WIND, null);
      if (airborneAt < 0 && !player.onSurface && !player.grounded) {
        airborneAt = simT;
        airborneAtReal = elapsed;
        enemyDistAtAirborne = enemies[0]!.pos.distanceTo(player.pos);
        // Loiter orbit point the AI is chasing (time = elapsed):
        const oa = elapsed * 0.08;
        orbitLagAtAirborne = Math.hypot(
          enemies[0]!.pos.x - (player.pos.x + Math.cos(oa) * 800),
          enemies[0]!.pos.y - (player.pos.y + 500),
          enemies[0]!.pos.z - (player.pos.z + Math.sin(oa) * 800),
        );
      }
      if (elapsed < TAKEOFF_DELAY_S) {
        maxEnemyDistDuringHold = Math.max(
          maxEnemyDistDuringHold,
          enemies[0]!.pos.distanceTo(player.pos),
        );
      }
      if (player.alive && !player.dying) {
        maxPlayerAlt = Math.max(maxPlayerAlt, player.pos.y);
        maxPlayerVsi = Math.max(maxPlayerVsi, Math.abs(player.vsi));
        maxPlayerG = Math.max(maxPlayerG, player.gload);
      }
      lockScan(dt);
      // Pass 15 lock-geometry instrumentation: the nearest enemy's
      // nose angle / range / altitude gap every substep, so a 0.00 s
      // lock can be attributed to heading, altitude or range.
      {
        let bestAng = Infinity;
        let bestD = Infinity;
        for (const e of enemies) {
          if (!e.alive || e.dying) continue;
          const to = e.pos.clone().sub(player.pos);
          const d = to.length();
          if (d < bestD) {
            bestD = d;
            bestAng = player.fwd().angleTo(to.normalize()) * R2D;
          }
        }
        if (bestD < 2500) {
          minLockAngle = Math.min(minLockAngle, bestAng);
          if (bestAng < 25) timeWithin25 += dt;
          if (bestAng < 15) timeWithin15 += dt;
        }
      }
      for (let j = 0; j < enemies.length; j++) {
        const e = enemies[j]!;
        const ai = ais[j]!;
        if (!e.alive) {
          if (enemyDeathAt < 0) enemyDeathAt = simT;
          continue;
        }
        const ctrl = ai.update(dt, player, carrierStub, elapsed);
        stateSeconds[ai.state] = (stateSeconds[ai.state] ?? 0) + dt;
        if (ai.state === 'engage') {
          engageSeconds += dt;
          engageSecondsReal += rawDt / sub;
          if (firstEngageAt < 0) firstEngageAt = simT;
          if (firstEngageAtReal < 0) firstEngageAtReal = elapsed;
        }
        if (ai.fireGun) {
          weapons.aiGunShot(e, player, ai.skill.err);
          aiGunShots++;
          if (firstFireAt < 0) firstFireAt = simT;
        }
        if (ai.fireMissile) {
          const before = e.msl;
          weapons.launchMissile(e, player);
          if (e.msl < before) aiMslLaunched++;
          if (firstFireAt < 0) firstFireAt = simT;
        }
        if (ai.dropBomb) {
          weapons.dropBomb(e);
          aiBombsDropped++;
        }
        const d = e.pos.distanceTo(player.pos);
        minDist = Math.min(minDist, d);
        maxEnemyG = Math.max(maxEnemyG, e.gload);
        if (Math.abs(simT % 1) < dt) {
          const ctx = deathCtx[j] ?? (deathCtx[j] = []);
          const eul = new THREE.Euler().setFromQuaternion(e.quat, 'YXZ');
          ctx.push(
            `t=${simT.toFixed(0)} y=${e.pos.y.toFixed(0)} hf=${heightAtLive(e.pos.x, e.pos.z).toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt aoa=${e.aoaDeg.toFixed(1)} g=${e.gload.toFixed(1)} vsi=${e.vsi.toFixed(0)} pitch=${(-eul.x * R2D).toFixed(0)} bank=${(eul.z * R2D).toFixed(0)} pitchIn=${e.ai.lastPitchIn.toFixed(2)} rollIn=${e.ai.lastRollIn.toFixed(2)} vsDes=${e.ai.lastVsDes.toFixed(0)} terrTurn=${e.ai.lastTerrainTurn.toFixed(2)} state=${e.ai.state} stall=${e.stalled}`,
          );
          if (ctx.length > 25) ctx.shift();
        }
        if (d < 2200) timeWithin2200 += dt;
        if (d < 900) timeWithin900 += dt;
        e.step(dt, ctrl, WIND, heightAtLive, null);
        if (!e.alive && enemyDeathAt < 0) enemyDeathAt = simT;
        if (!e.alive && terrainDeathAt < 0 && (e.lastCrashReason === 'hard landing' || e.lastCrashReason === 'terrain')) terrainDeathAt = simT;
        if (!e.alive && enemyDeathReasons.length < 4) {
          enemyDeathReasons.push(
            `e${j} t=${simT.toFixed(1)}s reason=${e.lastCrashReason ?? '-'} y=${e.pos.y.toFixed(0)} hf=${heightAtLive(e.pos.x, e.pos.z).toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt state=${ai.state}` +
              `\n    ${(deathCtx[j] ?? []).join('\n    ')}`,
          );
        }
      }
      weapons.update(dt, enemies);
      // Live-faithful respawn: a shot-down player picks a new life
      // and takes off again (respawn picker), so the run measures
      // engagement over the full 240 s instead of ending at the
      // first kill. resetAt restores hp 100 / msl 6; the probe's
      // hp-1000 scaffolding is re-applied.
      if (!player.alive) {
        if (playerDeathAt < 0) playerDeathAt = simT;
        playerDeaths++;
        playerHpLostTotal += Math.max(0, 1000 - player.hp);
        playerDeathReasons.push(
          `t=${simT.toFixed(1)}s reason=${player.lastCrashReason ?? player.dyingReason ?? '-'} hpLost=${(1000 - player.hp).toFixed(0)}`,
        );
        player.resetAt(alpha.x - origin.cumulativeShift.x, alpha.elev + 2.25, alpha.z - origin.cumulativeShift.z, alpha.hdg);
        player.hp = 1000;
        playerHpLast = 1000;
        breakUntil = -1;
        breakTarget = null;
      }
    }
    const shift = origin.shiftIfNeeded(player.pos);
    if (shift) origin.applyShift(shift);
    if (elapsed - lastLogSim >= 30) {
      lastLogSim = elapsed;
      console.log(
        `  [t=${elapsed.toFixed(0)}s real / ${simT.toFixed(0)}s sim] player y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt vsi=${player.vsi.toFixed(0)} aoa=${player.aoaDeg.toFixed(1)} hp=${player.hp.toFixed(0)} lockT=${player.lockT.toFixed(2)} msl=${player.msl} alive=${player.alive} ` +
          enemies
            .map(
              (e, j) =>
                `e${j} ${e.alive ? 'alive' : 'DEAD'} ${ais[j]!.state} d=${e.pos.distanceTo(player.pos).toFixed(0)}m y=${e.pos.y.toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt hp=${e.hp.toFixed(0)} msl=${e.msl}`,
            )
            .join(' '),
      );
    }
    // Pass 15: break-window trace — while the cruise player is
    // breaking toward its attacker, log the lock geometry every 1 s
    // real so the 0.00 s lock can be seen, not inferred.
    if (IS_CRUISE && elapsed < breakUntil && breakTarget && breakTarget.alive) {
      if (elapsed - (lastBreakLog ?? -10) >= 1) {
        lastBreakLog = elapsed;
        const toB = breakTarget.pos.clone().sub(player.pos);
        const dB = toB.length();
        const angB = player.fwd().angleTo(toB.normalize()) * R2D;
        const bankP = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').z * R2D;
        console.log(
          `  [break t=${elapsed.toFixed(0)}s real] d=${dB.toFixed(0)}m noseAng=${angB.toFixed(1)}deg dAlt=${(breakTarget.pos.y - player.pos.y).toFixed(0)}m lockT=${player.lockT.toFixed(2)} playerBank=${bankP.toFixed(0)}deg playerY=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt enemyState=${breakTarget.ai.state} enemyY=${breakTarget.pos.y.toFixed(0)} enemyIas=${(breakTarget.ias * 1.944).toFixed(0)}kt`,
        );
      }
    }
    if (IS_CRUISE && elapsed - (lastTraceSim ?? -10) >= 2) {
      lastTraceSim = elapsed;
      const e = enemies[0]!;
      const ai0 = ais[0]!;
      const noseAng = e.alive
        ? (e.fwd().angleTo(e.pos.clone().sub(player.pos).normalize().negate()) * R2D).toFixed(0)
        : '-';
      const eEuler = new THREE.Euler().setFromQuaternion(e.quat, 'YXZ');
      const pitchDbg = (new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x * R2D).toFixed(1);
      console.log(
        `  [trace t=${elapsed.toFixed(0)}s real / ${simT.toFixed(0)}s sim] e0 ${e.alive ? ai0.state : 'DEAD'} d=${e.pos.distanceTo(player.pos).toFixed(0)}m noseToPlayer=${noseAng}deg y=${e.pos.y.toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt ePitch=${(eEuler.x * R2D).toFixed(1)} eBank=${(eEuler.z * R2D).toFixed(0)} eAoA=${e.aoaDeg.toFixed(1)} eVsi=${e.vsi.toFixed(0)} eThr=${e.thrTarget.toFixed(2)} pitchIn=${ai0.lastPitchIn.toFixed(2)} rollIn=${ai0.lastRollIn.toFixed(2)} vsDes=${ai0.lastVsDes.toFixed(0)} terrTurn=${ai0.lastTerrainTurn.toFixed(2)} spin=${e.spin.toFixed(2)} stall=${e.stalled} beta=${e.betaDeg.toFixed(1)} g=${e.gload.toFixed(1)} playerFromAlpha=${Math.hypot(player.pos.x + origin.cumulativeShift.x - alpha.x, player.pos.z + origin.cumulativeShift.z - alpha.z).toFixed(0)}m playerY=${player.pos.y.toFixed(0)} pitch=${pitchDbg} pitchIn=${player.pitchIn.toFixed(2)} thr=${player.thrTarget.toFixed(2)} vsi=${player.vsi.toFixed(0)}`,
      );
    }
  }
  const playerHpLost = playerHpLostTotal + (player.alive ? Math.max(0, 1000 - player.hp) : 0);
  const enemyHpLost = enemies.reduce(
    (sum, e, j) => sum + (enemyHpStart[j]! - e.hp),
    0,
  );
  console.log(
    `  summary: elapsed=${elapsed.toFixed(1)}s real simT=${simT.toFixed(1)}s simRatio=${(simT / Math.max(elapsed, 0.001)).toFixed(2)} airborneAt=${airborneAt >= 0 ? airborneAt.toFixed(1) + 's sim / ' + airborneAtReal.toFixed(1) + 's real' : 'never'} enemyDistAtAirborne=${enemyDistAtAirborne >= 0 ? enemyDistAtAirborne.toFixed(0) + 'm' : '-'} orbitLagAtAirborne=${orbitLagAtAirborne >= 0 ? orbitLagAtAirborne.toFixed(0) + 'm' : '-'} maxEnemyDistDuringHold=${maxEnemyDistDuringHold.toFixed(0)}m maxAlt=${maxPlayerAlt.toFixed(0)}m max|vsi|=${maxPlayerVsi.toFixed(0)}m/s maxEnemyG=${maxEnemyG.toFixed(1)} maxPlayerG=${maxPlayerG.toFixed(1)} ` +
      `firstEngageAt=${firstEngageAt >= 0 ? firstEngageAt.toFixed(1) + 's sim / ' + firstEngageAtReal.toFixed(1) + 's real' : 'never'} engageSeconds=${engageSeconds.toFixed(1)}s sim / ${engageSecondsReal.toFixed(1)}s real ` +
      `firstFireAt=${firstFireAt >= 0 ? firstFireAt.toFixed(1) + 's' : 'never'} aiGunShots=${aiGunShots} aiMslLaunched=${aiMslLaunched} aiBombs=${aiBombsDropped} ` +
      `minDist=${minDist.toFixed(0)}m within2200=${timeWithin2200.toFixed(1)}s within900=${timeWithin900.toFixed(1)}s ` +
      `maxLockT=${maxLockT.toFixed(2)}s lockSeconds=${lockSeconds.toFixed(1)}s minLockAngle=${minLockAngle === Infinity ? '-' : minLockAngle.toFixed(1) + 'deg'} within25=${timeWithin25.toFixed(1)}s within15=${timeWithin15.toFixed(1)}s playerMslFired=${playerMslFired} (msl ${playerMslStart}->${player.msl}) ` +
      `playerHpLost=${playerHpLost.toFixed(0)} enemyHpLost=${enemyHpLost.toFixed(0)} states=${JSON.stringify(stateSeconds)} ` +
      `enemyDeathAt=${enemyDeathAt >= 0 ? enemyDeathAt.toFixed(1) + 's' : 'none'} terrainDeathAt=${terrainDeathAt >= 0 ? terrainDeathAt.toFixed(1) + 's' : 'none'} playerDeaths=${playerDeaths} firstPlayerDeathAt=${playerDeathAt >= 0 ? playerDeathAt.toFixed(1) + 's' : 'none'}`,
  );
  for (const r of enemyDeathReasons) console.log(`  ENEMY DEATH: ${r}`);
  for (const r of playerDeathReasons) console.log(`  PLAYER DEATH: ${r}`);

  console.log(`  scenario=${SCENARIO} player=${PLAYER_KEY} enemies=${ENEMY_COUNT} real=${REAL_SECONDS}s fpsGround=${FPS_GROUND} fpsAir=${FPS_AIR} cruiseSpd=${CRUISE_SPD}`);
  if (IS_CRUISE) {
    // Pass 15 live reproduction: the live round showed NO LOCK for
    // 5:30 — the enemy never closed. Bars are in REAL time (the
    // player's experience): the enemy must find and fight a player
    // who is NOT flying toward it, at live frame rates.
    check('cruise: enemy stays near the player through the ground phase (dist at airborne < 2600 m)', enemyDistAtAirborne >= 0 && enemyDistAtAirborne < 2600, `enemyDistAtAirborne=${enemyDistAtAirborne.toFixed(0)}m maxDuringHold=${maxEnemyDistDuringHold.toFixed(0)}m orbitLag=${orbitLagAtAirborne.toFixed(0)}m`);
    check('cruise: AI engages within 60 s REAL of the player going airborne', firstEngageAtReal >= 0 && airborneAtReal >= 0 && firstEngageAtReal - airborneAtReal <= 60, firstEngageAtReal >= 0 ? `first at ${firstEngageAtReal.toFixed(1)}s real (airborne ${airborneAtReal.toFixed(1)}s real)` : 'never engaged');
    check('cruise: AI stays engaged (total engage time >= 30 s sim)', engageSeconds >= 30, `engageSeconds=${engageSeconds.toFixed(1)}s sim / ${engageSecondsReal.toFixed(1)}s real states=${JSON.stringify(stateSeconds)}`);
    check('cruise: AI closes to gun range (min dist < 900 m) and holds < 2200 m for >= 20 s', minDist < 900 && timeWithin2200 >= 20, `minDist=${minDist.toFixed(0)}m within2200=${timeWithin2200.toFixed(1)}s within900=${timeWithin900.toFixed(1)}s`);
    check('cruise: AI fires on the player (gun and/or missile, and the fire lands)', (aiGunShots >= 1 || aiMslLaunched >= 1) && playerHpLost > 0, `gun=${aiGunShots} msl=${aiMslLaunched} firstFireAt=${firstFireAt >= 0 ? firstFireAt.toFixed(1) + 's' : 'never'} playerHpLost=${playerHpLost.toFixed(0)}`);
    check('cruise: player achieves full missile lock (lockT > 1.2 s)', maxLockT > LOCK_TIME, `maxLockT=${maxLockT.toFixed(2)}s lockSeconds=${lockSeconds.toFixed(1)}s`);
    check('cruise: enemy survives (no terrain death)', terrainDeathAt < 0, terrainDeathAt >= 0 ? `first terrain death at t=${terrainDeathAt.toFixed(1)}s` : (enemyDeathAt >= 0 ? `shot down at t=${enemyDeathAt.toFixed(1)}s (legitimate player kill)` : 'alive'));
    check('cruise: probe runs the full real budget (player respawns like live)', elapsed >= REAL_SECONDS - 1, `elapsed=${elapsed.toFixed(1)}s playerDeaths=${playerDeaths}`);
    for (const e of enemies) e.dispose();
    player.dispose();
    console.log(failures === 0 ? 'ALL CRUISE CHECKS PASSED' : `${failures} CRUISE CHECK(S) FAILED`);
    process.exit(failures === 0 ? 0 : 1);
  }
  check('scripted player flies a sane profile (max alt < 3000 m, max |vsi| < 70 m/s — a dogfighting player climbs/dives at up to 15 deg, i.e. ~39 m/s at 150 m/s, and a hard pursuit pull can reach ~60 m/s; the old zoom-climb failure was 20,000 m at +50..100 m/s sustained, max|vsi| 109-119)', maxPlayerAlt < 3000 && maxPlayerVsi < 70, `maxAlt=${maxPlayerAlt.toFixed(0)}m max|vsi|=${maxPlayerVsi.toFixed(0)}m/s`);
  check('AI engages within 30 s REAL of the player going airborne', firstEngageAtReal >= 0 && airborneAtReal >= 0 && firstEngageAtReal - airborneAtReal <= 30, firstEngageAtReal >= 0 ? `first at ${firstEngageAtReal.toFixed(1)}s real (airborne ${airborneAtReal.toFixed(1)}s real)` : 'never engaged');
  check('AI stays engaged (total engage time >= 60 s across enemies)', engageSeconds >= 60, `engageSeconds=${engageSeconds.toFixed(1)}s states=${JSON.stringify(stateSeconds)}`);
  check('AI closes to gun range (min dist < 900 m) and holds < 2200 m for >= 20 s', minDist < 900 && timeWithin2200 >= 20, `minDist=${minDist.toFixed(0)}m within2200=${timeWithin2200.toFixed(1)}s within900=${timeWithin900.toFixed(1)}s`);
  // Pass 12 (coordinator): the AI fires on the player with gun OR
  // missile, and the fire must LAND (playerHpLost > 0). Missiles are
  // the AI's primary air-to-air weapon (4-13 launched per run, the
  // player shot down 1-3x); the gun is close-range and geometry-
  // dependent — it fired 5x in one configuration and 0-1x in others
  // (the nose is >45 deg off in a sustained matched-speed turning
  // fight), so a gun>=5 bar makes the probe fail on geometry, not on
  // engagement. The gun count is still reported, and a run with
  // zero AI fire of either kind fails here.
  check('AI fires on the player (gun and/or missile, and the fire lands)', (aiGunShots >= 1 || aiMslLaunched >= 1) && playerHpLost > 0, `gun=${aiGunShots} msl=${aiMslLaunched} firstFireAt=${firstFireAt >= 0 ? firstFireAt.toFixed(1) + 's' : 'never'} playerHpLost=${playerHpLost.toFixed(0)}`);
  check('AI fire lands (player takes damage)', playerHpLost > 0, `playerHpLost=${playerHpLost.toFixed(0)}`);
  check('player achieves full missile lock (lockT > 1.2 s)', maxLockT > LOCK_TIME, `maxLockT=${maxLockT.toFixed(2)}s lockSeconds=${lockSeconds.toFixed(1)}s`);
  check('player fires a missile on lock (msl decreases)', playerMslFired >= 1, `playerMslFired=${playerMslFired} msl ${playerMslStart}->${player.msl}`);
  check('player missile damages an enemy', enemyHpLost > 0, `enemyHpLost=${enemyHpLost.toFixed(0)}`);
  check('enemies survive 240 s (no terrain death)', terrainDeathAt < 0, terrainDeathAt >= 0 ? `first terrain death at t=${terrainDeathAt.toFixed(1)}s` : (enemyDeathAt >= 0 ? `one shot down at t=${enemyDeathAt.toFixed(1)}s (legitimate player kill)` : 'all alive'));
  check('probe runs the full real budget (player respawns like live)', elapsed >= REAL_SECONDS - 1, `elapsed=${elapsed.toFixed(1)}s playerDeaths=${playerDeaths}`);

  for (const e of enemies) e.dispose();
  player.dispose();
  console.log(failures === 0 ? 'ALL AI-LIVE CHECKS PASSED' : `${failures} AI-LIVE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
