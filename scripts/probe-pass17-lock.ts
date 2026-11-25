// probe-pass17-lock.ts — pass 17, Worker B: PLAYER LOCK / FIRE.
//
// The v4.10 live verdict: the enemy engages (a MiG-29 Veteran shot the
// F/A-18F down at ~300 kt / ~1,900 ft) but the player can NEVER lock or
// fire — NO LOCK never changed in ~5 flights, MSL stayed at 6, gun did
// nothing. The harness pursuit probes (probe-ai-live / probe-ai-live15)
// use FOUR enemies, where one is almost always approaching head-on, so
// the pursuit player locks (maxLockT 5.36 s in probe-ai-live15 pursuit,
// 180 s real). The live check used ONE enemy. This probe closes that
// gap: a competent pursuit pilot (sticky most-ahead target, 3D lead
// pursuit — the probe-ai-live15 pursuit autopilot) against 1 enemy, 4
// enemies, and a ~700 kt level case.
//
// Scenarios:
//   one  — 1 enemy (MiG-29), F-18 player (the live jet), pursuit.
//   four — 4 enemies (MiG-29/Su-34), F-22 player, pursuit.
//   fast — 1 enemy, F-15E player, pursuit at ~700 kt (360 m/s).
//
// Live path: variable frame dt (4 fps on the ground, 11 fps in the
// air — rawDt up to 0.25 s, simDt capped at 0.1 s/frame), live wind
// 3 m/s, the REAL startRound enemy spawn code, real GLBs with a
// globalThis.Image stub and an assertion that modelStatus === 'ok',
// and REAL input events for fire: a KeyM keydown is dispatched into
// the real PCFlightInput window handler and read back through
// consumeEvents() (the exact path main.ts uses); a KeyT keydown is
// dispatched as a negative control (there is no T binding — the help
// text and PCFlightInput both say M is missile); Space is held the
// same way for the gun. The MSL button in main.ts uses the identical
// lock condition (main.ts:891-895) and is verified by inspection —
// there is no DOM in this probe to click it.
//
// The lock scan under test is the REAL WeaponSystem.updateLock
// (pass 17, Worker B) — the production scan, moved there from
// main.ts playerInput() with identical semantics (15 deg cone,
// < 2500 m, lockT += dt, decay dt * 2 when out of cone, full lock at
// lockT > 1.2 s). No replica: the probe calls the method main.ts
// will call (see the lockScan() note).
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass17-lock.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass17-lock.mjs
//   && node /tmp/probe-pass17-lock.mjs [--scenario one|four|fast]
//   [--real 240] [--fps-ground 4] [--fps-air 11]

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
// Real-GLB stub: three's texture path references the bare `Image`
// identifier (the F-18 GLB's embedded textures fail with
// "Image is not defined" without it, while the untextured MiG-29
// loads). A data-URI src "loads" on the next microtask tick.
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class ImageStub {
    width = 1;
    height = 1;
    complete = false;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    private readonly listeners: Record<string, Array<() => void>> = {};
    addEventListener: (type: string, fn: () => void) => void;
    removeEventListener: () => void;
    constructor() {
      this.addEventListener = (type: string, fn: () => void) => {
        (this.listeners[type] ??= []).push(fn);
      };
      this.removeEventListener = () => {};
    }
    set src(_v: string) {
      this.complete = true;
      queueMicrotask(() => {
        this.onload?.();
        for (const fn of this.listeners['load'] ?? []) fn();
      });
    }
  };
}
// Minimal window stub so the REAL PCFlightInput can be constructed in
// Node: it only needs window.addEventListener (keydown/keyup/pointerup)
// and a canvas with addEventListener. The key handlers are captured
// here so the probe can dispatch REAL KeyboardEvent-shaped keydowns
// (KeyM / KeyT / Space) into them, exactly as the browser would.
const windowListeners: Record<string, Array<(e: unknown) => void>> = {};
if (typeof (globalThis as unknown as { window?: unknown }).window === 'undefined') {
  (globalThis as unknown as { window?: unknown }).window = {
    addEventListener: (type: string, fn: (e: unknown) => void) => {
      (windowListeners[type] ??= []).push(fn);
    },
    removeEventListener: () => {},
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { WeaponSystem, LOCK_TIME } from '../src/combat/WeaponSystem';
import { PCFlightInput } from '../src/input/PCFlightInput';
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
// Math.random() directly (gun error, flare break, ignition rolls), so
// two runs of the same code diverged wildly from unseeded randomness
// alone. A probe must be reproducible. Seed it from a SEPARATE LCG so
// the AI/spawn rng sequence above is untouched. This is probe
// scaffolding only — the live game keeps the real Math.random.
const rngMathState = { s: 987654321 };
Math.random = (): number => {
  rngMathState.s = (rngMathState.s * 1103515245 + 12345) & 0x7fffffff;
  return rngMathState.s / 0x7fffffff;
};

const R2D = 180 / Math.PI;
const argv = process.argv.slice(2);
const scenarioArg = (argv[argv.indexOf('--scenario') + 1] ?? 'one') as string;
const SCENARIO: 'one' | 'four' | 'fast' =
  scenarioArg === 'four' || scenarioArg === 'fast' ? scenarioArg : 'one';
const realIdx = argv.indexOf('--real');
const REAL_SECONDS = realIdx >= 0 ? Number(argv[realIdx + 1] ?? 240) : 240;
const fgIdx = argv.indexOf('--fps-ground');
const FPS_GROUND = fgIdx >= 0 ? Number(argv[fgIdx + 1] ?? 4) : 4;
const faIdx = argv.indexOf('--fps-air');
const FPS_AIR = faIdx >= 0 ? Number(argv[faIdx + 1] ?? 11) : 11;
const ENEMY_COUNT = SCENARIO === 'four' ? 4 : 1;
const PLAYER_KEY = SCENARIO === 'one' ? 'fa18' : SCENARIO === 'fast' ? 'f15e' : 'f22';
const SPD_TARGET = SCENARIO === 'fast' ? 360 : 150; // m/s (~700 kt / ~290 kt)
const LOCK_CONE_DEG = 15;
// Diagnostic overrides (probe only). --spawn-ahead re-asserts the
// post-fix dead-ahead spawn; --spawn-old restores the pre-fix
// production offsets (side 120/-150, up 120/180) so a post-fix run
// can prove WeaponSystem.updateLock is semantics-identical to the old
// inline scan: with --spawn-old the 1-enemy case must still show
// maxLockT 0.00 s. (The --cone / --decay overrides used during the
// diagnosis were removed once the probe started calling the real
// updateLock: the production cone, range, threshold and decay are
// fixed in WeaponSystem, not probe inputs. The matrix result stands:
// cone 25 deg -> maxLockT 0.85 s (FAIL), cone 30 deg -> 1.30 s (PASS,
// via the opening head-on only), decay 0.5x -> 0.00 s (no in-cone
// time to retain), spawn-ahead with the production 15 deg cone ->
// 1.70 s (PASS).)
const SPAWN_AHEAD = argv.includes('--spawn-ahead');
const SPAWN_OLD = argv.includes('--spawn-old');

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
  check('player GLB loads (modelStatus === ok)', player.modelStatus === 'ok', `modelStatus=${player.modelStatus} ${player.modelError ?? ''}`);
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
  // Probe scaffolding: hp 1000 so a working AI's hits are measured
  // as damage over the full run instead of ending it early.
  player.hp = 1000;

  // ---- startRound's enemy spawn, verbatim from main.ts ----
  // (live defaults: veteran skill, spawn distance 1200 m; the first
  // two spawn 450-600 m ahead of the player, facing the player.)
  const carrierStub = {
    deckHeightAt: () => -Infinity,
  } as unknown as Parameters<AircraftAIController['update']>[2];
  const enemies: Aircraft[] = [];
  const ais: AircraftAIController[] = [];
  const spawnHeadings: number[] = [];
  const spawnAngles: number[] = [];
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
    check(`enemy e${i} (${key}) GLB loads (modelStatus === ok)`, e.modelStatus === 'ok', `modelStatus=${e.modelStatus} ${e.modelError ?? ''}`);
    let ex: number;
    let ey: number;
    let ez: number;
    if (i < 2) {
      const fwd = player.fwd();
      const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
      const along = 450 + i * 150;
      // Pass 17 fix (mirrors the main.ts startRound patch): the lead
      // enemies spawn DEAD-AHEAD (side 0, up 30/60) so the designed
      // head-on intercept sits inside the 15 deg lock cone. The
      // pre-fix offsets (side 120/-150, up 120/180) put both at
      // ~21 deg off the nose — outside the cone from frame one.
      // --spawn-old restores the pre-fix offsets for the
      // semantics check (updateLock must still show 0.00 s there).
      const side = SPAWN_OLD ? (i === 0 ? 120 : -150) : 0;
      const up = SPAWN_OLD ? 120 + i * 60 : 30 + i * 30;
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
    const hdgToPlayer =
      (Math.atan2(player.pos.x - ex, player.pos.z - ez) * 180) / Math.PI;
    spawnHeadings.push(hdgToPlayer);
    e.resetAt(ex, ey, ez, hdgToPlayer);
    e.setAirspeed(150);
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    const ai = new AircraftAIController(e, 'veteran', 'enemy', {
      groundHeight: heightAtLive,
      originShift: origin.cumulativeShift,
      rng,
    });
    origin.registerVector(ai.navTarget);
    ais.push(ai);
    enemies.push(e);
    // Spawn geometry: the nose angle from the (runway) player to
    // this enemy at spawn — the opening lock opportunity, if any.
    const toE = e.pos.clone().sub(player.pos);
    spawnAngles.push(player.fwd().angleTo(toE.normalize()) * R2D);
  }
  console.log(
    `  enemy spawn headings (deg, all facing player): ${spawnHeadings.map((h) => h.toFixed(0)).join(', ')} (player hdg ${alpha.hdg})`,
  );
  console.log(
    `  enemy spawn nose angles: ${spawnAngles.map((a) => a.toFixed(1)).join(', ')} deg (lock cone is ${LOCK_CONE_DEG} deg) dists: ${enemies.map((e) => e.pos.distanceTo(player.pos).toFixed(0)).join(', ')} m`,
  );

  // ---- REAL fire input path: PCFlightInput + real key events ----
  // The canvas is a stub that only needs addEventListener; the window
  // key handlers were captured in windowListeners above at import
  // time (PCFlightInput registers them in its constructor).
  const fakeCanvas = {
    addEventListener: () => {},
    removeEventListener: () => {},
  } as unknown as HTMLCanvasElement;
  const pcInput = new PCFlightInput(fakeCanvas);
  const dispatchKey = (code: string, type: 'keydown' | 'keyup'): void => {
    const ev = { code, repeat: false, preventDefault: () => {} };
    for (const fn of windowListeners[type] ?? []) fn(ev);
  };
  // Negative control, run BEFORE the loop: there is NO KeyT binding
  // (PCFlightInput maps missile to KeyM; the in-game help says
  // "Missile M"). A T press must not raise missilePressed.
  let keyTFiredMissile = false;
  {
    dispatchKey('KeyT', 'keydown');
    const evT = pcInput.consumeEvents();
    keyTFiredMissile = evT.missilePressed;
    dispatchKey('KeyT', 'keyup');
    // Drain the latch so the negative control cannot leak into the run.
    pcInput.consumeEvents();
    console.log(`  negative control: KeyT keydown -> missilePressed=${evT.missilePressed} (must be false)`);
  }
  // Positive control: a KeyM keydown with no lock must still raise
  // missilePressed (the event path works; the lock gate is what
  // blocks the launch, not the key).
  let keyMEventWorks = false;
  {
    dispatchKey('KeyM', 'keydown');
    const evM = pcInput.consumeEvents();
    keyMEventWorks = evM.missilePressed;
    dispatchKey('KeyM', 'keyup');
    pcInput.consumeEvents();
    console.log(`  positive control: KeyM keydown -> missilePressed=${evM.missilePressed} (must be true)`);
  }

  // ---- live loop state ----
  const vrMs = SPECS[PLAYER_KEY]!.rotateSpeedKt * 0.514444;
  const ALT_TARGET = 1000;
  let simT = 0;
  let elapsed = 0;
  let frame = 0;
  let airborneAt = -1;
  let firstEngageAt = -1;
  let firstFireAt = -1;
  let aiGunShots = 0;
  let aiMslLaunched = 0;
  let minDist = Infinity;
  let maxLockT = 0;
  let lockSeconds = 0;
  let playerMslFired = 0;
  let firedForLock = false; // one missile per lock acquisition
  let playerMslStart = player.msl;
  let missilePressedSeen = false; // a live KeyM press reached the gate
  let keyMDispatches = 0;
  let gunShots = 0;
  let gunDamage = 0;
  let enemyShotDown = false;
  let enemyShotDownAt = -1;
  let terrainDeathAt = -1;
  let playerDeaths = 0;
  let playerHpLostTotal = 0;
  let maxPlayerAlt = 0;
  let maxLevelKt = 0; // fastest level (|vsi| < 5) flight above 500 m
  let maxPlayerIasKt = 0;
  const enemyHpStart = enemies.map((e) => e.hp);
  let playerTarget: Aircraft | null = null; // sticky pursuit target
  let playerOmegaSm = 0;
  let playerHpLast = 1000;
  // Lock-geometry diagnostics: why is the 1-enemy lock 0.00 s?
  let minLockAngle = Infinity; // min nose angle to any enemy while < 2500 m
  let timeWithin25 = 0;
  let timeWithin15 = 0;
  let longestInCone = 0; // longest CONTINUOUS in-cone stretch, sim s
  let inConeStretches = 0; // number of in-cone entries
  let curStretch = 0;
  let inConeNow = false;
  const angleBuckets = { lt15: 0, b15_25: 0, b25_45: 0, b45_90: 0, gt90: 0 }; // substeps, nearest enemy < 2500 m
  let spaceDown = false;
  let lastLogSim = -30;
  const stateSeconds: Record<string, number> = {};
  // Rolling per-enemy context (last ~25 s at 1 s cadence) so a
  // terrain death can be diagnosed from what the jet was doing.
  const deathCtx: Record<number, string[]> = {};

  // Scripted pursuit pilot — LEAD pursuit, not pure pursuit (pass 17,
  // Worker B): a competent human in a turning fight steers at the LEAD
  // point (target pos + vel * tGo * 0.5, the same 0.5 lead the AI's own
  // break uses), turns at the MAX sustained rate (0.31 rad/s) and slows
  // to corner speed (120 m/s) while the nose is > 30 deg off, so the
  // turn actually closes. The probe-ai-live15 pursuit autopilot steers
  // at the target's CURRENT bearing at 0.6x rate — pure pursuit against
  // a turning target always lags the turn, which is why the 1-enemy
  // case never got the nose inside 15 deg at all (minLockAngle 20.7
  // deg, the spawn angle). The fast scenario keeps its 360 m/s speed
  // target (corner speed would defeat the ~700 kt point) and turns at
  // whatever rate that speed allows.
  const flyPlayer = (dt: number): void => {
    if (!player.alive) return;
    const noseAngToTarget = playerTarget?.alive
      ? player.fwd().angleTo(playerTarget.pos.clone().sub(player.pos).normalize()) * R2D
      : 0;
    const spdTarget =
      SCENARIO === 'fast'
        ? SPD_TARGET
        : noseAngToTarget > 30
          ? 120 // corner speed while turning hard onto the target
          : SPD_TARGET;
    player.thrTarget = clamp(
      player.thrTarget + (player.ias < spdTarget ? dt * 0.5 : -dt * 0.5),
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
    const pitchNow = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x;
    let desHdg = (90 * Math.PI) / 180;
    if (player.pos.y > 300) {
      if (!playerTarget || !playerTarget.alive || playerTarget.dying || playerTarget.pos.distanceTo(player.pos) > 4000) {
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
        // Lead pursuit: aim the nose at where the target WILL be,
        // not where it is — the heading and the pitch both track the
        // lead point (0.5 lead, as the AI's break does).
        const dLead = playerTarget.pos.distanceTo(player.pos);
        const leadPt = playerTarget.pos
          .clone()
          .addScaledVector(playerTarget.vel, (dLead / 280) * 0.5);
        desHdg = Math.atan2(leadPt.x - player.pos.x, leadPt.z - player.pos.z);
      }
    }
    let pitchTarget: number;
    if (playerTarget && player.pos.y > 300) {
      const dx = playerTarget.pos.x - player.pos.x;
      const dz = playerTarget.pos.z - player.pos.z;
      const dLead = Math.hypot(dx, dz);
      const leadPt = playerTarget.pos
        .clone()
        .addScaledVector(playerTarget.vel, (dLead / 280) * 0.5);
      const elev = Math.atan2(
        leadPt.y - player.pos.y,
        Math.hypot(leadPt.x - player.pos.x, leadPt.z - player.pos.z),
      );
      pitchTarget = clamp(-elev, (-15 * Math.PI) / 180, (15 * Math.PI) / 180);
      if (player.pos.y < 300) pitchTarget = Math.min(pitchTarget, 0);
    } else if (player.pos.y < 950) {
      pitchTarget = (-15 * Math.PI) / 180;
    } else {
      pitchTarget = clamp((player.pos.y - ALT_TARGET) * 0.0004, (-3.5 * Math.PI) / 180, (3.5 * Math.PI) / 180);
    }
    player.pitchIn = clamp((pitchTarget - pitchNow) * 3.0, -0.5, 0.5);
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
    const omegaDes = clamp(hdgErr * 1.5, -0.31, 0.31); // max sustained rate, not 0.6x
    playerOmegaSm += (omegaDes - playerOmegaSm) * Math.min(1, dt * 6);
    const bankDes = clamp(
      -Math.atan((playerOmegaSm * Math.max(player.ias, 1)) / 9.81),
      (-70 * Math.PI) / 180,
      (70 * Math.PI) / 180,
    );
    const bankNow = new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').z;
    player.rollIn = clamp((bankDes - bankNow) * 2.0, -1, 1);
  };

  // Player missile lock — the REAL production scan:
  // WeaponSystem.updateLock (pass 17, Worker B), moved there from
  // main.ts playerInput() with identical semantics (15 deg cone,
  // < 2500 m, lockT += dt, decay dt * 2, full lock at lockT > 1.2 s).
  // This probe used to replicate the main.ts block verbatim; it now
  // calls the method main.ts will call, so a drift between the probe
  // copy and the game is impossible. The fire-on-lock logic below (one missile
  // per acquisition, < 1500 m envelope, real KeyM keydown) is the
  // probe's, mirroring main.ts's fire gate.
  const lockScan = (dt: number): void => {
    if (!player.alive) return;
    weapons.updateLock(player, enemies, dt);
    const best = player.lockTarget;
    if (best) {
      lockSeconds += dt;
      const bd = best.pos.distanceTo(player.pos);
      // Fire ONCE per lock acquisition, inside the missile's
      // effective envelope (< 1500 m vs a fleeing target — a 280 m/s
      // missile closes on a 150 m/s target at only ~130 m/s, so
      // 2000 m takes ~15 s, longer than the 12 s missile life).
      // The press itself is a REAL KeyM keydown into PCFlightInput,
      // dispatched at the top of the next substep and read back
      // through consumeEvents() — the live fire path, not a direct
      // launchMissile call.
      if (
        player.lockT > LOCK_TIME &&
        player.msl > 0 &&
        !firedForLock &&
        bd < 1500
      ) {
        console.log(
          `  LOCKED t=${simT.toFixed(1)}s range=${bd.toFixed(0)}m target=${enemies.indexOf(best)} — dispatching real KeyM`,
        );
        pendingKeyM = true;
        firedForLock = true;
      }
    } else {
      firedForLock = false;
    }
    maxLockT = Math.max(maxLockT, player.lockT);
  };
  let pendingKeyM = false;

  // Live frame loop (probe-ai-live15 / main.ts animate()):
  //   rawDt = min(1/fps, 0.25); elapsed += rawDt (REAL time)
  //   simDt = min(rawDt, 0.1); sub = ceil(simDt/0.05); dt = simDt/sub
  //   ai.update(dt, player, carrier, elapsed) (AI clock = elapsed)
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
      }
      if (player.alive && !player.dying) {
        maxPlayerAlt = Math.max(maxPlayerAlt, player.pos.y);
        maxPlayerIasKt = Math.max(maxPlayerIasKt, player.ias * 1.944);
        if (player.pos.y > 500 && Math.abs(player.vsi) < 5) {
          maxLevelKt = Math.max(maxLevelKt, player.ias * 1.944);
        }
      }
      // --- fire input, exactly as main.ts playerInput() does it ---
      // A pending real KeyM press is dispatched into the window
      // handler first, then consumeEvents() reads it back; the
      // launch gate uses lockT as of the END of the previous
      // substep (the lock scan below runs after this block).
      if (pendingKeyM) {
        dispatchKey('KeyM', 'keydown');
        pendingKeyM = false;
        keyMDispatches++;
      }
      const ev = pcInput.consumeEvents();
      if (ev.missilePressed) missilePressedSeen = true;
      if (ev.missilePressed && player.lockT > LOCK_TIME && player.lockTarget) {
        weapons.launchMissile(player, player.lockTarget);
        playerMslFired++;
        console.log(
          `  PLAYER MSL FIRED t=${simT.toFixed(1)}s msl ${playerMslStart}->${player.msl} targetHp=${player.lockTarget.hp.toFixed(0)}`,
        );
        dispatchKey('KeyM', 'keyup');
      }
      // Gun: hold Space (real key events) while a target is in the
      // cone, exactly as main.ts does (firing || ev.fireHeld).
      const wantSpace = player.lockTarget !== null;
      if (wantSpace && !spaceDown) {
        dispatchKey('Space', 'keydown');
        spaceDown = true;
      } else if (!wantSpace && spaceDown) {
        dispatchKey('Space', 'keyup');
        spaceDown = false;
      }
      if (ev.fireHeld) {
        const before = enemies.map((e) => e.hp);
        weapons.fireGunAtTargets(player, enemies, dt);
        gunShots++;
        enemies.forEach((e, j) => {
          if (e.hp < before[j]!) gunDamage += before[j]! - e.hp;
        });
      }
      lockScan(dt);
      // Lock-geometry instrumentation on the NEAREST live enemy.
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
          if (bestAng < 15) {
            timeWithin15 += dt;
            if (!inConeNow) {
              inConeNow = true;
              inConeStretches++;
              curStretch = 0;
            }
            curStretch += dt;
            longestInCone = Math.max(longestInCone, curStretch);
          } else {
            inConeNow = false;
            curStretch = 0;
          }
          if (bestAng < 15) angleBuckets.lt15++;
          else if (bestAng < 25) angleBuckets.b15_25++;
          else if (bestAng < 45) angleBuckets.b25_45++;
          else if (bestAng < 90) angleBuckets.b45_90++;
          else angleBuckets.gt90++;
        } else {
          inConeNow = false;
          curStretch = 0;
        }
      }
      for (let j = 0; j < enemies.length; j++) {
        const e = enemies[j]!;
        const ai = ais[j]!;
        if (!e.alive) continue;
        const ctrl = ai.update(dt, player, carrierStub, elapsed);
        stateSeconds[ai.state] = (stateSeconds[ai.state] ?? 0) + dt;
        if (ai.state === 'engage' && firstEngageAt < 0) firstEngageAt = simT;
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
        if (ai.dropBomb) weapons.dropBomb(e);
        const d = e.pos.distanceTo(player.pos);
        minDist = Math.min(minDist, d);
        if (Math.abs(simT % 1) < dt) {
          const ctx = deathCtx[j] ?? (deathCtx[j] = []);
          const eul = new THREE.Euler().setFromQuaternion(e.quat, 'YXZ');
          ctx.push(
            `t=${simT.toFixed(0)} y=${e.pos.y.toFixed(0)} hf=${heightAtLive(e.pos.x, e.pos.z).toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt aoa=${e.aoaDeg.toFixed(1)} g=${e.gload.toFixed(1)} vsi=${e.vsi.toFixed(0)} pitch=${(-eul.x * R2D).toFixed(0)} bank=${(eul.z * R2D).toFixed(0)} pitchIn=${ai.lastPitchIn.toFixed(2)} rollIn=${ai.lastRollIn.toFixed(2)} vsDes=${ai.lastVsDes.toFixed(0)} state=${ai.state} stall=${e.stalled}`,
          );
          if (ctx.length > 25) ctx.shift();
        }
        e.step(dt, ctrl, WIND, heightAtLive, null);
        if (!e.alive && enemyShotDownAt < 0) {
          enemyShotDownAt = simT;
          if (e.lastHitBy?.isPlayer) enemyShotDown = true;
          if (terrainDeathAt < 0 && (e.lastCrashReason === 'hard landing' || e.lastCrashReason === 'terrain')) {
            terrainDeathAt = simT;
          }
          console.log(
            `  ENEMY e${j} DOWN t=${simT.toFixed(1)}s reason=${e.lastCrashReason ?? e.dyingReason ?? '-'} lastHitByPlayer=${e.lastHitBy?.isPlayer ?? false}` +
              `\n    ${(deathCtx[j] ?? []).join('\n    ')}`,
          );
        }
      }
      weapons.update(dt, enemies);
      if (!player.alive) {
        playerDeaths++;
        playerHpLostTotal += Math.max(0, 1000 - player.hp);
        player.resetAt(alpha.x - origin.cumulativeShift.x, alpha.elev + 2.25, alpha.z - origin.cumulativeShift.z, alpha.hdg);
        player.hp = 1000;
        playerHpLast = 1000;
        playerTarget = null;
        firedForLock = false;
        pendingKeyM = false;
        if (spaceDown) {
          dispatchKey('Space', 'keyup');
          spaceDown = false;
        }
      }
    }
    const shift = origin.shiftIfNeeded(player.pos);
    if (shift) origin.applyShift(shift);
    if (simT - lastLogSim >= 30) {
      lastLogSim = simT;
      console.log(
        `  [t=${elapsed.toFixed(0)}s real / ${simT.toFixed(0)}s sim] player y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt lockT=${player.lockT.toFixed(2)} msl=${player.msl} alive=${player.alive} ` +
          enemies
            .map(
              (e, j) =>
                `e${j} ${e.alive ? 'alive' : 'DEAD'} ${ais[j]!.state} d=${e.pos.distanceTo(player.pos).toFixed(0)}m y=${e.pos.y.toFixed(0)} ias=${(e.ias * 1.944).toFixed(0)}kt hp=${e.hp.toFixed(0)}`,
            )
            .join(' '),
      );
    }
  }
  if (spaceDown) dispatchKey('Space', 'keyup');
  const enemyHpLost = enemies.reduce(
    (sum, e, j) => sum + (enemyHpStart[j]! - e.hp),
    0,
  );
  const missileDamage = Math.max(0, enemyHpLost - gunDamage);
  console.log(
    `  summary: scenario=${SCENARIO} player=${PLAYER_KEY} enemies=${ENEMY_COUNT} elapsed=${elapsed.toFixed(1)}s real simT=${simT.toFixed(1)}s airborneAt=${airborneAt >= 0 ? airborneAt.toFixed(1) + 's sim' : 'never'} ` +
      `firstEngageAt=${firstEngageAt >= 0 ? firstEngageAt.toFixed(1) + 's' : 'never'} firstFireAt=${firstFireAt >= 0 ? firstFireAt.toFixed(1) + 's' : 'never'} aiGunShots=${aiGunShots} aiMslLaunched=${aiMslLaunched} minDist=${minDist.toFixed(0)}m ` +
      `maxLockT=${maxLockT.toFixed(2)}s lockSeconds=${lockSeconds.toFixed(1)}s timeWithin15=${timeWithin15.toFixed(1)}s timeWithin25=${timeWithin25.toFixed(1)}s longestInCone=${longestInCone.toFixed(2)}s inConeStretches=${inConeStretches} minLockAngle=${minLockAngle === Infinity ? '-' : minLockAngle.toFixed(1) + 'deg'} ` +
      `angleBuckets(<2500m, substeps)=<15:${angleBuckets.lt15} 15-25:${angleBuckets.b15_25} 25-45:${angleBuckets.b25_45} 45-90:${angleBuckets.b45_90} >90:${angleBuckets.gt90} ` +
      `keyMDispatches=${keyMDispatches} missilePressedSeen=${missilePressedSeen} playerMslFired=${playerMslFired} (msl ${playerMslStart}->${player.msl}) gunShots=${gunShots} gunDamage=${gunDamage.toFixed(0)} ` +
      `enemyHpLost=${enemyHpLost.toFixed(0)} missileDamage~${missileDamage.toFixed(0)} enemyShotDown=${enemyShotDown} (t=${enemyShotDownAt >= 0 ? enemyShotDownAt.toFixed(1) + 's' : '-'}) terrainDeathAt=${terrainDeathAt >= 0 ? terrainDeathAt.toFixed(1) + 's' : 'none'} ` +
      `playerDeaths=${playerDeaths} maxAlt=${maxPlayerAlt.toFixed(0)}m maxIas=${maxPlayerIasKt.toFixed(0)}kt maxLevel=${maxLevelKt.toFixed(0)}kt states=${JSON.stringify(stateSeconds)}`,
  );

  console.log(`  scenario=${SCENARIO} player=${PLAYER_KEY} enemies=${ENEMY_COUNT} real=${REAL_SECONDS}s fpsGround=${FPS_GROUND} fpsAir=${FPS_AIR} spawnAhead=${SPAWN_AHEAD} spawnOld=${SPAWN_OLD}`);
  check('fire input: real KeyM keydown reaches consumeEvents as missilePressed', keyMEventWorks, `keyMEventWorks=${keyMEventWorks}`);
  check('fire input: KeyT keydown does NOT raise missilePressed (no T binding)', !keyTFiredMissile, `keyTFiredMissile=${keyTFiredMissile}`);
  check('player achieves full missile lock (lockT > 1.2 s)', maxLockT > LOCK_TIME, `maxLockT=${maxLockT.toFixed(2)}s lockSeconds=${lockSeconds.toFixed(1)}s timeWithin15=${timeWithin15.toFixed(1)}s longestInCone=${longestInCone.toFixed(2)}s stretches=${inConeStretches} minLockAngle=${minLockAngle === Infinity ? '-' : minLockAngle.toFixed(1) + 'deg'}`);
  check('player fires a missile on lock via the real KeyM path (msl decreases)', playerMslFired >= 1 && missilePressedSeen, `playerMslFired=${playerMslFired} msl ${playerMslStart}->${player.msl} keyMDispatches=${keyMDispatches} missilePressedSeen=${missilePressedSeen}`);
  check('player missile damages an enemy (at least one full missile hit, 40 dmg)', missileDamage >= 40, `missileDamage~${missileDamage.toFixed(0)} enemyHpLost=${enemyHpLost.toFixed(0)} gunDamage=${gunDamage.toFixed(0)}`);
  check('enemy shot down by the player', enemyShotDown, enemyShotDownAt >= 0 ? `at t=${enemyShotDownAt.toFixed(1)}s` : 'never');
  if (SCENARIO === 'fast') {
    check('~700 kt level flight reached while fighting', maxLevelKt >= 650, `maxLevel=${maxLevelKt.toFixed(0)}kt maxIas=${maxPlayerIasKt.toFixed(0)}kt`);
  }
  for (const e of enemies) e.dispose();
  player.dispose();
  console.log(failures === 0 ? 'ALL PASS17-LOCK CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
