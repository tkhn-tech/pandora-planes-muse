// main.ts — boot + game loop (the integrator).
// Ported from the v2 template (pandora-template.html ~1921–2271 sandbox
// manager, camera, input merge, main loop, menu wiring, boot), keeping
// numeric behaviour: substeps <= 0.05 s (v1.1 pacing fix), first two
// enemies 450–600 m ahead / 120–180 m up (v1.1 fix), respawn timer tracked
// and cancelled by Restart / no-op if already alive (v1.1 fix), 1 game
// minute per real second, fog presets clear [900,5200] / cloud [700,3600] /
// storm [400,1900] / rain [500,2600], lock cone 15 deg / 2500 m / 1.2 s,
// VTOL pad credit +75, kill +100, survival +1/s, trap +100.
//
// Boot order: await RAPIER.init() BEFORE anything else (Rapier world,
// missiles and bombs need the WASM module).
//
// ASSUMED APIS (other workers own these files — confirm against their
// real exports; see the worker report):
//   ../physics/FlightPhysicsController — class Aircraft:
//     new Aircraft(key, isPlayer), attachModel(): Promise<void>,
//     resetAt(x,y,z,hdgDeg), step(dt, wind, turbAcc|null),
//     pos/vel/quat, fwd()/up()/right(), group: THREE.Group,
//     alive, isPlayer, spec {name, short, vtol, carrier, bombs},
//     thrTarget/throttle, pitchIn/rollIn/yawIn, flaps/airbrake/vector,
//     gear/hook/brake, fuel/hp, ias/vsi/aoa/gload, stalled, grounded,
//     onDeck, trapped, vtolCounted, msl/flares/bombs, gunCd/flareCd,
//     lockT, lockTarget: Aircraft|null, lastHitBy: Aircraft|null,
//     failEngine/failEngineSide/failFire/failHydro/failGearJam/failFuel,
//     wingDmg/engineDmg, applyDamage(amount, zone, byPlayer).
//     Crash consequences (explosion VFX, score, respawn) are owned HERE —
//     Aircraft should only flip alive=false and record lastHitBy.
//   ../ai/AIController — new AIController(ac, 'enemy'|'traffic', skill),
//     update(dt), pickNav().
//   ../world/PandoraWorldGenerator — new PandoraWorldGenerator(scene),
//     build(), groundHeight(x,z), runways: RunwayDef[],
//     originShift: THREE.Vector3, resetOrigin(),
//     shiftOriginIfNeeded(focus), update(dt, wind),
//     floraMaterials: { emissiveIntensity: number }[].
//   ../world/RunwayPlacer — new RunwayPlacer(scene),
//     place(runways), nearest(pos): RunwayDef.
//   ../world/HelicarrierController — new HelicarrierController(scene, world),
//     build(): Promise<void>, update(dt), pos/yaw/vel, alive,
//     pods: THREE.Object3D[], damagePod(i, dmg),
//     deckHeight(x,z), localPosition(p): THREE.Vector3|null,
//     reset(pos, yaw).
//   ../physics/IAerodynamicsSolver — FlightControls
//     { pitch, roll, yaw, throttle: number | null }.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from './physics/FlightPhysicsController';
import type { DeckProvider } from './physics/FlightPhysicsController';
import { AircraftAIController } from './entities/AircraftAIController';
import { SPECS } from './aircraft/AircraftSpec';
import { FloatingOrigin } from './world/FloatingOrigin';
import { HELICARRIER_GLB } from './assets/models/helicarrier';
import { PandoraWorldGenerator } from './world/PandoraWorldGenerator';
import type { RunwayDef } from './world/PandoraWorldGenerator';
import { RunwayPlacer } from './world/RunwayPlacer';
import { RUNWAY_TABLE } from './world/PandoraTerrainWorker';
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS, carrierLoop } from './entities/HelicarrierController';
import { WeaponSystem } from './combat/WeaponSystem';
import { CrashVFXManager } from './combat/CrashVFXManager';
import { FlightHUD } from './ui/FlightHUD';
import { SandboxMenu } from './ui/SandboxMenu';
import type { SandboxConfig } from './ui/SandboxMenu';
import { PCFlightInput } from './input/PCFlightInput';
import { MobileFlightInput } from './input/MobileFlightInput';

await RAPIER.init();

const VERSION_FOOTER = 'v4.1 · Vite + Rapier';
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const VTOL_PADS: [number, number][] = [
  [0, 60],
  [0, 110],
]; // marked pad centres, carrier-local (x, z), radius 16 m (G5)
const FAIL_CHOICES = [
  'engineL',
  'engineR',
  'engineFire',
  'hydraulic',
  'gearJam',
  'fuelLeak',
] as const;

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}
function $(id: string): HTMLElement | null {
  return document.getElementById(id);
}

// ---- audio (WebAudio, lazy — created on first gesture, template ~1680) ----
let AC: AudioContext | null = null;
let soundOn = true;
let lockOsc: OscillatorNode | null = null;
let lockGain: GainNode | null = null;

function audio(): AudioContext | null {
  if (!AC) {
    try {
      AC = new (window.AudioContext ??
        (window as unknown as { webkitAudioContext: typeof AudioContext })
          .webkitAudioContext)();
    } catch {
      AC = null;
    }
  }
  if (AC && AC.state === 'suspended') void AC.resume();
  return AC;
}
function beep(freq: number, dur: number, type: OscillatorType = 'sine', vol = 0.12): void {
  if (!soundOn) return;
  const ac = audio();
  if (!ac) return;
  const o = ac.createOscillator();
  const g = ac.createGain();
  o.type = type;
  o.frequency.value = freq;
  g.gain.setValueAtTime(vol, ac.currentTime);
  g.gain.exponentialRampToValueAtTime(0.001, ac.currentTime + dur);
  o.connect(g).connect(ac.destination);
  o.start();
  o.stop(ac.currentTime + dur);
}
function noiseBurst(dur: number, freq: number, vol = 0.25): void {
  if (!soundOn) return;
  const ac = audio();
  if (!ac) return;
  const len = Math.floor(ac.sampleRate * dur);
  const buf = ac.createBuffer(1, len, ac.sampleRate);
  const d = buf.getChannelData(0);
  for (let i = 0; i < len; i++) d[i] = (Math.random() * 2 - 1) * (1 - i / len);
  const src = ac.createBufferSource();
  src.buffer = buf;
  const f = ac.createBiquadFilter();
  f.type = 'lowpass';
  f.frequency.value = freq;
  const g = ac.createGain();
  g.gain.value = vol;
  src.connect(f).connect(g).connect(ac.destination);
  src.start();
}
function sfx(name: 'gun' | 'gunFar' | 'boom' | 'msl' | 'flare' | 'hit'): void {
  if (!soundOn) return;
  if (name === 'gun') noiseBurst(0.06, 2400, 0.1);
  else if (name === 'gunFar') noiseBurst(0.05, 1200, 0.04);
  else if (name === 'boom') noiseBurst(0.7, 400, 0.4);
  else if (name === 'msl') beep(520, 0.25, 'sawtooth', 0.08);
  else if (name === 'flare') noiseBurst(0.2, 3000, 0.08);
  else if (name === 'hit') beep(880, 0.08, 'square', 0.07);
}
function lockTone(on: boolean, rate = 0): void {
  if (!soundOn || !on) {
    if (lockOsc) {
      try {
        lockOsc.stop();
      } catch {
        /* already stopped */
      }
      lockOsc = null;
    }
    return;
  }
  const ac = audio();
  if (!ac) return;
  if (!lockOsc) {
    lockOsc = ac.createOscillator();
    lockGain = ac.createGain();
    lockOsc.type = 'sine';
    lockOsc.frequency.value = 880;
    lockGain.gain.value = 0.05;
    lockOsc.connect(lockGain).connect(ac.destination);
    lockOsc.start();
  }
  lockOsc.frequency.value = 660 + rate * 440;
}

// ---- debug capture (?debug=1, template ~442–470) ----
const DEBUG_ON = /[?&]debug(=|&|$)/.test(location.search);
const debugLog: string[] = [];
const modelStatus: Record<string, { state: string; tris?: number; error?: string }> = {
  fa18: { state: 'pending' },
  f35a: { state: 'pending' },
  f22: { state: 'pending' },
  f15e: { state: 'pending' },
  mig29: { state: 'pending' },
  su34: { state: 'pending' },
  ford: { state: 'pending' },
  heli: { state: 'pending' },
};
{
  const warn = console.warn.bind(console);
  const err = console.error.bind(console);
  const push = (kind: string, args: unknown[]): void => {
    const line =
      kind +
      ': ' +
      args
        .map((a) => {
          try {
            return a instanceof Error
              ? (a.name ? a.name + ': ' : '') + a.message
              : String(a);
          } catch {
            return '[?]';
          }
        })
        .join(' ');
    debugLog.push(line);
    if (debugLog.length > 30) debugLog.shift();
  };
  console.warn = (...a: unknown[]): void => {
    push('WARN', a);
    warn(...a);
  };
  console.error = (...a: unknown[]): void => {
    push('ERR', a);
    err(...a);
  };
}

// ---- renderer / scene (template ~499–512) ----
const canvas = $('game') as HTMLCanvasElement;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x87b5d9);
scene.fog = new THREE.Fog(0x9db8cc, 900, 11000);
const camera = new THREE.PerspectiveCamera(
  62,
  window.innerWidth / window.innerHeight,
  0.5,
  20000,
);
camera.position.set(0, 320, -420);
const sun = new THREE.DirectionalLight(0xfff2d8, 2.2);
scene.add(sun);
scene.add(sun.target);
// Hemisphere ground colour lightened (was 0x2a4a2a): the very dark
// ground bounce left downward-facing Lambert faces (floating-island
// undersides) reading as pure black at distance. 0x4a5a3a keeps the
// night look but lifts shadowed faces above black.
const hemi = new THREE.HemisphereLight(0xbfd9ff, 0x4a5a3a, 0.9);
scene.add(hemi);
const moon = new THREE.DirectionalLight(0x8fb0ff, 0);
scene.add(moon);
scene.add(moon.target);
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---- physics world ----
const rapierWorld = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
// Collision-event dispatch: aircraft / terrain / deck colliders report
// contacts into this queue; the loop drains it into notifyContact().
// NOTE: rapierWorld.step() is called in the animate loop (it was missing
// in the first draft — without it no Rapier body ever integrates).
const eventQueue = new RAPIER.EventQueue(true);
const colliderOwners = new Map<number, Aircraft | 'terrain' | 'deck'>();
function drainCollisionEvents(): void {
  eventQueue.drainCollisionEvents((h1, h2, started) => {
    if (!started) return;
    const a = colliderOwners.get(h1);
    const b = colliderOwners.get(h2);
    const ac = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
    if (!ac || !ac.alive) return;
    const other = ac === a ? b : a;
    ac.notifyContact(ac.vel.y, other === 'deck');
  });
}

// ---- floating origin (v2: shift everything past 2 km) ----
const floatingOrigin = new FloatingOrigin(2000);

// ---- world / carrier ----
const worldGen = new PandoraWorldGenerator(scene, rapierWorld, floatingOrigin);
await worldGen.build();
worldGen.registerWithFloatingOrigin(floatingOrigin);
{
  const th = worldGen.terrainColliderHandle;
  if (th !== null) colliderOwners.set(th, 'terrain');
  // Runway boxes are ground too — the gear rests on them, not on the
  // heightfield, so they must be recognised as terrain for contact.
  for (const h of worldGen.runwayColliderHandles) colliderOwners.set(h, 'terrain');
}
const runwayPlacer = new RunwayPlacer(scene);
{
  const placed = runwayPlacer.place(worldGen.runways);
  // Runway strips are world geometry — shift them with the world.
  for (const m of placed.meshes) floatingOrigin.registerObject(m);
}
runwayPlacer.setOriginShift(floatingOrigin.cumulativeShift);
// Helicarriers (issue 5 — count 0–3, default 1). Up to three SHIELD
// carriers are built once (each loads the GLB) and the chosen count
// is activated per round; the rest are deactivated (hidden, deck
// disabled). Each flies its own loop — a scaled hexagon, so the loops
// are concentric and never overlap — with its own start index, so the
// carriers are spread around their circuits instead of stacked.
const CARRIER_LOOP_SCALES = [1.0, 0.75, 1.25];
const CARRIER_START_INDEXES = [0, 2, 4];
const carriers: HelicarrierController[] = CARRIER_LOOP_SCALES.map(
  (scale, i) => {
    const loop = carrierLoop(scale);
    return new HelicarrierController(scene, rapierWorld, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      // Exact Rapier heightfield surface (issue 8) — NOT the analytic
      // groundHeight, which sits up to ~107 m below the real surface
      // in the ranges. The terrain-following look-ahead and the death
      // check both read this, so the carrier is judged against the
      // surface it would actually hit.
      heightAt: (x, z) => worldGen.heightfieldHeightAt(x, z),
      waypoints: loop.waypoints,
      startIndex: CARRIER_START_INDEXES[i]!,
      onPodDestroyed: (_index, worldPos) => {
        vfx.spawnExplosion(worldPos.clone(), 1);
        sfx('boom');
      },
      onCarrierImpact: (worldPos) => {
        // Carrier destruction impact (issue 9): a carrier-scale
        // explosion + debris scatter + a large scorch at the wreck.
        // Scaled to a 333 m ship — far bigger than an aircraft crash.
        vfx.spawnExplosion(worldPos.clone(), 8);
        sfx('boom');
        for (let i = 0; i < 6; i++) {
          const ang = Math.random() * Math.PI * 2;
          const sp = 10 + Math.random() * 20;
          vfx.spawnDebris(
            null,
            rapierWorld,
            worldPos.clone(),
            new THREE.Vector3(
              Math.cos(ang) * sp,
              5 + Math.random() * 15,
              Math.sin(ang) * sp,
            ),
          );
        }
        vfx.scorchDecal(worldPos.clone(), 30);
        hud.toast('CARRIER DOWN', 'A helicarrier is lost.');
      },
      onCarrierDown: () => {
        hud.toast('CARRIER DOWN', 'A helicarrier is lost.');
      },
    });
  },
);
/** The primary carrier (index 0) — the deck spawn, the AI's carrier
 *  reference and the minimap's first carrier. */
const carrier = carriers[0]!;
/** Active carrier count for this round (set in startRound) — the AI
 *  and other per-frame code read this instead of re-deriving it. */
let heliCountActive = 1;
/** The carrier whose deck is under world (x, z), if any (active and
 *  alive). The deck provider, weapon pod checks and VTOL credit all
 *  resolve through this, so several carriers work as one deck surface. */
function carrierAt(x: number, z: number): HelicarrierController | null {
  for (const c of carriers) {
    if (c.active && c.alive && c.deckHeightAt(x, z) > -Infinity) return c;
  }
  return null;
}
/** Flat pod-position list for the WeaponSystem, rebuilt on each call:
 *  every active carrier's pods, concatenated. `podOwner[i]` is the
 *  (carrier, podIndex) for positions[i], so onCarrierPodHit can route
 *  the damage to the right carrier. */
const podOwner: Array<{ carrier: HelicarrierController; pod: number }> = [];
function carrierPodPositionsAll(): THREE.Vector3[] {
  podOwner.length = 0;
  const out: THREE.Vector3[] = [];
  for (const c of carriers) {
    if (!c.active || !c.alive) continue;
    c.podObjects.forEach((p, i) => {
      out.push(p.getWorldPosition(new THREE.Vector3()));
      podOwner.push({ carrier: c, pod: i });
    });
  }
  return out;
}
// Deck surface for the aircraft controller (adapts the carriers' API
// to the DeckProvider interface: deckHeightAt / velocity / yawRate /
// local). Every method resolves to the carrier under the queried
// point, so a jet can land on, ride and take off from any of them.
const deckProvider: DeckProvider = {
  get alive() {
    return carriers.some((c) => c.active && c.alive);
  },
  deckHeightAt: (x, z) => {
    let best = -Infinity;
    for (const c of carriers) {
      if (!c.active || !c.alive) continue;
      best = Math.max(best, c.deckHeightAt(x, z));
    }
    return best;
  },
  velocity: () => carrier.vel.clone(),
  // Point velocity at (x, z): linear vel + yawRate x r (issue 3 — the
  // deck point under a jet 120 m aft of the centre moves sideways at
  // up to 14 m/s when the carrier turns; the linear velocity alone
  // leaves the jet behind and it falls off the deck).
  velocityAt: (x, z) => {
    const c = carrierAt(x, z) ?? carrier;
    const rx = x - c.pos.x;
    const rz = z - c.pos.z;
    const w = c.yawRate;
    return new THREE.Vector3(c.vel.x + w * rz, c.vel.y, c.vel.z - w * rx);
  },
  yawRate: () => carrier.yawRate,
  yawRateAt: (x, z) => (carrierAt(x, z) ?? carrier).yawRate,
  local: (x, z) => {
    const c = carrierAt(x, z);
    if (!c) return null;
    const v = c.localPosition(new THREE.Vector3(x, 0, z));
    return v ? { x: v.x, z: v.z } : null;
  },
};
/** Exact Rapier-heightfield surface height (triangle interpolation —
 *  see heightfieldHeightAt in PandoraTerrainWorker). The bilinear
 *  sampled height sits up to ~7.4 m below the collider surface
 *  at the steepest spots, and the analytic groundHeight up to ~107 m
 *  below it. Anything that must agree with where a jet physically
 *  rests — the controller's ground detection, crash VFX, weapon
 *  ground impacts, the camera clamp — uses THIS. (The v2 hang: a jet
 *  resting on the collider while the analytic height put it 90+ m in
 *  the air, so the controller never saw it land.) */
const heightfieldGroundHeight = (x: number, z: number): number =>
  worldGen.heightfieldHeightAt(x, z);

// ---- combat / VFX ----
let shakeT = 0;
const vfx = new CrashVFXManager(scene, {
  groundHeight: heightfieldGroundHeight,
  onShake: (amount) => {
    shakeT = Math.max(shakeT, amount);
  },
});
const weapons = new WeaponSystem(scene, rapierWorld, {
  sfx,
  lockTone,
  onExplosion: (pos, scale) => {
    vfx.spawnExplosion(pos, scale);
    sfx('boom');
  },
  onCarrierPodHit: (podIndex, damage) => {
    const owner = podOwner[podIndex];
    if (owner) owner.carrier.damagePod(owner.pod, damage);
  },
  carrierPodPositions: () => carrierPodPositionsAll(),
  groundHeight: heightfieldGroundHeight,
  deckHeight: (x, z) => {
    let best = -Infinity;
    for (const c of carriers) {
      if (!c.active || !c.alive) continue;
      best = Math.max(best, c.deckHeight(x, z));
    }
    return best;
  },
});

// ---- UI / input ----
const hud = new FlightHUD();
const pcInput = new PCFlightInput(canvas);
const tiltInput = new MobileFlightInput();
const menu = new SandboxMenu((next) => {
  void (async () => {
    const startBtn = $('startBtn') as HTMLButtonElement | null;
    const loadErr = $('loadErr');
    if (startBtn) {
      startBtn.textContent = 'Loading…';
      startBtn.disabled = true;
    }
    // Carrier build errors are the Ford's fault; startRound errors are
    // not. Split the two so a bad jet key (or any round-setup failure)
    // never gets reported as a carrier failure — and never leaves the
    // game in the dead state the F-35A/F-15E key mismatch caused (menu
    // hidden, started=true, no player, no way back except reload).
    try {
      // Build all three carriers (in parallel — each loads the SHIELD
      // GLB once); the chosen count is activated per round in
      // startRound, the rest are deactivated.
      await Promise.all(
        carriers.map(async (c) => {
          await c.build();
          c.registerWithFloatingOrigin(floatingOrigin);
          const dc = c.deckColliderRef;
          if (dc) colliderOwners.set(dc.handle, 'deck');
        }),
      );
      modelStatus['helicarrier'] = { state: 'ok' };
    } catch (e) {
      modelStatus['helicarrier'] = {
        state: 'FAILED',
        error: e instanceof Error ? e.message : String(e),
      };
      if (loadErr) loadErr.textContent = 'Carrier load error: ' + (e instanceof Error ? e.message : String(e));
      if (startBtn) {
        startBtn.textContent = 'Take Off 🛫';
        startBtn.disabled = false;
      }
      return;
    }
    try {
      await startRound(next);
    } catch (e) {
      // Recovery: back to the pre-flight menu with the error visible.
      started = false;
      paused = false;
      menu.show();
      if (loadErr) loadErr.textContent = 'Could not start round: ' + (e instanceof Error ? e.message : String(e));
      if (startBtn) {
        startBtn.textContent = 'Take Off 🛫';
        startBtn.disabled = false;
      }
    }
  })();
});

// ---- game state ----
let cfg: SandboxConfig = menu.read();
// Stick shaping from the (possibly stored) setup, so the keyboard
// answers the way the menu says before the first round starts.
pcInput.setStickShaping({
  invertPitch: cfg.invertPitch,
  invertRoll: cfg.invertRoll,
  pitchSens: cfg.pitchSens,
  rollSens: cfg.rollSens,
});
let player: Aircraft | null = null;
let playerKey = '';
const enemies: Aircraft[] = [];
const traffic: Aircraft[] = [];
const aiControllers = new Map<Aircraft, AircraftAIController>();
let camMode = 0; // 0 chase, 1 cockpit, 2 free-orbit
let camDist = 30;
let camOrbit = { yaw: 0, pitch: 0.32 };
let paused = false;
let started = false;
let elapsed = 0;
let score = 0;
let kills = 0;
let traps = 0;
let vtolLandings = 0;
let respawnTimer: ReturnType<typeof setTimeout> | null = null;
/** Spectate state (damage-model §5): true while the player's wreck is
 *  falling (player.dying) and for 2 s after impact, until respawn.
 *  The camera follows the wreck; R (or the Respawn button) skips to
 *  an immediate respawn. */
let spectating = false;
let prevPlayerDying = false;
/** Damage-visual timer: spawn a smoke/fire plume every 0.5 s per
 *  damaged aircraft (the plume lives 6 s, so this keeps a continuous
 *  trail without flooding the VFX pool). */
let smokeT = 0;
let failAt = 0;
let failDone = false;
let failScenario: string = 'none';
const windVec = new THREE.Vector3(3, 0, 0);
let nightF = 0;
let todH = 10;
let fpsEMA = 60;
let lastDt = 0;
const prevAlive = new Map<Aircraft, boolean>();
let prevTrapped = false;

// Touch stick (template ~1718–1730): bottom-left joy/joyKnob.
const joy = { x: 0, y: 0, active: false, id: -1 };
{
  const joyEl = $('joy');
  const knob = $('joyKnob');
  const joySet = (cx: number, cy: number): void => {
    if (!joyEl) return;
    const r = joyEl.getBoundingClientRect();
    let dx = (cx - (r.left + r.width / 2)) / (r.width / 2);
    let dy = (cy - (r.top + r.height / 2)) / (r.height / 2);
    const len = Math.hypot(dx, dy);
    if (len > 1) {
      dx /= len;
      dy /= len;
    }
    joy.x = dx;
    joy.y = dy;
    if (knob) knob.style.transform = `translate(${dx * 44}px, ${dy * 44}px)`;
  };
  joyEl?.addEventListener('pointerdown', (e) => {
    audio();
    joy.active = true;
    joy.id = e.pointerId;
    (joyEl as HTMLElement).setPointerCapture(e.pointerId);
    joySet(e.clientX, e.clientY);
  });
  joyEl?.addEventListener('pointermove', (e) => {
    if (joy.active && e.pointerId === joy.id) joySet(e.clientX, e.clientY);
  });
  const joyEnd = (e: PointerEvent): void => {
    if (e.pointerId === joy.id) {
      joy.active = false;
      joy.x = 0;
      joy.y = 0;
      if (knob) knob.style.transform = '';
    }
  };
  joyEl?.addEventListener('pointerup', joyEnd);
  joyEl?.addEventListener('pointercancel', joyEnd);
}

// Free-orbit drag + pinch zoom on the canvas (template ~1855–1885).
let orbitDrag: { x: number; y: number; id: number } | null = null;
let pinchD = 0;
canvas.addEventListener('pointerdown', (e) => {
  audio();
  if (camMode === 2) orbitDrag = { x: e.clientX, y: e.clientY, id: e.pointerId };
});
canvas.addEventListener('pointermove', (e) => {
  if (orbitDrag && e.pointerId === orbitDrag.id && camMode === 2) {
    camOrbit.yaw -= (e.clientX - orbitDrag.x) * 0.005;
    camOrbit.pitch = clamp(
      camOrbit.pitch + (e.clientY - orbitDrag.y) * 0.004,
      -0.1,
      1.2,
    );
    orbitDrag.x = e.clientX;
    orbitDrag.y = e.clientY;
  }
});
canvas.addEventListener('pointerup', (e) => {
  if (orbitDrag && e.pointerId === orbitDrag.id) orbitDrag = null;
});
canvas.addEventListener(
  'wheel',
  (e) => {
    camDist = clamp(camDist + e.deltaY * 0.03, 14, 120);
  },
  { passive: true },
);
canvas.addEventListener(
  'touchmove',
  (e) => {
    if (e.touches.length === 2) {
      const d = Math.hypot(
        e.touches[0]!.clientX - e.touches[1]!.clientX,
        e.touches[0]!.clientY - e.touches[1]!.clientY,
      );
      if (pinchD) camDist = clamp(camDist + (pinchD - d) * 0.08, 14, 120);
      pinchD = d;
    }
  },
  { passive: true },
);
canvas.addEventListener('touchend', () => {
  pinchD = 0;
});

// ---- control panel wiring (template ids, ~1732–1749) ----
function bindSlider(
  id: string,
  valId: string,
  fmt: (v: number) => string,
  apply: (v: number) => void,
): void {
  const input = $(id) as HTMLInputElement | null;
  input?.addEventListener('input', () => {
    const v = Number(input.value);
    apply(v);
    const val = $(valId);
    if (val) val.textContent = fmt(v);
  });
}
bindSlider('thr', 'thrVal', (v) => v + '%', (v) => {
  if (player) {
    player.thrTarget = v / 100;
    pcInput.setThrottle(v / 100);
  }
});
bindSlider('flapSl', 'flapVal', (v) => v + '°', (v) => {
  if (player) player.flaps = v;
});
bindSlider('brkSl', 'brkVal', (v) => v + '%', (v) => {
  if (player) player.airbrake = v / 100;
});
bindSlider('vecSl', 'vecVal', (v) => v + '°', (v) => {
  if (player) player.vector = v;
  vtolTarget = null; // manual slider overrides a toggle slew
});

/** Set the flaps / airbrake panel sliders from code (keyboard [ ] / X)
 *  so the touch UI and the keyboard agree. */
function syncFlapSlider(v: number): void {
  const input = $('flapSl') as HTMLInputElement | null;
  if (input) input.value = String(v);
  const val = $('flapVal');
  if (val) val.textContent = v + '°';
}
function syncBrkSlider(v01: number): void {
  const input = $('brkSl') as HTMLInputElement | null;
  if (input) input.value = String(Math.round(v01 * 100));
  const val = $('brkVal');
  if (val) val.textContent = Math.round(v01 * 100) + '%';
}
/** Set the thrust-vector slider from code (VTOL toggle) so the touch
 *  UI and the keyboard agree. */
function syncVecSlider(v: number): void {
  const input = $('vecSl') as HTMLInputElement | null;
  if (input) input.value = String(v);
  const val = $('vecVal');
  if (val) val.textContent = v + '°';
}

/** VTOL toggle (Issue 2): F-35A lift-fan hover on/off. Engaging slews
 *  the thrust vector to 90 deg (full hover — the lift fan + FCS hold
 *  attitude, ~94% throttle holds altitude at combat mass); leaving
 *  slews it back to 0 deg (forward flight). The slew runs at 36 deg/s
 *  (~2.5 s end to end, like a real nozzle rotation) so a transition
 *  from a dead hover does not snap the lift away and drop the jet —
 *  the wing takes over progressively as the vector comes down. The
 *  slider still sets any angle directly (and cancels a slew in
 *  progress). No-op on non-VTOL jets. */
let vtolTarget: number | null = null;
function toggleVtol(): void {
  if (!player || !player.spec.vtol) return;
  const toHover = player.vector < 70;
  vtolTarget = toHover ? 90 : 0;
  $('vtolBtn')?.classList.toggle('on', toHover);
  hud.toast(
    toHover ? 'VTOL HOVER' : 'FORWARD FLIGHT',
    toHover
      ? 'Lift fan engaging — ~94% throttle holds altitude. V again to transition.'
      : 'Nozzle rotating aft — full throttle, accelerate past 120 kt to fly on the wing.',
  );
}

let firing = false;
let touchBrake = false;
function toggleBtn(id: string, fn: () => void): void {
  $(id)?.addEventListener('click', fn);
}
toggleBtn('gearBtn', () => {
  if (!player) return;
  if (player.failGearJam) {
    hud.toast('GEAR JAMMED', 'The gear will not move.');
    return;
  }
  player.gear = !player.gear;
  $('gearBtn')?.classList.toggle('on', player.gear);
});
toggleBtn('hookBtn', () => {
  if (!player) return;
  player.hook = !player.hook;
  $('hookBtn')?.classList.toggle('on', player.hook);
});
toggleBtn('vtolBtn', () => toggleVtol());
toggleBtn('flareBtn', () => {
  if (player) weapons.fireFlares(player);
});
toggleBtn('mslBtn', () => {
  if (player && player.lockT > 1.2 && player.lockTarget) {
    weapons.launchMissile(player, player.lockTarget);
  }
});
const fireBtn = $('fireBtn');
fireBtn?.addEventListener('pointerdown', (e) => {
  firing = true;
  e.preventDefault();
});
window.addEventListener('pointerup', () => {
  firing = false;
});
const brakeBtn = $('brakeBtn');
// Pass 5 (item 5): the Brake button was momentary (pointerdown sets
// it, a global pointerup clears it), so on a phone the jet could only
// be held while a finger stayed on the button — and the play-test
// found it "does not hold it (momentary, not a latch)". It is now a
// LATCH like gear/hook: tap to engage (stays on, button lit), tap
// again to release. The keyboard brake (KeyB, held) is unchanged.
brakeBtn?.addEventListener('click', () => {
  touchBrake = !touchBrake;
  brakeBtn.classList.toggle('on', touchBrake);
});
window.addEventListener('pointerup', () => {
  firing = false;
  // Pass 5 (item 5): do NOT clear touchBrake here — the Brake button
  // is a latch now (click toggles it); a global pointerup would
  // release it the moment the finger lifts, which is exactly the
  // momentary behaviour the play-test reported.
});
toggleBtn('bombBtn', () => {
  if (!player) return;
  if (!player.spec.bombs) {
    hud.toast('NO BOMBS', player.spec.name + ' carries gun + missiles only in v2.');
    return;
  }
  if (player.bombs <= 0) {
    hud.toast('BOMBS EMPTY', 'Out of bombs — restart to re-arm.');
    return;
  }
  weapons.dropBomb(player);
});
toggleBtn('tiltBtn', () => {
  void tiltInput
    .toggle()
    .then((on) => {
      $('tiltBtn')?.classList.toggle('on', on);
      hud.toast(
        on ? 'TILT ON' : 'TILT OFF',
        on ? 'Tilt to steer — hold level, then tap again to re-centre.' : '',
      );
    })
    .catch(() => {
      hud.toast('TILT DENIED', 'Motion permission was not granted.');
    });
});

// ---- session / score / respawn ----
/** Build an aircraft wired into the world: deck provider, terrain height,
 *  floating-origin body registration and the contact registry. */
function makeAircraft(key: string, isPlayer: boolean): Aircraft {
  const spec = SPECS[key];
  if (!spec) throw new Error('unknown aircraft key: ' + key);
  const ac = new Aircraft({
    scene,
    world: rapierWorld,
    spec,
    isPlayer,
    modelKey: spec.modelKey,
    deck: deckProvider,
    events: {
      onDetachSegment: (_a, segment, pos, vel) => {
        if (segment) vfx.spawnDebris(segment, rapierWorld, pos, vel);
      },
      // AI flares did their bookkeeping in Aircraft.fireFlares(); this
      // hook only draws the sprites (never call WeaponSystem.fireFlares
      // here — it would decrement the count a second time).
      onFlares: (a) => weapons.showFlares(a),
    },
  });
  // The controller's ground logic (ground detection, respawn elevation
  // clamp) must read the EXACT heightfield surface, not the analytic
  // heightAt() — the analytic can sit 90+ m below the collider on
  // steep slopes, and a jet resting there reads as airborne forever
  // (the v2 respawn hang: grounded=false, G 0.2, no crash, no
  // respawn). See heightfieldGroundHeight above.
  ac.terrainHeightAt = heightfieldGroundHeight;
  floatingOrigin.registerBody(ac.body);
  for (const h of ac.colliderHandles()) colliderOwners.set(h, ac);
  return ac;
}

function addScore(n: number, kind: 'kill' | 'trap' | 'vtol'): void {
  score += n;
  if (kind === 'kill') kills++;
  if (kind === 'trap') traps++;
  if (kind === 'vtol') vtolLandings++;
}

/** Show/hide the spectate banner ("SHOT DOWN — watching the wreck ·
 *  R to skip"). The banner is a DOM element (#spectate) so it stays
 *  up for the whole fall, unlike a 3.2 s toast. */
function setSpectateBanner(show: boolean, reason?: string, killer?: string): void {
  const el = document.getElementById('spectate');
  const tx = document.getElementById('spectateText');
  if (!el) return;
  if (show) {
    // Pass 5 (item 3): the banner hardcoded "SHOT DOWN" even for a
    // terrain/hard-landing crash. A shoot-down (dyingReason, killer)
    // still reads SHOT DOWN; an instant crash reads WENT DOWN with
    // its reason (terrain / hard landing), so the player knows what
    // happened while the camera holds on the wreck.
    const isShotDown =
      killer !== undefined ||
      (reason !== undefined &&
        (reason === 'shot down' || reason === 'fuel explosion'));
    const label = isShotDown
      ? `SHOT DOWN${killer ? ` — shot down by ${killer}` : ''}`
      : `WENT DOWN${reason ? ` — ${reason}` : ''}`;
    if (tx) tx.textContent = `${label} · watching the wreck`;
    if (reason) el.title = reason;
    el.classList.add('show');
  } else {
    el.classList.remove('show');
  }
}

/** Skip the spectate (R key or the Respawn button): show the respawn
 *  picker (issue 4) instead of respawning immediately. If the jet is
 *  still dying (falling), the picker is shown anyway — picking a
 *  target respawns at once (respawnPlayer allows a dying jet). If the
 *  jet is already dead, the picker is the respawn step. */
function skipSpectate(): void {
  if (!spectating && !respawnTimer) return;
  if (respawnTimer) {
    clearTimeout(respawnTimer);
    respawnTimer = null;
  }
  spectating = false;
  setSpectateBanner(false);
  showRespawnPicker(false);
}

/** Respawn target (issue 4): a runway by name, or the helicarrier
 *  deck. The last choice is the highlighted default in the picker. */
type RespawnTargetId = 'alpha' | 'bravo' | 'charlie' | 'carrier';
const RESPAWN_CHOICE_KEY = 'pandora-planes-last-respawn';
let lastRespawnChoice: RespawnTargetId = 'alpha';
try {
  const saved = localStorage.getItem(RESPAWN_CHOICE_KEY);
  if (saved === 'alpha' || saved === 'bravo' || saved === 'charlie' || saved === 'carrier') {
    lastRespawnChoice = saved;
  }
} catch {
  // localStorage unavailable — keep the default.
}
let pickerOpen = false;
/** Picker opened while the jet is alive (the stuck case) — Cancel is
 *  offered and no respawn is forced. */
let pickerForStuck = false;
/** Stuck detector (issue 4 / live finding 2): the jet survived onto
 *  a steep slope at a steep tilt, beached — alive, ~0 kt, not
 *  grounded and not on a deck, so no control authority and no
 *  respawn timer ever fires. While this is true, R opens the picker
 *  instead of resetting the camera. */
let stuckHintActive = false;
let stuckTime = 0;
let stuckToastShown = false;

/** Show the respawn picker: Alpha / Bravo / Charlie / Helicarrier
 *  deck, with the last choice highlighted. The carrier deck is hidden
 *  when there are 0 carriers (issue 5). `forStuck` is the alive case
 *  (Cancel offered); otherwise the picker is the only way back. */
function showRespawnPicker(forStuck: boolean): void {
  if (!player) return;
  pickerForStuck = forStuck;
  const el = $('respawnPicker');
  if (!el) return;
  const carrierBtn = el.querySelector(
    'button[data-target="carrier"]',
  ) as HTMLButtonElement | null;
  if (carrierBtn) {
    carrierBtn.style.display = heliCountActive === 0 ? 'none' : '';
  }
  // The highlighted default is the last choice — unless it is the
  // carrier and there is no carrier, in which case fall back to Alpha.
  let def = lastRespawnChoice;
  if (def === 'carrier' && heliCountActive === 0) def = 'alpha';
  el.querySelectorAll('button[data-target]').forEach((b) => {
    b.classList.toggle('on', (b as HTMLElement).dataset['target'] === def);
  });
  const cancel = $('pickerCancel');
  if (cancel) cancel.style.display = forStuck ? '' : 'none';
  el.classList.add('show');
  pickerOpen = true;
}

function hideRespawnPicker(): void {
  const el = $('respawnPicker');
  if (el) el.classList.remove('show');
  pickerOpen = false;
  pickerForStuck = false;
}

/** The picker's currently highlighted choice (the 'on' button) — the
 *  one R picks. Falls back to the last choice if the DOM is missing. */
function highlightedRespawnChoice(): RespawnTargetId {
  const el = $('respawnPicker');
  const on = el?.querySelector('button[data-target].on') as HTMLElement | null;
  const t = on?.dataset['target'];
  if (t === 'alpha' || t === 'bravo' || t === 'charlie' || t === 'carrier') {
    return t;
  }
  return lastRespawnChoice;
}

/** Pick a respawn target from the picker: remember it (it becomes the
 *  highlighted default next time) and respawn there. */
function pickRespawn(target: RespawnTargetId): void {
  lastRespawnChoice = target;
  try {
    localStorage.setItem(RESPAWN_CHOICE_KEY, target);
  } catch {
    // localStorage unavailable — the in-memory choice still applies.
  }
  // A pick from the stuck picker (jet alive, beached) must force the
  // respawn: respawnPlayer's guard would otherwise return early
  // because the jet is alive and not dying.
  const force = pickerForStuck;
  hideRespawnPicker();
  respawnPlayer(target, force);
}

/** Clear every transient warning/banner (restart / respawn) — the
 *  HUD's #warn / ROTATE cue / toast plus the spectate banner. The
 *  v2 play-test saw STALL, "YOU WENT DOWN" and ROTATE survive
 *  restarts and respawns; the HUD clears them here, and the solver
 *  reset on respawn clears the underlying stall state. */
function clearTransientUI(): void {
  hud.clearTransient();
  setSpectateBanner(false);
  hideRespawnPicker();
}

function respawnPlayer(target?: RespawnTargetId, force = false): void {
  if (!player) return;
  // v1.1: Restart inside the 3 s window already revived the jet — but
  // a DYING player (alive, falling) must still respawn on skip (R /
  // Respawn button), so only return early when alive AND not dying.
  // `force` is the stuck case (issue 4): the picker was opened while
  // the jet is alive and beached, and the pick must respawn it.
  if (player.alive && !player.dying && !force) return;
  respawnTimer = null;
  spectating = false;
  prevPlayerDying = false;
  hideRespawnPicker();
  // Clear the burn plume attached to the jet (live finding 3): a
  // dying jet's fire plume is parented to its group, and the group
  // is reused on respawn — without this the respawned jet would
  // carry the previous life's fire.
  vfx.clearAttached(player.group);
  const choice: RespawnTargetId = target ?? lastRespawnChoice;
  const shift = floatingOrigin.cumulativeShift;
  if (choice === 'carrier' && heliCountActive > 0) {
    // Deck respawn (issue 4): the primary carrier's CURRENT position
    // and yaw, 120 m aft of the deck centre, with the deck point
    // velocity (issue 3 — the linear velocity alone leaves the jet
    // behind when the carrier turns). A carrier choice with 0
    // carriers falls through to the Alpha runway below.
    const c = carrier;
    const fwd = new THREE.Vector3(Math.sin(c.yaw), 0, Math.cos(c.yaw));
    const sx = c.pos.x - fwd.x * 120;
    const sz = c.pos.z - fwd.z * 120;
    player.resetAt(sx, c.deckTopY + 2.2, sz, (c.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    player.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    player.vel.copy(dv);
  } else {
    // Runway respawn: the chosen runway (Alpha / Bravo / Charlie),
    // defaulting to Alpha. The runway table is in ORIGINAL-frame
    // coords — but resetAt() takes SCENE coords. After any
    // floating-origin shift (the player flying > 2,000 m from the
    // origin), passing the original-frame coords straight in
    // respawned the jet up to 2+ km from the actual runway — in
    // mid-air or inside a mountain — where it fell, crashed, and the
    // 3 s timer fired again: the death loop behind the v2 respawn
    // hang. Convert back to scene coords here.
    const idx = choice === 'bravo' ? 1 : choice === 'charlie' ? 2 : 0;
    const r = RUNWAY_TABLE[idx]!;
    // Pass 5 (item 1): spawn the gear feet ON the runway box top
    // (elev + 0.05), not 0.25 m inside it. resetAt(y) puts the centre
    // at y and the gear feet at y - 2.2, so y = elev + 2.25 lands the
    // feet exactly on the box top — the deck spawn's convention
    // (deckTopY + 2.2 = feet at the deck). The old elev + 2 buried the
    // gear 0.25 m into the box; Rapier has to push it back out every
    // substep, and on a slow phone the contact loses to the
    // controller's per-frame gravity increment (-88 fpm sink, crash
    // in 10-27 s live on Alpha/Bravo). Charlie's spawn worked because
    // its plateau/terrain let the contact win; the box top is the
    // same 0.05 m above the heightfield on all three.
    player.resetAt(r.x - shift.x, r.elev + 2.25, r.z - shift.z, r.hdg);
  }
  player.thrTarget = 0;
  pcInput.setThrottle(0);
  hud.syncThrottle(0);
  camMode = 0;
  pcInput.setCameraMode(0);
  clearTransientUI();
}

function clearAircraftList(list: Aircraft[]): void {
  for (const a of list) {
    const ai = aiControllers.get(a);
    if (ai) floatingOrigin.unregisterVector(ai.navTarget);
    floatingOrigin.unregisterBody(a.body);
    for (const h of a.colliderHandles()) colliderOwners.delete(h);
    a.dispose();
    aiControllers.delete(a);
    prevAlive.delete(a);
  }
  list.length = 0;
}

function weatherLabel(c: SandboxConfig): string {
  const w = c.weather;
  if (w.rain && w.turbulence > 2) return 'storm';
  if (w.rain) return 'rain';
  if (w.clouds) return 'cloud';
  return 'clear';
}

async function startRound(next: SandboxConfig): Promise<void> {
  // Validate the aircraft key BEFORE hiding the menu or touching state:
  // an unknown key used to throw after menu.hide() + started=true, which
  // left the game with no player and no way back (the F-35A/F-15E freeze).
  if (!SPECS[next.aircraftKey]) {
    hud.toast(
      'UNKNOWN JET',
      'No aircraft "' + next.aircraftKey + '" — pick another jet.',
    );
    menu.show();
    return;
  }
  cfg = next;
  menu.hide();
  $('pauseScreen')?.classList.add('hidden');
  started = true;
  paused = false;
  score = 0;
  kills = 0;
  traps = 0;
  vtolLandings = 0;
  elapsed = 0;
  failDone = false;
  // Failure timing: 'none' never fires. A named scenario fires at 3 s
  // when failWhen is 'start', or at a random 30–90 s when failWhen is
  // 'random'. 'random' as the scenario picks a random scenario; the
  // timing still comes from failWhen.
  if (cfg.failure === 'none') {
    failScenario = 'none';
    failAt = Infinity;
  } else {
    failScenario =
      cfg.failure === 'random'
        ? FAIL_CHOICES[Math.floor(Math.random() * FAIL_CHOICES.length)]!
        : cfg.failure;
    failAt = cfg.failWhen === 'random' ? 30 + Math.random() * 60 : 3;
  }
  // Stick shaping (inversion + sensitivity) from the setup menu.
  pcInput.setStickShaping({
    invertPitch: cfg.invertPitch,
    invertRoll: cfg.invertRoll,
    pitchSens: cfg.pitchSens,
    rollSens: cfg.rollSens,
  });
  if (respawnTimer) {
    clearTimeout(respawnTimer);
    respawnTimer = null;
  } // v1.1: no stale respawn mid-round
  // No stale warnings from the previous round (the v2 play-test saw
  // STALL survive three restarts — the solver reset on respawn clears
  // the state, this clears the banners).
  clearTransientUI();
  // Pass 5 (item 6): the spectate banner ("WENT DOWN / SHOT DOWN —
  // watching the wreck") was NOT cleared by clearTransientUI, so it
  // survived a full round restart (Apply & Restart) and showed at
  // the start of a fresh round. A new round is never spectating.
  spectating = false;
  setSpectateBanner(false);
  clearAircraftList(enemies);
  clearAircraftList(traffic);
  worldGen.resetOrigin();

  // Weather: fog preset by label; wind dir = direction wind blows toward.
  const wxFog: Record<string, [number, number]> = {
    clear: [900, 11000],
    cloud: [700, 7200],
    storm: [400, 3800],
    rain: [500, 5200],
  };
  const fog = wxFog[weatherLabel(cfg)] ?? wxFog['clear']!;
  windVec.set(
    Math.sin(cfg.weather.windDir * D2R) * cfg.weather.windSpeed,
    0,
    Math.cos(cfg.weather.windDir * D2R) * cfg.weather.windSpeed,
  );
  if (scene.fog instanceof THREE.Fog) {
    scene.fog.near = fog[0];
    scene.fog.far = fog[1];
  }
  todH = cfg.tod;

  // Player (recreate when the jet changes — the template's Object.assign
  // hack cannot work in TS, so the binding is replaced instead).
  if (!player || playerKey !== cfg.aircraftKey) {
    if (player) {
      floatingOrigin.unregisterBody(player.body);
      for (const h of player.colliderHandles()) colliderOwners.delete(h);
      player.dispose();
    }
    player = makeAircraft(cfg.aircraftKey, true);
    playerKey = cfg.aircraftKey;
    prevAlive.set(player, true);
  }
  hud.syncThrottle(0);
  const jetName = player.spec.name;
  $('hudJet') && (($('hudJet') as HTMLElement).textContent = jetName);
  $('vecRow')?.classList.toggle('hidden', !player.spec.vtol);
  $('vecPill')?.classList.toggle('hidden', !player.spec.vtol);
  $('vtolBtn')?.classList.toggle('hidden', !player.spec.vtol);
  $('vtolBtn')?.classList.toggle('on', player.spec.vtol && player.vector >= 70);
  const hookBtn = $('hookBtn') as HTMLElement | null;
  if (hookBtn) hookBtn.style.display = player.spec.carrier ? '' : 'none';
  try {
    await player.attachModel();
    modelStatus[cfg.aircraftKey] = { state: 'ok' };
  } catch (e) {
    modelStatus[cfg.aircraftKey] = {
      state: 'FAILED',
      error: e instanceof Error ? e.message : String(e),
    };
    console.warn(cfg.aircraftKey + ' model failed', e);
  }
  tiltInput.setAircraftKind(
    player.spec.vtol ? 'vtol' : 'fixed-wing',
  );

  // Carriers — activate the chosen count (issue 5), each on its own
  // loop (a scaled hexagon + its own start index). The primary is dead
  // ahead of Alpha (900 m east on the takeoff heading), so it is in
  // the portrait-phone view at round start. Reset BEFORE the player
  // spawns: a deck spawn needs the carrier's current position /
  // heading / cruise velocity. Carriers beyond the count are
  // deactivated (hidden, deck disabled).
  const heliCount = clamp(Math.round(cfg.helicarriers ?? 1), 0, 3);
  heliCountActive = heliCount;
  carriers.forEach((c, i) => {
    if (i < heliCount) {
      const loop = carrierLoop(CARRIER_LOOP_SCALES[i]!);
      const wp0 = loop.waypoints[CARRIER_START_INDEXES[i]!]!;
      c.reset(
        loop.startPos.clone(),
        Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
      );
    } else {
      c.deactivate();
    }
  });

  // Player spawn: Alpha runway, or the helicarrier deck. A deck spawn
  // with 0 carriers falls back to Alpha (the menu hides the option,
  // but a stale saved config could still ask for it).
  const r0: RunwayDef = worldGen.runways[0]!;
  if (cfg.spawn === 'carrier' && heliCount > 0) {
    // Deck spawn: 120 m aft of the deck centre (the full deck ahead for
    // the roll), facing the bow at the carrier's heading, inheriting the
    // carrier's cruise velocity so the jet rides the deck instead of
    // sliding off the stern (the deck moves at 40 m/s from the first
    // update; a zero-velocity spawn would be left behind). Works for all
    // four jets — the F-35A's VTOL home is the deck. The arresting-wire
    // logic applies on landing exactly as for a carrier trap.
    const fwd = new THREE.Vector3(
      Math.sin(carrier.yaw),
      0,
      Math.cos(carrier.yaw),
    );
    const sx = carrier.pos.x - fwd.x * 120;
    const sz = carrier.pos.z - fwd.z * 120;
    player.resetAt(sx, carrier.deckTopY + 2.2, sz, carrier.yaw * R2D);
    // Inherit the DECK-POINT velocity at the spawn point (issue 3):
    // the linear cruise velocity plus the yaw-rotation term. At round
    // start the carrier is heading straight for waypoint 0 so yawRate
    // is ~0 and this equals the cruise velocity — but after a respawn
    // mid-loop the carrier may be turning, and the point 120 m aft of
    // the centre moves sideways at up to 14 m/s; a linear-only spawn
    // velocity leaves the jet behind and it falls off the deck.
    const dv = deckProvider.velocityAt(sx, sz);
    player.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    player.vel.copy(dv);
  } else {
    // Runway spawn — gear feet on the box top (elev + 2.25, see
    // respawnPlayer above for why not elev + 2).
    player.resetAt(r0.x, r0.elev + 2.25, r0.z, r0.hdg);
  }
  player.gear = true;
  $('gearBtn')?.classList.add('on');

  // Enemies: mix of MiG-29 and Su-34.
  for (let i = 0; i < cfg.enemies; i++) {
    const key = i % 2 === 0 ? 'mig29' : 'su34';
    const e = makeAircraft(key, false);
    try {
      await e.attachModel();
      modelStatus[key] = { state: 'ok' };
    } catch (err) {
      modelStatus[key] = {
        state: 'FAILED',
        error: err instanceof Error ? err.message : String(err),
      };
      console.warn(key + ' model failed', err);
    }
    let ex: number;
    let ey: number;
    let ez: number;
    if (i < 2) {
      // v1.1: first two enemies start in the player's forward view,
      // 450–600 m out and 120–180 m up. Forward is the player's own
      // heading, so this holds for a deck spawn too.
      const fwd = player.fwd();
      const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
      const along = 450 + i * 150;
      const side = i === 0 ? 120 : -150;
      const up = 120 + i * 60;
      ex = player.pos.x + fwd.x * along + rgt.x * side;
      ey = player.pos.y + up;
      ez = player.pos.z + fwd.z * along + rgt.z * side;
    } else {
      const a = Math.random() * Math.PI * 2;
      const d = cfg.enemySpawnDistance * (0.8 + Math.random() * 0.5);
      ex = player.pos.x + Math.cos(a) * d;
      ey = 650 + Math.random() * 350;
      ez = player.pos.z + Math.sin(a) * d;
    }
    e.resetAt(ex, ey, ez, Math.random() * 360);
    // Air spawn: the jet is flying, not hovering at 0 kt (setAirspeed —
    // below stall speed the wing cannot hold 1 g and no steering law
    // can turn it).
    e.setAirspeed(150);
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    const ai = new AircraftAIController(e, cfg.enemySkill, 'enemy', {
      // Exact heightfield surface (issue 6): the analytic groundHeight
      // sits up to ~107 m below the real surface, so the AI's terrain
      // avoidance fired ~107 m late and it flew into hills the analytic
      // model underestimates — the same class of bug that killed the
      // carrier in v3. The player controller, VFX and weapons already
      // use heightfieldGroundHeight; the AI now does too.
      groundHeight: heightfieldGroundHeight,
      originShift: floatingOrigin.cumulativeShift,
    });
    floatingOrigin.registerVector(ai.navTarget);
    aiControllers.set(e, ai);
    prevAlive.set(e, true);
    enemies.push(e);
  }

  // Passive traffic hops runways/carrier (G5).
  if (cfg.traffic) {
    for (let i = 0; i < cfg.trafficCount; i++) {
      const key = i % 3 === 2 ? 'heli' : i % 2 === 0 ? 'fa18' : 'mig29';
      const t = makeAircraft(key, false);
      try {
        await t.attachModel();
        modelStatus[key] = { state: 'ok' };
      } catch (err) {
        modelStatus[key] = {
          state: 'FAILED',
          error: err instanceof Error ? err.message : String(err),
        };
        console.warn(key + ' model failed', err);
      }
      const r = worldGen.runways[i % worldGen.runways.length]!;
      if (key === 'heli') {
        t.resetAt(r.x, r.elev + 150, r.z, r.hdg);
        t.setAirspeed(60);
        t.thrTarget = 0.55;
      } else {
        t.resetAt(r.x, r.elev + 220, r.z, r.hdg);
        t.setAirspeed(130);
        t.thrTarget = 0.7;
      }
      t.gear = false;
      t.grounded = false;
      const ai = new AircraftAIController(t, 'veteran', 'traffic', {
        // Exact heightfield surface (issue 6) — see the enemy AI above.
        groundHeight: heightfieldGroundHeight,
        originShift: floatingOrigin.cumulativeShift,
      });
      floatingOrigin.registerVector(ai.navTarget);
      ai.pickNav();
      aiControllers.set(t, ai);
      prevAlive.set(t, true);
      traffic.push(t);
    }
  }

  hud.toast(
    'TAKE OFF',
    jetName +
      ' · ' +
      (cfg.enemies === 0 ? 'no enemies' : cfg.enemies + ' enemies') +
      ' · ' +
      (cfg.spawn === 'carrier' ? 'deck spawn' : 'Alpha') +
      ' · ' +
      weatherLabel(cfg) +
      ' weather',
  );
}

function restartRound(): void {
  void startRound(menu.read()).catch((e) => {
    // Same recovery as the menu path: never leave a dead screen.
    started = false;
    paused = false;
    menu.show();
    const loadErr = $('loadErr');
    if (loadErr) {
      loadErr.textContent =
        'Could not restart round: ' + (e instanceof Error ? e.message : String(e));
    }
  });
}

function togglePause(): void {
  if (!started) return;
  paused = !paused;
  $('pauseScreen')?.classList.toggle('hidden', !paused);
  if (paused && player) {
    const sub = $('pauseSub');
    if (sub) {
      sub.textContent =
        'Score ' + score + ' · ' + kills + ' kills · ' + traps + ' traps · ' +
        fmtTime(elapsed) + ' survived';
    }
    const stats = $('pauseStats');
    if (stats) {
      stats.textContent =
        player.spec.name + ' · ' + cfg.enemies + ' enemies (' + cfg.enemySkill +
        ') · ' + weatherLabel(cfg);
    }
  }
}

function fmtTime(s: number): string {
  const m = Math.floor(s / 60);
  const ss = Math.floor(s % 60);
  return String(m).padStart(2, '0') + ':' + String(ss).padStart(2, '0');
}

function cycleCam(): void {
  camMode = (camMode + 1) % 3;
  pcInput.setCameraMode(camMode);
  hud.cameraToast(camMode);
}

$('pauseBtn')?.addEventListener('click', togglePause);
$('respawnBtn')?.addEventListener('click', skipSpectate);
// Respawn picker buttons (issue 4): each picks its target at once.
// The carrier button is hidden when there are 0 carriers (the
// picker hides it in showRespawnPicker too).
document
  .querySelectorAll('#respawnPicker button[data-target]')
  .forEach((b) => {
    b.addEventListener('click', () => {
      const t = (b as HTMLElement).dataset['target'];
      if (t === 'alpha' || t === 'bravo' || t === 'charlie' || t === 'carrier') {
        pickRespawn(t);
      }
    });
  });
$('pickerCancel')?.addEventListener('click', hideRespawnPicker);
$('restartBtn')?.addEventListener('click', () => {
  if (started) restartRound();
});
$('restartBtn2')?.addEventListener('click', restartRound);
$('resumeBtn')?.addEventListener('click', togglePause);
// Pause-menu Setup: open the same SandboxMenu drawer over the paused
// game. "Apply & Restart" restarts the round with the new setup (via
// startRound — the carrier is reset, never rebuilt, so no double hull);
// "Resume" (the menu's own button) closes the drawer and unpauses with
// the old settings. The pre-flight and in-game menus share the one
// component and the one config, so settings stay consistent.
$('setupBtn')?.addEventListener('click', () => {
  if (!started) return;
  menu.applyLive(cfg);
  menu.setMode(
    'ingame',
    (next) => {
      menu.setMode('preflight');
      void startRound(next).catch((e) => {
        started = false;
        paused = false;
        menu.setMode('preflight');
        menu.show();
        const loadErr = $('loadErr');
        if (loadErr) {
          loadErr.textContent =
            'Could not restart round: ' +
            (e instanceof Error ? e.message : String(e));
        }
      });
    },
    () => {
      // Resume with the old settings: close the drawer, unpause.
      menu.setMode('preflight');
      menu.hide();
      $('pauseScreen')?.classList.add('hidden');
      paused = false;
    },
  );
  $('pauseScreen')?.classList.add('hidden');
  menu.show();
});
$('camBtn')?.addEventListener('click', cycleCam);
$('soundBtn')?.addEventListener('click', () => {
  soundOn = !soundOn;
  const btn = $('soundBtn');
  if (btn) {
    btn.textContent = soundOn ? '🔊' : '🔇';
    btn.classList.toggle('off', !soundOn);
  }
  if (!soundOn) lockTone(false);
});
$('helpBtn')?.addEventListener('click', () => {
  $('helpScreen')?.classList.remove('hidden');
});
$('helpDone')?.addEventListener('click', () => {
  $('helpScreen')?.classList.add('hidden');
});
$('helpScreen')?.addEventListener('click', (e) => {
  if (e.target === $('helpScreen')) $('helpScreen')?.classList.add('hidden');
});

// Footer version string (v4.1 · Vite + Rapier).
{
  const ver = document.querySelector('#startScreen .ver');
  if (ver) ver.textContent = VERSION_FOOTER;
}

// ---- time of day / sky / night glow ----
function updateSky(dt: number): void {
  todH = (todH + dt / 60) % 24; // 1 game minute per real second
  const day = clamp(Math.sin(((todH - 6) / 12) * Math.PI), 0, 1);
  nightF = 1 - day;
  const sunEl = Math.sin(((todH - 6) / 12) * Math.PI);
  const sunAz = (todH / 24) * Math.PI * 2;
  sun.position.set(
    Math.cos(sunAz) * 800,
    Math.max(60, sunEl * 900),
    Math.sin(sunAz) * 800,
  );
  sun.intensity = 0.15 + 2.1 * day;
  sun.color.setHSL(0.09, 0.7, 0.55 + 0.2 * day);
  moon.position.set(
    -sun.position.x,
    Math.max(120, -sunEl * 700),
    -sun.position.z,
  );
  moon.intensity = 0.55 * nightF;
  hemi.intensity = 0.25 + 0.65 * day;
  const skyDay = new THREE.Color(0x87b5d9);
  const skyDusk = new THREE.Color(0xe8905a);
  const skyNight = new THREE.Color(0x0a1226);
  const sky = new THREE.Color();
  if (day > 0.55) sky.copy(skyDay);
  else if (day > 0.05) sky.copy(skyDusk).lerp(skyDay, (day - 0.05) / 0.5);
  else sky.copy(skyNight).lerp(skyDusk, day / 0.05);
  scene.background = sky;
  if (scene.fog instanceof THREE.Fog) scene.fog.color.copy(sky);
  for (const m of worldGen.floraMaterials) m.emissiveIntensity = 0.25 + nightF * 2.4;
}

// ---- camera (chase / cockpit / free orbit) ----
function updateCamera(dt: number): void {
  if (!player) return;
  const p = player;
  const fwd = p.fwd();
  const up = p.up();
  // Spectate (damage-model §5): while the player's wreck is falling
  // (dying) or in the 2 s after impact, the camera stays with the
  // wreck — chase math at a longer distance (40-45 m) so the fire and
  // the tumble read clearly. C may cycle to free-orbit around the
  // wreck (handled by cycleCam, which still works — camMode 2 orbits
  // p.pos, i.e. the wreck).
  // The chase offset uses the wreck's VELOCITY direction (live
  // finding 3), NOT its tumbling body axes: the old code built the
  // offset from fwd/up (the wreck's quaternion), so as the wreck
  // tumbled the camera swung chaotically and the wreck left the
  // frame. The velocity direction is stable (the wreck falls along
  // it), and the camera looks AT the wreck's position, not 40 m
  // ahead along a tumbling axis. When the wreck is nearly still
  // (on the ground after impact) the velocity is ~0, so fall back
  // to the body fwd.
  if (spectating) {
    const dist = 42;
    const velDir = p.vel.length() > 5
      ? p.vel.clone().normalize()
      : fwd.clone();
    const want = p.pos
      .clone()
      .addScaledVector(velDir, -dist)
      .addScaledVector(new THREE.Vector3(0, 1, 0), dist * 0.34);
    const floor = Math.max(
      heightfieldGroundHeight(want.x, want.z) + 6,
      p.pos.y + 2,
    );
    if (want.y < floor) want.y = floor;
    camera.position.lerp(want, 1 - Math.pow(0.001, dt));
    if (shakeT > 0) {
      camera.position.x += (Math.random() - 0.5) * shakeT * 4;
      camera.position.y += (Math.random() - 0.5) * shakeT * 4;
    }
    camera.lookAt(p.pos.clone());
    return;
  }
  if (camMode === 0) {
    const want = p.pos
      .clone()
      .addScaledVector(fwd, -camDist)
      .addScaledVector(up, camDist * 0.34);
    // v1.1: keep the sightline clear — clear not just the ground under the
    // camera but the terrain between it and the jet. Sampled mesh height:
    // the analytic heightAt() can sit ~13 m below the rendered surface
    // at plateau edges, which clipped the camera into terrain.
    const mid = p.pos.clone().addScaledVector(fwd, -camDist * 0.5);
    const floor = Math.max(
      heightfieldGroundHeight(want.x, want.z) + 6,
      heightfieldGroundHeight(mid.x, mid.z) + 8,
      p.pos.y + 2,
    );
    if (want.y < floor) want.y = floor;
    camera.position.lerp(want, 1 - Math.pow(0.001, dt));
    if (shakeT > 0) {
      camera.position.x += (Math.random() - 0.5) * shakeT * 4;
      camera.position.y += (Math.random() - 0.5) * shakeT * 4;
    }
    camera.lookAt(p.pos.clone().addScaledVector(fwd, 40));
  } else if (camMode === 1) {
    camera.position
      .copy(p.pos)
      .addScaledVector(fwd, 5)
      .addScaledVector(up, 2.2);
    camera.quaternion.copy(p.quat);
  } else {
    const cx =
      p.pos.x + Math.sin(camOrbit.yaw) * Math.cos(camOrbit.pitch) * camDist;
    const cz =
      p.pos.z + Math.cos(camOrbit.yaw) * Math.cos(camOrbit.pitch) * camDist;
    const cy = p.pos.y + Math.sin(camOrbit.pitch) * camDist;
    camera.position.set(cx, Math.max(cy, heightfieldGroundHeight(cx, cz) + 3), cz);
    camera.lookAt(p.pos);
  }
}

// ---- player input merge (template playerInput) ----
function playerInput(dt: number): void {
  const p = player;
  if (!p) return;
  // VTOL vector slew (Issue 2): the toggle moves the nozzle at
  // 36 deg/s toward its target instead of snapping it, so the
  // transition is progressive. When slewing AFT (to forward flight)
  // the nozzle may only come down as fast as the wing ACTUALLY takes
  // over the weight — the STOVL transition rule: vertical thrust +
  // wing lift must stay >= weight, so sin(vector) >= (1 - liftFrac)
  // / (T/W), where liftFrac is the wing's real lift right now
  // (gload). At zero airspeed the thrust/weight margin (1.07) lets
  // the nozzle to ~70 deg while holding 1 g; as the pilot rotates
  // and speed builds, the wing takes the load and the nozzle follows
  // it down. The first version of this floor used the CLmax airspeed
  // bound (1 - (v/Vs)^2) instead of the real lift — but the wing is
  // nowhere near CLmax at level pitch, so the nozzle came down far
  // too fast and the jet dropped ~60 m before the wing flew (the
  // transition probe caught it: AoA only reached 10 deg as the jet
  // sank, too late). The gload floor tracks the real lift, so the
  // nozzle cannot outrun the wing. Runs every frame, alive or not.
  if (vtolTarget !== null && p.spec.vtol) {
    let goal = vtolTarget;
    if (vtolTarget === 0) {
      const liftFrac = Math.max(0, Math.min(1, p.gload));
      const tw =
        (p.throttle * p.spec.thrustMax) / (p.spec.mass * 9.81);
      const need = (1 - liftFrac) / Math.max(tw, 0.01);
      const minVec = Math.asin(Math.min(1, Math.max(0, need))) * (180 / Math.PI);
      goal = Math.max(0, minVec);
    }
    const step = 36 * dt;
    const cur = p.vector;
    if (Math.abs(goal - cur) <= step) {
      p.vector = goal;
      // Done only when the vector reached the toggle's target itself
      // (for the aft slew the goal is the moving airspeed floor, so
      // the slew continues until that floor is 0).
      if (goal === vtolTarget) vtolTarget = null;
    } else {
      p.vector = cur + Math.sign(goal - cur) * step;
    }
    syncVecSlider(p.vector);
  }
  // Consume the PC events FIRST, before the alive/dying guard: R and P
  // must work while the player is dead or dying. R is the spectate
  // skip ("R to skip" on the banner) and the dead-state respawn — the
  // old code returned before consuming events when !alive || dying,
  // so R did NOTHING in exactly the states that need it (the v2
  // play-test: "pressing R does nothing", only the manual Restart
  // button recovered the stuck jet).
  const ev = pcInput.consumeEvents();
  if (ev.cameraResetPressed) {
    // R is the respawn key whenever respawning is relevant (issue 4):
    // with the picker open it picks the highlighted default; during
    // spectate or when dead it opens the picker (skipSpectate); when
    // the jet is beached (the stuck detector has fired) it opens the
    // picker with Cancel. In normal flight R is still the camera
    // reset, as before.
    if (pickerOpen) {
      pickRespawn(highlightedRespawnChoice());
    } else if (spectating || !p.alive) {
      skipSpectate();
    } else if (stuckTime > 2) {
      // Pass 5 (item 7): R used to require stuckHintActive, which is
      // only set after the 10 s stuck toast fires — so pressing R in
      // the first 10 s of being beached just reset the camera and did
      // nothing. R now opens the picker as soon as the jet has been
      // beached for > 2 s (stuckTime accumulates only while beached
      // and resets the moment the jet moves / takes off).
      showRespawnPicker(true);
    } else {
      camOrbit = { yaw: 0, pitch: 0.32 };
      camDist = 30;
    }
  }
  if (ev.pausePressed) togglePause();
  if (!p.alive || p.dying) {
    // Dead or dying: no control (damage-model §5 — while spectating
    // the player cannot control anything). The controller's stepInternal
    // also zeroes the controls, but the input merge must not fight it.
    p.pitchIn = 0;
    p.rollIn = 0;
    return;
  }
  // Merge: touch stick, then PC (gamepad > mouse-yoke > keyboard, merged
  // inside PCFlightInput), then tilt adds. An active PC stick replaces the
  // touch stick; keyboard arrows already add on top inside PCFlightInput.
  const pc = pcInput.getControls();
  const tilt = tiltInput.getControls();
  let px: number;
  let rx: number;
  const pcPitch = pc.pitch ?? 0;
  const pcRoll = pc.roll ?? 0;
  if (pcPitch !== 0 || pcRoll !== 0) {
    px = pcPitch;
    rx = pcRoll;
  } else {
    // Touch stick, sim-style: pull BACK (drag down, joy.y > 0) = nose
    // up (px < 0); stick right = bank right (rx < 0). This matches
    // the help text ("pull the stick back") and the keyboard (ArrowUp
    // = nose up). The old mapping had stick up = nose up, which is
    // why the live tester pulled down and the jet did not rotate —
    // the pull they expected was the opposite of what the stick did.
    px = -joy.y;
    rx = -joy.x;
  }
  px += tilt.pitch ?? 0;
  rx += tilt.roll ?? 0;
  p.pitchIn = clamp(px, -1, 1);
  p.rollIn = clamp(rx, -1, 1);
  if (pc.throttle != null) {
    p.thrTarget = pc.throttle;
    hud.syncThrottle(pc.throttle);
  }
  // Rudder: Q/E through the PC provider (was hardcoded to 0).
  p.yawIn = pc.yaw ?? 0;

  if (firing || ev.fireHeld) {
    weapons.fireGunAtTargets(p, enemies.concat(traffic), dt);
  }
  if (ev.missilePressed && p.lockT > 1.2 && p.lockTarget) {
    weapons.launchMissile(p, p.lockTarget);
  }
  if (ev.flarePressed) weapons.fireFlares(p);
  if (ev.gearPressed) $('gearBtn')?.dispatchEvent(new Event('click'));
  if (ev.hookPressed) $('hookBtn')?.dispatchEvent(new Event('click'));
  // Flaps [ / ] step 10 deg, X toggles the airbrake — and the right
  // panel sliders follow so the touch UI and the keyboard agree.
  if (ev.flapsDownPressed) {
    p.flaps = clamp(p.flaps + 10, 0, 30);
    syncFlapSlider(p.flaps);
  }
  if (ev.flapsUpPressed) {
    p.flaps = clamp(p.flaps - 10, 0, 30);
    syncFlapSlider(p.flaps);
  }
  if (ev.airbrakePressed) {
    p.airbrake = p.airbrake > 0.5 ? 0 : 1;
    syncBrkSlider(p.airbrake);
  }
  if (ev.vtolPressed) toggleVtol();
  if (ev.cameraPressed) cycleCam();
  if (ev.bombPressed) $('bombBtn')?.dispatchEvent(new Event('click'));
  p.brake = ev.brakeHeld || touchBrake;

  // NOTE: gunCd / flareCd are decayed inside Aircraft.step() — do not
  // decay them here too (the first draft did, halving both cooldowns).

  // VTOL pad credit (G5): F-35, vector 70 deg+, on a MARKED deck pad,
  // slow + gentle descent relative to the moving deck. Any active
  // carrier's pad counts (issue 5).
  if (p.onDeck && p.spec.vtol && p.vector >= 70 && !p.vtolCounted) {
    for (const c of carriers) {
      if (!c.active || !c.alive) continue;
      const rel = c.localPosition(p.pos);
      const onPad =
        rel &&
        VTOL_PADS.some(
          ([px2, pz2]) => Math.abs(rel.x - px2) < 18 && Math.abs(rel.z - pz2) < 18,
        );
      if (onPad) {
        const relV = p.vel.clone().sub(c.vel).length();
        if (relV < 6 && p.vsi > -4) {
          p.vtolCounted = true;
          addScore(75, 'vtol');
          hud.toast('VTOL LANDING', '+75 — lift-fan down on the pad.');
        }
        break;
      }
    }
  }
  if (!p.onDeck && !p.grounded) p.vtolCounted = false;

  // Carrier trap credit: trapped flips false -> true on a wire catch.
  if (p.trapped && !prevTrapped) {
    addScore(100, 'trap');
    hud.toast('TRAP!', '+100 — wire caught, welcome aboard.');
  }
  prevTrapped = p.trapped;

  // Missile lock (G6): hold the target inside the 15 deg cone, < 2500 m.
  // A dying (falling) jet is untargetable — the lock breaks.
  let best: Aircraft | null = null;
  let bd = 2500;
  for (const e of enemies) {
    if (!e.alive || e.dying) continue;
    const to = e.pos.clone().sub(p.pos);
    const d = to.length();
    if (d < bd && p.fwd().angleTo(to.normalize()) * R2D < 15) {
      bd = d;
      best = e;
    }
  }
  if (best) {
    p.lockTarget = best;
    p.lockT += dt;
    weapons.setLockTone(true, clamp(p.lockT / 1.2, 0, 1));
    if (p.lockT > 1.2) beep(1320, 0.05, 'sine', 0.03);
  } else {
    p.lockTarget = null;
    p.lockT = Math.max(0, p.lockT - dt * 2);
    weapons.setLockTone(false, 0);
  }
}

// ---- crash consequences (main owns these; Aircraft only flips alive) ----
function handleDeaths(): void {
  const all: Aircraft[] = [
    ...(player ? [player] : []),
    ...enemies,
    ...traffic,
  ];
  for (const a of all) {
    const was = prevAlive.get(a) ?? true;
    if (was && !a.alive) {
      // Ground-impact explosion, scaled by the remaining fuel
      // (damage-model §4/§6 step 5): a wreck with fuel left fireballs;
      // without fuel it crumples. The position is the IMPACT position
      // (finishCrash runs at ground contact), not where the jet was
      // shot down.
      vfx.spawnExplosion(a.pos.clone(), 1 + a.fuel);
      sfx('boom');
      vfx.scorchDecal(a.pos.clone());
      // Debris scatter (pass 5, item 3): was gated on
      // crashedFromDying, so an instant terrain/hard-landing crash
      // left NO wreckage — the jet just vanished. A terrain crash is
      // a crash too: it scatters debris at the (now surface-clamped)
      // impact position, so the wreck reads on the ground.
      for (let i = 0; i < 4; i++) {
        const ang = Math.random() * Math.PI * 2;
        const sp = 10 + Math.random() * 20;
        vfx.spawnDebris(
          null,
          rapierWorld,
          a.pos.clone(),
          new THREE.Vector3(
            Math.cos(ang) * sp,
            5 + Math.random() * 10,
            Math.sin(ang) * sp,
          ),
        );
      }
      if (a.isPlayer) {
        hud.toast('YOU WENT DOWN', 'Choose where to respawn.');
        // Pass 5 (item 3): an instant crash (hard landing / terrain)
        // never engaged the spectate camera — the jet vanished and the
        // chase camera had nothing to follow. Engage spectate for the
        // 3 s until the picker so the camera holds on the wreck site
        // (the wreck position is surface-clamped by crash(), so the
        // camera targets the surface, not a point under the terrain).
        // A dying crash already spectates from the dying detection.
        if (!a.crashedFromDying && !spectating) {
          spectating = true;
          setSpectateBanner(true, a.lastCrashReason ?? undefined);
        }
        // A dying crash (shot down, fell, impacted) shows the picker
        // 2 s after impact (the spectate already showed the fall); an
        // instant crash (hard landing / terrain) keeps the 3 s window.
        // Issue 4: the player CHOOSES the runway/deck — no more
        // auto-respawn at the nearest runway.
        respawnTimer = setTimeout(
          () => {
            respawnTimer = null;
            if (!pickerOpen) showRespawnPicker(false);
          },
          a.crashedFromDying ? 2000 : 3000,
        );
      } else {
        // v1.1: only PLAYER-caused kills score.
        if (a.lastHitBy && a.lastHitBy.isPlayer) addScore(100, 'kill');
        const ai = aiControllers.get(a);
        if (ai) ai.state = 'dead';
      }
    }
    prevAlive.set(a, a.alive);
  }
}

// ---- failure scenarios ----
function applyFailure(dtElapsed: number): void {
  if (failDone || cfg.failure === 'none' || !player) return;
  if (dtElapsed <= failAt) return;
  failDone = true;
  const p = player;
  if (failScenario === 'engineL') {
    p.failEngine = true;
    p.failEngineSide = -1;
    hud.toast('FAILURE — LEFT ENGINE OUT', 'Thrust halved, yawing left. Rudder to hold it.');
  } else if (failScenario === 'engineR') {
    p.failEngine = true;
    p.failEngineSide = 1;
    hud.toast('FAILURE — RIGHT ENGINE OUT', 'Thrust halved, yawing right. Rudder to hold it.');
  } else if (failScenario === 'engineFire') {
    p.failEngine = true;
    p.failFire = true;
    hud.toast('FAILURE — ENGINE FIRE', 'Fire! HP is burning away — land now.');
  } else if (failScenario === 'hydraulic') {
    p.failHydro = true;
    hud.toast('FAILURE — HYDRAULIC', 'Elevator authority lost. Pitch is heavy.');
  } else if (failScenario === 'gearJam') {
    p.failGearJam = true;
    p.gear = true;
    $('gearBtn')?.classList.add('on');
    const gb = $('gearBtn') as HTMLButtonElement | null;
    if (gb) gb.disabled = true;
    hud.toast('FAILURE — GEAR JAM', 'Gear is stuck down. Drag is up, top speed is down.');
  } else if (failScenario === 'fuelLeak') {
    p.failFuel = true;
    hud.toast('FAILURE — FUEL LEAK', 'Fuel draining fast. Get home.');
  }
}

// ---- main loop ----
/** One AI aircraft fixed step: run its controller, fire on its flags,
 *  then step its physics with the returned controls (the old code called
 *  update() and threw the controls away, so AI inputs never reached the
 *  aircraft). */
function stepAi(ac: Aircraft, dt: number, turbAcc: THREE.Vector3 | null): void {
  const ai = aiControllers.get(ac);
  if (ai && player) {
    // The AI uses the primary carrier (index 0); with 0 carriers it
    // gets a null-object stub (deckHeightAt → −Infinity, so no carrier
    // landing is ever attempted).
    const aiCarrier: HelicarrierController =
      heliCountActive > 0
        ? carrier
        : ({ deckHeightAt: () => -Infinity } as unknown as HelicarrierController);
    const ctrl = ai.update(dt, player, aiCarrier, elapsed);
    if (ai.fireGun) weapons.aiGunShot(ac, player, ai.skill.err);
    if (ai.fireMissile) weapons.launchMissile(ac, player);
    if (ai.dropBomb) weapons.dropBomb(ac);
    // Exact heightfield surface for the AI's ground detection / crash
    // checks (issue 6) — the analytic groundHeight let an AI jet rest
    // on the collider while the controller thought it was 90+ m in the
    // air (the v2 hang), and let it fly into underestimated hills.
    ac.step(dt, ctrl, windVec, heightfieldGroundHeight, turbAcc);
  } else {
    ac.step(dt, windVec, turbAcc);
  }
}

const clock = new THREE.Clock();
let hudT = 0;
function animate(): void {
  requestAnimationFrame(animate);
  // v1.1: substep the physics so sim time tracks real time even on a slow
  // renderer. Each substep stays <= 0.05 s.
  const rawDt = Math.min(clock.getDelta(), 0.25);
  fpsEMA += (1 / Math.max(rawDt, 1e-3) - fpsEMA) * 0.05;
  lastDt = rawDt;
  if (!started || paused) {
    // Consume input events even while paused (live finding 10): P
    // pauses the game but animate() returned before playerInput /
    // consumeEvents() ran, so P could never unpause it. The HUD
    // pause button worked because it is a DOM click handler. Now P
    // (and Escape) toggle pause from the paused branch too.
    if (paused) {
      const ev = pcInput.consumeEvents();
      if (ev.pausePressed) togglePause();
    }
    renderer.render(scene, camera);
    return;
  }
  const prevElapsed = elapsed;
  elapsed += rawDt;
  applyFailure(elapsed);
  updateSky(rawDt);
  const turb = cfg.weather.turbulence;
  const sub = Math.max(1, Math.ceil(rawDt / 0.05));
  const dt = rawDt / sub;
  for (let s = 0; s < sub; s++) {
    const turbAcc = turb
      ? new THREE.Vector3(
          (Math.random() - 0.5) * turb,
          (Math.random() - 0.5) * turb * 0.5,
          (Math.random() - 0.5) * turb,
        )
      : null;
    playerInput(dt);
    // Carriers move first so the kinematic deck velocity is live for
    // this step; then Rapier integrates, contacts are dispatched into
    // notifyContact(), and the aero steps read fresh body state.
    for (const c of carriers) c.update(dt);
    // Rapier's step length MUST match this substep's dt. world.timestep
    // was never set, so Rapier integrated a fixed 1/60 s while the
    // controller's substep dt could be 0.05 s — a 3x velocity/position
    // desync at low frame rates (the spawn-crash candidate: the
    // controller's velocity grew 3x faster than Rapier moved the body).
    rapierWorld.timestep = dt;
    rapierWorld.step(eventQueue);
    drainCollisionEvents();
    if (player) player.step(dt, windVec, turbAcc);
    for (const e of enemies) {
      if (!e.alive) continue;
      stepAi(e, dt, turbAcc);
    }
    for (const t of traffic) {
      if (!t.alive) continue;
      stepAi(t, dt, turbAcc);
    }
    weapons.update(dt, enemies);
    vfx.update(dt);
    handleDeaths();
  }
  // Spectate detection (damage-model §5): when the player starts the
  // dying fall, the camera follows the wreck until impact + 2 s (or
  // until the player presses R / the Respawn button). The banner shows
  // the killer when lastHitBy is known.
  if (player) {
    if (player.dying && !prevPlayerDying) {
      spectating = true;
      const killer =
        player.lastHitBy?.isPlayer === false ? 'an enemy' : undefined;
      setSpectateBanner(true, player.dyingReason ?? undefined, killer);
    }
    prevPlayerDying = player.dying;
    // If the player respawned (alive again), spectate is over.
    if (player.alive && !player.dying && spectating && !respawnTimer) {
      spectating = false;
      setSpectateBanner(false);
    }
  }
  // Stuck detector (issue 4 / live finding 2): the jet survived onto
  // a steep slope at a steep tilt, beached — alive, ~0 kt, not
  // grounded and not on a deck, so no control authority (qAuth ≈ 0)
  // and no respawn timer ever fires. After 10 s of it, hint that R
  // opens the respawn picker. A jet parked on a runway/deck is
  // grounded/onDeck, so it never triggers.
  // Pass 5 (item 6): the detector fired in a VTOL HOVER — a hovering
  // jet is alive, not grounded, not on a deck, and ~0 kt, which is
  // exactly the beached signature. Gate it on being NEAR THE GROUND
  // (AGL < 10 m): a beached jet is on a slope at ground level, a
  // hovering jet is hundreds of metres up.
  if (player) {
    const agl = player.pos.y - heightfieldGroundHeight(player.pos.x, player.pos.z);
    const beached =
      player.alive &&
      !player.dying &&
      !player.grounded &&
      !player.onDeck &&
      player.vel.length() < 1 &&
      agl < 10;
    if (beached) {
      stuckTime += dt;
      if (stuckTime > 10 && !stuckToastShown) {
        stuckToastShown = true;
        stuckHintActive = true;
        hud.toast('STUCK?', 'Press R to respawn at a runway or the deck.');
      }
    } else {
      stuckTime = 0;
      stuckHintActive = false;
      stuckToastShown = false;
    }
  }
  // Fire / smoke visuals (damage-model §6 step 6): getDamageFx() was
  // dead code — it is now called every frame for the player and every
  // enemy/traffic jet, and drives CrashVFXManager.spawnSmoke. Orange
  // flame + black smoke for a fuel fire, grey smoke for engine damage
  // / light damage, white for light damage. A plume every 0.5 s per
  // damaged aircraft keeps a continuous trail (plume life 6 s).
  smokeT += rawDt;
  if (smokeT >= 0.5) {
    smokeT = 0;
    const allFx: Aircraft[] = [
      ...(player ? [player] : []),
      ...enemies,
      ...traffic,
    ];
    for (const a of allFx) {
      if (!a.alive) continue;
      const fx = a.getDamageFx();
      if (fx.fire) {
        // Fuel fire: orange flame + black smoke; engine fire: orange
        // flame; low-hp fire (no state machine): orange flame.
        // A DYING jet gets an ATTACHED plume (live finding 3): the
        // wreck falls at 60-100 m/s and outruns a static puff in
        // 0.1 s, so the v3 shoot-down showed no fire/smoke in flight.
        // An attached plume is parented to the jet's group and falls
        // with it, so the fire is visibly ON the wreck. A merely
        // damaged (flying) jet gets the static plume as before.
        const colour =
          fx.fireKind === 'fuel' ? 0x1a1a1a : fx.fireKind === 'engine' ? 0xff7722 : 0xff7722;
        if (a.dying) {
          vfx.spawnSmoke(a.pos.clone(), colour, a.group);
          if (fx.fireKind === 'fuel') {
            vfx.spawnSmoke(a.pos.clone(), 0xff7722, a.group);
          }
        } else {
          vfx.spawnSmoke(a.pos.clone(), colour);
          if (fx.fireKind === 'fuel') {
            vfx.spawnSmoke(a.pos.clone(), 0xff7722);
          }
        }
      } else if (fx.smoke) {
        vfx.spawnSmoke(a.pos.clone(), 0x888888);
      }
    }
    // Dying helicarrier (issue 9): a sinking carrier trails black
    // smoke + orange flame from its hull and pods while it goes down,
    // and keeps smoking as a wreck on the ground until it is written
    // off. Same 0.5 s cadence as the aircraft plumes.
    for (const c of carriers) {
      if (!c.active || !c.alive || !c.dying) continue;
      vfx.spawnSmoke(c.pos.clone(), 0x1a1a1a);
      vfx.spawnSmoke(c.pos.clone(), 0xff7722);
    }
  }
  worldGen.shiftOriginIfNeeded(player?.pos ?? new THREE.Vector3());
  worldGen.update(rawDt, windVec);
  // Survival score: +1 per second alive AND AIRBORNE (research §6 — the
  // old version paid +1/s while the jet sat stuck on the runway).
  // Gated on AGL > 10 m (live finding 7), not !grounded: grounded
  // flickers false while the jet rolls on the ground, so the score
  // ticked ~1/s at 6 kt rolling after a respawn. AGL is the exact
  // heightfield surface, so it cannot flicker.
  // Pass 5 (item 9): the AGL gate alone was not enough — a jet
  // PARKED ON THE DECK sits at y=350 over terrain at ~43 m, so its
  // "AGL" is ~307 m and the score ticked ~1/s while parked on the
  // deck at 0% throttle (and a hovering jet, airborne at ~0 kt,
  // ticked too). The score is a flight/survival counter: it now
  // requires the jet to be actually flying — airborne (not grounded,
  // not on a deck), above 10 m, AND moving (> 20 m/s). Parked and
  // hovering no longer tick.
  if (
    player?.alive &&
    !player.grounded &&
    !player.onDeck &&
    player.pos.y - heightfieldGroundHeight(player.pos.x, player.pos.z) > 10 &&
    player.vel.length() > 20 &&
    Math.floor(elapsed) !== Math.floor(prevElapsed)
  ) {
    score += 1;
  }
  if (shakeT > 0) shakeT = Math.max(0, shakeT - rawDt);
  updateCamera(rawDt);
  hudT += rawDt;
  if (hudT > 0.1) {
    hudT = 0;
    if (player) {
      // CCIP (bomb impact point) is computed before the HUD update so
      // the below-min-release-alt flag feeds the warn priority.
      const ccip = weapons.computeCcip(
        player,
        heightfieldGroundHeight(player.pos.x, player.pos.z),
        camera,
      );
      if (ccip) hud.showCcip(ccip.x, ccip.y, ccip.belowMinAlt);
      else hud.hideCcip();
      hud.update(player, {
        score,
        kills,
        traps,
        elapsed,
        lockTarget: player.lockTarget,
        belowMinReleaseAlt: ccip?.belowMinAlt ?? false,
      });
      const pipper = weapons.computeLeadPipper(
        player,
        player.lockTarget,
        camera,
      );
      if (pipper) hud.showPipper(pipper.x, pipper.y);
      else hud.hidePipper();
      hud.drawMinimapDefault({
        player,
        enemies,
        traffic,
        carriers: carriers
          .filter((c) => c.active)
          .map((c) => ({ alive: c.alive, pos: c.pos, yaw: c.yaw })),
        runways: worldGen.runways,
        originShift: worldGen.originShift,
      });
    }
    updateDebugPanel();
  }
  renderer.render(scene, camera);
}

function updateDebugPanel(): void {
  if (!DEBUG_ON) return;
  const rows: [string, string][] = [
    ['fa18', 'F/A-18F'],
    ['f35a', 'F-35A'],
    ['f22', 'F-22'],
    ['f15e', 'F-15E'],
    ['mig29', 'MiG-29'],
    ['su34', 'Su-34'],
    ['helicarrier', 'SHIELD Helicarrier'],
    ['heli', 'UH-60 (proc)'],
  ];
  const modelLines = rows.map(([k, label]) => {
    const s = modelStatus[k]!;
    return (
      label +
      ': ' +
      (s.state === 'ok'
        ? 'OK' + (s.tris ? ' ' + s.tris + ' tris' : '')
        : s.state === 'FAILED'
          ? 'FAILED ' + (s.error ?? '')
          : s.state)
    );
  });
  const lines = [
    `PANDORA ${VERSION_FOOTER} ?debug — FPS ${fpsEMA.toFixed(0)} · sim dt ${lastDt.toFixed(3)}s · rapier: on`,
    ...modelLines,
    `origin shift: ${worldGen.originShift.x.toFixed(0)}, ${worldGen.originShift.z.toFixed(0)} · gamepad: ${pcInput.gamepadName || 'none'} · tilt: ${tiltInput.isEnabled ? 'on' : 'off'}`,
    `stick: pitch ${cfg.invertPitch ? 'INV' : 'norm'} ${cfg.pitchSens.toFixed(2)}× · roll ${cfg.invertRoll ? 'INV' : 'norm'} ${cfg.rollSens.toFixed(2)}× · spawn ${cfg.spawn} · enemies ${cfg.enemies === 0 ? 'Off' : cfg.enemies} · fail ${cfg.failure} / ${cfg.failWhen}`,
    debugLog.length
      ? '--- warnings ---\n' + debugLog.slice(-8).join('\n')
      : '(no warnings yet)',
  ];
  hud.setDebug(lines);
}

// ---- boot: audio unlock on first gesture, then run ----
window.addEventListener('pointerdown', () => audio(), { once: true });
window.addEventListener('keydown', () => audio(), { once: true });
animate();
