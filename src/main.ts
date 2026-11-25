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
import { fixCanopyMaterials } from './aircraft/CanopyMaterials';
import {
  applyCameraFloor,
  chaseCameraPosition,
  deckOrbitYaw,
  orbitCameraPosition,
  orbitCameraPositionDeckAware,
} from './camera/OrbitCamera';
import { TerrainStreamer } from './world/TerrainStreamer';
import { FloatingOrigin } from './world/FloatingOrigin';
import { HELICARRIER_GLB } from './assets/models/helicarrier';
import { PandoraWorldGenerator } from './world/PandoraWorldGenerator';
import { RunwayPlacer } from './world/RunwayPlacer';
import { RUNWAY_TABLE } from './world/PandoraTerrainWorker';
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS, carrierLoop } from './entities/HelicarrierController';
import { WeaponSystem } from './combat/WeaponSystem';
import { CrashVFXManager } from './combat/CrashVFXManager';
import { FlightHUD, MS2KT, gameToTrueHdg } from './ui/FlightHUD';
import { SandboxMenu } from './ui/SandboxMenu';
import type { SandboxConfig } from './ui/SandboxMenu';
import { PCFlightInput } from './input/PCFlightInput';
import { MobileFlightInput, touchStickScale, touchPitchScale, touchDeadzone } from './input/MobileFlightInput';
import { clFromCurve } from './physics/IAerodynamicsSolver';

await RAPIER.init();

const VERSION_FOOTER = 'v4.12 · Vite + Rapier';
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
// Pass 8 (item 1): spawn trace — the first 3 s after a spawn, sampled
// per frame (VSI, AoA, onSurface, grounded, bank, pos.y, floorY), shown
// in the ?debug=1 panel. The live Alpha/Bravo spawn death was never
// reproduced in a driver across two passes, so the next live test
// reports these numbers instead of just "WENT DOWN — terrain". The
// trace restarts on every spawn (respawnPlayer / startRound) and a
// crash appends the death reason with the actual vs/bank values.
let spawnTrace: string[] = [];
let spawnTraceT = Infinity; // seconds since spawn; Infinity = inactive
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
// Pass 14 (worker E — MEASUREMENT ONLY, defaults unchanged): the
// v4.10 live check found FPS 4-9 everywhere and the render-side cost
// is still unmeasured from Node, so the ?debug=1 panel now shows a
// renderer.info readout (see updateDebugPanel) and these two URL
// knobs let the live check A/B the prime suspect — fragment fill at
// DPR<=2 + 4x MSAA + ACES — in one run:
//   ?dpr=1   (or 1.5) — drawing-buffer pixel ratio for this session
//   ?noaa=1  — create the renderer with antialias:false (no MSAA)
// With neither param the renderer is EXACTLY as before
// (antialias:true, DPR=min(devicePixelRatio, 2)). Do NOT turn either
// knob into a shipped cap until a live check measures the FPS effect.
const DEBUG_DPR_RAW = new URLSearchParams(location.search).get('dpr');
const DEBUG_DPR = DEBUG_DPR_RAW === null ? null : Math.min(2, Math.max(0.5, Number(DEBUG_DPR_RAW) || 2));
const DEBUG_NOAA = new URLSearchParams(location.search).get('noaa') === '1';
const RENDER_ANTIALIAS = !DEBUG_NOAA;
const canvas = $('game') as HTMLCanvasElement;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: RENDER_ANTIALIAS });
renderer.setPixelRatio(DEBUG_DPR ?? Math.min(window.devicePixelRatio, 2));
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
// Pass 11 (infinite terrain): chunked terrain streaming beyond the
// central 12 km map. The streamer owns everything outside the central
// 4x4 chunk block (which worldGen keeps — its mesh, heightfield and
// runway boxes are untouched, and chunk boundaries sit exactly on the
// old map edge, so there is no overlap and no gap). The map has no
// edge to hit and no visible snap: chunks stream in ahead of the
// player (radius 2 = 25 chunks / 15x15 km) and retire behind.
const terrainStreamer = new TerrainStreamer(scene, rapierWorld, floatingOrigin, {
  onColliderAdded: (h) => colliderOwners.set(h, 'terrain'),
  onColliderRemoved: (h) => colliderOwners.delete(h),
});
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
// disabled). All three fly the SAME loop (carrierLoop slot 0/1/2),
// phase-offset by ~1/3 lap, so the loops never overlap — with their
// own start index, so the carriers are spread around the circuit
// instead of stacked.
const CARRIER_SLOTS = [0, 1, 2];
/** Deck-spawn offset (m aft of the carrier group origin). The aft
 *  deck rect reaches local z = -128.8 (deck_top meshes), so 85 m aft
 *  is 16 m inside the stern with ~265 m of deck ahead. The old 120 m
 *  was only 8.8 m from the stern, and the live deck spawn fell off
 *  the stern to the terrain under the carrier and slid at the
 *  carrier's 78 kt (pass 9, item 4). */
const DECK_SPAWN_AFT_M = 85;
const carriers: HelicarrierController[] = CARRIER_SLOTS.map(
  (slot, i) => {
    const loop = carrierLoop(slot);
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
      startIndex: loop.startIndex,
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
        // Pass 8 (item 3): the toast names WHICH carrier and that this
        // is the ground impact (the start of the wreck), not the
        // write-off ~10 s later — the old identical 'CARRIER DOWN'
        // on both paths made one death read as two, and gave no way to
        // tell which of the three carriers was lost.
        hud.toast(
          `CARRIER ${i + 1} IMPACT`,
          `Helicarrier ${i + 1} (slot ${slot + 1}) hit the ground.`,
        );
      },
      onCarrierDown: () => {
        // Pass 8 (item 3): named, and distinct from the impact toast
        // above — this is the ~10 s-later write-off. The debug panel
        // (updateDebugPanel) carries the carrier's clearance and pod
        // HP at the time, so a live test can say which carrier and why.
        hud.toast(
          `CARRIER ${i + 1} DOWN`,
          `Helicarrier ${i + 1} (slot ${slot + 1}) is lost.`,
        );
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
  // Pass 11: the deck-top Y, so the auto-gear can measure AGL
  // above the deck (a carrier approach is 300 m above the terrain
  // but 30 m above the deck). The highest live carrier's deck.
  get deckTopY() {
    let best = -Infinity;
    for (const c of carriers) {
      if (!c.active || !c.alive) continue;
      best = Math.max(best, c.deckTopY);
    }
    return best;
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
  terrainStreamer.heightAt(x, z);

// ---- combat / VFX ----
let shakeT = 0;
const vfx = new CrashVFXManager(scene, {
  groundHeight: heightfieldGroundHeight,
  onShake: (amount) => {
    shakeT = Math.max(shakeT, amount);
  },
  // Pass 6 (item 2): register persistent VFX with the floating origin
  // so scorches/debris/plumes shift with the terrain instead of being
  // left behind as stray black discs after an origin shift.
  origin: floatingOrigin,
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
  // Pass 11 (infinite terrain, step 5): in-flight missiles/bombs
  // must shift with the world at an origin shift, or they jump ~2 km
  // relative to their target (which is registered) and fly wrong.
  origin: floatingOrigin,
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
let camMode = 0; // 0 chase, 1 free-orbit
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
/** Pass 11 (wreck free orbit): the camera mode the player had before
 *  spectate engaged, restored by skipSpectate (and by the respawned
 *  safety net) so the player returns to their chase view. */
let preSpectateCamMode = 0;
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
    // Pass 14 (worker A): capture is a progressive enhancement, not
    // the drag itself. A synthetic pointer event (or a browser that
    // rejects the capture) throws NotFoundError here — and the throw
    // used to skip joySet below, leaving joy.active true with
    // joy.x/joy.y = 0, i.e. the merge took the touch branch with a
    // centred stick for the whole drag: a dead stick that looks
    // exactly like the v4.10 live "joystick held full-right, bank 0".
    // The drag still works without capture while the pointer stays
    // over the stick, so a failed capture must never lose the drag.
    try {
      (joyEl as HTMLElement).setPointerCapture(e.pointerId);
    } catch {
      // No capture — the pointermove handler still tracks the drag.
    }
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

// Pass 8 (item 1): (re)start the spawn trace — called on every spawn
// (respawnPlayer / startRound) so the ?debug=1 panel shows the first
// 3 s of the CURRENT life, not a stale one. Pass 9 (item 7): the
// warnings section (debugLog) is cleared here too — it accumulated
// every "CRASH terrain ..." line across rounds and restarts and was
// never cleared (5-6 stale lines by the end of a session).
function startSpawnTrace(): void {
  spawnTrace = [];
  spawnTraceT = 0;
  debugLog.length = 0;
}

// Pass 8 (item 1): reset every flight input to neutral on a spawn, so
// a stuck or residual input from the previous life cannot deflect the
// fresh jet. The PC mouse yoke engages on any canvas pointerdown and
// a pointerup outside the canvas never reaches it, so yoke.active can
// stick true with a frozen deflection; the touch joystick is a
// module-level object that respawnPlayer never cleared; tilt holds a
// calibration offset. A new life starts with neutral controls.
function resetFlightInputs(): void {
  joy.active = false;
  joy.x = 0;
  joy.y = 0;
  const knob = $('joyKnob');
  if (knob) knob.style.transform = '';
  pcInput.calibrate();
  tiltInput.calibrate();
}

// Free-orbit drag + pinch zoom on the canvas (template ~1855–1885).
let orbitDrag: { x: number; y: number; id: number } | null = null;
let pinchD = 0;
canvas.addEventListener('pointerdown', (e) => {
  audio();
  if (camMode === 1) orbitDrag = { x: e.clientX, y: e.clientY, id: e.pointerId };
});
canvas.addEventListener('pointermove', (e) => {
  if (orbitDrag && e.pointerId === orbitDrag.id && camMode === 1) {
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
  // Manual override: the pilot's command holds off the automatic
  // gear (pass 10, item 3) for 8 s.
  player.setGearManual(!player.gear);
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
  // Pass 11 (wreck free orbit): back to the view the player had
  // before the wreck (chase).
  camMode = preSpectateCamMode;
  pcInput.setCameraMode(camMode);
  setSpectateBanner(false);
  showRespawnPicker(false);
}

/** Respawn target (issue 4): a runway by name, or the helicarrier
 *  deck. The last choice is the highlighted default in the picker. */
type RespawnTargetId = 'alpha' | 'bravo' | 'charlie' | 'carrier';
const RESPAWN_CHOICE_KEY = 'pandora-planes-last-respawn';
let lastRespawnChoice: RespawnTargetId = 'alpha';
// Pass 9 (item 2): the ?debug=1 panel's "spawn ..." label. It showed
// cfg.spawn (the MENU choice) so it always read "spawn alpha" even
// after a Bravo/Charlie/deck respawn. This tracks the ACTUAL current
// spawn target, set in respawnPlayer/startRound.
let lastSpawnLabel: RespawnTargetId = 'alpha';
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
/** Previous-frame player position for the unresponsive watchdog
 *  (pass 15, worker E, item 6): a frozen jet's `vel` field is stale
 *  (deck velocity ~40 m/s on a deck respawn, 0 kt on a runway spawn),
 *  so the beached detector's `vel < 1` gate never fires for it and R
 *  can never open the picker. Movement is measured from the position
 *  instead — a jet that does not move is unresponsive whatever its
 *  stale velocity says. */
let prevWatchPos: THREE.Vector3 | null = null;
/** Pass 17 (worker E, item 1): a fresh life starts with a fresh stuck
 *  detector — a stuckTime carried over from the previous life would
 *  let R open the picker on a healthy jet in its first seconds, and
 *  a stale prevWatchPos would read the respawn teleport as movement
 *  (or mask the first frames of a real freeze). Called from
 *  respawnPlayer and startRound. */
function resetStuckDetector(): void {
  stuckTime = 0;
  stuckHintActive = false;
  stuckToastShown = false;
  prevWatchPos = null;
}

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
  // Pass 14 (worker D, item 3): Cancel is ALWAYS offered — the old
  // code hid it whenever the jet was dead, so the picker covered
  // the centre of the wreck view with no way to dismiss it until
  // respawn. Dead: the button reads "Watch the wreck" and closing
  // the picker returns to the spectate orbit (the respawn timer
  // already fired, so nothing re-opens the picker; R / the Respawn
  // button re-opens it via skipSpectate). Stuck/alive: it keeps
  // reading "Cancel" and just closes the picker.
  if (cancel) {
    cancel.style.display = '';
    cancel.textContent = forStuck ? 'Cancel' : 'Watch the wreck';
  }
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
  lastSpawnLabel = choice;
  const shift = floatingOrigin.cumulativeShift;
  let spawnedOnDeck = false;
  // Pass 8 (item 4): a deck respawn needs a LIVE carrier. The old
  // check was only `heliCountActive > 0` — after a CARRIER DOWN the
  // primary carrier is dead (its deck is gone: deckHeightAt returns
  // -Infinity for a dead carrier), so the jet respawned at the dead
  // wreck's deck position, on the ground with no deck under it, and
  // arrived dead/frozen with the HUD stuck on stale values. Fall
  // back to Alpha when no active carrier is alive.
  // Pass 17 (worker E, item 1): a DYING carrier is still alive, so
  // it also passes the test above — but its deck is already going
  // (deckHeightAt reads -Infinity once the wreck is written off,
  // and the deck is sinking under the jet until then), so a deck
  // respawn onto it strands the jet over a vanishing deck.
  // A dying carrier is not a live carrier: fall back to Alpha.
  const liveCarrier = carriers.find((c) => c.active && c.alive && !c.dying) ?? null;
  if (choice === 'carrier' && liveCarrier) {
    // Deck respawn (issue 4): the carrier's CURRENT position
    // and yaw, DECK_SPAWN_AFT_M aft of the deck centre, with the deck
    // point velocity (issue 3 — the linear velocity alone leaves the
    // jet behind when the carrier turns). A carrier choice with 0
    // carriers (or all carriers dead) falls through to the Alpha
    // runway below. 85 m aft is 16 m inside the stern edge (the aft
    // deck rect reaches local z = -128.8) with ~265 m of deck ahead;
    // the old 120 m was only 8.8 m from the stern, and the live deck
    // spawn fell off the stern to the terrain under the carrier and
    // slid at the carrier's 78 kt (pass 9) — the extra stern margin
    // plus the fixed origin shift (correct deck/ground detection)
    // keeps the jet on the deck.
    const c = liveCarrier;
    const fwd = new THREE.Vector3(Math.sin(c.yaw), 0, Math.cos(c.yaw));
    const sx = c.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = c.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    player.resetAt(sx, c.deckTopY + 2.2, sz, (c.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    player.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    player.vel.copy(dv);
    spawnedOnDeck = true;
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
  // Pass 8 (item 1): a new life starts with neutral controls — clear
  // any stuck yoke / deflected touch joystick / tilt offset from the
  // previous life so it cannot deflect the fresh jet.
  resetFlightInputs();
  // Pass 8 (item 1): restart the spawn trace for this life.
  startSpawnTrace();
  camMode = 0;
  pcInput.setCameraMode(0);
  // Pass 11 (chase framing): every life starts at the default
  // framing — a zoomed-out (120 m, jet a speck) or zoomed-in (14 m,
  // jet fills the frame) state from the previous life must not
  // persist into the new one.
  camDist = 30;
  camOrbit = spawnedOnDeck
    ? { yaw: deckOrbitYaw(liveCarrier ? liveCarrier.yaw : carrier.yaw), pitch: 0.32 }
    : { yaw: 0, pitch: 0.32 };
  // Pass 14 (worker D, item 1): snap the camera onto the chase
  // want too — the mode/distance reset above is not enough, the
  // camera position itself was left at the wreck spectate orbit
  // (possibly kilometres from the respawn runway) and lerped in
  // through terrain. Deck respawn snaps under the overhead sheet
  // (the live carrier the jet spawned on); runway respawn snaps
  // with no deck ceiling.
  snapCameraToChase(spawnedOnDeck && liveCarrier ? liveCarrier.deckTopY : null);
  clearTransientUI();
  resetStuckDetector();
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
  resetStuckDetector();
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
    // Pass 11: glass canopies (the GLB's own canopy meshes made
    // transparent — the "Glass" material renders black otherwise).
    // v4.11: the procedural cockpit kit is GONE (Nicholas: "remove
    // the ugly extra additions on the nose. remove cockpit view") —
    // no panel, dials, screens, HUD glass, seat or headrest on any
    // jet; the GLB's own canopy stays.
    fixCanopyMaterials(player.group);
  } catch (e) {
    modelStatus[cfg.aircraftKey] = {
      state: 'FAILED',
      error: e instanceof Error ? e.message : String(e),
    };
    console.warn(cfg.aircraftKey + ' model failed', e);
  }
  // Pass 15 (worker E, standing item — during-load stale HUD): the
  // HUD updated while the new jet's GLB was still loading, so a fresh
  // Aircraft (identity quaternion, game heading 0 = true 180 deg)
  // showed 180 deg until resetAt ran at spawn. started=true used to
  // be set before the await, so animate() ran the HUD through the
  // whole load. It is set here instead, after the player model
  // loads: during the load the HUD holds the previous round's values
  // (a real flown heading), and the first live HUD update after the
  // spawn reads the spawned jet. The carrier reset and the spawn
  // below run after this, so the player is placed before the first
  // post-load frame steps it. (The enemy/traffic model loads later
  // still await with started=true, but the player is spawned by
  // then, so the HUD reads a live jet.)
  started = true;
  tiltInput.setAircraftKind(
    player.spec.vtol ? 'vtol' : 'fixed-wing',
  );

  // Carriers — activate the chosen count (issue 5), each on the
  // shared loop at its own slot (phase-offset ~1/3 lap). The primary
  // starts 1800 m east of Alpha, 600 m north of its centreline, so
  // it is 18° off the takeoff heading at round start — visible, but
  // never over the runway. Reset BEFORE the player spawns: a deck
  // spawn needs the carrier's current position / heading / cruise
  // velocity. Carriers beyond the count are deactivated (hidden,
  // deck disabled).
  const heliCount = clamp(Math.round(cfg.helicarriers ?? 1), 0, 3);
  heliCountActive = heliCount;
  carriers.forEach((c, i) => {
    if (i < heliCount) {
      const loop = carrierLoop(i);
      const wp0 = loop.waypoints[loop.startIndex]!;
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
  const spawnedOnDeck = cfg.spawn === 'carrier' && heliCount > 0;
  if (spawnedOnDeck) {
    // Deck spawn: DECK_SPAWN_AFT_M aft of the deck centre (the full
    // deck ahead for the roll), facing the bow at the carrier's
    // heading, inheriting the carrier's cruise velocity so the jet
    // rides the deck instead of sliding off the stern (the deck moves
    // at 40 m/s from the first update; a zero-velocity spawn would be
    // left behind). Works for all four jets — the F-35A's VTOL home is
    // the deck. The arresting-wire logic applies on landing exactly as
    // for a carrier trap.
    const fwd = new THREE.Vector3(
      Math.sin(carrier.yaw),
      0,
      Math.cos(carrier.yaw),
    );
    const sx = carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M;
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
    lastSpawnLabel = 'carrier';
  } else {
    // Runway spawn — Alpha / Bravo / Charlie (pass 12, worker D).
    // Gear feet on the box top (elev + 2.25, see respawnPlayer
    // above for why not elev + 2). The runway table is in
    // original-frame coords; startRound runs right after
    // worldGen.resetOrigin(), so the shift is 0 and the table
    // coords are scene coords here (the same reason the Alpha
    // branch used r0 directly).
    const idx = cfg.spawn === 'bravo' ? 1 : cfg.spawn === 'charlie' ? 2 : 0;
    const r = RUNWAY_TABLE[idx]!;
    player.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
    lastSpawnLabel = cfg.spawn;
  }
  // Pass 14 (worker D, item 1): a new round starts in the default
  // chase view, snapped to the jet — the old code reset NONE of the
  // camera state here (respawnPlayer did), so a round that ended
  // spectating restarted in free orbit at a stale zoom, and the
  // chase camera lerped in from wherever the previous round left
  // it: the v4.10 side-on, below-ground first frame after every
  // Apply & Restart. Reset the mode/distance/orbit to the chase
  // defaults (deck orbit yaw on a deck spawn, world-south on a
  // runway spawn — the same defaults respawnPlayer uses) and snap
  // the camera onto the chase want before the first frame renders.
  camMode = 0;
  pcInput.setCameraMode(0);
  camDist = 30;
  camOrbit = spawnedOnDeck
    ? { yaw: deckOrbitYaw(carrier.yaw), pitch: 0.32 }
    : { yaw: 0, pitch: 0.32 };
  snapCameraToChase(spawnedOnDeck ? carrier.deckTopY : null);
  // Pass 8 (item 1): a new round starts with neutral controls (see
  // resetFlightInputs — a stuck yoke / deflected joystick / tilt
  // offset from the menu or a previous round must not deflect the
  // fresh jet).
  resetFlightInputs();
  // Pass 8 (item 1): restart the spawn trace for this round.
  startSpawnTrace();
  player.gear = true;
  player.clearGearManual();
  $('gearBtn')?.classList.add('on');

  // Enemies: mix of MiG-29 and Su-34.
  for (let i = 0; i < cfg.enemies; i++) {
    const key = i % 2 === 0 ? 'mig29' : 'su34';
    const e = makeAircraft(key, false);
    try {
      await e.attachModel();
      modelStatus[key] = { state: 'ok' };
      fixCanopyMaterials(e.group);
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
      // 450-600 m out and 30-60 m up, dead-ahead. Forward is the
      // player's own heading, so this holds for a deck spawn too.
      // Pass 17 (Worker B): dead-ahead, not offset - the old offsets
      // (side 120/-150, up 120/180) put both enemies ~21 deg off the
      // nose, OUTSIDE the 15 deg lock cone from frame one, so the
      // designed head-on intercept could never lock (probe-pass17-lock
      // 1-enemy: maxLockT 0.00 s, 0 in-cone substeps in 240 s;
      // dead-ahead: lock 1.70 s, missile fired, enemy shot down).
      const fwd = player.fwd();
      const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
      const along = 450 + i * 150;
      const side = 0;
      const up = 30 + i * 30;
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
    // Pass 8 (item 2): enemies spawn FACING the player (head-on
    // intercept), not on a random heading. A random heading let the
    // first two fly AWAY at 150 m/s for ~18 s before the engage turn
    // brought them back — by then they were 2-5 km out and patrolling
    // 10-23 km away (probe-ai-live). Facing the player, they close at
    // 150 + player speed from the first second. Heading is
    // atan2(dx, dz) in the game's convention (fwd = sin/cos of hdg).
    const hdgToPlayer =
      (Math.atan2(player.pos.x - ex, player.pos.z - ez) * 180) / Math.PI;
    e.resetAt(ex, ey, ez, hdgToPlayer);
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
        fixCanopyMaterials(t.group);
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
      (cfg.spawn === 'carrier' ? 'deck spawn' : cfg.spawn === 'bravo' ? 'Bravo' : cfg.spawn === 'charlie' ? 'Charlie' : 'Alpha') +
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
  camMode = (camMode + 1) % 2;
  pcInput.setCameraMode(camMode);
  hud.cameraToast(camMode);
}

$('pauseBtn')?.addEventListener('click', togglePause);
$('respawnBtn')?.addEventListener('click', skipSpectate);
// Pass 14 (worker D, item 3): the spectate banner ("WENT DOWN — … ·
// watching the wreck") gets its own dismiss button — hiding the
// banner only, while spectating continues (the camera stays on the
// wreck; R / the Respawn button still open the picker).
$('spectateDismiss')?.addEventListener('click', () => setSpectateBanner(false));
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

// Footer version string (v4.12 · Vite + Rapier).
{
  const ver = document.querySelector('#startScreen .ver');
  if (ver) ver.textContent = VERSION_FOOTER;
}

// ---- time of day / sky / night glow ----
function updateSky(dt: number): void {
  // Pass 8 (item 7): 1 game minute per 4 real seconds (was 1 per
  // second) — a full day now takes ~96 min instead of 24, so a round
  // no longer runs from daylight to near-darkness in ~8 min.
  todH = (todH + dt / 240) % 24; // 1 game minute per 4 real seconds
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

// ---- camera (chase / free orbit) ----
/** Snap the camera to the chase position for the current spawn
 *  (pass 14, worker D, item 1): the first frame after a restart or
 *  respawn must already BE the chase view. The old code left
 *  camera.position wherever the previous life ended (kilometres
 *  away, or orbiting a wreck) and let the chase lerp converge onto
 *  it — the straight-line lerp runs through terrain (the v4.8
 *  below-ground half-black frame) and at 1 fps during a janky
 *  restart takes 4-9 s to converge. `deckTopY` is the deck the jet
 *  spawned on (null for a runway spawn): the snap uses the same
 *  ceiling as the chase branch, so a deck spawn never starts above
 *  the overhead sheet. Called from startRound and respawnPlayer
 *  right after the spawn, once camMode / camDist / camOrbit are
 *  reset to the chase defaults. */
function snapCameraToChase(deckTopY: number | null): void {
  if (!player) return;
  camera.position.copy(
    chaseCameraPosition(
      player.pos,
      player.fwd(),
      player.up(),
      camDist,
      heightfieldGroundHeight,
      deckTopY,
    ),
  );
  camera.lookAt(player.pos.clone());
}

/** Deck-top Y under the jet (pass 15, worker E, item 9): the deck
 *  ceiling for the chase / free-orbit / spectate branches must come
 *  from the carrier the jet is actually ON, not carriers[0].
 *  respawnPlayer respawns onto the first LIVE carrier, and a jet can
 *  land on any carrier — but updateCamera capped the camera to
 *  carriers[0].deckTopY, so with carriers[0] dead (wreck on the
 *  ground, deckTopY ~25 m) or at a different altitude than the
 *  jet's carrier, the chase want sat hundreds of metres below the
 *  jet: the v4.10 live front-on, below-deck view looking up at the
 *  carrier underside. deckProvider.deckHeightAt resolves to the
 *  carrier under the queried point (the same resolution respawnPlayer
 *  uses for the spawn), so the ceiling follows the jet's own deck.
 *  Returns null off a deck (or when no live deck is under the jet —
 *  a wrong deck hundreds of metres below is worse than no ceiling). */
function deckTopUnderJet(): number | null {
  if (!player || !player.onDeck) return null;
  const y = deckProvider.deckHeightAt(player.pos.x, player.pos.z);
  return y === -Infinity ? null : y;
}

function updateCamera(dt: number): void {
  if (!player) return;
  const p = player;
  const fwd = p.fwd();
  const up = p.up();
  // Spectate (damage-model §5): while the player's wreck is falling
  // (dying) or in the 2 s after impact, the camera stays with the
  // wreck. Pass 11 (wreck free orbit): the wreck view IS the free
  // orbit — drag orbits (camOrbit), wheel/pinch zoom (camDist), the
  // camera stays focused on the wreck. The old code returned the
  // fixed 42 m chase here before ever checking camMode, so camMode 1
  // never applied while spectating despite the comment claiming it
  // did; the chase below is now only the non-orbit fallback (e.g. if
  // the player C-cycles away from orbit mid-spectate). The chase
  // offset uses the wreck's VELOCITY direction (live finding 3), NOT
  // its tumbling body axes: the old code built the offset from
  // fwd/up (the wreck's quaternion), so as the wreck tumbled the
  // camera swung chaotically and the wreck left the frame. The
  // velocity direction is stable (the wreck falls along it), and the
  // camera looks AT the wreck's position, not 40 m ahead along a
  // tumbling axis. When the wreck is nearly still (on the ground
  // after impact) the velocity is ~0, so fall back to the body fwd.
  if (spectating) {
    if (camMode === 1) {
      const orbitPos = orbitCameraPositionDeckAware(
        p.pos,
        camOrbit,
        camDist,
        heightfieldGroundHeight,
        deckTopUnderJet(),
      );
      camera.position.copy(orbitPos);
      camera.lookAt(p.pos.clone());
      return;
    }
    const dist = 42;
    const velDir = p.vel.length() > 5
      ? p.vel.clone().normalize()
      : fwd.clone();
    const want = p.pos
      .clone()
      .addScaledVector(velDir, -dist)
      .addScaledVector(new THREE.Vector3(0, 1, 0), dist * 0.34);
    // Pass 8 (item 5): keep the sightline clear, like the chase camera
    // — the camera floor alone lets a ridge between the camera and the
    // wreck hide the wreck (the live "flat voids" view, where the
    // camera showed empty terrain). Sample the midpoint too.
    const mid = p.pos.clone().addScaledVector(velDir, -dist * 0.5);
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
    camera.lookAt(p.pos.clone());
    return;
  }
  if (camMode === 0) {
    // Pass 12 (carrier deck camera): on a deck, cap the chase under
    // the detail_0 overhead sheet (deckTop + 4.7 m) so the jet stays
    // visible. Off a deck deckTopY is null and the plain chase is returned.
    // Pass 14 (worker D): the want comes from chaseCameraPosition —
    // the same helper snapCameraToChase uses — so the lerp target and
    // the snap position can never drift apart. The floor rule (v1.1:
    // clear not just the ground under the camera but the terrain
    // between it and the jet — the analytic heightAt() can sit ~13 m
    // below the rendered surface at plateau edges) lives in the helper.
    const deckTopY = deckTopUnderJet();
    const want = chaseCameraPosition(
      p.pos,
      fwd,
      up,
      camDist,
      heightfieldGroundHeight,
      deckTopY,
    );
    camera.position.lerp(want, 1 - Math.pow(0.001, dt));
    if (shakeT > 0) {
      camera.position.x += (Math.random() - 0.5) * shakeT * 4;
      camera.position.y += (Math.random() - 0.5) * shakeT * 4;
    }
    // Pass 11 (chase framing): look AT the jet, not 40 m ahead of it
    // — the old look-ahead put the jet at the bottom edge of the
    // frame, so at short camDist it filled the frame and at long
    // camDist it was a speck at the edge. Centred, the framing is
    // consistent at every zoom.
    camera.lookAt(p.pos.clone());
  } else {
    // Free orbit (camMode 1 — v4.11: the cockpit camera is gone,
    // so the cycle is chase / free-orbit only). Pass 12: the live
    // inline orbit clamped only at the camera's own (x, z) and had
    // no deck awareness. Orbit the JET (never the carrier), with
    // the pass-8 floor (camera + midpoint + target + 2 m) and the
    // deck ceiling.
    const orbitPos = orbitCameraPositionDeckAware(
      p.pos,
      camOrbit,
      camDist,
      heightfieldGroundHeight,
      deckTopUnderJet(),
    );
    camera.position.copy(orbitPos);
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
      // Pass 15 (item 5): the floor is the wing's POTENTIAL lift at
      // a sane AoA (10 deg), not the lift it is making right now and
      // not a blanket "above stall speed the wing can fly it". The
      // old floor said liftFrac = 1 once ias >= stallSpeed — but
      // stall speed is where the wing CAN fly at CLmax, not where it
      // IS flying at level pitch: a jet level at 134 kt makes ~0.1 g
      // on the wing, the nozzle came all the way down, and the
      // transition crashed at -6,512 fpm after the vector reached 0
      // (v4.10 live; probe-vtol-live D4 reproduces it: 141.6 m lost
      // hands-off, vsi -53.8 m/s). The gload-only floor before that
      // had the opposite failure: it stalled the vector at ~73 deg
      // at 200+ kt while the jet descended unloaded (gload ~ 0),
      // even though the wing could fly it at that speed. The
      // potential lift is the middle: the lift the wing WOULD make
      // at 10 deg AoA at the current speed and altitude — if the
      // pilot rotates, the wing takes the load and the nozzle
      // follows; if the pilot stays level, the nozzle waits. The
      // actual gload is still the floor when the pilot IS pulling
      // (max of the two), so a flown AoA is credited immediately.
      const rho = 1.225 * Math.exp(-Math.max(p.pos.y, 0) / 8500);
      const q = 0.5 * rho * p.ias * p.ias;
      const sLift = p.spec.wingAreaEffective ?? p.spec.wingArea;
      const cl10 = clFromCurve(10, p.flaps / 30, p.spec.clCurve);
      const potential =
        (q * sLift * cl10) / (p.spec.mass * 9.81);
      const liftFrac = Math.max(0, Math.min(1, Math.max(potential, p.gload)));
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
    } else if (p.dying) {
      // Pass 17 (worker E, item 1 — the v4.10 frozen state): a DYING
      // jet whose spectate was already dismissed (R / Respawn once,
      // then Cancel / Watch-the-wreck dismissed the picker) has no
      // R path at all — spectating is false, no respawn timer is
      // running, and the stuck detector below excludes dying jets,
      // so R fell through to the camera reset forever while W stayed
      // dead (playerInput returns early for a dying jet, so the
      // throttle never rises). R opens the picker for a dying jet
      // outright: picking a target respawns at once (respawnPlayer
      // allows a dying jet), exactly as the spectate-skip path does.
      showRespawnPicker(false);
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
  // inside PCFlightInput), then tilt adds.
  // Pass 13 (worker B): an ACTIVE touch stick wins over the PC stick.
  // The old rule was "PC wins when nonzero" — but the PC stick is
  // nonzero for ~0.4 s after ANY arrow tap (the kbRoll return tail),
  // and a mouse yoke can sit a hair off zero, so a touch drag in
  // that window was silently ignored: the v4.10 live check saw a
  // left drag do nothing at all while the bank kept rising the
  // wrong way (measured in probe-roll section F: rollIn stayed
  // +0.36..+0.08 for ~0.45 s after a tap while the stick dragged
  // left). The stick under the pilot's finger is the live input;
  // a stale PC tail is not. Keyboard arrows still win the moment
  // the finger lifts (joy.active false).
  // Pass 14 (worker A): the keyboard stick slew runs in SIM time —
  // playerInput's substep dt is passed into getControls, so the
  // stick ramps at the same rate per sim second at any frame rate
  // (the wall-clock slew made a 100 ms tap bank 1.1 deg at 4-9 fps
  // and 6.5 deg at ~17 fps — probe-pass14 S1 vs probe-roll F2).
  pcInput.setAirspeedKt(p.ias * MS2KT);
  const pc = pcInput.getControls(dt);
  const tilt = tiltInput.getControls();
  let px: number;
  let rx: number;
  const pcPitch = pc.pitch ?? 0;
  const pcRoll = pc.roll ?? 0;
  if (joy.active || (pcPitch === 0 && pcRoll === 0)) {
    // Touch stick, direct (pass 12, worker D): drag UP (joy.y < 0)
    // = nose up (px < 0), so the touch stick agrees with the
    // keyboard (ArrowUp = nose up) and the mouse yoke (drag up =
    // nose up). The old sim-style px = -joy.y (drag down = nose up)
    // read as backwards live — the pilot dragged up expecting a
    // climb and got a dive, then dragged down and looped. Both axes
    // run through the deadzone and the speed scales (roll linear,
    // pitch squared — probe-roll section E), so a full drag at
    // 586 kt no longer yanks the jet to ~10 G in half a second.
    // When the stick is at rest this branch reads 0 through the
    // deadzone, so the PC-zero case needs no separate branch.
    const iasKt = p.ias * MS2KT;
    px = touchDeadzone(joy.y) * touchPitchScale(iasKt);
    rx = touchDeadzone(joy.x) * touchStickScale(iasKt);
  } else {
    px = pcPitch;
    rx = pcRoll;
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

  // Missile lock (G6) - WeaponSystem.updateLock (pass 17, Worker B):
  // hold the target inside the 15 deg cone, < 2500 m; a dying
  // (falling) jet is untargetable. Same 1.2 s threshold, same 2x
  // decay; the scan lives in the combat system so probes can call
  // the production code instead of a copy.
  const locked = weapons.updateLock(p, enemies, dt);
  if (locked && p.lockT > 1.2) beep(1320, 0.05, 'sine', 0.03);
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
      // Pass 8 (item 5): an instant crash sets no damage zones and no
      // fire, and the animate plume loop skips dead aircraft — so the
      // wreck showed no fire or smoke, only the 0.9 s explosion sprite
      // (the live "no fire/explosion/debris/scorch" finding). Attach a
      // fire + smoke plume to the wreck's group so it burns visibly
      // for the spectate window (6 s life; respawnPlayer clears
      // attached plumes on the next life).
      if (!a.crashedFromDying) {
        vfx.spawnSmoke(a.pos.clone(), 0xff7722, a.group);
        vfx.spawnSmoke(a.pos.clone(), 0x1a1a1a, a.group);
      }
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
        // Pass 8 (item 1): log the crash with the ACTUAL numbers
        // (reason + vertical speed + bank at the crash moment) so the
        // ?debug=1 panel shows why the jet went down instead of just
        // the reason string. Appended to the spawn trace too, so the
        // trace reads as a complete 3 s story ending in the death.
        const crashLine = `CRASH ${a.lastCrashReason ?? '-'} vs=${a.lastCrashVs.toFixed(2)}m/s bank=${a.lastCrashBankDeg.toFixed(1)}deg y=${a.pos.y.toFixed(1)}`;
        debugLog.push(crashLine);
        if (debugLog.length > 30) debugLog.shift();
        spawnTrace.push(crashLine);
        spawnTraceT = Infinity; // stop sampling this life
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
          // Pass 11 (wreck free orbit): the wreck view starts as the
          // free-orbit camera (drag orbits, wheel/pinch zooms), framed
          // at the default 30 m / 0.32 pitch. The mode the player had
          // is restored by skipSpectate.
          preSpectateCamMode = camMode;
          camMode = 1;
          pcInput.setCameraMode(1);
          camOrbit = { yaw: 0, pitch: 0.32 };
          camDist = 30;
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
  elapsed += rawDt;
  // Pass 8 (item 1): the floating-origin shift fires at the START of
  // the frame, not the end. At the end of the frame player.pos is
  // stale (pre-shift) until the next step's syncFromBody, so the
  // score, camera and HUD read a position up to 2x the shift away
  // for one frame (probe-spawn-shift caught this: a -109 m clearance
  // reading that was pure staleness). At the start of the frame
  // player.pos is fresh (synced in the previous frame's last step),
  // so the shift is computed from the true position and every read
  // after the substeps sees the shifted position.
  // Pass 11 (infinite terrain): capture the shift, stream terrain
  // around the player, and move the CAMERA with the shift. The camera
  // was never registered with the FloatingOrigin, so at every shift
  // the chase lerp left it ~2 km behind the jet for ~0.6 s — the
  // "glitch back" Nicholas felt every ~2 km of flight. The jet and
  // the terrain shift invisibly (original-frame jump 0.000 m over 21
  // shifts / 40 km); moving the camera with the world makes the shift
  // invisible for the chase view too (free-orbit positions the camera
  // absolutely, so it never glitched). The streamer update runs
  // right after the shift check, once per frame.
  const shift = worldGen.shiftOriginIfNeeded(player?.pos ?? new THREE.Vector3());
  if (shift) {
    camera.position.x -= shift.x;
    camera.position.z -= shift.z;
  }
  terrainStreamer.update(player?.pos ?? new THREE.Vector3());
  applyFailure(elapsed);
  updateSky(rawDt);
  const turb = cfg.weather.turbulence;
  // Pass 13 (Worker F — substep spiral breaker): cap the SIM time
  // advanced per frame at 0.10 s, i.e. at most 2 substeps of <= 0.05 s.
  // The v1.1 rule (sub = ceil(rawDt / 0.05), rawDt clamped to 0.25 s)
  // ran up to 5 substeps/frame at 4-6 fps, so once the renderer dipped
  // below 20 fps the physics bill grew 2-5x per frame and locked the
  // frame rate down — a spiral of death (Glenn Fiedler's term). The
  // probe measured the live-path substep at ~1 ms for 5 jets on the
  // probe VM, so the spiral is an amplifier, not the whole 4-6 fps
  // story (the rest is render-side — see the pass-13 report), but it
  // is real and it is free to remove: at 4 fps the physics bill falls
  // 4.85 -> 1.94 ms/frame on the probe VM (2.5x), at 5 fps 3.88 ->
  // 1.94 ms/frame. Above 10 fps nothing changes (rawDt <= 0.10 s, so
  // sub is identical to the old code). Below 10 fps the sim runs
  // slower than real time instead (0.4x at 4 fps) — slow motion, not
  // a locked frame rate — and recovers to full speed as soon as the
  // frame time drops under 0.10 s. The <= 0.05 s substep invariant
  // (physics stability) is preserved: dt = simDt / sub <= 0.05 s.
  // `elapsed` above still advances on rawDt (real time), so failure
  // timers and the clock keep real time; only the physics advances
  // less on a slow frame.
  const simDt = Math.min(rawDt, 0.1);
  const sub = Math.max(1, Math.ceil(simDt / 0.05));
  const dt = simDt / sub;
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
      // Pass 11 (wreck free orbit): see handleDeaths — the wreck view
      // starts as the free-orbit camera, framed at the default.
      preSpectateCamMode = camMode;
      camMode = 1;
      pcInput.setCameraMode(1);
      camOrbit = { yaw: 0, pitch: 0.32 };
      camDist = 30;
      const killer =
        player.lastHitBy?.isPlayer === false ? 'an enemy' : undefined;
      setSpectateBanner(true, player.dyingReason ?? undefined, killer);
    }
    prevPlayerDying = player.dying;
    // If the player respawned (alive again), spectate is over.
    if (player.alive && !player.dying && spectating && !respawnTimer) {
      spectating = false;
      camMode = preSpectateCamMode;
      pcInput.setCameraMode(camMode);
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
  // Pass 17 (worker E, item 1): the FROZEN state — a live jet that
  // does not move at 100% throttle (v4.10 live: F-35A hanging mid-air
  // at 78 kt, 1,138 ft, surf=0, G 0.0, carrier receding; and once on
  // an Alpha spawn at 0 kt). The beached gate cannot catch it: its
  // `vel` field is stale at the deck velocity (~40 m/s) or at 0 with
  // `grounded` still true from respawn, so `vel < 1` never fires and
  // R does nothing — the only in-game recovery is dead. The second
  // gate below measures MOVEMENT, not velocity: alive, not dying,
  // not on a surface, and the position moved < 0.1 m in the frame.
  // It deliberately does NOT read vel or grounded (both are stale in
  // exactly this state), and it does NOT use the AGL gate (a frozen
  // jet hangs hundreds of metres up). It also does NOT require the
  // throttle to be up: in the frozen state W never reaches thrTarget
  // — playerInput returns early while the jet is dying, and a
  // throttle that never rose reads 0 — so a throttle term made the
  // gate unreachable in exactly the state it guards (probe-pass17
  // S4b: with thrTarget 0 the gate never fired in 60 s). Movement
  // alone is the signal: a healthy jet at any throttle moves
  // > 0.1 m/frame from the second frame on, a VTOL hover is
  // excluded by vector >= 70, and a parked/braked jet is on a
  // surface. Any movement resets the timer, so the 2 s R gate needs
  // 2 continuous sim-seconds of not moving.
  if (player) {
    const agl = player.pos.y - heightfieldGroundHeight(player.pos.x, player.pos.z);
    const moved = prevWatchPos ? player.pos.distanceTo(prevWatchPos) : Infinity;
    if (prevWatchPos) prevWatchPos.copy(player.pos);
    else prevWatchPos = player.pos.clone();
    const unresponsive =
      player.alive &&
      !player.dying &&
      !player.onSurface &&
      player.vector < 70 &&
      moved < 0.1;
    const beached =
      (player.alive &&
        !player.dying &&
        !player.grounded &&
        !player.onDeck &&
        player.vel.length() < 1 &&
        agl < 10) ||
      unresponsive;
    if (beached) {
      // Pass 17 (worker E, item 1): accumulate the frame's TOTAL sim
      // time, not one substep dt. The detector runs once per frame
      // but the old code added `dt` (the substep dt, <= 0.05 s), so
      // at 4 fps stuckTime advanced at 0.2x real time and R needed
      // 40 frames / 10 real s (probe-pass17 S4c); simDt is the
      // frame's sim advance (<= 0.10 s), so R is available after
      // 20 frames / 5 real s at 4 fps.
      stuckTime += simDt;
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
  worldGen.update(rawDt, windVec);
  // Pass 8 (item 1): spawn trace sampling — one line per frame for the
  // first 3 s after a spawn (VSI, AoA, onSurface, grounded, bank,
  // pos.y, floorY). Sampled here, after the substeps, so the values
  // are the post-step state. The trace is shown in the ?debug=1 panel
  // (updateDebugPanel) and a crash appends its reason + vs/bank.
  if (DEBUG_ON && player && spawnTraceT < 3) {
    spawnTraceT += rawDt;
    const bankDeg =
      Math.abs(new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').z) *
      (180 / Math.PI);
    const floorY = Math.max(
      heightfieldGroundHeight(player.pos.x, player.pos.z),
      deckProvider.deckHeightAt(player.pos.x, player.pos.z),
    );
    spawnTrace.push(
      `t=${spawnTraceT.toFixed(2)} vsi=${player.vsi.toFixed(2)} aoa=${player.aoaDeg.toFixed(1)} surf=${player.onSurface ? 1 : 0} gnd=${player.grounded ? 1 : 0} bank=${bankDeg.toFixed(1)} y=${player.pos.y.toFixed(1)} floor=${floorY.toFixed(1)}`,
    );
    if (spawnTrace.length > 180) spawnTrace.shift();
  }
  // Score is kills / traps / VTOL landings only (pass 12, worker D): the survival +1/s tick was removed — it paid the pilot for merely flying (the v4.6.1 live check's 0→439 with 0 kills).
  if (shakeT > 0) shakeT = Math.max(0, shakeT - rawDt);
  updateCamera(rawDt);
  hudT += rawDt;
  if (hudT > 0.1) {
    hudT = 0;
    if (player) {
      // CCIP (bomb impact point) is computed before the HUD update so
      // the below-min-release-alt flag feeds the warn priority.
      // Pass 13 (worker C): the CCIP PIPPER itself is GONE — it was
      // the yellow/amber ⊕ crosshair at screen centre in the v4.7
      // live check (F-18 776 kt, F-15E 666 kt, in the chase view;
      // never on the F-22/F-35A, which carry no bombs). Nicholas
      // asked for the reticle to be gone from all jets. The
      // computation stays ONLY for the PULL UP warning; nothing
      // displays it any more.
      const ccip = weapons.computeCcip(
        player,
        heightfieldGroundHeight(player.pos.x, player.pos.z),
        camera,
      );
      // Carrier bearing/distance (pass 12, worker F): the v4.6.1 live
      // check's "carrier cannot be found" was a HUD gap — the player
      // had no bearing or distance to fly back on. Formula verified
      // in probe-deck-roll §F (rel 0.00°/85.0 m at deck spawn,
      // rel 180.00°/3000.0 m at 3 km east heading east).
      // Pass 13 (worker E): all three headings are converted to TRUE
      // (compass) headings for display (gameToTrueHdg: true =
      // 180 − game, north = −Z). The relative bearing is recomputed
      // in true space — its sign FLIPS vs the old mirrored value,
      // which is the fix: the old relDeg pointed the player the
      // wrong way round the compass. Internal headings (AI, spawn,
      // carrier yaw) are untouched; only these displayed numbers
      // convert.
      const cdx = carrier.pos.x - player.pos.x;
      const cdz = carrier.pos.z - player.pos.z;
      const carrierAbsDeg = gameToTrueHdg(((Math.atan2(cdx, cdz) * 180) / Math.PI + 360) % 360);
      const playerHdgDeg = gameToTrueHdg(((Math.atan2(player.fwd().x, player.fwd().z) * 180) / Math.PI + 360) % 360);
      const carrierRelDeg = ((carrierAbsDeg - playerHdgDeg + 540) % 360) - 180;
      const carrierHdgDeg = gameToTrueHdg(((carrier.yaw * 180) / Math.PI + 360) % 360);
      hud.update(player, {
        score,
        kills,
        traps,
        elapsed,
        lockTarget: player.lockTarget,
        belowMinReleaseAlt: ccip?.belowMinAlt ?? false,
        carrier: { absDeg: carrierAbsDeg, relDeg: carrierRelDeg, distM: Math.hypot(cdx, cdz), carrierHdgDeg, deckRelHdgDeg: player.onDeck ? ((playerHdgDeg - carrierHdgDeg + 540) % 360) - 180 : null },
      });
      // The automatic gear (pass 10, item 3) can move the gear without
      // a button press — keep the button's state in step with it.
      $('gearBtn')?.classList.toggle('on', player.gear);
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
  // Pass 14 (worker E): renderer.info readout — the live check's
  // instrument for the FPS-low-everywhere finding. renderer.info is
  // reset by each render, and this panel is built BEFORE that frame's
  // renderer.render call (animate: updateDebugPanel, then render), so
  // these are the LAST COMPLETED frame's submitted calls/tris.
  // programs is null until the first render populates it (three r160).
  const bufSize = renderer.getDrawingBufferSize(new THREE.Vector2());
  const renderInfoLine = hud.renderInfoLine({
    calls: renderer.info.render.calls,
    triangles: renderer.info.render.triangles,
    geometries: renderer.info.memory.geometries,
    textures: renderer.info.memory.textures,
    programs: Array.isArray(renderer.info.programs) ? renderer.info.programs.length : 0,
    pixelRatio: renderer.getPixelRatio(),
    bufferW: bufSize.x,
    bufferH: bufSize.y,
    antialias: RENDER_ANTIALIAS,
  });
  const lines = [
    `PANDORA ${VERSION_FOOTER} ?debug — FPS ${fpsEMA.toFixed(0)} · sim dt ${lastDt.toFixed(3)}s · rapier: on`,
    renderInfoLine,
    ...modelLines,
    `origin shift: ${worldGen.originShift.x.toFixed(0)}, ${worldGen.originShift.z.toFixed(0)} · gamepad: ${pcInput.gamepadName || 'none'} · tilt: ${tiltInput.isEnabled ? 'on' : 'off'}`,
    // Pass 14 (worker A): this line used to show ONLY the sensitivity
    // CONFIG (norm/INV + multiplier) — the v4.10 live check misread it
    // as a stale stick readout. It now also shows the LIVE stick:
    // the merged pitchIn/rollIn the controller actually sees, and
    // the touch stick's live position (joy active? x/y). If the jet
    // does not answer the stick, this line says which side of the
    // merge is dead: live 0.00 with joy (0.99, 0.00) = the merge
    // dropped it; live 0.96 with no response = the controller/regime.
    `stick: pitch ${cfg.invertPitch ? 'INV' : 'norm'} ${cfg.pitchSens.toFixed(2)}× · roll ${cfg.invertRoll ? 'INV' : 'norm'} ${cfg.rollSens.toFixed(2)}× · live p${(player?.pitchIn ?? 0).toFixed(2)} r${(player?.rollIn ?? 0).toFixed(2)} joy ${joy.active ? 'on' : 'off'} (${joy.x.toFixed(2)}, ${joy.y.toFixed(2)}) · spawn ${lastSpawnLabel} · enemies ${cfg.enemies === 0 ? 'Off' : cfg.enemies} · fail ${cfg.failure} / ${cfg.failWhen}`,
    // Pass 8 (item 3): per-carrier live readout — y vs ground vs
    // look-ahead ground, alive/dying, pod HP. If a CARRIER DOWN toast
    // fires, this says which carrier and what its clearance was.
    ...carriers.map((c, i) => {
      const g = heightfieldGroundHeight(c.pos.x, c.pos.z);
      const pods = c.pods.map((p) => `${p.hp}`).join('/');
      return `carrier ${i + 1} (slot ${i + 1}): y=${c.pos.y.toFixed(0)} gnd=${g.toFixed(0)} ahead2500=${c.lookAheadGround.toFixed(0)} clr=${(c.pos.y - g).toFixed(0)} ${c.active ? (c.alive ? (c.dying ? 'DYING' : 'flying') : 'DEAD') : 'inactive'} pods=[${pods}]`;
    }),
    // Pass 8 (item 1): the spawn trace — first 3 s after the spawn.
    spawnTrace.length
      ? '--- spawn trace (first 3 s) ---\n' + spawnTrace.slice(-12).join('\n')
      : '(no spawn trace yet)',
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
