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
import { FORD_GLB } from './assets/models/ford';
import { PandoraWorldGenerator } from './world/PandoraWorldGenerator';
import type { RunwayDef } from './world/PandoraWorldGenerator';
import { RunwayPlacer } from './world/RunwayPlacer';
import { HelicarrierController } from './entities/HelicarrierController';
import { WeaponSystem } from './combat/WeaponSystem';
import { CrashVFXManager } from './combat/CrashVFXManager';
import { FlightHUD } from './ui/FlightHUD';
import { SandboxMenu } from './ui/SandboxMenu';
import type { SandboxConfig } from './ui/SandboxMenu';
import { PCFlightInput } from './input/PCFlightInput';
import { MobileFlightInput } from './input/MobileFlightInput';

await RAPIER.init();

const VERSION_FOOTER = 'v3.0 · Vite + Rapier';
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
scene.fog = new THREE.Fog(0x9db8cc, 900, 5200);
const camera = new THREE.PerspectiveCamera(
  62,
  window.innerWidth / window.innerHeight,
  0.5,
  12000,
);
camera.position.set(0, 320, -420);
const sun = new THREE.DirectionalLight(0xfff2d8, 2.2);
scene.add(sun);
scene.add(sun.target);
const hemi = new THREE.HemisphereLight(0xbfd9ff, 0x2a4a2a, 0.9);
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
}
const runwayPlacer = new RunwayPlacer(scene);
{
  const placed = runwayPlacer.place(worldGen.runways);
  // Runway strips are world geometry — shift them with the world.
  for (const m of placed.meshes) floatingOrigin.registerObject(m);
}
runwayPlacer.setOriginShift(floatingOrigin.cumulativeShift);
const carrier = new HelicarrierController(scene, rapierWorld, {
  fordGlbUri: FORD_GLB,
  heightAt: (x, z) => worldGen.groundHeight(x, z),
  onPodDestroyed: (_index, worldPos) => {
    vfx.spawnExplosion(worldPos.clone(), 1);
    sfx('boom');
  },
  onCarrierDown: () => {
    hud.toast('CARRIER DOWN', 'The Gerald Ford is lost.');
  },
});
// Deck surface for the aircraft controller (adapts the carrier's API to
// the DeckProvider interface: deckHeightAt / velocity / yawRate / local).
const deckProvider: DeckProvider = {
  get alive() {
    return carrier.alive;
  },
  deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
  velocity: () => carrier.vel.clone(),
  yawRate: () => carrier.yawRate,
  local: (x, z) => {
    const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
    return v ? { x: v.x, z: v.z } : null;
  },
};
const groundHeight = (x: number, z: number): number =>
  worldGen.groundHeight(x, z);

// ---- combat / VFX ----
let shakeT = 0;
const vfx = new CrashVFXManager(scene, {
  groundHeight,
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
  onCarrierPodHit: (podIndex, damage) => carrier.damagePod(podIndex, damage),
  carrierPodPositions: () =>
    carrier.podObjects.map((p) =>
      p.getWorldPosition(new THREE.Vector3()),
    ),
  groundHeight,
  deckHeight: (x, z) => carrier.deckHeight(x, z),
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
    try {
      await carrier.build();
      carrier.registerWithFloatingOrigin(floatingOrigin);
      {
        const dc = carrier.deckColliderRef;
        if (dc) colliderOwners.set(dc.handle, 'deck');
      }
      modelStatus['ford'] = { state: 'ok' };
      await startRound(next);
    } catch (e) {
      modelStatus['ford'] = {
        state: 'FAILED',
        error: e instanceof Error ? e.message : String(e),
      };
      if (loadErr) loadErr.textContent = 'Load error: ' + (e instanceof Error ? e.message : String(e));
      if (startBtn) {
        startBtn.textContent = 'Take Off 🛫';
        startBtn.disabled = false;
      }
    }
  })();
});

// ---- game state ----
let cfg: SandboxConfig = menu.read();
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
});

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
brakeBtn?.addEventListener('pointerdown', () => {
  touchBrake = true;
  brakeBtn.classList.add('on');
});
window.addEventListener('pointerup', () => {
  firing = false;
  touchBrake = false;
  brakeBtn?.classList.remove('on');
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
  ac.terrainHeightAt = groundHeight;
  floatingOrigin.registerBody(ac.body);
  colliderOwners.set(ac.body.collider(0).handle, ac);
  return ac;
}

function addScore(n: number, kind: 'kill' | 'trap' | 'vtol'): void {
  score += n;
  if (kind === 'kill') kills++;
  if (kind === 'trap') traps++;
  if (kind === 'vtol') vtolLandings++;
}

function respawnPlayer(): void {
  if (!player || player.alive) return; // v1.1: Restart inside the 3 s window already revived the jet
  respawnTimer = null;
  const r = runwayPlacer.nearest(player.pos);
  player.resetAt(r.x, r.elev + 2, r.z, r.hdg);
  player.thrTarget = 0;
  pcInput.setThrottle(0);
  hud.syncThrottle(0);
  camMode = 0;
  pcInput.setCameraMode(0);
}

function clearAircraftList(list: Aircraft[]): void {
  for (const a of list) {
    const ai = aiControllers.get(a);
    if (ai) floatingOrigin.unregisterVector(ai.navTarget);
    floatingOrigin.unregisterBody(a.body);
    colliderOwners.delete(a.body.collider(0).handle);
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
  // Failure timing: a named scenario hits at 3 s; 'random' picks a random
  // scenario at a random 30–90 s (template segFailWhen).
  if (cfg.failure === 'random') {
    failScenario =
      FAIL_CHOICES[Math.floor(Math.random() * FAIL_CHOICES.length)]!;
    failAt = 30 + Math.random() * 60;
  } else {
    failScenario = cfg.failure;
    failAt = 3;
  }
  if (respawnTimer) {
    clearTimeout(respawnTimer);
    respawnTimer = null;
  } // v1.1: no stale respawn mid-round
  clearAircraftList(enemies);
  clearAircraftList(traffic);
  worldGen.resetOrigin();

  // Weather: fog preset by label; wind dir = direction wind blows toward.
  const wxFog: Record<string, [number, number]> = {
    clear: [900, 5200],
    cloud: [700, 3600],
    storm: [400, 1900],
    rain: [500, 2600],
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
      colliderOwners.delete(player.body.collider(0).handle);
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

  const r0: RunwayDef = worldGen.runways[0]!;
  player.resetAt(r0.x, r0.elev + 2, r0.z, r0.hdg);
  player.gear = true;
  $('gearBtn')?.classList.add('on');

  // Carrier — v1.1: starts ~1.1 km from Alpha, in the player's forward
  // view (v1 parked it 2.5 km away heading away — invisible). First
  // waypoint is east, with the takeoff direction.
  carrier.reset(
    new THREE.Vector3(-500, 350, -200),
    Math.atan2(1400 - -500, 0 - -200),
  );

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
      // 450–600 m out and 120–180 m up.
      const fwd = r0.dir;
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
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    const ai = new AircraftAIController(e, cfg.enemySkill, 'enemy', {
      groundHeight,
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
        t.thrTarget = 0.55;
      } else {
        t.resetAt(r.x, r.elev + 220, r.z, r.hdg);
        t.thrTarget = 0.7;
      }
      t.gear = false;
      t.grounded = false;
      const ai = new AircraftAIController(t, 'veteran', 'traffic', {
        groundHeight,
        originShift: floatingOrigin.cumulativeShift,
      });
      floatingOrigin.registerVector(ai.navTarget);
      ai.pickNav();
      aiControllers.set(t, ai);
      prevAlive.set(t, true);
      traffic.push(t);
    }
  }

  hud.toast('TAKE OFF', jetName + ' · ' + cfg.enemies + ' enemies · ' + weatherLabel(cfg) + ' weather');
}

function restartRound(): void {
  void startRound(menu.read());
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
$('restartBtn')?.addEventListener('click', () => {
  if (started) restartRound();
});
$('restartBtn2')?.addEventListener('click', restartRound);
$('resumeBtn')?.addEventListener('click', togglePause);
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

// Footer version string (v3.0 · Vite + Rapier).
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
  if (camMode === 0) {
    const want = p.pos
      .clone()
      .addScaledVector(fwd, -camDist)
      .addScaledVector(up, camDist * 0.34);
    // v1.1: keep the sightline clear — clear not just the ground under the
    // camera but the terrain between it and the jet.
    const mid = p.pos.clone().addScaledVector(fwd, -camDist * 0.5);
    const floor = Math.max(
      groundHeight(want.x, want.z) + 6,
      groundHeight(mid.x, mid.z) + 8,
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
    camera.position.set(cx, Math.max(cy, groundHeight(cx, cz) + 3), cz);
    camera.lookAt(p.pos);
  }
}

// ---- player input merge (template playerInput) ----
function playerInput(dt: number): void {
  const p = player;
  if (!p) return;
  if (!p.alive) {
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
    px = joy.y; // stick up = pitch up (px<0); stick right = bank right (rx<0)
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
  p.yawIn = 0;

  const ev = pcInput.consumeEvents();
  if (firing || ev.fireHeld) {
    weapons.fireGunAtTargets(p, enemies.concat(traffic), dt);
  }
  if (ev.missilePressed && p.lockT > 1.2 && p.lockTarget) {
    weapons.launchMissile(p, p.lockTarget);
  }
  if (ev.flarePressed) weapons.fireFlares(p);
  if (ev.gearPressed) $('gearBtn')?.dispatchEvent(new Event('click'));
  if (ev.hookPressed) $('hookBtn')?.dispatchEvent(new Event('click'));
  if (ev.cameraPressed) cycleCam();
  if (ev.cameraResetPressed) {
    camOrbit = { yaw: 0, pitch: 0.32 };
    camDist = 30;
  }
  if (ev.pausePressed) togglePause();
  if (ev.bombPressed) $('bombBtn')?.dispatchEvent(new Event('click'));
  p.brake = ev.brakeHeld || touchBrake;

  // NOTE: gunCd / flareCd are decayed inside Aircraft.step() — do not
  // decay them here too (the first draft did, halving both cooldowns).

  // VTOL pad credit (G5): F-35, vector 70 deg+, on a MARKED deck pad,
  // slow + gentle descent relative to the moving deck.
  if (p.onDeck && p.spec.vtol && p.vector >= 70 && !p.vtolCounted) {
    const rel = carrier.localPosition(p.pos);
    const onPad =
      rel &&
      VTOL_PADS.some(
        ([px2, pz2]) => Math.abs(rel.x - px2) < 18 && Math.abs(rel.z - pz2) < 18,
      );
    if (onPad) {
      const relV = p.vel.clone().sub(carrier.vel).length();
      if (relV < 6 && p.vsi > -4) {
        p.vtolCounted = true;
        addScore(75, 'vtol');
        hud.toast('VTOL LANDING', '+75 — lift-fan down on the pad.');
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
  let best: Aircraft | null = null;
  let bd = 2500;
  for (const e of enemies) {
    if (!e.alive) continue;
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
      vfx.spawnExplosion(a.pos.clone(), 1);
      sfx('boom');
      vfx.scorchDecal(a.pos.clone());
      if (a.isPlayer) {
        hud.toast('YOU WENT DOWN', 'Respawning at the nearest runway.');
        respawnTimer = setTimeout(respawnPlayer, 3000);
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
    const ctrl = ai.update(dt, player, carrier, elapsed);
    if (ai.fireGun) weapons.aiGunShot(ac, player, ai.skill.err);
    if (ai.fireMissile) weapons.launchMissile(ac, player);
    ac.step(dt, ctrl, windVec, groundHeight, turbAcc);
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
    // Carrier moves first so the kinematic deck velocity is live for
    // this step; then Rapier integrates, contacts are dispatched into
    // notifyContact(), and the aero steps read fresh body state.
    carrier.update(dt);
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
  worldGen.shiftOriginIfNeeded(player?.pos ?? new THREE.Vector3());
  worldGen.update(rawDt, windVec);
  // Survival score: +1 per second alive.
  if (player?.alive && Math.floor(elapsed) !== Math.floor(prevElapsed)) {
    score += 1;
  }
  if (shakeT > 0) shakeT = Math.max(0, shakeT - rawDt);
  updateCamera(rawDt);
  hudT += rawDt;
  if (hudT > 0.1) {
    hudT = 0;
    if (player) {
      hud.update(player, {
        score,
        kills,
        traps,
        elapsed,
        lockTarget: player.lockTarget,
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
        carrier: { alive: carrier.alive, pos: carrier.pos, yaw: carrier.yaw },
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
    ['ford', 'Ford'],
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
