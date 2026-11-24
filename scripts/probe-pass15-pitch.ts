// probe-pass15-pitch.ts — Pass 15 Worker A, item 14: PITCH
// OSCILLATORY AT 4 FPS.
//
// v4.9 live: a 6 s joystick-up hold gave +15,000 fpm, a 6 s down hold
// gave -9,444 fpm, and a gentle 28 px up hold in a dive did not arrest
// it at all. Pass 14 fixed the ROLL slew to sim dt
// (PCFlightInput.getControls(simDt)) — the PITCH slew uses the same
// `dt` variable (line 338: same slew() call, PITCH_SLEW 3/s), so the
// pitch path was NOT left on wall-clock dt. What is still unmeasured:
// pitch response (deg of pitch change per second of full and partial
// stick) across the FPS range, through the LIVE input path, per SIM
// second AND per WALL second. At 4 fps the pass-13 spiral breaker
// caps sim at 0.10 s/frame, so the sim runs 0.4x wall — per-wall
// response MUST be ~0.4x the 60 fps value by design; per-sim response
// is the physically meaningful consistency check. A per-sim
// inconsistency (or ringing/overshoot) at dt=0.05 substeps would be
// a real defect: the BET pitch mode is stiff at high q (large tail
// moment, CMQ=-5 damping) and explicit-Euler omega integration at
// 20 Hz can overshoot where 60 Hz does not.
//
// Sections (all through the live playerInput merge, real GLBs, live
// wind -3,0,0):
//   A: KB ArrowUp hold 6 s wall, 240 kt / 300 m — 4 fps vs 60 fps
//   B: KB ArrowDown hold 6 s wall, same
//   C: JOY full-up hold 6 s wall, same
//   D: JOY full-down hold 6 s wall, same
//   E: JOY gentle 28 px up hold in a -30 deg dive at 400 kt — does it
//      arrest the dive? (+ KB ArrowUp hold as control)
//   F: pitch-rate ringing: full KB pull at 400 kt, pitch-rate trace
//      per 0.1 s sim at 4 fps vs 60 fps (overshoot / sign flips)
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass15-pitch.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass15-pitch.mjs
//   && node /tmp/probe-pass15-pitch.mjs
//   (then rm -f /tmp/probe-pass15-pitch.mjs)

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
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class ImageStub {
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    crossOrigin: string | null = null;
    width = 1;
    height = 1;
    private _src = '';
    get src(): string {
      return this._src;
    }
    set src(v: string) {
      this._src = v;
      queueMicrotask(() => {
        if (this.onload) this.onload();
      });
    }
    addEventListener(type: string, fn: () => void): void {
      if (type === 'load') this.onload = fn;
      if (type === 'error') this.onerror = fn;
    }
    removeEventListener(): void {}
  };
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
import { PCFlightInput } from '../src/input/PCFlightInput';
import {
  MobileFlightInput,
  touchStickScale,
  touchPitchScale,
  touchDeadzone,
} from '../src/input/MobileFlightInput';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(-3, 0, 0);
const R2D = 180 / Math.PI;
const MS2KT = 1.94384;
const clamp01 = (v: number): number => Math.max(-1, Math.min(1, v));

let vnow = 0;
Object.defineProperty(globalThis, 'performance', {
  value: { now: () => vnow },
  configurable: true,
});

type WinHandler = (e: unknown) => void;
const winListeners = new Map<string, WinHandler[]>();
const g = globalThis as unknown as Record<string, unknown>;
g['window'] = globalThis;
g['addEventListener'] = (t: string, f: WinHandler): void => {
  const arr = winListeners.get(t) ?? [];
  arr.push(f);
  winListeners.set(t, arr);
};
g['removeEventListener'] = (t: string, f: WinHandler): void => {
  winListeners.set(t, (winListeners.get(t) ?? []).filter((x) => x !== f));
};
Object.defineProperty(globalThis, 'navigator', {
  value: { userAgent: 'NodeProbe/1.0', getGamepads: () => [] as unknown[] },
  configurable: true,
});
const fireKey = (type: 'keydown' | 'keyup', code: string): void => {
  for (const f of winListeners.get(type) ?? []) {
    f({ code, repeat: false, preventDefault: (): void => {} });
  }
};
const fakeCanvas = {
  addEventListener: (): void => {},
  removeEventListener: (): void => {},
} as unknown as HTMLCanvasElement;

interface FakeJoyEl {
  listeners: Map<string, Array<(e: unknown) => void>>;
  style: Record<string, string>;
  addEventListener(t: string, f: (e: unknown) => void): void;
  setPointerCapture(_id: number): void;
  getBoundingClientRect(): { left: number; top: number; width: number; height: number };
}
const makeJoyEl = (): FakeJoyEl => {
  const el: FakeJoyEl = {
    listeners: new Map(),
    style: {},
    addEventListener(t: string, f: (e: unknown) => void): void {
      const arr = el.listeners.get(t) ?? [];
      arr.push(f);
      el.listeners.set(t, arr);
    },
    setPointerCapture(_id: number): void {},
    getBoundingClientRect: () => ({ left: 14, top: 800, width: 148, height: 148 }),
  };
  return el;
};
const fireJoy = (
  el: FakeJoyEl,
  type: string,
  clientX: number,
  clientY: number,
  pointerId = 7,
): void => {
  for (const f of el.listeners.get(type) ?? []) {
    f({ clientX, clientY, pointerId });
  }
};
const joyCenter = (el: FakeJoyEl): { x: number; y: number } => {
  const r = el.getBoundingClientRect();
  return { x: r.left + r.width / 2, y: r.top + r.height / 2 };
};

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
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
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
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const alpha = RUNWAY_TABLE[0]!;

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

  const pitchOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * R2D;

  const DT_4FPS = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const DT_60FPS = [1 / 60];
  const PHYS_MS = 2;

  class LiveSession {
    readonly pc = new PCFlightInput(fakeCanvas);
    readonly tilt = new MobileFlightInput();
    readonly joyEl = makeJoyEl();
    readonly joy = { x: 0, y: 0, active: false, id: -1 };
    player: Aircraft;
    wall = 0;
    sim = 0;
    private pi = 0;
    lastPcPitch = 0;

    constructor(
      readonly key: string,
      readonly dtPattern: number[],
    ) {
      const spec = SPECS[key]!;
      const joy = this.joy;
      const joyEl = this.joyEl;
      const knob = { style: { transform: '' } };
      const joySet = (cx: number, cy: number): void => {
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
        knob.style.transform = `translate(${dx * 44}px, ${dy * 44}px)`;
      };
      joyEl.addEventListener('pointerdown', (e) => {
        const pe = e as { clientX: number; clientY: number; pointerId: number };
        joy.active = true;
        joy.id = pe.pointerId;
        joyEl.setPointerCapture(pe.pointerId);
        joySet(pe.clientX, pe.clientY);
      });
      joyEl.addEventListener('pointermove', (e) => {
        const pe = e as { clientX: number; clientY: number; pointerId: number };
        if (joy.active && pe.pointerId === joy.id) joySet(pe.clientX, pe.clientY);
      });
      const joyEnd = (e: unknown): void => {
        const pe = e as { pointerId: number };
        if (pe.pointerId === joy.id) {
          joy.active = false;
          joy.x = 0;
          joy.y = 0;
          knob.style.transform = '';
        }
      };
      joyEl.addEventListener('pointerup', joyEnd);
      joyEl.addEventListener('pointercancel', joyEnd);

      this.player = new Aircraft({
        scene,
        world,
        spec,
        isPlayer: true,
        modelKey: spec.modelKey,
      });
      this.player.terrainHeightAt = heightAt;
      origin.registerBody(this.player.body);
      for (const h of this.player.colliderHandles()) owners.set(h, this.player);
    }

    async attach(): Promise<void> {
      await this.player.attachModel();
    }

    spawnAirborne(speedKt: number, altM = 300): void {
      origin.reset();
      const y = alpha.elev + altM;
      this.player.resetAt(alpha.x, y, alpha.z, alpha.hdg);
      this.player.setAirspeed(speedKt / MS2KT);
      this.player.gear = false;
      this.player.thrTarget = 0.8;
      this.joy.active = false;
      this.joy.x = 0;
      this.joy.y = 0;
      this.pc.calibrate();
      this.tilt.calibrate();
    }

    playerInput(dt: number): void {
      const p = this.player;
      const ev = this.pc.consumeEvents();
      void ev;
      if (!p.alive || p.dying) {
        p.pitchIn = 0;
        p.rollIn = 0;
        return;
      }
      this.pc.setAirspeedKt(p.ias * MS2KT);
      const pc = this.pc.getControls(dt);
      const tilt = this.tilt.getControls();
      let px: number;
      let rx: number;
      const pcPitch = pc.pitch ?? 0;
      const pcRoll = pc.roll ?? 0;
      if (this.joy.active || (pcPitch === 0 && pcRoll === 0)) {
        const iasKt = p.ias * MS2KT;
        px = touchDeadzone(this.joy.y) * touchPitchScale(iasKt);
        rx = touchDeadzone(this.joy.x) * touchStickScale(iasKt);
      } else {
        px = pcPitch;
        rx = pcRoll;
      }
      px += tilt.pitch ?? 0;
      rx += tilt.roll ?? 0;
      p.pitchIn = clamp01(px);
      p.rollIn = clamp01(rx);
      p.yawIn = pc.yaw ?? 0;
      this.lastPcPitch = pcPitch;
    }

    frame(script?: (s: LiveSession) => void): void {
      const rawDt = Math.min(this.dtPattern[this.pi % this.dtPattern.length]!, 0.25);
      this.pi++;
      if (script) script(this);
      const shift = origin.shiftIfNeeded(this.player.pos);
      if (shift) origin.applyShift(shift);
      const simDt = Math.min(rawDt, 0.1);
      const sub = Math.max(1, Math.ceil(simDt / 0.05));
      const dt = simDt / sub;
      for (let k = 0; k < sub; k++) {
        vnow += PHYS_MS;
        this.playerInput(dt);
        world.timestep = dt;
        world.step(eventQueue);
        drain();
        this.player.step(dt, WIND, null);
        this.sim += dt;
      }
      const renderMs = Math.max(0, rawDt * 1000 - sub * PHYS_MS);
      vnow += renderMs;
      this.wall += rawDt;
    }

    dispose(): void {
      origin.unregisterBody(this.player.body);
      for (const h of this.player.colliderHandles()) owners.delete(h);
      this.player.dispose();
      this.pc.dispose();
      this.tilt.dispose();
    }
  }

  interface HoldResult {
    label: string;
    fps: string;
    pitch0: number;
    pitchAt1Sim: number;
    pitchAt2Sim: number;
    pitchAt3Sim: number;
    pitchAt6Wall: number;
    vsiAt6WallFpm: number;
    meanPitchIn: number;
    simAt6Wall: number;
    alive: boolean;
    pitchTraceSim: Array<[number, number]>;
  }

  /** Hold one input for 6 s wall through the live loop; sample pitch
   *  against SIM time (the physically meaningful clock) and wall. */
  const runHold = async (
    label: string,
    fpsLabel: string,
    dtPattern: number[],
    kind: 'kb-up' | 'kb-down' | 'joy-up' | 'joy-down',
    speedKt = 240,
    altM = 300,
  ): Promise<HoldResult> => {
    const s = new LiveSession('fa18', dtPattern);
    await s.attach();
    s.spawnAirborne(speedKt, altM);
    for (let i = 0; i < 12; i++) s.frame();
    const pitch0 = pitchOf(s.player);
    const t0wall = s.wall;
    const t0sim = s.sim;
    let started = false;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    const samples: Array<[number, number]> = [];
    let pitchInSum = 0;
    let pitchInN = 0;
    let pitchAt1: number | null = null;
    let pitchAt2: number | null = null;
    let pitchAt3: number | null = null;
    while (s.wall < t0wall + 6) {
      s.frame(() => {
        if (!started) {
          if (kind === 'kb-up') fireKey('keydown', 'ArrowUp');
          else if (kind === 'kb-down') fireKey('keydown', 'ArrowDown');
          else if (kind === 'joy-up') {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
          } else {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, r.top + r.height - 1, 7);
          }
          started = true;
        }
      });
      pitchInSum += s.player.pitchIn;
      pitchInN++;
      const simRel = s.sim - t0sim;
      samples.push([simRel, pitchOf(s.player)]);
      if (pitchAt1 === null && simRel >= 1) pitchAt1 = pitchOf(s.player);
      if (pitchAt2 === null && simRel >= 2) pitchAt2 = pitchOf(s.player);
      if (pitchAt3 === null && simRel >= 3) pitchAt3 = pitchOf(s.player);
      if (!s.player.alive) break;
    }
    if (kind === 'kb-up' || kind === 'kb-down') {
      fireKey('keyup', kind === 'kb-up' ? 'ArrowUp' : 'ArrowDown');
    } else {
      fireJoy(s.joyEl, 'pointerup', c.x, c.y, 7);
    }
    const res: HoldResult = {
      label,
      fps: fpsLabel,
      pitch0,
      pitchAt1Sim: pitchAt1 ?? pitchOf(s.player),
      pitchAt2Sim: pitchAt2 ?? pitchOf(s.player),
      pitchAt3Sim: pitchAt3 ?? pitchOf(s.player),
      pitchAt6Wall: pitchOf(s.player),
      vsiAt6WallFpm: s.player.vsi * 196.85,
      meanPitchIn: pitchInN ? pitchInSum / pitchInN : 0,
      simAt6Wall: s.sim - t0sim,
      alive: s.player.alive,
      pitchTraceSim: samples,
    };
    console.log(
      `  ${label} [${fpsLabel}]: pitch ${pitch0.toFixed(1)} -> 1s-sim ${res.pitchAt1Sim.toFixed(1)} / 2s-sim ${res.pitchAt2Sim.toFixed(1)} / 3s-sim ${res.pitchAt3Sim.toFixed(1)} / 6s-wall ${res.pitchAt6Wall.toFixed(1)} (sim elapsed ${res.simAt6Wall.toFixed(2)}s) vsi@6s-wall=${res.vsiAt6WallFpm.toFixed(0)}fpm meanPitchIn=${res.meanPitchIn.toFixed(2)} alive=${res.alive}`,
    );
    s.dispose();
    return res;
  };

  const holds: HoldResult[] = [];
  for (const [fpsLabel, pattern] of [
    ['4fps', DT_4FPS],
    ['60fps', DT_60FPS],
  ] as const) {
    holds.push(await runHold('KB ArrowUp hold 6s', fpsLabel, pattern, 'kb-up'));
    holds.push(await runHold('KB ArrowDown hold 6s', fpsLabel, pattern, 'kb-down'));
    holds.push(await runHold('JOY full-up hold 6s', fpsLabel, pattern, 'joy-up'));
    holds.push(await runHold('JOY full-down hold 6s', fpsLabel, pattern, 'joy-down'));
  }

  console.log('\n=== per-SIM-second pitch rates (deg pitch change per sim second) ===');
  for (const h of holds) {
    const rate1 = h.pitchAt1Sim - h.pitch0;
    const rate2 = (h.pitchAt2Sim - h.pitch0) / 2;
    const rate3 = (h.pitchAt3Sim - h.pitch0) / 3;
    console.log(
      `  ${h.label} [${h.fps}]: dPitch/sim-s: 0-1s ${rate1.toFixed(1)} / 0-2s ${rate2.toFixed(1)} / 0-3s ${rate3.toFixed(1)} (Euler x: negative = nose UP)`,
    );
  }
  // Consistency checks: per-sim 0-2s rates should agree across FPS
  // within ~25% (the slew + physics are sim-time quantities); per-wall
  // they differ by the 0.4x sim factor by design and are NOT asserted.
  const byLabel = (label: string): HoldResult[] => holds.filter((h) => h.label === label);
  for (const label of [
    'KB ArrowUp hold 6s',
    'KB ArrowDown hold 6s',
    'JOY full-up hold 6s',
    'JOY full-down hold 6s',
  ]) {
    const [a, b] = byLabel(label);
    const ra = (a!.pitchAt2Sim - a!.pitch0) / 2;
    const rb = (b!.pitchAt2Sim - b!.pitch0) / 2;
    const denom = Math.max(Math.abs(ra), Math.abs(rb), 1e-6);
    const rel = Math.abs(ra - rb) / denom;
    check(
      `${label}: pitch rate per SIM second consistent 4fps vs 60fps (0-2s)`,
      rel <= 0.25,
      `4fps ${ra.toFixed(1)} deg/sim-s vs 60fps ${rb.toFixed(1)} deg/sim-s (rel diff ${(rel * 100).toFixed(0)}%)`,
    );
  }

  // ============ E: gentle 28 px up hold in a dive ============
  // Setup: PD-drive the pitch to +30 deg (nose DOWN) at 400 kt /
  // 1000 m AGL with direct pitchIn at fixed dt (probe-pass14 E3
  // method), then hold the gentle joystick (28 px up = joy.y -0.378)
  // through the live 4 fps loop for 6 s wall. Control: KB ArrowUp.
  const runDiveArrest = async (
    label: string,
    kind: 'joy28' | 'kb-up',
    dtPattern: number[],
  ): Promise<void> => {
    const s = new LiveSession('fa18', dtPattern);
    await s.attach();
    s.spawnAirborne(400, 1000);
    // PD setup at fixed dt: drive Euler pitch to +30 (nose down).
    for (let i = 0; i < 600 && s.player.alive; i++) {
      const pNow = pitchOf(s.player);
      s.player.rollIn = 0;
      s.player.pitchIn = clamp01((30 - pNow) * 0.1);
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      s.player.step(dt, WIND, null);
      if (Math.abs(pNow - 30) < 1 && s.player.vsi < -40) break;
    }
    const pitch0 = pitchOf(s.player);
    const vsi0 = s.player.vsi * 196.85;
    const ias0 = s.player.ias * MS2KT;
    console.log(
      `  ${label}: dive setup pitch=${pitch0.toFixed(1)} vsi=${vsi0.toFixed(0)}fpm ias=${ias0.toFixed(0)}kt agl=${(s.player.pos.y - s.player.floorY).toFixed(0)}m`,
    );
    const t0wall = s.wall;
    const t0sim = s.sim;
    let started = false;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    // 28 px up on the 148 px stick: dy = -28/74 = -0.378.
    const joy28Y = c.y - 28;
    let minPitch = pitch0;
    let pitchInSum = 0;
    let pitchInN = 0;
    while (s.wall < t0wall + 6) {
      s.frame(() => {
        if (!started) {
          if (kind === 'kb-up') fireKey('keydown', 'ArrowUp');
          else {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, joy28Y, 7);
          }
          started = true;
        }
      });
      pitchInSum += s.player.pitchIn;
      pitchInN++;
      if (pitchOf(s.player) < minPitch) minPitch = pitchOf(s.player);
      if (!s.player.alive) break;
    }
    if (kind === 'kb-up') fireKey('keyup', 'ArrowUp');
    else fireJoy(s.joyEl, 'pointerup', c.x, joy28Y, 7);
    const pitchEnd = pitchOf(s.player);
    const vsiEnd = s.player.vsi * 196.85;
    console.log(
      `  ${label}: pitch ${pitch0.toFixed(1)} -> ${pitchEnd.toFixed(1)} (min ${minPitch.toFixed(1)}) vsi ${vsi0.toFixed(0)} -> ${vsiEnd.toFixed(0)}fpm meanPitchIn=${(pitchInN ? pitchInSum / pitchInN : 0).toFixed(2)} (joy.y=-0.378 -> touchPitchScale@${ias0.toFixed(0)}kt=${touchPitchScale(ias0).toFixed(2)}) sim elapsed=${(s.sim - t0sim).toFixed(2)}s alive=${s.player.alive}`,
    );
    check(
      `${label}: gentle up hold arrests the dive (pitch drops >= 10 deg toward level)`,
      pitchEnd <= pitch0 - 10,
      `pitch ${pitch0.toFixed(1)} -> ${pitchEnd.toFixed(1)} deg`,
    );
    s.dispose();
  };
  console.log('--- E: 28 px up hold in a -30 deg dive, 400 kt, 4 fps ---');
  await runDiveArrest('E1 JOY 28px up hold 6s wall (dive)', 'joy28', DT_4FPS);
  await runDiveArrest('E2 KB ArrowUp hold 6s wall (dive, control)', 'kb-up', DT_4FPS);

  // ============ F: pitch-rate ringing at 400 kt ============
  // Full KB pull from level flight at 400 kt; sample pitch rate
  // (deg/s of sim, 0.1 s sim bins) at 4 fps vs 60 fps. Ringing =
  // the rate changes sign or oscillates bin-to-bin while the stick
  // is held constant.
  const runRing = async (fpsLabel: string, dtPattern: number[]): Promise<void> => {
    const s = new LiveSession('fa18', dtPattern);
    await s.attach();
    s.spawnAirborne(400, 300);
    for (let i = 0; i < 12; i++) s.frame();
    const t0sim = s.sim;
    fireKey('keydown', 'ArrowUp');
    const bins: Array<[number, number]> = [];
    let lastPitch = pitchOf(s.player);
    let lastSim = s.sim;
    while (s.sim < t0sim + 2.0) {
      s.frame();
      if (s.sim - lastSim >= 0.1) {
        const pNow = pitchOf(s.player);
        bins.push([s.sim - t0sim, (pNow - lastPitch) / (s.sim - lastSim)]);
        lastPitch = pNow;
        lastSim = s.sim;
      }
      if (!s.player.alive) break;
    }
    fireKey('keyup', 'ArrowUp');
    const rates = bins.map((b) => b[1]);
    const signFlips = rates.slice(1).filter((r, i) => Math.sign(r) !== Math.sign(rates[i]!)).length;
    const maxRate = Math.min(...rates);
    const minRate = Math.max(...rates);
    console.log(
      `  F full KB pull @400kt [${fpsLabel}]: pitch-rate bins (deg/sim-s, - = nose up): ${rates.map((r) => r.toFixed(0)).join(' ')} | sign flips=${signFlips} range=[${maxRate.toFixed(0)}, ${minRate.toFixed(0)}] alive=${s.player.alive}`,
    );
    check(
      `F full KB pull @400kt [${fpsLabel}]: pitch rate does not ring (<= 2 sign flips in 2 s sim)`,
      signFlips <= 2,
      `sign flips=${signFlips}`,
    );
    s.dispose();
  };
  console.log('--- F: pitch-rate ringing, full KB pull @400kt ---');
  await runRing('4fps', DT_4FPS);
  await runRing('60fps', DT_60FPS);

  console.log(failures === 0 ? '\nALL PASS-15 PITCH CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
