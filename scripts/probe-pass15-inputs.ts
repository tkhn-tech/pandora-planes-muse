// probe-pass14-inputs.ts — Pass 14 Worker A: INPUTS DEAD LIVE.
//
// The v4.8 live check (24 Nov 10:26-10:54 SGT, footer v4.8, 5 rounds,
// all from Alpha) found pitch/roll inputs effectively DEAD live:
//   * one Right Arrow tap at 223 kt (F-18) and at 524 kt (F-35A): Bank 0 deg
//   * on-screen joystick held full-right 33 s (F-18, ~210-270 kt):
//     Bank stayed 0 deg, heading moved only 109 -> 112 deg
//   * pitch locked at ~28 deg through two 24-45 s full-down joystick
//     holds and ArrowDown taps; pitch only changed when the jet stalled
//     at 21 deg AoA / ~210 kt and dropped its nose.
//
// probe-roll (sections D/F) passes 68/68 on this code, so it does NOT
// reproduce the failure. The differences it misses — and this probe
// replicates exactly — are:
//   1. the REAL takeoff: Alpha runway spawn, gear down, rotate at
//      Vr with a pull and climb out — NOT a trimmed 300 m airborne
//      spawn. (Pass 14 coordinator: the original hands-off
//      self-liftoff premise was v4.8 behaviour; pass 14 item 4 fixed
//      auto-liftoff, so the helper now pulls like a pilot must.)
//   2. live frame times: 4-9 fps (rawDt 0.11-0.25 s), NOT the ~17 fps
//      average of probe-roll's DT_PATTERN;
//   3. the pass-13 substep spiral breaker: simDt capped at 0.10 s, so
//      at 4 fps the sim runs 0.4x real time (probe-roll advances full
//      sim time per frame);
//   4. wall-clock input polling: PCFlightInput.getControls() derives its
//      own dt from performance.now(), which advances by the RENDER time
//      between frames and only ~2 ms between substeps in one frame;
//   5. the real main.ts merge VERBATIM (incl. pitch through PC + touch,
//      consumeEvents-before-getControls ordering, setAirspeedKt per
//      substep), the real DOM joystick wiring (pointer events through
//      a fake #joy element with the exact joySet math), real GLB
//      attachModel, and FloatingOrigin shifts.
//
// Every scenario records BOTH sides of the fault line: the stick the
// input chain produces (pc.pitch/pc.roll, merged pitchIn/rollIn) AND
// the bank/pitch the jet actually reaches. If the stick is ~0, the
// input chain is dead; if the stick is live but the jet does not move,
// the fault is downstream (controller/regime).
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass14-inputs.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass14-inputs.mjs
//   && node /tmp/probe-pass14-inputs.mjs
//   (then rm -f /tmp/probe-pass14-inputs.mjs)

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (Same stubs as probe-carrier-deck.ts /
/// probe-live-round.ts.)
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
// THREE's ImageLoader uses the global Image constructor, NOT
// document.createElementNS — without this, every GLB with an embedded
// texture fails with "Image is not defined" and attachModel falls back
// to the procedural stand-in (tris=218). A minimal Image that fires
// onload when src is set.
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

let WIND = new THREE.Vector3(-3, 0, 0); // LIVE wind: windDir 270, 3 m/s -> (-3,0,0) (pass14 probe used +3, wrong sign)
const R2D = 180 / Math.PI;
const MS2KT = 1.94384;
const clamp01 = (v: number): number => Math.max(-1, Math.min(1, v));

// --- virtual wall clock: performance.now() ---
// The browser's wall clock advances by the RENDER time between frames
// and by ~2 ms of physics work between substeps in one frame. vnow is
// advanced by the loop (see LiveLoop.frame), never by the physics.
let vnow = 0;
Object.defineProperty(globalThis, 'performance', {
  value: { now: () => vnow },
  configurable: true,
});

// --- window event stub (PCFlightInput + MobileFlightInput listeners) ---
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

// --- fake #joy DOM element + main.ts wiring, VERBATIM ---
// main.ts ~648-687: joy object, joySet (getBoundingClientRect math),
// pointerdown (setPointerCapture) / pointermove / pointerup+cancel.
// The probe drives it with synthetic PointerEvents at real client
// coordinates, exactly as a browser drag would.
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
    // 148x148 fixed stick, bottom-left (index.html #joy).
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

  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
    void ac;
  };

  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * R2D;
  const pitchOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * R2D;
  const hdgOf = (ac: Aircraft): number =>
    ((Math.atan2(ac.fwd().x, ac.fwd().z) * R2D) % 360 + 360) % 360;

  // Live frame times: 4-9 fps. Pattern mean ~0.17 s (~6 fps).
  // Pass 15: the 4fps pattern is live; DT_PATTERN_60FPS is the control.
  const DT_PATTERN_4FPS = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const DT_PATTERN_60FPS = [0.0167, 0.0167, 0.0167, 0.0167, 0.0167];
  // Physics cost between two getControls() calls inside one frame.
  const PHYS_MS = 2;

  interface TraceRow {
    wall: number;
    sim: number;
    bank: number;
    pitch: number;
    hdg: number;
    iasKt: number;
    aoa: number;
    agl: number;
    pcPitch: number;
    pcRoll: number;
    pitchIn: number;
    rollIn: number;
    joyX: number;
    joyY: number;
    rollInt: number; // integral of rollIn over this frame's sim dt
    joyActive: boolean;
    onSurface: boolean;
    stalled: boolean;
  }

  /** One live session: fresh aircraft, real GLB, Alpha spawn, the
   *  main.ts animate() loop (frame start shift -> substeps ->
   *  render), and the main.ts playerInput merge verbatim. */
  class LiveSession {
    readonly pc = new PCFlightInput(fakeCanvas);
    readonly tilt = new MobileFlightInput();
    readonly joyEl = makeJoyEl();
    readonly joy = { x: 0, y: 0, active: false, id: -1 };
    player: Aircraft;
    wall = 0; // wall seconds (== vnow/1000 at frame starts)
    sim = 0;
    trace: TraceRow[] = [];
    private pi = 0;
    private renderMs = 0; // render time of the previous frame (ms)
    dtPattern: number[] = DT_PATTERN_4FPS;

    constructor(
      readonly key: string,
      useBet = true,
    ) {
      const spec = SPECS[key]!;
      // --- main.ts joy wiring, verbatim (lines ~648-687) ---
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
        useBet,
      });
      this.player.terrainHeightAt = heightAt;
      origin.registerBody(this.player.body);
      for (const h of this.player.colliderHandles()) owners.set(h, this.player);
    }

    async attach(): Promise<void> {
      await this.player.attachModel();
    }

    spawnAlpha(): void {
      // main.ts startRound calls worldGen.resetOrigin(): every spawn
      // starts from a zero shift, or the runway coords (original
      // frame) land in shifted terrain.
      origin.reset();
      this.player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      this.player.gear = true;
      // resetFlightInputs (main.ts): neutral controls on spawn.
      this.joy.active = false;
      this.joy.x = 0;
      this.joy.y = 0;
      this.pc.calibrate();
      this.tilt.calibrate();
    }

    spawnAirborne(speedKt: number, altM = 300): void {
      origin.reset();
      // altM is height ABOVE the Alpha terrain, not absolute y: the
      // runway sits at elev ~748 m, so an absolute y=300 spawns the
      // jet inside the mountain.
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

    /** main.ts playerInput merge, VERBATIM (minus VTOL/fire/weapons
     *  tails, which do not touch the stick). The alive/dying guard,
     *  consumeEvents-first ordering, setAirspeedKt per substep and
     *  the touch-priority branch are all exactly as in main.ts. */
    playerInput(_dt: number): void {
      const p = this.player;
      const ev = this.pc.consumeEvents();
      void ev;
      if (!p.alive || p.dying) {
        p.pitchIn = 0;
        p.rollIn = 0;
        return;
      }
      this.pc.setAirspeedKt(p.ias * MS2KT);
      const pc = this.pc.getControls(_dt);
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
      // Record the frame's stick values for the fault-line split.
      this.lastPcPitch = pcPitch;
      this.lastPcRoll = pcRoll;
      this.framePeakPcPitch = Math.max(this.framePeakPcPitch, Math.abs(pcPitch));
      this.framePeakPcRoll = Math.max(this.framePeakPcRoll, Math.abs(pcRoll));
      this.framePeakPitchIn = Math.max(this.framePeakPitchIn, Math.abs(p.pitchIn));
      this.framePeakRollIn = Math.max(this.framePeakRollIn, Math.abs(p.rollIn));
      this.frameRollInt += p.rollIn * _dt;
    }
    lastPcPitch = 0;
    lastPcRoll = 0;
    // Pass 15: per-FRAME peak stick values. record() runs once per
    // frame, after all substeps, so the last substep's value hid a
    // tap that slewed up and back down inside one frame (the pitch
    // tap read peakPitchIn 0.00 at frame ends while the physics saw
    // 0.15 for one substep). playerInput accumulates the peaks.
    framePeakPcPitch = 0;
    framePeakPcRoll = 0;
    framePeakPitchIn = 0;
    framePeakRollIn = 0;
    frameRollInt = 0; // integral of rollIn over this frame's sim dt

    /** One live frame (main.ts animate): shift at frame start, then
     *  substeps (simDt capped at 0.10 s — the pass-13 spiral breaker),
     *  then the render wall-time. script() runs at frame start and
     *  may fire key/joy events keyed on this.wall. */
    frame(script?: (s: LiveSession) => void): number {
      const rawDt = Math.min(this.dtPattern[this.pi % this.dtPattern.length]!, 0.25);
      this.pi++;
      if (script) script(this);
      // Frame-start origin shift (main.ts: shift before substeps).
      const shift = origin.shiftIfNeeded(this.player.pos);
      if (shift) origin.applyShift(shift);
      this.framePeakPcPitch = 0;
      this.framePeakPcRoll = 0;
      this.framePeakPitchIn = 0;
      this.framePeakRollIn = 0;
      this.frameRollInt = 0;
      const simDt = Math.min(rawDt, 0.1);
      const sub = Math.max(1, Math.ceil(simDt / 0.05));
      const dt = simDt / sub;
      for (let k = 0; k < sub; k++) {
        vnow += PHYS_MS; // physics work since the last poll
        this.playerInput(dt);
        world.timestep = dt;
        world.step(eventQueue);
        drain(this.player);
        this.player.step(dt, WIND, null);
        this.sim += dt;
      }
      // Render wall-time: the rest of the frame.
      const renderMs = Math.max(0, rawDt * 1000 - sub * PHYS_MS);
      vnow += renderMs;
      this.wall += rawDt;
      this.renderMs = renderMs;
      return rawDt;
    }

    record(): void {
      const p = this.player;
      this.trace.push({
        wall: this.wall,
        sim: this.sim,
        bank: bankOf(p),
        pitch: pitchOf(p),
        hdg: hdgOf(p),
        iasKt: p.ias * MS2KT,
        aoa: p.aoaDeg,
        agl: p.pos.y - p.floorY,
        pcPitch: this.framePeakPcPitch,
        pcRoll: this.framePeakPcRoll,
        pitchIn: this.framePeakPitchIn,
        rollIn: this.framePeakRollIn,
        rollInt: this.frameRollInt,
        joyX: this.joy.x,
        joyY: this.joy.y,
        joyActive: this.joy.active,
        onSurface: p.onSurface,
        stalled: p.stalled,
      });
    }

    dispose(): void {
      origin.unregisterBody(this.player.body);
      for (const h of this.player.colliderHandles()) owners.delete(h);
      this.player.dispose();
      this.pc.dispose();
      this.tilt.dispose();
    }
  }

  const atWall = (tr: TraceRow[], t: number): TraceRow =>
    tr.reduce((a, b) => (Math.abs(b.wall - t) < Math.abs(a.wall - t) ? b : a));
  const peakBank = (tr: TraceRow[]): number =>
    tr.reduce((m, r) => Math.max(m, Math.abs(r.bank)), 0);
  const maxAbs = (tr: TraceRow[], f: (r: TraceRow) => number): number =>
    tr.reduce((m, r) => Math.max(m, Math.abs(f(r))), 0);

  /** Hands-off takeoff exactly like the live rounds: Alpha, gear
   *  down, throttle via the slider (thrTarget), ROTATE with a pull
   *  at Vr and release once airborne and climbing.
   *  Returns when airborne in the speed band, or null on timeout /
   *  crash.
   *  Pass 14 coordinator: the original helper was hands-off with NO
   *  pull because v4.8 auto-lifted off at ~190-240 kt. Pass 14 item 4
   *  (worker C) fixed auto-liftoff by design — a jet now needs a
   *  rotation input, and hands-off it skims at ~2 m AGL and flies
   *  into rising terrain at ~15 s sim. So the no-pull helper left
   *  S5/S7 measuring a doomed skim (the jet died mid-hold), not the
   *  input chain. The helper now pulls at Vr (140 kt, fa18) like a
   *  pilot must (worker C §C: rotation lifts off inside the runway
   *  and climbs 9,639-14,835 fpm); the sections still test the input
   *  chain in the post-takeoff regime, as intended. */
  const takeoff = async (
    key: string,
    thr: number,
    bandLo: number,
    bandHi: number,
    maxWall = 60,
  ): Promise<LiveSession | null> => {
    const s = new LiveSession(key);
    await s.attach();
    console.log(`  [${key}] model: ${s.player.modelStatus} tris=${s.player.modelTris} err=${s.player.modelError ?? 'none'}`);
    s.spawnAlpha();
    s.player.thrTarget = thr;
    let liftoff: TraceRow | null = null;
    let pulled = false;
    let releasedPull = false;
    while (s.wall < maxWall) {
      s.frame((_ss) => {
        const iasKt = s.player.ias * MS2KT;
        if (!pulled && iasKt >= 140) {
          fireKey('keydown', 'ArrowUp');
          pulled = true;
        } else if (
          pulled &&
          !releasedPull &&
          !s.player.onSurface &&
          s.player.pos.y - s.player.floorY > 30
        ) {
          fireKey('keyup', 'ArrowUp');
          releasedPull = true;
        }
      });
      s.record();
      const p = s.player;
      if (!p.alive) {
        console.log(
          `  [${key}] TAKEOFF: CRASHED at wall=${s.wall.toFixed(1)}s reason=${p.lastCrashReason ?? '-'} ias=${(p.ias * MS2KT).toFixed(0)}kt y=${p.pos.y.toFixed(0)}`,
        );
        s.dispose();
        return null;
      }
      if (!liftoff && !p.onSurface && !p.grounded) {
        liftoff = s.trace[s.trace.length - 1]!;
        console.log(
          `  [${key}] liftoff: wall=${liftoff.wall.toFixed(1)}s sim=${liftoff.sim.toFixed(1)}s ias=${liftoff.iasKt.toFixed(0)}kt pitch=${liftoff.pitch.toFixed(1)} aoa=${liftoff.aoa.toFixed(1)} agl=${liftoff.agl.toFixed(1)}m`,
        );
      }
      if (liftoff && !p.onSurface && p.ias * MS2KT >= bandLo && p.ias * MS2KT <= bandHi) {
        console.log(
          `  [${key}] in band ${bandLo}-${bandHi}kt: wall=${s.wall.toFixed(1)}s ias=${(p.ias * MS2KT).toFixed(0)}kt pitch=${pitchOf(p).toFixed(1)} aoa=${p.aoaDeg.toFixed(1)} agl=${(p.pos.y - p.floorY).toFixed(0)}m bank=${bankOf(p).toFixed(1)}`,
        );
        return s;
      }
    }
    console.log(`  [${key}] TAKEOFF: timed out (wall=${s.wall.toFixed(1)}s ias=${(s.player.ias * MS2KT).toFixed(0)}kt onSurface=${s.player.onSurface})`);
    s.dispose();
    return null;
  };

  // ============ PASS 15, ITEM 2: keyboard taps through the REAL input path ============
  // A real 100 ms tap at 4 fps: keydown AND keyup both land BETWEEN
  // frames (frame period ~250 ms), so the next frame's FIRST substep
  // sees the latched `pressed` key and the SECOND substep already sees
  // it released (PCFlightInput.getControls deletes the latch after one
  // call). Pass 14's probe fired keydown in one frame's script and
  // keyup in the NEXT frame's script, so the key was held across a
  // whole frame's substeps (kbRoll slewed 0.4 + 0.4 = 0.8) — that is a
  // ~170 ms straddling tap, NOT a 100 ms in-frame tap. This section
  // measures BOTH phasings, at 4 fps and at 60 fps, at 223/524/776 kt.
  const tapBank = async (
    label: string,
    key: string,
    kt: number,
    phasing: 'in-frame' | 'straddle',
    dtPat: number[],
    code: 'ArrowRight' | 'ArrowLeft' | 'ArrowUp' | 'ArrowDown' = 'ArrowRight',
  ): Promise<{ peakBank: number; peakRollIn: number; peakPcRoll: number; peakPitchIn: number; stickInt: number; bankAt2: number }> => {
    const s = new LiveSession('fa18');
    s.dtPattern = dtPat;
    await s.attach();
    s.spawnAirborne(kt, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    const sim0 = s.sim;
    const bank0 = bankOf(s.player);
    let firedDown = false;
    let firedUp = false;
    let peakRollIn = 0;
    let peakPcRoll = 0;
    let peakPitchIn = 0;
    let stickInt = 0; // integral of rollIn over sim seconds (stick-seconds)
    const testTrace: TraceRow[] = [];
    while (s.wall < t0 + 2) {
      s.frame((ss) => {
        if (phasing === 'in-frame') {
          if (!firedDown) {
            // Both events land between frames: down then up in the
            // same frame-start script, exactly as the browser delivers
            // a 100 ms tap inside a 250 ms frame interval.
            fireKey('keydown', code);
            fireKey('keyup', code);
            firedDown = true;
            firedUp = true;
          }
        } else {
          if (!firedDown) { fireKey('keydown', code); firedDown = true; }
          else if (!firedUp && ss.wall - t0 >= 0.1) { fireKey('keyup', code); firedUp = true; }
        }
      });
      s.record();
      const r = s.trace[s.trace.length - 1]!;
      testTrace.push(r);
      peakRollIn = Math.max(peakRollIn, Math.abs(r.rollIn));
      peakPcRoll = Math.max(peakPcRoll, Math.abs(r.pcRoll));
      peakPitchIn = Math.max(peakPitchIn, Math.abs(r.pitchIn));
      stickInt += r.rollInt;
      if (!s.player.alive) break;
    }
    const peak = peakBank(testTrace);
    const signedPeak = testTrace.reduce((m, r) => (Math.abs(r.bank - bank0) > Math.abs(m) ? r.bank - bank0 : m), 0);
    const at2 = atWall(testTrace, t0 + 2.0);
    console.log(
      `  ${label}: peakRollIn=${peakRollIn.toFixed(2)} peakPcRoll=${peakPcRoll.toFixed(2)} stickInt=${stickInt.toFixed(3)} stick-s peakBank=${peak.toFixed(1)} (signed ${signedPeak >= 0 ? '+' : ''}${signedPeak.toFixed(1)}) bank@+2s=${(at2.bank - bank0).toFixed(1)} bank0=${bank0.toFixed(1)} alive=${s.player.alive}`,
    );
    s.dispose();
    return { peakBank: peak, peakRollIn, peakPcRoll, peakPitchIn, stickInt, bankAt2: at2.bank - bank0 };
  };

  const DT4 = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const DT60 = [0.0167, 0.0167, 0.0167, 0.0167, 0.0167];

  console.log('--- T1: Right Arrow 100ms tap, IN-FRAME phasing (the real 4fps tap) ---');
  const t1a = await tapBank('T1a fa18 223kt 4fps in-frame', 'fa18', 223, 'in-frame', DT4);
  const t1b = await tapBank('T1b fa18 524kt 4fps in-frame', 'fa18', 524, 'in-frame', DT4);
  const t1c = await tapBank('T1c fa18 776kt 4fps in-frame', 'fa18', 776, 'in-frame', DT4);
  check('T1 in-frame tap @223kt 4fps: peak bank 5-15 deg RIGHT', t1a.peakBank >= 5 && t1a.peakBank <= 15, `peak=${t1a.peakBank.toFixed(1)} bank@2s=${t1a.bankAt2.toFixed(1)} (auto-level returns it — the tap is the peak) peakRollIn=${t1a.peakRollIn.toFixed(2)}`);
  check('T1 in-frame tap @524kt 4fps: peak bank 5-15 deg RIGHT', t1b.peakBank >= 5 && t1b.peakBank <= 15, `peak=${t1b.peakBank.toFixed(1)} bank@2s=${t1b.bankAt2.toFixed(1)} peakRollIn=${t1b.peakRollIn.toFixed(2)}`);
  check('T1 in-frame tap @776kt 4fps: peak bank 5-15 deg RIGHT (live: 5 deg LEFT)', t1c.peakBank >= 5 && t1c.peakBank <= 15, `peak=${t1c.peakBank.toFixed(1)} bank@2s=${t1c.bankAt2.toFixed(1)} peakRollIn=${t1c.peakRollIn.toFixed(2)}`);

  console.log('--- T2: same tap, STRADDLE phasing (pass-14 probe style) + 60fps control ---');
  const t2a = await tapBank('T2a fa18 223kt 4fps straddle', 'fa18', 223, 'straddle', DT4);
  const t2b = await tapBank('T2b fa18 776kt 4fps straddle', 'fa18', 776, 'straddle', DT4);
  const t2c = await tapBank('T2c fa18 223kt 60fps in-frame', 'fa18', 223, 'in-frame', DT60);
  const t2d = await tapBank('T2d fa18 776kt 60fps in-frame', 'fa18', 776, 'in-frame', DT60);
  console.log(`  phasing contrast @223kt: in-frame peakRollIn=${t1a.peakRollIn.toFixed(2)} stickInt=${t1a.stickInt.toFixed(3)} vs straddle peakRollIn=${t2a.peakRollIn.toFixed(2)} stickInt=${t2a.stickInt.toFixed(3)}`);
  console.log(`  phasing contrast @776kt: in-frame peakRollIn=${t1c.peakRollIn.toFixed(2)} stickInt=${t1c.stickInt.toFixed(3)} vs straddle peakRollIn=${t2b.peakRollIn.toFixed(2)} stickInt=${t2b.stickInt.toFixed(3)}`);
  console.log(`  fps contrast in-frame @223kt: 4fps peakBank=${t1a.peakBank.toFixed(1)} vs 60fps peakBank=${t2c.peakBank.toFixed(1)}; @776kt: 4fps ${t1c.peakBank.toFixed(1)} vs 60fps ${t2d.peakBank.toFixed(1)}`);
  check('T2 straddle tap @223kt 4fps: peak bank 5-15 deg RIGHT', t2a.peakBank >= 5 && t2a.peakBank <= 15, `peak=${t2a.peakBank.toFixed(1)}`);
  check('T2 in-frame tap @223kt 60fps: peak bank 5-15 deg RIGHT', t2c.peakBank >= 5 && t2c.peakBank <= 15, `peak=${t2c.peakBank.toFixed(1)}`);
  check('T2 in-frame tap @776kt 60fps: peak bank 5-15 deg RIGHT', t2d.peakBank >= 5 && t2d.peakBank <= 15, `peak=${t2d.peakBank.toFixed(1)}`);

  // ============ T3: ArrowUp / ArrowDown in-frame taps (pitch) ============
  // The tap's signature is its TRANSIENT (peak pitch / VSI in the 1 s
  // of sim after the tap): the 1g auto-trim restores level afterwards,
  // exactly as the auto-level returns the bank after a roll tap — so
  // dPitch@2s is expected to be ~0 and is NOT the measure.
  console.log('--- T3: ArrowUp / ArrowDown 100ms in-frame taps @265kt, 4fps ---');
  {
    const runPitchTap = async (code: 'ArrowUp' | 'ArrowDown'): Promise<{ peakDPitch: number; peakPitchIn: number; peakVsiDelta: number }> => {
      const s = new LiveSession('fa18');
      s.dtPattern = DT4;
      await s.attach();
      s.spawnAirborne(265, 300);
      for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
      const t0 = s.wall;
      const simTap = s.sim;
      const pitch0 = pitchOf(s.player);
      const vsi0 = s.player.vsi * 196.85;
      let fired = false;
      let peakPitchIn = 0;
      let peakDPitch = 0; // signed, Up negative, Down positive
      let peakVsiDelta = 0;
      const tr: TraceRow[] = [];
      while (s.wall < t0 + 2) {
        s.frame(() => {
          if (!fired) { fireKey('keydown', code); fireKey('keyup', code); fired = true; }
        });
        s.record();
        const r = s.trace[s.trace.length - 1]!;
        tr.push(r);
        peakPitchIn = Math.max(peakPitchIn, Math.abs(r.pitchIn));
        if (s.sim - simTap <= 1.0) {
          const dP = pitchOf(s.player) - pitch0;
          if (code === 'ArrowUp') peakDPitch = Math.min(peakDPitch, dP);
          else peakDPitch = Math.max(peakDPitch, dP);
          const dv = s.player.vsi * 196.85 - vsi0;
          if (code === 'ArrowUp') peakVsiDelta = Math.max(peakVsiDelta, dv);
          else peakVsiDelta = Math.min(peakVsiDelta, dv);
        }
        if (!s.player.alive) break;
      }
      const at2 = atWall(tr, t0 + 2.0);
      console.log(`  ${code} tap: peakPitchIn=${peakPitchIn.toFixed(2)} peakTransient dPitch=${peakDPitch.toFixed(1)} deg peakVSI delta=${peakVsiDelta.toFixed(0)}fpm dPitch@2s=${(at2.pitch - pitch0).toFixed(1)} alive=${s.player.alive}`);
      if (code === 'ArrowDown') {
        console.log('    down-tap trace (first 8 frames):');
        for (const r of tr.slice(0, 8)) {
          console.log(`      wall=${r.wall.toFixed(2)} sim=${r.sim.toFixed(2)} pitch=${r.pitch.toFixed(2)} pitchIn(peak)=${r.pitchIn.toFixed(2)} pcPitch(peak)=${r.pcPitch.toFixed(2)} ias=${r.iasKt.toFixed(0)} aoa=${r.aoa.toFixed(1)}`);
        }
      }
      s.dispose();
      return { peakDPitch, peakPitchIn, peakVsiDelta };
    };
    const up = await runPitchTap('ArrowUp');
    const down = await runPitchTap('ArrowDown');
    check('T3 ArrowUp in-frame tap @265kt: peak |pitchIn| >= 0.3, transient nose-up >= 1.5 deg', up.peakPitchIn >= 0.3 && up.peakDPitch <= -1.5, `peakPitchIn=${up.peakPitchIn.toFixed(2)} peakDPitch=${up.peakDPitch.toFixed(1)} peakVSI=${up.peakVsiDelta.toFixed(0)}fpm`);
    check('T3 ArrowDown in-frame tap @265kt: peak |pitchIn| >= 0.3, transient nose-down >= 1.0 deg', down.peakPitchIn >= 0.3 && down.peakDPitch >= 1.0, `peakPitchIn=${down.peakPitchIn.toFixed(2)} peakDPitch=${down.peakDPitch.toFixed(1)} peakVSI=${down.peakVsiDelta.toFixed(0)}fpm`);
  }

  // ============ T3b: direct pitch-step response (is the down/up asymmetry in the SOLVER?) ============
  console.log('--- T3b: direct pitchIn steps, fixed dt, fa18 265kt (no input path) ---');
  {
    const stepTest = async (pitchIn: number, durSim: number): Promise<{ peakDP: number; peakVsi: number }> => {
      const s = new LiveSession('fa18');
      s.dtPattern = DT60;
      await s.attach();
      s.spawnAirborne(265, 300);
      for (let i = 0; i < 60; i++) { s.frame(); s.record(); }
      const pitch0 = pitchOf(s.player);
      const vsi0 = s.player.vsi * 196.85;
      const sim0 = s.sim;
      let peakDP = 0;
      let peakVsi = 0;
      while (s.sim < sim0 + durSim + 1.0) {
        // Drive the physics directly at fixed dt, bypassing the input merge.
        const dt = 1 / 60;
        s.player.pitchIn = s.sim < sim0 + durSim ? pitchIn : 0;
        s.player.rollIn = 0;
        world.timestep = dt;
        world.step(eventQueue);
        drain(s.player);
        s.player.step(dt, WIND, null);
        s.sim += dt;
        const dP = pitchOf(s.player) - pitch0;
        if (pitchIn < 0) peakDP = Math.min(peakDP, dP);
        else peakDP = Math.max(peakDP, dP);
        const dv = s.player.vsi * 196.85 - vsi0;
        if (pitchIn < 0) peakVsi = Math.max(peakVsi, dv);
        else peakVsi = Math.min(peakVsi, dv);
        if (!s.player.alive) break;
      }
      console.log(`  pitchIn=${pitchIn} for ${durSim}s sim: peak dPitch=${peakDP.toFixed(1)} peak VSI delta=${peakVsi.toFixed(0)}fpm`);
      s.dispose();
      return { peakDP, peakVsi };
    };
    await stepTest(-0.3, 0.12);
    await stepTest(0.3, 0.12);
    await stepTest(-1, 0.5);
    await stepTest(1, 0.5);
  }

  // ============ T4: hold + release levels wings ============
  console.log('--- T4: KB hold Right 1s wall @223kt 4fps, then release — bank grows, then levels ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(223, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    let pressed = false;
    let released = false;
    const tr: TraceRow[] = [];
    while (s.wall < t0 + 6) {
      s.frame((ss) => {
        if (!pressed) { fireKey('keydown', 'ArrowRight'); pressed = true; }
        else if (!released && ss.wall - t0 >= 1.0) { fireKey('keyup', 'ArrowRight'); released = true; }
      });
      s.record();
      tr.push(s.trace[s.trace.length - 1]!);
      if (!s.player.alive) break;
    }
    const at1 = atWall(tr, t0 + 1.0);
    const at6 = atWall(tr, t0 + 6.0);
    const peak = peakBank(tr);
    console.log(`  hold: bank@1s=${at1.bank.toFixed(1)} peak=${peak.toFixed(1)} bank@6s (5s after release)=${at6.bank.toFixed(1)} alive=${s.player.alive}`);
    check('T4 hold Right 1s: bank@1s >= 15 deg RIGHT', at1.bank >= 15, `bank@1s=${at1.bank.toFixed(1)}`);
    check('T4 release: |bank| <= 10 deg 5 s after release', Math.abs(at6.bank) <= 10, `bank@6s=${at6.bank.toFixed(1)}`);
    s.dispose();
  }

  // ============ T5b (first): direct-step control for post-release travel ============
  // pitchIn +1 / -1 for 0.6s sim (the same sim length as a 1 s wall
  // hold at 4 fps), then 0 instantly. The jet's own pitch inertia
  // + trim recovery is the further travel with NO input tail.
  console.log('--- T5b: direct pitchIn step 0.6s sim, then 0 instantly (control) ---');
  const controlFurther = new Map<number, number>();
  for (const pi of [1, -1] as const) {
    const s = new LiveSession('fa18');
    s.dtPattern = DT60;
    await s.attach();
    s.spawnAirborne(265, 1500);
    for (let i = 0; i < 60; i++) { s.frame(); s.record(); }
    const pitch0 = pitchOf(s.player);
    const sim0 = s.sim;
    let pitchAtCut = 0;
    while (s.sim < sim0 + 0.6) {
      const dt = 1 / 60;
      s.player.pitchIn = pi; s.player.rollIn = 0;
      world.timestep = dt; world.step(eventQueue); drain(s.player);
      s.player.step(dt, WIND, null); s.sim += dt;
      pitchAtCut = pitchOf(s.player);
    }
    let pitchLater = pitchAtCut;
    while (s.sim < sim0 + 2.4 && s.player.alive) {
      const dt = 1 / 60;
      s.player.pitchIn = 0; s.player.rollIn = 0;
      world.timestep = dt; world.step(eventQueue); drain(s.player);
      s.player.step(dt, WIND, null); s.sim += dt;
      pitchLater = pitchOf(s.player);
    }
    controlFurther.set(pi, pitchLater - pitchAtCut);
    console.log(`  direct pitchIn=${pi} 0.6s: dPitch during step=${(pitchAtCut - pitch0).toFixed(1)} deg, further travel in next 1.8s sim=${(pitchLater - pitchAtCut).toFixed(1)} deg alive=${s.player.alive}`);
    s.dispose();
  }

  // ============ T5: pitch HOLD 1s then release — the release tail must add nothing ============
  // The live-path hold (1 s wall ~ 0.6 s sim) must travel after
  // release like the direct control above: the input's return tail
  // (slew-down + any tap-hold) must not extend the hold. The
  // tap-hold is gated to taps only (TAP_MAX_WALL_MS), so a 1 s
  // hold releases clean.
  console.log('--- T5: ArrowUp / ArrowDown hold 1s wall @265kt, 1500 m, then release ---');
  for (const [code, pi] of [['ArrowUp', -1], ['ArrowDown', 1]] as const) {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(265, 1500);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    const pitch0 = pitchOf(s.player);
    let pressed = false;
    let released = false;
    let pitchAtRelease = 0;
    let simAtRelease = 0;
    let pitchInAfter05 = -1;
    let further = 0;
    while (s.wall < t0 + 5) {
      s.frame((ss) => {
        if (!pressed) { fireKey('keydown', code); pressed = true; }
        else if (!released && ss.wall - t0 >= 1.0) {
          fireKey('keyup', code); released = true;
          pitchAtRelease = pitchOf(s.player); simAtRelease = s.sim;
        }
      });
      s.record();
      if (released) {
        if (pitchInAfter05 < 0 && s.sim - simAtRelease >= 0.5) pitchInAfter05 = s.trace[s.trace.length - 1]!.pitchIn;
        if (s.sim - simAtRelease >= 1.8) { further = pitchOf(s.player) - pitchAtRelease; break; }
      }
      if (!s.player.alive) break;
    }
    const dHold = pitchAtRelease - pitch0;
    const ctrl = controlFurther.get(pi)!;
    console.log(`  ${code} hold 1s: dPitch during hold=${dHold.toFixed(1)} deg, pitchIn 0.5s sim after release=${pitchInAfter05.toFixed(2)}, further travel 1.8s sim after release=${further.toFixed(1)} deg (direct control ${ctrl.toFixed(1)} deg) alive=${s.player.alive}`);
    check(`T5 ${code} hold: pitchIn is 0 within 0.5s sim of release`, pitchInAfter05 === 0, `pitchIn@+0.5s sim=${pitchInAfter05.toFixed(2)}`);
    check(`T5 ${code} hold: post-release travel matches the direct control within 8 deg (no input-tail extension)`, Math.abs(further - ctrl) <= 8, `live further=${further.toFixed(1)} control=${ctrl.toFixed(1)}`);
    s.dispose();
  }

  // ============ PASS 15, ITEM 7: hands-off bank drift through the LIVE loop ============
  // Establish the attitude through the LIVE loop (joystick hold via real
  // pointer events), then release everything and watch the bank hands-off
  // for 10 s of SIM time at 4 fps, live wind (-3,0,0). rollIn is traced
  // every frame: if it is ever nonzero hands-off, the merge is leaking.
  const driftRun = async (
    label: string,
    setup: 'level' | 'dive' | 'climb' | 'bank60',
    wind: THREE.Vector3,
    altM = 300,
  ): Promise<{ bankStart: number; bankEnd: number; maxAbsBank: number; maxAbsRollIn: number; minBeta: number; maxBeta: number; alive: boolean }> => {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, altM);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    // Setup phase through the live loop with the real joystick.
    const setupWall = setup === 'level' ? 0 : setup === 'bank60' ? 4 : 3;
    if (setup !== 'level') {
      const cx = setup === 'bank60' ? r.left + r.width - 1 : c.x;
      const cy = setup === 'dive' ? r.top + r.height - 1 : setup === 'climb' ? r.top + 1 : c.y;
      fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
      fireJoy(s.joyEl, 'pointermove', cx, cy, 7);
      const tSetup = s.wall;
      while (s.wall < tSetup + setupWall) { s.frame(); s.record(); if (!s.player.alive) break; }
      fireJoy(s.joyEl, 'pointerup', cx, cy, 7);
    }
    // Settle 0.5 s wall so the kb/joy tails clear, then hands-off.
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    const savedWind = WIND;
    WIND = wind;
    const sim0 = s.sim;
    const bankStart = bankOf(s.player);
    const pitchStart = pitchOf(s.player);
    let maxAbsBank = Math.abs(bankStart);
    let maxAbsRollIn = 0;
    let minBeta = Infinity;
    let maxBeta = -Infinity;
    const tr: TraceRow[] = [];
    while (s.sim < sim0 + 10) {
      s.frame();
      s.record();
      const row = s.trace[s.trace.length - 1]!;
      tr.push(row);
      maxAbsBank = Math.max(maxAbsBank, Math.abs(row.bank));
      maxAbsRollIn = Math.max(maxAbsRollIn, Math.abs(row.rollIn));
      minBeta = Math.min(minBeta, s.player.betaDeg);
      maxBeta = Math.max(maxBeta, s.player.betaDeg);
      if (!s.player.alive) break;
    }
    WIND = savedWind;
    const bankEnd = bankOf(s.player);
    console.log(
      `  ${label}: pitchStart=${pitchStart.toFixed(1)} bank ${bankStart.toFixed(1)} -> ${bankEnd.toFixed(1)} (max|bank|=${maxAbsBank.toFixed(1)}) max|rollIn| hands-off=${maxAbsRollIn.toFixed(3)} beta=[${minBeta.toFixed(1)}, ${maxBeta.toFixed(1)}] alive=${s.player.alive} sim=${(s.sim - sim0).toFixed(1)}s`,
    );
    // Per-second bank log for the drift shape.
    let line = '    bank/s:';
    for (let sec = 1; sec <= 10; sec++) {
      const row = tr.find((x) => x.sim >= sim0 + sec);
      if (row) line += ` ${sec}s:${row.bank.toFixed(0)}`;
    }
    console.log(line);
    s.dispose();
    return { bankStart, bankEnd, maxAbsBank, maxAbsRollIn, minBeta, maxBeta, alive: s.player.alive };
  };

  console.log('--- D1: hands-off drift, LEVEL flight, live wind ---');
  const d1 = await driftRun('D1 level, wind -3', 'level', new THREE.Vector3(-3, 0, 0));
  check('D1 level hands-off 10s sim: |bank| stays <= 5 deg', d1.maxAbsBank <= 5, `max|bank|=${d1.maxAbsBank.toFixed(1)} end=${d1.bankEnd.toFixed(1)}`);
  check('D1 level hands-off: merged rollIn stays 0 (no input leak)', d1.maxAbsRollIn === 0, `max|rollIn|=${d1.maxAbsRollIn.toFixed(3)}`);

  console.log('--- D2: hands-off drift, DIVE, live wind ---');
  const d2 = await driftRun('D2 dive, wind -3', 'dive', new THREE.Vector3(-3, 0, 0), 1500);
  check('D2 dive hands-off 10s sim: |bank| stays <= 5 deg', d2.maxAbsBank <= 5, `max|bank|=${d2.maxAbsBank.toFixed(1)} end=${d2.bankEnd.toFixed(1)}`);

  console.log('--- D3: hands-off drift, CLIMB, live wind ---');
  const d3 = await driftRun('D3 climb, wind -3', 'climb', new THREE.Vector3(-3, 0, 0));
  check('D3 climb hands-off 10s sim: |bank| stays <= 5 deg', d3.maxAbsBank <= 5, `max|bank|=${d3.maxAbsBank.toFixed(1)} end=${d3.bankEnd.toFixed(1)}`);

  console.log('--- D4: hands-off drift, LEVEL flight, WIND 0 (isolation) ---');
  const d4 = await driftRun('D4 level, wind 0', 'level', new THREE.Vector3(0, 0, 0));
  check('D4 level hands-off wind 0: |bank| stays <= 5 deg', d4.maxAbsBank <= 5, `max|bank|=${d4.maxAbsBank.toFixed(1)} end=${d4.bankEnd.toFixed(1)}`);

  console.log('--- D5: hands-off from 60 deg bank, live wind ---');
  const d5 = await driftRun('D5 from 60deg, wind -3', 'bank60', new THREE.Vector3(-3, 0, 0));
  check('D5 hands-off from 60 deg: |bank| <= 10 deg after 10s sim', Math.abs(d5.bankEnd) <= 10, `end=${d5.bankEnd.toFixed(1)} max|bank|=${d5.maxAbsBank.toFixed(1)}`);

  console.log(failures === 0 ? '\nALL PASS-15 CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
