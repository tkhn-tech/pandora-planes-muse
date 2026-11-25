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

  // ============ PASS 17, WORKER A: ROLL/PITCH INPUTS DEAD LIVE ============
  // The v4.10 live verdict (25 Nov 2026, 16:13-17:24 SGT, footer v4.10,
  // 4-9 fps on deck, 24-32 fps at altitude) says roll/pitch inputs are
  // DEAD live: Right Arrow tap at 295-299 kt and 420-434 kt left Bank
  // 0 deg (heading 090->091 deg), ArrowUp tap produced no pitch change,
  // joystick roll drags up to 13 s left Bank 0 deg (joystick pitch drag
  // works - takeoffs succeed). In the v4.9 live check the joystick DID
  // bank 66 deg right on a hold, so this is a regression vs v4.9 or an
  // environment difference.
  //
  // What this probe ESTABLISHED (all through the LIVE path: variable
  // frame dt 16-250 ms, live wind (-3,0,0), REAL keydown/pointer events
  // dispatched through the real handlers, real GLBs with an Image stub
  // and a modelStatus === 'ok' assertion, the main.ts playerInput merge
  // verbatim, consumeEvents-before-getControls ordering):
  //   * the input chain is NOT dead: taps bank 5-11 deg peak, holds
  //     bank 18-180 deg, joystick pitch lifts the jet off Alpha.
  //   * the live "Bank 0" is the AUTO-LEVEL erasing the tap: a Right
  //     tap peaks at 6.0 deg (4 fps) / 9.3 deg (28 fps) at 295 kt and
  //     the spring returns the bank to ~0 deg within 1-2 s (measured
  //     S7 below: bank@+2s wall -0.2 deg @4fps, 0.0 deg @28fps). A live
  //     checker reading the HUD a second or two after acting sees Bank
  //     0 deg - and the heading change 090->091 deg is exactly the
  //     turn a 6-9 deg transient bank produces (~1-2 deg of heading).
  //     The ArrowUp tap is the same story on pitch: transient -2.4
  //     deg nose-up / +408 fpm, restored by the 1g trim within ~2 s.
  //   * a sustained airborne joystick roll drag CANNOT give Bank 0 in
  //     any regime tested (level, climb, gear down, flaps 30, 28 fps,
  //     all four jets, high AoA, mouse yoke, stale yoke): it banks
  //     100-180 deg in 10 s. The only code paths that hold Bank 0 for
  //     a sustained roll drag are the on-ground gear spring (E4, by
  //     design - a jet cannot roll on its gear) and the dying/dead
  //     early-return in playerInput. So the live drags were done on
  //     the ground during the takeoff roll, in a frozen/dying state,
  //     or the pointer events never reached #joy - all outside the
  //     input files, and all needing a live-browser re-test with the
  //     ?debug=1 stick line (live p/r + joy x/y) visible.
  //
  // THE FIX (controller patch spec - Worker A may not edit the
  // auto-level code; the coordinator applies it): a 3 deg deadband on
  // the airborne auto-level spring (FlightPhysicsController.ts). The
  // spring is a pure rate spring with no deadband, so it chases the
  // bank to exactly 0 deg - a tap's 6-9 deg bank is gone before a
  // human can see it. With a 3 deg deadband a tap settles at ~3 deg
  // and stays (visible, and the jet turns slowly), a hold-release
  // settles at ~3 deg, and hands-off levelling from 30-60 deg still
  // works (the spring is unchanged above 3 deg). 3 deg - not 5 - is
  // chosen so every existing return-to-level assertion still holds:
  // probe-roll D1/D3 (bank <= 5 deg at +0.5 s), F (bank <= 5 deg
  // within 2 s), F4/S8 (<= 10 deg after 3 s), pass15 T4/D5 (<= 10 deg)
  // all pass with a 3 deg settle. The exact patch text is in the
  // Worker A report; S7 below FAILS pre-fix and PASSES post-fix.
  //
  // Usage: node_modules/.bin/esbuild scripts/probe-pass17-inputs.ts
  //   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass17-inputs.mjs
  //   && node /tmp/probe-pass17-inputs.mjs
  //   (then rm -f /tmp/probe-pass17-inputs.mjs)

  const DT4 = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const DT28 = [0.036, 0.036, 0.036, 0.036, 0.036];
  const DT60 = [0.0167, 0.0167, 0.0167, 0.0167, 0.0167];

  const tapRight = async (
    s: LiveSession,
    label: string,
  ): Promise<{ peak: number; bankAt2: number; bankAt3: number; peakRollIn: number }> => {
    const t0 = s.wall;
    const bank0 = bankOf(s.player);
    let fired = false;
    let peak = 0;
    let peakRollIn = 0;
    const series: Array<{ wall: number; bank: number }> = [];
    while (s.wall < t0 + 3) {
      s.frame(() => {
        if (!fired) { fireKey('keydown', 'ArrowRight'); fireKey('keyup', 'ArrowRight'); fired = true; }
      });
      s.record();
      const row = s.trace[s.trace.length - 1]!;
      peak = Math.max(peak, Math.abs(row.bank - bank0));
      peakRollIn = Math.max(peakRollIn, Math.abs(row.rollIn));
      series.push({ wall: row.wall - t0, bank: row.bank - bank0 });
      if (!s.player.alive) break;
    }
    const at = (t: number): number =>
      series.reduce((a, b) => (Math.abs(b.wall - t) < Math.abs(a.wall - t) ? b : a)).bank;
    const r = s.trace[s.trace.length - 1]!;
    console.log(
      `  ${label}: peakBank=${peak.toFixed(1)} peakRollIn=${peakRollIn.toFixed(2)} bank@+2s=${at(2).toFixed(1)} bank@+3s=${at(3).toFixed(1)} ias=${r.iasKt.toFixed(0)}kt alive=${s.player.alive}`,
    );
    return { peak, bankAt2: at(2), bankAt3: at(3), peakRollIn };
  };

  const joyHold = async (
    s: LiveSession,
    jx: number,
    jy: number,
    wallSecs: number,
    label: string,
  ): Promise<{ end: number; peak: number; meanRollIn: number }> => {
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x + jx * (r.width / 2), c.y + jy * (r.height / 2), 7);
    const t0 = s.wall;
    const bank0 = bankOf(s.player);
    let peak = 0;
    let meanRollIn = 0;
    let n = 0;
    while (s.wall < t0 + wallSecs) {
      s.frame();
      s.record();
      const row = s.trace[s.trace.length - 1]!;
      peak = Math.max(peak, Math.abs(row.bank - bank0));
      meanRollIn += row.rollIn; n++;
      if (!s.player.alive) break;
    }
    fireJoy(s.joyEl, 'pointerup', c.x + jx * (r.width / 2), c.y + jy * (r.height / 2), 7);
    const last = s.trace[s.trace.length - 1]!;
    console.log(
      `  ${label}: meanRollIn=${(meanRollIn / Math.max(1, n)).toFixed(2)} peakBank=${peak.toFixed(1)} bankEnd=${(last.bank - bank0).toFixed(1)} hdgEnd=${last.hdg.toFixed(0)} ias=${last.iasKt.toFixed(0)}kt pitch=${last.pitch.toFixed(1)} alive=${s.player.alive}`,
    );
    return { end: last.bank - bank0, peak, meanRollIn: meanRollIn / Math.max(1, n) };
  };

  // ---- S1: Right Arrow taps at 223/524/776 kt (the live speeds bracketed) ----
  console.log('--- S1: Right Arrow 100ms in-frame taps @223/524/776kt, 4fps ---');
  {
    const runs: Array<[string, number]> = [['fa18', 223], ['f35a', 524], ['fa18', 776]];
    for (const [key, kt] of runs) {
      const s = new LiveSession(key);
      s.dtPattern = DT4;
      await s.attach();
      if (key === 'fa18' && kt === 223) {
        check('S1 model: real GLB attached (modelStatus ok)', s.player.modelStatus === 'ok', `modelStatus=${s.player.modelStatus} tris=${s.player.modelTris} err=${s.player.modelError ?? 'none'}`);
      }
      s.spawnAirborne(kt, 300);
      for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
      const r = await tapRight(s, `S1 ${key} ${kt}kt tap`);
      check(`S1 tap Right @${kt}kt: peak bank 5-15 deg RIGHT`, r.peak >= 5 && r.peak <= 15, `peak=${r.peak.toFixed(1)} peakRollIn=${r.peakRollIn.toFixed(2)}`);
      s.dispose();
    }
  }

  // ---- S2: ArrowUp tap produces pitch ----
  console.log('--- S2: ArrowUp 100ms in-frame tap @265kt, 4fps ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(265, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    const pitch0 = pitchOf(s.player);
    let fired = false;
    let peakPitchIn = 0;
    let peakUp = 0;
    while (s.wall < t0 + 2) {
      s.frame(() => {
        if (!fired) { fireKey('keydown', 'ArrowUp'); fireKey('keyup', 'ArrowUp'); fired = true; }
      });
      s.record();
      const row = s.trace[s.trace.length - 1]!;
      peakPitchIn = Math.max(peakPitchIn, Math.abs(row.pitchIn));
      peakUp = Math.min(peakUp, pitchOf(s.player) - pitch0);
      if (!s.player.alive) break;
    }
    console.log(`  S2 ArrowUp tap: peakPitchIn=${peakPitchIn.toFixed(2)} peakTransient dPitch=${peakUp.toFixed(1)} deg`);
    check('S2 ArrowUp tap @265kt: peak |pitchIn| >= 0.3, transient nose-up >= 1.5 deg', peakPitchIn >= 0.3 && peakUp <= -1.5, `peakPitchIn=${peakPitchIn.toFixed(2)} peakDPitch=${peakUp.toFixed(1)}`);
    s.dispose();
  }

  // ---- S3: 1 s roll hold ----
  console.log('--- S3: KB hold Right 1s wall @223kt, 4fps ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(223, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    const bank0 = bankOf(s.player);
    let pressed = false;
    let released = false;
    let bankAt1 = 0;
    while (s.wall < t0 + 1.2) {
      s.frame((ss) => {
        if (!pressed) { fireKey('keydown', 'ArrowRight'); pressed = true; }
        else if (!released && ss.wall - t0 >= 1.0) { fireKey('keyup', 'ArrowRight'); released = true; }
      });
      s.record();
      if (s.wall >= t0 + 1.0 && bankAt1 === 0) bankAt1 = bankOf(s.player) - bank0;
      if (!s.player.alive) break;
    }
    console.log(`  S3 hold: bank@1s=${bankAt1.toFixed(1)}`);
    check('S3 hold Right 1s @223kt: bank@1s >= 15 deg RIGHT', bankAt1 >= 15, `bank@1s=${bankAt1.toFixed(1)}`);
    s.dispose();
  }

  // ---- S4: joystick full-right hold 10 s (v4.9 regression case: ~66 deg) ----
  console.log('--- S4: JOY full-right hold 10s wall, F-18, 240kt / 300m ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const h = await joyHold(s, 1, 0, 10, 'S4 joy full-right 10s');
    check('S4 JOY full-right hold 10s: bank reaches >= 60 deg (v4.9 live: 66 deg)', Math.abs(h.end) >= 60 || h.peak >= 60, `bankEnd=${h.end.toFixed(1)} peak=${h.peak.toFixed(1)} meanRollIn=${h.meanRollIn.toFixed(2)}`);
    s.dispose();
  }

  // ---- S5: keyboard pitch hold ----
  console.log('--- S5: KB ArrowUp hold 1s wall @265kt, 1500m ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(265, 1500);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    const pitch0 = pitchOf(s.player);
    let pressed = false;
    let released = false;
    let minDPitch = 0;
    while (s.wall < t0 + 1.5) {
      s.frame((ss) => {
        if (!pressed) { fireKey('keydown', 'ArrowUp'); pressed = true; }
        else if (!released && ss.wall - t0 >= 1.0) { fireKey('keyup', 'ArrowUp'); released = true; }
      });
      s.record();
      minDPitch = Math.min(minDPitch, pitchOf(s.player) - pitch0);
      if (!s.player.alive) break;
    }
    console.log(`  S5 ArrowUp hold 1s: min dPitch during hold=${minDPitch.toFixed(1)} deg`);
    check('S5 ArrowUp hold 1s @265kt: nose-up >= 5 deg', minDPitch <= -5, `min dPitch=${minDPitch.toFixed(1)}`);
    s.dispose();
  }

  // ---- S6: joystick pitch still works (Alpha takeoff on a joystick pull) ----
  console.log('--- S6: joystick pitch - Alpha takeoff on a joystick-up pull ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAlpha();
    s.player.thrTarget = 0.9;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    let pulled = false;
    let lifted = false;
    let maxVsi = 0;
    while (s.wall < 40 && !lifted) {
      s.frame(() => {
        const iasKt = s.player.ias * MS2KT;
        if (!pulled && iasKt >= 140) {
          fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
          fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
          pulled = true;
        }
      });
      s.record();
      maxVsi = Math.max(maxVsi, s.player.vsi * 196.85);
      if (pulled && !s.player.onSurface && s.player.pos.y - s.player.floorY > 30) lifted = true;
      if (!s.player.alive) break;
    }
    console.log(`  S6 takeoff: pulled=${pulled} lifted=${lifted} maxVSI=${maxVsi.toFixed(0)}fpm ias=${(s.player.ias * MS2KT).toFixed(0)}kt alive=${s.player.alive}`);
    check('S6 joystick-up pull lifts off Alpha and climbs', lifted && maxVsi > 3000, `lifted=${lifted} maxVSI=${maxVsi.toFixed(0)}fpm`);
    s.dispose();
  }

  // ---- S7: tap-bank PERSISTENCE (THE live failure - FAILS pre-fix) ----
  // A live checker reads the HUD a second or two after acting. Pre-fix
  // the auto-level spring (no deadband) erases the tap's 6-9 deg bank
  // to ~0 deg by +2 s wall, so the live read is "Bank 0 deg". Post-fix
  // (3 deg auto-level deadband, coordinator patch) the tap settles at
  // ~3 deg and stays - visible, and the jet turns slowly. The S7 bar
  // is >= 2.5 deg at +2 s (not 3.0): the spring drives the bank toward
  // 3 deg asymptotically, so at 28 fps the +2 s read sits at ~3.05 deg
  // and a 3.0 bar would have no margin against the bank0 offset; 2.5
  // still proves persistence decisively (pre-fix reads -0.2/0.0 deg).
  console.log('--- S7: tap-bank persistence @295kt (bank 2-3 s after the tap) ---');
  for (const [pat, patName] of [[DT4, '4fps'], [DT28, '28fps (live altitude fps)']] as const) {
    const s = new LiveSession('fa18');
    s.dtPattern = pat;
    await s.attach();
    s.spawnAirborne(295, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const r = await tapRight(s, `S7 ${patName} tap persistence`);
    check(`S7 tap @295kt ${patName}: bank still >= 2.5 deg at +2 s wall (visible to a live read)`, r.bankAt2 >= 2.5, `bank@+2s=${r.bankAt2.toFixed(1)} (peak was ${r.peak.toFixed(1)})`);
    check(`S7 tap @295kt ${patName}: bank still >= 2 deg at +3 s wall`, r.bankAt3 >= 2, `bank@+3s=${r.bankAt3.toFixed(1)}`);
    s.dispose();
  }

  // ---- S8: guard - a hold-release still levels (deadband must not break this) ----
  console.log('--- S8: guard - hold Right 1s then release, bank 5 s later <= 10 deg ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(223, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const t0 = s.wall;
    let pressed = false;
    let released = false;
    while (s.wall < t0 + 6) {
      s.frame((ss) => {
        if (!pressed) { fireKey('keydown', 'ArrowRight'); pressed = true; }
        else if (!released && ss.wall - t0 >= 1.0) { fireKey('keyup', 'ArrowRight'); released = true; }
      });
      s.record();
      if (!s.player.alive) break;
    }
    const last = s.trace[s.trace.length - 1]!;
    console.log(`  S8 release: bank@+6s=${last.bank.toFixed(1)}`);
    check('S8 hold-release: |bank| <= 10 deg 5 s after release', Math.abs(last.bank) <= 10, `bank@6s=${last.bank.toFixed(1)}`);
    s.dispose();
  }

  // ---- S9: guard - hands-off auto-level from 60 deg still works ----
  console.log('--- S9: guard - hands-off auto-level from 60 deg, 10 s sim ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(287, 300);
    for (let i = 0; i < 240; i++) {
      const b = bankOf(s.player);
      s.player.rollIn = clamp01((60 - b) * 0.05);
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(s.player);
      s.player.step(dt, WIND, null);
    }
    const b0 = bankOf(s.player);
    s.player.rollIn = 0;
    const sim0 = s.sim;
    let bankEnd = b0;
    while (s.sim < sim0 + 10) {
      s.frame();
      s.record();
      bankEnd = bankOf(s.player);
      if (!s.player.alive) break;
    }
    console.log(`  S9 auto-level: from ${b0.toFixed(1)} deg to ${bankEnd.toFixed(1)} deg in 10 s sim`);
    check('S9 hands-off from 60 deg: |bank| <= 10 deg after 10 s sim', Math.abs(bankEnd) <= 10, `bank=${bankEnd.toFixed(1)}`);
    s.dispose();
  }

  console.log(failures === 0 ? '\nALL PASS-17 INPUT CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
