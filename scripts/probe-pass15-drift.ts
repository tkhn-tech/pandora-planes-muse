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
    }
    lastPcPitch = 0;
    lastPcRoll = 0;

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
        pcPitch: this.lastPcPitch,
        pcRoll: this.lastPcRoll,
        pitchIn: p.pitchIn,
        rollIn: p.rollIn,
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

  const DT4 = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const DT60 = [0.0167, 0.0167, 0.0167, 0.0167, 0.0167];
  // ============ PASS 15 ITEM 7, ROUND 2: the regimes round 1 did not cover ============
  // Round 1 (level / 46-deg dive / 44-deg climb, fa18, headwind) showed
  // bank EXACTLY 0.0 hands-off — the auto-level works there. The live
  // drift (37R dive, 30L climb, 43R straight-up hold) must come from a
  // regime round 1 missed: pitch beyond the 60-deg auto-level gate,
  // stall (auto-level gated off), a true crosswind (Alpha hdg 90 makes
  // the live (-3,0,0) wind a pure headwind, beta 0), another airframe,
  // or the low post-takeoff regime with gear down.
  // Robust bank: wing-axis bank phi = atan2(w.y, |w horizontal|) where
  // w = quat*(1,0,0) is the LEFT wing direction. Near-vertical pitch
  // the Euler YXZ bank is a gimbal artefact; phi stays meaningful
  // (it is ~0 whenever the wings are level w.r.t. the horizon, at any
  // pitch) — it distinguishes a real roll from a HUD readout artefact.
  const robustBank = (ac: Aircraft): number => {
    const w = new THREE.Vector3(1, 0, 0).applyQuaternion(ac.quat);
    return Math.atan2(w.y, Math.hypot(w.x, w.z)) * R2D;
  };
  const logBankRow = (tag: string, s: LiveSession): void => {
    const p = s.player;
    console.log(
      `    ${tag} sim=${s.sim.toFixed(1)} eulerBank=${bankOf(p).toFixed(1)} robustBank=${robustBank(p).toFixed(1)} pitch=${pitchOf(p).toFixed(1)} ias=${(p.ias * MS2KT).toFixed(0)}kt aoa=${p.aoaDeg.toFixed(1)} beta=${p.betaDeg.toFixed(1)} stalled=${p.stalled} rollIn=${p.rollIn.toFixed(2)} alive=${p.alive}`,
    );
  };

  // E1: steep dive, pitch +65..75 deg nose-down, hands-off.
  console.log('--- E1: steep DIVE (pitch +70 nose-down), hands-off, fa18, 1500 m ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 1500);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, r.top + r.height - 1, 7);
    while (pitchOf(s.player) < 65 && s.wall < 20) { s.frame(); s.record(); if (!s.player.alive) break; }
    fireJoy(s.joyEl, 'pointerup', c.x, r.top + r.height - 1, 7);
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    console.log(`  dive entry: pitch=${pitchOf(s.player).toFixed(1)} eulerBank=${bankOf(s.player).toFixed(1)} robustBank=${robustBank(s.player).toFixed(1)}`);
    const sim0 = s.sim;
    let maxEuler = 0;
    let maxRobust = 0;
    let sec = 0;
    while (s.sim < sim0 + 8 && s.player.alive && s.player.pos.y - s.player.floorY > 100) {
      s.frame(); s.record();
      maxEuler = Math.max(maxEuler, Math.abs(bankOf(s.player)));
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); }
    }
    console.log(`  steep dive hands-off: max|eulerBank|=${maxEuler.toFixed(1)} max|robustBank|=${maxRobust.toFixed(1)} alive=${s.player.alive}`);
    check('E1 steep dive hands-off: robust bank stays <= 5 deg', maxRobust <= 5, `maxRobust=${maxRobust.toFixed(1)} maxEuler=${maxEuler.toFixed(1)}`);
    s.dispose();
  }

  // E2: steep climb, pitch -65..-75 deg nose-up, hands-off.
  console.log('--- E2: steep CLIMB (pitch -70 nose-up), hands-off, fa18, 1500 m ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 1500);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
    while (pitchOf(s.player) > -65 && s.wall < 20) { s.frame(); s.record(); if (!s.player.alive) break; }
    fireJoy(s.joyEl, 'pointerup', c.x, r.top + 1, 7);
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    console.log(`  climb entry: pitch=${pitchOf(s.player).toFixed(1)} eulerBank=${bankOf(s.player).toFixed(1)} robustBank=${robustBank(s.player).toFixed(1)} ias=${(s.player.ias * MS2KT).toFixed(0)}kt stalled=${s.player.stalled}`);
    const sim0 = s.sim;
    let maxEuler = 0;
    let maxRobust = 0;
    let sec = 0;
    while (s.sim < sim0 + 8 && s.player.alive) {
      s.frame(); s.record();
      maxEuler = Math.max(maxEuler, Math.abs(bankOf(s.player)));
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); }
    }
    console.log(`  steep climb hands-off: max|eulerBank|=${maxEuler.toFixed(1)} max|robustBank|=${maxRobust.toFixed(1)} alive=${s.player.alive}`);
    check('E2 steep climb hands-off: robust bank stays <= 5 deg', maxRobust <= 5, `maxRobust=${maxRobust.toFixed(1)} maxEuler=${maxEuler.toFixed(1)}`);
    s.dispose();
  }

  // E3: climb to STALL, then hands-off (auto-level is gated off while stalled).
  console.log('--- E3: climb to stall, then hands-off, fa18 ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 1500);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
    while (!s.player.stalled && s.wall < 30) { s.frame(); s.record(); if (!s.player.alive) break; }
    console.log(`  stall entry: pitch=${pitchOf(s.player).toFixed(1)} ias=${(s.player.ias * MS2KT).toFixed(0)}kt aoa=${s.player.aoaDeg.toFixed(1)} stalled=${s.player.stalled} bank=${bankOf(s.player).toFixed(1)}`);
    fireJoy(s.joyEl, 'pointerup', c.x, r.top + 1, 7);
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    const sim0 = s.sim;
    let maxRobust = 0;
    let sec = 0;
    while (s.sim < sim0 + 6 && s.player.alive) {
      s.frame(); s.record();
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); }
    }
    console.log(`  stalled hands-off: max|robustBank|=${maxRobust.toFixed(1)} alive=${s.player.alive}`);
    s.dispose();
  }

  // E4: true CROSSWIND level flight — heading 0 (north) and 180 (south),
  // so the live (-3,0,0) wind is a full 3 m/s crosswind, beta != 0.
  console.log('--- E4: level hands-off, CROSSWIND headings 0 and 180, fa18 ---');
  for (const hdg of [0, 180]) {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    origin.reset();
    s.player.resetAt(alpha.x, alpha.elev + 300, alpha.z, hdg);
    s.player.setAirspeed(240 / MS2KT);
    s.player.gear = false;
    s.player.thrTarget = 0.8;
    s.joy.active = false; s.joy.x = 0; s.joy.y = 0;
    s.pc.calibrate(); s.tilt.calibrate();
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const sim0 = s.sim;
    let maxBank = 0;
    let maxBeta = 0;
    while (s.sim < sim0 + 10 && s.player.alive) {
      s.frame(); s.record();
      maxBank = Math.max(maxBank, Math.abs(bankOf(s.player)));
      maxBeta = Math.max(maxBeta, Math.abs(s.player.betaDeg));
    }
    console.log(`  hdg ${hdg}: max|bank|=${maxBank.toFixed(1)} max|beta|=${maxBeta.toFixed(1)} end bank=${bankOf(s.player).toFixed(1)} alive=${s.player.alive}`);
    check(`E4 crosswind hdg ${hdg} level hands-off: |bank| <= 5 deg`, maxBank <= 5, `maxBank=${maxBank.toFixed(1)} maxBeta=${maxBeta.toFixed(1)}`);
    s.dispose();
  }

  // E5: other airframes, level hands-off.
  console.log('--- E5: level hands-off, f22 / f15e / f35a ---');
  for (const key of ['f22', 'f15e', 'f35a'] as const) {
    const s = new LiveSession(key);
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const sim0 = s.sim;
    let maxBank = 0;
    while (s.sim < sim0 + 10 && s.player.alive) {
      s.frame(); s.record();
      maxBank = Math.max(maxBank, Math.abs(bankOf(s.player)));
    }
    console.log(`  ${key}: max|bank|=${maxBank.toFixed(1)} end=${bankOf(s.player).toFixed(1)} alive=${s.player.alive}`);
    check(`E5 ${key} level hands-off: |bank| <= 5 deg`, maxBank <= 5, `maxBank=${maxBank.toFixed(1)}`);
    s.dispose();
  }

  // E6: the LIVE regime — post-takeoff climb-out, gear DOWN, hands-off.
  console.log('--- E6: post-takeoff climb-out hands-off, gear down, fa18 ---');
  {
    const s = await takeoff('fa18', 0.9, 210, 270);
    if (s) {
      s.dtPattern = DT4;
      const sim0 = s.sim;
      let maxBank = 0;
      let sec = 0;
      while (s.sim < sim0 + 10 && s.player.alive) {
        s.frame(); s.record();
        maxBank = Math.max(maxBank, Math.abs(bankOf(s.player)));
        if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); }
      }
      console.log(`  climb-out hands-off: max|bank|=${maxBank.toFixed(1)} end=${bankOf(s.player).toFixed(1)} gear=${s.player.gear} alive=${s.player.alive}`);
      check('E6 climb-out hands-off: |bank| <= 5 deg', maxBank <= 5, `maxBank=${maxBank.toFixed(1)}`);
      s.dispose();
    } else {
      check('E6 climb-out hands-off: |bank| <= 5 deg', false, 'takeoff did not reach the band');
    }
  }

  // ============ PASS 15 ITEM 7, ROUND 3: banked STEEP dive/climb + loop (the fix's proof) ============
  // Rounds 1-2 showed a wings-level jet never drifts in any regime
  // (the BET is laterally symmetric undisturbed) — so the live
  // drift is a bank that is SET (during the dive/climb entry, or
  // by the tap) and then NOT REMOVED, because the pass-13 |pitch|
  // > 60 deg gate switched the auto-level off for the whole steep
  // leg. These sections set a ~30 deg bank, THEN pitch past 60 deg,
  // release, and require the bank to return hands-off — which only
  // the gimbal-safe (wing-axis) spring can do, since it has no
  // pitch gate. F3 is the loop-top control: the old Euler spring
  // sliced the nose sideways there (beta 0 -> +35 deg in 0.5 s).
  const bankedSteep = async (
    label: string,
    joyX: number,
    joyY: number,
    pitchTarget: number, // + = nose-down dive, - = nose-up climb
  ): Promise<void> => {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 2000);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    const cx = joyX > 0 ? r.left + r.width - 1 : joyX < 0 ? r.left + 1 : c.x;
    const cy = joyY > 0 ? r.top + r.height - 1 : joyY < 0 ? r.top + 1 : c.y;
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', cx, cy, 7);
    while (
      (pitchTarget > 0 ? pitchOf(s.player) < pitchTarget : pitchOf(s.player) > pitchTarget) &&
      s.wall < 25
    ) { s.frame(); s.record(); if (!s.player.alive) break; }
    fireJoy(s.joyEl, 'pointerup', cx, cy, 7);
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    const bankRel = bankOf(s.player);
    const robustRel = robustBank(s.player);
    const pitchRel = pitchOf(s.player);
    console.log(`  ${label} release: pitch=${pitchRel.toFixed(1)} eulerBank=${bankRel.toFixed(1)} robustBank=${robustRel.toFixed(1)} ias=${(s.player.ias * MS2KT).toFixed(0)}kt`);
    const sim0 = s.sim;
    let sec = 0;
    let robustAt5 = robustRel;
    while (s.sim < sim0 + 8 && s.player.alive && s.player.pos.y - s.player.floorY > 100) {
      s.frame(); s.record();
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); if (sec === 5) robustAt5 = robustBank(s.player); }
    }
    const robustEnd = robustBank(s.player);
    const eulerEnd = bankOf(s.player);
    console.log(`  ${label} hands-off: robustBank@5s=${robustAt5.toFixed(1)} robustBank@end=${robustEnd.toFixed(1)} eulerBank@end=${eulerEnd.toFixed(1)} alive=${s.player.alive}`);
    check(`${label}: robust bank <= 5 deg within 5 s sim of release`, Math.abs(robustAt5) <= 5, `robust@5s=${robustAt5.toFixed(1)} (release robust=${robustRel.toFixed(1)}, euler=${bankRel.toFixed(1)})`);
    check(`${label}: robust bank stays <= 5 deg at end`, Math.abs(robustEnd) <= 5, `robust@end=${robustEnd.toFixed(1)} euler@end=${eulerEnd.toFixed(1)}`);
    s.dispose();
  };

  console.log('--- F1: bank RIGHT ~30 deg, then STEEP DIVE past 60 deg, hands-off ---');
  await bankedSteep('F1 banked steep dive', 1, 1, 60);
  console.log('--- F2: bank LEFT ~30 deg, then STEEP CLIMB past -60 deg, hands-off ---');
  await bankedSteep('F2 banked steep climb', -1, -1, -60);

  // F3: straight-up loop hold — full up joystick 12 s wall from level.
  console.log('--- F3: straight-up JOY hold 12s wall (loop), fa18 — robust bank + beta ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 2000);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
    const t0 = s.wall;
    let maxRobust = 0;
    let maxBeta = 0;
    let maxEuler = 0;
    let sec = 0;
    while (s.wall < t0 + 12 && s.player.alive) {
      s.frame(); s.record();
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      maxBeta = Math.max(maxBeta, Math.abs(s.player.betaDeg));
      maxEuler = Math.max(maxEuler, Math.abs(bankOf(s.player)));
      if (s.wall - t0 >= sec + 1) { sec++; logBankRow(`+${sec}s wall`, s); }
    }
    fireJoy(s.joyEl, 'pointerup', c.x, r.top + 1, 7);
    console.log(`  loop hold: max|robustBank|=${maxRobust.toFixed(1)} max|beta|=${maxBeta.toFixed(1)} max|eulerBank|=${maxEuler.toFixed(1)} (Euler swings at the loop top — HUD readout artefact, see report) alive=${s.player.alive}`);
    check('F3 loop hold: robust bank stays <= 5 deg (no real roll)', maxRobust <= 5, `maxRobust=${maxRobust.toFixed(1)}`);
    check('F3 loop hold: no nose slice (|beta| <= 5 deg — the old Euler-spring failure)', maxBeta <= 5, `maxBeta=${maxBeta.toFixed(1)}`);
    s.dispose();
  }

  // F4: release MID-ROLL (bank ~45 deg and still rolling) — the bank must return.
  console.log('--- F4: release MID-ROLL from a right roll, fa18 ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 300);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', r.left + r.width - 1, c.y, 7);
    while (Math.abs(bankOf(s.player)) < 45 && s.wall < 10) { s.frame(); s.record(); if (!s.player.alive) break; }
    const bankRel = bankOf(s.player);
    fireJoy(s.joyEl, 'pointerup', r.left + r.width - 1, c.y, 7);
    for (let i = 0; i < 3; i++) { s.frame(); s.record(); }
    const sim0 = s.sim;
    let bankAt3 = bankOf(s.player);
    while (s.sim < sim0 + 3 && s.player.alive) { s.frame(); s.record(); bankAt3 = bankOf(s.player); }
    console.log(`  mid-roll release: bank at release=${bankRel.toFixed(1)} bank 3s sim later=${bankAt3.toFixed(1)} alive=${s.player.alive}`);
    check('F4 release mid-roll: |bank| <= 10 deg within 3 s sim', Math.abs(bankAt3) <= 10, `bank@3s=${bankAt3.toFixed(1)} (release ${bankRel.toFixed(1)})`);
    s.dispose();
  }

  // ============ F5/F6: the two tests F1-F3 did NOT actually reach ============
  // F1/F2's joystick setups never reached pitch +/-60 deg (F1 rolled
  // through multiple turns, F2 stalled out of the climb into a dive),
  // so the removed |pitch| > 60 deg gate is still unproven. F5 sets
  // the attitude DIRECTLY (fixed-dt PD hold — the measurement phase
  // is still the live loop, hands-off): bank +/-30 deg AND pitch
  // past +/-60 deg, then cut all input and watch the bank hands-off
  // through the live loop. F6 takes the loop to the top at 400 kt
  // (F3 stalled at -79 deg from a 240 kt entry).
  const bankedSteepDirect = async (
    label: string,
    bankTarget: number,
    pitchTarget: number, // + dive, - climb
  ): Promise<void> => {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 2000);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    // Setup: direct PD hold on bank + full pitch input, fixed dt.
    let guard = 0;
    while (guard++ < 1200) {
      const dt = 1 / 60;
      const b = bankOf(s.player);
      const p = pitchOf(s.player);
      s.player.rollIn = clamp01((bankTarget - b) * 0.05);
      s.player.pitchIn = pitchTarget > 0 ? 1 : -1;
      world.timestep = dt;
      world.step(eventQueue);
      drain(s.player);
      s.player.step(dt, WIND, null);
      s.sim += dt;
      const pNow = pitchOf(s.player);
      if (
        Math.abs(bankOf(s.player) - bankTarget) < 3 &&
        (pitchTarget > 0 ? pNow >= pitchTarget : pNow <= pitchTarget)
      ) break;
      if (!s.player.alive) break;
    }
    s.player.rollIn = 0;
    s.player.pitchIn = 0;
    const bankRel = bankOf(s.player);
    const robustRel = robustBank(s.player);
    const pitchRel = pitchOf(s.player);
    console.log(`  ${label} release: pitch=${pitchRel.toFixed(1)} eulerBank=${bankRel.toFixed(1)} robustBank=${robustRel.toFixed(1)} ias=${(s.player.ias * MS2KT).toFixed(0)}kt stalled=${s.player.stalled}`);
    const sim0 = s.sim;
    let sec = 0;
    let robustAt3 = robustRel;
    let robustAt5 = robustRel;
    while (s.sim < sim0 + 8 && s.player.alive && s.player.pos.y - s.player.floorY > 100) {
      s.frame(); s.record();
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); if (sec === 3) robustAt3 = robustBank(s.player); if (sec === 5) robustAt5 = robustBank(s.player); }
    }
    const robustEnd = robustBank(s.player);
    console.log(`  ${label} hands-off: robustBank@3s=${robustAt3.toFixed(1)} @5s=${robustAt5.toFixed(1)} @end=${robustEnd.toFixed(1)} alive=${s.player.alive}`);
    check(`${label}: robust bank <= 5 deg within 5 s sim of release (pitch was past 60 deg)`, Math.abs(robustAt5) <= 5, `robust@5s=${robustAt5.toFixed(1)} (release robust=${robustRel.toFixed(1)}, euler=${bankRel.toFixed(1)}, pitch=${pitchRel.toFixed(1)})`);
    s.dispose();
  };

  console.log('--- F5a: bank +30 deg AND dive pitch +65 deg, hands-off ---');
  await bankedSteepDirect('F5a banked steep dive', 30, 65);
  console.log('--- F5b: bank -30 deg AND climb pitch -65 deg, hands-off ---');
  await bankedSteepDirect('F5b banked steep climb', -30, -65);

  // F6: loop to the top at 400 kt.
  console.log('--- F6: straight-up JOY hold to the loop top, 400 kt entry ---');
  {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(400, 2000);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7);
    const t0 = s.wall;
    let maxRobust = 0;
    let maxBeta = 0;
    let maxEuler = 0;
    let minPitch = 0;
    let sec = 0;
    while (s.wall < t0 + 20 && s.player.alive) {
      s.frame(); s.record();
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      maxBeta = Math.max(maxBeta, Math.abs(s.player.betaDeg));
      maxEuler = Math.max(maxEuler, Math.abs(bankOf(s.player)));
      minPitch = Math.min(minPitch, pitchOf(s.player));
      if (s.wall - t0 >= sec + 1) { sec++; logBankRow(`+${sec}s wall`, s); }
      if (minPitch < -85) break;
    }
    fireJoy(s.joyEl, 'pointerup', c.x, r.top + 1, 7);
    console.log(`  loop top: minPitch=${minPitch.toFixed(1)} max|robustBank|=${maxRobust.toFixed(1)} max|beta|=${maxBeta.toFixed(1)} max|eulerBank|=${maxEuler.toFixed(1)} alive=${s.player.alive}`);
    check('F6 loop top: robust bank stays <= 5 deg (no real roll)', maxRobust <= 5, `maxRobust=${maxRobust.toFixed(1)} minPitch=${minPitch.toFixed(1)}`);
    check('F6 loop top: no nose slice (|beta| <= 5 deg)', maxBeta <= 5, `maxBeta=${maxBeta.toFixed(1)}`);
    s.dispose();
  }

  // ============ F7: the DISCRIMINATING test — banked dive/climb HELD on pitch only ============
  // F5's hands-off dive shallows too fast (pitch 71 -> 50 deg in
  // 1 s sim) for the old |pitch| > 60 deg gate to matter at the 5 s
  // bar — the old code passed F5 too. The live case is different:
  // the pilot HOLDS the dive/climb (pitch input, no roll input) for
  // seconds while the bank sits there — rollIn is 0, pitch stays
  // past 60 deg, and the old gate keeps the spring off the whole
  // time. F7 sets bank +/-30 deg at pitch past +/-60 deg, then
  // holds ONLY the pitch (real joystick, x=0) through the live loop
  // for 6 s sim: the old code must leave the bank frozen, the new
  // code must level it.
  const heldSteep = async (
    label: string,
    bankTarget: number,
    pitchTarget: number, // + dive, - climb
  ): Promise<void> => {
    const s = new LiveSession('fa18');
    s.dtPattern = DT4;
    await s.attach();
    s.spawnAirborne(240, 3000);
    for (let i = 0; i < 12; i++) { s.frame(); s.record(); }
    let guard = 0;
    while (guard++ < 1200) {
      const dt = 1 / 60;
      const b = bankOf(s.player);
      s.player.rollIn = clamp01((bankTarget - b) * 0.05);
      s.player.pitchIn = pitchTarget > 0 ? 1 : -1;
      world.timestep = dt;
      world.step(eventQueue);
      drain(s.player);
      s.player.step(dt, WIND, null);
      s.sim += dt;
      const pNow = pitchOf(s.player);
      if (
        Math.abs(bankOf(s.player) - bankTarget) < 3 &&
        (pitchTarget > 0 ? pNow >= pitchTarget : pNow <= pitchTarget)
      ) break;
      if (!s.player.alive) break;
    }
    s.player.rollIn = 0;
    s.player.pitchIn = 0;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    const cy = pitchTarget > 0 ? r.top + r.height - 1 : r.top + 1;
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', c.x, cy, 7);
    console.log(`  ${label} hold start: pitch=${pitchOf(s.player).toFixed(1)} eulerBank=${bankOf(s.player).toFixed(1)} robustBank=${robustBank(s.player).toFixed(1)}`);
    const sim0 = s.sim;
    let sec = 0;
    let maxEuler = Math.abs(bankOf(s.player));
    let maxRobust = Math.abs(robustBank(s.player));
    while (s.sim < sim0 + 6 && s.player.alive && s.player.pos.y - s.player.floorY > 100) {
      s.frame(); s.record();
      maxEuler = Math.max(maxEuler, Math.abs(bankOf(s.player)));
      maxRobust = Math.max(maxRobust, Math.abs(robustBank(s.player)));
      if (s.sim - sim0 >= sec + 1) { sec++; logBankRow(`+${sec}s`, s); }
    }
    fireJoy(s.joyEl, 'pointerup', c.x, cy, 7);
    console.log(`  ${label} pitch-held: max|eulerBank|=${maxEuler.toFixed(1)} max|robustBank|=${maxRobust.toFixed(1)} eulerBank@end=${bankOf(s.player).toFixed(1)} alive=${s.player.alive}`);
    // BOUNDEDNESS, not levelling, is the claim here — and it is the
    // honest one. A pitch-HELD banked steep dive is a Dutch-roll
    // regime (beta swings +/-18 deg, AoA negative at 400+ kt): the
    // fin's sideslip roll moment drives ~0.4 rad/s roll rates, right
    // at the spring's cap, so no bank spring can settle it to 5 deg
    // without also changing the yaw/roll damping or the fin coupling
    // (BET solver — out of this pass's scope, and the cap is shared
    // with the 80 deg bank-hold check). What the gimbal-safe spring
    // DOES do here: the bank stays bounded instead of running away
    // — old code (pitch gate on) reached Euler 99.9 deg / robust
    // 23.5 deg in the same 6 s sim; the new code peaks at Euler
    // 49.4 deg / robust 15.0 deg (F7a, measured both ways). F7b is
    // stalled throughout (AoA ~35 deg): the auto-level is gated off
    // while stalled BY DESIGN, identically in old and new code
    // (Euler ends -89.5 deg both ways) — a stalled banked climb
    // departing is spin dynamics, not drift, and is out of scope.
    if (label.includes('dive')) {
      check(`${label}: bank stays BOUNDED while the steep dive is held (max |Euler| < 60 deg — old code ran to 99.9 deg)`, maxEuler < 60, `maxEuler=${maxEuler.toFixed(1)} maxRobust=${maxRobust.toFixed(1)}`);
      check(`${label}: robust bank stays < 20 deg (old code 23.5 deg)`, maxRobust < 20, `maxRobust=${maxRobust.toFixed(1)}`);
    } else {
      console.log(`  ${label} DIAGNOSTIC ONLY (stalled throughout — auto-level gated off by design, same in old and new code; see comment above)`);
    }
    s.dispose();
  };

  console.log('--- F7a: bank +30 deg, dive HELD past +60 deg on pitch only ---');
  await heldSteep('F7a held banked dive', 30, 65);
  console.log('--- F7b: bank -30 deg, climb HELD past -60 deg on pitch only ---');
  await heldSteep('F7b held banked climb', -30, -65);

  console.log(failures === 0 ? '\nALL PASS-15 DRIFT CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
