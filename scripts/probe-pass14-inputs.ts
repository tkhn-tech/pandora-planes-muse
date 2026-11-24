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

const WIND = new THREE.Vector3(3, 0, 0); // live wind: 3 m/s
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
  const DT_PATTERN = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
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
      const rawDt = Math.min(DT_PATTERN[this.pi % DT_PATTERN.length]!, 0.25);
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

  // ============ S1: KB tap Right 100 ms at 223 kt, post-takeoff regime ============
  // The v4.8 live repro, F-18. Tap = keydown at frame start, keyup
  // 100 ms of WALL time later (at 4-9 fps this is inside one frame).
  {
    console.log('--- S1: KB tap Right 100ms, F-18, post-takeoff ~223kt ---');
    const s = await takeoff('fa18', 0.9, 215, 235);
    if (s) {
      const t0 = s.wall;
      const bank0 = bankOf(s.player);
      const hdg0 = hdgOf(s.player);
      let tapped = false;
      let released = false;
      const testTrace: TraceRow[] = [];
      while (s.wall < t0 + 3) {
        s.frame((ss) => {
          if (!tapped) {
            fireKey('keydown', 'ArrowRight');
            tapped = true;
          } else if (!released && ss.wall - t0 >= 0.1) {
            fireKey('keyup', 'ArrowRight');
            released = true;
          }
        });
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      const peak = peakBank(testTrace);
      const at05 = atWall(testTrace, t0 + 0.5);
      const at20 = atWall(testTrace, t0 + 2.0);
      console.log(
        `  tap: peakRollIn=${maxAbs(testTrace, (r) => r.rollIn).toFixed(2)} peakPcRoll=${maxAbs(testTrace, (r) => r.pcRoll).toFixed(2)} peakBank=${peak.toFixed(1)} bank@+0.5s=${at05.bank.toFixed(1)} bank@+2s=${at20.bank.toFixed(1)} bank0=${bank0.toFixed(1)} hdg ${hdg0.toFixed(0)}->${at20.hdg.toFixed(0)} alive=${s.player.alive}`,
      );
      console.log(
        `  tap window regime: onSurface frames=${testTrace.filter((r) => r.onSurface).length}/${testTrace.length} agl range=[${Math.min(...testTrace.map((r) => r.agl)).toFixed(1)}, ${Math.max(...testTrace.map((r) => r.agl)).toFixed(1)}]m pitch range=[${Math.min(...testTrace.map((r) => r.pitch)).toFixed(1)}, ${Math.max(...testTrace.map((r) => r.pitch)).toFixed(1)}] aoa range=[${Math.min(...testTrace.map((r) => r.aoa)).toFixed(1)}, ${Math.max(...testTrace.map((r) => r.aoa)).toFixed(1)}]`,
      );
      check(
        'S1 tap Right 100ms @223kt (takeoff regime): peak bank 3-12 deg (sane tap)',
        peak >= 3 && peak <= 12,
        `peak bank=${peak.toFixed(1)} deg, peak rollIn=${maxAbs(testTrace, (r) => r.rollIn).toFixed(2)}`,
      );
      check(
        'S1 tap: input chain produces stick (peak |pcRoll| >= 0.2)',
        maxAbs(testTrace, (r) => r.pcRoll) >= 0.2,
        `peak pcRoll=${maxAbs(testTrace, (r) => r.pcRoll).toFixed(2)}`,
      );
      s.dispose();
    } else {
      check('S1 tap Right 100ms @223kt (takeoff regime): peak bank 3-12 deg (sane tap)', false, 'takeoff did not reach the band');
      check('S1 tap: input chain produces stick (peak |pcRoll| >= 0.2)', false, 'takeoff did not reach the band');
    }
  }

  // ============ S4: KB hold Right 1 s at ~223 kt, post-takeoff regime ============
  {
    console.log('--- S4: KB hold Right 1s, F-18, post-takeoff ~223kt ---');
    const s = await takeoff('fa18', 0.9, 215, 235);
    if (s) {
      const t0 = s.wall;
      const t0sim = s.sim;
      let pressed = false;
      let released = false;
      const testTrace: TraceRow[] = [];
      // Hold 1 s of WALL time, then keep running hands-off until 1 s
      // of SIM time has passed under the hold: at 4-9 fps the spiral
      // breaker caps sim at 0.10 s/frame, so 1 s wall is only ~0.6 s
      // sim — the "1 s hold" the jet feels is the sim second.
      while (s.sim < t0sim + 1.5) {
        s.frame((ss) => {
          if (!pressed) {
            fireKey('keydown', 'ArrowRight');
            pressed = true;
          } else if (!released && ss.wall - t0 >= 1.0) {
            fireKey('keyup', 'ArrowRight');
            released = true;
          }
        });
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      const at1sim = testTrace.reduce((a, b) =>
        Math.abs(b.sim - (t0sim + 1.0)) < Math.abs(a.sim - (t0sim + 1.0)) ? b : a,
      );
      const at1wall = atWall(testTrace, t0 + 1.0);
      console.log(
        `  hold 1s: peakRollIn=${maxAbs(testTrace, (r) => r.rollIn).toFixed(2)} bank@1s-sim=${at1sim.bank.toFixed(1)} bank@1s-wall=${at1wall.bank.toFixed(1)} peakBank=${peakBank(testTrace).toFixed(1)} ias@1s-sim=${at1sim.iasKt.toFixed(0)}kt alive=${s.player.alive}`,
      );
      check(
        'S4 hold Right 1s @223kt (takeoff regime): bank@1s-sim >= 20 deg',
        Math.abs(at1sim.bank) >= 20,
        `bank@1s-sim=${at1sim.bank.toFixed(1)} deg (bank@1s-wall=${at1wall.bank.toFixed(1)} deg — sim runs ~0.6x wall at 4-9 fps)`,
      );
      s.dispose();
    } else {
      check('S4 hold Right 1s @223kt (takeoff regime): bank@1s-sim >= 20 deg', false, 'takeoff did not reach the band');
    }
  }

  // ============ S5: JOY full-right hold, post-takeoff regime (THE live repro) ============
  // Joystick held full-right for 33 s of WALL time via real pointer
  // events on the fake #joy element. Live: bank stayed 0 deg, heading
  // 109 -> 112 deg.
  {
    console.log('--- S5: JOY full-right hold 33s wall, F-18, post-takeoff ~210-270kt ---');
    const s = await takeoff('fa18', 0.9, 210, 270);
    if (s) {
      const t0 = s.wall;
      const hdg0 = hdgOf(s.player);
      const c = joyCenter(s.joyEl);
      const r = s.joyEl.getBoundingClientRect();
      let down = false;
      const testTrace: TraceRow[] = [];
      while (s.wall < t0 + 33) {
        s.frame((ss) => {
          if (!down) {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', r.left + r.width - 1, c.y, 7);
            down = true;
          }
          void ss;
        });
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      fireJoy(s.joyEl, 'pointerup', r.left + r.width - 1, c.y, 7);
      const last = testTrace[testTrace.length - 1]!;
      const at3 = atWall(testTrace, t0 + 3);
      const at10 = atWall(testTrace, t0 + 10);
      console.log(
        `  joy hold: joyActive=${last.joyActive} joyX~${maxAbs(testTrace, (x) => x.joyX).toFixed(2)} peakRollIn=${maxAbs(testTrace, (x) => x.rollIn).toFixed(2)} meanRollIn(1-3s)=${(testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).reduce((a, x) => a + x.rollIn, 0) / Math.max(1, testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).length)).toFixed(2)}`,
      );
      console.log(
        `  bank@3s=${at3.bank.toFixed(1)} bank@10s=${at10.bank.toFixed(1)} bank@end=${last.bank.toFixed(1)} peakBank=${peakBank(testTrace).toFixed(1)} hdg ${hdg0.toFixed(0)}->${last.hdg.toFixed(0)} (dHdg=${(((last.hdg - hdg0 + 540) % 360) - 180).toFixed(1)}) iasEnd=${last.iasKt.toFixed(0)}kt aglEnd=${last.agl.toFixed(0)}m alive=${s.player.alive}`,
      );
      console.log(
        `  regime during hold: onSurface frames=${testTrace.filter((x) => x.onSurface).length}/${testTrace.length} stalled frames=${testTrace.filter((x) => x.stalled).length}/${testTrace.length} agl range=[${Math.min(...testTrace.map((x) => x.agl)).toFixed(0)}, ${Math.max(...testTrace.map((x) => x.agl)).toFixed(0)}]m aoa range=[${Math.min(...testTrace.map((x) => x.aoa)).toFixed(1)}, ${Math.max(...testTrace.map((x) => x.aoa)).toFixed(1)}]`,
      );
      for (let sec = 1; sec <= 12; sec++) {
        const row = testTrace.find((x) => x.wall >= t0 + sec);
        if (row) {
          console.log(
            `    +${sec}s wall: bank=${row.bank.toFixed(1)} hdg=${row.hdg.toFixed(1)} pitch=${row.pitch.toFixed(1)} ias=${row.iasKt.toFixed(0)}kt agl=${row.agl.toFixed(0)}m rollIn=${row.rollIn.toFixed(2)} alive=${s.player.alive}`,
          );
        }
      }
      check(
        'S5 JOY full-right hold (takeoff regime): bank reaches >= 20 deg within 3 s',
        Math.abs(at3.bank) >= 20,
        `bank@3s=${at3.bank.toFixed(1)} deg (live: 0 deg over 33 s)`,
      );
      // The 3 s bar is 20 deg, not 30: the takeoff-regime roll
      // response is knife-edge in the old skim regime — measured
      // 40.3 deg with the rotation pull (pass 14 coordinator;
      // 25.3 deg in the pre-fix skim run), far above live's 0 deg.
      // The 10 s bar below is the stable assertion: the jet keeps
      // rolling to 152 deg and dies inverted.
      check(
        'S5 JOY full-right hold (takeoff regime): bank reaches >= 60 deg within 10 s',
        Math.abs(at10.bank) >= 60,
        `bank@10s=${at10.bank.toFixed(1)} deg (earlier run: 84.6 deg)`,
      );
      // Heading is NOT asserted here: the hands-off jet skims at AGL
      // ~2 m with AoA ~0, so its lift is ~0.3 g — banking tilts
      // almost nothing, and the jet rolls without turning (dHdg
      // -0.7 deg while banking to 83 deg). The turn check belongs at
      // altitude, where lift exists — see S5b.
      check(
        'S5 JOY hold: merged rollIn is live (mean |rollIn| 1-3 s >= 0.15)',
        Math.abs(testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).reduce((a, x) => a + x.rollIn, 0) / Math.max(1, testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).length)) >= 0.15,
        `mean rollIn 1-3s=${(testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).reduce((a, x) => a + x.rollIn, 0) / Math.max(1, testTrace.filter((x) => x.wall >= t0 + 1 && x.wall <= t0 + 3).length)).toFixed(2)}`,
      );
      s.dispose();
    } else {
      check('S5 JOY full-right hold (takeoff regime): bank reaches >= 20 deg within 3 s', false, 'takeoff did not reach the band');
      check('S5 JOY full-right hold (takeoff regime): bank reaches >= 60 deg within 10 s', false, 'takeoff did not reach the band');
      check('S5 JOY hold: merged rollIn is live (mean |rollIn| 1-3 s >= 0.15)', false, 'takeoff did not reach the band');
    }
  }

  // ============ S5b: JOY full-right hold at altitude (control) ============
  {
    console.log('--- S5b: JOY full-right hold 3s, F-18, 300 m / 240 kt (altitude control) ---');
    const s = new LiveSession('fa18');
    await s.attach();
    s.spawnAirborne(240, 300);
    for (let i = 0; i < 12; i++) {
      s.frame();
      s.record();
    }
    const t0 = s.wall;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
    fireJoy(s.joyEl, 'pointermove', r.left + r.width - 1, c.y, 7);
    const testTrace: TraceRow[] = [];
    // 6 s wall (~3.5 s sim): the turn needs sim time to develop —
    // at 240 kt / 40 deg bank the turn rate is ~3.8 deg/s of sim.
    while (s.wall < t0 + 6) {
      s.frame();
      s.record();
      testTrace.push(s.trace[s.trace.length - 1]!);
      if (!s.player.alive) break;
    }
    const at3 = atWall(testTrace, t0 + 3);
    const lastB = testTrace[testTrace.length - 1]!;
    const hdgB0 = testTrace[0]!.hdg;
    console.log(
      `  joy@alt: joyX~${maxAbs(testTrace, (x) => x.joyX).toFixed(2)} meanRollIn=${(testTrace.reduce((a, x) => a + x.rollIn, 0) / testTrace.length).toFixed(2)} bank@3s=${at3.bank.toFixed(1)} peakBank=${peakBank(testTrace).toFixed(1)} hdg ${hdgB0.toFixed(1)}->${lastB.hdg.toFixed(1)} alive=${s.player.alive}`,
    );
    check(
      'S5b JOY full-right hold (altitude control): bank@3s >= 30 deg',
      Math.abs(at3.bank) >= 30,
      `bank@3s=${at3.bank.toFixed(1)} deg`,
    );
    check(
      'S5b JOY hold at altitude: heading turns >= 8 deg over 6 s wall (lift exists here)',
      Math.abs(((lastB.hdg - hdgB0 + 540) % 360) - 180) >= 8,
      `dHdg=${(((lastB.hdg - hdgB0 + 540) % 360) - 180).toFixed(1)} deg (~3.8 deg/s of sim at 240 kt / 40 deg bank)`,
    );
    s.dispose();
  }

  // ============ S6: JOY full-DOWN pitch hold, post-takeoff regime ============
  {
    console.log('--- S6: JOY full-down pitch hold 10s wall, F-18, post-takeoff ~210-270kt ---');
    const s = await takeoff('fa18', 0.9, 210, 270);
    if (s) {
      const t0 = s.wall;
      const pitch0 = pitchOf(s.player);
      const c = joyCenter(s.joyEl);
      const r = s.joyEl.getBoundingClientRect();
      let down = false;
      const testTrace: TraceRow[] = [];
      while (s.wall < t0 + 10) {
        s.frame(() => {
          if (!down) {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, r.top + r.height - 1, 7);
            down = true;
          }
        });
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      const last = testTrace[testTrace.length - 1]!;
      const meanPitchIn =
        testTrace.reduce((a, x) => a + x.pitchIn, 0) / testTrace.length;
      console.log(
        `  pitch hold: pitch ${pitch0.toFixed(1)} -> ${last.pitch.toFixed(1)} (dPitch=${(last.pitch - pitch0).toFixed(1)}) peakPitchIn=${maxAbs(testTrace, (x) => x.pitchIn).toFixed(2)} meanPitchIn=${meanPitchIn.toFixed(2)} aoa ${testTrace[0]!.aoa.toFixed(1)}->${last.aoa.toFixed(1)} alive=${s.player.alive}`,
      );
      // DIAGNOSTIC, not an attitude band: in the skimming regime the
      // controller's low-altitude nose-down clamp ("cannot pitch
      // below level" under 7 m AGL, FlightPhysicsController — NOT a
      // Worker A file) holds the jet at level whatever the stick
      // says. What Worker A owns is the STICK: it must be live. The
      // attitude response to the same input at altitude is E2b.
      check(
        'S6 JOY full-down hold (takeoff regime): stick is live (mean pitchIn >= 0.25)',
        meanPitchIn >= 0.25,
        `mean pitchIn=${meanPitchIn.toFixed(2)} (attitude dPitch=${(last.pitch - pitch0).toFixed(1)} deg is clamp-limited here — see report)`,
      );
      s.dispose();
    } else {
      check('S6 JOY full-down hold (takeoff regime): stick is live (mean pitchIn >= 0.25)', false, 'takeoff did not reach the band');
    }
  }

  // ============ S7: KB ArrowDown hold 5 s, post-takeoff regime ============
  {
    console.log('--- S7: KB ArrowDown hold 5s wall, F-18, post-takeoff ~210-270kt ---');
    const s = await takeoff('fa18', 0.9, 210, 270);
    if (s) {
      const t0 = s.wall;
      const pitch0 = pitchOf(s.player);
      let pressed = false;
      let released = false;
      const testTrace: TraceRow[] = [];
      while (s.wall < t0 + 5) {
        s.frame((ss) => {
          if (!pressed) {
            fireKey('keydown', 'ArrowDown');
            pressed = true;
          } else if (!released && ss.wall - t0 >= 5.0) {
            fireKey('keyup', 'ArrowDown');
            released = true;
          }
        });
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      const last = testTrace[testTrace.length - 1]!;
      const meanPitchIn =
        testTrace.reduce((a, x) => a + x.pitchIn, 0) / testTrace.length;
      console.log(
        `  ArrowDown hold: pitch ${pitch0.toFixed(1)} -> ${last.pitch.toFixed(1)} (dPitch=${(last.pitch - pitch0).toFixed(1)}) peakPcPitch=${maxAbs(testTrace, (x) => x.pcPitch).toFixed(2)} meanPitchIn=${meanPitchIn.toFixed(2)} alive=${s.player.alive}`,
      );
      // DIAGNOSTIC (see S6): the stick must be live; the attitude in
      // the skimming regime is held at level by the controller's
      // low-altitude nose-down clamp (FlightPhysicsController, not a
      // Worker A file). The same input at altitude is E2a.
      check(
        'S7 KB ArrowDown hold 5s (takeoff regime): stick is live (mean pitchIn >= 0.9)',
        meanPitchIn >= 0.9,
        `mean pitchIn=${meanPitchIn.toFixed(2)} (attitude dPitch=${(last.pitch - pitch0).toFixed(1)} deg is clamp-limited here — see report)`,
      );
      s.dispose();
    } else {
      check('S7 KB ArrowDown hold 5s (takeoff regime): stick is live (mean pitchIn >= 0.9)', false, 'takeoff did not reach the band');
    }
  }

  // ============ S2/S3: KB taps at 524 kt (F-35A) and 776 kt (F-18), altitude ============
  // The live taps at speed happened in flight at altitude; the
  // takeoff regime cannot reach 524/776 kt, so these use the
  // trimmed 300 m spawn — but through the FAITHFUL loop (4-9 fps,
  // sim cap, wall-clock polling), which probe-roll F does not have.
  for (const [key, kt] of [['f35a', 524], ['fa18', 776]] as const) {
    console.log(`--- S2/S3: KB tap Right 100ms, ${key}, ${kt}kt at 300 m (faithful loop) ---`);
    const s = new LiveSession(key);
    await s.attach();
    s.spawnAirborne(kt, 300);
    for (let i = 0; i < 12; i++) {
      s.frame();
      s.record();
    }
    const t0 = s.wall;
    let tapped = false;
    let released = false;
    const testTrace: TraceRow[] = [];
    while (s.wall < t0 + 2) {
      s.frame((ss) => {
        if (!tapped) {
          fireKey('keydown', 'ArrowRight');
          tapped = true;
        } else if (!released && ss.wall - t0 >= 0.1) {
          fireKey('keyup', 'ArrowRight');
          released = true;
        }
      });
      s.record();
      testTrace.push(s.trace[s.trace.length - 1]!);
      if (!s.player.alive) break;
    }
    const peak = peakBank(testTrace);
    const at20 = atWall(testTrace, t0 + 2.0);
    console.log(
      `  tap@${kt}kt: peakRollIn=${maxAbs(testTrace, (r) => r.rollIn).toFixed(2)} peakPcRoll=${maxAbs(testTrace, (r) => r.pcRoll).toFixed(2)} peakBank=${peak.toFixed(1)} bank@+2s=${at20.bank.toFixed(1)} alive=${s.player.alive}`,
    );
    check(
      `tap Right 100ms (${key} ${kt}kt, faithful loop): peak bank 3-12 deg`,
      peak >= 3 && peak <= 12,
      `peak bank=${peak.toFixed(1)} deg (live: 0 deg)`,
    );
    s.dispose();
  }

  // ============ E2: pitch authority at ALTITUDE (control) ============
  // Same full-down inputs as S6/S7, but at 300 m where the low-
  // altitude nose-down clamp (AGL < 7 m) and the onSurface rotation
  // gate cannot apply. If pitch drops freely here, the S6/S7
  // weakness is the low-altitude regime, not the stick.
  for (const [label, kind] of [
    ['E2a KB ArrowDown hold 5s', 'kb'],
    ['E2b JOY full-down hold 5s', 'joy'],
  ] as const) {
    console.log(`--- ${label}, F-18, 300 m / 240 kt ---`);
    const s = new LiveSession('fa18');
    await s.attach();
    s.spawnAirborne(240, 300);
    for (let i = 0; i < 12; i++) {
      s.frame();
      s.record();
    }
    const t0 = s.wall;
    const pitch0 = pitchOf(s.player);
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    let started = false;
    const testTrace: TraceRow[] = [];
    while (s.wall < t0 + 5) {
      s.frame(() => {
        if (!started) {
          if (kind === 'kb') fireKey('keydown', 'ArrowDown');
          else {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, r.top + r.height - 1, 7);
          }
          started = true;
        }
      });
      s.record();
      testTrace.push(s.trace[s.trace.length - 1]!);
      if (!s.player.alive) break;
    }
    const last = testTrace[testTrace.length - 1]!;
    console.log(
      `  ${label}: pitch ${pitch0.toFixed(1)} -> ${last.pitch.toFixed(1)} (dPitch=${(last.pitch - pitch0).toFixed(1)}, + = nose DOWN) meanPitchIn=${(testTrace.reduce((a, x) => a + x.pitchIn, 0) / testTrace.length).toFixed(2)} aglEnd=${last.agl.toFixed(0)}m alive=${s.player.alive}`,
    );
    check(
      `${label} at altitude: pitch drops >= 10 deg nose-down`,
      last.pitch - pitch0 >= 10,
      `dPitch=${(last.pitch - pitch0).toFixed(1)} deg (Euler x: + = nose down)`,
    );
    s.dispose();
  }

  // ============ E3: pitch-down from high AoA / 28 deg nose-up ============
  // The live lock: pitch ~28 deg nose-up, AoA ~21 deg, full-down
  // holds did nothing. Reproduce the attitude at altitude, then hold
  // ArrowDown 5 s through the live loop.
  {
    console.log('--- E3: KB ArrowDown hold 5s from 28 deg nose-up / AoA ~20, F-18, 300 m ---');
    const s = new LiveSession('fa18');
    await s.attach();
    s.spawnAirborne(210, 300);
    // Setup: PD-hold Euler pitch at -28 deg (nose-up) with direct
    // pitchIn, fixed dt, until AoA builds to ~20 deg.
    for (let i = 0; i < 600 && s.player.alive; i++) {
      const pNow = pitchOf(s.player);
      s.player.rollIn = 0;
      s.player.pitchIn = clamp01((-28 - pNow) * 0.1);
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(s.player);
      s.player.step(dt, WIND, null);
      if (Math.abs(pNow + 28) < 1 && s.player.aoaDeg > 18) break;
    }
    const pitch0 = pitchOf(s.player);
    const aoa0 = s.player.aoaDeg;
    console.log(
      `  setup: pitch=${pitch0.toFixed(1)} aoa=${aoa0.toFixed(1)} ias=${(s.player.ias * MS2KT).toFixed(0)}kt stalled=${s.player.stalled}`,
    );
    const t0 = s.wall;
    let pressed = false;
    const testTrace: TraceRow[] = [];
    while (s.wall < t0 + 5) {
      s.frame(() => {
        if (!pressed) {
          fireKey('keydown', 'ArrowDown');
          pressed = true;
        }
      });
      s.record();
      testTrace.push(s.trace[s.trace.length - 1]!);
      if (!s.player.alive) break;
    }
    fireKey('keyup', 'ArrowDown');
    const last = testTrace[testTrace.length - 1]!;
    console.log(
      `  ArrowDown from high AoA: pitch ${pitch0.toFixed(1)} -> ${last.pitch.toFixed(1)} (dPitch=${(last.pitch - pitch0).toFixed(1)}, + = nose DOWN) aoa ${aoa0.toFixed(1)}->${last.aoa.toFixed(1)} meanPitchIn=${(testTrace.reduce((a, x) => a + x.pitchIn, 0) / testTrace.length).toFixed(2)} alive=${s.player.alive}`,
    );
    check(
      'E3 ArrowDown from 28 deg nose-up / AoA ~20: pitch drops >= 10 deg nose-down',
      last.pitch - pitch0 >= 10,
      `dPitch=${(last.pitch - pitch0).toFixed(1)} deg (live: locked ~28 deg)`,
    );
    s.dispose();
  }

  // ============ E4: roll input while firmly ON THE GROUND ============
  // During the takeoff roll (~150 kt, onSurface true), hold the
  // joystick full-right 3 s wall. The controller's ground branch
  // overwrites rollRate with the gear-leveling spring — if the bank
  // stays ~0 here while the same stick rolls the jet airborne (S5),
  // a live test done before liftoff reads as "dead roll".
  {
    console.log('--- E4: JOY full-right hold 3s during takeoff roll (~150 kt, onSurface), F-18 ---');
    const s = new LiveSession('fa18');
    await s.attach();
    s.spawnAlpha();
    s.player.thrTarget = 0.9;
    // Roll until 140-160 kt and onSurface.
    let inWindow = false;
    while (s.wall < 40 && !inWindow) {
      s.frame();
      s.record();
      const p = s.player;
      if (p.alive && p.onSurface && p.ias * MS2KT >= 140 && p.ias * MS2KT <= 160) inWindow = true;
      if (!p.alive) break;
    }
    if (inWindow && s.player.alive) {
      const t0 = s.wall;
      const bank0 = bankOf(s.player);
      const hdg0 = hdgOf(s.player);
      const c = joyCenter(s.joyEl);
      const r = s.joyEl.getBoundingClientRect();
      fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
      fireJoy(s.joyEl, 'pointermove', r.left + r.width - 1, c.y, 7);
      const testTrace: TraceRow[] = [];
      while (s.wall < t0 + 3) {
        s.frame();
        s.record();
        testTrace.push(s.trace[s.trace.length - 1]!);
        if (!s.player.alive) break;
      }
      fireJoy(s.joyEl, 'pointerup', r.left + r.width - 1, c.y, 7);
      const last = testTrace[testTrace.length - 1]!;
      console.log(
        `  on-ground roll: bank ${bank0.toFixed(1)} -> ${last.bank.toFixed(1)} (peak ${peakBank(testTrace).toFixed(1)}) hdg ${hdg0.toFixed(1)}->${last.hdg.toFixed(1)} meanRollIn=${(testTrace.reduce((a, x) => a + x.rollIn, 0) / testTrace.length).toFixed(2)} onSurface=${testTrace.filter((x) => x.onSurface).length}/${testTrace.length} ias=${last.iasKt.toFixed(0)}kt alive=${s.player.alive}`,
      );
      check(
        'E4 JOY full-right ON THE GROUND: bank stays ~0 (ground spring owns the roll)',
        peakBank(testTrace) <= 5,
        `peak bank=${peakBank(testTrace).toFixed(1)} deg — documents the on-ground behaviour, not a failure of the stick (mean rollIn=${(testTrace.reduce((a, x) => a + x.rollIn, 0) / testTrace.length).toFixed(2)})`,
      );
    } else {
      check('E4 JOY full-right ON THE GROUND: bank stays ~0 (ground spring owns the roll)', false, 'never reached 140-160 kt onSurface');
    }
    s.dispose();
  }

  // ============ S8: hands-off auto-level from 60 deg (control) ============
  {
    console.log('--- S8: hands-off auto-level from 60 deg, F-18, 287 kt ---');
    const s = new LiveSession('fa18');
    await s.attach();
    s.spawnAirborne(287, 300);
    for (let i = 0; i < 240; i++) {
      const b = bankOf(s.player);
      s.player.rollIn = clamp01((60 - b) * 0.05);
      // Drive the physics directly for the setup (PD hold), then
      // switch to the live loop hands-off.
      const dt = 1 / 60;
      world.timestep = dt;
      world.step(eventQueue);
      drain(s.player);
      s.player.step(dt, WIND, null);
    }
    const b0 = bankOf(s.player);
    s.player.rollIn = 0;
    let bankAt3 = 0;
    const t0 = s.wall;
    while (s.wall < t0 + 3) {
      s.frame();
      s.record();
      bankAt3 = bankOf(s.player);
      if (!s.player.alive) break;
    }
    console.log(`  auto-level: from ${b0.toFixed(1)} deg to ${bankAt3.toFixed(1)} deg in 3 s wall`);
    check(
      'S8 hands-off auto-level from 60 deg: |bank| <= 10 deg after 3 s',
      Math.abs(bankAt3) <= 10,
      `bank=${bankAt3.toFixed(1)} deg`,
    );
    s.dispose();
  }

  console.log(failures === 0 ? '\nALL PASS-14 INPUT CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
