// probe-pass15-takeoff.ts — Pass 15 Worker A, item 1: F-22 / F-15E PITCH
// DEAD FROM ALPHA, live-path reproduction.
//
// v4.9 live (24 Nov 14:06-14:53 SGT): F-22 (3 attempts) and F-15E (2)
// rolled to 377-505 kt and crashed at the SAME terrain point
// (vs=-4.29/-4.77 m/s, bank 0.4-0.5 deg, y=41.3/41.4) with neither
// joystick up/down nor ArrowUp lifting them. F-18 / F-35A lift fine.
// probe-takeoff-held-pull (fixed 1/60, direct pitchIn=-1 held from
// t=0) passes ALL six jets — so the fault is NOT raw rotation physics
// with a full direct pull. It must be in the live path: variable
// 4-9 fps dt + sim cap, the real input chain (KB slew / touch scale),
// the rotation gate / hasRotated / stay-on-ground hold at those dts,
// or a per-jet interaction (elevator schedule, CL curve, TVC).
//
// This probe runs the LIVE path (copied from probe-pass14-inputs:
// LiveSession, main.ts playerInput merge verbatim, real GLBs,
// FloatingOrigin, live 3 m/s wind, DT_PATTERN 4-9 fps, sim cap
// 0.10 s/frame) for all six jets in three input modes:
//   A: KB ArrowUp held from Vr (per-jet rotateSpeedKt), released at AGL>30
//   B: JOY full-up drag held from Vr, released at AGL>30
//   C: DIRECT pitchIn=-1 from Vr (bypasses the input chain entirely)
// If C passes but A/B fail for a jet, the fault is the input chain /
// gates. If C fails too, it is physics at live dt.
// Trace records pitchIn, hasRotated, onSurface, pitch, AoA, ias so the
// mechanism is measured, not guessed.
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass15-takeoff.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass15-takeoff.mjs
//   && node /tmp/probe-pass15-takeoff.mjs
//   (then rm -f /tmp/probe-pass15-takeoff.mjs)

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

const WIND = new THREE.Vector3(-3, 0, 0); // live wind: dir 270, 3 m/s — a headwind for the eastbound Alpha takeoff. (+3, 0, 0) was WRONG: a tailwind makes the MiG-29 crash deterministically at 104 kt / 349 m (measured 24 Nov), while the other five jets tolerate it.
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

  const DT_PATTERN = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
  const PHYS_MS = 2;

  interface Row {
    wall: number;
    sim: number;
    pitch: number;
    bank: number;
    iasKt: number;
    aoa: number;
    agl: number;
    pitchIn: number;
    hasRotated: boolean;
    onSurface: boolean;
    distM: number;
  }

  class LiveSession {
    readonly pc = new PCFlightInput(fakeCanvas);
    readonly tilt = new MobileFlightInput();
    readonly joyEl = makeJoyEl();
    readonly joy = { x: 0, y: 0, active: false, id: -1 };
    player: Aircraft;
    wall = 0;
    sim = 0;
    trace: Row[] = [];
    private pi = 0;
    lastPcPitch = 0;
    lastPcRoll = 0;
    /** Direct-input override (mode C): when set, playerInput is
     *  skipped and pitchIn is written directly each substep. */
    directPitch: number | null = null;

    constructor(readonly key: string) {
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

    spawnAlpha(): void {
      origin.reset();
      this.player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      this.player.gear = true;
      this.joy.active = false;
      this.joy.x = 0;
      this.joy.y = 0;
      this.pc.calibrate();
      this.tilt.calibrate();
    }

    playerInput(dt: number): void {
      const p = this.player;
      if (this.directPitch !== null) {
        p.pitchIn = this.directPitch;
        p.rollIn = 0;
        this.lastPcPitch = this.directPitch;
        this.lastPcRoll = 0;
        return;
      }
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
      this.lastPcRoll = pcRoll;
    }

    frame(script?: (s: LiveSession) => void): number {
      const rawDt = Math.min(DT_PATTERN[this.pi % DT_PATTERN.length]!, 0.25);
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
        drain(this.player);
        this.player.step(dt, WIND, null);
        this.sim += dt;
      }
      const renderMs = Math.max(0, rawDt * 1000 - sub * PHYS_MS);
      vnow += renderMs;
      this.wall += rawDt;
      return rawDt;
    }

    record(): void {
      const p = this.player;
      this.trace.push({
        wall: this.wall,
        sim: this.sim,
        pitch: pitchOf(p),
        bank: bankOf(p),
        iasKt: p.ias * MS2KT,
        aoa: p.aoaDeg,
        agl: p.pos.y - p.floorY,
        pitchIn: p.pitchIn,
        hasRotated: (p as unknown as { hasRotated: boolean }).hasRotated,
        onSurface: p.onSurface,
        distM: p.pos.x + origin.cumulativeShift.x - alpha.x,
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

  type Mode = 'kb' | 'joy' | 'direct';
  interface TakeoffResult {
    mode: Mode;
    key: string;
    alive: boolean;
    liftoffWall: number | null;
    liftoffSim: number | null;
    liftoffIas: number | null;
    liftoffDist: number | null;
    liftoffPitch: number | null;
    crashReason: string | null;
    crashIas: number | null;
    crashPos: string | null;
    crashVs: number | null;
    crashBank: number | null;
    maxPitchOnRoll: number;
    minPitchInOnRoll: number;
    hasRotatedAtVrPlus: boolean | null;
    yAt15Wall: number | null;
    vsiAtLiftoffPlus5: number | null;
  }

  const runTakeoff = async (key: string, mode: Mode): Promise<TakeoffResult> => {
    const spec = SPECS[key]!;
    const vr = spec.rotateSpeedKt;
    const s = new LiveSession(key);
    await s.attach();
    const modelOk = s.player.modelStatus === 'ok';
    s.spawnAlpha();
    s.player.thrTarget = 1;
    let pulled = false;
    let released = false;
    let liftoff: Row | null = null;
    let maxPitchOnRoll = -Infinity;
    let minPitchInOnRoll = Infinity;
    let hasRotatedAtVrPlus: boolean | null = null;
    const c = joyCenter(s.joyEl);
    const r = s.joyEl.getBoundingClientRect();
    while (s.wall < 75) {
      s.frame((_ss) => {
        const p = s.player;
        const iasKt = p.ias * MS2KT;
        if (!pulled && iasKt >= vr) {
          if (mode === 'kb') fireKey('keydown', 'ArrowUp');
          else if (mode === 'joy') {
            fireJoy(s.joyEl, 'pointerdown', c.x, c.y, 7);
            fireJoy(s.joyEl, 'pointermove', c.x, r.top + 1, 7); // full up
          } else {
            s.directPitch = -1;
          }
          pulled = true;
        } else if (pulled && !released && !p.onSurface && p.pos.y - p.floorY > 30) {
          if (mode === 'kb') fireKey('keyup', 'ArrowUp');
          else if (mode === 'joy') fireJoy(s.joyEl, 'pointerup', c.x, r.top + 1, 7);
          else s.directPitch = 0;
          released = true;
        }
      });
      s.record();
      const row = s.trace[s.trace.length - 1]!;
      const p = s.player;
      if (p.onSurface) {
        if (row.pitch > maxPitchOnRoll) maxPitchOnRoll = row.pitch;
        if (row.pitchIn < minPitchInOnRoll) minPitchInOnRoll = row.pitchIn;
        if (hasRotatedAtVrPlus === null && row.iasKt >= vr + 20) {
          hasRotatedAtVrPlus = row.hasRotated;
        }
      }
      if (!p.alive) break;
      // Sustained-climb liftoff (AGL > 15 m and climbing), NOT a
      // bounce: the Su-34's Rapier ejection off the runway end and
      // the MiG-29's skirt bounce both trip a bare !onSurface test
      // at 11-41 kt with pitchIn 0 — they are not takeoffs.
      if (
        !liftoff &&
        p.pos.y - p.floorY > 15 &&
        p.vsi > 2 &&
        p.ias * MS2KT > spec.rotateSpeedKt
      ) {
        liftoff = row;
        console.log(
          `  [${key}/${mode}] liftoff wall=${row.wall.toFixed(1)}s sim=${row.sim.toFixed(1)}s ias=${row.iasKt.toFixed(0)}kt (Vr ${vr}) dist=${row.distM.toFixed(0)}m pitch=${row.pitch.toFixed(1)} aoa=${row.aoa.toFixed(1)} pitchIn=${row.pitchIn.toFixed(2)} hasRotated=${row.hasRotated} model=${modelOk ? 'ok' : s.player.modelStatus}`,
        );
      }
      // Stop 5 s wall after liftoff with a climb check.
      if (liftoff && s.wall >= liftoff.wall + 5) break;
    }
    const last = s.trace[s.trace.length - 1]!;
    const at15 = s.trace.find((x) => x.wall >= 15);
    const res: TakeoffResult = {
      mode,
      key,
      alive: s.player.alive,
      liftoffWall: liftoff ? liftoff.wall : null,
      liftoffSim: liftoff ? liftoff.sim : null,
      liftoffIas: liftoff ? liftoff.iasKt : null,
      liftoffDist: liftoff ? liftoff.distM : null,
      liftoffPitch: liftoff ? liftoff.pitch : null,
      crashReason: s.player.alive ? null : s.player.lastCrashReason,
      crashIas: s.player.alive ? null : last.iasKt,
      crashPos: s.player.alive
        ? null
        : `(${s.player.pos.x.toFixed(0)}, ${s.player.pos.y.toFixed(1)}, ${s.player.pos.z.toFixed(0)}) dist=${last.distM.toFixed(0)}m`,
      crashVs: s.player.alive ? null : s.player.lastCrashVs,
      crashBank: s.player.alive ? null : s.player.lastCrashBankDeg * R2D,
      maxPitchOnRoll: maxPitchOnRoll === -Infinity ? 0 : maxPitchOnRoll,
      minPitchInOnRoll: minPitchInOnRoll === Infinity ? 0 : minPitchInOnRoll,
      hasRotatedAtVrPlus,
      yAt15Wall: at15 ? at15.agl : null,
      vsiAtLiftoffPlus5: liftoff ? s.player.vsi * 196.85 : null,
    };
    if (!s.player.alive) {
      console.log(
        `  [${key}/${mode}] CRASHED wall=${s.wall.toFixed(1)}s reason=${res.crashReason} ias=${res.crashIas?.toFixed(0)}kt pos=${res.crashPos} vs=${res.crashVs?.toFixed(2)} bank=${res.crashBank?.toFixed(1)} maxPitchOnRoll=${res.maxPitchOnRoll.toFixed(1)} minPitchIn=${res.minPitchInOnRoll.toFixed(2)} hasRotated@Vr+20=${res.hasRotatedAtVrPlus}`,
      );
    } else if (!liftoff) {
      console.log(
        `  [${key}/${mode}] NO LIFTOFF in 75s wall: ias=${last.iasKt.toFixed(0)}kt dist=${last.distM.toFixed(0)}m pitch=${last.pitch.toFixed(1)} maxPitchOnRoll=${res.maxPitchOnRoll.toFixed(1)} minPitchIn=${res.minPitchInOnRoll.toFixed(2)} hasRotated@Vr+20=${res.hasRotatedAtVrPlus}`,
      );
    } else {
      console.log(
        `  [${key}/${mode}] end: y-agl=${last.agl.toFixed(0)}m vsi=${(s.player.vsi * 196.85).toFixed(0)}fpm ias=${last.iasKt.toFixed(0)}kt alive=${s.player.alive}`,
      );
    }
    s.dispose();
    return res;
  };

  const keys = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const;
  const results: TakeoffResult[] = [];
  for (const mode of ['kb', 'joy', 'direct'] as const) {
    console.log(`--- mode ${mode}: pull at Vr, release at AGL>30, live 4-9 fps ---`);
    for (const key of keys) {
      results.push(await runTakeoff(key, mode));
    }
  }

  // Bar: sustained-climb point (AGL > 15 m, climbing, past Vr) inside the
  // 650 m strip, alive at the end of the run. The gear leaves the ground
  // earlier; this criterion measures where the climb is established,
  // which is 100-200 m further down the strip because the pull is held
  // to AGL 30 m — so the bar is the strip length (650 m), not the flat
  // plateau (485 m).
  console.log('\n=== SUMMARY (sustained-climb point inside the 650 m strip; climb 5 s after liftoff) ===');
  for (const r of results) {
    const ok =
      r.alive && r.liftoffDist !== null && r.liftoffDist < 650;
    check(
      `${r.key}/${r.mode}: lifts off inside the runway and survives`,
      ok,
      r.liftoffDist !== null
        ? `liftoff dist=${r.liftoffDist.toFixed(0)}m ias=${r.liftoffIas?.toFixed(0)}kt wall=${r.liftoffWall?.toFixed(1)}s sim=${r.liftoffSim?.toFixed(1)}s pitch=${r.liftoffPitch?.toFixed(1)} vsi+5s=${r.vsiAtLiftoffPlus5?.toFixed(0)}fpm`
        : `NO LIFTOFF — crashed ${r.crashReason ?? '?'} at ${r.crashIas?.toFixed(0)}kt ${r.crashPos ?? ''} vs=${r.crashVs?.toFixed(2)} bank=${r.crashBank?.toFixed(1)} (live F-22/F-15E: 377-505kt, vs -4.29/-4.77, bank 0.4-0.5, y=41.3/41.4)`,
    );
  }

  console.log(failures === 0 ? '\nALL PASS-15 TAKEOFF CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
