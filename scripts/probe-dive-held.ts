// probe-dive-held.ts — Pass 14 Worker A, cross-worker flag from Worker D.
//
// Worker D's PULL UP probe: an F-18 dive with full nose-down HELD
// pulled out by itself (bottomed 348 m AGL, climbed away — no
// impact), while probe-crash-vfx D3 shows F-22 hands-off dives do
// crash. So the dive auto-recovery correlates with HELD input, and
// pass 13's PI trim did not fully fix it for the held-input case.
//
// This probe runs the same dive through the REAL input path
// (PCFlightInput + main.ts merge, live variable dt 4-9 fps, live
// wind 3 m/s, origin shift, real spawn, real GLBs) and asks: with
// full nose-down HELD from altitude, does the jet crash, or pull
// out on its own? Cases:
//   D1  F-18, ArrowDown HELD from level flight at 1500 m AGL.
//   D2  F-18, dive entered (3 s ArrowDown) then RELEASED — the
//       hands-off control (should crash, per D3).
//   D3  F-22, ArrowDown HELD (cross-check against D3's F-22).
//   D4  F-18, on-screen joystick full-down HELD (the live case
//       Worker D reported: a joystick hold, not a key hold).
//
// A pull-out is pitch rising (pitchRate turning positive) while
// pitchIn stays at full nose-down — the controller overruling a
// held input. Logged per second: pitch, pitchIn, AoA, ias, vsi,
// AGL, stalled, alive.
//
// Usage: node_modules/.bin/esbuild scripts/probe-dive-held.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-dive-held.mjs
//   && node /tmp/probe-dive-held.mjs
//   (then rm -f /tmp/probe-dive-held.mjs)

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (Same stubs as probe-pass14-inputs.ts.)
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

const WIND = new THREE.Vector3(3, 0, 0);
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

  const pitchOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x * R2D;

  const DT_PATTERN = [0.25, 0.14, 0.11, 0.2, 0.125, 0.167, 0.22, 0.13, 0.25, 0.143];
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
    private renderMs = 0;

    constructor(
      readonly key: string,
      useBet = true,
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
        try {
          joyEl.setPointerCapture(pe.pointerId);
        } catch {
          /* capture failure must not lose the drag (pass 14 fix) */
        }
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

    spawnAirborne(speedKt: number, altM = 1500): void {
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

    /** main.ts playerInput merge, VERBATIM (minus VTOL/fire/weapons
     *  tails, which do not touch the stick). */
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
      this.renderMs = renderMs;
      return rawDt;
    }

    dispose(): void {
      origin.unregisterBody(this.player.body);
      for (const h of this.player.colliderHandles()) owners.delete(h);
      this.player.dispose();
      this.pc.dispose();
      this.tilt.dispose();
    }
  }

  interface DiveResult {
    crashed: boolean;
    minAgl: number;
    maxPitch: number; // most nose-DOWN (Euler x + = nose down)
    maxIasKt: number;
    pullOutAgl: number; // AGL where pitch bottomed and started rising, -1 if none
    endPitch: number;
    reason: string;
  }

  /**
   * Fly a dive. `hold` drives the input each frame (key hold / joy
   * hold / release at a wall time). Returns the outcome; logs a
   * per-second trace.
   */
  async function runDive(
    tag: string,
    key: string,
    hold: (s: LiveSession) => void,
    maxWall = 60,
  ): Promise<DiveResult> {
    const s = new LiveSession(key);
    await s.attach();
    s.spawnAirborne(240, 1500);
    // Settle 2 s wall level, hands-off.
    while (s.wall < 2) s.frame();
    const startAgl = s.player.pos.y - s.player.floorY;
    let minAgl = startAgl;
    let maxPitch = pitchOf(s.player);
    let maxIasKt = s.player.ias * MS2KT;
    let pullOutAgl = -1;
    let prevPitch = pitchOf(s.player);
    let nextLog = s.wall + 1;
    let lastPitchIn = 0;
    while (s.wall < maxWall) {
      s.frame(hold);
      const p = s.player;
      const pitch = pitchOf(p);
      const agl = p.pos.y - p.floorY;
      minAgl = Math.min(minAgl, agl);
      maxPitch = Math.max(maxPitch, pitch);
      maxIasKt = Math.max(maxIasKt, p.ias * MS2KT);
      lastPitchIn = p.pitchIn;
      // Pull-out (Worker D's signature: bottomed out and CLIMBED
      // AWAY with the stick still held): vsi turns positive after
      // diving at least 100 m, while pitchIn stays > 0.5. Euler
      // pitch alone cannot define it — in a near-vertical dive the
      // jet tumbles at extreme negative AoA (-20 to -45 deg) and
      // the nose wanders 87 -> 33 deg while vsi stays -200 m/s;
      // that is an oscillation, not a recovery.
      if (pullOutAgl < 0 && p.vsi > 0 && p.pitchIn > 0.5 && agl < startAgl - 100) {
        pullOutAgl = agl;
      }
      prevPitch = pitch;
      if (s.wall >= nextLog) {
        nextLog += 1;
        console.log(
          `    [${tag} +${(s.wall - 2).toFixed(0)}s] pitch=${pitch.toFixed(1)} pitchIn=${p.pitchIn.toFixed(2)} aoa=${p.aoaDeg.toFixed(1)} ias=${(p.ias * MS2KT).toFixed(0)}kt vsi=${p.vsi.toFixed(1)} agl=${agl.toFixed(0)} stalled=${p.stalled} alive=${p.alive}`,
        );
      }
      if (!p.alive) break;
    }
    const p = s.player;
    const res: DiveResult = {
      crashed: !p.alive,
      minAgl,
      maxPitch,
      maxIasKt,
      pullOutAgl,
      endPitch: pitchOf(p),
      reason: p.lastCrashReason ?? '-',
    };
    console.log(
      `  [${tag}] ${res.crashed ? 'CRASHED' : 'SURVIVED'} (${res.reason}) minAgl=${minAgl.toFixed(0)} m maxPitch=${maxPitch.toFixed(1)} deg maxIas=${maxIasKt.toFixed(0)} kt pullOutAgl=${pullOutAgl < 0 ? 'none' : pullOutAgl.toFixed(0) + ' m'} endPitch=${res.endPitch.toFixed(1)} lastPitchIn=${lastPitchIn.toFixed(2)}`,
    );
    s.dispose();
    return res;
  }

  // D1. F-18, ArrowDown HELD from level flight at 1500 m AGL.
  console.log('--- D1: F-18 ArrowDown HELD from 1500 m AGL ---');
  const d1 = await runDive('D1', 'fa18', (s) => {
    if (s.wall >= 2) fireKey('keydown', 'ArrowDown');
  });
  fireKey('keyup', 'ArrowDown');
  check(
    'D1 F-18 held nose-down dive crashes (no auto pull-out)',
    d1.crashed,
    d1.crashed
      ? `impact, maxPitch=${d1.maxPitch.toFixed(1)} deg`
      : `SURVIVED — pulled out at ${d1.pullOutAgl < 0 ? '?' : d1.pullOutAgl.toFixed(0) + ' m AGL'} (Worker D saw 348 m)`,
  );

  // D2. F-18, dive entered (3 s ArrowDown) then RELEASED — the
  // hands-off control. Should crash (probe-crash-vfx D3 pattern).
  console.log('--- D2: F-18 dive entered, then RELEASED (hands-off) ---');
  const d2 = await runDive('D2', 'fa18', (s) => {
    if (s.wall >= 2 && s.wall < 5) fireKey('keydown', 'ArrowDown');
    if (s.wall >= 5) fireKey('keyup', 'ArrowDown');
  });
  fireKey('keyup', 'ArrowDown');
  check(
    'D2 F-18 hands-off dive crashes (control)',
    d2.crashed,
    d2.crashed ? `impact, maxPitch=${d2.maxPitch.toFixed(1)} deg` : 'SURVIVED',
  );

  // D3. F-22, ArrowDown HELD — cross-check against D3's F-22.
  console.log('--- D3: F-22 ArrowDown HELD from 1500 m AGL ---');
  const d3 = await runDive('D3', 'f22', (s) => {
    if (s.wall >= 2) fireKey('keydown', 'ArrowDown');
  });
  fireKey('keyup', 'ArrowDown');
  check(
    'D3 F-22 held nose-down dive crashes (no auto pull-out)',
    d3.crashed,
    d3.crashed
      ? `impact, maxPitch=${d3.maxPitch.toFixed(1)} deg`
      : `SURVIVED — pulled out at ${d3.pullOutAgl < 0 ? '?' : d3.pullOutAgl.toFixed(0) + ' m AGL'}`,
  );

  // D4. F-18, on-screen joystick full-down HELD — the live case
  // Worker D reported (a joystick hold, not a key hold).
  console.log('--- D4: F-18 joystick full-down HELD from 1500 m AGL ---');
  const d4 = await runDive('D4', 'fa18', (s) => {
    if (s.wall >= 2) {
      const c = joyCenter(s.joyEl);
      // Full down: pointer at the bottom edge of the stick.
      fireJoy(s.joyEl, 'pointerdown', c.x, c.y + 74);
      fireJoy(s.joyEl, 'pointermove', c.x, c.y + 74);
    }
  });
  check(
    'D4 F-18 joystick full-down dive crashes (no auto pull-out)',
    d4.crashed,
    d4.crashed
      ? `impact, maxPitch=${d4.maxPitch.toFixed(1)} deg`
      : `SURVIVED — pulled out at ${d4.pullOutAgl < 0 ? '?' : d4.pullOutAgl.toFixed(0) + ' m AGL'}`,
  );

  console.log(failures === 0 ? 'ALL DIVE CHECKS PASSED' : `${failures} DIVE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
