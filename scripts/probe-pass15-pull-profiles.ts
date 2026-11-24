// probe-pass15-pull-profiles.ts — Pass 15 Worker A, item 1.
//
// probe-pass15-takeoff showed a pull HELD from Vr lifts all six jets
// through the live path (F-22 liftoff 287 m / 153 kt, F-15E 341 m /
// 169 kt). The v4.9 live F-22/F-15E failures therefore cannot be a
// held-Vr-pull defect — so this probe varies the pull PROFILE the way
// a real pilot actually flies it, and looks for a profile that
// discriminates F-22/F-15E from F-18/F-35A:
//   early-release: ArrowUp held from t=0, released at Vr (pilot pulls
//     early, sees nothing below Vr — the rotation gate zeroes it —
//     and gives up exactly when rotation becomes possible)
//   tap: ArrowUp 100 ms wall at Vr (one tap, live v4.8 methodology)
//   short-hold: ArrowUp held from Vr for 0.5 s of SIM, then released
//   late200 / late300: pull only once the jet is already at 200 /
//     300 kt (missed ROTATE cue — the jet is already off the strip
//     and skimming the plateau skirt when the pull starts)
//   joy-late250-full / joy-late350-full / joy-late350-half: the same
//     late pull but with the TOUCH joystick — full drag, or half drag
//     — where touchPitchScale (200/ias)^1.5 crushes the input at speed
//     (full drag at 350 kt -> pitchIn -0.44; half drag -> -0.22, below
//     the -0.25 rotation threshold entirely)
// Liftoff here is a SUSTAINED climb (AGL > 15 m and vsi > 2 m/s), not
// a bounce — probe-pass15-takeoff's !onSurface test counted the
// Su-34's Rapier ejection bounce as a liftoff at 11 kt.
//
// Live path throughout (probe-pass14 LiveSession: main.ts playerInput
// merge verbatim, real GLBs, FloatingOrigin, live wind (-3,0,0) dir
// 270, DT_PATTERN 4-9 fps, sim cap 0.10 s/frame).
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass15-pull-profiles.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass15-pull-profiles.mjs
//   && node /tmp/probe-pass15-pull-profiles.mjs
//   (then rm -f /tmp/probe-pass15-pull-profiles.mjs)

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

const WIND = new THREE.Vector3(-3, 0, 0); // live wind: dir 270, 3 m/s
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

  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * R2D;
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
    lastPcPitch = 0;

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
      const pcPitch = pc.pitch ?? 0;
      const pcRoll = pc.roll ?? 0;
      let px: number;
      let rx: number;
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

  type Profile =
    | 'early-release'
    | 'tap'
    | 'short-hold'
    | 'late200'
    | 'late300'
    | 'joy-late250-full'
    | 'joy-late350-full'
    | 'joy-late350-half';
  interface Outcome {
    profile: Profile;
    key: string;
    climbed: boolean;
    crashReason: string | null;
    maxIasKt: number;
    endDistM: number;
    pullIasKt: number | null;
    pullDistM: number | null;
    maxPitchAfterPull: number | null;
    pitchAtPullPlus1Sim: number | null;
    pitchAtPullPlus2Sim: number | null;
    hasRotatedAtPullPlus1: boolean | null;
    minPitchInAfterPull: number | null;
  }

  const runProfile = async (key: string, profile: Profile): Promise<Outcome> => {
    const spec = SPECS[key]!;
    const vr = spec.rotateSpeedKt;
    const s = new LiveSession(key);
    await s.attach();
    if (s.player.modelStatus !== 'ok') {
      console.log(`  [${key}/${profile}] MODEL NOT OK: ${s.player.modelStatus}`);
    }
    s.spawnAlpha();
    s.player.thrTarget = 1;
    let pulled = false;
    let released = false;
    let pullWall = -1;
    let pullSim = -1;
    let pullIas: number | null = null;
    let pullDist: number | null = null;
    let maxPitchAfterPull: number | null = null;
    let pitchPlus1: number | null = null;
    let pitchPlus2: number | null = null;
    let hasRotPlus1: boolean | null = null;
    let minPitchInAfterPull: number | null = null;
    let climbed = false;
    let maxIas = 0;
    const jc = joyCenter(s.joyEl);
    const jr = s.joyEl.getBoundingClientRect();
    if (profile === 'early-release') {
      fireKey('keydown', 'ArrowUp');
      pulled = true;
      pullWall = 0;
      pullSim = 0;
      pullIas = 0;
      pullDist = 0;
    }
    while (s.wall < 60) {
      s.frame((_ss) => {
        const p = s.player;
        const iasKt = p.ias * MS2KT;
        const dist = p.pos.x + origin.cumulativeShift.x - alpha.x;
        const isJoyLate = profile.startsWith('joy-late');
        if (profile === 'early-release') {
          if (!released && iasKt >= vr) {
            fireKey('keyup', 'ArrowUp');
            released = true;
          }
        } else if (!pulled) {
          const atVr = profile === 'tap' || profile === 'short-hold';
          const thresh = atVr
            ? vr
            : profile === 'late200'
              ? 200
              : profile === 'late300'
                ? 300
                : profile === 'joy-late250-full'
                  ? 250
                  : 350;
          if (iasKt >= thresh) {
            if (isJoyLate) {
              const half = profile === 'joy-late350-half';
              fireJoy(s.joyEl, 'pointerdown', jc.x, jc.y, 7);
              fireJoy(
                s.joyEl,
                'pointermove',
                jc.x,
                half ? jc.y - 37 : jr.top + 1,
                7,
              );
            } else {
              fireKey('keydown', 'ArrowUp');
            }
            pulled = true;
            pullWall = s.wall;
            pullSim = s.sim;
            pullIas = iasKt;
            pullDist = dist;
          }
        } else if (!released) {
          if (profile === 'tap' && s.wall - pullWall >= 0.1) {
            fireKey('keyup', 'ArrowUp');
            released = true;
          } else if (profile === 'short-hold' && s.sim - pullSim >= 0.5) {
            fireKey('keyup', 'ArrowUp');
            released = true;
          } else if (
            (profile === 'late200' ||
              profile === 'late300' ||
              isJoyLate) &&
            !p.onSurface &&
            p.pos.y - p.floorY > 30
          ) {
            if (isJoyLate) fireJoy(s.joyEl, 'pointerup', jc.x, jc.y, 7);
            else fireKey('keyup', 'ArrowUp');
            released = true;
          }
        }
      });
      const p = s.player;
      if (p.ias * MS2KT > maxIas) maxIas = p.ias * MS2KT;
      if (pulled && pullSim >= 0) {
        if (p.pitchIn < (minPitchInAfterPull ?? 0)) minPitchInAfterPull = p.pitchIn;
        if (maxPitchAfterPull === null || pitchOf(p) < maxPitchAfterPull) {
          maxPitchAfterPull = pitchOf(p); // Euler x: negative = nose UP; track most nose-up
        }
        if (pitchPlus1 === null && s.sim >= pullSim + 1) {
          pitchPlus1 = pitchOf(p);
          hasRotPlus1 = (p as unknown as { hasRotated: boolean }).hasRotated;
        }
        if (pitchPlus2 === null && s.sim >= pullSim + 2) pitchPlus2 = pitchOf(p);
      }
      const agl = p.pos.y - p.floorY;
      if (!climbed && agl > 15 && p.vsi > 2 && p.ias * MS2KT > vr) {
        climbed = true;
        console.log(
          `  [${key}/${profile}] CLIMBED wall=${s.wall.toFixed(1)}s sim=${s.sim.toFixed(1)}s ias=${(p.ias * MS2KT).toFixed(0)}kt agl=${agl.toFixed(0)}m vsi=${(p.vsi * 196.85).toFixed(0)}fpm pitch=${pitchOf(p).toFixed(1)} pullIas=${pullIas?.toFixed(0) ?? '-'} pullDist=${pullDist?.toFixed(0) ?? '-'}m`,
        );
        break;
      }
      if (!p.alive) break;
    }
    const p = s.player;
    const endDist = p.pos.x + origin.cumulativeShift.x - alpha.x;
    if (!climbed) {
      console.log(
        `  [${key}/${profile}] NO CLIMB: ${p.alive ? 'still rolling/skimming' : 'CRASHED ' + (p.lastCrashReason ?? '?')} maxIas=${maxIas.toFixed(0)}kt endDist=${endDist.toFixed(0)}m pullIas=${pullIas?.toFixed(0) ?? '-'} pullDist=${pullDist?.toFixed(0) ?? '-'}m maxPitch(nose-up)=${maxPitchAfterPull === null ? '-' : (-maxPitchAfterPull).toFixed(1)} pitch+1s=${pitchPlus1 === null ? '-' : pitchPlus1.toFixed(1)} pitch+2s=${pitchPlus2 === null ? '-' : pitchPlus2.toFixed(1)} hasRot+1s=${hasRotPlus1} minPitchIn=${minPitchInAfterPull === null ? '-' : minPitchInAfterPull.toFixed(2)}`,
      );
    }
    const out: Outcome = {
      profile,
      key,
      climbed,
      crashReason: p.alive ? null : p.lastCrashReason,
      maxIasKt: maxIas,
      endDistM: endDist,
      pullIasKt: pullIas,
      pullDistM: pullDist,
      maxPitchAfterPull: maxPitchAfterPull === null ? null : -maxPitchAfterPull,
      pitchAtPullPlus1Sim: pitchPlus1,
      pitchAtPullPlus2Sim: pitchPlus2,
      hasRotatedAtPullPlus1: hasRotPlus1,
      minPitchInAfterPull,
    };
    s.dispose();
    return out;
  };

  const keys = ['fa18', 'f35a', 'f22', 'f15e'] as const;
  const profiles: Profile[] = [
    'early-release',
    'tap',
    'short-hold',
    'late200',
    'late300',
    'joy-late250-full',
    'joy-late350-full',
    'joy-late350-half',
  ];
  const outcomes: Outcome[] = [];
  for (const profile of profiles) {
    console.log(`--- profile ${profile} (live 4-9 fps, wind -3) ---`);
    for (const key of keys) {
      outcomes.push(await runProfile(key, profile));
    }
  }

  console.log('\n=== SUMMARY: sustained climb (AGL>15m, vsi>2m/s) by profile ===');
  for (const o of outcomes) {
    check(
      `${o.key}/${o.profile}: climbs away`,
      o.climbed,
      o.climbed
        ? `pull at ${o.pullIasKt?.toFixed(0)}kt/${o.pullDistM?.toFixed(0)}m`
        : `${o.crashReason ? 'crashed ' + o.crashReason : 'never climbed'} maxIas=${o.maxIasKt.toFixed(0)}kt endDist=${o.endDistM.toFixed(0)}m maxPitchUp=${o.maxPitchAfterPull?.toFixed(1) ?? '-'}deg pitch+1s=${o.pitchAtPullPlus1Sim?.toFixed(1) ?? '-'} hasRot+1s=${o.hasRotatedAtPullPlus1} minPitchIn=${o.minPitchInAfterPull?.toFixed(2) ?? '-'}`,
    );
  }

  console.log(failures === 0 ? '\nALL PULL-PROFILE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
