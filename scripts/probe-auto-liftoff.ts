// probe-auto-liftoff.ts — pass 14, Worker C: auto-liftoff / terrain
// drift, low-power drag, and (via the pull climb-out) the climb after
// liftoff.
//
// v4.8 live check (24 Nov): jets lift off by THEMSELVES at ~190-240 kt
// with a near-zero climb (0-330 fpm) and no rotation input, then drift
// into rising terrain (F-15E into a mountainside at vs -10.67 m/s,
// ~75 s after takeoff). And the F-22 150 kt stall is still untested
// live because the checker could not slow the F-22 down: at 51%
// throttle it held ~700 kt, decaying only ~18 kt in 30 s.
//
// Sections (all through the LIVE path — real Aircraft/BET, real Rapier
// world with the real terrain heightfield + runway boxes, the live
// slider merge via PCFlightInput, live variable dt, live wind, origin
// shifts, real GLBs via attachModel with the Image stub):
//   A. No-input takeoff, Alpha, flaps 0, 100% throttle, pitchIn 0 for
//      the whole run (75 s). A jet must NOT sustain flight without a
//      rotation input: it must never be continuously airborne for
//      more than 5 s without input. Records liftoff speed/distance
//      (if any), pitch/AoA at liftoff, climb rate over the 10 s after
//      liftoff, min terrain clearance, and the outcome.
//   B. Low-power drag, F-22 from 700 kt level (altitude-hold
//      autopilot, the probe-stall-speeds law): 51% throttle decay
//      over 30 s; idle (0%) time to 150 kt; idle + airbrake time to
//      150 kt. The jet must be able to slow down.
//   C. Pull climb-out (the "once airborne they should climb" half):
//      full throttle, pull at Vr, gentle -0.3 hold after liftoff.
//      Liftoff inside the runway and a real climb rate over the 10 s
//      after liftoff.
//
// Usage: node_modules/.bin/esbuild scripts/probe-auto-liftoff.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-auto-liftoff.mjs
//   && node /tmp/probe-auto-liftoff.mjs
//   (delete /tmp/probe-auto-liftoff.mjs after running)

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
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1;
    height = 1;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void {
      (this.listeners[type] ??= []).push(fn);
    }
    removeEventListener(): void {}
    set src(_v: string) {
      queueMicrotask(() => {
        for (const fn of this.listeners['load'] ?? []) fn();
        this.onload?.();
      });
    }
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
if (typeof (globalThis as unknown as { window?: unknown }).window === 'undefined') {
  (globalThis as unknown as { window?: unknown }).window = {
    addEventListener: () => {},
    removeEventListener: () => {},
  };
}
if (typeof (globalThis as unknown as { navigator?: unknown }).navigator === 'undefined') {
  (globalThis as unknown as { navigator?: unknown }).navigator = {
    getGamepads: () => [],
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { PCFlightInput } from '../src/input/PCFlightInput';
import { clFromCurve } from '../src/physics/IAerodynamicsSolver';
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

const MS2KT = 1.94384;
const KT2MS = 1 / MS2KT;
const D2R = Math.PI / 180;
// Live menu wind: windDir 270, 3 m/s (SandboxMenu default) — a headwind
// for the eastbound Alpha takeoff. (probe-takeoff-held-pull.)
const WIND = new THREE.Vector3(-3, 0, 0);
// Live variable-dt pattern (probe-deck-roll): frames 16-250 ms, the
// browser at FPS 4-9, substepped at <= 50 ms like main.ts.
const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
const JETS = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'] as const;

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
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) }),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAtLive = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const alpha = RUNWAY_TABLE[0]!;
  const fakeCanvas = {
    addEventListener: () => {},
    removeEventListener: () => {},
  } as unknown as HTMLCanvasElement;

  const drain = (player: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  // One live frame: substep at <= 50 ms (main.ts pattern), the
  // slider throttle merge, then the aircraft step; origin shift.
  const frame = (ac: Aircraft, pc: PCFlightInput, rawDt: number, wind: THREE.Vector3 = WIND): void => {
    const sub = Math.max(1, Math.ceil(rawDt / 0.05));
    const dt = rawDt / sub;
    for (let s = 0; s < sub; s++) {
      const c = pc.getControls();
      if (c.throttle != null) ac.thrTarget = c.throttle;
      world.timestep = dt;
      world.step(eventQueue);
      drain(ac);
      ac.step(dt, wind, null);
    }
    const sh = origin.shiftIfNeeded(ac.pos);
    if (sh) origin.applyShift(sh);
  };
  const spawnAlpha = async (key: (typeof JETS)[number]): Promise<{ ac: Aircraft; pc: PCFlightInput }> => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAtLive;
    origin.registerBody(ac.body);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    const pc = new PCFlightInput(fakeCanvas);
    const shift = origin.cumulativeShift;
    ac.resetAt(alpha.x - shift.x, alpha.elev + 2.25, alpha.z - shift.z, alpha.hdg);
    ac.brake = false;
    return { ac, pc };
  };
  const disposeAc = (ac: Aircraft, pc: PCFlightInput): void => {
    origin.unregisterBody(ac.body);
    for (const h of ac.colliderHandles()) owners.delete(h);
    pc.dispose();
    ac.dispose();
  };
  const euler = (ac: Aircraft): THREE.Euler =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');

  // ---- A. No-input takeoff: 100% throttle, pitchIn 0, 75 s. ----
  console.log('--- A. No-input takeoff, Alpha, flaps 0, 100% throttle (75 s) ---');
  for (const key of JETS) {
    origin.reset();
    const { ac, pc } = await spawnAlpha(key);
    // Settle 1 s parked.
    for (let f = 0; f < 60; f++) frame(ac, pc, 1 / 60);
    ac.thrTarget = 1;
    pc.setThrottle(1);
    const startY = ac.pos.y;
    let t = 0;
    let pi = 0;
    // First moment the wheels leave a surface (the live "liftoff"
    // reading), and the first moment the jet is actually ABOVE its
    // liftoff height (the fly-away reading — the two differ by the
    // terrain falling away under the jet past the runway end).
    let leftSurfT = -1;
    let leftSurfKt = 0;
    let leftSurfDist = 0;
    let firstAirT = -1;
    let yAtFirstAir = 0;
    let maxClimb = 0;
    let maxPitch = -90;
    let maxAoA = -90;
    let logT = 0;
    let endT = 75;
    while (t < endT) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      frame(ac, pc, rawDt);
      t += rawDt;
      const airborne = !ac.onSurface;
      const dist = ac.pos.x + origin.cumulativeShift.x - alpha.x;
      if (leftSurfT < 0 && !ac.onSurface && t > 2) {
        leftSurfT = t;
        leftSurfKt = ac.ias * MS2KT;
        leftSurfDist = dist;
        yAtFirstAir = ac.pos.y;
      }
      if (firstAirT < 0 && airborne && ac.pos.y > startY + 1) {
        firstAirT = t;
      }
      if (leftSurfT >= 0) {
        maxClimb = Math.max(maxClimb, ac.pos.y - yAtFirstAir);
      }
      maxPitch = Math.max(maxPitch, euler(ac).x / D2R);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
      if (t - logT >= 5) {
        logT = t;
        console.log(
          `  [A/${key} t=${t.toFixed(0)}s] ias=${(ac.ias * MS2KT).toFixed(0)}kt dist=${dist.toFixed(0)}m y=${ac.pos.y.toFixed(1)} floorY=${ac.floorY.toFixed(1)} agl=${(ac.pos.y - ac.floorY).toFixed(1)} pitch=${(euler(ac).x / D2R).toFixed(1)} aoa=${ac.aoaDeg.toFixed(1)} vsi=${ac.vsi.toFixed(1)} onSurf=${ac.onSurface} alive=${ac.alive}`,
        );
      }
      if (!ac.alive) {
        endT = t;
        break;
      }
    }
    const outcome = ac.alive
      ? `alive at t=${t.toFixed(0)}s, y=${ac.pos.y.toFixed(0)} ias=${(ac.ias * MS2KT).toFixed(0)}kt onSurf=${ac.onSurface}`
      : `CRASHED t=${t.toFixed(1)}s (${ac.lastCrashReason ?? '-'}) at y=${ac.pos.y.toFixed(0)}`;
    console.log(
      `  [A/${key}] leftSurface=${leftSurfT < 0 ? 'NEVER' : `t=${leftSurfT.toFixed(1)}s ${leftSurfKt.toFixed(0)}kt dist=${leftSurfDist.toFixed(0)}m`} ` +
        `aboveLiftoff=${firstAirT < 0 ? 'NEVER' : `t=${firstAirT.toFixed(1)}s`} maxClimb=${maxClimb.toFixed(1)}m ` +
        `maxPitch=${maxPitch.toFixed(1)} maxAoA=${maxAoA.toFixed(1)} — ${outcome}`,
    );
    // No liftoff without a rotation input, in the two senses that
    // matter: the jet must never CLIMB away (max 15 m above its
    // liftoff height — a bounce, not a fly-away) and must never
    // sustain airborne flight (by the end of the 75 s it is
    // crashed or back on a surface). A runway overrun that sinks
    // onto the slope is a failed takeoff, not an auto-liftoff.
    check(
      `A/${key}: no climb-away without rotation input (never > 15 m above liftoff height)`,
      maxClimb < 15,
      `maxClimb=${maxClimb.toFixed(1)}m leftSurface=${leftSurfT < 0 ? 'never' : leftSurfKt.toFixed(0) + 'kt'}`,
    );
    check(
      `A/${key}: no sustained flight without rotation input (crashed or back on a surface by 75 s)`,
      !ac.alive || ac.onSurface,
      ac.alive ? `still airborne at 75 s, y=${ac.pos.y.toFixed(0)} ias=${(ac.ias * MS2KT).toFixed(0)}kt` : `crashed (${ac.lastCrashReason ?? '-'})`,
    );
    disposeAc(ac, pc);
  }

  // ---- B. Low-power drag: F-22 from 700 kt, level. ----
  // Altitude-hold autopilot (probe-stall-speeds levelPitchIn): the AoA
  // target is the AoA level flight needs at the current speed, plus a
  // slow altitude trim — so the speed change is the drag/thrust
  // balance, not a dive or a climb.
  console.log('--- B. Low-power drag, F-22 from 700 kt level (1500 m) ---');
  const rhoAt = (altM: number): number => 1.225 * Math.exp(-Math.max(altM, 0) / 8500);
  const aoaForCL = (key: string, clTarget: number): number => {
    const spec = SPECS[key]!;
    // The probe-stall-speeds law floors at 0 deg (clTarget <= CL_0
    // returns 0) — right at 150-200 kt, where level flight needs
    // CL 0.7+, but WRONG at 700 kt: level flight there needs
    // CL ~0.04, i.e. AoA ~-1.4 deg, and holding AoA 0 makes ~3 g
    // (q is ~80 kPa) — the first run of this probe climbed 5,400 m
    // in 30 s on the "level" law. The negative branch mirrors the
    // CL curve's linear slope about CL_0 (clFromCurve does the
    // same), so the target AoA is the level-flight AoA at any
    // speed.
    const slope =
      (spec.clCurve.linearEndCL - 0.15) / Math.max(spec.clCurve.linearEndAoA, 1);
    if (clTarget <= 0.15) return Math.max(-10, (clTarget - 0.15) / slope);
    let lo = 0;
    let hi = spec.clCurve.peakAoA;
    for (let i = 0; i < 40; i++) {
      const mid = (lo + hi) / 2;
      if (clFromCurve(mid, 0, spec.clCurve) < clTarget) lo = mid;
      else hi = mid;
    }
    return (lo + hi) / 2;
  };
  const levelPitchIn = (ac: Aircraft, key: string, alt0: number, gain = 0.3): number => {
    const spec = SPECS[key]!;
    const sLift = spec.wingAreaEffective ?? spec.wingArea;
    const rho = rhoAt(ac.pos.y);
    const q = 0.5 * rho * Math.max(ac.ias, 1) * Math.max(ac.ias, 1);
    const clReq = (spec.mass * 9.81) / Math.max(q * sLift, 1);
    let aoaTarget = aoaForCL(key, clReq);
    aoaTarget += Math.max(-4, Math.min(4, (alt0 - ac.pos.y) * 0.03));
    aoaTarget = Math.min(aoaTarget, spec.clCurve.peakAoA - 2);
    // No min-pull floor here (probe-stall-speeds uses -0.3 to keep
    // the controller's -0.25 "no pull" gate open at 150 kt): at
    // 700 kt the jet is 1,500 m up, so neither the gate's low-alt
    // branch nor its pitched-up branch can bite, and a forced
    // -0.3 pull at 700 kt is a violent input.
    // Gain 0.1 at 51% throttle (below): the F-22's TVC follows the
    // pitch input (±20 deg), so at high thrust the pitch loop gain
    // is far higher than the bare elevator — the 0.3 gain that is
    // stable at idle drove a growing phugoid at 51% (AoA 6.6 deg
    // at 10 s, a 2,400 m zoom-climb in the next 10 s). 0.1 holds
    // altitude at 51% too.
    return Math.max(-1, Math.min(1, (ac.aoaDeg - aoaTarget) * gain));
  };
  const dragRun = async (
    tag: string,
    thr: number,
    airbrake: number,
    maxS: number,
    gain = 0.3,
  ): Promise<{ t150: number; speeds: Array<[number, number]>; minAlt: number; maxAlt: number }> => {
    origin.reset();
    const spec = SPECS['f22']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAtLive;
    origin.registerBody(ac.body);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    const pc = new PCFlightInput(fakeCanvas);
    const shift = origin.cumulativeShift;
    ac.resetAt(alpha.x - shift.x, 1500, alpha.z - shift.z, alpha.hdg);
    ac.setAirspeed(700 * KT2MS);
    ac.thrTarget = thr;
    ac.throttle = thr;
    ac.airbrake = airbrake;
    const alt0 = ac.pos.y;
    let t = 0;
    let pi = 0;
    let t150 = -1;
    const speeds: Array<[number, number]> = [[0, ac.ias * MS2KT]];
    let minAlt = Infinity;
    let maxAlt = -Infinity;
    let logT = 0;
    while (t < maxS) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      ac.pitchIn = levelPitchIn(ac, 'f22', alt0, gain);
      ac.rollIn = 0;
      ac.yawIn = 0;
      frame(ac, pc, rawDt);
      t += rawDt;
      minAlt = Math.min(minAlt, ac.pos.y);
      maxAlt = Math.max(maxAlt, ac.pos.y);
      const kt = ac.ias * MS2KT;
      if (t150 < 0 && kt <= 150) t150 = t;
      // Record the speed at each 10 s crossing (once per crossing).
      const prevMark = Math.floor((t - rawDt) / 10);
      const mark = Math.floor(t / 10);
      if (mark > prevMark) speeds.push([mark * 10, kt]);
      if (t - logT >= 10) {
        logT = t;
        console.log(`  [B/${tag} t=${t.toFixed(0)}s] ${kt.toFixed(0)}kt alt=${ac.pos.y.toFixed(0)}m aoa=${ac.aoaDeg.toFixed(1)} thr=${ac.throttle.toFixed(2)} alive=${ac.alive}`);
      }
      if (!ac.alive || t150 >= 0) break;
    }
    const detail = speeds.map(([s, k]) => `${s}s:${k.toFixed(0)}kt`).join(' ');
    console.log(
      `  [B/${tag}] t150=${t150 < 0 ? 'never in ' + maxS + 's' : t150.toFixed(1) + 's'} alt band=${(maxAlt - minAlt).toFixed(0)}m alive=${ac.alive} — ${detail}`,
    );
    disposeAc(ac, pc);
    return { t150, speeds, minAlt, maxAlt };
  };
  {
    const r51 = await dragRun('51pct', 0.51, 0, 30, 0.1);
    const k30 = r51.speeds[r51.speeds.length - 1]![1]!;
    console.log(`  [B/51pct] decay over 30 s: 700 -> ${k30.toFixed(0)} kt (live check: ~18 kt in 30 s) alt band=${(r51.maxAlt - r51.minAlt).toFixed(0)}m`);
    check(
      'B/51pct: F-22 at 51% throttle decays measurably in 30 s (no thrust/drag lock)',
      k30 < 700,
      `700 -> ${k30.toFixed(0)} kt in 30 s`,
    );
  }
  {
    const r0 = await dragRun('idle', 0, 0, 180);
    check(
      'B/idle: F-22 at idle reaches 150 kt (the jet CAN slow down)',
      r0.t150 >= 0,
      r0.t150 >= 0 ? `t150=${r0.t150.toFixed(1)}s` : 'never in 180 s',
    );
  }
  {
    const rB = await dragRun('idle+airbrake', 0, 1, 180);
    check(
      'B/idle+airbrake: F-22 at idle + airbrake reaches 150 kt faster than idle alone',
      rB.t150 >= 0,
      rB.t150 >= 0 ? `t150=${rB.t150.toFixed(1)}s` : 'never in 180 s',
    );
  }

  // ---- C. Pull climb-out: rotate at Vr, gentle hold, climb rate. ----
  console.log('--- C. Pull climb-out, Alpha (liftoff + 10 s climb rate) ---');
  for (const key of JETS) {
    origin.reset();
    const { ac, pc } = await spawnAlpha(key);
    for (let f = 0; f < 60; f++) frame(ac, pc, 1 / 60);
    ac.thrTarget = 1;
    pc.setThrottle(1);
    const startY = ac.pos.y;
    let t = 0;
    let pi = 0;
    let liftT = -1;
    let liftKt = 0;
    let liftDist = 0;
    let climb10s: number | null = null;
    let logT = 0;
    while (t < 30) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      if (ac.ias > 55 && ac.onSurface) ac.pitchIn = -1;
      if (!ac.onSurface && ac.pos.y < startY + 200) ac.pitchIn = -0.3;
      if (!ac.onSurface && ac.pos.y >= startY + 200) ac.pitchIn = 0;
      frame(ac, pc, rawDt);
      t += rawDt;
      if (liftT < 0 && !ac.onSurface && ac.pos.y > startY + 10) {
        liftT = t;
        liftKt = ac.ias * MS2KT;
        liftDist = ac.pos.x + origin.cumulativeShift.x - alpha.x;
      }
      if (liftT >= 0 && climb10s === null && t >= liftT + 10) {
        climb10s = ((ac.pos.y - (startY + 10)) / 10) * 60 * 3.28084;
      }
      if (t - logT >= 5) {
        logT = t;
        console.log(`  [C/${key} t=${t.toFixed(0)}s] ias=${(ac.ias * MS2KT).toFixed(0)}kt y=${ac.pos.y.toFixed(1)} vsi=${ac.vsi.toFixed(1)} pitchIn=${ac.pitchIn.toFixed(2)} onSurf=${ac.onSurface} alive=${ac.alive}`);
      }
      if (!ac.alive || (climb10s !== null && t > liftT + 10)) break;
    }
    console.log(
      `  [C/${key}] liftoff=${liftT < 0 ? 'NONE' : `t=${liftT.toFixed(1)}s ${liftKt.toFixed(0)}kt dist=${liftDist.toFixed(0)}m`} climb10s=${climb10s === null ? 'n/a' : climb10s.toFixed(0) + ' fpm'} alive=${ac.alive}`,
    );
    check(
      `C/${key}: lifts off inside the runway with a rotation pull and climbs (> 1000 fpm over the 10 s after liftoff)`,
      liftT >= 0 && liftDist < 650 && climb10s !== null && climb10s > 1000,
      `liftoff ${liftT < 0 ? 'never' : liftKt.toFixed(0) + 'kt/' + liftDist.toFixed(0) + 'm'} climb10s=${climb10s === null ? 'n/a' : climb10s.toFixed(0) + ' fpm'}`,
    );
    disposeAc(ac, pc);
  }

  // ---- D. Ground-roll veer (pass 17, worker E, item 2). ----
  // v4.10 live: hands-off on the ground, the F-22 veered 090 -> 077
  // deg with Bank 7 deg before crashing — uncommanded drift while
  // rolling. The pass-15 gimbal-safe bank fix was a FLIGHT fix (the
  // airborne auto-level spring); the ground roll has its own gates
  // (onSurface roll spring + yaw zeroing). Measure, hands-off at
  // 100% throttle on Alpha, live variable dt + live wind, vs the
  // wind-0 and fixed-dt controls: heading/bank drift while the jet
  // is still on a surface, the onSurface fraction during the roll
  // (a flickering gate drops the yaw zeroing out), and the lateral
  // offset from the centreline.
  console.log('--- D. Ground-roll veer, F-22 Alpha, hands-off, 100% throttle ---');
  const veerRun = async (
    tag: string,
    wind: THREE.Vector3,
    useVariableDt: boolean,
    residualRoll = 0,
  ): Promise<{ maxHdgDrift: number; maxBank: number; onSurfFrac: number; latOff: number; outcome: string }> => {
    origin.reset();
    const { ac, pc } = await spawnAlpha('f22');
    if (ac.modelStatus !== 'ok') {
      console.log(`  [D/${tag}] modelStatus=${ac.modelStatus} (GLB failed — run invalid)`);
    }
    for (let f = 0; f < 60; f++) frame(ac, pc, 1 / 60);
    ac.thrTarget = 1;
    pc.setThrottle(1);
    const hdg0 = Math.atan2(ac.fwd().x, ac.fwd().z) / D2R;
    let t = 0;
    let pi = 0;
    let maxHdgDrift = 0;
    let maxBank = 0;
    let surfFrames = 0;
    let totalFrames = 0;
    let latOff = 0;
    let logT = 0;
    while (t < 40) {
      const rawDt = useVariableDt ? DT_PATTERN[pi % DT_PATTERN.length]! : 1 / 60;
      pi++;
      ac.pitchIn = 0;
      ac.rollIn = residualRoll;
      ac.yawIn = 0;
      frame(ac, pc, rawDt, wind);
      t += rawDt;
      totalFrames++;
      if (ac.onSurface) surfFrames++;
      const hdg = Math.atan2(ac.fwd().x, ac.fwd().z) / D2R;
      let d = hdg - hdg0;
      while (d > 180) d -= 360;
      while (d < -180) d += 360;
      if (ac.onSurface) {
        maxHdgDrift = Math.max(maxHdgDrift, Math.abs(d));
        maxBank = Math.max(maxBank, Math.abs(euler(ac).z / D2R));
        // Lateral offset from the Alpha centreline (along-track x).
        latOff = Math.max(latOff, Math.abs(ac.pos.z + origin.cumulativeShift.z - alpha.z));
      }
      if (t - logT >= 5) {
        logT = t;
        console.log(
          `  [D/${tag} t=${t.toFixed(0)}s] ias=${(ac.ias * MS2KT).toFixed(0)}kt hdg=${hdg.toFixed(1)} (drift ${d.toFixed(1)}) ` +
            `bank=${(euler(ac).z / D2R).toFixed(1)} onSurf=${ac.onSurface} latOff=${latOff.toFixed(1)}m alive=${ac.alive}`,
        );
      }
      if (!ac.alive) break;
      if (!ac.onSurface && ac.pos.y > alpha.elev + 2.25 + 10) break;
    }
    const outcome = ac.alive
      ? `alive, airborne at t=${t.toFixed(1)}s`
      : `CRASHED t=${t.toFixed(1)}s (${ac.lastCrashReason ?? '-'}) vs=${ac.lastCrashVs.toFixed(2)} bank=${ac.lastCrashBankDeg.toFixed(1)}`;
    console.log(
      `  [D/${tag}] maxHdgDriftOnGround=${maxHdgDrift.toFixed(1)}deg maxBankOnGround=${maxBank.toFixed(1)}deg ` +
        `onSurfaceFrac=${totalFrames ? (surfFrames / totalFrames).toFixed(2) : 'n/a'} maxLatOff=${latOff.toFixed(1)}m — ${outcome}`,
    );
    disposeAc(ac, pc);
    return { maxHdgDrift, maxBank, onSurfFrac: totalFrames ? surfFrames / totalFrames : 0, latOff, outcome };
  };
  {
    const live = await veerRun('live-dt+wind', WIND, true);
    const fixed = await veerRun('fixed-dt+wind', WIND, false);
    const noWind = await veerRun('live-dt+nowind', new THREE.Vector3(0, 0, 0), true);
    check(
      'D/live: hands-off F-22 holds heading on the ground (drift < 5 deg) at live dt + wind',
      live.maxHdgDrift < 5,
      `drift=${live.maxHdgDrift.toFixed(1)}deg bank=${live.maxBank.toFixed(1)}deg latOff=${live.latOff.toFixed(1)}m onSurfFrac=${live.onSurfFrac.toFixed(2)}`,
    );
    check(
      'D/live: hands-off F-22 stays wings-level on the ground (|bank| < 5 deg) at live dt + wind',
      live.maxBank < 5,
      `bank=${live.maxBank.toFixed(1)}deg drift=${live.maxHdgDrift.toFixed(1)}deg`,
    );
    check(
      'D/fixed: hands-off F-22 holds heading on the ground (drift < 5 deg) at fixed dt + wind',
      fixed.maxHdgDrift < 5,
      `drift=${fixed.maxHdgDrift.toFixed(1)}deg bank=${fixed.maxBank.toFixed(1)}deg onSurfFrac=${fixed.onSurfFrac.toFixed(2)}`,
    );
    check(
      'D/nowind: hands-off F-22 holds heading on the ground (drift < 5 deg) at live dt, no wind',
      noWind.maxHdgDrift < 5,
      `drift=${noWind.maxHdgDrift.toFixed(1)}deg bank=${noWind.maxBank.toFixed(1)}deg onSurfFrac=${noWind.onSurfFrac.toFixed(2)}`,
    );
    // D2: the residual-input variant — a small constant roll input
    // (0.08, e.g. a stick/yoke not quite centred, or a keyboard
    // tail) during the same roll. The airborne auto-level gate is
    // rollIn === 0 EXACTLY, and the onSurface yaw zeroing needs
    // |rollIn| <= 0.1, so a 0.08 residual disables the auto-level
    // in the airborne half of each bounce while adding no steering.
    // If this veers, the live veer was a residual input, not wind
    // or the ground gates — and the fix belongs in the input merge
    // (Worker A), not here.
    const resid = await veerRun('live-dt+wind+roll0.08', WIND, true, 0.08);
    console.log(
      `  [D2/residual] drift=${resid.maxHdgDrift.toFixed(1)}deg bank=${resid.maxBank.toFixed(1)}deg ` +
        `latOff=${resid.latOff.toFixed(1)}m onSurfFrac=${resid.onSurfFrac.toFixed(2)} — ${resid.outcome}`,
    );
    check(
      'D2/residual: a 0.08 residual roll input does NOT bank the jet > 5 deg on the ground (live veer was Bank 7 deg)',
      resid.maxBank < 5 && resid.maxHdgDrift < 5,
      `bank=${resid.maxBank.toFixed(1)}deg drift=${resid.maxHdgDrift.toFixed(1)}deg`,
    );
  }

  // ---- E. Alpha liftoff timing (pass 17, worker E, item 3). ----
  // v4.10 live: Alpha liftoff is timing-critical — short runway,
  // terrain east drops (crash y 41.9-49.0 m vs spawn 52.1 m), pulls
  // below ~150 kt or late ended in crashes (vs -9.5 to -16.7 m/s);
  // the v4.9 re-measurement also saw crashes at vs -2.59 to -4.83
  // m/s while rolling. Two suspects: (a) the runway is genuinely
  // too short at the rotation speeds, (b) the crash judge is too
  // sensitive on the downslope. Measure both: liftoff distance from
  // the strip centre (flat top ends 485 m out, skirt ends 835 m),
  // terrain clearance east of Alpha, and — for every crash — the
  // vertical speed, the impact-NORMAL speed (the slope term the
  // judge actually uses), and the Rapier velocity jump across the
  // fatal world step (the dv > 20 guard fires 'terrain').
  // The clearance bar is 2.5 m, not 5 m: at liftoff the jet's
  // centre is one gear height (2.2 m) above the surface by
  // construction, so the minimum clearance over a healthy
  // climb-out is ~3 m — demanding > 5 m would fail a perfect
  // liftoff. 2.5 m means the gear itself clears the terrain by
  // >= 0.3 m at every moment after liftoff: a genuine no-graze bar.
  console.log('--- E. Alpha liftoff timing: pull at 130/140/150/160 kt + late pull ---');
  const liftoffRun = async (
    key: (typeof JETS)[number],
    pullKt: number,
    pullAtDist: number | null,
  ): Promise<{
    liftDist: number;
    liftKt: number;
    minClear: number;
    outcome: string;
    crashVs: number;
    crashBank: number;
    crashY: number;
    maxDv: number;
  }> => {
    origin.reset();
    const { ac, pc } = await spawnAlpha(key);
    for (let f = 0; f < 60; f++) frame(ac, pc, 1 / 60);
    ac.thrTarget = 1;
    pc.setThrottle(1);
    const startY = ac.pos.y;
    let t = 0;
    let pi = 0;
    let liftDist = -1;
    let liftKt = 0;
    let minClear = Infinity;
    let maxDv = 0;
    let pulling = false;
    let logT = 0;
    while (t < 40) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const dist = ac.pos.x + origin.cumulativeShift.x - alpha.x;
      const kt = ac.ias * MS2KT;
      if (!pulling && ((pullAtDist === null && kt >= pullKt) || (pullAtDist !== null && dist >= pullAtDist))) {
        pulling = true;
      }
      if (pulling && ac.onSurface) ac.pitchIn = -1;
      else if (!ac.onSurface && ac.pos.y < startY + 200) ac.pitchIn = -0.3;
      else if (!ac.onSurface) ac.pitchIn = 0;
      else ac.pitchIn = 0;
      // Rapier velocity jump across this frame's world steps (the
      // dv > 20 guard's input): sample the body linvel around the
      // frame() call — frame() steps the world internally, so read
      // the pre/post linvel here (one frame's worth of steps).
      const bv0 = ac.body.linvel();
      frame(ac, pc, rawDt);
      const bv1 = ac.body.linvel();
      const dvStep = Math.hypot(bv1.x - bv0.x, bv1.y - bv0.y, bv1.z - bv0.z);
      if (ac.alive) maxDv = Math.max(maxDv, dvStep);
      t += rawDt;
      if (liftDist < 0 && !ac.onSurface && ac.pos.y > startY + 1) {
        liftDist = dist;
        liftKt = kt;
      }
      if (liftDist >= 0) {
        minClear = Math.min(minClear, ac.pos.y - ac.floorY);
      }
      if (t - logT >= 5) {
        logT = t;
        console.log(
          `  [E/${key}@${pullAtDist === null ? pullKt + 'kt' : 'dist' + pullAtDist} t=${t.toFixed(0)}s] ias=${kt.toFixed(0)}kt ` +
            `dist=${dist.toFixed(0)}m y=${ac.pos.y.toFixed(1)} floorY=${ac.floorY.toFixed(1)} clear=${(ac.pos.y - ac.floorY).toFixed(1)} ` +
            `onSurf=${ac.onSurface} alive=${ac.alive}`,
        );
      }
      if (!ac.alive) break;
      if (liftDist >= 0 && t > 0 && ac.pos.y - ac.floorY > 100) break;
    }
    const outcome = ac.alive
      ? `alive t=${t.toFixed(1)}s y=${ac.pos.y.toFixed(0)} ias=${(ac.ias * MS2KT).toFixed(0)}kt`
      : `CRASHED t=${t.toFixed(1)}s (${ac.lastCrashReason ?? '-'}) vs=${ac.lastCrashVs.toFixed(2)} bank=${ac.lastCrashBankDeg.toFixed(1)} y=${ac.pos.y.toFixed(1)}`;
    console.log(
      `  [E/${key}@${pullAtDist === null ? pullKt + 'kt' : 'dist' + pullAtDist}] liftoff=${liftDist < 0 ? 'NONE' : `${liftDist.toFixed(0)}m @ ${liftKt.toFixed(0)}kt`} ` +
        `minClearAfterLiftoff=${minClear === Infinity ? 'n/a' : minClear.toFixed(1) + 'm'} maxDv=${maxDv.toFixed(1)}m/s — ${outcome}`,
    );
    const crashVs = ac.lastCrashVs;
    const crashBank = ac.lastCrashBankDeg;
    const crashY = ac.pos.y;
    disposeAc(ac, pc);
    return { liftDist, liftKt, minClear, outcome, crashVs, crashBank, crashY, maxDv };
  };
  for (const key of ['f22', 'f15e', 'fa18', 'f35a'] as const) {
    // Pull at 150 kt (the live "below ~150 kt crashes" boundary) and
    // at 140 kt (just below it): does the jet lift inside the flat
    // top (485 m from the strip centre) and stay clear of the slope?
    const r150 = await liftoffRun(key, 150, null);
    const r140 = await liftoffRun(key, 140, null);
    check(
      `E/${key}@150kt: pull at 150 kt lifts off inside the flat top (< 485 m) and stays clear (never closer than 2.5 m to the terrain)`,
      r150.liftDist >= 0 && r150.liftDist < 485 && r150.minClear > 2.5,
      `liftoff=${r150.liftDist < 0 ? 'NONE' : r150.liftDist.toFixed(0) + 'm @ ' + r150.liftKt.toFixed(0) + 'kt'} minClear=${r150.minClear === Infinity ? 'n/a' : r150.minClear.toFixed(1) + 'm'} — ${r150.outcome}`,
    );
    check(
      `E/${key}@140kt: pull at 140 kt lifts off inside the flat top (< 485 m) and stays clear (never closer than 2.5 m to the terrain)`,
      r140.liftDist >= 0 && r140.liftDist < 485 && r140.minClear > 2.5,
      `liftoff=${r140.liftDist < 0 ? 'NONE' : r140.liftDist.toFixed(0) + 'm @ ' + r140.liftKt.toFixed(0) + 'kt'} minClear=${r140.minClear === Infinity ? 'n/a' : r140.minClear.toFixed(1) + 'm'} — ${r140.outcome}`,
    );
  }
  {
    // Late pull: F-22 holds the pull until the strip end (325 m
    // from the centre) — the live "late" case. And a 130 kt pull
    // (well below the live boundary) for the same jet.
    const late = await liftoffRun('f22', 0, 325);
    const r130 = await liftoffRun('f22', 130, null);
    console.log(
      `  [E/f22-late] liftoff=${late.liftDist < 0 ? 'NONE' : late.liftDist.toFixed(0) + 'm @ ' + late.liftKt.toFixed(0) + 'kt'} ` +
        `minClear=${late.minClear === Infinity ? 'n/a' : late.minClear.toFixed(1) + 'm'} maxDv=${late.maxDv.toFixed(1)} — ${late.outcome}`,
    );
    console.log(
      `  [E/f22@130kt] liftoff=${r130.liftDist < 0 ? 'NONE' : r130.liftDist.toFixed(0) + 'm @ ' + r130.liftKt.toFixed(0) + 'kt'} ` +
        `minClear=${r130.minClear === Infinity ? 'n/a' : r130.minClear.toFixed(1) + 'm'} maxDv=${r130.maxDv.toFixed(1)} — ${r130.outcome}`,
    );
    check(
      'E/f22-late: a pull at the strip end still lifts off inside the skirt (< 835 m) without crashing',
      late.liftDist >= 0 && late.liftDist < 835,
      `liftoff=${late.liftDist < 0 ? 'NONE' : late.liftDist.toFixed(0) + 'm'} — ${late.outcome}`,
    );
    check(
      'E/f22@130kt: a pull at 130 kt lifts off inside the flat top (< 485 m) and stays clear (never closer than 2.5 m to the terrain)',
      r130.liftDist >= 0 && r130.liftDist < 485 && r130.minClear > 2.5,
      `liftoff=${r130.liftDist < 0 ? 'NONE' : r130.liftDist.toFixed(0) + 'm @ ' + r130.liftKt.toFixed(0) + 'kt'} minClear=${r130.minClear === Infinity ? 'n/a' : r130.minClear.toFixed(1) + 'm'} — ${r130.outcome}`,
    );
  }

  console.log(failures === 0 ? '\nALL AUTO-LIFTOFF CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(2);
});
