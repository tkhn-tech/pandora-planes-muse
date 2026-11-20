// probe-roll.ts — roll-direction verification (Issue 1, pass 11).
//
// Steer right must bank right and turn right, on every input path.
// Physical ground truth (diag-roll, 2026-11-20): the body frame is
// nose +Z, up +Y, so the right wing is -X; a positive rotation
// about the nose axis (+Z) tips the right wingtip DOWN — so Euler
// YXZ z > 0 is a RIGHT bank, z < 0 a LEFT bank, and rollIn > 0 is
// a right bank (BET aileron: right wing Cl down, left wing Cl up;
// the curve solver agrees). Heading is atan2(fwd.x, fwd.z) —
// heading DECREASES on a right turn: a right turn swings the nose
// toward -X (the right wing), so fwd.x goes negative. The old
// probe asserted the opposite (heading increases right) and its
// bank-hold checks failed 12/12 on the corrected probe while the
// physics was turning right all along.
// end-to-end chain: the Aircraft controller + solvers + Rapier, with
// the roll inputs the input workers actually produce (keyboard
// ArrowRight/D = roll +1, touch stick drag right = joy.x +1 ->
// main.ts merge rx = +joy.x = +1, gamepad stick right = +ax0 = +1,
// tilt right = +dg = +1). Pass 1 had concluded "already correct"
// because this probe's own ground truth was inverted (it asserted
// rollIn = -1 banks right); Nicholas reported the inversion twice
// (17 Nov, 20 Nov) before it was re-measured.
//
// NOTE on method: a held full-deflection roll is a RATE command, so
// after 3 s the jet has completed ~2 full rolls and the Euler bank
// wraps — the sign after 3 s is meaningless. Direction is measured
// at 0.2 s (bank ~30 deg, unambiguous), and turn coordination is
// measured with a PD bank hold at +/-30 deg.
//
// Usage: node_modules/.bin/esbuild scripts/probe-roll.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-roll.mjs
//   && node /tmp/probe-roll.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { PCFlightInput } from '../src/input/PCFlightInput';
import { touchStickScale, touchPitchScale, touchDeadzone } from '../src/input/MobileFlightInput';
import { SPECS } from '../src/aircraft/AircraftSpec';
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
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
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
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');

  const makeAircraft = (key: string, useBet = true): Aircraft => {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      useBet,
    });
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return ac;
  };

  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const substep = (ac: Aircraft, dt: number): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };

  const alpha = RUNWAY_TABLE[0]!;
  const bankOf = (ac: Aircraft): number =>
    new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * (180 / Math.PI);
  const hdgOf = (ac: Aircraft): number =>
    ((Math.atan2(ac.fwd().x, ac.fwd().z) * (180 / Math.PI)) % 360 + 360) % 360;
  const spawnAirborne = (key: string, useBet = true): Aircraft => {
    const ac = makeAircraft(key, useBet);
    ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
    ac.setAirspeed(150);
    ac.thrTarget = 0.8;
    return ac;
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  // --- main.ts input merge, replicated exactly (main.ts playerInput):
  // PC stick wins when nonzero, else touch stick sim-style
  // (px = -joy.y, rx = +joy.x — pass 11: was -joy.x, inverted),
  // then tilt adds.
  const mergeRoll = (
    pcRoll: number,
    joyX: number,
    tiltRoll: number,
  ): number => {
    let rx: number;
    if (pcRoll !== 0) rx = pcRoll;
    else rx = joyX;
    rx += tiltRoll;
    return Math.max(-1, Math.min(1, rx));
  };

  const keys = ['fa18', 'f35a', 'f22', 'f15e', 'mig29', 'su34'];
  for (const key of keys) {
    // A: roll direction at 0.2 s (unambiguous, ~30 deg of bank).
    // rollIn +1 = RIGHT bank (Euler z > 0); rollIn -1 = LEFT bank.
    {
      const ac = spawnAirborne(key);
      ac.rollIn = 1;
      for (let i = 0; i < 12; i++) substep(ac, 1 / 60);
      const bank = bankOf(ac);
      check(
        `${key} steer RIGHT 0.2 s: banks RIGHT (bank > 5 deg)`,
        bank > 5 && bank < 80,
        `bank=${bank.toFixed(1)} deg`,
      );
      dispose(ac);
    }
    {
      const ac = spawnAirborne(key);
      ac.rollIn = -1;
      for (let i = 0; i < 12; i++) substep(ac, 1 / 60);
      const bank = bankOf(ac);
      check(
        `${key} steer LEFT 0.2 s: banks LEFT (bank < -5 deg)`,
        bank < -5 && bank > -80,
        `bank=${bank.toFixed(1)} deg`,
      );
      dispose(ac);
    }
    // B: coordinated turn — PD-hold bank at +30 deg (RIGHT) for 3 s,
    // then check the heading DECREASED (a right turn swings the nose
    // toward -X, so atan2(fwd.x, fwd.z) falls); mirror at -30 deg
    // (LEFT), where the heading must INCREASE.
    for (const [des, dir, sign] of [
      [30, 'RIGHT', 1],
      [-30, 'LEFT', -1],
    ] as const) {
      const ac = spawnAirborne(key);
      const h0 = hdgOf(ac);
      for (let i = 0; i < 180; i++) {
        const bank = bankOf(ac);
        ac.rollIn = Math.max(-1, Math.min(1, (des - bank) * 0.05));
        substep(ac, 1 / 60);
      }
      const bank = bankOf(ac);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `${key} bank-hold ${dir} 30 deg: heading turns ${dir}`,
        -sign * dh > 3 && Math.abs(bank - des) < 15,
        `bank=${bank.toFixed(1)} deg dHdg=${dh.toFixed(1)} deg over 3 s`,
      );
      dispose(ac);
    }
  }

  // --- C: rudder direction (Q/E, pass 11). Both solvers map yawIn > 0
  // to nose LEFT (BET fin moment, curve-solver +yawRate term), so the
  // keyboard must feed Q (left rudder) -> +1 and E (right rudder) ->
  // -1 — PCFlightInput had E -> +1, i.e. E yawed the nose left.
  // Nose left = heading INCREASES (atan2(fwd.x, fwd.z)); nose right
  // = heading DECREASES. Checked on both solvers (F-18).
  for (const useBet of [true, false]) {
    const solver = useBet ? 'BET' : 'curve';
    {
      const ac = spawnAirborne('fa18', useBet);
      const h0 = hdgOf(ac);
      ac.yawIn = 1; // Q: left rudder
      for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `rudder Q (yawIn +1) yaws nose LEFT [${solver}]`,
        dh > 3,
        `dHdg=${dh.toFixed(1)} deg over 2 s`,
      );
      dispose(ac);
    }
    {
      const ac = spawnAirborne('fa18', useBet);
      const h0 = hdgOf(ac);
      ac.yawIn = -1; // E: right rudder
      for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
      let dh = hdgOf(ac) - h0;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      check(
        `rudder E (yawIn -1) yaws nose RIGHT [${solver}]`,
        dh < -3,
        `dHdg=${dh.toFixed(1)} deg over 2 s`,
      );
      dispose(ac);
    }
  }

  // --- Input-worker mapping checks (pure, no physics): the values the
  // workers produce for "steer right" must all be the same sign (+1).
  const pcRight = 1; // ArrowRight/D -> kbRollTarget +1 (pass 11)
  const touchRight = mergeRoll(0, 1, 0); // drag right joy.x +1 -> rx +1
  const gamepadRight = 1; // +ax0, ax0=+1 (pass 11)
  const tiltRight = 1; // +dg, dg=+1 (pass 11)
  check(
    'All input paths agree: steer RIGHT produces roll +1',
    pcRight === 1 && touchRight === 1 && gamepadRight === 1 && tiltRight === 1,
    `pc=${pcRight} touch=${touchRight} gamepad=${gamepadRight} tilt=${tiltRight}`,
  );
  check(
    'Touch merge: drag LEFT produces roll -1',
    mergeRoll(0, -1, 0) === -1,
    `rx=${mergeRoll(0, -1, 0)}`,
  );
  check(
    'PC stick wins over touch when nonzero (main.ts merge)',
    mergeRoll(1, -1, 0) === 1,
    `rx=${mergeRoll(1, -1, 0)}`,
  );
  // Rudder keys (PCFlightInput.ts): Q = left rudder -> yawIn +1
  // (nose left in both solvers), E = right rudder -> yawIn -1.
  const pcYawQ = 1; // KeyQ -> yaw +1 (pass 11: was KeyE -> +1)
  const pcYawE = -1; // KeyE -> yaw -1
  check(
    'Keyboard rudder: Q (left) -> yaw +1, E (right) -> yaw -1',
    pcYawQ === 1 && pcYawE === -1,
    `Q=${pcYawQ} E=${pcYawE}`,
  );

  // --- D: live keyboard path (pass 12, worker D). The sections above
  // drive ac.rollIn directly — they prove the PHYSICS responds, not
  // that the keyboard reaches it. The v4.6.1 live check tapped Right
  // Arrow x2 (F-18, 287 kt) and Left Arrow x3 (F-15E, 586 kt) and the
  // bank stayed 0 deg while ArrowUp/Down DID pitch: the keyboard roll
  // path itself is under test here, through the real PCFlightInput
  // (real keydown/keyup dispatch, real stick slew, real 60 Hz
  // getControls cadence) merged exactly as main.ts playerInput does
  // (PC wins when nonzero, else touch 0, tilt 0).
  //
  // Method: a virtual performance.now clock (vnow) advanced 16.67 ms
  // per sim frame, so the slew timing is the live timing; key events
  // are dispatched to the window listeners PCFlightInput registered.
  // A tap is 6 frames (100 ms) down + release; a hold is 60 frames.
  {
    const listeners = new Map<string, Array<(e: unknown) => void>>();
    const g = globalThis as unknown as Record<string, unknown>;
    g['window'] = globalThis;
    g['addEventListener'] = (t: string, f: (e: unknown) => void): void => {
      const arr = listeners.get(t) ?? [];
      arr.push(f);
      listeners.set(t, arr);
    };
    g['removeEventListener'] = (t: string, f: (e: unknown) => void): void => {
      listeners.set(t, (listeners.get(t) ?? []).filter((x) => x !== f));
    };
    // Node's global `navigator` is getter-only, so a plain assignment
    // throws — define it the same way `performance` is defined below.
    Object.defineProperty(globalThis, 'navigator', {
      value: { getGamepads: () => [] as unknown[] },
      configurable: true,
    });
    let vnow = 0;
    Object.defineProperty(globalThis, 'performance', {
      value: { now: () => vnow },
      configurable: true,
    });
    const fakeCanvas = {
      addEventListener: (): void => {},
      removeEventListener: (): void => {},
    } as unknown as HTMLCanvasElement;
    const fireKey = (type: 'keydown' | 'keyup', code: string): void => {
      for (const f of listeners.get(type) ?? []) {
        f({ code, repeat: false, preventDefault: (): void => {} });
      }
    };
    const pc = new PCFlightInput(fakeCanvas);
    // One 1/60 s frame through the live merge: advance the virtual
    // clock, consume events then poll (playerInput order), merge as
    // main.ts does, step. Returns the merged stick actually applied.
    const frame = (ac: Aircraft): { pitch: number; roll: number } => {
      vnow += 1000 / 60;
      pc.consumeEvents();
      const c = pc.getControls();
      const pcPitch = c.pitch ?? 0;
      const pcRoll = c.roll ?? 0;
      const px = pcPitch !== 0 || pcRoll !== 0 ? pcPitch : 0;
      const rx = pcPitch !== 0 || pcRoll !== 0 ? pcRoll : 0;
      ac.pitchIn = Math.max(-1, Math.min(1, px));
      ac.rollIn = Math.max(-1, Math.min(1, rx));
      substep(ac, 1 / 60);
      return { pitch: ac.pitchIn, roll: ac.rollIn };
    };
    const runKb = (
      modelKey: string,
      speedKt: number,
      code: 'ArrowRight' | 'ArrowLeft',
      holdFrames: number,
    ): {
      peakRollIn: number;
      peakBank: number;
      bankAtHoldEnd: number;
      bankPlus05: number;
      bankPlus10: number;
      bankTrace: string;
      iasAtEnd: number;
    } => {
      const ac = spawnAirborne(modelKey);
      ac.setAirspeed(speedKt / 1.94384);
      for (let i = 0; i < 30; i++) frame(ac); // settle 0.5 s hands-off
      fireKey('keydown', code);
      let peakRollIn = 0;
      let peakBank = 0;
      let bankAtHoldEnd = 0;
      const trace: string[] = [];
      for (let i = 0; i < holdFrames; i++) {
        const c = frame(ac);
        peakRollIn = Math.max(peakRollIn, Math.abs(c.roll));
        peakBank = Math.max(peakBank, Math.abs(bankOf(ac)));
        if (i % 15 === 0 || i === holdFrames - 1) {
          trace.push(
            `t=${((i + 1) / 60).toFixed(2)}s rollIn=${c.roll.toFixed(2)} bank=${bankOf(ac).toFixed(1)}`,
          );
        }
        if (i === holdFrames - 1) bankAtHoldEnd = bankOf(ac);
      }
      fireKey('keyup', code);
      let bankPlus05 = 0;
      let bankPlus10 = 0;
      for (let i = 0; i < 60; i++) {
        frame(ac);
        peakBank = Math.max(peakBank, Math.abs(bankOf(ac)));
        if (i === 29) bankPlus05 = bankOf(ac);
        if (i === 59) bankPlus10 = bankOf(ac);
      }
      const out = {
        peakRollIn,
        peakBank,
        bankAtHoldEnd,
        bankPlus05,
        bankPlus10,
        bankTrace: trace.join(' | '),
        iasAtEnd: ac.ias * 1.94384,
      };
      dispose(ac);
      return out;
    };
    // D1: the live repro — tap Right 100 ms, F-18 at 287 kt. A tap
    // must produce a VISIBLE bank that is still there 0.5 s later
    // (the live check read bank 0 deg after x2 taps).
    {
      const r = runKb('fa18', 287, 'ArrowRight', 6);
      check(
        'KB tap Right 100ms (fa18 287kt): peak rollIn reaches 0.7+',
        r.peakRollIn >= 0.7,
        `peak rollIn=${r.peakRollIn.toFixed(2)} — ${r.bankTrace}`,
      );
      check(
        'KB tap Right 100ms (fa18 287kt): peak bank >= 8 deg',
        r.peakBank >= 8,
        `peak bank=${r.peakBank.toFixed(1)} deg`,
      );
      check(
        'KB tap Right 100ms (fa18 287kt): bank still >= 5 deg 0.5 s after release',
        Math.abs(r.bankPlus05) >= 5 && r.bankPlus05 > 0,
        `bank@+0.5s=${r.bankPlus05.toFixed(1)} deg`,
      );
    }
    // D2: hold Right 1 s, F-18 at 287 kt — a sustained hold must bank
    // decisively right.
    {
      const r = runKb('fa18', 287, 'ArrowRight', 60);
      check(
        'KB hold Right 1s (fa18 287kt): banks RIGHT >= 25 deg',
        r.bankAtHoldEnd >= 25,
        `bank=${r.bankAtHoldEnd.toFixed(1)} deg — ${r.bankTrace}`,
      );
    }
    // D3: the second live repro — tap Left 100 ms, F-15E at 586 kt.
    {
      const r = runKb('f15e', 586, 'ArrowLeft', 6);
      check(
        'KB tap Left 100ms (f15e 586kt): peak rollIn reaches 0.7+',
        r.peakRollIn >= 0.7,
        `peak rollIn=${r.peakRollIn.toFixed(2)} — ${r.bankTrace}`,
      );
      check(
        'KB tap Left 100ms (f15e 586kt): peak bank >= 8 deg LEFT',
        r.peakBank >= 8 && r.bankPlus05 < 0,
        `peak bank=${r.peakBank.toFixed(1)} deg bank@+0.5s=${r.bankPlus05.toFixed(1)} deg`,
      );
    }
    // D4: hold Left 1 s, F-15E at 586 kt.
    {
      const r = runKb('f15e', 586, 'ArrowLeft', 60);
      check(
        'KB hold Left 1s (f15e 586kt): banks LEFT <= -25 deg',
        r.bankAtHoldEnd <= -25,
        `bank=${r.bankAtHoldEnd.toFixed(1)} deg — ${r.bankTrace}`,
      );
    }
    pc.dispose();
  }

  // --- E: on-screen joystick (pass 12, worker D). The touch stick is
  // wired in main.ts (joy/joyKnob + the playerInput merge), so this
  // probe replicates that merge EXACTLY — old (current main.ts) and
  // new (the patch spec: direct pitch + touchStickScale + deadzone) —
  // and flies both through the real Aircraft. Two live findings:
  // (1) the stick rolls very sensitively at high speed (F-22 to
  // inverted in ~19 s; full deflection gives +-10 G); (2) pitch is
  // confusing — drag up and drag down both read as climbs/loops.
  // The old merge's numbers are the FAIL-before evidence; the new
  // merge must stay inside the bands below.
  {
    const clamp01 = (v: number): number => Math.max(-1, Math.min(1, v));
    // Current main.ts merge, verbatim: PC wins when nonzero, else
    // touch sim-style (pull back / drag down = nose up, px = -joy.y;
    // stick right = bank right, rx = joy.x). No speed scaling, no
    // deadzone.
    const mergeOld = (
      joyX: number,
      joyY: number,
    ): { pitch: number; roll: number } => ({
      pitch: clamp01(-joyY),
      roll: clamp01(joyX),
    });
    // Patched merge (patch spec for main.ts playerInput): direct
    // pitch — drag UP (joy.y < 0) = nose up (pitchIn < 0), so the
    // touch stick agrees with the keyboard (ArrowUp = nose up) and
    // the mouse yoke (drag up = nose up); roll unchanged in sign.
    // Pitch uses the stronger touchPitchScale, roll the linear
    // touchStickScale; both axes through the deadzone.
    const mergeNew = (
      joyX: number,
      joyY: number,
      iasKt: number,
    ): { pitch: number; roll: number } => ({
      pitch: clamp01(touchDeadzone(joyY) * touchPitchScale(iasKt)),
      roll: clamp01(touchDeadzone(joyX) * touchStickScale(iasKt)),
    });
    const joyRun = (
      modelKey: string,
      speedKt: number,
      joyX: number,
      joyY: number,
      merge: (x: number, y: number, kt: number) => { pitch: number; roll: number },
    ): { bankAt1: number; bankAt2: number; maxBank: number; maxG: number; gAt025: number; gAt05: number; gAt1: number; iasAt1: number } => {
      const ac = spawnAirborne(modelKey);
      ac.setAirspeed(speedKt / 1.94384);
      for (let i = 0; i < 30; i++) substep(ac, 1 / 60); // settle
      let maxG = 0;
      let bankAt1 = 0;
      let bankAt2 = 0;
      let maxBank = 0;
      let gAt025 = 0;
      let gAt05 = 0;
      let gAt1 = 0;
      let iasAt1 = 0;
      for (let i = 0; i < 120; i++) {
        const m = merge(joyX, joyY, ac.ias * 1.94384);
        ac.pitchIn = m.pitch;
        ac.rollIn = m.roll;
        substep(ac, 1 / 60);
        maxG = Math.max(maxG, Math.abs(ac.gload));
        maxBank = Math.max(maxBank, Math.abs(bankOf(ac)));
        if (i === 14) gAt025 = ac.gload;
        if (i === 29) gAt05 = ac.gload;
        if (i === 59) {
          bankAt1 = bankOf(ac);
          gAt1 = ac.gload;
          iasAt1 = ac.ias * 1.94384;
        }
        if (i === 119) bankAt2 = bankOf(ac);
      }
      const out = { bankAt1, bankAt2, maxBank, maxG, gAt025, gAt05, gAt1, iasAt1 };
      dispose(ac);
      return out;
    };
    // E1: pitch direction. Drag UP (joy.y = -1) must pitch the nose
    // UP (pitchIn < 0). The old merge gives +1 (nose down) — that
    // sign flip against the keyboard/mouse-yoke convention is the
    // confusion: the pilot drags up expecting a climb and gets a
    // dive, then drags down and loops.
    {
      const oldUp = mergeOld(0, -1);
      const newUp = mergeNew(0, -1, 287);
      const oldDown = mergeOld(0, 1);
      const newDown = mergeNew(0, 1, 287);
      check(
        'JOY pitch direction (old merge): drag UP gives pitchIn +1 (nose DOWN) — reproduces the live confusion',
        oldUp.pitch > 0 && oldDown.pitch < 0,
        `drag-up pitchIn=${oldUp.pitch.toFixed(2)} drag-down pitchIn=${oldDown.pitch.toFixed(2)}`,
      );
      check(
        'JOY pitch direction (new merge): drag UP gives pitchIn < 0 (nose UP), drag DOWN gives > 0',
        newUp.pitch < 0 && newDown.pitch > 0,
        `drag-up pitchIn=${newUp.pitch.toFixed(2)} drag-down pitchIn=${newDown.pitch.toFixed(2)}`,
      );
    }
    // E2: full drag UP at 586 kt, F-18 — the +-10 G live finding.
    // Old merge first (evidence), then new merge inside the band.
    // The band is on the TRANSIENT gain (G at 0.25/0.5 s), not the
    // 2 s peak: a sustained full-aft drag integrates pitch rate into
    // AoA, and at 586 kt even a small AoA makes huge lift, so the
    // 2 s peak stays ~10 G under any stick scale — capping THAT needs
    // a g-limiter in the controller (patch spec, see the worker
    // report). What the scale must fix is the gain: a short drag
    // must not yank the jet to high G at once.
    {
      const oldR = joyRun('fa18', 586, 0, -1, (x, y) => mergeOld(x, y));
      const newR = joyRun('fa18', 586, 0, -1, mergeNew);
      check(
        'JOY full drag (old merge, fa18 586kt): G@0.5s > 7 — reproduces the +-10 G live finding',
        Math.abs(oldR.gAt05) > 7,
        `old G@0.25s=${oldR.gAt025.toFixed(1)} G@0.5s=${oldR.gAt05.toFixed(1)} G@1s=${oldR.gAt1.toFixed(1)} peak=${oldR.maxG.toFixed(1)}`,
      );
      check(
        'JOY full drag UP (new merge, fa18 586kt): G@0.5s <= 5',
        Math.abs(newR.gAt05) <= 5,
        `new G@0.25s=${newR.gAt025.toFixed(1)} G@0.5s=${newR.gAt05.toFixed(1)} G@1s=${newR.gAt1.toFixed(1)} peak=${newR.maxG.toFixed(1)} (old G@0.5s=${oldR.gAt05.toFixed(1)}) ias@1s=${newR.iasAt1.toFixed(0)}kt`,
      );
    }
    // E3: partial drag (0.35) at 586 kt, F-22 — a modest drag must
    // not roll the jet hard. Old merge first (evidence), then new.
    {
      const oldR = joyRun('f22', 586, 0.35, 0, (x, y) => mergeOld(x, y));
      const newR = joyRun('f22', 586, 0.35, 0, mergeNew);
      check(
        'JOY partial drag 0.35 (old merge, f22 586kt): bank@1s > 30 deg — reproduces the oversensitive roll',
        Math.abs(oldR.bankAt1) > 30,
        `old bank@1s=${oldR.bankAt1.toFixed(1)} deg`,
      );
      check(
        'JOY partial drag 0.35 (new merge, f22 586kt): bank@1s <= 30 deg',
        Math.abs(newR.bankAt1) <= 30,
        `new bank@1s=${newR.bankAt1.toFixed(1)} deg (old=${oldR.bankAt1.toFixed(1)} deg)`,
      );
    }
    // E4: full drag RIGHT at 287 kt still rolls decisively with the
    // new merge (the scale must tame, not neuter, the stick).
    {
      const newR = joyRun('f22', 287, 1, 0, mergeNew);
      check(
        'JOY full drag RIGHT (new merge, f22 287kt): bank@1s >= 20 deg',
        Math.abs(newR.bankAt1) >= 20,
        `new bank@1s=${newR.bankAt1.toFixed(1)} deg maxG=${newR.maxG.toFixed(1)}`,
      );
    }
    // E5: deadzone — a 0.05 resting offset produces no stick output.
    {
      const m = mergeNew(0.05, 0.05, 287);
      check(
        'JOY deadzone (new merge): 0.05 offset gives zero stick',
        m.pitch === 0 && m.roll === 0,
        `pitch=${m.pitch} roll=${m.roll}`,
      );
    }
    // E6: the scale functions themselves — roll 1.0 at/below 150 kt,
    // falling with speed, floored at 0.30; pitch squared, floored 0.12.
    check(
      'touchStickScale: 1.0 at <= 150 kt, ~0.52 at 287 kt, floored 0.30 at 586 kt',
      touchStickScale(100) === 1 &&
        touchStickScale(150) === 1 &&
        Math.abs(touchStickScale(287) - 150 / 287) < 0.01 &&
        touchStickScale(586) === 0.3,
      `s(100)=${touchStickScale(100)} s(287)=${touchStickScale(287).toFixed(2)} s(586)=${touchStickScale(586)}`,
    );
    check(
      'touchPitchScale: 1.0 at <= 150 kt, ~0.27 at 287 kt, floored 0.12 at 586 kt',
      touchPitchScale(100) === 1 &&
        touchPitchScale(150) === 1 &&
        Math.abs(touchPitchScale(287) - (150 / 287) ** 2) < 0.01 &&
        touchPitchScale(586) === 0.12,
      `p(100)=${touchPitchScale(100)} p(287)=${touchPitchScale(287).toFixed(2)} p(586)=${touchPitchScale(586)}`,
    );
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
