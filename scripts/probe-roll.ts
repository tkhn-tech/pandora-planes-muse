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
import { PCFlightInput, kbRollScale } from '../src/input/PCFlightInput';
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
    // Keep userAgent: THREE's GLTFParser reads
    // navigator.userAgent.indexOf at parse time, so a userAgent-less
    // shim throws inside every later GLB load in this process.
    Object.defineProperty(globalThis, 'navigator', {
      value: { userAgent: 'NodeProbe/1.0', getGamepads: () => [] as unknown[] },
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
    // Pass 13: main.ts feeds the live IAS into the keyboard roll
    // scale every playerInput call, so this harness does the same
    // from the aircraft's live IAS.
    const frame = (ac: Aircraft): { pitch: number; roll: number } => {
      vnow += 1000 / 60;
      pc.setAirspeedKt(ac.ias * 1.94384);
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
    // (the live check read bank 0 deg after x2 taps). Pass 13
    // retune: the old bands (peak rollIn 0.7+, peak bank 8 deg+)
    // were calibrated to the pass-12 12/s slew, which at 586 kt
    // peaked at 39 deg of bank from one tap — far too strong live.
    // With the speed scale the stick still travels to ~70% (peak
    // rollIn ~0.5 after the 0.70 scale) and the tap banks a few
    // degrees; the upper bound locks the fix in (a tap must never
    // again bank 15 deg+, let alone 39).
    {
      const r = runKb('fa18', 287, 'ArrowRight', 6);
      check(
        'KB tap Right 100ms (fa18 287kt): peak rollIn reaches 0.4+',
        r.peakRollIn >= 0.4,
        `peak rollIn=${r.peakRollIn.toFixed(2)} — ${r.bankTrace}`,
      );
      check(
        'KB tap Right 100ms (fa18 287kt): peak bank 3-15 deg',
        r.peakBank >= 3 && r.peakBank <= 15,
        `peak bank=${r.peakBank.toFixed(1)} deg`,
      );
      // Pass 13: the old "still >= 5 deg 0.5 s later" check
      // contradicts the fixed auto-level — a 3-5 deg tap bank is
      // SUPPOSED to be levelled within half a second now (the
      // spring cap rose to 0.40 rad/s and the 45 deg gate is gone),
      // so the persistence assertion is that the bank returns
      // toward level instead of sticking or diverging.
      check(
        'KB tap Right 100ms (fa18 287kt): bank back within 5 deg of level 0.5 s after release',
        Math.abs(r.bankPlus05) <= 5,
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
    // Pass 13 retune (same reason as D1): the stick travels to
    // ~70% but the 0.34 scale caps the output at ~0.24, and the tap
    // banks a few degrees left instead of the old 39 deg.
    {
      const r = runKb('f15e', 586, 'ArrowLeft', 6);
      check(
        'KB tap Left 100ms (f15e 586kt): peak rollIn reaches 0.2+',
        r.peakRollIn >= 0.2,
        `peak rollIn=${r.peakRollIn.toFixed(2)} — ${r.bankTrace}`,
      );
      check(
        'KB tap Left 100ms (f15e 586kt): peak bank 4-15 deg LEFT',
        r.peakBank >= 4 && r.peakBank <= 15 && r.bankAtHoldEnd < 0,
        `peak bank=${r.peakBank.toFixed(1)} deg bank@end-of-tap=${r.bankAtHoldEnd.toFixed(1)} deg`,
      );
      check(
        'KB tap Left 100ms (f15e 586kt): bank back within 5 deg of level 0.5 s after release',
        Math.abs(r.bankPlus05) <= 5,
        `bank@+0.5s=${r.bankPlus05.toFixed(1)} deg`,
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
    // falling with speed, floored at 0.15; pitch (200/ias)^1.5,
    // floored 0.15 (pass 14 worker A: the old squared curve
    // (150/ias)^2, floor 0.12, was too weak to fly with — a full
    // drag down at 240 kt commanded pitchIn 0.39 and pitched 4.4
    // deg in 5 s at altitude, probe-pass14 E2b).
    // Pass 13: the roll curve steepened 150/ias -> 110/ias (a full
    // 1.5 s drag at 700 kt rolled the F-35A 92 deg on the old curve).
    check(
      'touchStickScale: 1.0 at <= 150 kt, ~0.38 at 287 kt, ~0.19 at 586 kt, floored 0.15 at 733+ kt',
      touchStickScale(100) === 1 &&
        touchStickScale(150) === 1 &&
        Math.abs(touchStickScale(287) - 110 / 287) < 0.01 &&
        Math.abs(touchStickScale(586) - 110 / 586) < 0.01 &&
        touchStickScale(1000) === 0.15,
      `s(100)=${touchStickScale(100)} s(287)=${touchStickScale(287).toFixed(2)} s(586)=${touchStickScale(586).toFixed(2)} s(1000)=${touchStickScale(1000)}`,
    );
    check(
      'touchPitchScale: 1.0 at <= 200 kt, ~0.76 at 240 kt, ~0.58 at 287 kt, ~0.20 at 586 kt, floored 0.15 at 776+ kt',
      touchPitchScale(100) === 1 &&
        touchPitchScale(150) === 1 &&
        touchPitchScale(200) === 1 &&
        Math.abs(touchPitchScale(240) - (200 / 240) ** 1.5) < 0.01 &&
        Math.abs(touchPitchScale(287) - (200 / 287) ** 1.5) < 0.01 &&
        Math.abs(touchPitchScale(586) - (200 / 586) ** 1.5) < 0.01 &&
        touchPitchScale(1000) === 0.15,
      `p(100)=${touchPitchScale(100)} p(240)=${touchPitchScale(240).toFixed(2)} p(287)=${touchPitchScale(287).toFixed(2)} p(586)=${touchPitchScale(586).toFixed(2)} p(1000)=${touchPitchScale(1000)}`,
    );
  }

  // --- F: LIVE paths at high speed (pass 13, worker B). Sections D/E
  // run at fixed dt=1/60 with WIND=0 — the harness the pass-12 fix
  // was tuned against — but the live game runs variable frame dt
  // (16–250 ms, FPS 4–36) with 3 m/s wind, and getControls() is
  // called once per SUBSTEP with a wall-clock dt. At fixed dt the
  // pass-12 tap looked fine (15.5 deg at 287 kt); live-style at
  // 776 kt the SAME code banked 62 deg from one 100 ms tap (111 deg
  // in the v4.7 live check) and a 1 s hold rolled the jet inverted
  // (178 deg), because the first substep of a 250 ms frame slewed
  // 12 * 0.1 = 1.2 — full deflection in one frame — and the BET
  // roll rate at full deflection is ~150 deg/s at that speed. This
  // section replicates the live loop exactly (DT_PATTERN frames,
  // substeps <= 0.05 s, WIND 3 m/s, real PCFlightInput, the real
  // main.ts merge incl. touch priority, setAirspeedKt every
  // substep) and must FAIL on the old code / PASS on the fix. The
  // numbers in the band comments are the measured before/after.
  // Pitch is held at the level-flight trim AoA by a P loop (the
  // roll scripts never touch pitch): at 700+ kt CL_0 alone makes
  // ~3 g, so an untrimmed pitch-0 spawn climbs steeply.
  {
    const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
    const WIND_LIVE = new THREE.Vector3(3, 0, 0);
    const listenersF = new Map<string, Array<(e: unknown) => void>>();
    const gF = globalThis as unknown as Record<string, unknown>;
    gF['window'] = globalThis;
    gF['addEventListener'] = (t: string, f: (e: unknown) => void): void => {
      const arr = listenersF.get(t) ?? [];
      arr.push(f);
      listenersF.set(t, arr);
    };
    gF['removeEventListener'] = (t: string, f: (e: unknown) => void): void => {
      listenersF.set(t, (listenersF.get(t) ?? []).filter((x) => x !== f));
    };
    let vnowF = 0;
    Object.defineProperty(globalThis, 'performance', {
      value: { now: () => vnowF },
      configurable: true,
    });
    const fakeCanvasF = {
      addEventListener: (): void => {},
      removeEventListener: (): void => {},
    } as unknown as HTMLCanvasElement;
    const fireKeyF = (type: 'keydown' | 'keyup', code: string): void => {
      for (const f of listenersF.get(type) ?? []) {
        f({ code, repeat: false, preventDefault: (): void => {} });
      }
    };
    const pcF = new PCFlightInput(fakeCanvasF);
    type ScriptOut = { keyDown?: string[]; keyUp?: string[]; joyX?: number; joyY?: number; joyActive?: boolean };
    const runLive = (
      modelKey: string,
      speedKt: number,
      script: (t: number) => ScriptOut,
      seconds: number,
    ): { trace: Array<{ t: number; bank: number; rollIn: number }>; peakBank: number } => {
      const spec = SPECS[modelKey]!;
      const ac = new Aircraft({ scene, world, spec, isPlayer: true, modelKey: spec.modelKey, useBet: true });
      ac.terrainHeightAt = (x: number, z: number) => heightfieldHeightAt(heights, res, x, z);
      for (const h of ac.colliderHandles()) owners.set(h, ac);
      ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
      ac.setAirspeed(speedKt / 1.94384);
      ac.thrTarget = 0.8;
      const vMs = speedKt / 1.94384;
      const clNeed = (2 * spec.mass * 9.81) / (1.225 * vMs * vMs * spec.wingArea);
      const aoaTrimDeg = Math.max(-5, Math.min(15, (clNeed - 0.15) / 0.08));
      const pitchDes = (-aoaTrimDeg * Math.PI) / 180;
      const trace: Array<{ t: number; bank: number; rollIn: number }> = [];
      let peakBank = 0;
      let t = 0;
      let pi = 0;
      let joyX = 0;
      let joyY = 0;
      let joyActive = false;
      // One live frame: script at frame start, vnow += rawDt, then
      // substeps (consumeEvents + getControls per substep, exactly
      // like main.ts playerInput inside the animate() substep loop).
      const liveFrame = (dur: number, scripted: boolean): void => {
        const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
        pi++;
        if (scripted) {
          const s = script(t);
          for (const k of s.keyDown ?? []) fireKeyF('keydown', k);
          for (const k of s.keyUp ?? []) fireKeyF('keyup', k);
          if (s.joyX !== undefined) joyX = s.joyX;
          if (s.joyY !== undefined) joyY = s.joyY;
          if (s.joyActive !== undefined) joyActive = s.joyActive;
        }
        vnowF += rawDt * 1000;
        const sub = Math.ceil(rawDt / 0.05);
        const dt = rawDt / sub;
        for (let k = 0; k < sub; k++) {
          pcF.setAirspeedKt(ac.ias * 1.94384);
          pcF.consumeEvents();
          // Pass 14: pass the substep dt, exactly like main.ts
          // playerInput (pcInput.getControls(dt)). The old
          // no-arg call used the wall-clock fallback, which the
          // live game no longer exercises: vnowF advances once
          // per frame here, so the first substep saw a wall dt of
          // 0.11-0.25 s and the rest ~0 — the slew jumped to its
          // 0.4 cap once per frame instead of slewing in sim time.
          const c = pcF.getControls(dt);
          const pcPitch = c.pitch ?? 0;
          const pcRoll = c.roll ?? 0;
          let rx: number;
          if (joyActive || (pcPitch === 0 && pcRoll === 0)) {
            rx = touchDeadzone(joyX) * touchStickScale(ac.ias * 1.94384);
          } else {
            rx = pcRoll;
          }
          const pitchNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').x;
          ac.pitchIn = Math.max(-0.5, Math.min(0.5, (pitchDes - pitchNow) * 4));
          ac.rollIn = Math.max(-1, Math.min(1, rx));
          world.timestep = dt;
          world.step(eventQueue);
          drain(ac);
          ac.step(dt, WIND_LIVE, null);
        }
        t += rawDt;
        const b = bankOf(ac);
        peakBank = Math.max(peakBank, Math.abs(b));
        trace.push({ t, bank: b, rollIn: ac.rollIn });
        void dur;
      };
      // Settle 0.5 s hands-off (level trim only, no roll input).
      let st = 0;
      while (st < 0.5) {
        liveFrame(0, false);
        st = t;
        // liveFrame advances t from 0; settle uses the same t — reset
        // the script clock after settling so the script sees t=0.
        if (st >= 0.5) break;
      }
      t = 0;
      pi = 0;
      trace.length = 0;
      peakBank = 0;
      joyX = 0;
      joyY = 0;
      joyActive = false;
      while (t < seconds) liveFrame(0, true);
      const out = { trace, peakBank };
      ac.dispose();
      owners.clear();
      owners.set(terrainCollider.handle, 'terrain');
      return out;
    };
    const atF = (
      r: { trace: Array<{ t: number; bank: number; rollIn: number }> },
      tt: number,
    ): { bank: number; rollIn: number } =>
      r.trace.reduce((a, b) => (Math.abs(b.t - tt) < Math.abs(a.t - tt) ? b : a));
    // F1: the v4.7 live repro — one 100 ms Right-Arrow tap, F-18 at
    // 776 kt. Before: peak bank 62 deg live-style (111 deg live),
    // still 62 deg 3 s later (no auto-level past 45 deg). After
    // pass 14 (sim-dt slew, ROLL_SLEW_UP 10/s): peak 11.0 deg
    // here, 7.2 deg in the faithful 4-9 fps probe-pass14-inputs
    // S2/S3 (this section runs sim time == wall time, so its 100 ms
    // tap delivers more sim time than the 4-9 fps live loop, where
    // the 0.10 s sim cap runs at ~0.4-0.6x). Band <= 12 deg: it
    // must stay a small tap (v4.7 was 62 deg), and it levels
    // again within 2 s.
    {
      const r = runLive('fa18', 776, (t) => {
        if (t === 0) return { keyDown: ['ArrowRight'] };
        if (t >= 0.1 && t < 0.3) return { keyUp: ['ArrowRight'] };
        return {};
      }, 3);
      check(
        'LIVE tap Right 100ms (fa18 776kt): peak bank <= 12 deg',
        r.peakBank <= 12,
        `peak bank=${r.peakBank.toFixed(1)} deg (before fix: 62.1 deg; faithful 4-9 fps probe: 7.2 deg)`,
      );
      check(
        'LIVE tap Right 100ms (fa18 776kt): bank back <= 5 deg within 2 s (auto-levels)',
        Math.abs(atF(r, 2).bank) <= 5,
        `bank@2s=${atF(r, 2).bank.toFixed(1)} deg (before fix: 61.7 deg)`,
      );
    }
    // F2: the same tap at 287 kt still banks visibly (a few degrees).
    {
      const r = runLive('fa18', 287, (t) => {
        if (t === 0) return { keyDown: ['ArrowRight'] };
        if (t >= 0.1 && t < 0.3) return { keyUp: ['ArrowRight'] };
        return {};
      }, 2);
      check(
        'LIVE tap Right 100ms (fa18 287kt): peak bank 3-12 deg',
        r.peakBank >= 3 && r.peakBank <= 12,
        `peak bank=${r.peakBank.toFixed(1)} deg`,
      );
      check(
        'LIVE tap Right 100ms (fa18 287kt): bank back <= 5 deg within 2 s',
        Math.abs(atF(r, 2).bank) <= 5,
        `bank@2s=${atF(r, 2).bank.toFixed(1)} deg`,
      );
    }
    // F3: a 1 s hold at 776 kt rolls controllably and recovers.
    // Before: bank@1s 123.5 deg, peak 178 deg (inverted).
    {
      const r = runLive('fa18', 776, (t) => {
        if (t === 0) return { keyDown: ['ArrowRight'] };
        if (t >= 1.0 && t < 1.3) return { keyUp: ['ArrowRight'] };
        return {};
      }, 3);
      check(
        'LIVE hold Right 1s (fa18 776kt): bank@1s 20-55 deg',
        Math.abs(atF(r, 1).bank) >= 20 && Math.abs(atF(r, 1).bank) <= 55,
        `bank@1s=${atF(r, 1).bank.toFixed(1)} deg (before fix: 123.5 deg)`,
      );
      check(
        'LIVE hold Right 1s (fa18 776kt): peak bank <= 65 deg (never inverts)',
        r.peakBank <= 65,
        `peak bank=${r.peakBank.toFixed(1)} deg (before fix: 178.1 deg)`,
      );
      check(
        'LIVE hold Right 1s (fa18 776kt): bank back <= 15 deg within 3 s',
        Math.abs(atF(r, 3).bank) <= 15,
        `bank@3s=${atF(r, 3).bank.toFixed(1)} deg`,
      );
    }
    // F4: hands-off auto-level from steep banks (direct rollIn, fixed
    // dt — isolates the controller spring from the input chain).
    // Before: from 60 deg the bank sat at 59 deg after 1 s and 57
    // deg after 5 s; from 111 deg it sat at 105 deg after 5 s — the
    // |bank| < 45 deg gate never let the spring engage.
    for (const startBank of [60, 111]) {
      const spec = SPECS['fa18']!;
      const ac = new Aircraft({ scene, world, spec, isPlayer: true, modelKey: spec.modelKey, useBet: true });
      ac.terrainHeightAt = (x: number, z: number) => heightfieldHeightAt(heights, res, x, z);
      for (const h of ac.colliderHandles()) owners.set(h, ac);
      ac.resetAt(alpha.x, 300, alpha.z, alpha.hdg);
      ac.setAirspeed(287 / 1.94384);
      ac.thrTarget = 0.8;
      for (let i = 0; i < 240; i++) {
        const b = bankOf(ac);
        ac.rollIn = Math.max(-1, Math.min(1, (startBank - b) * 0.05));
        substep(ac, 1 / 60);
      }
      const b0 = bankOf(ac);
      ac.rollIn = 0;
      let bankAt3 = 0;
      for (let i = 0; i < 180; i++) {
        substep(ac, 1 / 60);
        if (i === 179) bankAt3 = bankOf(ac);
      }
      check(
        `hands-off auto-level from ${startBank} deg: |bank| <= 10 deg after 3 s`,
        Math.abs(bankAt3) <= 10,
        `from ${b0.toFixed(1)} deg to ${bankAt3.toFixed(1)} deg (before fix: 57-105 deg stuck)`,
      );
      ac.dispose();
      owners.clear();
      owners.set(terrainCollider.handle, 'terrain');
    }
    // F5: on-screen joystick, full drag RIGHT 1.5 s, F-35A at 700 kt.
    // Before: bank@1.5s 92.3 deg, peak 113.3 deg (the v4.7 live
    // check saw 59-81 deg from 1.5-2 s drags). After: a firm but
    // controllable roll that recovers on release.
    {
      const r = runLive('f35a', 700, (t) => (t < 1.5 ? { joyX: 1, joyActive: true } : { joyX: 0, joyActive: false }), 3);
      check(
        'LIVE JOY full drag RIGHT 1.5s (f35a 700kt): bank@1.5s 30-60 deg',
        Math.abs(atF(r, 1.5).bank) >= 30 && Math.abs(atF(r, 1.5).bank) <= 60,
        `bank@1.5s=${atF(r, 1.5).bank.toFixed(1)} deg (before fix: 92.3 deg)`,
      );
      check(
        'LIVE JOY full drag RIGHT 1.5s (f35a 700kt): peak bank <= 75 deg',
        r.peakBank <= 75,
        `peak bank=${r.peakBank.toFixed(1)} deg (before fix: 113.3 deg)`,
      );
      check(
        'LIVE JOY full drag RIGHT 1.5s (f35a 700kt): bank back <= 15 deg within 3 s',
        Math.abs(atF(r, 3).bank) <= 15,
        `bank@3s=${atF(r, 3).bank.toFixed(1)} deg`,
      );
    }
    // F6: on-screen joystick, HALF drag (0.5) 1.5 s, F-35A at 700 kt
    // — the realistic thumb drag, not the edge-to-edge case.
    {
      const r = runLive('f35a', 700, (t) => (t < 1.5 ? { joyX: 0.5, joyActive: true } : { joyX: 0, joyActive: false }), 3);
      check(
        'LIVE JOY half drag 0.5 RIGHT 1.5s (f35a 700kt): bank@1.5s 5-35 deg',
        Math.abs(atF(r, 1.5).bank) >= 5 && Math.abs(atF(r, 1.5).bank) <= 35,
        `bank@1.5s=${atF(r, 1.5).bank.toFixed(1)} deg`,
      );
    }
    // F7: the dead drag — tap Right, then drag LEFT on the stick
    // from t=0.3. Before: the PC kbRoll tail vetoed the touch stick
    // in the merge, so rollIn stayed +0.36..+0.08 until t=0.76 and
    // the bank kept rising to 55 deg the WRONG way. After: the
    // active stick wins, rollIn goes negative promptly and the bank
    // reverses.
    {
      const r = runLive('fa18', 700, (t) => {
        if (t === 0) return { keyDown: ['ArrowRight'] };
        if (t >= 0.1 && t < 0.3) return { keyUp: ['ArrowRight'] };
        if (t >= 0.3) return { joyX: -1, joyActive: true };
        return {};
      }, 2);
      check(
        'LIVE dead drag: left drag answers within 0.5 s (rollIn < 0 by t=0.8s)',
        atF(r, 0.8).rollIn < -0.05,
        `rollIn@0.8s=${atF(r, 0.8).rollIn.toFixed(2)} (before fix: +0.08 at t=0.76)`,
      );
      // The held left drag keeps rolling the jet left past level —
      // the assertion is that the bank DECREASES through level
      // (it was +55 deg and still rising before the fix), not that
      // it stops at zero.
      check(
        'LIVE dead drag: bank reverses through level (bank@1s < bank@0.5s and < 0)',
        atF(r, 1).bank < atF(r, 0.5).bank && atF(r, 1).bank < 0,
        `bank@0.5s=${atF(r, 0.5).bank.toFixed(1)} bank@1s=${atF(r, 1).bank.toFixed(1)} deg (before fix: 55 deg and still rising)`,
      );
    }
    // F8: the 80 deg bank-hold still holds — the guard for removing
    // the auto-level's |bank| < 45 deg gate. A PD hold at 80 deg for
    // 3 s (direct rollIn, fixed dt, like section B) must stay
    // within 15 deg of target: the spring only engages on frames
    // where the feedback rollIn is exactly 0, so it must not fight
    // a deliberate steep bank.
    {
      const ac = spawnAirborne('fa18');
      const h0 = hdgOf(ac);
      void h0;
      for (let i = 0; i < 180; i++) {
        const bank = bankOf(ac);
        ac.rollIn = Math.max(-1, Math.min(1, (80 - bank) * 0.05));
        substep(ac, 1 / 60);
      }
      const bank = bankOf(ac);
      check(
        'bank-hold 80 deg (PD, 3 s): auto-level does not fight a deliberate steep bank',
        Math.abs(bank - 80) < 15,
        `bank=${bank.toFixed(1)} deg`,
      );
      dispose(ac);
    }
    // F9: the keyboard roll scale itself.
    check(
      'kbRollScale: 1.0 at <= 200 kt, ~0.70 at 287 kt, ~0.34 at 586 kt, floored 0.15 at 1333+ kt',
      kbRollScale(100) === 1 &&
        kbRollScale(200) === 1 &&
        Math.abs(kbRollScale(287) - 200 / 287) < 0.01 &&
        Math.abs(kbRollScale(586) - 200 / 586) < 0.01 &&
        kbRollScale(2000) === 0.15,
      `s(100)=${kbRollScale(100)} s(287)=${kbRollScale(287).toFixed(2)} s(586)=${kbRollScale(586).toFixed(2)} s(2000)=${kbRollScale(2000)}`,
    );
    // F10: low-speed authority is preserved — a 1 s hold at 150 kt
    // (scale 1.0, takeoff/landing speeds) still banks decisively.
    {
      const r = runLive('fa18', 150, (t) => {
        if (t === 0) return { keyDown: ['ArrowRight'] };
        if (t >= 1.0 && t < 1.3) return { keyUp: ['ArrowRight'] };
        return {};
      }, 2);
      check(
        'LIVE hold Right 1s (fa18 150kt): banks RIGHT >= 15 deg (low-speed authority kept)',
        atF(r, 1).bank >= 15,
        `bank@1s=${atF(r, 1).bank.toFixed(1)} deg`,
      );
    }
    pcF.dispose();
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
