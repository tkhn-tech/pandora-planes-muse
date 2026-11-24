// PCFlightInput — keyboard + mouse-yoke + Gamepad API, merged in this
// priority (documented, template pollGamepad/playerInput):
//   gamepad > mouse-yoke > keyboard
// A higher-priority device replaces (not adds to) the stick when active;
// keyboard arrows add on top of whatever the stick is doing, as in the
// template. Throttle is incremental: W/S (or Shift/Ctrl) slew the stored
// throttle target at 0.7/s; gamepad triggers slew it at 1.2/s. getControls()
// returns throttle as a number only on frames where a throttle key/trigger
// is held — otherwise null, meaning "keep the aircraft's current target"
// (the touch slider in main.ts owns the value the rest of the time).
//
// Keys (template ~1795–1810): W/S throttle, arrows = stick pitch/roll,
// A/D roll, G gear, H hook, F flares, M missile, N bomb, C camera,
// R camera reset, P pause, B hold brakes, Space hold fire.
// Gamepad (template pollGamepad): left stick pitch/roll (deadzone 0.12),
// triggers throttle (RT up / LT down), A/RB fire, B missile, X flares,
// Y gear, Start pause.

import type { FlightControls } from '../physics/IAerodynamicsSolver';
import type { IFlightInputProvider } from './IFlightInputProvider';

export interface PCInputEvents {
  fireHeld: boolean;
  missilePressed: boolean;
  flarePressed: boolean;
  gearPressed: boolean;
  hookPressed: boolean;
  cameraPressed: boolean;
  cameraResetPressed: boolean;
  pausePressed: boolean;
  bombPressed: boolean;
  brakeHeld: boolean;
  /** Flaps step keys: [ = up 10 deg, ] = down 10 deg. */
  flapsUpPressed: boolean;
  flapsDownPressed: boolean;
  /** X toggles the airbrake. */
  airbrakePressed: boolean;
  /** V toggles VTOL hover (F-35A lift-fan mode). */
  vtolPressed: boolean;
}

/** Keyboard stick shaping, from the setup menu (SandboxConfig). The
 *  exponent gamma is applied as out = sign(x) * |x|^gamma (1.0 =
 *  linear); inversion flips the axis. Applies to the KEYBOARD stick
 *  only — the mouse yoke and gamepad are analogue and unaffected. */
export interface StickShaping {
  invertPitch: boolean;
  invertRoll: boolean;
  pitchSens: number;
  rollSens: number;
}

const DEFAULT_SHAPING: StickShaping = {
  invertPitch: false,
  invertRoll: false,
  pitchSens: 1,
  rollSens: 1,
};

/** Keyboard stick slew, units/s — the arrow-key stick ramps to its
 *  target instead of jumping, so it moves like a real stick. */
/** Keyboard ROLL slew rates (per second), pass 13 worker B, retimed
 *  pass 14 worker A. The pass-12 asymmetric slew (UP 12/s, DOWN
 *  2.5/s) is far too strong LIVE at high speed / variable dt — and
 *  the pass-13 retune (UP 7/s, DOWN 3/s, per-call cap 0.4,
 *  kbRollScale) swung the other way live: a 100 ms tap at 223 kt
 *  banked 1.1 deg (probe-pass14 S1) while the same tap banked 6.5
 *  deg in probe-roll's ~17 fps harness. The cause is WHERE the slew
 *  clock comes from: getControls() derived its dt from the WALL
 *  clock (performance.now() - lastPoll), but it is called once per
 *  SUBSTEP while wall time advances (almost) only between FRAMES —
 *  the render takes the frame. So the slew advanced ~once per frame
 *  (first substep dt ~0.1, the rest ~0.002): the effective slew rate
 *  was 0.4/frame, i.e. 1.6/s at 4 fps and 7/s at 60 fps, and a tap
 *  shorter than one frame moved the stick exactly once. The stick's
 *  effect is integrated in SIM time, so the slew must run in SIM
 *  time too: main.ts playerInput passes its substep dt into
 *  getControls(simDt), and the slew advances rate * simDt per
 *  substep — 7/s of sim at any frame rate, so a 100 ms tap slews
 *  the stick to ~0.7 whether the frame is 16 ms or 250 ms, and a
 *  1 s hold reaches full deflection in ~0.14 s of sim. The wall
 *  clock remains only as the fallback when no simDt is passed (old
 *  probes / other callers). With simDt <= 0.05 the per-call step is
 *  at most 7 * 0.05 = 0.35, so the 0.40 cap is now a safety bound,
 *  not the tap-impulse limiter it was in pass 13.
 *  The UP/DOWN asymmetry and the speed scale are unchanged: the
 *  return is 3/s (pass 12 used 2.5/s): the long tail was pass 12's
 *  way to make a tap visible, but with the speed scale the tail no
 *  longer needs to linger — the bank a tap creates (~5-8 deg)
 *  persists through the roll-inertia overshoot, and the auto-level
 *  now genuinely returns the wings — and a shorter tail also
 *  shortens the window in which a stale PC deflection vetoes the
 *  touch stick in the main.ts merge.
 *  Pass 14 (worker A): UP 7 -> 10/s. At 7/s a 100 ms tap (0.10 s
 *  of sim at live frame rates) slewed the stick only to 0.7, and at
 *  223 kt — where the BET roll rate at full deflection is only
 *  ~25-30 deg/s — that banked 2.4 deg (probe-pass14 S1), below the
 *  3-12 deg sane-tap band. 10/s traverses full stick travel in
 *  0.10 s of sim, i.e. inside one tap, so the tap commands ~full
 *  scaled deflection: 223 kt taps bank ~4 deg, 524/776 kt taps
 *  ~6/5 deg, and a 1 s hold still banks progressively without
 *  inverting (probe-pass14 S1/S2/S3/S4). This is NOT the pass-12
 *  12/s overshoot: that one slewed on the wall clock, so the first
 *  substep of a 250 ms frame jumped 1.2 (full deflection in one
 *  call) and the 2.5/s tail held it ~0.4 s; the sim-time slew
 *  advances at most 10 * 0.05 = 0.50 per substep and the tap's
 *  stick-time integral stays bounded by the tap's sim duration. */
const ROLL_SLEW_UP = 10;
const ROLL_SLEW_DOWN = 3;
/** Max keyboard-stick movement in one getControls() call (stick
 *  units). Bounds the tap impulse at low FPS: without it a 250 ms
 *  frame slews 8 * 0.1 = 0.8 in one call. */
const ROLL_SLEW_STEP_MAX = 0.4;
/** Keyboard roll authority vs indicated airspeed, pass 13 worker B.
 *  1.0 at/below 200 kt, falling as 200/ias above it, floored at 0.15.
 *  The 200 kt reference (not 150) is deliberate: the tap impulse is
 *  the same stick travel at every speed, so the scale must make
 *  scale x roll-rate roughly constant — the BET roll rate grows
 *  ~linearly with speed (bank@1 s at full deflection: 25 deg at
 *  150 kt, 58 at 287, 131 at 586, 144 at 776), and 200/ias tracks
 *  it: a 100 ms tap banks ~6-8 deg at 287, 586 AND 776 kt, and a
 *  1 s hold banks ~35-50 deg at any of them. (150/ias made the tap
 *  ~3 deg at 287 kt — barely visible — while 776 kt was no better.)
 *  Set from main.ts playerInput via setAirspeedKt(); 0 / unset
 *  means "unknown", scale 1 (the old behaviour — probes that never
 *  set it are unaffected). */
export function kbRollScale(iasKt: number): number {
  if (!Number.isFinite(iasKt) || iasKt <= 200) return 1;
  return Math.max(0.15, Math.min(1, 200 / iasKt));
}
/** Keyboard PITCH slew rate (per second) — slower than roll (live
 *  finding 8): one ArrowUp tap at 5/s swung the vertical speed by
 *  5,000-13,000 fpm and the pitch persisted (enough to loop/stall),
 *  because the curve solver is a pure rate command (55 deg/s) with no
 *  damping. 3/s for pitch keeps a tap from over-rotating while a held
 *  pull still rotates the jet for takeoff. Roll stays at 5/s. */
const PITCH_SLEW = 3;

const DEADZONE = 0.12;

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

export class PCFlightInput implements IFlightInputProvider {
  private readonly canvas: HTMLCanvasElement;
  private readonly keys = new Set<string>();
  /** Latched discrete presses (pass 6, items 7/10b): onKeyDown records
   *  a press here as well as in `keys`; consumeEvents() reads and
   *  clears it. A tap shorter than one frame (keydown + keyup between
   *  two consumeEvents calls — common on a slow device, where a frame
   *  is 50-100 ms) was lost entirely, because consumeEvents only read
   *  the live `keys` set and onKeyUp had already deleted the code.
   *  That is why V did nothing for VTOL, R did nothing during the
   *  wreck-watching camera, and quick W / ArrowUp taps never moved
   *  the throttle / stick live while the on-screen controls worked.
   *  Held keys (W/S/arrows/Space/B/brackets) still read from `keys`;
   *  only the single-shot edge keys are latched. */
  private readonly pressed = new Set<string>();
  private throttle = 0;
  private camMode = 0; // 0 chase, 1 cockpit, 2 free-orbit
  private lastPoll = performance.now();
  private shaping: StickShaping = { ...DEFAULT_SHAPING };
  /** Slewed keyboard stick (ramps toward the arrow-key target). */
  private kbPitch = 0;
  private kbRoll = 0;
  /** Indicated airspeed (kt) for the keyboard roll scale — set by
   *  main.ts playerInput each frame; 0 = unknown, scale 1. */
  private iasKt = 0;
  /** Flaps repeat timers (live finding 8): holding [ or ] steps the
   *  flaps every 0.25 s. consumeEvents reports the flag only when the
   *  interval has elapsed since the last report, so a held key
   *  repeats but a tap steps exactly once. */
  private lastFlapUpMs = -1e9;
  private lastFlapDownMs = -1e9;

  // Mouse yoke: drag on the canvas steers in chase/cockpit cam (template
  // ~1750); in free-orbit the same drag orbits (main.ts owns that path).
  private readonly yoke = {
    active: false,
    id: -1,
    dx: 0,
    dy: 0,
    sx: 0,
    sy: 0,
  };

  // Gamepad edge state.
  private gpPrev: boolean[] = [];
  private gpName = '';
  private readonly gpEdge = {
    missile: false,
    flare: false,
    gear: false,
    pause: false,
  };
  private gpFire = false;
  private gpThrDelta = 0;
  private gpPitch = 0;
  private gpRoll = 0;
  private gpActive = false;

  private disposed = false;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    window.addEventListener('keydown', this.onKeyDown, { passive: false });
    window.addEventListener('keyup', this.onKeyUp);
    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);
    // Pass 8 (item 1): the canvas never calls setPointerCapture, so a
    // pointerup OUTSIDE the canvas never reaches the canvas listener
    // and yoke.active sticks true with a frozen dx/dy — a permanent
    // full pitch+roll deflection on the next life. A window-level
    // release catches the pointerup wherever it lands.
    window.addEventListener('pointerup', this.onPointerUp);
    window.addEventListener('pointercancel', this.onPointerUp);
  }

  /** Free-orbit drag belongs to main.ts; tell this provider the mode so
   *  the mouse yoke only steers in chase/cockpit (template: camMode 2
   *  keeps the orbit role). */
  setCameraMode(mode: number): void {
    this.camMode = mode;
    if (mode === 2) {
      this.yoke.active = false;
      this.yoke.dx = 0;
      this.yoke.dy = 0;
    }
  }

  /** Stick shaping from the setup menu (inversion + sensitivity). */
  setStickShaping(shaping: Partial<StickShaping>): void {
    this.shaping = { ...this.shaping, ...shaping };
  }

  setAircraftKind(_kind: 'fixed-wing' | 'vtol' | 'rotary'): void {
    // No per-kind behaviour on PC.
  }

  calibrate(): void {
    // Pass 8 (item 1): a calibration is a fresh neutral state — clear
    // the yoke engagement as well as the deflection, so a yoke stuck
    // active from a previous life cannot carry a frozen deflection
    // into the new one. Pass 13: the slewed keyboard stick too — a
    // kbRoll tail from the previous life would otherwise keep rolling
    // the fresh jet (and veto the touch stick in the merge) for 0.4 s.
    this.yoke.active = false;
    this.yoke.dx = 0;
    this.yoke.dy = 0;
    this.kbPitch = 0;
    this.kbRoll = 0;
  }

  get gamepadName(): string {
    return this.gpName;
  }

  getControls(simDt?: number): FlightControls {
    // Pass 14 (worker A): the slew clock. When the caller passes the
    // simulation substep dt (main.ts playerInput does), the stick and
    // throttle slew in SIM time — the rate is then frame-rate
    // independent (see the ROLL_SLEW comment above). Otherwise fall
    // back to the wall clock, as before.
    const now = performance.now();
    const wallDt = Math.min(0.1, Math.max(0.001, (now - this.lastPoll) / 1000));
    this.lastPoll = now;
    const dt =
      simDt !== undefined && Number.isFinite(simDt) && simDt > 0
        ? Math.min(0.1, Math.max(0.001, simDt))
        : wallDt;

    this.pollGamepad();

    // Held keys (pass 6, item 10b): a tap shorter than one frame
    // (keydown + keyup between two getControls calls) was lost
    // entirely, because the live `keys` set was already empty when
    // getControls ran — W never raised the throttle and ArrowUp
    // never pitched from a quick tap. A key latched in `pressed`
    // counts as held for this one call, then is consumed below, so
    // a tap still moves the stick / throttle by the frame's slew.
    // Single-shot edge keys are NOT consumed here — consumeEvents()
    // owns them.
    const held = (code: string): boolean =>
      this.keys.has(code) || this.pressed.has(code);

    // Base: gamepad wins, else mouse yoke, else zero (keyboard adds below).
    let px = 0;
    let rx = 0;
    if (this.gpActive) {
      px = this.gpPitch;
      rx = this.gpRoll;
    } else if (this.yoke.active && this.camMode !== 2) {
      px = clamp(this.yoke.dy / 60, -1, 1);
      rx = clamp(this.yoke.dx / 60, -1, 1); // pass 11: was -dx (inverted roll)
    }
    // Keyboard stick: arrow keys (+ A/D roll) give a -1..1 target; the
    // stick slews toward it (pitch at PITCH_SLEW/s, roll at
    // ROLL_SLEW_UP/s rising and ROLL_SLEW_DOWN/s returning), then
    // inversion + the sensitivity exponent shape it. This is the
    // keyboard-only path — the mouse yoke and gamepad above are
    // analogue and unshaped.
    const kbPitchTarget =
      (held('ArrowDown') ? 1 : 0) - (held('ArrowUp') ? 1 : 0);
    // Pass 11 (item 7): the roll was INVERTED on every player input
    // path. The physics maps rollIn > 0 to a RIGHT bank (positive
    // rotation about the nose axis +Z tips the right wingtip, -X,
    // down), but every input produced rollIn = -1 for "right":
    // ArrowRight/D gave -1, the mouse yoke negated dx, the gamepad
    // negated axis 0, the touch merge negated joy.x. Pass 1 had
    // concluded "already correct" because probe-roll's own ground
    // truth was inverted (it asserted rollIn = -1 banks right).
    // All paths now produce rollIn = +1 for "steer right". The AI
    // already assumes rollIn > 0 = more right bank and is unchanged;
    // the auto-level operates on rates, not rollIn sign, and is
    // unchanged. Pitch is NOT inverted (ArrowUp = nose up).
    const kbRollTarget =
      (held('ArrowRight') ? 1 : 0) -
      (held('ArrowLeft') ? 1 : 0) +
      (held('KeyD') ? 1 : 0) -
      (held('KeyA') ? 1 : 0);
    const slew = (cur: number, target: number, rate: number): number => {
      const step = Math.min(rate * dt, ROLL_SLEW_STEP_MAX);
      if (cur < target) return Math.min(target, cur + step);
      if (cur > target) return Math.max(target, cur - step);
      return cur;
    };
    // Roll slews asymmetrically (pass 12): fast toward the held key,
    // slow back to centre — see ROLL_SLEW_UP / ROLL_SLEW_DOWN. The
    // step cap (ROLL_SLEW_STEP_MAX) bounds the per-call movement so
    // a low-FPS frame cannot snap the stick to full deflection.
    const slewRoll = (cur: number, target: number): number => {
      const rising = Math.abs(target) > Math.abs(cur);
      return slew(cur, target, rising ? ROLL_SLEW_UP : ROLL_SLEW_DOWN);
    };
    this.kbPitch = slew(this.kbPitch, clamp(kbPitchTarget, -1, 1), PITCH_SLEW);
    this.kbRoll = slewRoll(this.kbRoll, clamp(kbRollTarget, -1, 1));
    // Speed-scaled keyboard roll (pass 13): the slewed stick is
    // multiplied by kbRollScale(ias) BEFORE shaping, so at high
    // speed a tap or a hold commands a fraction of full deflection.
    // The mouse yoke / gamepad above are unscaled (proportional).
    const kbRollScaled = this.kbRoll * kbRollScale(this.iasKt);
    const shape = (v: number, gamma: number, invert: boolean): number => {
      const s = invert ? -v : v;
      return Math.sign(s) * Math.pow(Math.abs(s), gamma);
    };
    px += shape(this.kbPitch, this.shaping.pitchSens, this.shaping.invertPitch);
    rx += shape(kbRollScaled, this.shaping.rollSens, this.shaping.invertRoll);

    // Rudder: Q = left, E = right. BOTH solvers map yawIn > 0 to nose
    // LEFT (BET fin moment; curve solver +yawRate term — positive
    // yawRate rotates the nose toward +X, the left wing), so left
    // rudder feeds +1 and right rudder -1. Pass 11: this had E -> +1,
    // so E yawed the nose left — the same inversion family as roll.
    const yaw =
      (held('KeyQ') ? 1 : 0) - (held('KeyE') ? 1 : 0);

    // Throttle: incremental while held; null = no opinion this frame.
    let throttle: number | null = null;
    if (
      held('KeyW') ||
      held('ShiftLeft') ||
      held('ShiftRight')
    ) {
      this.throttle = clamp(this.throttle + dt * 0.7, 0, 1);
      throttle = this.throttle;
    }
    if (
      held('KeyS') ||
      held('ControlLeft') ||
      held('ControlRight')
    ) {
      this.throttle = clamp(this.throttle - dt * 0.7, 0, 1);
      throttle = this.throttle;
    }
    if (this.gpThrDelta !== 0) {
      this.throttle = clamp(this.throttle + this.gpThrDelta * dt, 0, 1);
      throttle = this.throttle;
    }

    // Consume the held-key taps latched in `pressed` (one getControls
    // call per tap): a tap is held for this call only. Edge keys stay
    // latched for consumeEvents().
    for (const k of [
      'ArrowDown', 'ArrowUp', 'ArrowLeft', 'ArrowRight', 'KeyA', 'KeyD',
      'KeyQ', 'KeyE', 'KeyW', 'KeyS', 'ShiftLeft', 'ShiftRight',
      'ControlLeft', 'ControlRight',
    ]) {
      this.pressed.delete(k);
    }

    return {
      pitch: clamp(px, -1, 1),
      roll: clamp(rx, -1, 1),
      yaw,
      throttle,
    };
  }

  /** Discrete + held actions since the last call. Edge flags (missile,
   *  flare, gear, hook, camera, pause, bomb, flaps, airbrake) are
   *  consumed — each press reports exactly once. Call once per frame
   *  from main.ts. */
  consumeEvents(): PCInputEvents {
    // Flaps repeat (live finding 8): [ / ] are HELD keys, not
    // single-shot — holding one steps the flaps every 0.25 s. The
    // key stays in `keys` while held (it is NOT deleted below), and
    // the flag reports only when the interval has elapsed since the
    // last report, so a tap steps exactly once. onKeyDown ignores
    // e.repeat, so the repeat is timed here, not by the OS.
    const nowMs = performance.now();
    const flapUp =
      this.keys.has('BracketLeft') && nowMs - this.lastFlapUpMs >= 250;
    if (flapUp) this.lastFlapUpMs = nowMs;
    const flapDown =
      this.keys.has('BracketRight') && nowMs - this.lastFlapDownMs >= 250;
    if (flapDown) this.lastFlapDownMs = nowMs;
    const ev: PCInputEvents = {
      fireHeld: this.keys.has('Space') || this.pressed.has('Space') || this.gpFire,
      missilePressed: this.pressed.has('KeyM') || this.gpEdge.missile,
      flarePressed: this.pressed.has('KeyF') || this.gpEdge.flare,
      gearPressed: this.pressed.has('KeyG') || this.gpEdge.gear,
      hookPressed: this.pressed.has('KeyH'),
      cameraPressed: this.pressed.has('KeyC'),
      cameraResetPressed: this.pressed.has('KeyR'),
      pausePressed:
        this.pressed.has('KeyP') || this.pressed.has('Escape') || this.gpEdge.pause,
      bombPressed: this.pressed.has('KeyN'),
      brakeHeld: this.keys.has('KeyB') || this.pressed.has('KeyB'),
      flapsUpPressed: flapUp,
      flapsDownPressed: flapDown,
      airbrakePressed: this.pressed.has('KeyX'),
      vtolPressed: this.pressed.has('KeyV'),
    };
    // Single-shot keys are consumed so a held key does not repeat.
    // BracketLeft / BracketRight are NOT here — they are held keys
    // with their own 0.25 s repeat above. Space / KeyB are held keys
    // too — they are cleared from `pressed` (the tap latch) only,
    // NOT from `keys`, so holding them keeps firing / braking.
    for (const k of ['KeyM', 'KeyF', 'KeyG', 'KeyH', 'KeyC', 'KeyR', 'KeyP', 'Escape', 'KeyN', 'KeyX', 'KeyV']) {
      this.keys.delete(k);
      this.pressed.delete(k);
    }
    this.pressed.delete('Space');
    this.pressed.delete('KeyB');
    this.gpEdge.missile = false;
    this.gpEdge.flare = false;
    this.gpEdge.gear = false;
    this.gpEdge.pause = false;
    return ev;
  }

  /** Sync the stored throttle target (e.g. after the touch slider moves
   *  it, or on respawn) so W/S slews from the right value. */
  setThrottle(value01: number): void {
    this.throttle = clamp(value01, 0, 1);
  }

  /** Indicated airspeed for the keyboard roll scale (pass 13). Called
   *  by main.ts playerInput every frame, before getControls(). */
  setAirspeedKt(iasKt: number): void {
    this.iasKt = Number.isFinite(iasKt) ? Math.max(0, iasKt) : 0;
  }

  private readonly onKeyDown = (e: KeyboardEvent): void => {
    if (e.repeat) return;
    // Space scrolls the page — prevent it while flying.
    if (e.code === 'Space') e.preventDefault();
    this.keys.add(e.code);
    // Latch the press (pass 6, items 7/10b): a tap shorter than one
    // frame must still reach consumeEvents() even if keyup already
    // removed the code from `keys`.
    this.pressed.add(e.code);
  };

  private readonly onKeyUp = (e: KeyboardEvent): void => {
    this.keys.delete(e.code);
  };

  private readonly onPointerDown = (e: PointerEvent): void => {
    if (this.camMode === 2) return; // free-orbit drag is main.ts's job
    this.yoke.active = true;
    this.yoke.id = e.pointerId;
    this.yoke.sx = e.clientX;
    this.yoke.sy = e.clientY;
    this.yoke.dx = 0;
    this.yoke.dy = 0;
  };

  private readonly onPointerMove = (e: PointerEvent): void => {
    if (this.yoke.active && e.pointerId === this.yoke.id) {
      this.yoke.dx = e.clientX - this.yoke.sx;
      this.yoke.dy = e.clientY - this.yoke.sy;
    }
  };

  private readonly onPointerUp = (e: PointerEvent): void => {
    if (this.yoke.active && e.pointerId === this.yoke.id) {
      this.yoke.active = false;
      this.yoke.dx = 0;
      this.yoke.dy = 0;
    }
  };

  private pollGamepad(): void {
    this.gpActive = false;
    this.gpPitch = 0;
    this.gpRoll = 0;
    this.gpThrDelta = 0;
    this.gpFire = false;
    let gp: Gamepad | null = null;
    try {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const g of pads) {
        if (g && g.connected) {
          gp = g;
          break;
        }
      }
    } catch {
      // Gamepad API unavailable — keyboard/mouse only.
    }
    if (!gp) {
      this.gpName = '';
      this.gpPrev = [];
      return;
    }
    this.gpName = gp.id || 'gamepad';
    const dz = (v: number): number => (Math.abs(v) < DEADZONE ? 0 : v);
    const ax0 = dz(gp.axes[0] ?? 0);
    const ax1 = dz(gp.axes[1] ?? 0);
    if (ax0 !== 0 || ax1 !== 0) {
      this.gpActive = true;
      this.gpRoll = ax0; // pass 11: was -ax0 (inverted roll; stick right = +x = roll right)
      this.gpPitch = ax1;
    }
    const b = (i: number): boolean => !!(gp!.buttons[i]?.pressed);
    const bv = (i: number): number => gp!.buttons[i]?.value ?? 0;
    this.gpThrDelta = (bv(7) - bv(6)) * 1.2; // triggers: RT up, LT down
    if (this.gpThrDelta !== 0) this.gpActive = true;
    this.gpFire = b(0) || b(5); // A / RB
    const edge = (i: number): boolean => b(i) && !this.gpPrev[i];
    if (edge(1)) this.gpEdge.missile = true; // B: missile
    if (edge(2)) this.gpEdge.flare = true; // X: flares
    if (edge(3)) this.gpEdge.gear = true; // Y: gear
    if (edge(9)) this.gpEdge.pause = true; // Start: pause
    this.gpPrev = [];
    gp.buttons.forEach((btn, i) => {
      this.gpPrev[i] = btn.pressed;
    });
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointermove', this.onPointerMove);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);
  }
}
