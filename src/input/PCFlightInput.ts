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
/** Keyboard stick slew rate (per second): how fast the virtual stick
 *  moves toward the held key's target. 5/s (was 3/s): at 3/s a 0.1 s
 *  tap only reached 0.3 of full pull and the live F-18 did not
 *  rotate from a tap at 142 kt; 5/s gives a meaningful pull from a
 *  short hold while a tap still does not rotate the jet by itself
 *  (rotation needs a SUSTAINED pull — the ROTATE cue says so). */
const STICK_SLEW = 5;
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
  private throttle = 0;
  private camMode = 0; // 0 chase, 1 cockpit, 2 free-orbit
  private lastPoll = performance.now();
  private shaping: StickShaping = { ...DEFAULT_SHAPING };
  /** Slewed keyboard stick (ramps toward the arrow-key target). */
  private kbPitch = 0;
  private kbRoll = 0;
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
    this.yoke.dx = 0;
    this.yoke.dy = 0;
  }

  get gamepadName(): string {
    return this.gpName;
  }

  getControls(): FlightControls {
    const now = performance.now();
    const dt = Math.min(0.1, Math.max(0.001, (now - this.lastPoll) / 1000));
    this.lastPoll = now;

    this.pollGamepad();

    // Base: gamepad wins, else mouse yoke, else zero (keyboard adds below).
    let px = 0;
    let rx = 0;
    if (this.gpActive) {
      px = this.gpPitch;
      rx = this.gpRoll;
    } else if (this.yoke.active && this.camMode !== 2) {
      px = clamp(this.yoke.dy / 60, -1, 1);
      rx = clamp(-this.yoke.dx / 60, -1, 1);
    }
    // Keyboard stick: arrow keys (+ A/D roll) give a -1..1 target; the
    // stick slews toward it at STICK_SLEW/s, then inversion + the
    // sensitivity exponent shape it. This is the keyboard-only path —
    // the mouse yoke and gamepad above are analogue and unshaped.
    const kbPitchTarget =
      (this.keys.has('ArrowDown') ? 1 : 0) - (this.keys.has('ArrowUp') ? 1 : 0);
    const kbRollTarget =
      (this.keys.has('ArrowLeft') ? 1 : 0) -
      (this.keys.has('ArrowRight') ? 1 : 0) +
      (this.keys.has('KeyA') ? 1 : 0) -
      (this.keys.has('KeyD') ? 1 : 0);
    const slew = (cur: number, target: number, rate: number): number => {
      const step = rate * dt;
      if (cur < target) return Math.min(target, cur + step);
      if (cur > target) return Math.max(target, cur - step);
      return cur;
    };
    this.kbPitch = slew(this.kbPitch, clamp(kbPitchTarget, -1, 1), PITCH_SLEW);
    this.kbRoll = slew(this.kbRoll, clamp(kbRollTarget, -1, 1), STICK_SLEW);
    const shape = (v: number, gamma: number, invert: boolean): number => {
      const s = invert ? -v : v;
      return Math.sign(s) * Math.pow(Math.abs(s), gamma);
    };
    px += shape(this.kbPitch, this.shaping.pitchSens, this.shaping.invertPitch);
    rx += shape(this.kbRoll, this.shaping.rollSens, this.shaping.invertRoll);

    // Rudder: Q = left, E = right (yawIn > 0 = nose right, solver yaw).
    const yaw =
      (this.keys.has('KeyE') ? 1 : 0) - (this.keys.has('KeyQ') ? 1 : 0);

    // Throttle: incremental while held; null = no opinion this frame.
    let throttle: number | null = null;
    if (
      this.keys.has('KeyW') ||
      this.keys.has('ShiftLeft') ||
      this.keys.has('ShiftRight')
    ) {
      this.throttle = clamp(this.throttle + dt * 0.7, 0, 1);
      throttle = this.throttle;
    }
    if (
      this.keys.has('KeyS') ||
      this.keys.has('ControlLeft') ||
      this.keys.has('ControlRight')
    ) {
      this.throttle = clamp(this.throttle - dt * 0.7, 0, 1);
      throttle = this.throttle;
    }
    if (this.gpThrDelta !== 0) {
      this.throttle = clamp(this.throttle + this.gpThrDelta * dt, 0, 1);
      throttle = this.throttle;
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
      fireHeld: this.keys.has('Space') || this.gpFire,
      missilePressed: this.keys.has('KeyM') || this.gpEdge.missile,
      flarePressed: this.keys.has('KeyF') || this.gpEdge.flare,
      gearPressed: this.keys.has('KeyG') || this.gpEdge.gear,
      hookPressed: this.keys.has('KeyH'),
      cameraPressed: this.keys.has('KeyC'),
      cameraResetPressed: this.keys.has('KeyR'),
      pausePressed:
        this.keys.has('KeyP') || this.keys.has('Escape') || this.gpEdge.pause,
      bombPressed: this.keys.has('KeyN'),
      brakeHeld: this.keys.has('KeyB'),
      flapsUpPressed: flapUp,
      flapsDownPressed: flapDown,
      airbrakePressed: this.keys.has('KeyX'),
      vtolPressed: this.keys.has('KeyV'),
    };
    // Single-shot keys are consumed so a held key does not repeat.
    // BracketLeft / BracketRight are NOT here — they are held keys
    // with their own 0.25 s repeat above.
    for (const k of ['KeyM', 'KeyF', 'KeyG', 'KeyH', 'KeyC', 'KeyR', 'KeyP', 'Escape', 'KeyN', 'KeyX', 'KeyV']) {
      this.keys.delete(k);
    }
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

  private readonly onKeyDown = (e: KeyboardEvent): void => {
    if (e.repeat) return;
    // Space scrolls the page — prevent it while flying.
    if (e.code === 'Space') e.preventDefault();
    this.keys.add(e.code);
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
      this.gpRoll = -ax0;
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
