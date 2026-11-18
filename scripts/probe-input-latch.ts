// probe-input-latch.ts — pass 6, items 7/10b: a key tap shorter than
// one frame (keydown + keyup between two consumeEvents / getControls
// calls) must still register. Before the fix the live `keys` set was
// already empty when the input was polled, so V did nothing for VTOL,
// R did nothing during the wreck-watching camera, and quick W /
// ArrowUp taps never moved the throttle / stick.
// Usage: esbuild scripts/probe-input-latch.ts --bundle --platform=node
//   --format=esm --outfile=/tmp/probe-input-latch.mjs && node /tmp/probe-input-latch.mjs

// Minimal DOM stubs: PCFlightInput only needs addEventListener on
// window + canvas, performance.now, and navigator.getGamepads.
type Handler = (e: { code: string; repeat?: boolean; preventDefault?: () => void }) => void;
const winHandlers: Record<string, Handler[]> = {};
(globalThis as unknown as { window: unknown }).window = {
  addEventListener: (type: string, fn: Handler) => {
    (winHandlers[type] ??= []).push(fn);
  },
  removeEventListener: () => {},
};
Object.defineProperty(globalThis, 'navigator', {
  value: { getGamepads: () => [] },
  configurable: true,
});
const canvasStub = {
  addEventListener: () => {},
  removeEventListener: () => {},
} as unknown as HTMLCanvasElement;

import { PCFlightInput } from '../src/input/PCFlightInput';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

function key(code: string, type: 'keydown' | 'keyup'): void {
  for (const fn of winHandlers[type] ?? []) {
    fn({ code, repeat: false, preventDefault: () => {} });
  }
}

function main(): void {
  const input = new PCFlightInput(canvasStub);

  // 1. Quick V tap (shorter than a frame): keydown + keyup, then poll.
  key('KeyV', 'keydown');
  key('KeyV', 'keyup');
  const ev1 = input.consumeEvents();
  check('quick V tap reaches consumeEvents (vtolPressed)', ev1.vtolPressed === true);
  // ... and exactly once (a held V must not repeat).
  const ev2 = input.consumeEvents();
  check('V tap reports exactly once', ev2.vtolPressed === false);

  // 2. Quick R tap.
  key('KeyR', 'keydown');
  key('KeyR', 'keyup');
  const ev3 = input.consumeEvents();
  check('quick R tap reaches consumeEvents (cameraResetPressed)', ev3.cameraResetPressed === true);

  // 3. Quick W tap: throttle must move (non-null, > 0).
  const c0 = input.getControls();
  const thr0 = c0.throttle ?? 0;
  key('KeyW', 'keydown');
  key('KeyW', 'keyup');
  const c1 = input.getControls();
  check(
    'quick W tap raises the throttle',
    c1.throttle !== null && (c1.throttle ?? 0) > thr0,
    `throttle ${thr0} -> ${c1.throttle}`,
  );

  // 4. Quick ArrowUp tap: pitch must go nose-up (negative).
  key('ArrowUp', 'keydown');
  key('ArrowUp', 'keyup');
  const c2 = input.getControls();
  check('quick ArrowUp tap pitches nose-up', c2.pitch < 0, `pitch=${c2.pitch.toFixed(3)}`);

  // 5. Held keys still work: hold W across several polls — the
  // throttle must rise monotonically (each poll slews a little; in
  // node the polls run back-to-back so dt clamps to its 0.001 floor
  // and the absolute rise is small — the real frame time drives it
  // live).
  key('KeyW', 'keydown');
  const tFirst = input.getControls().throttle ?? 0;
  let thr = tFirst;
  for (let i = 0; i < 10; i++) {
    const c = input.getControls();
    thr = c.throttle ?? thr;
  }
  check('held W slews the throttle up', thr > tFirst && thr > 0, `throttle ${tFirst.toFixed(4)} -> ${thr.toFixed(4)}`);
  key('KeyW', 'keyup');

  // 6. Held Space still fires continuously (not consumed by the latch).
  key('Space', 'keydown');
  const eA = input.consumeEvents();
  const eB = input.consumeEvents();
  check('held Space fires on consecutive polls', eA.fireHeld === true && eB.fireHeld === true);
  key('Space', 'keyup');
  const eC = input.consumeEvents();
  check('released Space stops firing', eC.fireHeld === false);

  if (failures > 0) {
    console.log(`${failures} INPUT-LATCH CHECK(S) FAILED`);
    process.exit(1);
  }
  console.log('ALL INPUT-LATCH CHECKS PASSED');
}

main();
