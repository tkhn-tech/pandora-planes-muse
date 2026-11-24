// IFlightInputProvider — the single interface every control device speaks.
// main.ts merges providers in this priority (template playerInput, v2):
//   touch stick (wired in main.ts) -> mouse yoke -> tilt -> gamepad -> keyboard
// Inside PCFlightInput the merge is gamepad > mouse-yoke > keyboard.
// getControls() returns the merged stick + throttle for this frame; discrete
// actions (fire held, missile/flare/gear presses) are exposed by the concrete
// classes via consumeEvents() — they are device-specific, so they are not
// part of this interface.

import type { FlightControls } from '../physics/IAerodynamicsSolver';

export interface IFlightInputProvider {
  /** Merged stick/yaw/throttle for this frame. Throttle is null when this
   *  provider has no throttle opinion this frame (main keeps the last value).
   *  Pass 14 (worker A): callers that step in sim time pass the substep
   *  dt so the provider's internal slews (keyboard stick, throttle)
   *  advance in SIM time and stay frame-rate independent; without it
   *  the provider falls back to its wall-clock dt. Providers without
   *  slews (tilt) ignore it. */
  getControls(simDt?: number): FlightControls;

  /** 'fixed-wing' | 'vtol' | 'rotary' — lets a provider adapt (e.g. tilt
   *  sensitivity). Optional; providers may ignore it. */
  setAircraftKind?(kind: 'fixed-wing' | 'vtol' | 'rotary'): void;

  /** Re-centre / re-zero the provider (tilt neutral angle, stick centre). */
  calibrate?(): void;

  /** Remove listeners / stop polling. */
  dispose?(): void;
}
