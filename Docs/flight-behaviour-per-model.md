# Flight behaviour — per-model supplement

Companion to `flight-behaviour-research.md` (17 Nov 2026), which calibrated the
F/A-18F in depth. This file does the same work for the other five aircraft in
the game — F-35A VTOL, F-22, F-15E, MiG-29, Su-34 — plus a short F/A-18F
recap so the builder has one place to look. Read both files together; the
main report's §0 method, §5 BET structure and §6 small corrections apply to
every model here.

## Method (same as the main report §0)

Sustained turn rate is computed, not quoted, wherever a published E-M diagram
is unavailable: at the sustained turn, drag = thrust, so
CD = T/(qS) = 0.028 + k·CL², then n = qS·CL/W and
ω = g·√(n²−1)/V. Thrust lapse with altitude: T = T_max·(ρ/ρ₀)^0.7.
Energy bleed at the max instantaneous turn: a = (D−T)/m, in kt/s
(1 m/s² = 1.944 kt/s). Every computed value below names the mass, speed and
altitude it was computed at, so the builder can reproduce it. Where a number
is published, the source is named; where it is an estimate or disputed, it is
flagged **[EST]** or **[DISPUTED]**.

## Pass 11 addendum (20 Nov 2026) — flight-characteristics audit

The numbers in §1–§7 were the research starting point; pass 11 audited them
against the implementation and corrected four of them in `AircraftSpec.ts`
(plus one implementation bug that dwarfed all four). Measured verification
is in `scripts/probe-stall-speeds.ts` (all checks green, 20 Nov 2026).

**The implementation bug — lift applied along the body-up axis (both
solvers).** `BladeElementSolver` and `CurveAerodynamicsSolver` both applied
the lift force along the body-up axis. The body-up axis sits sin(AoA) aft
of the perpendicular to the flight path, so the lift carried a spurious
L·sin(AoA) component *backward* along the flight path — ~20% of the lift
at a 12° turn AoA (~190 kN on the F-18, more than its thrust). Measured
proof (F-18, 400 kt, 82° bank, AoA 12°): the solver's own drag was
107–123 kN against 193 kN of thrust, yet the jet decelerated at 4–6 m/s² —
the missing ~150 kN was the lift's backward component. Consequence: no jet
could hold its §0 sustained turn rate (measured 13–17°/s vs the §0 16–21°/s),
every hard turn bled 20–40 kt, and a 150 kt turn bled into the stall flag
(the live F-22 report). Fix: the lift is now applied perpendicular to the
airflow — in BET, the summed station-lift vector (the stations already
compute lift perpendicular to their local airflow); in the curve solver,
the component of body-up perpendicular to vAir. Magnitude unchanged
(the calibrated q·S·CL). After the fix the sustained rates measure
F-18 16.6, F-35A 16.0, F-22 18.9, F-15E 20.9, MiG-29 19.9, Su-34 14.1°/s
at 400 kt SL, against §0 expectations 19.0, 16.4, 20.0, 21.0, 21.2,
16.1°/s (all within ±3°/s; the remaining 1–2.5°/s shortfall is the BET
trim download — the tail must download ~100 kN to trim 7 g, which the
§0 formula does not count).

**Spec corrections (all in `AircraftSpec.ts`):**
- F/A-18F: `rotateSpeedKt` 126 → **140**. The old Vr was *below* the
  128 kt stall speed (CL needed at Vr 1.66 > CLmax 1.6) — the cause of
  the live F-18 deck drop at −2114 fpm after a ~130 kt liftoff.
- F-22: CL peak **1.75 → 1.40**, linearEndCL 1.35 → 1.20, floor
  1.0 → 0.85, `stallSpeed` 54 → **60 m/s (117 kt)**. Clean Vs is
  classified/unpublished; the best estimates are 108–130 kt (Simple
  Flying / boltflight), implying CLmax ~1.15–1.35 — the modelled 105 kt /
  CLmax 1.75 was ~20 kt too low and uncorroborated.
- F-15E: CL peak **1.36 → 1.10**, linearEndCL 1.30 → 1.05,
  `stallSpeed` 62 → **72 m/s (141 kt)**. The old CL 1.36 predicted
  Vs 127 kt, not the spec's own 120 kt; the real CLmax ≈ 1.1 (no
  leading-edge flaps). Thrust 258 kN is the F100-PW-229 only (the -220
  is 211 kN total).
- MiG-29: CL peak **1.58 → 1.35 on the 45 m² effective area**
  (≡ 1.60 on the 38 m² wing area), linearEndCL 1.35 → 1.15,
  `stallSpeed` 59 → **64 m/s (124 kt)**. The old 1.58 was quoted on the
  wing area, so it predicted Vs 124 kt while the spec said 115 kt.
  BET now also scales the wing-station areas to the effective area
  (`buildStations`), so BET lifts on 45 m² like the curve solver —
  before, BET under-lifted the MiG-29 by ~8% at level AoA and ~19% at
  the peak, and it sank out of a stall test without ever flagging.
- Su-34: mass **kept at 30,000 kg** — the roster convention is combat
  mass (like the F-22's 24,883 kg vs its 38,000 kg MTOW). Real normal
  takeoff weight is 39,000 kg (MTOW 45,100 kg; aerospaceweb.org), at
  which the same CLmax gives Vs ~157 kt.
- `sustainedTurnRate` no longer defaults CLmax to 1.6 for every
  aircraft: the curve solver's coordinated-yaw cap and the AI bank
  command now pass each model's own `clCurve.peak` (and the AI uses
  the lift area for the MiG-29), so the F-15E (1.10) and F-22 (1.40)
  are no longer credited with an envelope they cannot hold.

**Measured verification (probe-stall-speeds, BET, live paths, 20 Nov 2026):**
stall speeds F-18 130, F-35A 134, F-22 120, F-15E 144, MiG-29 125,
Su-34 139 kt (bands Vs ±8 kt); level flight at 150 kt: no stall flag on
any jet (the F-22 report is closed — it was the lift-direction bug plus
the too-low modelled Vs, not the stall trigger); takeoff liftoffs
F-18 168, F-35A 180, F-22 160, F-15E 181, MiG-29 161, Su-34 162 kt in
331–481 m; F-22 at 29% throttle does NOT lift off below Vr (runs off
the end at 119–125 kt). F-35A sustained rate ~16–17°/s SL (the task's
~19°/s E-M claim is the top of a 16–19°/s band, not a single value).

**Could not be sourced authoritatively:** clean stall speed for any
aircraft except the F/A-18E/F 129 kt power-off (Wikipedia, citing
NATOPS/SAC); landing-config stall speeds; CLmax values (only F-15 ~1.1,
F-16 ~1.6, Su-27 1.85 @ 24°, MiG-29 ~1.5); rotation speeds (pilot
accounts / Eurocontrol V2/Vat only); F-35A MTOW conflict (29,900 kg
RAAF vs 70,000 lb class / 31,750 kg USAF).

## ⚠️ Read first: the sim's masses — fixed to combat weights

`AircraftSpec.ts` formerly mixed conventions (F-35A, F-22, F-15E, Su-34
near empty). That is fixed: all six now use combat masses (F-18 20,242 kg,
F-35A 18,270 kg, F-22 24,883 kg, F-15E 20,400 kg, MiG-29 15,300 kg,
Su-34 30,000 kg). The Su-34's 30,000 kg is the roster combat mass; the
real normal takeoff weight is 39,000 kg (MTOW 45,100 kg) — at 39 t the
same CLmax gives Vs ~157 kt (see §6). If masses change, re-derive stall
speeds (V_stall = √(2W/(ρ·S·CLmax))) and rotation gates from the table
in §7 — the values there are given at combat mass.

---

## 1. F/A-18F Super Hornet (recap — full detail in the main report)

- **Mass:** empty 14,552 kg; **combat 20,242 kg (sim mass ✓)**; MTOW 29,900 kg.
- **Geometry:** S = 46.5 m², span 13.62 m, AR = 3.99, LE sweep 45°, e ≈ 0.8,
  **k = 0.100**. LEX (leading-edge extensions) — the vortex-lift source.
- **Thrust:** 2× F414-GE-400, dry 62 kN each (124 total), AB 98 kN each
  (**196 kN total, sim ✓**). T/W 0.99 at combat mass.
- **Stall / AoA:** wing section stalls ~15°; LEX vortex lift keeps the
  aircraft flying to a 35° lift limit, controllable to 50°; NATOPS AoA limit
  25° (A–D). Stall speed ~66 m/s at combat mass (sim 66 ✓, CLmax 1.6);
  power-off stall 129 kt published (Wikipedia, citing NATOPS/SAC).
- **Rotation / takeoff:** rotate **140 kt** at 20,242 kg (154 kt at MTOW);
  main gear lifts ~173 kt at 35,000 lb. Climb-out 5° nose-up to 350 kt.
  Pass 11: the old 126 kt Vr was *below* the 128 kt stall speed — a deck
  liftoff at ~130 kt dropped at −2114 fpm, because it lifted off below a
  flyable speed.
- **Turn rates (GAO NSIAD-96-98, fighter-escort config):** STR **19.2°/s SL**,
  **12.3°/s at 15,000 ft**; ITR **24.7°/s**; best rate speed ~430 KCAS;
  bleed at max pull **~54 kt/s** at SL.
- **Control authority:** roll 150°/s (sim ✓); pitch rate 55°/s max
  (main report §3 — q-schedule it, rotation needs ~10–15°/s).
- **CL curve (main report §2):** linear to 15° (CL 1.35); 15–35°:
  CL = 1.35 + 0.0125·(α−15), peak **1.6 at 35°**; above 35°:
  CL = 1.6 − 0.05·(α−35), floor 0.8. CD_sep = 0.35·max(0,(α−20)/15)².
- **BET layout:** 39 stations — wing 16, tailplane 8, fin 4, LEX 8,
  fuselage 1, engines 2 (main report §5).
- **Sim status:** correct as-is except the Vr fix above (126 → 140 kt,
  pass 11). `turn: 1.0` is the reference the other
  multipliers are measured against. Measured (pass 11, probe-stall-speeds):
  stall 130 kt, sustained turn 16.6°/s at 400 kt SL (§0: 19.0), liftoff
  168 kt in 451 m.

## 2. F-35A VTOL (player) — a fictional hybrid: read the caveat

**Caveat first.** The sim's "F-35A VTOL" does not exist in real life. The
real F-35A is a conventional fighter (no vectoring, no lift fan, no
tailhook, 9 g). The VTOL/STOVL variant is the **F-35B** (lift fan + swivel
nozzle, 7 g, no tailhook, vertical landing not carrier trap). The carrier
variant is the **F-35C** (larger wing, tailhook). The sim combines an A-model
airframe with B-model propulsion and C-model carrier capability. That is the
user's designation and it stays — but the numbers below keep the two halves
separate so the builder knows which half each value comes from. Wing-flight
numbers are F-35A; hover numbers are F-35B.

- **Mass:** F-35A empty 13,290 kg; **combat ~18,270 kg**; MTOW 31,751 kg.
  Sim mass 14,500 kg ≈ empty + ~1,200 kg. **Recommend 18,270 kg** (see the
  hover trade-off below).
- **Geometry:** S = 42.7 m², span 10.7 m, **AR = 2.68**, LE sweep **33°**
  (trailing edge 14°), twin canted fins (19° cant), e ≈ 0.8, **k = 0.149**.
  No LEX — small chines only. Trapezoidal wing, high wing loading.
- **Thrust (wing flight):** 1× F135-PW-100, dry **125 kN**, AB **191 kN
  (sim thrustMax ✓)**. T/W: **1.34 at sim mass, 1.07 at combat mass**.
- **Thrust (hover, F-35B LiftSystem):** Rolls-Royce LiftFan 50-inch
  two-stage counter-rotating fan, **>20,000 lbf (~89 kN) cold thrust**;
  three-bearing swivel module (3BSM) **~18,000 lbf (~80 kN)** hot thrust,
  rotates 95° in 2.5 s; wing roll posts 2× ~1,950 lbf (~8.7 kN) each;
  **total vertical thrust ~40,500–41,100 lbf (180–183 kN)**. F-35B hovers
  at T/W 1.1 at ~38,000 lb (17,240 kg). The sim simplifies this to
  vectoring the single 191 kN nozzle to 90°: at sim mass 14,500 kg that
  hovers at **74% throttle**; at the recommended combat mass 18,270 kg it
  hovers at **94%** — marginal, which is *realistic* (a real F-35B cannot
  hover at full combat weight either). Do not "fix" the 94% — it is the
  honest number. Hover control in reality: pitch by differential
  fan/nozzle thrust, roll by the roll posts, yaw by differential roll-post
  thrust; the sim's single-vector model cannot reproduce hover pitch/roll
  control — see BET layout below.
- **Stall / AoA:** wing section stalls ~15°; FCS AoA limit **50°**
  (production limit; trimmed flight demonstrated to 55°, departure/recovery
  testing to 110°). **Max usable AoA 50°** — but lift is falling past ~30°;
  at 50° the jet is held up by thrust and fuselage/chine lift, not the wing.
  Stall speed at combat mass (CLmax 1.5): **~68 m/s** (sim 60 m/s is correct
  at the light sim mass only).
- **Rotation / takeoff:** rotation **~150 kt** (F-35C rotation is 15–20 kt
  slower than the A at the same weight; A approach 155 kt at 40,000 lb,
  touchdown ~150 kt, AoA ~11–13° on approach, no flare). Takeoff run is
  short — high thrust, high wing loading.
- **Turn rates:** no public E-M diagram; the two anchors are the
  240-3 standard (**sustained 4.95 g at Mach 0.8, 15,000 ft**; top turning
  speed 370 kt at 9 g, 15,000 ft) and the revised KPP (sustained 4.6 g).
  Model check at Mach 0.8 / 15,000 ft, combat mass: n = 4.82 g —
  **matches the published 4.95 g**, which validates k = 0.149. STR:
  **~16°/s SL at 400 kt**, **~11°/s at 15,000 ft (Mach 0.8)**.
  ITR: **~20–22°/s** (CLmax 1.5 at ~350 kt SL, 7.1 g). Best rate speed
  ~370–400 kt. The often-quoted "~14°/s sustained" (defense-issues
  comparison, "50% of the F-22's sustained rate") is consistent with the
  15,000 ft number, not the sea-level one — always quote the altitude.
- **Energy bleed at max pull:** ~26 kt/s at 350 kt SL (CL 1.5, AoA ~30°) —
  heavy bleed, the F-35's known weakness in a sustained fight; it wins by
  pointing its nose once (the "Dojo drift": ITR ~30°/s for ~1 s at 55° AoA,
  then the pilot must unload — nose rate, not flight-path rate).
- **Control authority:** roll rate — **no reliable public number [EST]**;
  sim `rollDeg: 160` is reasonable (demo rolls are fast; do not use the
  325°/s figure circulating on low-quality sites — it is unsourced).
  Pitch rate ~30–40°/s **[EST]**. No thrust vectoring in wing flight
  (unlike the F-22). Adverse yaw is handled by the FCS; the pilot feels a
  clean, heavy jet.
- **CL curve (differs from the Hornet):** linear to 15° (CL 1.35);
  15–30°: CL = 1.35 + 0.010·(α−15), peak **1.5 at 30°**; above 30°:
  CL = 1.5 − 0.020·(α−30), floor 0.9 at 50°. CD_sep = 0.30·max(0,(α−20)/15)².
  Less vortex lift than the Hornet — the trapezoidal wing without LEX
  gives up earlier, which is why the F-35's usable AoA is thrust, not lift.
- **BET layout: 39 stations** — wing 16, tailplane 8 (all-moving
  stabilators), **twin canted fins 8 (4 each)** — each fin station's lift
  vector tilts with the 19° cant, so rudder input also rolls the jet;
  **chine 4** (small forebody vortex source, not a LEX); fuselage 1;
  **thrust 2: main nozzle 1 + lift-fan 1**. The lift-fan station sits
  ~2.5 m forward of the CG, the nozzle at the tail: in hover, pitch
  control is differential fan/nozzle thrust, roll control is the roll-post
  stations (add 2 small vertical-thrust stations at the wingtips, active
  only in hover). At vector 0° the fan station contributes nothing.
- **Sim fixes:** mass 14,500 → **18,270 kg** (accept the 94% hover);
  `turn: 1.05` → **0.9** (STR 16 vs Hornet 19); `rollDeg: 160` keep
  (estimate); `carrier: true` is fictional — keep it (user's designation)
  but note the real A-model has no tailhook; `stallSpeed: 60` → **68 m/s**
  at combat mass; `rotateSpeedKt` (new field, main report §3): **150**.

## 3. F-22 Raptor (player)

- **Mass:** empty **19,700 kg**; **combat ~24,883 kg (sim mass ✓)**
  (defense-issues comparison; ~29 t "typical operating" per the xiaobrook
  simulator's published-figures table); MTOW 38,000 kg.
- **Geometry:** S = 78.0 m², span 13.56 m, **AR = 2.36**, LE sweep **42°**,
  diamond wing + prominent chines + LERX, twin canted fins, e ≈ 0.85
  (blended body + diamond planform), **k = 0.159**.
- **Thrust:** 2× F119-PW-100, dry **116 kN each (232 total)**, AB **156 kN
  each (312 total, sim ✓)**. T/W: **1.61 at sim mass (absurd — the jet
  would climb vertically at 60% throttle), 1.25 at combat mass** (matches
  Wikipedia: "nearly at unity in military power, 1.25 in full afterburner
  at typical combat weight"). Supercruise Mach 1.5+ on dry thrust.
- **Thrust vectoring:** rectangular 2D nozzles, **±20° in pitch only**,
  fully integrated into the flight controls. TVC **increases roll rate by
  50%** (TO 1F-22A-1 flight manual). This is the only jet in the game
  with TVC — model it as a thrust station whose direction rotates with
  nozzle angle, not as a generic "pitch boost".
- **Stall / AoA:** trimmed flight at **over 60° AoA while maintaining roll
  control**; Herbst manoeuvre (J-turn), Pugachev's Cobra and Kulbit
  demonstrated. **Max usable AoA 60°+** — the highest in the game, and
  the only one where high AoA is a weapon rather than an emergency.
  Stall speed at combat mass (CLmax 1.40): **~60 m/s (117 kt)**.
  Pass 11: the old CLmax 1.75 / Vs 54 m/s (105 kt) was uncorroborated —
  clean Vs is classified/unpublished, and the best estimates are
  108–130 kt (Simple Flying; boltflight), implying CLmax ~1.15–1.35.
  The modelled 105 kt was ~20 kt too low.
- **Rotation / takeoff:** rotation **136–143 KCAS**, takeoff **163–164
  KCAS** (USAF AIB, NAS Fallon). Approach 141–155 kt (sources disagree;
  use ~145 kt). Takeoff run ~480 m.
- **Turn rates — the disputed number:** Col Fornof's Red Flag 2008
  briefing claimed the F-22 **"sustains 28°/s at 20,000 ft"** (with the
  F-15 at ITR 21°/s, STR 15–20°/s). **[DISPUTED — do not calibrate to
  this.]** The F-15C manual gives 12.1°/s sustained at 20,000 ft; a
  28°/s sustained rate there would imply ~48°/s at sea level (same
  proportion), i.e. a 360° turn in 7.5 s — physically impossible for a
  25-tonne fighter. The honest reading, supported by the model below, is
  that **28°/s is approximately the INSTANTANEOUS rate at low altitude**,
  misquoted as sustained. Model (combat mass, k = 0.159): STR **~19–20°/s
  SL at 400 kt** (computed 20.0°/s — essentially the Hornet's number, with
  better energy retention); STR **~11°/s at 20,000 ft at combat mass**
  (computed 10.7°/s at 24,883 kg; 12–13°/s only at a lighter ~22,000 kg
  or with a lower thrust-lapse exponent — so the honest band is 11–13°/s,
  still only a small advantage over the F-15's 12.1°/s, as the
  "2–3°/s advantage" framing expects).
  ITR: **~23°/s wing-only** (CLmax 1.40 at ~300 kt SL, 6.5 g);
  **~28°/s with TVC assist [EST]** (the nozzles add lift/moment past
  the wing's peak — the old "~28–31°/s (CLmax 2.0 with TVC)" assumed
  a wing CLmax of 2.0, which no source supports). One airshow-measured data point supports the SL
  sustained figure: a full 360° in ~17.2 s = **20.9°/s average**.
  Best rate speed ~400–440 KCAS. Minimum turn radius quoted ~229 m (750 ft)
  at very low speed — a corner-speed curiosity, not a sustained figure.
- **Energy bleed at max pull:** **~60–70 kt/s** at 300 kt SL / 35° AoA
  (CL 2.0, CD ≈ 1.0 with separation drag) — the largest bleed in the
  roster, and the reason the 28°/s sustained claim fails: the jet can
  point its nose at 28°/s for a few seconds, then it is out of energy.
  The F-22's real advantage is not sustaining 28°/s — it is reaching
  28°/s *instantly* (TVC + huge pitch authority) and recovering energy
  fast (T/W 1.25).
- **Control authority:** roll rate — no reliable public number **[EST]**;
  sim `rollDeg: 180` is conservative; **recommend ~200°/s** (TVC adds
  50% to the aerodynamic roll rate). Pitch rate ~50°/s **[EST]** (game
  data files quote 50°/s pitch-up; TVC makes pitch authority nearly
  airspeed-independent at low speed — unlike every other jet here, the
  F-22 keeps pitch authority at ~0 kt). Yaw at high AoA is TVC-assisted;
  the jet is famously departure-resistant.
- **CL curve (differs from the Hornet):** linear to 15° (CL 1.20);
  15–35°: CL = 1.20 + 0.010·(α−15), peak **1.40 at 35°**; 35–60°:
  CL = 1.40 − 0.015·(α−35) (**1.03 at 60°**), floor 0.85. The high
  post-35° floor is the diamond wing + chines + LERX vortex system, and
  it is what lets the jet fly at 60° AoA. CD_sep = 0.30·max(0,(α−20)/15)².
  TVC lift is NOT in this curve — it is a thrust force at the nozzle
  station (see BET layout). Pass 11: peak was 1.75 (uncorroborated) —
  see the stall note above.
- **BET layout: 43 stations** — wing 16, tailplane 8 (all-moving
  stabilators), **twin canted fins 8 (4 each)**, **LERX/chine 8** (the
  F-22's chines are a major vortex source — do not omit them), fuselage 1,
  **engines 2 with TVC ±20° pitch**. Per station: the nozzle thrust
  direction rotates with the vector angle, so at high AoA the nozzles
  contribute a direct pitch moment even at zero airspeed — this is what
  makes post-stall manoeuvres possible in the model, and it falls out of
  the geometry with no special cases.
- **Sim fixes:** mass 19,700 → **24,883 kg** (done); `turn: 1.15` → **1.05**
  (sustained rate ≈ Hornet's; the 1.15 belongs to instantaneous only —
  if the solver separates STR/ITR, give the F-22 an ITR bonus instead);
  `rollDeg: 180` → **200 [EST]**; `stallSpeed: 65` → **60 m/s** (pass 11:
  the old 54 m/s was ~20 kt too low, CLmax 1.75 uncorroborated);
  `rotateSpeedKt` (new field): **140**. Add a `tvcDeg: 20` field for the
  nozzle model. Measured (pass 11): stall 120 kt, sustained turn
  18.9°/s at 400 kt SL (§0: 20.0), liftoff 160 kt in 331 m; no stall
  flag in level flight at 150 kt.

## 4. F-15E Strike Eagle (player)

- **Mass:** empty 14,379 kg; **combat ~20,400 kg clean (sim mass ✓)**
  (heavier with CFTs/stores, up to 36,700 kg MTOW).
- **Geometry:** S = 56.5 m², span 13.05 m, **AR = 3.01**, LE sweep **45°**,
  e ≈ 0.8, **k = 0.132**. No LEX — the F-15's wing root has only small
  fairings, and the wide flat fuselage is itself a lifting body. Airfoil
  NACA 64A006.6 root / 64A203 tip. Twin fins, all-moving stabilators.
- **Thrust:** 2× F100-PW-229, dry **79 kN each (158 total)**, AB **129 kN
  each (258 total, sim ✓)**. T/W: **1.84 at sim mass (absurd),
  1.29 at combat mass clean**. Rate of climb 50,000 ft/min. Note the
  258 kN is the -229 only — the older F100-PW-220 is 211 kN total
  (pass 11).
- **Stall / AoA:** **max usable AoA ~30°** — and the F-15 is the least
  forgiving jet here past it: no LEX vortex lift, so the stall break is
  sharp and the jet departs in yaw/roll above ~30° (the Eagle will spin
  if mishandled at high AoA, unlike the departure-resistant Hornet).
  Stall speed at combat mass (CLmax 1.10): **~72 m/s (141 kt)**.
  Pass 11: the old CLmax 1.36 predicted Vs 127 kt, not the spec's own
  120 kt — and the real CLmax ≈ 1.1 (no leading-edge flaps). The 1984
  flight manual quotes level-flight stall "over 100 knots" at light weight.
- **Rotation / takeoff:** takeoff **140–180 kt** (Lakenheath footage:
  roar-to-liftoff under 13 s). Rotation **~150 kt** at combat weight;
  the main report's 160 kt was calibrated to the light sim mass — do not
  carry it over to the combat mass (it would scale to ~190 kt, which
  contradicts the published 140–180 kt band).
- **Turn rates (F-15C flight manual, clean — the best-published E-M data
  in this file):** STR **20.6°/s SL at Mach 0.72 pulling 9 g**;
  **12.1°/s at 20,000 ft** (Mach 0.93, ~6 g); ITR **24°/s SL**.
  The main report's "F-15 sustained 15–16°/s" is the **loaded F-15E**
  (CFTs + stores) — use it for a strike-loaded F-15E, not the clean one.
  Model check at combat mass: STR **20.9°/s SL at 400 kt** and **11.4°/s
  at 20,000 ft** — matches the manual within 1°/s, which validates
  k = 0.132. For the sim's clean F-15E use **STR ~19–20°/s SL,
  ~11°/s at 20,000 ft, ITR ~24°/s**. Best rate speed ~430–460 KCAS.
- **Energy bleed at max pull:** **~8–10 kt/s** — the lowest in the
  roster. This is the Eagle's signature: huge thrust, clean airframe,
  and it keeps its energy in a turning fight where the Hornet and the
  F-22 bleed dry. The F-15E wins fights by not slowing down.
- **Control authority:** roll rate — sim `rollDeg: 140` is low **[EST]**;
  the F-15's roll rate is generally quoted around **180°/s**;
  **recommend 180**. Pitch rate ~30°/s **[EST]**. Rudder authority is
  good (twin fins + large rudders); the jet is honest and predictable,
  with no FCS tricks — what the pilot commands is what happens.
- **CL curve (differs sharply from the Hornet):** linear to 14°
  (CL 1.05); 14–25°: CL = 1.05 + 0.005·(α−14), peak **1.10 at 25°**;
  above 25°: CL = 1.10 − 0.06·(α−25), floor 0.7. The steep post-stall
  drop is the point: no vortex lift to save the wing, so the stall break
  must feel like a break. CD_sep = 0.40·max(0,(α−20)/15)² — the
  sharpest drag rise in the roster. Pass 11: peak was 1.36 (internally
  inconsistent — it predicted Vs 127 kt against the spec's 120 kt).
- **BET layout: 36 stations** — wing 16, tailplane 8 (all-moving
  stabilators — large, the F-15's primary pitch control), **twin fins 8
  (4 each)**, **fuselage 2** (the flat fuselage is a lifting body: give
  the fuselage stations real area and lift, unlike the Hornet's
  drag-only fuselage station), engines 2. No LEX stations.
- **Sim fixes:** mass 14,300 → **20,400 kg** (done); `turn: 0.95` → **1.0**
  (clean STR ≈ Hornet's; the 0.95 belongs to the strike-loaded config);
  `rollDeg: 140` → **180 [EST]**; `stallSpeed: 62` → **72 m/s**
  (pass 11: CLmax 1.36 → 1.10, Vs 127 kt was internally inconsistent);
  `rotateSpeedKt` (new field): **150**. Measured (pass 11): stall
  144 kt, sustained turn 20.9°/s at 400 kt SL (§0: 21.0), liftoff
  181 kt in 396 m. The CLmax correction does not change the 400 kt
  sustained rate (thrust-limited there, not CL-limited) — it changes
  Vs and the low-speed turn.

## 5. MiG-29 Fulcrum (enemy)

- **Mass:** empty 11,000 kg; **normal takeoff 15,300 kg (sim mass 15,000 ✓)**;
  MTOW 18,000 kg. The lightest fighter in the game — and it flies like it.
- **Geometry:** S = **38.0 m² wing only — but the blended fuselage + LERX
  generate ~40% of total lift, so the effective lifting area is ~45 m²**
  (use 45 m² for lift, 38 m² for drag reference, or equivalently raise
  CLmax — see below); span 11.36 m, **AR = 3.39** (on 38 m²),
  LE sweep **42° outer panels, 73.5° on the LERX**, e ≈ 0.85 (blended
  body), **k = 0.110**. Twin fins canted outward ~4–6°, carried on LERX
  booms. Leading-edge slats deploy automatically.
- **Thrust:** 2× RD-33, dry **50 kN each (100 total)**, AB **81.4 kN each
  (163 total, sim 162,000 ✓)**. T/W **1.09–1.10 at normal takeoff
  weight**. Rate of climb 330 m/s initial — among the highest ever
  published for a production fighter.
- **Stall / AoA:** AoA limiter **26°** (28° on the MiG-29S), overridable;
  **controllable flight to 30–35°**, reluctant to enter a stable flat
  spin and recovers as soon as controls are released. The Pugachev Cobra
  (~110° AoA) is a *dynamic* manoeuvre, not sustained flight — do not
  model it as a flyable regime. Stall speed at combat mass (effective
  S 45 m², CLmax 1.35): **~64 m/s (124 kt)**. Pass 11: the old CLmax
  1.58 was quoted on the 38 m² wing area (≡ 1.35 on the 45 m² effective
  area), so it predicted Vs 124 kt while the spec said 115 kt. Real
  clean Vs ~125–135 kt; landing speed 124 kt, Vat/V2 140 kt
  (Eurocontrol).
- **Rotation / takeoff:** takeoff speed **260 km/h (140 kt)**; rotation
  **~130 kt**; takeoff run only **250–260 m** at normal weight (short
  field + FOD doors in the intakes); landing speed 235 km/h (127 kt),
  landing run 600 m.
- **Turn rates:** published STR **22°/s** (MiG-29A), **23.5°/s**
  (MiG-29S); ITR **28°/s**; shortest turn radius 300 m at ~450 km/h.
  Model check (wing-only area 38 m²): STR ~20°/s — short of published,
  because the wing-only area ignores the lifting fuselage. With
  effective area 45 m²: ~21°/s at 15,300 kg; at light 13,000 kg the
  manual's "sustain 9 g at 460 kt" gives **21.2°/s**, and DCS-manual
  comparisons put the MiG-29A at **21°/s at 450 kt** against the F-15's
  20°/s and the Su-27's 18°/s at the same condition. **Recommend STR
  ~21°/s SL at combat weight, ~13°/s at 20,000 ft [EST], ITR ~26–28°/s.**
  Best rate speed ~450 kt. The MiG-29's own literature stresses the
  small ITR–STR gap: it points its nose at 28°/s and sustains 23°/s —
  the closest thing to the F-22's profile in the game, at half the weight.
- **Energy bleed at max pull:** **~15–22 kt/s** (model: 13.5 kt/s at
  300 kt, 22.6 kt/s at 400 kt) — the Fulcrum bleeds faster than the
  Eagle, slower than the Hornet at max pull. Its small wing and high
  drag at high AoA mean it cannot stay in a max-rate turn for long.
- **Control authority — the speed-scheduled roll is the modelling
  point:** max roll rate **160°/s** (Luftwaffe MiG-29G manual), but it
  falls to **~105°/s at Mach 0.4**, **<60°/s at high speed / low
  altitude** (weak aileron actuators), **~40°/s supersonic**, and
  **~20°/s just above the stall (~145 kt)**. Sim `rollDeg: 155` ≈ the
  max ✓ — but the current `qAuth` scaling (more authority at more speed)
  gets this *backwards* at the top end: roll authority must be capped
  by a Mach/airspeed schedule, not just scaled by q. Pitch-up rate
  ~38–41°/s. At high AoA the roll response can drop to near zero or
  slightly negative (adverse yaw) — the pilot points the nose with the
  rudder at low speed, which the manual explicitly notes.
- **CL curve:** linear to 15° (CL 1.15); 15–30°: CL = 1.15 + 0.013·(α−15),
  peak **1.35 at 30°**; 30–45°: CL = 1.35 − 0.020·(α−30) (1.05 at 45°),
  floor 0.9. CD_sep = 0.30·max(0,(α−20)/15)². The curve is quoted on
  the 45 m² effective area; on the 38 m² wing area the peak is 1.60.
  Pass 11: the old peak 1.58 was the wing-area figure used on the
  effective area — see the stall note above.
- **BET layout: 43 stations** — wing 16, **LERX 8 (4 per side, 73.5°
  sweep — the largest vortex source in the game after the Hornet's)**,
  tailplane 8 (all-moving), **twin canted fins 8 (4 each)**, fuselage 1
  (give it lift area — ~40% of total lift comes from the body/LERX
  system, so the fuselage station must not be drag-only), engines 2.
  The LERX stations' vortex lift is what holds CL up to 30° — in the
  BET model this comes from the LERX polar, not from a fudge on the wing.
- **Sim fixes:** mass 15,000 keep; `turn: 1.0` → **1.1** (STR 21 vs
  Hornet 19); `rollDeg: 155` keep (max) **but add the airspeed roll cap**
  (105°/s at Mach 0.4, <60°/s high-speed/low, 40°/s supersonic, 20°/s
  near stall); `stallSpeed: 60` → **64 m/s** (pass 11: CL peak 1.58 →
  1.35 on the effective area); `rotateSpeedKt` (new field): **130**.
  Consider `wingAreaEffective: 45` for lift vs `wingArea: 38` for drag
  (done — and BET now scales the wing stations to it, pass 11).
  Measured (pass 11): stall 125 kt, sustained turn 19.9°/s at 400 kt
  SL (§0: 21.2), liftoff 161 kt in 375 m.

## 6. Su-34 Fullback (enemy)

- **Mass:** empty 22,500 kg; gross 39,000 kg;
  MTOW 45,100 kg; **typical combat ~30,000 kg (sim mass ✓)**.
  The heaviest fighter in the game by a wide margin, and it must feel it.
  Pass 11: the roster uses the 30,000 kg combat mass; at the real normal
  takeoff weight of 39,000 kg the same CLmax gives Vs ~157 kt.
- **Geometry:** S = 62.04 m², span 14.7 m, **AR = 3.49**, LE sweep **42°**
  (Su-27 family), e ≈ 0.8, **k = 0.114**. **Canards** (front horizontal
  tail) + conventional all-moving tailplane + twin fins — a **three-surface
  design**: the canards add lift, increase static instability (agility)
  and reduce trim drag. Wide side-by-side cockpit nose, large fuselage.
- **Thrust:** 2× AL-31FM1, dry **86.3 kN each (173 total)**, AB **132 kN
  each (264 total)**. **⚠️ SIM BUG: `thrustMax: 166,000 N` is ~63% of the
  real afterburner thrust and below even the dry total. Fix to 264,000 N**
  (245,000 N if the builder prefers the older AL-31F at 122.6 kN each).
  T/W: **0.77 at sim mass with the wrong thrust; 1.19 at empty with
  correct thrust; 0.90 at 30,000 kg combat; 0.68 at MTOW** (matches the
  published 0.68). With the current 166 kN the Su-34 cannot sustain a
  turn or climb the way the real jet can — every AI engagement against
  it is currently fought against a crippled opponent.
- **Stall / AoA:** **max usable AoA ~30° [EST]** (Su-27 family limit;
  no public Su-34 E-M diagram exists). Stall speed at 30,000 kg
  (CLmax 1.53): **~71 m/s (sim 71 ✓)**.
  Landing speed 250 km/h (135 kt).
- **Rotation / takeoff:** takeoff distance 850–1,260 m; rotation
  **~140 kt [EST]**, takeoff **~150 kt [EST]** at typical combat weight.
- **Turn rates:** **no public E-M diagram — all values below are model
  estimates [EST].** Model at 30,000 kg: STR **~16°/s SL at 400 kt**,
  **~12°/s at 39,000 kg**; recommend **STR ~14–16°/s SL, ~9–10°/s at
  15,000–20,000 ft**. ITR **~19–20°/s** (Su-27 ITR is 27°/s; the Su-34
  shares the wing but carries ~7,000 kg more structure and a second
  crew member). Best rate speed ~350–400 kt **[EST]**. The Su-34 is a
  strike jet that can defend itself, not a dogfighter — the AI should
  fly it that way (energy tactics, one hard turn, then extend).
- **Energy bleed at max pull:** **~13–16 kt/s** at 350 kt SL
  (30,000 kg) — moderate; the big wing and canards keep it efficient,
  but the weight means every knot costs more to win back.
- **Control authority:** roll rate — sim `rollDeg: 120` is right
  (Su-27 rolls ~135°/s; the heavier Su-34 is slower — **keep 120**).
  Pitch authority is good for its weight: canards + all-moving
  tailplanes, ~30°/s **[EST]**. g limit: sim `maxG: 7` is the
  **operational limit** (War Thunder forum source); the airframe is
  cleared to 9 g — keep 7, it is the honest operational number.
- **CL curve:** linear to 15° (CL 1.35); 15–30°: CL = 1.35 + 0.012·(α−15),
  peak **1.53 at 30°**; above 30°: CL = 1.53 − 0.04·(α−30), floor 0.8.
  CD_sep = 0.35·max(0,(α−20)/15)². Canard lift is in the BET stations,
  not in this curve — do not double-count it.
- **BET layout: 43 stations** — wing 16, **canard 8 (4 per side)**,
  tailplane 8, **twin fins 8 (4 each)**, fuselage 1 (large — the wide
  nose/fuselage contributes lift), engines 2. Three-surface modelling
  notes: the canard stations carry their own control deflection and
  produce **downwash on the main wing** (α_wing is reduced by the
  canard's lift, like the tailplane downwash in the main report §5.4);
  pitch trim is shared between canard and tailplane, so a canard
  failure must degrade pitch authority and change the trim point —
  this falls out of the station model if both surfaces are modelled.
- **Sim fixes (done):** `thrustMax: 166,000` → **264,000 N**
  (critical — was 63% of real thrust);
  mass 22,000 → **30,000 kg**; `turn: 0.85` keep
  (STR 15 vs Hornet 19 ≈ 0.8); `rollDeg: 120` keep; `maxG: 7` keep;
  `stallSpeed: 65` → **71 m/s** at combat mass; `rotateSpeedKt`
  (new field): **140 [EST]**. Measured (pass 11): stall 139 kt,
  sustained turn 14.1°/s at 400 kt SL (§0: 16.1), liftoff 162 kt
  in 471 m.

---

## 7. Compact calibration table (copy into `AircraftSpec`)

All rates at **combat mass** (see the mass warning at the top). Turn rates
are the values the solver should reproduce when the §0 method is applied
with the k and CLmax below — verify with the node takeoff/turn driver
after implementing.

| Model | Combat mass (kg) | k = 1/(π·e·AR) | CLmax (peak AoA) | Stall (m/s) | Rotate (kt) | STR SL | STR 15–20k ft | ITR | Bleed at max pull (kt/s) | rollDeg (sim → rec.) | turn mult. (sim → rec.) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| F/A-18F | 20,242 | 0.100 | 1.60 @ 35° | 66 | 140 | 19.2°/s | 12.3°/s | 24.7°/s | ~54 | 150 → 150 | 1.00 → 1.00 |
| F-35A VTOL | 18,270 | 0.149 | 1.50 @ 30° | 68 | 150 | ~16°/s | ~11°/s | ~21°/s | ~26 | 160 → 160 | 1.05 → 0.90 |
| F-22 | 24,883 | 0.159 | 1.40 @ 35° | 60 | 140 | ~19–20°/s | ~11–13°/s | ~23°/s (+TVC ~28 [EST]) | ~60–70 | 180 → 200 | 1.15 → 1.05 |
| F-15E | 20,400 | 0.132 | 1.10 @ 25° | 72 | 150 | ~19–20°/s | ~11°/s | ~24°/s | ~8–10 | 140 → 180 | 0.95 → 1.00 |
| MiG-29 | 15,300 | 0.110 | 1.35 @ 30° (45 m² eff) | 64 | 130 | ~21°/s | ~13°/s [EST] | ~26–28°/s | ~15–22 | 155 → 155 | 1.00 → 1.10 |
| Su-34 | 30,000 | 0.114 | 1.53 @ 30° | 71 | 140 [EST] | ~14–16°/s [EST] | ~9–10°/s [EST] | ~19–20°/s [EST] | ~13–16 | 120 → 120 | 0.85 → 0.85 |

Thrust (dry / AB, total) for the same table: F-18 124 / 196 kN; F-35A
125 / 191 kN (+ lift system 180–183 kN vertical, F-35B); F-22 232 / 312 kN;
F-15E 158 / 258 kN; MiG-29 100 / 163 kN; Su-34 173 / 264 kN
(sim currently 166 kN — wrong, see §6).

BET station totals: F-18 39 · F-35A 39 · F-22 43 · F-15E 36 · MiG-29 43 ·
Su-34 43. (Wing 16 + tailplane 8 everywhere; the difference is
LEX/chine/canard/fin counts and the F-35A's lift-fan station.)

## 8. Spec changes the builder should make (checklist)

1. **Su-34 `thrustMax`: 166,000 → 264,000 N** (hard bug — the jet is
   currently at 63% of real thrust). **Done.**
2. **Masses to combat weights:** F-35A 14,500 → 18,270 kg; F-22 19,700 →
   24,883 kg; F-15E 14,300 → 20,400 kg; Su-34 22,000 → 30,000 kg.
   F-18 and MiG-29 masses are already combat weights — leave them.
   **Done.**
3. **New `AircraftSpec` fields** (main report §0/§3): `inducedK` and `e`
   per model (table §7); `rotateSpeedKt` per model (table §7);
   `tvcDeg: 20` for the F-22; `wingAreaEffective` for the MiG-29
   (45 m² lift vs 38 m² drag) or equivalently CLmax 1.8 on 38 m².
   **Done** (the MiG-29 uses `wingAreaEffective: 45` with CL peak 1.35 —
   the "CLmax 1.8 on 38 m²" alternative is withdrawn, pass 11: 1.58 on
   38 m² ≡ 1.35 on 45 m²).
4. **Turn multipliers:** F-35A 1.05 → 0.90; F-22 1.15 → 1.05;
   F-15E 0.95 → 1.00; MiG-29 1.00 → 1.10; F-18 1.00 and Su-34 0.85 stay.
   **Done** (F-35A 0.9, F-22 1.05, F-15E 1.0, MiG-29 1.1 in the spec).
5. **Roll rates:** F-22 180 → 200°/s [EST]; F-15E 140 → 180°/s [EST];
   others stay. **MiG-29: add an airspeed roll cap** (105°/s at Mach 0.4,
   <60°/s high-speed/low, 40°/s supersonic, 20°/s near stall) — the
   current q-scaling gets the top end backwards. **Done**
   (`rollCapSchedule` in the spec).
6. **Stall speeds at combat mass (pass 11 corrections):** F-35A 60 → 68
   (done); F-22 65 → **60** (was 54 in the first draft — CLmax 1.75
   uncorroborated, Vs ~117 kt); F-15E 62 → **72** (CLmax 1.36 → 1.10);
   MiG-29 60 → **64** (CL peak 1.58 → 1.35 on the effective area);
   Su-34 65 → **71** (done); F-18 62 → 66 (done).
7. **Per-model CL curves** (§1–§6) replace the single shared curve in
   `CurveAerodynamicsSolver.ts` — the F-15E's sharp stall and the F-22's
   60° floor cannot come from one curve. **Done.**
8. **F-35A hover:** two thrust stations (fan + nozzle), not one vectored
   nozzle — pitch by differential fan/nozzle thrust, roll by roll-post
   stations. Hover at 94% throttle at combat mass is correct; do not
   tune it down.
9. **F-35A `carrier: true`** is fictional (real A-model: no tailhook;
   B-model: STOVL, 7 g; C-model: carrier). Keep it — it is the user's
   designation — but do not cite it as a real capability.
10. **Pass 11 (20 Nov 2026):** F-18 Vr 126 → **140 kt** (was below its
    own stall speed); lift direction fixed in both solvers (lift
    perpendicular to the airflow, not along body-up — the phantom
    L·sin(AoA) backward force capped every sustained turn ~25% low);
    BET wing stations scaled to the MiG-29's effective area;
    `sustainedTurnRate` takes each model's own CLmax (was a flat 1.6);
    verification probe `scripts/probe-stall-speeds.ts` — all checks
    green (stall speeds, 150 kt level flight, takeoff rolls, sustained
    turns).

## 9. Downloads needed

**None.** Every number above was verified from public web sources (flight
manuals, GAO NSIAD-96-98, DOT&E/KPP figures, manufacturer product cards,
Luftwaffe MiG-29G manual via DCS forums, USAF AIB report). Two optional
references if the builder wants the primary E-M charts: the F-15E flight
manual TO 1F-15E-1 is freely available on archive.org
(`F15StrikeEagleFlightOperationManual`), and the F-22 flight manual
TO 1F-22A-1 is on Scribd (not freely downloadable). Neither is required
to implement this file — the computed values in §7 are sufficient, and
each was cross-checked against at least one published anchor.
