# Flight behaviour research — Pandora Planes Vite (BET + real-world corrections)

Source: research agent, 17 Nov 2026. Read-only; mapped to `src/physics/CurveAerodynamicsSolver.ts`, `FlightPhysicsController.ts`, `IAerodynamicsSolver.ts`, `AircraftSpec.ts`, `PCFlightInput.ts`, `AircraftAIController.ts`, `main.ts`. Assumes the parallel takeoff-defect fixes (1.3 m/s² acceleration, pitch-up, spawn crash) are repaired first.

## Pass 11 addendum (20 Nov 2026) — audit + physics fix

An audit of the implementation against this report's numbers found one
implementation bug that dwarfed every spec error, plus four spec errors
(full detail in `flight-behaviour-per-model.md`, pass-11 addendum).

**Lift applied along the body-up axis (both solvers) — the bug.** Both
`BladeElementSolver` and `CurveAerodynamicsSolver` applied lift along the
body-up axis. The body-up axis sits sin(AoA) aft of the perpendicular to
the flight path, so the lift carried a spurious L·sin(AoA) backward
component — ~20% of the lift at a 12° turn AoA, more than the thrust.
Measured proof (F-18, 400 kt, 82° bank): solver drag 107–123 kN vs
193 kN thrust, yet the jet decelerated at 4–6 m/s². Consequence: sustained
turns measured 13–17°/s against this report's §0 values of 16–21°/s, every
hard turn bled 20–40 kt, and a 150 kt turn bled into the stall flag (the
live F-22 report). Fixed in pass 11: lift is now perpendicular to the
airflow (BET: the summed station-lift vector; curve solver: the component
of body-up perpendicular to vAir). Magnitude unchanged. **Measured after
the fix** (`scripts/probe-stall-speeds.ts`, all green): sustained turns
F-18 16.6, F-35A 16.0, F-22 18.9, F-15E 20.9, MiG-29 19.9, Su-34 14.1°/s
at 400 kt SL vs §0 19.0, 16.4, 20.0, 21.0, 21.2, 16.1°/s — the §0 method
is validated, with the remaining 1–2.5°/s shortfall being the BET trim
download the §0 formula does not count. Stall speeds now measure F-18 130,
F-35A 134, F-22 120, F-15E 144, MiG-29 125, Su-34 139 kt.

**Spec corrections (pass 11):** F-18 Vr 126 → 140 kt (was below its own
stall speed); F-22 CLmax 1.75 → 1.40, Vs 54 → 60 m/s (clean Vs is
classified — best estimates 108–130 kt); F-15E CLmax 1.36 → 1.10,
Vs 62 → 72 m/s (real CLmax ≈ 1.1, no LE flaps); MiG-29 CL peak 1.58 →
1.35 on the 45 m² effective area, Vs 59 → 64 m/s. Su-34 mass kept at
30,000 kg (roster combat mass; real normal takeoff weight 39,000 kg).
`sustainedTurnRate` now takes each model's own CLmax instead of a flat 1.6.

## 0. Headline calibration — do first

Current code: `CD_INDUCED = 0.06` hardcoded for every aircraft. Too low: jet sustains turns it cannot sustain and barely bleeds speed at max pull.

Real value: k = 1 / (π · e · AR). F/A-18: AR = 13.62²/46.5 = 3.99, e ≈ 0.8 → **k ≈ 0.10**, not 0.06. Add thrust lapse with altitude: T = T_max · (ρ/ρ₀)^0.7.

Proof (F/A-18, 20,242 kg, S = 46.5 m², T = 196 kN, sea level, 400 kt = 205.8 m/s, qS = 1.207 MN): sustained turn is where drag = thrust, CD = 196/1207 = 0.162 = 0.028 + 0.10·CL² → CL = 1.16 → n = 7.05 g → ω = g·√(n²−1)/V = **19.0°/s**. Published (GAO NSIAD-96-98, fighter-escort config): **19.2°/s at sea level**. At 15,000 ft, same maths with thrust lapse (T = 142 kN) gives **12.7°/s** vs published **12.3°/s**. With k = 0.06 the code sustains 24.4°/s — 27% too much.
Correction: compute `inducedK` per spec from AR (add `inducedK` + `e` fields to `AircraftSpec`), apply thrust lapse in `CurveAerodynamicsSolver.solve()`.

## 1. Coordinated turns

Level coordinated turn: bank φ, load factor n = 1/cos φ, turn rate ω = g·tan φ / V, radius R = V²/(g·tan φ). V is **true** airspeed. Bank is the *result* of a commanded turn rate: φ_des = atan(ω_des · V / g).

F/A-18 at 400 kt TAS (205.8 m/s):

| Bank | Load factor | Turn rate |
|---|---|---|
| 60° | 2.0 g | 4.7°/s |
| 75° | 3.9 g | 10.2°/s |
| 80° | 5.8 g | 15.5°/s |
| 82° | 7.2 g | ~19°/s (sustained max, clean) |

Published anchors: F/A-18 max sustained 19.2°/s SL / 12.3°/s at 15,000 ft; max instantaneous 24.7°/s; max-rate band 400–460 KCAS, best rate speed ~430 KCAS; instantaneous bleed in a max-rate turn ~54 kt/s at sea level (GAO). F-22: reported 28°/s sustained at 20,000 ft (Col Fornof Red Flag briefing — anecdotal and widely disputed; needs CL ≈ 2.5 or thrust-vector assist, treat as upper bound, NOT a calibration target — the model with k from AR gives ~15°/s there, the honest number). F-15: instantaneous 21°/s, sustained 15–16°/s. F-35: no reliable public STR; use spec.turn multipliers relative to the F-18.

Current code vs fix:
- `CurveAerodynamicsSolver.ts` `solve()`: yaw is fixed `ROLL_COORD_DEG = -8°/s` per unit roll input. Wrong in kind: coordinated yaw rate must be ω = g·tan φ/V, derived from *actual bank and speed*. Replace with yawRate_coord = (g · tan(bankAngle) / V) · (sign of bank), bankAngle read from the current quaternion, capped at the sustained-rate envelope (ω_max from §0). The lift-tilt already turns the velocity vector; this yaw rate makes the *nose* follow it (weathercock). Add sideslip state β = atan2(vBody.x, vBody.z) and a weathercock term Cn_β·β in the BET version (§5); in the curve version, drive yaw toward β = 0 with a proportional term.
- `AircraftAIController.ts` `update()`: `rollIn = clamp(-local.x * 2.2, …)` drives bank directly from heading error. Replace with two loops: ω_des = clamp(headingError · 1.5, ±ω_max(V)); φ_des = atan(ω_des·V/g); rollIn = clamp((φ_des − φ_now) · 2.0, −1, 1); pitch to hold altitude (pitchIn from vertical-speed error, not just goal elevation).
- `main.ts` `playerInput()`: `p.yawIn = 0` always — rudder input is discarded. Wire `pc.yaw` through (see §4).

## 2. Stall and post-stall

Real F/A-18: wing section stalls ~15°, but LEX vortex lift keeps the *aircraft* flying far beyond — lift limit 35° AoA, controllable to 50°, NATOPS AOA limit 25° (A–D). HARV flew stabilised at 65–70° with thrust vectoring. Calibration point: at 30,000 lb the Hornet holds 100 kt at 35° AoA at 15,000 ft — needs CL ≈ 1.6 at 35°, with thrust carrying part of the weight. Spin: Hornet is departure-resistant; a sustained spin needs pro-spin inputs (full rudder + differential surfaces + asymmetric thrust, yaw rates to 90°/s in test entries). NATOPS spin recovery (A1-F18AC-NFM-500): **controls released, feet off rudders, throttles idle → if command arrow present, lateral stick FULL WITH ARROW; if no arrow, spin-recovery switch to RCVY then lateral stick full with arrow → when yaw stops, stick smoothly neutral → recover when AOA/yaw tones stop and speed > 180 KCAS; if dive recovery not started by 6,000 ft AGL, eject.** A spin recovery costs roughly 1,500–3,000 ft.

Current code vs fix (`CurveAerodynamicsSolver.ts` `solve()`):
- CL curve: stall at 15° + post-stall `CL = 1.35 − 0.045·(α−15)`, so at 35° CL = 0.45. **Wrong by 3.5×.** Replace with vortex-lift curve: linear `CL = 2π·0.9·α + 0.15` to 15° (CL 1.35); 15–35°: `CL = 1.35 + 0.0125·(α−15)` (peak **1.6 at 35°**); above 35°: `CL = 1.6 − 0.05·(α−35)`, floor 0.8. Add separation-drag term: `CD_sep = 0.35 · max(0, (α−20)/15)²`, CD ≈ 0.6 at 35° AoA. This also fixes "the jet never bleeds speed in a hard turn".
- Nose drop: `pitchRate -= STALL_NOSE_DROP * dt` (4.0·dt) is a *constant* −0.067 rad/s offset recomputed each call, not an accumulating drop. Real stall break is a pitch acceleration of roughly −0.5 to −1 rad/s². In the curve model, apply it as an acceleration on a persistent pitch-rate state, or (better) let it emerge from the BET pitching moment (§5).
- Spin: `spin` builds only while stalled *and* yaw input is held, only produces a lateral drift force (−spin·6 m/s²). No yaw-rate state, no roll coupling, no recovery logic. Replace with: spin drives a yaw rate (up to ~90°/s) and a roll rate; recovery is **neutral controls + opposite rudder + nose down** (per NATOPS above), spin decaying ~0.5/s on neutral controls and faster with correct anti-spin input; altitude loss ~2,000 ft per spin. Show the command arrow on the HUD when spin > 0.5 (lateral stick with the arrow = anti-spin roll input).
- Near-zero airspeed: `qAuth = clamp(v/60, 0.25, 1.15)` keeps **25% control authority at zero airspeed**. Real: no airflow = no control. Fix: `qAuth = clamp(v/60, 0, 1.15)` (no floor; nosewheel steering only while grounded). Below ~15 m/s the aircraft is a falling object: lift ≈ 0, drops ballistically at 9.81 m/s², nose drops (add a nose-down pitch moment ∝ (1 − q/q_stall) while stalled and slow).

## 3. Takeoff rotation and climb

Real numbers: F/A-18E: rotate ~154 kt at MTOW, nose 10–12.5°, flight-path vector 3–5°; legacy Hornet: hold stick just aft of neutral, nose 6–8°, main gear lifts ~173 kt at 35,000 lb. F-22 (USAF AIB, NAS Fallon): rotation 136–143 KCAS, takeoff 163–164 KCAS. F-15E: takeoff 140–180 kt. F-35A: approach ~155 kt at 40,000 lb; rotation ~15–20 kt slower than the F-35C's, i.e. ~150 kt. Climb-out: 5° nose-up accelerating to 350 kt to 10,000 ft; max-performance climb 28–32° nose-up, 44,600 ft/min initial (Super Hornet, afterburner).

Current code vs fix:
- `FlightPhysicsController.ts` `stepInternal()`: rotation gate is `sp > stallSpeed·0.85 && pitchIn < −0.25` — for the F-18 that is 53 m/s ≈ 102 kt, well below the real ~126 kt at this weight (154 kt scaled by √(20,242/30,000)). Raise the gate to `sp > stallSpeed·0.95` and, more importantly, make liftoff emerge from lift > weight rather than from a flag. Pitch authority: `PITCH_RATE_DEG = 55` is a plausible *maximum*, but at rotation the pilot needs ~10–15°/s, not 55 — the rate should be q-scheduled and stick-shaped (see §4 sensitivity), otherwise rotation is a twitch. Add per-spec `rotateSpeedKt` to `AircraftSpec` (F-18 140, F-22 140, F-15E 150, F-35A 150 at the sim's masses) and show "ROTATE" on the HUD at Vr. Pass 11: the F-18's old 126 kt Vr was *below* its 128 kt stall speed — a deck liftoff at ~130 kt dropped at −2114 fpm; 140 kt is the corrected value.
- Climb: no climb-attitude guidance; the HUD could show the 10° climb cue. Minor.

## 4. Adverse yaw, roll/yaw coupling, rudder, bombing run

Real behaviour: rolling with aileron raises induced drag on the rising (outer) wing, yawing the nose *away* from the turn — adverse yaw, worst at low speed / high AoA. Cure: rudder in the direction of the roll ("step on the ball"), plus differential/Frise ailerons. Rudder also rolls; aileron also yaws. A coordinated turn uses all three controls; the slip ball centred is the test. Bombing: classic profile (F-100): approach 10,000 ft, 30° dive, release 500 kt IAS at 3,000 ft AGL, **4 g pull-out → minimum altitude ~1,500 ft**. SBD: 70° dive, release 2,000–2,500 ft at 240–250 kt, 4 g pull-out bottoms ~900–1,100 ft. Pull-out altitude loss (verified against the SBD numbers): **Δh = R·(1 − cos θ), R = V² / (g·(n−1))** — e.g. 240 kt, 70°, 4 g: R = 514 m, Δh = 338 m = 1,109 ft; release at 2,000 ft → bottoms at ~890 ft. Matches the published account.

Current code vs fix:
- `PCFlightInput.ts` `getControls()`: returns `yaw: 0` **always** — the player has no rudder at all, and `main.ts` `playerInput()` then hard-sets `p.yawIn = 0`. Add rudder keys **Q/E** (and gamepad pedals if present), wire `pc.yaw` through in `main.ts`, and add the inversion/sensitivity settings the user asked for (exponent curve `out = sign(x)·|x|^γ`, γ 1.0–2.0, per-axis invert flags) in the same place.
- `CurveAerodynamicsSolver.ts`: `ROLL_COORD_DEG = −8` is a fixed auto-coordination fudge. Replace with (a) coordinated yaw from §1, and (b) a genuine adverse-yaw transient: while rolling, `yawRate -= sign(rollRate) · k_adv · |rollRate| · (CD_induced share)`, k_adv ≈ 0.15, *unless* the pilot holds rudder into the turn. This makes uncoordinated rolls slip and makes rudder skill matter; the AI should also feed rudder (it currently returns `yawIn: 0`).
- Bombing: `WeaponSystem` drops gravity bombs but there is no dive-bombing sight. Add a CCIP pipper (bomb impact point projected on the HUD, using the same lead-pipper machinery as the gun) and a minimum-release-altitude cue computed from the pull-out formula above with the current speed/dive angle at 4 g. The 4 g pull-out itself is already possible (maxG 9, pitch rate 55°/s) — it just needs the cue.

## 5. Blade Element Theory — structure behind `IAerodynamicsSolver`

The interface already fits BET: `solve(AeroState): AeroResult` is a pure function returning world-frame forces + body-frame torque. Add a `BladeElementSolver implements IAerodynamicsSolver` alongside `CurveAerodynamicsSolver`; the controller already swaps solvers by construction (`new CurveAerodynamicsSolver()` in `FlightPhysicsController.ts`).

**Station layout (per aircraft):** wing 8 stations per half (16 total), horizontal tailplane 4 per half (8), vertical fin 4, LEX 4 per side on the F/A-18/F-22 (8), fuselage 1, engines as thrust stations (2). **~40 stations.** Do not model the fuselage as a lifting surface — one drag/area station only.

**Per station, per fixed step:**
1. Local velocity: **v_local = v_cg + ω × r**, r = station position relative to CG (this one term is what produces all damping and coupling — pitch damping Cmq, yaw damping Cnr, roll damping Clp — for free).
2. Local AoA: α_i = atan2(−v_local_body.y, v_local_body.z) + twist_i + control deflection effect. Control surfaces change local camber: ΔCl = Cl_δ · δ, Cl_δ ≈ 2.0/rad for aileron/elevator/rudder (i.e. ~0.035 per degree).
3. Section coefficients from a polar table (Cl(α), Cd(α), Cm(α)) — see §6 for the data. Apply finite-wing corrections per surface: lift-slope × cos Λ (Λ = sweep; F/A-18 wing Λ ≈ 45°), CLmax reduction, and induced drag computed globally as CDi = CL²/(π·e·AR) so it is not double-counted per station.
4. Section force: dL = q_i · c_i · dy_i · Cl_i perpendicular to local velocity; dD = q_i · c_i · dy_i · Cd_i along it. Tailplane sees downwash: α_tail = α − ε, ε ≈ 2·CL/(π·AR).
5. Moment: **M = Σ r_i × F_i**, then ω̇ = I⁻¹(M − ω × Iω) with the existing `inertias()` (correct: Ix = mL²/12 pitch, Iy = m(span²+L²)/12 yaw, Iz = m·span²/12 roll). Integrate ω and hand it to Rapier via `setAngvel` exactly as the controller does today — rates become *emergent* instead of prescribed, which also fixes the "torque is fake" problem (today `torque = I·rate/TAU` is exactly inverted by `torqueToRates()`, so no real moment model exists: no Cm_α static stability, no weathercock, no dihedral effect).
6. Asymmetry falls out with no special cases: one wing stalled/damaged → its stations make less lift → rolling moment; engine-out → thrust station at y = ±0.55 m gives yaw moment T·y/I (F-18: ≈ 3.4°/s² — the current hardcoded 5°/s is in the right ballpark but should come from geometry); wing loss → zero that side's stations.
7. **F-35A lift fan:** a thrust station at the fan location (force along body-up, rotating with vector angle) plus the main nozzle station; in hover the fan + nozzle are the lift; blend with wing lift by vector angle. Add a wing-download term (~5–10% in hover) as a refinement. Rotary (UH-60): blade-element-momentum on the rotor, 4 blades × 8 stations, Pitt–Peters inflow; keep the current scalar rotor model until BET lands, but note the current `autorotationLift 0.35` is about right (UH-60 autorotates at ~1,500–2,000 fpm, 60–70 kt).
8. **Cost on mobile:** ~40 stations × ~60 flops ≈ 2,400 flops per solve; at 60 Hz with up to 5 substeps that is ~720k flops/s — negligible. The real cost is allocation churn: preallocate station arrays (Float64Array) and reuse them; no per-station object allocation. X-Plane itself runs its BET at 15 Hz, so even 30 Hz with interpolation is proven adequate — 60 Hz is comfortable.

**Recommended station counts:** wing 16, tailplane 8, fin 4, LEX 8, fuselage 1, engines 2 → **39 stations** for the F/A-18; fewer for the F-35A (no LEX stations, add 1 lift-fan station).

## 6. Smaller corrections (same files)

- `CurveAerodynamicsSolver.ts` `gLoad`: floored at 0.2 — a parked jet reads 0.2 g (the play-test saw exactly this). Fix: **gLoad = 1.0 when grounded**, and in flight include the thrust-vector vertical component for VTOL.
- `FlightPhysicsController.ts` `GROUND_FRICTION = 1.2 m/s²`: real rolling resistance is ~0.02–0.03·g ≈ **0.3 m/s²**. 1.2 m/s² eats ~12% of takeoff acceleration; reduce to 0.3 (brakes 6 m/s² is fine).
- `FlightPhysicsController.ts`: thrust has no altitude lapse — add the (ρ/ρ₀)^0.7 factor from §0 (needed for the 15,000 ft turn-rate match).
- `main.ts` survival score: `score += 1` per second alive, even while stuck on the runway — gate it on airborne (`!player.grounded`).
- `PCFlightInput.ts` keyboard arrows are binary (instant full deflection). Add a stick slew: deflection moves toward the key target at ~3/s (0.3 s to full), which also tames the 55°/s pitch twitch.
- `FlightHUD` (not read in full): "LOW FUEL" flashing at 100% fuel is a threshold/wiring bug — check the low-fuel threshold against `fuel` (0..1 fraction); the burn rate itself is about right (full-throttle endurance ≈ 13 min at the current 0.02 + 0.11·throttle per second on a 0..100 scale, consistent with 6,354 kg internal fuel at afterburner flow).

## 7. Files to download (user to fetch)

1. **NASA TM-104232, "Research Flight-Control System Development for the F-18 High Alpha Research Vehicle" (April 1991)** — https://archive.org/stream/NASA_NTRS_Archive_19910012818/NASA_NTRS_Archive_19910012818_djvu.txt — *why:* the only public source with required pitch/yaw control accelerations at high AoA (pitch 0.37–0.50 rad/s², yaw 0.28–0.35 rad/s²) for calibrating BET control authority.
2. **NACA Report 824, "Summary of Airfoil Data" (Abbott, von Doenhoff & Stivers, 1945)** — https://archive.org/stream/NASA_NTRS_Archive_19930090976/NASA_NTRS_Archive_19930090976_djvu.txt — *why:* section polars (Cl/Cd/Cm vs α) for the NACA 6-series airfoils, including the 65-series the Hornet's wing is derived from; the source data for the BET polar tables.
3. **"Analysis of Programs and Procedures Designed to Mitigate F/A-18 Mishaps Caused by Out-of-Control Flight" (Potter, Univ. of Tennessee, 2001)** — https://trace.tennessee.edu/bitstreams/8d72b889-6522-4cb2-93fc-73626a86e097/download — *why:* F/A-18 OOCF modes (including the falling-leaf mode), spin-entry conditions and recovery procedures.
4. **A1-F18AC-NFM-500, F/A-18 NATOPS Pocket Checklist** — https://www.docdroid.net/file/download/uQCJuVs/f-18abcd-hornet-pocket-checklist-pdf.pdf (link may expire; if dead, search "A1-F18AC-NFM-500") — *why:* the verbatim spin-recovery checklist quoted in §2.
5. **AirfoilTools polar database** — http://airfoiltools.com/ (search NACA 65-series, e.g. NACA 65(2)-215) — *why:* ready-made XFOIL Cl/Cd/Cm polars as CSV for the BET tables. **Caveat:** most polars there are at Re ≤ 10⁶; a fighter wing flies at Re ≈ 5×10⁷, so use the *shape* and re-anchor CLmax/slope to the Hornet numbers in §2, do not use the raw values.
6. **A1-F18AC-NFM-000, F/A-18 NATOPS Flight Manual (full)** — *no stable public direct link verified; search "A1-F18AC-NFM-000 pdf"* — *why:* the authoritative E-M diagrams (turn-rate vs speed charts) for calibrating every jet; worth fetching if a copy can be found, but items 1–5 above are sufficient to implement everything in this report.

None needed beyond the above for the F-22/F-15E/F-35A numbers used here — those came from the USAF AIB report (F-22 rotation 136–143 KCAS, takeoff 163–164 KCAS), test-pilot accounts (F-35A rotation ~150 kt, approach 155 kt at 40,000 lb), and the Fornof briefing (F-15 instantaneous 21°/s, sustained 15–16°/s).
