# Shoot-down, fire, explosion, fragmentation & spectate — research report

Source: research agent, 17 Nov 2026. Read-only. Code studied: `src/combat/DamageableEntity.ts`, `CrashVFXManager.ts`, `WeaponSystem.ts`, `src/physics/FlightPhysicsController.ts` (Aircraft), `CurveAerodynamicsSolver.ts`, `IAerodynamicsSolver.ts`, `src/aircraft/AircraftSpec.ts`, `src/entities/AircraftAIController.ts`, `src/main.ts`. Web sources: War Thunder official "Changes to Bomber Survivability" dev blog (warthunder.com/en/news/10189), War Thunder forums ("Severe damage", fire/FPE threads), War Thunder datamine camera config, NTSB/USAF real-world cases (F-15 one-wing landing 1983, A-10 Baghdad 2003, Voepass ATR flat spin 2024, Florida business-jet crash 2026).

## 0. What the code does today — the gap inventory

- **`DamageableEntity.ts` exists but is NOT wired in.** `Aircraft` keeps its own scalar `hp` (0–100) plus `damageZones` (`wingL/wingR/tail/engineL/engineR`, 0–1). Only the `lastHitBy` kill-credit rule is shared. Two parallel damage systems exist; one is dead code.
- **`Aircraft.applyDamage()`** (`FlightPhysicsController.ts`): zone multipliers wing ×1.2, engine ×1.5; zone damage accumulates at 0.002–0.005 per damage point; wing detaches at 0.75; `hp <= 0` → `crash('shot down')`. Weapon type does **not** affect the outcome — gun (12 dmg/hit, 800 rpm), missile (40 dmg) and bomb (≤60 dmg) differ only in the damage number, and the hit zone is random (25% wing / 30% engine / 45% fuselage) regardless of weapon. A missile always hits `'fuselage'`.
- **`crash()` is an instant vanish.** `alive = false`, `group.visible = false`, body disabled, `onCrash` → `main.ts handleDeaths()`: one explosion sprite at the death position, a boom sound, a scorch decal on the ground below, and the player respawns after a fixed 3,000 ms `setTimeout`. There is no fire, no falling wreck, no fragmentation of the airframe (only the wing-detach path spawns a debris body), and the jet disappears in mid-air.
- **Smoke/fire visuals are dead code.** `Aircraft.getDamageFx()` (smoke < 60 hp, fire < 35 hp) is defined but **never called anywhere** (verified by grep), and `CrashVFXManager.spawnSmoke()` is likewise never called. A damaged jet shows no smoke or fire today.
- **What already works and should be reused:** the solver degrades flight with damage — `CurveAerodynamicsSolver.ts` reduces roll authority by average wing damage, cuts thrust by `engineDmgAvg × 0.7`, adds asymmetric roll from `(wingL − wingR)`, engine-out yaw from `(engineR − engineL)`, and has a stall/spin model (`spin` 0–1, nose drop, side acceleration). `CrashVFXManager` already has pooled explosion sprites (0.9 s), smoke plumes (5 sprites, 6 s life), tumbling Rapier debris bodies (10 s life, removed on ground contact), scorch decals, a point-light flash and a camera-shake hook. `WeaponSystem` already has gun/missile/bomb Rapier bodies with proximity fusing.
- **AI:** `hp < 35` → evade, `hp < 20` → RTB. On death the AI jet vanishes like the player; it never enters a death spiral.
- **Camera/respawn (`main.ts`):** `updateCamera()` chase/cockpit/free-orbit all track `player.pos`; there is no death or spectate mode. `respawnPlayer()` resets to the nearest runway, full repair/refuel/rearm, 5 s spawn protection.

## 1. War Thunder's damage model — what to copy

WT models aircraft as a set of modules with individual HP: wing sections (3 per wing — **2 sections gone = "severe damage"**), engines, fuel tanks, tail (horizontal stabilizers separate from the vertical fin in the current model), control surfaces, pilot/crew, and oil/hydraulic/fuel lines. Key rules:

- **"Severe damage" (WT's kill-credit threshold)** is defined as any of: destruction of all engines (or the only remaining one), destruction of all elements of the control system, separation of **more than half of at least one wing**, or destruction of all horizontal stabilizers. This is the right kill-credit rule for Pandora: credit the kill the moment severe damage is inflicted, not when the wreck hits the ground.
- **Structural failure is rare from fragments alone.** Gaijin's research (BRL Memo 484, NACA RM L52H01a, NACA TN 4111) found small fragments (~3 g) cannot compromise spars, and blast damage from external explosions is usually negligible. **Over 60% of real combat losses were fires in fuel/oil systems, not structural failure.** Most damage in reality forces an abort and emergency landing within 30 min–2 h, not instant destruction. Implication: gun hits should mostly *degrade* (holes → less lift/control, leaks), while kills come from fire, spar destruction by sustained fire, or a direct warhead hit on a fuel tank/engine.
- **Progressive degradation, not binary:** "even if wings aren't sheared off, they should provide less lift and control as holes are blown into them" — WT's own community/design direction. The current solver already does this via `wingDmgAvg`/`engineDmgAvg`; it just needs more zones feeding it.
- **Visual language:** smoke colour/density encodes the damage — white/grey smoke for fuel/oil leaks, thick black smoke for an engine fire, orange flame for a burning fuel tank, thin grey trail for light airframe damage. The player reads the kill from the smoke before the explosion.

## 2. Explosion vs fragmentation — what decides the outcome

The outcome must depend on **warhead type + hit location**, not on a random zone roll:

| Weapon | Mechanism | vs fuel tank | vs engine | vs wing spar | vs fuselage |
|---|---|---|---|---|---|
| Gun, AP/API (current gun) | Kinetic + small incendiary | 2% ignition per hit (BRL Memo 484: AP without incendiary ignites fuel in 0–2% of cases); use 5% for API mix. Puncture → leak, which *raises* later ignition chance | Oil fire chance ~10% | Almost never breaks a spar (3 g fragments can't) — accumulates holes only | Holes, control-cable damage |
| Missile, proximity blast-fragmentation (current: 8 m fuse, 40 dmg) | Fragments + blast; e.g. AIM-54: 60 kg TNTeq warhead, wide proximity fuse; RB05A: 76.8 kg TNTeq, ~100 m fragment dispersion | Detonation within 3 m of the tank: **25% immediate fuel explosion** (instant kill, big fireball); otherwise starts a fuel fire | 15% engine fire; heavy fragment damage to that engine | Direct proximity detonation on the wing: 30% spar break → wing separation | Fragment damage to **all** zones with distance falloff |
| Bomb (current: 70 m blast, ≤60 dmg) | Blast + fragments | Direct hit: fuel explosion | Engine destroyed in blast radius | Spar break within ~10 m of detonation | Heavy damage, falloff with distance |
| Over-G / hard landing | Structural | — | — | Spar fails above limit load (already: damage above maxG + 1.5) | Crush on impact |

Implementation consequence (`WeaponSystem.ts`): the hit resolver must stop picking a random zone. Gun hits need an impact *point* (the current hitscan already computes the hit point — pass it through, map point → nearest zone by distance to zone anchor points on the airframe). Missile/bomb proximity detonations must apply damage to **all zones within radius with falloff**, plus the ignition rolls above. The current `applyDamage(amount, zone, byPlayer)` signature can stay; the *caller* changes.

## 3. Fire — the main killer (War Thunder's current model, "Eagle's Shadow" changelog)

- **Fuel-tank fire:** intensity builds over time — weak in the first seconds, full intensity at **40–60 s** (game pace: scale to ~12 s to full). It heats the structure around it; damage to the heated structure grows as the fire persists. Probability of catching fire **rises with accumulated tank damage** — each puncture leaks fuel, and each subsequent incendiary hit is more likely to ignite it. A 20 mm+ hit increases leak probability.
- **Engine fire:** builds **faster** than a fuel fire (oil igniting on a hot manifold) but has a **lower maximum intensity** and stays localised. It "lasts far longer and basically does not extinguish unless fuel-starved or the extinguisher is used."
- **Fire spread:** fire in the rear fuselage spreads to the stabilizers, control surfaces and vertical fin. **Sustained fire in the wing can destroy the spar** (thermal damage passes through spar material — WT fixed their model so fire is no longer blocked by the spar's armour value). So a wing fire that burns long enough *causes* wing separation — the correct mechanism for delayed wing loss, not instant spar breakage from fragments.
- **Extinguishing:** WT has a limited-use fire extinguisher (FPE); some fires (ammo burn) cannot be extinguished at all, and an engine fire can re-ignite. Pandora has no FPE, so use the real-world analogue: a fuel fire's self-extinguish chance rises with airspeed (slipstream) — recommended 2%/s below 150 m/s IAS, 8%/s above 300 m/s IAS — and an engine fire has a 15%/s chance to go out when that engine's throttle is cut to 0 (fuel starvation). Diving fast with the throttle cut is therefore the correct player response, which is realistic and teaches itself.
- **Fire damage rates (proposed, tuned to the current 100 hp scale):** fuel fire 2.0 hp/s at full intensity to the burning zone, spreading to one adjacent zone per ~8 s; engine fire 1.2 hp/s, localised. The existing `FIRE_BURN_DPS = 1.5` in `FlightPhysicsController.ts` is in the right ballpark — split it per fire type rather than replacing it.
- **New zone needed: fuel tank.** The current `DamageZones` (`wingL/wingR/tail/engineL/engineR`) has no fuel zone and no tail sub-zones. Add `fuel` (and split `tail` into `tailH`/`tailV` if the builder wants WT's separate stabilizer/fin modelling — recommended, since losing the vertical fin alone should cause wild roll/yaw without killing the jet outright).

## 4. Freefall and crash physics — real numbers

- **Terminal velocity of a wreck:** Vt = √(2·m·g / (ρ·Cd·A)). For a 20 t jet tumbling (Cd·A ≈ 40 m²): **Vt ≈ 89 m/s (~320 km/h, ~17,500 fpm)**. Nose-down streamlined (Cd·A ≈ 6 m²): **Vt ≈ 230 m/s (~830 km/h)**. These are the two regimes the falling wreck should move between — tumbling wreck ≈ 90 m/s, a wreck that noses down accelerates toward 200+ m/s.
- **Flat spin (real data):** Voepass ATR 72, Aug 2024 — entered a flat spin at 17,000 ft, maximum descent **24,000 fpm (120 m/s)**, slight nose-down, rotating slowly. The F-106 "Cornfield Bomber" flat spin: "very flat and rotating quite slowly, looked like the pitot tube was stationary with the aircraft rotating around it." A deep-stalled airliner falls "like a leaf" at ~100–140 mph (45–63 m/s). Use **50–120 m/s** as the flat-spin descent band.
- **Nose-down uncontrolled dive:** the Nov 2026 Florida business-jet crash — fell at 20,000 fpm (100 m/s) on the mayday call, reached an estimated **300–500 mph (135–225 m/s)** at impact, debris field ~0.5 × 0.4 miles. Impact with fuel remaining = fireball; without fuel = crumpled wreck, no explosion.
- **What is survivable (real cases):** F-15D, Negev 1983 — mid-air collision took the entire right wing; the jet fell into a tight spin, the pilot lit afterburners, regained control on fuselage/intake lift, and landed at **260 kt** (twice normal landing speed), stopping 10 m short of the barrier. A-10, Baghdad 2003 (Capt. Kim Campbell) — missile hit the tail, hydraulics out, aircraft spiralled; she switched to manual reversion and flew home with massive tail/fuselage/wing damage. Design lesson: **losing one wing or one engine or the hydraulics is survivable; losing both engines, all controls, or the tail is not.** This maps exactly onto WT's severe-damage rule in §1.
- **Ground impact:** on terrain contact the wreck should produce a large explosion **only if fuel remains** (scale with `fuel` fraction), a debris scatter (5–10 Rapier debris bodies, the existing `spawnDebris` path, thrown outward at 10–30 m/s), a scorch decal (exists), and a persistent smoke column. A wreck that hits at <15 m/s vertical speed with controls intact is a hard landing, not a crash (current `HARD_LANDING_VS = 9` m/s is already close to reality for gear-down; keep it).

## 5. Spectate-before-respawn — how War Thunder does it, mapped to `main.ts`

- **War Thunder's flow (Air RB):** when you are shot down, the camera follows the burning wreckage as it plummets and crashes into the terrain, creating a large explosion — then the view transitions to Spectator Mode (free-roaming camera, ~23 m offset from observed aircraft) with a "Return to the Hangar" option. In Ground RB the sequence is a ~5–10 s "kill cam" from the killer's point of view, then back to the vehicle picker. Key design points: (a) the death camera **stays with the wreck until impact** — you watch your own crash; (b) it is **time-bounded by the fall itself**, not a fixed timer; (c) the player can **skip** it; (d) while spectating you cannot control anything.
- **Mapping onto `main.ts`:** the current flow is `handleDeaths()` → explosion at death position → toast "YOU WENT DOWN" → `setTimeout(respawnPlayer, 3000)`. The change is:
  1. On player death, do **not** start the 3 s timer. Set a `spectating` state; `updateCamera()` gains a death mode (new `camMode = 3`, or a separate flag) that keeps the chase camera on the wreck — reuse the existing chase math with the wreck's `pos`/`fwd`, at a slightly longer distance (40–45 m) so the fire and tumble read clearly.
  2. The wreck falls for as long as the fall lasts (typically 8–15 s from combat altitude). Respawn triggers on **ground impact + 2 s**, or immediately when the player presses the skip (respawn button / `R` key / tap the "Respawn now" HUD button). While spectating, `playerInput()` must stay inert (it already zeroes inputs when `!p.alive`), and `C` may cycle to free-orbit around the wreck.
  3. Show a HUD banner: "SHOT DOWN — [reason/killer] — watching the wreck · Respawning after impact (R to skip)". Show the killer's name/type when `lastHitBy` is known.
  4. Enemy deaths need no camera change — the player's camera already looks at the world, so the falling enemy wreck is visible in frame. The only requirement is that the enemy wreck actually falls (see §6) instead of vanishing.
  5. Edge cases: death below ~100 m altitude (fall < 3 s) → respawn almost immediately after impact; death over water/terrain edge → impact is the terrain heightfield contact, same path; player disconnects/pauses during spectate → respawn immediately on resume.

## 6. The death sequence — recommended implementation (file/function targets)

The cleanest design reuses the existing solver instead of writing a second physics path:

1. **New state: `dying` (not dead).** In `FlightPhysicsController.ts`, split `crash()` into `startDying(reason)` and `finishCrash()`. `startDying`: `alive` stays true for physics but a `dying = true` flag makes the aircraft untargetable (AI skips it, weapons skip it, lock breaks), zeroes `thrTarget`, and records the death cause/killer. The aircraft keeps stepping with the solver running in degraded mode — throttle 0, controls zeroed (or, if the tail is gone, controls ignored) — so the existing stall/spin model, asymmetric roll and engine-out yaw produce the tumble naturally. **Restore gravity on the Rapier body** (`body.setGravityScale(1, true)` — it is currently 0 because gravity lives in the solver's force sum; in dying mode the solver's lift is ~0 anyway, so let Rapier gravity drive the fall and keep solver drag for the terminal-velocity behaviour).
2. **Wire `DamageableEntity` in** (`FlightPhysicsController.ts applyDamage`, `DamageableEntity.ts`): make it the single source of truth — construct one per aircraft with the zone table from §7, delete the parallel scalar-`hp` bookkeeping (or keep `hp` as a getter over the entity). Add the `fuel` zone and a `fire` state machine (fuel fire / engine fire, with the ramp/spread/extinguish numbers from §3). `applyDamage` must receive the **hit point** so the zone is derived from geometry, not a random roll (`WeaponSystem.ts fireGun/aiGunShot` already compute the hit point; pass it through).
3. **Weapon outcome table** (`WeaponSystem.ts`): implement the §2 table — missile/bomb proximity applies damage to all zones in radius with falloff + ignition rolls; gun applies per-hit zone damage + 5% fuel ignition on a fuel hit. Missile currently always hits `'fuselage'` — change to the proximity-zone loop.
4. **Wing/tail separation** (`FlightPhysicsController.ts detachWing`, `AircraftRigBindings.ts`): keep the existing `onDetachSegment` → `CrashVFXManager.spawnDebris` path, but trigger it from the **spar/fire** rules (§3), not only from the 0.75 damage threshold; add the same path for the tail (`detachSegment('tail_section')` — the rig already binds `tail_section` per its slot list). Detached segments are already tumbling Rapier bodies; spawn 2–4 extra small debris boxes on a fatal hit for fragmentation feel.
5. **Ground impact** (`main.ts handleDeaths`, `FlightPhysicsController.ts` terrain safety net): the dying aircraft's terrain contact (via `notifyContact` impactVy, or the `pos.y < terrainH − 0.5` net) becomes `finishCrash()`: big explosion scaled by remaining fuel (`vfx.spawnExplosion(pos, 1 + fuel)`), debris scatter, scorch decal, and only then `alive = false` + the respawn/spectate logic of §5. The current `handleDeaths` explosion-at-death-position moves to impact position.
6. **Fire/smoke visuals** (`main.ts` animate loop): call `player.getDamageFx()` (and the same for enemies) **every frame** and drive `vfx.spawnSmoke(pos, colour)` — orange `0xff7722` flame sprites + black smoke for fuel fire, grey for engine damage, white for light damage. This is a one-loop wiring fix for the dead code identified in §0. Add a trailing smoke column behind the falling wreck (a plume spawned every ~0.25 s along its path, fading over ~6 s — the existing plume lifetime).
7. **AI dying behaviour** (`AircraftAIController.ts`): when the aircraft enters `dying`, the controller stops firing and stops steering (return zero controls); if damage is non-fatal (engine out only, `hp < 20`), the existing RTB behaviour already applies — keep it, and make RTB actually attempt a landing instead of orbiting (currently it navigates to runway + 200 m and circles; a damaged AI should descend onto the runway or ditch).
8. **HUD** (`FlightHUD.ts`): add a damage-state indicator (per-zone colour chips: green/yellow/red/black), a "🔥 FIRE" warning with the affected zone, a "SEVERE DAMAGE" banner when the WT threshold is crossed, and the spectate/respawn UI from §5.

## 7. Component-damage table (proposed HP, tuned to current weapons: gun 12/hit, missile 40, bomb ≤60)

Total HP stays ~100 so time-to-kill does not change drastically; the *distribution* is what changes. `destroyed` = fuselage 0, OR both engines 0, OR tail 0 (controls gone), OR total 0. Wing 0 = wing detaches → dying spin (not instant vanish). Fuel 0 = fuel fire starts (not instant death).

| Zone | F/A-18F | F-35A* | F-22 | F-15E | MiG-29 | Su-34 |
|---|---|---|---|---|---|---|
| Fuselage | 40 | 40 | 45 | 42 | 38 | 45 |
| Wing L / Wing R (each) | 25 | 25 | 30 | 28 | 25 | 30 |
| Tail (H-stab) | 20 | 20 | 22 | 20 | 18 | 22 |
| Vertical fin | 15 | 15 | 15 | 15 | 15 | 15 |
| Engine L / Engine R (each) | 20 | — | 22 | 22 | 20 | 22 |
| Engine (single) | — | 25 | — | — | — | — |
| Fuel tank | 20 | 20 | 22 | 22 | 18 | 25 |
| **Effect when destroyed** | | | | | | |
| Wing | detach → violent roll/spin → dying | same | same | same | same | same |
| Tail | controls dead → dying (uncontrollable) | same | same | same | same | same |
| Fin | wild yaw/roll, flyable with difficulty | same | same | same | same | same |
| One engine (twin) | thrust −50%, yaw to dead side, RTB-able | n/a | same | same | same | same |
| Single engine (F-35A) | n/a | total thrust loss → glide/dying | n/a | n/a | n/a | n/a |
| Fuel | fire starts (see §3); leak → fuel drains 2× | same | same | same | same | same |

*F-35A is single-engine: one `engine` zone; losing it is a WT severe-damage kill (all engines destroyed), so it enters `dying` as a glider. All other player/enemy jets are twin-engine.

## 8. Downloads needed

**None.** All research was completed from public web sources. The single most useful reference the builder should open is Gaijin's official bomber damage-model dev blog (warthunder.com/en/news/10189 — fuel/engine fire curves, spar/fire interaction, BRL/NACA source documents listed there) and the "Severe damage" forum thread (forum.warthunder.com/t/severe-damage/178268). No flight manual, dataset, or model file needs downloading from the user.

## 9. Suggested build order for the implementer

1. Wire `DamageableEntity` + `fuel` zone + fire state machine (steps 2–3 of §6).
2. `dying` state + freefall via solver-degraded stepping (step 1 of §6) — the core ask.
3. Ground-impact explosion/debris/scorch (step 5).
4. Spectate camera + respawn flow (step 5 of §6, §5).
5. Weapon outcome table (step 3) and wing/tail separation (step 4).
6. Fire/smoke visual wiring (step 6) + HUD indicators (step 8).
7. AI dying/RTB behaviour (step 7).
