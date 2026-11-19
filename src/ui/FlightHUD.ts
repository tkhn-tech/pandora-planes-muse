// FlightHUD — 10 Hz HUD text, minimap, lead pipper, toasts and the
// ?debug=1 panel. Ported from the v2 template (pandora-template.html
// ~1835–1921 updateHud/drawMinimap, ~1921–1930 toast, ~428–470 debug).
// DOM ids are kept VERBATIM from the template — index.html must contain
// them (full list in the worker report).

import * as THREE from 'three';
import type { Aircraft } from '../physics/FlightPhysicsController';

export const MS2KT = 1.94384;
export const M2FT = 3.28084;

export interface HudGameState {
  score: number;
  kills: number;
  traps: number;
  elapsed: number; // seconds
  lockTarget: Aircraft | null;
  /** True when the jet is below the min-release altitude for its dive
   *  (CCIP, 4 g pull-out) — the HUD shows PULL UP. */
  belowMinReleaseAlt?: boolean;
}

export interface MinimapEntities {
  player: Aircraft | null;
  enemies: Aircraft[];
  traffic: Aircraft[];
  carriers: Array<{ alive: boolean; pos: THREE.Vector3; yaw: number }>;
  runways: { x: number; z: number; len: number; dir: THREE.Vector3 }[];
  /** Cumulative floating-origin shift (scene coords + shift = original). */
  originShift: THREE.Vector3;
}

const R2D = 180 / Math.PI;

function fmtTime(s: number): string {
  const m = Math.floor(s / 60);
  const ss = Math.floor(s % 60);
  return String(m).padStart(2, '0') + ':' + String(ss).padStart(2, '0');
}

/** Speed tape (kt): GROUND speed while the gear is on a surface,
 *  INDICATED airspeed once airborne. IAS is wind-relative (vAir =
 *  vel − wind), so a parked jet in the default 3 m/s wind reads
 *  ~6 kt on the tape while standing still — the v2 play-test read
 *  that as the jet "creeping at 6 kt with 0% throttle". On the
 *  ground the tape must show what the jet is actually doing: 0. */
function spdKt(ac: Aircraft): number {
  if (ac.grounded) return Math.hypot(ac.vel.x, ac.vel.z) * MS2KT;
  return ac.ias * MS2KT;
}

export class FlightHUD {
  private readonly el: Record<string, HTMLElement | null> = {};
  private readonly mini: CanvasRenderingContext2D | null;
  private toastTimer: ReturnType<typeof setTimeout> | null = null;

  constructor() {
    const ids = [
      'hudJet', 'hudSpd', 'hudAlt', 'hudVsi', 'hudHdg', 'hudAoA', 'hudBank', 'hudG',
      'hudFuel', 'hudWpn', 'hudScore', 'hudVec', 'vecPill', 'vecRow',
      'miniKills', 'miniTraps', 'miniTime', 'warn', 'lockInd', 'pipper',
      'ccip', 'rotateCue', 'toast', 'toastTitle', 'toastSub', 'debug',
      'soundBtn', 'dmgFuse', 'dmgWing', 'dmgEng', 'dmgFuel',
    ];
    for (const id of ids) this.el[id] = document.getElementById(id);
    const miniCv = document.getElementById('miniCv') as HTMLCanvasElement | null;
    this.mini = miniCv ? miniCv.getContext('2d') : null;
  }

  /** 10 Hz text update (main.ts calls this on its hudT timer). */
  update(ac: Aircraft, game: HudGameState): void {
    const set = (id: string, text: string) => {
      const e = this.el[id];
      if (e) e.textContent = text;
    };
    set('hudSpd', String(Math.round(spdKt(ac))));
    set('hudAlt', String(Math.round(Math.max(0, ac.pos.y) * M2FT)));
    set('hudVsi', String(Math.round(ac.vsi * M2FT * 60)));
    const hdg = ((Math.atan2(ac.fwd().x, ac.fwd().z) * R2D) % 360 + 360) % 360;
    set('hudHdg', String(Math.round(hdg)).padStart(3, '0') + '°');
    set('hudAoA', Math.round(ac.aoa) + '°');
    // Bank indicator (Issue 1): the ACTUAL bank from the physics
    // quaternion (Euler YXZ z; positive = left bank). Steer right must
    // read "N° ▶" (right bank) and steer left "◀ N°" — if this ever
    // disagrees with the stick, the input chain is inverted, not the
    // physics. Level within 2° reads 0°.
    const bankDeg = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z * R2D;
    set(
      'hudBank',
      bankDeg > 2
        ? `◀ ${Math.round(bankDeg)}°`
        : bankDeg < -2
          ? `${Math.round(-bankDeg)}° ▶`
          : '0°',
    );
    set('hudG', ac.gload.toFixed(1));
    set('hudFuel', Math.round(ac.fuel * 100) + '%');
    // ROTATE cue (BET §3): green, shown while the gear is on a surface
    // and the jet is at/above its per-spec rotation speed. Pass 9
    // (item 7): this toggle sits FIRST, before the damage chips and
    // the warn section — it is the only thing that clears the cue, and
    // in v4.3 it stayed ON while parked, dead, hovering and flying at
    // 247 kt (a regression from v4.2, where it cleared after liftoff).
    // The getter itself is correct (probe-rotate-cue) and the live
    // stick was the double-shifted height query holding onSurface
    // true while airborne; putting the toggle first means a throw in
    // a later section can no longer freeze the cue ON either.
    const rc = this.el['rotateCue'];
    if (rc) rc.classList.toggle('show', ac.alive && ac.rotateCue);
    const fuelEl = this.el['hudFuel'];
    if (fuelEl?.parentElement) {
      fuelEl.parentElement.classList.toggle('warn', ac.fuel < 0.2);
    }
    set(
      'hudWpn',
      `${ac.msl} MSL · ${ac.flares} FLR · ${ac.bombs} BMB`,
    );
    set('hudScore', String(game.score));
    set('miniKills', String(game.kills));
    set('miniTraps', String(game.traps));
    set('miniTime', fmtTime(game.elapsed));

    // Per-zone damage chips (issue 5): FUSE / WING / ENG / FUEL, from
    // the damage entity's per-zone HP (the same source the solver's
    // degradation and the severe-damage rule read). WING and ENG show
    // the WORST side (one shot-off wing / dead engine is what the
    // pilot needs to know); F-35A is single-engine (one `engine`
    // zone, not engineL/engineR). Bands: >=70% ok (dim green),
    // 35-70% hurt (amber), <35% crit (red), 0 dead (solid red). A
    // burning zone gets the 🔥 prefix + pulsing border, so the chips
    // say WHERE the fire is, not just that there is one (the FIRE
    // banner alone never said which zone was burning).
    const de = ac.damageEntity;
    const frac = (zone: string): number => {
      const max = de.maxHpFor(zone);
      return max > 0 ? de.hpFor(zone) / max : 1;
    };
    const engFrac =
      de.maxHpFor('engine') > 0
        ? frac('engine')
        : Math.min(frac('engineL'), frac('engineR'));
    const fireChip =
      ac.fireZone === 'fuel'
        ? 'dmgFuel'
        : ac.fireZone === 'engine' || ac.fireZone === 'engineL' || ac.fireZone === 'engineR'
          ? 'dmgEng'
          : ac.fireZone === 'wing' || ac.fireZone === 'wingL' || ac.fireZone === 'wingR'
            ? 'dmgWing'
            : ac.fireZone
              ? 'dmgFuse'
              : null;
    const chipDefs: Array<{ id: string; label: string; health: number }> = [
      { id: 'dmgFuse', label: 'FUSE', health: frac('fuselage') },
      { id: 'dmgWing', label: 'WING', health: Math.min(frac('wingL'), frac('wingR')) },
      { id: 'dmgEng', label: 'ENG', health: engFrac },
      { id: 'dmgFuel', label: 'FUEL', health: frac('fuel') },
    ];
    for (const c of chipDefs) {
      const chip = this.el[c.id];
      if (!chip) continue;
      const band =
        c.health <= 0 ? 'dead' : c.health < 0.35 ? 'crit' : c.health < 0.7 ? 'hurt' : 'ok';
      const burning = fireChip === c.id;
      chip.className = `dmg ${band}${burning ? ' fire' : ''}`;
      const lbl = chip.querySelector('b');
      if (lbl) lbl.textContent = burning ? `🔥 ${c.label}` : c.label;
      const fill = chip.querySelector('.bar i') as HTMLElement | null;
      if (fill) fill.style.width = `${Math.round(c.health * 100)}%`;
    }

    // Stall / spin / fire / pull-up / low-fuel warnings (priority order).
    // Spin first: the NATOPS arrow is the only thing that matters in
    // a spin (spinArrow -1 = push the stick LEFT, +1 = push it RIGHT).
    // Fire second: a burning jet (damage-model §3) — the kind tells
    // the player what to do (fuel fire: dive fast to blow it out;
    // engine fire: cut the throttle).
    const w = this.el['warn'];
    if (w) {
      // Dying jets own no #warn (live finding 4): while the jet is
      // dying the spectate banner ("SHOT DOWN — watching the wreck")
      // owns the screen. #warn is gated on !dying so a stale STALL /
      // PULL UP / LOW FUEL cannot linger through the death sequence.
      if (ac.dying) {
        w.classList.remove('show');
        w.textContent = '';
      } else if (ac.spin > 0.5 && ac.alive) {
        w.textContent =
          ac.spinArrow < 0
            ? '◀ SPIN — PUSH STICK LEFT'
            : 'SPIN — PUSH STICK RIGHT ▶';
        w.classList.add('show');
      } else if (ac.fireKind && ac.alive) {
        w.textContent =
          ac.fireKind === 'fuel'
            ? '🔥 FUEL FIRE — DIVE FAST TO BLOW IT OUT'
            : '🔥 ENGINE FIRE — CUT THROTTLE TO STARVE IT';
        w.classList.add('show');
      } else if (ac.stalled && ac.alive) {
        w.textContent = '⚠ STALL — NOSE DOWN, FULL POWER';
        w.classList.add('show');
      } else if (game.belowMinReleaseAlt && ac.alive) {
        w.textContent = '⚠ PULL UP — BELOW MIN RELEASE ALT';
        w.classList.add('show');
      } else if (ac.fuel < 0.15 && ac.alive) {
        // 0.15 on the 0..1 fuel scale — the old `ac.fuel < 15` compared a
        // 0..1 fraction against a 0..100 threshold, so LOW FUEL flashed
        // constantly even at 100% fuel.
        w.textContent = '⚠ LOW FUEL';
        w.classList.add('show');
      } else {
        w.classList.remove('show');
        // Clear the text too (live finding 4): removing `show` hides
        // the banner visually (opacity 0) but the textContent stays
        // in the DOM, so a DOM-reading check reports a stale banner
        // as present. Clearing the text makes the DOM and the pixels
        // agree.
        w.textContent = '';
      }
    }

    // Lock indicator.
    const li = this.el['lockInd'];
    if (li) {
      if (game.lockTarget && ac.lockT > 1.2) {
        li.textContent = '◉ LOCKED — ' + game.lockTarget.spec.short;
        li.className = 'locked';
      } else if (game.lockTarget) {
        li.textContent =
          '○ LOCKING ' + Math.round((ac.lockT / 1.2) * 100) + '%';
        li.className = 'lock';
      } else {
        li.textContent = 'NO LOCK';
        li.className = '';
      }
    }

    // Thrust-vector readout (F-35A only): angle + mode. HOVER at
    // vector 70 deg+ (lift fan + FCS holding attitude), TRANSIT in
    // between, forward flight below 20 deg.
    this.el['vecPill']?.classList.toggle('hidden', !ac.spec.vtol);
    if (ac.spec.vtol) {
      const v = Math.round(ac.vector);
      set('hudVec', v >= 70 ? v + '° HOVER' : v >= 20 ? v + '° TRANSIT' : v + '°');
    }
  }

  /** Minimap (144 x 144 canvas): green runways, blue carrier, grey traffic,
   *  red enemy diamonds, white player triangle. Template drawMinimap. */
  drawMinimap(ctx: CanvasRenderingContext2D, e: MinimapEntities): void {
    const W = 144;
    // 12 km map: the full world spans 12000 m + a 100 m margin each side.
    const S = W / 12200;
    const X = (x: number) => (x + e.originShift.x + 6100) * S;
    const Z = (z: number) => (z + e.originShift.z + 6100) * S;
    ctx.clearRect(0, 0, W, W);
    ctx.fillStyle = 'rgba(10,25,40,0.85)';
    ctx.fillRect(0, 0, W, W);
    ctx.strokeStyle = '#7dff8a';
    ctx.lineWidth = 2;
    for (const r of e.runways) {
      const sx = r.x - e.originShift.x;
      const sz = r.z - e.originShift.z;
      ctx.beginPath();
      ctx.moveTo(X(sx - r.dir.x * r.len / 2), Z(sz - r.dir.z * r.len / 2));
      ctx.lineTo(X(sx + r.dir.x * r.len / 2), Z(sz + r.dir.z * r.len / 2));
      ctx.stroke();
    }
    for (const c of e.carriers) {
      if (!c.alive) continue;
      ctx.fillStyle = '#6ab8ff';
      ctx.save();
      ctx.translate(X(c.pos.x), Z(c.pos.z));
      ctx.rotate(c.yaw);
      ctx.fillRect(-2, -5, 4, 10);
      ctx.restore();
    }
    ctx.fillStyle = '#9db8cc';
    for (const t of e.traffic) {
      if (!t.alive) continue;
      ctx.beginPath();
      ctx.arc(X(t.pos.x), Z(t.pos.z), 2, 0, 7);
      ctx.fill();
    }
    for (const en of e.enemies) {
      if (!en.alive) continue;
      ctx.fillStyle = '#ff5a5a';
      ctx.save();
      ctx.translate(X(en.pos.x), Z(en.pos.z));
      ctx.rotate(Math.PI / 4);
      ctx.fillRect(-2.4, -2.4, 4.8, 4.8);
      ctx.restore();
    }
    if (e.player?.alive) {
      const hdg = Math.atan2(e.player.fwd().x, e.player.fwd().z);
      ctx.fillStyle = '#ffffff';
      ctx.save();
      ctx.translate(X(e.player.pos.x), Z(e.player.pos.z));
      ctx.rotate(Math.PI - hdg);
      ctx.beginPath();
      ctx.moveTo(0, -5);
      ctx.lineTo(3.4, 4);
      ctx.lineTo(-3.4, 4);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }

  /** Convenience: draw with the canvas this HUD was constructed with. */
  drawMinimapDefault(e: MinimapEntities): void {
    if (this.mini) this.drawMinimap(this.mini, e);
  }

  /** Place the CCIP (bomb impact point) pipper at viewport pixel coords.
   *  Gold normally; red when the jet is below the min-release altitude
   *  for its dive (no room for the 4 g pull-out). Pass null to hide it. */
  showCcip(x: number | null, y: number | null, danger = false): void {
    const c = this.el['ccip'];
    if (!c) return;
    if (x === null || y === null) {
      c.style.display = 'none';
      return;
    }
    c.style.left = x + 'px';
    c.style.top = y + 'px';
    c.style.display = 'block';
    c.classList.toggle('danger', danger);
  }

  hideCcip(): void {
    this.showCcip(null, null);
  }

  /** Place the gold lead-pipper ring at viewport pixel coords (from
   *  WeaponSystem.computeLeadPipper). Pass null to hide it. */
  showPipper(x: number | null, y: number | null): void {
    const pip = this.el['pipper'];
    if (!pip) return;
    if (x === null || y === null) {
      pip.style.display = 'none';
      return;
    }
    pip.style.left = x + 'px';
    pip.style.top = y + 'px';
    pip.style.display = 'block';
  }

  hidePipper(): void {
    this.showPipper(null, null);
  }

  /** Toast banner, auto-hides after 3.2 s (template toast). */
  toast(msg: string, sub?: string): void {
    const t = this.el['toast'];
    if (this.el['toastTitle']) this.el['toastTitle'].textContent = msg;
    if (this.el['toastSub']) this.el['toastSub'].textContent = sub ?? '';
    if (!t) return;
    t.classList.add('show');
    if (this.toastTimer) clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => t.classList.remove('show'), 3200);
  }

  /** Clear every transient warning/banner (restart / respawn). The
   *  #warn line is recomputed from live state every update() so it
   *  clears itself when its condition ends — but a stale STALL
   *  survived three restarts in the v2 play-test because the solver
   *  kept the stall state (fixed by solver.reset() on respawn) and
   *  because the toast re-fires on every death in a respawn loop.
   *  Belt and braces: hide #warn, the ROTATE cue and the toast, and
   *  cancel the toast's auto-hide timer (a pending timer would
   *  otherwise hide a NEW toast early, or a re-fired toast would
   *  linger). Called from startRound() and respawnPlayer(). */
  clearTransient(): void {
    // Clear textContent as well as hiding (live finding 4): a hidden
    // banner's text persists in the DOM and reads as a stale banner.
    const w = this.el['warn'];
    if (w) {
      w.classList.remove('show');
      w.textContent = '';
    }
    const rc = this.el['rotateCue'];
    if (rc) {
      // Hide only — do NOT clear textContent: the ROTATE cue's text
      // is static HTML ("ROTATE — HOLD ↑ / PULL BACK"), not set by
      // JS, so clearing it would wipe the cue permanently.
      rc.classList.remove('show');
    }
    if (this.toastTimer) {
      clearTimeout(this.toastTimer);
      this.toastTimer = null;
    }
    this.el['toast']?.classList.remove('show');
    if (this.el['toastTitle']) this.el['toastTitle'].textContent = '';
    if (this.el['toastSub']) this.el['toastSub'].textContent = '';
  }

  /** ?debug=1 panel (v1.1 feature): FPS + sim dt + per-model load status +
   *  last 8 console.warn/error lines. main.ts owns the capture; this only
   *  renders the lines it is given. */
  setDebug(lines: string[]): void {
    const d = this.el['debug'];
    if (!d) return;
    d.classList.remove('hidden');
    d.textContent = lines.join('\n');
  }

  /** Camera-mode toast (C key / camBtn): Chase / Cockpit / Free orbit. */
  cameraToast(mode: number): void {
    const names = ['CHASE CAM', 'COCKPIT CAM', 'FREE ORBIT'];
    this.toast(
      names[mode] ?? 'CHASE CAM',
      mode === 2 ? 'Drag to orbit · pinch to zoom' : '',
    );
  }

  /** Sync the throttle slider + label to a 0..1 target (template syncThrUI). */
  syncThrottle(value01: number): void {
    const thr = document.getElementById('thr') as HTMLInputElement | null;
    const val = document.getElementById('thrVal');
    if (thr) thr.value = String(value01 * 100);
    if (val) val.textContent = Math.round(value01 * 100) + '%';
  }
}
