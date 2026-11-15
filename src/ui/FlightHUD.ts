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
}

export interface MinimapEntities {
  player: Aircraft | null;
  enemies: Aircraft[];
  traffic: Aircraft[];
  carrier: { alive: boolean; pos: THREE.Vector3; yaw: number } | null;
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

export class FlightHUD {
  private readonly el: Record<string, HTMLElement | null> = {};
  private readonly mini: CanvasRenderingContext2D | null;
  private toastTimer: ReturnType<typeof setTimeout> | null = null;

  constructor() {
    const ids = [
      'hudJet', 'hudSpd', 'hudAlt', 'hudVsi', 'hudHdg', 'hudAoA', 'hudG',
      'hudFuel', 'hudWpn', 'hudScore', 'hudVec', 'vecPill', 'vecRow',
      'miniKills', 'miniTraps', 'miniTime', 'warn', 'lockInd', 'pipper',
      'toast', 'toastTitle', 'toastSub', 'debug', 'soundBtn',
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
    set('hudSpd', String(Math.round(ac.ias * MS2KT)));
    set('hudAlt', String(Math.round(Math.max(0, ac.pos.y) * M2FT)));
    set('hudVsi', String(Math.round(ac.vsi * M2FT * 60)));
    const hdg = ((Math.atan2(ac.fwd().x, ac.fwd().z) * R2D) % 360 + 360) % 360;
    set('hudHdg', String(Math.round(hdg)).padStart(3, '0') + '°');
    set('hudAoA', Math.round(ac.aoa) + '°');
    set('hudG', ac.gload.toFixed(1));
    set('hudFuel', Math.round(ac.fuel * 100) + '%');
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

    // Stall + low-fuel warnings.
    const w = this.el['warn'];
    if (w) {
      if (ac.stalled && ac.alive) {
        w.textContent = '⚠ STALL — NOSE DOWN, FULL POWER';
        w.classList.add('show');
      } else if (ac.fuel < 15 && ac.alive) {
        w.textContent = '⚠ LOW FUEL';
        w.classList.add('show');
      } else {
        w.classList.remove('show');
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

    // Thrust-vector readout (F-35A only).
    this.el['vecPill']?.classList.toggle('hidden', !ac.spec.vtol);
    if (ac.spec.vtol) set('hudVec', Math.round(ac.vector) + '°');
  }

  /** Minimap (144 x 144 canvas): green runways, blue carrier, grey traffic,
   *  red enemy diamonds, white player triangle. Template drawMinimap. */
  drawMinimap(ctx: CanvasRenderingContext2D, e: MinimapEntities): void {
    const W = 144;
    const S = W / 6200;
    const X = (x: number) => (x + e.originShift.x + 3100) * S;
    const Z = (z: number) => (z + e.originShift.z + 3100) * S;
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
    if (e.carrier?.alive) {
      ctx.fillStyle = '#6ab8ff';
      ctx.save();
      ctx.translate(X(e.carrier.pos.x), Z(e.carrier.pos.z));
      ctx.rotate(e.carrier.yaw);
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
