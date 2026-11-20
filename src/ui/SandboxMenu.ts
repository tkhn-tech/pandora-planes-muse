// SandboxMenu — start-menu wiring + SandboxConfig. Ported from the v2
// template menu (pandora-template.html ~316–367 DOM, ~2210–2256 wiring).
// DOM ids are kept VERBATIM from the template — index.html must contain
// them (full list in the worker report).
//
// Ranges (template): enemies 1–20 (default 4), traffic 1–30 (default 2,
// on), spawn distance 400–3000 m (default 1200), time of day 0–24 h
// (default 10), wind 0–25 m/s (default 3), wind dir 0–360 deg (default
// 270), turbulence 0–3 (default 0). Failure scenarios: none, engine L/R,
// engine fire, hydraulic, gear jam, fuel leak — plus 'random', which picks
// a random scenario at a random 30–90 s (template segFailWhen 'random').
// Weather presets (segWx) map to the weather object: clear / cloud /
// storm / rain set wind + turbulence sliders and rain/clouds/fog.

export type EnemySkill = 'rookie' | 'veteran' | 'ace';

export type FailureScenario =
  | 'none'
  | 'engineL'
  | 'engineR'
  | 'engineFire'
  | 'hydraulic'
  | 'gearJam'
  | 'fuelLeak'
  | 'random';

/** Valid aircraft keys / failure values, for validating stored configs. */
const SPECS_KEYS = new Set(['fa18', 'f35a', 'f22', 'f15e']);
const FAILURE_VALUES = new Set([
  'none',
  'engineL',
  'engineR',
  'engineFire',
  'hydraulic',
  'gearJam',
  'fuelLeak',
  'random',
]);

export interface SandboxConfig {
  aircraftKey: string;
  enemies: number;
  enemySkill: EnemySkill;
  enemySpawnDistance: number;
  traffic: boolean;
  trafficCount: number;
  tod: number; // hours, 0–24
  weather: {
    windSpeed: number; // m/s
    windDir: number; // deg, direction the wind blows toward
    turbulence: number; // 0–3
    rain: boolean;
    fog: number; // fog near distance, m (preset-derived)
    clouds: boolean;
  };
  failure: FailureScenario;
  /** Failure timing: 'start' fires a named scenario at 3 s; 'random'
   *  fires it at a random 30–90 s. 'random' as the scenario picks a
   *  random scenario; the timing still comes from failWhen. */
  failWhen: 'start' | 'random';
  /** Spawn: Alpha / Bravo / Charlie land runway, or the helicarrier
   *  deck. Bravo/Charlie were respawn-picker-only (pass 12, worker
   *  D) — the setup menu offered only Alpha + deck, so they could
   *  never be a first spawn. */
  spawn: 'alpha' | 'bravo' | 'charlie' | 'carrier';
  /** Stick shaping (PC keyboard): invert + sensitivity exponent. */
  invertPitch: boolean;
  invertRoll: boolean;
  /** Sensitivity exponent gamma, 0.5–2.0 (1.0 = linear). Applied as
   *  out = sign(x) * |x|^gamma to the keyboard stick. */
  pitchSens: number;
  rollSens: number;
  /** Number of SHIELD helicarriers travelling around (issue 5):
   *  0–3, default 1. Each gets its own loop path (a scaled hexagon,
   *  so the loops are concentric and never overlap). 0 means no
   *  carrier at all — the carrier spawn option is hidden and falls
   *  back to Alpha. */
  helicarriers: number;
}

export const DEFAULT_CONFIG: SandboxConfig = {
  aircraftKey: 'fa18',
  enemies: 4,
  enemySkill: 'veteran',
  enemySpawnDistance: 1200,
  traffic: true,
  trafficCount: 2,
  tod: 10,
  weather: {
    windSpeed: 3,
    windDir: 270,
    turbulence: 0,
    rain: false,
    fog: 900,
    clouds: false,
  },
  failure: 'none',
  failWhen: 'start',
  spawn: 'alpha',
  invertPitch: false,
  invertRoll: false,
  pitchSens: 1,
  rollSens: 1,
  helicarriers: 1,
};

// Stall speeds at combat mass (spec.stallSpeed, m/s -> kt): F-18 66 m/s
// = 128 kt, F-35A 68 = 132 kt, F-22 54 = 105 kt, F-15E 62 = 120 kt.
const JET_DESC: Record<string, string> = {
  fa18: 'Stall 128 kt · 9 G · carrier-capable',
  f35a: 'Stall 132 kt · 9 G · lift-fan VTOL 0-90°',
  f22: 'Stall 105 kt · 9.5 G · fastest turn',
  f15e: 'Stall 120 kt · 9 G · highest top speed',
};

/** Weather preset -> slider values + flags (template segWx click handler). */
const WX_PRESET: Record<
  string,
  { wind: number; turb: number; rain: boolean; clouds: boolean; fog: number }
> = {
  clear: { wind: 3, turb: 0, rain: false, clouds: false, fog: 900 },
  cloud: { wind: 6, turb: 0.4, rain: false, clouds: true, fog: 700 },
  storm: { wind: 18, turb: 2.2, rain: true, clouds: true, fog: 400 },
  rain: { wind: 9, turb: 1.0, rain: true, clouds: true, fog: 500 },
};

function $(id: string): HTMLElement | null {
  return document.getElementById(id);
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

/** localStorage key for the persisted sandbox setup (whole config). */
const STORAGE_KEY = 'pandora-planes-config-v1';

export class SandboxMenu {
  private readonly onStart: (cfg: SandboxConfig) => void;
  private cfg: SandboxConfig = structuredClone(DEFAULT_CONFIG);
  private wxPreset = 'clear';
  /** In-game re-setup mode: the start button becomes "Apply & Restart"
   *  and routes to onApplyRestart instead of onStart (which builds the
   *  carrier — build() must never run twice in one session). */
  private mode: 'preflight' | 'ingame' = 'preflight';
  private onApplyRestart: ((cfg: SandboxConfig) => void) | null = null;
  private onResume: (() => void) | null = null;

  constructor(onStart: (cfg: SandboxConfig) => void) {
    this.onStart = onStart;
    this.loadStored();
    this.wireSegments();
    this.wireSteppers();
    this.wireSliders();
    const startBtn = $('startBtn');
    startBtn?.addEventListener('click', () => {
      if (this.mode === 'ingame' && this.onApplyRestart) {
        this.onApplyRestart(this.read());
      } else {
        this.onStart(this.read());
      }
    });
    $('menuResumeBtn')?.addEventListener('click', () => {
      this.onResume?.();
    });
    // Keep the menu's live state in sync while the user fiddles.
    this.applyLive(this.cfg);
  }

  /** Switch between the pre-flight menu and the in-game re-setup drawer.
   *  In-game the button reads "Apply & Restart" and a Resume button
   *  appears; the carrier is never rebuilt on this path. */
  setMode(
    mode: 'preflight' | 'ingame',
    onApplyRestart?: (cfg: SandboxConfig) => void,
    onResume?: () => void,
  ): void {
    this.mode = mode;
    this.onApplyRestart = onApplyRestart ?? null;
    this.onResume = onResume ?? null;
    const startBtn = $('startBtn') as HTMLButtonElement | null;
    if (startBtn) {
      startBtn.textContent =
        mode === 'ingame' ? 'Apply & Restart 🔄' : 'Take Off 🛫';
      startBtn.disabled = false;
    }
    $('menuResumeBtn')?.classList.toggle('hidden', mode !== 'ingame');
  }

  /** Persist the whole setup so the next session opens where this one
   *  left off (controls + spawn + everything else). */
  private save(): void {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.cfg));
    } catch {
      // Private mode / quota — the in-memory config still works.
    }
  }

  /** Load a stored setup over the defaults, validating every field so a
   *  stale or corrupt entry can never break the menu. */
  private loadStored(): void {
    let raw: string | null = null;
    try {
      raw = localStorage.getItem(STORAGE_KEY);
    } catch {
      return;
    }
    if (!raw) return;
    try {
      const s = JSON.parse(raw) as Partial<SandboxConfig>;
      const c = this.cfg;
      if (typeof s.aircraftKey === 'string' && SPECS_KEYS.has(s.aircraftKey)) {
        c.aircraftKey = s.aircraftKey;
      }
      if (typeof s.enemies === 'number') c.enemies = clamp(Math.round(s.enemies), 0, 20);
      if (s.enemySkill === 'rookie' || s.enemySkill === 'veteran' || s.enemySkill === 'ace') {
        c.enemySkill = s.enemySkill;
      }
      if (typeof s.enemySpawnDistance === 'number') {
        c.enemySpawnDistance = clamp(s.enemySpawnDistance, 400, 3000);
      }
      if (typeof s.traffic === 'boolean') c.traffic = s.traffic;
      if (typeof s.trafficCount === 'number') {
        c.trafficCount = clamp(Math.round(s.trafficCount), 1, 30);
      }
      if (typeof s.tod === 'number') c.tod = clamp(s.tod, 0, 24);
      if (s.weather && typeof s.weather === 'object') {
        const w = s.weather as Partial<SandboxConfig['weather']>;
        if (typeof w.windSpeed === 'number') c.weather.windSpeed = clamp(w.windSpeed, 0, 25);
        if (typeof w.windDir === 'number') c.weather.windDir = clamp(w.windDir, 0, 360);
        if (typeof w.turbulence === 'number') c.weather.turbulence = clamp(w.turbulence, 0, 3);
        if (typeof w.rain === 'boolean') c.weather.rain = w.rain;
        if (typeof w.fog === 'number') c.weather.fog = w.fog;
        if (typeof w.clouds === 'boolean') c.weather.clouds = w.clouds;
      }
      if (typeof s.failure === 'string' && FAILURE_VALUES.has(s.failure)) {
        c.failure = s.failure as FailureScenario;
      }
      if (s.failWhen === 'start' || s.failWhen === 'random') c.failWhen = s.failWhen;
      if (s.spawn === 'alpha' || s.spawn === 'bravo' || s.spawn === 'charlie' || s.spawn === 'carrier') c.spawn = s.spawn;
      if (typeof s.helicarriers === 'number') {
        c.helicarriers = clamp(Math.round(s.helicarriers), 0, 3);
      }
      if (typeof s.invertPitch === 'boolean') c.invertPitch = s.invertPitch;
      if (typeof s.invertRoll === 'boolean') c.invertRoll = s.invertRoll;
      if (typeof s.pitchSens === 'number') c.pitchSens = clamp(s.pitchSens, 0.5, 2);
      if (typeof s.rollSens === 'number') c.rollSens = clamp(s.rollSens, 0.5, 2);
    } catch {
      // Corrupt JSON — keep the defaults.
    }
  }

  private segWire(
    id: string,
    fn: (data: Record<string, string>) => void,
  ): void {
    const root = $(id);
    if (!root) return;
    root.querySelectorAll('button').forEach((b) => {
      b.addEventListener('click', () => {
        root
          .querySelectorAll('button')
          .forEach((x) => x.classList.remove('on'));
        b.classList.add('on');
        fn((b as HTMLElement).dataset as Record<string, string>);
      });
    });
  }

  private wireSegments(): void {
    this.segWire('jetPicker', (d) => {
      this.cfg.aircraftKey = d['k'] ?? 'fa18';
      const spec = $('jetSpec');
      if (spec) {
        spec.textContent =
          JET_DESC[this.cfg.aircraftKey] ?? JET_DESC['fa18']!;
      }
      this.save();
    });
    this.segWire('segSpawn', (d) => {
      const v = d['v'];
      this.cfg.spawn =
        v === 'bravo' || v === 'charlie' || v === 'carrier' ? v : 'alpha';
      this.save();
    });
    this.segWire('segSkill', (d) => {
      this.cfg.enemySkill = (d['v'] as EnemySkill) ?? 'veteran';
      this.save();
    });
    this.segWire('segTraffic', (d) => {
      this.cfg.traffic = d['v'] === '1';
      this.save();
    });
    this.segWire('segWx', (d) => {
      this.wxPreset = d['v'] ?? 'clear';
      this.applyWxPreset();
      this.save();
    });
    this.segWire('segFail', (d) => {
      this.cfg.failure = (d['v'] as FailureScenario) ?? 'none';
      this.save();
    });
    this.segWire('segFailWhen', (d) => {
      this.cfg.failWhen = d['v'] === 'random' ? 'random' : 'start';
      this.save();
    });
    this.segWire('segInvPitch', (d) => {
      this.cfg.invertPitch = d['v'] === '1';
      this.save();
    });
    this.segWire('segInvRoll', (d) => {
      this.cfg.invertRoll = d['v'] === '1';
      this.save();
    });
  }

  private applyWxPreset(): void {
    const p = WX_PRESET[this.wxPreset] ?? WX_PRESET['clear']!;
    this.cfg.weather.windSpeed = p.wind;
    this.cfg.weather.turbulence = p.turb;
    this.cfg.weather.rain = p.rain;
    this.cfg.weather.clouds = p.clouds;
    this.cfg.weather.fog = p.fog;
    const windSl = $('windSl') as HTMLInputElement | null;
    const windVal = $('windVal');
    const turbSl = $('turbSl') as HTMLInputElement | null;
    const turbVal = $('turbVal');
    if (windSl) windSl.value = String(p.wind);
    if (windVal) windVal.textContent = p.wind + ' m/s';
    if (turbSl) turbSl.value = String(p.turb);
    if (turbVal) turbVal.textContent = p.turb.toFixed(1);
  }

  private wireSteppers(): void {
    const wire = (
      id: string,
      valId: string,
      get: () => number,
      set: (v: number) => void,
      min: number,
      max: number,
      fmt: (v: number) => string = (v) => String(v),
    ) => {
      const root = $(id);
      if (!root) return;
      root.querySelectorAll('button').forEach((b) => {
        b.addEventListener('click', () => {
          set(
            clamp(get() + Number((b as HTMLElement).dataset['d'] ?? 0), min, max),
          );
          const val = $(valId);
          if (val) val.textContent = fmt(get());
          this.save();
        });
      });
    };
    // Enemies 0 = Off (first-class choice: no enemies spawn, no AI
    // combat). The stepper remembers the count, so stepping back up
    // restores it — no separate toggle needed.
    wire(
      'stepEnemy',
      'stepEnemyVal',
      () => this.cfg.enemies,
      (v) => {
        this.cfg.enemies = v;
      },
      0,
      20,
      (v) => (v === 0 ? 'Off' : String(v)),
    );
    wire(
      'stepTraffic',
      'stepTrafficVal',
      () => this.cfg.trafficCount,
      (v) => {
        this.cfg.trafficCount = v;
      },
      1,
      30,
    );
    // Helicarriers 0–3 (issue 5): each carrier gets its own loop.
    // 0 means no carrier — the carrier spawn option is hidden and
    // falls back to Alpha (a deck spawn with no deck would strand
    // the jet in mid-air).
    wire(
      'stepHeli',
      'stepHeliVal',
      () => this.cfg.helicarriers,
      (v) => {
        this.cfg.helicarriers = v;
        this.syncSpawnSeg();
      },
      0,
      3,
    );
  }

  /** Carrier spawn availability follows the helicarrier count: with
   *  0 carriers the deck button is disabled and a carrier spawn falls
   *  back to Alpha. Called from the helicarrier stepper and applyLive. */
  private syncSpawnSeg(): void {
    const root = $('segSpawn');
    if (!root) return;
    const carrierBtn = root.querySelector(
      'button[data-v="carrier"]',
    ) as HTMLButtonElement | null;
    if (!carrierBtn) return;
    if (this.cfg.helicarriers === 0) {
      carrierBtn.disabled = true;
      if (this.cfg.spawn === 'carrier') {
        this.cfg.spawn = 'alpha';
        root.querySelectorAll('button').forEach((b) => {
          b.classList.toggle('on', (b as HTMLElement).dataset['v'] === 'alpha');
        });
      }
    } else {
      carrierBtn.disabled = false;
    }
  }

  private wireSliders(): void {
    const wire = (
      id: string,
      valId: string,
      fmt: (v: number) => string,
      get: () => number,
      set: (v: number) => void,
    ) => {
      const input = $(id) as HTMLInputElement | null;
      if (!input) return;
      input.addEventListener('input', () => {
        set(Number(input.value));
        const val = $(valId);
        if (val) val.textContent = fmt(get());
        this.save();
      });
    };
    wire(
      'spawnSl',
      'spawnVal',
      (v) => v + ' m',
      () => this.cfg.enemySpawnDistance,
      (v) => {
        this.cfg.enemySpawnDistance = v;
      },
    );
    wire(
      'todSl',
      'todVal',
      (v) => {
        const h = Math.floor(v);
        const m = Math.round((v - h) * 60);
        return (
          String(h).padStart(2, '0') + ':' + String(m).padStart(2, '0')
        );
      },
      () => this.cfg.tod,
      (v) => {
        this.cfg.tod = v;
      },
    );
    wire(
      'windSl',
      'windVal',
      (v) => v + ' m/s',
      () => this.cfg.weather.windSpeed,
      (v) => {
        this.cfg.weather.windSpeed = v;
      },
    );
    wire(
      'windDirSl',
      'windDirVal',
      (v) => v + '°',
      () => this.cfg.weather.windDir,
      (v) => {
        this.cfg.weather.windDir = v;
      },
    );
    wire(
      'turbSl',
      'turbVal',
      (v) => v.toFixed(1),
      () => this.cfg.weather.turbulence,
      (v) => {
        this.cfg.weather.turbulence = v;
      },
    );
    // Stick sensitivity exponents, 0.5–2.0 (1.0 = linear).
    wire(
      'pitchSensSl',
      'pitchSensVal',
      (v) => v.toFixed(2) + '×',
      () => this.cfg.pitchSens,
      (v) => {
        this.cfg.pitchSens = v;
      },
    );
    wire(
      'rollSensSl',
      'rollSensVal',
      (v) => v.toFixed(2) + '×',
      () => this.cfg.rollSens,
      (v) => {
        this.cfg.rollSens = v;
      },
    );
  }

  /** Read the current menu state as a SandboxConfig. */
  read(): SandboxConfig {
    this.save();
    return structuredClone(this.cfg);
  }

  /** Push a config back into the menu DOM (sliders, steppers, segments). */
  applyLive(cfg: SandboxConfig): void {
    this.cfg = structuredClone(cfg);
    const setVal = (id: string, v: string) => {
      const e = $(id) as HTMLInputElement | null;
      if (e) e.value = v;
    };
    const setText = (id: string, v: string) => {
      const e = $(id);
      if (e) e.textContent = v;
    };
    setText('stepEnemyVal', cfg.enemies === 0 ? 'Off' : String(cfg.enemies));
    setText('stepTrafficVal', String(cfg.trafficCount));
    setText('stepHeliVal', String(cfg.helicarriers));
    setVal('spawnSl', String(cfg.enemySpawnDistance));
    setText('spawnVal', cfg.enemySpawnDistance + ' m');
    setVal('todSl', String(cfg.tod));
    const h = Math.floor(cfg.tod);
    const m = Math.round((cfg.tod - h) * 60);
    setText(
      'todVal',
      String(h).padStart(2, '0') + ':' + String(m).padStart(2, '0'),
    );
    setVal('windSl', String(cfg.weather.windSpeed));
    setText('windVal', cfg.weather.windSpeed + ' m/s');
    setVal('windDirSl', String(cfg.weather.windDir));
    setText('windDirVal', cfg.weather.windDir + '°');
    setVal('turbSl', String(cfg.weather.turbulence));
    setText('turbVal', cfg.weather.turbulence.toFixed(1));
    setVal('pitchSensSl', String(cfg.pitchSens));
    setText('pitchSensVal', cfg.pitchSens.toFixed(2) + '×');
    setVal('rollSensSl', String(cfg.rollSens));
    setText('rollSensVal', cfg.rollSens.toFixed(2) + '×');
    setText('jetSpec', JET_DESC[cfg.aircraftKey] ?? '');
    // Segment 'on' states.
    const markSeg = (id: string, match: (b: HTMLElement) => boolean) => {
      const root = $(id);
      if (!root) return;
      root.querySelectorAll('button').forEach((b) => {
        b.classList.toggle('on', match(b as HTMLElement));
      });
    };
    markSeg('jetPicker', (b) => b.dataset['k'] === cfg.aircraftKey);
    markSeg('segSpawn', (b) => b.dataset['v'] === cfg.spawn);
    this.syncSpawnSeg();
    markSeg('segSkill', (b) => b.dataset['v'] === cfg.enemySkill);
    markSeg('segTraffic', (b) => b.dataset['v'] === (cfg.traffic ? '1' : '0'));
    markSeg('segFail', (b) => b.dataset['v'] === cfg.failure);
    markSeg('segFailWhen', (b) => b.dataset['v'] === cfg.failWhen);
    markSeg('segInvPitch', (b) => b.dataset['v'] === (cfg.invertPitch ? '1' : '0'));
    markSeg('segInvRoll', (b) => b.dataset['v'] === (cfg.invertRoll ? '1' : '0'));
  }

  show(): void {
    $('startScreen')?.classList.remove('hidden');
  }

  hide(): void {
    $('startScreen')?.classList.add('hidden');
  }
}
