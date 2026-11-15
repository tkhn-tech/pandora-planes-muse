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
  carrier: string;
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
  carrier: 'ford',
};

const JET_DESC: Record<string, string> = {
  fa18: 'Stall 120 kt · 9 G · carrier-capable',
  f35: 'Stall 117 kt · 9 G · lift-fan VTOL 0-90°',
  f22: 'Stall 126 kt · 9.5 G · fastest turn',
  f15: 'Stall 120 kt · 9 G · highest top speed',
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

export class SandboxMenu {
  private readonly onStart: (cfg: SandboxConfig) => void;
  private cfg: SandboxConfig = structuredClone(DEFAULT_CONFIG);
  private wxPreset = 'clear';

  constructor(onStart: (cfg: SandboxConfig) => void) {
    this.onStart = onStart;
    this.wireSegments();
    this.wireSteppers();
    this.wireSliders();
    const startBtn = $('startBtn');
    startBtn?.addEventListener('click', () => this.onStart(this.read()));
    // Keep the menu's live state in sync while the user fiddles.
    this.applyLive(this.cfg);
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
    });
    this.segWire('segSkill', (d) => {
      this.cfg.enemySkill = (d['v'] as EnemySkill) ?? 'veteran';
    });
    this.segWire('segTraffic', (d) => {
      this.cfg.traffic = d['v'] === '1';
    });
    this.segWire('segWx', (d) => {
      this.wxPreset = d['v'] ?? 'clear';
      this.applyWxPreset();
    });
    this.segWire('segFail', (d) => {
      this.cfg.failure = (d['v'] as FailureScenario) ?? 'none';
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
    ) => {
      const root = $(id);
      if (!root) return;
      root.querySelectorAll('button').forEach((b) => {
        b.addEventListener('click', () => {
          set(
            clamp(get() + Number((b as HTMLElement).dataset['d'] ?? 0), min, max),
          );
          const val = $(valId);
          if (val) val.textContent = String(get());
        });
      });
    };
    wire(
      'stepEnemy',
      'stepEnemyVal',
      () => this.cfg.enemies,
      (v) => {
        this.cfg.enemies = v;
      },
      1,
      20,
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
  }

  /** Read the current menu state as a SandboxConfig. */
  read(): SandboxConfig {
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
    setText('stepEnemyVal', String(cfg.enemies));
    setText('stepTrafficVal', String(cfg.trafficCount));
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
    markSeg('segSkill', (b) => b.dataset['v'] === cfg.enemySkill);
    markSeg('segTraffic', (b) => b.dataset['v'] === (cfg.traffic ? '1' : '0'));
    markSeg('segFail', (b) => b.dataset['v'] === cfg.failure);
  }

  show(): void {
    $('startScreen')?.classList.remove('hidden');
  }

  hide(): void {
    $('startScreen')?.classList.add('hidden');
  }
}
