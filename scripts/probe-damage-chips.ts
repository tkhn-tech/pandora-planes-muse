// probe-damage-chips.ts — HUD per-zone damage chips (Issue 5).
//
// The HUD is DOM (FlightHUD.update reads the aircraft's damage
// entity and writes chip classes / bar widths / fire labels). Node
// has no DOM, so this driver stubs a minimal document: getElementById
// returns fake elements (textContent / className / querySelector for
// the chip's <b> label and .bar i fill / style / classList.toggle /
// getContext -> null so the minimap is skipped). Checks:
//   1. Undamaged: all four chips 'dmg ok', bars 100%.
//   2. Wing hurt (one wing at ~62%): WING 'dmg hurt', bar ~62%.
//   3. Engine destroyed (one engine at 0): ENG 'dmg dead', bar 0%.
//   4. Fuel fire (fireKind/fireZone set): FUEL chip 'fire', label
//      '🔥 FUEL' — the chips say WHERE the fire is, not just that
//      there is one.
//   5. Fuselage hurt: FUSE 'dmg hurt'.
//   6. F-35A single-engine: ENG reads the one `engine` zone (not
//      engineL/engineR) — destroying it reads 0%.
//
// Usage: node_modules/.bin/esbuild scripts/probe-damage-chips.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-damage-chips.mjs
//   && node /tmp/probe-damage-chips.mjs

// ---- Minimal DOM stub (must be set before new FlightHUD()). ----
class FakeEl {
  textContent = '';
  className = '';
  style: Record<string, string> = {};
  parentElement: FakeEl | null = null;
  private readonly kids = new Map<string, FakeEl>();
  classList = {
    toggle: (_c: string, _f?: boolean): void => {},
    contains: (_c: string): boolean => false,
    add: (_c: string): void => {},
    remove: (_c: string): void => {},
  };
  getContext(_kind: string): null {
    return null;
  }
  querySelector(sel: string): FakeEl | null {
    if (!this.kids.has(sel)) this.kids.set(sel, new FakeEl());
    return this.kids.get(sel) ?? null;
  }
}
const els = new Map<string, FakeEl>();
(globalThis as unknown as { document?: unknown }).document = {
  getElementById(id: string): FakeEl {
    if (!els.has(id)) {
      const e = new FakeEl();
      e.parentElement = new FakeEl();
      els.set(id, e);
    }
    return els.get(id)!;
  },
};

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FlightHUD } from '../src/ui/FlightHUD';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail: string): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name} — ${detail}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({
    seed: 1337, chunkX: 0, chunkZ: 0,
    size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++)
    for (let ix = 0; ix < res; ix++)
      hfHeights[ix * res + iz] = heights[iz * res + ix];
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    ).setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0),
    terrainBody,
  );
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const groundH = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  const hud = new FlightHUD();
  const game = { score: 0, kills: 0, traps: 0, elapsed: 0, lockTarget: null };
  const chip = (id: string): { cls: string; label: string; bar: string } => {
    const e = els.get(id)!;
    const lbl = e.querySelector('b');
    const fill = e.querySelector('.bar i');
    return {
      cls: e.className,
      label: lbl ? lbl.textContent : '',
      bar: fill ? fill.style.width ?? '' : '',
    };
  };
  const makeJet = (key: string): Aircraft => {
    // isPlayer=false: a player jet is spawn-protected after resetAt,
    // and applyDamage returns early while protected — the damage
    // would never register. The chip logic reads damageEntity, which
    // is identical for any aircraft.
    const ac = new Aircraft({
      scene, world, spec: SPECS[key]!, isPlayer: false,
      modelKey: SPECS[key]!.modelKey, events: {},
    });
    ac.terrainHeightAt = groundH;
    ac.resetAt(0, 600, -900, 90);
    ac.setAirspeed(100);
    return ac;
  };

  // 1. Undamaged.
  {
    const ac = makeJet('fa18');
    hud.update(ac, game);
    const f = chip('dmgFuse'); const w = chip('dmgWing');
    const e = chip('dmgEng'); const fu = chip('dmgFuel');
    check(
      'damage chips: undamaged jet reads all ok at 100%',
      f.cls === 'dmg ok' && w.cls === 'dmg ok' && e.cls === 'dmg ok' && fu.cls === 'dmg ok' &&
        f.bar === '100%' && w.bar === '100%' && e.bar === '100%' && fu.bar === '100%',
      `FUSE ${f.cls} ${f.bar} · WING ${w.cls} ${w.bar} · ENG ${e.cls} ${e.bar} · FUEL ${fu.cls} ${fu.bar}`,
    );
    ac.dispose();
  }
  // 2. Wing hurt (~62% left on wingL).
  {
    const ac = makeJet('fa18');
    ac.applyDamage(8, 'wingL', true); // entity zone mult is 1 for a
    // per-side zone (ZONE_MULT only has the generic 'wing'/'engine'),
    // so 8 dmg -> 17/25 = 68%
    hud.update(ac, game);
    const w = chip('dmgWing'); const f = chip('dmgFuse');
    check(
      'damage chips: one wing hurt reads WING hurt, others ok',
      w.cls === 'dmg hurt' && w.bar === '68%' && f.cls === 'dmg ok',
      `WING ${w.cls} ${w.bar} · FUSE ${f.cls}`,
    );
    ac.dispose();
  }
  // 3. Engine destroyed (engineL to 0).
  {
    const ac = makeJet('fa18');
    ac.applyDamage(30, 'engineL', true); // engine x1.5 -> 45 dmg, 0/20
    hud.update(ac, game);
    const e = chip('dmgEng');
    check(
      'damage chips: one engine destroyed reads ENG dead at 0%',
      e.cls.includes('dead') && e.bar === '0%',
      `ENG ${e.cls} ${e.bar}`,
    );
    ac.dispose();
  }
  // 4. Fuel fire: the chip says WHERE the fire is.
  {
    const ac = makeJet('fa18');
    ac.fireKind = 'fuel';
    ac.fireZone = 'fuel';
    hud.update(ac, game);
    const fu = chip('dmgFuel');
    check(
      'damage chips: fuel fire reads FUEL fire with flame label',
      fu.cls.includes('fire') && fu.label === '🔥 FUEL',
      `FUEL ${fu.cls} label=${fu.label}`,
    );
    ac.dispose();
  }
  // 5. Fuselage hurt.
  {
    const ac = makeJet('fa18');
    ac.applyDamage(15, 'fuselage', true); // 25/40 = 62.5% -> rounds to 63%
    hud.update(ac, game);
    const f = chip('dmgFuse');
    check(
      'damage chips: fuselage hurt reads FUSE hurt',
      f.cls === 'dmg hurt' && f.bar === '63%',
      `FUSE ${f.cls} ${f.bar}`,
    );
    ac.dispose();
  }
  // 6. F-35A single-engine: ENG reads the one `engine` zone.
  {
    const ac = makeJet('f35a');
    ac.applyDamage(30, 'engine', true); // engine x1.5 -> 45 dmg, 0/25
    hud.update(ac, game);
    const e = chip('dmgEng');
    check(
      'damage chips: F-35A single engine destroyed reads ENG dead at 0%',
      e.cls.includes('dead') && e.bar === '0%',
      `ENG ${e.cls} ${e.bar}`,
    );
    ac.dispose();
  }

  console.log(failures === 0 ? 'ALL PROBE CHECKS PASSED' : `${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => { console.error(e); process.exit(2); });
