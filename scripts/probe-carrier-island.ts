// probe-carrier-island.ts — pass 13, worker E, Task 5: helicarriers
// 2/3 "render as brown inverted cones with green flat tops + white
// dotted waterfalls below them".
//
// Hypothesis to test (from the source, before running): that
// description matches the FLOATING ISLANDS exactly —
// PandoraWorldGenerator.buildIslands: an inverted brown rock cone
// (ConeGeometry, rotation.x = PI, rock 0x9a8a76 / emissive 0x6a5a48),
// a green flat-top cylinder (top 0x5a9a62), and a white/blue dotted
// waterfall Points stream below (0x9fd4ff), at y 260-520 m — the
// same altitude band as the carriers (350 m). Inactive carriers are
// hidden by deactivate() (group.visible = false), and carrier groups
// contain no cone geometry at all, so the carriers cannot render
// that shape. This probe either confirms the misidentification (all
// carrier gates pass + an island sits near a carrier 2/3 slot
// start) or finds a real carrier rendering defect (a gate fails).
//
// Method: build the real world (seed 1337, as live) and all three
// carriers with the real SHIELD GLB (as main.ts does). Traverse each
// carrier group (SHIELD meshes present? any ConeGeometry?), check
// deactivated carriers are invisible, traverse the island groups
// (do they match the reported description exactly?), and measure the
// distance from every island to every carrier slot start.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-island.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-carrier-island.mjs
//   && node /tmp/probe-carrier-island.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(
      type: string,
      init?: { lengthComputable?: boolean; loaded?: number; total?: number },
    ) {
      this.type = type;
      this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0;
      this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1;
    height = 1;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void {
      (this.listeners[type] ??= []).push(fn);
    }
    removeEventListener(): void {}
    set src(_v: string) {
      queueMicrotask(() => {
        for (const fn of this.listeners['load'] ?? []) fn();
        this.onload?.();
      });
    }
  };
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            createLinearGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            clearRect: () => {},
            drawImage: () => {},
            getImageData: () => ({ data: new Uint8ClampedArray(128 * 128 * 4) }),
            putImageData: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
    createElementNS: () => ({}),
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { PandoraWorldGenerator, ISLAND_LOOP_CLEARANCE_M } from '../src/world/PandoraWorldGenerator';
import {
  HelicarrierController,
  carrierLoop,
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import {
  generateHeights,
  heightfieldHeightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const gen = new PandoraWorldGenerator(scene, world);
  await gen.build(1337);

  // ---- Build all three carriers with the real GLB, as main.ts does ----
  const carriers = [0, 1, 2].map((i) => {
    const loop = carrierLoop(i);
    return new HelicarrierController(scene, world, {
      modelGlbUri: HELICARRIER_GLB,
      deckTopSourceY: 29.78,
      heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
      waypoints: loop.waypoints,
      startIndex: loop.startIndex,
    });
  });
  await Promise.all(carriers.map((c) => c.build()));
  carriers.forEach((c, i) => {
    const loop = carrierLoop(i);
    const wp0 = loop.waypoints[loop.startIndex]!;
    c.reset(
      loop.startPos.clone(),
      Math.atan2(wp0.x - loop.startPos.x, wp0.z - loop.startPos.z),
    );
  });

  // ---- 1. Carrier groups: SHIELD meshes present, no cones ----
  const SHIELD_NAMES = ['deck_top', 'duct_ring', 'rotor_disc'];
  for (let i = 0; i < 3; i++) {
    const c = carriers[i]!;
    const meshNames: string[] = [];
    let cones = 0;
    let meshes = 0;
    c.group.traverse((o) => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      meshes++;
      meshNames.push(m.name);
      if ((m.geometry as THREE.BufferGeometry)?.type === 'ConeGeometry') cones++;
    });
    const found = SHIELD_NAMES.filter((n) =>
      meshNames.some((mn) => mn.includes(n)),
    );
    console.log(
      `  carrier ${i}: modelReady=${c.modelReady} meshes=${meshes} SHIELD parts=[${found.join(',')}] cones=${cones}`,
    );
    check(
      `carrier ${i}: built from the SHIELD GLB (deck_top / duct_ring / rotor_disc meshes present)`,
      c.modelReady && found.length === SHIELD_NAMES.length,
      `modelReady=${c.modelReady} found=[${found.join(',')}]`,
    );
    check(
      `carrier ${i}: group contains NO cone geometry (cannot render as an inverted cone)`,
      cones === 0,
      `cones=${cones}`,
    );
  }

  // ---- 2. Inactive carriers are invisible (count = 1, as in a
  // 1-carrier round; carriers 2/3 deactivated like main.ts does) ----
  carriers[1]!.deactivate();
  carriers[2]!.deactivate();
  check(
    'count 1: carriers 2/3 deactivated AND invisible (group.visible false)',
    !carriers[1]!.active &&
      !carriers[2]!.active &&
      !carriers[1]!.group.visible &&
      !carriers[2]!.group.visible,
    `active=[${carriers.map((c) => c.active).join(',')}] visible=[${carriers.map((c) => c.group.visible).join(',')}]`,
  );
  // And with count 3 all three are active SHIELD ships (not cones).
  carriers[1]!.reset(carrierLoop(1).startPos.clone(), 0);
  carriers[2]!.reset(carrierLoop(2).startPos.clone(), 0);
  // reset() reactivates; group.visible follows active in reset.
  check(
    'count 3: all three carriers active and visible',
    carriers.every((c) => c.active && c.group.visible),
    `active=[${carriers.map((c) => c.active).join(',')}]`,
  );

  // ---- 3. Island groups match the reported description exactly ----
  const islands = gen.islandGroups;
  console.log(`  islands: ${islands.length} groups`);
  let matching = 0;
  const islandSummaries: string[] = [];
  for (const g of islands) {
    let cone = false;
    let coneInverted = false;
    let greenTop = false;
    let waterfall = false;
    let coneColor = '';
    g.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh && (m.geometry as THREE.BufferGeometry)?.type === 'ConeGeometry') {
        cone = true;
        if (Math.abs(m.rotation.x - Math.PI) < 0.01) coneInverted = true;
        const mat = m.material as THREE.MeshLambertMaterial;
        if (mat?.color) coneColor = `#${mat.color.getHexString()}`;
      }
      if (m.isMesh && (m.geometry as THREE.BufferGeometry)?.type === 'CylinderGeometry') {
        const mat = m.material as THREE.MeshLambertMaterial;
        if (mat?.color && mat.color.getHex() === 0x5a9a62) greenTop = true;
      }
      if ((o as THREE.Points).isPoints) waterfall = true;
    });
    const p = g.position;
    islandSummaries.push(
      `${g.name} at (${p.x.toFixed(0)}, ${p.y.toFixed(0)}, ${p.z.toFixed(0)}) cone=${cone} inverted=${coneInverted} greenTop=${greenTop} waterfall=${waterfall} rock=${coneColor}`,
    );
    if (cone && coneInverted && greenTop && waterfall) matching++;
  }
  for (const s of islandSummaries) console.log(`  ${s}`);
  check(
    'islands match the reported description exactly (inverted brown cone + green flat top + waterfall Points)',
    matching === islands.length && islands.length === 8,
    `${matching}/${islands.length} islands match`,
  );

  // ---- 4. Distance: islands vs carrier 2/3 slot starts ----
  // If an island sits near a carrier 2/3 start, at carrier altitude,
  // a viewer at the carrier's position sees a brown inverted cone
  // with a green top and a dotted waterfall exactly where carrier
  // 2/3 should be — the misidentification mechanism.
  for (const slot of [1, 2]) {
    const start = carrierLoop(slot).startPos;
    let best: { d: number; h: number; name: string } | null = null;
    for (const g of islands) {
      const d = Math.hypot(g.position.x - start.x, g.position.z - start.z);
      if (!best || d < best.d) {
        best = { d, h: g.position.y, name: g.name };
      }
    }
    console.log(
      `  slot ${slot} start (${start.x.toFixed(0)}, ${start.y.toFixed(0)}, ${start.z.toFixed(0)}): nearest island ${best!.name} ${best!.d.toFixed(0)} m away horizontally, island y=${best!.h.toFixed(0)} m (carrier y=350 m)`,
    );
  }
  // Also: nearest island to ANY carrier slot start, and to the
  // primary's start — for the record.
  let globalBest = Infinity;
  let globalWhat = '';
  for (const slot of [0, 1, 2]) {
    const start = carrierLoop(slot).startPos;
    for (const g of islands) {
      const d = Math.hypot(g.position.x - start.x, g.position.z - start.z);
      if (d < globalBest) {
        globalBest = d;
        globalWhat = `${g.name} to slot ${slot} start`;
      }
    }
  }
  console.log(`  nearest island to any carrier slot start: ${globalBest.toFixed(0)} m (${globalWhat})`);

  // ---- 5. Pass 14 (worker F, item 11): every island clears the
  // carrier LOOP, not just the slot starts. The carrier flies the
  // polyline start -> W0 -> ... -> W5 -> W0 (entry leg + hexagon);
  // an island centre must be at least ISLAND_LOOP_CLEARANCE_M from
  // every segment, and its nearest surface (centre minus the main
  // cone radius) at least 800 m out. Before the fix island-3 was
  // 270 m (centre) from the W0->W1 leg, so this section FAILS on
  // the old build and PASSES after the relocation.
  const loopPts = [CARRIER_START_POS, ...CARRIER_WAYPOINTS];
  const loopSegs: Array<[THREE.Vector3, THREE.Vector3]> = [];
  for (let s = 0; s < loopPts.length - 1; s++) loopSegs.push([loopPts[s]!, loopPts[s + 1]!]);
  loopSegs.push([loopPts[loopPts.length - 1]!, loopPts[1]!]);
  const segDist = (x: number, z: number): { d: number; seg: number } => {
    let best = Infinity;
    let bestSeg = -1;
    loopSegs.forEach(([a, b], i) => {
      const dx = b.x - a.x;
      const dz = b.z - a.z;
      const len2 = dx * dx + dz * dz;
      const t =
        len2 === 0
          ? 0
          : Math.max(0, Math.min(1, ((x - a.x) * dx + (z - a.z) * dz) / len2));
      const d = Math.hypot(x - (a.x + t * dx), z - (a.z + t * dz));
      if (d < best) {
        best = d;
        bestSeg = i;
      }
    });
    return { d: best, seg: bestSeg };
  };
  let minCentre = Infinity;
  let minEdge = Infinity;
  let minCentreWhat = '';
  for (const g of islands) {
    let R = 0;
    g.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh && (m.geometry as THREE.BufferGeometry)?.type === 'ConeGeometry') {
        const params = (m.geometry as THREE.ConeGeometry).parameters;
        if (params && params.radius > R) R = params.radius;
      }
    });
    const { d, seg } = segDist(g.position.x, g.position.z);
    console.log(
      `  ${g.name} at (${g.position.x.toFixed(0)}, ${g.position.z.toFixed(0)}) R=${R.toFixed(0)}m: nearest loop segment ${seg} at ${d.toFixed(0)} m centre (${(d - R).toFixed(0)} m to cone edge)`,
    );
    if (d < minCentre) {
      minCentre = d;
      minCentreWhat = `${g.name} to segment ${seg}`;
    }
    if (d - R < minEdge) minEdge = d - R;
  }
  console.log(
    `  nearest island centre to the carrier loop: ${minCentre.toFixed(0)} m (${minCentreWhat}); nearest cone edge: ${minEdge.toFixed(0)} m`,
  );
  check(
    'islands clear the carrier loop (centre >= 1000 m from every flown segment)',
    minCentre >= ISLAND_LOOP_CLEARANCE_M,
    `min centre=${minCentre.toFixed(0)} m (${minCentreWhat})`,
  );
  check(
    'islands clear the carrier loop (cone edge >= 800 m)',
    minEdge >= 800,
    `min edge=${minEdge.toFixed(0)} m`,
  );

  console.log(failures === 0 ? 'ALL CARRIER-ISLAND CHECKS PASSED' : `${failures} CARRIER-ISLAND CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
