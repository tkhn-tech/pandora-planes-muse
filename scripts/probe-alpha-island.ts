// probe-alpha-island.ts — pass 15, worker F, item 12: brown disc/cone
// at the edge of Alpha runway in several spawn views.
//
// The v4.9 live check saw a brown disc/cone sitting at the edge of
// Alpha runway. Candidates: a floating island (inverted brown rock
// cone + green flat top, y 260-520 m) seen in the spawn view, or a
// scorch disc / debris from an earlier crash at the repeat crash
// point east of Alpha. This probe measures, for the REAL world
// (seed 1337) and the REAL Alpha spawn chase view (camera 12 m
// behind / 4 m above the jet, looking along the runway heading):
//   - plan distance from every island centre to every runway strip
//     rectangle (and to the plateau-top rectangle);
//   - whether each island projects in-frame in the spawn view
//     (portrait 1080x2400 AND landscape 2400x1080, fov 62 — the
//     live camera), its angular offset from the runway centreline,
//     and its angular size;
//   - whether each island sits in a runway's extended centreline
//     corridor (+/-500 m across, 0-4000 m past each strip end).
// An island is "at the runway edge" if it is within 1500 m plan
// of a strip, inside a runway's extended centreline corridor, or
// within 35 deg of a runway's centreline as seen from that runway's
// spawn chase camera (in-frame near the runway's vanishing point at
// horizon height in the landscape/desktop spawn view — island-4 was
// 28.6 deg off Alpha's centreline, island-6 25.6 deg, island-3
// 32.2 deg). The fix (PandoraWorldGenerator.clearIslandPosition)
// must make all three checks pass; this probe FAILS on the pre-fix
// tree and PASSES after.
//
// Usage: node_modules/.bin/esbuild scripts/probe-alpha-island.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-alpha-island.mjs
//   && node /tmp/probe-alpha-island.mjs
//   (delete /tmp/probe-alpha-island.mjs after running)

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
            beginPath: () => {},
            moveTo: () => {},
            lineTo: () => {},
            closePath: () => {},
            fill: () => {},
            arc: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
    createElementNS: () => ({}),
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { PandoraWorldGenerator } from '../src/world/PandoraWorldGenerator';
import { RUNWAY_TABLE, PLATEAU_TOP_ALONG, PLATEAU_TOP_ACROSS } from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const D2R = Math.PI / 180;

/** Plan distance from (x,z) to a runway's strip rectangle. */
function distToStrip(x: number, z: number, r: (typeof RUNWAY_TABLE)[number]): number {
  const dx = x - r.x;
  const dz = z - r.z;
  const sh = Math.sin(r.hdg * D2R);
  const ch = Math.cos(r.hdg * D2R);
  const lx = dx * sh + dz * ch;
  const lz = dx * ch - dz * sh;
  const ex = Math.max(0, Math.abs(lx) - r.len / 2);
  const ez = Math.max(0, Math.abs(lz) - r.width / 2);
  return Math.hypot(ex, ez);
}

/** Plan distance to the runway's plateau-top rectangle. */
function distToPlateau(x: number, z: number, r: (typeof RUNWAY_TABLE)[number]): number {
  const dx = x - r.x;
  const dz = z - r.z;
  const sh = Math.sin(r.hdg * D2R);
  const ch = Math.cos(r.hdg * D2R);
  const lx = dx * sh + dz * ch;
  const lz = dx * ch - dz * sh;
  const ex = Math.max(0, Math.abs(lx) - (r.len / 2 + PLATEAU_TOP_ALONG));
  const ez = Math.max(0, Math.abs(lz) - (r.width / 2 + PLATEAU_TOP_ACROSS));
  return Math.hypot(ex, ez);
}

/** Is (x,z) in the runway's extended centreline corridor: within
 *  500 m across, 0-4000 m past either strip end along the heading. */
function inCorridor(x: number, z: number, r: (typeof RUNWAY_TABLE)[number]): boolean {
  const dx = x - r.x;
  const dz = z - r.z;
  const sh = Math.sin(r.hdg * D2R);
  const ch = Math.cos(r.hdg * D2R);
  const lx = dx * sh + dz * ch;
  const lz = dx * ch - dz * sh;
  return Math.abs(lx) > r.len / 2 && Math.abs(lx) <= r.len / 2 + 4000 && Math.abs(lz) <= 500;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const gen = new PandoraWorldGenerator(scene, world);
  await gen.build(1337);

  const alpha = RUNWAY_TABLE[0]!;
  const fwd = new THREE.Vector3(Math.sin(alpha.hdg * D2R), 0, Math.cos(alpha.hdg * D2R));
  const camPos = new THREE.Vector3(alpha.x - fwd.x * 12, alpha.elev + 4, alpha.z - fwd.z * 12);
  const lookAt = new THREE.Vector3(alpha.x + fwd.x * 500, alpha.elev + 50, alpha.z + fwd.z * 500);
  console.log(`Alpha spawn chase cam at (${camPos.x.toFixed(0)}, ${camPos.y.toFixed(0)}, ${camPos.z.toFixed(0)}) looking along hdg ${alpha.hdg} to (${lookAt.x.toFixed(0)}, ${lookAt.y.toFixed(0)}, ${lookAt.z.toFixed(0)})`);

  let minStrip = Infinity;
  let minStripWhat = '';
  let worstView = '';
  let worstViewDeg = Infinity;
  for (const g of gen.islandGroups) {
    const p = g.position;
    let R = 0;
    g.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh && (m.geometry as THREE.BufferGeometry)?.type === 'ConeGeometry') {
        const params = (m.geometry as THREE.ConeGeometry).parameters;
        if (params && params.radius > R) R = params.radius;
      }
    });
    const strips = RUNWAY_TABLE.map((r) => ({ name: r.name, d: distToStrip(p.x, p.z, r), dp: distToPlateau(p.x, p.z, r) }));
    const nearest = strips.reduce((a, b) => (a.d < b.d ? a : b));
    if (nearest.d < minStrip) {
      minStrip = nearest.d;
      minStripWhat = `${g.name} to ${nearest.name} strip`;
    }
    const corridors = RUNWAY_TABLE.filter((r) => inCorridor(p.x, p.z, r)).map((r) => r.name);
    // Project the island centre + cone tip into the spawn view, both aspects.
    const views: string[] = [];
    let minOffDeg = Infinity;
    for (const [w, h, label] of [
      [1080, 2400, 'portrait'],
      [2400, 1080, 'landscape'],
    ] as const) {
      const cam = new THREE.PerspectiveCamera(62, w / h, 0.5, 20000);
      cam.position.copy(camPos);
      cam.lookAt(lookAt);
      cam.updateMatrixWorld(true);
      const toCentre = new THREE.Vector3(p.x, p.y, p.z).sub(camPos);
      const dist = toCentre.length();
      const offDeg = (Math.acos(Math.max(-1, Math.min(1, toCentre.clone().normalize().dot(fwd)))) * 180) / Math.PI;
      minOffDeg = Math.min(minOffDeg, offDeg);
      const angSize = (2 * Math.atan(R / Math.max(dist, 1)) * 180) / Math.PI;
      const ndc = new THREE.Vector3(p.x, p.y, p.z).project(cam);
      const tip = new THREE.Vector3(p.x, p.y - R * 1.7, p.z).project(cam);
      const inFrame = Math.abs(ndc.x) <= 1 && Math.abs(ndc.y) <= 1 && ndc.z < 1;
      views.push(
        `${label}: centre NDC(${ndc.x.toFixed(2)},${ndc.y.toFixed(2)}) tip NDC(${tip.x.toFixed(2)},${tip.y.toFixed(2)}) ${inFrame ? 'IN-FRAME' : 'out'} off-axis ${offDeg.toFixed(1)}deg size ${angSize.toFixed(1)}deg`,
      );
    }
    if (minOffDeg < worstViewDeg) {
      worstViewDeg = minOffDeg;
      worstView = g.name;
    }
    console.log(
      `  ${g.name} at (${p.x.toFixed(0)}, ${p.y.toFixed(0)}, ${p.z.toFixed(0)}) R=${R.toFixed(0)}m: ` +
        `nearest strip ${nearest.name} ${nearest.d.toFixed(0)}m (plateau ${nearest.dp.toFixed(0)}m) ` +
        `corridor=[${corridors.join(',') || 'none'}]`,
    );
    for (const v of views) console.log(`    ${v}`);
    check(
      `${g.name}: >= 1500 m plan from every runway strip`,
      nearest.d >= 1500,
      `nearest ${nearest.name} ${nearest.d.toFixed(0)} m`,
    );
    check(
      `${g.name}: >= 35 deg off Alpha's centreline from the spawn chase cam (not at the runway edge in the spawn view)`,
      minOffDeg >= 35,
      `off-axis=${minOffDeg.toFixed(1)} deg`,
    );
    check(
      `${g.name}: outside every runway's extended centreline corridor (+/-500 m across, 4000 m past each end)`,
      corridors.length === 0,
      `corridor=[${corridors.join(',') || 'none'}]`,
    );
  }
  console.log(`  nearest island to any runway strip: ${minStrip.toFixed(0)} m (${minStripWhat})`);
  console.log(`  closest island to Alpha's centreline in the spawn view: ${worstView || 'none'} ${worstViewDeg === Infinity ? '' : worstViewDeg.toFixed(1) + ' deg off-axis'}`);

  console.log(failures === 0 ? '\nALL ALPHA-ISLAND CHECKS PASSED' : `\n${failures} ALPHA-ISLAND CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
