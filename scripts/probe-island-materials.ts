// probe-island-materials.ts — island / scorch visibility (pass 5,
// item 2).
//
// The v4 play-test: the pass-2 island emissive fix did NOT work live
// — the islands still read as huge pure-black inverted cones, and a
// dark disc/ellipse sits on the ground by the runway. A shadowed
// Lambert face shows only emissive + hemisphere-ground * diffuse, so
// this probe builds the real world, finds every island mesh, and
// requires the shadowed-face brightness (emissive + hemi-ground *
// diffuse, the same lights as main.ts: hemi ground 0x4a5a3a @ 0.9)
// to be clearly visible (luminance > 0.25) — dark rock/grass, never
// pure black. It also checks the scorch decal is a dark brown-grey,
// not pure black (0x0a0a0a read as a black disc on the ground).
//
// Usage: node_modules/.bin/esbuild scripts/probe-island-materials.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-island-materials.mjs
//   && node /tmp/probe-island-materials.mjs

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
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { PandoraWorldGenerator } from '../src/world/PandoraWorldGenerator';
import { CrashVFXManager } from '../src/combat/CrashVFXManager';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

// main.ts lights: hemi sky 0xbfd9ff, ground 0x4a5a3a, intensity 0.9.
const HEMI_GROUND = new THREE.Color(0x4a5a3a);
const HEMI_INTENSITY = 0.9;

function luminance(c: THREE.Color): number {
  // three.js stores colours in linear-sRGB; perceived brightness is
  // the sRGB luminance, so convert back before measuring.
  const s = c.clone().convertLinearToSRGB();
  return 0.299 * s.r + 0.587 * s.g + 0.114 * s.b;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const gen = new PandoraWorldGenerator(scene, world);
  await gen.build(1337);
  // Island meshes: groups named island-i, materials rock/top/root.
  const mats = new Map<string, THREE.MeshLambertMaterial>();
  scene.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mat = mesh.material as THREE.MeshLambertMaterial;
    if (!mat || !('emissive' in mat)) return;
    // Only island meshes (cone / cylinder / root cones under an
    // island group) — the terrain mesh uses a different material.
    let p: THREE.Object3D | null = mesh.parent;
    let isIsland = false;
    while (p) {
      if (p.name.startsWith('island-')) {
        isIsland = true;
        break;
      }
      p = p.parent;
    }
    if (!isIsland) return;
    const key = `#${mat.color.getHexString()}/e#${mat.emissive.getHexString()}`;
    if (!mats.has(key)) mats.set(key, mat);
  });
  check('island materials found', mats.size >= 3, `distinct island materials=${mats.size}`);
  for (const [key, mat] of mats) {
    // Shadowed face: no direct sun, only emissive + hemisphere
    // ground light * diffuse.
    const shadowed = mat.emissive
      .clone()
      .add(
        new THREE.Color(
          HEMI_GROUND.r * HEMI_INTENSITY * mat.color.r,
          HEMI_GROUND.g * HEMI_INTENSITY * mat.color.g,
          HEMI_GROUND.b * HEMI_INTENSITY * mat.color.b,
        ),
      );
    const lum = luminance(shadowed);
    check(
      `island material ${key} reads as dark rock/grass in shadow (lum > 0.25)`,
      lum > 0.25,
      `shadowed luminance=${lum.toFixed(2)}`,
    );
  }
  // Scorch decal: dark brown-grey, not pure black.
  const vfx = new CrashVFXManager(scene, {});
  const decal = vfx.scorchDecal(new THREE.Vector3(0, 50, 0), 8);
  const dmat = decal.material as THREE.MeshBasicMaterial;
  const dLum = luminance(dmat.color);
  check(
    'scorch decal is dark brown-grey, not pure black',
    dLum > 0.1 && dmat.color.getHex() !== 0x0a0a0a,
    `color=#${dmat.color.getHexString()} lum=${dLum.toFixed(2)} opacity=${dmat.opacity}`,
  );

  console.log(failures === 0 ? '\nALL ISLAND-MATERIAL CHECKS PASSED' : `\n${failures} ISLAND-MATERIAL CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
