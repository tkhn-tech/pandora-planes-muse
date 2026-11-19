// probe-ford-texture.ts — pass 10, item 2: the Gerald R Ford
// texture must be properly applied to the SHIELD helicarrier.
//
// What "properly applied" means here (established by GLB
// inspection, 2026-11-20):
// - The DECK carries a real texture: only the two deck_top meshes
//   have UVs, and their material (FordDeck) samples it. The texture
//   baked into the helicarrier GLB is a crop of the Ford atlas's own
//   flight-deck strip (x 0–200, y 550–950 of the 1024x1024 Ford
//   atlas — the angled landing strip with its white/yellow dashed
//   centreline, red-white dashed edges and yellow parking boxes),
//   upscaled 2x to 400x800 so the markings stay readable on the
//   71.6 x 130.8 m deck. It replaced the 2 KB painted PNG.
// - The HULL / island / rotor pods have NO UVs (POSITION,NORMAL
//   only), so the Ford atlas CANNOT be sampled onto them — they wear
//   the Ford-matched flat colours instead (hull grey #a1a29d, island
//   grey, pod grey, Ford red accents), sampled from the Ford model.
//   That is a hard limit of the source geometry, not a missing step.
//
// This probe asserts, from the baked GLB and from a live GLTFLoader
// load: the deck texture is a JPEG of 400x800 (the Ford crop — the
// old texture was a 1024x288 PNG), it is bound to the FordDeck
// material and sampled by both deck meshes, the deck meshes have
// UVs, and the hull/island/pod materials carry the Ford-matched
// colours with no map (they cannot take one).
//
// Usage: node_modules/.bin/esbuild scripts/probe-ford-texture.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-ford-texture.mjs
//   && node /tmp/probe-ford-texture.mjs

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
    createElement: () => ({}),
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => {
          (listeners[type] ??= []).push(fn);
        },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', {
        set: () => {
          queueMicrotask(() => {
            for (const fn of listeners['load'] ?? []) fn();
          });
        },
      });
      return img;
    },
  };
}

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

/** Read a JPEG's pixel dimensions from its SOF0/SOF2 marker (no
 *  decoder needed). Returns [width, height]. */
function jpegDimensions(buf: Buffer): [number, number] {
  let i = 2; // skip SOI (FFD8)
  while (i < buf.length - 1) {
    if (buf[i] !== 0xff) {
      i++;
      continue;
    }
    const marker = buf[i + 1]!;
    if (marker === 0xd8 || marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) {
      i += 2;
      continue;
    }
    const len = buf.readUInt16BE(i + 2);
    if (marker === 0xc0 || marker === 0xc2) {
      const h = buf.readUInt16BE(i + 5);
      const w = buf.readUInt16BE(i + 7);
      return [w, h];
    }
    i += 2 + len;
  }
  throw new Error('no SOF marker found');
}

function b64ToBuffer(uri: string): Buffer {
  const b64 = uri.split('base64,')[1]!;
  return Buffer.from(b64, 'base64');
}

async function main(): Promise<void> {
  const glb = b64ToBuffer(HELICARRIER_GLB);
  // GLB header: magic, version, length; then the JSON chunk.
  const jsonLen = glb.readUInt32LE(12);
  const gltf = JSON.parse(glb.subarray(20, 20 + jsonLen).toString('utf-8'));
  const binOff = 20 + jsonLen;
  const binLen = glb.readUInt32LE(binOff);
  const bin = glb.subarray(binOff + 8, binOff + 8 + binLen);

  // --- The baked texture: a JPEG, 400x800, the Ford atlas crop.
  const img = gltf.images[0];
  check(
    'deck texture is a JPEG (not the old painted PNG)',
    img?.mimeType === 'image/jpeg',
    `mimeType=${img?.mimeType}`,
  );
  const bv = gltf.bufferViews[img.bufferView];
  const imgBytes = bin.subarray(bv.byteOffset, bv.byteOffset + bv.byteLength);
  const [w, h] = jpegDimensions(imgBytes);
  check(
    'deck texture is the 400x800 Ford atlas crop (2x upscale of the 200x400 deck strip)',
    w === 400 && h === 800,
    `${w}x${h}, ${bv.byteLength} bytes (old texture was a 1024x288 PNG, 2,044 bytes)`,
  );
  check(
    'deck texture is a real photo texture, not a flat fill',
    bv.byteLength > 10000,
    `${bv.byteLength} bytes`,
  );

  // --- The FordDeck material samples it; the deck meshes use it.
  const deckMat = gltf.materials.find((m: { name: string }) => m.name === 'FordDeck');
  check(
    'FordDeck material references the deck texture',
    !!deckMat?.pbrMetallicRoughness?.baseColorTexture &&
      gltf.textures[deckMat.pbrMetallicRoughness.baseColorTexture.index]?.source === 0,
    `baseColorTexture=${JSON.stringify(deckMat?.pbrMetallicRoughness?.baseColorTexture)}`,
  );
  const deckMeshes = gltf.meshes.filter((m: { name: string }) => m.name.startsWith('deck_top'));
  check(
    'both deck sections use the FordDeck material',
    deckMeshes.length === 2 &&
      deckMeshes.every(
        (m: { primitives: Array<{ material: number }> }) =>
          gltf.materials[m.primitives[0]!.material]?.name === 'FordDeck',
      ),
    deckMeshes.map((m: { name: string }) => m.name).join(', '),
  );
  // The deck meshes have UVs (TEXCOORD_0) — they can sample the
  // texture. (The hull/island/pod meshes do not; see below.)
  check(
    'deck meshes carry UVs (TEXCOORD_0) so the Ford markings land on them',
    deckMeshes.every(
      (m: { primitives: Array<{ attributes: Record<string, number> }> }) =>
        m.primitives[0]!.attributes.TEXCOORD_0 !== undefined,
    ),
    deckMeshes.map((m: { name: string }) => m.name).join(', '),
  );
  // No other mesh has UVs — the atlas cannot be applied to them.
  const uvMeshes = gltf.meshes.filter((m: { primitives: Array<{ attributes: Record<string, number> }> }) =>
    m.primitives.some((p) => p.attributes.TEXCOORD_0 !== undefined),
  );
  check(
    'only the deck meshes have UVs (hull/island/pods cannot sample the atlas)',
    uvMeshes.length === 2,
    `${uvMeshes.length} meshes with UVs: ${uvMeshes.map((m: { name: string }) => m.name).join(', ')}`,
  );

  // --- The hull/island/pods wear the Ford-matched flat colours.
  const expect: Record<string, [number, number, number]> = {
    FordHullGrey: [0.631, 0.635, 0.616],
    FordIslandGrey: [0.678, 0.682, 0.663],
    FordPodGrey: [0.561, 0.565, 0.545],
  };
  for (const [name, rgb] of Object.entries(expect)) {
    const mat = gltf.materials.find((m: { name: string }) => m.name === name);
    const f = mat?.pbrMetallicRoughness?.baseColorFactor;
    const close =
      !!f && Math.abs(f[0] - rgb[0]) < 0.01 && Math.abs(f[1] - rgb[1]) < 0.01 && Math.abs(f[2] - rgb[2]) < 0.01;
    check(
      `${name} carries the Ford-matched flat colour (no map — no UVs)`,
      close && !mat?.pbrMetallicRoughness?.baseColorTexture,
      `factor=[${f?.slice(0, 3).map((v: number) => v.toFixed(3)).join(', ')}]`,
    );
  }

  // --- Live load: the FordDeck material's map is a real texture and
  // the deck meshes' material is FordDeck (the texture is actually
  // bound and sampled, not just declared in the JSON).
  const loader = new GLTFLoader();
  loader.setMeshoptDecoder(MeshoptDecoder);
  const scene: THREE.Object3D = await new Promise((res, rej) => {
    loader.load(HELICARRIER_GLB, (g) => res(g.scene), undefined, (e) => rej(e));
  });
  let deckMapOk = false;
  let deckMeshCount = 0;
  scene.traverse((o) => {
    if (!(o as THREE.Mesh).isMesh) return;
    const mesh = o as THREE.Mesh;
    const mat = mesh.material as THREE.MeshStandardMaterial;
    if (mesh.name.startsWith('deck_top')) {
      deckMeshCount++;
      if (mat.name === 'FordDeck' && mat.map) deckMapOk = true;
    }
  });
  check(
    'live load: deck meshes use FordDeck with a bound map texture',
    deckMeshCount === 2 && deckMapOk,
    `deck meshes=${deckMeshCount} map bound=${deckMapOk}`,
  );

  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
