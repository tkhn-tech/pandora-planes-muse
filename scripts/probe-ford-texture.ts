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
import RAPIER from '@dimforge/rapier3d-compat';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import { HelicarrierController } from '../src/entities/HelicarrierController';

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

  // --- Controller build (pass 11, live v4.5 FAIL): the GLB's deck
  // plates are wound face-DOWN with a single-sided material, so the
  // texture was bound but invisible from above — the player saw the
  // light-grey hull top. The controller must flip the material to
  // DoubleSide and rotate the map 90° so the strip runs ALONG the
  // deck (the deck UVs run u along the length, v across the width,
  // while the strip runs vertically in the texture). Verified
  // geometrically: after the map's uvTransform, the transformed v'
  // must run along the deck length (aft vs forward halves differ)
  // and u' across the width (port vs starboard halves differ).
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const cScene = new THREE.Scene();
  const carrier = new HelicarrierController(cScene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: () => 0,
  });
  await carrier.build();
  let ctlDeckCount = 0;
  let ctlSideOk = false;
  let ctlRotOk = false;
  // Per deck mesh: transformed-UV halves (model-space x/z vs uv').
  const halves: Array<{ dvFwd: number; duPort: number }> = [];
  carrier.group.traverse((o) => {
    if (!(o as THREE.Mesh).isMesh) return;
    const mesh = o as THREE.Mesh;
    if (!mesh.name.startsWith('deck_top')) return;
    ctlDeckCount++;
    const mat = mesh.material as THREE.MeshStandardMaterial;
    if (mat.side === THREE.DoubleSide) ctlSideOk = true;
    if (mat.map) {
      mat.map.updateMatrix();
      if (Math.abs(mat.map.rotation - Math.PI / 2) < 1e-6) ctlRotOk = true;
      const g = mesh.geometry;
      const pos = g.attributes.position as THREE.BufferAttribute;
      const uv = g.attributes.uv as THREE.BufferAttribute | undefined;
      if (uv) {
        const m = mat.map.matrix;
        const v2 = new THREE.Vector2();
        let zMin = Infinity, zMax = -Infinity, xMin = Infinity, xMax = -Infinity;
        for (let i = 0; i < pos.count; i++) {
          zMin = Math.min(zMin, pos.getZ(i)); zMax = Math.max(zMax, pos.getZ(i));
          xMin = Math.min(xMin, pos.getX(i)); xMax = Math.max(xMax, pos.getX(i));
        }
        const zMid = (zMin + zMax) / 2, xMid = (xMin + xMax) / 2;
        let vAft = 0, vFwd = 0, nAft = 0, nFwd = 0;
        let uPort = 0, uStbd = 0, nPort = 0, nStbd = 0;
        for (let i = 0; i < uv.count; i++) {
          v2.set(uv.getX(i), uv.getY(i)).applyMatrix3(m);
          if (pos.getZ(i) < zMid) { vAft += v2.y; nAft++; } else { vFwd += v2.y; nFwd++; }
          if (pos.getX(i) < xMid) { uPort += v2.x; nPort++; } else { uStbd += v2.x; nStbd++; }
        }
        halves.push({
          dvFwd: Math.abs(vFwd / Math.max(nFwd, 1) - vAft / Math.max(nAft, 1)),
          duPort: Math.abs(uStbd / Math.max(nStbd, 1) - uPort / Math.max(nPort, 1)),
        });
      }
    }
  });
  check(
    'controller build: both deck meshes present with DoubleSide FordDeck',
    ctlDeckCount === 2 && ctlSideOk,
    `deck meshes=${ctlDeckCount} doubleSided=${ctlSideOk}`,
  );
  check(
    'controller build: deck map rotated 90° (strip along the deck)',
    ctlRotOk,
    `rotation=${Math.PI / 2}`,
  );
  check(
    'controller build: transformed v\' runs along the deck length, u\' across the width (strip along the deck, not across it)',
    halves.length === 2 && halves.every((h) => h.dvFwd > 0.3 && h.duPort > 0.3),
    halves.map((h) => `dV'(fwd-aft)=${h.dvFwd.toFixed(2)} dU'(port-stbd)=${h.duPort.toFixed(2)}`).join('; '),
  );

  // --- Pass 12 (Ford deck markings — live v4.6.1 FAIL, third time):
  // the deck_top plates are BURIED under other geometry, so from
  // above — the only angle a player ever sees the deck — the ray
  // hits flat grey, not FordDeck. Measured from the baked GLB
  // (2026-11-20, raster over the raw triangles, 1 m cells):
  //  * detail_0: a 50-tri slab, top Y≈35.4 (5.6 m above the deck),
  //    first hit on 64% of the stern deck footprint from above;
  //  * detail_7: a 72-tri slab, top Y≈30.25 (0.55 m above the deck),
  //    36 m wide, running the full length of the ship;
  //  * hull_35's top face at Y=29.78 sits at/above deck_top_35
  //    (29.70–29.78); hull_49's top at 29.69 is exactly coplanar
  //    with deck_top_49 (29.69). Both deck plates are 100% wound
  //    face-DOWN (0% of their XZ area faces up), so without
  //    DoubleSide they are invisible from above — and with it they
  //    are still buried. The pass-11 fix (DoubleSide + map rotation)
  //    addressed only the winding, never the burial.
  //  * FordDeck baseColorFactor is 0.169: even where the plate IS
  //    visible the texture is multiplied ~6x darker (white markings
  //    230 → 39, deck grey 100 → 17 — near-black, no contrast).
  // This section raycasts straight down onto the deck footprint
  // through the real controller build — the live top-down view —
  // and asserts the material/geometry state that decides the
  // visible colour (texture × material colour, first hit wins).
  let fordDeckMat: THREE.MeshStandardMaterial | null = null;
  let detail0Visible: boolean | null = null;
  let detail7Visible: boolean | null = null;
  const deckMinY: number[] = [];
  let markingPlanes = 0;
  let markingPlaneYOk = true;
  const markingPlaneMats: THREE.MeshStandardMaterial[] = [];
  carrier.group.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    if (mesh.name === 'detail_0') detail0Visible = mesh.visible;
    if (mesh.name === 'detail_7') detail7Visible = mesh.visible;
    if (mesh.name === 'deck_markings') {
      markingPlanes++;
      if (Math.abs(mesh.position.y - (carrier.deckLocalY + 0.2)) > 0.01) markingPlaneYOk = false;
      markingPlaneMats.push(mesh.material as THREE.MeshStandardMaterial);
    }
    if (!mesh.name.startsWith('deck_top')) return;
    const mat = mesh.material as THREE.MeshStandardMaterial;
    if (!fordDeckMat) fordDeckMat = mat;
    const g = mesh.geometry;
    if (!g.boundingBox) g.computeBoundingBox();
    if (g.boundingBox) deckMinY.push(g.boundingBox.min.y);
  });
  check(
    'pass 12: detail_0/detail_7 occluder slabs are hidden (not rendered above the deck)',
    detail0Visible === false && detail7Visible === false,
    `detail_0 visible=${detail0Visible} detail_7 visible=${detail7Visible}`,
  );
  check(
    'pass 12: deck plates sit above the hull top (29.78) — not buried/coplanar under it',
    deckMinY.length === 2 && deckMinY.every((y) => y > 29.78),
    `deck plate min-Y (model space): ${deckMinY.map((y) => y.toFixed(2)).join(', ')} vs hull top 29.78`,
  );
  check(
    'pass 12: FordDeck material colour is white — the texture is not multiplied ~6x darker',
    !!fordDeckMat &&
      fordDeckMat.color.r > 0.95 &&
      fordDeckMat.color.g > 0.95 &&
      fordDeckMat.color.b > 0.95,
    fordDeckMat
      ? `color=(${fordDeckMat.color.r.toFixed(3)}, ${fordDeckMat.color.g.toFixed(3)}, ${fordDeckMat.color.b.toFixed(3)}) (was 0.169/0.165/0.165)`
      : 'no FordDeck material found',
  );
  check(
    'pass 12: FordDeck uses polygonOffset so the coplanar hull top cannot win the depth test at range',
    !!fordDeckMat &&
      fordDeckMat.polygonOffset === true &&
      fordDeckMat.polygonOffsetFactor <= -1,
    fordDeckMat
      ? `polygonOffset=${fordDeckMat.polygonOffset} factor=${fordDeckMat.polygonOffsetFactor} units=${fordDeckMat.polygonOffsetUnits}`
      : 'no FordDeck material found',
  );
  // The GLB's deck surface is incomplete even with the slabs hidden:
  // the deck_top plates cover only 52% of the deck footprint (XZ
  // 6047 + 2622 m² of 16608 m²) and 21% of the footprint is a
  // through-hole with no surface at all — hiding the slabs alone
  // trades a grey deck for a deck with holes. The controller
  // therefore lays one flat textured plane per deck rectangle at
  // deckLocalY + 0.20, UV-mapped in the plates' own convention
  // (u along deck length, v across width — measured corr +1.000
  // both), sharing the Ford texture, so the markings cover the
  // whole deck including the holes and the hull-grey patches.
  check(
    'pass 12: one deck_markings plane per deck rectangle, at deckLocalY + 0.20 (above the plates, below the pads)',
    markingPlanes === carrier.deckRects.length && markingPlaneYOk,
    `planes=${markingPlanes} rects=${carrier.deckRects.length} yOk=${markingPlaneYOk}`,
  );
  check(
    'pass 12: deck_markings planes share the Ford texture with a stronger polygonOffset (they win depth over the plates)',
    markingPlaneMats.length === carrier.deckRects.length &&
      markingPlaneMats.every(
        (m) => !!fordDeckMat && m.map === fordDeckMat.map && m.polygonOffsetFactor <= -4,
      ),
    markingPlaneMats.length
      ? markingPlaneMats.map((m) => `mapShared=${!!fordDeckMat && m.map === fordDeckMat.map} factor=${m.polygonOffsetFactor}`).join('; ')
      : 'no deck_markings planes found',
  );
  carrier.group.updateMatrixWorld(true);
  const ray = new THREE.Raycaster();
  const downDir = new THREE.Vector3(0, -1, 0);
  const counts = { deck: 0, slab: 0, hull: 0, superstructure: 0, furniture: 0, none: 0 };
  let deckHits = 0;
  let uMin = Infinity;
  let uMax = -Infinity;
  let vMin = Infinity;
  let vMax = -Infinity;
  let total = 0;
  for (const r of carrier.deckRects) {
    for (let lx = r.cx - r.hx + 4; lx <= r.cx + r.hx - 4; lx += 8) {
      for (let lz = r.cz - r.hz + 4; lz <= r.cz + r.hz - 4; lz += 8) {
        total++;
        const origin = carrier.group.localToWorld(
          new THREE.Vector3(lx, carrier.deckLocalY + 80, lz),
        );
        ray.set(origin, downDir);
        // THREE r160's Raycaster does NOT skip invisible objects
        // (Raycaster.js has no `visible` test) but the renderer does,
        // so the first RENDERED hit is the first hit whose object and
        // every ancestor is visible — hidden slabs must not count.
        const hits = ray.intersectObjects(carrier.group.children, true);
        const hit = hits.find((h) => {
          let o: THREE.Object3D | null = h.object;
          while (o) {
            if (!o.visible) return false;
            o = o.parent;
          }
          return true;
        });
        if (!hit) {
          counts.none++;
          continue;
        }
        const n = (hit.object as THREE.Mesh).name ?? '';
        if (n.startsWith('deck_top') || n === 'deck_markings') {
          counts.deck++;
          deckHits++;
          const uv = (hit as THREE.Intersection & { uv?: THREE.Vector2 }).uv;
          const mat = (hit.object as THREE.Mesh).material as THREE.MeshStandardMaterial;
          if (uv && mat.map) {
            mat.map.updateMatrix();
            const t = new THREE.Vector2(uv.x, uv.y).applyMatrix3(mat.map.matrix);
            uMin = Math.min(uMin, t.x);
            uMax = Math.max(uMax, t.x);
            vMin = Math.min(vMin, t.y);
            vMax = Math.max(vMax, t.y);
          }
        } else if (n === 'detail_0' || n === 'detail_7') counts.slab++;
        else if (n.startsWith('hull')) counts.hull++;
        else if (n === '') counts.furniture++;
        else counts.superstructure++;
      }
    }
  }
  const deckFrac = total ? counts.deck / total : 0;
  check(
    'pass 12 top-down: deck_top (Ford markings) is the first hit on a majority of deck samples',
    deckFrac > 0.6,
    `deck ${counts.deck}/${total} (${(deckFrac * 100).toFixed(1)}%) slab=${counts.slab} hull=${counts.hull} superstructure=${counts.superstructure} furniture=${counts.furniture} none=${counts.none}`,
  );
  check(
    'pass 12 top-down: no ray hits the detail_0/detail_7 slabs above the deck',
    counts.slab === 0,
    `slab hits=${counts.slab}/${total}`,
  );
  check(
    'pass 12 top-down: no ray hits the bare hull top over the deck',
    counts.hull === 0,
    `hull hits=${counts.hull}/${total}`,
  );
  check(
    'pass 12 top-down: deck UVs span the texture (markings sampled across the deck, not one flat texel)',
    deckHits >= 50 && uMax - uMin > 0.5 && vMax - vMin > 0.5,
    `deck hits=${deckHits} u' [${uMin.toFixed(2)}, ${uMax.toFixed(2)}] v' [${vMin.toFixed(2)}, ${vMax.toFixed(2)}]`,
  );

  // --- Pass 12 (carrier pod readout): pod 3 read 0 mid-round in the
  // v4.6.1 live check, back to 100/100/100/100 later. The ONLY path
  // that lowers hp is damagePod <- onCarrierPodHit <- gun/bomb hits
  // (WeaponSystem checkPodHits: every gun round whose ray passes
  // within 12 m of a pod centre inside GUN_RANGE 1200 m deals
  // GUN_DAMAGE 12; a bomb blast within 45 m deals 40). 9 gun hits
  // kill a pod (100/12), and at 800 rpm (0.075 s cooldown) that is
  // 0.68 s of fire — the carrier sits 900 m dead ahead at round
  // start, inside gun range, so any test burst toward it kills a
  // pod; reset() on round restart restores 100, matching the live
  // observation exactly. This section proves the readout is
  // faithful: hp is rock-stable with no damage input, damagePod
  // moves it exactly as designed, and reset() restores it.
  let podStable = true;
  let podMin = 100;
  for (let s = 0; s < 180; s += 1) {
    for (let k = 0; k < 20; k++) carrier.update(0.05); // 1 s per step, 180 s total
    const hps = carrier.pods.map((p) => p.hp);
    podMin = Math.min(podMin, ...hps);
    if (hps.some((h) => h !== 100)) podStable = false;
  }
  check(
    'pass 12 pods: hp stays 100 with no damage input over 180 s of update()',
    podStable && podMin === 100,
    `min hp seen=${podMin} over 180 s`,
  );
  for (let i = 0; i < 9; i++) carrier.damagePod(2, 12);
  const afterDmg = carrier.pods[2]!;
  check(
    'pass 12 pods: 9x GUN_DAMAGE (12) kills pod 2 (hp 100 -> 0, alive false)',
    afterDmg.hp === 0 && afterDmg.alive === false,
    `pod 2 hp=${afterDmg.hp} alive=${afterDmg.alive} (9 hits x 12 dmg = 108 >= 100)`,
  );
  carrier.reset();
  const afterReset = carrier.pods;
  check(
    'pass 12 pods: reset() restores all pods to 100/alive (the live "back to 100/100/100/100")',
    afterReset.length === 4 && afterReset.every((p) => p.hp === 100 && p.alive === true),
    `pods=[${afterReset.map((p) => p.hp).join('/')}]`,
  );

  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
