// embed_models.mjs — embed the 7 verified Pandora Planes GLBs as base64
// TypeScript modules under src/assets/models/.
//
// Why: the GitHub connection can only push text files, so binary GLBs
// cannot be committed to the repo. Each model is stored as a
// `data:model/gltf-binary;base64,...` string in a .ts module instead,
// exactly like the single-file HTML v2.0 did with inline constants.
//
// Sources:
//   * F-18 (JET), MiG-29, Ford — re-extracted byte-identical from the v2
//     single-file build (../pandora-planes-html/index.html) data-URI
//     constants. Cross-checked against the on-disk originals where they
//     exist (mig29.glb, ford.glb, shipmodels/models_b64_jet.js).
//   * Su-34, F-22, F-15E, F-35A — read from the .glb files in
//     ../pandora-planes-html/models/ and cross-checked against the same
//     constants embedded in the v2 index.html.
//
// Verification (runs on every invocation, exits non-zero on failure):
//   * decoded bytes start with the GLB magic 'glTF' and version 2
//   * the JSON chunk parses; mesh / primitive counts and summed triangle
//     counts are reported (and checked against the expected counts)
//   * SHA-256 of the decoded bytes matches the source .glb file wherever
//     a source file exists, and matches the v2 index.html constant
//     wherever a constant exists
//   * each generated .ts file is under ~1 MB
//
// Usage: node scripts/embed_models.mjs   (run from the project root)
//   or:  npm run embed-models
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const HTML_ROOT = join(ROOT, '..', 'pandora-planes-html');
const OUT_DIR = join(ROOT, 'src', 'assets', 'models');
const MAX_TS_BYTES = 1_000_000;

// Expected triangle counts, from the v2 build reports
// (models/NEW_JETS.md, models/SU34_DOWNSIZE.md, template header).
const EXPECTED_TRIS = {
  fa18: 17517,
  mig29: 5931,
  su34: 15903,
  ford: 2678,
  f22: 27116,
  f15e: 24147,
  f35a: 4601,
};

// name, .ts export name, v2 index.html constant, source .glb file (if any)
const MODELS = [
  { key: 'fa18', file: 'fa18.ts', exportName: 'FA18_GLB', constant: 'JET_GLB',
    glbFile: null, altSource: join(ROOT, '..', 'shipmodels', 'models_b64_jet.js') },
  { key: 'mig29', file: 'mig29.ts', exportName: 'MIG29_GLB', constant: 'MIG_GLB',
    glbFile: join(HTML_ROOT, 'mig29.glb') },
  { key: 'su34', file: 'su34.ts', exportName: 'SU34_GLB', constant: 'SU34_GLB',
    glbFile: join(HTML_ROOT, 'models', 'su34-downsized.glb') },
  { key: 'ford', file: 'ford.ts', exportName: 'FORD_GLB', constant: 'FORD_GLB',
    glbFile: join(HTML_ROOT, 'ford.glb') },
  { key: 'f22', file: 'f22.ts', exportName: 'F22_GLB', constant: 'F22_GLB',
    glbFile: join(HTML_ROOT, 'models', 'f22.glb') },
  { key: 'f15e', file: 'f15e.ts', exportName: 'F15E_GLB', constant: 'F15E_GLB',
    glbFile: join(HTML_ROOT, 'models', 'f15e.glb') },
  { key: 'f35a', file: 'f35a.ts', exportName: 'F35A_GLB', constant: 'F35A_GLB',
    glbFile: join(HTML_ROOT, 'models', 'f35a.glb') },
];

const sha256 = (buf) => createHash('sha256').update(buf).digest('hex');

function extractConstant(source, name) {
  const m = source.match(new RegExp(`const ${name}\\s*=\\s*'data:model/gltf-binary;base64,([^']+)';`));
  if (!m) throw new Error(`constant ${name} not found`);
  return m[1];
}

function verifyGlb(buf, label) {
  if (buf.length < 20) throw new Error(`${label}: too short (${buf.length} B)`);
  if (buf.toString('ascii', 0, 4) !== 'glTF') throw new Error(`${label}: bad magic`);
  const version = buf.readUInt32LE(4);
  if (version !== 2) throw new Error(`${label}: GLB version ${version}, expected 2`);
  const jsonLen = buf.readUInt32LE(12);
  const jsonType = buf.toString('ascii', 16, 20);
  if (jsonType !== 'JSON') throw new Error(`${label}: first chunk is not JSON`);
  const json = JSON.parse(buf.toString('utf8', 20, 20 + jsonLen));
  const accessors = json.accessors ?? [];
  let meshes = 0, prims = 0, tris = 0;
  for (const mesh of json.meshes ?? []) {
    meshes++;
    for (const p of mesh.primitives ?? []) {
      prims++;
      const idx = p.indices !== undefined ? accessors[p.indices] : null;
      const pos = accessors[p.attributes?.POSITION];
      const count = idx ? idx.count : pos ? pos.count : 0;
      if (count % 3 !== 0) throw new Error(`${label}: primitive count ${count} not divisible by 3`);
      tris += count / 3;
    }
  }
  return { meshes, prims, tris };
}

function main() {
  mkdirSync(OUT_DIR, { recursive: true });
  const builtHtml = readFileSync(join(HTML_ROOT, 'index.html'), 'utf8');
  const rows = [];
  let totalTs = 0;

  for (const m of MODELS) {
    // 1. Source bytes: prefer the .glb file; F-18 has no file, use the
    //    v2 index.html constant (cross-checked against models_b64_jet.js).
    let glbBytes;
    let fileSha = null;
    if (m.glbFile) {
      glbBytes = readFileSync(m.glbFile);
      fileSha = sha256(glbBytes);
    }
    // 2. Extract the v2 constant and decode it.
    const b64 = extractConstant(builtHtml, m.constant);
    const fromHtml = Buffer.from(b64, 'base64');
    if (m.glbFile && sha256(fromHtml) !== fileSha) {
      throw new Error(`${m.key}: v2 index.html ${m.constant} bytes differ from ${m.glbFile}`);
    }
    if (m.altSource) {
      const altB64 = extractConstant(readFileSync(m.altSource, 'utf8'), m.constant);
      if (altB64 !== b64) throw new Error(`${m.key}: ${m.constant} differs from ${m.altSource}`);
      glbBytes = fromHtml;
      fileSha = sha256(glbBytes);
    }
    if (!glbBytes) glbBytes = fromHtml;

    // 3. Verify the GLB structure and count triangles.
    const { meshes, prims, tris } = verifyGlb(glbBytes, m.key);
    const expected = EXPECTED_TRIS[m.key];
    if (tris !== expected) {
      throw new Error(`${m.key}: ${tris} tris, expected ${expected}`);
    }

    // 4. Write the .ts module (canonical base64 of the verified bytes).
    const uri = 'data:model/gltf-binary;base64,' + glbBytes.toString('base64');
    const ts = `// Auto-generated by scripts/embed_models.mjs — do not edit by hand.\n// ${m.key.toUpperCase()} GLB (${glbBytes.length.toLocaleString('en-US')} B, ${tris.toLocaleString('en-US')} tris, sha256 ${fileSha.slice(0, 16)}…)\n// Source: ${m.glbFile ? m.glbFile.replace(ROOT + '/', '') : 'v2 index.html ' + m.constant}\n// Loaded with GLTFLoader + MeshoptDecoder (see src/).\nexport const ${m.exportName} = '${uri}';\n`;
    const outPath = join(OUT_DIR, m.file);
    writeFileSync(outPath, ts);
    const tsBytes = Buffer.byteLength(ts);
    if (tsBytes >= MAX_TS_BYTES) {
      throw new Error(`${m.file}: ${tsBytes} B exceeds the ~1 MB limit`);
    }
    totalTs += tsBytes;
    rows.push({ key: m.key, file: m.file, glbBytes: glbBytes.length, tsBytes, sha: fileSha, meshes, prims, tris });
  }

  console.log('model     .ts file       GLB bytes   .ts bytes  meshes prims    tris  sha256');
  for (const r of rows) {
    console.log(
      `${r.key.padEnd(8)} ${r.file.padEnd(12)} ${String(r.glbBytes).padStart(9)} ${String(r.tsBytes).padStart(10)}` +
      ` ${String(r.meshes).padStart(6)} ${String(r.prims).padStart(5)} ${String(r.tris).padStart(7)}  ${r.sha}`,
    );
  }
  console.log(`\n7 models embedded, total .ts size ${(totalTs / 1024).toFixed(1)} KiB (${totalTs.toLocaleString('en-US')} B).`);
  console.log('All checks passed: glTF magic + version 2, JSON chunk parses, tri counts match, SHA-256 matches source files and v2 constants.');
}

main();
