// probe-fps-readout.ts — pass 14, worker E: the ?debug=1
// renderer.info readout.
//
// The v4.8 live check could not read renderer.info (the browser
// tooling has no console access), so the render-side cost stayed
// unmeasured. This probe verifies the instrument that fixes that:
//   1. FlightHUD.renderInfoLine formats a renderer.info snapshot
//      into one small, clearly labelled line (calls, triangles,
//      geometries, textures, programs, DPR, drawing-buffer size,
//      antialias, tone mapping) — functionally, with a fake snapshot.
//   2. main.ts updateDebugPanel actually reads renderer.info.render /
//      .memory / .programs and puts that line in the ?debug=1 panel.
//   3. The measurement-only URL knobs (?dpr= / ?noaa=1) exist AND the
//      defaults are unchanged (antialias:true, DPR=min(dpr, 2)) —
//      nothing is shipped unmeasured; the knobs only let the live
//      check A/B the fragment-fill suspect.
//
// These checks FAIL on the pre-fix tree (no renderer.info anywhere
// in main.ts / FlightHUD.ts) and PASS after. The live GL values
// themselves (calls/tris on the phone) can only be read by the live
// browser check — this probe verifies the wiring, not the GPU.
//
// Usage: node_modules/.bin/esbuild scripts/probe-fps-readout.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-fps-readout.mjs
//   && node /tmp/probe-fps-readout.mjs

if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    getElementById: (_id: string) => null,
    createElement: () => ({}),
    createElementNS: () => ({}),
  };
}

import * as fs from 'node:fs';
import { FlightHUD } from '../src/ui/FlightHUD';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

function main(): void {
  const mainSrc = fs.readFileSync('src/main.ts', 'utf8');
  const hudSrc = fs.readFileSync('src/ui/FlightHUD.ts', 'utf8');

  // ---- 1. Formatting (functional, fake snapshot) ----
  const hud = new FlightHUD();
  const line = hud.renderInfoLine({
    calls: 260,
    triangles: 207774,
    geometries: 411,
    textures: 12,
    programs: 8,
    pixelRatio: 2,
    bufferW: 2160,
    bufferH: 4800,
    antialias: true,
  });
  console.log(`formatted line: ${line}`);
  check(
    'renderInfoLine shows calls + triangles',
    line.includes('260 calls') && line.includes('207,774 tris'),
    line,
  );
  check(
    'renderInfoLine shows geometries + textures + programs',
    line.includes('411 geo') && line.includes('12 tex') && line.includes('8 programs'),
    line,
  );
  check(
    'renderInfoLine shows DPR + drawing-buffer size + AA + tone mapping',
    line.includes('DPR 2') &&
      line.includes('buffer 2160x4800') &&
      line.includes('AA on') &&
      line.includes('ACES'),
    line,
  );
  const offLine = hud.renderInfoLine({
    calls: 59,
    triangles: 145156,
    geometries: 411,
    textures: 12,
    programs: 8,
    pixelRatio: 1,
    bufferW: 1080,
    bufferH: 2400,
    antialias: false,
  });
  check(
    'renderInfoLine reflects the no-AA / DPR 1 knob state',
    offLine.includes('59 calls') && offLine.includes('DPR 1') && offLine.includes('AA off'),
    offLine,
  );

  // ---- 2. Wiring in main.ts ----
  check(
    'main.ts reads renderer.info.render.calls + triangles',
    mainSrc.includes('renderer.info.render.calls') &&
      mainSrc.includes('renderer.info.render.triangles'),
  );
  check(
    'main.ts reads renderer.info.memory.geometries + textures',
    mainSrc.includes('renderer.info.memory.geometries') &&
      mainSrc.includes('renderer.info.memory.textures'),
  );
  check(
    'main.ts reads renderer.info.programs (guarded: null until first render)',
    mainSrc.includes('renderer.info.programs') &&
      mainSrc.includes('Array.isArray(renderer.info.programs)'),
  );
  check(
    'main.ts reads DPR + drawing-buffer size for the readout',
    mainSrc.includes('renderer.getPixelRatio()') &&
      mainSrc.includes('renderer.getDrawingBufferSize('),
  );
  check(
    'updateDebugPanel puts the renderer line in the ?debug=1 panel',
    mainSrc.includes('hud.renderInfoLine({') &&
      mainSrc.includes('renderInfoLine,'),
    'line is built from renderer.info and inserted after the FPS header',
  );
  check(
    'readout is labelled as the last completed frame (panel is built before render)',
    mainSrc.includes('LAST COMPLETED frame'),
  );

  // ---- 3. Measurement knobs, defaults unchanged ----
  check(
    '?dpr= knob exists (clamped 0.5..2, null = default)',
    mainSrc.includes("get('dpr')") && mainSrc.includes('DEBUG_DPR'),
  );
  check(
    '?noaa=1 knob exists (antialias off for that session only)',
    mainSrc.includes("get('noaa') === '1'") && mainSrc.includes('DEBUG_NOAA'),
  );
  check(
    'default renderer is exactly as before (antialias:true, DPR=min(dpr,2))',
    mainSrc.includes('antialias: RENDER_ANTIALIAS') &&
      mainSrc.includes('DEBUG_DPR ?? Math.min(window.devicePixelRatio, 2)') &&
      mainSrc.includes('const RENDER_ANTIALIAS = !DEBUG_NOAA'),
    'no cap ships by default — the knobs only enable a live A/B',
  );
  check(
    'FlightHUD exports the RenderInfoSnapshot type',
    hudSrc.includes('export interface RenderInfoSnapshot'),
  );

  console.log(failures === 0 ? '\nALL FPS-READOUT CHECKS PASSED' : `\n${failures} FPS-READOUT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
