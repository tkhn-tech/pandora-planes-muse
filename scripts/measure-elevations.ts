// measure-elevations.ts — one-off: find the max base terrain near each
// new 12 km runway (within ~50 m of the flat-box boundary + the ring
// at 520-700 m) so the plateau elevations can be set above it.
// Usage: node_modules/.bin/esbuild scripts/measure-elevations.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/measure-elevations.mjs
//   && node /tmp/measure-elevations.mjs
import { baseHeight, RUNWAY_TABLE } from '../src/world/PandoraTerrainWorker';

for (const r of RUNWAY_TABLE) {
  const sh = Math.sin((r.hdg * Math.PI) / 180);
  const ch = Math.cos((r.hdg * Math.PI) / 180);
  const toWorld = (lx: number, lz: number): [number, number] => [
    r.x + lx * sh + lz * ch,
    r.z + lx * ch - lz * sh,
  ];
  const LA = r.len / 2 + 160;
  const LC = r.width / 2 + 70;
  let edgeMax = -1e9;
  for (let lx = -LA - 50; lx <= LA + 50; lx += 10) {
    for (const lz of [-LC - 50, LC + 50]) {
      const [x, z] = toWorld(lx, lz);
      edgeMax = Math.max(edgeMax, baseHeight(x, z));
    }
  }
  for (let lz = -LC - 50; lz <= LC + 50; lz += 10) {
    for (const lx of [-LA - 50, LA + 50]) {
      const [x, z] = toWorld(lx, lz);
      edgeMax = Math.max(edgeMax, baseHeight(x, z));
    }
  }
  let ringMax = -1e9;
  let ringSum = 0;
  let ringN = 0;
  for (let a = 0; a < 360; a += 6) {
    const rad = (a * Math.PI) / 180;
    for (const rr of [520, 600, 700]) {
      const x = r.x + Math.cos(rad) * rr;
      const z = r.z + Math.sin(rad) * rr;
      const h = baseHeight(x, z);
      ringSum += h;
      ringN++;
      if (h > ringMax) ringMax = h;
    }
  }
  console.log(
    `${r.name} (${r.x},${r.z}) provisional elev ${r.elev}: edge max base = ${edgeMax.toFixed(1)} m, ring max = ${ringMax.toFixed(1)} m, ring mean = ${(ringSum / ringN).toFixed(1)} m -> suggested elev ${Math.ceil(edgeMax + 16)} m`,
  );
}
