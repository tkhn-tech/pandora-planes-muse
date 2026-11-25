// probe-pass17-debris-render.ts — pass 17, worker D (item: debris / slabs).
//
// The v4.10 live verdict: wrecks at +32 s / +78 s read as a single
// intact-looking jet with an orange glow; other wrecks are 3-4 flat
// black slabs, not torn panels. Pass 15's tornPanelGeometry passed in
// the harness (0/6 slab-like by geometry parameters) but live still
// read as slabs — so geometry parameters are NOT the verification.
// This probe verifies through RENDERED frames: it stages a real F-22
// and F-18 fast hard impact through the LIVE crash path (real GLBs,
// variable frame dt with the main.ts 0.10 s sim cap, live 3 m/s wind,
// handleDeaths poll — no onCrash event), steps to +30 s and +75 s sim
// time, and renders the crash site in headless Chromium (SwiftShader
// WebGL, live renderer/lighting settings) from the chase camera
// (42 m, chaseCameraPosition) and the spectate orbit (30 m, pitch
// 0.32). Each frame is measured from readPixels with ablation renders
// (wreck hidden / debris hidden / scorch hidden), so every number is
// what is actually visible — the same method as the Helicarriers
// fragmentation verification (rendered silhouettes, not geometry).
//
// The browser harness lives in HARNESS_SRC below (embedded as a
// string; edit it there — it is the whole scenario + render +
// pixel analysis). The node driver writes it to /tmp, bundles it
// with esbuild for the browser, serves it on 127.0.0.1, drives it
// with Playwright (one jet per page load), collects the metrics,
// asserts, and deletes every /tmp file it made.
//
// Pre-fix (v4.11 CrashVFXManager) this probe FAILS on: debris black
// slabs (generic pieces render at luminance 3-9), scorch invisible
// from chase (0 px at +30 s, pale stain), fire with no bright core
// (0 bright-orange px). Post-fix it PASSES those — plus the wreck
// char check, which REQUIRES the FlightPhysicsController patch
// reported alongside this probe (wreckModel's x0.22 char is ~#818181
// in sRGB because the multiply runs in linear space; the patch
// darkens it to ~#2c2c2c and adds a nose-down pitch so the wreck
// reads as plowed-in from the side view too). Without that patch
// the wreck-char checks fail and the probe fails — by design: the
// item is not done until the wreck itself reads as burnt.
//
// Usage: node_modules/.bin/esbuild scripts/probe-pass17-debris-render.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-pass17-debris-render.mjs
//   && node /tmp/probe-pass17-debris-render.mjs
//   (delete /tmp/probe-pass17-debris-render.mjs after running;
//   the probe deletes its own harness/bundle/server files itself)

import * as fs from 'node:fs';
import * as http from 'node:http';
import * as path from 'node:path';
import { execFileSync } from 'node:child_process';
import { createRequire } from 'node:module';

const HOME = process.env.HOME ?? '/home/hatch';
const PROJ = `${HOME}/workspace/pandora-planes-vite`;
const TMPDIR = '/tmp/pass17-debris-render';

const HARNESS_SRC = "// pass17-debris-render harness (browser side; embedded as a string in\n// scripts/probe-pass17-debris-render.ts, which bundles and serves it).\n// Renders a real crash through the LIVE path in headless Chromium\n// (SwiftShader WebGL) and measures the wreck from RENDERED pixels\n// (readPixels + ablation renders), from the chase camera (42 m) and the\n// spectate orbit (30 m, pitch 0.32) at +30 s and +75 s sim time after\n// impact.\nimport * as THREE from 'three';\nimport RAPIER from '@dimforge/rapier3d-compat';\nimport { Aircraft } from '../src/physics/FlightPhysicsController';\nimport type { DeckProvider } from '../src/physics/FlightPhysicsController';\nimport { SPECS } from '../src/aircraft/AircraftSpec';\nimport { CrashVFXManager } from '../src/combat/CrashVFXManager';\nimport { FloatingOrigin } from '../src/world/FloatingOrigin';\nimport { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';\nimport { chaseCameraPosition, orbitCameraPosition, applyCameraFloor } from '../src/camera/OrbitCamera';\n\nconst W = 900;\nconst H = 560;\nconst WIND = new THREE.Vector3(3, 0, 0);\nconst RAW_DT_PATTERN = [0.25, 0.07, 0.16, 0.09, 0.2, 0.12];\nlet rawIdx = 0;\n/** Live frame driver (main.ts animate): simDt = min(rawDt, 0.1), substeps <= 0.05 s. */\nfunction liveFrame(stepSub: (dt: number) => void): void {\n  const rawDt = RAW_DT_PATTERN[rawIdx++ % RAW_DT_PATTERN.length]!;\n  const simDt = Math.min(rawDt, 0.1);\n  const sub = Math.max(1, Math.ceil(simDt / 0.05));\n  const dt = simDt / sub;\n  for (let s = 0; s < sub; s++) stepSub(dt);\n}\n\ntype DebrisRec = { mesh: THREE.Object3D; body: RAPIER.RigidBody; life: number; age: number };\ntype ScorchRec = { mesh: THREE.Mesh; age: number };\ntype PlumeRec = { group: THREE.Group; life: number; maxLife: number; parent: THREE.Object3D | null; baseOpacity: number };\ntype BoomRec = { sprite: THREE.Sprite; t: number; scale: number };\n\ninterface PieceRender {\n  label: string;\n  rectW: number; rectH: number; rectArea: number;\n  darkPx: number; fill: number;\n  lumP10: number; lumP50: number; lumP90: number; lumRange: number;\n  centreAboveGround: number; restTiltDeg: number;\n}\ninterface FrameMetrics {\n  jet: string; sample: number; cam: string;\n  wreckPx: number; wreckBboxW: number; wreckBboxH: number; wreckFill: number;\n  wreckMeanLum: number; wreckP90Lum: number; wreckMeanR: number; wreckMeanG: number; wreckMeanB: number;\n  wreckSpanPx: number; wreckLenPx: number;\n  debrisPx: number; debrisCount: number;\n  scorchPx: number; scorchMeanLum: number; scorchP25Lum: number; scorchOutsideWreckPx: number;\n  firePx: number; fireBrightPx: number;\n  pieces: PieceRender[];\n  attachedPlumes: number; plumeMaxOpacity: number;\n  debrisBelowGround: number;\n  wreckTiltDeg: number; airframeColorHex: string;\n}\n\nconst results: FrameMetrics[] = [];\n\nasync function main(): Promise<void> {\n  await RAPIER.init();\n  const heights = generateHeights({ seed: 1337, chunkX: 0, chunkZ: 0, size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1 });\n  const res = TERRAIN_SEGMENTS + 1;\n  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });\n  const eventQueue = new RAPIER.EventQueue(true);\n  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());\n  const hfHeights = new Float32Array(heights.length);\n  for (let iz = 0; iz < res; iz++) for (let ix = 0; ix < res; ix++) hfHeights[ix * res + iz] = heights[iz * res + ix]!;\n  world.createCollider(\n    RAPIER.ColliderDesc.heightfield(TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights, new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE))\n      .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0)\n      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),\n    terrainBody,\n  );\n  const scene = new THREE.Scene();\n  scene.background = new THREE.Color(0x87b5d9);\n  const origin = new FloatingOrigin(2000);\n  origin.registerBody(terrainBody);\n  const heightAt = (x: number, z: number): number =>\n    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);\n  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };\n\n  // Renderer: live settings (antialias on, ACES, DPR 1).\n  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });\n  renderer.setSize(W, H);\n  renderer.toneMapping = THREE.ACESFilmicToneMapping;\n  document.body.appendChild(renderer.domElement);\n  const camera = new THREE.PerspectiveCamera(62, W / H, 0.5, 20000);\n  const sun = new THREE.DirectionalLight(0xfff2d8, 2.2);\n  sun.position.set(500, 800, 200);\n  scene.add(sun); scene.add(sun.target);\n  const hemi = new THREE.HemisphereLight(0xbfd9ff, 0x4a5a3a, 0.9);\n  scene.add(hemi);\n\n  // Flat base 300 m downhill of a 40-deg slope face (probe-crash-vfx B site).\n  let face = { x: 0, z: 0, g: 0, gx: 0, gz: 0 };\n  for (let x = -5000; x <= 5000; x += 120) {\n    for (let z = -5000; z <= 5000; z += 120) {\n      const gx = (heightAt(x + 60, z) - heightAt(x - 60, z)) / 120;\n      const gz = (heightAt(x, z + 60) - heightAt(x, z - 60)) / 120;\n      const g = Math.hypot(gx, gz);\n      if (g >= 0.45 && g <= 0.85 && g > face.g) face = { x, z, g, gx, gz };\n    }\n  }\n  const fgl = Math.hypot(face.gx, face.gz) || 1;\n  const fux = face.gx / fgl, fuz = face.gz / fgl;\n  const fhdg = Math.atan2(fux, fuz) * 180 / Math.PI;\n  const baseX = face.x - fux * 300, baseZ = face.z - fuz * 300;\n\n  /** Terrain patch mesh (600x600 m, live grass vertex-colour recipe, simplified noise). */\n  function buildTerrainPatch(cx: number, cz: number): THREE.Mesh {\n    const geo = new THREE.PlaneGeometry(600, 600, 100, 100);\n    geo.rotateX(-Math.PI / 2);\n    const pos = geo.getAttribute('position') as THREE.BufferAttribute;\n    const col = new Float32Array(pos.count * 3);\n    const cA = new THREE.Color(0x3d7a44);\n    const cB = new THREE.Color(0x8a7a4a);\n    const tmp = new THREE.Color();\n    for (let i = 0; i < pos.count; i++) {\n      const wx = cx + pos.getX(i);\n      const wz = cz + pos.getZ(i);\n      pos.setY(i, heightAt(wx, wz));\n      const n = 0.5 + 0.5 * Math.sin(wx * 0.021 + wz * 0.017) * Math.sin(wx * 0.008 - wz * 0.011 + 2.0);\n      tmp.copy(cA).lerp(cB, Math.max(0, Math.min(1, n * 1.4 - 0.3)));\n      col[i * 3] = tmp.r; col[i * 3 + 1] = tmp.g; col[i * 3 + 2] = tmp.b;\n    }\n    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));\n    geo.computeVertexNormals();\n    const mesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ vertexColors: true }));\n    mesh.position.set(cx, 0, cz);\n    return mesh;\n  }\n\n  const gl = renderer.getContext();\n  function readFrame(): Uint8Array {\n    const buf = new Uint8Array(W * H * 4);\n    gl.readPixels(0, 0, W, H, gl.RGBA, gl.UNSIGNED_BYTE, buf);\n    return buf;\n  }\n  function px(buf: Uint8Array, x: number, y: number): [number, number, number] {\n    // readPixels rows are bottom-up; flip y for screen coords.\n    const yy = H - 1 - y;\n    const i = (yy * W + x) * 4;\n    return [buf[i]!, buf[i + 1]!, buf[i + 2]!];\n  }\n  function diffMask(a: Uint8Array, b: Uint8Array, thresh = 25): Uint8Array {\n    // Returned mask is indexed by SCREEN rows (y = 0 is the top), so it\n    // lines up with projected screen rects and px(); the readPixels\n    // buffer itself is bottom-up, so buffer row yy = H-1-y.\n    const m = new Uint8Array(W * H);\n    for (let y = 0; y < H; y++) {\n      const yy = H - 1 - y;\n      for (let x = 0; x < W; x++) {\n        const i = (yy * W + x) * 4;\n        const d = Math.max(Math.abs(a[i]! - b[i]!), Math.abs(a[i + 1]! - b[i + 1]!), Math.abs(a[i + 2]! - b[i + 2]!));\n        if (d > thresh) m[y * W + x] = 1;\n      }\n    }\n    return m;\n  }\n  function maskStats(m: Uint8Array, full: Uint8Array): { count: number; minX: number; maxX: number; minY: number; maxY: number; meanR: number; meanG: number; meanB: number; meanLum: number; p25Lum: number; p90Lum: number } {\n    let count = 0, minX = W, maxX = -1, minY = H, maxY = -1, sR = 0, sG = 0, sB = 0;\n    const lums: number[] = [];\n    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {\n      if (!m[y * W + x]) continue;\n      count++;\n      if (x < minX) minX = x; if (x > maxX) maxX = x;\n      if (y < minY) minY = y; if (y > maxY) maxY = y;\n      const [r, g, b] = px(full, x, y);\n      sR += r; sG += g; sB += b;\n      lums.push(0.299 * r + 0.587 * g + 0.114 * b);\n    }\n    const meanR = count ? sR / count : 0, meanG = count ? sG / count : 0, meanB = count ? sB / count : 0;\n    // p25Lum: the darkest quarter of the mask — for the scorch the\n    // mean is dominated by its feathered rim, so the core's darkness\n    // (not the mean) is the discriminator. p90Lum: the brightest\n    // decile — for the wreck these are the sunlit top faces, whose\n    // brightness scales with the charred albedo and is stable\n    // against the random rest tilt (unlike the mean, which drops\n    // when a big roll turns the top faces away from the camera).\n    lums.sort((a, b) => a - b);\n    const p25Lum = count ? lums[Math.floor(0.25 * (count - 1))]! : 0;\n    const p90Lum = count ? lums[Math.floor(0.9 * (count - 1))]! : 0;\n    return { count, minX: count ? minX : 0, maxX: count ? maxX : 0, minY: count ? minY : 0, maxY: count ? maxY : 0, meanR, meanG, meanB, meanLum: 0.299 * meanR + 0.587 * meanG + 0.114 * meanB, p25Lum, p90Lum };\n  }\n  function project(p: THREE.Vector3): { x: number; y: number; behind: boolean } {\n    const v = p.clone().project(camera);\n    return { x: (v.x + 1) / 2 * W, y: (1 - v.y) / 2 * H, behind: v.z > 1 };\n  }\n\n  const onlyJet = new URLSearchParams(location.search).get('jet') as 'f22' | 'fa18' | null;\n  const jets = onlyJet ? [onlyJet] : (['f22', 'fa18'] as const);\n  for (const key of jets) {\n    origin.reset();\n    const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });\n    const spec = SPECS[key]!;\n    const attachedPlumes: THREE.Group[] = [];\n    const prevAlive = new Map<Aircraft, boolean>();\n    const handleDeaths = (all: Aircraft[]): void => {\n      for (const a of all) {\n        const was = prevAlive.get(a) ?? true;\n        if (was && !a.alive) {\n          vfx.spawnExplosion(a.pos.clone(), 1 + a.fuel);\n          vfx.scorchDecal(a.pos.clone());\n          if (!a.crashedFromDying) {\n            attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0xff7722, a.group));\n            attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0x1a1a1a, a.group));\n          }\n          for (let i = 0; i < 4; i++) {\n            const ang = Math.random() * Math.PI * 2;\n            const sp = 10 + Math.random() * 20;\n            vfx.spawnDebris(null, world, a.pos.clone(), new THREE.Vector3(Math.cos(ang) * sp, 5 + Math.random() * 10, Math.sin(ang) * sp));\n          }\n        }\n        prevAlive.set(a, a.alive);\n      }\n    };\n    const detachEvents: Array<{ seg: string | null }> = [];\n    const player = new Aircraft({\n      scene, world, spec, isPlayer: true, modelKey: spec.modelKey, deck,\n      events: {\n        onDetachSegment: (_a, segment, pos, vel) => {\n          detachEvents.push({ seg: segment ? segment.name || 'unnamed-node' : null });\n          if (segment) vfx.spawnDebris(segment, world, pos, vel);\n        },\n      },\n    });\n    player.terrainHeightAt = heightAt;\n    origin.registerBody(player.body);\n    const owners = new Map<number, Aircraft>();\n    for (const h of player.colliderHandles()) owners.set(h, player);\n    const drain = (): void => {\n      eventQueue.drainCollisionEvents((h1, h2, started) => {\n        if (!started) return;\n        const hit = owners.get(h1) ?? owners.get(h2);\n        if (!hit || !hit.alive) return;\n        hit.notifyContact(hit.vel.y, false);\n      });\n    };\n    await player.attachModel();\n    const modelStatus = player.modelStatus;\n    console.log(`[${key}] modelStatus=${modelStatus} tris=${player.modelTris}`);\n    if (modelStatus !== 'ok') throw new Error(`${key} model failed to load`);\n\n    // Stage the fast hard impact (probe-crash-vfx C2n staging).\n    const by = heightAt(baseX, baseZ) + 10;\n    player.resetAt(baseX, by, baseZ, fhdg);\n    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((8 * Math.PI) / 180, (fhdg * Math.PI) / 180, 0, 'YXZ'));\n    player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);\n    const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);\n    player.body.setLinvel({ x: f.x * 85, y: -16, z: f.z * 85 }, true);\n    player.vel.set(f.x * 85, -16, f.z * 85);\n    player.gear = true;\n    player.grounded = false;\n    player.thrTarget = 0.6;\n    player.throttle = 0.6;\n    player.pitchIn = 0;\n    let simT = 0;\n    let crashedAt = -1;\n    let crashQuat = new THREE.Quaternion();\n    while (simT < 6) {\n      liveFrame((dt) => {\n        simT += dt;\n        world.timestep = dt;\n        world.step(eventQueue);\n        drain();\n        player.step(dt, WIND, null);\n        vfx.update(dt);\n        handleDeaths([player]);\n        if (!player.alive && crashedAt < 0) { crashedAt = simT; crashQuat = player.quat.clone(); }\n      });\n      if (crashedAt >= 0) break;\n    }\n    console.log(`[${key}] crashed at simT=${crashedAt.toFixed(2)} reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(1)} pos=(${player.pos.x.toFixed(0)},${player.pos.y.toFixed(1)},${player.pos.z.toFixed(0)}) detach=${JSON.stringify(detachEvents)}`);\n    if (crashedAt < 0) throw new Error(`${key} did not crash`);\n\n    const wreckPos = player.pos.clone();\n    const crashFwd = new THREE.Vector3(0, 0, 1).applyQuaternion(crashQuat);\n    // Wreck tilt: group quaternion vs physics quat (wreckModel multiplies a roll tilt).\n    const tiltQ = player.quat.clone().invert().multiply(player.group.quaternion);\n    const tiltE = new THREE.Euler().setFromQuaternion(tiltQ, 'YXZ');\n    const wreckTiltDeg = Math.abs(tiltE.z) * 180 / Math.PI;\n    let airframeColorHex = '';\n    const model = (player as unknown as { model: THREE.Group | null }).model;\n    if (model) {\n      model.traverse((o) => {\n        const m = o as THREE.Mesh;\n        if (m.isMesh && (m.name === 'ground' || m.name === 'F-22-airframe')) {\n          const mats = Array.isArray(m.material) ? m.material : [m.material];\n          airframeColorHex = (mats[0] as THREE.MeshStandardMaterial).color?.getHexString() ?? '';\n        }\n      });\n    }\n    const patch = buildTerrainPatch(wreckPos.x, wreckPos.z);\n    scene.add(patch);\n\n    const V = vfx as unknown as { debris: DebrisRec[]; scorches: ScorchRec[]; plumes: PlumeRec[] };\n\n    function sampleFrame(sample: number, camName: 'chase' | 'spectate'): void {\n      if (camName === 'chase') {\n        const want = chaseCameraPosition(wreckPos, crashFwd, new THREE.Vector3(0, 1, 0), 42, heightAt, null);\n        camera.position.copy(want);\n        camera.lookAt(wreckPos);\n      } else {\n        const pos = orbitCameraPosition(wreckPos, { yaw: 0, pitch: 0.32 }, 30);\n        applyCameraFloor(pos, wreckPos, heightAt);\n        camera.position.copy(pos);\n        camera.lookAt(wreckPos);\n      }\n      camera.updateProjectionMatrix();\n      // Full render (metrics + evidence frame).\n      renderer.render(scene, camera);\n      const full = readFrame();\n      const frameName = `${key}_t${sample}_${camName}`;\n      (window as unknown as { __frames: Record<string, string> }).__frames[frameName] = renderer.domElement.toDataURL('image/png');\n      // Wreck-only render: the attached fire/smoke plume is a child\n      // of the wreck group, so the wreck mask includes its pixels —\n      // the flame's orange halo and the dark smoke confounded the\n      // wreck's own mean luminance (the halo pulled it up to 43-47\n      // even with a #2c2c2c charred airframe). Hide the plume groups\n      // for this render so the wreck mask is pure wreck.\n      const plumeGroups = V.plumes.map((p) => p.group);\n      plumeGroups.forEach((g) => { g.visible = false; });\n      renderer.render(scene, camera);\n      const wreckOnly = readFrame();\n      plumeGroups.forEach((g) => { g.visible = true; });\n      // Ablations.\n      const wreckGroup = player.group;\n      const wreckWasVisible = wreckGroup.visible;\n      wreckGroup.visible = false;\n      renderer.render(scene, camera);\n      const noWreck = readFrame();\n      wreckGroup.visible = wreckWasVisible;\n      const debrisMeshes = V.debris.map((d) => d.mesh);\n      const debrisVis = debrisMeshes.map((m) => m.visible);\n      debrisMeshes.forEach((m) => { m.visible = false; });\n      renderer.render(scene, camera);\n      const noDebris = readFrame();\n      debrisMeshes.forEach((m, i) => { m.visible = debrisVis[i]!; });\n      const scorchMeshes = V.scorches.map((s) => s.mesh);\n      const scorchVis = scorchMeshes.map((m) => m.visible);\n      scorchMeshes.forEach((m) => { m.visible = false; });\n      renderer.render(scene, camera);\n      const noScorch = readFrame();\n      scorchMeshes.forEach((m, i) => { m.visible = scorchVis[i]!; });\n\n      const wreckMask = diffMask(wreckOnly, noWreck);\n      const debrisMask = diffMask(full, noDebris);\n      const scorchMask = diffMask(full, noScorch);\n      // Pure wreck: the mask comes from the plume-free render, so\n      // its mean luminance is the wreck's own char, unconfounded.\n      const ws = maskStats(wreckMask, wreckOnly);\n      const ds = maskStats(debrisMask, full);\n      const ss = maskStats(scorchMask, full);\n      // Fire: orange pixels in the full frame.\n      let firePx = 0, fireBrightPx = 0;\n      for (let i = 0; i < W * H; i++) {\n        const r = full[i * 4]!, g = full[i * 4 + 1]!, b = full[i * 4 + 2]!;\n        if (r > 150 && r - g > 45 && g - b > 10) firePx++;\n        if (r > 200 && g > 70 && g < 190 && b < 110) fireBrightPx++;\n      }\n      // Scorch visible outside the wreck silhouette.\n      let scorchOutside = 0;\n      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {\n        if (scorchMask[y * W + x] && !wreckMask[y * W + x]) scorchOutside++;\n      }\n      // Per-piece rendered shape.\n      const pieces: PieceRender[] = [];\n      let debrisBelowGround = 0;\n      for (const d of V.debris) {\n        const p = d.body.translation();\n        const gh = heightAt(p.x, p.z);\n        if (p.y < gh - 0.5) debrisBelowGround++;\n        let label = 'generic';\n        d.mesh.traverse((o) => { if (o.name === 'torn_wing' || o.name === 'torn_tail') label = o.name; });\n        const bb = new THREE.Box3().setFromObject(d.mesh);\n        const corners: THREE.Vector3[] = [];\n        for (const cx of [bb.min.x, bb.max.x]) for (const cy of [bb.min.y, bb.max.y]) for (const cz of [bb.min.z, bb.max.z]) corners.push(new THREE.Vector3(cx, cy, cz));\n        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;\n        for (const c of corners) {\n          const s = project(c);\n          if (s.behind) continue;\n          minX = Math.min(minX, s.x); maxX = Math.max(maxX, s.x);\n          minY = Math.min(minY, s.y); maxY = Math.max(maxY, s.y);\n        }\n        if (!isFinite(minX)) { pieces.push({ label, rectW: 0, rectH: 0, rectArea: 0, darkPx: 0, fill: 0, lumP10: 0, lumP50: 0, lumP90: 0, lumRange: 0, centreAboveGround: p.y - gh, restTiltDeg: 0 }); continue; }\n        const rx0 = Math.max(0, Math.floor(minX)), rx1 = Math.min(W - 1, Math.ceil(maxX));\n        const ry0 = Math.max(0, Math.floor(minY)), ry1 = Math.min(H - 1, Math.ceil(maxY));\n        const rectW = Math.max(1, rx1 - rx0 + 1), rectH = Math.max(1, ry1 - ry0 + 1);\n        let darkPx = 0;\n        const lums: number[] = [];\n        for (let y = ry0; y <= ry1; y++) for (let x = rx0; x <= rx1; x++) {\n          if (!debrisMask[y * W + x]) continue;\n          darkPx++;\n          const [r, g, b] = px(full, x, y);\n          lums.push(0.299 * r + 0.587 * g + 0.114 * b);\n        }\n        lums.sort((a, b) => a - b);\n        const qAt = (q: number): number => lums.length ? lums[Math.min(lums.length - 1, Math.floor(q * lums.length))]! : 0;\n        // Rest tilt: angle of the piece's thin (local Y) axis vs world up.\n        let target: THREE.Mesh | null = null;\n        d.mesh.traverse((o) => { if (!target && (o as THREE.Mesh).isMesh) target = o as THREE.Mesh; });\n        let restTiltDeg = 0;\n        if (target) {\n          const qq = new THREE.Quaternion();\n          (target as THREE.Mesh).getWorldQuaternion(qq);\n          const thinWorld = new THREE.Vector3(0, 1, 0).applyQuaternion(qq);\n          restTiltDeg = Math.acos(Math.min(1, Math.abs(thinWorld.y))) * 180 / Math.PI;\n        }\n        pieces.push({\n          label, rectW, rectH, rectArea: rectW * rectH, darkPx,\n          fill: darkPx / (rectW * rectH),\n          lumP10: qAt(0.1), lumP50: qAt(0.5), lumP90: qAt(0.9), lumRange: qAt(0.9) - qAt(0.1),\n          centreAboveGround: p.y - gh, restTiltDeg,\n        });\n      }\n      let attachedPlumes = 0, plumeMaxOpacity = 0;\n      for (const pl of V.plumes) {\n        if (pl.parent) {\n          attachedPlumes++;\n          for (const c of pl.group.children) plumeMaxOpacity = Math.max(plumeMaxOpacity, ((c as THREE.Sprite).material as THREE.SpriteMaterial).opacity);\n        }\n      }\n      const wreckBboxArea = Math.max(1, (ws.maxX - ws.minX + 1) * (ws.maxY - ws.minY + 1));\n      results.push({\n        jet: key, sample, cam: camName,\n        wreckPx: ws.count, wreckBboxW: ws.maxX - ws.minX + 1, wreckBboxH: ws.maxY - ws.minY + 1,\n        wreckFill: ws.count / wreckBboxArea,\n        wreckMeanLum: ws.meanLum, wreckP90Lum: ws.p90Lum, wreckMeanR: ws.meanR, wreckMeanG: ws.meanG, wreckMeanB: ws.meanB,\n        wreckSpanPx: ws.maxX - ws.minX + 1, wreckLenPx: ws.maxY - ws.minY + 1,\n        debrisPx: ds.count, debrisCount: V.debris.length,\n        scorchPx: ss.count, scorchMeanLum: ss.meanLum, scorchP25Lum: ss.p25Lum, scorchOutsideWreckPx: scorchOutside,\n        firePx, fireBrightPx,\n        pieces, attachedPlumes, plumeMaxOpacity,\n        debrisBelowGround, wreckTiltDeg, airframeColorHex,\n      });\n      console.log(`[${key} t+${sample}s ${camName}] wreck=${ws.count}px ${ws.maxX - ws.minX + 1}x${ws.maxY - ws.minY + 1} fill=${(ws.count / wreckBboxArea).toFixed(2)} lum=${ws.meanLum.toFixed(0)} debris=${ds.count}px n=${V.debris.length} scorch=${ss.count}px lum=${ss.meanLum.toFixed(0)} outsideWreck=${scorchOutside} fire=${firePx}/${fireBrightPx} plumes=${attachedPlumes} op=${plumeMaxOpacity.toFixed(2)} tilt=${wreckTiltDeg.toFixed(1)}deg airframe=#${airframeColorHex}`);\n      for (const pc of pieces) {\n        console.log(`    piece ${pc.label}: rect=${pc.rectW}x${pc.rectH} fill=${pc.fill.toFixed(2)} lum p10=${pc.lumP10.toFixed(0)} p50=${pc.lumP50.toFixed(0)} p90=${pc.lumP90.toFixed(0)} range=${pc.lumRange.toFixed(0)} aboveGround=${pc.centreAboveGround.toFixed(2)}m tilt=${pc.restTiltDeg.toFixed(1)}deg`);\n      }\n    }\n\n    // Step from impact to +30 s sim, then +75 s sim, sampling both cameras.\n    for (const sample of [30, 75]) {\n      const target = crashedAt + sample;\n      while (simT < target) {\n        liveFrame((dt) => {\n          simT += dt;\n          world.timestep = dt;\n          world.step(eventQueue);\n          drain();\n          vfx.update(dt);\n        });\n      }\n      console.log(`[${key}] sampled at simT=${simT.toFixed(2)} (impact +${(simT - crashedAt).toFixed(1)}s) debris=${V.debris.length} scorches=${V.scorches.length} plumes=${V.plumes.length}`);\n      sampleFrame(sample, 'chase');\n      sampleFrame(sample, 'spectate');\n    }\n    scene.remove(patch);\n    patch.geometry.dispose();\n    (patch.material as THREE.Material).dispose();\n    origin.unregisterBody(player.body);\n    player.dispose();\n    // Clean VFX residue out of the shared world/scene.\n    const VV = vfx as unknown as { debris: DebrisRec[]; scorches: ScorchRec[]; plumes: PlumeRec[]; booms: BoomRec[] };\n    for (const d of VV.debris) { scene.remove(d.mesh); origin.unregisterBody(d.body); world.removeRigidBody(d.body); }\n    VV.debris.length = 0;\n    for (const s of VV.scorches) { scene.remove(s.mesh); origin.unregisterObject(s.mesh); }\n    VV.scorches.length = 0;\n    for (const pl of VV.plumes) { if (pl.parent) pl.parent.remove(pl.group); else { scene.remove(pl.group); origin.unregisterObject(pl.group); } }\n    VV.plumes.length = 0;\n    for (const b of VV.booms) { scene.remove(b.sprite); origin.unregisterObject(b.sprite); }\n    VV.booms.length = 0;\n  }\n\n  (window as unknown as { __result: FrameMetrics[] }).__result = results;\n  (window as unknown as { __done: boolean }).__done = true;\n  console.log('HARNESS DONE');\n}\n\n(window as unknown as { __frames: Record<string, string> }).__frames = {};\nvoid main().catch((e) => {\n  console.error('HARNESS ERROR', e);\n  (window as unknown as { __error: string }).__error = String(e?.stack ?? e);\n});\n";

interface PieceRender {
  label: string;
  rectW: number; rectH: number; rectArea: number;
  darkPx: number; fill: number;
  lumP10: number; lumP50: number; lumP90: number; lumRange: number;
  centreAboveGround: number; restTiltDeg: number;
}
interface FrameMetrics {
  jet: string; sample: number; cam: string;
  wreckPx: number; wreckBboxW: number; wreckBboxH: number; wreckFill: number;
  wreckMeanLum: number; wreckP90Lum: number; wreckMeanR: number; wreckMeanG: number; wreckMeanB: number;
  wreckSpanPx: number; wreckLenPx: number;
  debrisPx: number; debrisCount: number;
  scorchPx: number; scorchMeanLum: number; scorchP25Lum: number; scorchOutsideWreckPx: number;
  firePx: number; fireBrightPx: number;
  pieces: PieceRender[];
  attachedPlumes: number; plumeMaxOpacity: number;
  debrisBelowGround: number;
  wreckTiltDeg: number; airframeColorHex: string;
}

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  const all: FrameMetrics[] = [];
  let harnessError: string | null = null;
  fs.mkdirSync(TMPDIR, { recursive: true });
  // The harness imports '../src/...' and the bare 'three' /
  // '@dimforge/rapier3d-compat' specifiers; it is bundled from /tmp,
  // so rewrite the relative imports to absolute project paths and
  // the two bare specifiers to the project's node_modules dirs
  // (a --alias for 'three' would also rewrite the 'three/addons/...'
  // imports inside src/ and break their exports mapping).
  const harnessSrc = HARNESS_SRC
    .replaceAll(`from '../src/`, `from '${PROJ}/src/`)
    .replaceAll(`from 'three'`, `from '${PROJ}/node_modules/three'`)
    .replaceAll(`from '@dimforge/rapier3d-compat'`, `from '${PROJ}/node_modules/@dimforge/rapier3d-compat'`);
  fs.writeFileSync(`${TMPDIR}/harness.ts`, harnessSrc);
  fs.writeFileSync(
    `${TMPDIR}/index.html`,
    '<!DOCTYPE html><html><head><meta charset="utf-8"><style>body{margin:0}</style></head><body><script type="module" src="/harness.mjs"></script></body></html>',
  );
  try {
    execFileSync(
      `${PROJ}/node_modules/.bin/esbuild`,
      [
        `${TMPDIR}/harness.ts`, '--bundle', '--platform=browser', '--format=esm',
        `--outfile=${TMPDIR}/harness.mjs`,
      ],
      { stdio: 'pipe' },
    );
  } catch (e) {
    const err = e as { stdout?: Buffer; stderr?: Buffer };
    console.log(`FAIL  harness bundle failed — ${err.stderr?.toString() ?? err.stdout?.toString() ?? String(e)}`);
    failures++;
    console.log(`\n${failures} CHECK(S) FAILED`);
    process.exit(1);
  }
  const server = http.createServer((rq, rs) => {
    const p = (rq.url ?? '/').split('?')[0];
    const fp = path.join(TMPDIR, p === '/' ? '/index.html' : p);
    if (!fs.existsSync(fp)) { rs.writeHead(404); rs.end('nf'); return; }
    rs.writeHead(200, { 'Content-Type': p.endsWith('.mjs') ? 'text/javascript' : 'text/html' });
    fs.createReadStream(fp).pipe(rs);
  });
  await new Promise<void>((r) => server.listen(0, '127.0.0.1', () => r()));
  const port = (server.address() as { port: number }).port;
  const req = createRequire(`${HOME}/workspace/probe-helicarriers/package.json`);
  const { chromium } = req('playwright-core') as typeof import('playwright-core');
  const exe = `${HOME}/.cache/ms-playwright/chromium-1255/chrome-linux64/chrome`;
  if (!fs.existsSync(exe)) {
    console.log(`FAIL  headless Chromium not found at ${exe} — the rendered-frame probe cannot run`);
    failures++;
    server.close();
    console.log(`\n${failures} CHECK(S) FAILED`);
    process.exit(1);
  }
  let browser: Awaited<ReturnType<typeof chromium.launch>> | null = null;
  try {
    // A fresh browser per jet: SwiftShader headless Chromium dies
    // mid-run (renderer crash, second jet's first sample) if one
    // browser serves both jets, and --disable-dev-shm-usage avoids
    // the /dev/shm exhaustion that kills the renderer in containers.
    for (const jet of ['f22', 'fa18']) {
      browser = await chromium.launch({
        executablePath: exe,
        headless: true,
        args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--no-sandbox', '--disable-gpu-vsync', '--disable-dev-shm-usage'],
      });
      const page = await browser.newPage({ viewport: { width: 900, height: 560 } });
      page.on('console', (m) => console.log(`[page ${jet}]`, m.type(), m.text()));
      page.on('pageerror', (e) => console.log(`[pageerror ${jet}]`, e.message));
      await page.goto(`http://127.0.0.1:${port}/index.html?jet=${jet}`, { waitUntil: 'domcontentloaded' });
      try {
        await page.waitForFunction(() => (window as unknown as { __done?: boolean; __error?: string }).__done || (window as unknown as { __error?: string }).__error, null, { timeout: 300000 });
      } catch (e) {
        harnessError = `harness timeout (${jet}): ${String(e)}`;
        await page.close();
        await browser.close();
        browser = null;
        break;
      }
      const err = await page.evaluate(() => (window as unknown as { __error?: string }).__error ?? null).catch(() => 'page-closed');
      if (err) { harnessError = `harness error (${jet}): ${err}`; await page.close(); await browser.close(); browser = null; break; }
      const result = await page.evaluate(() => (window as unknown as { __result?: FrameMetrics[] }).__result ?? null).catch(() => null);
      if (result) all.push(...result);
      // Evidence frames, only when the caller asks (the suite does
      // not set this; a human verification run does).
      if (process.env.P17_SAVE_FRAMES === '1') {
        const frames = await page.evaluate(() => (window as unknown as { __frames?: Record<string, string> }).__frames ?? {}).catch(() => ({}));
        const outDir = `${HOME}/workspace/pass17-debris-frames`;
        fs.mkdirSync(outDir, { recursive: true });
        for (const [name, url] of Object.entries(frames)) {
          fs.writeFileSync(`${outDir}/${name}.png`, Buffer.from(url.split(',')[1], 'base64'));
          console.log('FRAME', name);
        }
      }
      await page.close();
      await browser.close();
      browser = null;
    }
  } finally {
    await browser?.close();
    server.close();
    fs.rmSync(TMPDIR, { recursive: true, force: true });
  }
  if (harnessError) {
    console.log(`FAIL  ${harnessError}`);
    failures++;
    console.log(`\n${failures} CHECK(S) FAILED`);
    process.exit(1);
  }
  const at = (jet: string, sample: number, cam: string): FrameMetrics | undefined =>
    all.find((f) => f.jet === jet && f.sample === sample && f.cam === cam);
  check('rendered frames collected for both jets at +30 s and +75 s (chase + spectate)', all.length === 8, `frames=${all.length} (jets: ${[...new Set(all.map((f) => f.jet))].join(',')})`);
  if (all.length !== 8) {
    console.log(`\n${failures} CHECK(S) FAILED`);
    process.exit(1);
  }
  for (const jet of ['f22', 'fa18']) {
    for (const sample of [30, 75]) {
      const chase = at(jet, sample, 'chase')!;
      const spec = at(jet, sample, 'spectate')!;
      console.log(
        `  [${jet} t+${sample}s] chase wreck=${chase.wreckPx}px lum=${chase.wreckMeanLum.toFixed(0)} debris=${chase.debrisPx}px scorch=${chase.scorchPx}px lum=${chase.scorchMeanLum.toFixed(0)} fire=${chase.firePx}/${chase.fireBrightPx} | ` +
        `spectate wreck=${spec.wreckPx}px lum=${spec.wreckMeanLum.toFixed(0)} debris=${spec.debrisPx}px scorch=${spec.scorchPx}px fire=${spec.firePx}/${spec.fireBrightPx}`,
      );
      // A. Debris must not read as black slabs: the VISIBLE pieces
      // (darkPx >= 100 — a piece occluded behind the wreck/fire
      // shows only a shadowed sliver and does not count) are brown,
      // not near-black. Pre-fix the generic pieces render at
      // luminance 3-9 (pure black silhouettes); post-fix 15-58
      // (charred brown-grey). One post-fix piece can render as low
      // as 15 (darkest char colour + shadowed fold orientation), so
      // the check is on the MEDIAN of the visible pieces, not every
      // piece: pre-fix medians are 6-9, post-fix medians 24-41.
      // At least 2 pieces must be visible, or the check would pass
      // vacuously; at least one piece must clearly read brown.
      const visible = chase.pieces.filter((p) => p.darkPx >= 100);
      const p50s = visible.map((p) => p.lumP50).sort((a, b) => a - b);
      const medianP50 = p50s.length ? p50s[Math.floor(p50s.length / 2)]! : -1;
      const maxP50 = p50s.length ? p50s[p50s.length - 1]! : -1;
      check(
        `${jet} t+${sample}s chase: debris reads as torn brown wreckage, not black slabs (>=2 pieces visible, median piece luminance >= 20, at least one >= 25)`,
        visible.length >= 2 && medianP50 >= 20 && maxP50 >= 25,
        `visible=${visible.length}/6 medianP50=${medianP50.toFixed(0)} maxP50=${maxP50.toFixed(0)} pieces=[${chase.pieces.map((p) => `${p.label}:px${p.darkPx}/p50=${p.lumP50.toFixed(0)}`).join(' ')}]`,
      );
      // B. The scorch must read as a dark burn mark from the chase
      // camera (not a pale stain, not hidden under the wreck) and
      // from spectate, at both sample times. Pre-fix: 0 px chase at
      // +30 s, 0 px at +75 s (pale stain, diffs 12-22 vs grass).
      // Darkness is judged by the darkest quarter of the mask
      // (p25): the mean is dominated by the decal's feathered rim
      // and inverts (post-fix mean 104 vs pre-fix mean 71 with the
      // old texture). The threshold 70 sits between the pre-fix
      // p25 (88-103) and the post-fix p25 (57-59).
      check(
        `${jet} t+${sample}s: scorch reads as a dark mark from chase and spectate (>= 300 px, darkest-quarter luminance <= 70)`,
        chase.scorchPx >= 300 && chase.scorchP25Lum <= 70 && spec.scorchPx >= 300 && spec.scorchP25Lum <= 70,
        `chase=${chase.scorchPx}px p25=${chase.scorchP25Lum.toFixed(0)} spectate=${spec.scorchPx}px p25=${spec.scorchP25Lum.toFixed(0)}`,
      );
      // C. The wreck fire must burn BRIGHT (a flame, not a glow or a
      // wash) in both views at both sample times. Pre-fix: 0
      // bright-orange pixels in either view (soft wash only).
      check(
        `${jet} t+${sample}s: wreck fire burns bright in chase and spectate (>= 100 bright-orange px, plume attached at full opacity)`,
        chase.fireBrightPx >= 100 && spec.fireBrightPx >= 100 && chase.attachedPlumes >= 1 && chase.plumeMaxOpacity >= 0.4,
        `chase bright=${chase.fireBrightPx} spectate bright=${spec.fireBrightPx} attached=${chase.attachedPlumes} op=${chase.plumeMaxOpacity.toFixed(2)}`,
      );
      // D/E. Regression guards: the debris field is still there, and
      // nothing sank through the terrain.
      check(
        `${jet} t+${sample}s: debris field present in the chase frame (>= 300 px, >= 4 pieces)`,
        chase.debrisPx >= 300 && chase.debrisCount >= 4,
        `debrisPx=${chase.debrisPx} count=${chase.debrisCount}`,
      );
      check(
        `${jet} t+${sample}s: no debris piece under the terrain`,
        chase.debrisBelowGround === 0,
        `below=${chase.debrisBelowGround}`,
      );
      // F. The wreck itself must read as CHARRED, not factory grey.
      // REQUIRES the FlightPhysicsController patch (see the header):
      // wreckModel's x0.22 runs in linear space, so the airframe
      // renders #818181. The discrimination is the SPECTATE view's
      // brightest decile (p90) — the sunlit top faces, whose
      // brightness scales with the charred albedo and is stable
      // against the random rest tilt (unlike the mean, which drops
      // to 11 unpatched when a big roll turns the top faces away).
      // Measured: unpatched p90 30-38, patched (x0.025) p90 8-11;
      // the threshold 20 sits between them. Without the patch this
      // check fails and the probe fails.
      check(
        `${jet} t+${sample}s spectate: wreck reads as charred, not factory grey (sunlit-top p90 luminance <= 20; REQUIRES the FlightPhysicsController wreckModel patch)`,
        spec.wreckP90Lum <= 20,
        `spectateWreckP90=${spec.wreckP90Lum.toFixed(0)} (mean=${spec.wreckMeanLum.toFixed(0)}) airframe=#${chase.airframeColorHex}`,
      );
    }
    // G. The wreck rest tilt is present (roll tilt from wreckModel).
    const tilt = at(jet, 30, 'chase')!.wreckTiltDeg;
    check(
      `${jet}: wreck rests tilted, not level like a parked jet (tilt >= 8 deg)`,
      tilt >= 8,
      `tilt=${tilt.toFixed(1)}deg`,
    );
  }
  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(1);
});
