# Pandora Planes — Vite + TypeScript + Rapier3D

Physics-driven open-world sandbox flight sim in the browser: dogfight MiGs
over floating islands, trap on the flying helicarrier, hover the F-35A
down onto a VTOL pad. X-Plane 9 Mobile style — stick, throttle, tapes.

This is the v3.0 port of the single-file HTML v2.0 build
(`../pandora-planes-html/`), rebuilt as a proper Vite + TypeScript project
with Rapier3D physics, so the game can grow past the old 5 MB single-file
ceiling. The start menu + HUD DOM shell and CSS in `index.html` are ported
verbatim from the v2 template — every element id matches the original
`$()` lookups.

Live site: https://tkhn-tech.github.io/pandora-planes-muse/

## Develop

```sh
npm install
npm run dev      # Vite dev server
npm run build    # tsc --noEmit && vite build  → dist/
npm run preview  # serve the production build
```

## Models — why they are base64 in .ts files

The GitHub connection can only push **text** files, so binary `.glb` files
cannot be committed to the repo. Each of the 7 real aircraft/carrier
models is therefore embedded as a `data:model/gltf-binary;base64,...`
string in a TypeScript module under `src/assets/models/`:

| module | model | tris |
|---|---|---|
| `fa18.ts` | F/A-18F Super Hornet (Yi Tsung Lee, CC-BY) | 17,517 |
| `mig29.ts` | MiG-29 (SIpriv, CC-BY) | 5,931 |
| `su34.ts` | Su-34 (Sketchfab free, downsized) | 15,903 |
| `ford.ts` | Gerald Ford helicarrier (Usman Zia, CC-BY) | 2,678 |
| `f22.ts` | F-22 Raptor | 27,116 |
| `f15e.ts` | F-15E Strike Eagle | 24,147 |
| `f35a.ts` | F-35A VTOL | 4,601 |

Re-generate them any time from the verified sources with:

```sh
npm run embed-models   # node scripts/embed_models.mjs
```

The script re-extracts F-18 / MiG-29 / Ford byte-identical from the v2
single-file build and reads Su-34 / F-22 / F-15E / F-35A from the `.glb`
files, then verifies each one: `glTF` magic + version 2, JSON chunk
parses, mesh/primitive/triangle counts match the build reports, and
SHA-256 matches the source file. It exits non-zero on any mismatch.

## Deploy

Pushes to `main` deploy automatically via `.github/workflows/deploy.yml`
(checkout → Node 20 → `npm ci` → `npm run build` → GitHub Pages).

**One-time repo step (already done by Nicholas, 16 Nov 2026):**
Settings → Pages → Source: **GitHub Actions**. The first Actions deploy
goes live once the workflow is pushed to `main`.
