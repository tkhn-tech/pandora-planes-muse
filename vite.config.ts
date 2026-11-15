import { defineConfig } from 'vite';

export default defineConfig({
  // GitHub Pages project site: https://tkhn-tech.github.io/pandora-planes-muse/
  base: '/pandora-planes-muse/',
  build: {
    outDir: 'dist',
    // es2022: top-level await (RAPIER.init() + world build) needs it;
    // es2020 cannot compile top-level await.
    target: 'es2022',
  },
});
