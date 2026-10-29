/// <reference types="vitest/config" />
import { cpSync, createReadStream, existsSync, statSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { defineConfig, type Plugin } from 'vite';
import react from '@vitejs/plugin-react';

// The in-browser compiler (@yowasp/clang) loads its 100 MB of WebAssembly and sysroot
// relative to its own bundle. Serve that folder untouched at /clang/ instead of letting
// Vite bundle it, and copy it next to the build output.
const ROOT = import.meta.dirname;
const CLANG_DIR = resolve(ROOT, 'node_modules/@yowasp/clang/gen');

function serveClang(): Plugin {
  return {
    name: 'serve-yowasp-clang',
    configureServer(server) {
      server.middlewares.use('/clang/', (req, res, next) => {
        const name = decodeURIComponent((req.url ?? '').split('?')[0]).replace(/^\/+/, '');
        const file = join(CLANG_DIR, name);
        if (!name || name.includes('..') || !existsSync(file) || !statSync(file).isFile()) return next();
        const type = name.endsWith('.wasm') ? 'application/wasm' : name.endsWith('.js') ? 'text/javascript' : 'application/octet-stream';
        res.setHeader('Content-Type', type);
        res.setHeader('Content-Length', statSync(file).size);
        res.setHeader('Cache-Control', 'public, max-age=86400');
        createReadStream(file).pipe(res);
      });
    },
    closeBundle() {
      const out = resolve(ROOT, 'dist/clang');
      if (existsSync(resolve(ROOT, 'dist'))) cpSync(CLANG_DIR, out, { recursive: true });
    },
  };
}

export default defineConfig({
  plugins: [react(), serveClang()],
  server: {
    port: 5173,
    // Allow importing the robot's own sketch from the repository root
    fs: { allow: [resolve(ROOT, '..')] },
  },
  worker: { format: 'es' },
  build: { target: 'es2022', chunkSizeWarningLimit: 2000 },
  test: {
    testTimeout: 600_000,
    hookTimeout: 600_000,
    pool: 'forks',
  },
});
