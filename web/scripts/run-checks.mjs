#!/usr/bin/env node
/**
 * The numerical self-check suite: bundle checks/run.ts with esbuild and run it under
 * Node against the compiled WebAssembly module. Requires `npm run wasm` first.
 */
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { build } from 'esbuild';

const WEB = resolve(dirname(fileURLToPath(import.meta.url)), '..');
if (!existsSync(join(WEB, 'public', 'wasm', 'robotics_wasm_bg.wasm'))) {
  console.error('The WebAssembly module is not built. Run `npm run wasm` first.');
  process.exit(1);
}
const outfile = join(WEB, '.checks', 'run.mjs');
mkdirSync(dirname(outfile), { recursive: true });
await build({
  entryPoints: [join(WEB, 'checks', 'run.ts')],
  bundle: true,
  platform: 'node',
  format: 'esm',
  outfile,
  logLevel: 'error',
  // katex is CommonJS-with-ESM-wrapper; keep it external so Node resolves it normally.
  external: ['katex'],
});
const r = spawnSync(process.execPath, [outfile], { stdio: 'inherit', cwd: WEB });
process.exit(r.status ?? 1);
