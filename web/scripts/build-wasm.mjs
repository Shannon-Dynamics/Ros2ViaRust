#!/usr/bin/env node
/**
 * Compile crates/robotics_wasm to WebAssembly and generate its JavaScript glue.
 *
 *   cargo build --release --target wasm32-unknown-unknown --locked
 *   wasm-bindgen --target web  →  lib/wasm/pkg/        (typed glue, imported by lib/wasm.ts)
 *   robotics_wasm_bg.wasm      →  public/wasm/         (what the browser fetches)
 *
 * The wasm-bindgen CLI must be exactly the version of the `wasm-bindgen` crate in the
 * crate's Cargo.lock; this script checks, because a mismatch produces a module that fails
 * at instantiation with an unhelpful message.
 *
 * Env: ROBOTICS_WASM_CRATE  path to the crate (default ../crates/robotics_wasm). Needed when
 *      web/ is built from a copy that does not sit next to crates/ (the WSL mirror).
 *      CARGO_TARGET_DIR     honoured as usual.
 */
import { spawnSync } from 'node:child_process';
import { copyFileSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const WEB = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const CRATE = resolve(process.env.ROBOTICS_WASM_CRATE ?? join(WEB, '..', 'crates', 'robotics_wasm'));
const OUT = join(WEB, 'lib', 'wasm', 'pkg');
const PUBLIC = join(WEB, 'public', 'wasm');

function run(cmd, args, opts = {}) {
  const r = spawnSync(cmd, args, { stdio: 'inherit', ...opts });
  if (r.status !== 0) {
    console.error(`\n${cmd} ${args.join(' ')} failed (exit ${r.status ?? r.signal})`);
    process.exit(r.status ?? 1);
  }
}

if (!existsSync(join(CRATE, 'Cargo.toml'))) {
  console.error(`robotics_wasm crate not found at ${CRATE}. Set ROBOTICS_WASM_CRATE.`);
  process.exit(1);
}

// Version match between the CLI and the crate's locked wasm-bindgen.
const lock = readFileSync(join(CRATE, 'Cargo.lock'), 'utf8');
const locked = lock.match(/name = "wasm-bindgen"\nversion = "([^"]+)"/)?.[1];
const cli = spawnSync('wasm-bindgen', ['--version'], { encoding: 'utf8' });
if (cli.error || cli.status !== 0) {
  console.error(
    `wasm-bindgen CLI not found. Install the matching version:\n` +
      `  cargo install wasm-bindgen-cli --version ${locked} --locked`,
  );
  process.exit(1);
}
const have = cli.stdout.match(/wasm-bindgen (\S+)/)?.[1];
if (have !== locked) {
  console.error(`wasm-bindgen CLI ${have} does not match the crate's locked wasm-bindgen ${locked}.`);
  process.exit(1);
}

const targetDir =
  process.env.CARGO_TARGET_DIR ?? join(CRATE, 'target');
run('cargo', ['build', '--release', '--target', 'wasm32-unknown-unknown', '--locked'], {
  cwd: CRATE,
  env: { ...process.env, CARGO_TARGET_DIR: targetDir },
});

mkdirSync(OUT, { recursive: true });
mkdirSync(PUBLIC, { recursive: true });
run('wasm-bindgen', [
  '--target', 'web',
  '--out-dir', OUT,
  '--out-name', 'robotics_wasm',
  join(targetDir, 'wasm32-unknown-unknown', 'release', 'robotics_wasm.wasm'),
]);
copyFileSync(join(OUT, 'robotics_wasm_bg.wasm'), join(PUBLIC, 'robotics_wasm_bg.wasm'));
console.log(`robotics_wasm (wasm-bindgen ${locked}): ${OUT}  →  ${join(PUBLIC, 'robotics_wasm_bg.wasm')}`);

// Recorded runs a widget replays. They are copies of files in evidence/ (never edited, never
// regenerated here); checks/run.ts asserts every copy equals its source byte for byte, so the
// page cannot drift from the evidence it cites. [source under the repository root, name served].
const REPO = resolve(CRATE, '..', '..');
const DATA = [
  ['evidence/phase-4-mapping/2026-10-11-evaluation-4-oblique/integrated-scans.csv', 'ch05-oblique-integrated-scans.csv'],
  ['evidence/phase-5-slam/v2-synthetic-4-eval-rectangle/scans.csv', 'ch06-rusty-rectangle-scans.csv'],
  ['evidence/phase-5-slam/2026-10-15-live-2-labels/integrated-scans.csv', 'ch06-webots-live-2-scans.csv'],
  ['evidence/phase-5-slam/2026-10-15-live-source-live-2/integrated-scans.csv', 'ch06-webots-live-2-recorded-labels-scans.csv'],
];
const DATA_OUT = join(WEB, 'public', 'data');
mkdirSync(DATA_OUT, { recursive: true });
for (const [from, name] of DATA) {
  const src = join(REPO, from);
  if (!existsSync(src)) {
    console.error(`recorded run not found: ${src}`);
    process.exit(1);
  }
  copyFileSync(src, join(DATA_OUT, name));
  console.log(`${from}  →  ${join(DATA_OUT, name)}`);
}
