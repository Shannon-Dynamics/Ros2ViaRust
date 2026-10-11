# ROS via Rust: the webbook

Problem-based robotics in Rust, as a static site. Each chapter opens with a problem the
TurtleBot3 has, derives the theory it needs, and runs **the repository's real Rust**, compiled
to WebAssembly, in interactive widgets. Contract: [`design/TOC.md`](design/TOC.md). How to
write a chapter: [`AUTHORING.md`](AUTHORING.md).

## Build and view

Needs Node ≥ 20.9, Rust with the `wasm32-unknown-unknown` target, and the `wasm-bindgen`
CLI at the version pinned in `crates/robotics_wasm/Cargo.lock`.

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked   # once

cd web
npm ci
npm run wasm        # crates/robotics_wasm → lib/wasm/pkg + public/wasm
npm run check       # numerical self-checks against the compiled .wasm
npm run check:book  # book consistency
npm run build       # typecheck + static export to out/
npm run preview     # http://localhost:3000/  (gzip, .html fallback, application/wasm)
npx playwright install chromium   # once, for the next line
npm run smoke       # headless Chromium, light + dark, desktop + phone: console errors, widgets, KaTeX
```

`npm run dev` runs the dev server (it builds the WASM first).

## On this machine (WSL2, repository on Windows)

`node_modules` and Next's cache are slow on `/mnt/d`. Edit `web/` on Windows, then:

```sh
bash scripts/wsl-sync.sh                 # one-way copy to ~/rvr-web (build products excluded)
cd ~/rvr-web && source scripts/wsl-env.sh
npm run build && npm run check && npm run check:book
```

Nothing flows back from `~/rvr-web`. Copy `package-lock.json` back by hand if you change dependencies.

## GitHub Pages

`.github/workflows/deploy.yml` (repository root) builds with
`NEXT_PUBLIC_BASE_PATH` set from `actions/configure-pages`, so the site works under
`https://<owner>.github.io/<repo>/`. Reproduce locally:
`NEXT_PUBLIC_BASE_PATH=/<repo> npm run build && PORT=3000 NEXT_PUBLIC_BASE_PATH=/<repo> npm run preview`.
