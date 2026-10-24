#!/usr/bin/env bash
#
# Vercel's install step for the webbook.
#
# The build compiles crates/robotics_wasm to WebAssembly (`npm run wasm`), and
# Vercel's build image has no Rust. This installs what that step needs, then
# runs the usual `npm ci`:
#
#   - a minimal stable Rust with the wasm32-unknown-unknown target, via rustup
#   - the wasm-bindgen CLI at exactly the version locked in the crate's
#     Cargo.lock, as the prebuilt release binary (compiling it would add
#     several minutes to every build)
#
# The build step runs in a fresh shell, so vercel.json's buildCommand sets
# CARGO_HOME, RUSTUP_HOME and PATH itself.
set -euo pipefail

cd "$(dirname "$0")/.."

# Vercel's image carries a cargo of its own and points CARGO_HOME at it
# (/rust), without the wasm32 target. Do not reuse it: pin both homes under
# $HOME, install a toolchain there and put it first on PATH. vercel.json's
# buildCommand sets the same three variables.
export CARGO_HOME="$HOME/.cargo" RUSTUP_HOME="$HOME/.rustup"
if [ ! -x "$HOME/.cargo/bin/rustup" ]; then
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs |
    RUSTUP_INIT_SKIP_PATH_CHECK=yes sh -s -- -y --no-modify-path --profile minimal \
      --default-toolchain stable --target wasm32-unknown-unknown
fi
export PATH="$HOME/.cargo/bin:$PATH"
rustup target add wasm32-unknown-unknown

LOCK=../crates/robotics_wasm/Cargo.lock
version=$(awk '/^name = "wasm-bindgen"$/ { getline; gsub(/version = |"/, ""); print; exit }' "$LOCK")
if [ -z "$version" ]; then
  echo "could not read the locked wasm-bindgen version from $LOCK" >&2
  exit 1
fi
if [ "$(wasm-bindgen --version 2>/dev/null)" != "wasm-bindgen $version" ]; then
  tmp=$(mktemp -d)
  curl -sSfL "https://github.com/wasm-bindgen/wasm-bindgen/releases/download/$version/wasm-bindgen-$version-x86_64-unknown-linux-musl.tar.gz" |
    tar -xz -C "$tmp"
  install -m 0755 "$tmp/wasm-bindgen-$version-x86_64-unknown-linux-musl/wasm-bindgen" "$HOME/.cargo/bin/"
fi
wasm-bindgen --version

npm ci
