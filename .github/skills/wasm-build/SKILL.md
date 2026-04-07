---
name: wasm-build
description: Build MuJoCo from the mujoco/ submodule as a WebAssembly static library and run an example under Node.js via Emscripten. Use this when asked to build for WebAssembly / WASM / Emscripten, or to run examples in Node.js.
---

# WebAssembly Build (Emscripten)

Build the `mujoco/` git submodule as a static WebAssembly library, link it with mujoco-rs, and run `examples/basic.rs` under Node.js.

## Prerequisites

- emsdk at `/home/davidhozic/repo/emsdk`.
- `rustup target add wasm32-unknown-emscripten` already done.

---

## Step 1 — Set up environment

```bash
source /home/davidhozic/repo/emsdk/emsdk_env.sh --no-check
```

Check that the `mujoco/` submodule is at the version mujoco-rs expects
(look for `+mj-X.Y.Z` in `Cargo.toml`). `mjVERSION_HEADER` encodes the version
as `MAJ*1000000 + MIN*1000 + PATCH` (e.g. 3006000 = 3.6.0):

```bash
grep "^version" Cargo.toml | head -1
grep "mjVERSION_HEADER" mujoco/include/mujoco/mujoco.h | head -1
```

If the versions differ, check out the right tag:

```bash
git -C mujoco fetch --tags
git -C mujoco checkout X.Y.Z
```

---

## Step 2 — Configure MuJoCo for WebAssembly

```bash
emcmake cmake -S mujoco -B mujoco/build/wasm
```

---

## Step 3 — Build MuJoCo

MuJoCo's `CMakeLists.txt` adds `-fexceptions` for Emscripten, which uses JS-based
sjlj and generates `emscripten_longjmp` (JS-only). `EMCC_CFLAGS` appends flags to
every emcc invocation after all cmake-managed flags, so `-fwasm-exceptions` overrides
`-fexceptions` (last flag wins) and switches to wasm-native sjlj (`__wasm_longjmp`,
present in the static sysroot).

```bash
EMCC_CFLAGS="-fwasm-exceptions" cmake --build mujoco/build/wasm --target mujoco --parallel 8
```

---

## Step 4 — Build and run the Rust example

```bash
LIB_DIR=$(realpath mujoco/build/wasm/lib)
MUJOCO_STATIC_LINK_DIR="$LIB_DIR" \
    cargo build --example basic --target wasm32-unknown-emscripten

NODE_BIN="$(command -v node 2>/dev/null || echo '/home/davidhozic/repo/emsdk/node/22.16.0_64bit/bin/node')"
"$NODE_BIN" target/wasm32-unknown-emscripten/debug/examples/basic.js
```

Expected output: `Step 0` through `Step 999`.

---

## Running in a browser

Copy `examples/viewer.html` next to the built `basic.js` and `basic.wasm`, then
serve over HTTP and open the HTML file:

```bash
cp examples/viewer.html target/wasm32-unknown-emscripten/debug/examples/
python3 -m http.server 8080 --directory target/wasm32-unknown-emscripten/debug/examples
# open http://localhost:8080/viewer.html
```

Output from `print!`/`println!` will appear on the page.

---

## Running natively (without WASM)

```bash
LIB_PATH=$(realpath mujoco-3.6.0/lib)
MUJOCO_DYNAMIC_LINK_DIR="$LIB_PATH" LD_LIBRARY_PATH="$LIB_PATH" \
    cargo run --example basic
```
