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

## Step 1 -- Set up environment

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
git -C mujoco checkout X.Y.Z
```

> **Note:** `git fetch` is a disallowed networked operation. If the required tag is not
> present locally, ask the user to run `git -C mujoco fetch --tags` manually before
> proceeding.

---

## Step 2 -- Configure MuJoCo for WebAssembly

```bash
emcmake cmake -S mujoco -B mujoco/build/wasm
```

---

## Step 3 -- Build MuJoCo

MuJoCo's `CMakeLists.txt` adds `-fexceptions` for Emscripten, which uses JS-based
sjlj and generates `emscripten_longjmp` (JS-only). `EMCC_CFLAGS` appends flags to
every emcc invocation after all cmake-managed flags, so `-fwasm-exceptions` overrides
`-fexceptions` (last flag wins) and switches to wasm-native sjlj (`__wasm_longjmp`,
present in the static sysroot).

```bash
EMCC_CFLAGS="-fwasm-exceptions" cmake --build mujoco/build/wasm --target mujoco --parallel 8
```

---

## Step 4 -- Build and run the Rust example

```bash
LIB_DIR=$(realpath mujoco/build/wasm/lib)
EMCC_CFLAGS="-fwasm-exceptions" MUJOCO_STATIC_LINK_DIR="$LIB_DIR" \
    cargo build --example basic --target wasm32-unknown-emscripten

NODE_BIN="$(command -v node 2>/dev/null || echo '/home/davidhozic/repo/emsdk/node/22.16.0_64bit/bin/node')"
"$NODE_BIN" target/wasm32-unknown-emscripten/debug/examples/basic.js
```

Expected output: `Step 0` through `Step 999`.

### Memory growth for complex models

The Emscripten WASM heap defaults to 16 MB. Headless examples that programmatically build
complex models (many geoms, tendons, or connected bodies) can exceed this limit during
`mj_compile()` and panic with `"Could not allocate memory"`. Pass
`-sALLOW_MEMORY_GROWTH=1` as a linker argument via `RUSTFLAGS`:

```bash
EMCC_CFLAGS="-fwasm-exceptions" MUJOCO_STATIC_LINK_DIR="$LIB_DIR" \
    RUSTFLAGS="-C link-arg=-sALLOW_MEMORY_GROWTH=1" \
    cargo build --example <your_example> --target wasm32-unknown-emscripten
```

---

## Step 5 -- Serve examples in the browser (optional)

`assets/wasm_examples.html` provides a simple browser UI for running WASM examples.
First copy it next to the compiled `.js`/`.wasm` files:

```bash
EXAMPLES_DIR="target/wasm32-unknown-emscripten/debug/examples"
cp assets/wasm_examples.html "$EXAMPLES_DIR/index.html"
```

Then start the HTTP server using the bash tool with `mode="async", detach: true`
(see the `launch-terminal` skill for details):

```bash
python3 -m http.server 8080 --bind 127.0.0.1 \
    --directory target/wasm32-unknown-emscripten/debug/examples
```

Open **http://127.0.0.1:8080/** and click **> Run basic**.
Reload the page (F5) between runs to reset the Emscripten globals cleanly.

To stop the server: `kill <PID>`.

---

## Running natively (without WASM)

```bash
LIB_PATH=$(realpath mujoco-3.6.0/lib)
MUJOCO_DYNAMIC_LINK_DIR="$LIB_PATH" LD_LIBRARY_PATH="$LIB_PATH" \
    cargo run --example basic
```
