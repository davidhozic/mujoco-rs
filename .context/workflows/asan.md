---
description: How to build and run MuJoCo and mujoco-rs with AddressSanitizer and UndefinedBehaviorSanitizer
---

# Running with AddressSanitizer (ASan) and UndefinedBehaviorSanitizer (UBSan)

Miri cannot track memory inside the FFI boundary because it doesn't hook into C's `malloc` and native function pointers. To properly check the interaction between Rust and C for memory errors (such as overflows, leaks, use-after-free, or undefined behavior like reference casting), we must compile **both** the C library and the Rust wrapper with sanitizers.

## Prerequisites
- **Clang/LLVM** compiler (recommended for advanced sanitizers).
- Rust Nightly toolchain (`cargo +nightly`).

## Step 1: Build MuJoCo with Sanitizers

First, navigate to the `mujoco` directory and clean any previous builds:

```bash
cd mujoco
rm -rf build
```

Then configure and compile MuJoCo statically with ASan and UBSan injected. We use Ninja for speed and Clang for the best sanitizer integration:

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
    -DCMAKE_INSTALL_PREFIX:STRING="$(pwd)/mujoco_install" \
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
    -DMUJOCO_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -G Ninja \
    -DCMAKE_C_COMPILER:STRING=clang \
    -DCMAKE_CXX_COMPILER:STRING=clang++ \
    -DMUJOCO_HARDEN:BOOL=ON \
    -DCMAKE_C_FLAGS="-fsanitize=address,undefined" \
    -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"

cmake --build build --parallel --target glfw libmujoco_simulate mujoco --config=Release
```

## Step 2: Build and Run `mujoco-rs` with ASan

Now return to the repository root. Link against the freshly compiled static library, and pass `-Zsanitizer=address` to `rustc` to instrument the Rust code side and pull in the ASan runtime.

To run tests:
```bash
export MUJOCO_STATIC_LINK_DIR="$(realpath mujoco/build/lib64)"
export RUSTFLAGS="-Zsanitizer=address"

# Example: Run unit tests
cargo +nightly test --lib --target x86_64-unknown-linux-gnu --no-default-features --features renderer
```

To run a specific example (e.g., `basic`):
```bash
export MUJOCO_STATIC_LINK_DIR="$(realpath mujoco/build/lib64)"
export RUSTFLAGS="-Zsanitizer=address"

cargo +nightly run --example basic --target x86_64-unknown-linux-gnu "--no-default-features" --features "<required features here>"
```

**IMPORTANT:** Always run the `examples/miri_test.rs` example with address sanitizer as well (in addition to other tests).

> [!TIP]
> **Why `--target x86_64-unknown-linux-gnu`?**
> Cargo requires the target triple to be explicitly provided when using `-Zsanitizer=address`, even if compiling for the host system.

## Error Reporting
If a memory violation occurs on either the C or Rust side (e.g., invalid reference casting, out-of-bounds array access, use-after-free), AddressSanitizer will intercept it and immediately dump a stack trace (`ERROR: AddressSanitizer: ...`) pinpointing exactly which line violated the memory rules.