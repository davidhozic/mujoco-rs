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

Then configure and compile MuJoCo statically with ASan and UBSan injected. We use Ninja for speed and Clang for best sanitizer integration:

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
    -DMUJOCO_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -G Ninja \
    -DCMAKE_C_COMPILER:STRING=clang \
    -DCMAKE_CXX_COMPILER:STRING=clang++ \
    -DMUJOCO_HARDEN:BOOL=ON \
    -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -DADDRESS_SANITIZER -Dasm=__asm__ -Wno-gcc-compat" \
    -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -DADDRESS_SANITIZER -Wno-gcc-compat"

cmake --build build --parallel --target mujoco --config=Release
```

> [!IMPORTANT]
> `-DADDRESS_SANITIZER` is **required** for MuJoCo's internal sanitizer-gated code paths (`mjsan.h` poison/unpoison instrumentation).
>
> `-Dasm=__asm__` is **required** for strict C11 compliance with inline assembly in the MuJoCo engine.
>
> `-Wno-gcc-compat` suppresses Clang warnings about GCC attribute syntax (e.g., `always_inline` placement), which are treated as errors by `-Werror` in the MuJoCo build.
>
> This workflow is **Clang-optimized**: We use Clang for superior sanitizer integration and keep warning flags minimal.
>
> The ASAN workflow only needs the core static library (`mujoco` target) for Rust-side ASAN linking.

## Step 2: Build and Run `mujoco-rs` with ASan

Now return to the repository root. Link against the freshly compiled static library, and pass `-Zsanitizer=address` to `rustc` to instrument the Rust code side and pull in the ASan runtime.

To run tests:
```bash
export MUJOCO_STATIC_LINK_DIR="$(realpath mujoco/build/lib64)"
export RUSTFLAGS="-Zsanitizer=address"
export CARGO_TERM_PROGRESS_WHEN=never

# Example: Run unit tests
cargo +nightly test --lib --target x86_64-unknown-linux-gnu --no-default-features --features renderer
```

To run a specific example (e.g., `basic`):
```bash
export MUJOCO_STATIC_LINK_DIR="$(realpath mujoco/build/lib64)"
export RUSTFLAGS="-Zsanitizer=address"
export CARGO_TERM_PROGRESS_WHEN=never

cargo +nightly run --example basic --target x86_64-unknown-linux-gnu --no-default-features
```

**IMPORTANT:** Always run the `examples/miri_test.rs` example with address sanitizer as well (in addition to other tests).

```bash
export MUJOCO_STATIC_LINK_DIR="$(realpath mujoco/build/lib64)"
export RUSTFLAGS="-Zsanitizer=address"
export CARGO_TERM_PROGRESS_WHEN=never

cargo +nightly run --example miri_test --target x86_64-unknown-linux-gnu --no-default-features --features renderer
```

> [!NOTE]
> `examples/miri_test.rs` executes a `#[cfg(not(miri))]` stub when run normally, so it is still required by project policy but does not provide deep runtime coverage outside actual Miri runs.

> [!TIP]
> **Why `--target x86_64-unknown-linux-gnu`?**
> Cargo requires the target triple to be explicitly provided when using `-Zsanitizer=address`, even if compiling for the host system.

## Error Reporting
If a memory violation occurs on either the C or Rust side (e.g., invalid reference casting, out-of-bounds array access, use-after-free), AddressSanitizer will intercept it and immediately dump a stack trace (`ERROR: AddressSanitizer: ...`) pinpointing exactly which line violated the memory rules.