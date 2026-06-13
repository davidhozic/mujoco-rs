---
name: asan
description: Run AddressSanitizer and UndefinedBehaviorSanitizer on the Rust+C FFI boundary. Use this when asked to check for memory errors, undefined behavior, or when verifying FFI safety.
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
    -DMUJOCO_BUILD_SIMULATE:BOOL=OFF \
    -DMUJOCO_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -G Ninja \
    -DCMAKE_C_COMPILER:STRING=clang \
    -DCMAKE_CXX_COMPILER:STRING=clang++ \
    -DCMAKE_C_FLAGS="-fsanitize=address,undefined" \
    -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"

cmake --build build --parallel --target mujoco
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

cargo +nightly run --example basic --target x86_64-unknown-linux-gnu --no-default-features --features "<required features here>"
```

> [!NOTE]
> **Release mode**: The MuJoCo C library is always built with optimizations (`Release`). To also
> enable the Rust optimizer -- which can expose bugs that only manifest with optimizations active
> (e.g., temporary value use-after-free, optimizer-assumed invariants) -- add `--release` to the
> cargo commands above.
>
> Release mode drops debug symbols, making stack traces harder to read. To get both optimizations
> and readable traces, use `RelWithDebInfo` for the CMake build
> (`-DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo`) and add `-C debuginfo=1` to `RUSTFLAGS`.

**IMPORTANT:** Always run the `examples/miri_test.rs` example with address sanitizer as well (in addition to other tests).

> [!TIP]
> **Why `--target x86_64-unknown-linux-gnu`?**
> Cargo requires the target triple to be explicitly provided when using `-Zsanitizer=address`, even if compiling for the host system.

## Error Reporting
If a memory violation occurs on either the C or Rust side (e.g., invalid reference casting, out-of-bounds array access, use-after-free), AddressSanitizer will intercept it and immediately dump a stack trace (`ERROR: AddressSanitizer: ...`) pinpointing exactly which line violated the memory rules.

## Deliverable -- HTML report

This skill diagnoses; it does not apply code fixes -- so its findings go into a self-contained
HTML report, not just terminal output. Write/overwrite `mujoco-rs-asan-report.html` at the repo
root (scope: this run). It must be standalone (inline `<style>`), ASCII-only, and match the
shared report aesthetic used by `/verify` (`mujoco-rs-verify-report.html`) and
`mujoco-rs-memory-safety-audit.html`: ivory canvas, coral accent, warm near-black ink, Georgia
serif headings, rounded pill badges, white cards, hairline-border tables. Reuse that styling.

Contents:

- A header noting the toolchain (Clang/LLVM, nightly), the sanitizers enabled
  (`address,undefined`), and exactly which targets were run (the test suite, the
  `examples/miri_test.rs` example, and any other examples).
- A **run table**: `Run (test / example) | Features | Result (clean / error pill) | Notes`.
- One **finding card** per sanitizer error: the error kind (`heap-buffer-overflow`,
  `use-after-free`, UBSan check, leak, etc.), the C-or-Rust side, the offending `file:line` from
  the trace, and a concise excerpt of the stack trace (trim to the relevant frames; do not paste
  the full dump).
- If every run is clean, state that explicitly with a `Clean` pill and no finding cards.

After writing the file, present a brief plain-text summary (which runs were clean, which errored).
