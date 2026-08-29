---
name: miri
description: Run the codebase under Miri with native FFI support to detect undefined behavior in Rust-C interop. Use this when asked to check for UB or verify memory safety.
---

# Running with Miri

Run the codebase under Miri's experimental FFI native-lib support to detect undefined behavior in Rust code interoperating with MuJoCo C FFI.


1. **Build MuJoCo with Miri support**:
   Navigate to the MuJoCo source directory (e.g., `mujoco/`) and build with the Miri allocator hooks enabled. We use a configuration similar to the official build environment but with shared libraries enabled for Miri:
   ```bash
   cd mujoco
   cmake -S . -B build \
       -DCMAKE_BUILD_TYPE:STRING=Release \
       -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
       -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
       -DMUJOCO_BUILD_TESTS=OFF \
       -DBUILD_SHARED_LIBS:BOOL=ON \
       -G Ninja \
       -DCMAKE_C_COMPILER:STRING=clang \
       -DCMAKE_CXX_COMPILER:STRING=clang++ \
       -DMUJOCO_HARDEN:BOOL=ON \
       -DMUJOCO_MIRI_SUPPORT:BOOL=ON
   cmake --build build --parallel --target mujoco
   ```

2. **Select the example and prepare a temporary minimal patch**:
   Pick the example requested by the user and patch it directly with the smallest possible temporary change so Miri can run.

   Use this pattern:
   - Do **not** add long-lived `#[cfg(miri)]` / `#[cfg(not(miri))]` scaffolding.
   - Apply only the minimal temporary edits needed (for example, call `mujoco_rs::miri::install_allocator()`, simplify/shorten runtime loop, avoid viewer/GL-only paths).
   - Keep edits tightly scoped to the selected example and preserve behavior as much as possible.

3. **Setup environment and run the selected example**:
   Run under Miri using the built library. Replace `<EXAMPLE_NAME>` and `X.Y.Z`:
   ```bash
   # Set paths and Miri flags (using build/lib64/ or build/lib/ depending on your system)
   # Replace X.Y.Z with the MuJoCo version from Cargo.toml (e.g. +mj-3.9.0 -> 3.9.0)
   export MUJOCO_DYNAMIC_LINK_DIR=$(realpath build/lib64/) && \
   export LD_LIBRARY_PATH=$(realpath build/lib64/) && \
   export MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-native-lib=$(realpath build/lib64/libmujoco.so.X.Y.Z) -Zmiri-permissive-provenance -Zmiri-symbolic-alignment-check -Zmiri-deterministic-concurrency -Zmiri-backtrace=full -Zmiri-report-progress -Zmiri-tree-borrows -Zmiri-tree-borrows-implicit-writes -Zmiri-address-reuse-rate=1 -Zmiri-user-relevant-crates=mujoco_rs -Zmiri-track-alloc-accesses" && \
   cd .. && \
   cargo +nightly miri run --example <EXAMPLE_NAME> --features <REQUIRED_FEATURES>
   ```
   - Do not pipe the Miri command through `tail`, `head`, or `grep` while streaming; those can buffer and delay visible output.

   For targeted deep-dive tracing, extend `MIRIFLAGS` with:
   - `-Zmiri-track-alloc-id=...`
   - `-Zmiri-track-pointer-tag=...`

4. **Verify results and clean up**:
   - Check for any `error: Undefined Behavior` reports.
   - The warning about "sharing memory with a native function called via FFI" is expected when calling `mujoco_rs::miri::install_allocator()`.
   - Treat the example edits as temporary: always revert them after investigation.
   - Do not commit persistent Miri-only wiring in user-facing examples.

> [!NOTE]
> - **Global Allocator**: When `MUJOCO_MIRI_SUPPORT` is enabled, `mujoco/src/user/user_miri.cc` overrides `operator new`/`delete` so that every internal C++ allocation goes through `mju_malloc`.
>   `mujoco_rs::miri::install_allocator()` then registers a pool of separate Rust allocations, and MuJoCo takes one whole chunk per block. Miri therefore knows the bounds of every MuJoCo block on its own, and reports a read or a write past a block. One shared buffer would hide that, because the overrun would stay inside the same Miri allocation.
> - **Pool size**: `CHUNK_CLASSES` in `src/miri.rs` sets the chunk sizes and counts, and it is the only thing to tune. A class that runs out spills into the next size up, so only an empty pool fails, with `MIRI_LOG: ... no free chunk holds`. Raise a count and rerun; no C rebuild is needed. Miri run time grows with the total pool size, so keep the pool near the live high-water mark.
> - **New flags**: `-Zmiri-tree-borrows-implicit-writes` makes Tree Borrows stricter, and `-Zmiri-address-reuse-rate=1` maximizes use-after-free detection. Both cost nothing measurable here.
> - **Leaks**: the pool is a deliberate leak, and `install_allocator` marks each chunk with the `miri_static_root` intrinsic, so the leak checker stays on for every other allocation. Do not add `-Zmiri-ignore-leaks`.
> - **Native call limits**: a native call supports only integer and pointer arguments and returns. `install_logging_hook()` therefore cannot run under Miri, because its handler returns `[i8; 1024]`.
> - **Verbose output**: The default flags intentionally increase log volume (`backtrace=full`, `report-progress`, alloc access tracking) to maximize diagnosability.
> - **Provenance**: `-Zmiri-permissive-provenance` is essential because MuJoCo (a C library) manages its own memory, which Rust then accesses.
> - **Strict provenance**: `-Zmiri-strict-provenance` is not compatible with native FFI calls; do not use it for MuJoCo-backed runs.
> - **Test harness**: Miri does run the standard test harness. `cargo +nightly miri test --test <NAME>` works once every test calls `mujoco_rs::miri::install_allocator()` first; without it, the first MuJoCo pointer fails with `[noalloc] has no provenance`. Add `-- --test-threads=1`, because the allocator keeps no lock.

## Deliverable -- HTML report

This skill diagnoses UB; it does not apply code fixes (the temporary example edits are reverted),
so its findings go into a self-contained HTML report, not just terminal output. Write/overwrite
`mujoco-rs-miri-report.html` at the repo root (scope: this run). It must be standalone (inline
`<style>`), ASCII-only, and match the shared report aesthetic used by `/verify`
(`mujoco-rs-verify-report.html`) and `mujoco-rs-memory-safety-audit.html`: ivory canvas, coral
accent, warm near-black ink, Georgia serif headings, rounded pill badges, white cards,
hairline-border tables. Reuse that styling.

Contents:

- A header noting the nightly toolchain and the key `MIRIFLAGS` (e.g. `-Zmiri-tree-borrows`,
  `-Zmiri-native-lib=...`, `-Zmiri-permissive-provenance`) and which example(s) were run.
- A **run table**: `Example | Features | Result (clean / UB pill) | Notes`.
- One **finding card** per `error: Undefined Behavior`: the UB kind (e.g. out-of-bounds access,
  invalid use of uninitialized memory, aliasing/Tree-Borrows violation), the offending
  `file:line`, and a concise excerpt of the Miri diagnostic and backtrace (relevant frames only,
  not the full verbose log).
- Note the expected, benign "sharing memory with a native function called via FFI" warning so it
  is not mistaken for a finding. If every run is clean, state that with a `Clean` pill.

After writing the file, present a brief plain-text summary (which examples were clean, which hit UB).
