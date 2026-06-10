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
   - Apply only the minimal temporary edits needed (for example, initialize `setup_miri_bump_allocator`, simplify/shorten runtime loop, avoid viewer/GL-only paths).
   - Keep edits tightly scoped to the selected example and preserve behavior as much as possible.

3. **Setup environment and run the selected example**:
   Run under Miri using the built library. Replace `<EXAMPLE_NAME>` and `X.Y.Z`:
   ```bash
   # Set paths and Miri flags (using build/lib64/ or build/lib/ depending on your system)
   # Replace X.Y.Z with the MuJoCo version from Cargo.toml (e.g. +mj-3.9.0 -> 3.9.0)
   export MUJOCO_DYNAMIC_LINK_DIR=$(realpath build/lib64/) && \
   export LD_LIBRARY_PATH=$(realpath build/lib64/) && \
   export MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-native-lib=$(realpath build/lib64/libmujoco.so.X.Y.Z) -Zmiri-permissive-provenance -Zmiri-symbolic-alignment-check -Zmiri-deterministic-concurrency -Zmiri-backtrace=full -Zmiri-report-progress -Zmiri-tree-borrows -Zmiri-track-alloc-accesses" && \
   cd .. && \
   cargo +nightly miri run --example <EXAMPLE_NAME> --features <REQUIRED_FEATURES>
   ```
   - Do not pipe the Miri command through `tail`, `head`, or `grep` while streaming; those can buffer and delay visible output.

   For targeted deep-dive tracing, extend `MIRIFLAGS` with:
   - `-Zmiri-track-alloc-id=...`
   - `-Zmiri-track-pointer-tag=...`

4. **Verify results and clean up**:
   - Check for any `error: Undefined Behavior` reports.
   - The warning about "sharing memory with a native function called via FFI" is expected when calling `setup_miri_bump_allocator`.
   - Treat the example edits as temporary: always revert them after investigation.
   - Do not commit persistent Miri-only wiring in user-facing examples.

> [!NOTE]
> - **Global Allocator**: When `MUJOCO_MIRI_SUPPORT` is enabled, the MuJoCo build uses linker-level wrapping (`--wrap`) to intercept all internal C++ `operator new`/`delete` calls and redirect them to a specialized Miri bump allocator.
> - **Verbose output**: The default flags intentionally increase log volume (`backtrace=full`, `report-progress`, alloc access tracking) to maximize diagnosability.
> - **Provenance**: `-Zmiri-permissive-provenance` is essential because MuJoCo (a C library) manages its own memory, which Rust then accesses.
> - **Strict provenance**: `-Zmiri-strict-provenance` is not compatible with native FFI calls; do not use it for MuJoCo-backed runs.
> - **Target Limitation**: Miri with native libs can only run on `bin` targets or `examples`. It cannot currently run the standard Rust test harness on `lib` targets due to compiler limitations.

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
