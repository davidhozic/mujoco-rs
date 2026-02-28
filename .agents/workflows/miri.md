---
description: How to run Miri on mujoco-rs with C FFI
---

# Running with Miri

Run the codebase under Miri's experimental FFI native-lib support to detect undefined behavior in Rust code interoperating with MuJoCo C FFI.

// turbo-all

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Run Miri (replace `X.Y.Z` with the version found in step 1, and `<EXAMPLE_NAME>`):
```bash
export MUJOCO_DYNAMIC_LINK_DIR="$(realpath mujoco-X.Y.Z/lib/)" && export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR" && export MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-native-lib=$(realpath mujoco-X.Y.Z/lib/libmujoco.so.X.Y.Z) -Zmiri-permissive-provenance -Zmiri-disable-validation -Zmiri-disable-stacked-borrows -Zmiri-symbolic-alignment-check" && cargo +nightly miri run --example <EXAMPLE_NAME> --features <needed features>
```

The dedicated `miri_test` example (at `examples/miri_test.rs`) is the recommended target:
```bash
export MUJOCO_DYNAMIC_LINK_DIR="$(realpath mujoco-X.Y.Z/lib/)" && export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR" && export MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-native-lib=$(realpath mujoco-X.Y.Z/lib/libmujoco.so.X.Y.Z) -Zmiri-permissive-provenance -Zmiri-disable-validation -Zmiri-disable-stacked-borrows -Zmiri-symbolic-alignment-check" && cargo +nightly miri run --example miri_test
```

3. Verify that the exit code is 0 (success) — check the terminal output for errors or Undefined Behavior panics.

> [!NOTE]
> - `libtest` and native libraries currently fail compilation in Miri due to a rustc monomorphization bug. Miri can only be run on `bin` targets, `examples`, or tests configured with `harness = false`.
> - Required `MIRIFLAGS` are passed to disable false positives on provenance from `malloc` allocations in C code that Rust dereferences.
> - `-Zmiri-symbolic-alignment-check` is added to make alignment checks more precise, catching cases where alignment would only pass by chance.
> - `-Zmiri-permissive-provenance` is required to avoid false positives from MuJoCo's internal C allocations. Raw pointer tracking is now enabled by default in all Miri versions.
> - `examples/miri_shim.c` (the C-native allocation test helper) is compiled automatically by `build.rs` via the `cc` crate — no manual compilation step is required.
> - The `miri_test` example defines `set_mujoco_allocators` as a **pure Rust function** that writes to `mju_user_malloc`/`mju_user_free` extern statics. Under Miri, the function body is a no-op (`#[cfg(not(miri))]`) because: (1) Miri does not support `extern static` access in native-lib mode, and (2) C→Rust fn-ptr callbacks are unsupported by Miri's native-lib mode.