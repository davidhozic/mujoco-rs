# Important Context

## Where to look first

| Task | Action |
|---|---|
| Modifying wrappers | Read `src/util.rs` first, then the wrapper file, then MuJoCo C API docs |
| Build issues | Read `Cargo.toml`, then `build.rs`, then try the `/build` workflow |
| Type questions | Explore `src/wrappers/` for primitive type definitions |
| Code generation | Check if `../mujoco-rs-utils` exists and run with `--help` |
| Running tests | Use the `/test` workflow |
| Running examples | Use the `/run-example` workflow |
| Running Miri | Use the `/miri` workflow; apply a minimal temporary direct patch to the user-selected example if needed |
| Feature flags | Read `Cargo.toml` `[features]` section |
| Documentation URLs | See `project-overview.md` |

## Environment variables

| Variable | Purpose |
|---|---|
| `MUJOCO_DYNAMIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for dynamic linking (compile-time) |
| `MUJOCO_STATIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for static linking |
| `MUJOCO_DOWNLOAD_DIR` | Where the `auto-download-mujoco` feature extracts MuJoCo |
| `LD_LIBRARY_PATH` | Runtime library search path (Linux) |

All linking env vars must be absolute paths (`realpath`). Set **both** `MUJOCO_DYNAMIC_LINK_DIR`
(compile time, read by `build.rs`) and `LD_LIBRARY_PATH` (runtime); the `/test`, `/build`, and
`/run-example` workflows already set both. With `auto-download-mujoco`, a set
`MUJOCO_DYNAMIC_LINK_DIR` takes precedence over the downloaded copy.

## Common pitfalls

- **Multiple MuJoCo versions on disk**: check `Cargo.toml` for the correct version (`+mj-X.Y.Z`).
- **Dense vs sparse C arrays**: check the C API docs before indexing.
- **Address `-1` means None**: map to `Option`; address `-1` MUST yield `None`.
- **Stride / length cross-reference**: check the field's dimension comment in `mujoco/include/mujoco/mjmodel.h`
  or `mjdata.h` for every `info_method!` / `array_slice_dyn!` (e.g. `body_xpos` is `(nbody x 3)`, so
  stride = 3). Most common source of silent bugs; full checklists in `macro-system.md`.
- **`array_slice_dyn!` length field**: a wrong FFI length field (e.g. `ngeom` for a body array) reads
  out of bounds silently. The cast type's `N` must match the header's second dimension.
- **Error buffers**: model loaders use `[0i8; 100]`; pass the correct length to `mj_loadXML`.
- **`debug_assert!` compiles out in release**: safety-critical checks use `assert!` (see
  `coding-conventions.md`).
- **Bounds checks use `< max`**, never `<= max` (`jac()`, `object_velocity()`).
- **Boolean conversions**: `bool as i32` yields 0/1; verify the C function expects 0/1. Do not
  invert a flag accidentally (`!flg_local as i32` flips the meaning without a crash).
- **Nullable FFI output pointers**: `ptr::null_mut()`, NOT `vec![].as_mut_ptr()` (an empty `Vec`'s
  pointer is non-null; MuJoCo writes through it).
- **Return conventions vary per function**: `mj_saveLastXML` is 1=success, `mj_addContact` is
  0=success. Check the C header comment.
- **`mj_model_dyn_range!` last element**: falls back to the total-length field as the exclusive end.
- **`nq != njnt`** for free (7 `qpos`) and ball (4 `qpos`) joints; never iterate joints by `nq`.
- **Null pointer guards**: `PointerView::deref()` returns `&[]` for null; verify the same guard in
  similar patterns (contacts, array slices).
- **Generic `Send`/`Sync` impls** must require the generic parameter to be `Send`/`Sync` too
  (`MjData<M>`; `MjvScene` is non-generic and unconditionally `Send + Sync`).
- **Rendering in tests**: viewer/renderer features are off by default; enable them explicitly.
- **`MjData::model()` returns `&MjModel`**, not `&M`: `data.model().clone()` needs no `M: Clone`;
  only cloning the holder field directly (e.g. `try_clone()`) does.
- **`cfg!(target_os)` checks the host** in `build.rs`; use `CARGO_CFG_TARGET_OS` for the target.
- **`mj_contactForce` with `id >= ncon` returns `[0; 6]`** by C-side handling; do NOT add a Rust
  bounds check. Check other functions for similar built-in handling before reporting problems.

## MuJoCo version bump checklist

1. Update the `+mj-X.Y.Z` suffix in `Cargo.toml` `[package].version`.
2. Download and extract the new MuJoCo release into `mujoco-X.Y.Z/` at the repo root.
3. Update `build.rs` if the linking strategy or expected paths changed.
4. Regenerate FFI bindings with the `ffi-regenerate` feature (developer only; agents must NOT).
5. Review the MuJoCo release notes; update wrappers accordingly.
6. Update `changelog.rst` and `migration.rst`.
7. Run the full test suite, `/doc`, and spot-check examples.
