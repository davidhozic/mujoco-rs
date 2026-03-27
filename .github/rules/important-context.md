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
| Feature flags | Read `Cargo.toml` `[features]` section |
| Documentation URLs | See `project-overview.md` |

## Environment variables

| Variable | Purpose |
|---|---|
| `MUJOCO_DYNAMIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for dynamic linking (compile-time) |
| `MUJOCO_STATIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for static linking |
| `MUJOCO_DOWNLOAD_DIR` | Where the `auto-download-mujoco` feature extracts MuJoCo |
| `LD_LIBRARY_PATH` | Runtime library search path (Linux) |

All linking env vars must be **absolute paths**. Use `realpath` to convert relative paths.

**`MUJOCO_DYNAMIC_LINK_DIR` vs `LD_LIBRARY_PATH`**: these serve different roles.
`MUJOCO_DYNAMIC_LINK_DIR` is read by `build.rs` at compile time so the linker can find `libmujoco.so*`.
`LD_LIBRARY_PATH` is the OS runtime search path; without it the compiled binary fails with
"cannot open shared object file". Always set **both** when running tests or examples.
The `/test`, `/build`, and `/run-example` workflows already include both.

**`auto-download-mujoco` feature**: when active, the build downloads MuJoCo to `MUJOCO_DOWNLOAD_DIR`.
If `MUJOCO_DYNAMIC_LINK_DIR` is also set, the pre-downloaded copy takes precedence.

## Common pitfalls

- **Multiple MuJoCo versions on disk**: always check `Cargo.toml` for the correct version (`+mj-X.Y.Z`).
- **Dense vs sparse C arrays**: MuJoCo C arrays can be dense or address-array-indexed. Read the C API
  docs to determine which applies before writing a wrapper.
- **Address `-1` means None**: address array fields documented as `-1: none` must map to `Option` in
  Rust. If the address is -1, the resulting field MUST be `None`.
- **Stride / length cross-reference**: whenever you add or review an `info_method!` or
  `array_slice_dyn!` invocation, look up the field in `mujoco/include/mujoco/mjmodel.h` or `mjdata.h`
  and check its dimension comment. E.g., `body_xpos` is `(nbody x 3)` so stride = 3.
  This is the most common source of silent bugs.
- **Array slice length expressions**: `array_slice_dyn!` uses an FFI field as length (e.g.,
  `ffi().nbody`). A wrong field (e.g. `ngeom` for a body array) causes silent out-of-bounds reads.
- **Cast types in array slices**: when `array_slice_dyn!` casts to `[MjtNum; N]`, `N` must match the
  C header's second dimension. E.g., `body_pos` uses `[MjtNum; 3]` because the header says `(nbody x 3)`.
- **Error buffer sizes**: model loading functions use `[0i8; 100]` for error buffers. Verify the
  buffer is passed with the correct length to `mj_loadXML`.
- **`debug_assert`-only bounds checks**: `debug_assert!` is compiled out in release builds. Any
  bounds check that only uses `debug_assert!` before an FFI call silently skips validation in
  release mode. Use `assert!` for safety-critical checks, or at minimum ensure the C function
  itself has enough validation that skipping the Rust check cannot cause UB.
- **Bounds checking**: functions like `jac()`, `object_velocity()` validate IDs with `>= 0 && < max`.
  Off-by-one errors (`<=` instead of `<`) can cause UB.
- **Boolean-to-int conversions**: Rust `bool as i32` yields 0 or 1, matching MuJoCo's convention.
  Verify the C function actually expects 0/1 and not some other encoding.
- **Boolean flag inversion**: `flg_local as i32` is correct when C expects 1=local/0=world.
  Accidental `!flg_local as i32` silently inverts the meaning with no crash.
- **NULL vs dangling pointers in FFI output buffers**: when passing a nullable output pointer (e.g.,
  to `mj_jac`), use `ptr::null_mut()`, NOT `vec![].as_mut_ptr()`. An empty `Vec`'s pointer is
  non-null -- MuJoCo writes through it, causing UB.
- **Return value conventions vary per function**: `mj_saveLastXML` uses 1=success/0=error, but
  `mj_addContact` uses 0=success/1=buffer-full. Always verify from the C header comment.
- **`mj_view_indices!` last-joint edge case**: the final joint has no next address entry, so the
  macro falls back to `$max_n` (e.g. `nq`) as the exclusive end. Double-check this path.
- **Free/ball joints and nq vs njnt**: `nq != njnt` for models with free joints (7 qpos each) or
  ball joints (4 qpos each). `mj_model_nx_to_nitem!(model, nq)` returns `njnt` not `nq`.
- **Null pointer guards**: `PointerView::deref()` returns `&[]` for null pointers. Verify this guard
  exists in all similar patterns (contacts, array slices, etc.).
- **Send/Sync bounds for generic wrapper types**: any `unsafe impl Send/Sync` on a generic wrapper
  MUST require the generic parameter to also be `Send` / `Sync`. Without these bounds, non-thread-safe
  inner types are incorrectly marked thread-safe. (`MjvScene` is non-generic as of 3.0.0 and is
  unconditionally `Send + Sync`; this pitfall applies to `MjData<M>` and future generic wrappers.)
- **The test environment supports rendering**: viewer/renderer features can be explicitly enabled in
  tests; they are not part of the default feature set.
- **`MjData::model()` returns `&MjModel` (not `&M`)**: When reasoning about generic bounds, note that
  `data.model().clone()` clones `MjModel` directly (not `M`). The `M: Clone` bound is only required
  where `self.model.clone()` is called on the holder field directly (e.g., in `try_clone()`), not
  where `data.model().clone()` is used.
- **`cfg!(target_os)` checks the HOST, not the target**: In `build.rs`, `cfg!(target_os = "linux")`
  evaluates the machine running the build, not the compilation target. Use the
  `CARGO_CFG_TARGET_OS` environment variable when branching on the target platform (e.g., for
  selecting download URLs in cross-compilation scenarios).
- **`mj_contactForce` with `id >= ncon`**: The C function already handles out-of-range IDs and
  returns `[0; 6]` in that case. Do NOT add a Rust-side bounds check — the documented behavior
  is already enforced by the C implementation. Make sure to also check other functions for similar
  behavior before reporting problems.
