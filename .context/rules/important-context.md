---
trigger: always_on
---

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

## Key references

When in doubt, consult these documentation sources:

- **MuJoCo-rs guide**: https://mujoco-rs.readthedocs.io
- **MuJoCo C API docs**: https://mujoco.readthedocs.io/en/stable/overview.html
- **MuJoCo C API types**: https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html
- **MuJoCo C API functions**: https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html
- **Installation guide**: `docs/guide/source/installation.rst`
- **MuJoCo version**: check `Cargo.toml` for the version in the package version string (e.g., `+mj-X.Y.Z`)

## Environment variables

| Variable | Purpose |
|---|---|
| `MUJOCO_DYNAMIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for dynamic linking |
| `MUJOCO_STATIC_LINK_DIR` | Absolute path to MuJoCo's `lib/` for static linking |
| `MUJOCO_DOWNLOAD_DIR` | Where auto-download extracts MuJoCo |
| `LD_LIBRARY_PATH` | Runtime library search path (Linux) |

All linking env vars must be **absolute paths**. Use `realpath` to convert relative paths.

## Discovering the codebase

- Explore `src/` to understand the module structure - read `src/lib.rs` to see which modules exist and which are feature-gated.
- Explore `src/wrappers/` for safe Rust wrapper types.
- Explore `examples/` for usage patterns.
- Read `README.md` for a high-level overview.
- Read `Cargo.toml` for available features, dependencies, and the minimum Rust version.

## Common pitfalls
- MuJoCo C arrays can be dense or sparse (via address arrays). Read the C API docs to determine which applies.
- Address array fields documented as `-1: none` in MuJoCo should map to `Option` in the Rust wrappers for the item they point to. If the addressing value is -1, the resulting field MUST be `None`.
- The test environment handles rendering features gracefully; default features (including viewer) can be used.
- When in doubt, run `/expand-macros` and inspect the generated code.
- Multiple MuJoCo versions may be present on disk - always check `Cargo.toml` for the correct version.
- **`MUJOCO_DYNAMIC_LINK_DIR` vs `LD_LIBRARY_PATH`**: these do different things. `MUJOCO_DYNAMIC_LINK_DIR` is read by `build.rs` at compile time so the linker can find `libmujoco.so*`. `LD_LIBRARY_PATH` is the OS runtime search path; without it the compiled binary fails immediately with "cannot open shared object file". Always set both when running tests or examples. The `/test`, `/build`, and `/run-example` workflows already include both.
- **`auto-download-mujoco` feature**: when active, the build downloads MuJoCo to `MUJOCO_DOWNLOAD_DIR`. If you also set `MUJOCO_DYNAMIC_LINK_DIR` to a pre-downloaded copy it takes precedence. Stick to the preloaded workflow unless you specifically want auto-download.
- **Stride / length cross-reference**: whenever you add or review an `info_method!` or `array_slice_dyn!` invocation, look up the field in `mujoco/include/mujoco/mjmodel.h` or `mjdata.h` and check its dimension comment. For example `body_xpos` is declared `(nbody x 3)` so the stride must be 3, and `body_xmat` is `(nbody x 9)` so the stride must be 9. This is the most common source of silent bugs.
- **Array slice length expressions**: `array_slice_dyn!` invocations use an FFI field as the length (e.g., `ffi().nbody`). Verify this matches the C header's first dimension. A wrong field (e.g. `ngeom` for a body array) causes out-of-bounds reads.
- **Cast types in array slices**: When `array_slice_dyn!` casts to `[MjtNum; N]`, `N` must match the C header's second dimension. E.g., `body_pos` uses `[MjtNum; 3]` because the header says `(nbody x 3)`.
- **Error buffer sizes**: Model loading functions use `[0i8; 100]` for error buffers. MuJoCo error messages can be long -- verify the buffer is passed with correct length to `mj_loadXML`.
- **Bounds checking patterns**: Functions like `jac()`, `object_velocity()`, etc. validate IDs with `>= 0 && < max`. Off-by-one errors (`<=` instead of `<`, or `>` instead of `>=`) can cause UB.
- **Boolean-to-int conversions**: Rust `bool as i32` yields 0 or 1, which matches MuJoCo's convention. But verify the C function actually expects 0/1 and not some other encoding.
- **Null pointer guards**: `PointerView::deref()` returns `&[]` for null pointers. Verify this guard exists in all similar patterns (contacts, array slices, etc.).
- **NULL vs dangling pointers in FFI output buffers**: Many MuJoCo functions accept nullable output pointers (e.g., `mj_jac` takes `jacp` and `jacr` as nullable `mjtNum*`). When the Rust code conditionally skips a computation (e.g., `jacp=false`), it MUST pass `ptr::null_mut()`, NOT the result of `vec![].as_mut_ptr()`. An empty `Vec`'s `as_mut_ptr()` returns a dangling non-null pointer -- MuJoCo sees it as valid and writes through it, causing UB.
- **Return value conventions vary per function**: `mj_saveLastXML` uses 1=success/0=error, but `mj_addContact` uses 0=success/1=buffer-full. Always verify each function's convention from the C header comment. Flipping these is silent and hard to catch.
- **Boolean flag inversion**: `flg_local as i32` is correct when the C function expects 1=local/0=world. Watch for accidental `!flg_local as i32` which silently inverts the meaning with no crash.
- **`mj_view_indices!` last-joint edge case**: for the final joint in a model there is no next address entry, so the macro falls back to `$max_n` (e.g. `nq` for qpos-addressed fields) as the exclusive end. Double-check this path when adding new fields that use `mj_view_indices!`.
- **Free/ball joints and nq vs njnt**: `nq != njnt` for models with free joints (7 qpos each) or ball joints (4 qpos each). `mj_model_nx_to_nitem!(model, nq)` returns `njnt` not `nq`. The `mj_view_indices!` macro computes the correct per-joint slice within the total array.
- **Send/Sync bounds for generic wrapper types**: When a wrapper is generic over a type parameter, any `unsafe impl Send/Sync` MUST require the generic parameter to also be `Send` / `Sync` respectively. Without these bounds, non-thread-safe inner types would be incorrectly marked as `Send + Sync`. Note: `MjvScene` is now non-generic (as of 3.0.0) and unconditionally `Send + Sync`; this pitfall applies to `MjData<M>` and any future generic wrapper types.

## Comprehensive code verification
When doing a deep audit for type mismatches, stride errors, UB, or memory safety issues:
1. Read the codebase enough to build a numbered task list covering all wrappers and macros.
2. Split the task list into 10 equal, non-overlapping chunks and spawn 10 parallel agents -- each handling only its chunk.
3. Collect findings and check for consensus. Where agents disagree, spawn small focused batches to resolve.
4. Run 3 evaluation passes to deduplicate and rank findings.
5. Write up a final bug/fix report and apply fixes. See the `/verify` workflow for the full procedure.

> Do NOT run `cargo expand` and analyze on it unless the user explicitly requests it -- it is extremely slow.
