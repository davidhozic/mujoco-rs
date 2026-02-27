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

- Explore `src/` to understand the module structure — read `src/lib.rs` to see which modules exist and which are feature-gated.
- Explore `src/wrappers/` for safe Rust wrapper types.
- Explore `examples/` for usage patterns.
- Read `README.md` for a high-level overview.
- Read `Cargo.toml` for available features, dependencies, and the minimum Rust version.

## Common pitfalls
- MuJoCo C arrays can be dense or sparse (via address arrays). Read the C API docs to determine which applies.
- Address array fields documented as `-1: none` in MuJoCo should map to `Option` in the Rust wrappers for the item they point to. If the addressing value is -1, the resulting field MUST be `None`.
- The test environment handles rendering features gracefully; default features (including viewer) can be used.
- When in doubt, run `/expand-macros` and inspect the generated code.
- Multiple MuJoCo versions may be present on disk — always check `Cargo.toml` for the correct version.