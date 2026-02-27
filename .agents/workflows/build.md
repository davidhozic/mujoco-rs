---
description: How to build the mujoco-rs library
---

# Building MuJoCo-rs

Compile the library using a pre-downloaded MuJoCo shared library in the repository.

// turbo-all

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Build the library (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && cargo build --lib

```

3. Verify that the exit code is 0 (success) — check the terminal output for errors.

> [!NOTE]
> - If `.so` files are not found, read `build.rs` and `docs/guide/source/installation.rst` to understand the linking options.
> - Read `Cargo.toml` `[features]` section to discover available features.
> - To build with specific features: `cargo build --lib --no-default-features --features "feature1 feature2"`.
> - To build in release mode add `--release`.
> - `MUJOCO_DYNAMIC_LINK_DIR` must be an **absolute path** (hence `realpath`).
