---
description: How to run the library's unit tests
---

# Running Tests

Run the in-crate unit tests (tests live inside `src/` files under `#[cfg(test)]`).

// turbo-all

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Run all library tests (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && cargo test --lib

```

3. Verify that the exit code is 0 (success) — check the terminal output for errors.

> [!NOTE]
> - To run a single test: `cargo test --lib <test_name>`.
> - To run tests in release mode add `--release`.
> - Read `Cargo.toml` `[features]` for available features. On headless systems, disable defaults: `--no-default-features`.
