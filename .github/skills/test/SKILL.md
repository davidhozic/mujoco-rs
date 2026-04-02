---
name: test
description: Run the in-crate unit tests for mujoco-rs. Use this when asked to run tests or verify that changes work correctly.
---

# Running Tests

Run all tests (unit tests inside `src/`, integration tests in `tests/`, and doctests).


1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Run all tests (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && cargo test --features renderer

```

3. Verify that the exit code is 0 (success) - check the terminal output for errors.

> [!NOTE]
> - To run a single test: `cargo test --features renderer <test_name>`.
> - To run tests in release mode add `--release`.
> - Read `Cargo.toml` `[features]` for available features. Viewer/renderer features are opt-in (not in defaults).
