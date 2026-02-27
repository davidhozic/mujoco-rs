---
description: How to compile and run an example from the examples/ directory
---

# Running an Example

Compile and run one of the example programs.

// turbo-all

1. List available examples by exploring the `examples/` directory. The example name is the file stem (e.g., `basic.rs` → `basic`).

2. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

3. Run the chosen example (replace `X.Y.Z` and `<EXAMPLE_NAME>`):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && cargo run --example <EXAMPLE_NAME>

```

4. Check the terminal output for errors.

> [!NOTE]
> - Most examples open a GUI window and require a display + OpenGL.
> - Some examples may require specific Cargo features — read the example source file to check.
> - To run in release mode add `--release`.
