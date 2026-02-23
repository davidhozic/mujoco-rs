---
description: How to build, check, or test the mujoco-rs crate
---

# Building / Checking / Testing mujoco-rs

Before running any `cargo build`, `cargo check`, `cargo test`, `cargo expand`, or similar build commands, **always** export the MuJoCo library environment variables first in the same command.

// turbo-all

1. Run the build/check/test command with the required environment variables:
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-*/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-*/lib/) && <your cargo command here>

```

> [!IMPORTANT]
> - Always prepend the two exports before any cargo command (build, check, test, expand, run, etc.).
> - Always append an extra newline after the command so the process terminates properly.
> - The working directory must be `/home/davidhozic/repo/mujoco-rs`.
