---
description: How to build, check, or test the mujoco-rs crate
---

# Building / Checking / Testing mujoco-rs

Before running any `cargo build`, `cargo check`, `cargo test`, `cargo expand`, or similar build commands, **always** export the MuJoCo library environment variables first in the same command.

// turbo-all

1. Run the build/check/test command with the required environment variables:
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-3.5.0/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-3.5.0/lib/) && <your cargo command here> > /tmp/cargo_output.txt 2>&1; echo "EXIT=$?" >> /tmp/cargo_output.txt

```

2. Read the output file to see the results:
```
view_file /tmp/cargo_output.txt
```

> [!IMPORTANT]
> - Always prepend the two exports before any cargo command (build, check, test, expand, run, etc.).
> - Always redirect output to a file (e.g., `> /tmp/cargo_output.txt 2>&1`) because the terminal may not capture output reliably.
> - Always append a trailing newline **after** each command (leave an empty line after the command in the code block) to ensure the process completes.
> - Use `--lib` when running `cargo test` to avoid compiling examples that require optional features (e.g., `cargo test --lib`).
> - The working directory must be `/home/davidhozic/repo/mujoco-rs`.
> - The MuJoCo version directory is `mujoco-3.5.0`. Do **not** use a glob like `mujoco-*/lib` — multiple versions may be present.
