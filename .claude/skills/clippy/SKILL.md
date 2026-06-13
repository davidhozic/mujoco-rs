---
name: clippy
description: Run cargo clippy for Rust linting. Use this when asked to lint the code or check for common mistakes.
---

# Running Clippy

Run `cargo clippy` to catch common Rust mistakes, style issues, and potential bugs.

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the
   package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Run clippy (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && cargo clippy --all-targets --features renderer -- -D warnings 2>&1
```

3. Verify the exit code is 0 (success). Fix any warnings before considering the work done.

> [!NOTE]
> - `--all-targets` checks lib, tests, examples, and benches.
> - `-D warnings` treats all warnings as errors so nothing is silently ignored.
> - `--features renderer` enables the renderer feature; adjust if working on viewer code
>   (e.g. `--features "renderer viewer-ui"`).
> - Do NOT fix integer truncation casts (e.g. `usize as i32`) when the C API validates
>   or clamps the value (see `coding-conventions.md`).
> - Do NOT suppress clippy lints with `#[allow(...)]` without a justifying comment.
> - To check a single file: `cargo clippy --lib --features renderer -- -D warnings`.
> - To auto-fix simple lints: `cargo clippy --fix --allow-dirty --features renderer`.
