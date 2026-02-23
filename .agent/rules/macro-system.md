# Macro System

This crate uses declarative macros extensively to reduce boilerplate when wrapping MuJoCo's C arrays.

## Before modifying wrapper files
1. **Read `src/util.rs`** first — it contains all core macro definitions. Understand their syntax before touching `mj_model.rs` or `mj_data.rs`.
2. **Run `/expand-macros`** after any macro-related change to verify generated code.
3. **Cross-reference with MuJoCo C API docs** (see `important-context.md`) for correct field sizes and types.

## Code generation
The `mujoco-rs-utils` CLI tool (at `../mujoco-rs-utils`) can auto-generate macro invocations from MuJoCo's C headers. Run it with `--help` to see available subcommands (e.g., `create-views`, `create-array-slice-macro-call`, `create-getters-setters`).
