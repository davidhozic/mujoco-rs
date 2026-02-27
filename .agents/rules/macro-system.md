---
trigger: always_on
---

# Macro System

This crate uses declarative macros extensively to reduce boilerplate when wrapping MuJoCo's C arrays.

## Before modifying wrapper files
1. **Read `src/util.rs`** first — it contains all core macro definitions. Understand their syntax before modifying any wrapper file.
2. **Run `/expand-macros`** after any macro-related change to verify generated code.
3. **Cross-reference with MuJoCo C API docs** (see `important-context.md`) for correct field sizes and types.

## Finding macros
- All macros are defined in `src/util.rs`. Search for `macro_rules!` to find them all.
- Read name and doc comments of each macro to understand its purpose before invoking it.
- Look at existing invocations in the wrapper files (`src/wrappers/`) to see usage patterns.

## Code generation tool
Check if a `../mujoco-rs-utils` directory exists. If it does, it is a CLI tool that can auto-generate macro invocations from MuJoCo's C headers. Run it with `--help` to see available subcommands.
