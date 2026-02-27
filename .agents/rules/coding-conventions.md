---
trigger: always_on
---

# Coding Conventions

## Hard rules — MUST follow
- **Never edit `src/mujoco_c.rs`** — it is auto-generated via bindgen. Editing it will be overwritten.
- **Never commit `expanded.rs`** — it is a temporary artifact from `/expand-macros`.
- **Always verify macro changes** with `/expand-macros` before considering work done.
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- **Prefer existing macros** in `src/util.rs` over writing manual accessor methods. Read `macro-system.md` for details.

## Terminal commands
- **Always append a trailing newline** after every terminal command to ensure the process actually executes and completes.

## Feature flags
- Read `Cargo.toml` to discover available features and their default state.
- When building on headless systems (servers, CI), disable defaults: `--no-default-features`. NOTE: The current agentic test environment handles rendering gracefully, so defaults are fine here.
- The `cpp-viewer` feature requires static linking — do not enable it unless the static build has been set up.
- The `ffi-regenerate` feature rebuilds `src/mujoco_c.rs` from headers. **Never trigger this feature as an agent.**

## Code style
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
- Use `/// Safety` doc comments on all `unsafe` blocks.
- Public items should have `///` doc comments.