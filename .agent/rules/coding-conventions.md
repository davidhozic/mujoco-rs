# Coding Conventions

## Hard rules
- **Never edit `src/mujoco_c.rs`** — it is auto-generated via bindgen.
- **Always verify macro changes** with `/expand-macros` before considering work done.
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- Prefer using existing macros in `src/util.rs` over writing manual accessor methods.

## Patterns to follow
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
