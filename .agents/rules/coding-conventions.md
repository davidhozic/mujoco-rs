---
trigger: always_on
---

# Coding Conventions

## Hard rules
- **Never edit `src/mujoco_c.rs`** — it is auto-generated via bindgen.
- **Always verify macro changes** with `/expand-macros` before considering work done.
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- Prefer using existing macros in `src/util.rs` over writing manual accessor methods.

## Terminal commands
- **Always append a trailing newline** after every terminal command to ensure the process actually executes and completes.
- **Redirect output to a file** (e.g., `> /tmp/output.txt 2>&1`) and then read the file with `view_file`, because the terminal output capture can be unreliable and return empty output.

## Patterns to follow
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
