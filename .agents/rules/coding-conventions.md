---
trigger: always_on
---

# Coding Conventions

## Hard rules - MUST follow
- **Never edit `src/mujoco_c.rs`** - it is auto-generated via bindgen. Editing it will be overwritten.
- **Never commit `expanded.rs`** - it is a temporary artifact from `/expand-macros`.
- **Always verify macro changes** with `/expand-macros` before considering work done.
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- **Prefer existing macros** in `src/util.rs` over writing manual accessor methods. Read `macro-system.md` for details.
- **Verify stride against C headers**: every new `info_method!` field must have its stride confirmed against the dimension comment in `mujoco/include/mujoco/mjmodel.h` or `mjdata.h` (e.g. `// (nbody x 3)` means stride 3).

## Feature flags
- Read `Cargo.toml` to discover available features and their default state.
- When building on headless systems (servers, CI), disable defaults: `--no-default-features`. NOTE: The current agentic test environment handles rendering gracefully, so defaults are fine here.
- The `cpp-viewer` feature requires static linking - do not enable it unless the static build has been set up.
- The `ffi-regenerate` feature rebuilds `src/mujoco_c.rs` from headers. **Never trigger this feature as an agent.**

## Code style
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
- Use `/// Safety` doc comments on all `unsafe` blocks.
- Public items should have `///` doc comments.
- Always use ASCII characters only. Never use UTF-8 (e.g., em dashes, arrows, etc.).
- Imports (`use`) should be sorted by line length with the longest line on top.
- Prefer to group imports with the same parent module.

## Documentation
- Always verify any changes made to MuJoCo's official documentation to verify everything is correct.
- Any changes made should be reflected in the changelog: `docs/guide/source/changelog.rst`.
  Make sure to follow the conventions and style of previous changelog entries.
- Always make sure MuJoCo-rs's documentation in `docs/guide` stays up to date with the changes.

## Comprehensive verification
- For deep audits (new major features, suspicions about correctness), use the `/verify` workflow.
- `/verify` uses parallel subagents with split workloads and consensus checking.
- Do NOT run `cargo expand` unless the user explicitly requests it -- it is very slow.
- Run `/test` as a baseline before any audit and after every fix.
- For memory safety beyond Rust's guarantees (FFI boundary), run `/asan` and `/miri`.
