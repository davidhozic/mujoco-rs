# Coding Conventions

## Hard rules - MUST follow
- **Never edit `src/mujoco_c.rs`** - it is auto-generated via bindgen. Editing it will be overwritten.
- **Never commit `expanded.rs`** - it is a temporary artifact from `/expand-macros`.
- **Always verify macro changes** with `/expand-macros` before considering work done.
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- **Prefer existing macros** in `src/util.rs` over writing manual accessor methods. Read `macro-system.md` for details.
- **Verify stride against C headers**: every new `info_method!` field must have its stride confirmed against the dimension comment in `mujoco/include/mujoco/mjmodel.h` or `mjdata.h` (e.g. `// (nbody x 3)` means stride 3).
- **Update `Cargo.toml` excludes**: when adding files or directories that are not part of the published crate (e.g. CI configs, internal tooling, docs, `.context/` entries, dev scripts), add them to the `exclude` list in `Cargo.toml` so they are not included in crates.io publishes.

## Feature flags
- Read `Cargo.toml` to discover available features and their default state.
- When building on headless systems (servers, CI), disable defaults: `--no-default-features`. NOTE: The current agentic test environment handles rendering gracefully, so defaults are fine here.
- The `cpp-viewer` feature requires static linking - do not enable it unless the static build has been set up.
- The `ffi-regenerate` feature rebuilds `src/mujoco_c.rs` from headers. **Never trigger this feature as an agent.**

## Code style
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
- Use `/// Safety` doc comments on all `unsafe` blocks.
- Do NOT duplicate `// SAFETY:` comments: if the same or nearly identical safety justification has
  already been stated in an earlier block in the same function or impl scope, omit it from subsequent
  blocks and trust the reader to refer upward. Only add a new comment when the reasoning differs.
- Public items should have `///` doc comments.
- Always use ASCII characters only. Never use UTF-8 (e.g., em dashes, arrows, etc.).
- Imports (`use`) should be sorted by line length with the longest line on top.
- Prefer to group imports with the same parent module.
- Do NOT flag or fix integer truncation casts (e.g., `usize as i32`, `len() as i32`) when the
  underlying C API already validates or clamps the value (e.g., passing a count that MuJoCo itself
  allocated and therefore cannot exceed the C-side maximum). Only fix truncation casts when the
  value originates in pure Rust and is not subsequently validated by the C layer.

## Documentation
- Always verify any changes made to MuJoCo's official documentation to verify everything is correct.
- Any changes made should be reflected in the changelog: `docs/guide/source/changelog.rst`.
  Make sure to follow the conventions and style of previous changelog entries.
- Breaking changes must also have a before/after migration entry in `docs/guide/source/migration.rst`.
- Always make sure MuJoCo-rs's documentation in `docs/guide` stays up to date with the changes.
- After adding or modifying public items or doc comments, run `/doc` to check for rustdoc warnings/errors.
  See `workflows/doc.md` for the exact command.

## Testing
- When adding a new feature or fixing a bug, add a test for it if one does not already exist.
- Tests must be **correctness tests** (verify behaviour is right), not build tests (verify it compiles).
  A test that only calls a function and asserts no panic is not sufficient on its own.
- Keep tests concise; avoid duplicating coverage that already exists in nearby tests.
- Run tests with `--no-default-features --features renderer` for renderer-only changes.
  See `workflows/test.md` for the full test command.

## Comprehensive verification
- For deep audits (new major features, suspicions about correctness), use the `/verify` workflow.
- `/verify` uses parallel subagents with split workloads and consensus checking.
- Do NOT run `cargo expand` unless the user explicitly requests it -- it is very slow.
- Run `/test` as a baseline before any audit and after every fix.
- For memory safety beyond Rust's guarantees (FFI boundary), run `/asan` and `/miri`.
