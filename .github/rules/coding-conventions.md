# Coding Conventions

## Hard rules - MUST follow
- **Never edit `src/mujoco_c.rs`** - it is auto-generated via bindgen. Editing it will be overwritten.
- **Never commit `expanded.rs`** - it is a temporary artifact from `/expand-macros`.
- **Always verify macro changes** with `/expand-macros` before considering work done (see `macro-system.md`).
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- **Prefer existing macros** in `src/util.rs` over writing manual accessor methods. Read `macro-system.md` for details.
- **Verify stride against C headers**: see the verification checklist in `macro-system.md`.
- **Update `Cargo.toml` excludes**: when adding files or directories that are not part of the published crate (e.g. CI configs, internal tooling, docs, `.context/` entries, dev scripts), add them to the `exclude` list in `Cargo.toml` so they are not included in crates.io publishes.

## Feature flags
- Read `Cargo.toml` to discover available features and their default state.
- Viewer and renderer features are opt-in (not in defaults). Enable them explicitly when needed: `--features "viewer-ui renderer-winit-fallback"`.
- The `cpp-viewer` feature requires static linking - do not enable it unless the static build has been set up.
- The `ffi-regenerate` feature rebuilds `src/mujoco_c.rs` from headers. **Never trigger this feature as an agent.**

## Error handling
- Standard pattern: the panicking wrapper calls the `try_` variant with `.unwrap()`; never duplicate
  the logic. Only the `try_` variant contains the implementation; the panicking version is a thin
  wrapper. Use `.unwrap()` instead of `.expect("X failed")` when the failure context is obvious
  from the function name (visible in the panic backtrace). Reserve `.expect("...")` for messages
  that add genuine diagnostic value beyond the function name (e.g. allocation failures, specific
  C-API return codes).
- **When to apply the try_/panicking split**: only for methods returning `Result<T, E>` where
  **T is not `()`** and the failure represents a **programmer error** (bad index, size mismatch,
  invalid argument that could be validated beforehand). Methods with runtime/environmental errors
  (I/O, parsing, GL init) stay `Result`-only. Methods returning `Result<(), E>` stay `Result`-only
  since the ergonomic gain from a panicking wrapper is minimal (just `.unwrap()`).
- **CString/CStr panics are exempt**: converting C string inputs to `CString` or checking C string
  validity via `.unwrap()` is intentional. Do NOT convert these to `Result`.
- **`src/wrappers/fun/` functions stay panicking**: functions in this module should panic on failure,
  not return `Result`.

## Code style
- Read existing code in the file you're modifying to understand naming, safety, and documentation conventions.
- Follow the existing error handling patterns used in the same file.
- Use `/// # Safety` doc sections on `unsafe fn` declarations to document caller obligations,
  and `// SAFETY:` inline comments on `unsafe {}` blocks to explain why the block is sound.
- For guaranteed non-null raw FFI pointers, use `&*ptr` / `&mut *ptr` directly. Do NOT use
  `.as_ref().unwrap()` / `.as_mut().unwrap()` -- those add an unnecessary Option round-trip.
- Do NOT duplicate `// SAFETY:` comments: if the same or nearly identical safety justification has
  already been stated in an earlier block in the same function or impl scope, omit it from subsequent
  blocks and trust the reader to refer upward. Only add a new comment when the reasoning differs.
- Public items should have `///` doc comments.
- Always use ASCII characters only. Avoid non-ASCII Unicode characters (e.g., em dashes, arrows,
  smart quotes).
- Imports (`use`) should be sorted by line length with the longest line on top.
- Prefer to group imports with the same parent module.
- Do NOT flag or fix integer truncation casts (e.g., `usize as i32`, `len() as i32`) when the
  underlying C API already validates or clamps the value (e.g., passing a count that MuJoCo itself
  allocated and therefore cannot exceed the C-side maximum). Only fix truncation casts when the
  value originates in pure Rust and is not subsequently validated by the C layer.
- Prefer `if let` over `matches!` unless `matches!` is clearly more convenient (e.g. inside
  `assert!` in tests). In conditionals, use `if let Some(x) = expr` instead of
  `if matches!(expr, Some(x))`.

## Documentation
- Always verify any changes against MuJoCo's official documentation to ensure correctness.
- Any changes made should be reflected in the changelog: `docs/guide/source/changelog.rst`.
  Make sure to follow the conventions and style of previous changelog entries.
- Breaking changes must also have a before/after migration entry in `docs/guide/source/migration.rst`.
- The migration guide is **only for breaking changes**. New non-breaking additions (e.g. new `try_`
  variants that don't change existing signatures) belong in the changelog only.
- Always make sure MuJoCo-rs's documentation in `docs/guide` stays up to date with the changes.
- After adding or modifying public items or doc comments, run `/doc` to check for rustdoc warnings/errors.
  See `workflows/doc.md` for the exact command.
- All public API changes must adhere to the
  [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/). Consult
  the guidelines when adding or modifying public types, traits, methods, naming,
  or error handling.
- **Use `:gh-example:` for example references** in Sphinx docs (`docs/guide/source/`). Always use
  the `:gh-example:` role (e.g. `:gh-example:\`tippe_top.rs\``) instead of plain backticks when
  referring to example files. Use the `<display text>` form for custom labels:
  `:gh-example:\`My example <my_example.rs>\``.
- **RST/Sphinx formatting conventions** for `docs/guide/source/`:
  - Use `.. |name| replace::` substitutions (defined at the top of each file) when referencing
    common types like `MjData`, `MjModel`, `MjSpec`. Use `|mj_data|` not `` ``MjData`` ``.
  - Mark new items with `:sup:\`new\`` (e.g. `` ``try_jac`` :sup:\`new\` ``).
  - Use `` ``double backticks`` `` for inline code (method names, types, values).
  - Use `.. code-block:: rust` for multi-line code examples in migration.rst.
  - Keep lines within ~100 characters; continuation lines are indented to match the list item.
- **Changelog section ordering.** Each version entry in `changelog.rst` uses `.. rubric::` sections
  in this fixed order: Breaking changes, Error handling, New features and improvements, Bug fixes,
  Other changes. Additional component-specific subsections (e.g. MjViewer) may appear only when a
  component has substantial standalone changes; they are placed between New features and Bug fixes.
  Not all sections are required for every release, but the relative order must be preserved.
- **Migration guide entry format.** Each breaking change in `migration.rst` gets: a descriptive
  heading (RST `~` underline), a prose explanation, then **Before** and **After** code blocks using
  `.. code-block:: rust`. For simple type changes, use a `.. list-table::` with columns for
  type/method, old signature, and new signature. Group related changes under the same heading.
- **Verify changelog/migration after major doc changes.** After substantial edits to
  `changelog.rst` or `migration.rst`, run a verification pass (compare claims against actual code
  on HEAD and the previous release tag) before considering the work done. This includes verifying
  that all `.. code-block:: rust` blocks contain syntactically valid Rust with correct method names,
  types, and signatures matching the actual code.
- **Run `/doc` after RST changes.** After editing any `.rst` file under `docs/guide/source/`,
  run `/doc` to verify both rustdoc and Sphinx builds are clean. This catches RST syntax errors,
  broken cross-references, and invalid custom roles that rustdoc cannot detect.
- **Version tagging and branching conventions.** Git release tags use no `v` prefix (e.g., `2.3.5`,
  `3.0.0`). Release branches follow the `vMajor.Minor.x` convention (e.g., `v2.3.x`, `v3.0.x`),
  where `v` and `x` are literal/fixed. Use these when diffing against previous releases
  (e.g. `git diff 2.3.5 HEAD`).

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
