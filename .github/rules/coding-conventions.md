# Coding Conventions

## Hard rules - MUST follow
- **Never edit `src/mujoco_c.rs`** - it is auto-generated via bindgen. Editing it will be overwritten.
- **Never commit `expanded.rs`** - it is a temporary artifact from `/expand-macros`.
- **Always verify macro changes** with `/expand-macros` before considering work done (see `macro-system.md`).
- **Cross-reference with MuJoCo C docs** when adding or modifying wrapper fields (see `important-context.md`).
- **Prefer existing macros** in `src/util.rs` over writing manual accessor methods. Read `macro-system.md` for details.
- **Verify stride against C headers**: see the verification checklist in `macro-system.md`.
- **Update `Cargo.toml` excludes**: when adding *permanent* non-crate files (CI configs, dev
  scripts, docs), add them to the `exclude` list in `Cargo.toml` to prevent crates.io publishes.
- **Never add temporary or generated files to `Cargo.toml` `exclude`** - one-off artifacts such as
  the `/safety` audit report (`mujoco-rs-memory-safety-audit.html` / `.md`), `expanded.rs`, or other
  generated reports must NOT be listed in `exclude`. Keep them out of the published crate by
  `.gitignore`ing them instead (Cargo respects `.gitignore` when packaging); the `exclude` list is
  only for files that are committed to the repository long-term.
- **Do not fix pre-existing style issues in committed code.** Only flag and fix style problems
  (e.g. RST-style double backticks in Rust doc comments) in **new, uncommitted changes**. Existing
  committed code on `develop` or `main` should be left as-is unless it contains actually invalid
  or broken syntax.
- **Do not make unnecessary edits to already compiling code.** If the requested task is satisfied
  and the code compiles, avoid extra refactors, renames, or cleanup-only changes that are not
  required for correctness.

## Feature flags
- Read `Cargo.toml` to discover available features and their default state.
- Viewer and renderer features are opt-in (not in defaults). Enable them explicitly when needed:
  `--features "viewer-ui renderer-winit-fallback"`.
- The `cpp-viewer` feature requires static linking - do not enable it unless the static build has been set up.
- The `ffi-regenerate` feature rebuilds `src/mujoco_c.rs` from headers. **Never trigger this feature as an agent.**

## Error handling
- Standard pattern: the panicking wrapper calls the `try_` variant with `.unwrap()`; never duplicate
  the logic. Only the `try_` variant contains the implementation; the panicking version is a thin
  wrapper. Use `.unwrap()` instead of `.expect("X failed")` when the failure context is obvious
  from the function name (visible in the panic backtrace). Reserve `.expect("...")` for messages
  that add genuine diagnostic value beyond the function name (e.g. allocation failures, specific
  C-API return codes).
- **Method ordering**: place the panicking method first, then the `try_` variant immediately after.
  This matches the convention in `mj_data.rs` and other wrapper files.
- **When to apply the try_/panicking split**: only for methods returning `Result<T, E>` where
  **T is not `()`** and the failure represents a **programmer error** (bad index, size mismatch,
  invalid argument that could be validated beforehand). Methods with runtime/environmental errors
  (I/O, parsing, GL init) stay `Result`-only. Methods returning `Result<(), E>` stay `Result`-only
  since the ergonomic gain from a panicking wrapper is minimal (just `.unwrap()`).
- **CString/CStr panics are exempt from conversion to `Result`**: converting C string inputs to
  `CString` or checking C string validity via `.unwrap()` is intentional. Do NOT convert these to
  `Result`. However, `# Panics` documentation is still **required** for any function that can panic,
  including CString panics. Note: functions accepting `AsRef<Path>` should still return `Result` for
  invalid UTF-8 paths, since that is a realistic runtime condition (not a programmer error).
- **`src/wrappers/fun/` functions stay panicking**: functions in this module should panic on failure,
  not return `Result`.
- **`# Panics` docs state the trigger, not the reason**: a `# Panics` section must say **that** the
  function panics and **what** triggers it (the failing precondition, e.g. "Panics if `perturb.select`
  is out of range for the model in `data`"). Do NOT explain **why** the guard exists or what would
  otherwise go wrong (e.g. "because MuJoCo would read `xpos`/`body_bvhnum` out of bounds") -- that
  rationale belongs in a code comment at the guard site, not in the public panic docs. The same
  applies to `# Errors` sections: name the condition, not the underlying mechanism.

## Code minimality (DRY, YAGNI, KISS)

- **DRY**: Every piece of logic must have a single representation. If two functions share the same
  body differing only in a type or a minor parameter, make the function generic or extract a shared
  helper. Never duplicate an implementation.
- **YAGNI**: Only implement what is explicitly asked for. Do NOT add speculative parameters, extra
  overloads, "just in case" variants, or helper types that no caller currently uses.
- **KISS**: Prefer the simplest solution that fully solves the problem. Avoid clever or convoluted
  designs. A longer but readable solution is better than a short but tricky one.
- **Generics over duplication**: When two or more functions share identical logic but differ only
  in numeric type (e.g. `i32` vs `i64`), make the function generic with an appropriate trait bound
  (e.g. `Into<i64> + Copy`) rather than writing separate `_sz`/`_typed` variants.
- **No redundant wrappers**: Do not create a new function that solely delegates to another function
  of the same name with no added logic. Prefer a single function with a clear signature.
- **No unused parameters**: Every macro and function parameter must be used. Remove any parameter
  that was introduced speculatively but has no current caller.

## Safety guards at the FFI boundary

A safe (non-`unsafe`) wrapper must never be able to cause undefined behaviour for *any* input
reachable through the safe API. When a wrapper passes a caller value to C that C uses without its
own validation (e.g. as an unchecked array index or discriminant), pick the *cheapest sound* guard
in this order:

1. **Type / zero-cost encoding.** Choose a parameter type that cannot represent an invalid value
   (e.g. a Rust enum instead of `i32`), so validity holds by construction with no runtime cost.
   Beware enums that still include out-of-range discriminants for the specific C use (e.g. `MjtObj`
   contains meta variants `>= mjNOBJECT`) -- typing alone does not make those safe.
2. **O(1) check.** A single `assert!` (e.g. one range comparison, or a `fixedcamid < ncam` guard) is
   fine and should stay safe. Inline it; do NOT extract a one-off validation helper function (a
   shared helper is justified only when the same non-trivial logic recurs in three or more places).
3. **`unsafe fn` + `# Safety`.** If the only guard would require iterating caller-supplied data, or
   sits on a hot path (e.g. per-frame render/update), do NOT add the loop and do NOT leave the
   method safe-but-unsound. Make the method `unsafe` and document the precondition in a `# Safety`
   section. This is the honest contract: "there is a precondition I cannot cheaply check; the caller
   must uphold it." It fits the crate's existing unsafe-API tier and costs nothing at runtime.

Additional rules:

- **Do not duplicate checks the C layer already performs.** If the C/C++ function validates or
  clamps an argument itself, the wrapper must not re-check it (cross-reference the C source per
  `important-context.md`). Only guard cases C reads/indexes *before* its own check.
- **`unsafe` does not propagate soundly through a safe caller.** Marking a leaf method `unsafe` only
  helps when the user calls it directly with their own data. If a *safe* high-level method calls it
  internally on state the user can corrupt through another safe path (e.g. `MjViewer::render` ->
  `MjvScene::render`, where the user can mutate geom fields via `user_scene_mut`), making the leaf
  `unsafe` is insufficient -- the safe caller cannot uphold the precondition. Such cases need the
  guard at the choke point (an O(1)/cheap check) or a redesign that makes the invalid state
  unrepresentable; do not "fix" them by making the leaf `unsafe`.

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
- **Type aliases (`pub type`) are not wrappers.** Do NOT write "Wraps `X`" in doc comments for type
  aliases. Just describe what the type represents. "Wraps" language is fine for struct methods that
  call C functions.
- Always use ASCII characters only. Avoid non-ASCII Unicode characters (e.g., em dashes, arrows,
  smart quotes). Use `--` (double hyphen) as the ASCII substitute for em dashes. This applies to
  both source code and `.github/` rule/skill files.
- **Comment style**: Use `//` for regular comments and subsections. Use `/* */` for top-level
  section headers and special formatting.
- **Macro call alignment**: within a contiguous block of macro invocations (e.g. rows in a Grid),
  all arguments should be column-aligned. Pad the field expression so string label arguments start
  at the same column across all rows in the block. For rows that also have a trailing argument
  (e.g. `speed=`, `range=`, or a map constant), pad the string label so all trailing arguments
  also start at the same column. Rows without a trailing argument (e.g. `bool_row!`) only need
  the field-column alignment.
- **Import organization**: Group by module (std, external, internal), sort by line length
  (longest first), separate groups with empty lines.
- **Opening brace placement**: When a function signature fits on one line, place `{` on the
  same line (K&R style). When parameters are split across multiple lines, place `{` on its
  own line (Allman style) so the body is visually separated from the signature.
- **Avoid crate::prelude imports**: Import from actual modules instead of `crate::prelude`.
- Do NOT flag or fix integer truncation casts (e.g., `usize as i32`, `len() as i32`) when the
  underlying C API already validates or clamps the value (e.g., passing a count that MuJoCo itself
  allocated and therefore cannot exceed the C-side maximum). Only fix truncation casts when the
  value originates in pure Rust and is not subsequently validated by the C layer.
- Prefer `if let` over `matches!` unless `matches!` is clearly more convenient (e.g. inside
  `assert!` in tests). In conditionals, use `if let Some(x) = expr` instead of
  `if matches!(expr, Some(x))`.
- **`debug_assert!` policy**: Use `debug_assert!` for invariants that are guaranteed by the
  implementation and would only fail due to a bug in MuJoCo or mujoco-rs itself (not due to
  valid user input or normal runtime conditions). Use `assert!` (or `Result`-returning error
  handling) for conditions that can fail in correct code, such as allocation failures or
  user-facing precondition violations. Allocations are not guaranteed to succeed, so allocation
  checks must use `assert!` or `Result`-returning error handling instead of `debug_assert!`.

## Documentation
- Always verify any changes against MuJoCo's official documentation to ensure correctness.
- Any changes made should be reflected in the changelog: `docs/guide/source/changelog.rst`.
  Make sure to follow the conventions and style of previous changelog entries.
- **Documentation-only changes** (adding or improving doc comments, fixing typos in docs) do NOT
  need changelog or migration guide entries.
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
  - Prefer lines below ~100 characters. Hard limit: 120 characters.
- **Changelog section ordering.** Each version entry in `changelog.rst` uses `.. rubric::` sections
  in this fixed order: Breaking changes, Error handling, New features and improvements, Bug fixes,
  Other changes. Additional component-specific subsections (e.g. MjViewer) may appear only when a
  component has substantial standalone changes; they are placed between New features and Bug fixes.
  Not all sections are required for every release, but the relative order must be preserved.
- **Changelog content scope.** Only document public-facing changes in the changelog. Do NOT add
  entries for private/internal items (private fields, private methods, internal helpers) unless the
  change is critical to understanding behaviour visible from the public API. Bug fixes should only
  be documented if they were present in a previously released version; do not document fixes for
  bugs that were introduced and fixed within the same (unreleased) development cycle.
- **Changelog entries are concise about WHY.** Keep each entry focused on the user-facing change:
  name the item, what changed (e.g. "is now an `unsafe fn`", "now returns `Err(...)`"), and any
  action the user must take. A short one-sentence note on why the change was needed is fine (e.g.
  "which stored an unvalidated index the renderer trusted"), but do NOT include a detailed
  mechanism/root-cause walk-through of the underlying bug (which C function indexed what out of
  bounds, byte-overflow arithmetic, cursor-truncation behaviour, etc.). That depth belongs in a
  code comment at the fix site, not in the changelog. (This applies to new/edited entries only; do
  not rewrite existing committed entries.)
- **Migration guide entry format.** Each breaking change in `migration.rst` gets: a descriptive
  heading (RST `-` underline, matching the second-level subsection style already established in the file), a prose explanation, then **Before** and **After** code blocks using
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
- **docs-rs links must point to non-deprecated items.** When referencing an API item in RST/Sphinx
  documentation via `:docs-rs:`, always link to the current (non-deprecated) target path. When the
  display text needs to show the old/deprecated name, use the `display text <path>` form:
  `:docs-rs:\`MjData::get_state <mujoco_rs::...::<method>state>\`` -- displays the old name but
  links to the new method. Do not use `~`/`~~` inside the `<path>` of the display-text form; those
  prefixes only affect the auto-generated display name. Never create a `:docs-rs:` link whose target
  path resolves to a `#[deprecated]` item.
- **Version tagging and branching conventions.** Git release tags use no `v` prefix (e.g., `2.3.5`,
  `3.0.0`). Release branches follow the `vMajor.Minor.x` convention (e.g., `v2.3.x`, `v3.0.x`),
  where `v` and `x` are literal/fixed. Use these when diffing against previous releases
  (e.g. `git diff 2.3.5 HEAD`).

## Examples
- Every example in `examples/` must be registered in `Cargo.toml` with `[[example]]` and
  `required-features` if it uses optional features (e.g. `viewer`).
- Examples are user-facing demos. `use mujoco_rs::prelude::*` is the conventional import
  pattern; do NOT flag it as a violation of the "avoid crate::prelude" rule.
- Use `.unwrap()` (not `.expect("msg")`) for failures that are obvious from the function name.
- Use `/* */` block comments for top-level section headers inside function bodies (matches the
  established style in all other examples).
- Only add comments that clarify non-obvious intent. Do not comment self-explanatory builder
  chains or single-expression statements.
- New examples should be documented in the changelog under a `.. rubric:: New examples` section,
  referencing the file with `:gh-example:\`Display name <filename.rs>\``.

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
