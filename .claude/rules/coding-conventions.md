# Coding Conventions

The [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/checklist.html) (cited as `C-*`
ids) take precedence over this file; where they disagree, fix this file.

House style departs from the Rust Style Guide twice: imports sorted longest-line-first, and Allman
braces on multi-line signatures. This is why the repo does not run `rustfmt` (no `rustfmt.toml`, no
format check in CI). Do NOT run `cargo fmt`; format by hand to match the file you are in.

## Hard rules - MUST follow
- **Never edit `src/mujoco_c.rs`** (bindgen-generated). **Never commit `expanded.rs`** (`cargo expand`
  artifact).
- **Cross-reference the MuJoCo C docs** when adding or modifying wrapper fields (`important-context.md`).
- **Prefer the macros in `src/util.rs`** over hand-written accessors; **verify strides against the C
  headers** (checklists in `macro-system.md`).
- Add *permanent* non-crate files (CI configs, dev scripts, docs) to `exclude` in `Cargo.toml`.
  Never `exclude` temporary or generated files (`expanded.rs`, audit reports); `.gitignore` those.
- Fix style only in new, uncommitted changes; leave committed code as-is.
- **Do not churn code outside the task's blast radius**: no drive-by renames, reformatting, or
  cleanup-only edits. A cleanup task overrides this.
- Suppress lints with `#[expect(..., reason = "...")]`, never `#[allow]`.

## Feature flags
- Viewer and renderer features are opt-in: `--features "viewer-ui renderer-winit-fallback"`.
- The `cpp-viewer` feature requires static linking.
- **Never trigger the `ffi-regenerate` feature.**

## Error handling
- The panicking wrapper is a thin `.unwrap()` over the `try_` variant; only `try_` holds the
  implementation. Panicking method first, `try_` immediately after.
- Apply the split only when `T` is not `()` and the failure is a programmer error (bad index, size
  mismatch). Runtime/environmental failures (I/O, parsing, GL init) stay `Result`-only.
- `.unwrap()` when the function name is enough context; `.expect("...")` only for real diagnostic
  value (allocation failures, C-API return codes).
- Never `Result<T, String>` (C-GOOD-ERR). Errors are the typed enums in `src/error.rs`: a variant per
  failure carrying the identifying data, plus `Debug`, `Display`, and `Error::source()`. Hand-rolled;
  no error-derive dependency. `#[non_exhaustive]` when variants are feature-gated.
- Attach context with `.map_err(|source| ...)`; no blanket `From` impls; never `.map_err(|_| ...)`.
- `Display` states this layer's failure only; never interpolate `source()`. Lowercase, no trailing
  period. No `Box<dyn Error>` in the library.
- `CString`/`CStr` conversion panics are exempt from `Result`; `AsRef<Path>` UTF-8 failures are not.
  `src/wrappers/fun/` functions panic on failure.
- `# Panics` is required for anything that can panic (`assert!`, `unwrap()`, `todo!`, slice
  indexing). State the trigger, not the rationale; the rationale goes in a comment at the guard
  site. Same for `# Errors`.

## Traits, visibility, conversions
- Every public type derives `Debug` (C-DEBUG); hand-write it, ending in `finish_non_exhaustive()`,
  when a derive would dump something useless (C-DEBUG-NONEMPTY).
- Derive `Clone`/`Copy`/`PartialEq` where cheap and meaningful; no `Eq`/`Hash`/`Ord` on
  float-carrying types; no speculative derives (YAGNI).
- A conversion between two of our own types is a `From` impl (C-CONV-TRAITS), not an inherent
  `from_x`. Keep an inherent constructor when the input is foreign or needs extra arguments.
- Interdependent fields stay private (C-STRUCT-PRIVATE); self-standing values may be `pub`. (FFI
  structs follow the rules below.)
- `pub` for anything a downstream caller could use; private when module-internal; `pub(crate)` only
  for items that cross an internal boundary but would be misleading or unsound downstream (C-HIDDEN).

## Safety guards at the FFI boundary
A safe wrapper must be sound for every input reachable through the safe API. Pick the cheapest sound
guard, in order:

1. **Type / zero-cost encoding** (enum over `i32`). Beware out-of-range discriminants (`MjtObj` meta
   variants `>= mjNOBJECT`).
2. **O(1) inline `assert!`** (e.g. `fixedcamid < ncam`). No one-off validation helpers.
3. **`unsafe fn` + `# Safety`** when the only guard would iterate caller data or sit on a hot path.

- Do not duplicate checks the C layer already performs; guard only what C reads before its own check.
- `unsafe` on a leaf does not help if a safe caller can reach it with corruptible state
  (`MjViewer::render` -> `MjvScene::render` via `user_scene_mut`); guard at the choke point or make
  the invalid state unrepresentable.
- Edition 2024: an `unsafe fn` body is not an implicit unsafe block; each unsafe operation gets its
  own `unsafe { }` with a `// SAFETY:` comment.
- Build raw pointers with `&raw const` / `&raw mut`. A constant C string is a `c"..."` literal, never
  `CString::new("..").unwrap()`.
- Non-null raw FFI pointers: `&*ptr` / `&mut *ptr`, not `.as_ref().unwrap()`.
- State a `// SAFETY:` justification once per scope; do not repeat it on later blocks.

## Generated struct field visibility
Bindgen fields stay `pub` by default (plain data C reads as-is). Mediation is forced only by memory
safety, decided per struct (all fields demoted to `pub(crate)` together), when any field can cause
UB:

1. **Raw pointer** (auto-detected by `build.rs`).
2. **Char buffer C reads as a NUL-terminated string** (`[c_char; N]` via `strlen`/`printf`).
3. **Value C uses as an unchecked index/length/offset/discriminant** (e.g. `table[opt->field]`). An
   enum-as-int C only `switch`es on is safe and stays plain.

- A plain struct gets no accessors at all; raw field access is the interface. Never a `[force]`
  accessor on a plain struct (a user can write any value, so the enum cast is UB). Wanting an
  enum/`bool`/`&str` view means demoting the struct: add it to `CONFIG_DEMOTE` in `build.rs`.
- Exempt: structs with a dedicated wrapper in `src/wrappers/` (a `pub struct MjX` owning the raw
  struct; a bare `pub type` alias is not a wrapper), and internal-only structs the safe API never
  hands to a user (`mjui*`, `mjResource_`, `mjpPlugin_`, `mjSDF_`, `mjCache_`).
- A demoted struct gets an accessor per field: `getter_setter!` (`[force]` for enum-as-int, bool arm
  for `mjtByte`, `[&]` form for arrays), `c_str_as_str_method!` for char buffers.
- `build.rs` demotes in `generate_ffi()`; keep its hardcoded sets in sync with the wrappers.
  Regenerating bindings is developer-only (`ffi-regenerate`).

## Code minimality (DRY, YAGNI, KISS)
- **DRY**: one representation per logic; a generic (e.g. `Into<i64> + Copy`) or a shared helper
  instead of `_sz`/`_typed` duplicates.
- **YAGNI**: no speculative parameters, variants, or helper types; no unused parameters.
- **KISS**: the simplest solution that fully solves the problem.
- **No redundant wrappers or named one-liners**; exception: a `T::new` fronting a sole `From` impl.
- **Return `impl Iterator<Item = T>`; take `impl IntoIterator<Item = T>`**; the caller collects.
  Return `Vec` only when the body must materialize it (sorts, indexes, walks twice). Do not invent
  lifetime-carrying structs to make a lazy return possible. Name iterator methods as plurals
  (`lines`, `chars`; C-ITER).
- **Iterator combinators over index loops**; `.collect()` over `with_capacity` + `push`;
  `take_while`/`find`/`position`/`scan` over predicate `while` loops. Keep the explicit loop for
  real imperative state, early `return`/`break`, or a greedy unfold.
- `Option`/`Result` combinators over rewrapping `match`es; no needless `clone`/`to_vec` on borrowable
  data. Move a dead value instead of copying (pass by value, `mem::take`/`mem::replace`).
- Use an existing dependency's own type/method instead of hand-rolling; do not add a dependency for
  a one-liner.
- Consolidate genuine cross-module duplication into the owning module; extract shared infrastructure
  instead of widening many items to `pub(crate)`.

## Design and refactoring
- **Prefer the larger, correct refactor** over the small patch that keeps a smell. Delete superseded
  helpers and dead guards; never keep two ways to do one thing. Prove the refactor: build, clippy,
  tests, byte-identical output where deterministic (e.g. saved model XML).
- **Follow the smell to its cause**: a behaviour parameter means callers disagree about when work
  happens; mutually exclusive parameters mean one type does two jobs; a field dead in one variant
  means the variants are wrong.
- **Never trade a smell for a smell**: a callback wrapped in a fresh enum is the same design.
- **No workarounds for missing mechanisms**: no shadow structs, sentinel values, stringly flags,
  `static mut` globals, source-rewriting macros. Solve at the owning layer; a verbose honest impl
  beats a hack. (Sanctioned exception: the `build.rs` demotion pass.)
- **Parameters describe data, not behaviour**: no strategy callbacks, no mode flags, no mutually
  exclusive pairs; fix the pipeline so callers agree. Exempt: closures a foreign API demands
  (`mjcb_*` hooks) and combinator thunks (`.unwrap_or_else(|| ...)`).
- **A free function whose subject is our own `&T`/`&mut T` is a method**; write it on `T`. Foreign
  `T` or two equal subjects: free function in the owning module.
- **Constructors**: `from_<input>` associated functions on the built type, in the type's own module;
  a `From` impl when the single input is our own type; `T::new` for canonical assembly; a sole
  `From` still gets a delegating `T::new`. `from_*` returns one value, never a `Vec`. Exempt:
  `T::parse`, same-type transformations, foreign structs.

## Code style
- Match the conventions of the file you are modifying.
- `/// # Safety` on `unsafe fn`; `// SAFETY:` on `unsafe {}` blocks.
- `///` on every public item. A doc is **definition + contract only**: what the item is, returns, and
  its units, plus the precondition, invariant, panic/error condition, and formula. Cut story,
  provenance, exclusions, and self-evident mechanics. One or two sentences on an ordinary item;
  longer rationale goes in a `//` comment or the `//!` header. The `Wraps` sentence below is
  sanctioned on top of this.
- Every C-wrapping method's doc carries `Wraps [\`c_function_name\`].` (fall back to
  `[\`func\`](crate::mujoco_c::func)` on unresolved links; macro-generated accessors exempt). Never
  "Wraps" on a `pub type` alias.
- ASCII only; no em dash of any spelling (rustdoc prints `--` literally). Exception: RST under
  `docs/` uses `---` (see Documentation).
- Comments: `//` for regular comments, `/* */` for section headers. Capitalized, trailing period, one
  space after `//`, wrap at ~100 columns, at most two lines. Only the non-obvious *why* (gotcha,
  ordering, workaround), placed at the line it explains; never restate the code. Keep comments
  current with the code.
- Macro call alignment: column-align arguments within a contiguous macro block (Grid rows), trailing
  arguments included.
- Imports: grouped (std, external, internal), longest line first, blank line between groups.
- Braces: K&R on one-line signatures, Allman on multi-line. Modules: `foo.rs` + `foo/bar.rs`, never
  `foo/mod.rs`.
- Naming (C-CASE / C-GETTER / C-CONV): `MjvScene`, not `MJVScene`; getters without `get_`;
  `as_`/`to_`/`into_` by conversion cost. No abbreviations (`value` not `val`, `index` not `idx`);
  conventional ones stay (`i`, `j`, `e`/`err`, MuJoCo names: `nbody`, `qpos`, `jac`, `rgb`).
- Elide inferable types (`.collect::<Vec<_>>()`); annotate only when load-bearing.
- Constants live at module scope with a `///` doc, not inside function bodies.
- Widen with `From` (`f64::from(x)`), not `as`; `as` only for the lossy index/count casts to C ints.
  Do not flag truncation casts the C layer validates.
- Unit variant: `==`; binding: `if let`; multi-alternative: `matches!`.
- `assert!` for caller preconditions and FFI guards; `debug_assert!` only for invariants the
  implementation itself guarantees; never a side-effecting call inside `debug_assert!`; allocation
  checks use `assert!` or `Result`.

## Modern Rust (edition 2024, MSRV 1.95)
- A returned `impl Trait` captures all in-scope lifetimes automatically; no `<'a>` + `+ 'a` plumbing
  (`use<..>` only to narrow). Named borrowing return types still spell the lifetime.
- `let ... else` for bind-or-bail; let-chains (`if let ... && ...`) over nesting; `if let` guards
  (`Some(id) if let Some(x) = lookup(id) =>`) over an `if let` nested in the arm body. If the guard
  fails, the arm does not match and a later arm runs. Keep the nested `if` when the arm must consume
  the value and then do nothing.
- std helpers over hand-expanded formulas: `f64::midpoint`, `hypot`, `powi(2)`, `to_radians()`,
  `div_ceil`.
- Float ordering: `f64::total_cmp`, never `partial_cmp(..).unwrap()`.
- `is_some_and` / `is_none_or` over `map_or`.
- `LazyLock` / `OnceLock`, never `once_cell`/`lazy_static` or `static mut`.
- `cfg_select!` for a mutually exclusive platform or feature choice; each arm holds one expression
  or one braced block of items. Never the `cfg-if` crate. A match arm and a struct field keep their
  own `#[cfg(..)]`, because a macro cannot expand to either one.
- Fixed-length views over `slice[..N].try_into()`: `first_chunk::<N>()` / `first_chunk_mut::<N>()`
  for a prefix, `as_array::<N>()` / `as_mut_array::<N>()` only when the length is exactly `N`. A
  stride view is usually longer than `N` (a free joint `qpos` is 7), so `first_chunk` fits it.
  `[T; _]` works in a `let` annotation or a call, never in an item signature. An item signature
  includes a parameter, a return type, a struct field, a constant, a `static` and a type alias.
- Index and count math that must not wrap: `strict_add` / `strict_sub` / `strict_mul` over
  `checked_*(..).unwrap()`; they panic in release too. Keep `checked_*` where the overflow becomes
  a `Result`.
- Element position: `as_flattened()` + `element_offset()` over an `unsafe` `offset_from` on two
  element pointers.
- Honour the pointer and lifetime lints instead of silencing them: `dangling_pointers_from_locals`,
  `integer_to_ptr_transmutes` (use `ptr::with_exposed_provenance`), `function_casts_as_integer`,
  `mismatched_lifetime_syntaxes`. The first three mark a real defect, the last one an unclear
  signature; fix the cause.

## Module structure
- Siblings are different concerns; faces of one abstraction nest under a parent that names it and
  owns the shared helpers. No new `common`/`util`/`core` grab-bags (`src/util.rs` is the sanctioned
  macro home). A parent is never a bare re-export list. Nest at most three levels. A module
  everything uses is a layer, not a member.
- Moving a type renames its public path: a breaking change, recorded in `changelog.rst` and
  `migration.rst`. No re-exports at the old path.
- Item order (new files and files a task already rewrites): constants/statics -> structs -> enums ->
  functions; each type followed by its impls (inherent first, then traits); within a category `pub`
  -> `pub(crate)` -> private; then dependents before dependencies. Do not reorder existing files on
  their own.

## Documentation
- Verify changes against MuJoCo's official documentation. Keep `docs/guide` current; run `/doc`
  after any change to public items, doc comments, or `.rst` files.
- Public-facing changes go in `docs/guide/source/changelog.rst`; breaking changes also get a
  Before/After entry in `migration.rst` (`.. code-block:: rust`, or a `.. list-table::` for simple
  signature changes). Doc-only changes need neither.
- Link what you name (C-LINK). Pin concrete versions, never `latest` aliases or deprecated targets
  (use `display text <path>` to show an old name); the `:docs-rs:` and `:gh-example:` roles already
  pin the version.
- **Link every item at its first mention.** The first time a page or a changelog entry names a
  struct, enum, trait, type alias, function, method, field or variant, write it as a `:docs-rs:`
  link, or as a `|substitution|` that holds one. This covers a `mujoco_c` function and type as
  well. A crate we depend on, and a type of that crate, take an upstream link with a pinned
  version at their own first mention (`egui`, `log`, `env_logger`). Later mentions stay plain
  ``literals``. Three cases take no link: a `std` item, an item the release removes (its page is
  gone, so link the owning type instead), and a MuJoCo data field, XML attribute or C++ internal
  class that the prose only names (``qpos``, ``ctrlrange``, ``flexcomp``, ``mjCWrap``).
- **Keep the sentence, add the link.** When the item sits inside a compound literal
  (``Err(MjDataError::BufferTooSmall)``, a signature), use the role's `display text <path>` form,
  or put the link in a short parenthesis after the literal. Do not rewrite the sentence around the
  link.
- A doc does not mention less-visible items; make the item visible or drop the mention.
  (`crate::mujoco_c` is public.)
- Changelog conventions: rubric order Breaking changes, Deprecations, Error handling, New features
  and improvements, Bug fixes, Other changes; one concise WHY sentence per entry, no root-cause
  walk-throughs; no entries for private items or same-cycle fixes; new methods as fully-qualified
  `:docs-rs:` links nested under the owning type's `replace::` substitution; mark new items
  `:sup:\`new\``.
- **Only our decisions get a changelog entry.** A change that follows mechanically from a MuJoCo
  version bump gets none: an accessor added, removed or renamed because upstream changed the
  field, a shifted enum discriminant, an alias for a new upstream enum or struct, a new
  getter/setter pair, an accessor length that follows a new upstream layout. An entry records what
  we decided: a wrapper we wrote over a C function (with its config builder), a safety or type
  choice (`unsafe fn`, an enum instead of `i32`), an API shape, a deprecation, a bug fix, viewer
  and example work. A wrapper-defined item that a caller names by hand stays, even when upstream
  forced the change (`MjvCamera::move_` losing its `scene` parameter). The FFI-upgrade entry
  states the bump itself and points at MuJoCo's changelog. `migration.rst` follows the same
  split: it carries our decisions with Before/After code, and gives no section for an accessor
  that only mirrors a renamed, a removed or a resized upstream field, or for a shifted enum
  discriminant.
- **No mechanics in an entry.** State the change and what the reader must do. Do not explain how
  the C layer reaches a fault (which array a function indexes, which field it leaves unwritten,
  which cast made a test pass), do not list the fields of a struct or the variants of an enum, do
  not give per-type examples of a returned value, do not restate what the item already had, and do
  not argue that the wrapper is sound. A reason survives only when the reader acts on it (an
  unreachable error arm, a value that now differs, a range that changed without a compile error).
- **When trimming an existing doc, delete; do not reword.** The surviving sentences stay
  byte-identical, apart from the grammar fix that the deletion forces.
- **One entry per field, not one per surface.** A field that a view or an `Info` also exposes gets
  a single entry, written on the field or its array accessor, plus a short clause noting that the
  change also affects the views. Never describe the view change a second time in parallel, and
  never repeat the field list. Same rule for `migration.rst` prose; a Before/After block may still
  show view code when that is what the user must edit.
- RST: `|name|` substitutions for common types (`|mj_data|`, not `` ``MjData`` ``); double backticks
  for inline code; `---` for an em dash, `--` only as en dash or literal flag; lines under ~100
  (hard limit 120); `:gh-example:` for example references.
- After major changelog/migration edits, verify against HEAD and the previous release tag (code
  blocks must be valid Rust). Re-read edited docs and the README against the actual code.
- Release tags have no `v` prefix (`2.3.5`); branches are `vMajor.Minor.x` (`v2.3.x`).

## Examples
- Register every example in `Cargo.toml` (`[[example]]`, `required-features` when needed).
- `use mujoco_rs::prelude::*` is the convention here; `.unwrap()` on failure; `/* */` section
  headers; comments only for non-obvious intent.
- New examples go in the changelog under `.. rubric:: New examples`, referenced with `:gh-example:`.

## Testing
- Add a test per feature or fix. Correctness tests, not build or no-panic tests.
- **A test must be able to fail for a real bug**: no self-referential assertions (a getter against
  its own setter, a value against itself, a `Default` field against its own constant). Prefer round
  trips, independently derived numbers, boundary rejection, and refactor-fragile invariants. If you
  cannot name the bug it catches, do not write the test.
- Keep tests concise; no duplicate coverage. Renderer-only changes: `--no-default-features
  --features renderer`.

## Comprehensive verification
- `/verify` for deep audits; `/test` as baseline before and after; `/asan` and `/miri` for FFI
  memory safety.
- **Never run `cargo expand` or read expanded code.**
