---
name: doc
description: Build rustdoc and Sphinx documentation, verifying both for warnings, broken links, content correctness, and changelog/migration completeness against the code. Use this when asked to build docs, check doc comments, or verify documentation.
---

# Building Documentation

Build both rustdoc (API docs) and Sphinx (user guide) documentation and verify they
are clean.

## Part 1 - Rustdoc

Run rustdoc with all features enabled and the `DOCS_RS=y` flag (which activates
`#[cfg(docsrs)]` attributes) to catch any broken links, missing items, or
doc-comment errors.

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Build the docs (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && DOCS_RS=y cargo doc --no-deps --all-features 2>&1
```

3. Verify the output contains no `warning:` or `error:` lines.

## Part 2 - Sphinx (User Guide)

Build the Sphinx-based user guide (changelog, migration guide, installation, etc.)
to verify RST syntax, cross-references, and custom roles like `:gh-example:`.

The Sphinx toolchain lives in a `uv`-created virtual environment.
(it always provides `uv` and `sphinx-build`). `sphinx-build` is typically **not** on
the system `PATH`, so the build must go through that venv -- a bare `make html` fails
with `sphinx-build: command not found`.

1. Locate and activate the venv. By default it is `docs/guide/.venv`.

2. Build the HTML docs:
```bash
cd docs/guide && uv run --offline make html 2>&1
```
   Fallbacks if `uv` is unavailable: `make html`,
   or `. .venv/bin/activate && make html`.

3. Check the output for `WARNING:` or `ERROR:` lines. The following warning is
   pre-existing and can be ignored:
   - `WARNING: html_static_path entry '_static' does not exist`

## Part 3 - Content Verification

After both builds pass, verify the Sphinx documentation content is correct:

1. **Code blocks**: Every `.. code-block:: rust` block must contain syntactically valid
   Rust with method names, types, and signatures matching the actual code on HEAD.
2. **Factual claims**: Verify that claims about API behavior, error types, and return
   types are accurate against the source code.
3. **Outdated content**: Check that no documentation references removed methods, old
   signatures, or deprecated patterns that no longer exist on HEAD.
4. **Cross-references**: Verify that `:gh-example:`, `:docs-rs:`, and RST substitutions
   (`|mj_data|`, etc.) resolve correctly.
5. **Version-pinned links**: No documentation link may use the `latest` alias (e.g.
   `https://docs.rs/.../latest/...`) or a version-less docs.rs/shields badge; links must point to the
   latest *specific* version. The `:docs-rs:`/`:gh-example:` roles pin the `conf.py` documentation
   version automatically; hand-written links (e.g. the `index.rst` badge) use a concrete version.
   Grep for `latest` and version-less `shields.io/docsrs` and fix any hits.

Fix any issues found before considering documentation work done.

## Part 4 - Compile all RST code examples

Each `.. code-block:: rust` block that is a self-contained program (contains `fn main()` or
is a coherent set of statements) must compile verbatim. Blocks containing `...` or referencing
undefined symbols are illustrative snippets -- skip those.

### How to audit

1. Extract every Rust code block from `docs/guide/source/**/*.rst`.
2. Create a temporary test harness crate at `/tmp/doc_audit/` pointing to the local
   `mujoco-rs` path dependency.
3. Wrap each block in its own `mod block_N { ... }` (no shared top-level imports), then
   attempt `cargo check` (not build) to catch type errors without needing a display/GPU.
4. For blocks requiring the `viewer` feature, add a separate `[[bin]]` with
   `required-features = ["viewer-ui"]`. For `cpp-viewer` blocks use
   `required-features = ["cpp-viewer"]` and use `cargo check` (link errors are expected
   when the static C++ library is absent -- that is acceptable as long as type-checking passes).

### Fix policy

- **Missing imports**: Add `use mujoco_rs::prelude::*;` (and any other needed imports)
  at the top of the block -- do **not** rewrite the block with fully-qualified paths.
- **Deprecated / removed API**: Replace with the current equivalent, updating surrounding
  prose if needed.
- **Illustrative snippets** (contain `...`, use undefined vars, or are clearly not
  standalone programs): leave unchanged; they are not expected to compile.

## Part 5 - Changelog and migration completeness

Verify the changelog and migration guide are well-formed and complete against the actual
public-API changes since the last release.

1. **Changelog well-formed.** The top entry of `docs/guide/source/changelog.rst` must use
   `.. rubric::` sections in the fixed order: Breaking changes, Deprecations, Error handling,
   New features and improvements, Bug fixes, Other changes (not all are required, but the
   relative order must hold). Only public-facing changes are listed, and entries stay concise
   about *why* (see `.claude/rules/coding-conventions.md` changelog rules).
2. **Migration entries for breaking changes.** Every breaking change must have a
   `docs/guide/source/migration.rst` entry with a descriptive heading, prose, and
   `.. code-block:: rust` Before/After blocks (or a `.. list-table::` for simple signature
   changes). The migration guide is *only* for breaking changes; non-breaking additions belong
   in the changelog only (a non-breaking deprecation may still carry an optional migration entry).
3. **Completeness against the real diff.** Delegate this to a subagent (it is verbose): diff the
   public API between the most recent release tag and HEAD and confirm every public / breaking
   change is documented. Release tags have no `v` prefix (e.g. `4.0.1`); pick the highest such
   tag as the baseline unless the caller supplies one (e.g. `/release` passes its Phase 0
   baseline). Work accumulates on `develop`/pre-release branches that may be ahead of `main`,
   so diff against the **tag**, not `main`.

   Subagent prompt essentials (per `.claude/rules/subagent-policy.md` -- point it at the rules,
   do not paste them):
   - Read `.claude/rules/coding-conventions.md` (Documentation + changelog/migration rules)
     and `.claude/rules/disallowed-commands.md`.
   - Run `git diff <baseline_tag> HEAD -- src/ Cargo.toml` and extract every change to a public
     item (new/removed/renamed/re-typed `pub` fn/struct/enum/trait/variant, signature changes,
     new `unsafe fn`, error-type changes, deprecations, feature changes).
   - For each, confirm it appears in `changelog.rst`; for each breaking one, confirm a
     `migration.rst` entry with valid Before/After Rust.
   - Argue every gap from both sides (for/against) per the subagent-policy audit rule.
   - Return a concise table: change | documented? (changelog / migration) | verdict, plus a
     list of undocumented changes. No raw file dumps.

Fix any gaps before considering documentation work done.

> [!NOTE]
> - Always run `/doc` after adding or modifying public items, doc comments, `#[cfg(docsrs)]` attributes, or any `.rst` file.
> - `DOCS_RS=y` is required; without it, `#[cfg(docsrs)]` blocks are skipped and broken doc-links may go undetected.
> - `--no-deps` skips documenting dependency crates, keeping output focused on mujoco-rs.
> - `--all-features` ensures every conditional item (e.g. viewer, cpp-viewer) is included.
> - The Sphinx config lives in `docs/guide/source/conf.py`; custom extensions are in
>   `docs/guide/source/_extensions/`.
> - To clean previous Sphinx builds: `cd docs/guide && uv run --offline make clean && uv run --offline make html 2>&1`.
> - The Sphinx venv under `docs/guide` is `uv`-managed; always invoke `sphinx-build`/`make`
>   through it (`uv run`, or `.venv/bin/sphinx-build`), never a bare `sphinx-build`.
