---
name: doc
description: Build rustdoc and Sphinx documentation, verifying both for warnings, broken links, and content correctness. Use this when asked to build docs, check doc comments, or verify documentation.
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

1. Build the HTML docs:
```bash
cd docs/guide && make html 2>&1
```

2. Check the output for `WARNING:` or `ERROR:` lines. The following warning is
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

Fix any issues found before considering documentation work done.

> [!NOTE]
> - Always run `/doc` after adding or modifying public items, doc comments, `#[cfg(docsrs)]` attributes, or any `.rst` file.
> - `DOCS_RS=y` is required; without it, `#[cfg(docsrs)]` blocks are skipped and broken doc-links may go undetected.
> - `--no-deps` skips documenting dependency crates, keeping output focused on mujoco-rs.
> - `--all-features` ensures every conditional item (e.g. viewer, cpp-viewer) is included.
> - The Sphinx config lives in `docs/guide/source/conf.py`; custom extensions are in
>   `docs/guide/source/_extensions/`.
> - To clean previous Sphinx builds: `cd docs/guide && make clean && make html 2>&1`.
