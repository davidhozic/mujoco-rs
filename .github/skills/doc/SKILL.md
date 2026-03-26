---
name: doc
description: Build rustdoc documentation with all features enabled and DOCS_RS mode. Use this when asked to build docs, check doc comments, or verify documentation.
---

# Building Documentation

Run rustdoc with all features enabled and the `DOCS_RS=y` flag (which activates
`#[cfg(docsrs)]` attributes) to catch any broken links, missing items, or
doc-comment errors.

1. Check `Cargo.toml` for the current MuJoCo version (look for `+mj-X.Y.Z` in the package version). Then find the matching `mujoco-X.Y.Z/` directory at the repository root.

2. Build the docs (replace `X.Y.Z` with the version found in step 1):
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-X.Y.Z/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-X.Y.Z/lib/) && DOCS_RS=y cargo doc --no-deps --all-features 2>&1
```

3. Verify the output contains no `warning:` or `error:` lines.
   Fix any that appear before considering documentation work done.

> [!NOTE]
> - Always run `/doc` after adding or modifying public items, doc comments, or `#[cfg(docsrs)]` attributes.
> - `DOCS_RS=y` is required; without it, `#[cfg(docsrs)]` blocks are skipped and broken doc-links may go undetected.
> - `--no-deps` skips documenting dependency crates, keeping output focused on mujoco-rs.
> - `--all-features` ensures every conditional item (e.g. viewer, cpp-viewer) is included.
