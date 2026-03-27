---
name: sphinx-doc
description: Build the Sphinx RST documentation in docs/guide/. Use this when asked to build the user guide, check RST syntax, or verify Sphinx cross-references.
---

# Building Sphinx Documentation

Build the Sphinx-based user guide (changelog, migration guide, installation, etc.)
to verify RST syntax, cross-references, and custom roles like `:gh-example:`.

1. Build the HTML docs:
```bash
cd docs/guide && make html 2>&1
```

2. Check the output for `WARNING:` or `ERROR:` lines. The following warning is
   pre-existing and can be ignored:
   - `WARNING: html_static_path entry '_static' does not exist`

3. Fix any **new** warnings or errors before considering documentation work done.

> [!NOTE]
> - Run this after editing any `.rst` file under `docs/guide/source/` (changelog, migration, etc.).
> - Sphinx validates custom roles (`:gh-example:`, `:docs-rs:`), RST substitutions (`|mj_data|`),
>   and cross-references that rustdoc cannot check.
> - This is complementary to `/doc` (rustdoc). Run both when public API docs and the user guide
>   are both affected.
> - The Sphinx config lives in `docs/guide/source/conf.py`; custom extensions are in
>   `docs/guide/source/_extensions/`.
> - To clean previous builds: `cd docs/guide && make clean && make html 2>&1`.
