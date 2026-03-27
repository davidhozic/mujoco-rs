---
name: diff-release
description: Compare HEAD against a previous release tag to identify all changes. Use this when asked to review what changed between versions.
---

# Diffing Against a Previous Release

Compare the current HEAD against a previous release tag to catalog all changes,
categorized by type (breaking, new features, bug fixes, etc.).

## Procedure

1. **Identify the previous release tag.** Read `Cargo.toml` for the current version,
   then determine the previous release tag. Tags use no `v` prefix (e.g. `2.3.5`).
   List available tags with `git --no-pager tag --list`.

2. **Generate the diff summary:**
```bash
git --no-pager diff --stat <previous_tag> HEAD -- src/ examples/ tests/
```

3. **Inspect changed files.** For each significantly changed file, compare old vs new:
```bash
git --no-pager diff <previous_tag> HEAD -- <file>
```

4. **Categorize changes** into these groups (matching changelog section ordering):
   - **Breaking changes**: removed/renamed public items, changed signatures, changed return types
   - **Error handling**: changes to error types, Result vs panic behavior
   - **New features**: new public methods, types, modules, examples
   - **Bug fixes**: corrected behavior, safety fixes
   - **Other changes**: refactors, documentation, CI, dependencies

5. **Report findings** in a structured format:
```
## Breaking changes
- `MethodName` signature changed: old_sig -> new_sig (file.rs)

## New features
- `new_method` added to Type (file.rs)

## Bug fixes
- Fixed incorrect stride in `method` (file.rs)
```

> [!NOTE]
> - Use `git --no-pager show <tag> -- <file>` to view a file at the old tag.
> - For large diffs, focus on `src/` first, then `examples/` and `tests/`.
> - Cross-reference findings against `changelog.rst` and `migration.rst` to identify
>   undocumented changes.
> - Release branches follow `vMajor.Minor.x` convention (e.g. `v2.3.x`).
> - This skill pairs well with `/audit-changelog` to verify documentation completeness.
