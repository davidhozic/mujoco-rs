---
name: audit-changelog
description: Audit changelog and migration docs against actual code changes. Use this when asked to verify documentation accuracy, review what changed between versions, or check for outdated content.
---

# Auditing Changelog and Migration Guide

Comprehensively verify that `docs/guide/source/changelog.rst` and
`docs/guide/source/migration.rst` are accurate, complete, and up-to-date against
the actual code on HEAD and the previous release tag.

## Phase 1 - Identify Changes

1. **Find the previous release tag.** Read `Cargo.toml` for the current version,
   then find the previous release tag (e.g. `2.3.5`). Tags use no `v` prefix.
   List available tags with `git --no-pager tag --list`.

2. **Generate the diff summary:**
```bash
git --no-pager diff --stat <previous_tag> HEAD -- src/ examples/ tests/
```

3. **Inspect changed files.** For each significantly changed file, compare old vs new:
```bash
git --no-pager diff <previous_tag> HEAD -- <file>
```

4. **Categorize changes** into groups (matching changelog section ordering):
   - **Breaking changes**: removed/renamed public items, changed signatures, changed return types
   - **Error handling**: changes to error types, Result vs panic behavior
   - **New features**: new public methods, types, modules, examples
   - **Bug fixes**: corrected behavior, safety fixes
   - **Other changes**: refactors, documentation, CI, dependencies

5. **Cross-reference against docs.** Compare the categorized changes against what is
   documented in `changelog.rst` and `migration.rst`. Flag any undocumented changes
   or documented claims that don't match actual code.

## Phase 2 - Verify All Claims

1. **Catalog all claims.** Read both files and list every distinct factual claim
   (method names, return types, error types, parameter changes, new features, etc.).

2. **Split into batches.** Divide claims into 3 roughly equal batches (max 3 parallel
   premium-model agents per `subagent-policy.md`).

3. **Spawn verification agents.** Launch up to 3 background agents (one per batch).
   Each agent receives:
   - The numbered list of claims to verify.
   - The previous release tag for `git show <tag> -- <file>` comparisons.
   - Instructions to check each claim against actual code on HEAD and the old tag.

   Each agent reports per claim:
   ```
   OK: <claim> -- <brief justification>
   WRONG: <claim> -- expected X, found Y
   UNCERTAIN: <claim> -- <reason>
   ```

## Phase 3 - Verify Code Blocks and Content

For every `.. code-block:: rust` block in both files, verify:
- Method names, types, and signatures match the actual code on HEAD.
- The code is syntactically valid Rust (no typos, missing semicolons, etc.).
- Before/After examples accurately reflect the old tag and current HEAD respectively.

Additionally check all Sphinx documentation for:
- **Outdated content**: references to removed methods, old signatures, or deprecated patterns.
- **Factual correctness**: claims about API behavior, error types, and return types.
- **Cross-references**: `:gh-example:`, `:docs-rs:`, and RST substitutions resolve correctly.
- **Formatting**: RST syntax, indentation, and directive usage are correct.

## Phase 4 - Resolve and Fix

1. **Resolve discrepancies.** For any WRONG or UNCERTAIN items, investigate manually
   or spawn a focused resolution agent.

2. **Fix confirmed issues.** Apply fixes to the RST files.

3. **Run `/doc`** to verify both rustdoc and Sphinx builds are clean.

> [!NOTE]
> - Follow `subagent-policy.md` when spawning agents (include rule/skill reading instructions).
> - Use `git show <tag> -- <file>` to inspect old code; avoid `git diff` for line-by-line
>   claim verification (too noisy).
> - Release branches follow `vMajor.Minor.x` convention (e.g. `v2.3.x`).
> - Store audit results in a SQL table for tracking across rounds:
>   ```sql
>   CREATE TABLE IF NOT EXISTS audit_entries (
>     id INTEGER PRIMARY KEY,
>     claim TEXT NOT NULL,
>     file TEXT NOT NULL,
>     agent INTEGER,
>     status TEXT DEFAULT 'pending',
>     notes TEXT
>   );
>   ```
