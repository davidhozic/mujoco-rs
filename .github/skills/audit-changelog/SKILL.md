---
name: audit-changelog
description: Audit changelog and migration docs against actual code changes. Use this when asked to verify documentation accuracy after major releases or doc edits.
---

# Auditing Changelog and Migration Guide

Verify that every claim in `docs/guide/source/changelog.rst` and
`docs/guide/source/migration.rst` is accurate against the actual code on HEAD
and the previous release tag.

## Procedure

1. **Identify the previous release tag.** Read `Cargo.toml` for the current version,
   then find the previous release tag (e.g. `2.3.5`). Tags use no `v` prefix.

2. **Catalog all claims.** Read both files and list every distinct factual claim
   (method names, return types, error types, parameter changes, new features, etc.).

3. **Split into batches.** Divide claims into 3 roughly equal batches (max 3 parallel
   premium-model agents per `subagent-policy.md`).

4. **Spawn verification agents.** Launch up to 3 background agents (one per batch).
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

5. **Resolve discrepancies.** For any WRONG or UNCERTAIN items, investigate manually
   or spawn a focused resolution agent.

6. **Fix confirmed issues.** Apply fixes to the RST files. Run `/sphinx-doc` to verify
   no RST syntax errors were introduced.

> [!NOTE]
> - Follow `subagent-policy.md` when spawning agents (include rule/skill reading instructions).
> - Use `git show <tag> -- <file>` to inspect old code; avoid `git diff` for line-by-line
>   claim verification (too noisy).
> - After fixes, run `/sphinx-doc` and `/doc` to verify both Sphinx and rustdoc are clean.
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
