---
name: ffi-audit
description: >-
  Audit the impact of regenerated MuJoCo FFI bindings (the ffi-regenerate Cargo feature) on the
  wrapper layer. Diffs src/mujoco_c.rs, reads the MuJoCo changelog entries for the version range,
  deep-verifies every changed item against the C/C++ source, and writes a self-contained HTML
  report suggesting required wrapper additions/removals/updates. Strictly read-only -- it suggests
  changes but never edits code. Use after regenerating the FFI bindings or bumping the MuJoCo
  version.
---

# FFI Regeneration Impact Audit

Obey **all** files in `.claude/rules/` at all times (including the subagent policy when
authoring agent prompts). Re-read them after any context compaction.

Determine what must be fixed, added, removed, or updated in the mujoco-rs wrapper layer after
`src/mujoco_c.rs` has been regenerated against a new MuJoCo version. Drive the analysis with the
`Workflow` tool (parallel subagents).

## Ground rules (non-negotiable scope)

- **Strictly read-only.** No source files, tests, docs, or changelog entries are modified.
  The ONLY output is the HTML report. Fixes are suggested, never applied.
- **Never trigger the `ffi-regenerate` feature.** The audit analyzes an *already-regenerated*
  `src/mujoco_c.rs`. If the bindings have not changed relative to the chosen baseline, stop and
  tell the user there is nothing to audit.
- **Changed items only.** Scope is the FFI delta plus the matching MuJoCo changelog entries.
  Re-verifying unchanged wrappers is `/verify`'s and `/safety`'s job -- do not duplicate it.
- **C/C++ evidence is mandatory.** Every suggestion must be backed by *reading the actual C/C++
  implementation* (`mujoco/src/**`, `mujoco/include/**`) of the changed item -- deeply enough to
  understand the new logic (how the field is written/read, which functions consume it, whether
  values are used as unguarded indices, return-code conventions, dimension comments). Citing a
  changelog bullet or a function name without reading the implementation is not sufficient; such
  suggestions must be dropped. Cite `file:line` for every C-side claim.
- **Quote both sides.** Each suggestion quotes the old and new C definition (or C behaviour)
  AND the current Rust code it affects.

## Impact catalog -- what FFI regeneration can break

The inventory and mapping phases must classify every change into (at least) these categories:

1. **Removed / renamed struct fields** -- compile breaks in `info_method!`, `info_with_view!`,
   `array_slice_dyn!`, `getter_setter!`, `view_creator!`, `c_str_as_str_method!` invocations and
   hand-written methods. Suggest removal/rename plus a migration-guide entry.
2. **Added struct fields** -- missing accessors. Suggest the appropriate macro, the stride from
   the header dimension comment, `-1`-address -> `Option` mapping, and the mutability
   classification (`(unsafe)` / `(allow_mut = false)` / default) by tracing how C uses the
   field's VALUES (unguarded array index?) per `macro-system.md`.
3. **Dimension / stride changes** -- the field still exists, so this compiles and is *silently
   wrong* (OOB reads or garbage). Check BOTH the `info_method!` stride and the matching
   `info_with_view!` type, plus `array_slice_dyn!` length expressions and cast types.
4. **Field type changes** (e.g. `mjtByte` -> `mjtBool`) -- accessor element types, `bytemuck` /
   `cast_mut_info!` / `maybe_force_cast!` targets, and `build.rs` repr patches (e.g. the
   `mjtSameFrame` -> `repr(u8)` regex).
5. **Semantic-only changes** -- no binding diff at all; the meaning of an existing field or
   function changed (e.g. the 3.9.0 `margin`/`gap` redesign). Only discoverable via the MuJoCo
   changelog and the C source. Affects doc comments, helper logic, viewer behaviour, examples.
6. **Enum changes** -- added/removed variants, changed discriminants, repr changes: enum-dispatch
   match arms, casts of C integers into fieldless `#[repr]` enums (invalid-discriminant UB),
   meta-variant guards (`mjNOBJECT`-style), range checks.
7. **Function changes** -- added (suggest a wrapper: panicking + `try_` convention; safety-guard
   tier per `coding-conventions.md`: type encoding > O(1) assert > `unsafe fn`; verify the
   return-code convention from the C source -- conventions vary per function); removed (remove
   wrapper + migration entry); signature changed; behaviour changed (C now validates an argument
   itself -> a duplicated Rust guard should go; C stopped validating -> a guard is now missing).
8. **New / removed types** -- new wrapper type needed? Decide Drop (find the matching
   `mj_delete*`/`mju_free*` in C), `Send`/`Sync` soundness, and Clone derivation: the `build.rs`
   `CloneCallback` omits `Clone` on pointer-containing types by NAME PREFIX (`mjs*`, `mjSpec_`)
   -- a new pointer-containing type with a different prefix silently derives an unsound `Clone`.
9. **Constant changes** (`mjN*`, `mjMAX*`, buffer sizes) -- fixed-size arrays and buffers in
   wrappers sized from those constants.
10. **Count / address mapping tables** -- new `nX` count fields or address arrays may need rows in
    `mj_model_nx_to_mapping!` / `mj_model_nx_to_nitem!` in `src/util.rs`; changed address/count
    relationships break `mj_view_indices!` users.
11. **`build.rs` post-processing** -- the bindgen derive lists (Clone/Copy/PartialEq/
    ThreeWayMerge) and fix-up regexes reference type names; renamed or new types can fall
    through. New fields in `mjOption`/`mjStatistic`/`mjVisual*` enter the derived ThreeWayMerge
    used by the viewer's state sync.
12. **Docs and tests** -- required `changelog.rst` / `migration.rst` entries (per
    `coding-conventions.md` rubric ordering), guide pages, examples, and correctness tests for
    every suggested accessor. Report these as part of each finding's "docs/tests impact".

## Methodology -- multi-agent (drive with the `Workflow` tool)

### Phase 0 -- Inline recon (main conversation, before calling `Workflow`)

1. **Pick the baseline ref.** If `git diff HEAD -- src/mujoco_c.rs` is non-empty, the baseline is
   `HEAD` (uncommitted regeneration). Otherwise inspect `git log --oneline -- src/mujoco_c.rs`
   and the previous release tag (tags have no `v` prefix); if ambiguous, ask the user.
2. **Record versions.** OLD/NEW `mjVERSION_HEADER` from both sides of the diff and the
   `+mj-X.Y.Z` suffix in `Cargo.toml` (note if the suffix was not bumped yet -- checklist item).
3. **Dump the diffs** into `target/ffi-audit/` (gitignored):
   `git diff <BASE> -- src/mujoco_c.rs > target/ffi-audit/ffi.diff`. The `mujoco/` submodule
   carries upstream tags (e.g. `3.8.1`, `3.9.0`):
   `git -C mujoco diff <old-tag> <new-tag> -- include/ src/ > target/ffi-audit/c.diff` and note
   the new version's section(s) in `mujoco/doc/changelog.rst`. If the old tag is absent locally,
   do NOT fetch (network is disallowed) -- fall back to `ffi.diff` + changelog only.
4. **Compile probe** (cheap removed/renamed signal):
   `cargo check --features "viewer-ui renderer-winit-fallback" 2>&1 | tail -n 120`
   with the build env below. Capture all errors.
5. Call `Workflow` with `args: { base, old_version, new_version, diff paths, mujoco_lib,
   compile_errors, prior_findings }`.

### Phase 1 -- INVENTORY (parallel scouts, structured output)

- **Diff scouts** (one per category group: structs / enums / functions / constants+types, or
  chunked by hunks for large diffs) parse `ffi.diff` into entries:
  `{ id, kind, c_item, owner_struct, old, new }` where `kind` is one of
  `field-added | field-removed | field-type-changed | dim-changed | enum-changed | fn-added |
  fn-removed | fn-sig-changed | const-changed | type-added | type-removed | semantic`.
- **Changelog scout**: read ALL sections of `mujoco/doc/changelog.rst` between the old and new
  versions (multiple sections when releases are skipped). Emit an entry for every bullet that
  touches the public C API or semantics -- especially `semantic` entries that produce no binding
  diff. Cross-link bullets to diff entries.

### Phase 2 -- MERGE / DEDUP (barrier -- needs all scouts)

Merge and dedupe both inventories; drop pure bindgen noise (formatting churn, comment-only
hunks) but `log()` what was dropped. Every diff hunk and every changelog bullet must map to at
least one inventory entry or be explicitly classified as no-impact.

### Phase 3 -- DEEP C ANALYSIS + IMPACT MAPPING (pipeline, per entry)

For each entry, one analyst agent:

1. Reads the C/C++ implementation deeply (`mujoco/src/**`, `mujoco/include/**`) until the new
   logic of the changed item is understood; quotes `file:line` evidence.
2. Greps `src/` (and `build.rs`) for every usage the change touches: macro tables in
   `src/util.rs`, wrapper files, viewer/renderer, `build.rs` callbacks/regexes.
3. Emits a suggestion:
   `{ entry, c_evidence, rust_locations, action (add|remove|update|none), suggestion,
   snippet?, severity, docs_tests_impact }`.
   Fields the crate deliberately does not wrap (e.g. arena-internal `efc_*` style fields) get
   `action: none` but are still reported as optional additions.

### Phase 4 -- ADVERSARIAL VERIFICATION (pipeline, per suggestion)

Per the subagent policy, argue both sides. For each suggestion spawn, independently:

- a hostile **refuter** prompted to prove the suggestion wrong or unnecessary (C already
  validates; the item is not wrapped and need not be; the dimension did not actually change;
  default to "refuted" when uncertain), and
- an independent **re-deriver** that reconstructs the impact from scratch without the analyst's
  notes.

Keep only suggestions surviving both. The C/C++ citation rule from Phase 3 applies here too.

### Phase 5 -- COMPLETENESS CRITIC (loop until dry)

A final agent checks: is every `ffi.diff` hunk, every changelog bullet in the version range,
and every Phase 0 compile error accounted for by a finding or an explicit no-impact entry?
Each gap spawns another Phase 3 round. Loop until a round produces no new entries.

## Severities

| Severity | Meaning |
|---|---|
| Critical | Silent UB/OOB or unsoundness if left unfixed (dim/type change, index field, invalid enum discriminant) |
| High | Compile break, or wrong values returned through the safe API |
| Medium | New functionality not yet wrapped (fields, functions, types) |
| Low | Docs, comments, or tests only |
| Info | Verified no-impact |

## Build env

```bash
export MUJOCO_DYNAMIC_LINK_DIR="$(realpath mujoco-X.Y.Z/lib)"
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR"
```

Substitute `X.Y.Z` from the `+mj-X.Y.Z` suffix in `Cargo.toml`. For `cargo check`
(type-checking only, no link) only `MUJOCO_DYNAMIC_LINK_DIR` is strictly required, so the probe
works even before the new MuJoCo binary release is downloaded.

## Deliverable -- a single self-contained HTML report

The main loop writes/overwrites `mujoco-rs-ffi-audit-report.html` at the repo root from the
workflow's structured findings.

**Scope: the current chat session.** The report accumulates findings across ALL `/ffi-audit`
runs within this chat. Do NOT read the on-disk HTML file to repopulate findings -- it belongs
to a prior chat. On re-runs within the same chat, pass the prior findings (with statuses) as
`args` so the new run merges with them; on the first run start empty.

The file must be self-contained (inline `<style>`), ASCII-only, and contain:

- A header with the audited version range (`old -> new`), the baseline ref, and a short
  **method note** describing the phases and the impact catalog.
- An **overview table**:
  `ID | Severity (pill) | Kind (badge) | C item | One-line suggestion | Rust location(s) | Status`.
  If nothing was found, the table says "No wrapper impact found for this FFI delta."
- One **finding card** per finding: ID, severity, kind badge, the C item with its changelog
  reference, **C/C++ evidence** (quoted, `file:line`), affected Rust locations, a **concise
  (< 300 words) suggested change** (description; a short code snippet is allowed -- NOT
  applied), and a **docs/tests impact** line (changelog/migration/test entries needed).
- A **"Verified no-impact"** section: diff/changelog items checked and classified as requiring
  no wrapper change, each with a one-line reason.
- A **"Version bump checklist"** section: the status of each item from the
  `important-context.md` "MuJoCo version bump checklist" (version suffix, `mujoco-X.Y.Z/`
  directory, `build.rs`, docs) as observed during recon.

Styling: use the same Claude aesthetic as `mujoco-rs-memory-safety-audit.html` -- ivory canvas
background, coral accent, warm near-black ink, Georgia serif headings, rounded pill badges, white
finding cards on the canvas, and a hairline-border overview table. Severity pills
(`sev-critical/high/medium/low/info`), status badges (`st-open` / `st-already-handled` /
`st-no-action`), kind badges in distinct hues per category (added=green, removed=red,
type/sig-changed=orange, dim-changed=purple, enum-changed=violet, semantic=grey).

Statuses:

| Status | When to use |
|---|---|
| **Open** | Suggested change not yet applied |
| **Already handled** | The wrapper was already updated for this change before the audit |
| **No action** | Verified to require no wrapper change |

After writing the file, present the overview table to the user as a plain-text summary.
No interactive questions; do NOT modify any source files.
