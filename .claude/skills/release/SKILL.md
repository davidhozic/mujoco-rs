---
name: release
description: Pre-release preparation and verification workflow for cutting a new mujoco-rs version. Use this when asked to prepare a release, do a pre-release check, cut version X.Y.Z, or verify the crate is ready to publish. Runs every gate the maintainer needs before publishing, then hands off the networked publish steps.
---

# Pre-release workflow

Prepare and fully verify a new mujoco-rs release. This skill orchestrates the existing
gate skills (`/build`, `/test`, `/clippy`, `/doc`, `/asan`, `/miri`, `/verify`), audits
version-string consistency and crate packaging, and produces a go/no-go report.

It does **NOT** publish. Every networked or history-mutating step (commit, push, tag,
branch, merge, `cargo publish`, GitHub release) is **disallowed** for the agent (see
`.claude/rules/disallowed-commands.md`). Those are listed at the end for the maintainer to
run manually.

Work in the current branch only. Do not create branches, commit, or push.

---

## Phase 0 -- Establish the release context

Gather the facts the rest of the workflow depends on. Report them back before proceeding.

1. **New version** -- read `Cargo.toml` `[package].version`. It has the form
   `X.Y.Z+mj-A.B.C`, where `X.Y.Z` is the crate version and `A.B.C` is the bundled MuJoCo
   version. Record both. The doc version is `X.Y.x` (last component literally `x`); the
   release branch is `vX.Y.x`.
2. **Previous release** -- `git tag` lists release tags (no `v` prefix, e.g. `4.0.1`). Pick
   the highest tag below the new version. This is the diff baseline.
3. **MuJoCo dir** -- confirm `mujoco-A.B.C/lib` exists at the repo root (needed by the build
   gates). `Cargo.toml`'s `+mj-A.B.C` is the single source of truth; `build.rs` and
   `docs/guide/source/conf.py` both derive their version from it, so neither needs a manual
   edit.
4. **Semver sanity** -- confirm the bump matches the change set: a MuJoCo FFI bump or any
   removed/changed public signature requires a **major** bump. Flag a mismatch.

If the version still reflects the previous release (no bump applied yet), stop and ask the
maintainer for the intended `X.Y.Z` before continuing -- everything downstream depends on it.

---

## Phase 1 -- Version-string consistency audit

`Cargo.toml` drives `build.rs` and `conf.py` automatically, but several files hard-code the
version and **must be bumped by hand**. Verify every one of these reflects the new `X.Y.Z` /
`A.B.C` / `vX.Y.x`:

- `README.md`:
  - Guide badge link `.../readthedocs.io/en/vX.Y.x/`
  - docs.rs badge `img.shields.io/docsrs/mujoco-rs/X.Y.Z` and its `docs.rs/mujoco-rs/X.Y.Z/` target
  - the "Upgrading from <prev> to X.Y.Z" note and migration link `/en/vX.Y.x/migration.html`
  - "This library uses FFI bindings to MuJoCo **A.B.C**." and every other `vX.Y.x` link
- `docs/guide/source/index.rst`: the `shields.io/docsrs/mujoco-rs/X.Y.Z` badge and `docs.rs/.../X.Y.Z/` target
- `docs/guide/source/installation.rst`: the MuJoCo download tag `.../releases/tag/A.B.C`
- `CHANGELOG.md`: the `/en/vX.Y.x/changelog.html` link

**No `latest` alias and no version-less badge** may appear in any `.rst` or `README.md`
(coding-conventions: links pin to a concrete version). Grep to confirm:
```bash
grep -rnE "docs\.rs/mujoco-rs/latest|/en/latest/|shields\.io/docsrs/mujoco-rs\)|img\.shields\.io/docsrs/mujoco-rs[^/]" docs/guide/source README.md
```
Any hit is a bug -- fix it to the pinned version.

Report a table of every checked location with its current value vs the expected value.

---

## Phase 2 -- Changelog and migration completeness

1. **Changelog section exists and is well-formed.** `docs/guide/source/changelog.rst` must
   have a top entry `X.Y.Z (MuJoCo A.B.C)`. Its `.. rubric::` sections must follow the fixed
   order: Breaking changes, Deprecations, Error handling, New features and improvements,
   Bug fixes, Other changes (not all are required, but order must hold). Only public-facing
   changes; entries concise about *why* (see coding-conventions changelog rules).
2. **Migration section exists (major bumps).** For a major bump, `migration.rst` needs a
   `Migrating to X.Y.Z` section with a prose explanation and `.. code-block:: rust`
   Before/After blocks (or a `list-table` for simple signature changes) for **every**
   breaking change.
3. **Completeness against the real diff.** Delegate this to a subagent (it is verbose): diff
   the public API between the previous tag and HEAD and confirm every breaking/public change
   is documented in both changelog and (if breaking) migration. Release work accumulates on
   `develop` and the current pre-release branch, which may be ahead of `main`; diff against the
   previous release **tag** (Phase 0 baseline), not `main`, so unmerged changes are included.

   Subagent prompt essentials (per `.claude/rules/subagent-policy.md` -- point it at rules,
   do not paste them):
   - Read `.claude/rules/coding-conventions.md` (Documentation + changelog/migration rules)
     and `.claude/rules/disallowed-commands.md`.
   - Run `git diff <prev_tag> HEAD -- src/ Cargo.toml` and extract every change to a public
     item (new/removed/renamed/re-typed `pub` fn/struct/enum/trait/variant, signature
     changes, new `unsafe fn`, error-type changes, feature changes).
   - For each, confirm it appears in `changelog.rst`; for each breaking one, confirm a
     `migration.rst` entry with valid Before/After Rust.
   - Argue every gap from both sides (for/against) per the subagent-policy audit rule.
   - Return a concise table: change | documented? (changelog / migration) | verdict, plus a
     list of undocumented changes. No raw file dumps.

Fix any gaps before moving on.

---

## Phase 3 -- Build, test, lint, and doc gates

Run the existing gate skills. Each already sets `MUJOCO_DYNAMIC_LINK_DIR` /
`LD_LIBRARY_PATH` for the bundled MuJoCo. Delegate verbose runs to subagents/background so
only summaries return to the main thread (token-efficiency rules). All must be clean.

1. **`/build`** -- build the library across the feature surface, in **debug and release**
   (release matters: `debug_assert!` bounds checks compile out, so release exercises a
   different path). Cover at least:
   - `--no-default-features` (default/empty)
   - `--features renderer`
   - `--features "viewer-ui renderer-winit-fallback"`
   - `--all-features`
2. **`/test`** -- run the suite with `--features renderer` (and `--release` once). All green.
3. **`/clippy`** -- lint clean (CI does not run clippy, so this gate is local-only).
4. **`/doc`** -- rustdoc (`DOCS_RS=y ... --all-features`) and the Sphinx guide both clean,
   plus the content-verification and RST-example-compile passes the doc skill defines. This
   is where Phase 1/2 doc edits get validated.
5. **FFI safety (recommended for major bumps / FFI version bumps): `/asan`, `/miri`, and the
   `/verify` audit.** A MuJoCo FFI bump changes the C ABI surface; run these to catch UB at
   the boundary. For a patch release with no `unsafe`/FFI changes they may be skipped -- say
   so explicitly in the report rather than silently dropping them.
6. **Cross-architecture portability check.** Users compile MuJoCo themselves on
   non-official platforms (arm64, armv7, x86-32), so the wrapper must compile on all of them.
   `cargo check` runs `build.rs` but does not link, so no per-target MuJoCo library is needed;
   each target's std component must be installed (`rustup target add <triple>`) -- if a target
   is missing, note it as not-checked rather than failing. Run, at minimum:
   ```bash
   export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-A.B.C/lib)
   for t in aarch64-unknown-linux-gnu armv7-unknown-linux-gnueabihf i686-unknown-linux-gnu; do
     echo "## $t (core)"   && cargo check --target $t --lib --no-default-features
     echo "## $t (render)" && cargo check --target $t --lib --no-default-features --features renderer
   done
   ```
   armv7/i686/wasm32 are 32-bit: `i64 -> usize` narrowing in slice-length casts is expected
   and benign there (MuJoCo allocates the counts), but any *new* pointer-width-dependent code
   should be eyeballed. A clean `cargo check` per target is the gate.

Record pass/fail for each gate with the exact command and a one-line result.

---

## Phase 4 -- Packaging verification

Confirm the published crate is correct and lean. The build verification inside packaging
needs the MuJoCo lib path:
```bash
export MUJOCO_DYNAMIC_LINK_DIR=$(realpath mujoco-A.B.C/lib) && export LD_LIBRARY_PATH=$(realpath mujoco-A.B.C/lib)
```

1. **Package contents** -- `cargo package --list` and confirm:
   - `src/mujoco_c.rs` **is** present (committed generated bindings -- the crate cannot build
     without them).
   - **No** stray dirs/artifacts: `mujoco/`, `docs/`, `.claude/`, `.github/`, `assets/`,
     `scripts/`, `tests/`, `expanded.rs`, `MUJOCO_LOG.TXT`, audit reports
     (`*-memory-safety-audit.*`). The `exclude` list in `Cargo.toml` plus `.gitignore` should
     drop all of these.
   ```bash
   cargo package --list --allow-dirty 2>/dev/null \
     | grep -iE "mujoco/|docs/|expanded|MUJOCO_LOG|\.claude|\.github|assets/|scripts/|tests/|memory-safety-audit" \
     && echo "STRAY FILES -- FIX exclude/.gitignore" || echo "package clean"
   ```
2. **Exclude/.gitignore policy** -- if any new permanent non-crate file was added this cycle,
   it belongs in `Cargo.toml` `exclude`; one-off/generated artifacts belong in `.gitignore`
   (never `exclude`). See coding-conventions.
3. **Full dry-run build** -- `cargo publish --dry-run --allow-dirty` (or
   `cargo package --allow-dirty`) to verify the packaged crate actually compiles in isolation.
   This is a local build/verify only and does **not** upload. If the registry index is
   unavailable offline, note it and fall back to `cargo package --list` + a normal release
   build as the packaging evidence.
4. **Cargo.lock** -- `Cargo.lock` is `.gitignore`d for this crate; no action unless that
   changes. Note its status rather than assuming.
5. **MSRV** -- `rust-version` (workspace) is `1.88`; confirm no newer-than-MSRV syntax slipped
   in. If a `1.88` toolchain is installed, a `cargo +1.88 check` is the strongest signal;
   otherwise note MSRV was not re-verified.

---

## Phase 5 -- Go / no-go report (HTML deliverable)

Write a single self-contained HTML report to `mujoco-rs-release-report.html` at the repo root
(overwrite on re-run; scope is this release). It must be standalone (inline `<style>`),
ASCII-only, and match the shared report aesthetic used by `/verify`
(`mujoco-rs-verify-report.html`) and `mujoco-rs-memory-safety-audit.html`: ivory canvas
background, coral accent, warm near-black ink, Georgia serif headings, rounded pill badges,
white cards on the canvas, hairline-border tables. Reuse that styling -- do not invent a new look.

The report must contain:

- A header with the target version `X.Y.Z+mj-A.B.C`, doc version `X.Y.x`, release branch
  `vX.Y.x`, baseline tag, and a short method note (the phases this skill ran).
- A prominent **verdict** banner: `GO` or `NO-GO` (status pill), with the blocking items listed
  if NO-GO.
- **Gate results table**: `Gate | Command | Result (pass/fail/skipped/fixed pill) | Notes` --
  one row per Phase 3 gate (build debug/release, test, clippy, doc, asan, miri, verify,
  cross-arch check). Skipped gates state the reason.
- **Version-consistency table** (Phase 1): `Location | Current value | Expected value | Status`.
- **Docs completeness** (Phase 2): changelog + migration present and complete? list any gaps as
  cards.
- **Packaging** (Phase 4): `cargo package --list` clean? dry-run build result; any stray files.
- A **"Manual maintainer steps"** section rendering the Phase 6 checklist with concrete values
  filled in (so the report is a complete hand-off artifact).

Statuses: use `Pass` / `Fail` / `Skipped` / `Fixed` pills, mirroring `/verify`'s status badges.

After writing the file, present a brief plain-text summary to the user: the verdict and any
blocking items only (not the full report).

If anything is NO-GO, fix what is in scope (doc/version edits, code fixes), re-run only the
affected gate, and update the report -- do not re-run the whole suite blindly.

---

## Phase 6 -- Manual maintainer steps (agent must NOT run these)

These are networked / history-mutating and are **disallowed** for the agent. List them for
the maintainer with the concrete values filled in (no `v` prefix on the tag; `v` and `x`
literal on the branch):

1. Merge `develop` -> `main` via PR ("Merge changes for release of X.Y.Z (MuJoCo A.B.C)").
2. Tag the merge commit on `main`: `git tag X.Y.Z && git push origin X.Y.Z`.
3. Create/push the release branch: `git switch -c vX.Y.x && git push -u origin vX.Y.x`.
4. Publish a GitHub release for tag `X.Y.Z` (this triggers `.github/workflows/tests.yml`,
   which builds + tests on Ubuntu and Windows). Wait for it to go green.
5. `cargo publish` to crates.io (CI does not publish; this is manual). Use the same
   `MUJOCO_DYNAMIC_LINK_DIR` so the publish-time verify build links.
6. Post-publish: confirm the docs.rs build succeeded and that readthedocs has a `vX.Y.x`
   version, then check the new `docs.rs/mujoco-rs/X.Y.Z` and badge links resolve.

> [!NOTE]
> - This skill prepares and verifies; it never publishes. Stop at Phase 5 and present
>   Phase 6 as instructions.
> - The single source of truth for both the crate and MuJoCo versions is
>   `Cargo.toml` `version = "X.Y.Z+mj-A.B.C"`; `build.rs` and `conf.py` derive from it.
> - Reuse the gate skills rather than re-typing their commands; they own the correct env vars
>   and feature flags.
> - Delegate verbose gate runs and the API-diff completeness check to subagents/background so
>   only summaries reach the main context (`.claude/rules/token-efficiency.md`).
