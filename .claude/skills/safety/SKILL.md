---
name: safety
description: Adversarial multi-agent soundness audit of the mujoco-rs SAFE API surface -- memory safety, thread safety (every unsafe impl Send/Sync), &self vs &mut self mutability, and lifetime/variance correctness, restricted to defects reachable from 100% safe code. Produces a single self-contained HTML report. Use when asked to audit/check the crate for unsoundness or undefined behavior reachable from safe code. Distinct from /verify (which covers stride/length-field/type correctness).
---

# Safety & Soundness Audit (Claude Code only)

Obey **all** files in `.github/rules/` at all times. Re-read them after any context compaction.

## Ground rules (non-negotiable scope)

- **`src/` only.** Audit the safe Rust wrapper. `mujoco/src/**` C/C++ is read *only* to confirm
  a mechanism, never reported as a finding.
- **Reachable from 100% SAFE code only.** The safe API must never cause UB for any safe input.
  Anything only reachable through an `unsafe fn` or an `unsafe` block the *caller* must write is
  out of scope, unless it is always a bug/UB -- regardless of how the user uses it.
  The safe/unsafe tier is the dividing line.
- **Stride/length/type correctness is OUT of scope** -- that is `/verify`'s job. Do not duplicate
  it.
- **The audit is read-only.** No source files, tests, or docs are modified as part of the audit.
  The only output is the HTML report. Fixes, changelog entries, and regression guards are out of
  scope -- the audit just identifies and documents findings.

## The four dimensions

1. **Memory safety / UB** -- OOB read/write reachable from safe code; invalid transmute / invalid
   enum discriminant (e.g. `force_cast` of unchecked C `int*` into a fieldless `#[repr]` enum);
   use-after-free; uninitialized/stale reads; integer-overflow-driven OOB (e.g. `w*h*3` usize
   overflow defeating a buffer check); mis-sized buffers handed to C; `len() as i32` truncation.
   **For enum/discriminant casts specifically:** only flag if a safe-code caller can supply or
   mutate the integer value that is later cast to the enum (e.g. via a public integer setter on
   the same field). If the integer is written exclusively by MuJoCo internals and the safe API
   exposes no setter for it, the user cannot produce an invalid discriminant through safe code
   and the cast is out of scope.
2. **Thread safety** -- examine **every** `unsafe impl Send` and `unsafe impl Sync` in `src/`
   (`rg "unsafe impl (Send|Sync)" src/`). For each, decide whether the assertion is actually
   sound, then hunt for a SAFE-code path that turns an unsound impl into a data race. The classic
   pattern here: a *lending* iterator whose `type Item = &'a mut T` is tied to the iterator's
   lifetime (not to `next()`'s `&mut self`), so `collect()` yields many simultaneously-live
   `&mut` handles; if the handle is `Send`, those move to different threads and race on shared
   C++ state.
3. **`&self` vs `&mut self` mutability** -- if the wrapped C/C++ call mutates the item or shared
   model-global state, the method MUST take `&mut self`. A `&self` method that drives C-side
   mutation is a soundness bug: it breaks `&` aliasing guarantees and any `Sync` assumption.
4. **Lifetimes / variance** -- every child handle must carry its parent's lifetime (a child must
   not outlive its parent -> no use-after-free), and variance must be sound. Confirm conditional
   impls (e.g. `MjData<M>: Send/Sync` conditional on `M`) are correct.

## Methodology -- adversarial multi-agent (drive with the `Workflow` tool)

### Phase 0 -- Recon & task list (inline, before any fan-out)
1. Read `src/util.rs` (all macros) and glob + read `src/wrappers/**/*.rs`.
2. Enumerate every `unsafe impl Send`/`Sync` site, every lending/mutable iterator, and every
   element-handle type (e.g. the `mjs_struct!`-generated `Mjs*` aliases).
3. Record the MuJoCo build env for later compile checks (see "Build env" below).

### Phase 1 -- FINDERS (parallel `agent()` calls, structured output)
One finder per (dimension x subsystem) cell, reading `src/` **and** the relevant
`mujoco/include/` headers and `mujoco/src/` C++ source to confirm any claimed C/C++ behavior.
Always include three specialists: a **thread-safety** finder that walks *every* `Send`/`Sync`
site, a **mutability** finder hunting `&self` methods that drive C-side mutation, and a
**lifetime/variance** finder. Each returns structured candidates: `{ dimension, location
(file:line), mechanism, safe_code_trigger, severity, reachable_from_safe }`.

**Every finder must read the actual C/C++ source** before claiming that a C function mutates
state, performs an unchecked access, or does anything else relevant to the finding. Citing a
function name without reading its implementation is not sufficient. If the C/C++ source was
not read, the candidate must be dropped.

### Phase 2 -- TRIAGE / DEDUP (barrier)
Collect all candidates; dedupe by location+mechanism; drop out-of-scope items
(unsafe-only-reachable; stride/length/type).

### Phase 3 -- ADVERSARIAL VERIFICATION (pipeline, per surviving candidate)
For each candidate spawn, independently:
- a hostile **reachability refuter** prompted to *prove the issue is NOT reachable from safe
  code* (default to "refuted" when uncertain), and
- an independent **re-deriver** that reconstructs the mechanism from scratch without seeing the
  finder's notes.
Keep a finding only if it survives both (genuinely reachable + independently reproduced).
Apply the same C/C++ citation rule from Phase 1: read the source, cite file:line, or drop the claim.

For thread-safety and other type-level claims, settle disputes with **compile-level evidence**:
- a minimal **standalone repro** that mirrors the pattern (compile + run), and
- a **real-crate `cargo check` probe** (a throwaway example/test) proving the offending code
  type-checks against the *real* types -- e.g. that
  `body.geom_iter_mut(true).collect::<Vec<&mut MjsGeom>>()` followed by `thread::scope` spawning
  per-handle compiles, and that `&mut MjsGeom: Send`.
Delete all throwaway probes afterward.

### Phase 4 -- COMPLETENESS CRITIC
A final agent asks: "which dimension / module / `Send`-`Sync` site / claim was not covered or not
verified?" Each gap becomes another finder round. Loop until a round produces no new candidates.

Severities: Critical / High / Medium / Low / Uncertain.
Statuses (exactly four -- do not invent others; there is NO "Accepted"):

| Status | When to use | Assignment rule |
|---|---|---|
| **Open** | Confirmed, not yet fixed | everything else |
| **Defer (MuJoCo)** | Root cause is upstream; unfixable in the Rust wrapper alone | root cause is in MuJoCo C/C++ |
| **Open (latent)** | Real bug, not practically triggerable today | requires e.g. >2^31 elements or similar |
| **Fixed** | Fix was applied outside this skill run | only when the fix already exists in the codebase |

## Build env for compile-level verification

`cargo check`/build run `build.rs`, which must discover MuJoCo. Use the absolute env-var path so
the pinned-version pkg-config check is bypassed (`cargo check` does not link, so a version
mismatch is irrelevant):

```bash
export MUJOCO_DYNAMIC_LINK_DIR="$(realpath mujoco-3.9.0/lib)"
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR"
```

(See the `build` and `run-example` skills.) Use `cargo check` for type-level probes; only
`cargo build`/`run` when demonstrating a runtime crash.

## Deliverable -- a single self-contained HTML report

Write/overwrite `mujoco-rs-memory-safety-audit.html` at the repo root.

**Scope: the current chat session.** The report accumulates findings across ALL `/safety` runs
within this chat. Findings fixed between runs stay in the report with status `fixed` -- they
are never silently removed. Do NOT carry over findings from a prior chat.

Concretely:
- Do **not** read the on-disk HTML file to repopulate findings -- it belongs to a prior chat
  and must be discarded entirely.
- If `/safety` was already run earlier in this chat, the main loop (Claude) must pass those
  prior findings (including their current statuses) as `args` to the Workflow so the new run
  can merge with them: update statuses where fixes were applied, add any new confirmed findings.
- If this is the first `/safety` run in this chat, start with an empty finding list.
- If no new findings were confirmed, say so clearly. Do NOT back-fill with findings from prior
  chats or from memory.

The file must be self-contained (inline `<style>`), ASCII-only, and contain:

- A header and a short **method note** describing the passes and the four dimensions.
- An **overview table** of all findings accumulated this chat: `ID | Severity (pill) |
  Dimension (badge) | One-line description | Location(s) | Reachable from safe code? | Status`.
  Findings that were fixed this session appear with status `fixed`. If no findings exist yet,
  the table says "No findings confirmed in this session."
- One **finding card** per finding: ID, severity, title, status badge, location(s), a **concise
  (< 500 words)** mechanism explanation, a **"Safe-code trigger"** code snippet, and a **"Fix"**
  suggestion (description of a fix only -- do NOT implement it).
- An **"Observations & hardening recommendations"** section containing ONLY items that were
  **actually verified against `mujoco/src/` or `mujoco/include/`** during this session (cite
  file:line). **Do NOT include any observation not verified against the actual C/C++ source.**
  Unverified hypotheticals ("if X were to do Y...") are forbidden.
- An **"Areas reviewed and found sound"** section: negative results (what was checked and is OK).

Styling conventions: severity pills (`sev-critical/high/medium/low/uncertain`), status badges
(`st-open` / `st-fixed` / `st-defer-mujoco` / `st-open-latent`), dimension badges.

## After the audit

Assign statuses using the table in Phase 4 -- do NOT pause to ask the user. Present the
overview table to the user as a plain-text summary. No interactive questions. Do NOT modify
any source files, tests, docs, or the changelog -- the audit is strictly read-only.
