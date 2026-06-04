---
name: safety
description: Adversarial multi-agent soundness audit of the mujoco-rs SAFE API surface -- memory safety, thread safety (every unsafe impl Send/Sync), &self vs &mut self mutability, and lifetime/variance correctness, restricted to defects reachable from 100% safe code. Produces a single self-contained HTML report. Use when asked to audit/check the crate for unsoundness or undefined behavior reachable from safe code. Distinct from /verify (which covers stride/length-field/type correctness).
---

# Safety & Soundness Audit (Claude Code only)

This is a **Claude-Code-only** skill: it drives an adversarial, multi-agent audit using the
`Workflow` tool (fan-out finders -> triage/dedup -> adversarial verification -> completeness
critic), the `Agent` tool, and `AskUserQuestion`. It has no `.github/skills` counterpart, so the
full instructions live here. It is **not** the `/verify` skill: `/verify` checks
stride/length-field/type-mismatch correctness; this skill checks *soundness* (memory/thread/
mutability/lifetime UB) reachable from safe code. Do not run `/verify` as part of this audit.

Obey **all** files in `.github/rules/` at all times (ASCII-only output, never edit
`src/mujoco_c.rs`, no `cargo expand` unless asked, import ordering, disallowed git/network
commands, etc.). Re-read them after any context compaction.

## Ground rules (non-negotiable scope)

- **`src/` only.** Audit the safe Rust wrapper. `mujoco/src/**` C/C++ is read *only* to confirm
  a mechanism, never reported as a finding.
- **Reachable from 100% SAFE code only.** The safe API must never cause UB for any safe input.
  Anything only reachable through an `unsafe fn` or an `unsafe` block the *caller* must write is
  out of scope. The safe/unsafe tier is the dividing line.
- **Stride/length/type correctness is OUT of scope** -- that is `/verify`'s job. Do not duplicate
  it.
- **Do not re-flag known/dispositioned findings.** Build the do-not-re-flag list FIRST (see
  Phase 0) from the project's known-findings memory and the changelog.
- **The changelog is not part of the audit deliverable.** Per project convention, a changelog
  entry is added only when a fix is actually made -- not for an open finding. The audit's output
  is the HTML report.

## The four dimensions

1. **Memory safety / UB** -- OOB read/write reachable from safe code; invalid transmute / invalid
   enum discriminant (e.g. `force_cast` of unchecked C `int*` into a fieldless `#[repr]` enum);
   use-after-free; uninitialized/stale reads; integer-overflow-driven OOB (e.g. `w*h*3` usize
   overflow defeating a buffer check); mis-sized buffers handed to C; `len() as i32` truncation.
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
3. Build the **do-not-re-flag** list by reading the known-findings memory
   (`project_known_memory_safety.md`) and `docs/guide/source/changelog.rst`.
4. Record the MuJoCo build env for later compile checks (see "Build env" below).

### Phase 1 -- FINDERS (parallel `agent()` calls, structured output)
One finder per (dimension x subsystem) cell, reading only `src/` (+ relevant `mujoco/include`
headers and `mujoco/src` C++ to understand mechanisms). Always include three specialists:
a **thread-safety** finder that walks *every* `Send`/`Sync` site, a **mutability** finder hunting
`&self` methods that drive C-side mutation, and a **lifetime/variance** finder. Each returns
structured candidates: `{ dimension, location (file:line), mechanism, safe_code_trigger,
severity, reachable_from_safe }`.

### Phase 2 -- TRIAGE / DEDUP (barrier)
Collect all candidates; dedupe by location+mechanism; drop out-of-scope (unsafe-only-reachable;
stride/length/type) and already-known items.

### Phase 3 -- ADVERSARIAL VERIFICATION (pipeline, per surviving candidate)
For each candidate spawn, independently:
- a hostile **reachability refuter** prompted to *prove the issue is NOT reachable from safe
  code* (default to "refuted" when uncertain), and
- an independent **re-deriver** that reconstructs the mechanism from scratch without seeing the
  finder's notes.
Keep a finding only if it survives both (genuinely reachable + independently reproduced).
**Confirm every mechanism against the actual C/C++ source in `mujoco/src/`** (e.g. trace
`mjs_setName` -> `mjCModel::CheckRepeat` / `SetError`) -- do not trust summaries.

For thread-safety and other type-level claims, settle disputes with **compile-level evidence**:
- a minimal **standalone repro** that mirrors the pattern (compile + run), and
- a **real-crate `cargo check` probe** (a throwaway example/test) proving the offending code
  type-checks against the *real* types -- e.g. that
  `body.geom_iter_mut(true).collect::<Vec<&mut MjsGeom>>()` followed by `thread::scope` spawning
  per-handle compiles, and that `&mut MjsGeom: Send`.
Delete all throwaway probes afterward.

### Phase 4 -- COMPLETENESS CRITIC
A final agent asks: "which dimension / module / `Send`-`Sync` site / claim was not covered or not
verified?" Each gap becomes another finder round. Loop until a round is dry.

Severities: Critical / High / Medium / Low / Uncertain. Statuses: open / fixed / deferred /
accepted / latent.

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

Write/overwrite `mujoco-rs-memory-safety-audit.html` at the repo root. The report scope is the
**current chat conversation**: it must include findings from ALL `/safety` runs within this chat,
but MUST NOT carry over findings from previous chats.

Concretely:
- Do **not** read the on-disk HTML file to repopulate findings -- that file may be from a prior
  chat and its findings must be discarded.
- If `/safety` was already run earlier in this chat, the main loop (Claude) must pass those
  prior findings as `args` to the Workflow so they are merged with the new run's findings.
- If this is the first `/safety` run in this chat, start with an empty finding list.

The file must be self-contained (inline `<style>`), ASCII-only, and contain:

- A header and a short **method note** describing the passes and the four dimensions.
- An **overview table**: `ID | Severity (pill) | Dimension (badge) | One-line description |
  Location(s) | Reachable from safe code? (Yes/No) | Status`.
- One **finding card** each: ID, severity, title, status badge, location(s), a **concise
  (< 500 words)** mechanism explanation, a **"Safe-code trigger"** code snippet, and a **"Fix"**
  suggestion. If a fix has been applied, set status `fixed` and state what was done and how it was
  verified.
- An **"Observations & hardening recommendations"** section: faith-based invariants and latent
  items (e.g. "`const mjModel*` => no mutation" assumption behind `MjModel: Sync`; iterator
  traversal-injectivity assumption; native-viewer GL-thread affinity relying on `winit`
  auto-`!Send`).
- An **"Areas reviewed and found sound"** section: negative results (what was checked and is OK).

Styling conventions used previously: severity pills (`sev-critical/high/medium/low/uncertain`),
status badges (`st-open/fixed/deferred/accepted/latent`), and dimension badges.

## After the audit

1. Present the overview table to the user and collect a **disposition per NEW finding** with
   `AskUserQuestion` (fix now / fix by maintainer / accept / defer / document-only).
2. When a finding is **fixed**: add a `docs/guide/source/changelog.rst` entry under
   `.. rubric:: Bug fixes` (plus a `(Potentially breaking)` note under `.. rubric:: Breaking
   changes` if the public API/traits change); add a regression guard where feasible (e.g. a
   `compile_fail` doctest locking in a `!Send` guarantee); and update the known-findings memory
   so it is not re-flagged.
3. Reflect every disposition's status in the HTML report (current-session findings only).

## Current known findings (do NOT re-flag; reconcile with `project_known_memory_safety.md` + changelog)

- **F1 (Critical)** -- `Info::view`/`view_mut` authorize indexing a different model/data by
  comparing only `model_signature` (a structural-tree hash that omits `na`/`nuser_*`/
  `nsensordata` ...), so two structurally-identical models with different such dims collide ->
  non-panicking OOB. *Deferred upstream.*
- **F2 (High)** -- `MjvPerturb.select` OOB into `mjv_updateScene`. *Fixed.*
- **F3 (Low)** -- `len() as i32` truncation in `mjs` vector writers. *Accepted.*
- **F4 (High)** -- blanket `unsafe impl Send`/`Sync` on `Mjs*`/`MjsDefault` + lending mutable
  iterators let aliasing `&mut` handles into one shared spec arena cross threads -> data race in
  `mjs_setName`. *Fixed: handles made `!Send + !Sync`; `MjSpec` stays `Send + Sync`.*
- **F5 (Medium)** -- safe getters (`efc_state`/`iefc_state`, scalar enum getters) transmute
  unchecked C `int*` into fieldless `#[repr]` enums with no discriminant check -> invalid-enum UB
  on stale arena bytes. *Maintainer to fix.*
- **F6 (Uncertain/latent)** -- 32-bit `w*h*3` usize overflow can defeat the `read_pixels` buffer
  check; no safe-reachable OOB write confirmed. *Latent (use `checked_mul`).*
