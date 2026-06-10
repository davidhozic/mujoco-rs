---
name: verify
description: Systematically verify correctness of the mujoco-rs wrapper layer using parallel subagents. Use this when asked to audit, verify, or deeply check the codebase for bugs.
---

# Comprehensive Codebase Verification

Obey **all** files in `.github/rules/` at all times. Re-read them after any context compaction.

Systematically verify the mujoco-rs wrapper layer for type mismatches with MuJoCo C, stride/length
errors in macro invocations, unsafe code correctness, and potential undefined behaviour.

## Ground rules (non-negotiable scope)

- **`src/` only.** Verify the safe Rust wrapper. `mujoco/include/` and `mujoco/src/` are read
  *only* as the reference for field dimensions, types, and C function semantics.
- **Correctness only.** Stride/length/type correctness, logic bugs, lifecycle correctness, and
  setter/builder value-validation completeness (concern group I). Deep soundness analysis (UB
  reachable from safe code) is **OUT of scope** -- that is `/safety`'s job. For group I, only check
  that the existing validation *mechanism* (the optional `{ check, "reason" }` on a setter) is
  present wherever a field's value is constrained; do not perform a full soundness audit.
- **Quote both sides.** Every finding must quote the Rust code value AND the C header value.
  E.g., `info_method! says stride=3, header says (nbody x 3) -> OK` or `stride=4 vs (nbody x 3) -> BUG`.
  If you cannot find the Rust value, report UNCERTAIN; never assume it is correct.
- **No fixes during scouting or verification.** All code edits happen only in Phase 5 (Fix),
  after findings are fully evaluated. Do not edit source files in any earlier phase.

## The nine concern groups

### A. Stride / dimension correctness (`info_method!` fields)
- Each `info_method!` field stride vs `mjmodel.h` / `mjdata.h` dimension comment.
- Each `info_with_view!` accessor type dimension vs the corresponding `info_method!` stride.
- **Critical**: always read the `info_method!` block (stride numbers), NOT the `info_with_view!`
  block (accessor types). The `info_with_view!` types may be correct even when the stride is wrong.

### B. Array slice correctness (`array_slice_dyn!`)
- All `array_slice_dyn! { ... }` invocations (basic form): length expression matches the C header
  first-dimension count field; cast type (e.g. `[MjtNum; 3]`) matches C second dimension.
- `sublen_dep { ... }` block: inner * outer dimensions match C layout.
- `summed { ... }` block: length array, length-array count, and multiplier are correct.
- `(mut = unsafe)` prefix on fields whose values are used as unguarded C array indices: verify
  the field warrants unsafe mutation (check `macro-system.md` "Known `(unsafe)` fields" list).

### C. Memory safety in unsafe blocks
- Address-sentinel / dynamic-range macros (`mj_model_dyn_range!`): off-by-one in
  sentinel/-1 handling, last-element fallback, correct address/count fields per dimension.
- Pointer-view types (`PointerViewMut`): null pointer must return empty slice, not UB.
- Index-to-mapping: correct count/mapping field for the requested dimension.
- Pointer arithmetic (`.add()` offsets): offset matches field definition in C struct.
- Error string buffers passed to C: size large enough for MuJoCo's maximum error length.
- Slices built from a raw pointer + C count field: null-pointer guard present; count matches.
- Output array buffers passed to C functions: size matches what the C function writes.
- Push/append on fixed-size C arrays: bounds checked before write.

### D. Logic correctness
- ID/index bounds checks before FFI calls: comparison operators (`>=`/`>`, `<`/`<=`) correct.
- Enum-dispatch match blocks: all variants map to the correct C value.
- Boolean return value interpretation: `== 1` vs `!= 0` matches C function docs.
- Boolean-to-int conversions passed to C: encoding matches C convention.
- Length/size checks on user-supplied slices: check against the right C field.
- Bitmask subset checks and buffer-size validation: correct operator and reference value.
- **Return code completeness**: every C return code must map to a Rust arm; invented arms and
  missing real codes are both bugs.

### E. Type safety
- C utility function calls: no type mismatches in element type or size.
- `CStr::from_ptr` and similar: pointer must be NUL-terminated and valid for the lifetime used.
- `bytemuck` casts: alignment and size of source and target types must be compatible.
- Union-based transmutes: size and alignment assertions present and correct.
- Enum-to-int conversions: result must be a valid discriminant for the C enum.

### F. Lifecycle and concurrency
- Drop impls: correct C free/destroy function called exactly once.
- `MaybeUninit::uninit()` patterns: all fields of the struct must be written by the C
  initialiser (e.g. `mj_defaultOption`, `mj_defaultVFS`) before `assume_init()` is called.
- `unsafe impl Send`/`Sync` on generic wrapper types: generic parameter bounded `Send`/`Sync`.
- Model-signature checks in view methods: present and covers all access paths.

### G. Safety-absent guards (`debug_assert`-only bounds)
Any bounds check before an FFI call that uses only `debug_assert!` is absent in release builds.
Flag these as Medium severity; fix is to promote to `assert!` or verify the C function handles
the invalid value gracefully.

### H. Edge cases and misc
- `nX != nY` for related concepts (e.g. `nq` vs `njnt` for joints with multiple DOFs).
- Last-element / boundary fallback in address-array macros.
- Test coverage gaps.
- Build linkage correctness.

### I. Setter / builder value validation (`getter_setter!` / `vec_set!` checks)
The write-arms of `getter_setter!` (`set`, `with`, `[&] with`, their `force!` variants, and the
`get, set` / `with, get, set` / `with, get` aggregates) and the enum-conversion form of `vec_set!`
accept an optional check tail -- `{ check, "reason" } [=> ErrType]` -- that validates the value
before writing it and turns `set_*` into a fallible `Result<(), ErrType>` (the builder `with_*`
panics). For every such field, decide whether the value is **constrained** by the C side, and verify
the check is applied iff it is needed:

- **Constrained but unchecked = BUG.** A field whose value the C code later uses without its own
  bounds/validity check -- as an array index (e.g. an `MjtObj` used to index `object_lists_[type]`),
  a count/capacity, a discriminant switched on without a `default`, or any value with a C-side range
  precondition -- must carry a `{ check, "reason" }`. A `set`/`with` invocation that omits it writes
  an unvalidated value. Quote the Rust invocation (does it carry a check?) AND the C usage site.
- **Unconstrained but checked = Low/Info.** A check on a field the C side already validates, clamps,
  or accepts in full range (e.g. a physics scalar) is unnecessary noise; flag it as Low.
- **Check correctness.** Where a check *is* present, the predicate must actually reject every invalid
  value (e.g. `t < mjNOBJECT`, not `t <= mjNOBJECT`) and the `"reason"` doc fragment must name the
  real error/condition.
- **bool fields never need a check** (always in range); do not flag them.

This group overlaps with `/safety` (a constrained-but-unchecked field is often an OOB-from-safe-code
hole); here verify only the *presence and shape* of the check mechanism, not a full soundness proof.

## Methodology -- parallel multi-agent (drive with the `Workflow` tool)

### Phase 0 -- Inline recon (before launching Workflow)

Do this in the main conversation before calling `Workflow`:

1. Read `Cargo.toml` to find the MuJoCo version suffix (`+mj-X.Y.Z`).
2. Confirm `mujoco-X.Y.Z/lib/` exists and record its absolute path as `MUJOCO_LIB`.
3. List `src/wrappers/` to see which wrapper files are present.

Then call `Workflow` with `args: { mujoco_lib: "<abs-path>" }`.

### Phase 1 -- SCOUT (single agent, structured task list output)

One agent reads all source files and C headers and emits a structured, numbered task list:

1. Read `src/util.rs` (all macro definitions).
2. Glob and read all files in `src/wrappers/**/*.rs`.
3. Glob `mujoco/include/mujoco/*.h`; read those relevant to the wrappers
   (`mjmodel.h`, `mjdata.h`, `mjspec.h`, `mjtype.h`, `mujoco.h`).
4. For every `info_method!`, `array_slice_dyn! { ... }` (basic form),
   `array_slice_dyn! { sublen_dep { ... } }`, `array_slice_dyn! { summed { ... } }`,
   `mj_model_dyn_range!`, unsafe block, enum dispatch, bounds check, Drop impl,
   `unsafe impl Send/Sync`, and every value-writing `getter_setter!` / `vec_set!` field
   (for group I) -- emit one numbered task entry with:
   `{ id, category (A-I), description, rust_file, rust_lines, c_header }`

The scout returns a JSON array of tasks. The total task count typically ranges from 150-400.

### Phase 2 -- VERIFY (parallel, 10 agents, structured output)

Split the task list into 10 roughly equal non-overlapping chunks (chunk `k` = tasks
`[k*N/10 .. (k+1)*N/10)`). Spawn all 10 agents simultaneously.

Each agent receives its chunk and follows this protocol verbatim:

```
VERIFICATION PROTOCOL -- follow exactly for EVERY item:

1. READ THE RUST SOURCE FILE FIRST. Find the actual value, expression, or logic.
   Write it down. Do NOT assume correctness.
2. READ THE REFERENCE (C header or Rust safety rule) SECOND.
   Write down the expected value or behavior.
3. COMPARE. Report a bug if they differ.

QUOTING RULE: Quote the actual code for every item. Examples:
  "body_pos: info_method says 3, header says (nbody x 3) -> OK"
  "geom_aabb: info_method says 6, header says (ngeom x 6) -> OK"
  "contacts() null check: present -> OK"
  "jac body_id check: >= 0 && < nbody -> OK"
  "error_buffer: [0i8; 100], mj_loadXML gets 100 -> OK"
  "MjsSensor::objtype setter: has { check_objtype } check; C indexes object_lists_[objtype] -> OK"
  "<field> setter: no { check }; C uses value as unguarded array index / unchecked discriminant -> BUG (group I)"

For group I, the reference is the C *usage* of the field value (how the C side consumes it), not a
dimension comment. If you cannot find the Rust value, say UNCERTAIN.
```

Each agent returns structured findings:
`{ findings: [{ id, status ("OK"|"BUG"|"UNCERTAIN"), location, code_says, reference_says, severity, description }] }`

### Phase 3 -- RESOLUTION (pipeline, per disputed finding)

For each `BUG` or `UNCERTAIN` finding, spawn a small batch (3 agents) to confirm or reject it.
Keep a finding only if a majority confirms it. Drop items confirmed as false positives.

### Phase 4 -- EVALUATION (parallel, 3 agents)

Three independent evaluators each read all Phase 2-3 reports and:

- Deduplicate identical findings (same file + same issue).
- Assign final severity: Critical / High / Medium / Low / Info.
- Flag false positives.

Majority-vote on severity; take the highest severity when votes are split.

### Phase 5 -- FIX (single agent, sequential)

A single agent applies all Critical and High fixes in sequence:

1. For each fix, read the relevant Rust source and C reference.
2. Apply the minimal correct change.
3. After every individual fix, run:
   ```bash
   MUJOCO_DYNAMIC_LINK_DIR=<mujoco_lib> LD_LIBRARY_PATH=<mujoco_lib> \
     cargo test --features renderer 2>&1 | tail -20
   ```
4. If a test fails, diagnose and fix before continuing.
5. Do NOT fix Medium / Low / Info items unless the user explicitly asked.

### Phase 6 -- DOC CHECK (single agent)

After all code fixes are applied:

1. Run rustdoc:
   ```bash
   MUJOCO_DYNAMIC_LINK_DIR=<mujoco_lib> LD_LIBRARY_PATH=<mujoco_lib> \
     DOCS_RS=y cargo doc --no-deps --all-features 2>&1 | grep -E "^warning:|^error:"
   ```
2. Run Sphinx:
   ```bash
   cd docs/guide && make html 2>&1 | grep -E "WARNING:|ERROR:"
   ```
   (The pre-existing `_static` path warning is acceptable.)
3. Verify code blocks in `changelog.rst` and `migration.rst` match the code on HEAD.

## Build env

```bash
export MUJOCO_DYNAMIC_LINK_DIR="$(realpath mujoco-X.Y.Z/lib)"
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR"
```

Substitute `X.Y.Z` from the `+mj-X.Y.Z` suffix in `Cargo.toml`. Both vars must be set; see
`important-context.md`. For `cargo check` (type-checking only, no link), only
`MUJOCO_DYNAMIC_LINK_DIR` is strictly required.

## Deliverable -- a single self-contained HTML report

Write/overwrite `mujoco-rs-verify-report.html` at the repo root after Phase 5 (fixes applied).

**Scope: the current chat session.** The report accumulates findings across ALL `/verify` runs
within this chat. Findings fixed between runs stay in the report with status `Fixed` -- they
are never silently removed. Do NOT carry over findings from a prior chat.

Concretely:
- Do **not** read the on-disk HTML file to repopulate findings -- it belongs to a prior chat
  and must be discarded entirely.
- If `/verify` was already run earlier in this chat, the main loop (Claude) must pass those
  prior findings (including their current statuses) as `args` to the Workflow so the new run
  can merge with them.
- If this is the first `/verify` run in this chat, start with an empty finding list.

The file must be self-contained (inline `<style>`), ASCII-only, and contain:

- A header and a short **method note** describing the phases and the nine concern groups (A-I).
- An **overview table** of all findings accumulated this chat:
  `ID | Severity (pill) | Category (badge) | One-line description | Location | Fix applied? | Status`
  If no findings exist, the table says "No bugs confirmed in this session."
- One **finding card** per finding: ID, severity, title, status badge, location,
  a **concise (< 300 words)** description quoting both the Rust code value and the C reference
  value, and a **"Fix"** section describing what was changed (or what needs to change).
- A **"Confirmed OK"** section listing notable items that were checked and found correct
  (file:line, what was verified, result).

Styling: use the same Claude aesthetic as `mujoco-rs-memory-safety-audit.html` -- ivory canvas
background, coral accent, warm near-black ink, Georgia serif headings, rounded pill badges, white
finding cards on the canvas, and a hairline-border overview table. Severity pills
(`sev-critical/high/medium/low/info`), status badges (`st-open` / `st-fixed` / `st-wont-fix`),
category badges `cat-a` through `cat-i` each in a distinct hue.

Statuses:

| Status | When to use |
|---|---|
| **Open** | Confirmed bug, fix not yet applied |
| **Fixed** | Fix was applied in Phase 5 of this or a prior run this chat |
| **Wont Fix** | Confirmed but intentionally deferred (e.g. upstream MuJoCo issue) |

After writing the file, present the overview table to the user as a plain-text summary.
