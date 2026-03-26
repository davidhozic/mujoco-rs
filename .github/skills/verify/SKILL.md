---
name: verify
description: Systematically verify correctness of the mujoco-rs wrapper layer using parallel subagents. Use this when asked to audit, verify, or deeply check the codebase for bugs.
---

# Comprehensive Codebase Verification

Use this workflow to systematically verify correctness of the mujoco-rs wrapper layer: type
mismatches with MuJoCo C, stride/length errors in macro invocations, unsafe code safety,
and potential undefined behavior.


## Phase 1 - Understand the codebase
Before spawning any agents, read enough of the codebase to build a precise task list:
1. Read `src/util.rs` - all macro definitions.
2. Glob `src/wrappers/**/*.rs` and read ALL resulting files.
3. Glob `mujoco/include/mujoco/*.h` to discover available C headers; read those relevant to the wrappers.

## Phase 2 - Build a subtask list
Build a numbered list covering all verification items. Organize by concern group:

### A. Stride / dimension correctness (info_method! fields)
- Each `info_method!` field stride vs mjmodel.h / mjdata.h dimension comment
- Each `info_with_view!` accessor type dimension vs the corresponding `info_method!` stride

### B. Array slice correctness (array_slice_dyn!)
- All `array_slice_dyn!` invocations: check the length expression matches the C header
- `sublen_dep` variant: inner * outer dimensions match C layout
- `summed` variant: length array and multiplier are correct
- Cast types in array slices (e.g., `[MjtNum; 3]`) match C struct dimensions

### C. Memory safety in unsafe blocks
- Address-sentinel macros (e.g. `mj_view_indices!`): off-by-one in sentinel/-1 handling, last-element fallback
- Pointer-view types: null pointer must return empty slice, not UB
- Index-to-mapping macros: verify they select the correct count/mapping field for the requested dimension
- Pointer arithmetic (`.add()` offsets): offset must match the field's definition in the C struct
- Error string buffers passed to C: size must be large enough for MuJoCo's maximum error length
- Slices built from a raw pointer + C count field: null-pointer guard must exist; count field must match the array
- Output array buffers passed to C functions: size must match what the C function expects to write
- Push/pop or append operations on fixed-size C arrays: bounds must be checked before write

### D. Logic correctness
- ID/index bounds checks before FFI calls: verify comparison operators (`>=`/`>`, `<`/`<=`) are correct
- Enum-dispatch match blocks: verify all variants map to the correct C value; no arm matches the wrong case
- Boolean return value interpretation: verify `== 1` vs `!= 0` matches what the C function documents
- Boolean-to-int conversions passed to C: verify the encoding matches the C convention (0/1 or other)
- Length/size checks on user-supplied slices before passing to C: verify the check is against the right field
- Bitmask subset checks and buffer-size validation: correct operator and correct reference value
- **Return code completeness**: for every C function whose return codes are mapped in Rust,
  enumerate ALL return codes defined in the C header comment. Any Rust arm that maps a code
  the C API never produces is a bug (invented error variant). Any real C code not matched is
  also a bug (falls through to a wrong catch-all).

### E. Type safety
- C utility function calls (e.g. zero/fill/copy helpers): check for type mismatches in element type or size
- `CStr::from_ptr` and similar: pointer must be NUL-terminated and valid for the lifetime used
- `bytemuck` casts: alignment and size of source and target types must be compatible
- Union-based transmutes: size and alignment assertions must be present and correct
- Enum-to-int conversions: resulting integer must be a valid discriminant for the C enum

### F. Lifecycle and concurrency
- Drop impls for wrapper types: verify the correct C free/destroy function is called and only once
- `Box<MaybeUninit>` patterns: all fields must be fully written before `assume_init` is called
- `unsafe impl Send`/`Sync` on generic wrapper types: the generic parameter must also be bounded `Send`/`Sync`
- Model-signature checks in view methods: verify the check is present and covers all access paths

### G. Safety-absent guards
- **`debug_assert`-only bounds**: any bounds check before an FFI call that uses only
  `debug_assert!` is absent in release builds. Flag these as Medium severity; the fix is
  to promote to `assert!` or verify the C function handles the invalid value gracefully.

### H. Edge cases and misc
- Count fields where `nX != nY` for related concepts (e.g. nq vs njnt for joints with multiple DOFs)
- Last-element / boundary fallback in address-array macros
- Test coverage gaps
- Build linkage correctness

## Phase 3 - Spawn 10 parallel verification subagents
**Critical: split the task list into 10 roughly equal, non-overlapping chunks.**
Do NOT give every agent the full list. Each agent receives ONLY its chunk.

Assignment scheme (for N tasks total):
- Agent 1: tasks 1..N/10
- Agent 2: tasks N/10+1..2*N/10
- ...
- Agent 10: tasks 9*N/10+1..N

Each agent's prompt MUST include these exact instructions (copy them verbatim):

```
VERIFICATION PROTOCOL - follow this exactly for EVERY item you check:

1. READ THE RUST SOURCE FILE FIRST. Find the actual code and write down the exact value,
   expression, or logic. Do NOT assume it is correct.
2. READ THE REFERENCE (C header, MuJoCo docs, or Rust safety rules) SECOND.
   Write down the expected value, type, or behavior.
3. COMPARE the two. If they differ, report a bug.

QUOTING RULE: You MUST quote the actual code for every item you check. E.g.:
  "geom aabb: info_method says 6, header says (ngeom x 6) -> OK"
  "body_pos slice uses ffi().nbody, header says (nbody x 3) -> length OK, cast [MjtNum; 3] OK"
  "contacts() null check: present -> OK"
  "error_buffer size: [0i8; 100], mj_loadXML gets 100 -> OK (MuJoCo uses < 500 chars)"
  "jac body_id check: >= 0 && < nbody -> OK (no off-by-one)"
If you cannot find the code value, say UNCERTAIN.

BUG CATEGORIES TO CHECK (depending on your assigned tasks):

A. STRIDE BUGS: Read info_method! blocks (field: NUMBER syntax), compare with C header
   dimension comments. CRITICAL: read info_method! block, NOT info_with_view! block.

B. ARRAY SIZE BUGS: Read array_slice_dyn! length expressions, compare with C header.
   Check cast types match C dimensions (e.g. [MjtNum; 3] for x3 arrays).

C. MEMORY SAFETY: Check null/zero guards in unsafe blocks, error buffer sizes,
   slice construction from raw pointers, off-by-one in bounds.

D. LOGIC BUGS: Check comparison operators (>= vs >), boolean conversions,
   enum matching completeness, return value interpretation.

E. TYPE SAFETY: Check casts, bytemuck usage, enum-to-int conversions.
```

Additionally include:
1. The specific numbered tasks the agent must verify (copy-pasted, not summarized).
2. Pointers to the files and C headers it needs to read.

Keep agent prompts concise -- include only the protocol, tasks, and file references.
Do NOT include lengthy preambles, motivation, or full codebase summaries.

Each subagent returns:
```
CONFIRMED_OK: field(code_value == header_value), ...
BUGS_FOUND: field in file -- code says X, header says (dim x Y), should be Y
UNCERTAIN: <list of items that need another look>
```

All 10 agents MUST be spawned simultaneously (in a single parallel batch).

## Phase 4 - Consensus check & resolution
After all 10 agents report:
1. Collect all `BUGS_FOUND` items.
2. For each bug found by any agent, spawn a small batch of focused resolution agents
   (3-5) to confirm or reject it. Only do this for items that are ambiguous.
3. Repeat until all disputed items are resolved.

## Phase 5 - Evaluation agents
Spawn 3 evaluation agents. Each reads all previous agent reports and:
- Deduplicates identical findings.
- Assigns final severity (Critical / High / Medium / Low / Info).
- Flags any confirmed bugs that are actually false positives.

## Phase 6 - Final report and fixes
Produce a structured final report:
```
# Verification Report

## Critical bugs
...

## High bugs
...

## Medium / Low
...

## Confirmed OK (notable items)
...
```
Then implement fixes for all Critical and High items. Run `/test` after every fix.

> [!NOTE]
> - Always run `/test` as a baseline before starting.
> - Set both `MUJOCO_DYNAMIC_LINK_DIR` and `LD_LIBRARY_PATH` (see `important-context.md`).
> - Do NOT run `cargo expand` unless the user explicitly requests it -- it is very slow.
> - For memory/UB verification beyond static analysis, run `/asan` and `/miri` after fixing.
