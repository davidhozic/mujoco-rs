````markdown
---
description: Comprehensive correctness audit - type/stride safety, UB, memory
---

# Comprehensive Codebase Verification

Use this workflow to systematically verify correctness of the mujoco-rs wrapper layer: type
mismatches with MuJoCo C, stride/length errors in macro invocations, unsafe code safety,
and potential undefined behavior.

// turbo-all

## Phase 1 - Understand the codebase
Before spawning any agents, read enough of the codebase to build a precise task list:
1. Read `src/util.rs` - all macro definitions.
2. Read `src/wrappers/mj_data.rs`, `src/wrappers/mj_model.rs`, `src/wrappers/mj_visualization.rs`.
3. Read MuJoCo C headers: `mujoco/include/mujoco/mjmodel.h`, `mjdata.h`, `mjvisualize.h`.

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
- `mj_view_indices!` sentinel / fallback logic (off-by-one, -1 handling)
- `PointerView` / `PointerViewMut` null pointer returns empty slice (not UB)
- `mj_model_nx_to_mapping!` / `mj_model_nx_to_nitem!` map to correct fields
- `view_creator!` pointer arithmetic: `.add()` offset matches field definition
- Error buffer sizes in model loading (too small = truncation or overflow)
- `contacts()` slice: ptr null check, `ncon` cast, and struct layout
- `contact_force()`: output array size matches `mj_contactForce` expectation (6 elements)
- `MjvFigure` push/pop bounds

### D. Logic correctness
- Bounds checks in Jacobian functions (off-by-one, wrong comparison operator)
- Object type dispatch in `object_velocity`/`object_acceleration` (correct enum matching)
- `is_pyramidal` / `is_sparse` / `is_dual` return value interpretation (== 1 vs != 0)
- `forward_skip` / `inverse_skip`: boolean-to-int conversion correctness
- `constraint_update`: jar length check against nefc
- State extraction: bitmask subset check logic, buffer size validation

### E. Type safety
- `mju_zero` and similar calls: type mismatches (mjtNum, mjtByte, enum values)
- `c_str_as_str_method!` NUL termination / CStr::from_ptr safety
- `cast_mut_info!` bytemuck alignment and size compatibility
- `force_cast` union transmute: size and alignment assertions
- Enum conversions via `as i32` -- valid discriminant values

### F. Lifecycle and concurrency
- Visualization struct lifecycle (Drop impl correctness)
- Box<MaybeUninit> initialization patterns (fully written before use)
- Send/Sync impl safety for MjModel, MjData
- MjData model signature check in view methods

### G. Edge cases and misc
- Free joints and nq vs njnt discrepancy
- Last-joint fallback in mj_view_indices!
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
````
