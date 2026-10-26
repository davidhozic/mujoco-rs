# Macro System

This crate uses declarative macros extensively to reduce boilerplate when wrapping MuJoCo's C arrays.

## Before modifying wrapper files
1. **Read `src/util.rs`** first: all core macro definitions. Understand their syntax first.
2. **Cross-reference the MuJoCo C API docs** (`coding-conventions.md`) for field sizes and types.

## Finding macros
- All macros live in `src/util.rs` (search `macro_rules!`); read each macro's doc comment before
  invoking it. Existing invocations in `src/wrappers/` show usage patterns.
- If `../mujoco-rs-utils` exists, its CLI auto-generates macro invocations from the C headers
  (`--help`).

## Key macro reference

### `mj_model_dyn_range!(model, id, nx)`
Resolves the `(start, len)` pair for the contiguous slice owned by element `id`; `nx` selects the
address-array mapping:

| `nx` | address array | total length |
|---|---|---|
| `nq` | `jnt_qposadr` | `nq` |
| `nv` | `jnt_dofadr` | `nv` |
| `nsensordata` | `sensor_adr` | `nsensordata` |
| `ntupledata` | `tuple_adr` | `ntupledata` |
| `ntexdata` | `tex_adr` | `ntexdata` |
| `nnumericdata` | `numeric_adr` | `nnumericdata` |
| `nhfielddata` | `hfield_adr` | `nhfielddata` |
| `na` | `actuator_actadr` | `na` |
| `nJten` | `ten_j_rowadr` | `n_jten()` |

Returns `(start_addr, n)`. Address entry `-1` yields `(0, 0)`. For the last element with all
subsequent entries `-1`, the total-length field is the exclusive end.

### `array_slice_dyn!` - three variants
Creates raw-pointer slices safely (null-pointer and zero-length guarded):

| Variant | Usage | Length formula |
|---|---|---|
| Basic | `array_slice_dyn!(ptr, len)` | `len` |
| `sublen_dep` | `array_slice_dyn!(sublen_dep => ptr, outer, inner)` | `outer * inner` |
| `summed` | `array_slice_dyn!(summed => ptr, len_array)` | sum of `len_array` entries |

#### Sanitizer probe: `probe = <name>;`
A scalar-arm block may open with `probe = <name>;`. The macro then also generates two
`#[cfg(test)] pub(crate)` methods that touch the first and last element of every slice the block
declares, on the read path and the write path. Each write restores the value it read, so no field
changes.

| Method | Signature | Covers |
|---|---|---|
| `<name>` | `fn (&mut self)` | reads through a safe getter; writes through a safe setter |
| `<name>_unsafe` | `unsafe fn (&mut self)` | every accessor the first one leaves out |

The split is exact: each accessor lands in one method, never both, never neither. Call `<name>` on
a freshly built value, before any pipeline stage: that is the claim a safe accessor makes, so a
sanitizer fault there is a real bug. Call `<name>_unsafe` only after the stage its
`(read = unsafe)` fields need. The safe method being a safe `fn` is itself a check: an accessor
that the split misroutes fails to compile.

The two helpers it calls live in `src/util/testing.rs`, a `#[cfg(test)]` `pub(crate)` submodule
of `util`. Both clone the element and pass the clone through `black_box`: the sanitizer
instruments a load or a store, not the creation of a reference, and a plain identity write
disappears in the optimizer before the sanitizer pass runs. Every element type an
`array_slice_dyn!` block casts to is therefore `Clone`.

Call both from one `#[test]` per type. `MjData`, `MjModel` and `MjvScene` have one; `MjrContext`
does not, because it needs a live GL context.

Under `/asan` a length that overruns its allocation faults inside the probe. MuJoCo defines
`mjUSEASAN` automatically when it is compiled with `-fsanitize=address`, and then poisons the arena
past `parena`, so an arena array that reads beyond its true length is caught as well. Against a
stock prebuilt `libmujoco` that poisoning is absent, so only overruns past a whole `malloc` block
are caught.

#### Field safety: `(mut = unsafe)`, `(read = unsafe)` and `(allow_mut = false)`
Fields are safe and mutable by default. Three prefixes restrict access; the first two belong to
`array_slice_dyn!` and cannot be combined with each other, the third belongs to a **different
macro**:

- `(mut = unsafe)` in **`array_slice_dyn!`**: generates `unsafe fn field_mut()`; the caller upholds
  the C-side invariants.
- `(read = unsafe)` in **`array_slice_dyn!`**: generates `unsafe fn field()` **and**
  `unsafe fn field_mut()`. For an `mjData` arena array that MuJoCo allocates but does not zero, so
  a read before its computing stage reads uninitialized memory. The caller runs the stage first.
- `(allow_mut = false)` in **`getter_setter!`**: suppresses `field_mut()` entirely.

**`(mut = unsafe)` criterion**: C uses the field's VALUES as unguarded indices, offsets, lengths
or discriminants (`arr[field[i]]` with no upper-bound check, or only a `>= 0` guard). Pure numeric
data (forces, positions, velocities, matrices) is safe to mutate. A field that C rewrites before
every read stays safe.

**`(read = unsafe)` criterion**: an `mjData` arena array that MuJoCo allocates in one stage and
fills in a later stage, or fills only under some option. The arena is NEVER zeroed (`mju_malloc`,
and a reset only sets `parena = 0`); a fresh read of zero is only the kernel handing out zero pages.
For a numeric field the `_mut` accessor is unsafe only because `&mut [T]` also allows reads; do not
read that as an index hazard.

**How to decide.** Trace the C source, not the header comment:

1. Find the allocation (`mj_arenaAlloc*`, a module-local arena macro) and the pipeline stage that
   runs it (e.g., `mj_fwdPosition`, `mj_fwdVelocity`, `mj_fwdActuation`, `mj_fwdAcceleration`,
   `mj_fwdConstraint`, `mj_inverse`).
2. Find every writer. Note whether it fills the whole length the accessor exposes, or only a
   counted prefix, and in which stage it runs.
3. Check the conditions on each writer (e.g., `opt.solver`, `opt.jacobian`, `opt.integrator`, enable and
   disable flags). A branch that skips the fill leaves the array uninitialized.
4. Check what the pointer holds when the array is not built: `mj_clearEfc` sets arena pointers to
   NULL, and the accessor then returns `&[]`.
5. Check the length field. A count that the same stage sets together with the values (`nefmA`
   with the actuator arrays) keeps the slice initialized.
6. Check for packed buffers: a numeric array can carry an integer layout in its tail. Such a field
   is unsafe both to read and to mutate.
7. Trace how C uses the VALUES (indices, loop bounds, divisors) to choose `(mut = unsafe)`.
8. Other unwritten rules.

### `view_creator!(field, start_ffi_field, data_ptr, type_)`
Generates `fn field(&self) -> &[T]` and the `_mut` variant via `(offset, len)` and
`data_ptr.add(offset).cast::<T>()`. The cast target must match `type_` in the info struct.

### `info_method!(Kind, ffi_model, element_type, id, fields...)`
Generates per-element accessors (`body()`, `joint()`, ...) on `MjModel`/`MjData`. A field entry is a
static stride (`xpos: 3` -> `id * 3` offset, 3-element slice), a dynamic view via
`mj_model_dyn_range!`, or a zero stride returned as `Option::None`.

> **WARNING**: Each element type has TWO macro blocks. Verify strides in the `info_method!` block
> (stride numbers, first half of the file, where bugs live), NOT the `info_with_view!` block
> (types and doc strings, second half); its annotations can be right while the stride is wrong.

### `c_str_as_str_method!(field, inner_field, len)`
Getter returns `&str` via `CStr::from_ptr`; setter copies into the fixed `[i8; N]` buffer with
`copy_from_slice`. Always NUL-terminated.

### `cast_mut_info!(expr, TargetType)`
Wraps `bytemuck::checked::try_cast_mut`. Valid only when target and source match in size and
alignment.

## Verification checklist when adding/reviewing an `info_method!` field
1. Read the actual stride in the `info_method!` invocation; write it down.
2. Find the field in `mujoco/include/mujoco/mjmodel.h` (or `mjdata.h`); read its dimension comment
   (e.g. `// (nbody x 3)`).
3. The two must match exactly. Element types: `mjtNum` = `f64`, `int` = `i32`, `mjtByte` = `u8`.
4. Add a unit test on a real non-trivial model.

> **WARNING**: never confirm from the header alone; quote the actual code value alongside the
> header value.

## Verification checklist for `array_slice_dyn!` invocations
1. Read the length expression (e.g. `ffi().nbody`) and cast type (e.g. `[MjtNum; 3]`); write both
   down.
2. Check against the header's dimension comment: length uses the FIRST dimension's count, the
   cast's inner size matches the SECOND (`body_pos`: `ffi().nbody`, `[MjtNum; 3]` for `(nbody x 3)`).
3. Null/zero guards: the generated code returns `&[]` for a null pointer or zero length.

## Verification checklist for unsafe blocks (non-macro)
1. **Bounds**: index validated before the FFI call; `< max` not `<= max`, `>= 0` not `> 0`.
2. **Buffer sizes**: caller-supplied write buffers are large enough (`mj_contactForce` needs
   `[MjtNum; 6]`).
3. **Null pointers**: every dereference guarded (`PointerView` guards via `Deref`; manual slices
   need explicit checks).
4. **Return values**: sentinels differ per function (0 vs 1, -1 vs null); check the header.
5. **Casts**: `as i32`/`as usize` must not truncate or wrap; handle `-1` sentinels before unsigned
   conversion.
