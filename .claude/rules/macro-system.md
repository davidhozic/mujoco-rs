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

#### Mutable field safety: `(mut = unsafe)` and `(allow_mut = false)`
Fields are mutable by default. Two prefixes restrict mutability, each belonging to a **different
macro** (they cannot be combined):

- `(mut = unsafe)` in **`array_slice_dyn!`**: generates `unsafe fn field_mut()`; the caller upholds
  the C-side invariants.
- `(allow_mut = false)` in **`getter_setter!`**: suppresses `field_mut()` entirely.

**Safety criterion**: a field whose VALUES are used by C as unguarded array INDICES
(`arr[field[i]]` with no upper-bound check) needs `(mut = unsafe)` at minimum. Pure numeric data
(forces, positions, velocities, matrices) is always safe to mutate.

**Known `(mut = unsafe)` fields**: `contact` (`mj_sensorAcc()` indexes `geom_bodyid[]` with only a
`>= 0` guard), `flexedge` and `geoms` (`render_gl3.c` vertex/material indices without bounds
checks). **Known safe**: `geomorder` (`mjr_render()` repopulates it before reading).

When adding or reviewing an `array_slice_dyn!` invocation, trace how C uses the field's VALUES
before choosing the prefix.

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
