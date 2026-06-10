# Macro System

This crate uses declarative macros extensively to reduce boilerplate when wrapping MuJoCo's C arrays.

## Before modifying wrapper files
1. **Read `src/util.rs`** first - it contains all core macro definitions. Understand their syntax before modifying any wrapper file.
2. **Cross-reference with MuJoCo C API docs** (see `coding-conventions.md`) for correct field sizes and types.

## Finding macros
- All macros are defined in `src/util.rs`. Search for `macro_rules!` to find them all.
- Read name and doc comments of each macro to understand its purpose before invoking it.
- Look at existing invocations in the wrapper files (`src/wrappers/`) to see usage patterns.

## Code generation tool
Check if a `../mujoco-rs-utils` directory exists. If it does, it is a CLI tool that can auto-generate macro invocations from MuJoCo's C headers. Run it with `--help` to see available subcommands.

## Key macro reference

### `mj_model_dyn_range!(model, id, nx)`
Resolves the `(start, len)` pair for the contiguous slice owned by element `id`, where `nx`
is a keyword selecting the address-array mapping:

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

Returns `(start_addr, n)` where `n = end_addr - start_addr`. If the address entry for `id` is -1,
returns `(0, 0)`. For the last element, when all subsequent address entries are -1, the macro falls
back to the total-length field as the exclusive end.

### `array_slice_dyn!` - three variants
Creates raw-pointer slices safely (null-pointer and zero-length guarded):

| Variant | Usage | Length formula |
|---|---|---|
| Basic | `array_slice_dyn!(ptr, len)` | `len` |
| `sublen_dep` | `array_slice_dyn!(sublen_dep => ptr, outer, inner)` | `outer * inner` |
| `summed` | `array_slice_dyn!(summed => ptr, len_array)` | sum of `len_array` entries |

#### Mutable field safety -- `(unsafe)` and `(allow_mut = false)`
Fields are mutable by default (both `field()` and `field_mut()` accessors are generated).
Two prefixes restrict mutability, each belonging to a **different macro**:

- `(unsafe)`: used in **`array_slice_dyn!`** -- generates `unsafe fn field_mut()`. The caller must
  use an `unsafe` block and takes responsibility for maintaining C-side invariants.
- `(allow_mut = false)`: used in **`getter_setter!`** -- suppresses `field_mut()` entirely via the
  `eval_or_expand!` helper macro. No mutation path exists.

> **Important**: these prefixes belong to **different macros** and cannot be combined in the same
> invocation. `array_slice_dyn!` does not support `(allow_mut = false)`, and `getter_setter!` does
> not support `(unsafe)`. If full suppression is needed for an array field, the `array_slice_dyn!`
> macro itself would need to be extended.

**Safety criterion**: A field whose VALUES are used by C code as unguarded array INDICES into
other arrays (e.g., `arr[field[i]]` without an upper-bound check against `max_n`) must use
`(unsafe)` at minimum. Pure numeric / float data (forces, positions, velocities, matrices) is
always safe to mutate and needs neither prefix.

**Known `(unsafe)` fields** (in `array_slice_dyn!`) -- mutation is gated behind `unsafe`:
- `contact` in `mj_data.rs`: `mj_sensorAcc()` uses `contact[i].geom[0]` as an index into
  `geom_bodyid[]` with only a `>= 0` guard (no upper-bound check against `ngeom`).
- `flexedge` in `mj_visualization.rs`: `render_gl3.c` uses `flexedge[2*e]` as a vertex index with
  no upper-bound check.
- `geoms` in `mj_visualization.rs`: `render_gl3.c` uses `geom->matid`, `geom->objid`, and
  `geom->dataid` as indices into multiple arrays without upper-bound checks.

**Known safe mutable fields** (must NOT have `(unsafe)` or `(allow_mut = false)`):
- `geomorder` in `mj_visualization.rs`: `mjr_render()` always repopulates `geomorder[0..nt-1]`
  with fresh valid indices before reading them, so user modifications are always overwritten first.

When adding or reviewing `array_slice_dyn!` invocations, trace how C code uses each field's VALUES
and verify whether unguarded index dereferences exist before choosing the appropriate prefix.

### `view_creator!(field, start_ffi_field, data_ptr, type_)`
Generates `fn field(&self) -> &[T]` and the `_mut` variant by reading `(offset, len)` then calling
`data_ptr.add(offset).cast::<T>()`. The cast target type must match `type_` in the info struct.

### `info_method!(Kind, ffi_model, element_type, id, fields...)`
Generates per-element accessor methods like `body()`, `joint()`, etc. on `MjModel` and `MjData`.
Each field entry can use a static stride (e.g. `xpos: 3` produces an `id * 3` offset and a 3-element
slice), a dynamic view through `mj_model_dyn_range!` for variable-length ranges like dof/qpos, or a
zero stride which the accessor returns as `Option::None`.

> **WARNING**: Each element type has TWO macro blocks in the wrapper file:
> 1. `info_method! { ... [field: STRIDE, ...] ... }` -- defines stride **numbers**. This is
>    where bugs live. Located in the first half of the file, inside `impl MjModel`/`impl MjData`.
> 2. `info_with_view! { ... field: &[[Type; N] ...] ... }` -- defines accessor **types** and
>    doc strings. Located in the second half of the file. Uses `[Type; N]` syntax.
>
> When verifying strides, always read the `info_method!` block, NOT the `info_with_view!` block.
> The `info_with_view!` type annotations may be correct even when the `info_method!` stride is wrong.

### `c_str_as_str_method!(field, inner_field, len)`
Getter returns `&str` via `CStr::from_ptr`; setter copies into the fixed `[i8; N]` buffer using
`copy_from_slice`. The buffer is always NUL-terminated.

### `cast_mut_info!(expr, TargetType)`
Wraps `bytemuck::checked::try_cast_mut` to reinterpret a mutable C struct reference.
Only valid when the target type has the same size and alignment as the source.

## Verification checklist when adding/reviewing an `info_method!` field
1. **Read the Rust source first.** Find the actual stride value in the `info_method!` invocation and write it down.
2. Open `mujoco/include/mujoco/mjmodel.h` (or `mjdata.h`) and find the field. Read the dimension
   comment next to it, e.g. `// (nbody x 3)`.
3. **Compare** the Rust stride against the header dimension. They must match exactly.
4. Confirm the Rust element type matches the C type: `mjtNum` is `f64`, `int` is `i32`, `mjtByte` is `u8`.
5. Make sure a unit test exercises the new accessor on a real non-trivial model.
6. If needed, grep `src/` for existing invocations of the same macro to cross-check the generated shape.

> **WARNING**: Do NOT assume the code is correct and only check the header. Always quote the actual
> code value alongside the header value. A common failure mode is reading the header, seeing the
> correct value, and confirming without verifying the source actually uses that value.

## Verification checklist for `array_slice_dyn!` invocations
1. **Read the Rust source first.** Find the length expression (e.g., `ffi().nbody`) and the cast type
   (e.g., `[MjtNum; 3]`). Write both down.
2. Open the C header and find the field. Read its dimension comment (e.g., `// (nbody x 3)`).
3. **Compare**: the length expression must use the FIRST dimension's count field, and the cast type's
   inner size must match the SECOND dimension. E.g., `body_pos` should use `ffi().nbody` and cast to
   `[MjtNum; 3]` because the header says `(nbody x 3)`.
4. Check null/zero guards: the generated code should return `&[]` when the pointer is null or length is 0.

## Verification checklist for unsafe blocks (non-macro)
1. **Bounds checking**: Before any FFI call with an index parameter, verify the Rust code validates
   the index is in range. Check for off-by-one: `< max` not `<= max`, `>= 0` not `> 0`.
2. **Buffer sizes**: For functions that write into caller-supplied buffers (like `mj_contactForce`),
   verify the buffer is large enough. E.g., `contact_force` needs `[MjtNum; 6]`.
3. **Null pointer handling**: All raw pointer dereferences must be guarded. `PointerView` handles this
   via its `Deref` impl, but manually constructed slices need explicit checks.
4. **Return value interpretation**: MuJoCo functions return different sentinel values. Check that
   Rust correctly interprets 0 vs 1, -1 vs null, etc.
5. **Type compatibility**: `as i32`, `as usize`, etc. must not truncate or wrap. Especially check
   that negative C values (like -1 sentinels) are handled before unsigned conversion.
