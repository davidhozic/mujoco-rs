# Macro System

This crate uses declarative macros extensively to reduce boilerplate when wrapping MuJoCo's C arrays.

## Before modifying wrapper files
1. **Read `src/util.rs`** first - it contains all core macro definitions. Understand their syntax before modifying any wrapper file.
2. **Run `/expand-macros`** after any macro-related change to verify generated code.
3. **Cross-reference with MuJoCo C API docs** (see `important-context.md`) for correct field sizes and types.

## Finding macros
- All macros are defined in `src/util.rs`. Search for `macro_rules!` to find them all.
- Read name and doc comments of each macro to understand its purpose before invoking it.
- Look at existing invocations in the wrapper files (`src/wrappers/`) to see usage patterns.

## Code generation tool
Check if a `../mujoco-rs-utils` directory exists. If it does, it is a CLI tool that can auto-generate macro invocations from MuJoCo's C headers. Run it with `--help` to see available subcommands.

## Key macro reference

### `mj_view_indices!(model_ffi, addr_field, njnt, max_n, id)`
Computes the `(start, len)` tuple for the contiguous slice belonging to element `id` within a
MuJoCo address-array-indexed field.

- `addr_field`: address array in `mjModel`, e.g. `jnt_qposadr` (i32 values; -1 means "none").
- `njnt`: total count of items, used as the exclusive upper bound when scanning forward.
- `max_n`: total length of the data array (e.g. `nq`, `nv`), used as the fallback end.
- Returns `(start_addr as usize, n)` where `n = end_addr - start_addr`.
- If `addr_field[id] == -1`, returns `(0, 0)` and the caller should treat the field as `None`.
- For the last element, if all subsequent entries are -1 the macro falls back to `max_n`.

### `mj_model_nx_to_mapping!(model_ffi, nx)` / `mj_model_nx_to_nitem!(model_ffi, nx)`
Convenience lookups that map a total-count field name to the corresponding address/item-count fields:

| `nx` | address array | item count |
|---|---|---|
| `nq` | `jnt_qposadr` | `njnt` |
| `nv` | `jnt_dofadr` | `njnt` |
| `nsensordata` | `sensor_adr` | `nsensor` |
| `ntupledata` | `tuple_adr` | `ntuple` |
| `ntexdata` | `tex_adr` | `ntex` |
| `nnumericdata` | `numeric_adr` | `nnumeric` |
| `nhfielddata` | `hfield_adr` | `nhfield` |
| `na` | `actuator_actadr` | `nu` |

### `array_slice_dyn!` - three variants
Creates raw-pointer slices safely (null-pointer and zero-length guarded):

| Variant | Usage | Length formula |
|---|---|---|
| Basic | `array_slice_dyn!(ptr, len)` | `len` |
| `sublen_dep` | `array_slice_dyn!(sublen_dep => ptr, outer, inner)` | `outer * inner` |
| `summed` | `array_slice_dyn!(summed => ptr, len_array)` | sum of `len_array` entries |

### `view_creator!(field, start_ffi_field, data_ptr, type_)`
Generates `fn field(&self) -> &[T]` and the `_mut` variant by reading `(offset, len)` then calling
`data_ptr.add(offset).cast::<T>()`. The cast target type must match `type_` in the info struct.

### `info_method!(Kind, ffi_model, element_type, id, fields...)`
Generates per-element accessor methods like `body()`, `joint()`, etc. on `MjModel` and `MjData`.
Each field entry can use a static stride (e.g. `xpos: 3` produces an `id * 3` offset and a 3-element
slice), a dynamic view through `mj_view_indices!` for variable-length ranges like dof/qpos, or a
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
6. Optionally, run `/expand-macros` and grep `expanded.rs` for the generated method body (slow).

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
