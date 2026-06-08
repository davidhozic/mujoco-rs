//! Utility types and macros used throughout the crate.
use std::{marker::PhantomData, ops::{Deref, DerefMut}};
use std::sync::{Mutex, MutexGuard};
use std::ffi::c_char;

use crate::mujoco_c::{mj_version, mjVERSION_HEADER};

/// Standard size of temporary error buffers passed to MuJoCo C functions.
/// MuJoCo NUL-terminates within this size, so the effective maximum
/// message length is `ERROR_BUF_LEN - 1` characters.
pub(crate) const ERROR_BUF_LEN: usize = 100;

/// Copies an ASCII `&str` into a fixed-size `c_char` buffer, NUL-terminating and
/// zero-filling the remainder.
///
/// # Panics
/// Panics if `value` is not valid ASCII, contains an interior NUL byte,
/// or if `value` (plus NUL) does not fit in `buf`.
pub(crate) fn write_ascii_to_buf(buf: &mut [c_char], value: &str) {
    assert!(value.is_ascii(), "value must be valid ASCII");
    let c_string = std::ffi::CString::new(value).unwrap();
    let bytes = c_string.into_bytes_with_nul();
    let dest: &mut [u8] = bytemuck::cast_slice_mut(buf);
    dest[..bytes.len()].copy_from_slice(&bytes);
    dest[bytes.len()..].fill(0);
}

/// Returns `Some((start, len))` for item `id` inside a packed data array,
/// or `None` if the item has no data (address entry is negative).
///
/// Each entry in `addr_array` is either the item's start offset in the data array
/// or a negative value (conventionally `-1`) meaning the item has no associated data.
///
/// The length is determined by scanning `addr_array[id + 1..]` for the first
/// non-negative entry (the next item that *does* have data). If no such entry exists,
/// the region extends to the end of the data array (`data_len`).
///
/// # Examples
/// ```ignore
/// if let Some((graph_adr, graph_len)) = optional_sparse_addr_range(
///     model.mesh_graphadr(), mesh_id, model.mesh_graph().len()
/// ) {
///     // copy mesh_graph[graph_adr..graph_adr + graph_len]
/// }
/// ```
///
/// # Panics
/// Panics if `id >= addr_array.len()`.
pub(crate) fn optional_sparse_addr_range<T>(addr_array: &[T], id: usize, data_len: usize) -> Option<(usize, usize)>
where
    T: Into<i64> + Copy,
{
    let adr: i64 = addr_array[id].into();
    if adr < 0 {
        return None;
    }
    let adr = adr as usize;
    let len = addr_array
        .get(id + 1..)
        .unwrap_or(&[])
        .iter()
        .find(|&&next| next.into() != -1)
        .map(|&next| next.into() as usize)
        .unwrap_or(data_len)
        - adr;
    Some((adr, len))
}


/// Sets or clears a bit flag based on a boolean value.
///
/// # Examples
/// ```ignore
/// let mut flags = 0i32;
/// set_flag!(flags, 0x01, true);   // sets bit 0
/// set_flag!(flags, 0x01, false);  // clears bit 0
/// ```
#[doc(hidden)]
#[macro_export]
macro_rules! set_flag {
    ($flags:expr, $mask:expr, $enabled:expr) => {
        if $enabled {
            $flags |= $mask;
        } else {
            $flags &= !$mask;
        }
    };
}


/// Returns the correct address mapping based on the X in nX (nq, nv, nu, ...).
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_dyn_range {
    ($model:expr, $id:expr, nq) => {
        $crate::util::optional_sparse_addr_range($model.jnt_qposadr(), $id, $model.nq() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nv) => {
        $crate::util::optional_sparse_addr_range($model.jnt_dofadr(), $id, $model.nv() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nsensordata) => {
        $crate::util::optional_sparse_addr_range($model.sensor_adr(), $id, $model.nsensordata() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, ntupledata) => {
        $crate::util::optional_sparse_addr_range($model.tuple_adr(), $id, $model.ntupledata() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, ntexdata) => {
        $crate::util::optional_sparse_addr_range($model.tex_adr(), $id, $model.ntexdata() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nnumericdata) => {
        $crate::util::optional_sparse_addr_range($model.numeric_adr(), $id, $model.nnumericdata() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nhfielddata) => {
        $crate::util::optional_sparse_addr_range($model.hfield_adr(), $id, $model.nhfielddata() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, na) => {
        $crate::util::optional_sparse_addr_range($model.actuator_actadr(), $id, $model.na() as usize).unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nJten) => {
        $crate::util::optional_sparse_addr_range($model.ten_j_rowadr(), $id, $model.n_jten() as usize).unwrap_or((0, 0))
    };
}

/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerViewMut<'d, T> {
    ptr: *mut T,
    len: usize,
    phantom: PhantomData<&'d mut ()>
}

impl<'d, T> PointerViewMut<'d, T> {
    pub(crate) const fn new(ptr: *mut T, len: usize, phantom: PhantomData<&'d mut ()>) -> Self {
        Self {ptr, len, phantom}
    }
}

/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerViewMut<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}

impl<T> Eq for PointerViewMut<'_, T> {}

impl<T> Deref for PointerViewMut<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        // SAFETY: ptr is non-null (checked above), properly aligned, and points to
        // self.len initialized elements owned by the parent wrapper for lifetime 'd.
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

impl<T> DerefMut for PointerViewMut<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        if self.ptr.is_null() {
            return &mut [];
        }
        // SAFETY: ptr is non-null (checked above), properly aligned, points to self.len
        // initialized elements, and &mut self guarantees exclusive access.
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

/// Provides a read-only view to a C array with explicit unsafe mutable access.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// Mutable access is only available via [`PointerViewUnsafeMut::as_mut_slice`],
/// where the caller must uphold Rust aliasing and validity guarantees.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerViewUnsafeMut<'d, T> {
    ptr: *mut T,
    len: usize,
    phantom: PhantomData<&'d mut ()>
}

impl<'d, T> PointerViewUnsafeMut<'d, T> {
    pub(crate) const fn new(ptr: *mut T, len: usize, phantom: PhantomData<&'d mut ()>) -> Self {
        Self { ptr, len, phantom }
    }

    /// Returns a mutable slice over the underlying data.
    ///
    /// # Safety
    /// Caller must ensure that:
    /// - `self.ptr` points to `self.len` properly aligned and initialized `T` values (or is null with `len == 0`);
    /// - no other references (shared or mutable) to overlapping memory are alive while the returned slice is used;
    /// - written values preserve Rust type validity and MuJoCo invariants.
    pub unsafe fn as_mut_slice(&mut self) -> &mut [T] {
        if self.ptr.is_null() {
            return &mut [];
        }
        // SAFETY: caller upholds aliasing and validity guarantees (documented above).
        // ptr is non-null (checked above) and points to self.len elements.
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerViewUnsafeMut<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}

impl<T> Eq for PointerViewUnsafeMut<'_, T> {}

impl<T> Deref for PointerViewUnsafeMut<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        // SAFETY: ptr is non-null (checked above), properly aligned, and points to
        // self.len initialized elements owned by the parent wrapper for lifetime 'd.
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerView<'d, T> {
    ptr: *const T,
    len: usize,
    phantom: PhantomData<&'d ()>
}

impl<'d, T> PointerView<'d, T> {
    pub(crate) const fn new(ptr: *const T, len: usize, phantom: PhantomData<&'d ()>) -> Self {
        Self {ptr, len, phantom}
    }
}

/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerView<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}

impl<T> Eq for PointerView<'_, T> {}

impl<T> Deref for PointerView<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        // SAFETY: ptr is non-null (checked above), properly aligned, and points to
        // self.len initialized elements owned by the parent wrapper for lifetime 'd.
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

/***************************/
//  Evaluation helper macro
/***************************/
/// When @eval is given false, ignore the given contents.
/// In other cases, expand the given contents.
#[macro_export]
#[doc(hidden)]
macro_rules! eval_or_expand {
    (@eval $(true)? { $($data:tt)* } ) => { $($data)* };
    (@eval false { $($data:tt)* } ) => {};
}

/**************************************************************************************************/
// View creation for MjData and MjModel
/**************************************************************************************************/

/// Constructs a view struct by mapping fields to their corresponding locations in `$data`.
///
/// - `$field` list uses `$ptr_view` (read-write in `ViewMut`, read-only in `View`).
/// - `$field_ro` list uses `$ptr_view_ro` (`PointerViewUnsafeMut` in `ViewMut`, `PointerView` in `View`).
/// - `$opt_field` list uses `$ptr_view`, wrapped in `Option`.
///
/// # Safety
/// Caller must ensure the data pointers remain valid for the lifetime of the view.
#[macro_export]
#[doc(hidden)]
macro_rules! view_creator {
    (
        $self:expr, $view:ident, $data:expr,
        [$($([$prefix_field:ident])? $field:ident : $type_:ty $([$force:ident])?),*],
        [$($([$prefix_field_ro:ident])? $field_ro:ident : $type_ro:ty $([$force_ro:ident])?),*],
        [$($([$prefix_opt_field:ident])? $opt_field:ident : $type_opt:ty $([$force_opt:ident])?),*],
        $ptr_view:expr,
        $ptr_view_ro:expr
    ) => {
        paste::paste! {
            unsafe {
                $view {
                    $(
                        $field: $ptr_view(
                            $crate::maybe_force_cast!($data.[<$($prefix_field)? $field>].add($self.$field.0), $type_ $(, $force)?),
                            $self.$field.1,
                            std::marker::PhantomData
                        ),
                    )*
                    $(
                        $field_ro: $ptr_view_ro(
                            $crate::maybe_force_cast!($data.[<$($prefix_field_ro)? $field_ro>].add($self.$field_ro.0), $type_ro $(, $force_ro)?),
                            $self.$field_ro.1,
                            std::marker::PhantomData
                        ),
                    )*
                    $(
                        $opt_field: if $self.$opt_field.1 > 0 {
                            Some($ptr_view(
                                $crate::maybe_force_cast!($data.[<$($prefix_opt_field)? $opt_field>].add($self.$opt_field.0), $type_opt $(, $force_opt)?),
                                $self.$opt_field.1,
                                std::marker::PhantomData
                            ))
                        } else {
                            None
                        },
                    )*
                }
            }
        }
    };
}


/// Generates a lookup method `$type_(&self, name: &str) -> Option<Mj{Type}{InfoType}Info>` on
/// a wrapper.
///
/// The returned `Info` struct stores the name, id, and index ranges needed to
/// create views into the corresponding `MjData` or `MjModel` arrays.
///
/// # Entry formats
///
/// - **Fixed stride**: `attr: N`: index range is `(id * N, N)`.
/// - **FFI stride** (with optional multiplier): `attr: ffi_field (* k)`: stride taken from
///   `model.ffi_field`, optionally scaled by `k`.
/// - **Dynamic range**: `attr: nXXX (* k)`: start and length resolved via [`mj_model_dyn_range!`],
///   where `nXXX` is the major-dimension field (e.g. `nhfielddata`, `ntexdata`).
///   The optional `* k` is a stride multiplier: each logical unit occupies `k` flat elements,
///   so both the start offset and the length are scaled by `k`. Use this when the target array
///   stores `k` flat values per logical unit (e.g. `dof_dampingpoly (nv × mjNPOLY)` viewed as
///   a flat `MjtNum` slice: offset = `dof_start * mjNPOLY`, length = `n_dofs * mjNPOLY`).
#[doc(hidden)]
#[macro_export]
macro_rules! info_method {
    (
        $info_type:ident, $([$model:ident],)?
        $type_:ident,
        [$($attr:ident: $len:expr),*],
        [$($attr_ffi:ident: $len_ffi:ident $(* $multiplier:expr)?),*],
        [$($attr_dyn:ident: $ffi_len_dyn:ident $(* $offset_mult:expr)?),*]
    ) => {
        paste::paste! {
            #[doc = concat!(
                "Returns a [`", stringify!([<Mj $type_:camel $info_type Info>]), "`] for the named ", stringify!($type_), ", ",
                "containing the indices required to create views into [`Mj", stringify!($info_type), "`] arrays.\n\n",
                "Call [`view`](", stringify!([<Mj $type_:camel $info_type Info>]), "::view) or ",
                "[`try_view`](", stringify!([<Mj $type_:camel $info_type Info>]), "::try_view) on the result to obtain the actual view.\n\n",
                "# Panics\n",
                "Panics if `name` contains a `\\0` byte."
            )]
            #[allow(non_snake_case)]
            pub fn $type_(&self, name: &str) -> Option<[<Mj $type_:camel $info_type Info>]> {
                let model_ref = self$(.$model())?;
                let id = model_ref.name_to_id(MjtObj::[<mjOBJ_ $type_:upper>], name)?;
                let model_ffi = model_ref.ffi();

                let id = id as usize;
                $(
                    let $attr = (id * $len, $len);
                )*
                $(
                    let $attr_ffi = (
                        id * model_ffi.$len_ffi as usize $( * $multiplier)*,
                        model_ffi.$len_ffi as usize $( * $multiplier)*,
                    );
                )*
                $(
                    let $attr_dyn = {
                        let (dyn_start, dyn_len) = $crate::mj_model_dyn_range!(model_ref, id, $ffi_len_dyn);
                        (dyn_start $(* $offset_mult)?, dyn_len $(* $offset_mult)?)
                    };
                )*

                let model_signature = model_ffi.signature;
                Some([<Mj $type_:camel $info_type Info>] { name: name.to_string(), id, model_signature, $($attr,)* $($attr_ffi,)* $($attr_dyn),* })
            }
        }
    }
}


/// Generates `Info`, `ViewMut`, and `View` types for a named MuJoCo object, along with
/// `view`, `try_view`, `view_mut`, and `try_view_mut` methods on the `Info` type.
///
/// # Field lists
///
/// - **`[rw fields]`**: read-write: `PointerViewMut` in `ViewMut`, `PointerView` in `View`.
/// - **`[ro fields]`**: read as `PointerViewUnsafeMut` in `ViewMut` (unsafe to mutate), `PointerView` in `View`.
/// - **`[opt fields]`**: optional read-write: `Option<PointerViewMut>` / `Option<PointerView>`.
///
/// # Field entry syntax
///
/// ```text
/// [prefix_] field_name : ElementType [force]
/// ```
///
/// - `[prefix_]`: optional prefix prepended to the FFI field name (e.g. `[actuator_]`).
/// - `[force]`: emit a forced pointer cast via [`maybe_force_cast!`] (needed when the Rust
///   element type differs from the C array element type, e.g. `f64` -> `[f64; 3]`).
#[doc(hidden)]
#[macro_export]
macro_rules! info_with_view {
    (
        $info_type:ident, $name:ident,
        [$($([$prefix_attr:ident])? $attr:ident: $type_:ty $([$force:ident])?),*],
        [$($([$prefix_attr_ro:ident])? $attr_ro:ident: $type_ro:ty $([$force_ro:ident])?),*],
        [$($([$prefix_opt_attr:ident])? $opt_attr:ident: $type_opt:ty $([$force_opt:ident])?),*]
        $(,$generics:ty: $bound:ty)?
    ) => {
        paste::paste! {
            #[doc = "Index ranges required to create views into [`Mj" $info_type "`] arrays for a " $name "."]
            #[allow(non_snake_case)]
            #[derive(Debug, Clone)]
            pub struct [<Mj $name:camel $info_type Info>] {
                /// Name of the element.
                pub name: String,
                /// Index of the element.
                pub id: usize,
                model_signature: u64,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $attr_ro: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel $info_type Info>] {
                /// Returns the model signature this `Info` was created from.
                pub fn model_signature(&self) -> u64 {
                    self.model_signature
                }

                #[doc = concat!(
                    "Returns a mutable view into the [`Mj", stringify!($info_type), "`] arrays for this ", stringify!($name), ".\n\n",
                    "Fields listed as read-only use [`PointerViewUnsafeMut`](crate::util::PointerViewUnsafeMut): ",
                    "read is safe, mutation requires [`as_mut_slice`](crate::util::PointerViewUnsafeMut::as_mut_slice) and `unsafe`.\n\n",
                    "# Errors\n",
                    "Returns [`SignatureMismatch`](", stringify!([<Mj $info_type Error>]), "::SignatureMismatch) if `",
                    stringify!($info_type), "` was built from a different model than this `Info`."
                )]
                pub fn try_view_mut<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p mut [<Mj $info_type>]$(<$generics>)?) -> Result<[<Mj $name:camel $info_type ViewMut>]<'p>, $crate::error::[<Mj $info_type Error>]> {
                    let destination_signature = [<$info_type:lower>].signature();
                    if self.model_signature != destination_signature {
                        return Err($crate::error::[<Mj $info_type Error>]::SignatureMismatch {
                            source: self.model_signature,
                            destination: destination_signature,
                        });
                    }
                    Ok(view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(),
                        [$($([$prefix_attr])? $attr : $type_ $([$force])?),*],
                        [$($([$prefix_attr_ro])? $attr_ro : $type_ro $([$force_ro])?),*],
                        [$($([$prefix_opt_attr])? $opt_attr : $type_opt $([$force_opt])?),*],
                        $crate::util::PointerViewMut::new,
                        $crate::util::PointerViewUnsafeMut::new))
                }

                #[doc = concat!(
                    "Returns a mutable view into the [`Mj", stringify!($info_type), "`] arrays for this ", stringify!($name), ".\n\n",
                    "Fields listed as read-only use [`PointerViewUnsafeMut`](crate::util::PointerViewUnsafeMut): ",
                    "read is safe, mutation requires [`as_mut_slice`](crate::util::PointerViewUnsafeMut::as_mut_slice) and `unsafe`.\n\n",
                    "# Panics\n",
                    "Panics if `", stringify!($info_type), "` was built from a different model than this `Info`. ",
                    "Use [`try_view_mut`](Self::try_view_mut) to handle this as a `Result`."
                )]
                pub fn view_mut<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p mut [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type ViewMut>]<'p> {
                    self.try_view_mut([<$info_type:lower>]).unwrap_or_else(|_| panic!("model signature mismatch"))
                }

                #[doc = concat!(
                    "Returns an immutable view into the [`Mj", stringify!($info_type), "`] arrays for this ", stringify!($name), ".\n\n",
                    "# Errors\n",
                    "Returns [`SignatureMismatch`](", stringify!([<Mj $info_type Error>]), "::SignatureMismatch) if `",
                    stringify!($info_type), "` was built from a different model than this `Info`."
                )]
                pub fn try_view<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p [<Mj $info_type>]$(<$generics>)?) -> Result<[<Mj $name:camel $info_type View>]<'p>, $crate::error::[<Mj $info_type Error>]> {
                    let destination_signature = [<$info_type:lower>].signature();
                    if self.model_signature != destination_signature {
                        return Err($crate::error::[<Mj $info_type Error>]::SignatureMismatch {
                            source: self.model_signature,
                            destination: destination_signature,
                        });
                    }
                    Ok(view_creator!(self, [<Mj $name:camel $info_type View>], [<$info_type:lower>].ffi(),
                        [$($([$prefix_attr])? $attr : $type_ $([$force])?),*],
                        [$($([$prefix_attr_ro])? $attr_ro : $type_ro $([$force_ro])?),*],
                        [$($([$prefix_opt_attr])? $opt_attr : $type_opt $([$force_opt])?),*],
                        $crate::util::PointerView::new,
                        $crate::util::PointerView::new))
                }

                #[doc = concat!(
                    "Returns an immutable view into the [`Mj", stringify!($info_type), "`] arrays for this ", stringify!($name), ".\n\n",
                    "# Panics\n",
                    "Panics if `", stringify!($info_type), "` was built from a different model than this `Info`. ",
                    "Use [`try_view`](Self::try_view) to handle this as a `Result`."
                )]
                pub fn view<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type View>]<'p> {
                    self.try_view([<$info_type:lower>]).unwrap_or_else(|_| panic!("model signature mismatch"))
                }
            }

            #[doc = "Mutable view into [`Mj" $info_type "`] arrays for a " $name ".\n\n"
                    "Read-write fields are [`PointerViewMut`](crate::util::PointerViewMut); "
                    "read-only fields are [`PointerViewUnsafeMut`](crate::util::PointerViewUnsafeMut), "
                    "which require [`as_mut_slice`](crate::util::PointerViewUnsafeMut::as_mut_slice) and explicit `unsafe` to mutate."]
            #[allow(non_snake_case)]
            #[derive(Debug)]
            pub struct [<Mj $name:camel $info_type ViewMut>]<'d> {
                $(
                    #[doc = concat!("Mutable view of `", stringify!($attr), "`.")]
                    pub $attr: $crate::util::PointerViewMut<'d, $type_>,
                )*
                $(
                    #[doc = concat!("Read-only view of `", stringify!($attr_ro), "`. Requires `unsafe` for mutation.")]
                    pub $attr_ro: $crate::util::PointerViewUnsafeMut<'d, $type_ro>,
                )*
                $(
                    #[doc = concat!("Optional mutable view of `", stringify!($opt_attr), "`.")]
                    pub $opt_attr: Option<$crate::util::PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel $info_type ViewMut>]<'_> {
                /// Zeroes all read-write fields. Read-only fields are left unchanged.
                pub fn zero(&mut self) {
                    $(
                        self.$attr.fill(bytemuck::Zeroable::zeroed());
                    )*
                    $(
                        if let Some(x) = &mut self.$opt_attr {
                            x.fill(bytemuck::Zeroable::zeroed());
                        }
                    )*
                }
            }

            #[doc = "Immutable view into [`Mj" $info_type "`] arrays for a " $name "."]
            #[allow(non_snake_case)]
            #[derive(Debug)]
            pub struct [<Mj $name:camel $info_type View>]<'d> {
                $(
                    #[doc = concat!("View of `", stringify!($attr), "`.")]
                    pub $attr: $crate::util::PointerView<'d, $type_>,
                )*
                $(
                    #[doc = concat!("View of `", stringify!($attr_ro), "`.")]
                    pub $attr_ro: $crate::util::PointerView<'d, $type_ro>,
                )*
                $(
                    #[doc = concat!("Optional view of `", stringify!($opt_attr), "`.")]
                    pub $opt_attr: Option<$crate::util::PointerView<'d, $type_opt>>,
                )*
            }
        }
    };
}


/// Generates getters, setters, and builder methods for struct fields.
///
/// ## Optional value constraints
///
/// Every arm that writes a value (`set`, `with`, `[&] with`, their `force!` variants, and the
/// `get, set` / `with, get, set` / `with, get` aggregates) accepts an **optional per-field check**
/// for fields whose value can be invalid. Bool arms have no check (a bool is always in range).
///
/// - On a `set`-style field: `name: Type { check } => ErrType; "comment"` makes `set_name` return
///   `Result<(), ErrType>`; `check` (a `Fn(Type) -> Result<(), ErrType>`) runs first and, on `Err`,
///   the field is left unchanged.
/// - On a `with`-style field: `name: Type { check }; "comment"` makes the builder `with_name`
///   call `check` and **panic** (`expect`) on `Err`, since a builder must keep returning `Self`.
/// - In the combined `with, get, set` aggregate, supply `{ check } => ErrType`; the check is wired
///   into both the fallible `set_name` and the panicking `with_name`.
///
/// Omitting the check yields the plain infallible setter/builder.
#[doc(hidden)]
#[macro_export]
macro_rules! getter_setter {
    (get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: bool; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Check ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> bool {
                (self$(.$ffi())?.$name as i32) != 0
            }
        )*
    }};

    (get, [$($([$ffi:ident $(,$ffi_mut:ident)?])? $((allow_mut = $cfg_mut:literal))? $name:ident $(+ $symbol:tt)?: & $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return an immutable reference to ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> &$type {
                &self$(.$ffi())?.$name
            }

            $crate::eval_or_expand! {
                @eval $($cfg_mut)? {
                    #[doc = concat!("Return a mutable reference to ", $comment)]
                    pub fn [<$name:camel:snake _mut>](&mut self) -> &mut $type {
                        #[allow(unused_unsafe)]
                        unsafe { &mut self$(.$($ffi_mut())?)?.$name }
                    }
                }
            }
        )*
    }};

    (get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> $type {
                self$(.$ffi())?.$name.into()
            }
        )*
    }};

    (set, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr);* $(;)?]) => {
        paste::paste!{
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) $(-> Result<(), $err>)? {
                    $(($check)(value)?;)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    $(Ok::<(), $err>(()))?
                }
            )*
        }
    };

    /* Enum conversion */
    (force!, get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)? : $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> $type {
                unsafe { $crate::util::force_cast(self$(.$ffi())?.$name) }
            }
        )*
    }};

    (force!, set, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr);* $(;)?]) => {
        paste::paste!{
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) $(-> Result<(), $err>)? {
                    $(($check)(value)?;)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                    $(Ok::<(), $err>(()))?
                }
            )*
        }
    };

    /* Builder pattern */
    (force!, with, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})?; $comment:expr);* $(;)?]) => {
        paste::paste!{
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    $(($check)(value).expect("invalid builder argument");)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                    self
                }
            )*
        }
    };

    (force!, [&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})?; $comment:expr);* $(;)?]) => {
        paste::paste!{
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    $(($check)(value).expect("invalid builder argument");)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                    self
                }
            )*
        }
    };

    (with, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})?; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    $($check(value).expect("invalid builder argument");)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };

    ([&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty $({$check:expr})?; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    $($check(value).expect("invalid builder argument");)?
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };
    
    /* Handling of optional arguments */
    /* Enum pass */
    (force!, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [ $($([$ffi_mut])? $name : $type $({$check})? $(=> $err)?; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type $({$check})? $(=> $err)?; $comment );* ]);
    };

    /* Builder pattern */
    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)?: $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type $({$check})? $(=> $err)?; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : $type $({$check})?; $comment );* ]);
    };

    (force!, $([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty $({$check:expr})? $(=> $err:ty)?; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [$($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [$($([$ffi_mut])? $name : $type $({$check})? $(=> $err)?; $comment );* ]);
        $crate::getter_setter!(force!, $([$token])? with, [$($([$ffi_mut])? $name : $type $({$check})?; $comment );* ]);
    };

    ($([$token:tt])? with, get, [$( $([$ffi: ident, $ffi_mut:ident])? $((allow_mut = $allow_mut:literal))? $name:ident $(+ $symbol:tt)? : & $type:ty $({$check:expr})? ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi, $ffi_mut])? $((allow_mut = $allow_mut))? $name $(+ $symbol)? : & $type ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $( $([$ffi_mut])? $name : $type $({$check})? ; $comment );* ]);
    };
}


#[doc(hidden)]
#[macro_export]
/// Constructs builder methods.
macro_rules! builder_setters {
    ($($name:ident: $type:ty $(where $generic_type:ident: $generic:path)?; $comment:expr);* $(;)?) => {
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn $name$(<$generic_type: $generic>)?(mut self, value: $type) -> Self {
                self.$name = value.into();
                self
            }
        )*
    };
}

/// Helper macro for conditionally generating `# Safety` docs on mutable array slice methods.
/// When `unsafe` is passed (method is unsafe), includes the safety section.
/// When no `unsafe` is passed (method is safe), only generates the basic doc.
#[doc(hidden)]
#[macro_export]
macro_rules! array_mut_doc {
    (unsafe, $doc:literal) => {
        concat!("Mutable slice of the ", $doc, " array.\n\n# Safety\n\nDirect mutation of this array bypasses MuJoCo's internal consistency checks. The caller must ensure that all values written remain valid for MuJoCo's internal state.")
    };
    ($doc:literal) => {
        concat!("Mutable slice of the ", $doc, " array.")
    };
}

/// A macro for creating a slice over a raw array of dynamic size (given by some other variable in $len_accessor).
/// Syntax: attribute: <optional pre-transformations (e.g., `as_ptr as_mut_ptr`)> &[datatype; documentation string;
/// code to access the length attribute, appearing after `self.`]
/// Syntax for arrays whose size is a sum from some length array:
///     summed {
///         ...
///         attribute: &[datatype; documentation; [
///                 size multiplier;
///                 (code to access the length array, appearing after self);
///                 (code to access the length array's length, appearing after self)
///             ]
///         ],
///         ...
///     }
///
#[doc(hidden)]
#[macro_export]
macro_rules! array_slice_dyn {
    // Arrays that are of scalar variable size
    ($($((mut = $unsafe_mut:ident))? $name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[$type:ty $([$force:ident])?; $doc:literal; $($len_accessor:tt)*]),*) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize;
                    let ptr = $crate::maybe_force_cast!(self.ffi().$name$(.$as_ptr())?, $type $(, $force)?);
                    if ptr.is_null() || length == 0 {
                        return &[];
                    }
                    unsafe { std::slice::from_raw_parts(ptr, length) }
                }

                #[doc = $crate::array_mut_doc!($($unsafe_mut,)? $doc)]
                pub $($unsafe_mut)? fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                    let length = self.$($len_accessor)* as usize;
                    let ptr = $crate::maybe_force_cast!(unsafe { self.ffi_mut().$name$(.$as_mut_ptr())? }, $type $(, $force)?);
                    if ptr.is_null() || length == 0 {
                        return &mut [];
                    }
                    unsafe { std::slice::from_raw_parts_mut(ptr, length) }
                }
            )*
        }
    };

    // Arrays that are of summed variable size
    (summed { $( $(($unsafe_mut:ident))? $name:ident: &[$type:ty; $doc:literal; [$multiplier:literal ; ($($len_array:tt)*) ; ($($len_array_length:tt)*)]]),* }) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[[$type; $multiplier]] {
                    let length_array_length = self.$($len_array_length)* as usize;
                    let data_ptr = self.ffi().$name;
                    let length_ptr = self.$($len_array)*;
                    if data_ptr.is_null() || length_ptr.is_null() || length_array_length == 0 {
                        return &[];
                    }

                    let length = unsafe { std::slice::from_raw_parts(
                        length_ptr,
                        length_array_length
                    ).iter().map(|&x| x as u32).sum::<u32>() as usize };

                    if length == 0 {
                        return &[];
                    }

                    unsafe { std::slice::from_raw_parts($crate::maybe_force_cast!(data_ptr, [$type; $multiplier], force), length) }
                }
                
                #[doc = $crate::array_mut_doc!($($unsafe_mut,)? $doc)]
                pub $($unsafe_mut)? fn [<$name:camel:snake _mut>](&mut self) -> &mut [[$type; $multiplier]] {
                    let length_array_length = self.$($len_array_length)* as usize;
                    let data_ptr = unsafe { self.ffi_mut().$name };
                    let length_ptr = self.$($len_array)*;
                    if data_ptr.is_null() || length_ptr.is_null() || length_array_length == 0 {
                        return &mut [];
                    }

                    let length = unsafe { std::slice::from_raw_parts(
                        length_ptr,
                        length_array_length
                    ).iter().map(|&x| x as u32).sum::<u32>() as usize };

                    if length == 0 {
                        return &mut [];
                    }

                    unsafe { std::slice::from_raw_parts_mut($crate::maybe_force_cast!(data_ptr, [$type; $multiplier], force), length) }
                }
            )*
        }
    };

    // Arrays whose second dimension is dependent on some variable
    (sublen_dep {$( $(($unsafe_mut:ident))? $name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[[$type:ty; $($inner_len_accessor:tt)*] $([$force:ident])?; $doc:literal; $($len_accessor:tt)*]),*}) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize * (self.$($inner_len_accessor)*) as usize;
                    let ptr = $crate::maybe_force_cast!(self.ffi().$name$(.$as_ptr())?, $type $(, $force)?);
                    if ptr.is_null() || length == 0 {
                        return &[];
                    }

                    unsafe { std::slice::from_raw_parts(ptr, length) }
                }

                #[doc = $crate::array_mut_doc!($($unsafe_mut,)? $doc)]
                pub $($unsafe_mut)? fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                    let length = self.$($len_accessor)* as usize * (self.$($inner_len_accessor)*) as usize;
                    let ptr = $crate::maybe_force_cast!(unsafe { self.ffi_mut().$name$(.$as_mut_ptr())? }, $type $(, $force)?);
                    if ptr.is_null() || length == 0 {
                        return &mut [];
                    }
                    unsafe { std::slice::from_raw_parts_mut(ptr, length) }
                }
            )*
        }
    };
}

/// Generates getter and setter methods for converting between Rust's &str type and C's char arrays.
///
/// # Safety
/// The generated getters blindly interpret a `char` array as a C string; the
/// array must be NUL-terminated and contain valid UTF-8. Setters validate
/// ASCII encoding and will **panic** if the value exceeds the buffer length.
/// The macro works by first specifying the methods to create (get = getter, set = setter) --- c_str_as_str_method {get, set, {...}} ---
/// and then providing the rest of the parameters.
/// 
/// The rest of the parameters are recursive and are as follows:
/// - ffi (optional): name of the method that returns some lower-level struct,
///                   which contains the actual attributes we want to read;
/// - name: the attribute name;
/// - sub_index_name: sub_index_type (optional): creates an additional parameter which indexes the `name` array
///                   in order to get a sub-array
///                   (e.g., `name` could be `[[i8; 100]; 10]` and we wish to get `[i8; 100]`);
/// - comment: the documentation comment to insert as the methods documentation.
/// 
#[doc(hidden)]
#[macro_export]
macro_rules! c_str_as_str_method {
    (get {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {
        $(
            #[doc = concat!("Returns ", $comment, "\n\n# Panics", "\nPanics if the buffer has no NUL terminator or if the resulting string contains invalid UTF-8.")]
            pub fn $name(&self $(, $sub_index_name: $sub_index_type)? ) -> &str {
                let bytes: &[u8] = bytemuck::cast_slice(&self$(.$ffi())?.$name$([$sub_index_name])?[..]);
                std::ffi::CStr::from_bytes_until_nul(bytes)
                    .expect("no NUL terminator in C string buffer")
                    .to_str().unwrap()
            }
        )*
    };

    (set {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {paste::paste!{
        $(
            #[doc = concat!("Sets ", $comment, "\n\n# Panics", "\nPanics when `", stringify!($name), "` contains invalid ASCII, an interior NUL byte, or is too long.")]
            pub fn [<set_ $name>](&mut self, $($sub_index_name: $sub_index_type,)? $name: &str) {
                $crate::util::write_ascii_to_buf(
                    &mut self$(.$ffi())?.$name$([$sub_index_name])?,
                    $name,
                );
            }
        )*
    }};

    (with {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {paste::paste!{
        $(
            #[doc = concat!("Builder method for setting ", $comment, "\n\n# Panics", "\nPanics when `", stringify!($name), "` contains invalid ASCII, an interior NUL byte, or is too long.")]
            pub fn [<with_ $name>](mut self, $($sub_index_name: $sub_index_type,)? $name: &str) -> Self {
                $crate::util::write_ascii_to_buf(
                    &mut self$(.$ffi())?.$name$([$sub_index_name])?,
                    $name,
                );
                self
            }
        )*
    }};

    // Mixed patterns
    (with, get, set {$($other:tt)*}) => {
        $crate::c_str_as_str_method!(get {$($other)*});
        $crate::c_str_as_str_method!(set {$($other)*});
        $crate::c_str_as_str_method!(with {$($other)*});
    };

    (get, set {$($other:tt)*}) => {
        $crate::c_str_as_str_method!(get {$($other)*});
        $crate::c_str_as_str_method!(set {$($other)*});
    };

    (with, set {$($other:tt)*}) => {
        $crate::c_str_as_str_method!(set {$($other)*});
        $crate::c_str_as_str_method!(with {$($other)*});
    };

    (with, get {$($other:tt)*}) => {
        $crate::c_str_as_str_method!(get {$($other)*});
        $crate::c_str_as_str_method!(with {$($other)*});
    };
}

/// assert_eq!, but with tolerance for floating point rounding.
#[doc(hidden)]
#[macro_export]
macro_rules! assert_relative_eq {
    ($a:expr, $b:expr, epsilon = $eps:expr) => {{
        let (a, b, eps) = ($a as f64, $b as f64, $eps as f64);
        assert!((a - b).abs() <= eps, "left={:?} right={:?} eps={:?}", a, b, eps);
    }};
}


/// Tries to cast $value into requested type.
/// # Panics
/// Panics if the cast fails.
#[doc(hidden)]
#[macro_export]
macro_rules! cast_mut_info {
    ($value:expr $(, $debug_expr:expr)?) => {
        {
            match bytemuck::checked::try_cast_mut($value) {
                Ok(v) => v,
                Err(e) => {
                    let evaluated = format!("{:?}", $value);
                    #[allow(unused)]
                    let mut debug_info = String::new();
                    $(
                        debug_info = format!(" (debug info: '{} = {}')", stringify!($debug_expr), $debug_expr);
                    )?

                    panic!(
                        "failed to cast expression '{}', which evaluates to '{}' into requested type (error: {})\
                         {debug_info} --- \
                         most likely you have a bug in your program.",
                        stringify!($value), evaluated, e
                    );
                }
            }
        }
    };
}

/// Asserts that the MuJoCo version used matches
/// the one MuJoCo-rs was compiled with.
///
/// # Panics
/// Panics if the linked MuJoCo library version does not match
/// the version MuJoCo-rs was compiled against.
pub fn assert_mujoco_version() {
    // SAFETY: mj_version() is a pure query function with no side effects; safe to call at any
    // time after the library is loaded.
    let linked_version = unsafe { mj_version() as u32 };
    let mujoco_rs_version_string = option_env!("CARGO_PKG_VERSION").unwrap_or_else(|| "unknown+mj-unknown");
    assert_eq!(
        linked_version, mjVERSION_HEADER,
        "linked MuJoCo version value ({linked_version}) does not match expected version value ({mjVERSION_HEADER}), \
        with which MuJoCo-rs {mujoco_rs_version_string} FFI bindings were generated.",
    );
}


/// Forcefully casts a value of type `T` to type `U`.
/// Performs compile-time size and alignment checks, but does **not** guarantee
/// that the bit patterns are compatible.
///
/// # Safety
/// The bit pattern of `val` must be a valid representation for type `U`;
/// otherwise the behavior is undefined.
#[inline(always)]
pub unsafe fn force_cast<T, U>(val: T) -> U {
    const {
        // The underlying type should be the same in representation.
        assert!(std::mem::size_of::<T>() == std::mem::size_of::<U>());
        assert!(std::mem::align_of::<T>() == std::mem::align_of::<U>());
    }
    #[repr(C)]
    union Transmuter<T, U> {
        from: std::mem::ManuallyDrop<T>,
        to: std::mem::ManuallyDrop<U>,
    }
    // SAFETY: size and alignment equality is verified by the const assertions above; the caller
    // guarantees bit-pattern validity (see # Safety).
    unsafe { std::mem::ManuallyDrop::into_inner(Transmuter { from: std::mem::ManuallyDrop::new(val) }.to) }
}


/// Asserts at compile time that casting from `Src` to `Dst` is
/// size-and-alignment compatible.
///
/// The target element size must be a multiple of the source element size
/// (covers both same-size type conversions and array-grouping casts like
/// `*const f64` to `*const [f64; 3]`), and the source and target alignments
/// must be equal.
///
/// The pointer argument is only used for type inference of `Src`; it is
/// never dereferenced.
#[inline(always)]
pub const fn assert_ptr_cast_valid<Src, Dst>(_ptr: *const Src) {
    const {
        assert!(std::mem::size_of::<Dst>().is_multiple_of(std::mem::size_of::<Src>()),
            "ptr cast: target size must be a multiple of source size");

        // The underlying type should have the same alignment. This is for converting
        // between e.g. *f64 to *[f64; 3].
        assert!(std::mem::align_of::<Src>() == std::mem::align_of::<Dst>(),
            "ptr cast: source alignment must be == target alignment");
    }
}

/// Conditionally casts a raw pointer to `$type` with compile-time
/// size and alignment checks.  When the `force` token is absent the
/// pointer is returned as-is.
#[doc(hidden)]
#[macro_export]
macro_rules! maybe_force_cast {
    ($ptr:expr, $type:ty) => { $ptr };
    ($ptr:expr, $type:ty, force) => {{
        let ptr = $ptr;
        $crate::util::assert_ptr_cast_valid::<_, $type>(ptr as *const _);
        ptr.cast::<$type>()
    }};
}


/* Utility traits */
/// Locks a synchronization primitive and resets its poison status.
/// This is useful on locations that don't need any special handling
/// after a thread panicked while holding a mutex lock.
pub trait LockUnpoison<T> {
    /// Locks the synchronization primitive, resetting its poison status if necessary.
    fn lock_unpoison(&self) -> MutexGuard<'_, T>;
}

/// Implements automatic unpoisoning on the [`Mutex`].
impl<T> LockUnpoison<T> for Mutex<T> {
    fn lock_unpoison(&self) -> MutexGuard<'_, T> {
        match self.lock() {
            Ok(lock) => lock,
            Err(e) => {
                self.clear_poison();
                e.into_inner()
            }
        }
    }
}

#[cfg(feature = "viewer")]
/// Performs a three-way merge of a value.
///
/// Given three versions of a value (`self`, `other`, `other_prev`), the merge
/// updates `self` when `other` has changed relative to `other_prev`, then
/// writes the resolved value back into both `other` and `other_prev` so the
/// next call starts from a consistent baseline.
pub(crate) trait ThreeWayMerge {
    /// Merges `other` into `self` using `other_prev` as the baseline.
    fn merge(&mut self, other: &mut Self, other_prev: &mut Self);
}

#[cfg(feature = "viewer")]
impl<T: Copy + PartialEq> ThreeWayMerge for T {
    #[inline]
    fn merge(&mut self, other: &mut Self, other_prev: &mut Self) {
        if *other != *other_prev {
            *self = *other;
        }

        *other = *self;
        *other_prev = *other;
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};
    use std::ffi::c_char;
    use super::LockUnpoison;

    /// Verifies that `lock_unpoison` recovers a poisoned mutex and preserves the inner value.
    #[test]
    fn test_lock_unpoison_recovers_poisoned_mutex() {
        let mutex = Arc::new(Mutex::new(42_i32));
        let mutex_clone = Arc::clone(&mutex);

        // Panic while holding the lock to poison it.
        let _ = std::panic::catch_unwind(move || {
            let _guard = mutex_clone.lock().unwrap();
            panic!("intentional panic to poison mutex");
        });

        // The mutex must be poisoned now.
        assert!(mutex.lock().is_err(), "mutex should be poisoned");

        // lock_unpoison must recover the lock and preserve the value.
        let value = *mutex.lock_unpoison();
        assert_eq!(value, 42, "inner value must be preserved after unpoison");

        // After unpoison, regular lock must succeed.
        assert!(mutex.lock().is_ok(), "mutex should no longer be poisoned");
    }

    /// A non-negative-only check used across the comprehensive arm tests below.
    fn non_negative(v: i32) -> Result<(), &'static str> {
        if v >= 0 { Ok(()) } else { Err("must be non-negative") }
    }

    /// An array check (first element non-negative) used by the reference-array arm tests below.
    fn first_non_negative(a: [f64; 2]) -> Result<(), &'static str> {
        if a[0] >= 0.0 { Ok(()) } else { Err("first must be non-negative") }
    }

    /// Comprehensively exercises **every** arm of `getter_setter!` on minimal dummy structs,
    /// verifying both the generated method shapes and the optional `{ check }` constraint on each
    /// arm that writes a value. Grouped by arm; a separate struct per arm avoids method-name
    /// collisions.
    #[test]
    #[allow(clippy::bool_assert_comparison)]
    // Several arms generate companion methods (e.g. a `set_*` alongside the `with_*`/getter under
    // test) that this test intentionally does not all call -- it verifies generation, not every path.
    #[allow(dead_code)]
    fn test_getter_setter_all_arms() {
        use crate::getter_setter as gs;

        // --- Getter arms ---------------------------------------------------------------------
        // Arm: (get, [bool])
        struct GetBool { flag: i32 }
        impl GetBool { gs!(get, [flag: bool; "flag.";]); }
        assert_eq!(GetBool { flag: 1 }.flag(), true);
        assert_eq!(GetBool { flag: 0 }.flag(), false);

        // Arm: (get, [&Type]) -- default allow_mut generates `_mut`; `allow_mut = false` omits it.
        struct GetRef { arr: [f64; 3] }
        impl GetRef { gs!(get, [arr: &[f64; 3]; "array.";]); }
        let mut gr = GetRef { arr: [1.0, 2.0, 3.0] };
        assert_eq!(gr.arr(), &[1.0, 2.0, 3.0]);
        gr.arr_mut()[0] = 9.0;
        assert_eq!(gr.arr(), &[9.0, 2.0, 3.0]);

        struct GetRefNoMut { arr: [f64; 2] }
        impl GetRefNoMut { gs!(get, [(allow_mut = false) arr: &[f64; 2]; "array.";]); }
        assert_eq!(GetRefNoMut { arr: [4.0, 5.0] }.arr(), &[4.0, 5.0]);

        // Arm: (get, [Type]) -- with a `+ symbol` getter-name suffix.
        struct GetVal { val: i32 }
        impl GetVal { gs!(get, [val + _renamed: i32; "value.";]); }
        assert_eq!(GetVal { val: 7 }.val_renamed(), 7);

        // Arm: (force!, get, [Type])
        struct ForceGet { v: i32 }
        impl ForceGet { gs!(force!, get, [v: i32; "v.";]); }
        assert_eq!(ForceGet { v: 42 }.v(), 42);

        // --- Setter / builder base arms ------------------------------------------------------
        // Arm: (set, [...]) -- plain and checked.
        struct Set { a: i32, b: i32 }
        impl Set {
            gs!(set, [
                a: i32; "a.";
                b: i32 { non_negative } => &'static str; "b.";
            ]);
        }
        let mut s = Set { a: 0, b: 0 };
        s.set_a(5);
        assert_eq!(s.a, 5);
        assert_eq!(s.set_b(3), Ok(()));
        assert_eq!(s.b, 3);
        assert_eq!(s.set_b(-1), Err("must be non-negative"));
        assert_eq!(s.b, 3); // unchanged on error

        // Arm: (force!, set, [...]) -- plain and checked.
        struct ForceSet { a: i32, b: i32 }
        impl ForceSet {
            gs!(force!, set, [
                a: i32; "a.";
                b: i32 { non_negative } => &'static str; "b.";
            ]);
        }
        let mut fs = ForceSet { a: 0, b: 0 };
        fs.set_a(8);
        assert_eq!(fs.a, 8);
        assert_eq!(fs.set_b(2), Ok(()));
        assert_eq!(fs.set_b(-1), Err("must be non-negative"));
        assert_eq!(fs.b, 2);

        // Arm: (with, [...]) -- consuming builder, checked (valid input; panic path tested below).
        struct With { v: i32 }
        impl With { gs!(with, [v: i32 { non_negative }; "v.";]); }
        assert_eq!(With { v: 0 }.with_v(6).v, 6);

        // Arm: ([&] with, [...]) -- &mut builder, checked.
        struct RefWith { v: i32 }
        impl RefWith { gs!([&] with, [v: i32 { non_negative }; "v.";]); }
        let mut rw = RefWith { v: 0 };
        rw.with_v(4);
        assert_eq!(rw.v, 4);

        // Arm: (force!, with, [...]) and (force!, [&] with, [...]) -- checked.
        struct ForceWith { v: i32 }
        impl ForceWith { gs!(force!, with, [v: i32 { non_negative }; "v.";]); }
        assert_eq!(ForceWith { v: 0 }.with_v(3).v, 3);

        struct ForceRefWith { v: i32 }
        impl ForceRefWith { gs!(force!, [&] with, [v: i32 { non_negative }; "v.";]); }
        let mut frw = ForceRefWith { v: 0 };
        frw.with_v(5);
        assert_eq!(frw.v, 5);

        // --- Aggregate arms ------------------------------------------------------------------
        // Arm: (force!, get, set, [...]) -- plain and checked.
        struct ForceGetSet { v: i32 }
        impl ForceGetSet { gs!(force!, get, set, [v: i32 { non_negative } => &'static str; "v.";]); }
        let mut fgs = ForceGetSet { v: 0 };
        assert_eq!(fgs.set_v(11), Ok(()));
        assert_eq!(fgs.v(), 11);
        assert_eq!(fgs.set_v(-1), Err("must be non-negative"));

        // Arm: (get, set, [bool]) and (get, set, [Type] + check).
        struct GetSetBool { flag: i32 }
        impl GetSetBool { gs!(get, set, [flag: bool; "flag.";]); }
        let mut gsb = GetSetBool { flag: 1 };
        assert_eq!(gsb.flag(), true);
        gsb.set_flag(false);
        assert_eq!(gsb.flag(), false);

        struct GetSetVal { count: i32, checked: i32 }
        impl GetSetVal {
            gs!(get, set, [
                count: i32; "count.";
                checked: i32 { non_negative } => &'static str; "checked.";
            ]);
        }
        let mut gsv = GetSetVal { count: 5, checked: 0 };
        gsv.set_count(10);
        assert_eq!(gsv.count(), 10);
        assert_eq!(gsv.set_checked(7), Ok(()));
        assert_eq!(gsv.checked(), 7);
        assert_eq!(gsv.set_checked(-1), Err("must be non-negative"));
        assert_eq!(gsv.checked(), 7);

        // Arm: (with, get, set, [bool]) and ([&] with, get, set, [bool]).
        struct WithGetSetBool { flag: i32 }
        impl WithGetSetBool { gs!(with, get, set, [flag: bool; "flag.";]); }
        assert_eq!(WithGetSetBool { flag: 0 }.with_flag(true).flag(), true);

        struct RefWithGetSetBool { flag: i32 }
        impl RefWithGetSetBool { gs!([&] with, get, set, [flag: bool; "flag.";]); }
        let mut rwgsb = RefWithGetSetBool { flag: 0 };
        rwgsb.with_flag(true);
        assert_eq!(rwgsb.flag(), true);

        // Arm: (with, get, set, [Type]) and ([&] with, get, set, [Type] + check).
        struct WithGetSetVal { v: i32 }
        impl WithGetSetVal { gs!(with, get, set, [v: i32; "v.";]); }
        let mut wgsv = WithGetSetVal { v: 0 }.with_v(3);
        assert_eq!(wgsv.v(), 3);
        wgsv.set_v(4);
        assert_eq!(wgsv.v(), 4);

        struct RefWithGetSetVal { v: i32, w: i32 }
        impl RefWithGetSetVal {
            gs!([&] with, get, set, [
                v: i32; "v.";
                w: i32 { non_negative } => &'static str; "w.";
            ]);
        }
        let mut rwgsv = RefWithGetSetVal { v: 0, w: 0 };
        rwgsv.with_v(1);
        assert_eq!(rwgsv.v(), 1);
        assert_eq!(rwgsv.set_w(2), Ok(()));
        assert_eq!(rwgsv.w(), 2);
        assert_eq!(rwgsv.set_w(-1), Err("must be non-negative"));

        // Arm: (force!, with, get, set, [...]) and (force!, [&] with, get, set, [...]).
        struct ForceWithGetSet { v: i32 }
        impl ForceWithGetSet { gs!(force!, with, get, set, [v: i32 { non_negative } => &'static str; "v.";]); }
        let fwgs = ForceWithGetSet { v: 0 }.with_v(9);
        assert_eq!(fwgs.v(), 9);

        struct ForceRefWithGetSet { v: i32 }
        impl ForceRefWithGetSet { gs!(force!, [&] with, get, set, [v: i32; "v.";]); }
        let mut frwgs = ForceRefWithGetSet { v: 0 };
        frwgs.with_v(6);
        assert_eq!(frwgs.v(), 6);

        // Arm: (with, get, [&Type]) and ([&] with, get, [&Type]) -- array fields, plain and checked.
        struct WithGetRef { arr: [f64; 3] }
        impl WithGetRef { gs!(with, get, [arr: &[f64; 3]; "array.";]); }
        let wgr = WithGetRef { arr: [0.0; 3] }.with_arr([1.0, 2.0, 3.0]);
        assert_eq!(wgr.arr(), &[1.0, 2.0, 3.0]);

        struct RefWithGetRef { arr: [f64; 2] }
        impl RefWithGetRef { gs!([&] with, get, [arr: &[f64; 2] { first_non_negative }; "array.";]); }
        let mut rwgr = RefWithGetRef { arr: [0.0; 2] };
        rwgr.with_arr([5.0, 6.0]);
        assert_eq!(rwgr.arr(), &[5.0, 6.0]);
    }

    /* `#[should_panic]` tests: a checked `with_*` builder must panic on invalid input, since a
       builder has to keep returning `Self` and cannot surface a `Result`. One per distinct
       check-bearing builder code path (the four base `with` arms plus each aggregate that forwards
       a check into a builder, which also guards against an aggregate silently dropping the check). */

    /// Base `(with, [...])` builder panics on a failing check.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    fn test_with_check_panics() {
        struct S { v: i32 }
        impl S { crate::getter_setter!(with, [v: i32 { non_negative }; "v.";]); }
        S { v: 0 }.with_v(-1);
    }

    /// Base `([&] with, [...])` builder panics on a failing check.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    fn test_ref_with_check_panics() {
        struct S { v: i32 }
        impl S { crate::getter_setter!([&] with, [v: i32 { non_negative }; "v.";]); }
        S { v: 0 }.with_v(-1);
    }

    /// `(force!, with, [...])` builder panics on a failing check.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    fn test_force_with_check_panics() {
        struct S { v: i32 }
        impl S { crate::getter_setter!(force!, with, [v: i32 { non_negative }; "v.";]); }
        S { v: 0 }.with_v(-1);
    }

    /// `(force!, [&] with, [...])` builder panics on a failing check.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    fn test_force_ref_with_check_panics() {
        struct S { v: i32 }
        impl S { crate::getter_setter!(force!, [&] with, [v: i32 { non_negative }; "v.";]); }
        S { v: 0 }.with_v(-1);
    }

    /// The `([&] with, get, set, [...])` aggregate forwards the check into `with_*`, which panics.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    #[allow(dead_code)] // the aggregate also generates `w`/`set_w`, unused in this panic test
    fn test_with_get_set_aggregate_check_panics() {
        struct S { w: i32 }
        impl S {
            crate::getter_setter!([&] with, get, set, [
                w: i32 { non_negative } => &'static str; "w.";
            ]);
        }
        S { w: 0 }.with_w(-1);
    }

    /// The `(force!, with, get, set, [...])` aggregate forwards the check into `with_*`, which panics.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    #[allow(dead_code)] // the aggregate also generates `v`/`set_v`, unused in this panic test
    fn test_force_with_get_set_aggregate_check_panics() {
        struct S { v: i32 }
        impl S {
            crate::getter_setter!(force!, with, get, set, [
                v: i32 { non_negative } => &'static str; "v.";
            ]);
        }
        S { v: 0 }.with_v(-1);
    }

    /// The `([&] with, get, [&Type])` reference-array aggregate forwards the check into `with_*`.
    #[test]
    #[should_panic(expected = "invalid builder argument")]
    #[allow(dead_code)] // the aggregate also generates `arr`/`arr_mut`, unused in this panic test
    fn test_with_get_ref_aggregate_check_panics() {
        struct S { arr: [f64; 2] }
        impl S {
            crate::getter_setter!([&] with, get, [arr: &[f64; 2] { first_non_negative }; "array.";]);
        }
        S { arr: [0.0; 2] }.with_arr([-1.0, 0.0]);
    }

    /// Tests both arms of `cast_mut_info!`: without and with the optional debug expression.
    #[test]
    fn test_cast_mut_info_both_arms() {
        let mut val: [u8; 4] = [7, 0, 0, 0];

        // Without debug expression.
        let r: &mut [u8; 4] = crate::cast_mut_info!(&mut val);
        r[0] = 42;
        assert_eq!(val[0], 42);

        // With debug expression.
        let idx: usize = 0;
        let r2: &mut [u8; 4] = crate::cast_mut_info!(&mut val, idx);
        r2[0] = 99;
        assert_eq!(val[0], 99);
    }

    /// Tests the three combination arms of `c_str_as_str_method!` that are never
    /// used in production code: `(get, set)` (arm 5), `(with, set)` (arm 6),
    /// and `(with, get)` (arm 7).
    #[test]
    fn test_c_str_as_str_method_combination_arms() {
        // arm 5: (get, set) -- getter + setter, no builder.
        struct GetSet { name: [c_char; 16] }
        impl GetSet {
            crate::c_str_as_str_method!(get, set { name; "name."; });
        }

        // arm 6: (with, set) -- setter + builder, no getter.
        struct WithSet { label: [c_char; 16] }
        impl WithSet {
            crate::c_str_as_str_method!(with, set { label; "label."; });
        }

        // arm 7: (with, get) -- getter + builder, no setter.
        struct WithGet { title: [c_char; 16] }
        impl WithGet {
            crate::c_str_as_str_method!(with, get { title; "title."; });
        }

        // arm 5: set then get.
        let mut gs = GetSet { name: [0; 16] };
        gs.set_name("hello");
        assert_eq!(gs.name(), "hello");

        // arm 6: with_ builder; raw bytes confirm the write.
        let ws = WithSet { label: [0; 16] }.with_label("world");
        let bytes: &[u8] = bytemuck::cast_slice(&ws.label[..]);
        assert!(bytes.starts_with(b"world\0"), "with_label did not write label");
        // arm 6 setter path.
        let mut ws2 = WithSet { label: [0; 16] };
        ws2.set_label("bye");
        let bytes2: &[u8] = bytemuck::cast_slice(&ws2.label[..]);
        assert!(bytes2.starts_with(b"bye\0"), "set_label did not write label");

        // arm 7: with_ builder then getter.
        let wg = WithGet { title: [0; 16] }.with_title("foo");
        assert_eq!(wg.title(), "foo");
    }
}
