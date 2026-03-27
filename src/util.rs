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


/// Creates a (start, length) tuple based on
/// lookup variables that define the mapping in MuJoCo's mjModel struct.
/// The tuple is used to create views to correct addresses in corresponding structs.
/// Format: item id, map from item id to index inside the array of all items' values,
///         number of items, maximum number of elements inside the array of all items' values
#[macro_export]
#[doc(hidden)]
macro_rules! mj_view_indices {
    ($id:expr, $addr_map:expr, $njnt:expr, $max_n:expr) => {
        {
            let start_addr = *$addr_map.add($id) as isize;
            if start_addr == -1 {
                (0, 0)
            }
            else
            {
                // Some addr maps (e.g. actuator_actadr) contain -1 for items with no
                // allocated data (stateless actuators).  Skip over those sentinels when
                // looking for the end boundary of the current range.
                let mut next_idx = $id + 1;
                let end_addr: usize = loop {
                    if next_idx >= $njnt as usize {
                        break $max_n as usize;
                    }
                    let next_addr = *$addr_map.add(next_idx) as isize;
                    if next_addr != -1 {
                        break next_addr as usize;
                    }
                    next_idx += 1;
                };
                let n = end_addr - start_addr as usize;
                (start_addr as usize, n)
            }
        }
    };
}

/// Returns the correct address mapping based on the X in nX (nq, nv, nu, ...).
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_nx_to_mapping {
    ($model_ffi:ident, nq) => {
        $model_ffi.jnt_qposadr
    };

    ($model_ffi:ident, nv) => {
        $model_ffi.jnt_dofadr
    };

    ($model_ffi:ident, nsensordata) => {
        $model_ffi.sensor_adr
    };
    ($model_ffi:ident, ntupledata) => {
        $model_ffi.tuple_adr
    };
    ($model_ffi:ident, ntexdata) => {
        $model_ffi.tex_adr
    };
    ($model_ffi:ident, nnumericdata) => {
        $model_ffi.numeric_adr
    };
    ($model_ffi:ident, nhfielddata) => {
        $model_ffi.hfield_adr
    };
    ($model_ffi:ident, na) => {
        $model_ffi.actuator_actadr
    };
    ($model_ffi:ident, nJten) => {
        $model_ffi.ten_J_rowadr
    };
}


/// Returns the correct number of items based on the X in nX (nq, nv, nu, ...).
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_nx_to_nitem {
    ($model_ffi:ident, nq) => {
        $model_ffi.njnt
    };

    ($model_ffi:ident, nv) => {
        $model_ffi.njnt
    };

    ($model_ffi:ident, nsensordata) => {
        $model_ffi.nsensor
    };
    ($model_ffi:ident, ntupledata) => {
        $model_ffi.ntuple
    };
    ($model_ffi:ident, ntexdata) => {
        $model_ffi.ntex
    };
    ($model_ffi:ident, nnumericdata) => {
        $model_ffi.nnumeric
    };
    ($model_ffi:ident, nhfielddata) => {
        $model_ffi.nhfield
    };
    ($model_ffi:ident, na) => {
        $model_ffi.nu
    };
    ($model_ffi:ident, nJten) => {
        $model_ffi.ntendon
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
/// - **Dynamic range**: `attr: nXXX`: start and length resolved via [`mj_view_indices!`],
///   where `nXXX` is the major-dimension field (e.g. `nhfielddata`, `ntexdata`).
#[doc(hidden)]
#[macro_export]
macro_rules! info_method {
    ($info_type:ident, $ffi:expr, $type_:ident, [$($attr:ident: $len:expr),*], [$($attr_ffi:ident: $len_ffi:ident $(* $multiplier:expr)?),*], [$($attr_dyn:ident: $ffi_len_dyn:expr),*]) => {
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
                let c_name = CString::new(name).unwrap();
                let ffi = self.$ffi;
                let id = unsafe { mj_name2id(ffi, MjtObj::[<mjOBJ_ $type_:upper>] as i32, c_name.as_ptr()) };
                if id == -1 {
                    return None;
                }

                let id = id as usize;
                $(
                    let $attr = (id * $len, $len);
                )*
                $(
                    let $attr_ffi = (id * ffi.$len_ffi as usize $( * $multiplier)*, ffi.$len_ffi as usize $( * $multiplier)*);
                )*
                $(
                    let $attr_dyn = unsafe { mj_view_indices!(
                        id,
                        mj_model_nx_to_mapping!(ffi, $ffi_len_dyn),
                        mj_model_nx_to_nitem!(ffi, $ffi_len_dyn),
                        ffi.$ffi_len_dyn
                    ) };
                )*

                let model_signature = ffi.signature;
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
            #[derive(Debug)]
            pub struct [<Mj $name:camel $info_type Info>] {
                pub name: String,
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
                    pub $attr: $crate::util::PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $attr_ro: $crate::util::PointerViewUnsafeMut<'d, $type_ro>,
                )*
                $(
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
                    pub $attr: $crate::util::PointerView<'d, $type_>,
                )*
                $(
                    pub $attr_ro: $crate::util::PointerView<'d, $type_ro>,
                )*
                $(
                    pub $opt_attr: Option<$crate::util::PointerView<'d, $type_opt>>,
                )*
            }
        }
    };
}


#[doc(hidden)]
#[macro_export]
macro_rules! getter_setter {
    (get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: bool; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Check ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> bool {
                self$(.$ffi())?.$name == 1
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

    (set, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
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

    (force!, set, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                }
            )*
        }
    };

    /* Builder pattern */
    (force!, with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                    self
                }
            )*
        }
    };

    (force!, [&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = $crate::util::force_cast(value) };
                    self
                }
            )*
        }
    };

    (with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };

    ([&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };
    
    /* Handling of optional arguments */
    /* Enum pass */
    (force!, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    /* Builder pattern */
    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)?: $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    (force!, $([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [$($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [$($([$ffi_mut])? $name : $type ; $comment );* ]);
        $crate::getter_setter!(force!, $([$token])? with, [$($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    ($([$token:tt])? with, get, [$( $([$ffi: ident, $ffi_mut:ident])? $((allow_mut = $allow_mut:literal))? $name:ident $(+ $symbol:tt)? : & $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi, $ffi_mut])? $((allow_mut = $allow_mut))? $name $(+ $symbol)? : & $type ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $( $([$ffi_mut])? $name : $type ; $comment );* ]);
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
    ($($(($unsafe_mut:ident))? $name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[$type:ty $([$force:ident])?; $doc:literal; $($len_accessor:tt)*]),*) => {
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
/// array must be NUL-terminated and contain valid UTF-8. Setters ensure
/// ASCII and length bounds but the caller must still guarantee the destination
/// buffer is large enough.  The macro itself simply emits the unsafe code
/// without additional checks.
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
pub fn assert_mujoco_version() {
    let linked_version = unsafe { mj_version() as u32 };
    let mujoco_rs_version_string = option_env!("CARGO_PKG_VERSION").unwrap_or_else(|| "unknown+mj-unknown");
    assert_eq!(
        linked_version, mjVERSION_HEADER,
        "linked MuJoCo version value ({linked_version}) does not match expected version value ({mjVERSION_HEADER}), \
        with which MuJoCo-rs {mujoco_rs_version_string} FFI bindings were generated.",
    );
}


/// Forcefully casts a value of type `T` to type `U`.
/// # Safety
/// This is a safer alternative to `std::mem::transmute` that performs compile-time
/// size and alignment checks.  It does **not** guarantee that the bit patterns
/// are compatible -- the caller must still ensure semantic validity.
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
    unsafe { std::mem::ManuallyDrop::into_inner(Transmuter { from: std::mem::ManuallyDrop::new(val) }.to) }
}


/// Asserts at compile time that casting from `Src` to `Dst` is
/// size-and-alignment compatible.
///
/// The target element size must be a multiple of the source element size
/// (covers both same-size type conversions and array-grouping casts like
/// `*const f64` to `*const [f64; 3]`), and the source alignment must be at
/// least as strict as the target alignment.
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

    /// Exercises the three "dead" combination arms of `getter_setter!` (arms 11, 12, 13)
    /// by instantiating each on a minimal dummy struct.
    ///
    /// - Arm 11: `(force!, get, set, [...])` -- force-cast getter + setter.
    /// - Arm 12: `(get, set, [... : bool ...])` -- bool getter (field `== 1`) + setter.
    /// - Arm 13: `(get, set, [... : $type ...])` -- `.into()` getter + setter.
    #[test]
    fn test_getter_setter_dead_arms_11_12_13() {
        struct ArmEleven { x: i32, y: i32 }
        impl ArmEleven {
            crate::getter_setter!(force!, get, set, [x: i32; "x field."; y: i32; "y field.";]);
        }

        struct ArmTwelve { flag: i32 }
        impl ArmTwelve {
            crate::getter_setter!(get, set, [flag: bool; "flag field.";]);
        }

        struct ArmThirteen { count: i32 }
        impl ArmThirteen {
            crate::getter_setter!(get, set, [count: i32; "count field.";]);
        }

        let mut arm11 = ArmEleven { x: 3, y: 7 };
        assert_eq!(arm11.x(), 3);
        assert_eq!(arm11.y(), 7);
        arm11.set_x(9);
        assert_eq!(arm11.x(), 9);
        arm11.set_y(10);
        assert_eq!(arm11.y(), 10);

        let mut arm12 = ArmTwelve { flag: 1 };
        assert!(arm12.flag());
        arm12.set_flag(false);
        assert!(!arm12.flag());

        let mut arm13 = ArmThirteen { count: 5 };
        assert_eq!(arm13.count(), 5);
        arm13.set_count(10);
        assert_eq!(arm13.count(), 10);
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
