//! Utilities for model editing purposes.
use std::ffi::{CStr, CString};

use crate::util::checked_c_len;
use crate::error::MjEditError;
use crate::mujoco_c::*;


/***************************
** Utility functions
***************************/
/// Reads MJS string (C++) as a `&str`.
///
/// # Safety
/// `string` must point to a valid `mjString` object for the duration `'a`, and the string must
/// not be written during `'a`, because a write reallocates the C++ buffer.
///
/// # Panics
/// Panics if the string contains invalid UTF-8.
pub(crate) unsafe fn read_mjs_string<'a>(string: *const mjString) -> &'a str {
    let ptr = unsafe { mjs_getString(string) };
    if ptr.is_null() {
        ""
    } else {
        // SAFETY: `ptr` points into the buffer of the C++ std::string, valid for `'a`.
        unsafe { CStr::from_ptr(ptr) }.to_str().unwrap()
    }
}

/// Writes to a `destination` MJS string (C++) from a `source` `&str`.
///
/// # Safety
/// `destination` must point to a valid `mjString` object.
///
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) unsafe fn write_mjs_string(source: &str, destination: *mut mjString) {
    let c_source = CString::new(source).unwrap();
    unsafe { mjs_setString(destination, c_source.as_ptr()) };
}

/// Reads MJS double vector (C++) as a `&\[f64\]`.
/// # Safety
/// `array` must point to a valid `mjDoubleVec` object for the duration `'a`.
pub(crate) unsafe fn read_mjs_vec_f64<'a>(array: *const mjDoubleVec) -> &'a [f64] {
    let mut userdata_length = 0;
    let ptr_arr = unsafe { mjs_getDouble(array, &mut userdata_length) };
    if ptr_arr.is_null() {
        return &[];
    }
    unsafe { std::slice::from_raw_parts(ptr_arr, userdata_length as usize) }
}

/// Writes MJS double vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjDoubleVec` object.
///
/// # Panics
/// Panics if `source` has more than [`i32::MAX`] elements.
pub(crate) unsafe fn write_mjs_vec_f64(source: &[f64], destination: *mut mjDoubleVec) {
    unsafe {
        mjs_setDouble(destination, source.as_ptr(), checked_c_len(source.len()));
    }
}

/// Writes MJS float vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjFloatVec` object.
///
/// # Panics
/// Panics if `source` has more than [`i32::MAX`] elements.
pub(crate) unsafe fn write_mjs_vec_f32(source: &[f32], destination: *mut mjFloatVec) {
    unsafe {
        mjs_setFloat(destination, source.as_ptr(), checked_c_len(source.len()));
    }
}

/// Appends MJS float vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjFloatVecVec` object.
///
/// # Panics
/// Panics if `source` has more than [`i32::MAX`] elements.
pub(crate) unsafe fn append_mjs_vec_vec_f32(source: &[f32], destination: *mut mjFloatVecVec) {
    unsafe {
        mjs_appendFloatVec(destination, source.as_ptr(), checked_c_len(source.len()));
    }
}

/// Writes MJS int vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjIntVec` object.
///
/// # Panics
/// Panics if `source` has more than [`i32::MAX`] elements.
pub(crate) unsafe fn write_mjs_vec_i32(source: &[i32], destination: *mut mjIntVec) {
    unsafe {
        mjs_setInt(destination, source.as_ptr(), checked_c_len(source.len()));
    }
}

/// Appends MJS int vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjIntVecVec` object.
///
/// # Panics
/// Panics if `source` has more than [`i32::MAX`] elements.
pub(crate) unsafe fn append_mjs_vec_vec_i32(source: &[i32], destination: *mut mjIntVecVec) {
    unsafe {
        mjs_appendIntVec(destination, source.as_ptr(), checked_c_len(source.len()));
    }
}

/// Split `source` to entries and copy to `destination` (C++).
///
/// # Safety
/// `destination` must point to a valid `mjStringVec` object.
///
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) unsafe fn write_mjs_vec_string(source: &str, destination: *mut mjStringVec) {
    let c_source = CString::new(source).unwrap();
    unsafe {
        mjs_setStringVec(destination, c_source.as_ptr());
    }
}

/// Appends `source` as a single entry to `destination` (C++).
///
/// # Safety
/// `destination` must point to a valid `mjStringVec` object.
///
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) unsafe fn append_mjs_vec_string(source: &str, destination: *mut mjStringVec) {
    let c_source = CString::new(source).unwrap();
    unsafe {
        mjs_appendString(destination, c_source.as_ptr());
    }
}

/// Writes MJS byte vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjByteVec` object.
///
/// # Panics
/// Panics if `source` occupies more than [`i32::MAX`] bytes.
pub(crate) unsafe fn write_mjs_vec_byte<T: bytemuck::NoUninit>(source: &[T], destination: *mut mjByteVec) {
    let bytes: &[u8] = bytemuck::cast_slice(source);
    unsafe {
        mjs_setBuffer(destination, bytes.as_ptr().cast(), checked_c_len(bytes.len()));
    }
}

/// Deletes `element` from the specification that holds it, with the checks MuJoCo omits.
///
/// # Errors
/// Returns [`MjEditError::UnsupportedOperation`] for a default class, a frame, a tendon wrap and
/// the world body, and [`MjEditError::DeleteFailed`] when MuJoCo refuses the deletion.
///
/// # Safety
/// Same contract as [`SpecObject::delete`](super::traits::SpecObject::delete), and `element` must
/// point to an element of a specification.
pub(crate) unsafe fn delete_element(element: *mut mjsElement) -> Result<(), MjEditError> {
    let elemtype = unsafe { (*element).elemtype };

    // mjCDef is not an mjCBase, so this precedes mjs_getSpec. A frame's type is 100, past the
    // mjNOBJECT end of the element list MuJoCo indexes with it, and a wrap's slot in it is null.
    if matches!(
        elemtype,
        mjtObj::mjOBJ_DEFAULT | mjtObj::mjOBJ_FRAME | mjtObj::mjOBJ_UNKNOWN
    ) {
        return Err(MjEditError::UnsupportedOperation);
    }

    let spec = unsafe { mjs_getSpec(element) };

    // Prevent deletion of the world-bodies.
    if elemtype == mjtObj::mjOBJ_BODY && element == unsafe { mjs_firstElement(spec, elemtype) } {
        return Err(MjEditError::UnsupportedOperation);
    }

    match unsafe { mjs_delete(spec, element) } {
        0 => Ok(()),
        _ => {
            // SAFETY: the message belongs to the spec and lives until the next call on it.
            let error_msg = unsafe {
                let ptr = mjs_getError(spec);
                if ptr.is_null() {
                    "Unknown error".to_owned()
                } else {
                    CStr::from_ptr(ptr).to_string_lossy().into_owned()
                }
            };
            Err(MjEditError::DeleteFailed(error_msg))
        }
    }
}


/***************************
** Helper macros
***************************/
/// Generates both an `add_$name` method (panics on OOM, delegates to `try_add_$name`) and a
/// `try_add_$name` method (returns `Result`) for adding child elements that accept a default.
macro_rules! add_x_method {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!(
                "Add and return a child [`", stringify!([<Mjs $name:camel>]), "`].\n\n",
                "# Note\n",
                "MuJoCo ends the process when the allocation fails, so this never fails."
            )]
            #[expect(deprecated, reason = "try_add_* keeps the implementation until it is removed")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!([<$name:camel>]), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Note\n\n",
                "<div class=\"warning\">\n\n",
                "`mjs_add", stringify!([<$name:camel>]), "` allocates with C++ `new`, which ends ",
                "the process instead of returning null, so this never returns `Err`. Prefer ",
                "[`Self::add_", stringify!($name), "`].\n\n",
                "</div>\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate the element."
            )]
            #[deprecated(
                since = "6.0.0",
                note = "always returns Ok; use the panicking variant"
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                // SAFETY: the element is freshly allocated by C++ operator new, so it is aligned,
                // initialized and unaliased. A null pointer becomes `None`.
                let ptr = unsafe { [<mjs_add $name:camel>](self.ffi_mut(), ptr::null()) };
                unsafe { [<Mjs $name:camel>]::from_ffi_ptr_mut(ptr) }.ok_or(MjEditError::AllocationFailed)
            }
        )*
    }};
}

/// Generates both `add_$name` (panics, delegates to `try_`) and `try_add_$name` (returns
/// `Result`) for elements parented by a frame.
macro_rules! add_x_method_by_frame {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!(
                "Add and return a child [`", stringify!([<Mjs $name:camel>]), "`].\n\n",
                "# Note\n",
                "MuJoCo ends the process when the allocation fails, so this never fails."
            )]
            #[expect(deprecated, reason = "try_add_* keeps the implementation until it is removed")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!([<$name:camel>]), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Note\n\n",
                "<div class=\"warning\">\n\n",
                "`mjs_add", stringify!([<$name:camel>]), "` allocates with C++ `new`, which ends ",
                "the process instead of returning null, so this never returns `Err`. Prefer ",
                "[`Self::add_", stringify!($name), "`].\n\n",
                "</div>\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate the element."
            )]
            #[deprecated(
                since = "6.0.0",
                note = "always returns Ok; use the panicking variant"
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                // SAFETY: mjs_addFrame always calls SetParent(body), so the parent is non-null,
                // and the element that mjs_addXxx returns is fresh, so nothing aliases it.
                unsafe {
                    let ep = self.element_mut_pointer();
                    let body_ptr = mjs_getParent(ep);
                    debug_assert!(!body_ptr.is_null(), "mjs_getParent returned null; frame has no parent body");
                    let ptr = [<mjs_add $name:camel>](body_ptr, ptr::null());
                    if ptr.is_null() {
                        return Err(MjEditError::AllocationFailed);
                    }
                    let set_result = mjs_setFrame((*ptr).element, self.ffi_mut());
                    debug_assert_eq!(set_result, 0, "mjs_setFrame failed; element or frame is invalid");
                    Ok([<Mjs $name:camel>]::from_ffi_ptr_mut(ptr).unwrap())
                }
            }
        )*
    }};
}

/// Generates both `add_$name` (panics, delegates to `try_`) and `try_add_$name` (returns
/// `Result`) for elements whose `mjs_addXxx` function takes no default argument.
macro_rules! add_x_method_no_default {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!(
                "Add and return a child [`", stringify!([<Mjs $name:camel>]), "`].\n\n",
                "# Note\n",
                "MuJoCo ends the process when the allocation fails, so this never fails."
            )]
            #[expect(deprecated, reason = "try_add_* keeps the implementation until it is removed")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!([<$name:camel>]), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Note\n\n",
                "<div class=\"warning\">\n\n",
                "`mjs_add", stringify!([<$name:camel>]), "` allocates with C++ `new`, which ends ",
                "the process instead of returning null, so this never returns `Err`. Prefer ",
                "[`Self::add_", stringify!($name), "`].\n\n",
                "</div>\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate the element."
            )]
            #[deprecated(
                since = "6.0.0",
                note = "always returns Ok; use the panicking variant"
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                let ptr = unsafe { [<mjs_add $name:camel>](self.ffi.as_ptr()) };
                unsafe { [<Mjs $name:camel>]::from_ffi_ptr_mut(ptr) }.ok_or(MjEditError::AllocationFailed)
            }
        )*
    }};
}


/// Creates `$item` and `$item_mut` methods that look an item up by name through `mjs_findElement`.
macro_rules! find_x_method {
    ($($item:ident),*) => {paste::paste! {
        $(
            #[doc = concat!(
                "Obtain an immutable reference to the ", stringify!($item), " with the given `name`.\n",
                "# Panics\n",
                "When the `name` contains '\\0' characters, a panic occurs."
            )]
            pub fn $item(&self, name: &str) -> Option<&[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = mjs_findElement(self.ffi.as_ptr(), MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<Mjs $item:camel>]::from_ffi_ptr([<mjs_as $item:camel>](ptr))
                    }
                }
            }

            #[doc = concat!(
                "Obtain a mutable reference to the ", stringify!($item), " with the given `name`.\n",
                "# Panics\n",
                "When the `name` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<$item _mut>](&mut self, name: &str) -> Option<&mut [<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = mjs_findElement(self.ffi.as_ptr(), MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<Mjs $item:camel>]::from_ffi_ptr_mut([<mjs_as $item:camel>](ptr))
                    }
                }
            }
        )*
    }};
}

/// Same as [`find_x_method`], but for types that have corresponding methods (instead of `mjs_findElement`).
macro_rules! find_x_method_direct {
    ($($item:ident),*) => {paste::paste!{
        $(
            #[doc = concat!(
                "Obtain an immutable reference to the ", stringify!($item), " with the given `name`.\n",
                "# Panics\n",
                "When the `name` contains '\\0' characters, a panic occurs."
            )]
            pub fn $item(&self, name: &str) -> Option<&[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = [<mjs_find $item:camel>](self.ffi.as_ptr(), c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<Mjs $item:camel>]::from_ffi_ptr(ptr)
                    }
                }
            }

            #[doc = concat!(
                "Obtain a mutable reference to the ", stringify!($item), " with the given `name`.\n",
                "# Panics\n",
                "When the `name` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<$item _mut>](&mut self, name: &str) -> Option<&mut [<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = [<mjs_find $item:camel>](self.ffi.as_ptr(), c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<Mjs $item:camel>]::from_ffi_ptr_mut(ptr)
                    }
                }
            }
        )*
    }};
}


/// Declares an opaque zero-sized element handle that stands at the address of `$raw`.
///
/// The handle must stay zero-sized, because [`std::mem::swap`] copies `size_of::<T>()` bytes and
/// exchanging two elements would leave each specification holding pointers that the other owns.
/// Do not give the handle a field of type `$raw`, and do not add [`DerefMut`](std::ops::DerefMut).
macro_rules! mjs_opaque {
    ($handle:ident <= $raw:ident, $doc:expr) => {
        #[doc = $doc]
        #[repr(C)]
        pub struct $handle {
            // A private field with no constructor keeps the handle non-instantiable downstream.
            _data: (),
            // Removes the automatic `Send`, `Sync` and `Unpin`; the element belongs to its spec.
            _marker: std::marker::PhantomData<(*mut u8, std::marker::PhantomPinned)>,
        }

        impl $handle {
            /// Returns the FFI struct that the handle stands on.
            pub fn ffi(&self) -> &$raw {
                // SAFETY: the handle stands at the address of a live, aligned `$raw`, and the cast
                // keeps the provenance of the pointer that built the handle.
                unsafe { &*(self as *const Self).cast::<$raw>() }
            }

            /// Returns the FFI struct that the handle stands on, mutably.
            ///
            /// # Safety
            /// The caller must not exchange the contents of the struct with another element's.
            pub unsafe fn ffi_mut(&mut self) -> &mut $raw {
                // SAFETY: a unique borrow of the handle is a unique borrow of the struct.
                unsafe { &mut *(self as *mut Self).cast::<$raw>() }
            }

            /// Borrows the element that `ptr` addresses, or [`None`] when `ptr` is null.
            ///
            /// # Safety
            /// `ptr` must address a valid element that stays alive and unmoved for `'a`.
            pub(crate) unsafe fn from_ffi_ptr<'a>(ptr: *const $raw) -> Option<&'a Self> {
                // SAFETY: the cast keeps the address, and the handle reads no bytes of its own.
                unsafe { ptr.cast::<Self>().as_ref() }
            }

            /// Borrows the element mutably, or [`None`] when `ptr` is null.
            ///
            /// # Safety
            /// `ptr` must address a valid element that stays alive and unmoved for `'a`, and no
            /// other handle for that element may be live.
            pub(crate) unsafe fn from_ffi_ptr_mut<'a>(ptr: *mut $raw) -> Option<&'a mut Self> {
                unsafe { ptr.cast::<Self>().as_mut() }
            }
        }

        impl std::fmt::Debug for $handle {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                std::fmt::Debug::fmt(self.ffi(), f)
            }
        }
    };
}

/// Generates the accessor pair for an element handle that the FFI struct embeds by value.
///
/// `$name` must be a field of the type that `ffi()` returns.
macro_rules! nested_handle {
    ($name:ident: $handle:ty; $doc:expr) => {paste::paste!{
        #[doc = concat!("Returns an immutable reference to ", $doc)]
        pub fn $name(&self) -> &$handle {
            // SAFETY: the owner keeps the field alive and unmoved for as long as it is borrowed.
            unsafe { <$handle>::from_ffi_ptr(&raw const self.ffi().$name) }.unwrap()
        }

        #[doc = concat!("Returns a mutable reference to ", $doc)]
        pub fn [<$name _mut>](&mut self) -> &mut $handle {
            // SAFETY: as above, and a unique borrow of the owner is a unique borrow of the field.
            unsafe { <$handle>::from_ffi_ptr_mut(&raw mut self.ffi_mut().$name) }.unwrap()
        }
    }};
}


/// Creates the wrapper `$handle` around the FFI struct `$raw`. It also implements the methods
/// `info()` and `set_info()`, and the traits `Sealed` and [`SpecItem`](super::traits::SpecItem).
/// The handles are deliberately neither [`Send`] nor [`Sync`].
///
/// A leading `$kind with SpecObject:` also implements [`SpecObject`](super::traits::SpecObject).
/// `$kind` is the element kind as MuJoCo camel-cases it (`Texture`, `HField`), which names both the
/// [`MjtObj`] variant and the `mjs_as*` function. A trailing brace block adds methods to the
/// `SpecItem` implementation.
macro_rules! mjs_struct {
    (
        $kind:ident with SpecObject: $handle:ident <= $raw:ident
        $({ $($extra_trait_methods:tt)* })?
    ) => {paste::paste!{
        mjs_struct!($handle <= $raw $({ $($extra_trait_methods)* })?);

        impl SpecObject for $handle {
            const OBJ_TYPE: MjtObj = MjtObj::[<mjOBJ_ $kind:upper>];
            unsafe fn from_element_as_ptr_mut(element: *mut mjsElement) -> *mut Self {
                // The annotation ties the conversion function to `$raw`, so a call site cannot
                // pair the handle with another element's kind.
                let raw: *mut $raw = unsafe { [<mjs_as $kind>](element) };
                // SAFETY: the handle stands at the address of the struct that mjs_as returns.
                raw.cast::<Self>()
            }
        }
    }};

    (
        $handle:ident <= $raw:ident
        $({ $($extra_trait_methods:tt)* })?
    ) => {
        mjs_opaque!($handle <= $raw, concat!(
            stringify!($handle), " specification. An opaque handle for the FFI type [`",
            stringify!($raw), "`], reached through [`ffi`](Self::ffi)."));

        impl $handle {
            /// Return the message appended to compiler errors.
            /// # Panics
            /// Panics if it contains invalid UTF-8.
            pub fn info(&self) -> &str {
                // SAFETY: self.info is a valid mjString pointer for the lifetime of self.
                unsafe { read_mjs_string(self.ffi().info) }
            }

            /// Set the message appended to compiler errors.
            /// # Panics
            /// When the `info` contains '\0' characters, a panic occurs.
            pub fn set_info(&mut self, info: &str) {
                // SAFETY: self.info is a valid mjString pointer for the lifetime of self.
                unsafe { write_mjs_string(info, self.ffi().info) };
            }
        }

        impl crate::wrappers::mj_editing::traits::sealed::Sealed for $handle {}

        impl SpecItem for $handle {
            fn element_pointer(&self) -> *const mjsElement {
                self.ffi().element
            }

            $($(
                $extra_trait_methods
            )*)?
        }
    };
}

/// Implements the userdata method.
macro_rules! userdata_method {
    ($type:ty) => {paste::paste!{
        /// Return an immutable slice to userdata.
        pub fn userdata(&self) -> &[$type] {
            // SAFETY: self.userdata is a valid mjDoubleVec pointer for the lifetime of self.
            unsafe { [<read_mjs_vec_ $type>](self.ffi().userdata) }
        }
        
        /// Set `userdata`.
        pub fn set_userdata<T: AsRef<[$type]>>(&mut self, value: T) {
            // SAFETY: self.userdata is a valid pointer for the lifetime of self.
            unsafe { [<write_mjs_vec_ $type>](value.as_ref(), self.ffi().userdata) };
        }

        /// Builder method for setting `userdata`.
        pub fn with_userdata<T: AsRef<[$type]>>(&mut self, value: T) -> &mut Self {
            // SAFETY: self.userdata is a valid pointer for the lifetime of self.
            unsafe { [<write_mjs_vec_ $type>](value.as_ref(), self.ffi().userdata) };
            self
        }
    }};
}

/// Implements vector of strings methods for given attribute $name.
macro_rules! vec_string_set_append {
    ($($name:ident; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!(
                "Splits the `", stringify!($name), "` and put the split text as ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<set_ $name>](&mut self, value: &str) {
                // SAFETY: self.$name is a valid mjStringVec pointer for the lifetime of self.
                unsafe { write_mjs_vec_string(value, self.ffi().$name) };
            }

            #[doc = concat!(
                "Appends `", stringify!($name), "` as a single entry to ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<append_ $name>](&mut self, value: &str) {
                // SAFETY: self.$name is a valid mjStringVec pointer for the lifetime of self.
                unsafe { append_mjs_vec_string(value, self.ffi().$name) };
            }
        )*
    }};

    // Indexed variant: the vector is pre-sized with one entry per enum variant, so an entry is
    // set by index rather than appended.
    ($name:ident[$role_ty:ty] => $singular:ident; $comment:expr $(;)?) => {paste::paste!{
        #[doc = concat!(
            "Sets the entry at index `role` in `", stringify!($name), "` to `name`. ",
            $comment,
            "\n\n",
            "# Note\n",
            "MuJoCo pre-sizes `", stringify!($name), "` with one slot per [`", stringify!($role_ty),
            "`] value. An index with no slot ends the process through `mju_error`.\n",
            "\n",
            "# Panics\n",
            "When `name` contains '\\0' characters, a panic occurs."
        )]
        pub fn [<set_ $singular>](&mut self, role: $role_ty, name: &str) {
            let c_name = CString::new(name).unwrap();
            // SAFETY: self.$name is a valid mjStringVec pre-sized to one entry per role.
            unsafe { mjs_setInStringVec(self.ffi().$name, role as std::ffi::c_int, c_name.as_ptr()) };
        }

        #[doc = concat!(
            "Sets the entry at index `role` in `", stringify!($name), "` to `name`, ",
            "returning `&mut Self` for chaining. ",
            $comment,
            "\n\n",
            "Equivalent to [`set_", stringify!($singular), "`](Self::set_",
            stringify!($singular), ").\n",
            "\n",
            "# Panics\n",
            "When `name` contains '\\0' characters, a panic occurs."
        )]
        pub fn [<with_ $singular>](&mut self, role: $role_ty, name: &str) -> &mut Self {
            self.[<set_ $singular>](role, name);
            self
        }

        #[doc = concat!(
            "Replaces the entire `", stringify!($name), "` vector with whitespace-split entries from `value`. ",
            $comment,
            "\n\n",
            "<div class=\"warning\">\n\n",
            "This replaces the pre-sized vector. Prefer [`set_",
            stringify!($singular), "`](Self::set_", stringify!($singular),
            ") to set individual entries by role.\n\n",
            "</div>\n\n",
            "# Panics\n",
            "When the `value` contains '\\0' characters, a panic occurs."
        )]
        pub fn [<set_ $name>](&mut self, value: &str) {
            // SAFETY: self.$name is a valid mjStringVec pointer for the lifetime of self.
            unsafe { write_mjs_vec_string(value, self.ffi().$name) };
        }

        #[doc = concat!(
            "Appends `value` to the end of `", stringify!($name), "`. ",
            $comment,
            "\n\n",
            "<div class=\"warning\">\n\n",
            "Appending extends past the pre-sized vector. Prefer [`set_",
            stringify!($singular), "`](Self::set_", stringify!($singular),
            ") to set individual entries by role.\n\n",
            "</div>\n\n",
            "# Panics\n",
            "When the `value` contains '\\0' characters, a panic occurs."
        )]
        pub fn [<append_ $name>](&mut self, value: &str) {
            // SAFETY: self.$name is a valid mjStringVec pointer for the lifetime of self.
            unsafe { append_mjs_vec_string(value, self.ffi().$name) };
        }
    }};
}

/// Implements string methods for given attribute $name.
macro_rules! string_set_get_with {
    (@impl common $name:ident; $comment:expr;) => {paste::paste!{
        #[doc = concat!(
            "Return ", $comment,
            "\n",
            "# Panics\n",
            "Panics if the stored string is not valid UTF-8."
        )]
        pub fn $name(&self) -> &str {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { read_mjs_string(self.ffi().$name) }
        }

        #[allow(unused_unsafe)]
        #[doc = concat!(
            "Set ", $comment,
            "\n",
            "# Panics\n",
            "When the `value` contains '\\0' characters, a panic occurs."
        )]
        pub fn [<set_ $name>](&mut self, value: &str) {
            // SAFETY: the mjString field is valid for the lifetime of self.
            unsafe { write_mjs_string(value, unsafe { self.ffi_mut() }.$name) };
        }
    }};

    ( $($name:ident; $comment:expr;)* ) => {paste::paste!{
        $(
            string_set_get_with!(@impl common $name; $comment;);
            #[allow(unused_unsafe)]
            #[doc = concat!(
                "Builder method for setting ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<with_ $name>](mut self, value: &str) -> Self {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { write_mjs_string(value, unsafe { self.ffi_mut() }.$name) };
                self
            }
        )*
    }};

    ([&] $($name:ident; $comment:expr;)* ) => {paste::paste!{
        $(
            string_set_get_with!(@impl common $name; $comment;);
            #[allow(unused_unsafe)]
            #[doc = concat!(
                "Builder method for setting ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<with_ $name>](&mut self, value: &str) -> &mut Self {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { write_mjs_string(value, unsafe { self.ffi_mut() }.$name) };
                self
            }
        )*
    }};
}

/// Implements getters and setters for floating point (f32 or f64) attributes.
macro_rules! vec_set_get {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Return ", $comment)]
            pub fn $name(&self) -> &[$type] {
                // SAFETY: self.$name is a valid mjDoubleVec/mjFloatVec pointer for the lifetime of self.
                unsafe { [<read_mjs_vec_ $type>](self.ffi().$name) }
            }
        )*

        vec_set!($($name: $type; $comment);*);
    }};
}

/// Implements setters for non-string attributes.
///
/// Three forms are supported:
/// - `name: Type; "comment"`: a safe setter that takes `&[Type]` and writes it unchanged.
/// - `name: InputType => StoredType { check, "reason" } => ErrType; "comment"`: a safe setter
///   taking `&[InputType]` (an enum) and storing it as the C type `StoredType`. Every element
///   passes `check`, a `Fn(InputType) -> Result<(), ErrType>`, before anything is written, which
///   is what makes the cast sound without `unsafe`. `"reason"` is a doc fragment in the crate's
///   `# Errors` style. Omit the `{ check, "reason" } => ErrType` tail for a plain cast.
/// - `[unsafe: "safety"] name: InputType => StoredType; "comment"`: the same cast without the
///   per-element check, for a vector the C side later trusts as an unchecked index, count or
///   length. `"safety"` becomes the caller's `# Safety` obligation.
macro_rules! vec_set {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                // SAFETY: self.$name is a valid pointer for the lifetime of self.
                unsafe { [<write_mjs_vec_ $type>](value, self.ffi().$name) };
            }
        )*
    }};

    ($($([$unsafe_kw:ident : $safety:literal])? $name:ident: $input_type:ty => $type:ty $({$check:expr , $reason:literal} => $err:ty)?; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Set ", $comment
                $(, "\n\n# Errors\nReturns ", $reason, " (in that case nothing is written).")?
                $(, "\n\n# Safety\n", $safety)?
            )]
            pub $($unsafe_kw)? fn [<set_ $name>](&mut self, value: &[$input_type]) $(-> Result<(), $err>)? {
                $(for &v in value { ($check)(v)?; })?
                $crate::util::assert_ptr_cast_valid::<$input_type, $type>(value.as_ptr());
                // SAFETY: the assert proves the types are layout-compatible, and the `$check` loop
                // or the caller's `# Safety` contract covers the value range that C relies on.
                let raw = unsafe { std::slice::from_raw_parts(value.as_ptr().cast(), value.len()) };
                unsafe { [<write_mjs_vec_ $type>](raw, self.ffi().$name) };
                $(Ok::<(), $err>(()))?
            }
        )*
    }};
}

/// Implements appenders for non-string attributes of a vector of vectors.
macro_rules! vec_vec_append {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Append to ", $comment)]
            pub fn [<append_ $name>](&mut self, value: &[$type]) {
                // SAFETY: self.$name is a valid pointer for the lifetime of self.
                unsafe { [<append_mjs_vec_vec_ $type>](value, self.ffi().$name) };
            }

            #[doc = concat!("Set ", $comment, " (deprecated; use ", stringify!([<append_ $name>]), " instead).")]
            #[deprecated(note = "use append_ instead of set_ for vector-of-vectors attributes", since = "3.0.0")]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                self.[<append_ $name>](value);
            }
        )*
    }};
}

/// Generates methods for obtaining iterators to `$iter_over` spec items.
macro_rules! spec_get_iter {
    ($($iter_over: ident),*) => {paste::paste!{
        $(
            #[doc = concat!("Return an iterator over ", stringify!($iter_over)," items that allows modifying each value.")]
            pub fn [<$iter_over _iter_mut>](&mut self) -> MjsSpecItemIterMut<'_, [<Mjs $iter_over:camel>]> {
                MjsSpecItemIterMut::<[<Mjs $iter_over:camel>]>::new(self)
            }

            #[doc = concat!("Return an immutable iterator over ", stringify!($iter_over)," items.")]
            pub fn [<$iter_over _iter>](&self) -> MjsSpecItemIter<'_, [<Mjs $iter_over:camel>]> {
                MjsSpecItemIter::<[<Mjs $iter_over:camel>]>::new(self)
            }
        )*
    }};
}


/// Generates methods for obtaining iterators to `$iter_over` body items.
macro_rules! body_get_iter {
    ([$($iter_over: ident),*]) => {paste::paste!{
        $(
            #[doc = concat!("Return an iterator over ", stringify!($iter_over)," items that allows modifying each value.")]
            pub fn [<$iter_over _iter_mut>](&mut self, recurse: bool) -> MjsBodyItemIterMut<'_, [<Mjs $iter_over:camel>]> {
                MjsBodyItemIterMut::<[<Mjs $iter_over:camel>]>::new(self, recurse)
            }

            #[doc = concat!("Return an immutable iterator over ", stringify!($iter_over)," items.")]
            pub fn [<$iter_over _iter>](&self, recurse: bool) -> MjsBodyItemIter<'_, [<Mjs $iter_over:camel>]> {
                MjsBodyItemIter::<[<Mjs $iter_over:camel>]>::new(self, recurse)
            }
        )*
    }};
}
