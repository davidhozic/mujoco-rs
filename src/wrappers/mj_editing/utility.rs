//! Utilities for model editing purposes.
use std::ffi::{CStr, CString};
use crate::mujoco_c::*;


/***************************
** Utility functions
***************************/
/// Reads MJS string (C++) as a `&str`.
///
/// The returned `&str` borrows from the `mjString` object pointed to by `string`.
/// It remains valid as long as that object is alive and the string is not mutated
/// (which would reallocate the internal C++ `std::string` buffer).
///
/// # Safety
/// `string` must point to a valid `mjString` object for the duration `'a`.
///
/// # Panics
/// Panics if the string contains invalid UTF-8.
pub(crate) unsafe fn read_mjs_string<'a>(string: *const mjString) -> &'a str {
    let ptr = unsafe { mjs_getString(string) };
    if ptr.is_null() {
        ""
    } else {
        // SAFETY: `ptr` points into the internal buffer of the C++ std::string
        // referenced by `string`, which is valid for lifetime 'a. MuJoCo
        // strings are always valid UTF-8 (ASCII), so to_str() cannot fail.
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
pub(crate) unsafe fn write_mjs_vec_f64(source: &[f64], destination: *mut mjDoubleVec) {
    unsafe {
        mjs_setDouble(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes MJS float vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjFloatVec` object.
pub(crate) unsafe fn write_mjs_vec_f32(source: &[f32], destination: *mut mjFloatVec) {
    unsafe {
        mjs_setFloat(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends MJS float vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjFloatVecVec` object.
pub(crate) unsafe fn append_mjs_vec_vec_f32(source: &[f32], destination: *mut mjFloatVecVec) {
    unsafe {
        mjs_appendFloatVec(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes MJS int vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjIntVec` object.
pub(crate) unsafe fn write_mjs_vec_i32(source: &[i32], destination: *mut mjIntVec) {
    unsafe {
        mjs_setInt(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends MJS int vector (C++) from a `source` to `destination`.
///
/// # Safety
/// `destination` must point to a valid `mjIntVecVec` object.
pub(crate) unsafe fn append_mjs_vec_vec_i32(source: &[i32], destination: *mut mjIntVecVec) {
    unsafe {
        mjs_appendIntVec(destination, source.as_ptr(), source.len() as i32);
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

/// Split `source` to entries and append to `destination` (C++).
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
pub(crate) unsafe fn write_mjs_vec_byte<T: bytemuck::NoUninit>(source: &[T], destination: *mut mjByteVec) {
    let bytes: &[u8] = bytemuck::cast_slice(source);
    unsafe {
        mjs_setBuffer(destination, bytes.as_ptr().cast(), bytes.len() as i32);
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
                "Delegates to [`Self::try_add_", stringify!($name), "`] and panics if allocation fails.\n",
                "# Panics\n",
                "Panics if MuJoCo fails to allocate the element."
            )]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!($name:camel), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate ",
                "the element, instead of panicking."
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                let ptr = unsafe { [<mjs_add $name:camel>](self.ffi_mut(), ptr::null()) };
                // SAFETY: ptr.as_mut() returns None for null, handled by ok_or; when non-null
                // the pointee is properly aligned and initialized by C++ operator new.
                unsafe { ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
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
                "Delegates to [`Self::try_add_", stringify!($name), "`] and panics on failure.\n",
                "# Panics\n",
                "Panics if MuJoCo fails to allocate the element."
            )]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!($name:camel), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate the element."
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                // SAFETY:
                // - element_mut_pointer() reads `self.element`, a field always valid after construction.
                // - body_ptr is non-null for any MjsFrame reachable through the Rust API because
                //   mjs_addFrame always calls SetParent(body); the debug_assert catches violations.
                // - The is_null() guard is defensive; mjs_addXxx functions do not perform
                //   null-check error handling internally, so under current MuJoCo the
                //   pointer is always non-null.
                // - ptr.cast() is safe: mjs structs embed mjsElement as their first field, so
                //   *mut mjsXxx and *mut mjsElement share the same address.
                // - mjs_setFrame: both dest and frame are non-null and valid; failure for a
                //   freshly-created element is treated as a bug via debug_assert.
                // - `&mut *ptr`: ptr is confirmed non-null by the guard above, properly aligned
                //   and initialized by C++ operator new, and freshly allocated so no Rust
                //   reference can alias it for the returned lifetime.
                unsafe {
                    let ep = self.element_mut_pointer();
                    let body_ptr = mjs_getParent(ep);
                    debug_assert!(!body_ptr.is_null(), "mjs_getParent returned null; frame has no parent body");
                    let ptr = [<mjs_add $name:camel>](body_ptr, ptr::null());
                    if ptr.is_null() {
                        return Err(MjEditError::AllocationFailed);
                    }
                    let set_result = mjs_setFrame((*ptr).element, self);
                    debug_assert_eq!(set_result, 0, "mjs_setFrame failed; element or frame is invalid");
                    Ok(&mut *ptr)
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
                "Delegates to [`Self::try_add_", stringify!($name), "`] and panics if allocation fails.\n",
                "# Panics\n",
                "Panics if MuJoCo fails to allocate the element."
            )]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                self.[<try_add_ $name>]()
                    .expect(concat!("mjs_add", stringify!($name:camel), " returned null; allocation failed"))
            }

            #[doc = concat!(
                "Fallible version of [`Self::add_", stringify!($name), "`].\n\n",
                "# Errors\n",
                "Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate ",
                "the element, instead of panicking."
            )]
            pub fn [<try_add_ $name>](&mut self) -> Result<&mut [<Mjs $name:camel>], MjEditError> {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0.as_ptr()) };
                unsafe { ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
            }
        )*
    }};
}


/// Creates a `get_$name` method for finding items in spec / body.
/// Also sets the default to null();
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
                    let ptr = mjs_findElement(self.0.as_ptr(), MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<mjs_as $item:camel>](ptr).as_ref()
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
                    let ptr = mjs_findElement(self.0.as_ptr(), MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        [<mjs_as $item:camel>](ptr).as_mut()
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
                    let ptr = [<mjs_find $item:camel>](self.0.as_ptr(), c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        ptr.as_ref()
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
                    let ptr = [<mjs_find $item:camel>](self.0.as_ptr(), c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        ptr.as_mut()
                    }
                }
            }
        )*
    }};
}


/// Creates a wrapper around a mjs$ffi_name item. It also implements the methods: `ffi()`, `ffi_mut()`
/// and traits: [`SpecItem`](super::traits::SpecItem), [`Sync`], [`Send`].
macro_rules! mjs_struct {
    ($ffi_name:ident $({ $($extra_trait_methods:tt)* })?) => {paste::paste!{
        #[doc = concat!(stringify!($ffi_name), " specification. This is an alias to the FFI type [`", stringify!([<mjs $ffi_name>]), "`].")]
        pub type [<Mjs $ffi_name>] = [<mjs $ffi_name>];

        impl [<Mjs $ffi_name>] {
            /// Return the message appended to compiler errors.
            /// # Panics
            /// Panics if it contains invalid UTF-8.
            pub fn info(&self) -> &str {
                // SAFETY: self.info is a valid mjString pointer for the lifetime of self.
                unsafe { read_mjs_string(self.info) }
            }

            /// Set the message appended to compiler errors.
            /// # Panics
            /// When the `info` contains '\0' characters, a panic occurs.
            pub fn set_info(&mut self, info: &str) {
                // SAFETY: self.info is a valid mjString pointer for the lifetime of self.
                unsafe { write_mjs_string(info, self.info) };
            }
        }

        impl SpecItem for [<Mjs $ffi_name>] {
            unsafe fn element_pointer(&self) -> *mut mjsElement {
                self.element
            }

            $($(
                $extra_trait_methods
            )*)?
        }

        // SAFETY: These are safe to implement, as access to them is available only
        // through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
        unsafe impl Sync for [<Mjs $ffi_name>] {}
        unsafe impl Send for [<Mjs $ffi_name>] {}
    }};
}


/// Implements the userdata method.
macro_rules! userdata_method {
    ($type:ty) => {paste::paste!{
        /// Return an immutable slice to userdata.
        pub fn userdata(&self) -> &[$type] {
            // SAFETY: self.userdata is a valid mjDoubleVec pointer for the lifetime of self.
            unsafe { [<read_mjs_vec_ $type>](self.userdata) }
        }
        
        /// Set `userdata`.
        pub fn set_userdata<T: AsRef<[$type]>>(&mut self, value: T) {
            // SAFETY: self.userdata is a valid pointer for the lifetime of self.
            unsafe { [<write_mjs_vec_ $type>](value.as_ref(), self.userdata) };
        }

        /// Builder method for setting `userdata`.
        pub fn with_userdata<T: AsRef<[$type]>>(&mut self, value: T) -> &mut Self {
            // SAFETY: self.userdata is a valid pointer for the lifetime of self.
            unsafe { [<write_mjs_vec_ $type>](value.as_ref(), self.userdata) };
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
                unsafe { write_mjs_vec_string(value, self.$name) };
            }

            #[doc = concat!(
                "Splits the `", stringify!($name), "` and append the split text to ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<append_ $name>](&mut self, value: &str) {
                // SAFETY: self.$name is a valid mjStringVec pointer for the lifetime of self.
                unsafe { append_mjs_vec_string(value, self.$name) };
            }
        )*
    }};
}

/// Implements string methods for given attribute $name.
macro_rules! string_set_get_with {
    (@impl common $([$ffi:ident, $ffi_mut:ident])? $name:ident; $comment:expr;) => {paste::paste!{
        #[allow(unused_unsafe)]
        #[doc = concat!("Return ", $comment)]
        pub fn $name(&self) -> &str {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { read_mjs_string(self$(.$ffi())?.$name) }
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
            unsafe { write_mjs_string(value, unsafe { self$(.$ffi_mut())?.$name }) };
        }
    }};

    ( $($([$ffi:ident, $ffi_mut:ident])? $name:ident; $comment:expr;)* ) => {paste::paste!{
        $(
            string_set_get_with!(@impl common $([$ffi, $ffi_mut])? $name; $comment;);
            #[allow(unused_unsafe)]
            #[doc = concat!(
                "Builder method for setting ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<with_ $name>](mut self, value: &str) -> Self {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { write_mjs_string(value, unsafe { self$(.$ffi_mut())?.$name }) };
                self
            }
        )*
    }};

    ([&] $($([$ffi:ident, $ffi_mut:ident])? $name:ident; $comment:expr;)* ) => {paste::paste!{
        $(
            string_set_get_with!(@impl common $([$ffi, $ffi_mut])? $name; $comment;);
            #[allow(unused_unsafe)]
            #[doc = concat!(
                "Builder method for setting ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<with_ $name>](&mut self, value: &str) -> &mut Self {
                // SAFETY: the mjString field is valid for the lifetime of self.
                unsafe { write_mjs_string(value, unsafe { self$(.$ffi_mut())?.$name }) };
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
                unsafe { [<read_mjs_vec_ $type>](self.$name) }
            }
        )*

        vec_set!($($name: $type; $comment);*);
    }};
}

/// Implements setters for non-string attributes.
macro_rules! vec_set {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                // SAFETY: self.$name is a valid pointer for the lifetime of self.
                unsafe { [<write_mjs_vec_ $type>](value, self.$name) };
            }
        )*
    }};
}

/// Implements appenders for non-string attributes of a double vec.
macro_rules! vec_vec_append {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                // SAFETY: self.$name is a valid pointer for the lifetime of self.
                unsafe { [<append_mjs_vec_vec_ $type>](value, self.$name) };
            }
        )*
    }};
}

/// Implements iterators for individual items in [MjSpec](super::MjSpec).
macro_rules! item_spec_iterator {
    ($($iter_over: ident),*) => {paste::paste!{
        $(
            impl<'a> MjsSpecItemIterMut<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a mut MjSpec) -> Self {
                    let last = unsafe { mjs_firstElement(root.0.as_ptr(), MjtObj::[<mjOBJ_ $iter_over:upper>]) };
                    Self { root, last, item_type: PhantomData }
                }
            }

            impl<'a> MjsSpecItemIter<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a MjSpec) -> Self {
                    let last = unsafe { mjs_firstElement(root.0.as_ptr(), MjtObj::[<mjOBJ_ $iter_over:upper>]) };
                    Self { root, last, item_type: PhantomData }
                }
            }

            impl<'a> Iterator for MjsSpecItemIterMut<'a, [<Mjs $iter_over>]> {
                type Item = &'a mut [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    unsafe {
                        let out = [<mjs_as $iter_over>](self.last).as_mut();
                        // Use as_ptr() instead of ffi_mut() to avoid creating &mut mjSpec,
                        // which would alias with previously yielded &mut items.
                        self.last = mjs_nextElement(self.root.0.as_ptr(), self.last);
                        out
                    }
                }
            }

            impl<'a> Iterator for MjsSpecItemIter<'a, [<Mjs $iter_over>]> {
                type Item = &'a [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    unsafe {
                        let out = [<mjs_as $iter_over>](self.last).as_ref();
                        self.last = mjs_nextElement(self.root.0.as_ptr(), self.last);
                        out
                    }
                }
            }

            // Once self.last is null, next() always returns None.
            impl<'a> std::iter::FusedIterator for MjsSpecItemIterMut<'a, [<Mjs $iter_over>]> {}
            impl<'a> std::iter::FusedIterator for MjsSpecItemIter<'a, [<Mjs $iter_over>]> {}
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


/// Implements iterators for individual items in [MjsBody](super::MjsBody).
macro_rules! item_body_iterator {
    ($($iter_over: ident),*) => {paste::paste!{
        $(
            impl<'a> MjsBodyItemIterMut<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a mut MjsBody, recurse: bool) -> Self {
                    let last = unsafe { mjs_firstChild(root, MjtObj::[<mjOBJ_ $iter_over:upper>], recurse.into()) };
                    Self { root, last, recurse, item_type: PhantomData }
                }
            }

            impl<'a> MjsBodyItemIter<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a MjsBody, recurse: bool) -> Self {
                    // SAFETY: mjs_firstChild requires a *mut pointer but does not mutate
                    // the body. The const-to-mut cast is sound because no mutation occurs.
                    let last = unsafe { mjs_firstChild(root as *const _ as *mut _, MjtObj::[<mjOBJ_ $iter_over:upper>], recurse.into()) };
                    Self { root, last, recurse, item_type: PhantomData }
                }
            }

            impl<'a> Iterator for MjsBodyItemIterMut<'a, [<Mjs $iter_over>]> {
                type Item = &'a mut [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    unsafe {
                        let out = [<mjs_as $iter_over>](self.last).as_mut();
                        self.last = mjs_nextChild(self.root, self.last, self.recurse.into());
                        out
                    }
                }
            }

            impl<'a> Iterator for MjsBodyItemIter<'a, [<Mjs $iter_over>]> {
                type Item = &'a [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    unsafe {
                        let out = [<mjs_as $iter_over>](self.last).as_ref();
                        // SAFETY: mjs_nextChild requires *mut but does not mutate. Cast is sound.
                        self.last = mjs_nextChild(self.root as *const _ as *mut _, self.last, self.recurse.into());
                        out
                    }
                }
            }

            // Once self.last is null, next() always returns None.
            impl<'a> std::iter::FusedIterator for MjsBodyItemIterMut<'a, [<Mjs $iter_over>]> {}
            impl<'a> std::iter::FusedIterator for MjsBodyItemIter<'a, [<Mjs $iter_over>]> {}
        )*
    }};
}

/// Generates methods for obtaining iterators to `$iter_over` body items.
/// The $self_lf represents the iterated item's borrow and $parent_lf the lifetime of its parent.
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
