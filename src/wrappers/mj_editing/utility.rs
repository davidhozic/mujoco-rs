//! Utilities for model editing purposes.
use std::ffi::{CStr, CString};
use std::mem::size_of;
use crate::mujoco_c::*;


/***************************
** Utility functions
***************************/
/// Reads MJS string (C++) as a `&str`.
pub(crate) fn read_mjs_string(string: &mjString) -> &str {
    unsafe {
        let ptr = mjs_getString(string);
        CStr::from_ptr(ptr).to_str().unwrap()
    }
}

/// Writes to a `destination` MJS string (C++) from a `source` `&str`.
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) fn write_mjs_string(source: &str, destination: &mut mjString) {
    unsafe {
        let c_source = CString::new(source).unwrap();
        mjs_setString(destination, c_source.as_ptr());
    }
}

/// Reads MJS double vector (C++) as a `&\[f64\]`.
pub(crate) fn read_mjs_vec_f64(array: &mjDoubleVec) -> &[f64] {
    let mut userdata_length = 0;
    unsafe {
        let ptr_arr = mjs_getDouble(array, &mut userdata_length);
        if ptr_arr.is_null() {
            return &[];
        }

        std::slice::from_raw_parts(ptr_arr, userdata_length as usize)
    }
}

/// Writes MJS double vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_f64(source: &[f64], destination: &mut mjDoubleVec) {
    unsafe {
        mjs_setDouble(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes MJS float vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_f32(source: &[f32], destination: &mut mjFloatVec) {
    unsafe {
        mjs_setFloat(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends MJS float vector (C++) from a `source` to `destination`.
pub(crate) fn append_mjs_vec_vec_f32(source: &[f32], destination: &mut mjFloatVecVec) {
    unsafe {
        mjs_appendFloatVec(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes MJS int vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_i32(source: &[i32], destination: &mut mjIntVec) {
    unsafe {
        mjs_setInt(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends MJS int vector (C++) from a `source` to `destination`.
pub(crate) fn append_mjs_vec_vec_i32(source: &[i32], destination: &mut mjIntVecVec) {
    unsafe {
        mjs_appendIntVec(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Split `source` to entries and copy to `destination` (C++).
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) fn write_mjs_vec_string(source: &str, destination: &mut mjStringVec) {
    let c_source = CString::new(source).unwrap();
    unsafe {
        mjs_setStringVec(destination, c_source.as_ptr());
    }
}

/// Split `source` to entries and append to `destination` (C++).
/// # Panics
/// When the `source` contains '\0' characters, a panic occurs.
pub(crate) fn append_mjs_vec_string(source: &str, destination: &mut mjStringVec) {
    let c_source = CString::new(source).unwrap();
    unsafe {
        mjs_appendString(destination, c_source.as_ptr());
    }
}

/// Writes MJS byte vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_byte<T>(source: &[T], destination: &mut mjByteVec) {
    unsafe {
        mjs_setBuffer(destination, source.as_ptr().cast(), (size_of::<T>() * source.len()) as i32);
    }
}


/***************************
** Helper macros
***************************/
/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// Also sets the default to null();
macro_rules! add_x_method {
    ($($name:ident),*) => {paste::paste! {
        $(
            /* With default */
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.ffi_mut(), ptr::null()) };
                unsafe { ptr.as_mut().unwrap() }
            }
        )*
    }};
}

/// Creates an `add_$name` method for adding new elements into a frame.
macro_rules! add_x_method_by_frame {
    ($($name:ident),*) => {paste::paste! {
        $(
            /* With default */
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                unsafe {
                    let ep = self.element_mut_pointer();
                    let body_ptr = mjs_getParent(ep);
                    let ptr = [<mjs_add $name:camel>](body_ptr, ptr::null());
                    mjs_attach(ep, ptr.cast(), ptr::null(), ptr::null());
                    ptr.as_mut().unwrap()
                }
            }
        )*
    }};
}

/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// This is for methods that don't accept a default.
macro_rules! add_x_method_no_default {
    ($($name:ident),*) => {paste::paste! {
        $(
            /* Without default */
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> &mut [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0) };
                unsafe { ptr.as_mut().unwrap() }
            }
        )*
    }};
}


/// Creates an `get_ $name` method for finding items in spec / body.
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
                    let ptr = mjs_findElement(self.0, MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
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
                    let ptr = mjs_findElement(self.0, MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
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
                    let ptr = [<mjs_find $item:camel>](self.0, c_name.as_ptr());
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
                    let ptr = [<mjs_find $item:camel>](self.0, c_name.as_ptr());
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
    ($ffi_name:ident) => {paste::paste!{
        #[doc = concat!(stringify!($ffi_name), " specification. This is an alias to the FFI type [`", stringify!([<mjs $ffi_name>]), "`].")]
        pub type [<Mjs $ffi_name>] = [<mjs $ffi_name>];

        impl [<Mjs $ffi_name>] {
            /// Immutable proxy FFI method that returns self. Exists for interface reasons.

            /// Return the message appended to compiler errors.
            pub fn info(&self) -> &str {
                read_mjs_string(unsafe { self.info.as_ref().unwrap() })
            }

            /// Set the message appended to compiler errors.
            /// # Panics
            /// When the `info` contains '\0' characters, a panic occurs.
            pub fn set_info(&mut self, info: &str) {
                write_mjs_string(info, unsafe { self.info.as_mut().unwrap() });
            }
        }

        impl SpecItem for [<Mjs $ffi_name>] {
            unsafe fn element_pointer(&self) -> *mut mjsElement {
                self.element
            }
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
            [<read_mjs_vec_ $type>](unsafe { self.userdata.as_ref().unwrap() })
        }
        
        /// Set `userdata`.
        pub fn set_userdata<T: AsRef<[$type]>>(&mut self, value: T) {
            [<write_mjs_vec_ $type>](value.as_ref(), unsafe {self.userdata.as_mut().unwrap() })
        }

        /// Builder method for setting `userdata`.
        pub fn with_userdata<T: AsRef<[$type]>>(&mut self, value: T) -> &mut Self {
            [<write_mjs_vec_ $type>](value.as_ref(), unsafe {self.userdata.as_mut().unwrap() });
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
                write_mjs_vec_string(value, unsafe { self.$name.as_mut().unwrap() });
            }

            #[doc = concat!(
                "Splits the `", stringify!($name), "` and append the split text to ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<append_ $name>](&mut self, value: &str) {
                append_mjs_vec_string(value, unsafe { self.$name.as_mut().unwrap() });
            }
        )*
    }};
}

/// Implements string methods for given attribute $name.
macro_rules! string_set_get_with {
    ($($([$ffi:ident, $ffi_mut:ident])? $name:ident; $comment:expr;)*) => {paste::paste!{
        $(
            #[doc = concat!("Return ", $comment)]
            pub fn $name(&self) -> &str {
                read_mjs_string(unsafe { self$(.$ffi())?.$name.as_ref().unwrap() })
            }

            #[doc = concat!(
                "Set ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<set_ $name>](&mut self, value: &str) {
                write_mjs_string(value, unsafe { self$(.$ffi_mut())?.$name.as_mut().unwrap() })
            }

            #[doc = concat!(
                "Builder method for setting ", $comment,
                "\n",
                "# Panics\n",
                "When the `value` contains '\\0' characters, a panic occurs."
            )]
            pub fn [<with_ $name>](&mut self, value: &str) -> &mut Self {
                write_mjs_string(value, unsafe { self$(.$ffi_mut())?.$name.as_mut().unwrap() });
                self
            }
        )*
    }};
}

/// Implements getters and setters for floating point  (f32 or f64) attributes.
macro_rules! vec_set_get {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Return ", $comment)]
            pub fn $name(&self) -> &[$type] {
                [<read_mjs_vec_ $type>](unsafe { self.$name.as_ref().unwrap() })
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
                [<write_mjs_vec_ $type>](value, unsafe { self.$name.as_mut().unwrap() })
            }
        )*
    }};
}

/// Implements appenders for non-string attributes  of a double vec.
macro_rules! vec_vec_append {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                [<append_mjs_vec_vec_ $type>](value, unsafe { self.$name.as_mut().unwrap() })
            }
        )*
    }};
}

/// Implements the plugin wrapper.
macro_rules! plugin_wrapper_method {
    () => {
        /// Return a wrapper around the `plugin` attribute.
        pub fn plugin_wrapper(&mut self) -> &mut MjsPlugin {
            &mut self.plugin
        }
    };
}

/// Implements iterators for individual items in [MjSpec](super::MjSpec).
macro_rules! item_spec_iterator {
    ($($iter_over: ident),*) => {paste::paste!{
        $(
            impl<'a> MjsSpecItemIterMut<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a mut MjSpec) -> Self {
                    let last = unsafe { mjs_firstElement(root.0, MjtObj::[<mjOBJ_ $iter_over:upper>]) };
                    Self { root, last, item_type: PhantomData }
                }
            }

            impl<'a> MjsSpecItemIter<'a, [<Mjs $iter_over>]> {
                fn new(root: &'a MjSpec) -> Self {
                    let last = unsafe { mjs_firstElement(root.0, MjtObj::[<mjOBJ_ $iter_over:upper>]) };
                    Self { root, last, item_type: PhantomData }
                }
            }

            impl<'a> Iterator for MjsSpecItemIterMut<'a, [<Mjs $iter_over>]> {
                type Item = &'a mut [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    let out = unsafe { [<mjs_as $iter_over>](self.last).as_mut().unwrap() };
                    self.last = unsafe { mjs_nextElement(self.root.ffi_mut(), self.last) };
                    Some(out)
                }
            }

            impl<'a> Iterator for MjsSpecItemIter<'a, [<Mjs $iter_over>]> {
                type Item = &'a [<Mjs $iter_over>];

                fn next(&mut self) -> Option<Self::Item> {
                    if self.last.is_null() {
                        return None;
                    }

                    let out = unsafe { [<mjs_as $iter_over>](self.last).as_mut().unwrap() };
                    self.last = unsafe { mjs_nextElement(self.root.0, self.last) };
                    Some(out)
                }
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
                    // transmute here because mjs iterator functions require mutable pointers,
                    // even though they don't actually modify.
                    let last = unsafe { mjs_firstChild(std::mem::transmute(root), MjtObj::[<mjOBJ_ $iter_over:upper>], recurse.into()) };
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
                        // mjs_nextChild doesn't actually modify, but still demands a mutable pointer
                        self.last = mjs_nextChild(std::mem::transmute(self.root), self.last, self.recurse.into());
                        out
                    }
                }
            }
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
