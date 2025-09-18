//! Utilities for model editing purposes.
use std::ffi::{CStr, CString};
use std::mem::size_of;
use crate::mujoco_c::*;


/***************************
** Utility functions
***************************/
/// Reads a MJS string (C++) as a `&str`.
pub(crate) fn read_mjs_string(string: &mjString) -> &str {
    unsafe {
        let ptr = mjs_getString(string);
        CStr::from_ptr(ptr).to_str().unwrap()  // can't be invalid UTF-8.
    }
}

/// Writes to a `destination` MJS string (C++) from a `source` `&str`.
pub(crate) fn write_mjs_string(source: &str, destination: &mut mjString) {
    unsafe {
        let c_source = CString::new(source).unwrap();  // can't be invalid UTF-8.
        mjs_setString(destination, c_source.as_ptr());
    }
}

/// Reads as MJS double vector (C++) as a `&\[f64\]`.
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

/// Writes as MJS double vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_f64(source: &[f64], destination: &mut mjDoubleVec) {
    unsafe {
        mjs_setDouble(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes as MJS float vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_f32(source: &[f32], destination: &mut mjFloatVec) {
    unsafe {
        mjs_setFloat(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends as MJS float vector (C++) from a `source` to `destination`.
pub(crate) fn append_mjs_vec_vec_f32(source: &[f32], destination: &mut mjFloatVecVec) {
    unsafe {
        mjs_appendFloatVec(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Writes as MJS int vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_i32(source: &[i32], destination: &mut mjIntVec) {
    unsafe {
        mjs_setInt(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Appends as MJS int vector (C++) from a `source` to `destination`.
pub(crate) fn append_mjs_vec_vec_i32(source: &[i32], destination: &mut mjIntVecVec) {
    unsafe {
        mjs_appendIntVec(destination, source.as_ptr(), source.len() as i32);
    }
}

/// Split `source` to entries and copy to `destination` (C++).
pub(crate) fn write_mjs_vec_string(source: &str, destination: &mut mjStringVec) {
    let c_source = CString::new(source).unwrap();  // can't be invalid UTF-8.
    unsafe {
        mjs_setStringVec(destination, c_source.as_ptr());
    }
}

/// Split `source` to entries and append to `destination` (C++).
pub(crate) fn append_mjs_vec_string(source: &str, destination: &mut mjStringVec) {
    let c_source = CString::new(source).unwrap();  // can't be invalid UTF-8.
    unsafe {
        mjs_appendString(destination, c_source.as_ptr());
    }
}

// /// Writes as MJS byte vector (C++) from a `source` to `destination`.
pub(crate) fn write_mjs_vec_byte<T>(source: &[T], destination: &mut mjByteVec) {
    unsafe {
        mjs_setBuffer(destination, source.as_ptr().cast(), (size_of::<T>() * source.len()) as i32);
    }
}


/***************************
** Helper macros
***************************/
/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// Also sets the default as null();
macro_rules! add_x_method {
    ($($name:ident),*) => {paste::paste! {
        $(
            /* Without default */
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0, ptr::null()) };
                [<Mjs $name:camel>](ptr, PhantomData)
            }
        )*
    }};
}

/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// This is for methods that don't accept a default.
macro_rules! add_x_method_no_default {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0) };
                [<Mjs $name:camel>](ptr, PhantomData)
            }
        )*
    }};
}


/// Creates an `get_ $name` method for finding items in spec / body.
/// Also sets the default as null();
macro_rules! find_x_method {
    ($($item:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Obtain a reference to the ", stringify!($item), " with the given `name`.")]
            pub fn $item(&self, name: &str) -> Option<[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = mjs_findElement(self.0, MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        Some([<Mjs $item:camel>]([<mjs_as $item:camel>](ptr), PhantomData))
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
            #[doc = concat!("Obtain a reference to the ", stringify!($item), " with the given `name`.")]
            pub fn $item(&self, name: &str) -> Option<[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = [<mjs_find $item:camel>](self.0, c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        Some([<Mjs $item:camel>](ptr, PhantomData))
                    }
                }
            }
        )*
    }};
}


/// Creates a wrapper around a mjs$ffi_name item. It also implements the methods: `ffi()`, `ffi_mut()`
/// and traits: [`SpecItem`](super::traits::SpecItem), [`Sync`], [`Send`].
macro_rules! mjs_wrapper {
    ($ffi_name:ident) => {paste::paste!{
        #[doc = concat!(stringify!($ffi_name), " specification. This acts as a safe reference to a FFI type [`", stringify!([<mjs $ffi_name>]), "`] internally.")]
        pub struct [<Mjs $ffi_name>]<'s>(*mut [<mjs $ffi_name>], PhantomData<&'s mut ()>);  // the lifetime belongs to the parent

        impl [<Mjs $ffi_name>]<'_> {
            /// Returns an immutable reference to the inner struct.
            pub fn ffi(&self) -> &[<mjs $ffi_name>] {
                unsafe { self.0.as_ref().unwrap() }
            }

            /// Returns a mutable reference to the inner struct.
            pub unsafe fn ffi_mut(&mut self) -> &mut [<mjs $ffi_name>] {
                unsafe { self.0.as_mut().unwrap() }
            }

            /// Returns the message appended to compiler errors.
            pub fn info(&self) {
                read_mjs_string(unsafe { self.ffi().info.as_ref().unwrap() });
            }

            /// Sets the message appended to compiler errors.
            pub fn set_info(&mut self, info: &str) {
                write_mjs_string(info, unsafe { self.ffi_mut().info.as_mut().unwrap() });
            }
        }

        impl SpecItem for [<Mjs $ffi_name>]<'_> {
            unsafe fn element_pointer(&self) -> *mut mjsElement {
                self.ffi().element
            }
        }

        // SAFETY: These are safe to implement, as access to them is available only
        // through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
        unsafe impl Sync for [<Mjs $ffi_name>]<'_> {}
        unsafe impl Send for [<Mjs $ffi_name>]<'_> {}
    }};
}


/// Implements the userdata method.
macro_rules! userdata_method {
    ($type:ty) => {paste::paste!{
        /// Returns an immutable slice to userdata.
        pub fn userdata(&self) -> &[$type] {
            [<read_mjs_vec_ $type>](unsafe { (*self.0).userdata.as_ref().unwrap() })
        }
        
        /// Sets `userdata`.
        pub fn set_userdata<T: AsRef<[$type]>>(&mut self, value: T) {
            [<write_mjs_vec_ $type>](value.as_ref(), unsafe {self.ffi_mut().userdata.as_mut().unwrap() })
        }
    }};
}

/// Implements vector of strings methods for given attribute $name.
macro_rules! vec_string_set_append {
    ($($name:ident; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Splits the `", stringify!($name), "` and put the split text as ", $comment, ".")]
            pub fn [<set_ $name>](&mut self, value: &str) {
                write_mjs_vec_string(value, unsafe { self.ffi_mut().$name.as_mut().unwrap() });
            }

            #[doc = concat!("Splits the `", stringify!($name), "` and append the split text to ", $comment, ".")]
            pub fn [<append_ $name>](&mut self, value: &str) {
                append_mjs_vec_string(value, unsafe { self.ffi_mut().$name.as_mut().unwrap() });
            }
        )*
    }};
}

/// Implements string methods for given attribute $name.
macro_rules! string_set_get {
    ($($name:ident; $comment:expr;)*) => {paste::paste!{
        $(
            #[doc = concat!("Returns ", $comment)]
            pub fn $name(&self) -> &str {
                read_mjs_string(unsafe { (*self.0).$name.as_ref().unwrap() })
            }

            #[doc = concat!("Sets ", $comment)]
            pub fn [<set_ $name>](&mut self, value: &str) {
                write_mjs_string(value, unsafe { self.ffi_mut().$name.as_mut().unwrap() })
            }
        )*
    }};
}

/// Implements getters and setters for floating point  (f32 or f64) attributes.
macro_rules! vec_set_get {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Returns `", $comment, "`.")]
            pub fn $name(&self) -> &[$type] {
                [<read_mjs_vec_ $type>](unsafe { (*self.0).$name.as_ref().unwrap() })
            }
        )*

        vec_set!($($name: $type; $comment);*);
    }};
}

/// Implements setters for floating point  (f32 or f64) attributes.
macro_rules! vec_set {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Sets `", $comment, "`.")]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                [<write_mjs_vec_ $type>](value, unsafe { self.ffi_mut().$name.as_mut().unwrap() })
            }
        )*
    }};
}

/// Implements appenders for floating point  (f32 or f64) attributes  of a double vec.
macro_rules! vec_vec_append {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {paste::paste!{
        $(
            #[doc = concat!("Sets `", $comment, "`.")]
            pub fn [<set_ $name>](&mut self, value: &[$type]) {
                [<append_mjs_vec_vec_ $type>](value, unsafe { self.ffi_mut().$name.as_mut().unwrap() })
            }
        )*
    }};
}

/// Implements the plugin wrapper.
macro_rules! plugin_wrapper_method {
    () => {
        /// Returns a wrapper around the `plugin` attribute.
        pub fn plugin_wrapper(&mut self) -> MjsPlugin<'_> {
            unsafe { MjsPlugin(&mut self.ffi_mut().plugin, PhantomData) }
        }
    };
}
