//! Trait definitions for model editing.
use std::io::{Error, ErrorKind};
use std::ffi::{CStr, CString};
use std::marker::PhantomData;

use crate::mujoco_c::*;

use super::default::MjsDefault;
use super::utility::*;


/// Represents all the types that [`MjSpec`](super::MjSpec) supports.
/// This is pre-implemented for all the specification types and is not
/// meant to be implemented by the user.
pub trait SpecItem: Sized {
    /// Returns the internal element struct.
    /// The element struct is the C++ implementation of the
    /// actual item, which is hidden to the user, but is needed
    /// in some functions.
    /// 
    /// # SAFETY
    /// This borrows immutably, but returns a mutable pointer. This is done to overcome MJS's wrong
    /// use of mutable pointers in functions, such as [`mjs_getName`].
    unsafe fn element_pointer(&self) -> *mut mjsElement;

    /// Same as [`SpecItem::element_pointer`], but with a mutable borrow.
    unsafe fn element_mut_pointer(&mut self) -> *mut mjsElement {
        unsafe { self.element_pointer() }
    }

    /// Returns the item's name.
    fn name(&self) -> &str {
        read_mjs_string( unsafe { mjs_getName(self.element_pointer()).as_ref().unwrap() } )
    }

    /// Set a new name.
    fn set_name(&mut self, name: &str) {
        let cstr = CString::new(name).unwrap();  // always has valid UTF-8
        unsafe { mjs_setName(self.element_mut_pointer(), cstr.as_ptr()) };
    }

    /// Returns the used default.
    fn default(&self) -> MjsDefault<'_> {
        MjsDefault(unsafe { mjs_getDefault(self.element_pointer()) }, PhantomData)
    }

    /// Make the item inherit properties from `default`.
    /// # Errors
    /// Returns a [`ErrorKind::NotFound`] when the default with the `class_name` doesn't exist.
    fn set_default(&mut self, class_name: &str) -> Result<(), Error> {
        /* Workaround to pass the borrow checker (we use the existing borrow) */
        let cname = CString::new(class_name).unwrap();  // class_name is always valid UTF-8.
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let default = unsafe { mjs_findDefault(spec, cname.as_ptr()) };
        if default.is_null() {
            return Err(Error::new(ErrorKind::NotFound, "class doesn't exist"));
        }

        unsafe { mjs_setDefault(self.element_mut_pointer(), default); }
        Ok(())
    }

    /// Delete the item.
    fn delete(mut self) -> Result<(), Error> {
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => Err(Error::new(ErrorKind::Other, unsafe { CStr::from_ptr(mjs_getError(spec)).to_string_lossy().into_owned() }))
        }
    }
}
