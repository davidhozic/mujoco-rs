//! Trait definitions for model editing.
use std::io::{Error, ErrorKind};
use std::ffi::{CStr, CString};

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
    /// # Panics
    /// When the `name` contains '\0' characters mid string, a panic occurs.
    fn set_name(&mut self, name: &str) {
        let cstr = CString::new(name).unwrap();  // always has valid UTF-8
        unsafe { mjs_setName(self.element_mut_pointer(), cstr.as_ptr()) };
    }

    /// Builder style set a new name.
    fn with_name(&mut self, name: &str) -> &mut Self {
        self.set_name(name);
        self
    }

    /// Returns the used default.
    fn default(&self) -> &MjsDefault {
        unsafe { mjs_getDefault(self.element_pointer()).as_ref().unwrap() }
    }

    /// Make the item inherit properties from a default class.
    /// # Errors
    /// Returns a [`ErrorKind::NotFound`] when the default with the `class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` contains '\0' characters, a panic occurs.
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

    /// Builder style make the item inherit from a default class.
    fn with_default(&mut self, class_name: &str) -> Result<&mut Self, Error> {
        self.set_default(class_name)?;
        Ok(self)
    }

    /// Delete the item.
    /// # Safety
    /// Since this method can't consume variables holding pointers, nor can we consume the
    /// actual struct, this accepts a mutable reference to the item.
    /// Consequently, the compiler still allows the original reference to be used, which
    /// should be considered deallocated. Using the item after deleting it is in this case **use-after-free**!.
    unsafe fn delete(&mut self) -> Result<(), Error> {
        unsafe { self.__delete_default__() }
    }

    /// Default implementation of the delete method.
    /// Override [`SpecItem::delete`] for custom deletion logic.
    /// # Safety
    /// Since this method can't consume variables holding pointers, nor can we consume the
    /// actual struct, this accepts a mutable reference to the item.
    /// Consequently, the compiler still allows the original reference to be used, which
    /// should be considered deallocated. Using the item after deleting it is in this case **use-after-free**!.
    unsafe fn __delete_default__(&mut self) -> Result<(), Error> {
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => Err(Error::new(ErrorKind::Other, unsafe { CStr::from_ptr(mjs_getError(spec)).to_string_lossy().into_owned() }))
        }
    }
}
