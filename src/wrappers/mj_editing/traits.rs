//! Trait definitions for model editing.
use std::ffi::{CStr, CString};

use crate::error::MjEditError;
use crate::mujoco_c::*;

use super::default::MjsDefault;
use super::utility::*;

pub(crate) mod sealed {
    /// Prevents external implementations of [`SpecItem`](super::SpecItem).
    pub trait Sealed {}
}

/// Represents all the types that [`MjSpec`](super::MjSpec) supports.
/// This is pre-implemented for all the specification types and is not
/// meant to be implemented by the user.
pub trait SpecItem: Sized + sealed::Sealed {
    /// Returns the internal element struct.
    /// The element struct is the C++ implementation of the
    /// actual item, which is hidden from the user, but is needed
    /// in some functions.
    /// 
    /// # Safety
    /// This borrows immutably, but returns a mutable pointer. This is done to overcome MJS's wrong
    /// use of mutable pointers in functions, such as [`mjs_getName`].
    unsafe fn element_pointer(&self) -> *mut mjsElement;

    /// Same as [`SpecItem::element_pointer`], but with a mutable borrow.
    ///
    /// # Safety
    /// See [`SpecItem::element_pointer`].
    unsafe fn element_mut_pointer(&mut self) -> *mut mjsElement {
        // SAFETY: self.element is a valid non-null pointer to the C spec element
        // for the lifetime of the parent MjSpec (struct invariant).
        unsafe { self.element_pointer() }
    }

    /// Returns the item's name.
    ///
    /// # Panics
    /// Panics if the stored MuJoCo string is not valid UTF-8.
    fn name(&self) -> &str {
        // SAFETY: mjs_getName returns a pointer to a null-terminated string owned
        // by the spec element, valid for the element's lifetime.
        unsafe { read_mjs_string(mjs_getName(self.element_pointer())) }
    }

    /// Set a new name.
    /// # Errors
    /// Returns [`MjEditError::AlreadyExists`] when an element with the same name already exists.
    /// # Panics
    /// When the `name` contains '\0' characters mid string, a panic occurs.
    fn set_name(&mut self, name: &str) -> Result<(), MjEditError> {
        let cstr = CString::new(name).unwrap();  // panics on interior NUL bytes; &str guarantees UTF-8
        let result = unsafe { mjs_setName(self.element_mut_pointer(), cstr.as_ptr()) };
        if result != 0 {
            return Err(MjEditError::AlreadyExists);
        }
        Ok(())
    }

    /// Builder style set a new name.
    /// # Panics
    /// Panics when an element with the same name already exists, or when `name` contains '\0'.
    fn with_name(&mut self, name: &str) -> &mut Self {
        self.set_name(name).expect("mjs_setName failed: duplicate name or null byte");
        self
    }

    /// Returns the used default.
    fn default(&self) -> &MjsDefault {
        // SAFETY: mjs_getDefault indexes into mjCModel::def_map which always
        // contains the element's classname (inserted at construction), so the
        // returned pointer is never null.
        unsafe { &*mjs_getDefault(self.element_pointer()) }
    }

    /// Make the item inherit properties from a default class.
    /// # Errors
    /// Returns [`MjEditError::NotFound`] when the default with the `class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` contains '\0' characters, a panic occurs.
    fn set_default(&mut self, class_name: &str) -> Result<(), MjEditError> {
        /* Workaround to pass the borrow checker (we use the existing borrow) */
        let cname = CString::new(class_name).unwrap();  // class_name is always valid UTF-8.
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let default = unsafe { mjs_findDefault(spec, cname.as_ptr()) };
        if default.is_null() {
            return Err(MjEditError::NotFound);
        }

        unsafe { mjs_setDefault(self.element_mut_pointer(), default); }
        Ok(())
    }

    /// Builder style make the item inherit from a default class.
    /// # Errors
    /// Returns [`MjEditError::NotFound`] when the default with the `class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` contains '\0' characters, a panic occurs.
    fn with_default(&mut self, class_name: &str) -> Result<&mut Self, MjEditError> {
        self.set_default(class_name)?;
        Ok(self)
    }

    /// Delete the item.
    ///
    /// This method must be called **at most once** per item. After a successful deletion
    /// the underlying C element is freed by MuJoCo; any further use of `self` -- including
    /// calling `delete` again or reading any field -- is **use-after-free** undefined behavior.
    /// If the call returns `Err`, no memory is freed and the item remains valid.
    ///
    /// # Errors
    /// - [`MjEditError::DeleteFailed`] if MuJoCo cannot delete the element.
    /// - [`MjEditError::UnsupportedOperation`] if the element cannot be deleted
    ///   (e.g. the world body or default classes).
    ///
    /// # Safety
    /// The `&mut self` receiver prevents aliased mutable access at the call site, but the
    /// Rust compiler cannot prevent the caller from retaining other references (shared or
    /// mutable) that were obtained before this call. The caller must guarantee that no such
    /// references remain live after a successful return, as the underlying C memory will
    /// have been freed. Violating this invariant is **use-after-free** undefined behavior.
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        unsafe { self.__delete_default__() }
    }

    /// Default implementation of the delete method.
    /// Override [`SpecItem::delete`] for custom deletion logic.
    ///
    /// # Errors
    /// Returns [`MjEditError::DeleteFailed`] if MuJoCo's internal deletion fails.
    ///
    /// # Safety
    /// Same contract as [`SpecItem::delete`]: must be called at most once per item; any use
    /// of `self` after a successful call is **use-after-free** undefined behavior.
    unsafe fn __delete_default__(&mut self) -> Result<(), MjEditError> {
        // SAFETY: element_mut_pointer() is valid (struct invariant); mjs_getSpec
        // returns the owning spec, also valid.
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => {
                let error_msg: String = unsafe {
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
}
