//! Module implements [`MjsDefault`], which is a special type of [`SpecItem`].

use std::marker::PhantomData;
use std::io::Error;

use super::traits::SpecItem;
use crate::mujoco_c::*;


// This is implemented manually since we can't directly borrow check if something is using the default.
// We also override the delete method to panic instead of deleting.

/// Default specification. This wraps the FFI type [`mjsDefault`] internally.
pub struct MjsDefault<'s>(pub(crate) *mut mjsDefault, pub(crate) PhantomData<&'s mut ()>);  // the lifetime belongs to the parent

impl MjsDefault<'_> {
    /// Returns an immutable reference to the inner struct.
    pub fn ffi(&self) -> &mjsDefault {
        unsafe { self.0.as_ref().unwrap() }
    }

    /// Returns a mutable reference to the inner struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjsDefault {
        unsafe { self.0.as_mut().unwrap() }
    }
}

impl SpecItem for MjsDefault<'_> {
    unsafe fn element_pointer(&self) -> *mut mjsElement {
        self.ffi().element
    }

    /// Defaults can't be deleted.
    fn delete(self) -> Result<(), Error> {
        unimplemented!()
    }
}

// SAFETY: These are safe to implement, as access to them is available only
// through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
unsafe impl Sync for MjsDefault<'_> {}
unsafe impl Send for MjsDefault<'_> {}
