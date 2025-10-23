//! Module implements [`MjsDefault`], which is a special type of [`SpecItem`].

use std::marker::PhantomData;
use std::io::{Error, ErrorKind};

use super::traits::SpecItem;
use crate::mujoco_c::*;
use crate::wrappers::mj_editing::{
    MjsJoint, MjsGeom, MjsSite, MjsCamera, MjsLight, MjsFlex, MjsMesh, MjsMaterial,
    MjsPair, MjsEquality, MjsTendon, MjsActuator
};


macro_rules! default_accessor_wrapper {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Returns a wrapper to  `", stringify!($name), "`.")]
            pub fn $name(&mut self) -> &[<Mjs $name:camel>] {
                unsafe { self.ffi_mut().$name.as_ref().unwrap() }
            }
        )*
    }};
}

// This is implemented manually since we can't directly borrow check if something is using the default.
// We also override the delete method to panic instead of deleting.

/// Default specification. This wraps the FFI type [`mjsDefault`] internally.
pub type MjsDefault = mjsDefault;

impl MjsDefault {
    default_accessor_wrapper! {
        joint, geom, camera, light, flex, mesh, material,
        pair, equality, tendon, actuator
    }

    /// Returns an immutable reference to the inner struct.
    pub fn ffi(&self) -> &mjsDefault {
        self
    }

    /// Returns a mutable reference to the inner struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjsDefault {
        self
    }
}

impl SpecItem for MjsDefault {
    unsafe fn element_pointer(&self) -> *mut mjsElement {
        self.ffi().element
    }

    /// Defaults can't be deleted.
    /// # Errors
    /// This will always error with [`ErrorKind::Unsupported`].
    fn delete(self) -> Result<(), Error> {
        Err(Error::new(ErrorKind::Unsupported, "can't delete defaults"))
    }
}

// SAFETY: These are safe to implement, as access to them is available only
// through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
unsafe impl Sync for MjsDefault {}
unsafe impl Send for MjsDefault {}
