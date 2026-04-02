//! Module implements [`MjsDefault`], which is a special type of [`SpecItem`].

use crate::wrappers::mj_editing::{
    MjsJoint, MjsGeom, MjsSite, MjsCamera, MjsLight, MjsFlex, MjsMesh, MjsMaterial,
    MjsPair, MjsEquality, MjsTendon, MjsActuator
};
use crate::error::MjEditError;
use crate::mujoco_c::*;

use super::traits::SpecItem;


macro_rules! default_accessor_wrapper {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Returns an immutable reference to ", stringify!($name), "'s defaults.")]
            pub fn $name(&self) -> &[<Mjs $name:camel>] {
                // SAFETY: MuJoCo's mjCDef::PointToLocal() always initializes these
                // pointers to non-null addresses of the owning mjCDef's local members.
                unsafe { &*self.$name }
            }

            #[doc = concat!("Returns a mutable reference to ", stringify!($name), "'s defaults.")]
            pub fn [<$name _mut>](&mut self) -> &mut [<Mjs $name:camel>] {
                // SAFETY: see above.
                unsafe { &mut *self.$name }
            }
        )*
    }};
}

// This is implemented manually since we can't directly borrow check if something is using the default.
// We also override the delete method to panic instead of deleting.

/// Default specification. This is a type alias to [`mjsDefault`].
pub type MjsDefault = mjsDefault;

impl MjsDefault {
    default_accessor_wrapper! {
        joint, geom, site, camera, light, flex, mesh, material,
        pair, equality, tendon, actuator
    }
}

impl super::traits::sealed::Sealed for MjsDefault {}

impl SpecItem for MjsDefault {
    unsafe fn element_pointer(&self) -> *mut mjsElement {
        self.element
    }

    fn default(&self) -> &MjsDefault {
        self
    }

    /// `MjsDefault` cannot be assigned to a named default class.
    ///
    /// This override always returns [`MjEditError::UnsupportedOperation`] without using
    /// `class_name` or performing any operation.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn set_default(&mut self, _class_name: &str) -> Result<(), MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }

    /// `MjsDefault` cannot be assigned to a named default class.
    ///
    /// This override always returns [`MjEditError::UnsupportedOperation`] without using
    /// `class_name` or performing any operation.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn with_default(&mut self, _class_name: &str) -> Result<&mut Self, MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }

    /// Defaults can't be deleted.
    ///
    /// This override always returns [`MjEditError::UnsupportedOperation`] without performing
    /// any operation, so the post-deletion use-after-free restriction from [`SpecItem::delete`]
    /// does not apply -- `self` remains valid after an `Err` return.
    ///
    /// # Errors
    /// This will always error with [`MjEditError::UnsupportedOperation`].
    ///
    /// # Safety
    /// No unsafe operations are performed; the struct is unchanged on return.
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }
}

// SAFETY: MjsDefault is a raw pointer wrapper. All shared-reference access goes
// through &self methods that do not mutate state. All mutation requires &mut self,
// which guarantees no concurrent aliasing. The pointer is valid for the lifetime
// of the owning MjSpec, which must outlive any MjsDefault reference.
unsafe impl Sync for MjsDefault {}
unsafe impl Send for MjsDefault {}
