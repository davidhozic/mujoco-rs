//! Module implements [`MjsDefault`], which is a special type of [`SpecItem`].

use crate::error::MjEditError;

use super::traits::SpecItem;
use crate::mujoco_c::*;
use crate::wrappers::mj_editing::{
    MjsJoint, MjsGeom, MjsSite, MjsCamera, MjsLight, MjsFlex, MjsMesh, MjsMaterial,
    MjsPair, MjsEquality, MjsTendon, MjsActuator
};


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

impl SpecItem for MjsDefault {
    unsafe fn element_pointer(&self) -> *mut mjsElement {
        self.element
    }

    /// Defaults can't be deleted.
    /// # Errors
    /// This will always error with [`MjEditError::UnsupportedDeletion`].
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        Err(MjEditError::UnsupportedDeletion)
    }
}

// SAFETY: MjsDefault is a raw pointer wrapper. All shared-reference access goes
// through &self methods that do not mutate state. All mutation requires &mut self,
// which guarantees no concurrent aliasing. The pointer is valid for the lifetime
// of the owning MjSpec, which must outlive any MjsDefault reference.
unsafe impl Sync for MjsDefault {}
unsafe impl Send for MjsDefault {}
