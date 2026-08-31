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
                unsafe { [<Mjs $name:camel>]::from_ffi_ptr(self.ffi().$name) }.unwrap()
            }

            #[doc = concat!("Returns a mutable reference to ", stringify!($name), "'s defaults.")]
            pub fn [<$name _mut>](&mut self) -> &mut [<Mjs $name:camel>] {
                // SAFETY: see above.
                unsafe { [<Mjs $name:camel>]::from_ffi_ptr_mut(self.ffi().$name) }.unwrap()
            }
        )*
    }};
}

mjs_opaque!(MjsDefault <= mjsDefault,
    "Default specification. An opaque handle for the FFI type [`mjsDefault`], reached through \
[`ffi`](Self::ffi).");

impl MjsDefault {
    default_accessor_wrapper! {
        joint, geom, site, camera, light, flex, mesh, material,
        pair, equality, tendon, actuator
    }
}

impl super::traits::sealed::Sealed for MjsDefault {}

impl SpecItem for MjsDefault {
    fn element_pointer(&self) -> *const mjsElement {
        self.ffi().element
    }

    fn default(&self) -> Option<&MjsDefault> {
        Some(self)
    }

    /// A default class carries no id. Always returns `None`.
    fn id(&self) -> Option<usize> {
        // mjCDef derives from mjsElement, not mjCBase, so mjs_getId would read an unrelated offset.
        None
    }

    /// A default class cannot be assigned to another default class.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn set_default(&mut self, _class_name: &str) -> Result<(), MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }

    /// A default class cannot be assigned to another default class.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn with_default(&mut self, _class_name: &str) -> Result<&mut Self, MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }
}
