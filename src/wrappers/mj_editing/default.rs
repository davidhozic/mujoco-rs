//! Module implements [`MjsDefault`], the handle of a default class.

use crate::wrappers::mj_editing::{
    MjsJoint, MjsGeom, MjsSite, MjsCamera, MjsLight, MjsFlex, MjsMesh, MjsMaterial,
    MjsPair, MjsEquality, MjsTendon, MjsActuator
};
use crate::mujoco_c::*;

use super::traits::{sealed, SpecElement};


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

impl sealed::Sealed for MjsDefault {}

impl SpecElement for MjsDefault {
    fn element_pointer(&self) -> *const mjsElement {
        self.ffi().element
    }
}
