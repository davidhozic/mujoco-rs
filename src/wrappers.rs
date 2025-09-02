//! A set of wrappers around the MuJoCo types.
pub mod mj_visualization;
pub mod mj_interface;
pub mod mj_statistic;
pub mod mj_rendering;
pub mod mj_auxiliary;
pub mod mj_option;
pub mod mj_model;
pub mod mj_data;

pub use mj_visualization::*;
pub use mj_rendering::*;
pub use mj_option::*;
pub use mj_model::*;
pub use mj_data::*;


/// Creates getter and setter methods for C structs.
#[macro_export]
    macro_rules! impl_getter_setter {
    (get, set, $attr:ident, $comment:literal, $ctype:ty, &$rtype:ty, $accessor_g:expr, $accessor_s:expr) => {
        impl_getter_setter!(get, $attr, $comment, &$rtype, $accessor_g);
        impl_getter_setter!(set, $attr, $comment, $ctype, &$rtype, $accessor_s);
    };

    (get, set, $attr:ident, $comment:literal, $ctype:ty, $rtype:ty, $accessor_g:expr, $accessor_s:expr) => {
        impl_getter_setter!(get, $attr, $comment, $rtype, $accessor_g);
        impl_getter_setter!(set, $attr, $comment, $ctype, $rtype, $accessor_s);
    };

    (get, unsafe set, $attr:ident, $comment:literal, $ctype:ty, &$rtype:ty, $accessor_g:expr, $accessor_s:expr) => {
        impl_getter_setter!(get, $attr, $comment, &$rtype, $accessor_g);
        impl_getter_setter!(unsafe set, $attr, $comment, $ctype, &$rtype, $accessor_s);
    };

    (get, unsafe set, $attr:ident, $comment:literal, $ctype:ty, $rtype:ty, $accessor_g:expr, $accessor_s:expr) => {
        impl_getter_setter!(get, $attr, $comment, $rtype, $accessor_g);
        impl_getter_setter!(unsafe set, $attr, $comment, $ctype, $rtype, $accessor_s);
    };

    /* Getter by reference */
    (get, $attr:ident, $comment:literal, &$rtype:ty, $accessor:expr) => {
        paste::paste! {
            #[doc = concat!("Get the ", $comment, ".")]
            #[allow(non_snake_case)]
            pub fn [<$attr>](&self) -> &$rtype {
                &self.$accessor.$attr as &$rtype
            }
        }
    };

    /* Getter by value */
    (get, $attr:ident, $comment:literal, $rtype:ty, $accessor:expr) => {
        paste::paste! {
            #[doc = concat!("Get the ", $comment, ".")]
            #[allow(non_snake_case)]
            pub fn [<$attr>](&self) -> $rtype {
                self.$accessor.$attr as $rtype
            }
        }
    };
    
    /* Unsafe setter */
    (unsafe set, $attr:ident, $comment:literal, $ctype:ty, $rtype:ty, $accessor:expr) => {
        paste::paste! {
            #[doc = concat!("Set the ", $comment, ".")]
            #[allow(non_snake_case)]
            pub unsafe fn [<set_ $attr>](&mut self, value: $rtype) {
                unsafe { self.$accessor.$attr = value as $ctype };
            }
        }
    };

    /* Safe setter */
    (set, $attr:ident, $comment:literal, $ctype:ty, $rtype:ty, $accessor:expr) => {
        paste::paste! {
            #[doc = concat!("Set the ", $comment, ".")]
            #[allow(non_snake_case)]
            pub fn [<set_ $attr>](&mut self, value: $rtype) {
                self.$accessor.$attr = value as $ctype;
            }
        }
    };
}
