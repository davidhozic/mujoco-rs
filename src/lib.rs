//! ## MuJoCo-Rust-W
//! A wrapper around the MuJoCo C library with a Rust-native viewer.
//! If you're familiar with MuJoCo, this should be pretty straightforward to use as the wrappers
//! mainly encapsulate some C structs or just rename them to match the Rust's PascalCase style.
//! 
//! Currently, no direct functions are provided. Some of the functions are made into methods at appropriate
//! structs, while others can be found under ``mujoco_rs_w::mujoco_c`` module. Missing structs
//! can also be obtained there.
//! 
//! To access the lower-level ffi structs in the wrappers, call either the ``ffi()`` method
//! or the ``ffi_mut`` method.
//! 

use std::ffi::CStr;

pub mod wrappers;
pub mod prelude;
pub mod util;

#[cfg(feature = "viewer")]
pub mod viewer;

#[allow(warnings)]
pub mod mujoco_c;  // raw MuJoCo C and C++ bindings


/// Returns the version string of the MuJoCo library
pub fn get_mujoco_version() -> &'static str {
    let arr = unsafe { mujoco_c::mj_versionString() };
    unsafe { CStr::from_ptr(arr).to_str().unwrap() }
}

#[cfg(test)]
mod tests {
    use crate::get_mujoco_version;

    #[test]
    fn test_version() {
        let version = get_mujoco_version();
        println!("{version}");
    }
}
