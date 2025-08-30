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
