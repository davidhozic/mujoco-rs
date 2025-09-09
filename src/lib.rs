//! ## MuJoCo-rs
//! A wrapper around the MuJoCo C library with a Rust-native viewer.
//! If you're familiar with MuJoCo, this should be pretty straightforward to use as the wrappers
//! mainly encapsulate some C structs or just rename them to match the Rust's PascalCase style.
//! 
//! Currently, no direct functions are provided. Some of the functions are made into methods at appropriate
//! structs, while others can be found under ``mujoco_rs::mujoco_c`` module. Missing structs
//! can also be obtained there.
//! 
//! To access the lower-level ffi structs in the wrappers, call either the ``ffi()`` method
//! or the ``ffi_mut()`` method.
//! 
//! The main structs are [`wrappers::mj_model::MjModel`] and [`wrappers::mj_data::MjData`].
//! The Rust-native viewer is available in [`viewer::MjViewer`].
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
        assert!(!version.is_empty());
    }


    #[allow(unused)]
    pub(crate) fn test_leaks() {
        use super::*;
        use wrappers::*;
        use std::hint::black_box;

        const N_ITEMS: usize = 10000;
        const N_REPEATS: usize = 1000;
        const EXAMPLE_MODEL: &str = "
        <mujoco>
        <worldbody>
            <light ambient=\"0.2 0.2 0.2\"/>
            <body name=\"ball\">
                <geom name=\"sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
                <joint name=\"sphere\" type=\"free\"/>
            </body>

            <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

        </worldbody>
        </mujoco>
        ";

        for _ in 0..N_REPEATS {
            let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("failed to load the model.");
            let mut datas: Vec<_> = (0..N_ITEMS).map(|_| model.make_data()).collect();

            for data in datas.iter_mut() {
                data.joint("sphere").unwrap().view_mut(data).qpos[0] /= 2.0;
                data.step();
            }
        }
    }
}
