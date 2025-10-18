//! # MuJoCo-rs
//! A wrapper around the MuJoCo C library with a Rust-native viewer.
//! If you're familiar with MuJoCo, this should be pretty straightforward to use as the wrappers
//! mainly encapsulate some C structs or just rename them to match the Rust's PascalCase style.
//! 
//! The main structs are [`wrappers::mj_model::MjModel`] and [`wrappers::mj_data::MjData`].
//! These two structs (and some other) wrap the C structure in order to achieve memory safety.
//! 
//! Their fields aren't publicly exposed and can instead be manipulated through views
//! (e.g., [`MjData::joint`](wrappers::mj_data::MjData::joint) and then [`wrappers::mj_data::MjJointDataInfo::view`]).
//! To access the wrapped attributes directly, call the corresponding `ffi()` methods
//! (e.g., [`MjData::ffi`](wrappers::MjData::ffi))
//! 
//! ## Documentation
//! A more guided documentation can be obtained [here](https://mujoco-rs.readthedocs.io/en/latest/).
//! 
//! ## 3D viewer
//! The Rust-native viewer is available ([`viewer::MjViewer`]), as well as the MuJoCo's original C++
//! one ([`viewer::MjViewerCpp`]).
//! The C++ viewer however requires manual compilation of a patched MuJoCo repository,
//! like described [here](https://mujoco-rs.readthedocs.io/en/latest/installation.html#static-linking-with-c-viewer).
//! 
//! ## Model editing
//! [`MjModel`](wrappers::MjModel) can be procedurally generated through the model editing module.
//! The specification representing the model is [`wrappers::mj_editing::MjSpec`]
//! 
//! ## Features
//! This crate has the following public features:
//! - `viewer`, which enables the Rust-native viewer ([`viewer::MjViewer`]),
//!   - `cpp-viewer`, which additionally enables the MuJoCo's original viewer (C++ based) ([`viewer::MjViewerCpp`]),
//! - `renderer`, which enables the image renderer ([`renderer::MjRenderer`]).
//! 
//! ## Functions
//! Most functions are wrapped under methods at different structs. Some functions
//! are available under the [`wrappers::fun`] module.
//! 
//! If a certain function can't be found, you can use the raw FFI bindings, available under 
//! the [`mujoco_c`] module. Note that to access the lower-level ffi structs inside of wrappers,
//! `ffi()` or `ffi_mut()` must be called (e.g., [`MjData::ffi`](wrappers::MjData::ffi) and [`MjModel::ffi`](wrappers::MjModel::ffi)). 
use std::ffi::CStr;

pub mod wrappers;
pub mod prelude;
pub mod util;


#[cfg(feature = "renderer")]
pub mod renderer;

#[cfg(feature = "viewer")]
pub mod viewer;

#[allow(warnings)]
pub mod mujoco_c;  // raw MuJoCo C and C++ bindings


/// Returns the version string of the MuJoCo library.
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

        std::hint::black_box(
            for _ in 0..N_REPEATS {
                let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("failed to load the model.");
                for mut data in (0..N_ITEMS).map(|_| model.make_data()) {
                    data.joint("sphere").unwrap().view_mut(&mut data).qpos[0] /= 2.0;
                    data.step();
                }
            }
        )
    }
}
