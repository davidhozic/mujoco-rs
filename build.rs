use bindgen::callbacks::{ParseCallbacks, DeriveInfo};
use std::path::PathBuf;
use std::env;
use std::fs;


#[derive(Debug)]
struct CloneCallback;

impl ParseCallbacks for CloneCallback {
    fn add_derives(&self, info: &DeriveInfo) -> Vec<String> {
        let mut data = vec!["Clone".into()];
        if info.name.starts_with("mjui") || info.name == "mjrRect_" {
            data.push("Copy".into());
        }

        data
    }
}



fn main() {
    println!("cargo::rerun-if-changed={}", fs::canonicalize("./mujoco/build/lib/libsimulate.a").unwrap().display());
    println!("cargo:rustc-link-search={}", fs::canonicalize("./mujoco/build/lib/").unwrap().display());
    // println!("cargo:rustc-link-arg=-Wl,-rpath=./mujoco/build/lib/");
    println!("cargo:rustc-link-lib=simulate");
    println!("cargo:rustc-link-lib=mujoco");
    println!("cargo:rustc-link-lib=lodepng");
    println!("cargo:rustc-link-lib=tinyxml2");
    println!("cargo:rustc-link-lib=qhullstatic_r");
    println!("cargo:rustc-link-lib=ccd");

    if cfg!(unix) {
        println!("cargo:rustc-link-lib=stdc++");
    }

    let output_path = PathBuf::from("./src/");
    let current_dir = env::current_dir().unwrap();
    let include_dir_mujoco = current_dir.join("src/cpp/include/mujoco");
    let include_dir_simulate = include_dir_mujoco.join("viewer");

    // Generate MuJoCo bindings
    let bindings_mujoco = bindgen::Builder::default()
        .header(include_dir_mujoco.join("mujoco.h").to_str().unwrap())
        .header(include_dir_simulate.join("simulate.hpp").to_str().unwrap())
        .header(include_dir_simulate.join("glfw_dispatch.hpp").to_str().unwrap())
        .clang_arg("-std=c++20")
        .clang_arg("-stdlib=libc++")
        .clang_arg(format!("-I{}", current_dir.join("mujoco/build/_deps/glfw3-src/include/").display()))
        .blocklist_item("std::tuple.*")
        .allowlist_item("mj.*")
        .allowlist_item("mujoco::.*")
        .allowlist_item("new_simulate")
        .allowlist_item("free_simulate")
        .layout_tests(false)
        .derive_default(false)
        .opaque_type("std::.*")
        .derive_copy(false)
        .parse_callbacks(Box::new(CloneCallback))
        .generate()
        .expect("unable to generate MuJoCo bindings");

    let outputfile_dir = output_path.join("mujoco_c.rs");
    let mut fdata = bindings_mujoco
        .to_string();

    // Extra adjustments
    fdata = fdata.replace("pub __lx: std_basic_string_value_type<_CharT>,", "pub __lx: std::mem::ManuallyDrop<std_basic_string_value_type<_CharT>>,");
    fs::write(outputfile_dir, fdata).unwrap();


    // Generate lodepng bindings
    // rust/mujoco_rust/mujoco/build/_deps/lodepng-src
    let lodepng_dir = current_dir.join("mujoco/build/_deps/lodepng-src");
    bindgen::Builder::default()
        .header(lodepng_dir.join("lodepng.h").to_str().unwrap())
        .allowlist_item("lodepng_.*")
        .layout_tests(false)
        .derive_default(true)
        .clang_args(["-x", "c++"])
        .generate()
        .expect("unable to generate lodepng bindings")
        .write_to_file(output_path.join("lodepng_c.rs"))
        .expect("could not write lodepng bindings");
}
