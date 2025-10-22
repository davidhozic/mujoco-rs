
use std::path::PathBuf;
use std::env;


#[cfg(feature = "ffi-regenerate")]
mod build_dependencies {
    use bindgen::callbacks::{ParseCallbacks, DeriveInfo};
    use super::*;
    use std::fs;

    #[derive(Debug)]
    struct CloneCallback;

    impl ParseCallbacks for CloneCallback {
        fn add_derives(&self, info: &DeriveInfo) -> Vec<String> {
            let mut data = vec!["Clone".into()];
            if info.name.starts_with("mjui") || info.name == "mjrRect_" {
                data.push("Copy".into());
            }

            else if info.name.starts_with("mjt") {  // enums
                data.push("Copy".into());
            }

            data
        }
    }


    pub(crate) fn generate_ffi() {
        let output_path = PathBuf::from("./src/");
        let current_dir = env::current_dir().unwrap();
        let include_dir_mujoco = current_dir.join("mujoco/include/mujoco");
        let include_dir_simulate = current_dir.join("mujoco/simulate/");
        let include_dir_glfw = current_dir.join("mujoco/build/_deps/glfw3-src/include/");

        let bindings_mujoco = bindgen::Builder::default()
            .header(include_dir_mujoco.join("mujoco.h").to_str().unwrap())
            .clang_arg(format!("-I{}", include_dir_mujoco.display()))
            .clang_arg(format!("-I{}", include_dir_mujoco.parent().unwrap().display()))
            .clang_arg(format!("-I{}", include_dir_glfw.display()))
            .allowlist_item("mj.*")
            .layout_tests(false)
            .derive_default(false)
            .derive_copy(false)
            .rustified_enum(".*")
            .parse_callbacks(Box::new(CloneCallback))
            /* Simulate C++ stuff */
            .clang_args(["-x", "c++", "-std=c++20"])
            .header(include_dir_simulate.join("simulate.h").to_str().unwrap())
            .header(include_dir_simulate.join("glfw_dispatch.h").to_str().unwrap())
            .blocklist_item("std::tuple.*")
            .allowlist_item("mj.*")
            .allowlist_item("mujoco::.*")
            .allowlist_item("new_simulate")
            .allowlist_item("free_simulate")
            .opaque_type("std::.*")
            /* Generate */
            .generate()
            .expect("unable to generate MuJoCo bindings");

        let outputfile_dir = output_path.join("mujoco_c.rs");
        let mut fdata = bindings_mujoco.to_string();

        /* Extra adjustments */
        fdata = fdata.replace("pub __lx: std_basic_string_value_type<_CharT>,", "pub __lx: std::mem::ManuallyDrop<std_basic_string_value_type<_CharT>>,");
        // Remove extra Clone
        let mut re = regex::Regex::new(r"#\[derive\((.*?Clone.*?), Clone,?(.*?)\)\]").unwrap();
        fdata = re.replace_all(&fdata, "#[derive($1, $2)]").to_string();

        // Make mjtSameFrame be MjtByte as used in all fields.
        re = regex::Regex::new(r"#\[repr\(u32\)\]\n(.*\npub enum mjtSameFrame_)").unwrap();
        fdata = re.replace(&fdata, "#[repr(u8)]\n$1").to_string();

        fs::write(outputfile_dir, fdata).unwrap();
    }
}



fn main() {
    if std::env::var("DOCS_RS").is_ok() {
        return;
    }

    /// Environmental variable which contains the path to the MuJoCo's build/lib/ directory.
    /// This mean for static linking, otherwise the dynamic MuJoCo library can just be installed and used.
    const MUJOCO_STATIC_LIB_PATH_VAR: &str = "MUJOCO_STATIC_LINK_DIR";
    const MUJOCO_DYN_LIB_PATH_VAR: &str = "MUJOCO_DYNAMIC_LINK_DIR";

    println!("cargo:rerun-if-env-changed={MUJOCO_STATIC_LIB_PATH_VAR}");
    println!("cargo:rerun-if-env-changed={MUJOCO_DYN_LIB_PATH_VAR}");

    let mujoco_lib_path= env::var(MUJOCO_STATIC_LIB_PATH_VAR);

    /* Static linking */
    if let Ok(path) = mujoco_lib_path {
        let mj_lib_pathbuf = PathBuf::from(path);
        let mj_lib_simulate_path = mj_lib_pathbuf.join("libsimulate.a");

        println!("cargo::rerun-if-changed={}", mj_lib_simulate_path.canonicalize().unwrap().display());
        println!("cargo:rustc-link-search={}", mj_lib_pathbuf.canonicalize().unwrap().display());

        #[cfg(feature = "cpp-viewer")]
        {
            println!("cargo:rustc-link-lib=simulate");
            println!("cargo:rustc-link-lib=glfw3");
        }

        println!("cargo:rustc-link-lib=mujoco");
        println!("cargo:rustc-link-lib=lodepng");
        println!("cargo:rustc-link-lib=tinyxml2");
        println!("cargo:rustc-link-lib=qhullstatic_r");
        println!("cargo:rustc-link-lib=ccd");

        if cfg!(unix) {
            println!("cargo:rustc-link-lib=stdc++");
        }
    }

    /* Dynamic linking */
    else {
        let mujoco_dylib_path = PathBuf::from(env::var(MUJOCO_DYN_LIB_PATH_VAR)
            .unwrap_or_else(|_| panic!("nor the static library path ({MUJOCO_STATIC_LIB_PATH_VAR}),\
                nor the dynamic library path ({MUJOCO_DYN_LIB_PATH_VAR}) was given.")));

        println!("cargo:rustc-link-search={}", mujoco_dylib_path.canonicalize().unwrap().display());
        println!("cargo:rustc-link-lib=mujoco");
    }


    #[cfg(feature = "ffi-regenerate")]
    build_dependencies::generate_ffi();
}
