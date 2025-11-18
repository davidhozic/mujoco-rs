
use std::path::{PathBuf};
use std::fs::File;
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

        fn field_visibility(
                &self,
                _info: bindgen::callbacks::FieldInfo<'_>,
            ) -> Option<bindgen::FieldVisibilityKind> {
            if _info.type_name.starts_with("mjs") {
                Some(bindgen::FieldVisibilityKind::PublicCrate)
            } else { None }
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
            .header(include_dir_simulate.join("simulate_c_api.h").to_str().unwrap())
            .header(include_dir_simulate.join("glfw_dispatch.h").to_str().unwrap())
            .blocklist_item("std::tuple.*")
            .allowlist_item("mj.*")
            .allowlist_item("mujoco::.*")
            .allowlist_function("mujoco_cSimulate.*")
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

    // Environmental variable which contains the path to the MuJoCo's build/lib/ directory.
    // This mean for static linking, otherwise the dynamic MuJoCo library can just be installed and used.
    const MUJOCO_STATIC_LIB_PATH_VAR: &str = "MUJOCO_STATIC_LINK_DIR";
    const MUJOCO_DYN_LIB_PATH_VAR: &str = "MUJOCO_DYNAMIC_LINK_DIR";
    const MUJOCO_DOWNLOAD_PATH_VAR: &str = "MUJOCO_DOWNLOAD_PATH";
    const MUJOCO_BASE_DOWNLOAD_LINK: &str = "https://github.com/google-deepmind/mujoco/releases/download";

    println!("cargo:rerun-if-env-changed={MUJOCO_STATIC_LIB_PATH_VAR}");
    println!("cargo:rerun-if-env-changed={MUJOCO_DYN_LIB_PATH_VAR}");
    println!("cargo:rerun-if-env-changed={MUJOCO_DOWNLOAD_PATH_VAR}");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    let mujoco_static_link_dir = env::var(MUJOCO_STATIC_LIB_PATH_VAR).ok();
    let mujoco_dyn_link_dir = env::var(MUJOCO_DYN_LIB_PATH_VAR).ok();

    /* Static linking */
    if let Some(path) = mujoco_static_link_dir {
        let mj_lib_pathbuf = PathBuf::from(path);

        #[cfg(unix)]
        let mj_lib_mujoco_path = mj_lib_pathbuf.join("libmujoco.a");

        #[cfg(windows)]
        let mj_lib_mujoco_path = mj_lib_pathbuf.join("mujoco.lib");

        println!("cargo::rerun-if-changed={}", mj_lib_mujoco_path.display());
        println!("cargo:rustc-link-search={}", mj_lib_pathbuf.display());

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
    else if let Some(path) = mujoco_dyn_link_dir {
        println!("cargo:rustc-link-search={}", PathBuf::from(path).display());
        println!("cargo:rustc-link-lib=mujoco");
    }

    /* pkg-config fallback or automatically download */
    else {
        pkg_config::Config::new()
            .probe("mujoco")
            .unwrap_or_else(|err| panic!("Unable to locate MuJoCo via pkg-config and neither {MUJOCO_STATIC_LIB_PATH_VAR} nor {MUJOCO_DYN_LIB_PATH_VAR} is set ({err})"));

        let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_else(|_| {
            panic!(
                "unable to obtain the OS information -- please manually download MuJoCo\
                and specify {MUJOCO_DYN_LIB_PATH_VAR} (and potentially add the path to LD_LIBRARY_PATH)"
            )
        });

        let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| {
            panic!(
                "unable to obtain the ARCHITECTURE information -- please manually download MuJoCo\
                and specify {MUJOCO_DYN_LIB_PATH_VAR} (and potentially add the path to LD_LIBRARY_PATH)"
            )
        });

        let mujoco_version = env!("CARGO_PKG_VERSION").split_once("mj-").unwrap().1;
        let download_url = if cfg!(target_os = "linux") {
            format!("{MUJOCO_BASE_DOWNLOAD_LINK}/{mujoco_version}/mujoco-{mujoco_version}-linux-{target_arch}.tar.gz")
        }
        else if cfg!(target_os = "windows") {
            format!("{MUJOCO_BASE_DOWNLOAD_LINK}/{mujoco_version}/mujoco-{mujoco_version}-windows-{target_arch}.zip")
        }
        else {
            panic!(
                "automatic download of MuJoCo is not supported on {target_os} --\
                please manually download MuJoCo and specify {MUJOCO_DYN_LIB_PATH_VAR}\
                (and potentially add the path to LD_LIBRARY_PATH)"
            );
        };

        let download_dir = PathBuf::from(
            std::env::var(MUJOCO_DOWNLOAD_PATH_VAR)
                .unwrap_or_else(|_| ".".to_string())
        );
        let download_path = download_dir.join(download_url.split("/").last().unwrap());

        println!("cargo::rerun-if-changed={}", download_path.display());
        if !download_path.exists() {  // skip if it exists
            /* Download the file */
            let mut response = ureq::get(&download_url).call().unwrap();
            let mut body_reader = response.body_mut().as_reader();

            let mut file = File::create(&download_path).unwrap();
            std::io::copy(&mut body_reader, &mut file).unwrap();

            /* Extract */
            #[cfg(target_os = "windows")]
            extract_windows(file);

            #[cfg(target_os = "linux")]
            extract_linux(&download_path)
        }

        let libdir_path = download_dir.join(download_path.file_name().unwrap().to_str().unwrap().split("-").take(2)
            .collect::<Box<[_]>>()
            .join("-")
        ).join("lib");
        let ldp_display = libdir_path.display();

        #[cfg(target_os = "linux")]
        {
            let current = std::env::var("LD_LIBRARY_PATH").unwrap();
            println!("cargo:rustc-env=LD_LIBRARY_PATH={current}:{ldp_display}");
        }

        println!("cargo:rustc-link-search={ldp_display}");
        println!("cargo:rustc-link-lib=mujoco");
    }

    #[cfg(feature = "ffi-regenerate")]
    build_dependencies::generate_ffi();
}

#[cfg(target_os = "windows")]
fn extract_windows(file: File) {
    /* Extract */
    let mut zip = zip::ZipArchive::new(file).unwrap();
    for i in 0..zip.len() {
        let mut zipfile = zip.by_index(i).unwrap();
        let path = if let Some(path) = zipfile.enclosed_name() { path } else { continue };

        if zipfile.is_dir() {
            std::fs::create_dir_all(&path).unwrap();
            continue;
        }

        let mut file = File::create(&path).unwrap();
        std::io::copy(&mut zipfile, &mut file).unwrap();
    }
}

#[cfg(target_os = "linux")]
fn extract_linux(filename: &PathBuf) {
    let file = File::open(filename).unwrap();
    let tar = flate2::read::GzDecoder::new(file);
    let mut archive = tar::Archive::new(tar);
    archive.unpack(filename.parent().unwrap()).unwrap();
}
