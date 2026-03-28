#[allow(unused_imports)]
use std::path::{Path, PathBuf};
#[allow(unused_imports)]
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
            // mjs* types: omit Clone because unsafe impl Send + Clone on
            // pointer-containing types is unsound -- clones share aliased raw
            // pointers and can cause data races across thread boundaries.
            // mjSpec_ has the same issue (contains *mut mjsElement etc.).
            if info.name.starts_with("mjs") || info.name == "mjSpec_" {
                return vec![];
            }

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

        let bindings_mujoco = bindgen::Builder::default()
            .header(include_dir_mujoco.join("mujoco.h").to_str().unwrap())
            .clang_arg(format!("-I{}", include_dir_mujoco.display()))
            .clang_arg(format!("-I{}", include_dir_mujoco.parent().unwrap().display()))
            .allowlist_item("mj.*")
            .layout_tests(false)
            .derive_default(false)
            .derive_copy(false)
            .rustified_enum(".*")
            // Respect C API size definitions to enhance type safety.
            // Instead of ffi function bindings containing raw pointers to a scalar type,
            // they now contain raw pointers to arrays of fixed size.
            .array_pointers_in_arguments(true)
            .parse_callbacks(Box::new(CloneCallback))
            /* Generate (C API only) */
            .generate()
            .expect("unable to generate MuJoCo bindings");

        let outputfile_dir = output_path.join("mujoco_c.rs");
        let mut fdata = bindings_mujoco.to_string();

        /* Extra adjustments */
        fdata = fdata.replace(
            "pub __lx: std_basic_string_value_type<_CharT>,",
            "pub __lx: std::mem::ManuallyDrop<std_basic_string_value_type<_CharT>>,",
        );
        // Remove extra Clone
        let mut re = regex::Regex::new(r"#\[derive\((.*?Clone.*?), Clone,?(.*?)\)\]").unwrap();
        fdata = re.replace_all(&fdata, "#[derive($1, $2)]").to_string();

        // Make mjtSameFrame be MjtByte as used in all fields.
        re = regex::Regex::new(r"#\[repr\(u32\)\]\n(.*\npub enum mjtSameFrame_)").unwrap();
        fdata = re.replace(&fdata, "#[repr(u8)]\n$1").to_string();

        // Replace "zero"-sized and one-sized arrays with raw pointers
        re = regex::Regex::new(r"\*\s*(const|mut)\s*\[\s*(.+?)\s*;\s*[0-1]+[A-z]+\s*\]").unwrap();
        fdata = re.replace_all(&fdata, "*$1 $2").to_string();

        fs::write(outputfile_dir, fdata).unwrap();
    }
}



fn main() {
    // Setup everything.
    println!("cargo::rerun-if-env-changed=DOCS_RS");
    if std::env::var("DOCS_RS").is_ok() {
        return;
    }

    // Read the target OS from Cargo's environment (reflects the compilation TARGET, not the host).
    // Using CARGO_CFG_TARGET_OS here is intentional: build scripts are compiled for the HOST
    // machine, so cfg!(target_os = "...") would reflect the host OS, not the target being
    // compiled for. CARGO_CFG_TARGET_OS is always set by Cargo before invoking build scripts.
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    // Verify feature selection before compilation.
    #[cfg(feature = "renderer")]
    #[cfg(not(feature = "renderer-winit-fallback"))]
    if target_os != "linux" {
        panic!("MuJoCo-rs requires that the 'renderer-winit-fallback' Cargo feature is enabled on non-Linux platforms as only winit based renderer is available.");
    }

    const MUJOCO_STATIC_LIB_PATH_VAR: &str = "MUJOCO_STATIC_LINK_DIR";
    const MUJOCO_DYN_LIB_PATH_VAR: &str = "MUJOCO_DYNAMIC_LINK_DIR";
    const MUJOCO_DOWNLOAD_PATH_VAR: &str = "MUJOCO_DOWNLOAD_DIR";
    #[allow(dead_code)]
    const MUJOCO_BASE_DOWNLOAD_LINK: &str = "https://github.com/google-deepmind/mujoco/releases/download";

    println!("cargo::rerun-if-env-changed={MUJOCO_STATIC_LIB_PATH_VAR}");
    println!("cargo::rerun-if-env-changed={MUJOCO_DYN_LIB_PATH_VAR}");
    println!("cargo::rerun-if-env-changed={MUJOCO_DOWNLOAD_PATH_VAR}");
    println!("cargo::rerun-if-env-changed=PKG_CONFIG_PATH");

    let mujoco_static_link_dir = env::var(MUJOCO_STATIC_LIB_PATH_VAR).ok();
    let mujoco_dyn_link_dir = env::var(MUJOCO_DYN_LIB_PATH_VAR).ok();

    /* Static linking */
    if let Some(path) = mujoco_static_link_dir {
        let path_lib_dir = PathBuf::from(path);
        let path_lib_dir_display = path_lib_dir.display();

        if path_lib_dir.is_relative() {
            panic!("{MUJOCO_STATIC_LIB_PATH_VAR} must be an absolute path ('{path_lib_dir_display}' is not).");
        }

        let path_lib_file = if target_os == "windows" {
            path_lib_dir.join("mujoco.lib")
        } else {
            path_lib_dir.join("libmujoco.a")
        };

        // Must be mujoco-x.x.x/lib/ not mujoco-x.x.x/
        if !path_lib_file.is_file() {
            panic!(
                "{MUJOCO_STATIC_LIB_PATH_VAR} must be path to the 'lib/' subdirectory (i.e., 'mujoco-x.x.x/lib/') --- \
                '{path_lib_dir_display}' does not appear to contain '{}' or it doesn't exist.",
                path_lib_file.file_name().unwrap().to_str().unwrap()
            );
        }

        println!("cargo::rerun-if-changed={}", path_lib_file.display());
        println!("cargo::rustc-link-search={}", path_lib_dir_display);

        #[cfg(feature = "cpp-viewer")]
        {
            println!("cargo::rustc-link-lib=static=simulate");
            println!("cargo::rustc-link-lib=static=glfw3");
        }

        println!("cargo::rustc-link-lib=static=mujoco");
        println!("cargo::rustc-link-lib=static=lodepng");
        println!("cargo::rustc-link-lib=static=tinyxml2");
        println!("cargo::rustc-link-lib=static=qhullstatic_r");
        println!("cargo::rustc-link-lib=static=ccd");

        if target_os == "linux" {
            println!("cargo::rustc-link-lib=stdc++");
        } else if target_os == "macos" {
            println!("cargo::rustc-link-lib=c++");
        }
    }

    /* Dynamic linking */
    else if let Some(path) = mujoco_dyn_link_dir {
        let path_lib_dir = PathBuf::from(path);
        let path_lib_dir_display = path_lib_dir.display();

        if path_lib_dir.is_relative() {
            panic!("{MUJOCO_DYN_LIB_PATH_VAR} must be an absolute path ('{path_lib_dir_display}' is not).");
        }

        let path_lib_file = if target_os == "windows" {
            path_lib_dir.join("mujoco.lib")
        } else if target_os == "macos" {
            path_lib_dir.join("libmujoco.dylib")
        } else {
            path_lib_dir.join("libmujoco.so")
        };

        // Must be mujoco-x.x.x/lib/ not mujoco-x.x.x/
        if !path_lib_file.is_file() {
            panic!(
                "{MUJOCO_DYN_LIB_PATH_VAR} must be path to the 'lib/' subdirectory (i.e., 'mujoco-x.x.x/lib/') --- \
                '{path_lib_dir_display}' does not appear to contain '{}' or it doesn't exist.",
                path_lib_file.file_name().unwrap().to_str().unwrap()
            );
        }

        println!("cargo::rustc-link-search={}", path_lib_dir_display);
        println!("cargo::rustc-link-lib=dylib=mujoco");
    }

    /* pkg-config / auto-download fallback */
    else {
        let mujoco_version = env!("CARGO_PKG_VERSION").split_once("mj-").unwrap().1;

        // In build.rs, #[cfg(target_os)] evaluates against the HOST, not the target.
        #[cfg(target_os = "linux")]
        const HOST_OS: &str = "linux";
        #[cfg(target_os = "windows")]
        const HOST_OS: &str = "windows";
        #[cfg(target_os = "macos")]
        const HOST_OS: &str = "macos";
        #[cfg(not(any(target_os = "linux", target_os = "windows", target_os = "macos")))]
        compile_error!("unsupported host OS");

        // Cross-OS builds require explicit library paths; neither pkg-config
        // (queries host packages) nor auto-download (extracts host binaries) can help.
        if HOST_OS != target_os {
            panic!(
                "cross-compiling for '{target_os}' from '{HOST_OS}' requires \
                {MUJOCO_STATIC_LIB_PATH_VAR} or {MUJOCO_DYN_LIB_PATH_VAR} to be set"
            );
        }

        // ---- Native builds only below this point ----

        // Try pkg-config (unix hosts only).
        #[cfg(unix)]
        #[allow(unused)]
        let found = match pkg_config::Config::new().exactly_version(mujoco_version).probe("mujoco") {
            Ok(_) => true,
            Err(_err) => {
                #[cfg(target_os = "macos")]
                {
                    #[cfg(feature = "auto-download-mujoco")]
                    compile_error!("automatic downloads are not supported on macOS");

                    panic!(
                        "{_err}\
                        \n---------------- ^^^ pkg-config output ^^^ ----------------\n\
                        \nUnable to locate MuJoCo via pkg-config and neither {MUJOCO_STATIC_LIB_PATH_VAR} \
                        nor {MUJOCO_DYN_LIB_PATH_VAR} is set."
                    );
                }

                #[cfg(not(target_os = "macos"))]
                {
                    #[cfg(not(feature = "auto-download-mujoco"))]
                    panic!(
                        "{_err}\
                        \n---------------- ^^^ pkg-config output ^^^ ----------------\n\
                        \nUnable to locate MuJoCo via pkg-config and neither {MUJOCO_STATIC_LIB_PATH_VAR} \
                        nor {MUJOCO_DYN_LIB_PATH_VAR} is set.\
                        \nConsider enabling automatic download: \
                        'cargo add mujoco-rs --features \"auto-download-mujoco\"'."
                    );
                    #[cfg(feature = "auto-download-mujoco")]
                    false
                }
            }
        };

        #[cfg(not(unix))]
        let found = {
            #[cfg(not(feature = "auto-download-mujoco"))]
            panic!(
                "neither {MUJOCO_STATIC_LIB_PATH_VAR} nor {MUJOCO_DYN_LIB_PATH_VAR} is set \
                and the 'auto-download-mujoco' Cargo feature is disabled.\
                \nConsider enabling automatic download: \
                'cargo add mujoco-rs --features \"auto-download-mujoco\"'."
            );
            #[cfg(feature = "auto-download-mujoco")]
            false
        };

        // Auto-download MuJoCo (Linux and Windows native builds only).
        #[cfg(not(target_os = "macos"))]
        #[cfg(feature = "auto-download-mujoco")]
        if !found {
            use std::io::{BufReader, Read};
            use sha2::Digest;

            let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| {
                panic!(
                    "unable to obtain the ARCHITECTURE information -- please manually download \
                    MuJoCo and specify {MUJOCO_DYN_LIB_PATH_VAR} \
                    (and potentially add the path to LD_LIBRARY_PATH)"
                )
            });

            #[cfg(target_os = "linux")]
            let download_url = format!(
                "{MUJOCO_BASE_DOWNLOAD_LINK}/{mujoco_version}/mujoco-{mujoco_version}-linux-{target_arch}.tar.gz"
            );
            #[cfg(target_os = "windows")]
            let download_url = format!(
                "{MUJOCO_BASE_DOWNLOAD_LINK}/{mujoco_version}/mujoco-{mujoco_version}-windows-{target_arch}.zip"
            );

            let download_hash_url = format!("{download_url}.sha256");

            let download_dir = PathBuf::from(std::env::var(MUJOCO_DOWNLOAD_PATH_VAR).unwrap_or_else(|_| {
                let os_example = if cfg!(target_os = "linux") {
                    format!("e.g., export {MUJOCO_DOWNLOAD_PATH_VAR}=\"$(realpath .)\"")
                } else {
                    format!("e.g., $env:{MUJOCO_DOWNLOAD_PATH_VAR}=\"/full/absolute/path/\"")
                };
                panic!(
                    "when Cargo feature 'auto-download-mujoco' is enabled, \
                    {MUJOCO_DOWNLOAD_PATH_VAR} must be set to an absolute path where MuJoCo \
                    will be extracted --- {os_example}",
                );
            }));

            if download_dir.is_relative() {
                panic!(
                    "{MUJOCO_DOWNLOAD_PATH_VAR} must be an absolute path to the location \
                    where MuJoCo will be downloaded and extracted (was given '{}').",
                    download_dir.display()
                )
            }

            let download_path = download_dir.join(download_url.rsplit_once("/").unwrap().1);
            let outdirname = download_dir.join(format!("mujoco-{mujoco_version}"));
            let download_hash_path = download_dir.join(download_hash_url.rsplit_once("/").unwrap().1);

            // Download the archive.
            let mut response = ureq::get(&download_url).call().expect("failed to download MuJoCo");
            let mut body_reader = response.body_mut().as_reader();
            std::fs::create_dir_all(download_path.parent().unwrap()).unwrap_or_else(
                |err| panic!("failed to create parent directory of '{}' ({err})", download_path.display())
            );
            let mut file = File::create(&download_path).unwrap_or_else(
                |err| panic!("could not save archive '{}' ({err})", download_path.display())
            );
            std::io::copy(&mut body_reader, &mut file).unwrap_or_else(
                |err| panic!("failed to copy archive contents to '{}' ({err})", download_path.display())
            );

            // Download the hash file.
            response = ureq::get(&download_hash_url).call().expect("failed to download MuJoCo's hash file");
            body_reader = response.body_mut().as_reader();
            let mut file = File::create(&download_hash_path).unwrap_or_else(
                |err| panic!("could not save archive '{}' ({err})", download_hash_path.display())
            );
            std::io::copy(&mut body_reader, &mut file).unwrap_or_else(
                |err| panic!("failed to copy archive contents to '{}' ({err})", download_hash_path.display())
            );

            // Verify file integrity via SHA-256.
            let mut hashfile_data = String::new();
            file = File::open(&download_hash_path).expect("failed to open the hash file");
            file.read_to_string(&mut hashfile_data).expect("failed to read hash file contents");
            let hash_official = hashfile_data.split_once(' ').unwrap().0;

            file = File::open(&download_path).unwrap();
            let mut reader = BufReader::new(file);
            let mut buffer = [0u8; 1024];
            let mut hasher = sha2::Sha256::new();
            loop {
                let n = reader.read(&mut buffer).unwrap_or_else(|err|
                    panic!("could not read archive '{}' ({err})", download_path.display())
                );
                if n == 0 {
                    break;
                }
                hasher.update(&buffer[..n]);
            }
            let result = format!("{:x}", hasher.finalize());
            if hash_official != result {
                panic!(
                    "sha256sum of '{}' does not match \
                    the one stored in the official hash file --- stopping due to security concerns!",
                    download_path.display()
                );
            }

            // Extract.
            #[cfg(target_os = "linux")]
            extract_linux(&download_path);
            #[cfg(target_os = "windows")]
            extract_windows(&download_path, &outdirname);

            std::fs::remove_file(&download_path).unwrap_or_else(
                |err| panic!("failed to delete archive '{}' ({err})", download_path.display())
            );
            std::fs::remove_file(&download_hash_path).unwrap_or_else(
                |err| panic!("failed to delete hash file '{}' ({err})", download_hash_path.display())
            );

            let libdir_path = outdirname.join("lib");
            println!("cargo::rustc-link-search={}", libdir_path.display());
            println!("cargo::rustc-link-lib=mujoco");
        }
    }

    #[cfg(feature = "ffi-regenerate")]
    build_dependencies::generate_ffi();
}

#[cfg(target_os = "windows")]
#[cfg(feature = "auto-download-mujoco")]
fn extract_windows(filename: &Path, outdirname: &Path) {
    let file = File::open(filename).unwrap_or_else(|err|
        panic!("failed to open archive '{}' ({err}).", filename.display())
    );
    let mut zip = zip::ZipArchive::new(file).unwrap_or_else(|err|
        panic!("failed to read ZIP metadata ({err}).")
    );

    for i in 0..zip.len() {
        let mut zipfile = zip.by_index(i).unwrap();
        let mut path = if let Some(path) = zipfile.enclosed_name() { path } else {
            println!("cargo:warning=Skipped potentially unsafe ZIP entry '{}' during extraction", zipfile.name());
            continue;
        };

        if zipfile.is_file() {
            // On Windows, place everything in a new folder.
            // This is for consistency with Linux targets.
            path = outdirname.join(path);

            // Create parents
            std::fs::create_dir_all(path.parent().unwrap()).unwrap_or_else(|err|
                panic!("failed to create {} and its parents ({err}).", outdirname.display())
            );

            let mut outfile = File::create(&path).unwrap_or_else(|err|
                panic!("failed to create file '{}' ({err})", path.display())
            );

            std::io::copy(&mut zipfile, &mut outfile).unwrap_or_else(|err|
                panic!("failed to copy {} to {} ({err})", zipfile.name(), path.display())
            );

            outfile.sync_all().unwrap_or_else(|err|
                panic!("failed to flush contents to file '{}' ({err})", path.display())
            );
        }
    }
}

#[cfg(target_os = "linux")]
#[cfg(feature = "auto-download-mujoco")]
fn extract_linux(filename: &Path) {
    let file = File::open(filename).unwrap_or_else(
        |err| panic!("failed to open '{}' ({err})", filename.display())
    );
    let tar = flate2::read::GzDecoder::new(file);
    let mut archive = tar::Archive::new(tar);
    archive.unpack(filename.parent().unwrap()).expect("failed to unpack MuJoCo archive");
}
