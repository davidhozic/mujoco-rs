#[allow(unused_imports)]
use std::path::{Path, PathBuf};
#[allow(unused_imports)]
use std::fs::File;
use std::env;


#[cfg(feature = "ffi-regenerate")]
mod build_dependencies {
    use bindgen::callbacks::{DeriveInfo, EnumVariantValue, ParseCallbacks};
    use super::*;
    use std::{cell::RefCell, collections::{BTreeMap, BTreeSet}, fs, rc::Rc};

    /// The safe variants (name and value) of each enum, in declaration order.
    type EnumSafeVariants = Rc<RefCell<BTreeMap<String, Vec<(String, u64)>>>>;

    #[derive(Debug)]
    struct BindgenOverrides {
        /// The variants of each enum that come before its count variant (e.g., `mjNOBJECT`).
        /// RefCell due to bindgen API limits.
        enum_safe_variants: EnumSafeVariants,
        /// The enums whose count variant was already seen, thus no later variant is safe.
        enum_safe_exhausted: RefCell<BTreeSet<String>>,
    }

    impl BindgenOverrides {
        fn new(enum_safe_variants: EnumSafeVariants) -> Self {
            Self { enum_safe_variants, enum_safe_exhausted: RefCell::new(BTreeSet::new()) }
        }
    }

    impl ParseCallbacks for BindgenOverrides {
        fn add_derives(&self, info: &DeriveInfo) -> Vec<String> {
            // mjs* types: omit Clone because unsafe impl Send + Clone on
            // pointer-containing types is unsound -- clones share aliased raw
            // pointers and can cause data races across thread boundaries.
            // mjSpec_ has the same issue (contains *mut mjsElement etc.).
            // mjLogMessage_: MuJoCo builds `body` on the stack of the caller, so a clone that a
            // log handler keeps alive would outlive the string and read freed memory.
            if info.name.starts_with("mjs") || ["mjSpec_", "mjLogMessage_"].contains(&info.name) {
                return vec![];
            }

            let mut data = vec!["Clone".into()];
            if info.name.starts_with("mjui") || info.name == "mjrRect_" {
                data.push("Copy".into());
            }

            else if info.name.starts_with("mjt") {  // enums
                data.push("Copy".into());
            }
            else if ["mjOption_", "mjStatistic_"].contains(&info.name) || info.name.starts_with("mjVisual") {
                data.push("PartialEq".into());
                if info.name == "mjStatistic_" {
                    data.push("Default".into());
                }

                // Special synchronization requirement.
                data.push("mujoco_rs_derive::ThreeWayMerge".into());
            }

            data
        }

        fn enum_variant_behavior(
            &self,
            enum_name: Option<&str>,
            original_variant_name: &str,
            variant_value: bindgen::callbacks::EnumVariantValue,
        ) -> Option<bindgen::callbacks::EnumVariantCustomBehavior>
        {
            let enum_name = enum_name.and_then(|n| n.strip_prefix("enum "))?;
            let mut borrow_safe_variants = self.enum_safe_variants.borrow_mut();
            let mut borrow_exhausted = self.enum_safe_exhausted.borrow_mut();

            let count_variant = match enum_name {
                "mjtGeom"  => "mjNGEOMTYPES",
                "mjtObj"   => "mjNOBJECT",
                "mjtTextureRole" => "mjNTEXROLE",
                _ => return None,
            };

            if original_variant_name == count_variant {
                borrow_exhausted.insert(enum_name.to_string());
            }
            else if !borrow_exhausted.contains(enum_name) {
                let value = match variant_value {
                    EnumVariantValue::Boolean(v) => v as u64,
                    EnumVariantValue::Unsigned(v) => v,
                    EnumVariantValue::Signed(v) => v as u64
                };

                borrow_safe_variants.entry(enum_name.to_string()).or_default()
                    .push((original_variant_name.to_string(), value));
            }

            None
        }
    }


    pub(crate) fn generate_ffi() {
        let output_path = PathBuf::from("./src/");
        let current_dir = env::current_dir().unwrap();
        let include_dir_mujoco = current_dir.join("mujoco/include/mujoco");

        // The variants of each enum that come before its count variant. These form the base enums.
        let enum_safe_variants = Rc::new(RefCell::new(BTreeMap::new()));

        // Generate bindings
        let bindings_mujoco = bindgen::Builder::default()
            .header(include_dir_mujoco.join("mujoco.h").to_str().unwrap())
            .header(include_dir_mujoco.join("mjbatch.h").to_str().unwrap())
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
            .parse_callbacks(Box::new(BindgenOverrides::new(enum_safe_variants.clone())))
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
        re = regex::Regex::new(r"#\[repr\(u32\)\]\n(.*\npub enum mjtSameFrame \{)").unwrap();
        fdata = re.replace(&fdata, "#[repr(u8)]\n$1").to_string();

        // Replace "zero"-sized and one-sized arrays with raw pointers
        re = regex::Regex::new(r"\*\s*(const|mut)\s*\[\s*(.+?)\s*;\s*[0-1]+[A-z]+\s*\]").unwrap();
        fdata = re.replace_all(&fdata, "*$1 $2").to_string();

        // Per-struct field visibility. A struct that the safe API exposes directly (through
        // a bare pub type alias, with no dedicated wrapper) "needs mediation" when at least
        // one of its fields requires the safe API to validate writes: a raw pointer (a public
        // one would let a user substitute an arbitrary address that C dereferences), a fixed
        // char buffer (must stay NUL-terminated), or a value C uses as an unchecked memory
        // index. When a struct needs mediation, ALL of its fields are demoted to pub(crate)
        // together -- a per-struct rule, so the access pattern is uniform (every field
        // reached through a wrapper accessor) rather than mixing direct field access with
        // accessors.
        //
        // Demotion is applied as a text pass (rather than the field_visibility callback)
        // because bindgen reports the same anonymous type name for pointers and arrays and
        // so cannot tell them apart, and because the per-struct decision needs to see the
        // whole struct body at once. Raw pointers are detected from the text; pointer-free
        // structs that still have a memory-unsafe field (a char buffer C reads as a string, or
        // a value C uses as an unchecked memory index) are listed explicitly in CONFIG_DEMOTE
        // (those field kinds are not reliably detectable from the text). A struct whose
        // rule-breaking fields are all memory-safe stays a plain, fully-public struct.
        //
        // Structs not exposed in the safe API (wrapped structs, internal helpers, UI types)
        // keep all fields public -- they are only reachable through FFI code that already
        // requires unsafe. See coding-conventions.md "Generated struct field visibility".
        const WRAPPED_STRUCTS: [&str; 6] =
            ["mjModel_", "mjData_", "mjvScene_", "mjrContext_", "mjVFS_", "mjSpec_"];
        // mjpResourceProvider has a bare `pub type MjpResourceProvider` alias but no safe API
        // consumes/produces it (only unwrapped unsafe FFI such as mjp_registerProvider and
        // mj_mountVFS uses it) and it has no accessors, so it is reachable only through
        // unsafe FFI -- treat it as internal and keep its (pointer) fields public.
        const INTERNAL_STRUCTS: [&str; 9] = [
            "mjCache_", "mjpDecoder", "mjpEncoder", "mjpPlugin_",
            "mjResource_", "mjSDF_", "mjrVertexAttribute_", "mjsElement_",
            "mjpResourceProvider",
        ];
        // Exposed pointer-free structs that still have a memory-unsafe field (a char buffer C
        // reads as a NUL-terminated string, or a value C uses as an unchecked memory index),
        // which the text scan cannot detect:
        //   mjLogConfig_ -- `logfile` is fopen()'d as a C string (non-NUL-terminated -> OOB read).
        // Structs whose rule-breaking fields are all memory-safe (e.g. an enum-as-int C only
        // switches on, like mjOption_/mjLROpt_/mjvOption_) are plain structs and are NOT listed.
        //
        // NOTE: mjvGeom_ and mjvFigure_ also have memory-unsafe fields (the renderer uses
        // mjvGeom_.matid/objid/dataid as unchecked array indices, and reads mjvFigure_'s
        // xlabel/linename/xformat as C strings), so they would normally belong here. They are
        // deliberately kept fully public for now and are intentionally NOT listed; revisit before
        // hardening those two structs behind accessors.
        const CONFIG_DEMOTE: [&str; 1] = ["mjLogConfig_"];
        let struct_re = regex::Regex::new(r"(?s)(pub struct (\w+) \{)(.*?)(\n\})").unwrap();
        let ptr_field_re =
            regex::Regex::new(r"(?m)^(\s*)pub (\w+: \*(?:const|mut) )").unwrap();
        // Matches any `pub <field>:` line so the whole struct can be demoted at once.
        let any_field_re = regex::Regex::new(r"(?m)^(\s*)pub (\w+:)").unwrap();
        fdata = struct_re
            .replace_all(&fdata, |caps: &regex::Captures| {
                let header = &caps[1];
                let name = &caps[2];
                let body = &caps[3];
                let tail = &caps[4];
                let exposed = !(WRAPPED_STRUCTS.contains(&name)
                    || INTERNAL_STRUCTS.contains(&name)
                    || name.starts_with("mjui")
                    || name.starts_with("mjUI"));
                let needs_mediation =
                    exposed && (ptr_field_re.is_match(body) || CONFIG_DEMOTE.contains(&name));
                let body = if needs_mediation {
                    any_field_re.replace_all(body, "${1}pub(crate) ${2}").to_string()
                } else {
                    body.to_string()
                };
                format!("{header}{body}{tail}")
            })
            .to_string();

        // Wrap ThreeWayMerge in cfg_attr so it is only derived when the viewer feature is active.
        re = regex::Regex::new(r"(#\[derive\([^)]*?),\s*mujoco_rs_derive\s*::\s*ThreeWayMerge(\)\])").unwrap();
        fdata = re.replace_all(
            &fdata,
            "#[cfg_attr(feature = \"viewer\", derive(mujoco_rs_derive::ThreeWayMerge))]\n$1$2",
        ).to_string();

        /* Extra enumerated variants and their enums */
        let enum_safe_variants = enum_safe_variants.take();
        let extra_enums: String = enum_safe_variants.into_iter()
            .map(|(k, v)|
                format!(
                    "\n#[repr(u32)]\n\
                    #[derive(Debug, Clone, Hash, PartialEq, Eq, Copy)]\n\
                    pub enum _{k}Base {{\n\
                        {}\
                    }}\n", v.into_iter().map(|(name, value)| format!("    {name} = {value},\n")).collect::<String>())
            )
            .collect();

        fdata += "\n/* Some enums contain extra variants that can be used in some specific context but would cause\n\
                       unsafety, undefined-behavior or out-of-bound reads if used incorrectly.\n\
                       These enums contain variants that are safe to use in all contexts.\n\
                       The following enums are not part of the actual C API directly and are\n\
                       generated by build.rs instead. */";
        fdata += &extra_enums;

        fs::write(outputfile_dir, fdata).unwrap();
    }
}



const MUJOCO_STATIC_LIB_PATH_VAR: &str = "MUJOCO_STATIC_LINK_DIR";
const MUJOCO_DYN_LIB_PATH_VAR: &str = "MUJOCO_DYNAMIC_LINK_DIR";
const MUJOCO_DOWNLOAD_PATH_VAR: &str = "MUJOCO_DOWNLOAD_DIR";
/// File in a static `lib/` directory whose presence means the archives embed the C++ runtime.
const EMBEDDED_CXX_MARKER: &str = ".embedded-cxx-runtime";

fn main() {
    // Setup everything.
    println!("cargo::rerun-if-env-changed=DOCS_RS");
    if std::env::var("DOCS_RS").is_ok() {
        return;
    }

    // Deprecated
    #[cfg(feature = "auto-download-mujoco")]
    println!(
        "cargo::warning=Cargo feature 'auto-download-mujoco' has been replaced with \
        'auto-download-dynamic-mujoco' for dynamic linking."
    );

    // Incompatible features
    #[cfg(all(feature = "auto-download-dynamic-mujoco", feature = "auto-download-static-mujoco"))]
    compile_error!("cannot have both dynamic and static MuJoCo library downloads enabled, pick one!");

    // Use cargo's target host instead of env!(), for cross-compilation purposes.
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    // Verify feature selection before compilation.
    #[cfg(feature = "renderer")]
    #[cfg(not(feature = "renderer-winit-fallback"))]
    if target_os != "linux" {
        panic!("MuJoCo-rs requires that the 'renderer-winit-fallback' Cargo feature is enabled on non-Linux platforms as only winit based renderer is available.");
    }

    println!("cargo::rerun-if-env-changed={MUJOCO_STATIC_LIB_PATH_VAR}");
    println!("cargo::rerun-if-env-changed={MUJOCO_DYN_LIB_PATH_VAR}");
    println!("cargo::rerun-if-env-changed={MUJOCO_DOWNLOAD_PATH_VAR}");
    println!("cargo::rerun-if-env-changed=PKG_CONFIG_PATH");

    let mujoco_static_link_dir = env::var(MUJOCO_STATIC_LIB_PATH_VAR).ok();
    let mujoco_dyn_link_dir = env::var(MUJOCO_DYN_LIB_PATH_VAR).ok();

    /* Static linking */
    let library_dir_path = if let Some(path) = mujoco_static_link_dir {
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
        link_static(&target_os);
        // C++ runtime is not embedded in regular static builds.
        // Do skip linking if we have a marker that tells us the CXX library is embedded.
        if target_os == "linux" && !path_lib_dir.join(EMBEDDED_CXX_MARKER).exists() {
            println!("cargo::rustc-link-lib=stdc++");
        }
        path_lib_dir
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

        link_dynamic();
        path_lib_dir
    }

    /* pkg-config / auto-download fallback */
    else {
        #[cfg_attr(
            all(windows, not(any(feature = "auto-download-static-mujoco", feature = "auto-download-dynamic-mujoco"))),
            expect(unused_variables, reason = "only pkg-config and the downloads use the version")
        )]
        let mujoco_version = env!("CARGO_PKG_VERSION").split_once("mj-").unwrap().1;

        // In build.rs, a target_os predicate evaluates against the HOST, not the target.
        const HOST_OS: &str = cfg_select! {
            target_os = "linux"   => "linux",
            target_os = "windows" => "windows",
            target_os = "macos"   => "macos",
            _ => compile_error!("unsupported host OS"),
        };

        // Only supported for static downloads. Dynamic linking (ang pkg-config) would need to
        // have the exact system libraries available.
        if HOST_OS != target_os && cfg!(not(feature = "auto-download-static-mujoco")) {
            panic!(
                "cross-compiling for '{target_os}' from '{HOST_OS}' requires \
                {MUJOCO_STATIC_LIB_PATH_VAR} or {MUJOCO_DYN_LIB_PATH_VAR} to be set"
            );
        }

        cfg_select! {
            // Un-official build of the statically linked library.
            feature = "auto-download-static-mujoco" => download_mujoco_static(mujoco_version, &target_os),

            // ---- Same-OS builds only below this point ----
            // TODO in future: make pkg-config an explicit Cargo feature.
            _ => {
                cfg_select! {
                    unix => pkg_config::Config::new()
                        .exactly_version(mujoco_version)
                        .probe("mujoco").map(
                            |library| library.link_paths.into_iter().next()
                                .or_else(||
                                    pkg_config::get_variable("mujoco", "libdir").ok()
                                        .filter(|libdir| !libdir.is_empty()).map(PathBuf::from)
                                )
                                .unwrap_or_else(|| panic!(
                                    "pkg-config located MuJoCo, but its 'mujoco.pc' gives neither a '-L' flag nor a \
                                    'libdir' variable --- set {MUJOCO_STATIC_LIB_PATH_VAR} or {MUJOCO_DYN_LIB_PATH_VAR} instead."
                            )))
                        .inspect_err(|err| eprintln!(  // output is suppressed until panics.
                            "{err}\n---------------- ^^^ pkg-config output ^^^ ----------------\n"
                        ))
                        .ok(),
                    _ => None
                }.unwrap_or_else(||  // pkg-config did not find library or we are on Windows
                    cfg_select! {
                        // Download the official build of the dynamically linked library.
                        feature = "auto-download-dynamic-mujoco" => {cfg_select! {
                            // We don't support macOS here due to the complexity of MuJoCo's packaging for macOS.
                            // Feel free to submit a PR if you believe this can be universally implemented easily.
                            target_os = "macos" => compile_error!("automatic downloads are not supported on macOS"),
                            _ => download_mujoco_dynamic(mujoco_version, HOST_OS)
                        }}

                        _ => {
                            panic!(
                                "Unable to locate MuJoCo via pkg-config and neither {MUJOCO_STATIC_LIB_PATH_VAR} \
                                nor {MUJOCO_DYN_LIB_PATH_VAR} is set.\
                                \nConsider enabling automatic download:\n\
                                \t- Dynamic linking (official):  cargo add mujoco-rs --features \"auto-download-dynamic-mujoco\"\n\
                                \t- Static linking (unofficial): cargo add mujoco-rs --features \"auto-download-static-mujoco\""
                            )
                        }
                    }
                )
            }
        }
    };

    // Metadata path for users to be able to detect which library path is used.
    let full_path = std::path::absolute(&library_dir_path).unwrap();
    let full_path_display = full_path.display();
    println!("cargo::metadata=lib_dir={full_path_display}");

    // Setup library search path
    println!("cargo::rustc-link-search={full_path_display}");

    #[cfg(feature = "ffi-regenerate")]
    build_dependencies::generate_ffi();
}

fn link_dynamic() {
    println!("cargo::rustc-link-lib=dylib=mujoco");
}

fn link_static(target_os: &str) {
    #[cfg(feature = "cpp-viewer")]
    {
        // The fork's libsimulate has GLFW embedded inside.
        println!("cargo::rustc-link-lib=static=simulate");
        // Frameworks that GLFW and simulate use
        if target_os == "macos" {
            for framework in ["Cocoa", "IOKit", "CoreFoundation", "CoreVideo", "OpenGL"] {
                println!("cargo::rustc-link-lib=framework={framework}");
            }
        }
        else if target_os == "windows" {
            for lib in ["user32", "gdi32", "shell32"] {
                println!("cargo::rustc-link-lib=dylib={lib}");
            }
        }
    }

    // whole-archive ensures mjz decoders are bundled.
    println!("cargo::rustc-link-lib=static:+whole-archive=mujoco");

    if target_os == "macos" {
        println!("cargo::rustc-link-lib=c++");
    }
}

/// Returns the path to the download directory, based on the environmental
/// variable (see [`MUJOCO_DOWNLOAD_PATH_VAR`]), or [`None`] if the path is not configured.
#[cfg(any(feature = "auto-download-dynamic-mujoco", feature = "auto-download-static-mujoco"))]
fn get_download_path_from_env() -> Option<PathBuf> {
    let download_path = PathBuf::from(std::env::var(MUJOCO_DOWNLOAD_PATH_VAR).ok()?);
    if download_path.is_relative() {
        panic!(
            "{MUJOCO_DOWNLOAD_PATH_VAR} must be an absolute path to the location \
            where MuJoCo will be downloaded and extracted (was given '{}').",
            download_path.display()
        )
    }

    Some(download_path)
}

/// Downloads the official MuJoCo dynamically linked library.
#[cfg(not(target_os = "macos"))]
#[cfg(feature = "auto-download-dynamic-mujoco")]
fn download_mujoco_dynamic(version: &str, host_os: &str) -> PathBuf {
    const MUJOCO_DOWNLOAD_LINK: &str = "https://github.com/google-deepmind/mujoco/releases/download";
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| {
        panic!(
            "unable to obtain the ARCHITECTURE information -- please manually download \
            MuJoCo and specify {MUJOCO_DYN_LIB_PATH_VAR} \
            (and potentially add the path to LD_LIBRARY_PATH)"
        )
    });

    let filename = format!(
        "mujoco-{version}-{host_os}-{target_arch}.{}",
        cfg_select! {
            unix => "tar.gz",
            _ => "zip"
        }
    );
    let download_url = format!("{MUJOCO_DOWNLOAD_LINK}/{version}/{filename}");
    let hash_verifier = |name: &str| {
        match name {
            "mujoco-3.17.0-linux-aarch64.tar.gz" => Some("55467a37b74e7d943899768aee820e6754a66811ae5ffa17d2768f0c40fc983a"),
            "mujoco-3.17.0-linux-x86_64.tar.gz" => Some("b3246fc698ca27e05ce36047fe2b6866fe23c9e95a26b3a84dd48f011b85e3e4"),
            "mujoco-3.17.0-windows-x86_64.zip" => Some("84d92054e767d0b24b7a8a5c7f3e34e883eb8e4bf7e8eefd7604dd09d999a517"),
            _ => {
                println!(
                    "cargo::warning=Skipping hash verification for '{name}' due to missed match case. \
                    Please report (to MuJoCo-rs GitHub issues) a bug for a missing configuration."
                );
                None
            }
        }
    };

    // Dynamically linked-libraries do not get saved into temporary directories due to linking and distribution reasons.
    // In any case, there is no reason to do that as we have statically linked builds available for this.
    let download_path = get_download_path_from_env()
        .unwrap_or_else(||
    {
        let os_example = cfg_select! {
                unix => format!("e.g., export {MUJOCO_DOWNLOAD_PATH_VAR}=\"$(realpath .)\""),
                target_os = "windows" => format!("e.g., $env:{MUJOCO_DOWNLOAD_PATH_VAR}=\"/full/absolute/path/\"")
            };

            panic!(
                "when Cargo feature 'auto-download-dynamic-mujoco' or auto-download-static-mujoco is enabled, \
                {MUJOCO_DOWNLOAD_PATH_VAR} must be set to an absolute path where MuJoCo \
                will be extracted --- {os_example}",
            );
    });

    link_dynamic();
    download_mujoco(version, download_url, &download_path, hash_verifier, "", ".mujoco")
}

/// Downloads the unofficial MuJoCo staticically linked library.
#[cfg(feature = "auto-download-static-mujoco")]
fn download_mujoco_static(version: &str, target_os: &str) -> PathBuf {
    const MUJOCO_DOWNLOAD_LINK: &str = "https://github.com/davidhozic/mujoco-fork/releases/download";

    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| {
        panic!(
            "unable to obtain the ARCHITECTURE information -- please manually download \
            MuJoCo and specify {MUJOCO_STATIC_LIB_PATH_VAR}."
        )
    });

    // Download either into the env-set path or into target/ out directory.
    // One subdirectory per target, because the archives extract to the same directory name.
    let download_path = get_download_path_from_env()
        .unwrap_or_else(|| env::var("OUT_DIR").unwrap().into())
        .join(format!("{target_os}-{target_arch}"));

    // Choose static/dynamic CRT here. Linux and macOS determine the CRT (C RunTime) at final binary linkage. 
    let (crt_type, extracted_suffix) = if target_os == "windows" && std::env::var("CARGO_CFG_TARGET_FEATURE")
        .is_ok_and(|features| features.split(',').any(|feature| feature == "crt-static"))
    {
        ("static", "-static-crt-static")
    } else { ("dynamic", "-static") };

    let filename_of_dep = |dep| format!(
        "{dep}-unofficial-{version}-static-{target_os}-{target_arch}-crt-{crt_type}.{}",
        if target_os == "windows" { "zip" } else { "tar.gz" }
    );

    let filename_mujoco = filename_of_dep("mujoco");
    let download_url_mujoco = format!("{MUJOCO_DOWNLOAD_LINK}/unofficial-{version}-static/{filename_mujoco}");
    let hash_verifier_mujoco = |name: &str| {
        match name {
            "mujoco-unofficial-3.17.0-static-linux-aarch64-crt-dynamic.tar.gz" => Some("481eeb359042dffe5417cdade701482b80fa38f6767bc49e6a2906264fa70cf0"),
            "mujoco-unofficial-3.17.0-static-linux-x86_64-crt-dynamic.tar.gz" => Some("f9b816afa9dd8c88dcd16b96a01a0d062b5ab39aef6f245a6270f424ecb67dbd"),
            "mujoco-unofficial-3.17.0-static-macos-aarch64-crt-dynamic.tar.gz" => Some("d8e01937fe9fff1c97a091168fcbd9029af927c27f5d39b099f314d9753dc768"),
            "mujoco-unofficial-3.17.0-static-macos-x86_64-crt-dynamic.tar.gz" => Some("c687452c97cee6096a93b8adf19e8a869e4d96763d345bf362d51cfe525c4da8"),
            "mujoco-unofficial-3.17.0-static-windows-aarch64-crt-dynamic.zip" => Some("50633ef5459aa21309c80e1679ffe0c29b708467df64011750f905a4944f0fd2"),
            "mujoco-unofficial-3.17.0-static-windows-aarch64-crt-static.zip" => Some("17308b7bfd4d7ca69e780b2b8b22297be7529c3a35971e40deeda67b9242708b"),
            "mujoco-unofficial-3.17.0-static-windows-x86_64-crt-dynamic.zip" => Some("78b50de6a84ebe3cd5176820e6fae6d08f50c44ff96670ab3fe03ddbf6bfabd5"),
            "mujoco-unofficial-3.17.0-static-windows-x86_64-crt-static.zip" => Some("763f6dcacdf904c30c577806c8cfa1fd9025c7c711bc56dea75474ef257b65b5"),
            _ => {
                println!(
                    "cargo::warning=Skipping hash verification for '{name}' due to missed match case. \
                    Please report (to MuJoCo-rs GitHub issues) a bug for a missing configuration."
                );
                None
            }
        }
    };

    link_static(target_os);
    #[cfg(feature = "cpp-viewer")]  // Also download libsimulate dependency
    {
        let filename_simulate = filename_of_dep("simulate");
        let download_url_simulate = format!("{MUJOCO_DOWNLOAD_LINK}/unofficial-{version}-static/{filename_simulate}");
        let hash_verifier_simulate = |name: &str| {
            match name {
                "simulate-unofficial-3.17.0-static-linux-aarch64-crt-dynamic.tar.gz" => Some("452a5f9a3e43483ebbdb3b7d37f9c71b4b1d0cc8a78cb9fb96455a5eb97d08d6"),
                "simulate-unofficial-3.17.0-static-linux-x86_64-crt-dynamic.tar.gz" => Some("03140a6205bd4154186549084ca15b736be22559e33ec1e6e77b3f586eef2e8c"),
                "simulate-unofficial-3.17.0-static-macos-aarch64-crt-dynamic.tar.gz" => Some("194d92164bfb6da0761ccfef84a7dc2b936ced210300bf8403e7f9ddec7bdd76"),
                "simulate-unofficial-3.17.0-static-macos-x86_64-crt-dynamic.tar.gz" => Some("bbf97afb047ce47da3b1b9dd051415d4e7aaf0608e9fe9328e62d3c40d94d435"),
                "simulate-unofficial-3.17.0-static-windows-aarch64-crt-dynamic.zip" => Some("60fc5cb17f7d9783bfb452add4e87154eed221fa2ee214d0269200ec6455dddf"),
                "simulate-unofficial-3.17.0-static-windows-aarch64-crt-static.zip" => Some("58a0097d4968d30165217cb8c067f73fadf5bf6dedc9828c519cabea6b22a050"),
                "simulate-unofficial-3.17.0-static-windows-x86_64-crt-dynamic.zip" => Some("67859e2bf0ff07e3d2ce86594fac4ecce92ab88e6c08515bb22dd185dc138631"),
                "simulate-unofficial-3.17.0-static-windows-x86_64-crt-static.zip" => Some("b497a704076d2aeb7f378472f9bd6991ceb596c4de9b9255c701fb811fff8d5f"),
                _ => {
                    println!(
                        "cargo::warning=Skipping hash verification for '{name}' due to missed match case. \
                        Please report (to MuJoCo-rs GitHub issues) a bug for a missing configuration."
                    );
                    None
                }
            }
        };
        download_mujoco(version, download_url_simulate, &download_path, hash_verifier_simulate, extracted_suffix, ".simulate");
    }

    download_mujoco(version, download_url_mujoco, &download_path, hash_verifier_mujoco, extracted_suffix, ".mujoco")
}

/// Downloads MuJoCo of version `mujoco_version` from `download_url`.
/// For security reasons, `hash_verifier` is used to verify the hash of the downloaded archive based on the
/// pre-defined hash inside this build script.
#[cfg(any(feature = "auto-download-dynamic-mujoco", feature = "auto-download-static-mujoco"))]
fn download_mujoco(
    mujoco_version: &str,
    download_url: String,
    download_path: &Path,
    hash_verifier: fn(&str) -> Option<&'static str>,
    extracted_suffix: &str,
    touchfile_name: &str,
) -> PathBuf {
    use std::io::{BufReader, Read};
    use sha2::Digest;

    let outdirname = download_path.join(format!("mujoco-{mujoco_version}{extracted_suffix}"));
    let libdir_path = outdirname.join("lib");
    let touchfile_path = outdirname.join(touchfile_name);

    std::fs::create_dir_all(download_path).unwrap_or_else(
        |err| panic!("failed to create download directory '{}' ({err})", download_path.display())
    );
    let lock_path = download_path.join(".mujoco-rs.lock");
    let lock = File::create(&lock_path).unwrap_or_else(
        |err| panic!("could not create lock file '{}' ({err})", lock_path.display())
    );

    // Wait for prior builds to finish.
    lock.lock().unwrap_or_else(|err| panic!("could not lock '{}' ({err})", lock_path.display()));

    if !touchfile_path.exists() {
        // Download the archive.
        let mut response = ureq::get(&download_url).call().unwrap_or_else(
            |err| panic!("failed to download '{download_url}' ({err})")
        );
        let mut body_reader = response.body_mut().as_reader();

        let download_file_name = download_url.rsplit_once("/").unwrap().1;
        let download_path = download_path.join(download_file_name);
        let mut file = File::create(&download_path).unwrap_or_else(
            |err| panic!("could not save archive '{}' ({err})", download_path.display())
        );
        std::io::copy(&mut body_reader, &mut file).unwrap_or_else(
            |err| panic!("failed to copy archive contents to '{}' ({err})", download_path.display())
        );

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
        let calculated_hash: String = hasher.finalize()
            .iter().map(|byte| format!("{byte:02x}")).collect();

        let maybe_expected_hash = hash_verifier(download_file_name);

        // Check that the hash is the same.
        if let Some(hash) = maybe_expected_hash && hash != calculated_hash {
            panic!(
                "sha256sum ({calculated_hash}) of '{}' does not match the expected in \
                MuJoCo-rs build script ({hash}). Stopping for security reasons.",
                download_path.display()
            );
        }

        // Extract.
        if download_file_name.ends_with(".zip") {
            extract_zip(&download_path, &outdirname);
        } else {
            extract_tar_gz(&download_path);
        }

        std::fs::remove_file(&download_path).unwrap_or_else(
            |err| panic!("failed to delete archive '{}' ({err})", download_path.display())
        );

        std::fs::File::create(&touchfile_path).unwrap_or_else(
            |err| panic!("could not create touchfile {} ({err})", touchfile_path.display())
        );
    }
    libdir_path
}

#[cfg(any(feature = "auto-download-dynamic-mujoco", feature = "auto-download-static-mujoco"))]
fn extract_zip(filename: &Path, outdirname: &Path) {
    let file = File::open(filename).unwrap_or_else(|err|
        panic!("failed to open archive '{}' ({err}).", filename.display())
    );
    let mut zip = zip::ZipArchive::new(file).unwrap_or_else(|err|
        panic!("failed to read ZIP metadata ({err}).")
    );

    for i in 0..zip.len() {
        let mut zipfile = zip.by_index(i).unwrap();
        let mut path = if let Some(path) = zipfile.enclosed_name() { path } else {
            println!("cargo::warning=Skipped potentially unsafe ZIP entry '{}' during extraction", zipfile.name());
            continue;
        };

        if zipfile.is_file() {
            // ZIP archives have no top-level folder, thus we need to put everything in a new directory.
            // This makes results match the .tar.gz archives.
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

#[cfg(any(feature = "auto-download-dynamic-mujoco", feature = "auto-download-static-mujoco"))]
fn extract_tar_gz(filename: &Path) {
    let file = File::open(filename).unwrap_or_else(
        |err| panic!("failed to open '{}' ({err})", filename.display())
    );
    let tar = flate2::read::GzDecoder::new(file);
    let mut archive = tar::Archive::new(tar);
    archive.unpack(filename.parent().unwrap()).expect("failed to unpack MuJoCo archive");
}
