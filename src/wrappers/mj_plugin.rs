//! MuJoCo plugin library loading.
use std::ffi::{CString, c_char, c_int};
use std::path::Path;

use crate::mujoco_c::{mj_loadAllPluginLibraries, mj_loadPluginLibrary};
use crate::error::MjPluginError;

/// Callback invoked by [`load_all_plugin_libraries`] for each loaded library.
///
/// Parameters: `filename`, `first` plugin index, `count` of plugins registered.
pub type MjPluginLibraryLoadCallback =
    Option<unsafe extern "C" fn(*const c_char, c_int, c_int)>;

/// Loads a single MuJoCo plugin shared library.
///
/// # Errors
/// Returns [`MjPluginError`] if `path` is not valid UTF-8 or contains a null byte.
///
/// # Examples
///
/// Load only the STL decoder plugin before loading a model with STL meshes:
///
/// ```no_run
/// use mujoco_rs::prelude::*;
///
/// load_plugin_library("path/to/mujoco/bin/mujoco_plugin/libstl_decoder.so")
///     .expect("failed to load STL decoder plugin");
///
/// let model = MjModel::from_xml("model.xml").expect("could not load the model");
/// ```
pub fn load_plugin_library<P: AsRef<Path>>(path: P) -> Result<(), MjPluginError> {
    let s = path.as_ref().to_str().ok_or(MjPluginError::InvalidUtf8Path)?;
    let c = CString::new(s).map_err(|_| MjPluginError::NullBytePath)?;
    // SAFETY: `c` is a valid null-terminated string; MuJoCo does not retain the pointer.
    unsafe { mj_loadPluginLibrary(c.as_ptr()) };
    Ok(())
}

/// Loads all MuJoCo plugin shared libraries found in `directory`.
///
/// Pass `None` for `callback` to omit per-library notification.
///
/// # Errors
/// Returns [`MjPluginError`] if `directory` is not valid UTF-8 or contains a null byte.
///
/// # Examples
///
/// Load all MuJoCo plugins from the plugin directory before loading a model with
/// STL or OBJ meshes:
///
/// ```no_run
/// use mujoco_rs::prelude::*;
///
/// // Load all MuJoCo plugins from the plugin directory.
/// // Adjust the path to match your MuJoCo installation.
/// load_all_plugin_libraries("path/to/mujoco/bin/mujoco_plugin", None)
///     .expect("failed to load plugin libraries");
///
/// let model = MjModel::from_xml("model.xml").expect("could not load the model");
/// ```
pub fn load_all_plugin_libraries<P: AsRef<Path>>(
    directory: P,
    callback: MjPluginLibraryLoadCallback,
) -> Result<(), MjPluginError> {
    let s = directory.as_ref().to_str().ok_or(MjPluginError::InvalidUtf8Path)?;
    let c = CString::new(s).map_err(|_| MjPluginError::NullBytePath)?;
    // SAFETY: `c` is a valid null-terminated string. `callback`, if non-null, matches the
    // expected signature and is valid for the duration of the call.
    unsafe { mj_loadAllPluginLibraries(c.as_ptr(), callback) };
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{load_all_plugin_libraries, load_plugin_library};

    use crate::wrappers::mj_auxiliary::MjVfs;
    use crate::wrappers::mj_model::MjModel;

    use crate::error::MjPluginError;

    /// Minimal binary STL: a tetrahedron with four triangular faces.
    ///
    /// Vertices: V0=(0,0,0), V1=(1,0,0), V2=(0,1,0), V3=(0,0,1).
    /// Format: 80-byte header | u32 triangle count | 4 x (normal + 3 vertices + attribute).
    const TETRA_STL: &[u8] = &[
        // 80-byte header (unused, all zeros)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // triangle count: 4 (little-endian u32)
        0x04, 0x00, 0x00, 0x00,
        // face 0: normal (0,0,-1), V0 V2 V1
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xbf,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        // face 1: normal (0,-1,0), V0 V1 V3
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xbf, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f,
        0x00, 0x00,
        // face 2: normal (-1,0,0), V0 V3 V2
        0x00, 0x00, 0x80, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        // face 3: normal (1/sqrt(3), 1/sqrt(3), 1/sqrt(3)), V1 V2 V3
        0x3a, 0xcd, 0x13, 0x3f, 0x3a, 0xcd, 0x13, 0x3f, 0x3a, 0xcd, 0x13, 0x3f,
        0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f,
        0x00, 0x00,
    ];

    const STL_MESH_XML: &str = r#"<mujoco>
  <asset>
    <mesh name="m" file="tetra.stl"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="m"/>
  </worldbody>
</mujoco>"#;

    /// Verifies that [`load_all_plugin_libraries`] enables loading a model with an STL mesh file.
    ///
    /// Requires `MUJOCO_DYNAMIC_LINK_DIR` to be set (the crate's standard build-time env var).
    /// The test is skipped (not failed) when the variable is absent.
    #[test]
    fn load_all_plugin_libraries_enables_stl_mesh_loading() {
        let lib_dir = match std::env::var("MUJOCO_DYNAMIC_LINK_DIR") {
            Ok(d) => d,
            Err(_) => return,
        };
        let plugin_dir = std::path::Path::new(&lib_dir)
            .parent()
            .expect("MUJOCO_DYNAMIC_LINK_DIR should have a parent directory")
            .join("bin/mujoco_plugin");

        load_all_plugin_libraries(&plugin_dir, None).expect("plugin dir should load");

        let mut vfs = MjVfs::new();
        vfs.add_from_buffer("tetra.stl", TETRA_STL).expect("add STL to VFS");
        vfs.add_from_buffer("model.xml", STL_MESH_XML.as_bytes()).expect("add MJCF to VFS");

        MjModel::from_xml_vfs("model.xml", &vfs).expect("STL mesh should load after plugin is registered");
    }

    #[test]
    fn load_plugin_library_null_byte_error() {
        let result = load_plugin_library("path\0with\0nulls");
        assert!(matches!(result, Err(MjPluginError::NullBytePath)));
    }

    #[test]
    fn load_all_plugin_libraries_null_byte_error() {
        let result = load_all_plugin_libraries("dir\0null", None);
        assert!(matches!(result, Err(MjPluginError::NullBytePath)));
    }

    #[cfg(unix)]
    #[test]
    fn load_plugin_library_invalid_utf8_error() {
        use std::os::unix::ffi::OsStrExt;
        use std::ffi::OsStr;
        let path = OsStr::from_bytes(&[0xFF, 0xFE]);
        assert!(matches!(
            load_plugin_library(path),
            Err(MjPluginError::InvalidUtf8Path)
        ));
    }

    #[cfg(unix)]
    #[test]
    fn load_all_plugin_libraries_invalid_utf8_error() {
        use std::os::unix::ffi::OsStrExt;
        use std::ffi::OsStr;
        let path = OsStr::from_bytes(&[0xFF, 0xFE]);
        assert!(matches!(
            load_all_plugin_libraries(path, None),
            Err(MjPluginError::InvalidUtf8Path)
        ));
    }
}

