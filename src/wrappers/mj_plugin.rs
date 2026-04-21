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

    use crate::error::MjPluginError;

    /// Verifies that [`load_all_plugin_libraries`] works.
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

