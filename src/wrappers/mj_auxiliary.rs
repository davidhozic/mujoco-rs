//! MuJoCo's auxiliary structs.
use std::ffi::{c_void, CString};
use std::mem::MaybeUninit;
use std::path::Path;
use std::ptr;

use crate::error::MjVfsError;
use crate::mujoco_c::*;


/***********************************************************************************************************************
** MjVisual
***********************************************************************************************************************/
/// Visual properties of the model (headlight, rgba defaults, scale, etc.).
pub type MjVisual = mjVisual;
impl Default for MjVisual {
    fn default() -> Self {
        // SAFETY: mj_defaultVisual initializes all fields of an uninitialized mjVisual to valid
        // defaults; the pointer from MaybeUninit is properly aligned and writable.
        unsafe {
            let mut s = MaybeUninit::uninit();
            mj_defaultVisual(s.as_mut_ptr());
            s.assume_init()
        }
    }
}

/***********************************************************************************************************************
** MjStatistic
***********************************************************************************************************************/
/// Model statistics (center, extent, mean body mass, mean inertia, etc.).
pub type MjStatistic = mjStatistic;

/***********************************************************************************************************************
** MjContact
***********************************************************************************************************************/
/// Contact point data (position, frame, friction/solver parameters, geom/flex ids, etc.).
pub type MjContact = mjContact;

// SAFETY: mjContact_ contains only f64 and c_int fields, which are all zero-valid.
unsafe impl bytemuck::Zeroable for mjContact_ {}

/***********************************************************************************************************************
** MjLROpt
***********************************************************************************************************************/
/// Options for the length-range computation of actuator length ranges.
pub type MjLROpt = mjLROpt;
impl Default for MjLROpt {
    fn default() -> Self {
        // SAFETY: mj_defaultLROpt initializes all fields of an uninitialized mjLROpt to valid
        // defaults; the pointer from MaybeUninit is properly aligned and writable.
        unsafe {
            let mut s = MaybeUninit::uninit();
            mj_defaultLROpt(s.as_mut_ptr());
            s.assume_init()
        }
    }
}

/***********************************************************************************************************************
** MjTask
***********************************************************************************************************************/
// wrapper_with_default!(mjTask);


/***********************************************************************************************************************
** MjVfs
***********************************************************************************************************************/
/// Wrapper around the virtual-file system.
#[derive(Debug)]
pub struct MjVfs {
    ffi: Box<mjVFS>
}

impl MjVfs {
    /// Creates a new, empty virtual file system.
    pub fn new() -> Self {
        // SAFETY: mj_defaultVFS initializes all fields of an uninitialized mjVFS to valid defaults;
        // the pointer from Box::new_uninit is properly aligned and writable.
        unsafe {
            let mut maybe_uninit = Box::new_uninit();
            mj_defaultVFS(maybe_uninit.as_mut_ptr());
            Self { ffi: maybe_uninit.assume_init() }
        }
    }

    /// Adds a file from disk to the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When `directory` or `filename` contain interior `\0` characters.
    #[deprecated(since = "3.0.0", note = "use `add_file` or `add_file_from` instead")]
    pub fn add_from_file(&mut self, directory: Option<&str>, filename: &str) -> Result<(), MjVfsError> {
        match directory {
            Some(d) => self.add_file_from(d, filename),
            None => self.add_file(filename),
        }
    }

    /// Adds a file from disk to the virtual file system, searching in `directory`.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `directory` or `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When `directory` or `filename` contain interior `\0` characters.
    pub fn add_file_from<T: AsRef<Path>, U: AsRef<Path>>(&mut self, directory: T, filename: U) -> Result<(), MjVfsError> {
        let c_directory = CString::new(
            directory.as_ref().to_str().ok_or(MjVfsError::InvalidUtf8Path)?
        ).unwrap();
        let c_filename = CString::new(
            filename.as_ref().to_str().ok_or(MjVfsError::InvalidUtf8Path)?
        ).unwrap();
        Self::handle_add_result(
            unsafe {
            mj_addFileVFS(
                self.ffi_mut(),
                c_directory.as_ptr(),
                c_filename.as_ptr()
            )
        })
    }

    /// Adds a file from disk to the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When `filename` contains interior `\0` characters.
    pub fn add_file<T: AsRef<Path>>(&mut self, filename: T) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename.as_ref().to_str().ok_or(MjVfsError::InvalidUtf8Path)?
        ).unwrap();
        Self::handle_add_result(unsafe {
            mj_addFileVFS(
                self.ffi_mut(),
                ptr::null(),
                c_filename.as_ptr()
            )
        })
    }

    /// Adds a file to the virtual file system from a byte buffer.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if MuJoCo fails to register the buffer.
    /// - [`MjVfsError::BufferTooLarge`] if `buffer` exceeds `i32::MAX` bytes.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains interior `\0` characters.
    pub fn add_from_buffer<T: AsRef<Path>>(&mut self, filename: T, buffer: &[u8]) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename.as_ref().to_str().ok_or(MjVfsError::InvalidUtf8Path)?
        ).unwrap();
        let nbuffer = i32::try_from(buffer.len()).map_err(|_| MjVfsError::BufferTooLarge)?;
        Self::handle_add_result(unsafe {
            mj_addBufferVFS(
                self.ffi_mut(), c_filename.as_ptr(),
                buffer.as_ptr() as *const c_void, nbuffer
            )
        })
    }

    /// Removes a file from the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::NotFound`] if the file doesn't exist.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains interior `\0` characters.
    pub fn delete_file<T: AsRef<Path>>(&mut self, filename: T) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename.as_ref().to_str().ok_or(MjVfsError::InvalidUtf8Path)?
        ).unwrap();
        unsafe {
            Self::handle_remove_result(
                mj_deleteFileVFS(self.ffi_mut(), c_filename.as_ptr())
            )
        }
    }

    /// Check if file exists in VFS in the given directory.
    /// 
    /// A mutable borrow is required due to the internal mutex. 
    /// 
    /// # Panics
    /// When `name` or `directory` contain path of non-UTF elements or with null elements.
    pub fn contains_file_in<T: AsRef<Path>>(&mut self, directory: T, name: T) -> bool {
        let c_name = if let Some(name) = name.as_ref().to_str() {
            CString::new(name).unwrap()
        } else {
            return false;
        };

        let directory = if let Some(directory) = directory.as_ref().to_str() {
            CString::new(directory).unwrap()
        } else {
            return false
        }; 

        unsafe { mj_containsFileVFS(
            self.ffi_mut(),
            directory.as_ptr(),
            c_name.as_ptr()
        ) != 0 }
    }

    /// Check if file exists in VFS.
    /// 
    /// Use [`MjVfs::contains_file_in`] to search within a directory.
    /// 
    /// A mutable borrow is required due to the internal mutex. 
    /// 
    /// # Panics
    /// When `name` or `directory` contain path of non-UTF elements or with null elements.
    pub fn contains_file<T: AsRef<Path>>(&mut self, name: T) -> bool {
        let c_name = if let Some(name) = name.as_ref().to_str() {
            CString::new(name).unwrap()
        } else {
            return false;
        };

        unsafe { mj_containsFileVFS(
            self.ffi_mut(),
            ptr::null(),
            c_name.as_ptr()
        ) != 0 }
    }

    fn handle_add_result(result: i32) -> Result<(), MjVfsError> {
        match result {
            0 => Ok(()),
            2 => Err(MjVfsError::AlreadyExists),
            -1 => Err(MjVfsError::LoadFailed),
            code => Err(MjVfsError::Unknown(code))
        }
    }

    fn handle_remove_result(result: i32) -> Result<(), MjVfsError> {
        match result {
            0 => Ok(()),
            -1 => Err(MjVfsError::NotFound),
            code => Err(MjVfsError::Unknown(code))
        }
    }

    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjVFS {
        &self.ffi
    }

    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjVFS {
        &mut self.ffi
    }
}

impl Default for MjVfs {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: MjVfs owns its data exclusively (no shared mutable aliasing)
// and the underlying mjVFS does not use thread-local state.
unsafe impl Send for MjVfs {}
unsafe impl Sync for MjVfs {}

impl Drop for MjVfs {
    fn drop(&mut self) {
        // SAFETY: self.ffi is a valid pointer to a live mjVFS; called exactly once in Drop.
        unsafe {
            mj_deleteVFS(self.ffi.as_mut());
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::wrappers::MjModel;
    use super::*;
    use std::fs;
    use std::path::{Path, PathBuf};

    const RAW_FILE_DATA: &str = "
<mujoco>
    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        <body name=\"ball\">
        <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
        </body>
        
        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>
    </worldbody>
</mujoco>";

    const RAW_FILE_NAME: &str = "mujoco-rs-name.txt";
    const REAL_FILE_NAME: &str = "mujoco-rs-real-name.txt";

    #[test]
    fn test_vfs_file_add_buffer() {
        let mut vfs = MjVfs::new();

        /* Add from the buffer */
        assert!(vfs.add_from_buffer(RAW_FILE_NAME, RAW_FILE_DATA.as_bytes()).is_ok());
        /* Double write should error */
        assert!(vfs.add_from_buffer(RAW_FILE_NAME, RAW_FILE_DATA.as_bytes()).is_err());

        /* Test whether the model is actually loaded correctly */
        assert!(MjModel::from_xml_vfs(RAW_FILE_NAME, &vfs).is_ok());
    }

    #[test]
    fn test_vfs_file_add_file() {
        let mut vfs;

        /* Add from a file */
        fs::write(REAL_FILE_NAME, RAW_FILE_DATA).expect("could not write the file to disk");

        /* No directory */
        vfs = MjVfs::new();
        assert!(!vfs.contains_file(REAL_FILE_NAME));
        assert!(vfs.add_file(REAL_FILE_NAME).is_ok());
        assert!(vfs.contains_file(REAL_FILE_NAME));
        drop(vfs);

        /* With directory */
        vfs = MjVfs::new();
        assert!(!vfs.contains_file_in("./", REAL_FILE_NAME));
        assert!(vfs.add_file_from("./", REAL_FILE_NAME).is_ok());
        assert!(vfs.contains_file_in("./", REAL_FILE_NAME));

        fs::remove_file(REAL_FILE_NAME).expect("could not delete the file from disk");

        /* Test whether the model is actually loaded correctly */
        assert!(MjModel::from_xml_vfs(REAL_FILE_NAME, &vfs).is_ok());
    }

    #[test]
    fn test_vfs_file_remove() {
        let mut vfs = MjVfs::new();

        /* Add some file */
        assert!(vfs.add_from_buffer(RAW_FILE_NAME, RAW_FILE_DATA.as_bytes()).is_ok());

        /* Remove it once (no error should occur) */
        assert!(vfs.delete_file(RAW_FILE_NAME).is_ok());

        /* Remove it once (an error should occur) */
        assert!(vfs.delete_file(RAW_FILE_NAME).is_err());
    }


    /// Tests that deleting a nonexistent file from VFS returns the `NotFound` error variant.
    #[test]
    fn test_vfs_delete_nonexistent() {
        let mut vfs = MjVfs::new();
        let err = vfs.delete_file("does_not_exist.xml").unwrap_err();
        assert!(matches!(err, MjVfsError::NotFound));
    }

    /// Tests adding a model buffer to VFS and loading it back: verifies the model is
    /// parsed correctly and has the expected body count.
    #[test]
    fn test_vfs_buffer_round_trip() {
        const CUSTOM_MODEL: &str = "
<mujoco>
  <worldbody>
    <body name='extra_body'>
      <geom size='0.1'/>
    </body>
  </worldbody>
</mujoco>";

        let mut vfs = MjVfs::new();
        vfs.add_from_buffer("round_trip.xml", CUSTOM_MODEL.as_bytes()).unwrap();

        let model = MjModel::from_xml_vfs("round_trip.xml", &vfs).unwrap();
        // 2 bodies: worldbody + extra_body
        assert_eq!(model.ffi().nbody, 2);
        assert!(model.body("extra_body").is_some());
    }

    /// Adding the same filename to a VFS twice must return `AlreadyExists`.
    #[test]
    fn test_vfs_add_buffer_already_exists() {
        let mut vfs = MjVfs::new();
        vfs.add_from_buffer(RAW_FILE_NAME, RAW_FILE_DATA.as_bytes())
            .expect("first add should succeed");
        let err = vfs
            .add_from_buffer(RAW_FILE_NAME, RAW_FILE_DATA.as_bytes())
            .expect_err("second add with same name should fail");
        assert!(
            matches!(err, MjVfsError::AlreadyExists),
            "expected AlreadyExists, got {err:?}"
        );
    }

    /// `add_from_buffer` accepts `&Path`, `PathBuf`, `&str`, and `String` as the filename.
    #[test]
    fn test_vfs_add_from_buffer_path_types() {
        let mut vfs = MjVfs::new();

        // &str
        vfs.add_from_buffer("str.xml", RAW_FILE_DATA.as_bytes()).unwrap();
        assert!(MjModel::from_xml_vfs("str.xml", &vfs).is_ok());

        // String
        vfs.add_from_buffer(String::from("string.xml"), RAW_FILE_DATA.as_bytes()).unwrap();
        assert!(MjModel::from_xml_vfs("string.xml", &vfs).is_ok());

        // &Path
        vfs.add_from_buffer(Path::new("path.xml"), RAW_FILE_DATA.as_bytes()).unwrap();
        assert!(MjModel::from_xml_vfs("path.xml", &vfs).is_ok());

        // PathBuf
        vfs.add_from_buffer(PathBuf::from("pathbuf.xml"), RAW_FILE_DATA.as_bytes()).unwrap();
        assert!(MjModel::from_xml_vfs("pathbuf.xml", &vfs).is_ok());
    }

    /// `delete_file` accepts `&Path`, `PathBuf`, `&str`, and `String` as the filename.
    #[test]
    fn test_vfs_delete_file_path_types() {
        let mut vfs = MjVfs::new();

        // Add four files, then delete each with a different path type
        for name in &["del_str.xml", "del_string.xml", "del_path.xml", "del_pathbuf.xml"] {
            vfs.add_from_buffer(*name, RAW_FILE_DATA.as_bytes()).unwrap();
        }

        vfs.delete_file("del_str.xml").unwrap();
        vfs.delete_file(String::from("del_string.xml")).unwrap();
        vfs.delete_file(Path::new("del_path.xml")).unwrap();
        vfs.delete_file(PathBuf::from("del_pathbuf.xml")).unwrap();

        // All deleted -- re-deleting any should fail
        assert!(matches!(vfs.delete_file("del_str.xml").unwrap_err(), MjVfsError::NotFound));
    }

    /// `add_file` accepts `&Path`, `PathBuf`, `&str`, and `String`.
    #[test]
    fn test_vfs_add_file_path_types() {
        const FILE: &str = "mujoco-rs-add-file-path-types.xml";
        fs::write(FILE, RAW_FILE_DATA).expect("could not write temp file");

        let mut vfs;

        // &str
        vfs = MjVfs::new();
        vfs.add_file(FILE).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // String
        vfs = MjVfs::new();
        vfs.add_file(String::from(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // &Path
        vfs = MjVfs::new();
        vfs.add_file(Path::new(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // PathBuf
        vfs = MjVfs::new();
        vfs.add_file(PathBuf::from(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        fs::remove_file(FILE).expect("could not clean up temp file");
    }

    /// `add_file_from` accepts `&Path`, `PathBuf`, `&str`, and `String` for both arguments.
    #[test]
    fn test_vfs_add_file_from_path_types() {
        const FILE: &str = "mujoco-rs-add-file-from-path-types.xml";
        fs::write(FILE, RAW_FILE_DATA).expect("could not write temp file");

        let mut vfs;

        // &str, &str
        vfs = MjVfs::new();
        vfs.add_file_from("./", FILE).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // String, PathBuf
        vfs = MjVfs::new();
        vfs.add_file_from(String::from("./"), PathBuf::from(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // &Path, String
        vfs = MjVfs::new();
        vfs.add_file_from(Path::new("./"), String::from(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // PathBuf, &Path
        vfs = MjVfs::new();
        vfs.add_file_from(PathBuf::from("./"), Path::new(FILE)).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        fs::remove_file(FILE).expect("could not clean up temp file");
    }

    /// The deprecated `add_from_file` still delegates correctly.
    #[test]
    fn test_vfs_deprecated_add_from_file() {
        const FILE: &str = "mujoco-rs-deprecated-add-from-file.xml";
        fs::write(FILE, RAW_FILE_DATA).expect("could not write temp file");

        let mut vfs;

        // With directory (delegates to add_file_from)
        vfs = MjVfs::new();
        #[allow(deprecated)]
        vfs.add_from_file(Some("./"), FILE).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        // Without directory (delegates to add_file)
        vfs = MjVfs::new();
        #[allow(deprecated)]
        vfs.add_from_file(None, FILE).unwrap();
        assert!(MjModel::from_xml_vfs(FILE, &vfs).is_ok());

        fs::remove_file(FILE).expect("could not clean up temp file");
    }
}
