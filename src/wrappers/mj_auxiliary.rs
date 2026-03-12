//! MuJoCo's auxiliary structs.
use std::ffi::{c_void, CString};
use std::mem::MaybeUninit;
use std::ptr;

use crate::error::MjVfsError;
use crate::mujoco_c::*;


/***********************************************************************************************************************
** MjVisual
***********************************************************************************************************************/
pub type MjVisual = mjVisual;
impl Default for MjVisual {
    fn default() -> Self {
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
pub type MjStatistic = mjStatistic;

/***********************************************************************************************************************
** MjContact
***********************************************************************************************************************/
pub type MjContact = mjContact;

// SAFETY: mjContact_ contains only f64 and c_int fields, which are all zero-valid.
unsafe impl bytemuck::Zeroable for mjContact_ {}

/***********************************************************************************************************************
** MjLROpt
***********************************************************************************************************************/
pub type MjLROpt = mjLROpt;
impl Default for MjLROpt {
    fn default() -> Self {
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
    /// - [`MjVfsError::Full`] if the VFS has no more room.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// A panic will occur if `directory` or `filename` contain `\0` characters.
    pub fn add_from_file(&mut self, directory: Option<&str>, filename: &str) -> Result<(), MjVfsError> {
        let c_directory = directory.map(|d| CString::new(d).unwrap());
        let c_filename = CString::new(filename).unwrap();
        Self::handle_add_result(unsafe {
            mj_addFileVFS(
                self.ffi_mut(),
                c_directory.as_ref().map_or(ptr::null(), |d| d.as_ptr()),
                c_filename.as_ptr()
            )
        })
    }

    /// Adds a file to the virtual file system from a byte buffer.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::Full`] if the VFS has no more room.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if MuJoCo fails to register the buffer.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn add_from_buffer(&mut self, filename: &str, buffer: &[u8]) -> Result<(), MjVfsError> {
        let c_filename = CString::new(filename).unwrap();
        Self::handle_add_result(unsafe {
            mj_addBufferVFS(
                self.ffi_mut(), c_filename.as_ptr(),
                buffer.as_ptr() as *const c_void, buffer.len() as i32
            )
        })
    }

    /// Removes a file from the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::NotFound`] if the file doesn't exist.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn delete_file(&mut self, filename: &str) -> Result<(), MjVfsError> {
        let c_filename = CString::new(filename).unwrap();
        unsafe {
            Self::handle_remove_result(
                mj_deleteFileVFS(self.ffi_mut(), c_filename.as_ptr())
            )
        }
    }

    fn handle_add_result(result: i32) -> Result<(), MjVfsError> {
        match result {
            0 => Ok(()),
            1 => Err(MjVfsError::Full),
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
        assert!(vfs.add_from_file(None, REAL_FILE_NAME).is_ok());
        drop(vfs);

        /* With directory */
        vfs = MjVfs::new();
        assert!(vfs.add_from_file(Some("./"), REAL_FILE_NAME).is_ok());

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
}
