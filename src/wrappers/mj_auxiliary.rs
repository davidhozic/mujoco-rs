//! MuJoCo's auxiliary structs.
use std::io::{self, Error, ErrorKind};
use std::ffi::{c_void, CString};
use std::mem::MaybeUninit;
use std::ptr;


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
pub struct MjVfs {
    ffi: Box<mjVFS>
}

impl MjVfs {
    pub fn new() -> Self {
        unsafe {
            let mut maybe_uninit = Box::new_uninit();
            mj_defaultVFS(maybe_uninit.as_mut_ptr());
            Self { ffi: maybe_uninit.assume_init() }
        }
    }

    /// Adds a file to the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// Returns an error if the file already exists or fails to load.
    /// # Panics
    /// A panic will occur if `directory` or `filename` contain `\0` characters.
    pub fn add_from_file(&mut self, directory: Option<&str>, filename: &str) -> io::Result<()> {
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

    /// Adds a file to the virtual file system from a buffer.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// Returns an error if the file already exists or fails to load.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn add_from_buffer(&mut self, filename: &str, buffer: &[u8]) -> io::Result<()> {
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
    /// Returns an error of kind `NotFound` if the file doesn't exist.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn delete_file(&mut self, filename: &str) -> io::Result<()> {
        let c_filename = CString::new(filename).unwrap();
        unsafe {
            Self::handle_remove_result(
                mj_deleteFileVFS(self.ffi_mut(), c_filename.as_ptr())
            )
        }
    }

    fn handle_add_result(result: i32) -> io::Result<()> {
        match result {
            0 => Ok(()),
            2 => Err(Error::new(ErrorKind::AlreadyExists, "repeated name")),
            -1 => Err(Error::new(ErrorKind::InvalidData, "failed to load")),
            _ => Err(Error::new(ErrorKind::Other, "unknown MuJoCo error"))
        }
    }

    /// Mounts a directory into the VFS.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// Returns an error if the mount fails.
    /// # Panics
    /// When `filepath` contain `\0` characters, a panic occurs.
    pub fn mount(&mut self, filepath: &str) -> io::Result<()> {
        let c_filepath = CString::new(filepath).unwrap();
        Self::handle_add_result(unsafe {
            mj_mountVFS(
                self.ffi_mut(),
                c_filepath.as_ptr(),
                ptr::null()
            )
        })
    }

    /// Unmounts a directory from the VFS.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// Returns an error if the directory is not mounted.
    /// # Panics
    /// When `mountdir` contains `\0` characters, a panic occurs.
    pub fn unmount(&mut self, mountdir: &str) -> io::Result<()> {
        let c_mountdir = CString::new(mountdir).unwrap();
        unsafe {
            Self::handle_remove_result(
                mj_unmountVFS(self.ffi_mut(), c_mountdir.as_ptr())
            )
        }
    }

    fn handle_remove_result(result: i32) -> io::Result<()> {
        match result {
            0 => Ok(()),
            -1 => Err(Error::new(ErrorKind::NotFound, "file not found in the VFS")),
            _ => Err(Error::new(ErrorKind::Other, "unknown MuJoCo error"))
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

    #[test]
    fn test_vfs_mount_unmount() {
        let mut vfs = MjVfs::new();
        // Since we don't have a resource provider, this just calls the API
        // It might return an Err since the provider is null. We just ensure it doesn't crash.
        let _ = vfs.mount("/tmp");
        let _ = vfs.unmount("/tmp");
    }
}
