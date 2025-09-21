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
    ffi: mjVFS
}

impl MjVfs {
    pub fn new() -> Self {
        let ffi = unsafe {
            let mut s = MaybeUninit::uninit();
            mj_defaultVFS(s.as_mut_ptr());
            s.assume_init()
        };
        Self { ffi }
    }

    /// Adds a file to the virtual file system.
    /// NOTE: Currently, a [bug](https://github.com/google-deepmind/mujoco/issues/2839)
    /// exists in the MuJoCo library, where setting NULL for `directory`
    /// results in a crash. For that purpose, always pass `Some("./")` if you wish to omit the directory.
    /// # Panics
    /// A panic will occur if `directory` or `filename` contain invalid UTF-8 or have `\0` characters mid string.
    pub fn add_from_file(&mut self, directory: Option<&str>, filename: &str) -> io::Result<()> {
        if directory.is_none() {
            return Err(Error::new(ErrorKind::InvalidInput, "due to a bug in the MuJoCo C library, the 'directory' argument must always be given. \
            This assertion will be removed when the bug is fixed. \
            See https://github.com/google-deepmind/mujoco/issues/2839 for more information."))
        };

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
    /// # Panics
    /// When the `filename` contains invalid UTF-8 or has '\0' characters mid string, a panic occurs.
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
    /// # Panics
    /// When the `filename` contains invalid UTF-8 or has '\0' characters mid string, a panic occurs.
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
    pub unsafe fn ffi_mut(&mut self) -> &mut mjVFS {
        &mut self.ffi
    }
}

impl Drop for MjVfs {
    fn drop(&mut self) {
        unsafe {
            mj_deleteVFS(&mut self.ffi);
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
        /* 
        ** TODO: Enable this test in the future.
        ** It is disabled due to a segmentation fault bug at the MuJoCo's side:
        ** https://github.com/google-deepmind/mujoco/issues/2839
        */
        vfs = MjVfs::new();
        let err = dbg!(vfs.add_from_file(None, REAL_FILE_NAME).unwrap_err());
        assert!(err.kind() == ErrorKind::InvalidInput);
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
}
