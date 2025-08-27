//! Virtual filesystem
use std::io::{self, Error, ErrorKind};
use std::ffi::{c_void, CString};
use std::mem::MaybeUninit;


use crate::mujoco_c::*;

/***********************************************************************************************************************
** MjVfs
***********************************************************************************************************************/
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

    /// Adds a file to the virtual file system from a buffer.
    pub fn add_from_buffer(&mut self, filename: &str, buffer: &[u8]) -> io::Result<()> {
        unsafe {
            match mj_addBufferVFS(
                self.ffi_mut(), CString::new(filename).unwrap().as_ptr(),
                buffer.as_ptr() as *const c_void, buffer.len() as i32
            ) {
                2 => Err(Error::new(ErrorKind::AlreadyExists, "repeated name")),
                -1 => Err(Error::new(ErrorKind::InvalidData, "failed to load")),
                _ => Ok(())
            }
        }
    }

    pub(crate) fn ffi(&self) -> &mjVFS {
        &self.ffi
    }

    pub(crate) fn ffi_mut(&mut self) -> &mut mjVFS {
        &mut self.ffi
    }
}

impl Drop for MjVfs {
    fn drop(&mut self) {
        unsafe {
            mj_deleteVFS(self.ffi_mut());
        }
    }
}

/***********************************************************************************************************************
** MjContact
***********************************************************************************************************************/
pub type MjContact = mjContact;
