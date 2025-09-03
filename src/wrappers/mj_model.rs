//! Module for mjModel
use std::io::{self, Error, ErrorKind};
use std::ffi::{c_int, CString};
use std::path::Path;
use std::ptr;

use crate::wrappers::mj_data::MjData;
use super::mj_auxiliary::MjVfs;
use crate::mujoco_c::*;


/// A Rust-safe wrapper around mjModel.
/// Automatically clean after itself on destruction.
#[derive(Debug)]
pub struct MjModel(*mut mjModel);

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl Send for MjModel {}
unsafe impl Sync for MjModel {}


impl MjModel {
    /// Loads the model from an XML file.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        let mut error_buffer = [0i8; 100];
        unsafe {
            let path = CString::new(path.as_ref().to_str().expect("invalid utf")).unwrap();
            let raw_ptr = mj_loadXML(
                path.as_ptr(), ptr::null(),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            Self::check_raw_model(raw_ptr, &error_buffer)
        }
    }

    /// Loads the model from an XML string.
    pub fn from_xml_string(data: &str) -> Result<Self, Error> {
        let mut vfs = MjVfs::new();
        let filename = "model.xml";

        // Add the file into a virtual file system
        vfs.add_from_buffer(filename, data.as_bytes())?;

        let mut error_buffer = [0i8; 100];
        unsafe {
            let filename_c = CString::new(filename).unwrap();
            let raw_ptr = mj_loadXML(
                filename_c.as_ptr(), vfs.ffi(),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            Self::check_raw_model(raw_ptr, &error_buffer)
        }
    }

    /// Loads the model from MJB raw data.
    pub fn from_buffer(data: &[u8]) -> Result<Self, Error> {
        unsafe {
            // Create a virtual FS since we don't have direct access to the load buffer function (or at least it isn't officially exposed).
            // let raw_ptr = mj_loadModelBuffer(data.as_ptr() as *const c_void, data.len() as i32);
            let mut vfs = MjVfs::new();
            let filename = "model.mjb";

            // Add the file into a virtual file system
            vfs.add_from_buffer(filename, data)?;

            // Load the model from the virtual file system
            let filename_c = CString::new(filename).unwrap();
            let raw_model = mj_loadModel(filename_c.as_ptr(), vfs.ffi());
            Self::check_raw_model(raw_model, &[])
        }
    }


    /// Saves the last XML loaded.
    pub fn save_last_xml(&self, filename: &str) -> io::Result<()> {
        let mut error = [0i8; 100];
        unsafe {
            let cstring = CString::new(filename)?;
            match mj_saveLastXML(
                cstring.as_ptr(), self.ffi(),
                error.as_mut_ptr(), (error.len() - 1) as i32
            ) {
                1 => Ok(()),
                0 => {
                    let cstr_error = String::from_utf8_lossy(
                        // Reinterpret as u8 data. This does not affect the data as it is ASCII
                        // encoded and thus negative values aren't possible.
                        std::slice::from_raw_parts(error.as_ptr() as *const u8, error.len())
                    );
                    Err(Error::new(ErrorKind::Other, cstr_error))
                },
                _ => unreachable!()
            }
        }
    }

    /// Translates `name` to the correct id. Wrapper around `mj_name2id`.
    pub fn name2id(&self, type_: mjtObj, name: &str) -> i32 {
        unsafe {
            mj_name2id(self.0, type_ as i32, CString::new(name).unwrap().as_ptr())
        }
    }

    /// Creates a new [`MjData`] instances linked to this model.
    pub fn make_data<'m>(&'m self) -> MjData<'m> {
        MjData::new(self)
    }

    fn check_raw_model(ptr_model: *mut mjModel, error_buffer: &[i8]) -> Result<Self, Error> {
        if ptr_model.is_null() {
            let err_u8 = error_buffer.into_iter().map(|x| *x as u8).take_while(|&x| x != 0).collect();
            Err(Error::new(ErrorKind::UnexpectedEof,  String::from_utf8(err_u8).expect("could not parse error")))
        }
        else {
            Ok(Self(ptr_model))
        }
    }

    /// Returns a reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjModel {
        unsafe { self.0.as_ref().unwrap() }
    }

    /// Returns a mutable reference to the wrapped FFI struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjModel {
        unsafe { self.0.as_mut().unwrap() }
    }

    /// Returns a direct pointer to the underlying model.
    /// THIS IS NOT TO BE USED.
    /// It is only meant for the viewer code, which currently still depends
    /// on mutable pointers to model and data. This will be removed in the future.
    pub(crate) unsafe fn __raw(&self) -> *mut mjModel {
        self.0
    }
}

impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.0);
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    const EXAMPLE_MODEL: &str = "
    <mujoco>
    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        <body name=\"ball\">
            <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
            <joint type=\"free\"/>
        </body>

        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

    </worldbody>
    </mujoco>
    ";
    const MODEL_SAVE_XML_PATH: &str = "./__TMP_MODEL.xml";

    /// Tests if the model can be loaded and then saved.
    #[test]
    fn test_model_load_save() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        model.save_last_xml(MODEL_SAVE_XML_PATH).expect("could not save the model XML.");      
        fs::remove_file(MODEL_SAVE_XML_PATH).unwrap();
    }

    /// Tests whether the automatic attributes work as expected.
    #[test]
    fn test_model_getters() {
        MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
    }
}
