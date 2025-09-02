//! Module for mjModel
use std::io::{self, Error, ErrorKind};
use std::ffi::{c_int, CString};
use std::path::Path;
use std::ptr;

use crate::wrappers::mj_data::MjData;
use super::mj_auxiliary::MjVfs;
use crate::mujoco_c::*;
use crate::impl_getter_setter;


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

/// Getter / setters to the mjModel FFI struct.
impl MjModel {
    impl_getter_setter!(get, nv, "number of degrees of freedom = dim(qvel)", usize, ffi());
    impl_getter_setter!(get, nq, "number of generalized coordinates = dim(qpos)", usize, ffi());
    impl_getter_setter!(get, nu, "number of actuators/controls = dim(ctrl)", usize, ffi());
    impl_getter_setter!(get, na, "number of activation states = dim(act)", usize, ffi());
    impl_getter_setter!(get, nbody, "number of bodies", usize, ffi());
    impl_getter_setter!(get, nbvh, "number of total bounding volumes in all bodies", usize, ffi());
    impl_getter_setter!(get, nbvhstatic, "number of static bounding volumes (aabb stored in mjModel)", usize, ffi());
    impl_getter_setter!(get, nbvhdynamic, "number of dynamic bounding volumes (aabb stored in mjData)", usize, ffi());
    impl_getter_setter!(get, noct, "number of total octree cells in all meshes", usize, ffi());
    impl_getter_setter!(get, njnt, "number of joints", usize, ffi());
    impl_getter_setter!(get, ngeom, "number of geoms", usize, ffi());
    impl_getter_setter!(get, nsite, "number of sites", usize, ffi());
    impl_getter_setter!(get, ncam, "number of cameras", usize, ffi());
    impl_getter_setter!(get, nlight, "number of lights", usize, ffi());
    impl_getter_setter!(get, nflex, "number of flexes", usize, ffi());
    impl_getter_setter!(get, nflexnode, "number of dofs in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexvert, "number of vertices in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexedge, "number of edges in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexelem, "number of elements in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexelemdata, "number of element vertex ids in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexelemedge, "number of element edge ids in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexshelldata, "number of shell fragment vertex ids in all flexes", usize, ffi());
    impl_getter_setter!(get, nflexevpair, "number of element-vertex pairs in all flexes", usize, ffi());
    impl_getter_setter!(get, nflextexcoord, "number of vertices with texture coordinates", usize, ffi());
    impl_getter_setter!(get, nmesh, "number of meshes", usize, ffi());
    impl_getter_setter!(get, nmeshvert, "number of vertices in all meshes", usize, ffi());
    impl_getter_setter!(get, nmeshnormal, "number of normals in all meshes", usize, ffi());
    impl_getter_setter!(get, nmeshtexcoord, "number of texcoords in all meshes", usize, ffi());
    impl_getter_setter!(get, nmeshface, "number of triangular faces in all meshes", usize, ffi());
    impl_getter_setter!(get, nmeshgraph, "number of ints in mesh auxiliary data", usize, ffi());
    impl_getter_setter!(get, nmeshpoly, "number of polygons in all meshes", usize, ffi());
    impl_getter_setter!(get, nmeshpolyvert, "number of vertices in all polygons", usize, ffi());
    impl_getter_setter!(get, nmeshpolymap, "number of polygons in vertex map", usize, ffi());
    impl_getter_setter!(get, nskin, "number of skins", usize, ffi());
    impl_getter_setter!(get, nskinvert, "number of vertices in all skins", usize, ffi());
    impl_getter_setter!(get, nskintexvert, "number of vertices with texcoords in all skins", usize, ffi());
    impl_getter_setter!(get, nskinface, "number of triangular faces in all skins", usize, ffi());
    impl_getter_setter!(get, nskinbone, "number of bones in all skins", usize, ffi());
    impl_getter_setter!(get, nskinbonevert, "number of vertices in all skin bones", usize, ffi());
    impl_getter_setter!(get, nhfield, "number of heightfields", usize, ffi());
    impl_getter_setter!(get, nhfielddata, "number of data points in all heightfields", usize, ffi());
    impl_getter_setter!(get, ntex, "number of textures", usize, ffi());
    impl_getter_setter!(get, ntexdata, "number of bytes in texture rgb data", usize, ffi());
    impl_getter_setter!(get, nmat, "number of materials", usize, ffi());
    impl_getter_setter!(get, npair, "number of predefined geom pairs", usize, ffi());
    impl_getter_setter!(get, nexclude, "number of excluded geom pairs", usize, ffi());
    impl_getter_setter!(get, neq, "number of equality constraints", usize, ffi());
    impl_getter_setter!(get, ntendon, "number of tendons", usize, ffi());
    impl_getter_setter!(get, nwrap, "number of wrap objects in all tendon paths", usize, ffi());
    impl_getter_setter!(get, nsensor, "number of sensors", usize, ffi());
    impl_getter_setter!(get, nnumeric, "number of numeric custom fields", usize, ffi());
    impl_getter_setter!(get, nnumericdata, "number of mjtNums in all numeric fields", usize, ffi());
    impl_getter_setter!(get, ntext, "number of text custom fields", usize, ffi());
    impl_getter_setter!(get, ntextdata, "number of mjtBytes in all text fields", usize, ffi());
    impl_getter_setter!(get, ntuple, "number of tuple custom fields", usize, ffi());
    impl_getter_setter!(get, ntupledata, "number of objects in all tuple fields", usize, ffi());
    impl_getter_setter!(get, nkey, "number of keyframes", usize, ffi());
    impl_getter_setter!(get, nmocap, "number of mocap bodies", usize, ffi());
    impl_getter_setter!(get, nplugin, "number of plugin instances", usize, ffi());
    impl_getter_setter!(get, npluginattr, "number of chars in all plugin config attributes", usize, ffi());
    impl_getter_setter!(get, nuser_body, "number of mjtNums in body_user", usize, ffi());
    impl_getter_setter!(get, nuser_jnt, "number of mjtNums in jnt_user", usize, ffi());
    impl_getter_setter!(get, nuser_geom, "number of mjtNums in geom_user", usize, ffi());
    impl_getter_setter!(get, nuser_site, "number of mjtNums in site_user", usize, ffi());
    impl_getter_setter!(get, nuser_cam, "number of mjtNums in cam_user", usize, ffi());
    impl_getter_setter!(get, nuser_tendon, "number of mjtNums in tendon_user", usize, ffi());
    impl_getter_setter!(get, nuser_actuator, "number of mjtNums in actuator_user", usize, ffi());
    impl_getter_setter!(get, nuser_sensor, "number of mjtNums in sensor_user", usize, ffi());
    impl_getter_setter!(get, nnames, "number of chars in all names", usize, ffi());
    impl_getter_setter!(get, npaths, "number of chars in all paths", usize, ffi());
    impl_getter_setter!(get, nnames_map, "number of slots in the names hash map", usize, ffi());
    impl_getter_setter!(get, nM, "number of non-zeros in sparse inertia matrix", usize, ffi());

    impl_getter_setter!(get, nB, "number of non-zeros in sparse body-dof matrix", usize, ffi());
    impl_getter_setter!(get, nC, "number of non-zeros in sparse reduced dof-dof matrix", usize, ffi());
    impl_getter_setter!(get, nD, "number of non-zeros in sparse dof-dof matrix", usize, ffi());
    impl_getter_setter!(get, nJmom, "number of non-zeros in sparse actuator_moment matrix", usize, ffi());
    impl_getter_setter!(get, ntree, "number of kinematic trees under world body", usize, ffi());
    impl_getter_setter!(get, ngravcomp, "number of bodies with nonzero gravcomp", usize, ffi());
    impl_getter_setter!(get, nemax, "number of potential equality-constraint rows", usize, ffi());
    impl_getter_setter!(get, njmax, "number of available rows in constraint Jacobian (legacy)", usize, ffi());
    impl_getter_setter!(get, nconmax, "number of potential contacts in contact list (legacy)", usize, ffi());
    impl_getter_setter!(get, nuserdata, "number of mjtNums reserved for the user", usize, ffi());
    impl_getter_setter!(get, nsensordata, "number of mjtNums in sensor data vector", usize, ffi());
    impl_getter_setter!(get, npluginstate, "number of mjtNums in plugin state vector", usize, ffi());

    impl_getter_setter!(get, narena, "number of bytes in the mjData arena (inclusive of stack)", usize, ffi());
    impl_getter_setter!(get, nbuffer, "number of bytes in buffer", usize, ffi());

    impl_getter_setter!(get, opt, "physics options", &mjOption, ffi());
    impl_getter_setter!(get, vis, "visualization options", &mjVisual, ffi());
    impl_getter_setter!(get, stat, "model statistics", &mjStatistic, ffi());

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
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
    }
}
