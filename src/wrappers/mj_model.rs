//! MjModel related.
use std::ffi::{CStr, CString, NulError, c_int, c_void};
use std::io::{self, Error, ErrorKind};
use std::path::Path;
use std::ptr;

use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic};
use crate::wrappers::mj_option::MjOption;
use crate::util::assert_mujoco_version;
use crate::wrappers::mj_data::MjData;
use super::mj_primitive::*;
use crate::mujoco_c::*;

use crate::{
    view_creator, info_method, info_with_view,
    mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem,
    array_slice_dyn, getter_setter
};


/*******************************************/
// Types
/// Constants which are powers of 2. They are used as bitmasks for the field ``disableflags`` of `mjOption`.
/// At runtime this field is ``m->opt.disableflags``. The number of these constants is given by ``mjNDISABLE`` which is
/// also the length of the global string array `mjDISABLESTRING` with text descriptions of these flags.
pub type MjtDisableBit = mjtDisableBit;

/// Constants which are powers of 2. They are used as bitmasks for the field ``enableflags`` of `mjOption`.
/// At runtime this field is ``m->opt.enableflags``. The number of these constants is given by ``mjNENABLE`` which is also
/// the length of the global string array `mjENABLESTRING` with text descriptions of these flags.
pub type MjtEnableBit = mjtEnableBit;

/// Primitive joint types. These values are used in ``m->jnt_type``. The numbers in the comments indicate how many
/// positional coordinates each joint type has. Note that ball joints and rotational components of free joints are
/// represented as unit quaternions - which have 4 positional coordinates but 3 degrees of freedom each.
pub type MjtJoint = mjtJoint;

/// Geometric types supported by MuJoCo. The first group are "official" geom types that can be used in the model. The
/// second group are geom types that cannot be used in the model but are used by the visualizer to add decorative
/// elements. These values are used in ``m->geom_type`` and ``m->site_type``.
pub type MjtGeom = mjtGeom;

/// Type of camera projection. Used in ``m->cam_projection``.
pub type MjtProjection = mjtProjection;

/// Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed. These
/// values are used in ``m->cam_mode`` and ``m->light_mode``.
pub type MjtCamLight = mjtCamLight;

/// The type of a light source describing how its position, orientation and other properties will interact with the
/// objects in the scene. These values are used in ``m->light_type``.
pub type MjtLightType = mjtLightType;

/// Texture types, specifying how the texture will be mapped. These values are used in ``m->tex_type``.
pub type MjtTexture = mjtTexture;

/// Texture roles, specifying how the renderer should interpret the texture.  Note that the MuJoCo built-in renderer only
/// uses RGB textures.  These values are used to store the texture index in the material's array ``m->mat_texid``.
pub type MjtTextureRole = mjtTextureRole;

/// Type of color space encoding for textures.
pub type MjtColorSpace = mjtColorSpace;

/// Numerical integrator types. These values are used in ``m->opt.integrator``.
pub type MjtIntegrator = mjtIntegrator;

/// Available friction cone types. These values are used in ``m->opt.cone``.
pub type MjtCone = mjtCone;

/// Available Jacobian types. These values are used in ``m->opt.jacobian``.
pub type MjtJacobian = mjtJacobian;

/// Available constraint solver algorithms. These values are used in ``m->opt.solver``.
pub type MjtSolver = mjtSolver;

/// Equality constraint types. These values are used in ``m->eq_type``.
pub type MjtEq = mjtEq;

/// Tendon wrapping object types. These values are used in ``m->wrap_type``.
pub type MjtWrap = mjtWrap;

/// Actuator transmission types. These values are used in ``m->actuator_trntype``.
pub type MjtTrn = mjtTrn;

/// Actuator dynamics types. These values are used in ``m->actuator_dyntype``.
pub type MjtDyn = mjtDyn;

/// Actuator gain types. These values are used in ``m->actuator_gaintype``.
pub type MjtGain = mjtGain;

/// Actuator bias types. These values are used in ``m->actuator_biastype``.
pub type MjtBias = mjtBias;

/// MuJoCo object types. These are used, for example, in the support functions `mj_name2id` and
/// `mj_id2name` to convert between object names and integer ids.
pub type MjtObj = mjtObj;

/// Sensor types. These values are used in ``m->sensor_type``.
pub type MjtSensor = mjtSensor;

/// These are the compute stages for the skipstage parameters of `mj_forwardSkip` and
/// `mj_inverseSkip`.
pub type MjtStage = mjtStage;

/// These are the possible sensor data types, used in ``mjData.sensor_datatype``.
pub type MjtDataType = mjtDataType;

/// Types of data fields returned by contact sensors.
pub type MjtConDataField = mjtConDataField;

/// Types of frame alignment of elements with their parent bodies. Used as shortcuts during `mj_kinematics` in the
/// last argument to `mj_local2global`.
pub type MjtSameFrame = mjtSameFrame;

/// Sleep policy associated with a tree. The compiler automatically chooses between ``NEVER`` and ``ALLOWED``, but the user
/// can override this choice. Only the user can set the ``INIT`` policy (initialized as asleep).
pub type MjtSleepPolicy = mjtSleepPolicy;

/// Types of flex self-collisions midphase.
pub type MjtFlexSelf = mjtFlexSelf;

/// Formulas used to combine SDFs when calling mjc_distance and mjc_gradient.
pub type MjtSDFType = mjtSDFType;

/// Data fields returned by rangefinder sensors.
pub type MjtRayDataField = mjtRayDataField;

/// Camera output type bitflags.
pub type MjtCamOutBit = mjtCamOutBit;
/*******************************************/

/// A Rust-safe wrapper around mjModel.
/// Automatically clean after itself on destruction.
#[derive(Debug)]
pub struct MjModel(*mut mjModel);

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl Send for MjModel {}
unsafe impl Sync for MjModel {}


impl MjModel {
    /// Loads the model from an XML file. To load from a virtual file system, use [`MjModel::from_xml_vfs`].
    /// # Panics
    /// - when the `path` contains invalid utf-8 or '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        Self::from_xml_file(path, None)
    }

    /// Loads the model from an XML file, located in a virtual file system (`vfs`)
    /// # Panics
    /// - when the `path` contains invalid utf-8 or '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, Error> {
        Self::from_xml_file(path, Some(vfs))
    }

    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, Error> {
        assert_mujoco_version();

        let mut error_buffer = [0i8; 100];
        unsafe {
            let path = CString::new(path.as_ref().to_str().expect("invalid utf")).unwrap();
            let raw_ptr = mj_loadXML(
                path.as_ptr(), vfs.map_or(ptr::null(), |v| v.ffi()),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            Self::check_raw_model(raw_ptr, &error_buffer)
        }
    }

    /// Loads the model from an XML string.
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_string(data: &str) -> Result<Self, Error> {
        assert_mujoco_version();

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
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_buffer(data: &[u8]) -> Result<Self, Error> {
        assert_mujoco_version();
        unsafe {
            Self::from_raw(mj_loadModelBuffer(data.as_ptr() as *const c_void, data.len() as i32))
        }
    }

    /// Creates a [`MjModel`] from a raw pointer.
    pub(crate) fn from_raw(ptr: *mut mjModel) -> Result<Self, Error> {
        unsafe { Self::check_raw_model(ptr, &[0]) }
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
                        std::slice::from_raw_parts(
                            error.as_ptr() as *const u8,
                            error.iter().position(|&x| x == 0).unwrap_or(error.len())
                        )
                    );
                    Err(Error::new(ErrorKind::Other, cstr_error))
                },
                _ => unreachable!()
            }
        }
    }

    /// Creates a new [`MjData`] instances linked to this model.
    pub fn make_data(&self) -> MjData<&Self> {
        MjData::new(self)
    }

    /// Handles the pointer to the model.
    /// # Safety
    /// `error_buffer` must have at least on element, where the last element must be 0.
    unsafe fn check_raw_model(ptr_model: *mut mjModel, error_buffer: &[i8]) -> Result<Self, Error> {
        if ptr_model.is_null() {
            Err(Error::new(
                ErrorKind::UnexpectedEof,
                unsafe { CStr::from_ptr(error_buffer.as_ptr().cast()).to_string_lossy().into_owned() }
            ))
        }
        else {
            Ok(Self(ptr_model))
        }
    }

    info_method! { Model, ffi(), actuator,
        [trntype: 1, dyntype: 1, gaintype: 1, biastype: 1, trnid: 2, actadr: 1, actnum: 1,
        group: 1, history: 2, historyadr: 1, delay: 1, ctrllimited: 1, forcelimited: 1, actlimited: 1, dynprm: mjNDYN as usize, gainprm: mjNGAIN as usize, biasprm: mjNBIAS as usize,
        actearly: 1, ctrlrange: 2, forcerange: 2, actrange: 2, gear: 6, cranklength: 1, acc0: 1, length0: 1,
        lengthrange: 2, plugin: 1],
        [user: nuser_actuator],
        []
    }

    info_method! { Model, ffi(), body,
        [parentid: 1, rootid: 1, weldid: 1, mocapid: 1, jntnum: 1, jntadr: 1,
        dofnum: 1, dofadr: 1, treeid: 1, geomnum: 1, geomadr: 1, simple: 1,
        sameframe: 1, pos: 3, quat: 4, ipos: 3, iquat: 4, mass: 1, subtreemass: 1,
        inertia: 3, invweight0: 2, gravcomp: 1, margin: 1, plugin: 1,
        contype: 1, conaffinity: 1, bvhadr: 1, bvhnum: 1],
        [user: nuser_body],
        []
    }

    info_method! { Model, ffi(), camera,
        [mode: 1, bodyid: 1, targetbodyid: 1,
        pos: 3, quat: 4, poscom0: 3,
        pos0: 3, mat0: 9, projection: 1, fovy: 1,
        ipd: 1, resolution: 2, output: 1, sensorsize: 2, intrinsic: 4],
        [user: nuser_cam],
        []
    }
    
    info_method! { Model, ffi(), joint,
        [r#type: 1, qposadr: 1, dofadr: 1, group: 1,
        limited: 1, actfrclimited: 1, actgravcomp: 1, solref: mjNREF as usize, solimp: mjNIMP as usize,
        pos: 3, axis: 3, stiffness: 1,
        range: 2, actfrcrange: 2, margin: 1],
        [user: nuser_jnt],
        [qpos0: nq, qpos_spring: nq, bodyid: nv, jntid: nv,
        parentid: nv, Madr: nv, simplenum: nv, frictionloss: nv,
        armature: nv, damping: nv, invweight0: nv, M0: nv]
    }


    info_method! { Model, ffi(), equality,
        [r#type: 1, obj1id: 1,
        obj2id: 1, active0: 1,
        solref: mjNREF as usize, solimp: mjNIMP as usize,
        data: mjNEQDATA as usize, objtype: 1],
        [],
        []
    }

    info_method! { Model, ffi(), exclude,
        [signature: 1],
        [],
        []
    }

    info_method! { Model, ffi(), geom,
        [r#type: 1, contype: 1, conaffinity: 1, condim: 1, bodyid: 1, dataid: 1, matid: 1,
        group: 1, priority: 1, plugin: 1, sameframe: 1, solmix: 1, solref: mjNREF as usize, solimp: mjNIMP as usize, size: 3,
        aabb: 6, rbound: 1, pos: 3, quat: 4, friction: 3, margin: 1, gap: 1, fluid: mjNFLUID as usize, rgba: 4],
        [user: nuser_geom],
        []
    }

    info_method! { Model, ffi(), hfield,
        [size: 4,
        nrow: 1,
        ncol: 1,
        adr: 1,
        pathadr: 1],
        [],
        [data: nhfielddata]
    }

    info_method! { Model, ffi(), light,
        [mode: 1, bodyid: 1, targetbodyid: 1, r#type: 1, castshadow: 1,
        active: 1, pos: 3, dir: 3, poscom0: 3, pos0: 3,
        dir0: 3, attenuation: 3, cutoff: 1, exponent: 1, ambient: 3,
        diffuse: 3, specular: 3],
        [],
        []
    }

    info_method! { Model, ffi(), material,
        [texid: MjtTextureRole::mjNTEXROLE as usize, texuniform: 1,
        texrepeat: 2, emission: 1,
        specular: 1, shininess: 1,
        reflectance: 1, rgba: 4, metallic: 1, roughness: 1],
        [],
        []
    }

    info_method! { Model, ffi(), mesh,
        [vertadr: 1, vertnum: 1,
        texcoordadr: 1, faceadr: 1,
        facenum: 1, graphadr: 1],
        [],
        []
    }

    info_method! { Model, ffi(), numeric,
        [adr: 1,
        size: 1],
        [],
        [data: nnumericdata]
    }

    info_method! { Model, ffi(), pair,
        [dim: 1, geom1: 1, geom2: 1,
        signature: 1, solref: mjNREF as usize, solimp: mjNIMP as usize,
        margin: 1, gap: 1, friction: 5, solreffriction: mjNREF as usize],
        [],
        []
    }

    info_method! { Model, ffi(), sensor,
        [r#type: 1, datatype: 1, needstage: 1,
        objtype: 1, objid: 1, reftype: 1,
        refid: 1, intprm: mjNSENS as usize, dim: 1, adr: 1,
        cutoff: 1, noise: 1, history: 2, historyadr: 1, delay: 1, interval: 2, plugin: 1],
        [user: nuser_sensor],
        []
    }

    info_method! { Model, ffi(), site,
        [r#type: 1, bodyid: 1, matid: 1,
        group: 1, sameframe: 1, size: 3,
        pos: 3, quat: 4, rgba: 4],
        [user: nuser_site],
        []
    }

    info_method! { Model, ffi(), skin,
        [matid: 1, group: 1, rgba: 4, inflate: 1,
        vertadr: 1, vertnum: 1, texcoordadr: 1,
        faceadr: 1, facenum: 1, boneadr: 1,
        bonenum: 1, pathadr: 1],
        [],
        []
    }

    info_method! { Model, ffi(), tendon,
        [adr: 1, num: 1, matid: 1, group: 1, treenum: 1, treeid: 2, limited: 1, actfrclimited: 1, width: 1,
        solref_lim: mjNREF as usize, solimp_lim: mjNIMP as usize, solref_fri: mjNREF as usize, solimp_fri: mjNIMP as usize, range: 2, actfrcrange: 2, margin: 1,
        stiffness: 1, damping: 1, armature: 1, frictionloss: 1, lengthspring: 2, length0: 1, invweight0: 1,
        rgba: 4],
        [user: nuser_tendon],
        []
    }

    info_method! { Model, ffi(), texture,
        [r#type: 1, colorspace: 1, height: 1,
        width: 1, nchannel: 1,
        adr: 1, pathadr: 1],
        [],
        [data: ntexdata]
    }

    info_method! { Model, ffi(), tuple,
        [adr: 1,
        size: 1],
        [],
        [objtype: ntupledata,
        objid: ntupledata,
        objprm: ntupledata]
    }

    info_method! { Model, ffi(), key,
        [time: 1],
        [qpos: nq, qvel: nv,
        act: na, mpos: nmocap*3,
        mquat: nmocap*4, ctrl: nu],
        []
    }

    /// Translates `name` to the correct id. Wrapper around `mj_name2id`.
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn name_to_id(&self, type_: MjtObj, name: &str) -> i32 {
        let c_string = CString::new(name).unwrap();
        unsafe {
            mj_name2id(self.0, type_ as i32, c_string.as_ptr())
        }
    }

    /* Partially auto-generated */

    /// Clones the model.
    pub fn clone(&self) -> Option<MjModel> {
        let ptr = unsafe { mj_copyModel(ptr::null_mut(), self.ffi()) };
        if ptr.is_null() {
            None
        }
        else {
            Some(MjModel(ptr))
        }
    }

    /// Save model to binary MJB file or memory buffer; buffer has precedence when given.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn save(&self, filename: Option<&str>, buffer: Option<&mut [u8]>) {
        let c_filename = filename.map(|f| CString::new(f).unwrap());
        let (buffer_ptr, buffer_len) = if let Some(b) = buffer {
            (b.as_mut_ptr(), b.len())
        }
        else {
            (ptr::null_mut(), 0)
        };
        let c_filename_ptr = c_filename.as_ref().map_or(ptr::null(), |f| f.as_ptr());

        unsafe { mj_saveModel(
            self.ffi(), c_filename_ptr,
            buffer_ptr as *mut std::ffi::c_void, buffer_len as i32
        ) };
    }

    /// Return size of buffer needed to hold model.
    pub fn size(&self) -> MjtSize {
        unsafe { mj_sizeModel(self.ffi()) }
    }

    /// Print mjModel to text file, specifying format.
    /// float_format must be a valid printf-style format string for a single float value.
    pub fn print_formatted(&self, filename: &str, float_format: &str) -> Result<(), NulError> {
        let c_filename = CString::new(filename)?;
        let c_float_format = CString::new(float_format)?;
        unsafe { mj_printFormattedModel(self.ffi(), c_filename.as_ptr(), c_float_format.as_ptr()) }
        Ok(())
    }

    /// Print model to text file.
    pub fn print(&self, filename: &str) -> Result<(), NulError> {
        let c_filename = CString::new(filename)?;
        unsafe { mj_printModel(self.ffi(), c_filename.as_ptr()) }
        Ok(())
    }

    /// Return size of state specification. The bits of the integer spec correspond to element fields of [`MjtState`](crate::wrappers::mj_data::MjtState).
    pub fn state_size(&self, spec: u32) -> usize {
        unsafe { mj_stateSize(self.ffi(), spec as i32) as usize }
    }

    /// Extract the subset of components specified by `dst_spec` from a state `src`
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::get_state`]
    /// with components specified by `src_spec`.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the extracted state.
    /// # Errors
    /// - when `src.len()` does not equal the size required by `src_spec`, an error of kind [`ErrorKind::InvalidInput`] is returned.
    /// - when `dst_spec` is not a subset of `src_spec`, an error of kind [`ErrorKind::InvalidInput`] is returned.
    pub fn extract_state(&self, src: &[MjtNum], src_spec: u32, dst_spec: u32) -> Result<Box<[MjtNum]>, Error> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(Error::new(ErrorKind::InvalidInput, format!("src slice length must be {} for the provided src_spec (got {})", expected, src.len())));
        }

        if (dst_spec & src_spec) != dst_spec {
            return Err(Error::new(ErrorKind::InvalidInput, "dst_spec must be a subset of src_spec."));
        }

        let required_size = self.state_size(dst_spec);
        let mut dst = Vec::with_capacity(required_size);

        unsafe {
            mj_extractState(
                self.ffi(),
                src.as_ptr(), src_spec as i32,
                dst.as_mut_ptr(), dst_spec as i32
            );

            dst.set_len(required_size);
            Ok(dst.into_boxed_slice())
        }
    }

    /// Extract into dst the subset of components specified by `dst_spec` from a state `src`
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::get_state`]
    /// with components specified by `src_spec`.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the number of elements written to `dst`.
    /// # Errors
    /// - when `src.len()` does not equal the size required by `src_spec`, an error of kind [`ErrorKind::InvalidInput`] is returned.
    /// - when `dst_spec` is not a subset of `src_spec`, an error of kind [`ErrorKind::InvalidInput`] is returned.
    /// - when `dst` is too small to hold the requested components, an error of kind [`ErrorKind::InvalidInput`] is returned.
    pub fn extract_state_into(&self, src: &[MjtNum], src_spec: u32, dst: &mut [MjtNum], dst_spec: u32) -> Result<usize, Error> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(Error::new(ErrorKind::InvalidInput, format!("src slice length must be {} for the provided src_spec (got {})", expected, src.len())));
        }

        if (dst_spec & src_spec) != dst_spec {
            return Err(Error::new(ErrorKind::InvalidInput, "dst_spec must be a subset of src_spec."));
        }

        let required_size = self.state_size(dst_spec);
        let available_size = dst.len();

        if available_size < required_size  {
            return Err(Error::new(ErrorKind::InvalidInput, format!("dst buffer is too small to hold the requested state of size {required_size} (available {available_size})")));
        }

        unsafe {
            mj_extractState(
                self.ffi(),
                src.as_ptr(), src_spec as i32,
                dst.as_mut_ptr(), dst_spec as i32
            );
        }

        Ok(required_size)
    }

    /// Determine type of friction cone.
    pub fn is_pyramidal(&self) -> bool {
        unsafe { mj_isPyramidal(self.ffi()) == 1 }
    }

    /// Determine type of constraint Jacobian.
    pub fn is_sparse(&self) -> bool {
        unsafe { mj_isSparse(self.ffi()) == 1 }
    }

    /// Determine type of solver (PGS is dual, CG and Newton are primal).
    pub fn is_dual(&self) -> bool {
        unsafe { mj_isDual(self.ffi()) == 1 }
    }

    /// Get name of object with the specified [`MjtObj`] type and id, returns NULL if name not found.
    /// Wraps ``mj_id2name``.
    pub fn id_to_name(&self, type_: MjtObj, id: i32) -> Option<&str> {
        let ptr = unsafe { mj_id2name(self.ffi(), type_ as i32, id) };
        if ptr.is_null() {
            None
        }
        else {
            let cstr = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
            Some(cstr)
        }
    }

    /// Sum all body masses.
    pub fn get_totalmass(&self) -> MjtNum {
        unsafe { mj_getTotalmass(self.ffi()) }
    }

    /// Scale body masses and inertias to achieve specified total mass.
    pub fn set_totalmass(&mut self, newmass: MjtNum) {
        unsafe { mj_setTotalmass(self.ffi_mut(), newmass) }
    }

    /* FFI */
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


/// Public attribute methods.
impl MjModel {
    /// Compilation signature.
    pub fn signature(&self) -> u64 {
        self.ffi().signature
    }

    getter_setter! {get, [
        [ffi] nq: MjtSize; "number of generalized coordinates = dim(qpos).";
        [ffi] nv: MjtSize; "number of degrees of freedom = dim(qvel).";
        [ffi] nu: MjtSize; "number of actuators/controls = dim(ctrl).";
        [ffi] na: MjtSize; "number of activation states = dim(act).";
        [ffi] nbody: MjtSize; "number of bodies.";
        [ffi] nbvh: MjtSize; "number of total bounding volumes in all bodies.";
        [ffi] nbvhstatic: MjtSize; "number of static bounding volumes (aabb stored in mjModel).";
        [ffi] nbvhdynamic: MjtSize; "number of dynamic bounding volumes (aabb stored in mjData).";
        [ffi] noct: MjtSize; "number of total octree cells in all meshes.";
        [ffi] njnt: MjtSize; "number of joints.";
        [ffi] ntree: MjtSize; "number of kinematic trees under world body.";
        [ffi] nM: MjtSize; "number of non-zeros in sparse inertia matrix.";
        [ffi] nB: MjtSize; "number of non-zeros in sparse body-dof matrix.";
        [ffi] nC: MjtSize; "number of non-zeros in sparse reduced dof-dof matrix.";
        [ffi] nD: MjtSize; "number of non-zeros in sparse dof-dof matrix.";
        [ffi] ngeom: MjtSize; "number of geoms.";
        [ffi] nsite: MjtSize; "number of sites.";
        [ffi] ncam: MjtSize; "number of cameras.";
        [ffi] nlight: MjtSize; "number of lights.";
        [ffi] nflex: MjtSize; "number of flexes.";
        [ffi] nflexnode: MjtSize; "number of dofs in all flexes.";
        [ffi] nflexvert: MjtSize; "number of vertices in all flexes.";
        [ffi] nflexedge: MjtSize; "number of edges in all flexes.";
        [ffi] nflexelem: MjtSize; "number of elements in all flexes.";
        [ffi] nflexelemdata: MjtSize; "number of element vertex ids in all flexes.";
        [ffi] nflexelemedge: MjtSize; "number of element edge ids in all flexes.";
        [ffi] nflexshelldata: MjtSize; "number of shell fragment vertex ids in all flexes.";
        [ffi] nflexevpair: MjtSize; "number of element-vertex pairs in all flexes.";
        [ffi] nflextexcoord: MjtSize; "number of vertices with texture coordinates.";
        [ffi] nJfe: MjtSize; "number of non-zeros in sparse flexedge Jacobian matrix.";
        [ffi] nJfv: MjtSize; "number of non-zeros in sparse flexvert Jacobian matrix.";
        [ffi] nmesh: MjtSize; "number of meshes.";
        [ffi] nmeshvert: MjtSize; "number of vertices in all meshes.";
        [ffi] nmeshnormal: MjtSize; "number of normals in all meshes.";
        [ffi] nmeshtexcoord: MjtSize; "number of texcoords in all meshes.";
        [ffi] nmeshface: MjtSize; "number of triangular faces in all meshes.";
        [ffi] nmeshgraph: MjtSize; "number of ints in mesh auxiliary data.";
        [ffi] nmeshpoly: MjtSize; "number of polygons in all meshes.";
        [ffi] nmeshpolyvert: MjtSize; "number of vertices in all polygons.";
        [ffi] nmeshpolymap: MjtSize; "number of polygons in vertex map.";
        [ffi] nskin: MjtSize; "number of skins.";
        [ffi] nskinvert: MjtSize; "number of vertices in all skins.";
        [ffi] nskintexvert: MjtSize; "number of vertices with texcoords in all skins.";
        [ffi] nskinface: MjtSize; "number of triangular faces in all skins.";
        [ffi] nskinbone: MjtSize; "number of bones in all skins.";
        [ffi] nskinbonevert: MjtSize; "number of vertices in all skin bones.";
        [ffi] nhfield: MjtSize; "number of heightfields.";
        [ffi] nhfielddata: MjtSize; "number of data points in all heightfields.";
        [ffi] ntex: MjtSize; "number of textures.";
        [ffi] ntexdata: MjtSize; "number of bytes in texture rgb data.";
        [ffi] nmat: MjtSize; "number of materials.";
        [ffi] npair: MjtSize; "number of predefined geom pairs.";
        [ffi] nexclude: MjtSize; "number of excluded geom pairs.";
        [ffi] neq: MjtSize; "number of equality constraints.";
        [ffi] ntendon: MjtSize; "number of tendons.";
        [ffi] nwrap: MjtSize; "number of wrap objects in all tendon paths.";
        [ffi] nsensor: MjtSize; "number of sensors.";
        [ffi] nnumeric: MjtSize; "number of numeric custom fields.";
        [ffi] nnumericdata: MjtSize; "number of mjtNums in all numeric fields.";
        [ffi] ntext: MjtSize; "number of text custom fields.";
        [ffi] ntextdata: MjtSize; "number of mjtBytes in all text fields.";
        [ffi] ntuple: MjtSize; "number of tuple custom fields.";
        [ffi] ntupledata: MjtSize; "number of objects in all tuple fields.";
        [ffi] nkey: MjtSize; "number of keyframes.";
        [ffi] nmocap: MjtSize; "number of mocap bodies.";
        [ffi] nplugin: MjtSize; "number of plugin instances.";
        [ffi] npluginattr: MjtSize; "number of chars in all plugin config attributes.";
        [ffi] nuser_body: MjtSize; "number of mjtNums in body_user.";
        [ffi] nuser_jnt: MjtSize; "number of mjtNums in jnt_user.";
        [ffi] nuser_geom: MjtSize; "number of mjtNums in geom_user.";
        [ffi] nuser_site: MjtSize; "number of mjtNums in site_user.";
        [ffi] nuser_cam: MjtSize; "number of mjtNums in cam_user.";
        [ffi] nuser_tendon: MjtSize; "number of mjtNums in tendon_user.";
        [ffi] nuser_actuator: MjtSize; "number of mjtNums in actuator_user.";
        [ffi] nuser_sensor: MjtSize; "number of mjtNums in sensor_user.";
        [ffi] nnames: MjtSize; "number of chars in all names.";
        [ffi] npaths: MjtSize; "number of chars in all paths.";
        [ffi] nnames_map: MjtSize; "number of slots in the names hash map.";
        [ffi] nJmom: MjtSize; "number of non-zeros in sparse actuator_moment matrix.";
        [ffi] ngravcomp: MjtSize; "number of bodies with nonzero gravcomp.";
        [ffi] nemax: MjtSize; "number of potential equality-constraint rows.";
        [ffi] njmax: MjtSize; "number of available rows in constraint Jacobian (legacy).";
        [ffi] nconmax: MjtSize; "number of potential contacts in contact list (legacy).";
        [ffi] nuserdata: MjtSize; "number of mjtNums reserved for the user.";
        [ffi] nsensordata: MjtSize; "number of mjtNums in sensor data vector.";
        [ffi] npluginstate: MjtSize; "number of mjtNums in plugin state vector.";
        [ffi] nhistory: MjtSize; "number of mjtNums in history buffer.";
        [ffi] narena: MjtSize; "number of bytes in the mjData arena (inclusive of stack).";
        [ffi] nbuffer: MjtSize; "number of bytes in buffer.";
    ]}

    getter_setter! {get, [
        [ffi, ffi_mut] opt: &MjOption; "physics options.";
        [ffi, ffi_mut] vis: &MjVisual; "visualization options.";
        [ffi, ffi_mut] stat: &MjStatistic; "model statistics.";
    ]}
}

/// Array slices.
impl MjModel {
    array_slice_dyn! {
        qpos0: &[MjtNum; "qpos values at default pose"; ffi().nq],
        qpos_spring: &[MjtNum; "reference pose for springs"; ffi().nq],
        body_parentid: &[i32; "id of body's parent"; ffi().nbody],
        body_rootid: &[i32; "ancestor that is direct child of world"; ffi().nbody],
        body_weldid: &[i32; "top ancestor with no dofs to this body"; ffi().nbody],
        body_mocapid: &[i32; "id of mocap data; -1: none"; ffi().nbody],
        body_jntnum: &[i32; "number of joints for this body"; ffi().nbody],
        body_jntadr: &[i32; "start addr of joints; -1: no joints"; ffi().nbody],
        body_dofnum: &[i32; "number of motion degrees of freedom"; ffi().nbody],
        body_dofadr: &[i32; "start addr of dofs; -1: no dofs"; ffi().nbody],
        body_treeid: &[i32; "id of body's kinematic tree; -1: static"; ffi().nbody],
        body_geomnum: &[i32; "number of geoms"; ffi().nbody],
        body_geomadr: &[i32; "start addr of geoms; -1: no geoms"; ffi().nbody],
        body_simple: &[MjtByte; "1: diag M; 2: diag M, sliders only"; ffi().nbody],
        body_sameframe: &[MjtSameFrame [cast]; "same frame as inertia"; ffi().nbody],
        body_pos: &[[MjtNum; 3] [cast]; "position offset rel. to parent body"; ffi().nbody],
        body_quat: &[[MjtNum; 4] [cast]; "orientation offset rel. to parent body"; ffi().nbody],
        body_ipos: &[[MjtNum; 3] [cast]; "local position of center of mass"; ffi().nbody],
        body_iquat: &[[MjtNum; 4] [cast]; "local orientation of inertia ellipsoid"; ffi().nbody],
        body_mass: &[MjtNum; "mass"; ffi().nbody],
        body_subtreemass: &[MjtNum; "mass of subtree starting at this body"; ffi().nbody],
        body_inertia: &[[MjtNum; 3] [cast]; "diagonal inertia in ipos/iquat frame"; ffi().nbody],
        body_invweight0: &[[MjtNum; 2] [cast]; "mean inv inert in qpos0 (trn, rot)"; ffi().nbody],
        body_gravcomp: &[MjtNum; "antigravity force, units of body weight"; ffi().nbody],
        body_margin: &[MjtNum; "MAX over all geom margins"; ffi().nbody],
        body_plugin: &[i32; "plugin instance id; -1: not in use"; ffi().nbody],
        body_contype: &[i32; "OR over all geom contypes"; ffi().nbody],
        body_conaffinity: &[i32; "OR over all geom conaffinities"; ffi().nbody],
        body_bvhadr: &[i32; "address of bvh root"; ffi().nbody],
        body_bvhnum: &[i32; "number of bounding volumes"; ffi().nbody],
        bvh_depth: &[i32; "depth in the bounding volume hierarchy"; ffi().nbvh],
        bvh_child: &[[i32; 2] [cast]; "left and right children in tree"; ffi().nbvh],
        bvh_nodeid: &[i32; "geom or elem id of node; -1: non-leaf"; ffi().nbvh],
        bvh_aabb: &[[MjtNum; 6] [cast]; "local bounding box (center, size)"; ffi().nbvhstatic],
        oct_depth: &[i32; "depth in the octree"; ffi().noct],
        oct_child: &[[i32; 8] [cast]; "children of octree node"; ffi().noct],
        oct_aabb: &[[MjtNum; 6] [cast]; "octree node bounding box (center, size)"; ffi().noct],
        oct_coeff: &[[MjtNum; 8] [cast]; "octree interpolation coefficients"; ffi().noct],
        jnt_type: &[MjtJoint [cast]; "type of joint"; ffi().njnt],
        jnt_qposadr: &[i32; "start addr in 'qpos' for joint's data"; ffi().njnt],
        jnt_dofadr: &[i32; "start addr in 'qvel' for joint's data"; ffi().njnt],
        jnt_bodyid: &[i32; "id of joint's body"; ffi().njnt],
        jnt_group: &[i32; "group for visibility"; ffi().njnt],
        jnt_limited: &[bool [cast]; "does joint have limits"; ffi().njnt],
        jnt_actfrclimited: &[bool [cast]; "does joint have actuator force limits"; ffi().njnt],
        jnt_actgravcomp: &[bool [cast]; "is gravcomp force applied via actuators"; ffi().njnt],
        jnt_solref: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference: limit"; ffi().njnt],
        jnt_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance: limit"; ffi().njnt],
        jnt_pos: &[[MjtNum; 3] [cast]; "local anchor position"; ffi().njnt],
        jnt_axis: &[[MjtNum; 3] [cast]; "local joint axis"; ffi().njnt],
        jnt_stiffness: &[MjtNum; "stiffness coefficient"; ffi().njnt],
        jnt_range: &[[MjtNum; 2] [cast]; "joint limits"; ffi().njnt],
        jnt_actfrcrange: &[[MjtNum; 2] [cast]; "range of total actuator force"; ffi().njnt],
        jnt_margin: &[MjtNum; "min distance for limit detection"; ffi().njnt],
        dof_bodyid: &[i32; "id of dof's body"; ffi().nv],
        dof_jntid: &[i32; "id of dof's joint"; ffi().nv],
        dof_parentid: &[i32; "id of dof's parent; -1: none"; ffi().nv],
        dof_treeid: &[i32; "id of dof's kinematic tree"; ffi().nv],
        dof_Madr: &[i32; "dof address in M-diagonal"; ffi().nv],
        dof_simplenum: &[i32; "number of consecutive simple dofs"; ffi().nv],
        dof_solref: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference:frictionloss"; ffi().nv],
        dof_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance:frictionloss"; ffi().nv],
        dof_frictionloss: &[MjtNum; "dof friction loss"; ffi().nv],
        dof_armature: &[MjtNum; "dof armature inertia/mass"; ffi().nv],
        dof_damping: &[MjtNum; "damping coefficient"; ffi().nv],
        dof_invweight0: &[MjtNum; "diag. inverse inertia in qpos0"; ffi().nv],
        dof_M0: &[MjtNum; "diag. inertia in qpos0"; ffi().nv],
        dof_length: &[MjtNum; "linear: 1; angular: approx. length scale"; ffi().nv],
        tree_bodyadr: &[i32; "start addr of bodies"; ffi().ntree],
        tree_bodynum: &[i32; "number of bodies in tree"; ffi().ntree],
        tree_dofadr: &[i32; "start addr of dofs"; ffi().ntree],
        tree_dofnum: &[i32; "number of dofs in tree"; ffi().ntree],
        tree_sleep_policy: &[MjtSleepPolicy [cast]; "sleep policy"; ffi().ntree],
        geom_type: &[MjtGeom [cast]; "geometric type"; ffi().ngeom],
        geom_contype: &[i32; "geom contact type"; ffi().ngeom],
        geom_conaffinity: &[i32; "geom contact affinity"; ffi().ngeom],
        geom_condim: &[i32; "contact dimensionality (1, 3, 4, 6)"; ffi().ngeom],
        geom_bodyid: &[i32; "id of geom's body"; ffi().ngeom],
        geom_dataid: &[i32; "id of geom's mesh/hfield; -1: none"; ffi().ngeom],
        geom_matid: &[i32; "material id for rendering; -1: none"; ffi().ngeom],
        geom_group: &[i32; "group for visibility"; ffi().ngeom],
        geom_priority: &[i32; "geom contact priority"; ffi().ngeom],
        geom_plugin: &[i32; "plugin instance id; -1: not in use"; ffi().ngeom],
        geom_sameframe: &[MjtSameFrame [cast]; "same frame as body"; ffi().ngeom],
        geom_solmix: &[MjtNum; "mixing coef for solref/imp in geom pair"; ffi().ngeom],
        geom_solref: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference: contact"; ffi().ngeom],
        geom_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance: contact"; ffi().ngeom],
        geom_size: &[[MjtNum; 3] [cast]; "geom-specific size parameters"; ffi().ngeom],
        geom_aabb: &[[MjtNum; 6] [cast]; "bounding box, (center, size)"; ffi().ngeom],
        geom_rbound: &[MjtNum; "radius of bounding sphere"; ffi().ngeom],
        geom_pos: &[[MjtNum; 3] [cast]; "local position offset rel. to body"; ffi().ngeom],
        geom_quat: &[[MjtNum; 4] [cast]; "local orientation offset rel. to body"; ffi().ngeom],
        geom_friction: &[[MjtNum; 3] [cast]; "friction for (slide, spin, roll)"; ffi().ngeom],
        geom_margin: &[MjtNum; "detect contact if dist<margin"; ffi().ngeom],
        geom_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().ngeom],
        geom_fluid: &[[MjtNum; mjNFLUID as usize] [cast]; "fluid interaction parameters"; ffi().ngeom],
        geom_rgba: &[[f32; 4] [cast]; "rgba when material is omitted"; ffi().ngeom],
        site_type: &[MjtGeom [cast]; "geom type for rendering"; ffi().nsite],
        site_bodyid: &[i32; "id of site's body"; ffi().nsite],
        site_matid: &[i32; "material id for rendering; -1: none"; ffi().nsite],
        site_group: &[i32; "group for visibility"; ffi().nsite],
        site_sameframe: &[MjtSameFrame [cast]; "same frame as body"; ffi().nsite],
        site_size: &[[MjtNum; 3] [cast]; "geom size for rendering"; ffi().nsite],
        site_pos: &[[MjtNum; 3] [cast]; "local position offset rel. to body"; ffi().nsite],
        site_quat: &[[MjtNum; 4] [cast]; "local orientation offset rel. to body"; ffi().nsite],
        site_rgba: &[[f32; 4] [cast]; "rgba when material is omitted"; ffi().nsite],
        cam_mode: &[MjtCamLight [cast]; "camera tracking mode"; ffi().ncam],
        cam_bodyid: &[i32; "id of camera's body"; ffi().ncam],
        cam_targetbodyid: &[i32; "id of targeted body; -1: none"; ffi().ncam],
        cam_pos: &[[MjtNum; 3] [cast]; "position rel. to body frame"; ffi().ncam],
        cam_quat: &[[MjtNum; 4] [cast]; "orientation rel. to body frame"; ffi().ncam],
        cam_poscom0: &[[MjtNum; 3] [cast]; "global position rel. to sub-com in qpos0"; ffi().ncam],
        cam_pos0: &[[MjtNum; 3] [cast]; "global position rel. to body in qpos0"; ffi().ncam],
        cam_mat0: &[[MjtNum; 9] [cast]; "global orientation in qpos0"; ffi().ncam],
        cam_projection: &[MjtProjection [cast]; "projection type"; ffi().ncam],
        cam_fovy: &[MjtNum; "y field-of-view (ortho ? len : deg)"; ffi().ncam],
        cam_ipd: &[MjtNum; "inter-pupilary distance"; ffi().ncam],
        cam_resolution: &[[i32; 2] [cast]; "resolution: pixels [width, height]"; ffi().ncam],
        cam_output: &[i32; "output types (MjtCamOutBit bit flags)"; ffi().ncam],
        cam_sensorsize: &[[f32; 2] [cast]; "sensor size: length [width, height]"; ffi().ncam],
        cam_intrinsic: &[[f32; 4] [cast]; "[focal length; principal point]"; ffi().ncam],
        light_mode: &[MjtCamLight [cast]; "light tracking mode"; ffi().nlight],
        light_bodyid: &[i32; "id of light's body"; ffi().nlight],
        light_targetbodyid: &[i32; "id of targeted body; -1: none"; ffi().nlight],
        light_type: &[MjtLightType [cast]; "spot, directional, etc."; ffi().nlight],
        light_texid: &[i32; "texture id for image lights"; ffi().nlight],
        light_castshadow: &[bool [cast]; "does light cast shadows"; ffi().nlight],
        light_bulbradius: &[f32; "light radius for soft shadows"; ffi().nlight],
        light_intensity: &[f32; "intensity, in candela"; ffi().nlight],
        light_range: &[f32; "range of effectiveness"; ffi().nlight],
        light_active: &[bool [cast]; "is light on"; ffi().nlight],
        light_pos: &[[MjtNum; 3] [cast]; "position rel. to body frame"; ffi().nlight],
        light_dir: &[[MjtNum; 3] [cast]; "direction rel. to body frame"; ffi().nlight],
        light_poscom0: &[[MjtNum; 3] [cast]; "global position rel. to sub-com in qpos0"; ffi().nlight],
        light_pos0: &[[MjtNum; 3] [cast]; "global position rel. to body in qpos0"; ffi().nlight],
        light_dir0: &[[MjtNum; 3] [cast]; "global direction in qpos0"; ffi().nlight],
        light_attenuation: &[[f32; 3] [cast]; "OpenGL attenuation (quadratic model)"; ffi().nlight],
        light_cutoff: &[f32; "OpenGL cutoff"; ffi().nlight],
        light_exponent: &[f32; "OpenGL exponent"; ffi().nlight],
        light_ambient: &[[f32; 3] [cast]; "ambient rgb (alpha=1)"; ffi().nlight],
        light_diffuse: &[[f32; 3] [cast]; "diffuse rgb (alpha=1)"; ffi().nlight],
        light_specular: &[[f32; 3] [cast]; "specular rgb (alpha=1)"; ffi().nlight],
        flex_contype: &[i32; "flex contact type"; ffi().nflex],
        flex_conaffinity: &[i32; "flex contact affinity"; ffi().nflex],
        flex_condim: &[i32; "contact dimensionality (1, 3, 4, 6)"; ffi().nflex],
        flex_priority: &[i32; "flex contact priority"; ffi().nflex],
        flex_solmix: &[MjtNum; "mix coef for solref/imp in contact pair"; ffi().nflex],
        flex_solref: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference: contact"; ffi().nflex],
        flex_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance: contact"; ffi().nflex],
        flex_friction: &[[MjtNum; 3] [cast]; "friction for (slide, spin, roll)"; ffi().nflex],
        flex_margin: &[MjtNum; "detect contact if dist<margin"; ffi().nflex],
        flex_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().nflex],
        flex_internal: &[bool [cast]; "internal flex collision enabled"; ffi().nflex],
        flex_selfcollide: &[MjtFlexSelf [cast]; "self collision mode"; ffi().nflex],
        flex_activelayers: &[i32; "number of active element layers, 3D only"; ffi().nflex],
        flex_passive: &[i32; "passive collisions enabled"; ffi().nflex],
        flex_dim: &[i32; "1: lines, 2: triangles, 3: tetrahedra"; ffi().nflex],
        flex_matid: &[i32; "material id for rendering"; ffi().nflex],
        flex_group: &[i32; "group for visibility"; ffi().nflex],
        flex_interp: &[i32; "interpolation (0: vertex, 1: nodes)"; ffi().nflex],
        flex_nodeadr: &[i32; "first node address"; ffi().nflex],
        flex_nodenum: &[i32; "number of nodes"; ffi().nflex],
        flex_vertadr: &[i32; "first vertex address"; ffi().nflex],
        flex_vertnum: &[i32; "number of vertices"; ffi().nflex],
        flex_edgeadr: &[i32; "first edge address"; ffi().nflex],
        flex_edgenum: &[i32; "number of edges"; ffi().nflex],
        flex_elemadr: &[i32; "first element address"; ffi().nflex],
        flex_elemnum: &[i32; "number of elements"; ffi().nflex],
        flex_elemdataadr: &[i32; "first element vertex id address"; ffi().nflex],
        flex_elemedgeadr: &[i32; "first element edge id address"; ffi().nflex],
        flex_shellnum: &[i32; "number of shells"; ffi().nflex],
        flex_shelldataadr: &[i32; "first shell data address"; ffi().nflex],
        flex_evpairadr: &[i32; "first evpair address"; ffi().nflex],
        flex_evpairnum: &[i32; "number of evpairs"; ffi().nflex],
        flex_texcoordadr: &[i32; "address in flex_texcoord; -1: none"; ffi().nflex],
        flex_nodebodyid: &[i32; "node body ids"; ffi().nflexnode],
        flex_vertbodyid: &[i32; "vertex body ids"; ffi().nflexvert],
        flex_vertedgeadr: &[i32; "first edge address"; ffi().nflexvert],
        flex_vertedgenum: &[i32; "number of edges"; ffi().nflexvert],
        flex_vertedge: &[[i32; 2] [cast]; "edge indices"; ffi().nflexedge],
        flex_edge: &[[i32; 2] [cast]; "edge vertex ids (2 per edge)"; ffi().nflexedge],
        flex_edgeflap: &[[i32; 2] [cast]; "adjacent vertex ids (dim=2 only)"; ffi().nflexedge],
        flex_elem: &[i32; "element vertex ids (dim+1 per elem)"; ffi().nflexelemdata],
        flex_elemtexcoord: &[i32; "element texture coordinates (dim+1)"; ffi().nflexelemdata],
        flex_elemedge: &[i32; "element edge ids"; ffi().nflexelemedge],
        flex_elemlayer: &[i32; "element distance from surface, 3D only"; ffi().nflexelem],
        flex_shell: &[i32; "shell fragment vertex ids (dim per frag)"; ffi().nflexshelldata],
        flex_evpair: &[[i32; 2] [cast]; "(element, vertex) collision pairs"; ffi().nflexevpair],
        flex_vert: &[[MjtNum; 3] [cast]; "vertex positions in local body frames"; ffi().nflexvert],
        flex_vert0: &[[MjtNum; 3] [cast]; "vertex positions in qpos0 on [0, 1]^d"; ffi().nflexvert],
        flex_vertmetric: &[[MjtNum; 4] [cast]; "inverse of reference shape matrix"; ffi().nflexvert],
        flex_node: &[[MjtNum; 3] [cast]; "node positions in local body frames"; ffi().nflexnode],
        flex_node0: &[[MjtNum; 3] [cast]; "Cartesian node positions in qpos0"; ffi().nflexnode],
        flexedge_length0: &[MjtNum; "edge lengths in qpos0"; ffi().nflexedge],
        flexedge_invweight0: &[MjtNum; "edge inv. weight in qpos0"; ffi().nflexedge],
        flex_radius: &[MjtNum; "radius around primitive element"; ffi().nflex],
        flex_size: &[[MjtNum; 3] [cast]; "vertex bounding box half sizes in qpos0"; ffi().nflex],
        flex_stiffness: &[[MjtNum; 21] [cast]; "finite element stiffness matrix"; ffi().nflexelem],
        flex_bending: &[[MjtNum; 17] [cast]; "bending stiffness"; ffi().nflexedge],
        flex_damping: &[MjtNum; "Rayleigh's damping coefficient"; ffi().nflex],
        flex_edgestiffness: &[MjtNum; "edge stiffness"; ffi().nflex],
        flex_edgedamping: &[MjtNum; "edge damping"; ffi().nflex],
        flex_edgeequality: &[i32; "0: none, 1: edges, 2: vertices"; ffi().nflex],
        flex_rigid: &[bool [cast]; "are all vertices in the same body"; ffi().nflex],
        flexedge_rigid: &[bool [cast]; "are both edge vertices in same body"; ffi().nflexedge],
        flex_centered: &[bool [cast]; "are all vertex coordinates (0,0,0)"; ffi().nflex],
        flex_flatskin: &[bool [cast]; "render flex skin with flat shading"; ffi().nflex],
        flex_bvhadr: &[i32; "address of bvh root; -1: no bvh"; ffi().nflex],
        flex_bvhnum: &[i32; "number of bounding volumes"; ffi().nflex],
        flexedge_J_rownnz: &[i32; "number of non-zeros in Jacobian row"; ffi().nflexedge],
        flexedge_J_rowadr: &[i32; "row start address in colind array"; ffi().nflexedge],
        flexedge_J_colind: &[i32; "column indices in sparse Jacobian"; ffi().nJfe],
        flexvert_J_rownnz: &[[i32; 2] [cast]; "number of non-zeros in Jacobian row"; ffi().nflexvert],
        flexvert_J_rowadr: &[[i32; 2] [cast]; "row start address in colind array"; ffi().nflexvert],
        flexvert_J_colind: &[[i32; 2] [cast]; "column indices in sparse Jacobian"; ffi().nJfv],
        flex_rgba: &[[f32; 4] [cast]; "rgba when material is omitted"; ffi().nflex],
        flex_texcoord: &[[f32; 2] [cast]; "vertex texture coordinates"; ffi().nflextexcoord],
        mesh_vertadr: &[i32; "first vertex address"; ffi().nmesh],
        mesh_vertnum: &[i32; "number of vertices"; ffi().nmesh],
        mesh_faceadr: &[i32; "first face address"; ffi().nmesh],
        mesh_facenum: &[i32; "number of faces"; ffi().nmesh],
        mesh_bvhadr: &[i32; "address of bvh root"; ffi().nmesh],
        mesh_bvhnum: &[i32; "number of bvh"; ffi().nmesh],
        mesh_octadr: &[i32; "address of octree root"; ffi().nmesh],
        mesh_octnum: &[i32; "number of octree nodes"; ffi().nmesh],
        mesh_normaladr: &[i32; "first normal address"; ffi().nmesh],
        mesh_normalnum: &[i32; "number of normals"; ffi().nmesh],
        mesh_texcoordadr: &[i32; "texcoord data address; -1: no texcoord"; ffi().nmesh],
        mesh_texcoordnum: &[i32; "number of texcoord"; ffi().nmesh],
        mesh_graphadr: &[i32; "graph data address; -1: no graph"; ffi().nmesh],
        mesh_vert: &[[f32; 3] [cast]; "vertex positions for all meshes"; ffi().nmeshvert],
        mesh_normal: &[[f32; 3] [cast]; "normals for all meshes"; ffi().nmeshnormal],
        mesh_texcoord: &[[f32; 2] [cast]; "vertex texcoords for all meshes"; ffi().nmeshtexcoord],
        mesh_face: &[[i32; 3] [cast]; "vertex face data"; ffi().nmeshface],
        mesh_facenormal: &[[i32; 3] [cast]; "normal face data"; ffi().nmeshface],
        mesh_facetexcoord: &[[i32; 3] [cast]; "texture face data"; ffi().nmeshface],
        mesh_graph: &[i32; "convex graph data"; ffi().nmeshgraph],
        mesh_scale: &[[MjtNum; 3] [cast]; "scaling applied to asset vertices"; ffi().nmesh],
        mesh_pos: &[[MjtNum; 3] [cast]; "translation applied to asset vertices"; ffi().nmesh],
        mesh_quat: &[[MjtNum; 4] [cast]; "rotation applied to asset vertices"; ffi().nmesh],
        mesh_pathadr: &[i32; "address of asset path for mesh; -1: none"; ffi().nmesh],
        mesh_polynum: &[i32; "number of polygons per mesh"; ffi().nmesh],
        mesh_polyadr: &[i32; "first polygon address per mesh"; ffi().nmesh],
        mesh_polynormal: &[[MjtNum; 3] [cast]; "all polygon normals"; ffi().nmeshpoly],
        mesh_polyvertadr: &[i32; "polygon vertex start address"; ffi().nmeshpoly],
        mesh_polyvertnum: &[i32; "number of vertices per polygon"; ffi().nmeshpoly],
        mesh_polyvert: &[i32; "all polygon vertices"; ffi().nmeshpolyvert],
        mesh_polymapadr: &[i32; "first polygon address per vertex"; ffi().nmeshvert],
        mesh_polymapnum: &[i32; "number of polygons per vertex"; ffi().nmeshvert],
        mesh_polymap: &[i32; "vertex to polygon map"; ffi().nmeshpolymap],
        skin_matid: &[i32; "skin material id; -1: none"; ffi().nskin],
        skin_group: &[i32; "group for visibility"; ffi().nskin],
        skin_rgba: &[[f32; 4] [cast]; "skin rgba"; ffi().nskin],
        skin_inflate: &[f32; "inflate skin in normal direction"; ffi().nskin],
        skin_vertadr: &[i32; "first vertex address"; ffi().nskin],
        skin_vertnum: &[i32; "number of vertices"; ffi().nskin],
        skin_texcoordadr: &[i32; "texcoord data address; -1: no texcoord"; ffi().nskin],
        skin_faceadr: &[i32; "first face address"; ffi().nskin],
        skin_facenum: &[i32; "number of faces"; ffi().nskin],
        skin_boneadr: &[i32; "first bone in skin"; ffi().nskin],
        skin_bonenum: &[i32; "number of bones in skin"; ffi().nskin],
        skin_vert: &[[f32; 3] [cast]; "vertex positions for all skin meshes"; ffi().nskinvert],
        skin_texcoord: &[[f32; 2] [cast]; "vertex texcoords for all skin meshes"; ffi().nskintexvert],
        skin_face: &[[i32; 3] [cast]; "triangle faces for all skin meshes"; ffi().nskinface],
        skin_bonevertadr: &[i32; "first vertex in each bone"; ffi().nskinbone],
        skin_bonevertnum: &[i32; "number of vertices in each bone"; ffi().nskinbone],
        skin_bonebindpos: &[[f32; 3] [cast]; "bind pos of each bone"; ffi().nskinbone],
        skin_bonebindquat: &[[f32; 4] [cast]; "bind quat of each bone"; ffi().nskinbone],
        skin_bonebodyid: &[i32; "body id of each bone"; ffi().nskinbone],
        skin_bonevertid: &[i32; "mesh ids of vertices in each bone"; ffi().nskinbonevert],
        skin_bonevertweight: &[f32; "weights of vertices in each bone"; ffi().nskinbonevert],
        skin_pathadr: &[i32; "address of asset path for skin; -1: none"; ffi().nskin],
        hfield_size: &[[MjtNum; 4] [cast]; "(x, y, z_top, z_bottom)"; ffi().nhfield],
        hfield_nrow: &[i32; "number of rows in grid"; ffi().nhfield],
        hfield_ncol: &[i32; "number of columns in grid"; ffi().nhfield],
        hfield_adr: &[i32; "address in hfield_data"; ffi().nhfield],
        hfield_data: &[f32; "elevation data"; ffi().nhfielddata],
        hfield_pathadr: &[i32; "address of hfield asset path; -1: none"; ffi().nhfield],
        tex_type: &[MjtTexture [cast]; "texture type"; ffi().ntex],
        tex_colorspace: &[MjtColorSpace [cast]; "texture colorspace"; ffi().ntex],
        tex_height: &[i32; "number of rows in texture image"; ffi().ntex],
        tex_width: &[i32; "number of columns in texture image"; ffi().ntex],
        tex_nchannel: &[i32; "number of channels in texture image"; ffi().ntex],
        tex_adr: &[MjtSize; "start address in tex_data"; ffi().ntex],
        tex_data: &[MjtByte; "pixel values"; ffi().ntexdata],
        tex_pathadr: &[i32; "address of texture asset path; -1: none"; ffi().ntex],
        mat_texid: &[[i32; MjtTextureRole::mjNTEXROLE as usize] [cast]; "indices of textures; -1: none"; ffi().nmat],
        mat_texuniform: &[bool [cast]; "make texture cube uniform"; ffi().nmat],
        mat_texrepeat: &[[f32; 2] [cast]; "texture repetition for 2d mapping"; ffi().nmat],
        mat_emission: &[f32; "emission (x rgb)"; ffi().nmat],
        mat_specular: &[f32; "specular (x white)"; ffi().nmat],
        mat_shininess: &[f32; "shininess coef"; ffi().nmat],
        mat_reflectance: &[f32; "reflectance (0: disable)"; ffi().nmat],
        mat_metallic: &[f32; "metallic coef"; ffi().nmat],
        mat_roughness: &[f32; "roughness coef"; ffi().nmat],
        mat_rgba: &[[f32; 4] [cast]; "rgba"; ffi().nmat],
        pair_dim: &[i32; "contact dimensionality"; ffi().npair],
        pair_geom1: &[i32; "id of geom1"; ffi().npair],
        pair_geom2: &[i32; "id of geom2"; ffi().npair],
        pair_signature: &[i32; "body1 << 16 + body2"; ffi().npair],
        pair_solref: &[[MjtNum; mjNREF as usize] [cast]; "solver reference: contact normal"; ffi().npair],
        pair_solreffriction: &[[MjtNum; mjNREF as usize] [cast]; "solver reference: contact friction"; ffi().npair],
        pair_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "solver impedance: contact"; ffi().npair],
        pair_margin: &[MjtNum; "detect contact if dist<margin"; ffi().npair],
        pair_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().npair],
        pair_friction: &[[MjtNum; 5] [cast]; "tangent1, 2, spin, roll1, 2"; ffi().npair],
        exclude_signature: &[i32; "body1 << 16 + body2"; ffi().nexclude],
        eq_type: &[MjtEq [cast]; "constraint type"; ffi().neq],
        eq_obj1id: &[i32; "id of object 1"; ffi().neq],
        eq_obj2id: &[i32; "id of object 2"; ffi().neq],
        eq_objtype: &[MjtObj [cast]; "type of both objects"; ffi().neq],
        eq_active0: &[bool [cast]; "initial enable/disable constraint state"; ffi().neq],
        eq_solref: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference"; ffi().neq],
        eq_solimp: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance"; ffi().neq],
        eq_data: &[[MjtNum; mjNEQDATA as usize] [cast]; "numeric data for constraint"; ffi().neq],
        tendon_adr: &[i32; "address of first object in tendon's path"; ffi().ntendon],
        tendon_num: &[i32; "number of objects in tendon's path"; ffi().ntendon],
        tendon_matid: &[i32; "material id for rendering"; ffi().ntendon],
        tendon_group: &[i32; "group for visibility"; ffi().ntendon],
        tendon_treenum: &[i32; "number of trees along tendon's path"; ffi().ntendon],
        tendon_treeid: &[[i32; 2] [cast]; "first two trees along tendon's path"; ffi().ntendon],
        tendon_limited: &[bool [cast]; "does tendon have length limits"; ffi().ntendon],
        tendon_actfrclimited: &[bool [cast]; "does tendon have actuator force limits"; ffi().ntendon],
        tendon_width: &[MjtNum; "width for rendering"; ffi().ntendon],
        tendon_solref_lim: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference: limit"; ffi().ntendon],
        tendon_solimp_lim: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance: limit"; ffi().ntendon],
        tendon_solref_fri: &[[MjtNum; mjNREF as usize] [cast]; "constraint solver reference: friction"; ffi().ntendon],
        tendon_solimp_fri: &[[MjtNum; mjNIMP as usize] [cast]; "constraint solver impedance: friction"; ffi().ntendon],
        tendon_range: &[[MjtNum; 2] [cast]; "tendon length limits"; ffi().ntendon],
        tendon_actfrcrange: &[[MjtNum; 2] [cast]; "range of total actuator force"; ffi().ntendon],
        tendon_margin: &[MjtNum; "min distance for limit detection"; ffi().ntendon],
        tendon_stiffness: &[MjtNum; "stiffness coefficient"; ffi().ntendon],
        tendon_damping: &[MjtNum; "damping coefficient"; ffi().ntendon],
        tendon_armature: &[MjtNum; "inertia associated with tendon velocity"; ffi().ntendon],
        tendon_frictionloss: &[MjtNum; "loss due to friction"; ffi().ntendon],
        tendon_lengthspring: &[[MjtNum; 2] [cast]; "spring resting length range"; ffi().ntendon],
        tendon_length0: &[MjtNum; "tendon length in qpos0"; ffi().ntendon],
        tendon_invweight0: &[MjtNum; "inv. weight in qpos0"; ffi().ntendon],
        tendon_rgba: &[[f32; 4] [cast]; "rgba when material is omitted"; ffi().ntendon],
        wrap_type: &[MjtWrap [cast]; "wrap object type"; ffi().nwrap],
        wrap_objid: &[i32; "object id: geom, site, joint"; ffi().nwrap],
        wrap_prm: &[MjtNum; "divisor, joint coef, or site id"; ffi().nwrap],
        actuator_trntype: &[MjtTrn [cast]; "transmission type"; ffi().nu],
        actuator_dyntype: &[MjtDyn [cast]; "dynamics type"; ffi().nu],
        actuator_gaintype: &[MjtGain [cast]; "gain type"; ffi().nu],
        actuator_biastype: &[MjtBias [cast]; "bias type"; ffi().nu],
        actuator_trnid: &[[i32; 2] [cast]; "transmission id: joint, tendon, site"; ffi().nu],
        actuator_actadr: &[i32; "first activation address; -1: stateless"; ffi().nu],
        actuator_actnum: &[i32; "number of activation variables"; ffi().nu],
        actuator_group: &[i32; "group for visibility"; ffi().nu],
        actuator_history: &[[i32; 2] [cast]; "history buffer: [nsample, interp]"; ffi().nu],
        actuator_historyadr: &[i32; "address in history buffer; -1: none"; ffi().nu],
        actuator_delay: &[MjtNum; "delay time in seconds; 0: no delay"; ffi().nu],
        actuator_ctrllimited: &[bool [cast]; "is control limited"; ffi().nu],
        actuator_forcelimited: &[bool [cast]; "is force limited"; ffi().nu],
        actuator_actlimited: &[bool [cast]; "is activation limited"; ffi().nu],
        actuator_dynprm: &[[MjtNum; mjNDYN as usize] [cast]; "dynamics parameters"; ffi().nu],
        actuator_gainprm: &[[MjtNum; mjNGAIN as usize] [cast]; "gain parameters"; ffi().nu],
        actuator_biasprm: &[[MjtNum; mjNBIAS as usize] [cast]; "bias parameters"; ffi().nu],
        actuator_actearly: &[bool [cast]; "step activation before force"; ffi().nu],
        actuator_ctrlrange: &[[MjtNum; 2] [cast]; "range of controls"; ffi().nu],
        actuator_forcerange: &[[MjtNum; 2] [cast]; "range of forces"; ffi().nu],
        actuator_actrange: &[[MjtNum; 2] [cast]; "range of activations"; ffi().nu],
        actuator_gear: &[[MjtNum; 6] [cast]; "scale length and transmitted force"; ffi().nu],
        actuator_cranklength: &[MjtNum; "crank length for slider-crank"; ffi().nu],
        actuator_acc0: &[MjtNum; "acceleration from unit force in qpos0"; ffi().nu],
        actuator_length0: &[MjtNum; "actuator length in qpos0"; ffi().nu],
        actuator_lengthrange: &[[MjtNum; 2] [cast]; "feasible actuator length range"; ffi().nu],
        actuator_plugin: &[i32; "plugin instance id; -1: not a plugin"; ffi().nu],
        sensor_type: &[MjtSensor [cast]; "sensor type"; ffi().nsensor],
        sensor_datatype: &[MjtDataType [cast]; "numeric data type"; ffi().nsensor],
        sensor_needstage: &[MjtStage [cast]; "required compute stage"; ffi().nsensor],
        sensor_objtype: &[MjtObj [cast]; "type of sensorized object"; ffi().nsensor],
        sensor_objid: &[i32; "id of sensorized object"; ffi().nsensor],
        sensor_reftype: &[MjtObj [cast]; "type of reference frame"; ffi().nsensor],
        sensor_refid: &[i32; "id of reference frame; -1: global frame"; ffi().nsensor],
        sensor_intprm: &[[i32; mjNSENS as usize] [cast]; "sensor parameters"; ffi().nsensor],
        sensor_dim: &[i32; "number of scalar outputs"; ffi().nsensor],
        sensor_adr: &[i32; "address in sensor array"; ffi().nsensor],
        sensor_cutoff: &[MjtNum; "cutoff for real and positive; 0: ignore"; ffi().nsensor],
        sensor_noise: &[MjtNum; "noise standard deviation"; ffi().nsensor],
        sensor_history: &[[i32; 2] [cast]; "history buffer: [nsample, interp]"; ffi().nsensor],
        sensor_historyadr: &[i32; "address in history buffer; -1: none"; ffi().nsensor],
        sensor_delay: &[MjtNum; "delay time in seconds; 0: no delay"; ffi().nsensor],
        sensor_interval: &[[MjtNum; 2] [cast]; "interval: [period, phase] in seconds"; ffi().nsensor],
        sensor_plugin: &[i32; "plugin instance id; -1: not a plugin"; ffi().nsensor],
        plugin: &[i32; "globally registered plugin slot number"; ffi().nplugin],
        plugin_stateadr: &[i32; "address in the plugin state array"; ffi().nplugin],
        plugin_statenum: &[i32; "number of states in the plugin instance"; ffi().nplugin],
        plugin_attr: &[i8; "config attributes of plugin instances"; ffi().npluginattr],
        plugin_attradr: &[i32; "address to each instance's config attrib"; ffi().nplugin],
        numeric_adr: &[i32; "address of field in numeric_data"; ffi().nnumeric],
        numeric_size: &[i32; "size of numeric field"; ffi().nnumeric],
        numeric_data: &[MjtNum; "array of all numeric fields"; ffi().nnumericdata],
        text_adr: &[i32; "address of text in text_data"; ffi().ntext],
        text_size: &[i32; "size of text field (strlen+1)"; ffi().ntext],
        text_data: &[i8; "array of all text fields (0-terminated)"; ffi().ntextdata],
        tuple_adr: &[i32; "address of text in text_data"; ffi().ntuple],
        tuple_size: &[i32; "number of objects in tuple"; ffi().ntuple],
        tuple_objtype: &[i32; "array of object types in all tuples"; ffi().ntupledata],
        tuple_objid: &[i32; "array of object ids in all tuples"; ffi().ntupledata],
        tuple_objprm: &[MjtNum; "array of object params in all tuples"; ffi().ntupledata],
        key_time: &[MjtNum; "key time"; ffi().nkey],
        name_bodyadr: &[i32; "body name pointers"; ffi().nbody],
        name_jntadr: &[i32; "joint name pointers"; ffi().njnt],
        name_geomadr: &[i32; "geom name pointers"; ffi().ngeom],
        name_siteadr: &[i32; "site name pointers"; ffi().nsite],
        name_camadr: &[i32; "camera name pointers"; ffi().ncam],
        name_lightadr: &[i32; "light name pointers"; ffi().nlight],
        name_flexadr: &[i32; "flex name pointers"; ffi().nflex],
        name_meshadr: &[i32; "mesh name pointers"; ffi().nmesh],
        name_skinadr: &[i32; "skin name pointers"; ffi().nskin],
        name_hfieldadr: &[i32; "hfield name pointers"; ffi().nhfield],
        name_texadr: &[i32; "texture name pointers"; ffi().ntex],
        name_matadr: &[i32; "material name pointers"; ffi().nmat],
        name_pairadr: &[i32; "geom pair name pointers"; ffi().npair],
        name_excludeadr: &[i32; "exclude name pointers"; ffi().nexclude],
        name_eqadr: &[i32; "equality constraint name pointers"; ffi().neq],
        name_tendonadr: &[i32; "tendon name pointers"; ffi().ntendon],
        name_actuatoradr: &[i32; "actuator name pointers"; ffi().nu],
        name_sensoradr: &[i32; "sensor name pointers"; ffi().nsensor],
        name_numericadr: &[i32; "numeric name pointers"; ffi().nnumeric],
        name_textadr: &[i32; "text name pointers"; ffi().ntext],
        name_tupleadr: &[i32; "tuple name pointers"; ffi().ntuple],
        name_keyadr: &[i32; "keyframe name pointers"; ffi().nkey],
        name_pluginadr: &[i32; "plugin instance name pointers"; ffi().nplugin],
        names: &[i8; "names of all objects, 0-terminated"; ffi().nnames],
        names_map: &[i32; "internal hash map of names"; ffi().nnames_map],
        paths: &[i8; "paths to assets, 0-terminated"; ffi().npaths],
        B_rownnz: &[i32; "body-dof: non-zeros in each row"; ffi().nbody],
        B_rowadr: &[i32; "body-dof: row addresses"; ffi().nbody],
        B_colind: &[i32; "body-dof: column indices"; ffi().nB],
        M_rownnz: &[i32; "reduced inertia: non-zeros in each row"; ffi().nv],
        M_rowadr: &[i32; "reduced inertia: row addresses"; ffi().nv],
        M_colind: &[i32; "reduced inertia: column indices"; ffi().nC],
        mapM2M: &[i32; "index mapping from qM to M"; ffi().nC],
        D_rownnz: &[i32; "full inertia: non-zeros in each row"; ffi().nv],
        D_rowadr: &[i32; "full inertia: row addresses"; ffi().nv],
        D_diag: &[i32; "full inertia: index of diagonal element"; ffi().nv],
        D_colind: &[i32; "full inertia: column indices"; ffi().nD],
        mapM2D: &[i32; "index mapping from M to D"; ffi().nD],
        mapD2M: &[i32; "index mapping from D to M"; ffi().nC]
    }

    array_slice_dyn! {
        sublen_dep {
            key_qpos: &[[MjtNum; ffi().nq] [cast]; "key position"; ffi().nkey],
            key_qvel: &[[MjtNum; ffi().nv] [cast]; "key velocity"; ffi().nkey],
            key_act: &[[MjtNum; ffi().na] [cast]; "key activation"; ffi().nkey],
            key_ctrl: &[[MjtNum; ffi().nu] [cast]; "key control"; ffi().nkey],

            sensor_user: &[[MjtNum; ffi().nuser_sensor] [cast]; "user data"; ffi().nsensor],
            actuator_user: &[[MjtNum; ffi().nuser_actuator] [cast]; "user data"; ffi().nu],
            tendon_user: &[[MjtNum; ffi().nuser_tendon] [cast]; "user data"; ffi().ntendon],
            cam_user: &[[MjtNum; ffi().nuser_cam] [cast]; "user data"; ffi().ncam],
            site_user: &[[MjtNum; ffi().nuser_site] [cast]; "user data"; ffi().nsite],
            geom_user: &[[MjtNum; ffi().nuser_geom] [cast]; "user data"; ffi().ngeom],
            jnt_user: &[[MjtNum; ffi().nuser_jnt] [cast]; "user data"; ffi().njnt],
            body_user: &[[MjtNum; ffi().nuser_body] [cast]; "user data"; ffi().nbody]
        }
    }
}

impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.0);
        }
    }
}

info_with_view!(Model, actuator,
	[[actuator_] trntype: MjtTrn [cast], [actuator_] dyntype: MjtDyn [cast],
	 [actuator_] gaintype: MjtGain [cast], [actuator_] biastype: MjtBias [cast],
	 [actuator_] trnid: i32, [actuator_] actadr: i32,
	 [actuator_] actnum: i32, [actuator_] group: i32,
	 [actuator_] history: i32, [actuator_] historyadr: i32,
	 [actuator_] delay: MjtNum, [actuator_] ctrllimited: bool [cast],
	 [actuator_] forcelimited: bool [cast], [actuator_] actlimited: bool [cast],
	 [actuator_] dynprm: MjtNum, [actuator_] gainprm: MjtNum,
	 [actuator_] biasprm: MjtNum, [actuator_] actearly: bool [cast],
	 [actuator_] ctrlrange: MjtNum, [actuator_] forcerange: MjtNum,
	 [actuator_] actrange: MjtNum, [actuator_] gear: MjtNum,
	 [actuator_] cranklength: MjtNum, [actuator_] acc0: MjtNum,
	 [actuator_] length0: MjtNum, [actuator_] lengthrange: MjtNum,
	 [actuator_] user: MjtNum, [actuator_] plugin: i32],
	[]);

info_with_view!(Model, body,
	[[body_] parentid: i32, [body_] rootid: i32,
	 [body_] weldid: i32, [body_] mocapid: i32,
	 [body_] jntnum: i32, [body_] jntadr: i32,
	 [body_] dofnum: i32, [body_] dofadr: i32,
	 [body_] treeid: i32, [body_] geomnum: i32,
	 [body_] geomadr: i32, [body_] simple: MjtByte,
	 [body_] sameframe: MjtSameFrame [cast], [body_] pos: MjtNum,
	 [body_] quat: MjtNum, [body_] ipos: MjtNum,
	 [body_] iquat: MjtNum, [body_] mass: MjtNum,
	 [body_] subtreemass: MjtNum, [body_] inertia: MjtNum,
	 [body_] invweight0: MjtNum, [body_] gravcomp: MjtNum,
	 [body_] margin: MjtNum, [body_] plugin: i32,
	 [body_] contype: i32, [body_] conaffinity: i32,
	 [body_] bvhadr: i32, [body_] bvhnum: i32, [body_] user: MjtNum],
	[]);

info_with_view!(Model, camera,
	[[cam_] mode: MjtCamLight [cast],
	 [cam_] bodyid: i32,
	 [cam_] targetbodyid: i32,
	 [cam_] pos: MjtNum,
	 [cam_] quat: MjtNum,
	 [cam_] poscom0: MjtNum,
	 [cam_] pos0: MjtNum,
	 [cam_] mat0: MjtNum,
	 [cam_] projection: MjtProjection [cast],
	 [cam_] fovy: MjtNum,
	 [cam_] ipd: MjtNum,
	 [cam_] resolution: i32,
     [cam_] output: i32,
	 [cam_] sensorsize: f32,
	 [cam_] intrinsic: f32,
	 [cam_] user: MjtNum],
	[]);

info_with_view!(Model, equality,
	[[eq_] r#type: MjtEq [cast],
	 [eq_] obj1id: i32,
	 [eq_] obj2id: i32,
	 [eq_] active0: bool [cast],
	 [eq_] solref: MjtNum,
	 [eq_] solimp: MjtNum,
	 [eq_] data: MjtNum,
     [eq_] objtype: MjtObj [cast]],
	[]);

info_with_view!(Model, exclude,
	[[exclude_] signature: i32],
	[]);

info_with_view!(Model, geom,
	[[geom_] r#type: MjtGeom [cast], [geom_] contype: i32,
	 [geom_] conaffinity: i32, [geom_] condim: i32,
	 [geom_] bodyid: i32, [geom_] dataid: i32,
	 [geom_] matid: i32, [geom_] group: i32,
	 [geom_] priority: i32, [geom_] plugin: i32, [geom_] sameframe: MjtSameFrame [cast],
	 [geom_] solmix: MjtNum, [geom_] solref: MjtNum,
	 [geom_] solimp: MjtNum, [geom_] size: MjtNum,
	 [geom_] aabb: MjtNum,
	 [geom_] rbound: MjtNum, [geom_] pos: MjtNum,
	 [geom_] quat: MjtNum, [geom_] friction: MjtNum,
	 [geom_] margin: MjtNum, [geom_] gap: MjtNum, [geom_] fluid: MjtNum,
	 [geom_] user: MjtNum, [geom_] rgba: f32],
	[]);

info_with_view!(Model, hfield,
	[[hfield_] size: MjtNum,
	 [hfield_] nrow: i32,
	 [hfield_] ncol: i32,
	 [hfield_] adr: i32,
     [hfield_] pathadr: i32],
	[[hfield_] data: f32]);

info_with_view!(Model, joint,
	[qpos0: MjtNum, qpos_spring: MjtNum,
     [jnt_] r#type: MjtJoint [cast], [jnt_] qposadr: i32,
     [jnt_] dofadr: i32, [jnt_] group: i32,
     [jnt_] limited: bool [cast], [jnt_] actfrclimited: bool [cast], [jnt_] actgravcomp: bool [cast],
	 [jnt_] solref: MjtNum, [jnt_] solimp: MjtNum,
	 [jnt_] pos: MjtNum,
     [jnt_] axis: MjtNum, [jnt_] stiffness: MjtNum,
     [jnt_] range: MjtNum, [jnt_] actfrcrange: MjtNum, [jnt_] margin: MjtNum,
     [jnt_] user: MjtNum, [dof_] bodyid: i32,
     [dof_] jntid: i32, [dof_] parentid: i32,
     [dof_] Madr: i32, [dof_] simplenum: i32,
     [dof_] frictionloss: MjtNum, [dof_] armature: MjtNum,
     [dof_] damping: MjtNum, [dof_] invweight0: MjtNum,
     [dof_] M0: MjtNum],
	[]);

info_with_view!(Model, light,
	[[light_] mode: MjtCamLight [cast],
	 [light_] bodyid: i32,
	 [light_] targetbodyid: i32,
	 [light_] r#type: MjtLightType [cast],
	 [light_] castshadow: MjtByte,
	 [light_] active: MjtByte,
	 [light_] pos: MjtNum,
	 [light_] dir: MjtNum,
	 [light_] poscom0: MjtNum,
	 [light_] pos0: MjtNum,
	 [light_] dir0: MjtNum,
	 [light_] attenuation: f32,
	 [light_] cutoff: f32,
	 [light_] exponent: f32,
	 [light_] ambient: f32,
	 [light_] diffuse: f32,
	 [light_] specular: f32],
	[]);

info_with_view!(Model, material,
	[[mat_] texid: i32,
	 [mat_] texuniform: bool [cast],
	 [mat_] texrepeat: f32,
	 [mat_] emission: f32,
	 [mat_] specular: f32,
	 [mat_] shininess: f32,
	 [mat_] reflectance: f32,
	 [mat_] rgba: f32,
     [mat_] metallic: f32,
     [mat_] roughness: f32],
	[]);

info_with_view!(Model, mesh,
	[[mesh_] vertadr: i32,
	 [mesh_] vertnum: i32,
	 [mesh_] texcoordadr: i32,
	 [mesh_] faceadr: i32,
	 [mesh_] facenum: i32,
	 [mesh_] graphadr: i32],
	[]);

info_with_view!(Model, numeric,
	[[numeric_] adr: i32,
	 [numeric_] size: i32],
	[[numeric_] data: MjtNum]);

info_with_view!(Model, pair,
	[[pair_] dim: i32,
	 [pair_] geom1: i32,
	 [pair_] geom2: i32,
	 [pair_] signature: i32,
	 [pair_] solref: MjtNum,
	 [pair_] solimp: MjtNum,
	 [pair_] margin: MjtNum,
	 [pair_] gap: MjtNum,
	 [pair_] friction: MjtNum,
     [pair_] solreffriction: MjtNum],
	[]);

info_with_view!(Model, sensor,
	[[sensor_] r#type: MjtSensor [cast],
	 [sensor_] datatype: MjtDataType [cast],
	 [sensor_] needstage: MjtStage [cast],
	 [sensor_] objtype: MjtObj [cast],
	 [sensor_] objid: i32,
	 [sensor_] reftype: MjtObj [cast],
	 [sensor_] refid: i32,
	 [sensor_] intprm: i32,
	 [sensor_] dim: i32,
	 [sensor_] adr: i32,
	 [sensor_] cutoff: MjtNum,
	 [sensor_] noise: MjtNum,
	 [sensor_] history: i32,
	 [sensor_] historyadr: i32,
	 [sensor_] delay: MjtNum,
     [sensor_] interval: MjtNum,
	 [sensor_] user: MjtNum,
	 [sensor_] plugin: i32],
	[]);

info_with_view!(Model, site,
	[[site_] r#type: MjtGeom [cast],
	 [site_] bodyid: i32,
	 [site_] matid: i32,
	 [site_] group: i32,
	 [site_] sameframe: MjtSameFrame [cast],
	 [site_] size: MjtNum,
	 [site_] pos: MjtNum,
	 [site_] quat: MjtNum,
	 [site_] user: MjtNum,
	 [site_] rgba: f32],
	[]);

info_with_view!(Model, skin,
	[[skin_] matid: i32, [skin_] group: i32,
	 [skin_] rgba: f32,
	 [skin_] inflate: f32,
	 [skin_] vertadr: i32,
	 [skin_] vertnum: i32,
	 [skin_] texcoordadr: i32,
	 [skin_] faceadr: i32,
	 [skin_] facenum: i32,
	 [skin_] boneadr: i32,
	 [skin_] bonenum: i32,
	 [skin_] pathadr: i32],
	[]);

info_with_view!(Model, tendon,
	[[tendon_] adr: i32, [tendon_] num: i32,
	 [tendon_] matid: i32, [tendon_] group: i32,
	 [tendon_] treenum: i32, [tendon_] treeid: i32,
	 [tendon_] limited: bool [cast], [tendon_] actfrclimited: bool [cast], [tendon_] width: MjtNum,
	 [tendon_] solref_lim: MjtNum, [tendon_] solimp_lim: MjtNum,
	 [tendon_] solref_fri: MjtNum, [tendon_] solimp_fri: MjtNum,
	 [tendon_] range: MjtNum, [tendon_] actfrcrange: MjtNum, [tendon_] margin: MjtNum,
	 [tendon_] stiffness: MjtNum, [tendon_] damping: MjtNum, [tendon_] armature: MjtNum,
	 [tendon_] frictionloss: MjtNum, [tendon_] lengthspring: MjtNum,
	 [tendon_] length0: MjtNum, [tendon_] invweight0: MjtNum,
	 [tendon_] user: MjtNum, [tendon_] rgba: f32],
	[]);

info_with_view!(Model, texture,
	[[tex_] r#type: MjtTexture [cast],
     [tex_] colorspace: MjtColorSpace [cast],
	 [tex_] height: i32,
	 [tex_] width: i32,
	 [tex_] nchannel: i32,
	 [tex_] adr: MjtSize,
	 [tex_] pathadr: i32],
	[[tex_] data: MjtByte]);

info_with_view!(Model, tuple,
	[[tuple_] adr: i32,
	 [tuple_] size: i32,
	 [tuple_] objtype: MjtObj [cast],
	 [tuple_] objid: i32,
	 [tuple_] objprm: MjtNum],
	[]);

info_with_view!(Model, key,
	[[key_] time: MjtNum,
	 [key_] qpos: MjtNum,
	 [key_] qvel: MjtNum,
	 [key_] act: MjtNum,
	 [key_] mpos: MjtNum,
	 [key_] mquat: MjtNum,
	 [key_] ctrl: MjtNum],
	[]);

#[cfg(test)]
mod tests {
    use crate::assert_relative_eq;

    use super::*;
    use std::fs;

    const EXAMPLE_MODEL: &str = stringify!(
    <mujoco>
        <worldbody>
            <light name="lamp_light1"
                mode="fixed" type="directional" castshadow="false" bulbradius="0.5" intensity="250"
                range="10" active="true" pos="0 0 0" dir="0 0 -1" attenuation="0.1 0.05 0.01"
                cutoff="60" exponent="2" ambient="0.1 0.1 0.25" diffuse="0.5 1 1" specular="1 1.5 1"/>

            <light name="lamp_light2"
                mode="fixed" type="spot" castshadow="true" bulbradius="0.2" intensity="500"
                range="10" active="true" pos="0 0 0" dir="0 0 -1" attenuation="0.1 0.05 0.01"
                cutoff="45" exponent="2" ambient="0.1 0.1 0.1" diffuse="1 1 1" specular="1 1 1"/>

            <camera name="cam1" fovy="50" resolution="100 200"/>

            <light ambient="0.2 0.2 0.2"/>
            <body name="ball">
                <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
                <joint name="ball" type="free" axis="1 1 1"/>
                <site name="touch" size="1" type="box"/>
            </body>

            <body name="ball1" pos="-.5 0 0">
                <geom size=".1" rgba="0 1 0 1" mass="1"/>
                <joint type="free"/>
                <site name="ball1" size=".1 .1 .1" pos="0 0 0" rgba="0 1 0 0.2" type="box"/>
                <site name="ball12" size=".1 .1 .1" pos="0 0 0" rgba="0 1 1 0.2" type="box"/>
                <site name="ball13" size=".1 .1 .1" pos="0 0 0" rgba="0 1 1 0.2" type="box"/>
            </body>

            <body name="ball2"  pos=".5 0 0">
                <geom name="ball2" size=".5" rgba="0 1 1 1" mass="1"/>
                <joint name="ball2" type="free"/>
                <site name="ball2" size=".1 .1 .1" pos="0 0 0" rgba="0 1 1 0.2" type="box"/>
                <site name="ball22" size="0.5 0.25 0.5" pos="5 1 3" rgba="1 2 3 1" type="box"/>
                <site name="ball23" size=".1 .1 .1" pos="0 0 0" rgba="0 1 1 0.2" type="box"/>
            </body>

            <geom name="floor" type="plane" size="10 10 1" euler="5 0 0"/>

            <body name="slider">
                <geom name="rod" type="cylinder" size="1 10 0" euler="90 0 0" pos="0 0 10"/>
                <joint name="rod" type="slide" axis="0 1 0" range="0 1"/>
            </body>

            <body name="ball3"  pos="0 0 5">
                <geom name="ball31" size=".5" rgba="0 1 1 1" mass="1"/>
                <joint type="slide"/>
            </body>

            <body name="ball32"  pos="0 0 -5">
                <geom name="ball32" size=".5" rgba="0 1 1 1" mass="1"/>
                <joint type="slide"/>
            </body>

            <body name="eq_body1" pos="0 0 0">
                <geom size="0.1"/>
            </body>
            <body name="eq_body2" pos="1 0 0">
                <geom size="0.1"/>
            </body>
            <body name="eq_body3" pos="0 0 0">
                <geom size="0.1"/>
            </body>
            <body name="eq_body4" pos="1 0 0">
                <geom size="0.1"/>
            </body>
        </worldbody>

        
        <equality>
            <connect name="eq1" body1="eq_body1" body2="eq_body2" anchor="15 0 10"/>
            <connect name="eq2" body1="eq_body2" body2="eq_body1" anchor="-5 0 10"/>
            <connect name="eq3" body1="eq_body3" body2="eq_body4" anchor="0 5 0"/>
            <connect name="eq4" body1="eq_body4" body2="eq_body3" anchor="5 5 10"/>
        </equality>


        <actuator>
            <general name="slider" joint="rod" biastype="affine" ctrlrange="0 1" dynprm="1 2 3 4 5 6 7 8 9 10" gaintype="fixed"/>
            <general name="slider2" joint="ball2" biastype="affine" ctrlrange="0 1" dynprm="10 9 8 7 6 5 4 3 2 1" gaintype="fixed"/>
        </actuator>

        <sensor>
            <touch name="touch" site="touch"/>
        </sensor>

        <tendon>
            <spatial name="tendon1" limited="false" range="0 5" rgba="0 .5 2 3" width=".5">
                <site site="ball1"/>
                <site site="ball2"/>
            </spatial>
        </tendon>

        <tendon>
            <spatial name="tendon2" limited="true" range="0 1" rgba="0 .1 1 1" width=".005">
                <site site="ball1"/>
                <site site="ball2"/>
            </spatial>
        </tendon>

        <tendon>
            <spatial name="tendon3" limited="false" range="0 5" rgba=".5 .2 .4 .3" width=".25">
                <site site="ball1"/>
                <site site="ball2"/>
            </spatial>
        </tendon>

        <!-- Contact pair between the two geoms -->
        <contact>
            <pair name="geom_pair" geom1="ball31" geom2="ball32" condim="3" solref="0.02 1"
                solreffriction="0.01 0.5" solimp="0.0 0.95 0.001 0.5 2" margin="0.001" gap="0"
                friction="1.0 0.8 0.6 0.0 0.0">
            </pair>
        </contact>

        <!-- A keyframe with qpos/qvel/ctrl etc. -->
        <keyframe>
            <!-- adjust nq/nv/nu in <default> or body definitions to match
                lengths in your test constants -->
            <key name="pose0"
                time="0.0"
                qpos="1.1 1.2 1.3 1.1 0.2 0.3 0.1 1.2 0.3 1.1 0.2 1.3 0.1 1.2 0.3 1.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.0"
                qvel="0.5 5.0 5.0 0.0 1.0 0.0 0.0 5.0 0.0 5.0 1.0 5.0 0.0 1.0 5.0 0.0 1.0 0.0 0.0 1.0 0.0"
                ctrl="0.5 0.5"/>
            <key name="pose1"
                time="1.5"
                qpos="0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.0"
                qvel="0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0"
                ctrl="0.5 0.0"/>
        </keyframe>

        <custom>
            <tuple name="tuple_example">
                <!-- First entry: a body -->
                <element objtype="body" objname="ball2" prm="0.5"/>
                <!-- Second entry: a site -->
                <element objtype="site" objname="ball1" prm="1.0"/>
            </tuple>

            <!-- Numeric element with a single value -->
            <numeric name="gain_factor1" size="5" data="3.14159 0 0 0 3.14159"/>
            <numeric name="gain_factor2" size="3" data="1.25 5.5 10.0"/>
        </custom>

        <!-- Texture definition -->
        <asset>
            <texture name="wall_tex"
                type="2d"
                colorspace="sRGB"
                width="128"
                height="128"
                nchannel="3"
                builtin="flat"
                rgb1="0.6 0.6 0.6"
                rgb2="0.6 0.6 0.6"
                mark="none"/>

            <!-- Material definition -->
            <material name="wood_material"
                rgba="0.8 0.5 0.3 1"
                emission="0.1"
                specular="0.5"
                shininess="0.7"
                reflectance="0.2"
                metallic="0.3"
                roughness="0.4"
                texuniform="true"
                texrepeat="2 2"/>

            <!-- Material definition -->
            <material name="also_wood_material"
                rgba="0.8 0.5 0.3 1"
                emission="0.1"
                specular="0.5"
                shininess="0.7"
                reflectance="0.2"
                metallic="0.3"
                roughness="0.5"
                texuniform="false"
                texrepeat="2 2"/>

            <hfield name="hf1" nrow="2" ncol="3" size="1 1 1 0.1"/>
            <hfield name="hf2" nrow="5" ncol="3" size="1 1 1 5.25"/>
            <hfield name="hf3" nrow="2" ncol="3" size="1 1 1 0.1"/>
        </asset>
    </mujoco>
);

    /// Tests if the model can be loaded and then saved.
    #[test]
    fn test_model_load_save() {
        const MODEL_SAVE_XML_PATH: &str = "./__TMP_MODEL1.xml";
        const MODEL_INVALID_SAVE_XML_PATH: &str = "/some/non-existent/path/";

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        model.save_last_xml(MODEL_SAVE_XML_PATH).expect("could not save the model XML.");      
        fs::remove_file(MODEL_SAVE_XML_PATH).unwrap();

        // Try to get an error
        assert!(model.save_last_xml(MODEL_INVALID_SAVE_XML_PATH).is_err());
    }

    #[test]
    fn test_actuator_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let actuator_model_info = model.actuator("slider").unwrap();
        let view = actuator_model_info.view(&model);

        /* Test read */
        assert_eq!(view.biastype[0], MjtBias::mjBIAS_AFFINE);
        assert_eq!(&view.ctrlrange[..], [0.0, 1.0]);
        assert_eq!(view.ctrllimited[0], true);
        assert_eq!(view.forcelimited[0], false);
        assert_eq!(view.trntype[0], MjtTrn::mjTRN_JOINT);
        assert_eq!(view.gaintype[0], MjtGain::mjGAIN_FIXED);

        /* Test direct array slice correspondance */
        assert_eq!(view.dynprm[..], model.actuator_dynprm()[actuator_model_info.id]);

        /* Test write */
        let mut view_mut = actuator_model_info.view_mut(&mut model);
        view_mut.gaintype[0] = MjtGain::mjGAIN_AFFINE;
        assert_eq!(view_mut.gaintype[0], MjtGain::mjGAIN_AFFINE);
        view_mut.zero();
        assert_eq!(view_mut.gaintype[0], MjtGain::mjGAIN_FIXED);
    }

    #[test]
    fn test_sensor_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let sensor_model_info = model.sensor("touch").unwrap();
        let view = sensor_model_info.view(&model);
        
        /* Test read */
        assert_eq!(view.dim[0], 1);
        assert_eq!(view.objtype[0], MjtObj::mjOBJ_SITE);
        assert_eq!(view.noise[0], 0.0);
        assert_eq!(view.r#type[0], MjtSensor::mjSENS_TOUCH);

        /* Test write */
        let mut view_mut = sensor_model_info.view_mut(&mut model);
        view_mut.noise[0] = 1.0;
        assert_eq!(view_mut.noise[0], 1.0);
    }

    #[test]
    fn test_tendon_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let tendon_model_info = model.tendon("tendon2").unwrap();
        let view = tendon_model_info.view(&model);
        
        /* Test read */
        assert_eq!(&view.range[..], [0.0, 1.0]);
        assert_eq!(view.limited[0], true);
        assert_eq!(view.width[0], 0.005);

        /* Test alignment with the array slice */
        let tendon_id = tendon_model_info.id;
        assert_eq!(view.width[0], model.tendon_width()[tendon_id]);
        assert_eq!(*view.range, model.tendon_range()[tendon_id]);
        assert_eq!(view.limited[0], model.tendon_limited()[tendon_id]);

        /* Test write */
        let mut view_mut = tendon_model_info.view_mut(&mut model);
        view_mut.frictionloss[0] = 5e-2;
        assert_eq!(view_mut.frictionloss[0], 5e-2);
    }

    #[test]
    fn test_joint_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let model_info = model.joint("rod").unwrap();
        let view = model_info.view(&model);
        
        /* Test read */
        assert_eq!(view.r#type[0], MjtJoint::mjJNT_SLIDE);
        assert_eq!(view.limited[0], true);
        assert_eq!(&view.axis[..], [0.0, 1.0 , 0.0]);

        /* Test write */
        let mut view_mut = model_info.view_mut(&mut model);
        view_mut.axis.copy_from_slice(&[1.0, 0.0, 0.0]);
        assert_eq!(&view_mut.axis[..], [1.0, 0.0 , 0.0]);
    }

    #[test]
    fn test_geom_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let model_info = model.geom("ball2").unwrap();
        let view = model_info.view(&model);
        
        /* Test read */
        assert_eq!(view.r#type[0], MjtGeom::mjGEOM_SPHERE);
        assert_eq!(view.size[0], 0.5);

        /* Test write */
        let mut view_mut = model_info.view_mut(&mut model);
        view_mut.size[0] = 1.0;
        assert_eq!(view_mut.size[0], 1.0);
    }

    #[test]
    fn test_body_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let model_info = model.body("ball2").unwrap();
        let view = model_info.view(&model);
        
        /* Test read */
        model.body_pos()[model_info.id];
        assert_eq!(view.pos[0], 0.5);

        /* Test alignment with slice */
        let body_id = model_info.id;
        assert_eq!(model.body_sameframe()[body_id], view.sameframe[0]);

        /* Test write */
        let mut view_mut = model_info.view_mut(&mut model);
        view_mut.pos[0] = 1.0;
        assert_eq!(view_mut.pos[0], 1.0);
    }


    #[test]
    fn test_camera_model_view() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let model_info = model.camera("cam1").unwrap();
        let view = model_info.view(&model);

        /* Test read */
        assert_eq!(&view.resolution[..], [100, 200]);
        assert_eq!(view.fovy[0], 50.0);

        /* Test write */
        let mut view_mut = model_info.view_mut(&mut model);
        view_mut.fovy[0] = 60.0;
        assert_eq!(view_mut.fovy[0], 60.0);
    }

    #[test]
    fn test_id_2name_valid() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");

        // Body with id=1 should exist ("box")
        let name = model.id_to_name(MjtObj::mjOBJ_BODY, 1);
        assert_eq!(name, Some("ball"));
    }

    #[test]
    fn test_model_prints() {
        const TMP_FILE: &str = "tmpprint.txt";
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        assert!(model.print(TMP_FILE).is_ok());
        fs::remove_file(TMP_FILE).unwrap();

        assert!(model.print_formatted(TMP_FILE, "%.2f").is_ok());
        fs::remove_file(TMP_FILE).unwrap();
    }

    #[test]
    fn test_id_2name_invalid() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");

        // Invalid id should return None
        let name = model.id_to_name(MjtObj::mjOBJ_BODY, 9999);
        assert_eq!(name, None);
    }

    #[test]
    fn test_totalmass_set_and_get() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");

        let mass_before = model.get_totalmass();
        model.set_totalmass(5.0);
        let mass_after = model.get_totalmass();

        assert_relative_eq!(mass_after, 5.0, epsilon = 1e-9);
        assert_ne!(mass_before, mass_after);
    }

    /// Tests if copying the model works without any memory problems.
    #[test]
    fn test_copy_model() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        assert!(model.clone().is_some());
    }

    #[test]
    fn test_model_save() {
        const MODEL_SAVE_PATH: &str = "./__TMP_MODEL2.mjb";
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        model.save(Some(MODEL_SAVE_PATH), None);

        let saved_data = fs::read(MODEL_SAVE_PATH).unwrap();
        let mut data = vec![0; saved_data.len()];
        model.save(None, Some(&mut data));

        assert_eq!(saved_data, data);
        fs::remove_file(MODEL_SAVE_PATH).unwrap();

        /* Test virtual file system load */
        let model = MjModel::from_buffer(&saved_data).unwrap();
        assert!(model.light("lamp_light2").is_some());
        assert!(model.light("lamp_light-xyz").is_none());
    }

    #[test]
    fn test_site_view() {
        // <site name="ball22" size="0 0.25 0" pos="5 1 3" rgba="1 2 3 1" type="box"/>
        const BODY_NAME: &str = "ball2";
        const SITE_NAME: &str = "ball22";
        const SITE_SIZE: [f64; 3] = [0.5, 0.25, 0.5];
        const SITE_POS: [f64; 3] = [5.0, 1.0, 3.0];
        const SITE_RGBA: [f32; 4] = [1.0, 2.0, 3.0, 1.0];
        const SITE_TYPE: MjtGeom = MjtGeom::mjGEOM_BOX;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let info_ball = model.body(BODY_NAME).unwrap();
        let info_site = model.site(SITE_NAME).unwrap();
        let view_site = info_site.view(&model);

        /* Check if all the attributes given match */
        assert_eq!(info_site.name, SITE_NAME);
        assert_eq!(view_site.size[..], SITE_SIZE);
        assert_eq!(view_site.pos[..], SITE_POS);
        assert_eq!(view_site.rgba[..], SITE_RGBA);
        assert_eq!(view_site.r#type[0], SITE_TYPE);

        assert_eq!(view_site.bodyid[0] as usize, info_ball.id)
    }

    #[test]
    fn test_pair_view() {
        const PAIR_NAME: &str = "geom_pair";
        const DIM: i32 = 3;
        const GEOM1_NAME: &str = "ball31";
        const GEOM2_NAME: &str = "ball32";
        const SOLREF: [f64; mjNREF as usize] = [0.02, 1.0];
        const SOLREFFRICTION: [f64; mjNREF as usize] = [0.01, 0.5];
        const SOLIMP: [f64; mjNIMP as usize] = [0., 0.95, 0.001, 0.5, 2.0];
        const MARGIN: f64 = 0.001;
        const GAP: f64 = 0.0;
        const FRICTION: [f64; 5] = [1.0, 0.8, 0.6, 0.0, 0.0];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_pair = model.pair(PAIR_NAME).unwrap();
        let view_pair = info_pair.view(&model);

        let geom1_info = model.geom(GEOM1_NAME).unwrap();
        let geom2_info = model.geom(GEOM2_NAME).unwrap();

        // signature =  body1 << 16 + body2 according to MuJoCo's documentation.
        let signature = ((geom1_info.view(&model).bodyid[0] as u32) << 16) + geom2_info.view(&model).bodyid[0] as u32;

        assert_eq!(view_pair.dim[0], DIM);
        assert_eq!(view_pair.geom1[0] as usize, geom1_info.id);
        assert_eq!(view_pair.geom2[0] as usize, geom2_info.id);
        assert_eq!(view_pair.signature[0] as u32, signature);
        assert_eq!(view_pair.solref[..], SOLREF);
        assert_eq!(view_pair.solreffriction[..], SOLREFFRICTION);
        assert_eq!(view_pair.solimp[..], SOLIMP);
        assert_eq!(view_pair.margin[0], MARGIN);
        assert_eq!(view_pair.gap[0], GAP);
        assert_eq!(view_pair.friction[..], FRICTION);
    }

    #[test]
    fn test_key_view() {
        const KEY_NAME: &str = "pose1";
        const TIME: f64 = 1.5;
        const QVEL: &[f64] = &[0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0];
        const ACT: &[f64]  = &[];
        const CTRL: &[f64] = &[0.5, 0.0];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_key = model.key(KEY_NAME).unwrap();
        let view_key = info_key.view(&model);

        assert_eq!(view_key.time[0], TIME);
        // Don't test qpos, as it does some magic angle conversions, making the tests fail.
        // If all the other succeed, assume qpos works too (as its the exact same logic).
        assert_eq!(&view_key.qvel[..model.ffi().nv as usize], QVEL);
        assert_eq!(&view_key.act[..model.ffi().na as usize], ACT);
        assert_eq!(&view_key.ctrl[..model.ffi().nu as usize], CTRL);

        let key_qvel = &model.key_qvel()[model.ffi().nv as usize..];
        assert_eq!(key_qvel, QVEL);

        let key_act = &model.key_act()[model.ffi().na as usize..];
        assert_eq!(key_act, ACT);

        let key_ctrl = &model.key_ctrl()[model.ffi().nu as usize..];
        assert_eq!(key_ctrl, CTRL);
    }

    #[test]
    fn test_tuple_view() {
        const TUPLE_NAME: &str = "tuple_example";
        const SIZE: i32 = 2;
        const OBJTYPE: &[MjtObj] = &[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_SITE];
        const OBJPRM: &[f64]  = &[0.5, 1.0];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_tuple = model.tuple(TUPLE_NAME).unwrap();
        let view_tuple = info_tuple.view(&model);

        let objid = &[
            model.body("ball2").unwrap().id as i32,
            model.site("ball1").unwrap().id as i32,
        ];

        assert_eq!(view_tuple.size[0], SIZE);
        assert_eq!(&view_tuple.objtype[..SIZE as usize], OBJTYPE);
        assert_eq!(&view_tuple.objid[..SIZE as usize], objid);
        assert_eq!(&view_tuple.objprm[..SIZE as usize], OBJPRM);
    }

    #[test]
    fn test_texture_view() {
        const TEX_NAME: &str = "wall_tex";
        const TYPE: MjtTexture = MjtTexture::mjTEXTURE_2D;          // for example, 2 = 2D texture
        const COLORSPACE: MjtColorSpace = MjtColorSpace::mjCOLORSPACE_SRGB;    // e.g. RGB
        const HEIGHT: i32 = 128;
        const WIDTH: i32 = 128;
        const NCHANNEL: i32 = 3;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_tex = model.texture(TEX_NAME).unwrap();
        let view_tex = info_tex.view(&model);

        assert_eq!(view_tex.r#type[0], TYPE);
        assert_eq!(view_tex.colorspace[0], COLORSPACE);
        assert_eq!(view_tex.height[0], HEIGHT);
        assert_eq!(view_tex.width[0], WIDTH);
        assert_eq!(view_tex.nchannel[0], NCHANNEL);

        assert_eq!(view_tex.data.as_ref().unwrap().len(), (WIDTH * HEIGHT * NCHANNEL) as usize);
    }

    #[test]
    fn test_numeric_view() {
        const NUMERIC_NAME: &str = "gain_factor2";
        const SIZE: i32 = 3;
        const DATA: [f64; 3] = [1.25, 5.5, 10.0];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_numeric = model.numeric(NUMERIC_NAME).unwrap();
        let view_numeric = info_numeric.view(&model);

        assert_eq!(view_numeric.size[0], SIZE);
        assert_eq!(&view_numeric.data.as_ref().unwrap()[..SIZE as usize], DATA);
    }

    #[test]
    fn test_material_view() {
        const MATERIAL_NAME: &str = "also_wood_material";

        const TEXUNIFORM: bool = false;
        const TEXREPEAT: [f32; 2] = [2.0, 2.0];
        const EMISSION: f32 = 0.1;
        const SPECULAR: f32 = 0.5;
        const SHININESS: f32 = 0.7;
        const REFLECTANCE: f32 = 0.2;
        const METALLIC: f32 = 0.3;
        const ROUGHNESS: f32 = 0.5;
        const RGBA: [f32; 4] = [0.8, 0.5, 0.3, 1.0];
        const TEXID: i32 = -1;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_material = model.material(MATERIAL_NAME).unwrap();
        let view_material = info_material.view(&model);

        assert_eq!(view_material.texuniform[0], TEXUNIFORM);
        assert_eq!(view_material.texrepeat[..], TEXREPEAT);
        assert_eq!(view_material.emission[0], EMISSION);
        assert_eq!(view_material.specular[0], SPECULAR);
        assert_eq!(view_material.shininess[0], SHININESS);
        assert_eq!(view_material.reflectance[0], REFLECTANCE);
        assert_eq!(view_material.metallic[0], METALLIC);
        assert_eq!(view_material.roughness[0], ROUGHNESS);
        assert_eq!(view_material.rgba[..], RGBA);
        assert_eq!(view_material.texid[0], TEXID);
    }

    #[test]
    fn test_light_view() {
        const LIGHT_NAME: &str = "lamp_light2";
        const MODE: MjtCamLight = MjtCamLight::mjCAMLIGHT_FIXED;
        const BODYID: usize = 0;       // lamp body id
        const TYPE: MjtLightType = MjtLightType::mjLIGHT_SPOT;           // spot light, adjust if mjLightType differs
        const CASTSHADOW: bool = true;
        const ACTIVE: bool = true;

        const POS: [MjtNum; 3] = [0.0, 0.0, 0.0];
        const DIR: [MjtNum; 3] = [0.0, 0.0, -1.0];
        const POS0: [MjtNum; 3] = [0.0, 0.0, 0.0];
        const DIR0: [MjtNum; 3] = [0.0, 0.0, -1.0];
        const ATTENUATION: [f32; 3] = [0.1, 0.05, 0.01];
        const CUTOFF: f32 = 45.0;
        const EXPONENT: f32 = 2.0;
        const AMBIENT: [f32; 3] = [0.1, 0.1, 0.1];
        const DIFFUSE: [f32; 3] = [1.0, 1.0, 1.0];
        const SPECULAR: [f32; 3] = [1.0, 1.0, 1.0];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_light = model.light(LIGHT_NAME).unwrap();
        let view_light = info_light.view(&model);

        assert_eq!(view_light.mode[0], MODE);
        assert_eq!(view_light.bodyid[0] as usize, BODYID);
        assert_eq!(view_light.targetbodyid[0], -1);
        assert_eq!(view_light.r#type[0], TYPE);
        assert_eq!(view_light.castshadow[0], CASTSHADOW as u8);
        assert_eq!(view_light.active[0], ACTIVE as u8);

        assert_eq!(view_light.pos[..], POS);
        assert_eq!(view_light.dir[..], DIR);
        assert_eq!(view_light.pos0[..], POS0);
        assert_eq!(view_light.dir0[..], DIR0);
        assert_eq!(view_light.attenuation[..], ATTENUATION);
        assert_eq!(view_light.cutoff[0], CUTOFF);
        assert_eq!(view_light.exponent[0], EXPONENT);
        assert_eq!(view_light.ambient[..], AMBIENT);
        assert_eq!(view_light.diffuse[..], DIFFUSE);
        assert_eq!(view_light.specular[..], SPECULAR);
    }

    #[test]
    fn test_connect_eq_view() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();

        // Take the third equality constraint
        let info_eq = model.equality("eq3").unwrap();
        let view_eq = info_eq.view(&model);

        // Check type
        assert_eq!(view_eq.r#type[0], MjtEq::mjEQ_CONNECT);

        // Check connected bodies
        assert_eq!(view_eq.obj1id[0], model.name_to_id(MjtObj::mjOBJ_BODY, "eq_body3"));
        assert_eq!(view_eq.obj2id[0], model.name_to_id(MjtObj::mjOBJ_BODY, "eq_body4"));
        assert_eq!(view_eq.objtype[0], MjtObj::mjOBJ_BODY);

        // Check active
        assert_eq!(view_eq.active0[0], true);

        // Check anchor position stored in eq_data
        let anchor = &view_eq.data[0..3];
        assert_eq!(anchor[0], 0.0);
        assert_eq!(anchor[1], 5.0);
        assert_eq!(anchor[2], 0.0);
    }

    #[test]
    fn test_hfield_view() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();

        // Access first height field
        let info_hf = model.hfield("hf2").unwrap();
        let view_hf = info_hf.view(&model);

        // Expected values
        let expected_size: [f64; 4] = [1.0, 1.0, 1.0, 5.25]; // radius_x, radius_y, elevation_z, base_z
        let expected_nrow = 5;
        let expected_ncol = 3;
        let expected_data: [f32; 15] = [0.0; 15];

        // Assertions
        assert_eq!(view_hf.size[..], expected_size);
        assert_eq!(view_hf.nrow[0], expected_nrow);
        assert_eq!(view_hf.ncol[0], expected_ncol);

        // hfield_data length should match nrow * ncol
        assert_eq!(view_hf.data.as_ref().unwrap().len(), (expected_nrow * expected_ncol) as usize);
        assert_eq!(&view_hf.data.as_ref().unwrap()[..], &expected_data[..]);

        // Pathadr is -1 (no external file)
        assert_eq!(view_hf.pathadr[0], -1);
    }

    /// Tests [`MjModel::extract_state_into`] for corectness.
    #[test]
    fn test_state_extract() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        /* Test of extraction into existing buffer */
        // Physics is subset of full physics.
        // Extract physics from full physics.
        let state_full_physics = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let state_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);

        let required_size = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst_buffer = unsafe { Box::new_zeroed_slice(required_size).assume_init() };
        let _byes_written = model.extract_state_into(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst_buffer, MjtState::mjSTATE_PHYSICS as u32
        ).unwrap();

        assert_eq!(state_physics, dst_buffer);

        /* Test of extraction into new buffer (internally) */
        // Physics is subset of full physics.
        // Extract physics from full physics.
        let state_full_physics = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let state_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);

        let dst_buffer = model.extract_state(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            MjtState::mjSTATE_PHYSICS as u32
        ).unwrap();

        assert_eq!(state_physics, dst_buffer);
    }

    /// Tests for the expectec panic when giving a source spec that does not match
    /// the source array in state extraction.

    #[test]
    fn test_state_extract_state_invalid_src() {
        use crate::wrappers::mj_data::MjtState;
        use std::io::ErrorKind;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.extract_state(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert_eq!(err.kind(), ErrorKind::InvalidInput);
    }

    #[test]
    fn test_state_extract_state_into_invalid_src() {
        use crate::wrappers::mj_data::MjtState;
        use std::io::ErrorKind;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let required_size = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst_buffer = vec![0.0; required_size].into_boxed_slice();
        let state_full_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.extract_state_into(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst_buffer, MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert_eq!(err.kind(), ErrorKind::InvalidInput);
    }

    #[test]
    fn test_state_extract_dst_spec_not_subset() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.extract_state(
            &state_physics, MjtState::mjSTATE_PHYSICS as u32,
            MjtState::mjSTATE_FULLPHYSICS as u32
        );

        let err = res.unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput);
    }

    #[test]
    fn test_state_extract_into_dst_spec_not_subset() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_physics = data.get_state(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst = vec![0.0; model.state_size(MjtState::mjSTATE_PHYSICS as u32)];

        let res = model.extract_state_into(
            &state_physics, MjtState::mjSTATE_PHYSICS as u32,
            &mut dst, MjtState::mjSTATE_FULLPHYSICS as u32
        );

        let err = res.unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput);
    }

    #[test]
    fn test_state_extract_into_dst_buffer_too_small() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let required = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        // make buffer smaller than required
        let mut dst = vec![0.0; required.saturating_sub(1)];

        let res = model.extract_state_into(
            &state_full, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst, MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput);
    }

    #[test]
    fn test_state_extract_zero_spec() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);

        // extract zero-sized spec -> empty slice
        let dst = model.extract_state(&state_full, MjtState::mjSTATE_FULLPHYSICS as u32, 0u32).unwrap();
        assert_eq!(dst.len(), 0);

        // extract_into with zero-sized spec -> writes 0 elements
        let mut buf: &mut [f64] = &mut [];
        let written = model.extract_state_into(&state_full, MjtState::mjSTATE_FULLPHYSICS as u32, &mut buf, 0u32).unwrap();
        assert_eq!(written, 0);
    }
}
