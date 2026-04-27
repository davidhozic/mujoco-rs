//! MjModel related.
use crate::{
    view_creator, info_method, info_with_view,
    mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem,
    array_slice_dyn, getter_setter
};
use crate::util::{assert_mujoco_version, ERROR_BUF_LEN};
use crate::error::{MjDataError, MjModelError};
use crate::wrappers::mj_option::MjOption;
use crate::wrappers::mj_data::MjData;
use crate::mujoco_c::*;

use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic};
use super::mj_primitive::*;

use std::ffi::{c_char, CStr, CString, c_int, c_void};
use std::ptr::{self, NonNull};
use std::path::Path;


/*******************************************/
// Types
/// Constants which are powers of 2. They are used as bitmasks for the field `disableflags` of `mjOption`.
/// At runtime this field is `m->opt.disableflags`. The number of these constants is given by `mjNDISABLE` which is
/// also the length of the global string array `mjDISABLESTRING` with text descriptions of these flags.
pub type MjtDisableBit = mjtDisableBit;

/// Constants which are powers of 2. They are used as bitmasks for the field `enableflags` of `mjOption`.
/// At runtime this field is `m->opt.enableflags`. The number of these constants is given by `mjNENABLE` which is also
/// the length of the global string array `mjENABLESTRING` with text descriptions of these flags.
pub type MjtEnableBit = mjtEnableBit;

/// Primitive joint types. These values are used in `m->jnt_type`. The numbers in the comments indicate how many
/// positional coordinates each joint type has. Note that ball joints and rotational components of free joints are
/// represented as unit quaternions - which have 4 positional coordinates but 3 degrees of freedom each.
pub type MjtJoint = mjtJoint;

/// Geometric types supported by MuJoCo. The first group are "official" geom types that can be used in the model. The
/// second group are geom types that cannot be used in the model but are used by the visualizer to add decorative
/// elements. These values are used in `m->geom_type` and `m->site_type`.
pub type MjtGeom = mjtGeom;

/// Type of camera projection. Used in `m->cam_projection`.
pub type MjtProjection = mjtProjection;

/// Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed. These
/// values are used in `m->cam_mode` and `m->light_mode`.
pub type MjtCamLight = mjtCamLight;

/// The type of a light source describing how its position, orientation and other properties will interact with the
/// objects in the scene. These values are used in `m->light_type`.
pub type MjtLightType = mjtLightType;

/// Texture types, specifying how the texture will be mapped. These values are used in `m->tex_type`.
pub type MjtTexture = mjtTexture;

/// Texture roles, specifying how the renderer should interpret the texture.  Note that the MuJoCo built-in renderer only
/// uses RGB textures.  These values are used to store the texture index in the material's array `m->mat_texid`.
pub type MjtTextureRole = mjtTextureRole;

/// Type of color space encoding for textures.
pub type MjtColorSpace = mjtColorSpace;

/// Mode for actuator length-range computation.
pub type MjtLRMode = mjtLRMode;

/// Cube map face indices used by [`MjsTexture::set_cubefile`](super::mj_editing::MjsTexture::set_cubefile).
///
/// Each variant corresponds to one face of a cube-map texture, matching the order
/// MuJoCo uses internally (right=0, left=1, up=2, down=3, front=4, back=5).
///
/// **Note:** this enum is defined in mujoco-rs only; MuJoCo's C API uses raw integer
/// indices for cube-map faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(i32)]
pub enum MjtCubeFace {
    /// Positive-X face (index 0).
    Right  = 0,
    /// Negative-X face (index 1).
    Left   = 1,
    /// Positive-Y face (index 2).
    Up     = 2,
    /// Negative-Y face (index 3).
    Down   = 3,
    /// Positive-Z face (index 4).
    Front  = 4,
    /// Negative-Z face (index 5).
    Back   = 5,
}

/// Numerical integrator types. These values are used in `m->opt.integrator`.
pub type MjtIntegrator = mjtIntegrator;

/// Available friction cone types. These values are used in `m->opt.cone`.
pub type MjtCone = mjtCone;

/// Available Jacobian types. These values are used in `m->opt.jacobian`.
pub type MjtJacobian = mjtJacobian;

/// Available constraint solver algorithms. These values are used in `m->opt.solver`.
pub type MjtSolver = mjtSolver;

/// Equality constraint types. These values are used in `m->eq_type`.
pub type MjtEq = mjtEq;

/// Tendon wrapping object types. These values are used in `m->wrap_type`.
pub type MjtWrap = mjtWrap;

/// Actuator transmission types. These values are used in `m->actuator_trntype`.
pub type MjtTrn = mjtTrn;

/// Actuator dynamics types. These values are used in `m->actuator_dyntype`.
pub type MjtDyn = mjtDyn;

/// Actuator gain types. These values are used in `m->actuator_gaintype`.
pub type MjtGain = mjtGain;

/// Actuator bias types. These values are used in `m->actuator_biastype`.
pub type MjtBias = mjtBias;

/// MuJoCo object types. These are used, for example, in the support functions `mj_name2id` and
/// `mj_id2name` to convert between object names and integer ids.
pub type MjtObj = mjtObj;

/// Sensor types. These values are used in `m->sensor_type`.
pub type MjtSensor = mjtSensor;

/// These are the compute stages for the skipstage parameters of `mj_forwardSkip` and
/// `mj_inverseSkip`.
pub type MjtStage = mjtStage;

/// These are the possible sensor data types, used in `mjData.sensor_datatype`.
pub type MjtDataType = mjtDataType;

/// Types of data fields returned by contact sensors.
pub type MjtConDataField = mjtConDataField;

/// Types of frame alignment of elements with their parent bodies. Used as shortcuts during `mj_kinematics` in the
/// last argument to `mj_local2global`.
pub type MjtSameFrame = mjtSameFrame;

/// Sleep policy associated with a tree. The compiler automatically chooses between `NEVER` and `ALLOWED`, but the user
/// can override this choice. Only the user can set the `INIT` policy (initialized as asleep).
pub type MjtSleepPolicy = mjtSleepPolicy;

/// Types of flex self-collisions midphase.
pub type MjtFlexSelf = mjtFlexSelf;

/// Formulas used to combine SDFs when calling mjc_distance and mjc_gradient.
pub type MjtSDFType = mjtSDFType;

/// Data fields returned by rangefinder sensors.
pub type MjtRayDataField = mjtRayDataField;

/// Camera output type bitflags.
pub type MjtCamOutBit = mjtCamOutBit;

// SAFETY: All MuJoCo C enums below are `#[repr(u32)]` (or `#[repr(u8)]`) and each
// has a variant with discriminant 0, so the all-zeros bit pattern is a valid value.
// This lets `info_with_view!`'s `zero()` method use the safe `Zeroable::zeroed()`
// instead of `unsafe { std::mem::zeroed() }`, providing a compile-time guarantee
// that only safe-to-zero types are used in views.
unsafe impl bytemuck::Zeroable for mjtTrn_ {}
unsafe impl bytemuck::Zeroable for mjtDyn_ {}
unsafe impl bytemuck::Zeroable for mjtGain_ {}
unsafe impl bytemuck::Zeroable for mjtBias_ {}
unsafe impl bytemuck::Zeroable for mjtObj_ {}
unsafe impl bytemuck::Zeroable for mjtSameFrame_ {}
unsafe impl bytemuck::Zeroable for mjtCamLight_ {}
unsafe impl bytemuck::Zeroable for mjtProjection_ {}
unsafe impl bytemuck::Zeroable for mjtEq_ {}
unsafe impl bytemuck::Zeroable for mjtGeom_ {}
unsafe impl bytemuck::Zeroable for mjtJoint_ {}
unsafe impl bytemuck::Zeroable for mjtLightType_ {}
unsafe impl bytemuck::Zeroable for mjtSensor_ {}
unsafe impl bytemuck::Zeroable for mjtDataType_ {}
unsafe impl bytemuck::Zeroable for mjtStage_ {}
unsafe impl bytemuck::Zeroable for mjtTexture_ {}
unsafe impl bytemuck::Zeroable for mjtColorSpace_ {}

/*******************************************/

/// A Rust-safe wrapper around mjModel.
/// Automatically clean after itself on destruction.
#[derive(Debug)]
pub struct MjModel(NonNull<mjModel>);

// SAFETY: MjModel owns its mjModel heap allocation exclusively. The data is not shared
// outside of Rust, except in the C++ code which is synchronized via wrapper APIs.
unsafe impl Send for MjModel {}
unsafe impl Sync for MjModel {}


impl MjModel {
    /// Loads the model from an XML file. To load from a virtual file system, use [`MjModel::from_xml_vfs`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, MjModelError> {
        Self::from_xml_file(path, None)
    }

    /// Loads the model from an XML file, located in a virtual file system (`vfs`)
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, MjModelError> {
        Self::from_xml_file(path, Some(vfs))
    }

    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, MjModelError> {
        assert_mujoco_version();

        let mut error_buffer = [0; ERROR_BUF_LEN];
        let path_str = path.as_ref().to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let path = CString::new(path_str).unwrap();
        let raw_ptr = unsafe { mj_loadXML(
            path.as_ptr(), vfs.map_or(ptr::null(), |v| v.ffi()),
            error_buffer.as_mut_ptr(), error_buffer.len() as c_int
        ) };

        Self::check_raw_model(raw_ptr, &error_buffer)
    }

    /// Loads the model from an XML string.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::VfsError`] if the internal VFS operation fails.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// Panics if the linked MuJoCo version does not match the version expected by mujoco-rs.
    pub fn from_xml_string(data: &str) -> Result<Self, MjModelError> {
        assert_mujoco_version();

        let mut vfs = MjVfs::new();
        let filename = "model.xml";

        // Add the file into a virtual file system
        vfs.add_from_buffer(filename, data.as_bytes())?;

        let mut error_buffer = [0; ERROR_BUF_LEN];
        let filename_c = CString::new(filename).unwrap();
        let raw_ptr = unsafe { mj_loadXML(
            filename_c.as_ptr(), vfs.ffi(),
            error_buffer.as_mut_ptr(), error_buffer.len() as c_int
        ) };

        Self::check_raw_model(raw_ptr, &error_buffer)
    }

    /// Loads the model from MJB raw data.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// Returns [`MjModelError::LoadFailed`] if MuJoCo fails to parse the MJB buffer.
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_buffer(data: &[u8]) -> Result<Self, MjModelError> {
        assert_mujoco_version();
        unsafe {
            Self::from_raw(mj_loadModelBuffer(data.as_ptr() as *const c_void, data.len() as i32))
        }
    }

    /// Creates a [`MjModel`] from a raw pointer.
    pub(crate) fn from_raw(ptr: *mut mjModel) -> Result<Self, MjModelError> {
        Self::check_raw_model(ptr, &[0])
    }

    /// Saves the last loaded XML to `filename`.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::SaveFailed`] with MuJoCo's error message if saving fails.
    /// # Panics
    /// When the path contains '\0' characters, a panic occurs.
    pub fn save_last_xml<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let mut error = [0; ERROR_BUF_LEN];
        let cstring = CString::new(path_str).unwrap();
        let result = unsafe { mj_saveLastXML(
            cstring.as_ptr(), self.ffi(),
            error.as_mut_ptr(), error.len() as i32
        ) };
        match result {
            1 => Ok(()),
            _ => {
                // SAFETY: error is zero-initialised and MuJoCo NUL-terminates the message it
                // writes into it; the resulting CStr borrows the stack buffer and is consumed
                // before the buffer goes out of scope.
                let cstr_error = unsafe { CStr::from_ptr(error.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjModelError::SaveFailed(cstr_error))
            },
        }
    }

    /// Creates a new [`MjData`] instance linked to this model.
    ///
    /// # Panics
    /// Panics if MuJoCo fails to allocate the data structure.
    /// Use [`MjModel::try_make_data`] for a fallible alternative.
    pub fn make_data(&self) -> MjData<&Self> {
        MjData::new(self)
    }

    /// Fallible version of [`MjModel::make_data`].
    ///
    /// # Errors
    /// Returns [`MjDataError::AllocationFailed`] if MuJoCo fails to allocate
    /// the data structure.
    pub fn try_make_data(&self) -> Result<MjData<&Self>, MjDataError> {
        MjData::try_new(self)
    }

    /// Wraps a raw model pointer returned by MuJoCo load functions.
    /// Returns an error with the C error-buffer message if the pointer is null.
    fn check_raw_model(ptr_model: *mut mjModel, error_buffer: &[c_char]) -> Result<Self, MjModelError> {
        match NonNull::new(ptr_model) {
            Some(nn) => Ok(Self(nn)),
            None => {
                // SAFETY: error_buffer is zero-initialised and MuJoCo always
                // NUL-terminates the message it writes into it.
                let message = unsafe { CStr::from_ptr(error_buffer.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjModelError::LoadFailed(message))
            }
        }
    }

    info_method! { Model, ffi(), actuator,
        [trntype: 1, dyntype: 1, gaintype: 1, biastype: 1, trnid: 2, actadr: 1, actnum: 1,
        group: 1, history: 2, historyadr: 1, delay: 1, ctrllimited: 1, forcelimited: 1, actlimited: 1, dynprm: mjNDYN as usize, gainprm: mjNGAIN as usize, biasprm: mjNBIAS as usize,
        actearly: 1, ctrlrange: 2, forcerange: 2, actrange: 2, damping: 1,
        dampingpoly: mjNPOLY as usize, armature: 1, gear: 6, cranklength: 1, acc0: 1, length0: 1,
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
        pos: 3, axis: 3, stiffness: 1, stiffnesspoly: mjNPOLY as usize,
        range: 2, actfrcrange: 2, margin: 1, bodyid: 1, actuatorid: 1],
        [user: nuser_jnt],
        [qpos0: nq, qpos_spring: nq, jntid: nv,
        dof_bodyid: nv, parentid: nv, dof_treeid: nv, Madr: nv, simplenum: nv, frictionloss: nv,
        armature: nv, damping: nv, dampingpoly: nv * mjNPOLY as usize, invweight0: nv, M0: nv]
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
        [mode: 1, bodyid: 1, targetbodyid: 1, r#type: 1, texid: 1, castshadow: 1,
        bulbradius: 1, intensity: 1, range: 1,
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
        facenum: 1, graphadr: 1,
        normaladr: 1, normalnum: 1, texcoordnum: 1,
        bvhadr: 1, bvhnum: 1, octadr: 1, octnum: 1,
        pathadr: 1, polynum: 1, polyadr: 1,
        scale: 3, pos: 3, quat: 4],
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
        [adr: 1, num: 1, matid: 1, actuatorid: 1, group: 1, treenum: 1, treeid: 2,
        limited: 1, actfrclimited: 1, width: 1,
        solref_lim: mjNREF as usize, solimp_lim: mjNIMP as usize, solref_fri: mjNREF as usize, solimp_fri: mjNIMP as usize, range: 2, actfrcrange: 2, margin: 1,
        stiffness: 1, stiffnesspoly: mjNPOLY as usize, damping: 1, dampingpoly: mjNPOLY as usize, armature: 1,
        frictionloss: 1, lengthspring: 2, length0: 1, invweight0: 1, J_rownnz: 1, J_rowadr: 1, rgba: 4],
        [user: nuser_tendon],
        [J_colind: nJten]
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
    /// Returns `None` if the name is not found.
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn name_to_id(&self, type_: MjtObj, name: &str) -> Option<usize> {
        let c_string = CString::new(name).unwrap();
        let id = unsafe {
            mj_name2id(self.ffi(), type_ as i32, c_string.as_ptr())
        };
        if id == -1 { None } else { Some(id as usize) }
    }

    /* Partially auto-generated */

    /// Fallible version of [`Clone::clone`].
    ///
    /// # Errors
    /// Returns [`MjModelError::AllocationFailed`] if MuJoCo fails to allocate
    /// the copy.
    pub fn try_clone(&self) -> Result<MjModel, MjModelError> {
        let ptr = unsafe { mj_copyModel(ptr::null_mut(), self.ffi()) };
        NonNull::new(ptr)
            .map(MjModel)
            .ok_or(MjModelError::AllocationFailed)
    }

    /// Save model to binary MJB file.
    ///
    /// # Returns
    /// `Ok(())` if the path is valid UTF-8 and contains no interior `\0` characters.
    /// **Note:** the underlying C function `mj_saveModel` returns `void`, so file I/O
    /// errors (e.g. permission denied, disk full) are not detectable; `Ok(())` does
    /// **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn save_to_file<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_saveModel(
            self.ffi(), c_filename.as_ptr(),
            ptr::null_mut(), 0
        ) };
        Ok(())
    }

    /// Save model to memory buffer.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjModelError::BufferTooSmall`] if the buffer is smaller than [`size()`](Self::size).
    pub fn save_to_buffer(&self, buffer: &mut [u8]) -> Result<(), MjModelError> {
        let needed = self.size();
        if buffer.len() < needed {
            return Err(MjModelError::BufferTooSmall {
                needed,
                available: buffer.len(),
            });
        }
        unsafe { mj_saveModel(
            self.ffi(), ptr::null(),
            buffer.as_mut_ptr() as *mut c_void, buffer.len() as i32
        ) };
        Ok(())
    }

    /// Return size of buffer needed to hold model.
    pub fn size(&self) -> usize {
        unsafe { mj_sizeModel(self.ffi()) as usize }
    }

    /// Print mjModel to text file, specifying format.
    /// float_format must be a valid printf-style format string for a single float value.
    ///
    /// # Returns
    /// `Ok(())` if the path and format string are valid UTF-8 and contain no interior `\0`
    /// characters. **Note:** the underlying C function `mj_printFormattedModel` returns `void`,
    /// so file I/O errors are not detectable; `Ok(())` does **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When either string contains '\0' characters, a panic occurs.
    pub fn print_formatted<T: AsRef<Path>>(&self, filename: T, float_format: &str) -> Result<(), MjModelError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        let c_float_format = CString::new(float_format).unwrap();
        unsafe { mj_printFormattedModel(self.ffi(), c_filename.as_ptr(), c_float_format.as_ptr()) }
        Ok(())
    }

    /// Print model to text file.
    ///
    /// # Returns
    /// `Ok(())` if the path is valid UTF-8 and contains no interior `\0` characters.
    /// **Note:** the underlying C function `mj_printModel` returns `void`, so file I/O
    /// errors are not detectable; `Ok(())` does **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn print<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_printModel(self.ffi(), c_filename.as_ptr()) }
        Ok(())
    }

    /// Return size of state specification. The bits of the integer spec correspond to element fields of [`MjtState`](crate::wrappers::mj_data::MjtState).
    pub fn state_size(&self, spec: u32) -> usize {
        unsafe { mj_stateSize(self.ffi(), spec as i32) as usize }
    }

    /// Extract the subset of components specified by `dst_spec` from a state `src`
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::state`]
    /// with components specified by `src_spec`.
    ///
    /// # Panics
    /// - When `src.len()` does not equal the size required by `src_spec`.
    /// - When `dst_spec` is not a subset of `src_spec`.
    ///
    /// Use [`MjModel::try_extract_state`] for a fallible alternative.
    pub fn extract_state(&self, src: &[MjtNum], src_spec: u32, dst_spec: u32) -> Box<[MjtNum]> {
        self.try_extract_state(src, src_spec, dst_spec).unwrap()
    }

    /// Fallible version of [`MjModel::extract_state`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the extracted state.
    /// # Errors
    /// - When `src.len()` does not equal the size required by `src_spec`, [`MjModelError::StateSliceLengthMismatch`] is returned.
    /// - When `dst_spec` is not a subset of `src_spec`, [`MjModelError::SpecNotSubset`] is returned.
    pub fn try_extract_state(&self, src: &[MjtNum], src_spec: u32, dst_spec: u32) -> Result<Box<[MjtNum]>, MjModelError> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(MjModelError::StateSliceLengthMismatch { expected, got: src.len() });
        }

        if (dst_spec & src_spec) != dst_spec {
            return Err(MjModelError::SpecNotSubset);
        }

        let required_size = self.state_size(dst_spec);
        let mut dst = Vec::with_capacity(required_size);

        // SAFETY: all pointer arguments are valid for the duration of this call. mj_extractState
        // writes exactly `required_size` elements into dst; set_len then exposes only those
        // initialized elements.
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
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::state`]
    /// with components specified by `src_spec`.
    ///
    /// # Panics
    /// - When `src.len()` does not equal the size required by `src_spec`.
    /// - When `dst_spec` is not a subset of `src_spec`.
    /// - When `dst` is too small to hold the requested components.
    ///
    /// Use [`MjModel::try_extract_state_into`] for a fallible alternative.
    pub fn extract_state_into(&self, src: &[MjtNum], src_spec: u32, dst: &mut [MjtNum], dst_spec: u32) -> usize {
        self.try_extract_state_into(src, src_spec, dst, dst_spec).unwrap()
    }

    /// Fallible version of [`MjModel::extract_state_into`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the number of elements written to `dst`.
    /// # Errors
    /// - When `src.len()` does not equal the size required by `src_spec`, [`MjModelError::StateSliceLengthMismatch`] is returned.
    /// - When `dst_spec` is not a subset of `src_spec`, [`MjModelError::SpecNotSubset`] is returned.
    /// - When `dst` is too small to hold the requested components, [`MjModelError::BufferTooSmall`] is returned.
    pub fn try_extract_state_into(&self, src: &[MjtNum], src_spec: u32, dst: &mut [MjtNum], dst_spec: u32) -> Result<usize, MjModelError> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(MjModelError::StateSliceLengthMismatch { expected, got: src.len() });
        }

        if (dst_spec & src_spec) != dst_spec {
            return Err(MjModelError::SpecNotSubset);
        }

        let required_size = self.state_size(dst_spec);
        let available_size = dst.len();

        if available_size < required_size  {
            return Err(MjModelError::BufferTooSmall { needed: required_size, available: available_size });
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

    /// Determine type of friction cone. Returns `true` if pyramidal, `false` if elliptic.
    pub fn is_pyramidal(&self) -> bool {
        unsafe { mj_isPyramidal(self.ffi()) == 1 }
    }

    /// Determine type of constraint Jacobian. Returns `true` if sparse, `false` if dense.
    pub fn is_sparse(&self) -> bool {
        unsafe { mj_isSparse(self.ffi()) == 1 }
    }

    /// Determine type of solver. Returns `true` if dual (PGS), `false` if primal (CG or Newton).
    pub fn is_dual(&self) -> bool {
        unsafe { mj_isDual(self.ffi()) == 1 }
    }

    /// Get name of object with the specified [`MjtObj`] type and id, returns `None` if name not found.
    /// Wraps `mj_id2name`.
    /// # Panics
    /// Panics if MuJoCo internally returns a C string that is not valid UTF-8. In practice
    /// MuJoCo names are always valid ASCII (and therefore UTF-8), so this should not occur.
    pub fn id_to_name(&self, type_: MjtObj, id: usize) -> Option<&str> {
        let ptr = unsafe { mj_id2name(self.ffi(), type_ as i32, id as i32) };
        if ptr.is_null() {
            None
        }
        else {
            // SAFETY: ptr was checked non-null above; MuJoCo guarantees the pointed-to string is
            // valid UTF-8 and lives as long as the model.
            let cstr = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
            Some(cstr)
        }
    }

    /// Sum all body masses.
    pub fn totalmass(&self) -> MjtNum {
        unsafe { mj_getTotalmass(self.ffi()) }
    }

    /// Scale body masses and inertias to achieve specified total mass.
    pub fn set_totalmass(&mut self, newmass: MjtNum) {
        unsafe { mj_setTotalmass(self.ffi_mut(), newmass) }
    }

    /// Return the maximum number of contacts that can be generated between two geoms.
    /// 
    /// To pull margin from model, set `has_margin` to [`None`], otherwise pass `true` or `false`
    /// inside [`Some`] (true indicating a present margin). 
    /// 
    /// # Panics
    /// Panics when either `geom1` or `geom2` are greater than [`MjModel::ngeom`].
    /// Use [`MjModel::try_max_contacts`] for a fallible alternative.
    pub fn max_contacts(&self, geom1: usize, geom2: usize, has_margin: Option<bool>) -> u32 {
        self.try_max_contacts(geom1, geom2, has_margin).unwrap()
    }

    /// Fallible version of [`MjModel::max_contacts`].
    /// # Errors
    /// Returns [`MjModelError::InvalidIndex`] when either `geom1` or `geom2` are greater than [`MjModel::ngeom`].
    pub fn try_max_contacts(&self, geom1: usize, geom2: usize, has_margin: Option<bool>) -> Result<u32, MjModelError> {
        let ngeom = self.ngeom() as usize;

        if geom1 >= ngeom {
            return Err(MjModelError::InvalidIndex(geom1, ngeom));
        }

        if geom2 >= ngeom {
            return Err(MjModelError::InvalidIndex(geom2, ngeom));
        }

        Ok(unsafe { mj_maxContact(
            self.ffi(),
            geom1 as i32, geom2 as i32,
            // if Some(...), pass 0 or 1, otherwise pull from model (-1)
            has_margin.map(|m| m as i32).unwrap_or(-1)
        ) as u32 })
    }

    /* FFI */
    /// Returns a reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjModel {
        // SAFETY: self.0 is a valid non-null mjModel pointer for the lifetime of self
        // (struct invariant).
        unsafe { self.0.as_ref() }
    }

    /// Returns a mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// The caller must ensure that any modifications to the underlying struct preserve
    /// the invariants that MuJoCo expects (e.g. do not corrupt computed fields or
    /// break index relationships). Violating these invariants can cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjModel {
        unsafe { self.0.as_mut() }
    }

    /// Returns a direct mutable pointer to the underlying C model struct.
    /// Only for internal use by viewer code that passes the pointer to C++ FFI.
    #[cfg(feature = "cpp-viewer")]
    pub(crate) fn as_raw_ptr(&self) -> *mut mjModel {
        self.0.as_ptr()
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
        [ffi] nflexstiffness: MjtSize; "number of stiffness parameters in all flexes.";
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
        [ffi] nJten: MjtSize; "number of non-zeros in sparse tendon Jacobian matrix.";
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
        (unsafe) body_parentid: &[i32; "id of body's parent"; ffi().nbody],
        (unsafe) body_rootid: &[i32; "ancestor that is direct child of world"; ffi().nbody],
        (unsafe) body_weldid: &[i32; "top ancestor with no dofs to this body"; ffi().nbody],
        (unsafe) body_mocapid: &[i32; "id of mocap data; -1: none"; ffi().nbody],
        (unsafe) body_jntnum: &[i32; "number of joints for this body"; ffi().nbody],
        (unsafe) body_jntadr: &[i32; "start addr of joints; -1: no joints"; ffi().nbody],
        (unsafe) body_dofnum: &[i32; "number of motion degrees of freedom"; ffi().nbody],
        (unsafe) body_dofadr: &[i32; "start addr of dofs; -1: no dofs"; ffi().nbody],
        (unsafe) body_treeid: &[i32; "id of body's kinematic tree; -1: static"; ffi().nbody],
        (unsafe) body_geomnum: &[i32; "number of geoms"; ffi().nbody],
        (unsafe) body_geomadr: &[i32; "start addr of geoms; -1: no geoms"; ffi().nbody],
        body_simple: &[MjtByte; "1: diag M; 2: diag M, sliders only"; ffi().nbody],
        body_sameframe: &[MjtSameFrame [force]; "same frame as inertia"; ffi().nbody],
        body_pos: &[[MjtNum; 3] [force]; "position offset rel. to parent body"; ffi().nbody],
        body_quat: &[[MjtNum; 4] [force]; "orientation offset rel. to parent body"; ffi().nbody],
        body_ipos: &[[MjtNum; 3] [force]; "local position of center of mass"; ffi().nbody],
        body_iquat: &[[MjtNum; 4] [force]; "local orientation of inertia ellipsoid"; ffi().nbody],
        body_mass: &[MjtNum; "mass"; ffi().nbody],
        body_subtreemass: &[MjtNum; "mass of subtree starting at this body"; ffi().nbody],
        body_inertia: &[[MjtNum; 3] [force]; "diagonal inertia in ipos/iquat frame"; ffi().nbody],
        body_invweight0: &[[MjtNum; 2] [force]; "mean inv inert in qpos0 (trn, rot)"; ffi().nbody],
        body_gravcomp: &[MjtNum; "antigravity force, units of body weight"; ffi().nbody],
        body_margin: &[MjtNum; "MAX over all geom margins"; ffi().nbody],
        (unsafe) body_plugin: &[i32; "plugin instance id; -1: not in use"; ffi().nbody],
        body_contype: &[i32; "OR over all geom contypes"; ffi().nbody],
        body_conaffinity: &[i32; "OR over all geom conaffinities"; ffi().nbody],
        (unsafe) body_bvhadr: &[i32; "address of bvh root"; ffi().nbody],
        (unsafe) body_bvhnum: &[i32; "number of bounding volumes"; ffi().nbody],
        bvh_depth: &[i32; "depth in the bounding volume hierarchy"; ffi().nbvh],
        (unsafe) bvh_child: &[[i32; 2] [force]; "left and right children in tree"; ffi().nbvh],
        (unsafe) bvh_nodeid: &[i32; "geom or elem id of node; -1: non-leaf"; ffi().nbvh],
        bvh_aabb: &[[MjtNum; 6] [force]; "local bounding box (center, size)"; ffi().nbvhstatic],
        oct_depth: &[i32; "depth in the octree"; ffi().noct],
        (unsafe) oct_child: &[[i32; 8] [force]; "children of octree node"; ffi().noct],
        oct_aabb: &[[MjtNum; 6] [force]; "octree node bounding box (center, size)"; ffi().noct],
        oct_coeff: &[[MjtNum; 8] [force]; "octree interpolation coefficients"; ffi().noct],
        (unsafe) jnt_type: &[MjtJoint [force]; "type of joint"; ffi().njnt],
        (unsafe) jnt_qposadr: &[i32; "start addr in 'qpos' for joint's data"; ffi().njnt],
        (unsafe) jnt_dofadr: &[i32; "start addr in 'qvel' for joint's data"; ffi().njnt],
        (unsafe) jnt_bodyid: &[i32; "id of joint's body"; ffi().njnt],
        (unsafe) jnt_actuatorid: &[i32; "actuator contributing damping / armature"; ffi().njnt],
        jnt_group: &[i32; "group for visibility"; ffi().njnt],
        jnt_limited: &[bool [force]; "does joint have limits"; ffi().njnt],
        jnt_actfrclimited: &[bool [force]; "does joint have actuator force limits"; ffi().njnt],
        jnt_actgravcomp: &[bool [force]; "is gravcomp force applied via actuators"; ffi().njnt],
        jnt_solref: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference: limit"; ffi().njnt],
        jnt_solimp: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance: limit"; ffi().njnt],
        jnt_pos: &[[MjtNum; 3] [force]; "local anchor position"; ffi().njnt],
        jnt_axis: &[[MjtNum; 3] [force]; "local joint axis"; ffi().njnt],
        jnt_stiffness: &[MjtNum; "stiffness coefficient"; ffi().njnt],
        jnt_stiffnesspoly: &[[MjtNum; mjNPOLY as usize] [force]; "high-order stiffness coefficients"; ffi().njnt],
        jnt_range: &[[MjtNum; 2] [force]; "joint limits"; ffi().njnt],
        jnt_actfrcrange: &[[MjtNum; 2] [force]; "range of total actuator force"; ffi().njnt],
        jnt_margin: &[MjtNum; "min distance for limit detection"; ffi().njnt],
        (unsafe) dof_bodyid: &[i32; "id of dof's body"; ffi().nv],
        (unsafe) dof_jntid: &[i32; "id of dof's joint"; ffi().nv],
        (unsafe) dof_parentid: &[i32; "id of dof's parent; -1: none"; ffi().nv],
        (unsafe) dof_treeid: &[i32; "id of dof's kinematic tree"; ffi().nv],
        (unsafe) dof_Madr: &[i32; "dof address in M-diagonal"; ffi().nv],
        dof_simplenum: &[i32; "number of consecutive simple dofs"; ffi().nv],
        dof_solref: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference:frictionloss"; ffi().nv],
        dof_solimp: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance:frictionloss"; ffi().nv],
        dof_frictionloss: &[MjtNum; "dof friction loss"; ffi().nv],
        dof_armature: &[MjtNum; "dof armature inertia/mass"; ffi().nv],
        dof_damping: &[MjtNum; "damping coefficient"; ffi().nv],
        dof_dampingpoly: &[[MjtNum; mjNPOLY as usize] [force]; "high-order damping coefficients"; ffi().nv],
        dof_invweight0: &[MjtNum; "diag. inverse inertia in qpos0"; ffi().nv],
        dof_M0: &[MjtNum; "diag. inertia in qpos0"; ffi().nv],
        dof_length: &[MjtNum; "linear: 1; angular: approx. length scale"; ffi().nv],
        (unsafe) tree_bodyadr: &[i32; "start addr of bodies"; ffi().ntree],
        (unsafe) tree_bodynum: &[i32; "number of bodies in tree"; ffi().ntree],
        (unsafe) tree_dofadr: &[i32; "start addr of dofs"; ffi().ntree],
        (unsafe) tree_dofnum: &[i32; "number of dofs in tree"; ffi().ntree],
        tree_sleep_policy: &[MjtSleepPolicy [force]; "sleep policy"; ffi().ntree],
        (unsafe) geom_type: &[MjtGeom [force]; "geometric type"; ffi().ngeom],
        geom_contype: &[i32; "geom contact type"; ffi().ngeom],
        geom_conaffinity: &[i32; "geom contact affinity"; ffi().ngeom],
        (unsafe) geom_condim: &[i32; "contact dimensionality (1, 3, 4, 6)"; ffi().ngeom],
        (unsafe) geom_bodyid: &[i32; "id of geom's body"; ffi().ngeom],
        (unsafe) geom_dataid: &[i32; "id of geom's mesh/hfield; -1: none"; ffi().ngeom],
        (unsafe) geom_matid: &[i32; "material id for rendering; -1: none"; ffi().ngeom],
        geom_group: &[i32; "group for visibility"; ffi().ngeom],
        geom_priority: &[i32; "geom contact priority"; ffi().ngeom],
        (unsafe) geom_plugin: &[i32; "plugin instance id; -1: not in use"; ffi().ngeom],
        geom_sameframe: &[MjtSameFrame [force]; "same frame as body"; ffi().ngeom],
        geom_solmix: &[MjtNum; "mixing coef for solref/imp in geom pair"; ffi().ngeom],
        geom_solref: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference: contact"; ffi().ngeom],
        geom_solimp: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance: contact"; ffi().ngeom],
        geom_size: &[[MjtNum; 3] [force]; "geom-specific size parameters"; ffi().ngeom],
        geom_aabb: &[[MjtNum; 6] [force]; "bounding box, (center, size)"; ffi().ngeom],
        geom_rbound: &[MjtNum; "radius of bounding sphere"; ffi().ngeom],
        geom_pos: &[[MjtNum; 3] [force]; "local position offset rel. to body"; ffi().ngeom],
        geom_quat: &[[MjtNum; 4] [force]; "local orientation offset rel. to body"; ffi().ngeom],
        geom_friction: &[[MjtNum; 3] [force]; "friction for (slide, spin, roll)"; ffi().ngeom],
        geom_margin: &[MjtNum; "detect contact if dist<margin"; ffi().ngeom],
        geom_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().ngeom],
        geom_fluid: &[[MjtNum; mjNFLUID as usize] [force]; "fluid interaction parameters"; ffi().ngeom],
        geom_rgba: &[[f32; 4] [force]; "rgba when material is omitted"; ffi().ngeom],
        site_type: &[MjtGeom [force]; "geom type for rendering"; ffi().nsite],
        (unsafe) site_bodyid: &[i32; "id of site's body"; ffi().nsite],
        (unsafe) site_matid: &[i32; "material id for rendering; -1: none"; ffi().nsite],
        site_group: &[i32; "group for visibility"; ffi().nsite],
        site_sameframe: &[MjtSameFrame [force]; "same frame as body"; ffi().nsite],
        site_size: &[[MjtNum; 3] [force]; "geom size for rendering"; ffi().nsite],
        site_pos: &[[MjtNum; 3] [force]; "local position offset rel. to body"; ffi().nsite],
        site_quat: &[[MjtNum; 4] [force]; "local orientation offset rel. to body"; ffi().nsite],
        site_rgba: &[[f32; 4] [force]; "rgba when material is omitted"; ffi().nsite],
        cam_mode: &[MjtCamLight [force]; "camera tracking mode"; ffi().ncam],
        (unsafe) cam_bodyid: &[i32; "id of camera's body"; ffi().ncam],
        (unsafe) cam_targetbodyid: &[i32; "id of targeted body; -1: none"; ffi().ncam],
        cam_pos: &[[MjtNum; 3] [force]; "position rel. to body frame"; ffi().ncam],
        cam_quat: &[[MjtNum; 4] [force]; "orientation rel. to body frame"; ffi().ncam],
        cam_poscom0: &[[MjtNum; 3] [force]; "global position rel. to sub-com in qpos0"; ffi().ncam],
        cam_pos0: &[[MjtNum; 3] [force]; "global position rel. to body in qpos0"; ffi().ncam],
        cam_mat0: &[[MjtNum; 9] [force]; "global orientation in qpos0"; ffi().ncam],
        cam_projection: &[MjtProjection [force]; "projection type"; ffi().ncam],
        cam_fovy: &[MjtNum; "y field-of-view (ortho ? len : deg)"; ffi().ncam],
        cam_ipd: &[MjtNum; "inter-pupillary distance"; ffi().ncam],
        (unsafe) cam_resolution: &[[i32; 2] [force]; "resolution: pixels [width, height]"; ffi().ncam],
        cam_output: &[i32; "output types (MjtCamOutBit bit flags)"; ffi().ncam],
        cam_sensorsize: &[[f32; 2] [force]; "sensor size: length [width, height]"; ffi().ncam],
        cam_intrinsic: &[[f32; 4] [force]; "[focal length; principal point]"; ffi().ncam],
        light_mode: &[MjtCamLight [force]; "light tracking mode"; ffi().nlight],
        (unsafe) light_bodyid: &[i32; "id of light's body"; ffi().nlight],
        (unsafe) light_targetbodyid: &[i32; "id of targeted body; -1: none"; ffi().nlight],
        light_type: &[MjtLightType [force]; "spot, directional, etc."; ffi().nlight],
        (unsafe) light_texid: &[i32; "texture id for image lights"; ffi().nlight],
        light_castshadow: &[bool [force]; "does light cast shadows"; ffi().nlight],
        light_bulbradius: &[f32; "light radius for soft shadows"; ffi().nlight],
        light_intensity: &[f32; "intensity, in candela"; ffi().nlight],
        light_range: &[f32; "range of effectiveness"; ffi().nlight],
        light_active: &[bool [force]; "is light on"; ffi().nlight],
        light_pos: &[[MjtNum; 3] [force]; "position rel. to body frame"; ffi().nlight],
        light_dir: &[[MjtNum; 3] [force]; "direction rel. to body frame"; ffi().nlight],
        light_poscom0: &[[MjtNum; 3] [force]; "global position rel. to sub-com in qpos0"; ffi().nlight],
        light_pos0: &[[MjtNum; 3] [force]; "global position rel. to body in qpos0"; ffi().nlight],
        light_dir0: &[[MjtNum; 3] [force]; "global direction in qpos0"; ffi().nlight],
        light_attenuation: &[[f32; 3] [force]; "OpenGL attenuation (quadratic model)"; ffi().nlight],
        light_cutoff: &[f32; "OpenGL cutoff"; ffi().nlight],
        light_exponent: &[f32; "OpenGL exponent"; ffi().nlight],
        light_ambient: &[[f32; 3] [force]; "ambient rgb (alpha=1)"; ffi().nlight],
        light_diffuse: &[[f32; 3] [force]; "diffuse rgb (alpha=1)"; ffi().nlight],
        light_specular: &[[f32; 3] [force]; "specular rgb (alpha=1)"; ffi().nlight],
        flex_contype: &[i32; "flex contact type"; ffi().nflex],
        flex_conaffinity: &[i32; "flex contact affinity"; ffi().nflex],
        (unsafe) flex_condim: &[i32; "contact dimensionality (1, 3, 4, 6)"; ffi().nflex],
        flex_priority: &[i32; "flex contact priority"; ffi().nflex],
        flex_solmix: &[MjtNum; "mix coef for solref/imp in contact pair"; ffi().nflex],
        flex_solref: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference: contact"; ffi().nflex],
        flex_solimp: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance: contact"; ffi().nflex],
        flex_friction: &[[MjtNum; 3] [force]; "friction for (slide, spin, roll)"; ffi().nflex],
        flex_margin: &[MjtNum; "detect contact if dist<margin"; ffi().nflex],
        flex_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().nflex],
        flex_internal: &[bool [force]; "internal flex collision enabled"; ffi().nflex],
        flex_selfcollide: &[MjtFlexSelf [force]; "self collision mode"; ffi().nflex],
        flex_activelayers: &[i32; "number of active element layers, 3D only"; ffi().nflex],
        flex_passive: &[i32; "passive collisions enabled"; ffi().nflex],
        (unsafe) flex_dim: &[i32; "1: lines, 2: triangles, 3: tetrahedra"; ffi().nflex],
        (unsafe) flex_matid: &[i32; "material id for rendering"; ffi().nflex],
        flex_group: &[i32; "group for visibility"; ffi().nflex],
        (unsafe) flex_interp: &[i32; "interpolation (0: vertex, 1: nodes)"; ffi().nflex],
        (unsafe) flex_bandwidth: &[i32; "precomputed solver bandwidth"; ffi().nflex],
        (unsafe) flex_cellnum: &[[i32; 3] [force]; "finite cell num per dimension"; ffi().nflex],
        (unsafe) flex_nodeadr: &[i32; "first node address"; ffi().nflex],
        (unsafe) flex_nodenum: &[i32; "number of nodes"; ffi().nflex],
        (unsafe) flex_vertadr: &[i32; "first vertex address"; ffi().nflex],
        (unsafe) flex_vertnum: &[i32; "number of vertices"; ffi().nflex],
        (unsafe) flex_edgeadr: &[i32; "first edge address"; ffi().nflex],
        (unsafe) flex_edgenum: &[i32; "number of edges"; ffi().nflex],
        (unsafe) flex_elemadr: &[i32; "first element address"; ffi().nflex],
        (unsafe) flex_elemnum: &[i32; "number of elements"; ffi().nflex],
        (unsafe) flex_elemdataadr: &[i32; "first element vertex id address"; ffi().nflex],
        (unsafe) flex_stiffnessadr: &[i32; "stiffness matrix address"; ffi().nflex],
        (unsafe) flex_elemedgeadr: &[i32; "first element edge id address"; ffi().nflex],
        (unsafe) flex_shellnum: &[i32; "number of shells"; ffi().nflex],
        (unsafe) flex_shelldataadr: &[i32; "first shell data address"; ffi().nflex],
        (unsafe) flex_evpairadr: &[i32; "first evpair address"; ffi().nflex],
        (unsafe) flex_evpairnum: &[i32; "number of evpairs"; ffi().nflex],
        (unsafe) flex_texcoordadr: &[i32; "address in flex_texcoord; -1: none"; ffi().nflex],
        (unsafe) flex_nodebodyid: &[i32; "node body ids"; ffi().nflexnode],
        (unsafe) flex_vertbodyid: &[i32; "vertex body ids"; ffi().nflexvert],
        (unsafe) flex_vertedgeadr: &[i32; "first edge address"; ffi().nflexvert],
        (unsafe) flex_vertedgenum: &[i32; "number of edges"; ffi().nflexvert],
        (unsafe) flex_vertedge: &[[i32; 2] [force]; "edge indices"; ffi().nflexedge],
        (unsafe) flex_edge: &[[i32; 2] [force]; "edge vertex ids (2 per edge)"; ffi().nflexedge],
        (unsafe) flex_edgeflap: &[[i32; 2] [force]; "adjacent vertex ids (dim=2 only)"; ffi().nflexedge],
        (unsafe) flex_elem: &[i32; "element vertex ids (dim+1 per elem)"; ffi().nflexelemdata],
        (unsafe) flex_elemtexcoord: &[i32; "element texture coordinates (dim+1)"; ffi().nflexelemdata],
        (unsafe) flex_elemedge: &[i32; "element edge ids"; ffi().nflexelemedge],
        flex_elemlayer: &[i32; "element distance from surface, 3D only"; ffi().nflexelem],
        (unsafe) flex_shell: &[i32; "shell fragment vertex ids (dim per frag)"; ffi().nflexshelldata],
        (unsafe) flex_evpair: &[[i32; 2] [force]; "(element, vertex) collision pairs"; ffi().nflexevpair],
        flex_vert: &[[MjtNum; 3] [force]; "vertex positions in local body frames"; ffi().nflexvert],
        flex_vert0: &[[MjtNum; 3] [force]; "vertex positions in qpos0 on [0, 1]^d"; ffi().nflexvert],
        flex_vertmetric: &[[MjtNum; 4] [force]; "inverse of reference shape matrix"; ffi().nflexvert],
        flex_node: &[[MjtNum; 3] [force]; "node positions in local body frames"; ffi().nflexnode],
        flex_node0: &[[MjtNum; 3] [force]; "Cartesian node positions in qpos0"; ffi().nflexnode],
        flexedge_length0: &[MjtNum; "edge lengths in qpos0"; ffi().nflexedge],
        flexedge_invweight0: &[MjtNum; "edge inv. weight in qpos0"; ffi().nflexedge],
        flex_radius: &[MjtNum; "radius around primitive element"; ffi().nflex],
        flex_size: &[[MjtNum; 3] [force]; "vertex bounding box half sizes in qpos0"; ffi().nflex],
        flex_stiffness: &[MjtNum; "finite element stiffness matrix"; ffi().nflexstiffness],
        flex_bending: &[[MjtNum; 17] [force]; "bending stiffness"; ffi().nflexedge],
        flex_damping: &[MjtNum; "Rayleigh's damping coefficient"; ffi().nflex],
        flex_edgestiffness: &[MjtNum; "edge stiffness"; ffi().nflex],
        flex_edgedamping: &[MjtNum; "edge damping"; ffi().nflex],
        flex_edgeequality: &[i32; "0: none, 1: edges, 2: vertices, 3: strain"; ffi().nflex],
        flex_rigid: &[bool [force]; "are all vertices in the same body"; ffi().nflex],
        flexedge_rigid: &[bool [force]; "are both edge vertices in same body"; ffi().nflexedge],
        flex_centered: &[bool [force]; "are all vertex coordinates (0,0,0)"; ffi().nflex],
        flex_flatskin: &[bool [force]; "render flex skin with flat shading"; ffi().nflex],
        (unsafe) flex_bvhadr: &[i32; "address of bvh root; -1: no bvh"; ffi().nflex],
        (unsafe) flex_bvhnum: &[i32; "number of bounding volumes"; ffi().nflex],
        (unsafe) flexedge_J_rownnz: &[i32; "number of non-zeros in Jacobian row"; ffi().nflexedge],
        (unsafe) flexedge_J_rowadr: &[i32; "row start address in colind array"; ffi().nflexedge],
        (unsafe) flexedge_J_colind: &[i32; "column indices in sparse Jacobian"; ffi().nJfe],
        (unsafe) flexvert_J_rownnz: &[[i32; 2] [force]; "number of non-zeros in Jacobian row"; ffi().nflexvert],
        (unsafe) flexvert_J_rowadr: &[[i32; 2] [force]; "row start address in colind array"; ffi().nflexvert],
        (unsafe) flexvert_J_colind: &[[i32; 2] [force]; "column indices in sparse Jacobian"; ffi().nJfv],
        flex_rgba: &[[f32; 4] [force]; "rgba when material is omitted"; ffi().nflex],
        flex_texcoord: &[[f32; 2] [force]; "vertex texture coordinates"; ffi().nflextexcoord],
        (unsafe) mesh_vertadr: &[i32; "first vertex address"; ffi().nmesh],
        (unsafe) mesh_vertnum: &[i32; "number of vertices"; ffi().nmesh],
        (unsafe) mesh_faceadr: &[i32; "first face address"; ffi().nmesh],
        (unsafe) mesh_facenum: &[i32; "number of faces"; ffi().nmesh],
        (unsafe) mesh_bvhadr: &[i32; "address of bvh root"; ffi().nmesh],
        (unsafe) mesh_bvhnum: &[i32; "number of bvh"; ffi().nmesh],
        (unsafe) mesh_octadr: &[i32; "address of octree root"; ffi().nmesh],
        (unsafe) mesh_octnum: &[i32; "number of octree nodes"; ffi().nmesh],
        (unsafe) mesh_normaladr: &[i32; "first normal address"; ffi().nmesh],
        (unsafe) mesh_normalnum: &[i32; "number of normals"; ffi().nmesh],
        (unsafe) mesh_texcoordadr: &[i32; "texcoord data address; -1: no texcoord"; ffi().nmesh],
        (unsafe) mesh_texcoordnum: &[i32; "number of texcoord"; ffi().nmesh],
        (unsafe) mesh_graphadr: &[i32; "graph data address; -1: no graph"; ffi().nmesh],
        mesh_vert: &[[f32; 3] [force]; "vertex positions for all meshes"; ffi().nmeshvert],
        mesh_normal: &[[f32; 3] [force]; "normals for all meshes"; ffi().nmeshnormal],
        mesh_texcoord: &[[f32; 2] [force]; "vertex texcoords for all meshes"; ffi().nmeshtexcoord],
        (unsafe) mesh_face: &[[i32; 3] [force]; "vertex face data"; ffi().nmeshface],
        (unsafe) mesh_facenormal: &[[i32; 3] [force]; "normal face data"; ffi().nmeshface],
        (unsafe) mesh_facetexcoord: &[[i32; 3] [force]; "texture face data"; ffi().nmeshface],
        (unsafe) mesh_graph: &[i32; "convex graph data"; ffi().nmeshgraph],
        mesh_scale: &[[MjtNum; 3] [force]; "scaling applied to asset vertices"; ffi().nmesh],
        mesh_pos: &[[MjtNum; 3] [force]; "translation applied to asset vertices"; ffi().nmesh],
        mesh_quat: &[[MjtNum; 4] [force]; "rotation applied to asset vertices"; ffi().nmesh],
        (unsafe) mesh_pathadr: &[i32; "address of asset path for mesh; -1: none"; ffi().nmesh],
        (unsafe) mesh_polynum: &[i32; "number of polygons per mesh"; ffi().nmesh],
        (unsafe) mesh_polyadr: &[i32; "first polygon address per mesh"; ffi().nmesh],
        mesh_polynormal: &[[MjtNum; 3] [force]; "all polygon normals"; ffi().nmeshpoly],
        (unsafe) mesh_polyvertadr: &[i32; "polygon vertex start address"; ffi().nmeshpoly],
        (unsafe) mesh_polyvertnum: &[i32; "number of vertices per polygon"; ffi().nmeshpoly],
        (unsafe) mesh_polyvert: &[i32; "all polygon vertices"; ffi().nmeshpolyvert],
        (unsafe) mesh_polymapadr: &[i32; "first polygon address per vertex"; ffi().nmeshvert],
        (unsafe) mesh_polymapnum: &[i32; "number of polygons per vertex"; ffi().nmeshvert],
        (unsafe) mesh_polymap: &[i32; "vertex to polygon map"; ffi().nmeshpolymap],
        (unsafe) skin_matid: &[i32; "skin material id; -1: none"; ffi().nskin],
        skin_group: &[i32; "group for visibility"; ffi().nskin],
        skin_rgba: &[[f32; 4] [force]; "skin rgba"; ffi().nskin],
        skin_inflate: &[f32; "inflate skin in normal direction"; ffi().nskin],
        (unsafe) skin_vertadr: &[i32; "first vertex address"; ffi().nskin],
        (unsafe) skin_vertnum: &[i32; "number of vertices"; ffi().nskin],
        (unsafe) skin_texcoordadr: &[i32; "texcoord data address; -1: no texcoord"; ffi().nskin],
        (unsafe) skin_faceadr: &[i32; "first face address"; ffi().nskin],
        (unsafe) skin_facenum: &[i32; "number of faces"; ffi().nskin],
        (unsafe) skin_boneadr: &[i32; "first bone in skin"; ffi().nskin],
        (unsafe) skin_bonenum: &[i32; "number of bones in skin"; ffi().nskin],
        skin_vert: &[[f32; 3] [force]; "vertex positions for all skin meshes"; ffi().nskinvert],
        skin_texcoord: &[[f32; 2] [force]; "vertex texcoords for all skin meshes"; ffi().nskintexvert],
        (unsafe) skin_face: &[[i32; 3] [force]; "triangle faces for all skin meshes"; ffi().nskinface],
        (unsafe) skin_bonevertadr: &[i32; "first vertex in each bone"; ffi().nskinbone],
        (unsafe) skin_bonevertnum: &[i32; "number of vertices in each bone"; ffi().nskinbone],
        skin_bonebindpos: &[[f32; 3] [force]; "bind pos of each bone"; ffi().nskinbone],
        skin_bonebindquat: &[[f32; 4] [force]; "bind quat of each bone"; ffi().nskinbone],
        (unsafe) skin_bonebodyid: &[i32; "body id of each bone"; ffi().nskinbone],
        (unsafe) skin_bonevertid: &[i32; "mesh ids of vertices in each bone"; ffi().nskinbonevert],
        skin_bonevertweight: &[f32; "weights of vertices in each bone"; ffi().nskinbonevert],
        (unsafe) skin_pathadr: &[i32; "address of asset path for skin; -1: none"; ffi().nskin],
        hfield_size: &[[MjtNum; 4] [force]; "(x, y, z_top, z_bottom)"; ffi().nhfield],
        (unsafe) hfield_nrow: &[i32; "number of rows in grid"; ffi().nhfield],
        (unsafe) hfield_ncol: &[i32; "number of columns in grid"; ffi().nhfield],
        (unsafe) hfield_adr: &[i32; "address in hfield_data"; ffi().nhfield],
        hfield_data: &[f32; "elevation data"; ffi().nhfielddata],
        (unsafe) hfield_pathadr: &[i32; "address of hfield asset path; -1: none"; ffi().nhfield],
        tex_type: &[MjtTexture [force]; "texture type"; ffi().ntex],
        tex_colorspace: &[MjtColorSpace [force]; "texture colorspace"; ffi().ntex],
        (unsafe) tex_height: &[i32; "number of rows in texture image"; ffi().ntex],
        (unsafe) tex_width: &[i32; "number of columns in texture image"; ffi().ntex],
        (unsafe) tex_nchannel: &[i32; "number of channels in texture image"; ffi().ntex],
        (unsafe) tex_adr: &[MjtSize; "start address in tex_data"; ffi().ntex],
        tex_data: &[MjtByte; "pixel values"; ffi().ntexdata],
        (unsafe) tex_pathadr: &[i32; "address of texture asset path; -1: none"; ffi().ntex],
        (unsafe) mat_texid: &[[i32; MjtTextureRole::mjNTEXROLE as usize] [force]; "indices of textures; -1: none"; ffi().nmat],
        mat_texuniform: &[bool [force]; "make texture cube uniform"; ffi().nmat],
        mat_texrepeat: &[[f32; 2] [force]; "texture repetition for 2d mapping"; ffi().nmat],
        mat_emission: &[f32; "emission (x rgb)"; ffi().nmat],
        mat_specular: &[f32; "specular (x white)"; ffi().nmat],
        mat_shininess: &[f32; "shininess coef"; ffi().nmat],
        mat_reflectance: &[f32; "reflectance (0: disable)"; ffi().nmat],
        mat_metallic: &[f32; "metallic coef"; ffi().nmat],
        mat_roughness: &[f32; "roughness coef"; ffi().nmat],
        mat_rgba: &[[f32; 4] [force]; "rgba"; ffi().nmat],
        (unsafe) pair_dim: &[i32; "contact dimensionality"; ffi().npair],
        (unsafe) pair_geom1: &[i32; "id of geom1"; ffi().npair],
        (unsafe) pair_geom2: &[i32; "id of geom2"; ffi().npair],
        pair_signature: &[i32; "body1 << 16 + body2"; ffi().npair],
        pair_solref: &[[MjtNum; mjNREF as usize] [force]; "solver reference: contact normal"; ffi().npair],
        pair_solreffriction: &[[MjtNum; mjNREF as usize] [force]; "solver reference: contact friction"; ffi().npair],
        pair_solimp: &[[MjtNum; mjNIMP as usize] [force]; "solver impedance: contact"; ffi().npair],
        pair_margin: &[MjtNum; "detect contact if dist<margin"; ffi().npair],
        pair_gap: &[MjtNum; "include in solver if dist<margin-gap"; ffi().npair],
        pair_friction: &[[MjtNum; 5] [force]; "tangent1, 2, spin, roll1, 2"; ffi().npair],
        exclude_signature: &[i32; "body1 << 16 + body2"; ffi().nexclude],
        (unsafe) eq_type: &[MjtEq [force]; "constraint type"; ffi().neq],
        (unsafe) eq_obj1id: &[i32; "id of object 1"; ffi().neq],
        (unsafe) eq_obj2id: &[i32; "id of object 2"; ffi().neq],
        (unsafe) eq_objtype: &[MjtObj [force]; "type of both objects"; ffi().neq],
        eq_active0: &[bool [force]; "initial enable/disable constraint state"; ffi().neq],
        eq_solref: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference"; ffi().neq],
        eq_solimp: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance"; ffi().neq],
        eq_data: &[[MjtNum; mjNEQDATA as usize] [force]; "numeric data for constraint"; ffi().neq],
        (unsafe) tendon_adr: &[i32; "address of first object in tendon's path"; ffi().ntendon],
        (unsafe) tendon_num: &[i32; "number of objects in tendon's path"; ffi().ntendon],
        (unsafe) tendon_matid: &[i32; "material id for rendering"; ffi().ntendon],
        (unsafe) tendon_actuatorid: &[i32; "actuator contributing damping / armature"; ffi().ntendon],
        tendon_group: &[i32; "group for visibility"; ffi().ntendon],
        tendon_treenum: &[i32; "number of trees along tendon's path"; ffi().ntendon],
        (unsafe) tendon_treeid: &[[i32; 2] [force]; "first two trees along tendon's path"; ffi().ntendon],
        (unsafe) ten_J_rownnz: &[i32; "number of non-zeros in Jacobian row"; ffi().ntendon],
        (unsafe) ten_J_rowadr: &[i32; "row start address in colind array"; ffi().ntendon],
        (unsafe) ten_J_colind: &[i32; "column indices in sparse Jacobian"; ffi().nJten],
        tendon_limited: &[bool [force]; "does tendon have length limits"; ffi().ntendon],
        tendon_actfrclimited: &[bool [force]; "does tendon have actuator force limits"; ffi().ntendon],
        tendon_width: &[MjtNum; "width for rendering"; ffi().ntendon],
        tendon_solref_lim: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference: limit"; ffi().ntendon],
        tendon_solimp_lim: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance: limit"; ffi().ntendon],
        tendon_solref_fri: &[[MjtNum; mjNREF as usize] [force]; "constraint solver reference: friction"; ffi().ntendon],
        tendon_solimp_fri: &[[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance: friction"; ffi().ntendon],
        tendon_range: &[[MjtNum; 2] [force]; "tendon length limits"; ffi().ntendon],
        tendon_actfrcrange: &[[MjtNum; 2] [force]; "range of total actuator force"; ffi().ntendon],
        tendon_margin: &[MjtNum; "min distance for limit detection"; ffi().ntendon],
        tendon_stiffness: &[MjtNum; "stiffness coefficient"; ffi().ntendon],
        tendon_stiffnesspoly: &[[MjtNum; mjNPOLY as usize] [force]; "high-order stiffness coefficients"; ffi().ntendon],
        tendon_damping: &[MjtNum; "damping coefficient"; ffi().ntendon],
        tendon_dampingpoly: &[[MjtNum; mjNPOLY as usize] [force]; "high-order damping coefficients"; ffi().ntendon],
        tendon_armature: &[MjtNum; "inertia associated with tendon velocity"; ffi().ntendon],
        tendon_frictionloss: &[MjtNum; "loss due to friction"; ffi().ntendon],
        tendon_lengthspring: &[[MjtNum; 2] [force]; "spring resting length range"; ffi().ntendon],
        tendon_length0: &[MjtNum; "tendon length in qpos0"; ffi().ntendon],
        tendon_invweight0: &[MjtNum; "inv. weight in qpos0"; ffi().ntendon],
        tendon_rgba: &[[f32; 4] [force]; "rgba when material is omitted"; ffi().ntendon],
        (unsafe) wrap_type: &[MjtWrap [force]; "wrap object type"; ffi().nwrap],
        (unsafe) wrap_objid: &[i32; "object id: geom, site, joint"; ffi().nwrap],
        (unsafe) wrap_prm: &[MjtNum; "divisor, joint coef, or site id"; ffi().nwrap],
        (unsafe) actuator_trntype: &[MjtTrn [force]; "transmission type"; ffi().nu],
        (unsafe) actuator_dyntype: &[MjtDyn [force]; "dynamics type"; ffi().nu],
        actuator_gaintype: &[MjtGain [force]; "gain type"; ffi().nu],
        actuator_biastype: &[MjtBias [force]; "bias type"; ffi().nu],
        (unsafe) actuator_trnid: &[[i32; 2] [force]; "transmission id: joint, tendon, site"; ffi().nu],
        (unsafe) actuator_actadr: &[i32; "first activation address; -1: stateless"; ffi().nu],
        (unsafe) actuator_actnum: &[i32; "number of activation variables"; ffi().nu],
        actuator_group: &[i32; "group for visibility"; ffi().nu],
        (unsafe) actuator_history: &[[i32; 2] [force]; "history buffer: [nsample, interp]"; ffi().nu],
        (unsafe) actuator_historyadr: &[i32; "address in history buffer; -1: none"; ffi().nu],
        actuator_delay: &[MjtNum; "delay time in seconds; 0: no delay"; ffi().nu],
        actuator_ctrllimited: &[bool [force]; "is control limited"; ffi().nu],
        actuator_forcelimited: &[bool [force]; "is force limited"; ffi().nu],
        actuator_actlimited: &[bool [force]; "is activation limited"; ffi().nu],
        actuator_dynprm: &[[MjtNum; mjNDYN as usize] [force]; "dynamics parameters"; ffi().nu],
        actuator_gainprm: &[[MjtNum; mjNGAIN as usize] [force]; "gain parameters"; ffi().nu],
        actuator_biasprm: &[[MjtNum; mjNBIAS as usize] [force]; "bias parameters"; ffi().nu],
        actuator_actearly: &[bool [force]; "step activation before force"; ffi().nu],
        actuator_ctrlrange: &[[MjtNum; 2] [force]; "range of controls"; ffi().nu],
        actuator_forcerange: &[[MjtNum; 2] [force]; "range of forces"; ffi().nu],
        actuator_actrange: &[[MjtNum; 2] [force]; "range of activations"; ffi().nu],
        actuator_damping: &[MjtNum; "linear damping coefficient"; ffi().nu],
        actuator_dampingpoly: &[[MjtNum; mjNPOLY as usize] [force]; "high-order damping coefficients"; ffi().nu],
        actuator_armature: &[MjtNum; "armature added to target"; ffi().nu],
        actuator_gear: &[[MjtNum; 6] [force]; "scale length and transmitted force"; ffi().nu],
        actuator_cranklength: &[MjtNum; "crank length for slider-crank"; ffi().nu],
        actuator_acc0: &[MjtNum; "acceleration from unit force in qpos0"; ffi().nu],
        actuator_length0: &[MjtNum; "actuator length in qpos0"; ffi().nu],
        actuator_lengthrange: &[[MjtNum; 2] [force]; "feasible actuator length range"; ffi().nu],
        (unsafe) actuator_plugin: &[i32; "plugin instance id; -1: not a plugin"; ffi().nu],
        (unsafe) sensor_type: &[MjtSensor [force]; "sensor type"; ffi().nsensor],
        sensor_datatype: &[MjtDataType [force]; "numeric data type"; ffi().nsensor],
        sensor_needstage: &[MjtStage [force]; "required compute stage"; ffi().nsensor],
        (unsafe) sensor_objtype: &[MjtObj [force]; "type of sensorized object"; ffi().nsensor],
        (unsafe) sensor_objid: &[i32; "id of sensorized object"; ffi().nsensor],
        (unsafe) sensor_reftype: &[MjtObj [force]; "type of reference frame"; ffi().nsensor],
        (unsafe) sensor_refid: &[i32; "id of reference frame; -1: global frame"; ffi().nsensor],
        sensor_intprm: &[[i32; mjNSENS as usize] [force]; "sensor parameters"; ffi().nsensor],
        (unsafe) sensor_dim: &[i32; "number of scalar outputs"; ffi().nsensor],
        (unsafe) sensor_adr: &[i32; "address in sensor array"; ffi().nsensor],
        sensor_cutoff: &[MjtNum; "cutoff for real and positive; 0: ignore"; ffi().nsensor],
        sensor_noise: &[MjtNum; "noise standard deviation"; ffi().nsensor],
        (unsafe) sensor_history: &[[i32; 2] [force]; "history buffer: [nsample, interp]"; ffi().nsensor],
        (unsafe) sensor_historyadr: &[i32; "address in history buffer; -1: none"; ffi().nsensor],
        sensor_delay: &[MjtNum; "delay time in seconds; 0: no delay"; ffi().nsensor],
        sensor_interval: &[[MjtNum; 2] [force]; "interval: [period, phase] in seconds"; ffi().nsensor],
        (unsafe) sensor_plugin: &[i32; "plugin instance id; -1: not a plugin"; ffi().nsensor],
        (unsafe) plugin: &[i32; "globally registered plugin slot number"; ffi().nplugin],
        (unsafe) plugin_stateadr: &[i32; "address in the plugin state array"; ffi().nplugin],
        (unsafe) plugin_statenum: &[i32; "number of states in the plugin instance"; ffi().nplugin],
        (unsafe) plugin_attr: &[c_char; "config attributes of plugin instances"; ffi().npluginattr],
        (unsafe) plugin_attradr: &[i32; "address to each instance's config attrib"; ffi().nplugin],
        (unsafe) numeric_adr: &[i32; "address of field in numeric_data"; ffi().nnumeric],
        (unsafe) numeric_size: &[i32; "size of numeric field"; ffi().nnumeric],
        numeric_data: &[MjtNum; "array of all numeric fields"; ffi().nnumericdata],
        (unsafe) text_adr: &[i32; "address of text in text_data"; ffi().ntext],
        (unsafe) text_size: &[i32; "size of text field (strlen+1)"; ffi().ntext],
        (unsafe) text_data: &[c_char; "array of all text fields (0-terminated)"; ffi().ntextdata],
        (unsafe) tuple_adr: &[i32; "address of text in text_data"; ffi().ntuple],
        (unsafe) tuple_size: &[i32; "number of objects in tuple"; ffi().ntuple],
        tuple_objtype: &[MjtObj [force]; "array of object types in all tuples"; ffi().ntupledata],
        (unsafe) tuple_objid: &[i32; "array of object ids in all tuples"; ffi().ntupledata],
        tuple_objprm: &[MjtNum; "array of object params in all tuples"; ffi().ntupledata],
        key_time: &[MjtNum; "key time"; ffi().nkey],
        (unsafe) name_bodyadr: &[i32; "body name pointers"; ffi().nbody],
        (unsafe) name_jntadr: &[i32; "joint name pointers"; ffi().njnt],
        (unsafe) name_geomadr: &[i32; "geom name pointers"; ffi().ngeom],
        (unsafe) name_siteadr: &[i32; "site name pointers"; ffi().nsite],
        (unsafe) name_camadr: &[i32; "camera name pointers"; ffi().ncam],
        (unsafe) name_lightadr: &[i32; "light name pointers"; ffi().nlight],
        (unsafe) name_flexadr: &[i32; "flex name pointers"; ffi().nflex],
        (unsafe) name_meshadr: &[i32; "mesh name pointers"; ffi().nmesh],
        (unsafe) name_skinadr: &[i32; "skin name pointers"; ffi().nskin],
        (unsafe) name_hfieldadr: &[i32; "hfield name pointers"; ffi().nhfield],
        (unsafe) name_texadr: &[i32; "texture name pointers"; ffi().ntex],
        (unsafe) name_matadr: &[i32; "material name pointers"; ffi().nmat],
        (unsafe) name_pairadr: &[i32; "geom pair name pointers"; ffi().npair],
        (unsafe) name_excludeadr: &[i32; "exclude name pointers"; ffi().nexclude],
        (unsafe) name_eqadr: &[i32; "equality constraint name pointers"; ffi().neq],
        (unsafe) name_tendonadr: &[i32; "tendon name pointers"; ffi().ntendon],
        (unsafe) name_actuatoradr: &[i32; "actuator name pointers"; ffi().nu],
        (unsafe) name_sensoradr: &[i32; "sensor name pointers"; ffi().nsensor],
        (unsafe) name_numericadr: &[i32; "numeric name pointers"; ffi().nnumeric],
        (unsafe) name_textadr: &[i32; "text name pointers"; ffi().ntext],
        (unsafe) name_tupleadr: &[i32; "tuple name pointers"; ffi().ntuple],
        (unsafe) name_keyadr: &[i32; "keyframe name pointers"; ffi().nkey],
        (unsafe) name_pluginadr: &[i32; "plugin instance name pointers"; ffi().nplugin],
        (unsafe) names: &[c_char; "names of all objects, 0-terminated"; ffi().nnames],
        (unsafe) names_map: &[i32; "internal hash map of names"; ffi().nnames_map],
        (unsafe) paths: &[c_char; "paths to assets, 0-terminated"; ffi().npaths],
        (unsafe) B_rownnz: &[i32; "body-dof: non-zeros in each row"; ffi().nbody],
        (unsafe) B_rowadr: &[i32; "body-dof: row addresses"; ffi().nbody],
        (unsafe) B_colind: &[i32; "body-dof: column indices"; ffi().nB],
        (unsafe) M_rownnz: &[i32; "reduced inertia: non-zeros in each row"; ffi().nv],
        (unsafe) M_rowadr: &[i32; "reduced inertia: row addresses"; ffi().nv],
        (unsafe) M_colind: &[i32; "reduced inertia: column indices"; ffi().nC],
        (unsafe) mapM2M: &[i32; "index mapping from qM to M"; ffi().nC],
        (unsafe) D_rownnz: &[i32; "non-zeros in each row"; ffi().nv],
        (unsafe) D_rowadr: &[i32; "full inertia: row addresses"; ffi().nv],
        (unsafe) D_diag: &[i32; "full inertia: index of diagonal element"; ffi().nv],
        (unsafe) D_colind: &[i32; "full inertia: column indices"; ffi().nD],
        (unsafe) mapM2D: &[i32; "index mapping from M to D"; ffi().nD],
        (unsafe) mapD2M: &[i32; "index mapping from D to M"; ffi().nC]
    }

    array_slice_dyn! {
        sublen_dep {
            key_qpos: &[[MjtNum; ffi().nq] [force]; "key position"; ffi().nkey],
            key_qvel: &[[MjtNum; ffi().nv] [force]; "key velocity"; ffi().nkey],
            key_act: &[[MjtNum; ffi().na] [force]; "key activation"; ffi().nkey],
            key_mpos: &[[MjtNum; ffi().nmocap * 3] [force]; "key mocap position"; ffi().nkey],
            key_mquat: &[[MjtNum; ffi().nmocap * 4] [force]; "key mocap quaternion"; ffi().nkey],
            key_ctrl: &[[MjtNum; ffi().nu] [force]; "key control"; ffi().nkey],

            sensor_user: &[[MjtNum; ffi().nuser_sensor] [force]; "user data"; ffi().nsensor],
            actuator_user: &[[MjtNum; ffi().nuser_actuator] [force]; "user data"; ffi().nu],
            tendon_user: &[[MjtNum; ffi().nuser_tendon] [force]; "user data"; ffi().ntendon],
            cam_user: &[[MjtNum; ffi().nuser_cam] [force]; "user data"; ffi().ncam],
            site_user: &[[MjtNum; ffi().nuser_site] [force]; "user data"; ffi().nsite],
            geom_user: &[[MjtNum; ffi().nuser_geom] [force]; "user data"; ffi().ngeom],
            jnt_user: &[[MjtNum; ffi().nuser_jnt] [force]; "user data"; ffi().njnt],
            body_user: &[[MjtNum; ffi().nuser_body] [force]; "user data"; ffi().nbody]
        }
    }
}

impl Clone for MjModel {
    /// # Panics
    /// Panics if MuJoCo fails to allocate the cloned model.
    /// Use [`MjModel::try_clone`] for a fallible alternative.
    fn clone(&self) -> Self {
        self.try_clone().expect("failed to clone model")
    }
}

impl Drop for MjModel {
    fn drop(&mut self) {
        // SAFETY: self.0 is a valid non-null mjModel pointer; called exactly once in Drop.
        unsafe {
            mj_deleteModel(self.0.as_ptr());
        }
    }
}

info_with_view!(Model, actuator,
	[[actuator_] group: i32,
	 [actuator_] delay: MjtNum, [actuator_] ctrllimited: bool [force],
	 [actuator_] forcelimited: bool [force], [actuator_] actlimited: bool [force],
	 [actuator_] dynprm: MjtNum, [actuator_] gainprm: MjtNum,
	 [actuator_] biasprm: MjtNum, [actuator_] actearly: bool [force],
	 [actuator_] ctrlrange: MjtNum, [actuator_] forcerange: MjtNum,
	 [actuator_] actrange: MjtNum, [actuator_] gear: MjtNum,
	 [actuator_] damping: MjtNum, [actuator_] dampingpoly: MjtNum,
	 [actuator_] armature: MjtNum,
	 [actuator_] cranklength: MjtNum, [actuator_] acc0: MjtNum,
	 [actuator_] length0: MjtNum, [actuator_] lengthrange: MjtNum,
	 [actuator_] user: MjtNum,
	 [actuator_] gaintype: MjtGain [force], [actuator_] biastype: MjtBias [force],
	 [actuator_] plugin: i32],
	[[actuator_] trntype: MjtTrn [force], [actuator_] dyntype: MjtDyn [force],
	 [actuator_] trnid: i32, [actuator_] actadr: i32,
	 [actuator_] actnum: i32, [actuator_] history: i32,
	 [actuator_] historyadr: i32],
	[]);

info_with_view!(Model, body,
	[[body_] sameframe: MjtSameFrame [force],
	 [body_] pos: MjtNum,
	 [body_] quat: MjtNum, [body_] ipos: MjtNum,
	 [body_] iquat: MjtNum, [body_] mass: MjtNum,
	 [body_] subtreemass: MjtNum, [body_] inertia: MjtNum,
	 [body_] invweight0: MjtNum, [body_] gravcomp: MjtNum,
	 [body_] margin: MjtNum,
	 [body_] contype: i32, [body_] conaffinity: i32,
	 [body_] user: MjtNum,
	 [body_] simple: MjtByte, [body_] plugin: i32],
	[[body_] parentid: i32, [body_] rootid: i32,
	 [body_] weldid: i32, [body_] mocapid: i32,
	 [body_] jntnum: i32, [body_] jntadr: i32,
	 [body_] dofnum: i32, [body_] dofadr: i32,
	 [body_] treeid: i32, [body_] geomnum: i32,
	 [body_] geomadr: i32,
	 [body_] bvhadr: i32, [body_] bvhnum: i32],
	[]);

info_with_view!(Model, camera,
	[[cam_] mode: MjtCamLight [force],
	 [cam_] pos: MjtNum,
	 [cam_] quat: MjtNum,
	 [cam_] poscom0: MjtNum,
	 [cam_] pos0: MjtNum,
	 [cam_] mat0: MjtNum,
	 [cam_] projection: MjtProjection [force],
	 [cam_] fovy: MjtNum,
	 [cam_] ipd: MjtNum,
     [cam_] output: i32,
	 [cam_] sensorsize: f32,
	 [cam_] intrinsic: f32,
	 [cam_] user: MjtNum],
	[[cam_] bodyid: i32,
	 [cam_] targetbodyid: i32,
	 [cam_] resolution: i32],
	[]);

info_with_view!(Model, equality,
	[[eq_] active0: bool [force],
	 [eq_] solref: MjtNum,
	 [eq_] solimp: MjtNum,
	 [eq_] data: MjtNum],
	[[eq_] r#type: MjtEq [force],
	 [eq_] obj1id: i32,
	 [eq_] obj2id: i32,
     [eq_] objtype: MjtObj [force]],
	[]);

info_with_view!(Model, exclude,
	[[exclude_] signature: i32],
	[],
	[]);

info_with_view!(Model, geom,
	[[geom_] contype: i32,
	 [geom_] conaffinity: i32,
	 [geom_] group: i32,
	 [geom_] priority: i32, [geom_] sameframe: MjtSameFrame [force],
	 [geom_] solmix: MjtNum, [geom_] solref: MjtNum,
	 [geom_] solimp: MjtNum, [geom_] size: MjtNum,
	 [geom_] aabb: MjtNum,
	 [geom_] rbound: MjtNum, [geom_] pos: MjtNum,
	 [geom_] quat: MjtNum, [geom_] friction: MjtNum,
	 [geom_] margin: MjtNum, [geom_] gap: MjtNum, [geom_] fluid: MjtNum,
	 [geom_] user: MjtNum, [geom_] rgba: f32],
	[[geom_] r#type: MjtGeom [force], [geom_] condim: i32,
	 [geom_] bodyid: i32, [geom_] dataid: i32,
	 [geom_] matid: i32, [geom_] plugin: i32],
	[]);

info_with_view!(Model, hfield,
	[[hfield_] size: MjtNum],
	[[hfield_] nrow: i32,
	 [hfield_] ncol: i32,
	 [hfield_] adr: i32,
     [hfield_] pathadr: i32],
	[[hfield_] data: f32]);

info_with_view!(Model, joint,
	[qpos0: MjtNum, qpos_spring: MjtNum,
     [jnt_] group: i32,
     [jnt_] limited: bool [force], [jnt_] actfrclimited: bool [force], [jnt_] actgravcomp: bool [force],
	 [jnt_] solref: MjtNum, [jnt_] solimp: MjtNum,
	 [jnt_] pos: MjtNum,
     [jnt_] axis: MjtNum, [jnt_] stiffness: MjtNum,
     [jnt_] stiffnesspoly: MjtNum,
     [jnt_] range: MjtNum, [jnt_] actfrcrange: MjtNum, [jnt_] margin: MjtNum,
     [jnt_] user: MjtNum,
     [dof_] frictionloss: MjtNum, [dof_] armature: MjtNum,
     [dof_] damping: MjtNum,
     [dof_] dampingpoly: MjtNum,
     [dof_] invweight0: MjtNum,
     [dof_] M0: MjtNum,
     [dof_] simplenum: i32],
	[[jnt_] r#type: MjtJoint [force],
     [jnt_] qposadr: i32,
     [jnt_] dofadr: i32, [jnt_] bodyid: i32, [jnt_] actuatorid: i32,
     dof_bodyid: i32, [dof_] jntid: i32,
     [dof_] parentid: i32, dof_treeid: i32,
     [dof_] Madr: i32],
	[]);

info_with_view!(Model, light,
	[[light_] mode: MjtCamLight [force],
	 [light_] r#type: MjtLightType [force],
	 [light_] castshadow: bool [force],
	 [light_] bulbradius: f32,
	 [light_] intensity: f32,
	 [light_] range: f32,
	 [light_] active: bool [force],
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
	[[light_] bodyid: i32,
	 [light_] targetbodyid: i32,
	 [light_] texid: i32],
	[]);

info_with_view!(Model, material,
	[[mat_] texuniform: bool [force],
	 [mat_] texrepeat: f32,
	 [mat_] emission: f32,
	 [mat_] specular: f32,
	 [mat_] shininess: f32,
	 [mat_] reflectance: f32,
	 [mat_] rgba: f32,
     [mat_] metallic: f32,
     [mat_] roughness: f32],
	[[mat_] texid: i32],
	[]);

info_with_view!(Model, mesh,
	[[mesh_] scale: MjtNum,
	 [mesh_] pos: MjtNum,
	 [mesh_] quat: MjtNum],
	[[mesh_] vertadr: i32,
	 [mesh_] vertnum: i32,
	 [mesh_] texcoordadr: i32,
	 [mesh_] faceadr: i32,
	 [mesh_] facenum: i32,
	 [mesh_] graphadr: i32,
	 [mesh_] normaladr: i32,
	 [mesh_] normalnum: i32,
	 [mesh_] texcoordnum: i32,
	 [mesh_] bvhadr: i32,
	 [mesh_] bvhnum: i32,
	 [mesh_] octadr: i32,
	 [mesh_] octnum: i32,
	 [mesh_] pathadr: i32,
	 [mesh_] polynum: i32,
	 [mesh_] polyadr: i32],
	[]);

info_with_view!(Model, numeric,
	[],
	[[numeric_] adr: i32,
	 [numeric_] size: i32],
	[[numeric_] data: MjtNum]);

info_with_view!(Model, pair,
	[[pair_] solref: MjtNum,
	 [pair_] solimp: MjtNum,
	 [pair_] margin: MjtNum,
	 [pair_] gap: MjtNum,
	 [pair_] friction: MjtNum,
     [pair_] solreffriction: MjtNum,
	 [pair_] signature: i32],
	[[pair_] dim: i32,
	 [pair_] geom1: i32,
	 [pair_] geom2: i32],
	[]);

info_with_view!(Model, sensor,
	[[sensor_] cutoff: MjtNum,
	 [sensor_] noise: MjtNum,
	 [sensor_] delay: MjtNum,
     [sensor_] interval: MjtNum,
	 [sensor_] user: MjtNum,
	 [sensor_] datatype: MjtDataType [force],
	 [sensor_] needstage: MjtStage [force],
	 [sensor_] intprm: i32],
	[[sensor_] r#type: MjtSensor [force],
	 [sensor_] objid: i32,
	 [sensor_] refid: i32,
	 [sensor_] objtype: MjtObj [force],
	 [sensor_] reftype: MjtObj [force],
	 [sensor_] dim: i32,
	 [sensor_] adr: i32,
	 [sensor_] history: i32,
	 [sensor_] historyadr: i32,
	 [sensor_] plugin: i32],
	[]);

info_with_view!(Model, site,
	[[site_] group: i32,
	 [site_] sameframe: MjtSameFrame [force],
	 [site_] size: MjtNum,
	 [site_] pos: MjtNum,
	 [site_] quat: MjtNum,
	 [site_] user: MjtNum,
	 [site_] rgba: f32,
	 [site_] r#type: MjtGeom [force]],
	[[site_] bodyid: i32,
	 [site_] matid: i32],
	[]);

info_with_view!(Model, skin,
	[[skin_] group: i32,
	 [skin_] rgba: f32,
	 [skin_] inflate: f32],
	[[skin_] matid: i32,
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
	[[tendon_] group: i32,
	 [tendon_] limited: bool [force], [tendon_] actfrclimited: bool [force], [tendon_] width: MjtNum,
	 [tendon_] solref_lim: MjtNum, [tendon_] solimp_lim: MjtNum,
	 [tendon_] solref_fri: MjtNum, [tendon_] solimp_fri: MjtNum,
	 [tendon_] range: MjtNum, [tendon_] actfrcrange: MjtNum, [tendon_] margin: MjtNum,
	 [tendon_] stiffness: MjtNum,
	 [tendon_] stiffnesspoly: MjtNum,
	 [tendon_] damping: MjtNum,
	 [tendon_] dampingpoly: MjtNum,
	 [tendon_] armature: MjtNum,
	 [tendon_] frictionloss: MjtNum, [tendon_] lengthspring: MjtNum,
	 [tendon_] length0: MjtNum, [tendon_] invweight0: MjtNum,
	 [tendon_] user: MjtNum, [tendon_] rgba: f32,
	 [tendon_] treenum: i32],
	[[tendon_] matid: i32, [tendon_] actuatorid: i32, [tendon_] treeid: i32,
	 [tendon_] adr: i32, [tendon_] num: i32,
     [ten_] J_rownnz: i32, [ten_] J_rowadr: i32, [ten_] J_colind: i32],
	[]);

info_with_view!(Model, texture,
	[[tex_] colorspace: MjtColorSpace [force],
	 [tex_] r#type: MjtTexture [force]],
	[[tex_] height: i32,
	 [tex_] width: i32,
	 [tex_] nchannel: i32,
	 [tex_] adr: MjtSize,
	 [tex_] pathadr: i32],
	[[tex_] data: MjtByte]);

info_with_view!(Model, tuple,
	[[tuple_] objprm: MjtNum,
	 [tuple_] objtype: MjtObj [force]],
	[[tuple_] adr: i32,
	 [tuple_] size: i32,
	 [tuple_] objid: i32],
	[]);

info_with_view!(Model, key,
	[[key_] time: MjtNum,
	 [key_] qpos: MjtNum,
	 [key_] qvel: MjtNum,
	 [key_] act: MjtNum,
	 [key_] mpos: MjtNum,
	 [key_] mquat: MjtNum,
	 [key_] ctrl: MjtNum],
	[],
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
        view_mut.delay[0] = 3.0;

        assert_eq!(view_mut.gaintype[0], MjtGain::mjGAIN_AFFINE);
        assert_eq!(view_mut.delay[0], 3.0);
        view_mut.zero();

        assert_eq!(view_mut.delay[0], 0.0);
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
        assert!(!view.dof_bodyid.is_empty());
        assert!(!view.dof_treeid.is_empty());

        let dof_start = model.jnt_dofadr()[model_info.id] as usize;
        assert_eq!(view.dof_bodyid[0], model.dof_bodyid()[dof_start]);
        assert_eq!(view.dof_treeid[0], model.dof_treeid()[dof_start]);

        // Also validate mapping for a multi-DOF joint.
        let free_info = model.joint("ball2").unwrap();
        let free_view = free_info.view(&model);
        assert!(free_view.dof_bodyid.len() > 1);
        assert_eq!(free_view.dof_bodyid.len(), free_view.dof_treeid.len());

        let free_dof_start = model.jnt_dofadr()[free_info.id] as usize;
        let free_dof_end = free_dof_start + free_view.dof_bodyid.len();
        assert_eq!(&free_view.dof_bodyid[..], &model.dof_bodyid()[free_dof_start..free_dof_end]);
        assert_eq!(&free_view.dof_treeid[..], &model.dof_treeid()[free_dof_start..free_dof_end]);

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
    fn test_try_view_model_signature_mismatch() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint type='free'/><geom size='0.1'/></body></worldbody></mujoco>").unwrap();
        let mut model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint type='free'/><geom size='0.1'/></body><body name='extra'/></worldbody></mujoco>").unwrap();

        let body_info = model1.body("b1").unwrap();

        let err = body_info.try_view(&model2).unwrap_err();
        match err {
            MjModelError::SignatureMismatch { source, destination } => {
                assert_eq!(source, model1.signature());
                assert_eq!(destination, model2.signature());
            }
            other => panic!("expected SignatureMismatch, got {other:?}"),
        }

        let err = body_info.try_view_mut(&mut model2).unwrap_err();
        match err {
            MjModelError::SignatureMismatch { source, destination } => {
                assert_eq!(source, model1.signature());
                assert_eq!(destination, model2.signature());
            }
            other => panic!("expected SignatureMismatch, got {other:?}"),
        }
    }

    #[test]
    #[should_panic(expected = "model signature mismatch")]
    fn test_view_mut_model_signature_mismatch_panics() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint type='free'/><geom size='0.1'/></body></worldbody></mujoco>").unwrap();
        let mut model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint type='free'/><geom size='0.1'/></body><body name='extra'/></worldbody></mujoco>").unwrap();

        let body_info = model1.body("b1").unwrap();
        let _view = body_info.view_mut(&mut model2);
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

        let mass_before = model.totalmass();
        model.set_totalmass(5.0);
        let mass_after = model.totalmass();

        assert_relative_eq!(mass_after, 5.0, epsilon = 1e-9);
        assert_ne!(mass_before, mass_after);
    }

    /// Tests if copying the model works without any memory problems.
    #[test]
    fn test_copy_model() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        let _cloned = model.clone();
    }

    #[test]
    fn test_model_save() {
        const MODEL_SAVE_PATH: &str = "./__TMP_MODEL2.mjb";
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        model.save_to_file(MODEL_SAVE_PATH).unwrap();

        let saved_data = fs::read(MODEL_SAVE_PATH).unwrap();
        let mut data = vec![0; saved_data.len()];
        model.save_to_buffer(&mut data).unwrap();

        assert_eq!(saved_data, data);
        fs::remove_file(MODEL_SAVE_PATH).unwrap();

        /* Test virtual file system load */
        let model = MjModel::from_buffer(&saved_data).unwrap();
        assert!(model.light("lamp_light2").is_some());
        assert!(model.light("lamp_light-xyz").is_none());
    }

    #[test]
    fn test_site_view() {
        // <site name="ball22" size="0.5 0.25 0.5" pos="5 1 3" rgba="1 2 3 1" type="box"/>
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
        assert_eq!(view_light.castshadow[0], CASTSHADOW);
        assert_eq!(view_light.active[0], ACTIVE);

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
        assert_eq!(view_eq.obj1id[0], model.name_to_id(MjtObj::mjOBJ_BODY, "eq_body3").unwrap() as i32);
        assert_eq!(view_eq.obj2id[0], model.name_to_id(MjtObj::mjOBJ_BODY, "eq_body4").unwrap() as i32);
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

    /// Tests [`MjModel::extract_state_into`] for correctness.
    #[test]
    fn test_state_extract() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        /* Test of extraction into existing buffer */
        // Physics is subset of full physics.
        // Extract physics from full physics.
        let state_full_physics = data.state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let state_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);

        let required_size = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst_buffer = vec![0.0; required_size].into_boxed_slice();
        let _bytes_written = model.extract_state_into(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst_buffer, MjtState::mjSTATE_PHYSICS as u32
        );

        assert_eq!(state_physics, dst_buffer);

        /* Test of extraction into new buffer (internally) */
        // Physics is subset of full physics.
        // Extract physics from full physics.
        let state_full_physics = data.state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let state_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);

        let dst_buffer = model.extract_state(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            MjtState::mjSTATE_PHYSICS as u32
        );

        assert_eq!(state_physics, dst_buffer);
    }

    /// Tests for the expected panic when giving a source spec that does not match
    /// the source array in state extraction.

    #[test]
    fn test_state_extract_state_invalid_src() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.try_extract_state(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert!(matches!(err, MjModelError::StateSliceLengthMismatch { .. }));
    }

    #[test]
    fn test_state_extract_state_into_invalid_src() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let required_size = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst_buffer = vec![0.0; required_size].into_boxed_slice();
        let state_full_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.try_extract_state_into(
            &state_full_physics, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst_buffer, MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert!(matches!(err, MjModelError::StateSliceLengthMismatch { .. }));
    }

    #[test]
    fn test_state_extract_dst_spec_not_subset() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);
        let res = model.try_extract_state(
            &state_physics, MjtState::mjSTATE_PHYSICS as u32,
            MjtState::mjSTATE_FULLPHYSICS as u32
        );

        let err = res.unwrap_err();
        assert!(matches!(err, MjModelError::SpecNotSubset));
    }

    #[test]
    fn test_state_extract_into_dst_spec_not_subset() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_physics = data.state(MjtState::mjSTATE_PHYSICS as u32);
        let mut dst = vec![0.0; model.state_size(MjtState::mjSTATE_PHYSICS as u32)];

        let res = model.try_extract_state_into(
            &state_physics, MjtState::mjSTATE_PHYSICS as u32,
            &mut dst, MjtState::mjSTATE_FULLPHYSICS as u32
        );

        let err = res.unwrap_err();
        assert!(matches!(err, MjModelError::SpecNotSubset));
    }

    #[test]
    fn test_state_extract_into_dst_buffer_too_small() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full = data.state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let required = model.state_size(MjtState::mjSTATE_PHYSICS as u32);
        // make buffer smaller than required
        let mut dst = vec![0.0; required.saturating_sub(1)];

        let res = model.try_extract_state_into(
            &state_full, MjtState::mjSTATE_FULLPHYSICS as u32,
            &mut dst, MjtState::mjSTATE_PHYSICS as u32
        );

        let err = res.unwrap_err();
        assert!(matches!(err, MjModelError::BufferTooSmall { .. }));
    }

    #[test]
    fn test_state_extract_zero_spec() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let data = MjData::new(&model);

        let state_full = data.state(MjtState::mjSTATE_FULLPHYSICS as u32);

        // extract zero-sized spec -> empty slice
        let dst = model.extract_state(&state_full, MjtState::mjSTATE_FULLPHYSICS as u32, 0u32);
        assert_eq!(dst.len(), 0);

        // extract_into with zero-sized spec -> writes 0 elements
        let mut buf: &mut [f64] = &mut [];
        let written = model.extract_state_into(&state_full, MjtState::mjSTATE_FULLPHYSICS as u32, &mut buf, 0u32);
        assert_eq!(written, 0);
    }

    /**************************************************************************/
    // Force-cast macro correctness tests for MjModel
    /**************************************************************************/

    /// Verifies [force]-cast array grouping for body_pos (&[[MjtNum; 3]]),
    /// body_quat (&[[MjtNum; 4]]), body_inertia (&[[MjtNum; 3]]), body_invweight0 (&[[MjtNum; 2]]).
    #[test]
    fn test_force_cast_body_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nbody = model.ffi().nbody as usize;

        let body_pos = model.body_pos();
        let body_quat = model.body_quat();
        let body_inertia = model.body_inertia();
        let body_invweight0 = model.body_invweight0();
        let body_ipos = model.body_ipos();
        let body_iquat = model.body_iquat();

        assert_eq!(body_pos.len(), nbody);
        assert_eq!(body_quat.len(), nbody);
        assert_eq!(body_inertia.len(), nbody);
        assert_eq!(body_invweight0.len(), nbody);
        assert_eq!(body_ipos.len(), nbody);
        assert_eq!(body_iquat.len(), nbody);

        // Cross-validate against raw FFI
        for i in 0..nbody {
            for j in 0..3 {
                assert_eq!(body_pos[i][j], unsafe { *model.ffi().body_pos.add(i * 3 + j) },
                    "body_pos[{}][{}] mismatch", i, j);
            }
            for j in 0..4 {
                assert_eq!(body_quat[i][j], unsafe { *model.ffi().body_quat.add(i * 4 + j) },
                    "body_quat[{}][{}] mismatch", i, j);
            }
            for j in 0..3 {
                assert_eq!(body_inertia[i][j], unsafe { *model.ffi().body_inertia.add(i * 3 + j) },
                    "body_inertia[{}][{}] mismatch", i, j);
            }
            for j in 0..2 {
                assert_eq!(body_invweight0[i][j], unsafe { *model.ffi().body_invweight0.add(i * 2 + j) },
                    "body_invweight0[{}][{}] mismatch", i, j);
            }
        }
    }

    /// Verifies [force]-cast enum for jnt_type (*mut i32 -> *mut MjtJoint).
    #[test]
    fn test_force_cast_jnt_type_enum() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let njnt = model.ffi().njnt as usize;
        let jnt_type = model.jnt_type();

        assert_eq!(jnt_type.len(), njnt);

        // Cross-validate with raw FFI
        for i in 0..njnt {
            let raw_i32 = unsafe { *model.ffi().jnt_type.add(i) };
            let expected: MjtJoint = unsafe { crate::util::force_cast(raw_i32) };
            assert_eq!(jnt_type[i], expected,
                "jnt_type[{}]: got {:?}, expected {:?} (raw={})", i, jnt_type[i], expected, raw_i32);
        }

        // Verify known joints: "ball" is free, "rod" is slide
        let ball_jnt = model.joint("ball").unwrap();
        assert_eq!(jnt_type[ball_jnt.id], MjtJoint::mjJNT_FREE);

        let rod_jnt = model.joint("rod").unwrap();
        assert_eq!(jnt_type[rod_jnt.id], MjtJoint::mjJNT_SLIDE);
    }

    /// Verifies [force]-cast bool for jnt_limited (*mut u8 -> *mut bool).
    #[test]
    fn test_force_cast_jnt_limited_bool() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let njnt = model.ffi().njnt as usize;
        let jnt_limited = model.jnt_limited();

        assert_eq!(jnt_limited.len(), njnt);

        // Cross-validate with raw FFI
        for i in 0..njnt {
            let raw_u8 = unsafe { *model.ffi().jnt_limited.add(i) };
            assert!(raw_u8 == 0 || raw_u8 == 1,
                "raw jnt_limited[{}]={} must be 0 or 1", i, raw_u8);
            assert_eq!(jnt_limited[i], raw_u8 != 0,
                "jnt_limited[{}] mismatch: bool={}, raw={}", i, jnt_limited[i], raw_u8);
        }

        // "rod" joint has range="0 1" -> limited=true; "ball" is free -> limited=false
        let rod_jnt = model.joint("rod").unwrap();
        assert_eq!(jnt_limited[rod_jnt.id], true);

        let ball_jnt = model.joint("ball").unwrap();
        assert_eq!(jnt_limited[ball_jnt.id], false);
    }

    /// Verifies [force]-cast enum for geom_type (*mut i32 -> *mut MjtGeom).
    #[test]
    fn test_force_cast_geom_type_enum() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let ngeom = model.ffi().ngeom as usize;
        let geom_type = model.geom_type();

        assert_eq!(geom_type.len(), ngeom);

        // Cross-validate
        for i in 0..ngeom {
            let raw_i32 = unsafe { *model.ffi().geom_type.add(i) };
            let expected: MjtGeom = unsafe { crate::util::force_cast(raw_i32) };
            assert_eq!(geom_type[i], expected);
        }

        // Verify known geoms
        let sphere_geom = model.geom("green_sphere").unwrap();
        assert_eq!(geom_type[sphere_geom.id], MjtGeom::mjGEOM_SPHERE);

        let floor_geom = model.geom("floor").unwrap();
        assert_eq!(geom_type[floor_geom.id], MjtGeom::mjGEOM_PLANE);

        let rod_geom = model.geom("rod").unwrap();
        assert_eq!(geom_type[rod_geom.id], MjtGeom::mjGEOM_CYLINDER);
    }

    /// Verifies [force]-cast for geom_size (&[[MjtNum; 3]]), geom_pos (&[[MjtNum; 3]]),
    /// geom_quat (&[[MjtNum; 4]]), geom_rgba (&[[f32; 4]]), geom_friction (&[[MjtNum; 3]]),
    /// geom_aabb (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_geom_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let ngeom = model.ffi().ngeom as usize;

        let geom_size = model.geom_size();
        let geom_pos = model.geom_pos();
        let geom_quat = model.geom_quat();
        let geom_rgba = model.geom_rgba();
        let geom_friction = model.geom_friction();
        let geom_aabb = model.geom_aabb();

        assert_eq!(geom_size.len(), ngeom);
        assert_eq!(geom_pos.len(), ngeom);
        assert_eq!(geom_quat.len(), ngeom);
        assert_eq!(geom_rgba.len(), ngeom);
        assert_eq!(geom_friction.len(), ngeom);
        assert_eq!(geom_aabb.len(), ngeom);

        // Cross-validate all against FFI
        for i in 0..ngeom {
            for j in 0..3 {
                assert_eq!(geom_size[i][j], unsafe { *model.ffi().geom_size.add(i * 3 + j) });
                assert_eq!(geom_pos[i][j], unsafe { *model.ffi().geom_pos.add(i * 3 + j) });
                assert_eq!(geom_friction[i][j], unsafe { *model.ffi().geom_friction.add(i * 3 + j) });
            }
            for j in 0..4 {
                assert_eq!(geom_quat[i][j], unsafe { *model.ffi().geom_quat.add(i * 4 + j) });
                assert_eq!(geom_rgba[i][j], unsafe { *model.ffi().geom_rgba.add(i * 4 + j) });
            }
            for j in 0..6 {
                assert_eq!(geom_aabb[i][j], unsafe { *model.ffi().geom_aabb.add(i * 6 + j) });
            }
        }

        // Verify a known geom: "green_sphere" has rgba="0 1 0 1" and size="0.1"
        let gs = model.geom("green_sphere").unwrap();
        assert_eq!(geom_rgba[gs.id], [0.0f32, 1.0, 0.0, 1.0]);
        assert_relative_eq!(geom_size[gs.id][0], 0.1, epsilon = 1e-9);
    }

    /// Verifies [force]-cast for camera arrays: cam_mode (MjtCamLight), cam_projection (MjtProjection),
    /// cam_resolution (&[[i32; 2]]), cam_sensorsize (&[[f32; 2]]), cam_intrinsic (&[[f32; 4]]),
    /// cam_mat0 (&[[MjtNum; 9]]).
    #[test]
    fn test_force_cast_camera_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let ncam = model.ffi().ncam as usize;

        if ncam == 0 {
            // The EXAMPLE_MODEL has a camera, but guard just in case
            return;
        }

        let cam_mode = model.cam_mode();
        let cam_projection = model.cam_projection();
        let cam_resolution = model.cam_resolution();
        let cam_sensorsize = model.cam_sensorsize();
        let cam_intrinsic = model.cam_intrinsic();
        let cam_mat0 = model.cam_mat0();

        assert_eq!(cam_mode.len(), ncam);
        assert_eq!(cam_projection.len(), ncam);
        assert_eq!(cam_resolution.len(), ncam);
        assert_eq!(cam_sensorsize.len(), ncam);
        assert_eq!(cam_intrinsic.len(), ncam);
        assert_eq!(cam_mat0.len(), ncam);

        // Cross-validate enum casts against raw FFI
        for i in 0..ncam {
            let raw_mode = unsafe { *model.ffi().cam_mode.add(i) };
            let expected_mode: MjtCamLight = unsafe { crate::util::force_cast(raw_mode) };
            assert_eq!(cam_mode[i], expected_mode);

            let raw_proj = unsafe { *model.ffi().cam_projection.add(i) };
            let expected_proj: MjtProjection = unsafe { crate::util::force_cast(raw_proj) };
            assert_eq!(cam_projection[i], expected_proj);

            for j in 0..2 {
                assert_eq!(cam_resolution[i][j], unsafe { *model.ffi().cam_resolution.add(i * 2 + j) });
                assert_eq!(cam_sensorsize[i][j], unsafe { *model.ffi().cam_sensorsize.add(i * 2 + j) });
            }
            for j in 0..4 {
                assert_eq!(cam_intrinsic[i][j], unsafe { *model.ffi().cam_intrinsic.add(i * 4 + j) });
            }
            for j in 0..9 {
                assert_eq!(cam_mat0[i][j], unsafe { *model.ffi().cam_mat0.add(i * 9 + j) });
            }
        }

        // Verify known camera: "cam1" has resolution="100 200"
        let cam1 = model.camera("cam1").unwrap();
        assert_eq!(cam_resolution[cam1.id], [100, 200]);
    }

    /// Verifies [force]-cast for bvh_child (&[[i32; 2]]) and bvh_aabb (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_bvh_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nbvh = model.ffi().nbvh as usize;

        let bvh_child = model.bvh_child();
        assert_eq!(bvh_child.len(), nbvh);

        for i in 0..nbvh {
            for j in 0..2 {
                assert_eq!(bvh_child[i][j], unsafe { *model.ffi().bvh_child.add(i * 2 + j) });
            }
        }

        let nbvhstatic = model.ffi().nbvhstatic as usize;
        let bvh_aabb = model.bvh_aabb();
        assert_eq!(bvh_aabb.len(), nbvhstatic);

        for i in 0..nbvhstatic {
            for j in 0..6 {
                assert_eq!(bvh_aabb[i][j], unsafe { *model.ffi().bvh_aabb.add(i * 6 + j) });
            }
        }
    }

    /// Verifies [force]-cast enum: body_sameframe and geom_sameframe (MjtSameFrame).
    #[test]
    fn test_force_cast_sameframe_enum() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nbody = model.ffi().nbody as usize;
        let ngeom = model.ffi().ngeom as usize;

        let body_sameframe = model.body_sameframe();
        let geom_sameframe = model.geom_sameframe();

        assert_eq!(body_sameframe.len(), nbody);
        assert_eq!(geom_sameframe.len(), ngeom);

        for i in 0..nbody {
            let raw = unsafe { *model.ffi().body_sameframe.add(i) };
            let expected: MjtSameFrame = unsafe { crate::util::force_cast(raw) };
            assert_eq!(body_sameframe[i], expected, "body_sameframe[{}] mismatch", i);
        }

        for i in 0..ngeom {
            let raw = unsafe { *model.ffi().geom_sameframe.add(i) };
            let expected: MjtSameFrame = unsafe { crate::util::force_cast(raw) };
            assert_eq!(geom_sameframe[i], expected, "geom_sameframe[{}] mismatch", i);
        }
    }

    /// Verifies [force]-cast for equality constraint arrays: eq_type (MjtEq),
    /// eq_objtype (MjtObj), eq_active0 (bool), eq_data (&[[MjtNum; mjNEQDATA]]).
    #[test]
    fn test_force_cast_equality_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let neq = model.ffi().neq as usize;

        if neq == 0 {
            return;
        }

        let eq_type = model.eq_type();
        let eq_objtype = model.eq_objtype();
        let eq_active0 = model.eq_active0();

        assert_eq!(eq_type.len(), neq);
        assert_eq!(eq_objtype.len(), neq);
        assert_eq!(eq_active0.len(), neq);

        // Cross-validate enum casts
        for i in 0..neq {
            let raw_type = unsafe { *model.ffi().eq_type.add(i) };
            let expected_type: MjtEq = unsafe { crate::util::force_cast(raw_type) };
            assert_eq!(eq_type[i], expected_type);

            let raw_objtype = unsafe { *model.ffi().eq_objtype.add(i) };
            let expected_objtype: MjtObj = unsafe { crate::util::force_cast(raw_objtype) };
            assert_eq!(eq_objtype[i], expected_objtype);

            let raw_active = unsafe { *model.ffi().eq_active0.add(i) };
            assert_eq!(eq_active0[i], raw_active != 0);
        }

        // Verify known equality: "eq1" is a connect constraint
        let eq1 = model.equality("eq1").unwrap();
        assert_eq!(eq_type[eq1.id], MjtEq::mjEQ_CONNECT);
        assert_eq!(eq_objtype[eq1.id], MjtObj::mjOBJ_BODY);
        assert_eq!(eq_active0[eq1.id], true);
    }

    /// Verifies [force]-cast for sensor arrays: sensor_type (MjtSensor), sensor_datatype (MjtDataType),
    /// sensor_needstage (MjtStage), sensor_objtype (MjtObj), sensor_reftype (MjtObj).
    #[test]
    fn test_force_cast_sensor_model_enums() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nsensor = model.ffi().nsensor as usize;

        if nsensor == 0 {
            return;
        }

        let sensor_type = model.sensor_type();
        let sensor_datatype = model.sensor_datatype();
        let sensor_needstage = model.sensor_needstage();
        let sensor_objtype = model.sensor_objtype();
        let sensor_reftype = model.sensor_reftype();

        assert_eq!(sensor_type.len(), nsensor);
        assert_eq!(sensor_datatype.len(), nsensor);
        assert_eq!(sensor_needstage.len(), nsensor);
        assert_eq!(sensor_objtype.len(), nsensor);
        assert_eq!(sensor_reftype.len(), nsensor);

        for i in 0..nsensor {
            let raw_type = unsafe { *model.ffi().sensor_type.add(i) };
            let raw_datatype = unsafe { *model.ffi().sensor_datatype.add(i) };
            let raw_needstage = unsafe { *model.ffi().sensor_needstage.add(i) };
            let raw_objtype = unsafe { *model.ffi().sensor_objtype.add(i) };
            let raw_reftype = unsafe { *model.ffi().sensor_reftype.add(i) };

            assert_eq!(sensor_type[i], unsafe { crate::util::force_cast::<_, MjtSensor>(raw_type) });
            assert_eq!(sensor_datatype[i], unsafe { crate::util::force_cast::<_, MjtDataType>(raw_datatype) });
            assert_eq!(sensor_needstage[i], unsafe { crate::util::force_cast::<_, MjtStage>(raw_needstage) });
            assert_eq!(sensor_objtype[i], unsafe { crate::util::force_cast::<_, MjtObj>(raw_objtype) });
            assert_eq!(sensor_reftype[i], unsafe { crate::util::force_cast::<_, MjtObj>(raw_reftype) });
        }

        // Verify known sensor: "touch" is a touch sensor on a site
        let touch = model.sensor("touch").unwrap();
        assert_eq!(sensor_type[touch.id], MjtSensor::mjSENS_TOUCH);
        assert_eq!(sensor_objtype[touch.id], MjtObj::mjOBJ_SITE);
    }

    /// Verifies [force]-cast for actuator enum arrays: trntype (MjtTrn), dyntype (MjtDyn),
    /// gaintype (MjtGain), biastype (MjtBias), and bool ctrllimited, forcelimited, actlimited.
    #[test]
    fn test_force_cast_actuator_model_enums_and_bools() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nu = model.ffi().nu as usize;

        if nu == 0 {
            return;
        }

        let trntype = model.actuator_trntype();
        let dyntype = model.actuator_dyntype();
        let gaintype = model.actuator_gaintype();
        let biastype = model.actuator_biastype();
        let ctrllimited = model.actuator_ctrllimited();
        let forcelimited = model.actuator_forcelimited();
        let actlimited = model.actuator_actlimited();
        let actearly = model.actuator_actearly();

        assert_eq!(trntype.len(), nu);
        assert_eq!(dyntype.len(), nu);
        assert_eq!(gaintype.len(), nu);
        assert_eq!(biastype.len(), nu);
        assert_eq!(ctrllimited.len(), nu);
        assert_eq!(forcelimited.len(), nu);
        assert_eq!(actlimited.len(), nu);
        assert_eq!(actearly.len(), nu);

        for i in 0..nu {
            // Enum cross-validation
            assert_eq!(trntype[i], unsafe { crate::util::force_cast::<_, MjtTrn>(*model.ffi().actuator_trntype.add(i)) });
            assert_eq!(dyntype[i], unsafe { crate::util::force_cast::<_, MjtDyn>(*model.ffi().actuator_dyntype.add(i)) });
            assert_eq!(gaintype[i], unsafe { crate::util::force_cast::<_, MjtGain>(*model.ffi().actuator_gaintype.add(i)) });
            assert_eq!(biastype[i], unsafe { crate::util::force_cast::<_, MjtBias>(*model.ffi().actuator_biastype.add(i)) });

            // Bool cross-validation
            let raw_ctrllimited = unsafe { *model.ffi().actuator_ctrllimited.add(i) };
            assert_eq!(ctrllimited[i], raw_ctrllimited != 0);
            let raw_forcelimited = unsafe { *model.ffi().actuator_forcelimited.add(i) };
            assert_eq!(forcelimited[i], raw_forcelimited != 0);
            let raw_actlimited = unsafe { *model.ffi().actuator_actlimited.add(i) };
            assert_eq!(actlimited[i], raw_actlimited != 0);
            let raw_actearly = unsafe { *model.ffi().actuator_actearly.add(i) };
            assert_eq!(actearly[i], raw_actearly != 0);
        }

        // Verify known actuator: "slider" has biastype=affine, gaintype=fixed, ctrllimited=true
        let slider = model.actuator("slider").unwrap();
        assert_eq!(biastype[slider.id], MjtBias::mjBIAS_AFFINE);
        assert_eq!(gaintype[slider.id], MjtGain::mjGAIN_FIXED);
        assert_eq!(ctrllimited[slider.id], true);
    }

    /// Verifies [force]-cast for actuator parameter arrays: dynprm, gainprm, biasprm, ctrlrange,
    /// gear (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_actuator_param_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nu = model.ffi().nu as usize;

        let dynprm = model.actuator_dynprm();
        let ctrlrange = model.actuator_ctrlrange();
        let gear = model.actuator_gear();
        let trnid = model.actuator_trnid();

        assert_eq!(dynprm.len(), nu);
        assert_eq!(ctrlrange.len(), nu);
        assert_eq!(gear.len(), nu);
        assert_eq!(trnid.len(), nu);

        // Verify "slider" dynprm and ctrlrange
        let slider = model.actuator("slider").unwrap();
        assert_eq!(dynprm[slider.id][0..10], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]);
        assert_eq!(ctrlrange[slider.id], [0.0, 1.0]);

        // Verify "slider2" has reversed dynprm
        let slider2 = model.actuator("slider2").unwrap();
        assert_eq!(dynprm[slider2.id][0..10], [10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]);

        // Cross-validate FFI for gear (stride 6)
        for i in 0..nu {
            for j in 0..6 {
                assert_eq!(gear[i][j], unsafe { *model.ffi().actuator_gear.add(i * 6 + j) });
            }
            for j in 0..2 {
                assert_eq!(trnid[i][j], unsafe { *model.ffi().actuator_trnid.add(i * 2 + j) });
                assert_eq!(ctrlrange[i][j], unsafe { *model.ffi().actuator_ctrlrange.add(i * 2 + j) });
            }
        }
    }

    /// Verifies [force]-cast for tendon bools and arrays: tendon_limited (bool),
    /// tendon_range (&[[MjtNum; 2]]), tendon_rgba (&[[f32; 4]]), tendon_treeid (&[[i32; 2]]).
    #[test]
    fn test_force_cast_tendon_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let ntendon = model.ffi().ntendon as usize;

        if ntendon == 0 {
            return;
        }

        let tendon_limited = model.tendon_limited();
        let tendon_range = model.tendon_range();
        let tendon_rgba = model.tendon_rgba();
        let tendon_treeid = model.tendon_treeid();
        let tendon_lengthspring = model.tendon_lengthspring();

        assert_eq!(tendon_limited.len(), ntendon);
        assert_eq!(tendon_range.len(), ntendon);
        assert_eq!(tendon_rgba.len(), ntendon);
        assert_eq!(tendon_treeid.len(), ntendon);
        assert_eq!(tendon_lengthspring.len(), ntendon);

        for i in 0..ntendon {
            let raw_limited = unsafe { *model.ffi().tendon_limited.add(i) };
            assert_eq!(tendon_limited[i], raw_limited != 0);

            for j in 0..2 {
                assert_eq!(tendon_range[i][j], unsafe { *model.ffi().tendon_range.add(i * 2 + j) });
                assert_eq!(tendon_treeid[i][j], unsafe { *model.ffi().tendon_treeid.add(i * 2 + j) });
                assert_eq!(tendon_lengthspring[i][j], unsafe { *model.ffi().tendon_lengthspring.add(i * 2 + j) });
            }

            for j in 0..4 {
                assert_eq!(tendon_rgba[i][j], unsafe { *model.ffi().tendon_rgba.add(i * 4 + j) });
            }
        }

        // Verify known tendon: "tendon2" limited=true, range=(0,1), rgba=(0, 0.1, 1, 1)
        let ten2 = model.tendon("tendon2").unwrap();
        assert_eq!(tendon_limited[ten2.id], true);
        assert_eq!(tendon_range[ten2.id], [0.0, 1.0]);
        assert_relative_eq!(tendon_rgba[ten2.id][0], 0.0f32, epsilon = 1e-6);
        assert_relative_eq!(tendon_rgba[ten2.id][1], 0.1f32, epsilon = 1e-6);
    }

    /// Verifies [force]-cast for texture enum: tex_type (MjtTexture), tex_colorspace (MjtColorSpace).
    #[test]
    fn test_force_cast_texture_model_enums() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let ntex = model.ffi().ntex as usize;

        if ntex == 0 {
            return;
        }

        let tex_type = model.tex_type();
        let tex_colorspace = model.tex_colorspace();

        assert_eq!(tex_type.len(), ntex);
        assert_eq!(tex_colorspace.len(), ntex);

        for i in 0..ntex {
            let raw_type = unsafe { *model.ffi().tex_type.add(i) };
            let expected_type: MjtTexture = unsafe { crate::util::force_cast(raw_type) };
            assert_eq!(tex_type[i], expected_type);

            let raw_cs = unsafe { *model.ffi().tex_colorspace.add(i) };
            let expected_cs: MjtColorSpace = unsafe { crate::util::force_cast(raw_cs) };
            assert_eq!(tex_colorspace[i], expected_cs);
        }

        // "wall_tex" is 2D, sRGB
        let wall_tex = model.texture("wall_tex").unwrap();
        assert_eq!(tex_type[wall_tex.id], MjtTexture::mjTEXTURE_2D);
        assert_eq!(tex_colorspace[wall_tex.id], MjtColorSpace::mjCOLORSPACE_SRGB);
    }

    /// Verifies [force]-cast for material bools and f32 arrays: mat_texuniform (bool),
    /// mat_texrepeat (&[[f32; 2]]), mat_rgba (&[[f32; 4]]).
    #[test]
    fn test_force_cast_material_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nmat = model.ffi().nmat as usize;

        if nmat == 0 {
            return;
        }

        let mat_texuniform = model.mat_texuniform();
        let mat_texrepeat = model.mat_texrepeat();
        let mat_rgba = model.mat_rgba();

        assert_eq!(mat_texuniform.len(), nmat);
        assert_eq!(mat_texrepeat.len(), nmat);
        assert_eq!(mat_rgba.len(), nmat);

        for i in 0..nmat {
            let raw_uniform = unsafe { *model.ffi().mat_texuniform.add(i) };
            assert_eq!(mat_texuniform[i], raw_uniform != 0);

            for j in 0..2 {
                assert_eq!(mat_texrepeat[i][j], unsafe { *model.ffi().mat_texrepeat.add(i * 2 + j) });
            }
            for j in 0..4 {
                assert_eq!(mat_rgba[i][j], unsafe { *model.ffi().mat_rgba.add(i * 4 + j) });
            }
        }

        // "also_wood_material" has texuniform=false, texrepeat=[2,2], rgba=[0.8,0.5,0.3,1.0]
        let mat = model.material("also_wood_material").unwrap();
        assert_eq!(mat_texuniform[mat.id], false);
        assert_eq!(mat_texrepeat[mat.id], [2.0f32, 2.0]);
        assert_eq!(mat_rgba[mat.id], [0.8f32, 0.5, 0.3, 1.0]);
    }

    /// Verifies [force]-cast for light arrays: light mode/type enums, light_castshadow/active bools,
    /// light_attenuation (&[[f32; 3]]), light_ambient/diffuse/specular (&[[f32; 3]]),
    /// light_pos/dir (&[[MjtNum; 3]]).
    #[test]
    fn test_force_cast_light_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nlight = model.ffi().nlight as usize;

        if nlight == 0 {
            return;
        }

        let light_mode = model.light_mode();
        let light_type = model.light_type();
        let light_castshadow = model.light_castshadow();
        let light_active = model.light_active();
        let light_attenuation = model.light_attenuation();
        let light_ambient = model.light_ambient();
        let light_diffuse = model.light_diffuse();
        let light_specular = model.light_specular();
        let light_pos = model.light_pos();
        let light_dir = model.light_dir();

        assert_eq!(light_mode.len(), nlight);
        assert_eq!(light_type.len(), nlight);
        assert_eq!(light_castshadow.len(), nlight);
        assert_eq!(light_active.len(), nlight);

        for i in 0..nlight {
            assert_eq!(light_mode[i], unsafe { crate::util::force_cast::<_, MjtCamLight>(*model.ffi().light_mode.add(i)) });
            assert_eq!(light_type[i], unsafe { crate::util::force_cast::<_, MjtLightType>(*model.ffi().light_type.add(i)) });

            let raw_shadow = unsafe { *model.ffi().light_castshadow.add(i) };
            assert_eq!(light_castshadow[i], raw_shadow != 0);
            let raw_active = unsafe { *model.ffi().light_active.add(i) };
            assert_eq!(light_active[i], raw_active != 0);

            for j in 0..3 {
                assert_eq!(light_attenuation[i][j], unsafe { *model.ffi().light_attenuation.add(i * 3 + j) });
                assert_eq!(light_ambient[i][j], unsafe { *model.ffi().light_ambient.add(i * 3 + j) });
                assert_eq!(light_diffuse[i][j], unsafe { *model.ffi().light_diffuse.add(i * 3 + j) });
                assert_eq!(light_specular[i][j], unsafe { *model.ffi().light_specular.add(i * 3 + j) });
                assert_eq!(light_pos[i][j], unsafe { *model.ffi().light_pos.add(i * 3 + j) });
                assert_eq!(light_dir[i][j], unsafe { *model.ffi().light_dir.add(i * 3 + j) });
            }
        }

        // Verify known: "lamp_light2" is spot, mode=fixed, castshadow=true
        let l2 = model.light("lamp_light2").unwrap();
        assert_eq!(light_mode[l2.id], MjtCamLight::mjCAMLIGHT_FIXED);
        assert_eq!(light_type[l2.id], MjtLightType::mjLIGHT_SPOT);
        assert_eq!(light_castshadow[l2.id], true);
    }

    /// Verifies [force]-cast for the site arrays: site_type (MjtGeom), site_sameframe (MjtSameFrame),
    /// site_size (&[[MjtNum; 3]]), site_pos (&[[MjtNum; 3]]), site_quat (&[[MjtNum; 4]]),
    /// site_rgba (&[[f32; 4]]).
    #[test]
    fn test_force_cast_site_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nsite = model.ffi().nsite as usize;

        let site_type = model.site_type();
        let site_sameframe = model.site_sameframe();
        let site_size = model.site_size();
        let site_pos = model.site_pos();
        let site_quat = model.site_quat();
        let site_rgba = model.site_rgba();

        assert_eq!(site_type.len(), nsite);
        assert_eq!(site_sameframe.len(), nsite);
        assert_eq!(site_size.len(), nsite);
        assert_eq!(site_pos.len(), nsite);
        assert_eq!(site_quat.len(), nsite);
        assert_eq!(site_rgba.len(), nsite);

        for i in 0..nsite {
            assert_eq!(site_type[i], unsafe { crate::util::force_cast::<_, MjtGeom>(*model.ffi().site_type.add(i)) });
            assert_eq!(site_sameframe[i], unsafe { crate::util::force_cast::<_, MjtSameFrame>(*model.ffi().site_sameframe.add(i)) });

            for j in 0..3 {
                assert_eq!(site_size[i][j], unsafe { *model.ffi().site_size.add(i * 3 + j) });
                assert_eq!(site_pos[i][j], unsafe { *model.ffi().site_pos.add(i * 3 + j) });
            }
            for j in 0..4 {
                assert_eq!(site_quat[i][j], unsafe { *model.ffi().site_quat.add(i * 4 + j) });
                assert_eq!(site_rgba[i][j], unsafe { *model.ffi().site_rgba.add(i * 4 + j) });
            }
        }
    }

    /// Verifies [force]-cast for joint solver arrays: jnt_solref (&[[MjtNum; mjNREF]]),
    /// jnt_solimp (&[[MjtNum; mjNIMP]]), jnt_pos (&[[MjtNum; 3]]), jnt_axis (&[[MjtNum; 3]]),
    /// jnt_range (&[[MjtNum; 2]]).
    #[test]
    fn test_force_cast_joint_model_solver_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let njnt = model.ffi().njnt as usize;

        let jnt_solref = model.jnt_solref();
        let jnt_solimp = model.jnt_solimp();
        let jnt_pos = model.jnt_pos();
        let jnt_axis = model.jnt_axis();
        let jnt_range = model.jnt_range();

        assert_eq!(jnt_solref.len(), njnt);
        assert_eq!(jnt_solimp.len(), njnt);
        assert_eq!(jnt_pos.len(), njnt);
        assert_eq!(jnt_axis.len(), njnt);
        assert_eq!(jnt_range.len(), njnt);

        let nref = mjNREF as usize;
        let nimp = mjNIMP as usize;

        for i in 0..njnt {
            for j in 0..nref {
                assert_eq!(jnt_solref[i][j], unsafe { *model.ffi().jnt_solref.add(i * nref + j) });
            }
            for j in 0..nimp {
                assert_eq!(jnt_solimp[i][j], unsafe { *model.ffi().jnt_solimp.add(i * nimp + j) });
            }
            for j in 0..3 {
                assert_eq!(jnt_pos[i][j], unsafe { *model.ffi().jnt_pos.add(i * 3 + j) });
                assert_eq!(jnt_axis[i][j], unsafe { *model.ffi().jnt_axis.add(i * 3 + j) });
            }
            for j in 0..2 {
                assert_eq!(jnt_range[i][j], unsafe { *model.ffi().jnt_range.add(i * 2 + j) });
            }
        }

        // "rod" has axis="0 1 0", range="0 1"
        let rod = model.joint("rod").unwrap();
        assert_eq!(&jnt_axis[rod.id][..], &[0.0, 1.0, 0.0]);
        assert_eq!(&jnt_range[rod.id][..], &[0.0, 1.0]);
    }

    /// Verifies [force]-cast for view fields with enum types: joint view r#type,
    /// geom view r#type, sensor view r#type + objtype, actuator view trntype + biastype.
    /// Tests that the view_creator! macro correctly passes [force] tokens through.
    #[test]
    fn test_force_cast_view_enum_fields() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();

        // Joint type via view
        let rod_info = model.joint("rod").unwrap();
        let rod_view = rod_info.view(&model);
        assert_eq!(rod_view.r#type[0], MjtJoint::mjJNT_SLIDE);
        assert_eq!(rod_view.limited[0], true);

        // Geom type via view
        let ball2_info = model.geom("ball2").unwrap();
        let ball2_view = ball2_info.view(&model);
        assert_eq!(ball2_view.r#type[0], MjtGeom::mjGEOM_SPHERE);

        // Sensor type via view
        let touch_info = model.sensor("touch").unwrap();
        let touch_view = touch_info.view(&model);
        assert_eq!(touch_view.r#type[0], MjtSensor::mjSENS_TOUCH);
        assert_eq!(touch_view.objtype[0], MjtObj::mjOBJ_SITE);

        // Actuator types via view
        let slider_info = model.actuator("slider").unwrap();
        let slider_view = slider_info.view(&model);
        assert_eq!(slider_view.trntype[0], MjtTrn::mjTRN_JOINT);
        assert_eq!(slider_view.biastype[0], MjtBias::mjBIAS_AFFINE);
        assert_eq!(slider_view.gaintype[0], MjtGain::mjGAIN_FIXED);
        assert_eq!(slider_view.ctrllimited[0], true);

        // Mutable enum roundtrip via view
        let mut slider_view_mut = slider_info.view_mut(&mut model);
        slider_view_mut.gaintype[0] = MjtGain::mjGAIN_AFFINE;
        let slider_view2 = slider_info.view(&model);
        assert_eq!(slider_view2.gaintype[0], MjtGain::mjGAIN_AFFINE);
    }

    /// Verifies mutable model-array roundtrip via info views.
    /// Tests that mutations through `view_mut` are reflected in the flat arrays and FFI.
    #[test]
    fn test_model_view_mut_roundtrip() {
        let mut model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();

        // Mutate body_pos via mutable info view.
        let ball2_info = model.body("ball2").unwrap();
        let ball2_id = ball2_info.id;
        let orig_pos = model.body_pos()[ball2_id];
        assert_eq!(orig_pos, [0.5, 0.0, 0.0]);

        {
            let mut body_view = ball2_info.view_mut(&mut model);
            body_view.pos.copy_from_slice(&[99.0, 88.0, 77.0]);
        }
        assert_eq!(model.body_pos()[ball2_id], [99.0, 88.0, 77.0]);

        // Verify FFI side
        for j in 0..3 {
            let ffi_val = unsafe { *model.ffi().body_pos.add(ball2_id * 3 + j) };
            assert_eq!(ffi_val, [99.0, 88.0, 77.0][j]);
        }

        // Mutate geom_rgba via mutable info view.
        let gs_info = model.geom("green_sphere").unwrap();
        let gs_id = gs_info.id;
        {
            let mut geom_view = gs_info.view_mut(&mut model);
            geom_view.rgba.copy_from_slice(&[1.0f32, 0.0, 0.0, 0.5]);
        }
        assert_eq!(model.geom_rgba()[gs_id], [1.0f32, 0.0, 0.0, 0.5]);
        for j in 0..4 {
            assert_eq!(unsafe { *model.ffi().geom_rgba.add(gs_id * 4 + j) }, [1.0f32, 0.0, 0.0, 0.5][j]);
        }
    }

    /// Verifies [force]-cast sublen_dep variant in model: key_qpos, key_qvel, key_act, key_ctrl.
    /// These have variable inner dimensions dependent on nq, nv, na, nu.
    #[test]
    fn test_force_cast_sublen_dep_key_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let nkey = model.ffi().nkey as usize;
        let nq = model.ffi().nq as usize;
        let nv = model.ffi().nv as usize;
        let na = model.ffi().na as usize;
        let nu = model.ffi().nu as usize;

        assert!(nkey >= 2, "EXAMPLE_MODEL must have at least 2 keyframes");

        let key_qpos = model.key_qpos();
        let key_qvel = model.key_qvel();
        let key_act = model.key_act();
        let key_ctrl = model.key_ctrl();

        // Total flat length must be nkey * inner_dim
        assert_eq!(key_qpos.len(), nkey * nq);
        assert_eq!(key_qvel.len(), nkey * nv);
        assert_eq!(key_act.len(), nkey * na);
        assert_eq!(key_ctrl.len(), nkey * nu);

        // Cross-validate with FFI
        for i in 0..(nkey * nq) {
            assert_eq!(key_qpos[i], unsafe { *model.ffi().key_qpos.add(i) });
        }
        for i in 0..(nkey * nv) {
            assert_eq!(key_qvel[i], unsafe { *model.ffi().key_qvel.add(i) });
        }
        for i in 0..(nkey * nu) {
            assert_eq!(key_ctrl[i], unsafe { *model.ffi().key_ctrl.add(i) });
        }
    }

    /// Tests empty model edge case: model with no optional objects should return
    /// empty slices for all force-cast enum/bool/array fields.
    #[test]
    fn test_force_cast_minimal_model_edge_case() {
        let xml = "<mujoco><worldbody><body><joint type='free'/><geom size='0.1'/></body></worldbody></mujoco>";
        let model = MjModel::from_xml_string(xml).unwrap();

        // No equalities, no tendons, no actuators, no sensors, no cameras, no lights, no textures, no materials
        assert_eq!(model.ffi().neq, 0);
        assert_eq!(model.ffi().ntendon, 0);
        assert_eq!(model.ffi().nu, 0);
        assert_eq!(model.ffi().nsensor, 0);
        assert_eq!(model.ffi().ncam, 0);
        assert_eq!(model.ffi().ntex, 0);
        assert_eq!(model.ffi().nmat, 0);

        // All force-cast slices should be empty
        assert!(model.eq_type().is_empty());
        assert!(model.eq_active0().is_empty());
        assert!(model.tendon_limited().is_empty());
        assert!(model.tendon_rgba().is_empty());
        assert!(model.actuator_trntype().is_empty());
        assert!(model.actuator_ctrllimited().is_empty());
        assert!(model.sensor_type().is_empty());
        assert!(model.cam_mode().is_empty());
        assert!(model.cam_resolution().is_empty());
        assert!(model.tex_type().is_empty());
        assert!(model.mat_texuniform().is_empty());
        assert!(model.mat_rgba().is_empty());

        // But body/geom/joint arrays should work (always have at least world body)
        let nbody = model.ffi().nbody as usize;
        assert!(nbody >= 2);
        assert_eq!(model.body_pos().len(), nbody);
        assert_eq!(model.body_sameframe().len(), nbody);
        assert_eq!(model.jnt_type().len(), model.ffi().njnt as usize);
        assert_eq!(model.geom_type().len(), model.ffi().ngeom as usize);
    }

    /// Verifies [force]-cast pair arrays: pair_solref (&[[MjtNum; mjNREF]]),
    /// pair_friction (&[[MjtNum; 5]]).
    #[test]
    fn test_force_cast_pair_model_arrays() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let npair = model.ffi().npair as usize;

        if npair == 0 {
            return;
        }

        let pair_solref = model.pair_solref();
        let pair_friction = model.pair_friction();
        let pair_solimp = model.pair_solimp();

        assert_eq!(pair_solref.len(), npair);
        assert_eq!(pair_friction.len(), npair);
        assert_eq!(pair_solimp.len(), npair);

        let nref = mjNREF as usize;
        let nimp = mjNIMP as usize;

        for i in 0..npair {
            for j in 0..nref {
                assert_eq!(pair_solref[i][j], unsafe { *model.ffi().pair_solref.add(i * nref + j) });
            }
            for j in 0..5 {
                assert_eq!(pair_friction[i][j], unsafe { *model.ffi().pair_friction.add(i * 5 + j) });
            }
            for j in 0..nimp {
                assert_eq!(pair_solimp[i][j], unsafe { *model.ffi().pair_solimp.add(i * nimp + j) });
            }
        }
    }

    /// Verifies [force]-cast non-aliasing: adjacent joints' solver ref slices
    /// point to different memory.
    #[test]
    fn test_force_cast_model_non_aliasing() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let njnt = model.ffi().njnt as usize;

        if njnt < 2 {
            return;
        }

        let jnt_solref = model.jnt_solref();
        let jnt_type = model.jnt_type();

        // Adjacent elements must not alias
        assert_ne!(jnt_solref[0].as_ptr(), jnt_solref[1].as_ptr());
        assert_ne!(std::ptr::addr_of!(jnt_type[0]), std::ptr::addr_of!(jnt_type[1]));

        // Pointer stride should be exactly mjNREF elements apart
        let ptr_diff = unsafe { jnt_solref[1].as_ptr().offset_from(jnt_solref[0].as_ptr()) };
        assert_eq!(ptr_diff, mjNREF as isize,
            "jnt_solref stride must be mjNREF={}", mjNREF);
    }

    /// Model with 2 mocap bodies and 2 keyframes carrying specific mocap data.
    /// This lets us verify key_mpos and key_mquat array-slice strides and values.
    const MOCAP_MODEL: &str = stringify!(
        <mujoco>
            <worldbody>
                <body name="mocap1" mocap="true" pos="0 0 0">
                    <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
                </body>
                <body name="mocap2" mocap="true" pos="1 0 0">
                    <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
                </body>
                <geom type="plane" size="5 5 0.1"/>
            </worldbody>
            <keyframe>
                <key name="k0"
                     mpos="1.0 2.0 3.0  4.0 5.0 6.0"
                     mquat="0.5 0.5 0.5 0.5  1.0 0.0 0.0 0.0"/>
                <key name="k1"
                     mpos="10.0 20.0 30.0  40.0 50.0 60.0"
                     mquat="0.0 0.0 0.0 1.0  0.0 1.0 0.0 0.0"/>
            </keyframe>
        </mujoco>
    );

    /// Test that `key_mpos` returns the correct slice length and exact values
    /// for a model with 2 mocap bodies and 2 keyframes.
    #[test]
    fn test_key_mpos() {
        let model = MjModel::from_xml_string(MOCAP_MODEL).unwrap();
        let nkey = model.ffi().nkey as usize;
        let nmocap = model.ffi().nmocap as usize;

        assert_eq!(nkey, 2, "expected 2 keyframes");
        assert_eq!(nmocap, 2, "expected 2 mocap bodies");

        let mpos = model.key_mpos();
        assert_eq!(mpos.len(), nkey * nmocap * 3,
            "key_mpos length must be nkey * nmocap * 3 = {}", nkey * nmocap * 3);

        // Keyframe 0: mocap1 at (1,2,3), mocap2 at (4,5,6)
        let k0 = &mpos[..nmocap * 3];
        let expected_k0: &[f64] = &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        assert_eq!(k0.len(), expected_k0.len());
        for (&a, &b) in k0.iter().zip(expected_k0.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-10);
        }

        // Keyframe 1: mocap1 at (10,20,30), mocap2 at (40,50,60)
        let k1 = &mpos[nmocap * 3..];
        let expected_k1: &[f64] = &[10.0, 20.0, 30.0, 40.0, 50.0, 60.0];
        assert_eq!(k1.len(), expected_k1.len());
        for (&a, &b) in k1.iter().zip(expected_k1.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-10);
        }
    }

    /// Test that `key_mquat` returns the correct slice length and exact values
    /// for a model with 2 mocap bodies and 2 keyframes.
    #[test]
    fn test_key_mquat() {
        let model = MjModel::from_xml_string(MOCAP_MODEL).unwrap();
        let nkey = model.ffi().nkey as usize;
        let nmocap = model.ffi().nmocap as usize;

        assert_eq!(nkey, 2, "expected 2 keyframes");
        assert_eq!(nmocap, 2, "expected 2 mocap bodies");

        let mquat = model.key_mquat();
        assert_eq!(mquat.len(), nkey * nmocap * 4,
            "key_mquat length must be nkey * nmocap * 4 = {}", nkey * nmocap * 4);

        // Keyframe 0: mocap1 quat (0.5,0.5,0.5,0.5), mocap2 quat (1,0,0,0)
        let k0 = &mquat[..nmocap * 4];
        let expected_k0: &[f64] = &[0.5, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0, 0.0];
        assert_eq!(k0.len(), expected_k0.len());
        for (&a, &b) in k0.iter().zip(expected_k0.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-10);
        }

        // Keyframe 1: mocap1 quat (0,0,0,1), mocap2 quat (0,1,0,0)
        let k1 = &mquat[nmocap * 4..];
        let expected_k1: &[f64] = &[0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0];
        assert_eq!(k1.len(), expected_k1.len());
        for (&a, &b) in k1.iter().zip(expected_k1.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-10);
        }
    }

    /// Test that key_mpos is accessible through the key view as well,
    /// and that both paths yield the same data.
    #[test]
    fn test_key_mpos_view_consistency() {
        let model = MjModel::from_xml_string(MOCAP_MODEL).unwrap();
        let nmocap = model.ffi().nmocap as usize;

        let info_k0 = model.key("k0").unwrap();
        let view_k0 = info_k0.view(&model);

        // key view mpos slice should equal array accessor key_mpos[0..nmocap*3]
        let array_k0 = &model.key_mpos()[..nmocap * 3];
        assert_eq!(
            &view_k0.mpos[..nmocap * 3], array_k0,
            "key view mpos and key_mpos array accessor must return identical data"
        );

        let info_k1 = model.key("k1").unwrap();
        let view_k1 = info_k1.view(&model);
        let array_k1 = &model.key_mpos()[nmocap * 3..];
        assert_eq!(
            &view_k1.mpos[..nmocap * 3], array_k1,
            "key view mpos and key_mpos array accessor must return identical data for key 1"
        );
    }

    /// Test that key_mpos/key_mquat return empty slices for models with no mocap bodies.
    #[test]
    fn test_key_mpos_mquat_no_mocap() {
        // EXAMPLE_MODEL has keyframes but no mocap bodies
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        assert_eq!(model.ffi().nmocap, 0, "EXAMPLE_MODEL should have no mocap bodies");
        assert!(model.ffi().nkey > 0, "EXAMPLE_MODEL should have keyframes");

        assert_eq!(model.key_mpos().len(), 0,
            "key_mpos must be empty when nmocap == 0");
        assert_eq!(model.key_mquat().len(), 0,
            "key_mquat must be empty when nmocap == 0");
    }

    /// Loading invalid XML must return an Err whose message is non-empty.
    #[test]
    fn test_from_xml_string_invalid() {
        let result = MjModel::from_xml_string("<this is not valid mujoco xml>");
        assert!(result.is_err(), "loading invalid XML must return Err");
        let msg = result.unwrap_err().to_string();
        assert!(!msg.is_empty(), "error message must not be empty for invalid XML");
    }

    /// Verifies the new mesh view fields added in MuJoCo 3.8.0:
    /// read-only index fields (`normaladr`, `normalnum`, etc.) have length 1,
    /// and the read-write fields (`scale`, `pos`, `quat`) have the right length
    /// and survive a roundtrip write.
    #[test]
    fn test_mesh_view_new_fields() {
        const MESH_MODEL: &str = "<mujoco>\
          <asset>\
            <mesh name=\"cube\" vertex=\"-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5  0.5 -0.5  0.5  0.5 -0.5  \
                                         -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5  0.5  0.5  0.5  0.5  0.5\"/>\
          </asset>\
          <worldbody>\
            <geom type=\"mesh\" mesh=\"cube\"/>\
          </worldbody>\
        </mujoco>";

        let mut model = MjModel::from_xml_string(MESH_MODEL).unwrap();
        let mesh_info = model.mesh("cube").unwrap();

        let view = mesh_info.view(&model);

        /* Verify field dimensions for read-write fields */
        assert_eq!(view.scale.len(), 3);
        assert_eq!(view.pos.len(), 3);
        assert_eq!(view.quat.len(), 4);

        /* Verify field dimensions for read-only index fields */
        assert_eq!(view.normaladr.len(), 1);
        assert_eq!(view.normalnum.len(), 1);
        assert_eq!(view.texcoordnum.len(), 1);
        assert_eq!(view.bvhadr.len(), 1);
        assert_eq!(view.bvhnum.len(), 1);
        assert_eq!(view.octadr.len(), 1);
        assert_eq!(view.octnum.len(), 1);
        assert_eq!(view.pathadr.len(), 1);
        assert_eq!(view.polynum.len(), 1);
        assert_eq!(view.polyadr.len(), 1);

        /* Verify write-read roundtrip for scale */
        let mut view_mut = mesh_info.view_mut(&mut model);
        view_mut.scale[0] = 2.0;
        view_mut.scale[1] = 3.0;
        view_mut.scale[2] = 4.0;

        let view2 = mesh_info.view(&model);
        assert_eq!(view2.scale[0], 2.0);
        assert_eq!(view2.scale[1], 3.0);
        assert_eq!(view2.scale[2], 4.0);
    }

    /// Verifies that `flex_bandwidth`, `flex_cellnum`, and `flex_stiffnessadr`
    /// return empty slices when the model contains no flex bodies.
    #[test]
    fn test_flex_array_slices_empty_for_non_flex_model() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        assert_eq!(model.ffi().nflex, 0);
        assert_eq!(model.flex_bandwidth().len(), 0);
        assert_eq!(model.flex_cellnum().len(), 0);
        assert_eq!(model.flex_stiffnessadr().len(), 0);
    }

    /// Tests the wrapper of `mj_maxContact` ([`MjModel::max_contacts`]).
    #[test]
    fn test_max_contacts() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let geom1 = model.name_to_id(MjtObj::mjOBJ_GEOM, "green_sphere").unwrap();
        let geom2 = model.name_to_id(MjtObj::mjOBJ_GEOM, "ball2").unwrap();

        let mc = model.max_contacts(geom1, geom2, None);  // pull margin from model.
        assert_eq!(mc, 1);

        let mc = model.max_contacts(geom1, geom2, Some(false));
        assert_eq!(mc, 1);

        // Spheres always have one contact, regardless of margin.
        let mc = model.max_contacts(geom1, geom2, Some(true));
        assert_eq!(mc, 1);

        // Test invalid geom index.
        assert!( model.try_max_contacts(999, geom2, Some(true)).is_err());
    }
}
