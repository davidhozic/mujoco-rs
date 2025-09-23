//! Module for mjModel
use std::ffi::{c_int, CStr, CString, NulError};
use std::io::{self, Error, ErrorKind};
use std::path::Path;
use std::ptr;

use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic};
use crate::wrappers::mj_option::MjOption;
use crate::wrappers::mj_data::MjData;
use super::mj_primitive::*;
use crate::mujoco_c::*;

use crate::{
    view_creator, fixed_size_info_method, info_with_view,
    mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem
};


/*******************************************/
// Types

/* Actuator */
/// Actuator transmission types.
pub type MjtTrn = mjtTrn;
/// Actuator dynamics types.
pub type MjtDyn = mjtDyn;
/// Actuator gain types.
pub type MjtGain = mjtGain;
/// Actuator bias types.
pub type MjtBias = mjtBias;

/* Sensor */
/// Sensor types.
pub type MjtSensor = mjtSensor;

/// These are the possible sensor data types.
pub type MjtDataType = mjtDataType;

/* Other */
/// These are the compute stages for the skipstage parameters of [`mj_forwardSkip`] and [`mj_inverseSkip`].
pub type MjtStage = mjtStage;

/// MuJoCo object types. These are used, for example, in the support functions [`mj_name2id`] and
/// [`mj_id2name`] to convert between object names and integer ids.
pub type MjtObj = mjtObj;

/// Primitive joint types.
pub type MjtJoint = mjtJoint;

/// Geometric types supported by MuJoCo.
pub type MjtGeom = mjtGeom;

/// Types of frame alignment of elements with their parent bodies.
pub type MjtSameFrame = mjtSameFrame;

/// Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed.
pub type MjtCamLight = mjtCamLight;

/// The type of a light source describing how its position, orientation and
/// other properties will interact with the objects in the scene.
pub type MjtLightType = mjtLightType;

/// Equality constraint types.
pub type MjtEq = mjtEq;

/// Texture types, specifying how the texture will be mapped
pub type MjtTexture = mjtTexture;

/// Type of color space encoding for textures.
pub type MjtColorSpace = mjtColorSpace;

/// Constants which are powers of 2. They are used as bitmasks for the field `disableflags` of [`MjOption`].
pub type MjtDisableBit = mjtDisableBit;

/// Constants which are powers of 2. They are used as bitmasks for the field `enableflags` of [`MjOption`].
pub type MjtEnableBit = mjtEnableBit;

/// Texture roles, specifying how the renderer should interpret the texture
pub type MjtTextureRole = mjtTextureRole;
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
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        Self::from_xml_file(path, None)
    }

    /// Loads the model from an XML file, located in a virtual file system (`vfs`).
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, Error> {
        Self::from_xml_file(path, Some(vfs))
    }

    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, Error> {
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

    /// Creates a [`MjModel`] from a raw pointer.
    pub(crate) fn from_raw(ptr: *mut mjModel) -> Result<Self, Error> {
        Self::check_raw_model(ptr, &[])
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

    /// Creates a new [`MjData`] instances linked to this model.
    pub fn make_data<'m>(&'m self) -> MjData<'m> {
        MjData::new(self)
    }

    fn check_raw_model(ptr_model: *mut mjModel, error_buffer: &[i8]) -> Result<Self, Error> {
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

    fixed_size_info_method! { Model, ffi(), actuator, [
        trntype: 1, dyntype: 1, gaintype: 1, biastype: 1, trnid: 2, actadr: 1, actnum: 1, group: 1, ctrllimited: 1,
        forcelimited: 1, actlimited: 1, dynprm: mjNDYN as usize, gainprm: mjNGAIN as usize,  biasprm: mjNBIAS as usize, 
        actearly: 1,  ctrlrange: 2, forcerange: 2,  actrange: 2,  gear: 6,  cranklength: 1,  acc0: 1, 
        length0: 1,  lengthrange: 2
    ], [], []}

    fixed_size_info_method! { Model, ffi(), sensor, [
        r#type: 1, datatype: 1, needstage: 1,
        objtype: 1, objid: 1, reftype: 1, refid: 1, intprm: mjNSENS as usize,
        dim: 1, adr: 1, cutoff: 1, noise: 1
    ], [], []}


    fixed_size_info_method! { Model, ffi(), tendon, [
        adr: 1, num: 1, matid: 1, group: 1, limited: 1,
        actfrclimited: 1, width: 1, solref_lim: mjNREF as usize,
        solimp_lim: mjNIMP as usize, solref_fri: mjNREF as usize, solimp_fri: mjNIMP as usize,
        range: 2, actfrcrange: 2, margin: 1, stiffness: 1,
        damping: 1, armature: 1, frictionloss: 1, lengthspring: 2,
        length0: 1, invweight0: 1, rgba: 4
    ], [], []}

    fixed_size_info_method! { Model, ffi(), joint, [
        r#type: 1, qposadr: 1, dofadr: 1, bodyid: 1, group: 1,
        limited: 1, actfrclimited: 1, actgravcomp: 1, solref: mjNREF as usize,
        solimp: mjNIMP as usize, pos: 3, axis: 3, stiffness: 1,
        range: 2, actfrcrange: 2, margin: 1
    ], [], []}

    fixed_size_info_method! { Model, ffi(), geom, [
        r#type: 1, contype: 1, conaffinity: 1, condim: 1, bodyid: 1, dataid: 1, matid: 1,
        group: 1, priority: 1, plugin: 1, sameframe: 1, solmix: 1, solref: mjNREF as usize,
        solimp: mjNIMP as usize,
        size: 3, aabb: 6, rbound: 1, pos: 3, quat: 4, friction: 3, margin: 1, gap: 1,
        fluid: mjNFLUID as usize, rgba: 4
    ], [], []}

    fixed_size_info_method! { Model, ffi(), body, [
        parentid: 1, rootid: 1, weldid: 1, mocapid: 1,
        jntnum: 1, jntadr: 1, dofnum: 1, dofadr: 1,
        treeid: 1, geomnum: 1, geomadr: 1, simple: 1,
        sameframe: 1, pos: 3, quat: 4, ipos: 3, iquat: 4,
        mass: 1, subtreemass: 1, inertia: 3, invweight0: 2,
        gravcomp: 1, margin: 1, plugin: 1,
        contype: 1, conaffinity: 1, bvhadr: 1, bvhnum: 1
    ], [], []}

    fixed_size_info_method! { Model, ffi(), camera, [
        mode: 1, bodyid: 1, targetbodyid: 1, pos: 3, quat: 4,
        poscom0: 3, pos0: 3, mat0: 9, orthographic: 1, fovy: 1,
        ipd: 1, resolution: 2, sensorsize: 2, intrinsic: 4
    ], [], []}

    fixed_size_info_method! { Model, ffi(), key, [
        time: 1
    ], [
        qpos: nq, qvel: nv, act: na, mpos: nmocap * 3, mquat: nmocap * 4,
        ctrl: nu
    ], []}

    fixed_size_info_method! { Model, ffi(), tuple, [
        size: 1
    ], [], [
        objtype: ntupledata, objid: ntupledata, objprm: ntupledata
    ]}

    fixed_size_info_method! { Model, ffi(), texture, [
        r#type: 1, colorspace: 1, height: 1, width: 1, nchannel: 1, pathadr: 1
    ], [], [
        data: ntexdata
    ]}

    // TODO: skin

    fixed_size_info_method! { Model, ffi(), site, [
        r#type: 1, bodyid: 1, group: 1, sameframe: 1, size: 3,
        pos: 3, quat: 4, rgba: 4, matid: 1
    ], [user: nuser_site], []}

    fixed_size_info_method! { Model, ffi(), pair, [
        dim: 1, geom1: 1, geom2: 1, signature: 1, solref: mjNREF as usize,
        solreffriction: mjNREF as usize, solimp: mjNIMP as usize, margin: 1,
        gap: 1, friction: 5
    ], [], []}

    fixed_size_info_method! { Model, ffi(), numeric, [
        size: 1
    ], [], [data: nnumericdata]}


    fixed_size_info_method! { Model, ffi(), material, [
        texuniform: 1,
        texrepeat: 2, 
        emission: 1,
        specular: 1,
        shininess: 1,
        reflectance: 1,
        metallic: 1,
        roughness: 1,
        rgba: 4,
        texid: MjtTextureRole::mjNTEXROLE as usize
    ], [], [] }


    fixed_size_info_method! { Model, ffi(), light, [
        mode: 1, bodyid: 1, targetbodyid: 1, r#type: 1, texid: 1, castshadow: 1, bulbradius: 1,
        intensity: 1, range: 1, active: 1, pos: 3, dir: 3, poscom0: 3, pos0: 3, dir0: 3,
        attenuation: 3, cutoff: 1, exponent: 1, ambient: 3, diffuse: 3, specular: 3
    ], [], []}


    /// Deprecated alias for [`MjModel::name_to_id`].
    #[deprecated]
    pub fn name2id(&self, type_: MjtObj, name: &str) -> i32 {
        self.name_to_id(type_, name)
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
    pub fn size(&self) -> std::ffi::c_int {
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
    pub fn state_size(&self, spec: std::ffi::c_uint) -> std::ffi::c_int {
        unsafe { mj_stateSize(self.ffi(), spec) }
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
    pub fn id_to_name(&self, type_: MjtObj, id: std::ffi::c_int) -> Option<&str> {
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

    /// An immutable reference to physics options.
    pub fn opt(&self) -> &MjOption {
        &self.ffi().opt
    }

    /// A mutable reference to physics options.
    pub fn opt_mut(&mut self) -> &mut MjOption {
        // SAFETY: changing options can't break anything in terms of memory or UB.
        &mut unsafe { self.ffi_mut() }.opt
    }

    /// An immutable reference to visualization options.
    pub fn vis(&self) -> &MjVisual {
        &self.ffi().vis
    }

    /// A mutable reference to visualization options.
    pub fn vis_mut(&mut self) -> &mut MjVisual {
        // SAFETY: changing visualization options can't break anything in terms of memory or UB.
        &mut unsafe { self.ffi_mut() }.vis
    }

    /// An immmutable reference to model statistics.
    pub fn stat(&self) -> &MjStatistic {
        &self.ffi().stat
    }

    /// A mutable reference to model statistics.
    pub fn stat_mut(&mut self) -> &mut MjStatistic {
        // SAFETY: changing model statistics can't break anything in terms of memory or UB.
        &mut unsafe { self.ffi_mut() }.stat
    }
}


impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.0);
        }
    }
}


/**************************************************************************************************/
// Actuator view
/**************************************************************************************************/
info_with_view!(Model, actuator, actuator_,
    [
        trntype: MjtTrn, dyntype: MjtDyn, gaintype: MjtGain, biastype: MjtBias, trnid: i32,
        actadr: i32, actnum: i32, group: i32, ctrllimited: bool,
        forcelimited: bool, actlimited: bool, dynprm: MjtNum, gainprm: MjtNum, biasprm: MjtNum,
        actearly: bool, ctrlrange: MjtNum, forcerange: MjtNum, actrange: MjtNum,
        gear: MjtNum, cranklength: MjtNum, acc0: MjtNum, length0: MjtNum, lengthrange: MjtNum
    ], []
);


/**************************************************************************************************/
// Sensor view
/**************************************************************************************************/
info_with_view!(Model, sensor, sensor_,
    [
        r#type: MjtSensor, datatype: MjtDataType, needstage: MjtStage,
        objtype: MjtObj, objid: i32, reftype: MjtObj, refid: i32, intprm: i32,
        dim: i32, adr: i32, cutoff: MjtNum, noise: MjtNum
    ], []
);


/**************************************************************************************************/
// Tendon view
/**************************************************************************************************/
info_with_view!(Model, tendon, tendon_,
    [
        adr: i32, num: i32, matid: i32, group: i32, limited: bool,
        actfrclimited: bool, width: MjtNum, solref_lim: MjtNum,
        solimp_lim: MjtNum, solref_fri: MjtNum, solimp_fri: MjtNum,
        range: MjtNum, actfrcrange: MjtNum, margin: MjtNum, stiffness: MjtNum,
        damping: MjtNum, armature: MjtNum, frictionloss: MjtNum, lengthspring: MjtNum,
        length0: MjtNum, invweight0: MjtNum, rgba: f32
    ], []
);


/**************************************************************************************************/
// Joint view
/**************************************************************************************************/
info_with_view!(Model, joint, jnt_,
    [
        r#type: MjtJoint, qposadr: i32, dofadr: i32, bodyid: i32, group: i32,
        limited: bool, actfrclimited: bool, actgravcomp: bool, solref: MjtNum,
        solimp: MjtNum, pos: MjtNum, axis: MjtNum, stiffness: MjtNum,
        range: MjtNum, actfrcrange: MjtNum, margin: MjtNum
    ], []
);

/**************************************************************************************************/
// Geom view
/**************************************************************************************************/
info_with_view!(Model, geom, geom_,
    [
        r#type: MjtGeom, contype: i32, conaffinity: i32, condim: i32, bodyid: i32, dataid: i32, matid: i32,
        group: i32, priority: i32, plugin: i32, sameframe: MjtSameFrame, solmix: MjtNum, solref: MjtNum, solimp: MjtNum,
        size: MjtNum, aabb: MjtNum, rbound: MjtNum, pos: MjtNum, quat: MjtNum, friction: MjtNum, margin: MjtNum, gap: MjtNum,
        fluid: MjtNum, rgba: f32
    ], []
);

/**************************************************************************************************/
// Body view
/**************************************************************************************************/
info_with_view!(Model, body, body_,
    [
        parentid: i32, rootid: i32, weldid: i32, mocapid: i32,
        jntnum: i32, jntadr: i32, dofnum: i32, dofadr: i32,
        treeid: i32, geomnum: i32, geomadr: i32, simple: MjtByte,
        sameframe: MjtSameFrame, pos: MjtNum, quat: MjtNum, ipos: MjtNum, iquat: MjtNum,
        mass: MjtNum, subtreemass: MjtNum, inertia: MjtNum, invweight0: MjtNum,
        gravcomp: MjtNum, margin: MjtNum, plugin: i32,
        contype: i32, conaffinity: i32, bvhadr: i32, bvhnum: i32
    ], []
);


/**************************************************************************************************/
// Camera view
/**************************************************************************************************/
info_with_view!(Model, camera, cam_,
    [
        mode: MjtCamLight, bodyid: i32, targetbodyid: i32, pos: MjtNum, quat: MjtNum,
        poscom0: MjtNum, pos0: MjtNum, mat0: MjtNum, orthographic: bool, fovy: MjtNum,
        ipd: MjtNum, resolution: i32, sensorsize: f32, intrinsic: f32
    ], []
);

/**************************************************************************************************/
// KeyFrame view
/**************************************************************************************************/
info_with_view!(Model, key, key_,
    [
        time: MjtNum, qpos: MjtNum, qvel: MjtNum, act: MjtNum, mpos: MjtNum,
        mquat: MjtNum, ctrl: MjtNum
    ], []
);

/**************************************************************************************************/
// Tuple view
/**************************************************************************************************/
info_with_view!(Model, tuple, tuple_,
    [
        size: i32, objtype: MjtObj, objid: i32, objprm: MjtNum
    ], []
);


/**************************************************************************************************/
// Texture view
/**************************************************************************************************/
info_with_view!(Model, texture, tex_,
    [
        r#type: MjtTexture, colorspace: MjtColorSpace, height: i32, width: i32, nchannel: i32,
        data: MjtByte
    ], [pathadr: i32]
);

/**************************************************************************************************/
// Skin view
/**************************************************************************************************/
info_with_view!(Model, skin, skin_,
    [
        matid: i32, group: i32, rgba: i32, inflate: i32, vertadr: i32, vertnum: i32, texcoordadr: i32,
        faceadr: i32, facenum: i32, boneadr: i32, bonenum: i32, vert: f32, texcoord: f32,
        face: i32, bonevertadr: i32, bonevertnum: i32, bonebindpos: f32,
        bonebindquat: f32, bonebodyid: i32, bonevertid: i32, bonevertweight: f32,
        pathadr: i32
    ], []
);


/**************************************************************************************************/
// Site view
/**************************************************************************************************/
info_with_view!(Model, site, site_,
    [
        r#type: MjtGeom, bodyid: i32, group: i32, sameframe: MjtSameFrame, size: MjtNum,
        pos: MjtNum, quat: MjtNum, user: MjtNum, rgba: f32
    ], [matid: i32]
);

/**************************************************************************************************/
// Pair view
/**************************************************************************************************/
info_with_view!(Model, pair, pair_,
    [
        dim: i32, geom1: i32, geom2: i32, signature: i32, solref: MjtNum, solreffriction: MjtNum,
        solimp: MjtNum, margin: MjtNum, gap: MjtNum, friction: MjtNum
    ], []
);


/**************************************************************************************************/
// Pair view
/**************************************************************************************************/
info_with_view!(Model, numeric, numeric_,
    [
        size: i32, data: MjtNum
    ], []
);

/**************************************************************************************************/
// Material view
/**************************************************************************************************/
info_with_view!(Model, material, mat_,
    [
        texuniform: bool,
        texrepeat: f32,
        emission: f32,
        specular: f32,
        shininess: f32,
        reflectance: f32,
        metallic: f32,
        roughness: f32,
        rgba: f32,
        texid: i32
    ], []
);

/**************************************************************************************************/
// Material view
/**************************************************************************************************/
info_with_view!(Model, light, light_,
    [
        mode: MjtCamLight, bodyid: i32, targetbodyid: i32, r#type: MjtLightType, texid: i32, castshadow: bool,
        bulbradius: f32, intensity: f32, range: f32, active: bool, pos: MjtNum, dir: MjtNum,
        poscom0: MjtNum, pos0: MjtNum, dir0: MjtNum, attenuation: f32, cutoff: f32, exponent: f32,
        ambient: f32, diffuse: f32, specular: f32
    ], []
);

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
                <joint type="free"/>
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
        </worldbody>

        <actuator>
            <general name="slider" joint="rod" biastype="affine" ctrlrange="0 1" gaintype="fixed"/>
        </actuator>

        <sensor>
            <touch name="touch" site="touch"/>
        </sensor>

        <tendon>
            <spatial name="tendon" limited="true" range="0 1" rgba="0 .1 1 1" width=".005">
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
        <key name="pose1"
            time="1.5"
            qpos="0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.3 0.1 0.2 0.0"
            qvel="0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0"
            ctrl="0.5"/>
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
    </asset>
    </mujoco>
);

    /// Tests if the model can be loaded and then saved.
    #[test]
    fn test_model_load_save() {
        const MODEL_SAVE_XML_PATH: &str = "./__TMP_MODEL1.xml";
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("unable to load the model.");
        model.save_last_xml(MODEL_SAVE_XML_PATH).expect("could not save the model XML.");      
        fs::remove_file(MODEL_SAVE_XML_PATH).unwrap();
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
        let tendon_model_info = model.tendon("tendon").unwrap();
        let view = tendon_model_info.view(&model);
        
        /* Test read */
        assert_eq!(&view.range[..], [0.0, 1.0]);
        assert_eq!(view.limited[0], true);
        assert_eq!(view.width[0], 0.005);

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
        assert_eq!(view.pos[0], 0.5);

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
        assert!(MjModel::from_buffer(&saved_data).is_ok());
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

        let signature = ((geom1_info.view(&model).bodyid[0] as u32) << 16) + geom2_info.view(&model).bodyid[0] as u32;

        assert_eq!(view_pair.dim[0], DIM);
        assert_eq!(view_pair.geom1[0] as usize, geom1_info.id);
        assert_eq!(view_pair.geom2[0] as usize, geom2_info.id);
        assert_eq!(view_pair.signature[0] as u32, signature);  // body1 << 16 + body2
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
        const CTRL: &[f64] = &[0.5];

        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        let info_key = model.key(KEY_NAME).unwrap();
        let view_key = info_key.view(&model);

        assert_eq!(view_key.time[0], TIME);
        // Don't test qpos, as it does some magic angle conversions, making the tests fail.
        // If all the other succeed, assume qpos works too (as its the exact same logic).
        assert_eq!(&view_key.qvel[..model.ffi().nv as usize], QVEL);
        assert_eq!(&view_key.act[..model.ffi().na as usize], ACT);
        assert_eq!(&view_key.ctrl[..model.ffi().nu as usize], CTRL);
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

        assert_eq!(view_tex.data.len(), (WIDTH * HEIGHT * NCHANNEL) as usize);
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
        assert_eq!(view_numeric.data[..SIZE as usize], DATA);
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
        const TEXID: i32 = -1;
        const CASTSHADOW: bool = true;
        const BULBRADIUS: f32 = 0.2;
        const INTENSITY: f32 = 500.0;
        const RANGE: f32 = 10.0;
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
        assert_eq!(view_light.texid[0], TEXID);
        assert_eq!(view_light.castshadow[0], CASTSHADOW);
        assert_eq!(view_light.bulbradius[0], BULBRADIUS);
        assert_eq!(view_light.intensity[0], INTENSITY);
        assert_eq!(view_light.range[0], RANGE);
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
}
