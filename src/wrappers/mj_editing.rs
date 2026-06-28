//! Definitions related to model editing.
use std::ffi::{c_char, c_int, CStr, CString};
use std::marker::PhantomData;
use std::ptr::{self, NonNull};
use std::path::Path;
use crate::error::MjEditError;

#[macro_use]
mod utility;
use utility::*;

mod traits;
pub use traits::*;

mod default;
pub use default::*;

use super::mj_model::{
    MjModel, MjtObj, MjtGeom, MjtJoint, MjtCamLight,
    MjtLightType, MjtSensor, MjtDataType, MjtGain,
    MjtBias, MjtDyn, MjtEq, MjtTexture, MjtColorSpace,
    MjtTrn, MjtStage, MjtFlexSelf, MjtProjection,
    MjtSleepPolicy, MjtWrap, MjtTextureRole, MjtCubeFace
};
use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic, MjLROpt};
use super::mj_option::MjOption;
use super::mj_primitive::*;
use crate::mujoco_c::*;

use crate::getter_setter;

// Re-export with lowercase 'f' to fix method generation
use crate::mujoco_c::{mjs_addHField as mjs_addHfield, mjsHField as mjsHfield, mjs_asHField as mjs_asHfield};
use crate::util::{assert_mujoco_version, ERROR_BUF_LEN};

/* Validation helpers */
/// Validates that an object-type value is a real object type, i.e. an [`MjtObj`] discriminant below
/// [`MjtObj::mjNOBJECT`]. Meta variants (`mjOBJ_FRAME`, `mjOBJ_DEFAULT`, `mjOBJ_MODEL`) and
/// `mjNOBJECT` itself are rejected because the model compiler uses such values as an unchecked
/// index into a `mjNOBJECT`-sized array (via `mjCModel::FindObject`), which would read out of
/// bounds. Used for any safe setter whose value reaches `FindObject` during `compile()`.
fn check_objtype(t: MjtObj) -> Result<(), MjEditError> {
    if (t as i32) < (MjtObj::mjNOBJECT as i32) {
        Ok(())
    } else {
        Err(MjEditError::InvalidParameter(format!(
            "object type must be a real MjtObj below MjtObj::mjNOBJECT, got {t:?}"
        )))
    }
}

/// Validates that a custom-numeric array size is non-negative.
///
/// A negative `size` slips through every guard in the model compiler's `mjCNumeric::Compile`
/// (the `size < data_.size()` check is gated on a non-empty init array, and the zero checks treat
/// negatives as non-zero), so it undersizes the shared `numeric_data` allocation while another
/// numeric's zero-fill loop still runs up to its own positive size, writing out of bounds.
fn check_numeric_size(size: i32) -> Result<(), MjEditError> {
    if size < 0 {
        Err(MjEditError::InvalidParameter(format!(
            "numeric size must be non-negative, got {size}"
        )))
    } else {
        Ok(())
    }
}

/* Types */
/// Type of inertia inference.
pub type MjtGeomInertia = mjtGeomInertia;

/// Type of mesh inertia.
pub type MjtMeshInertia = mjtMeshInertia;

/// Type of built-in procedural texture.
pub type MjtBuiltin = mjtBuiltin;

/// Type of built-in procedural mesh.
pub type MjtMeshBuiltin = mjtMeshBuiltin;

/// Mark type for procedural textures.
pub type MjtMark = mjtMark;

/// Type of limit specification.
pub type MjtLimited = mjtLimited;

/// Whether to align free joints with the inertial frame.
pub type MjtAlignFree = mjtAlignFree;

/// Whether to infer body inertias from child geoms.
pub type MjtInertiaFromGeom = mjtInertiaFromGeom;

/// Type of orientation specifier.
pub type MjtOrientation = mjtOrientation;

/// Compiler timing categories, used in `mjs_getTimer`.
pub type MjtCTimer = mjtCTimer;
/*******************************************************/

/******************************
** Type aliases
******************************/
/// Alternative orientation specifiers.
pub type MjsOrientation = mjsOrientation;
impl MjsOrientation {
    /// Sets orientation in Euler space.
    pub fn set_euler(&mut self, angle: &[f64; 3]) {
        self.type_ = MjtOrientation::mjORIENTATION_EULER;
        self.euler = *angle;
    }

    /// Sets orientation in axis angle space.
    pub fn set_axis_angle(&mut self, angle: &[f64; 4]) {
        self.type_ = MjtOrientation::mjORIENTATION_AXISANGLE;
        self.axisangle = *angle;
    }

    /// Sets orientation in XY axes space.
    pub fn set_xy_axis(&mut self, angle: &[f64; 6]) {
        self.type_ = MjtOrientation::mjORIENTATION_XYAXES;
        self.xyaxes = *angle;
    }

    /// Sets orientation in Z axis space.
    pub fn set_z_axis(&mut self, angle: &[f64; 3]) {
        self.type_ = MjtOrientation::mjORIENTATION_ZAXIS;
        self.zaxis = *angle;
    }

    /// Changes the orientation mode to quaternions. The orientation must
    /// be specified via the main angle attribute, not through [`MjsOrientation`].
    pub fn switch_quat(&mut self) {
        self.type_ = MjtOrientation::mjORIENTATION_QUAT;
    }
}

/// Compiler options.
pub type MjsCompiler = mjsCompiler;
impl MjsCompiler {
    getter_setter! {[&] with, get, set, [
        autolimits: bool;              "infer \"limited\" attribute based on range.";
        balanceinertia: bool;          "automatically impose A + B >= C rule.";
        fitaabb: bool;                 "meshfit to aabb instead of inertia box.";
        degree: bool;                  "angles in radians or degrees.";
        discardvisual: bool;           "discard visual geoms in parser.";
        usethread: bool;               "use multiple threads to speed up compiler.";
        fusestatic: bool;              "fuse static bodies with parent.";
        saveinertial: bool;            "save explicit inertial clause for all bodies to XML.";
        alignfree: bool;               "align free joints with inertial frame.";
    ]}

    getter_setter! {[&] with, get, set, [
        boundmass: f64;                "enforce minimum body mass.";
        boundinertia: f64;             "enforce minimum body diagonal inertia.";
        settotalmass: f64;             "rescale masses and inertias; <=0: ignore.";
    ]}

    getter_setter! {[&] with, get, set, [
        inertiafromgeom: MjtInertiaFromGeom [force];  "use geom inertias.";
    ]}

    getter_setter! {[&] with, get, [
        inertiagrouprange: &[i32; 2];       "range of geom groups used to compute inertia.";
        eulerseq: &[c_char; 3];             "sequence for euler rotations.";
        LRopt: &MjLROpt;                    "options for lengthrange computation.";
    ]}

    string_set_get_with! {[&]
        meshdir;        "mesh and hfield directory.";
        texturedir;     "texture directory.";
    }
}

/// Authored-field tracking bitmasks for [`mjModel`] structs.
///
/// Each field records, as a bitmask, which attributes of the corresponding
/// section were explicitly authored (set) in the specification. These are
/// maintained by the compiler and used to resolve conflicts during attachment;
/// they are exposed read-only.
pub type MjsAuthored = mjsAuthored;
impl MjsAuthored {
    getter_setter! {get, [
        option: u64;                "authored mjOption fields.";
        disableflags: i32;          "individual authored disable flags.";
        enableflags: i32;           "individual authored enable flags.";
        disableactuator: i32;       "individual authored actuator groups.";
        visual_global: u64;         "authored visual.global fields.";
        visual_quality: u64;        "authored visual.quality fields.";
        visual_headlight: u64;      "authored visual.headlight fields.";
        visual_map: u64;            "authored visual.map fields.";
        visual_scale: u64;          "authored visual.scale fields.";
        visual_rgba: u64;           "authored visual.rgba fields.";
    ]}
}

/***************************
** Model Specification
***************************/
/// Model specification. This wraps the FFI type [`mjSpec`] internally.
#[derive(Debug)]
pub struct MjSpec(NonNull<mjSpec>);

// SAFETY: `MjSpec` owns its `mjSpec` exclusively, so moving it between threads transfers
// sole ownership and cannot race.
//
// It is intentionally NOT `Sync`. `clone`/`try_clone` produce a faithful, independent copy, but
// the C++ copy constructor behind `mj_copySpec` is not strictly `const` on the source: for every
// actuator it calls `ForgetKeyframes`, which clears two `std::map` keyframe-resolution caches
// (`act_`/`ctrl_`). Those maps are empty for a normally-built spec, yet `std::map::clear` rewrites
// the tree header unconditionally, so two threads sharing a `&MjSpec` and cloning concurrently
// would race on those writes (a data race, though no spec data is lost). Hence the type stays `!Sync`.
unsafe impl Send for MjSpec {}

impl MjSpec {
    /// Creates an empty [`MjSpec`].
    ///
    /// # Panics
    /// - When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    /// - When MuJoCo fails to allocate the specification.
    ///   Use [`MjSpec::try_new`] for a fallible alternative.
    pub fn new() -> Self {
        Self::try_new().expect("MuJoCo failed to allocate MjSpec")
    }

    /// Fallible version of [`MjSpec::new`].
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo fails to allocate
    /// the specification.
    ///
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn try_new() -> Result<Self, MjEditError> {
        assert_mujoco_version();
        // SAFETY: mj_makeSpec allocates a new MjSpec; returns null on allocation
        // failure, handled by ok_or below.
        let ptr = unsafe { mj_makeSpec() };
        Ok(MjSpec(NonNull::new(ptr).ok_or(MjEditError::AllocationFailed)?))
    }

    /// Creates a deep copy of this [`MjSpec`].
    ///
    /// Internally calls `mj_copySpec`, which invokes the C++ copy constructor
    /// on the underlying model.  This is a proper deep copy: the returned spec
    /// is fully independent from the original.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo fails to allocate
    /// the copy (e.g. out of memory or an internal C++ exception).
    pub fn try_clone(&self) -> Result<Self, MjEditError> {
        // SAFETY: self.0 is a valid non-null mjSpec pointer for the lifetime of self
        // (struct invariant); mj_copySpec returns null on allocation failure, handled below.
        let ptr = unsafe { mj_copySpec(self.0.as_ptr()) };
        NonNull::new(ptr).map(MjSpec).ok_or(MjEditError::AllocationFailed)
    }

    /// Creates a [`MjSpec`] from the `path` to a file.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjSpec`].
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the XML.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, MjEditError> {
        Self::from_xml_file(path, None)
    }

    /// Creates a [`MjSpec`] from the `path` to a file, located in a virtual file system (`vfs`).
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjSpec`].
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the XML.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, MjEditError> {
        Self::from_xml_file(path, Some(vfs))
    }

    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, MjEditError> {
        assert_mujoco_version();

        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let path_str = path.as_ref().to_str()
                .ok_or(MjEditError::InvalidUtf8Path)?;
            let path = CString::new(path_str).unwrap();
            let raw_ptr = mj_parseXML(
                path.as_ptr(), vfs.map_or(ptr::null(), |v| v.ffi()),
                error_buffer.as_mut_ptr(), error_buffer.len() as c_int
            );

            Self::check_spec(raw_ptr, &error_buffer)
        }
    }

    /// Creates a [`MjSpec`] from an `xml` string.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjSpec`].
    /// # Errors
    /// Returns [`MjEditError::ParseFailed`] if MuJoCo encounters an error parsing the string.
    /// # Panics
    /// - when the `xml` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_string(xml: &str) -> Result<Self, MjEditError> {
        assert_mujoco_version();

        let c_xml = CString::new(xml).unwrap();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let spec_ptr = mj_parseXMLString(
                c_xml.as_ptr(), ptr::null(),
                error_buffer.as_mut_ptr(), error_buffer.len() as c_int
            );
            Self::check_spec(spec_ptr, &error_buffer)
        }
    }

    /// Parse and create a [`MjSpec`] from `filename`.
    /// The `content_type` controls the decoder to use.
    /// This is a wrapper around low-level method [`mj_parse`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjSpec`].
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the file.
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_parse<T: AsRef<Path>>(filename: T, content_type: &str) -> Result<Self, MjEditError> {
        Self::from_parse_file(filename, content_type, None)
    }

    /// Same as [`MjSpec::from_parse`], except `filename` is taken from `vfs`.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjSpec`].
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the file.
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_parse_vfs<T: AsRef<Path>>(filename: T, content_type: &str, vfs: &MjVfs) -> Result<Self, MjEditError> {
        Self::from_parse_file(filename, content_type, Some(vfs))
    }

    /// Parse and create a [`MjSpec`] from `filename`.
    /// The `content_type` controls the decoder to use.
    /// This is a wrapper around low-level method [`mj_parse`].
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the version MuJoCo-rs was compiled against.
    fn from_parse_file<T: AsRef<Path>>(filename: T, content_type: &str, vfs: Option<&MjVfs>) -> Result<Self, MjEditError> {
        assert_mujoco_version();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let c_filename = CString::new(
                filename.as_ref().to_str()
                .ok_or(MjEditError::InvalidUtf8Path)?
            ).unwrap();
            let c_content_type = CString::new(content_type).unwrap();
            let ptr = mj_parse(
                c_filename.as_ptr(), c_content_type.as_ptr(),
                vfs.map_or(ptr::null(), |v| v.ffi()),
                error_buffer.as_mut_ptr(), error_buffer.len() as i32
            );
            Self::check_spec(ptr, &error_buffer)
        }
    }

    /// Handles spec pointer input.
    fn check_spec(spec_ptr: *mut mjSpec, error_buffer: &[c_char]) -> Result<Self, MjEditError> {
        if spec_ptr.is_null() {
            // SAFETY: error_buffer is zero-initialised and MuJoCo always
            // NUL-terminates the message it writes into it.
            let message = unsafe { CStr::from_ptr(error_buffer.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            Err(MjEditError::ParseFailed(message))
        }
        else {
            // SAFETY: spec_ptr is confirmed non-null by the guard above.
            Ok(MjSpec(unsafe { NonNull::new_unchecked(spec_ptr) }))
        }
    }

    /// An immutable reference to the internal FFI struct.
    pub fn ffi(&self) -> &mjSpec {
        // SAFETY: self.0 is a valid non-null mjSpec pointer for the lifetime of
        // self (struct invariant).
        unsafe { self.0.as_ref() }
    }

    /// A mutable reference to the internal FFI struct.
    ///
    /// # Safety
    /// Callers must ensure that any mutations performed through the returned reference
    /// preserve the invariants that MuJoCo expects for `mjSpec`.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjSpec {
        unsafe { self.0.as_mut() }
    }

    /// Delete an element from this specification.
    ///
    /// # Errors
    /// Returns [`MjEditError::DeleteFailed`] if MuJoCo cannot delete the element.
    ///
    /// # Safety
    /// The caller must ensure:
    /// - `element` is a valid pointer to an mjsElement
    /// - `element` is owned by this spec
    /// - `element` has not been previously deleted
    /// - no Rust references derived from `element` exist
    pub unsafe fn delete_element(&mut self, element: *mut mjsElement) -> Result<(), MjEditError> {
        if element.is_null() {
            return Err(MjEditError::DeleteFailed("null element pointer".to_owned()));
        }

        let spec = unsafe { self.ffi_mut() };
        let owner = unsafe { mjs_getSpec(element) };
        if owner != spec {
            return Err(MjEditError::DeleteFailed("element does not belong to this spec".to_owned()));
        }

        if unsafe { (*element).elemtype } == MjtObj::mjOBJ_DEFAULT {
            return Err(MjEditError::UnsupportedOperation);
        }
        if unsafe { (*element).elemtype } == MjtObj::mjOBJ_BODY {
            let name = unsafe { read_mjs_string(mjs_getName(element)) };
            if name == "world" {
                return Err(MjEditError::UnsupportedOperation);
            }
        }

        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => {
                let error_msg = unsafe {
                    let ptr = mjs_getError(spec);
                    if ptr.is_null() {
                        "Unknown error".to_owned()
                    } else {
                        CStr::from_ptr(ptr).to_string_lossy().into_owned()
                    }
                };
                Err(MjEditError::DeleteFailed(error_msg))
            }
        }
    }

    /// Compile [`MjSpec`] to [`MjModel`].
    /// A spec can be edited and compiled multiple times,
    /// returning a new mjModel instance that takes the edits into account.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// Returns [`MjEditError::CompileFailed`] if the model fails to compile, including when a
    /// texture has a builtin pattern set while its `nchannel` is less than 3.
    pub fn compile(&mut self) -> Result<MjModel, MjEditError> {
        // The builtin-texture generators (`Builtin2D`/`BuiltinCube`) write a hard-coded 3 bytes per
        // pixel, but MuJoCo only allocates `nchannel*width*height` bytes and (unlike the file/data
        // paths) does not check `nchannel >= 3` for the builtin path, so `nchannel < 3` heap-overflows
        // during compilation. Both setters (`set_nchannel`, `set_builtin`) are safe and independent,
        // so this cross-field invariant can only be enforced at the compile choke point.
        for texture in self.texture_iter() {
            if texture.builtin() != MjtBuiltin::mjBUILTIN_NONE && texture.nchannel() < 3 {
                return Err(MjEditError::CompileFailed(
                    "texture with a builtin pattern requires nchannel >= 3".to_owned(),
                ));
            }
        }

        let result = unsafe { MjModel::from_raw( mj_compile(self.0.as_ptr(), ptr::null()) ) };
        result.map_err(|_| {
            // SAFETY: The spec is still valid after failed compilation.
            // The error pointer is valid until the next MuJoCo call on this spec.
            let error_msg: String = unsafe {
                let ptr = mjs_getError(self.ffi_mut());
                if ptr.is_null() {
                    "Compilation failed (unknown error)".to_owned()
                } else {
                    CStr::from_ptr(ptr).to_string_lossy().into_owned()
                }
            };
            MjEditError::CompileFailed(error_msg)
        })
    }

    /// Return compiler timers (`mjtCTimer` order).
    pub fn timer(&self) -> &[f64; MjtCTimer::mjNCTIMER as usize] {
        unsafe { &*mjs_getTimer(self.0.as_ptr()).cast() }
    }

    /// Saves the spec to an XML file.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::SaveFailed`] with MuJoCo's error message if saving fails.
    /// # Panics
    /// When `filename` contains interior `\0` characters.
    pub fn save_xml<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjEditError> {
        let mut error_buff = [0; ERROR_BUF_LEN];
        let cname = CString::new(
            filename.as_ref().to_str()
            .ok_or(MjEditError::InvalidUtf8Path)?
        ).unwrap();  // filename is always UTF-8
        let result = unsafe { mj_saveXML(
            self.ffi(), cname.as_ptr(),
            error_buff.as_mut_ptr(), error_buff.len() as i32
        ) };
        match result {
            0 => Ok(()),
            _ => {
                // SAFETY: error_buff is zero-initialised and MuJoCo always
                // NUL-terminates the message it writes into it.
                let message = unsafe { CStr::from_ptr(error_buff.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjEditError::SaveFailed(message))
            }
        }
    }

    /// Saves the spec to an XML string.
    /// `buffer_size` controls how many bytes are allocated for the output.
    /// # Returns
    /// On success, returns the generated XML string.
    /// # Errors
    /// - [`MjEditError::XmlBufferTooSmall`] when `buffer_size` is too small.
    ///   The `required_size` field uses `snprintf`-style semantics (bytes to write, excluding NUL),
    ///   so retry with `required_size as usize + 1` bytes.
    /// - [`MjEditError::SaveFailed`] with MuJoCo's error message on any other failure.
    /// # Panics
    /// Panics if MuJoCo reports success but returns XML that is not NUL-terminated
    /// within the allocated output buffer.
    pub fn save_xml_string(&self, buffer_size: usize) -> Result<String, MjEditError> {
        let mut error_buff = [0; ERROR_BUF_LEN];
        let mut result_buff = vec![0u8; buffer_size];
        let result = unsafe { mj_saveXMLString(
            self.ffi(), result_buff.as_mut_ptr().cast(), result_buff.len() as i32,
            error_buff.as_mut_ptr(), error_buff.len() as i32
        ) };
        match result {
            0 => Ok(CStr::from_bytes_until_nul(&result_buff).unwrap().to_string_lossy().into_owned()),
            r if r > 0 => Err(MjEditError::XmlBufferTooSmall { required_size: r as usize }),
            _ => {
                // SAFETY: error_buff is zero-initialised and MuJoCo always
                // NUL-terminates the message it writes into it.
                let message = unsafe { CStr::from_ptr(error_buff.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjEditError::SaveFailed(message))
            }
        }
    }
}

/// Children accessor methods.
impl MjSpec {
    find_x_method! {
        body, geom, joint, site, camera, light, frame, actuator, sensor, flex, pair, equality, exclude, tendon,
        numeric, text, tuple, key, mesh, hfield, skin, texture, material, plugin
    }

    find_x_method_direct! { default }

    /// Returns an immutable reference to the world body.
    /// # Panics
    /// Panics if the "world" body is not found.
    pub fn world_body(&self) -> &MjsBody {
        self.body("world").unwrap()
    }

    /// Returns a mutable reference to the world body.
    /// # Panics
    /// Panics if the "world" body is not found.
    pub fn world_body_mut(&mut self) -> &mut MjsBody {
        self.body_mut("world").unwrap()
    }
}

/// Public attributes.
impl MjSpec {
    string_set_get_with! {
        [ffi, ffi_mut] modelname; "model name.";
        [ffi, ffi_mut] comment; "comment at top of XML.";
        [ffi, ffi_mut] modelfiledir; "path to model file.";
    }

    getter_setter! {
        with, get, [
            [ffi, ffi_mut] compiler: &MjsCompiler; "compiler options.";
            [ffi, ffi_mut] stat: &MjStatistic; "statistic overrides.";
            [ffi, ffi_mut] visual: &MjVisual; "visualization options.";
            [ffi, ffi_mut] option: &MjOption; "simulation options.";
        ]
    }

    getter_setter! {
        get, [
            [ffi] (allow_mut = false) authored: &MjsAuthored; "authored-field tracking bitmasks.";
        ]
    }

    getter_setter! {
        with, get, set, [
            [ffi, ffi_mut] strippath: bool; "whether to strip paths from mesh files.";
            [ffi, ffi_mut] hasImplicitPluginElem: bool; "already encountered an implicit plugin sensor/actuator.";
        ]
    }

    getter_setter! {
        get, [
            [ffi] memory: MjtSize;     "number of bytes in arena+stack memory.";
            [ffi] nemax: i32;             "max number of equality constraints.";
            [ffi] nuserdata: i32;              "number of mjtNums in userdata.";
            [ffi] nuser_body: i32;            "number of mjtNums in body_user.";
            [ffi] nuser_jnt: i32;              "number of mjtNums in jnt_user.";
            [ffi] nuser_geom: i32;            "number of mjtNums in geom_user.";
            [ffi] nuser_site: i32;            "number of mjtNums in site_user.";
            [ffi] nuser_cam: i32;              "number of mjtNums in cam_user.";
            [ffi] nuser_tendon: i32;        "number of mjtNums in tendon_user.";
            [ffi] nuser_actuator: i32;    "number of mjtNums in actuator_user.";
            [ffi] nuser_sensor: i32;        "number of mjtNums in sensor_user.";
            [ffi] nkey: i32;                             "number of keyframes.";
        ]
    }
}

/// Methods for adding non-tree elements.
impl MjSpec {
    add_x_method! { actuator, pair, equality, tendon, mesh, material }
    add_x_method_no_default! {
        sensor, flex, exclude, numeric, text, tuple, key, plugin,
        hfield, skin, texture
        // Wrap
    }

    /// Adds a new `<default>` element.
    ///
    /// # Panics
    /// Panics when `class_name` already exists or `parent_class_name` doesn't exist.
    /// Also panics when the `class_name` or `parent_class_name` contain '\0' characters.
    ///
    /// Use [`MjSpec::try_add_default`] for a fallible alternative.
    pub fn add_default(&mut self, class_name: &str, parent_class_name: Option<&str>) -> &mut MjsDefault {
        self.try_add_default(class_name, parent_class_name).unwrap()
    }

    /// Fallible version of [`MjSpec::add_default`].
    /// # Returns
    /// On success, returns a mutable reference to the newly created [`MjsDefault`].
    /// # Errors
    /// Returns [`MjEditError::AlreadyExists`] when `class_name` already exists.
    /// Returns [`MjEditError::NotFound`] when `parent_class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` or `parent_class_name` contain '\0' characters, a panic occurs.
    pub fn try_add_default(&mut self, class_name: &str, parent_class_name: Option<&str>) -> Result<&mut MjsDefault, MjEditError> {
        let c_class_name = CString::new(class_name).unwrap();

        let parent_ptr = if let Some(name) = parent_class_name {
                self.default(name).ok_or(MjEditError::NotFound)?
        } else {
            ptr::null()
        };

        unsafe {
            let ptr_default = mjs_addDefault(
                self.ffi_mut(),
                c_class_name.as_ptr(),
                parent_ptr
            );
            if ptr_default.is_null() {
                Err(MjEditError::AlreadyExists)
            }
            else {
                Ok(&mut *ptr_default)
            }
        }
    }
}

/// Mutable iterator over items in [`MjSpec`].
#[derive(Debug)]
pub struct MjsSpecItemIterMut<'a, T> {
    /// Pointer to the wrapped mjSpec pointer.
    /// This is the FFI type wrapped inside [`MjSpec`].
    ffi_ptr: *mut mjSpec,
    /// Last obtained element in the iterator.
    /// This CAN be null, and this iterator it uses the null to detect
    /// end of iteration. 
    last: *mut mjsElement,
    /// Used for generic implementation of iterator's methods.
    item_type: PhantomData<&'a mut T>
}

/// Immutable iterator over items in [`MjSpec`].
#[derive(Debug, Clone)]
pub struct MjsSpecItemIter<'a, T> {
    /// Pointer to the wrapped mjSpec pointer.
    /// This is the FFI type wrapped inside [`MjSpec`].
    ffi_ptr: *const mjSpec,
    /// Last obtained element in the iterator.
    /// This CAN be null, and this iterator it uses the null to detect
    /// end of iteration. 
    last: *mut mjsElement,
    /// Used for generic implementation of iterator's methods.
    item_type: PhantomData<&'a T>
}

impl<'a, T: SpecObject> MjsSpecItemIterMut<'a, T> {
    fn new(root: &'a mut MjSpec) -> Self {
        let last = unsafe { mjs_firstElement(root.0.as_ptr(), T::OBJ_TYPE) };
        Self { ffi_ptr: root.0.as_ptr(), last, item_type: PhantomData }
    }
}

impl<'a, T: SpecObject> MjsSpecItemIter<'a, T> {
    fn new(root: &'a MjSpec) -> Self {
        // SAFETY: mjs_firstElement does not mutate mjsSpec, thus as_ptr is valid to be case to *mut
        // from the const reference to its wrapper.
        let last = unsafe { mjs_firstElement(root.0.as_ptr(), T::OBJ_TYPE) };
        Self { ffi_ptr: root.0.as_ptr(), last, item_type: PhantomData }
    }
}

impl<'a, T: SpecObject + 'a> Iterator for MjsSpecItemIterMut<'a, T> {
    type Item = &'a mut T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_mut();
            // Use as_ptr() instead of ffi_mut() to avoid creating &mut mjSpec,
            // which would alias with previously yielded &mut items.
            self.last = mjs_nextElement(self.ffi_ptr, self.last);
            out
        }
    }
}

impl<'a, T: SpecObject + 'a> Iterator for MjsSpecItemIter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_ref();
            // SAFETY: mjs_nextElement does not mutate mjsSpec, thus as_ptr is valid to be case to *mut
            // from the const reference to its wrapper.
            self.last = mjs_nextElement(self.ffi_ptr, self.last);
            out
        }
    }
}

// Once self.last is null, next() always returns None.
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsSpecItemIterMut<'a, T> {}
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsSpecItemIter<'a, T> {}

/// Iterator methods.
impl MjSpec {
    spec_get_iter! {
        body, geom, joint, site, camera, light, frame, actuator, sensor, flex, pair, equality,
        exclude, tendon, numeric, text, tuple, key, mesh, hfield, skin, texture, material, plugin
    }
}

impl Default for MjSpec {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for MjSpec {
    fn drop(&mut self) {
        // SAFETY: self.0 is a valid non-null mjSpec pointer; called exactly once
        // in Drop.
        unsafe { mj_deleteSpec(self.0.as_ptr()); }
    }
}

impl Clone for MjSpec {
    /// Creates a deep copy of this [`MjSpec`].
    ///
    /// # Panics
    /// Panics if MuJoCo fails to allocate the cloned spec.
    /// Use [`MjSpec::try_clone`] for a fallible alternative.
    fn clone(&self) -> Self {
        self.try_clone().expect("MuJoCo failed to clone MjSpec")
    }
}

/***************************
** Site specification
***************************/
mjs_struct!(Site [SpecObject]);
impl MjsSite {
    getter_setter! {
        [&] with, get, [
            // frame, size
            pos:  &[f64; 3];              "position.";
            quat: &[f64; 4];              "orientation.";
            alt:  &MjsOrientation;        "alternative orientation.";
            fromto: &[f64; 6];            "alternative for capsule, cylinder, box, ellipsoid.";
            size: &[f64; 3];              "geom size.";

            // visual
            rgba: &[f32; 4];              "rgba when material is omitted.";
    ]}

    getter_setter!([&] with, get, set, [
        type_ + _: MjtGeom;               "geom type.";
        group: i32;                       "group.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {[&]
        material; "name of material.";
    }
}

/***************************
** Joint specification
***************************/
mjs_struct!(Joint [SpecObject]);
impl MjsJoint {
    getter_setter! {
        [&] with, get, [
            // kinematics
            pos:     &[f64; 3];         "anchor position.";
            axis:    &[f64; 3];         "joint axis.";
            ref_ + _:    &f64;          "value at reference configuration: qpos0.";
            springdamper: &[f64; 2];    "timeconst, dampratio.";

            // stiffness
            stiffness: &[f64; mjNPOLY as usize + 1];            "stiffness coefficient.";

            // limits
            range:   &[f64; 2];         "joint limits.";
            solref_limit: &[MjtNum; mjNREF as usize];  "solver reference: joint limits.";
            solimp_limit: &[MjtNum; mjNIMP as usize];  "solver impedance: joint limits.";
            actfrcrange: &[f64; 2];     "actuator force limits.";

            // dof properties
            damping: &[f64; mjNPOLY as usize + 1];                 "damping coefficient.";
            solref_friction: &[MjtNum; mjNREF as usize]; "solver reference: dof friction.";
            solimp_friction: &[MjtNum; mjNIMP as usize]; "solver impedance: dof friction.";
        ]
    }

    getter_setter!([&] with, get, set, [
        type_ + _: MjtJoint;           "joint type.";
        group: i32;                    "joint group.";
        springref: f64;               "spring reference value: qpos_spring.";
        margin: f64;                  "margin value for joint limit detection.";
        armature: f64;                "armature inertia (mass for slider).";
        frictionloss: f64;            "friction loss.";
    ]);

    getter_setter! {
        [&] with, get, set, [
            align: MjtAlignFree [force];       "align free joint with body com (mjtAlignFree).";
            limited: MjtLimited [force];       "does joint have limits (mjtLimited).";
            actfrclimited: MjtLimited [force]; "are actuator forces on joint limited (mjtLimited).";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            actgravcomp: bool;         "is gravcomp force applied via actuators.";
        ]
    }

    userdata_method!(f64);
}

/***************************
** Geom specification
***************************/
mjs_struct!(Geom [SpecObject]);
impl MjsGeom {
    getter_setter! {
        [&] with, get, [
            pos: &[f64; 3];                         "geom position.";
            quat: &[f64; 4];                        "geom orientation.";
            alt: &MjsOrientation;                   "alternative orientation.";
            fromto: &[f64; 6];                      "alternative for capsule, cylinder, box, ellipsoid.";
            size: &[f64; 3];                        "geom size.";
            rgba: &[f32; 4];                        "rgba when material is omitted.";
            friction: &[f64; 3];                    "one-sided friction coefficients: slide, roll, spin.";
            solref: &[MjtNum; mjNREF as usize];     "solver reference.";
            solimp: &[MjtNum; mjNIMP as usize];     "solver impedance.";
            fluid_coefs: &[MjtNum; 5];              "ellipsoid-fluid interaction coefs."
        ]
    }

    getter_setter! {
        get, [
            plugin: &MjsPlugin;                     "sdf plugin.";
        ]
    }

    getter_setter!([&] with, get, set, [
        type_ + _: MjtGeom;            "geom type.";
        group: i32;                    "group.";
        contype: i32;                  "contact type.";
        conaffinity: i32;              "contact affinity.";
        condim: i32;                   "contact dimensionality.";
        priority: i32;                 "contact priority.";
        solmix: f64;                   "solver mixing for contact pairs.";
        margin: f64;                   "margin for contact detection.";
        gap: f64;                      "additional contact detection buffer.";
        mass: f64;                     "used to compute density.";
        density: f64;                  "used to compute mass and inertia from volume or surface.";
        typeinertia: MjtGeomInertia;   "selects between surface and volume inertia.";
        fluid_ellipsoid: MjtNum;       "whether ellipsoid-fluid model is active.";
        fitscale: f64;                 "scale mesh uniformly.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {[&]
        meshname;   "mesh attached to geom.";
        material;   "name of material.";
        hfieldname; "heightfield attached to geom.";
    }
}

/***************************
** Camera specification
***************************/
mjs_struct!(Camera [SpecObject]);
impl MjsCamera {
    getter_setter! {
        [&] with, get, [
            pos: &[f64; 3];               "camera position.";
            quat: &[f64; 4];              "camera orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
            intrinsic: &[f32; 4];         "intrinsic parameters.";
            sensor_size: &[f32; 2];       "sensor size.";
            resolution: &[i32; 2];        "resolution.";
            focal_length: &[f32; 2];      "focal length (length).";
            focal_pixel: &[f32; 2];       "focal length (pixel).";
            principal_length: &[f32; 2];  "principal point (length).";
            principal_pixel: &[f32; 2];   "principal point (pixel).";
        ]
    }

    getter_setter!([&] with, get, set, [
        mode: MjtCamLight;              "camera mode.";
        fovy: f64;                      "field of view in y direction.";
        ipd: f64;                       "inter-pupillary distance for stereo.";
        proj: MjtProjection;            "camera projection type.";
        output: i32;                    "bit flags for output type.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {[&]
        targetbody; "target body for tracking/targeting.";
    }
}

/***************************
** Light specification
***************************/
mjs_struct!(Light [SpecObject]);
impl MjsLight {
    getter_setter! {
        [&] with, get, [
            pos: &[f64; 3];               "light position.";
            dir: &[f64; 3];               "light direction.";
            ambient: &[f32; 3];           "ambient color.";
            diffuse: &[f32; 3];           "diffuse color.";
            specular: &[f32; 3];          "specular color.";
            attenuation: &[f32; 3];       "OpenGL attenuation (quadratic model).";
        ]
    }

    getter_setter!([&] with, get, set, [
        mode: MjtCamLight;             "light mode.";
        type_ + _: MjtLightType;       "light type.";
        bulbradius: f32;               "bulb radius, for soft shadows.";
        intensity: f32;                "intensity, in candelas.";
        range: f32;                    "range of effectiveness.";
        cutoff: f32;                   "OpenGL cutoff.";
        exponent: f32;                 "OpenGL exponent.";
    ]);

    getter_setter! {
        [&] with, get, set, [
            active: bool;       "active flag.";
            castshadow: bool;   "whether light cast shadows."
        ]
    }

    string_set_get_with! {[&]
        texture; "texture name for image lights.";
        targetbody; "target body for targeting.";
    }
}

/***************************
** Frame specification
***************************/
mjs_struct!(Frame [SpecObject]);
impl MjsFrame {
    add_x_method_by_frame! { body, site, joint, geom, camera, light }

    getter_setter! {
        [&] with, get, [
            pos: &[f64; 3];               "frame position.";
            quat: &[f64; 4];              "frame orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
        ]
    }

    string_set_get_with! {[&]
        childclass; "childclass name.";
    }

    /// Add and return a child frame.
    ///
    /// Delegates to [`Self::try_add_frame`] and panics if allocation fails.
    /// # Panics
    /// Panics if MuJoCo fails to allocate the frame.
    pub fn add_frame(&mut self) -> &mut MjsFrame {
        self.try_add_frame().expect("mjs_addFrame returned null; allocation failed")
    }

    /// Fallible version of [`Self::add_frame`].
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate
    /// the frame, instead of panicking.
    pub fn try_add_frame(&mut self) -> Result<&mut MjsFrame, MjEditError> {
        // SAFETY: element_mut_pointer() reads `self.element`, valid for any live MjsFrame.
        // mjs_getParent returns non-null because every Rust-API MjsFrame was created via
        // mjs_addFrame, which always calls SetParent(body).
        let parent_body = unsafe { mjs_getParent(self.element_mut_pointer()) };
        debug_assert!(!parent_body.is_null(), "mjs_getParent returned null; frame has no parent body");
        let ptr = unsafe { mjs_addFrame(parent_body, self) };
        // SAFETY: ptr.as_mut() returns None for null, handled by ok_or; when non-null the
        // pointee is properly aligned and initialized by C++ operator new, and freshly
        // allocated so no existing Rust reference aliases it for the returned lifetime.
        unsafe { ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }
}

/* Non-tree elements */

/***************************
** Actuator specification
***************************/
mjs_struct!(Actuator [SpecObject]);
impl MjsActuator {
    getter_setter! {
        [&] with, get, [
            gear: &[f64; 6];                            "gear parameters.";
            gainprm: &[f64; mjNGAIN as usize];          "gain parameters.";
            biasprm: &[f64; mjNBIAS as usize];          "bias parameters.";
            dynprm: &[f64; mjNDYN as usize];            "dynamic parameters.";
            lengthrange: &[f64; 2];                     "transmission length range.";
            damping: &[f64; mjNPOLY as usize + 1];      "damping coefficient.";
            ctrlrange: &[f64; 2];                       "control range.";
            forcerange: &[f64; 2];                      "force range.";
            actrange: &[f64; 2];                        "activation range.";
        ]
    }

    getter_setter! {
        get, [
            plugin: &MjsPlugin;                     "actuator plugin.";
        ]
    }

    getter_setter!([&] with, get, set, [
        gaintype: MjtGain;             "gain type.";
        biastype: MjtBias;             "bias type.";
        dyntype: MjtDyn;               "dyn type.";
        group: i32;                    "group.";
        actdim: i32;                   "number of activation variables.";
        trntype: MjtTrn;               "transmission type.";
        cranklength: f64;              "crank length, for slider-crank.";
        inheritrange: f64;             "automatic range setting for position and intvelocity.";
        armature: f64;                 "armature inertia.";
        nsample: i32;                  "number of samples in history buffer.";
        interp: i32;                   "interpolation order (0=ZOH, 1=linear, 2=cubic).";
        delay: f64;                    "delay time in seconds; 0: no delay.";
    ]);

    getter_setter! {
        [&] with, get, set, [
            ctrllimited: MjtLimited [force];        "are control limits defined.";
            forcelimited: MjtLimited [force];       "are force limits defined.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            actlimited: MjtLimited [force];         "are activation limits defined.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            actearly: bool;                "apply next activations to qfrc.";
        ]
    }

    userdata_method!(f64);

    string_set_get_with! {[&]
        target;                 "name of transmission target.";
        refsite;                "reference site, for site transmission.";
        slidersite;             "site defining cylinder, for slider-crank.";
    }
}


/// Converts the error string returned by MuJoCo's `mjs_setToX` actuator
/// configuration functions into a [`Result`].
///
/// Those functions return an empty string on success and a non-empty,
/// NUL-terminated message describing the rejected parameter on failure.
fn actuator_set_result(c_err_msg: *const c_char) -> Result<(), MjEditError> {
    // SAFETY: MuJoCo's error messages are always NUL terminated.
    let err_msg = unsafe { CStr::from_ptr(c_err_msg) }.to_string_lossy();
    if err_msg.is_empty() {
        Ok(())
    }
    else {
        Err(MjEditError::InvalidParameter(err_msg.into_owned()))
    }
}

/* Actuator configuration structs.
** Each `set_to_*` method taking optional (nullable in C) parameters has its own config struct.
** Build either with struct-update syntax or with the chainable `with_*` builder methods, e.g.
** `PositionConfig { kp: 10.0, kv: Some(2.0), ..Default::default() }` or
** `PositionConfig::default().with_kp(10.0).with_kv(2.0)`. Optional fields default to `None`,
** leaving the corresponding actuator parameter at its MuJoCo default; the `with_*` setters take
** the inner value directly and wrap it in `Some`. */

/// Configuration for [`MjsActuator::set_to_position`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionConfig {
    /// Proportional (position) gain.
    pub kp: f64,
    /// Automatic range-inheritance factor (0 disables it).
    pub inheritrange: f64,
    /// Velocity feedback gain. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// First-order activation-filter time constant.
    pub timeconst: Option<f64>,
}

impl PositionConfig {
    getter_setter! {
        with, [
            kp: f64;            "the proportional (position) gain.";
            inheritrange: f64;  "the automatic range-inheritance factor.";
            kv: f64;            "the velocity feedback gain (mutually exclusive with dampratio).";
            dampratio: f64;     "the damping ratio (mutually exclusive with kv).";
            timeconst: f64;     "the first-order activation-filter time constant.";
        ]
    }
}

/// Configuration for [`MjsActuator::set_to_int_velocity`]. Same parameters as [`PositionConfig`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IntVelocityConfig {
    /// Proportional gain.
    pub kp: f64,
    /// Automatic range-inheritance factor (0 disables it).
    pub inheritrange: f64,
    /// Velocity feedback gain. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// First-order activation-filter time constant.
    pub timeconst: Option<f64>,
}

impl IntVelocityConfig {
    getter_setter! {
        with, [
            kp: f64;            "the proportional gain.";
            inheritrange: f64;  "the automatic range-inheritance factor.";
            kv: f64;            "the velocity feedback gain (mutually exclusive with dampratio).";
            dampratio: f64;     "the damping ratio (mutually exclusive with kv).";
            timeconst: f64;     "the first-order activation-filter time constant.";
        ]
    }
}

/// Configuration for [`MjsActuator::set_to_dc_motor`].
///
/// Each optional field defaults to `None`, disabling the corresponding feature.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DcMotorConfig {
    /// Electrical resistance.
    pub resistance: f64,
    /// Input mode selector.
    pub input_mode: i32,
    /// Torque and back-EMF constants `[Kt, Ke]`.
    pub motorconst: Option<[f64; 2]>,
    /// Nominal ratings `[voltage, stall_torque, no_load_speed]`.
    pub nominal: Option<[f64; 3]>,
    /// Saturation `[tau_max, i_max, di_dt_max]`.
    pub saturation: Option<[f64; 3]>,
    /// Inductance `[L, te]`.
    pub inductance: Option<[f64; 2]>,
    /// Cogging `[amplitude, periodicity, phase]`.
    pub cogging: Option<[f64; 3]>,
    /// Controller `[kp, ki, kd, slewmax, Imax, v_max]`.
    pub controller: Option<[f64; 6]>,
    /// Thermal `[R_th, C, tau_th, alpha, T0, T_ambient]`.
    pub thermal: Option<[f64; 6]>,
    /// LuGre friction `[stiffness, damping, coulomb, static, stribeck]`.
    pub lugre: Option<[f64; 5]>,
}

impl DcMotorConfig {
    getter_setter! {
        with, [
            resistance: f64;       "the electrical resistance.";
            input_mode: i32;       "the input mode selector.";
            motorconst: [f64; 2];  "the torque and back-EMF constants [Kt, Ke].";
            nominal: [f64; 3];     "the nominal ratings [voltage, stall_torque, no_load_speed].";
            saturation: [f64; 3];  "the saturation [tau_max, i_max, di_dt_max].";
            inductance: [f64; 2];  "the inductance [L, te].";
            cogging: [f64; 3];     "the cogging [amplitude, periodicity, phase].";
            controller: [f64; 6];  "the controller [kp, ki, kd, slewmax, Imax, v_max].";
            thermal: [f64; 6];     "the thermal [R_th, C, tau_th, alpha, T0, T_ambient].";
            lugre: [f64; 5];       "the LuGre friction [stiffness, damping, coulomb, static, stribeck].";
        ]
    }
}

impl MjsActuator {
    /// Configure the actuator to be a motor.
    pub fn set_to_motor(&mut self) {
        // mjs_setToMotor cannot fail; it always returns an empty string.
        unsafe { mjs_setToMotor(self) };
    }

    /// Configure the actuator to be a positional-target motor (with a proportional regulator).
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when the configuration is rejected, e.g. `kv` and
    /// `dampratio` are both set, a value that must be non-negative is negative, or `inheritrange`
    /// is set together with a control range.
    pub fn set_to_position(&mut self, config: PositionConfig) -> Result<(), MjEditError> {
        let PositionConfig { kp, inheritrange, mut kv, mut dampratio, mut timeconst } = config;
        let c_err_msg = unsafe { mjs_setToPosition(
            self, kp,
            kv.as_mut().map_or(ptr::null_mut(), |x| x),
            dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
            timeconst.as_mut().map_or(ptr::null_mut(), |x| x),
            inheritrange
        ) };
        actuator_set_result(c_err_msg)
    }

    /// Configure the actuator to be an integrated-velocity servo. Behaves like
    /// [`MjsActuator::set_to_position`], but integrates the control signal into an activation
    /// variable.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `inheritrange` is set together with an
    /// activation range.
    pub fn set_to_int_velocity(&mut self, config: IntVelocityConfig) -> Result<(), MjEditError> {
        let IntVelocityConfig { kp, inheritrange, mut kv, mut dampratio, mut timeconst } = config;
        let c_err_msg = unsafe { mjs_setToIntVelocity(
            self, kp,
            kv.as_mut().map_or(ptr::null_mut(), |x| x),
            dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
            timeconst.as_mut().map_or(ptr::null_mut(), |x| x),
            inheritrange
        ) };
        actuator_set_result(c_err_msg)
    }

    /// Configure the actuator to be a velocity servo with velocity feedback gain `kv`.
    pub fn set_to_velocity(&mut self, kv: f64) {
        // mjs_setToVelocity cannot fail; it always returns an empty string.
        unsafe { mjs_setToVelocity(self, kv) };
    }

    /// Configure the actuator to be a damper with damping coefficient `kv`. The applied force is
    /// proportional to velocity and modulated by the (non-negative) control input.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `kv` is negative or the control range is
    /// negative.
    pub fn set_to_damper(&mut self, kv: f64) -> Result<(), MjEditError> {
        actuator_set_result(unsafe { mjs_setToDamper(self, kv) })
    }

    /// Configure the actuator to be a hydraulic or pneumatic cylinder. `timeconst` is the
    /// activation filter time constant, `bias` is added to the force, and the effective area is
    /// `area`; if `diameter` is non-negative the area is computed from it instead (pass a negative
    /// `diameter` to use `area` directly).
    pub fn set_to_cylinder(&mut self, timeconst: f64, bias: f64, area: f64, diameter: f64) {
        // mjs_setToCylinder cannot fail; it always returns an empty string.
        unsafe { mjs_setToCylinder(self, timeconst, bias, area, diameter) };
    }

    /// Configure the actuator to be a muscle. `timeconst` holds the activation and deactivation
    /// time constants, `range` the operating-length range, and the remaining scalars the muscle
    /// force-length-velocity parameters. A negative value for any array entry or scalar (except
    /// `tausmooth`) leaves the corresponding muscle default in place.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `tausmooth` is negative.
    #[allow(clippy::too_many_arguments)]
    pub fn set_to_muscle(
        &mut self, mut timeconst: [f64; 2], tausmooth: f64, mut range: [f64; 2],
        force: f64, scale: f64, lmin: f64, lmax: f64, vmax: f64, fpmax: f64, fvmax: f64
    ) -> Result<(), MjEditError>
    {
        let c_err_msg = unsafe { mjs_setToMuscle(
            self, &mut timeconst, tausmooth, &mut range,
            force, scale, lmin, lmax, vmax, fpmax, fvmax
        ) };
        actuator_set_result(c_err_msg)
    }

    /// Configure the actuator to be an active-adhesion actuator with the given `gain`.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `gain` is negative or the control range is
    /// negative.
    pub fn set_to_adhesion(&mut self, gain: f64) -> Result<(), MjEditError> {
        actuator_set_result(unsafe { mjs_setToAdhesion(self, gain) })
    }

    /// Configure the actuator to be a DC motor.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when MuJoCo cannot derive a positive motor
    /// constant or resistance, or when an inductance, thermal resistance, or thermal capacitance
    /// value is out of its allowed range.
    pub fn set_to_dc_motor(&mut self, config: DcMotorConfig) -> Result<(), MjEditError> {
        let DcMotorConfig {
            resistance, input_mode,
            mut motorconst, mut nominal, mut saturation, mut inductance,
            mut cogging, mut controller, mut thermal, mut lugre
        } = config;
        let c_err_msg = unsafe { mjs_setToDCMotor(
            self,
            motorconst.as_mut().map_or(ptr::null_mut(), |x| x),
            resistance,
            nominal.as_mut().map_or(ptr::null_mut(), |x| x),
            saturation.as_mut().map_or(ptr::null_mut(), |x| x),
            inductance.as_mut().map_or(ptr::null_mut(), |x| x),
            cogging.as_mut().map_or(ptr::null_mut(), |x| x),
            controller.as_mut().map_or(ptr::null_mut(), |x| x),
            thermal.as_mut().map_or(ptr::null_mut(), |x| x),
            lugre.as_mut().map_or(ptr::null_mut(), |x| x),
            input_mode
        ) };
        actuator_set_result(c_err_msg)
    }
}

/***************************
** Sensor specification
***************************/
mjs_struct!(Sensor [SpecObject]);
impl MjsSensor {
    getter_setter! {
        [&] with, get, [
            intprm: &[i32; mjNSENS as usize];            "integer parameters.";
            interval: &[f64; 2];                         "[period, time_prev] in seconds.";
        ]
    }

    getter_setter! {
        get, [
            plugin: &MjsPlugin;                     "sensor plugin.";
        ]
    }

    getter_setter!([&] with, get, set, [
        type_ + _: MjtSensor;          "sensor type.";
        objtype: MjtObj { check_objtype, "[`MjEditError::InvalidParameter`] when the object type is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])" } => MjEditError;
                                       "object type the sensor refers to.";
        reftype: MjtObj { check_objtype, "[`MjEditError::InvalidParameter`] when the reference type is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])" } => MjEditError;
                                       "type of referenced object.";
        datatype: MjtDataType;         "data type.";
        cutoff: f64;                   "cutoff for real and positive datatypes.";
        noise: f64;                    "noise stdev.";
        needstage: MjtStage;           "compute stage needed to simulate sensor.";
        dim: i32;                      "number of scalar outputs.";
        nsample: i32;                  "number of samples in history buffer.";
        interp: i32;                   "interpolation order (0=ZOH, 1=linear, 2=cubic).";
        delay: f64;                    "delay time in seconds; 0: no delay.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {[&]
        refname; "name of referenced object.";
        objname; "name of sensorized object.";
    }
}

/***************************
** Flex specification
***************************/
mjs_struct!(Flex [SpecObject]);
impl MjsFlex {
    getter_setter! {
        [&] with, get, [
            rgba: &[f32; 4];                                "rgba when material is omitted.";
            friction: &[f64; 3];                            "contact friction vector.";
            solref: &[MjtNum; mjNREF as usize];             "solref for the pair.";
            solimp: &[MjtNum; mjNIMP as usize];             "solimp for the pair.";
            size: &[f64; 3];                                "vertex bounding box half sizes in qpos0.";
            cellcount: &[i32; 3];                           "grid cell count for finite cell method.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            young: f64;                    "elastic stiffness.";
            group: i32;                    "group.";
            contype: i32;                  "contact type.";
            conaffinity: i32;              "contact affinity.";
            condim: i32;                   "contact dimensionality.";
            priority: i32;                 "contact priority.";
            solmix: f64;                   "solver mixing for contact pairs.";
            margin: f64;                   "margin for contact detection.";
            gap: f64;                      "additional contact detection buffer.";

            dim: i32;                "element dimensionality.";
            radius: f64;             "radius around primitive element.";
            activelayers: i32;       "number of active element layers in 3D.";
            edgestiffness: f64;      "edge stiffness.";
            edgedamping: f64;        "edge damping.";
            poisson: f64;            "Poisson's ratio.";
            damping: f64;            "Rayleigh's damping.";
            thickness: f64;          "thickness (2D only).";
            elastic2d: i32;          "2D passive forces; 0: none, 1: bending, 2: stretching, 3: both.";
            order: i32;              "interpolation order (1: trilinear, 2: quadratic).";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            internal: bool;       "enable internal collisions.";
            flatskin: bool;       "render flex skin with flat shading.";
            passive: bool;        "mode for passive collisions.";
        ]        
    }

    getter_setter! {
        [&] with, get, set, [
            selfcollide: MjtFlexSelf [force];        "mode for flex self collision.";
        ]
    }

    string_set_get_with! {[&]
        material; "name of material used for rendering.";
    }

    vec_string_set_append! {
        nodebody; "node body names.";
        vertbody; "vertex body names.";
    }

    vec_set_get! {
        node: f64;      "node positions.";
        vert: f64;      "vertex positions.";
    }

    vec_set! {
        texcoord: f32;          "vertex texture coordinates.";
        elem: i32;              "element vertex ids.";
    }

    vec_set! {
        [unsafe: "The slice must have exactly `(dim + 1) * nelem` entries and every entry \
                  must be a valid index into the flex texture coordinates."
        ] elemtexcoord: i32 => i32; "element texture coordinates.";
    }
}

/***************************
** Pair specification
***************************/
mjs_struct!(Pair [SpecObject]);
impl MjsPair {
    getter_setter! {
        [&] with, get, [
            friction: &[f64; 5];                            "contact friction vector.";
            solref: &[MjtNum; mjNREF as usize];             "solref for the pair.";
            solimp: &[MjtNum; mjNIMP as usize];             "solimp for the pair.";
            solreffriction: &[MjtNum; mjNREF as usize];     "solver reference, frictional directions.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            margin: f64;             "margin for contact detection.";
            gap: f64;         "additional contact detection buffer.";
            condim: i32;                   "contact dimensionality.";
        ]
    }

    string_set_get_with! {[&]
        geomname1; "name of geom 1.";
        geomname2; "name of geom 2.";
    }
}

/***************************
** Exclude specification
***************************/
mjs_struct!(Exclude [SpecObject]);
impl MjsExclude {
    string_set_get_with! {[&]
        bodyname1; "name of body 1.";
        bodyname2; "name of body 2.";
    }
}

/***************************
** Equality specification
***************************/
mjs_struct!(Equality [SpecObject]);
impl MjsEquality {
    getter_setter! {
        [&] with, get, [
            data: &[f64; mjNEQDATA as usize];   "data array for equality parameters.";
            solref: &[f64; mjNREF as usize];    "solver reference.";         
            solimp: &[f64; mjNIMP as usize];    "solver impedance.";
        ]
    }

    getter_setter! {[&] with, get, set, [
        active: bool;   "active flag.";
    ]}

    getter_setter! {[&] with, get, set, [
        type_ + _: MjtEq;   "equality type.";
        objtype: MjtObj;    "type of both objects.";
    ]}

    string_set_get_with! {[&]
        name1; "name of object 1";
        name2; "name of object 2";
    }
}

/***************************
** Tendon specification
***************************/
mjs_struct!(Tendon [SpecObject]);
impl MjsTendon {
    getter_setter! {
        [&] with, get, [
            damping: &[f64; mjNPOLY as usize + 1];       "damping coefficient.";
            stiffness: &[f64; mjNPOLY as usize + 1];     "stiffness coefficient.";
            springlength: &[f64; 2];                    "spring length.";
            solref_friction: &[f64; mjNREF as usize];   "solver reference: tendon friction.";
            solimp_friction: &[f64; mjNIMP as usize];   "solver impedance: tendon friction.";
            range: &[f64; 2];                           "range.";
            actfrcrange: &[f64; 2];                     "actuator force limits.";
            solref_limit: &[f64; mjNREF as usize];      "solver reference: tendon limits.";
            solimp_limit: &[f64; mjNIMP as usize];      "solver impedance: tendon limits.";
            rgba: &[f32; 4];                            "rgba when material omitted.";
        ]
    }

    getter_setter! {[&] with, get, set, [
        group: i32;         "group.";
        frictionloss: f64;  "friction loss.";
        armature: f64;      "inertia associated with tendon velocity.";
        margin: f64;        "margin value for tendon limit detection.";
        width: f64;         "width for rendering.";
    ]}

    getter_setter! {
        [&] with, get, set, [
            limited: MjtLimited [force];       "does tendon have limits (mjtLimited).";
            actfrclimited: MjtLimited [force]; "does tendon have actuator force limits."
        ]
    }

    userdata_method!(f64);
    string_set_get_with! {[&]
        material; "name of material for rendering.";
    }

    /// Wrap a site corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_site(&mut self, name: &str) -> &mut MjsWrap {
        self.try_wrap_site(name).expect("failed to wrap site")
    }

    /// Fallible version of [`MjsTendon::wrap_site`].
    ///
    /// # Note
    ///
    /// <div class="warning">
    ///
    /// By default, MuJoCo aborts the process on an allocation failure instead of
    /// returning null. Under a non-default error configuration MuJoCo writes
    /// through the null pointer on allocation failure before returning, so the
    /// failure cannot be recovered soundly. Prefer the panicking
    /// [`MjsTendon::wrap_site`]. This method may be undeprecated in the future
    /// if MuJoCo's upstream C++ code is changed to return null recoverably.
    ///
    /// </div>
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters.
    #[deprecated(
        since = "5.0.0",
        note = "allocation failure cannot be recovered soundly; use `wrap_site`"
    )]
    pub fn try_wrap_site(&mut self, name: &str) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapSite(self, cname.as_ptr()) };
        unsafe { wrap_ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }

    /// Wrap a geom corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When `name` or `sidesite` contain '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_geom(&mut self, name: &str, sidesite: &str) -> &mut MjsWrap {
        self.try_wrap_geom(name, sidesite).expect("failed to wrap geom")
    }

    /// Fallible version of [`MjsTendon::wrap_geom`].
    ///
    /// # Note
    ///
    /// <div class="warning">
    ///
    /// By default, MuJoCo aborts the process on an allocation failure instead of
    /// returning null. Under a non-default error configuration MuJoCo writes
    /// through the null pointer on allocation failure before returning, so the
    /// failure cannot be recovered soundly. Prefer the panicking
    /// [`MjsTendon::wrap_geom`]. This method may be undeprecated in the future
    /// if MuJoCo's upstream C++ code is changed to return null recoverably.
    ///
    /// </div>
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When `name` or `sidesite` contain '\0' characters.
    #[deprecated(
        since = "5.0.0",
        note = "allocation failure cannot be recovered soundly; use `wrap_geom`"
    )]
    pub fn try_wrap_geom(&mut self, name: &str, sidesite: &str) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let csidesite = CString::new(sidesite).unwrap();
        let wrap_ptr = unsafe { mjs_wrapGeom(
            self,
            cname.as_ptr(), csidesite.as_ptr()
        ) };
        unsafe { wrap_ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }

    /// Wrap a joint corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When `name` contains '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_joint(&mut self, name: &str, coef: f64) -> &mut MjsWrap {
        self.try_wrap_joint(name, coef).expect("failed to wrap joint")
    }

    /// Fallible version of [`MjsTendon::wrap_joint`].
    ///
    /// # Note
    ///
    /// <div class="warning">
    ///
    /// By default, MuJoCo aborts the process on an allocation failure instead of
    /// returning null. Under a non-default error configuration MuJoCo writes
    /// through the null pointer on allocation failure before returning, so the
    /// failure cannot be recovered soundly. Prefer the panicking
    /// [`MjsTendon::wrap_joint`]. This method may be undeprecated in the future
    /// if MuJoCo's upstream C++ code is changed to return null recoverably.
    ///
    /// </div>
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When `name` contains '\0' characters.
    #[deprecated(
        since = "5.0.0",
        note = "allocation failure cannot be recovered soundly; use `wrap_joint`"
    )]
    pub fn try_wrap_joint(&mut self, name: &str, coef: f64) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapJoint(self, cname.as_ptr(), coef) };
        unsafe { wrap_ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }

    /// Wrap a pulley using the tendon.
    #[allow(deprecated)]
    pub fn wrap_pulley(&mut self, divisor: f64) -> &mut MjsWrap {
        self.try_wrap_pulley(divisor).expect("failed to wrap pulley")
    }

    /// Fallible version of [`MjsTendon::wrap_pulley`].
    ///
    /// # Note
    ///
    /// <div class="warning">
    ///
    /// By default, MuJoCo aborts the process on an allocation failure instead of
    /// returning null. Under a non-default error configuration MuJoCo writes
    /// through the null pointer on allocation failure before returning, so the
    /// failure cannot be recovered soundly. Prefer the panicking
    /// [`MjsTendon::wrap_pulley`]. This method may be undeprecated in the future
    /// if MuJoCo's upstream C++ code is changed to return null recoverably.
    ///
    /// </div>
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    #[deprecated(
        since = "5.0.0",
        note = "allocation failure cannot be recovered soundly; use `wrap_pulley`"
    )]
    pub fn try_wrap_pulley(&mut self, divisor: f64) -> Result<&mut MjsWrap, MjEditError> {
        let wrap_ptr = unsafe { mjs_wrapPulley(self, divisor) };
        unsafe { wrap_ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }

    /// Return the number of wrap objects.
    pub fn wrap_num(&self) -> usize {
        unsafe { mjs_getWrapNum(self) as usize }
    }

    /// Return an indexed wrap object.
    ///
    /// # Panics
    /// Panics if `i >= wrap_num()`. Use [`MjsTendon::try_wrap`] for a fallible alternative.
    pub fn wrap(&self, i: usize) -> &MjsWrap {
        self.try_wrap(i).unwrap()
    }

    /// Fallible version of [`MjsTendon::wrap`].
    ///
    /// # Errors
    /// Returns [`MjEditError::IndexOutOfBounds`] if `i >= wrap_num()`.
    pub fn try_wrap(&self, i: usize) -> Result<&MjsWrap, MjEditError> {
        let len = self.wrap_num();
        // mjs_getWrap aborts the process via mju_error for an out-of-range index, so the bound is
        // checked here and turned into a recoverable error instead.
        if i >= len {
            return Err(MjEditError::IndexOutOfBounds { id: i, len });
        }
        let ptr = unsafe { mjs_getWrap(self, i as i32) };
        // SAFETY: index validated above; mjs_getWrap returns a non-null pointer for in-range indices.
        Ok(unsafe { &*ptr })
    }

    /// Return a mutable indexed wrap object.
    ///
    /// # Panics
    /// Panics if `i >= wrap_num()`. Use [`MjsTendon::try_wrap_mut`] for a fallible alternative.
    pub fn wrap_mut(&mut self, i: usize) -> &mut MjsWrap {
        self.try_wrap_mut(i).unwrap()
    }

    /// Fallible version of [`MjsTendon::wrap_mut`].
    ///
    /// # Errors
    /// Returns [`MjEditError::IndexOutOfBounds`] if `i >= wrap_num()`.
    pub fn try_wrap_mut(&mut self, i: usize) -> Result<&mut MjsWrap, MjEditError> {
        let len = self.wrap_num();
        if i >= len {
            return Err(MjEditError::IndexOutOfBounds { id: i, len });
        }
        let ptr = unsafe { mjs_getWrap(self, i as i32) };
        // SAFETY: see try_wrap().
        Ok(unsafe { &mut *ptr })
    }
}

/***************************
** Wrap specification
***************************/
mjs_struct!(Wrap);
impl MjsWrap {
    getter_setter! {
        [&] with, get, set, [
            type_ + _: MjtWrap; "wrap type.";
        ]
    }

    /// Return the side site element.
    pub fn side_site(&self) -> Option<&MjsSite> {
        let ptr = unsafe { mjs_getWrapSideSite(self) };
        if ptr.is_null() { None } else { Some(unsafe { &*ptr }) }
    }

    /// Return the side site element mutably.
    pub fn side_site_mut(&mut self) -> Option<&mut MjsSite> {
        let ptr = unsafe { mjs_getWrapSideSite(self) };
        if ptr.is_null() { None } else { Some(unsafe { &mut *ptr }) }
    }

    /// Return the wrap divisor.
    pub fn divisor(&self) -> f64 {
        unsafe { mjs_getWrapDivisor(self) }
    }

    /// Return the wrap coefficient.
    pub fn coef(&self) -> f64 {
        unsafe { mjs_getWrapCoef(self) }
    }
}

/***************************
** Numeric specification
***************************/
mjs_struct!(Numeric [SpecObject]);
impl MjsNumeric {
    getter_setter! {
        [&] with, get, set, [
            size: i32 { check_numeric_size, "[`MjEditError::InvalidParameter`] when the size is negative" } => MjEditError;     "size of the numeric array.";
        ]
    }

    vec_set_get! {
        data: f64; "initialization data.";
    }
}

/***************************
** Text specification
***************************/
mjs_struct!(Text [SpecObject]);
impl MjsText {
    string_set_get_with! {[&]
        data; "text string.";
    }
}

/***************************
** Tuple specification
***************************/
mjs_struct!(Tuple [SpecObject]);
impl MjsTuple {
    vec_set! {
        // `objtype` is stored as a raw C `int` and, at `compile()` time, used to index the model
        // compiler's `object_lists_` array (size `mjNOBJECT`) with no bounds check. Validating every
        // element against `mjNOBJECT` rules out the meta `MjtObj` variants (`mjOBJ_FRAME`,
        // `mjOBJ_DEFAULT`, `mjOBJ_MODEL`) that would otherwise cause an out-of-bounds read, which is
        // what lets this setter be safe.
        objtype: MjtObj => i32 { check_objtype, "[`MjEditError::InvalidParameter`] when any value is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])" } => MjEditError;
            "object types. Every value must be a real object type (an `MjtObj` below `mjNOBJECT`).";
    }

    vec_string_set_append! {
        objname; "object names.";
    }

    vec_set_get! {
        objprm: f64; "object parameters.";
    }
}

/***************************
** Key specification
***************************/
mjs_struct!(Key [SpecObject]);
impl MjsKey {
    getter_setter! {
        [&] with, get, set, [
            time: f64; "time."
        ]
    }

    vec_set_get! {
        qpos: f64; "qpos.";
        qvel: f64; "qvel.";
        act: f64; "act.";
        mpos: f64; "mocap pos.";
        mquat: f64; "mocap quat.";
        ctrl: f64; "ctrl.";
    }
}

/***************************
** Plugin specification
***************************/
mjs_struct!(Plugin [SpecObject]);
impl MjsPlugin {
    string_set_get_with! {[&]
        name; "instance name.";
        plugin_name; "plugin name.";
    }

    getter_setter! {
        [&] with, get, set, [
            active: bool; "is the plugin active.";
        ]
    }
}

/* Assets */

/***************************
** Mesh specification
***************************/
mjs_struct!(Mesh [SpecObject]);
impl MjsMesh {
    getter_setter! {
        [&] with, get, [
            refpos: &[f64; 3];            "reference position.";
            refquat: &[f64; 4];           "reference orientation.";
            scale: &[f64; 3];             "scale vector.";
        ]
    }

    getter_setter! {
        get, [
            plugin: &MjsPlugin;                     "sdf plugin.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            inertia: MjtMeshInertia;      "inertia type (convex, legacy, exact, shell).";
            maxhullvert: i32;             "maximum vertex count for the convex hull.";
            octree_maxdepth: i32;         "max octree depth.";
        ]
    }

    getter_setter! {
        [&] with, get,set, [
            smoothnormal: bool;           "do not exclude large-angle faces from normals.";
            needsdf: bool;                "compute sdf from mesh.";
        ]
    }

    string_set_get_with! {[&]
        content_type; "content type of file.";
        file; "mesh file.";
        material; "name of material.";
    }

    vec_set! {
        uservert: f32;               "user vertex data.";
        usernormal: f32;             "user normal data.";
        usertexcoord: f32;           "user texcoord data.";
        userface: i32;               "user vertex indices.";
    }

    vec_set! {
        [unsafe: "Every entry must be in `0..N`, where `N` is the number of user normals: the \
                  length of the slice passed to `set_usernormal` divided by 3 (each normal is 3 \
                  `f32`: x, y, z)."]
            userfacenormal: i32 => i32; "user face normal indices.";
        [unsafe: "Every entry must be in `0..ntexcoord` (the number of user texture coordinates), and \
                  the slice length must equal the length of the slice passed to `set_userface` (3 per \
                  face). Unlike face-normal data, MuJoCo does not validate the texcoord-index length, \
                  so an oversized slice overflows the model's face-texcoord buffer at compile time."]
            userfacetexcoord: i32 => i32; "user texcoord indices.";
    }
}

/***************************
** Hfield specification
***************************/
mjs_struct!(Hfield [SpecObject]);
impl MjsHfield {
    getter_setter! {
        [&] with, get, [
            size: &[f64; 4];              "size of the hfield.";
        ]
    }

    getter_setter! { [&] with, get, set, [
        nrow: i32;  "number of rows.";
        ncol: i32;  "number of columns.";
    ]}

    string_set_get_with! {[&]
        content_type; "content type of file.";
        file; "file: (nrow, ncol, [elevation data]).";
    }

    /// Sets `userdata`.
    pub fn set_userdata<T: AsRef<[f32]>>(&mut self, userdata: T) {
        // SAFETY: self.userdata is a valid mjFloatVec pointer for the lifetime of self.
        unsafe { write_mjs_vec_f32(userdata.as_ref(), self.userdata) };
    }
}

/***************************
** Skin specification
***************************/
mjs_struct!(Skin [SpecObject]);
impl MjsSkin {
    getter_setter! {
        [&] with, get, [
            rgba: &[f32; 4];    "rgba when material is omitted.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            inflate: f32;       "inflate in normal direction.";
            group: i32;         "group for visualization.";
        ]
    }

    string_set_get_with! {[&]
        material;               "name of material used for rendering.";
        file;                   "skin file.";
    }

    vec_string_set_append! {
        bodyname;               "body names.";
    }

    vec_set! {
        vert: f32;              "vertex positions.";
        texcoord: f32;          "texture coordinates.";
        bindpos: f32;           "bind pos.";
        bindquat: f32;          "bind quat.";
    }

    vec_set! {
        [
            unsafe:
                "The slice length must be a multiple of 3 and every entry must be in `0..nvert`  (the number of skin vertices)."
        ] face: i32 => i32; "faces.";
    }

    vec_vec_append! {
        vertid: i32;                     "vertex ids.";
        vertweight: f32;                 "vertex weights.";
    }
}

/***************************
** Texture specification
***************************/
mjs_struct!(Texture [SpecObject]);

/// # Note -- cube-map files
///
/// The `cubefiles` field is a **pre-sized** string vector (6 entries, one per cube face).
/// Use [`set_cubefile`](Self::set_cubefile) with a [`MjtCubeFace`] variant to assign a
/// file to a specific face. The bulk [`set_cubefiles`](Self::set_cubefiles) /
/// [`append_cubefiles`](Self::append_cubefiles) methods are also available but
/// operate on the vector as a whole.
impl MjsTexture {
    getter_setter! {
        [&] with, get, [
            rgb1: &[f64; 3];               "first color for builtin.";
            rgb2: &[f64; 3];               "second color for builtin.";
            markrgb: &[f64; 3];            "mark color.";
            gridsize: &[i32; 2];           "size of grid for composite file; (1,1)-repeat.";
            gridlayout: &[c_char; 12];     "row-major: L,R,F,B,U,D for faces; . for unused.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            random: f64;                  "probability of random dots.";
            width: i32;                   "image width.";
            height: i32;                  "image height.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            // `nchannel` must be `>= 3` whenever a builtin pattern is set
            // (`builtin != MjtBuiltin::mjBUILTIN_NONE`); this cross-field invariant is
            // enforced at the compile choke point in `MjSpec::compile`.
            nchannel: i32; "number of channels.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            type_ + _: MjtTexture [force];        "texture type.";
            colorspace: MjtColorSpace [force];    "colorspace.";
            builtin: MjtBuiltin [force];          "builtin type.";
            mark: MjtMark [force];                "mark type.";
        ]
    }

    vec_string_set_append! {
        cubefiles[MjtCubeFace] => cubefile; "different file for each side of the cube.";
    }

    getter_setter! {[&] with, get, set, [
        hflip: bool;    "horizontal flip.";
        vflip: bool;    "vertical flip.";
    ]}

    /// Sets texture `data`.
    pub fn set_data<T: bytemuck::NoUninit>(&mut self, data: &[T]) {
        // SAFETY: self.data is a valid mjByteVec pointer for the lifetime of self.
        unsafe { write_mjs_vec_byte(data, self.data) };
    }

    string_set_get_with! {[&]
        file; "png file to load; use for all sides of cube.";
        content_type; "content type of file.";
    }
}

/***************************
** Material specification
***************************/
mjs_struct!(Material [SpecObject]);

/// # Note -- texture assignment
///
/// The `textures` field is a **pre-sized** string vector (`mjNTEXROLE` entries, one per
/// [`MjtTextureRole`]). Use [`set_texture`](Self::set_texture) to assign a texture name
/// to a specific role (e.g. [`MjtTextureRole::mjTEXROLE_RGB`] for the base colour used by
/// the renderer). The bulk [`set_textures`](Self::set_textures) /
/// [`append_textures`](Self::append_textures) methods are also available but operate on
/// the vector as a whole and may disrupt the pre-sized layout.
impl MjsMaterial {
    getter_setter! {
        [&] with, get, [
            rgba: &[f32; 4];                               "rgba color.";
            texrepeat: &[f32; 2];    "texture repetition for 2D mapping.";
        ]
    }

    getter_setter! {[&] with, get, set, [
        texuniform: bool;       "make texture cube uniform.";
    ]}

    getter_setter! {
        [&] with, get, set, [
            emission: f32;                           "emission.";
            specular: f32;                           "specular.";
            shininess: f32;                         "shininess.";
            reflectance: f32;                     "reflectance.";
            metallic: f32;                           "metallic.";
            roughness: f32;                         "roughness.";
        ]
    }

    vec_string_set_append! {
        textures[MjtTextureRole] => texture; "names of textures (empty: none).";
    }
}


/***************************
** Body specification
***************************/
mjs_struct!(Body [SpecObject] {
    // Override the delete method to prevent deletion of world.
    /// Delete this body from its parent spec.
    ///
    /// # Deprecated
    /// This API is deprecated and will be removed in a future release.
    /// Use [`MjSpec::delete_element`](crate::wrappers::MjSpec::delete_element) instead.
    ///
    /// This method is inherently unsound: deleting a body mutates owner/ancestor graph
    /// structures outside the borrowed `&mut self` region.
    ///
    /// # Errors
    /// - Returns [`MjEditError::UnsupportedOperation`] if this is the world body.
    /// - Returns [`MjEditError::DeleteFailed`] if MuJoCo's internal deletion fails.
    ///
    /// # Safety
    /// This legacy method is not soundly callable; it exists only for backward compatibility.
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        if self.name() == "world" {
            return Err(MjEditError::UnsupportedOperation);
        }
        unsafe { SpecItem::__delete_default__(self) }
    }
});

impl MjsBody {
    add_x_method! { body, site, joint, geom, camera, light }

    /// Obtain an immutable reference to a child body with the given `name`.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn child(&self, name: &str) -> Option<&MjsBody> {
        let c_name = CString::new(name).unwrap();
        unsafe {
            let ptr = mjs_findChild(self, c_name.as_ptr());
            if ptr.is_null() { None } else { ptr.as_ref() }
        }
    }

    /// Obtain a mutable reference to a child body with the given `name`.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn child_mut(&mut self, name: &str) -> Option<&mut MjsBody> {
        let c_name = CString::new(name).unwrap();
        unsafe {
            let ptr = mjs_findChild(self, c_name.as_ptr());
            if ptr.is_null() { None } else { ptr.as_mut() }
        }
    }

    /// Dummy mutable FFI method used to simplify access through macros.
    ///
    /// # Safety
    /// Callers must ensure that any mutations performed through the returned reference
    /// preserve the invariants that MuJoCo expects for `mjsBody`.
    #[inline]
    unsafe fn ffi_mut(&mut self) -> &mut Self {
        self
    }

    // Special case
    /// Add and return a child frame.
    ///
    /// Delegates to [`Self::try_add_frame`] and panics if allocation fails.
    /// # Panics
    /// Panics if MuJoCo fails to allocate the frame.
    pub fn add_frame(&mut self) -> &mut MjsFrame {
        self.try_add_frame().expect("mjs_addFrame returned null; allocation failed")
    }

    /// Fallible version of [`Self::add_frame`].
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate
    /// the frame, instead of panicking.
    pub fn try_add_frame(&mut self) -> Result<&mut MjsFrame, MjEditError> {
        // SAFETY: ffi_mut() returns self unchanged; the coercion to *mut mjsBody is safe.
        // ptr::null_mut() for parentframe is valid: the MuJoCo API accepts null to mean
        // "attach directly to the body with no parent frame".
        let ptr = unsafe { mjs_addFrame(self.ffi_mut(), ptr::null_mut()) };
        // SAFETY: ptr.as_mut() returns None for null, handled by ok_or; when non-null the
        // pointee is properly aligned and initialized by C++ operator new, and freshly
        // allocated so no existing Rust reference aliases it for the returned lifetime.
        unsafe { ptr.as_mut() }.ok_or(MjEditError::AllocationFailed)
    }
}

impl MjsBody {
    // Complex types with mutable and immutable reference returns.
    getter_setter! {
        [&] with, get, [
            // body frame
            pos: &[f64; 3];                   "frame position.";
            quat: &[f64; 4];                  "frame orientation.";
            alt: &MjsOrientation;             "frame alternative orientation.";

            //inertial frame
            ipos: &[f64; 3];                  "inertial frame position.";
            iquat: &[f64; 4];                 "inertial frame orientation.";
            inertia: &[f64; 3];               "diagonal inertia (in i-frame).";
            ialt: &MjsOrientation;            "inertial frame alternative orientation.";
            fullinertia: &[f64; 6];           "non-axis-aligned inertia matrix.";
        ]
    }

    getter_setter! {
        get, [
            plugin: &MjsPlugin;                     "passive force plugin.";
        ]
    }

    // Plain types with normal getters and setters.
    getter_setter! {
        [&] with, get, set, [
            mass: f64;                     "mass.";
            gravcomp: f64;                 "gravity compensation.";
            sleep: MjtSleepPolicy;           "sleep policy.";
        ]
    }

    getter_setter! {
        [&] with, get, set, [
            mocap: bool;                   "whether this is a mocap body.";
            explicitinertial: bool;        "whether to save the body with explicit inertial clause.";
        ]
    }

    userdata_method!(f64);
}

/// Mutable iterator over items in [`MjsBody`].
#[derive(Debug)]
pub struct MjsBodyItemIterMut<'a, T> {
    /// NonNull pointer to the FFI type [`mjsBody`].
    /// [`MjsBody`] is its alias, thus storing it plainly
    /// would technically be UB as Rust can't see across
    /// boundary to verify . 
    ffi_ptr: *mut MjsBody,
    last: *mut mjsElement,
    recurse: bool,
    /// Used for generic implementation of iterator's methods.
    item_type: PhantomData<&'a mut T>
}

impl<'a, T: SpecObject> MjsBodyItemIterMut<'a, T> {
    fn new(root: &'a mut MjsBody, recurse: bool) -> Self {
        let last = unsafe { mjs_firstChild(root, T::OBJ_TYPE, recurse.into()) };
        Self { ffi_ptr: root, last, recurse, item_type: PhantomData }
    }
}

impl<'a, T: SpecObject + 'a> Iterator for MjsBodyItemIterMut<'a, T> {
    type Item = &'a mut T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }

        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_mut();
            self.last = mjs_nextChild(self.ffi_ptr, self.last, self.recurse.into());
            out
        }
    }
}

impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsBodyItemIterMut<'a, T> {}

/// Immutable iterator over items in [`MjsBody`].
#[derive(Debug, Clone)]
pub struct MjsBodyItemIter<'a, T> {
    ffi_ptr: *const MjsBody,
    last: *const mjsElement,
    recurse: bool,
    /// Used for generic implementation of iterator's methods.
    item_type: PhantomData<&'a T>
}


impl<'a, T: SpecObject> MjsBodyItemIter<'a, T> {
    fn new(root: &'a MjsBody, recurse: bool) -> Self {
        // SAFETY: mjs_firstChild requires a *mut pointer but does not mutate
        // the body. The const-to-mut cast is sound because no mutation occurs.
        let last = unsafe {
            mjs_firstChild(
                root,
                T::OBJ_TYPE,
                recurse.into()
            )
        };
        Self { ffi_ptr: root, last, recurse, item_type: PhantomData }
    }
}

impl<'a, T: SpecObject + 'a> Iterator for MjsBodyItemIter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last as *mut _).as_ref();
            // SAFETY: mjs_nextChild requires *mut but does not mutate. Cast is sound.
            self.last = mjs_nextChild(self.ffi_ptr, self.last, self.recurse.into());
            out
        }
    }
}

// Once self.last is null, next() always returns None.
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsBodyItemIter<'a, T> {}

/// Iterator methods.
impl MjsBody {
    body_get_iter! {[body, joint, geom, site, camera, light, frame] }
}

/******************************
** Tests
******************************/
#[cfg(test)]
mod tests {
    use std::io::Write;
    use std::path::{Path, PathBuf};
    use std::fs;

    use super::*;

    const MODEL: &str = "\
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\" pos=\".2 .2 .1\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball\" type=\"free\"/>
    </body>
    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" solref=\"0.004 1.0\"/>
  </worldbody>
</mujoco>";

    #[test]
    fn test_spec_authored_accessor() {
        use crate::mujoco_c::{mjtDisableBit, mjtEnableBit};

        // A model that explicitly authors option flags: disables the contact flag,
        // enables the energy flag, and disables actuator group 3.
        const AUTHORED: &str = "\
<mujoco>
  <option actuatorgroupdisable=\"3\">
    <flag contact=\"disable\" energy=\"enable\"/>
  </option>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\" pos=\".2 .2 .1\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint name=\"ball\" type=\"free\"/>
    </body>
  </worldbody>
</mujoco>";

        let contact = mjtDisableBit::mjDSBL_CONTACT as i32;
        let gravity = mjtDisableBit::mjDSBL_GRAVITY as i32;
        let energy = mjtEnableBit::mjENBL_ENERGY as i32;

        let spec = MjSpec::from_xml_string(AUTHORED).unwrap();
        let authored = spec.authored();

        // Flags that were explicitly set must be marked authored at the matching bit...
        assert_eq!(authored.disableflags() & contact, contact, "contact disable must be authored");
        assert_eq!(authored.enableflags() & energy, energy, "energy enable must be authored");
        assert_eq!(authored.disableactuator() & (1 << 3), 1 << 3, "actuator group 3 must be authored");
        // ...while flags that were never touched must stay unauthored.
        assert_eq!(authored.disableflags() & gravity, 0, "gravity was never authored");

        // A model that authors none of these leaves the bitmasks clear.
        let plain = MjSpec::from_xml_string(MODEL).unwrap();
        let plain_authored = plain.authored();
        assert_eq!(plain_authored.disableflags(), 0);
        assert_eq!(plain_authored.enableflags(), 0);
        assert_eq!(plain_authored.disableactuator(), 0);
    }

    #[test]
    fn test_parse_xml_string() {
        assert!(MjSpec::from_xml_string(MODEL).is_ok(), "failed to parse the model");
    }

    #[test]
    fn test_parse_xml_file() {
        const PATH: &str = "./mj_spec_test_parse_xml_file.xml";
        let mut file = fs::File::create(PATH).expect("file creation failed");
        file.write_all(MODEL.as_bytes()).expect("unable to write to file");
        file.flush().unwrap();

        let spec = MjSpec::from_xml(PATH);
        fs::remove_file(PATH).expect("file removal failed");
        assert!(spec.is_ok(), "failed to parse the model");
    }

    #[test]
    fn test_parse_xml_vfs() {
        const PATH: &str = "./mj_spec_test_parse_xml_vfs.xml";
        let mut vfs = MjVfs::new();
        vfs.add_from_buffer(PATH, MODEL.as_bytes()).unwrap();
        assert!(MjSpec::from_xml_vfs(PATH, &vfs).is_ok(), "failed to parse the model");
    }

    #[test]
    fn test_basic_edit_compile() {
        const TIMESTEP: f64 = 0.010;
        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        spec.option_mut().timestep = TIMESTEP;  // change time step to 10 ms.

        let compiled = spec.compile().expect("could not compile the model");
        assert_eq!(compiled.opt().timestep, TIMESTEP);

        spec.compile().unwrap();
    }

    #[test]
    fn test_model_name() {
        const DEFAULT_MODEL_NAME: &str = "MuJoCo Model";
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");

        /* Test read */
        assert_eq!(spec.modelname(), DEFAULT_MODEL_NAME);
        /* Test write */
        spec.set_modelname(NEW_MODEL_NAME);
        assert_eq!(spec.modelname(), NEW_MODEL_NAME);

        spec.compile().unwrap();
    }

    #[test]
    fn test_item_name() {
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let world = spec.world_body_mut();
        let body = world.add_body();
        assert_eq!(body.name(), "");
        body.set_name(NEW_MODEL_NAME).unwrap();
        assert_eq!(body.name(), NEW_MODEL_NAME);

        spec.compile().unwrap();
    }

    #[test]
    fn test_body_remove() {
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let world = spec.world_body_mut();
        let body = world.add_body();
        body.set_name(NEW_MODEL_NAME).unwrap();

        /* Test normal body deletion */
        let body_element = {
            let body = spec.body_mut(NEW_MODEL_NAME).expect("failed to obtain the body");
            body.element_mut_pointer()
        };
        assert!(unsafe { spec.delete_element(body_element) }.is_ok(), "failed to delete model");
        assert!(spec.body(NEW_MODEL_NAME).is_none(), "body was not removed from spec");

        /* Test world body deletion */
        let world_element = {
            let world = spec.world_body_mut();
            world.element_mut_pointer()
        };
        assert!(unsafe { spec.delete_element(world_element) }.is_err(), "the world model should not be deletable");

        spec.compile().unwrap();
    }

    #[test]
    fn test_joint_remove() {
        const NEW_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let world = spec.world_body_mut();
        let joint = world.add_joint();
        joint.set_name(NEW_NAME).unwrap();

        /* Test normal body deletion */
        let joint_element = {
            let joint = spec.joint_mut(NEW_NAME).expect("failed to obtain the body");
            joint.element_mut_pointer()
        };
        assert!(unsafe { spec.delete_element(joint_element) }.is_ok(), "failed to delete model");
        assert!(spec.joint(NEW_NAME).is_none(), "body was not removed fom spec");

        spec.compile().unwrap();
    }

    #[test]
    fn test_hfield_remove() {
        const NEW_NAME: &str = "Test hfield";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let hfield = spec.add_hfield();
        hfield.set_name(NEW_NAME).unwrap();

        /* Test normal hfield deletion */
        let hfield_element = {
            let hfield = spec.hfield_mut(NEW_NAME).expect("failed to obtain the hfield");
            hfield.element_mut_pointer()
        };
        assert!(unsafe { spec.delete_element(hfield_element) }.is_ok(), "failed to delete hfield");
        assert!(spec.hfield(NEW_NAME).is_none(), "hfield was not removed from spec");

        spec.compile().unwrap();
    }

    #[test]
    fn test_body_userdata() {
        const NEW_USERDATA: [f64; 3] = [1.0, 2.0, 3.0];

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let world = spec.world_body_mut();

        assert_eq!(world.userdata(), []);

        world.set_userdata(NEW_USERDATA);
        assert_eq!(world.userdata(), NEW_USERDATA);

        spec.compile().unwrap();
    }

    #[test]
    fn test_body_attrs() {
        const TEST_VALUE_F64: f64 = 5.25;

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let world = spec.world_body_mut();

        world.set_gravcomp(TEST_VALUE_F64);
        assert_eq!(world.gravcomp(), TEST_VALUE_F64);

        world.pos_mut()[0] = TEST_VALUE_F64;
        assert_eq!(world.pos()[0], TEST_VALUE_F64);

        spec.compile().unwrap();
    }

    #[test]
    fn test_default() {
        const DEFAULT_NAME: &str = "floor";
        const NOT_DEFAULT_NAME: &str = "floor-not";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");

        /* Test search */
        spec.add_default(DEFAULT_NAME, None);

        /* Test delete */
        assert!(spec.default(DEFAULT_NAME).is_some());
        assert!(spec.default(NOT_DEFAULT_NAME).is_none());

        let world = spec.world_body_mut();
        let some_body = world.add_body();
        some_body.add_joint().with_name("test");
        some_body.add_geom().with_size([0.010, 0.0, 0.0]);

        let actuator = spec.add_actuator()
            .with_trntype(MjtTrn::mjTRN_JOINT);
        actuator.set_target("test");
        
        assert!(actuator.set_default(DEFAULT_NAME).is_ok());

        spec.compile().unwrap();
    }

    #[test]
    fn test_actuator_set_to() {
        let mut spec = MjSpec::new();
        let body = spec.world_body_mut().add_body();
        body.add_geom().with_size([0.01, 0.0, 0.0]);
        body.add_joint().with_name("hinge").with_type(MjtJoint::mjJNT_HINGE);

        let actuator = spec.add_actuator().with_trntype(MjtTrn::mjTRN_JOINT);
        actuator.set_target("hinge");

        /* motor */
        actuator.set_to_motor();
        assert_eq!(actuator.gaintype(), MjtGain::mjGAIN_FIXED);
        assert_eq!(actuator.biastype(), MjtBias::mjBIAS_NONE);
        assert_eq!(actuator.dyntype(), MjtDyn::mjDYN_NONE);
        assert_eq!(actuator.gainprm()[0], 1.0);

        /* velocity servo */
        actuator.set_to_velocity(2.0);
        assert_eq!(actuator.gaintype(), MjtGain::mjGAIN_FIXED);
        assert_eq!(actuator.biastype(), MjtBias::mjBIAS_AFFINE);
        assert_eq!(actuator.gainprm()[0], 2.0);
        assert_eq!(actuator.biasprm()[2], -2.0);

        /* damper: negative feedback gain is rejected, otherwise affine gain */
        assert!(actuator.set_to_damper(-1.0).is_err());
        assert!(actuator.set_to_damper(5.0).is_ok());
        assert_eq!(actuator.gaintype(), MjtGain::mjGAIN_AFFINE);
        assert_eq!(actuator.gainprm()[2], -5.0);

        /* adhesion: negative gain is rejected */
        assert!(actuator.set_to_adhesion(-1.0).is_err());
        assert!(actuator.set_to_adhesion(1.0).is_ok());

        /* cylinder: filter dynamics, always succeeds (negative diameter keeps the area) */
        actuator.set_to_cylinder(0.1, 0.0, 1.0, -1.0);
        assert_eq!(actuator.dyntype(), MjtDyn::mjDYN_FILTER);

        /* muscle: negative tausmooth is rejected; negative entries keep MuJoCo's defaults */
        assert!(actuator.set_to_muscle([-1.0, -1.0], -1.0, [-1.0, -1.0], -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).is_err());
        actuator.set_to_muscle([-1.0, -1.0], 0.0, [-1.0, -1.0], -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).unwrap();
        assert_eq!(actuator.gaintype(), MjtGain::mjGAIN_MUSCLE);
        assert_eq!(actuator.biastype(), MjtBias::mjBIAS_MUSCLE);
        assert_eq!(actuator.dyntype(), MjtDyn::mjDYN_MUSCLE);

        /* position (builder API): kv and dampratio are mutually exclusive */
        assert!(actuator.set_to_position(
            PositionConfig::default().with_kp(1.0).with_kv(1.0).with_dampratio(1.0)
        ).is_err());
        actuator.set_to_position(PositionConfig::default().with_kp(1.0).with_kv(1.0)).unwrap();

        /* integrated velocity: integrator dynamics */
        actuator.set_to_int_velocity(IntVelocityConfig::default().with_kp(1.0)).unwrap();
        assert_eq!(actuator.dyntype(), MjtDyn::mjDYN_INTEGRATOR);

        /* dc motor (builder API): without a motor constant MuJoCo cannot derive K > 0 */
        assert!(actuator.set_to_dc_motor(DcMotorConfig::default()).is_err());
        actuator.set_to_dc_motor(
            DcMotorConfig::default().with_motorconst([1.0, 1.0]).with_resistance(1.0)
        ).unwrap();
        assert_eq!(actuator.gaintype(), MjtGain::mjGAIN_DCMOTOR);
        assert_eq!(actuator.biastype(), MjtBias::mjBIAS_DCMOTOR);
        assert_eq!(actuator.dyntype(), MjtDyn::mjDYN_DCMOTOR);

        /* reset to a plain motor and confirm the spec still compiles */
        actuator.set_to_motor();
        actuator.set_actearly(false);
        actuator.set_ctrllimited(MjtLimited::mjLIMITED_FALSE);
        spec.compile().unwrap();
    }

    #[test]
    fn test_save() {
        const EXPECTED_XML: &str = "\
<mujoco model=\"MuJoCo Model\">
  <compiler angle=\"radian\"/>

  <worldbody>
    <body>
      <geom size=\"0.01\"/>
      <site pos=\"0 0 0\"/>
      <camera pos=\"0 0 0\"/>
      <light pos=\"0 0 0\" dir=\"0 0 -1\"/>
    </body>
  </worldbody>
</mujoco>
";

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        let body = world.add_body();
        body.add_camera();
        body.add_geom().with_size([0.010, 0.0, 0.0]);
        body.add_light();
        body.add_site();

        spec.compile().unwrap();
        assert_eq!(spec.save_xml_string(1000).unwrap(), EXPECTED_XML);

        spec.compile().unwrap();
    }

    /// `save_xml_string` with a 1-byte buffer must return `XmlBufferTooSmall` and
    /// the reported `required_size` must be enough to succeed on retry.
    #[test]
    fn test_save_xml_string_buffer_too_small() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_body().add_geom().with_size([0.01, 0.0, 0.0]);
        spec.compile().unwrap();

        let err = spec.save_xml_string(1)
            .expect_err("expected XmlBufferTooSmall with a 1-byte buffer");
        let required_size = match err {
            MjEditError::XmlBufferTooSmall { required_size } => required_size,
            other => panic!("expected XmlBufferTooSmall, got {other:?}"),
        };
        assert!(required_size > 1, "required_size must exceed the original 1-byte buffer");

        // Retry with the size MuJoCo reported plus one extra byte for safety
        // (MuJoCo uses a strict less-than comparison, so the buffer must be
        // at least required_size + 1 to succeed).
        let xml = spec.save_xml_string(required_size + 1)
            .expect("save_xml_string should succeed with required_size + 1 bytes");
        assert!(!xml.is_empty(), "saved XML must be non-empty");
    }

    #[test]
    fn test_site() {
        const TEST_MATERIAL: &str = "material 1";
        const TEST_POSITION: [f64; 3] = [1.0, 2.0, 3.0];
        const SITE_NAME: &str = "test_site";

        let mut spec = MjSpec::new();

        /* add material */
        spec.add_material().with_name(TEST_MATERIAL);

        /* add site */
        let world = spec.world_body_mut();
        world.add_site()
            .with_name(SITE_NAME);
        let site = spec.site_mut(SITE_NAME).unwrap();

        /* material */
        assert_eq!(site.material(), "");
        site.set_material(TEST_MATERIAL);
        assert_eq!(site.material(), TEST_MATERIAL);

        /* userdata */
        let test_userdata: Vec<f64> = vec![0.0; 5];
        assert_eq!(site.userdata(), []);
        site.set_userdata(&test_userdata);
        assert_eq!(site.userdata(), test_userdata);

        /* position */
        assert_eq!(site.pos(), &[0.0; 3]);
        *site.pos_mut() = TEST_POSITION;
        assert_eq!(site.pos(), &TEST_POSITION);

        spec.compile().unwrap();
    }

    #[test]
    fn test_frame() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut()
            .with_gravcomp(10.0);

        world.add_frame()
            .with_name("frame_a")
            .with_pos([0.5, 0.5, 0.05])
            .add_body()
            .add_geom()
            .with_size([1.0, 0.0, 0.0]);

        assert!(spec.frame("frame_a").is_some());
        assert!(spec.frame_mut("frame_a").is_some());

        spec.compile().unwrap();
    }

    #[test]
    fn test_wrap() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        let body1= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body1.add_geom().with_size([0.010;3]);
        body1.add_site().with_name("ball1");
        body1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let body2= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body2.add_geom().with_size([0.010;3]);
        body2.add_site().with_name("ball2");
        body2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let tendon = spec.add_tendon()
            .with_range([0.0, 0.25])
            .with_rgba([1.0, 0.5, 0.0, 1.0]);  // orange
        tendon.wrap_site("ball1");
        tendon.wrap_site("ball2");

        spec.world_body_mut().add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        spec.compile().unwrap();
    }

    #[test]
    fn test_body_child_and_id() {
        let mut spec = MjSpec::new();
        let parent = spec.world_body_mut().add_body().with_name("parent");
        parent.add_body().with_name("child");

        let parent_ref = spec.body("parent").unwrap();
        assert!(parent_ref.child("child").is_some());
        assert!(parent_ref.child("missing").is_none());
        assert_eq!(parent_ref.id(), None);

        let parent_mut = spec.body_mut("parent").unwrap();
        assert!(parent_mut.child_mut("child").is_some());
        assert!(parent_mut.child_mut("missing").is_none());

        spec.compile().unwrap();
        assert!(spec.body("parent").unwrap().id().is_some());
    }

    #[test]
    fn test_geom() {
        const GEOM_NAME: &str = "test_geom";
        const GEOM_INVALID_NAME: &str = "geom_test";
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_geom()
            .with_name(GEOM_NAME);

        assert!(spec.geom(GEOM_NAME).is_some());
        assert!(spec.geom(GEOM_INVALID_NAME).is_none());
    }

    #[test]
    fn test_camera() {
        const CAMERA_NAME: &str = "test_cam";
        const CAMERA_INVALID_NAME: &str = "cam_test";
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_camera()
            .with_name(CAMERA_NAME);

        assert!(spec.camera(CAMERA_NAME).is_some());
        assert!(spec.camera(CAMERA_INVALID_NAME).is_none());
    }

    #[test]
    fn test_light() {
        const LIGHT_NAME: &str = "test_light";
        const LIGHT_INVALID_NAME: &str = "light_test";
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_light()
            .with_name(LIGHT_NAME);

        assert!(spec.light(LIGHT_NAME).is_some());
        assert!(spec.light(LIGHT_INVALID_NAME).is_none());
    }

    #[test]
    fn test_exclude() {
        const EXCLUDE_NAME: &str = "test_exclude";
        const EXCLUDE_INVALID_NAME: &str = "exclude_test";
        let mut spec = MjSpec::new();

        spec.world_body_mut().add_body().with_name("body1-left");
        spec.world_body_mut().add_body().with_name("body2-right");

        spec.add_exclude()
            .with_name(EXCLUDE_NAME)
            .with_bodyname1("body1-left")
            .with_bodyname2("body2-right");

        assert!(spec.exclude(EXCLUDE_NAME).is_some());
        assert!(spec.exclude(EXCLUDE_INVALID_NAME).is_none());

        assert!(spec.compile().is_ok());
    }

    #[test]
    fn test_mesh() {
        let mut spec = MjSpec::new();
        let mesh = spec.add_mesh();
        assert!(!mesh.needsdf());
        mesh.set_needsdf(true);
        assert!(mesh.needsdf());

        assert!(!mesh.smoothnormal());
        mesh.set_smoothnormal(true);
        assert!(mesh.smoothnormal());
    }

    #[test]
    fn test_iteration() {
        const LAST_BODY_NAME: &str = "subbody";
        const LAST_WORLD_BODY_NAME: &str = "body2";
        const N_GEOM:   usize = 3;
        const N_BODY:   usize = 4;  // three added + world
        const N_SITE:   usize = 2;
        const N_TENDON: usize = 1;
        const N_MESH:   usize = 0;

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        let body1= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body1.add_geom().with_size([0.010;3]);
        body1.add_site().with_name("ball1");
        body1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let body2= world.add_body().with_pos([0.0, 0.0, 0.5]).with_name(LAST_WORLD_BODY_NAME);
        body2.add_geom().with_size([0.010;3]);
        body2.add_site().with_name("ball2");
        body2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        body2.add_body().with_name(LAST_BODY_NAME);

        let tendon = spec.add_tendon()
            .with_range([0.0, 0.25])
            .with_rgba([1.0, 0.5, 0.0, 1.0]);  // orange
        tendon.wrap_site("ball1");
        tendon.wrap_site("ball2");

        spec.world_body_mut().add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        // Iter MjSpec
        assert_eq!(spec.geom_iter_mut().count(), N_GEOM);
        assert_eq!(spec.body_iter_mut().count(), N_BODY);
        assert_eq!(spec.site_iter_mut().count(), N_SITE);
        assert_eq!(spec.tendon_iter_mut().count(), N_TENDON);
        assert_eq!(spec.mesh_iter_mut().count(), N_MESH);
        assert_eq!(spec.body_iter_mut().last().unwrap().name(), LAST_BODY_NAME);

        // Iter MjsBody
        let world = spec.world_body_mut();
        assert_eq!(world.geom_iter_mut(true).count(), N_GEOM);
        assert_eq!(world.body_iter_mut(true).count(), N_BODY - 1);  // world must now be excluded
        assert_eq!(world.site_iter_mut(true).count(), N_SITE);
        assert_eq!(world.body_iter_mut(false).last().unwrap().name(), LAST_WORLD_BODY_NAME);
    }

    /// Tests wrapper method of [`mj_parse`] with VFS.
    #[test]
    fn test_parse_vfs() {
        let mut vfs = MjVfs::new();
        vfs.add_from_buffer("hello.xml", MODEL.as_bytes()).unwrap();
        let mut spec = MjSpec::from_parse_vfs("hello.xml", "XML", &vfs).unwrap();
        let model = spec.compile().unwrap();
        assert!(model.geom("floor1").is_some());
    }

    /// Tests wrapper method of [`mj_parse`] without VFS.
    #[test]
    fn test_parse_file() {
        std::fs::write("test_parse_vfs.xml", MODEL).unwrap();
        let mut spec = MjSpec::from_parse("test_parse_vfs.xml", "XML").unwrap();
        std::fs::remove_file("test_parse_vfs.xml").unwrap();
        let model = spec.compile().unwrap();
        assert!(model.geom("floor1").is_some());
    }

    #[test]
    fn test_tendon_wrap_methods() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_body().with_name("body1");
        spec.world_body_mut().add_body().with_name("body2");
        spec.world_body_mut().add_site().with_name("site1");

        let tendon = spec.add_tendon();
        tendon.wrap_site("site1");
        tendon.wrap_joint("joint1", 0.5);
        tendon.wrap_pulley(1.5);

        assert_eq!(tendon.wrap_num(), 3);

        let wrap = tendon.wrap(1);
        assert_eq!(wrap.coef(), 0.5);

        let wrap_pulley = tendon.wrap(2);
        assert_eq!(wrap_pulley.divisor(), 1.5);
    }

    #[test]
    fn test_tendon_wrap_out_of_bounds() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_site().with_name("site1");

        let tendon = spec.add_tendon();
        tendon.wrap_site("site1");
        assert_eq!(tendon.wrap_num(), 1);

        // Index 3 is out of range; the fallible accessor reports it rather than aborting in C.
        match tendon.try_wrap(3) {
            Err(MjEditError::IndexOutOfBounds { id, len }) => {
                assert_eq!(id, 3);
                assert_eq!(len, 1);
            }
            _ => panic!("expected IndexOutOfBounds"),
        }
    }

    #[test]
    #[should_panic]
    fn test_tendon_wrap_out_of_bounds_panics() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_site().with_name("site1");

        let tendon = spec.add_tendon();
        tendon.wrap_site("site1");

        // The panicking accessor must panic in Rust, not abort the process in C.
        let _ = tendon.wrap(3);
    }

    #[test]
    fn test_numeric_vec() {
        let mut spec = MjSpec::new();
        let numeric = spec.add_numeric();
        let name = "test_numeric";
        numeric.set_name(name).unwrap();
        assert_eq!(numeric.name(), name);

        let data = [1.5, 2.5, 3.5, 4.5];
        numeric.set_data(&data);
        assert_eq!(numeric.data(), &data);

        spec.compile().unwrap();
    }

    #[test]
    fn test_text_string() {
        let mut spec = MjSpec::new();
        let text = spec.add_text();
        let name = "test_text";
        text.set_name(name).unwrap();
        assert_eq!(text.name(), name);

        let content = "Hello MuJoCo!";
        text.set_data(content);
        assert_eq!(text.data(), content);

        spec.compile().unwrap();
    }

    #[test]
    fn test_tuple_names_and_params() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_body().with_name("body1");
        spec.world_body_mut().add_body().with_name("body2");

        let tuple_name = "test_tuple";
        let obj_param = [1.0, 2.0];

        let tuple = spec.add_tuple();
        tuple.set_name(tuple_name).unwrap();
        assert_eq!(tuple.name(), tuple_name);

        tuple.set_objname("body1 body2");
        tuple.set_objprm(&obj_param);
        tuple.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_BODY]).unwrap();

        assert_eq!(tuple.objprm(), &obj_param);

        spec.compile().unwrap();

        // Verify via XML as objname has no spec getter
        let xml = spec.save_xml_string(2000).unwrap();
        assert!(xml.contains("objname=\"body1\""));
        assert!(xml.contains("objname=\"body2\""));
        assert!(xml.contains("prm=\"1\""));
        assert!(xml.contains("prm=\"2\""));
    }

    /// Tests that wrapping sites on a tendon produces a compiled model
    /// with correct ntendon, nwrap, wrap types, and wrap object IDs.
    #[test]
    fn test_tendon_wrap_site_compiled_model() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();

        let b1 = world.add_body().with_pos([0.0, 0.0, 0.5]);
        b1.add_geom().with_size([0.01; 3]);
        b1.add_site().with_name("s1");
        b1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let b2 = world.add_body().with_pos([1.0, 0.0, 0.5]);
        b2.add_geom().with_size([0.01; 3]);
        b2.add_site().with_name("s2");
        b2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        world.add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        let tendon = spec.add_tendon().with_range([0.0, 1.0]);
        tendon.wrap_site("s1");
        tendon.wrap_site("s2");

        let model = spec.compile().unwrap();

        assert_eq!(model.ffi().ntendon, 1, "expected one tendon");
        assert_eq!(model.ffi().nwrap, 2, "expected two wrap elements");

        // Verify wrap types are mjWRAP_SITE (= 1)
        let wrap_types = model.wrap_type();
        assert_eq!(wrap_types[0], MjtWrap::mjWRAP_SITE);
        assert_eq!(wrap_types[1], MjtWrap::mjWRAP_SITE);

        // Verify wrap object IDs point to the correct sites
        let wrap_objid = model.wrap_objid();
        let s1_id = model.site("s1").unwrap().id as i32;
        let s2_id = model.site("s2").unwrap().id as i32;
        assert_eq!(wrap_objid[0], s1_id);
        assert_eq!(wrap_objid[1], s2_id);
    }

    /// Test that MjsTendon `limited` correctly round-trips all three enum states
    /// (FALSE, TRUE, AUTO), which would fail if the field were `bool`.
    #[test]
    fn test_tendon_limited_tristate() {
        use crate::mujoco_c::mjtLimited_::*;

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();

        let b1 = world.add_body().with_pos([0.0, 0.0, 0.5]);
        b1.add_geom().with_size([0.01; 3]);
        b1.add_site().with_name("s1");
        b1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let b2 = world.add_body().with_pos([1.0, 0.0, 0.5]);
        b2.add_geom().with_size([0.01; 3]);
        b2.add_site().with_name("s2");
        b2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        world.add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        // Create 3 tendons, one for each LIMITED enum value
        for (name, val) in [("t_false", mjLIMITED_FALSE), ("t_true", mjLIMITED_TRUE), ("t_auto", mjLIMITED_AUTO)] {
            let t = spec.add_tendon()
                .with_name(name)
                .with_range([0.0, 1.0])
                .with_limited(val);
            t.wrap_site("s1");
            t.wrap_site("s2");
        }

        // Verify before compilation: getter round-trip
        for (name, expected) in [("t_false", mjLIMITED_FALSE), ("t_true", mjLIMITED_TRUE), ("t_auto", mjLIMITED_AUTO)] {
            let t = spec.tendon(name).expect("tendon not found");
            assert_eq!(t.limited(), expected,
                "Before compile: tendon '{}' limited should be {:?}", name, expected);
        }

        // Compile and verify the compiled model's tendon_limited field
        let model = spec.compile().unwrap();
        let tendon_limited = model.tendon_limited();
        // After compilation, enum values resolve to bool: FALSE->false, TRUE->true, AUTO->resolved
        assert!(!tendon_limited[0], "Compiled tendon 0 limited should be false");
        assert!(tendon_limited[1], "Compiled tendon 1 limited should be true");
        // AUTO resolves to a concrete bool in the compiled model (always true or false)
        let _ = tendon_limited[2];
    }

    /// Test that MjsTendon `actfrclimited` correctly round-trips all three enum
    /// states (FALSE, TRUE, AUTO).
    #[test]
    fn test_tendon_actfrclimited_tristate() {
        use crate::mujoco_c::mjtLimited_::*;

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();

        let b1 = world.add_body().with_pos([0.0, 0.0, 0.5]);
        b1.add_geom().with_size([0.01; 3]);
        b1.add_site().with_name("s1");
        b1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let b2 = world.add_body().with_pos([1.0, 0.0, 0.5]);
        b2.add_geom().with_size([0.01; 3]);
        b2.add_site().with_name("s2");
        b2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        world.add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        // Set actfrclimited to each variant (with actfrcrange for TRUE/AUTO)
        for (name, val) in [("t_false", mjLIMITED_FALSE), ("t_true", mjLIMITED_TRUE), ("t_auto", mjLIMITED_AUTO)] {
            let t = spec.add_tendon()
                .with_name(name)
                .with_range([0.0, 1.0])
                .with_actfrcrange([-1.0, 1.0])
                .with_actfrclimited(val);
            t.wrap_site("s1");
            t.wrap_site("s2");
        }

        // Verify round-trip before compilation
        for (name, expected) in [("t_false", mjLIMITED_FALSE), ("t_true", mjLIMITED_TRUE), ("t_auto", mjLIMITED_AUTO)] {
            let t = spec.tendon(name).expect("tendon not found");
            assert_eq!(t.actfrclimited(), expected,
                "Before compile: tendon '{}' actfrclimited should be {:?}", name, expected);
        }

        // Must compile without error
        spec.compile().unwrap();
    }

    /// Test that MjsJoint `align` correctly round-trips all three enum states
    /// (FALSE, TRUE, AUTO), which would fail if the field were `i32`.
    #[test]
    fn test_joint_align_tristate() {
        use crate::mujoco_c::mjtAlignFree_::*;

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        world.add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        // Create 3 free joints, one for each align state
        for (name, val) in [("j_false", mjALIGNFREE_FALSE), ("j_true", mjALIGNFREE_TRUE), ("j_auto", mjALIGNFREE_AUTO)] {
            let body = world.add_body().with_pos([0.0, 0.0, 1.0]);
            body.add_geom().with_size([0.1; 3]);
            body.add_joint()
                .with_name(name)
                .with_type(MjtJoint::mjJNT_FREE)
                .with_align(val);
        }

        // Verify round-trip before compilation
        for (name, expected) in [("j_false", mjALIGNFREE_FALSE), ("j_true", mjALIGNFREE_TRUE), ("j_auto", mjALIGNFREE_AUTO)] {
            let j = spec.joint(name).expect("joint not found");
            assert_eq!(j.align(), expected,
                "Before compile: joint '{}' align should be {:?}", name, expected);
        }

        // Compile and verify -- AUTO should resolve, FALSE/TRUE should remain
        let model = spec.compile().unwrap();
        let jnt_count = model.ffi().njnt as usize;
        assert_eq!(jnt_count, 3, "expected 3 joints");

        // Must compile without error -- the key correctness is the round-trip above;
        // compilation proves MuJoCo accepted the enum values.
    }

    /// Test that MjsJoint `limited` also correctly round-trips all three states
    /// since it was also changed to MjtLimited.
    #[test]
    fn test_joint_limited_tristate() {
        use crate::mujoco_c::mjtLimited_::*;

        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        world.add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        for (name, val) in [("j_false", mjLIMITED_FALSE), ("j_true", mjLIMITED_TRUE), ("j_auto", mjLIMITED_AUTO)] {
            let body = world.add_body().with_pos([0.0, 0.0, 1.0]);
            body.add_geom().with_size([0.1; 3]);
            body.add_joint()
                .with_name(name)
                .with_type(MjtJoint::mjJNT_SLIDE)
                .with_range([0.0, 1.0])
                .with_limited(val);
        }

        // Verify round-trip before compilation
        for (name, expected) in [("j_false", mjLIMITED_FALSE), ("j_true", mjLIMITED_TRUE), ("j_auto", mjLIMITED_AUTO)] {
            let j = spec.joint(name).expect("joint not found");
            assert_eq!(j.limited(), expected,
                "Before compile: joint '{}' limited should be {:?}", name, expected);
        }

        // Compile -- AUTO resolves based on range presence
        let model = spec.compile().unwrap();
        let jnt_limited = model.jnt_limited();
        assert!(!jnt_limited[0]);
        assert!(jnt_limited[1]);
        // AUTO with range present should resolve to true
        assert!(jnt_limited[2], "Joint with limited=AUTO and range should resolve to true");
    }

    /// Parsing invalid XML must return Err with a non-empty message.
    #[test]
    fn test_parse_xml_string_invalid() {
        let result = MjSpec::from_xml_string("<not valid mujoco xml>");
        assert!(result.is_err(), "parsing invalid XML must return Err");
        let msg = result.unwrap_err().to_string();
        assert!(!msg.is_empty(), "error message must not be empty for invalid XML");
    }

    /// Parsing via VFS must produce a model with the expected properties, not just succeed.
    #[test]
    fn test_parse_xml_vfs_content() {
        const PATH: &str = "./mj_spec_test_parse_xml_vfs_content.xml";
        let mut vfs = MjVfs::new();
        vfs.add_from_buffer(PATH, MODEL.as_bytes()).unwrap();
        let mut spec = MjSpec::from_xml_vfs(PATH, &vfs).expect("VFS parse failed");
        let model = spec.compile().expect("compile failed");

        // MODEL has: worldbody + 1 named body ("ball") = 2 bodies total (world + ball)
        assert_eq!(model.ffi().nbody, 2, "expected 2 bodies (world + ball)");
        // MODEL has: 1 free joint on "ball"
        assert_eq!(model.ffi().njnt, 1, "expected 1 joint");
        // MODEL has: 1 sphere geom + 1 plane geom = 2 geoms
        assert_eq!(model.ffi().ngeom, 2, "expected 2 geoms (sphere + floor)");
    }

    /// Parsing via XML file must produce a model with the expected properties.
    #[test]
    fn test_parse_xml_file_content() {
        const PATH: &str = "./mj_spec_test_parse_xml_file_content.xml";
        let mut file = fs::File::create(PATH).expect("file creation failed");
        file.write_all(MODEL.as_bytes()).expect("unable to write");
        file.flush().unwrap();

        let result = MjSpec::from_xml(PATH);
        fs::remove_file(PATH).expect("file removal failed");

        let mut spec = result.expect("file parse failed");
        let model = spec.compile().expect("compile failed");

        assert_eq!(model.ffi().nbody, 2, "expected 2 bodies (world + ball)");
        assert_eq!(model.ffi().njnt, 1, "expected 1 joint");
        assert_eq!(model.ffi().ngeom, 2, "expected 2 geoms (sphere + floor)");
    }

    /// `from_parse` accepts `PathBuf` and `&Path` in addition to `&str`.
    #[test]
    fn test_from_parse_path_types() {
        const PATH: &str = "./mj_spec_test_from_parse_path_types.xml";
        let mut file = fs::File::create(PATH).expect("file creation failed");
        file.write_all(MODEL.as_bytes()).expect("write failed");
        file.flush().unwrap();

        // &str
        assert!(MjSpec::from_parse(PATH, "").is_ok());
        // String
        assert!(MjSpec::from_parse(String::from(PATH), "").is_ok());
        // &Path
        assert!(MjSpec::from_parse(Path::new(PATH), "").is_ok());
        // PathBuf
        assert!(MjSpec::from_parse(PathBuf::from(PATH), "").is_ok());

        fs::remove_file(PATH).expect("file removal failed");
    }

    /// `from_parse_vfs` accepts `PathBuf` and `&Path`.
    #[test]
    fn test_from_parse_vfs_path_types() {
        const PATH: &str = "./mj_spec_test_from_parse_vfs_path_types.xml";
        let mut vfs = MjVfs::new();
        vfs.add_from_buffer(PATH, MODEL.as_bytes()).unwrap();

        // &str
        assert!(MjSpec::from_parse_vfs(PATH, "", &vfs).is_ok());
        // PathBuf
        assert!(MjSpec::from_parse_vfs(PathBuf::from(PATH), "", &vfs).is_ok());
        // &Path
        assert!(MjSpec::from_parse_vfs(Path::new(PATH), "", &vfs).is_ok());
    }

    /// `save_xml` accepts `PathBuf` and `&Path` in addition to `&str`.
    #[test]
    fn test_save_xml_path_types() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_body().add_geom().with_size([0.01, 0.0, 0.0]);
        spec.compile().unwrap();

        let paths: [PathBuf; 3] = [
            PathBuf::from("./mj_spec_save_xml_str.xml"),
            PathBuf::from("./mj_spec_save_xml_pathbuf.xml"),
            PathBuf::from("./mj_spec_save_xml_path.xml"),
        ];

        // &str
        spec.save_xml(paths[0].to_str().unwrap()).unwrap();
        // PathBuf
        spec.save_xml(paths[1].clone()).unwrap();
        // &Path
        spec.save_xml(paths[2].as_path()).unwrap();

        for p in &paths {
            let content = fs::read_to_string(p).expect("saved file should be readable");
            assert!(content.contains("<mujoco"), "saved XML should contain <mujoco tag");
            fs::remove_file(p).expect("cleanup failed");
        }
    }

    #[test]
    fn test_material_set_texture() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();
        world.add_geom()
            .with_type(MjtGeom::mjGEOM_PLANE)
            .with_size([1.0, 1.0, 0.01])
            .with_material("floor");

        spec.add_texture()
            .with_name("floor")
            .with_type(MjtTexture::mjTEXTURE_2D)
            .with_builtin(MjtBuiltin::mjBUILTIN_CHECKER)
            .with_rgb1([0.9, 0.9, 0.9])
            .with_rgb2([0.1, 0.1, 0.1])
            .with_width(512)
            .with_height(512);

        let mat = spec.add_material().with_name("floor");
        mat.set_texture(MjtTextureRole::mjTEXROLE_RGB, "floor");

        let model = spec.compile().unwrap();
        let xml = spec.save_xml_string(8192).unwrap();
        assert!(xml.contains("texture=\"floor\""), "XML should reference the floor texture");

        let mat_info = model.material("floor").unwrap();
        let mat_view = mat_info.view(&model);
        let tex_id = mat_view.texid;
        assert_ne!(tex_id[MjtTextureRole::mjTEXROLE_RGB as usize], -1,
            "RGB texture slot should be resolved (not -1)");
    }

    /// A builtin texture with `nchannel < 3` must be rejected by `compile()` rather than
    /// heap-overflowing in MuJoCo's builtin generators (which write 3 bytes per pixel).
    #[test]
    fn test_builtin_texture_nchannel_rejected() {
        let mut spec = MjSpec::new();
        spec.add_texture()
            .with_name("badtex")
            .with_type(MjtTexture::mjTEXTURE_2D)
            .with_builtin(MjtBuiltin::mjBUILTIN_CHECKER)
            .with_rgb1([0.9, 0.9, 0.9])
            .with_rgb2([0.1, 0.1, 0.1])
            .with_width(64)
            .with_height(64)
            .set_nchannel(1);

        let err = spec.compile().unwrap_err();
        assert!(matches!(&err, MjEditError::CompileFailed(msg) if msg.contains("nchannel")),
            "compile must reject nchannel < 3 builtin texture, got {err:?}");

        // nchannel == 3 (and a builtin pattern) compiles cleanly.
        let mut ok_spec = MjSpec::new();
        ok_spec.add_texture()
            .with_name("goodtex")
            .with_type(MjtTexture::mjTEXTURE_2D)
            .with_builtin(MjtBuiltin::mjBUILTIN_CHECKER)
            .with_rgb1([0.9, 0.9, 0.9])
            .with_rgb2([0.1, 0.1, 0.1])
            .with_width(64)
            .with_height(64)
            .set_nchannel(3);
        assert!(ok_spec.compile().is_ok(), "nchannel == 3 builtin texture should compile");

        // nchannel < 3 with NO builtin pattern must not trip this guard. Such a texture still
        // fails to compile (it has no pixel source), but with MuJoCo's own error message rather
        // than the nchannel guard's, proving the guard is correctly scoped to the builtin path.
        let mut no_builtin = MjSpec::new();
        no_builtin.add_texture()
            .with_name("plain")
            .with_type(MjtTexture::mjTEXTURE_2D)
            .with_width(64)
            .with_height(64)
            .set_nchannel(1);
        assert!(matches!(no_builtin.compile().unwrap_err(),
                MjEditError::CompileFailed(msg) if !msg.contains("nchannel")),
            "nchannel < 3 without a builtin pattern should not trip the nchannel guard");
    }

    /// Verifies `MjsFlex::cellcount` (read-only `&[i32; 3]`) and `order` (read-write `i32`)
    /// by parsing a minimal flexcomp model and reading/writing through the spec.
    #[test]
    fn test_mjs_flex_cellcount_and_order() {
        const FLEX_MODEL: &str = "\
<mujoco>\
  <worldbody>\
    <body name=\"pin\" pos=\"0 0 1\">\
      <flexcomp type=\"grid\" count=\"3 3 1\" spacing=\".1 .1 .1\" mass=\"1\"\
                name=\"myflex\" radius=\"0.001\" dim=\"2\">\
        <elasticity young=\"1e4\" poisson=\"0.0\"/>\
      </flexcomp>\
    </body>\
  </worldbody>\
</mujoco>";

        let mut spec = MjSpec::from_xml_string(FLEX_MODEL).expect("failed to parse flex model");
        let flex = spec.flex("myflex").expect("flex 'myflex' not found in spec");

        /* Verify field dimensions */
        assert_eq!(flex.cellcount().len(), 3);

        /* Verify write-read roundtrip for order */
        {
            let flex_mut = spec.flex_mut("myflex").unwrap();
            flex_mut.set_order(2);
        }
        assert_eq!(spec.flex("myflex").unwrap().order(), 2);

        {
            let flex_mut = spec.flex_mut("myflex").unwrap();
            flex_mut.set_order(1);
        }
        assert_eq!(spec.flex("myflex").unwrap().order(), 1);
    }

    /// Verifies the sensor's objtype protection works.
    #[test]
    #[should_panic]
    fn test_sensor_objtype_failure() {
        let mut spec = MjSpec::new();
        spec.add_sensor()
            .with_objtype(MjtObj::mjOBJ_FRAME);
    }

    /// Verifies the sensor's reftype protection works.
    #[test]
    #[should_panic]
    fn test_sensor_reftype_failure() {
        let mut spec = MjSpec::new();
        spec.add_sensor()
            .with_reftype(MjtObj::mjOBJ_FRAME);
    }

    /// Verifies the fallible sensor objtype/reftype setters reject meta variants and accept real ones.
    #[test]
    fn test_sensor_objtype_reftype_setters() {
        let mut spec = MjSpec::new();
        let sensor = spec.add_sensor();

        assert!(matches!(
            sensor.set_objtype(MjtObj::mjOBJ_MODEL),
            Err(MjEditError::InvalidParameter(_))
        ));
        assert!(matches!(
            sensor.set_reftype(MjtObj::mjOBJ_DEFAULT),
            Err(MjEditError::InvalidParameter(_))
        ));
        assert!(sensor.set_objtype(MjtObj::mjOBJ_SITE).is_ok());
        assert!(sensor.set_reftype(MjtObj::mjOBJ_BODY).is_ok());
    }

    /// Verifies the tuple's objtype protection rejects meta object types and leaves the
    /// slice unwritten, while accepting real object types.
    #[test]
    fn test_tuple_objtype_validation() {
        let mut spec = MjSpec::new();
        let tuple = spec.add_tuple();

        assert!(matches!(
            tuple.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_FRAME]),
            Err(MjEditError::InvalidParameter(_))
        ));
        assert!(tuple.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_GEOM]).is_ok());
    }

    /// Verifies the numeric size setter rejects a negative size (which would undersize the
    /// `numeric_data` allocation in the model compiler) and accepts a non-negative one.
    #[test]
    fn test_numeric_size_validation() {
        let mut spec = MjSpec::new();
        let numeric = spec.add_numeric();

        assert!(matches!(
            numeric.set_size(-1),
            Err(MjEditError::InvalidParameter(_))
        ));
        assert!(numeric.set_size(4).is_ok());
    }
}
