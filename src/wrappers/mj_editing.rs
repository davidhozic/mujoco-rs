//! Definitions related to model editing.
use std::ffi::{c_int, CStr, CString};
use std::io::{Error, ErrorKind};
use std::marker::PhantomData;
use std::path::Path;
use std::ptr;

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
    MjtBias, MjtDyn, MjtEq, MjtTexture, MjtColorSpace
};
use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic,};
use super::mj_option::MjOption;
use super::mj_primitive::*;
use crate::mujoco_c::*;

use crate::getter_setter;

// Re-export with lowercase 'f' to fix method generation
use crate::mujoco_c::{mjs_addHField as mjs_addHfield, mjsHField as mjsHfield, mjs_asHField as mjs_asHfield};


/******************************
** Type aliases
******************************/
/// Alternative orientation specifiers.
pub type MjsOrientation = mjsOrientation;

/// Type of orientation specifier.
pub type MjtOrientation = mjtOrientation;

/// Type of built-in procedural texture.
pub type MjtBuiltin = mjtBuiltin;

/// Mark type for procedural textures.
pub type MjtMark = mjtMark;


/***************************
** Model Specification
***************************/
/// Model specification. This wraps the FFI type [`mjSpec`] internally.
pub struct MjSpec(*mut mjSpec);

// SAFETY: The pointer cannot be accessed without borrowing the wrapper.
unsafe impl Sync for MjSpec {}
unsafe impl Send for MjSpec {}

impl MjSpec {
    /// Creates an empty [`MjSpec`].
    pub fn new() -> Self {
        Self::check_spec(unsafe { mj_makeSpec() }, &[]).unwrap()
    }

    /// Creates a [`MjSpec`] from the `path` to a file.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        Self::from_xml_file(path, None)
    }

    /// Creates a [`MjSpec`] from the `path` to a file, located in a virtual file system (`vfs`).
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, Error> {
        Self::from_xml_file(path, Some(vfs))
    }

    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, Error> {
        let mut error_buffer = [0i8; 100];
        unsafe {
            let path = CString::new(path.as_ref().to_str().expect("invalid utf")).unwrap();
            let raw_ptr = mj_parseXML(
                path.as_ptr(), vfs.map_or(ptr::null(), |v| v.ffi()),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );

            Self::check_spec(raw_ptr, &error_buffer)
        }
    }

    /// Creates a [`MjSpec`] from an `xml` string.
    pub fn from_xml_string(xml: &str) -> Result<Self, Error> {
        let c_xml = CString::new(xml).unwrap();
        let mut error_buffer = [0i8; 100];
        unsafe {
            let spec_ptr = mj_parseXMLString(
                c_xml.as_ptr(), ptr::null(),
                &mut error_buffer as *mut i8, error_buffer.len() as c_int
            );
            Self::check_spec(spec_ptr, &error_buffer)
        }
    }

    fn check_spec(spec_ptr: *mut mjSpec, error_buffer: &[i8]) -> Result<Self, Error> {
        if spec_ptr.is_null() {
            // SAFETY: i8 and u8 have the same size, and no negative values can appear in the error_buffer.
            let err_u8 = unsafe { std::mem::transmute(error_buffer) };
            Err(Error::new(ErrorKind::UnexpectedEof,  String::from_utf8_lossy(err_u8)))
        }
        else {
            Ok(MjSpec(spec_ptr))
        }
    }

    /// An immutable reference to the internal FFI struct.
    pub fn ffi(&self) -> &mjSpec {
        unsafe { self.0.as_ref().unwrap() }
    }

    /// A mutable reference to the internal FFI struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjSpec {
        unsafe { self.0.as_mut().unwrap() }
    }

    /// Compile [`MjSpec`] to [`MjModel`].
    /// A spec can be edited and compiled multiple times,
    /// returning a new mjModel instance that takes the edits into account.
    pub fn compile(&mut self) -> Result<MjModel, Error> {
        let result = unsafe { MjModel::from_raw( mj_compile(self.0, ptr::null()) ) };
        result.map_err(|_| {
            let error = unsafe { CStr::from_ptr(mjs_getError(self.ffi_mut())).to_string_lossy().into_owned() };
            Error::new(ErrorKind::InvalidData, error)
        })
    }

    /// Save spec to an XML file.
    pub fn save_xml(&self, filename: &str) -> Result<(), Error> {
        let mut error_buff = [0; 100];
        let cname = CString::new(filename).unwrap();  // filename is always UTF-8
        let result = unsafe { mj_saveXML(
            self.ffi(), cname.as_ptr(),
            error_buff.as_mut_ptr(), error_buff.len() as i32
        ) };
        match result {
            0 => Ok(()),
            _ => Err(
                Error::new(
                    ErrorKind::Other,
                    unsafe { CStr::from_ptr(error_buff.as_ptr().cast()).to_string_lossy().into_owned() }
                ))
        }
    }

    /// Save spec to an XML string. The `buffer_size` controls
    /// how much space is allocated for conversion.
    pub fn save_xml_string(&self, buffer_size: usize) -> Result<String, Error> {
        let mut error_buff = [0; 100];
        let mut result_buff = vec![0u8; buffer_size];
        let result = unsafe { mj_saveXMLString(
            self.ffi(), result_buff.as_mut_ptr().cast(), result_buff.len() as i32,
            error_buff.as_mut_ptr(), error_buff.len() as i32
        ) };
        match result {
            0 => Ok(CStr::from_bytes_until_nul(&result_buff).unwrap().to_string_lossy().into_owned()),
            _ => Err(
                Error::new(
                    ErrorKind::Other,
                    unsafe { CStr::from_ptr(error_buff.as_ptr().cast()).to_string_lossy().to_string() }
                ))
        }
    }
}

/// Children accessor methods.
impl MjSpec {
    find_x_method! {
        body, joint, actuator, sensor, flex, pair, exclude, equality, tendon,
        numeric, text, tuple, key, plugin, mesh, hfield, skin, texture, material
    }

    find_x_method_direct! { default }

    /// Returns the world body.
    pub fn world_body(&mut self) -> MjsBody {
        self.body("world").unwrap()  // this exists always
    }
}

/// Public attributes.
impl MjSpec {
    /// Returns an immutable reference to simulation options ([`MjOption`]).
    pub fn option(&self) -> &MjOption {
        &self.ffi().option
    }

    /// Returns a mutable reference to simulation options ([`MjOption`]).
    pub fn option_mut(&mut self) -> &mut MjOption {
        unsafe { &mut self.ffi_mut().option }
    }
    
    /// Returns an immutable reference to visualization options ([`MjVisual`]).
    pub fn visual(&self) -> &MjVisual {
        &self.ffi().visual
    }

    /// Returns a mutable reference to visualization options ([`MjVisual`]).
    pub fn visual_mut(&mut self) -> &mut MjVisual {
        unsafe { &mut self.ffi_mut().visual }
    }

    /// Returns an immutable reference to statistic overrides ([`MjStatistic`]).
    pub fn stat(&self) -> &MjStatistic {
        &self.ffi().stat
    }

    /// Returns a mutable reference to statistic overrides ([`MjStatistic`]).
    pub fn stat_mut(&mut self) -> &mut MjStatistic {
        unsafe { &mut self.ffi_mut().stat }
    }

    /// Obtains the model name.
    pub fn model_name(&self) -> &str {
        read_mjs_string(unsafe { self.ffi().modelname.as_ref().unwrap() })
    }

    /// Sets the model name.
    pub fn set_model_name(&mut self, name: &str) {
        let cstr = CString::new(name).unwrap();  // always has valid UTF-8
        unsafe { mjs_setString(self.ffi_mut().modelname, cstr.as_ptr()); };
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
    /// # Errors
    /// Returns a [`ErrorKind::AlreadyExists`] error when `class_name` already exists.
    pub fn add_default(&mut self, class_name: &str) -> Result<MjsDefault, Error> {
        let c_class_name = CString::new(class_name).unwrap ();  // can't have non-valid UTF-8
        unsafe {
            let ptr_default = mjs_addDefault(self.ffi_mut(), c_class_name.as_ptr(), ptr::null());
            if ptr_default.is_null() {
                Err(Error::new(ErrorKind::AlreadyExists, "duplicated name"))
            }
            else {
                Ok(MjsDefault(ptr_default, PhantomData))
            }
        }
    }
}

impl Drop for MjSpec {
    fn drop(&mut self) {
        unsafe { mj_deleteSpec(self.0); }
    }
}

/***************************
** Site specification
***************************/
mjs_wrapper!(Site);
impl MjsSite<'_> {
    getter_setter! {
        get, [
            // frame, size
            pos:  &[f64; 3];              "position";
            quat: &[f64; 4];              "orientation";
            alt:  &MjsOrientation;        "alternative orientation";
            fromto: &[f64; 6];            "alternative for capsule, cylinder, box, ellipsoid";
            size: &[f64; 3];              "geom size";

            // visual
            rgba: &[f32; 4];              "rgba when material is omitted";
    ]}

    getter_setter!(get, set, [
        type_: MjtGeom;                   "geom type";
        group: i32;                       "group";
    ]);

    userdata_method!(f64);

    string_set_get!(material);
}

/***************************
** Joint specification
***************************/
mjs_wrapper!(Joint);
impl MjsJoint<'_> {
    getter_setter! {
        get, [
            pos:     &[f64; 3];         "joint position.";
            axis:    &[f64; 3];             "joint axis.";
            ref_:    &f64;             "joint reference.";
            range:   &[f64; 2];            "joint range.";
        ]
    }

    getter_setter!(get, set, [
        type_: MjtJoint;               "joint type.";
        group: i32;                    "joint group.";
    ]);

    userdata_method!(f64);
}

/***************************
** Geom specification
***************************/
mjs_wrapper!(Geom);
impl MjsGeom<'_> {
    getter_setter! {
        get, [
            pos: &[f64; 3];               "geom position.";
            quat: &[f64; 4];              "geom orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
            fromto: &[f64; 6];            "alternative for capsule, cylinder, box, ellipsoid.";
            size: &[f64; 3];              "geom size.";
            rgba: &[f32; 4];              "rgba when material is omitted.";
        ]
    }

    getter_setter!(get, set, [
        type_: MjtGeom;                "geom type.";
        group: i32;                    "group.";
    ]);

    userdata_method!(f64);

    string_set_get!(meshname, material);
}

/***************************
** Camera specification
***************************/
mjs_wrapper!(Camera);
impl MjsCamera<'_> {
    getter_setter! {
        get, [
            pos: &[f64; 3];               "camera position.";
            quat: &[f64; 4];              "camera orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
            intrinsic: &[f32; 4];         "intrinsic parameters.";
            sensor_size: &[f32; 2];       "sensor size.";
            resolution: &[f32; 2];        "resolution.";
        ]
    }

    getter_setter!(get, set, [
        mode: MjtCamLight;             "camera mode.";
        fovy: f64;                    "field of view in y direction.";
        ipd: f64;                     "inter-pupillary distance for stereo.";
    ]);

    userdata_method!(f64);

    string_set_get!(targetbody);
}

/***************************
** Light specification
***************************/
mjs_wrapper!(Light);
impl MjsLight<'_> {
    getter_setter! {
        get, [
            pos: &[f64; 3];               "light position.";
            dir: &[f64; 3];               "light direction.";
            ambient: &[f32; 3];           "ambient color.";
            diffuse: &[f32; 3];           "diffuse color.";
            specular: &[f32; 3];          "specular color.";
        ]
    }

    getter_setter!(get, set, [
        mode: MjtCamLight;             "light mode.";
        type_: MjtLightType;           "light type.";
        active: u8;                   "active flag.";
    ]);

    string_set_get!(texture, targetbody);
}

/***************************
** Frame specification
***************************/
mjs_wrapper!(Frame);
impl MjsFrame<'_> {
    getter_setter! {
        get, [
            pos: &[f64; 3];               "frame position.";
            quat: &[f64; 4];              "frame orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
        ]
    }

    string_set_get!(childclass);
}

/* Non-tree elements */

/***************************
** Actuator specification
***************************/
mjs_wrapper!(Actuator);
impl MjsActuator<'_> {
    getter_setter! {
        get, [
            gear: &[f64; 6];              "gear parameters.";
            gainprm: &[f64; 10];          "gain parameters.";
            biasprm: &[f64; 10];          "bias parameters.";
            dynprm: &[f64; 10];           "dynamic parameters.";
        ]
    }

    getter_setter!(get, set, [
        gaintype: MjtGain;             "gain type.";
        biastype: MjtBias;             "bias type.";
        dyntype: MjtDyn;               "dyn type.";
        group: i32;                    "group.";
    ]);

    userdata_method!(f64);

    string_set_get!(target);
}

/***************************
** Sensor specification
***************************/
mjs_wrapper!(Sensor);
impl MjsSensor<'_> {
    getter_setter! {
        get, [
            intprm: &[i32; 3];            "integer parameters.";
        ]
    }

    getter_setter!(get, set, [
        type_: MjtSensor;              "sensor type.";
        objtype: MjtObj;               "object type the sensor refers to.";
        datatype: MjtDataType;         "data type.";
        cutoff: f64;                  "cutoff parameter.";
        noise: f64;                   "noise parameter.";
    ]);

    userdata_method!(f64);

    string_set_get!(refname, objname);
}

/***************************
** Flex specification
***************************/
mjs_wrapper!(Flex);
impl MjsFlex<'_> {
    getter_setter! {
        get, [
            rgba: &[f32; 4];              "rgba when material is omitted.";
        ]
    }

    getter_setter!(get, set, [
        group: i32;                    "group.";
        young: f64;                    "elastic stiffness.";
    ]);


    string_set_get!(material);
}

/***************************
** Pair specification
***************************/
mjs_wrapper!(Pair);
impl MjsPair<'_> {
    getter_setter! {
        get, [
            friction: &[f64; 5];          "contact friction vector.";
            solref: &[MjtNum; 2];         "solref for the pair.";
            solimp: &[MjtNum; 5];         "solimp for the pair.";
        ]
    }

    string_set_get!(geomname1, geomname2);
}

/***************************
** Exclude specification
***************************/
mjs_wrapper!(Exclude);
impl MjsExclude<'_> {
    string_set_get!(bodyname1, bodyname2);
}

/***************************
** Equality specification
***************************/
mjs_wrapper!(Equality);
impl MjsEquality<'_> {
    getter_setter! {
        get, [
            data: &[f64; 11];           "data array for equality parameters.";
        ]
    }

    getter_setter!(get, set, [
        type_: MjtEq;                                        "equality type.";
        active: u8;                                            "active flag.";
    ]);

    string_set_get!(name1, name2);
}

/***************************
** Tendon specification
***************************/
mjs_wrapper!(Tendon);
impl MjsTendon<'_> {
    getter_setter! {
        get, [
            rgba: &[f32; 4];              "rgba when material omitted.";
            range: &[f64; 2];             "range.";
            springlength: &[f64; 2];      "spring length.";
        ]
    }

    getter_setter!(get, set, [
        group: i32;                    "group.";
    ]);

    userdata_method!(f64);

    string_set_get!(material);
}

/***************************
** Wrap specification
***************************/
mjs_wrapper!(Wrap);
impl MjsWrap<'_> {
}

/***************************
** Numeric specification
***************************/
mjs_wrapper!(Numeric);
impl MjsNumeric<'_> {
    getter_setter! {
        get, set, [
            size: i32;                     "size of the numeric array.";
        ]
    }
}

/***************************
** Text specification
***************************/
mjs_wrapper!(Text);
impl MjsText<'_> {
}

/***************************
** Tuple specification
***************************/
mjs_wrapper!(Tuple);
impl MjsTuple<'_> {
}

/***************************
** Key specification
***************************/
mjs_wrapper!(Key);
impl MjsKey<'_> {
}

/***************************
** Plugin specification
***************************/
mjs_wrapper!(Plugin);
impl MjsPlugin<'_> {
}

/* Assets */

/***************************
** Mesh specification
***************************/
mjs_wrapper!(Mesh);
impl MjsMesh<'_> {
    getter_setter! {
        get, [
            refpos: &[f64; 3];            "reference position.";
            refquat: &[f64; 4];           "reference orientation.";
            scale: &[f64; 3];             "scale vector.";
        ]
    }

    string_set_get!(file, content_type);
}

/***************************
** Hfield specification
***************************/
mjs_wrapper!(Hfield);
impl MjsHfield<'_> {
    getter_setter! {
        get, [
            size: &[f64; 4];              "size of the hfield.";
        ]
    }

    string_set_get!(content_type, file);

    /// Sets `userdata`.
    pub fn set_userdata<T: AsRef<[f32]>>(&mut self, userdata: T) {
        write_mjs_vec_f32(userdata.as_ref(), unsafe {self.ffi_mut().userdata.as_mut().unwrap() })
    }
}

/***************************
** Skin specification
***************************/
mjs_wrapper!(Skin);
impl MjsSkin<'_> {
    getter_setter! {
        get, [
            rgba: &[f32; 4];    "rgba when material is omitted.";
        ]
    }

    getter_setter! {
        get, set, [
            inflate: f32;       "inflate in normal direction";
            group: i32;         "group for visualization";
        ]
    }

    string_set_get!(material, file);
}

/***************************
** Texture specification
***************************/
mjs_wrapper!(Texture);
impl MjsTexture<'_> {
    getter_setter! {
        get, [
            /* Method 1: builtin */
            rgb1: &[f64; 3];                  "first color for builtin";
            rgb2: &[f64; 3];                  "second color for builtin";
            markrgb: &[f64; 3];               "mark color";

            /* Method 2: single file */

            /* Method 3: separate files */
        ]
    }

    getter_setter! {
        get, set, [
            /* Method 1: builtin */
            type_: MjtTexture;            "texture type.";
            colorspace: MjtColorSpace;    "colorspace.";
            builtin: MjtBuiltin;          "builtin type";
            mark: MjtMark;                "mark type";
            random: f64;                  "probability of random dots";
            width: i32;                   "image width.";
            height: i32;                  "image height.";
            nchannel: i32;                "number of channels.";

            /* Method 2: single file */
            gridsize: [i32; 2];           "size of grid for composite file; (1,1)-repeat";
            gridlayout: [i8; 13];         "row-major: L,R,F,B,U,D for faces; . for unused";

            /* Method 3: separate files */
        ]
    }

    vec_string_set_append!(cubefiles);

    // // method 4: from buffer read by user
    // mjByteVec* data;                  // texture data

    // // flip options
    // mjtByte hflip;                   // horizontal flip
    // mjtByte vflip;                   // vertical flip

    string_set_get!(file);
}

/***************************
** Material specification
***************************/
mjs_wrapper!(Material);
impl MjsMaterial<'_> {
    getter_setter! {
        get, [
            rgba: &[f32; 4];                               "rgba color.";
            texrepeat: &[f32; 2];    "texture repetition for 2D mapping";
        ]
    }

    getter_setter! {get, set, [
        texuniform: bool;       "make texture cube uniform";
    ]}

    getter_setter! {
        get, set, [
            emission: f32;                           "emission";
            specular: f32;                           "specular";
            shininess: f32;                         "shininess";
            reflectance: f32;                     "reflectance";
            metallic: f32;                           "metallic";
            roughness: f32;                         "roughness";
        ]
    }

    vec_string_set_append!(textures);
}


/***************************
** Body specification
***************************/
mjs_wrapper!(Body);
impl MjsBody<'_> {
    add_x_method! { body, site, joint, geom, camera, light }
    // add_frame

    // Special case: the world body can't be deleted, however MuJoCo doesn't prevent that.
    // When the world body is deleted, the drop of MjSpec will crash on cleanup.

    /// Delete the item.
    pub fn delete(self) -> Result<(), Error> {
        if self.name() != "world" {
            SpecItem::delete(self)
        }
        else {
            Err(Error::new(ErrorKind::Unsupported, "the world body can't be deleted"))
        }
    }
}

impl MjsBody<'_> {
    // Complex types with mutable and immutable reference returns.
    getter_setter! {
        get, [
            // body frame
            pos: &[f64; 3];                   "frame position.";
            quat: &[f64; 4];                  "frame orientation.";
            alt: &MjsOrientation;             "frame alternative orientation.";

            //inertial frame
            ipos: &[f64; 3];                  "inertial frame position.";
            iquat: &[f64; 4];                 "inertial frame orientation.";
            inertia: &[f64; 3];               "diagonal inertia (in i-frame).";
            ialt: &MjsOrientation;            "inertial frame alternative orientation.";
        ]
    }

    // Plain types with normal getters and setters.
    getter_setter! {
        get, set, [
            mass: f64;                     "mass.";
            gravcomp: f64;                 "gravity compensation.";
        ]
    }

    getter_setter! {
        get, set, [
            mocap: bool;                   "whether this is a mocap body.";
            explicitinertial: bool;        "whether to save the body with explicit inertial clause.";
        ]
    }

    /// Returns a wrapper around the `plugin` attribute.
    pub fn plugin_wrapper(&mut self) -> MjsPlugin<'_> {
        unsafe { MjsPlugin(&mut self.ffi_mut().plugin, PhantomData) }
    }

    userdata_method!(f64);
}


/******************************
** Tests
******************************/
#[cfg(test)]
mod tests {
    use std::io::Write;
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
    }

    #[test]
    fn test_model_name() {
        const DEFAULT_MODEL_NAME: &str = "MuJoCo Model";
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");

        /* Test read */
        assert_eq!(spec.model_name(), DEFAULT_MODEL_NAME);
        /* Test write */
        spec.set_model_name(NEW_MODEL_NAME);
        assert_eq!(spec.model_name(), NEW_MODEL_NAME)
    }

    #[test]
    fn test_item_name() {
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();
        let mut body = world.add_body();
        assert_eq!(body.name(), "");
        body.set_name(NEW_MODEL_NAME);
        assert_eq!(body.name(), NEW_MODEL_NAME);
    }

    #[test]
    fn test_body_remove() {
        const NEW_MODEL_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();
        let mut body = world.add_body();
        body.set_name(NEW_MODEL_NAME);
        drop(body);

        /* Test normal body deletion */
        let body = spec.body(NEW_MODEL_NAME).expect("failed to obtain the body");
        assert!(body.delete().is_ok(), "failed to delete model");
        assert!(spec.body(NEW_MODEL_NAME).is_none(), "body was not removed from spec");

        /* Test world body deletion */
        let world = spec.world_body();
        assert!(world.delete().is_err(), "the world model should not be deletable");
    }

    #[test]
    fn test_joint_remove() {
        const NEW_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();
        let mut joint = world.add_joint();
        joint.set_name(NEW_NAME);
        drop(joint);

        /* Test normal body deletion */
        let joint = spec.joint(NEW_NAME).expect("failed to obtain the body");
        assert!(joint.delete().is_ok(), "failed to delete model");
        assert!(spec.joint(NEW_NAME).is_none(), "body was not removed from spec");
    }

    #[test]
    fn test_hfield_remove() {
        const NEW_NAME: &str = "Test model";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();
        let mut joint = world.add_joint();

        joint.set_name(NEW_NAME);
        drop(joint);

        /* Test normal body deletion */
        let joint = spec.joint(NEW_NAME).expect("failed to obtain the body");
        assert!(joint.delete().is_ok(), "failed to delete model");
        assert!(spec.joint(NEW_NAME).is_none(), "body was not removed from spec");
    }

    #[test]
    fn test_body_userdata() {
        const NEW_USERDATA: [f64; 3] = [1.0, 2.0, 3.0];

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();

        assert_eq!(world.userdata(), []);

        world.set_userdata(NEW_USERDATA);
        assert_eq!(world.userdata(), NEW_USERDATA);
    }

    #[test]
    fn test_body_attrs() {
        const TEST_VALUE_F64: f64 = 5.25;

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();

        world.set_gravcomp(TEST_VALUE_F64);
        assert_eq!(world.gravcomp(), TEST_VALUE_F64);

        world.pos_mut()[0] = TEST_VALUE_F64;
        assert_eq!(world.pos()[0], TEST_VALUE_F64);
    }

    #[test]
    fn test_default() {
        const DEFAULT_NAME: &str = "floor";
        const NOT_DEFAULT_NAME: &str = "floor-not";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");

        /* Test search */
        spec.add_default(DEFAULT_NAME).unwrap();

        /* Test delete */
        assert!(spec.default(DEFAULT_NAME).is_some());
        assert!(spec.default(NOT_DEFAULT_NAME).is_none());

        assert!(spec.add_actuator().set_default(DEFAULT_NAME).is_ok());
    }

    #[test]
    fn test_save() {
        const EXPECTED_XML: &str = "\
<mujoco model=\"MuJoCo Model\">
  <compiler angle=\"radian\"/>

  <worldbody>
    <body>
      <site pos=\"0 0 0\"/>
      <camera pos=\"0 0 0\"/>
      <light pos=\"0 0 0\" dir=\"0 0 -1\"/>
    </body>
  </worldbody>
</mujoco>
";

        let mut spec = MjSpec::new();
        let mut world = spec.world_body();
        let mut body = world.add_body();
        body.add_camera();
        // let geom = body.add_geom();
        body.add_light();
        body.add_site();

        spec.compile().unwrap();
        assert_eq!(spec.save_xml_string(1000).unwrap(), EXPECTED_XML);
    }

    #[test]
    fn test_site() {
        const TEST_MATERIAL: &str = "material 1";
        const TEST_POSITION: [f64; 3] = [1.0, 2.0, 3.0];

        let mut spec = MjSpec::new();
        let mut world = spec.world_body();
        let mut site = world.add_site();

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
    }
}
