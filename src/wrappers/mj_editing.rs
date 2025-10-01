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
    MjtBias, MjtDyn, MjtEq, MjtTexture, MjtColorSpace,
    MjtTrn, MjtStage, MjtFlexSelf
};
use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic, MjLROpt};
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
    pub fn switch_quat<T: AsRef<[f64; 4]>>(&mut self) {
        self.type_ = MjtOrientation::mjORIENTATION_QUAT;
    }
}

/// Type of inertia inference.
pub type MjtGeomInertia = mjtGeomInertia;

/// Type of mesh inertia.
pub type MjtMeshInertia = mjtMeshInertia;

/// Type of built-in procedural texture.
pub type MjtBuiltin = mjtBuiltin;

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

/// Compiler options.
pub type MjsCompiler = mjsCompiler;
impl MjsCompiler {
    getter_setter! {with, get, set, [
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

    getter_setter! {with, get, set, [
        boundmass: f64;                "enforce minimum body mass.";
        boundinertia: f64;             "enforce minimum body diagonal inertia.";
        settotalmass: f64;             "rescale masses and inertias; <=0: ignore.";
    ]}

    getter_setter! {force!, with, get, set, [
        inertiafromgeom: MjtInertiaFromGeom;  "use geom inertias.";
    ]}

    getter_setter! {with, get, [
        inertiagrouprange: &[i32; 2];       "range of geom groups used to compute inertia.";
        eulerseq: &[i8; 3];                 "sequence for euler rotations.";
        LRopt: &MjLROpt;                    "options for lengthrange computation.";
    ]}


    /* Proxy methods to simplify macro access. */
    #[inline]
    fn ffi(&self) -> &Self {
        self
    }

    #[inline]
    unsafe fn ffi_mut(&mut self) -> &mut Self {
        self
    }
}

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
        unsafe { Self::check_spec(mj_makeSpec(), &[0]).unwrap() }
    }

    /// Creates a [`MjSpec`] from the `path` to a file.
    /// # Panics
    /// When the `path` contains '\0' characters, a panic occurs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, Error> {
        Self::from_xml_file(path, None)
    }

    /// Creates a [`MjSpec`] from the `path` to a file, located in a virtual file system (`vfs`).
    /// # Panics
    /// When the `path` contains '\0' characters, a panic occurs.
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
    /// # Panics
    /// When the `xml` contains '\0' characters, a panic occurs.
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

    /// Handles spec pointer input.
    /// # Safety
    /// `error_buffer` must not be empty and the last element must be 0.
    unsafe fn check_spec(spec_ptr: *mut mjSpec, error_buffer: &[i8]) -> Result<Self, Error> {
        if spec_ptr.is_null() {
            // SAFETY: i8 and u8 have the same size, and no negative values can appear in the error_buffer.
            Err(Error::new(
                ErrorKind::UnexpectedEof, 
                unsafe { CStr::from_ptr(error_buffer.as_ptr().cast()).to_string_lossy().into_owned() }
            ))
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
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
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
        body, geom, joint, site, camera, light, actuator, sensor, flex, pair, equality, exclude, tendon,
        numeric, text, tuple, key, mesh, hfield, skin, texture, material, plugin
    }

    find_x_method_direct! { default }

    /// Returns the world body.
    pub fn world_body(&mut self) -> MjsBody<'_> {
        self.body("world").unwrap()  // this exists always
    }
}

/// Public attributes.
impl MjSpec {
    string_set_get_with! {
        modelname; "model name.";
        comment; "comment at top of XML.";
        modelfiledir; "path to model file.";
    }

    getter_setter! {
        with, get, [
            compiler: &MjsCompiler; "compiler options.";
            stat: &MjStatistic; "statistic overrides.";
            visual: &MjVisual; "visualization options.";
            option: &MjOption; "simulation options";
        ]
    }

    getter_setter! {
        with, get, set, [strippath: bool; "whether to strip paths from mesh files."]
    }

    getter_setter! {
        get, [
            memory: MjtSize;      "number of bytes in arena+stack memory";
            nemax: i32;             "max number of equality constraints.";
            nuserdata: i32;              "number of mjtNums in userdata.";
            nuser_body: i32;            "number of mjtNums in body_user.";
            nuser_jnt: i32;              "number of mjtNums in jnt_user.";
            nuser_geom: i32;            "number of mjtNums in geom_user.";
            nuser_site: i32;            "number of mjtNums in site_user.";
            nuser_cam: i32;              "number of mjtNums in cam_user.";
            nuser_tendon: i32;        "number of mjtNums in tendon_user.";
            nuser_actuator: i32;    "number of mjtNums in actuator_user.";
            nuser_sensor: i32;        "number of mjtNums in sensor_user.";
            nkey: i32;                             "number of keyframes.";
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
    /// # Errors
    /// Errors a [`ErrorKind::AlreadyExists`] error when `class_name` already exists.
    /// Errors a [`ErrorKind::NotFound`] when `parent_class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` or `parent_class_name` contain '\0' characters, a panic occurs.
    pub fn add_default(&mut self, class_name: &str, parent_class_name: Option<&str>) -> Result<MjsDefault<'_>, Error> {
        let c_class_name = CString::new(class_name).unwrap();
        
        let parent_ptr = if let Some(name) = parent_class_name {
                self.default(name).ok_or_else(
                    || Error::new(ErrorKind::NotFound, "invalid parent name")
                )?.0
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
                Err(Error::new(ErrorKind::AlreadyExists, "duplicated name"))
            }
            else {
                Ok(MjsDefault(ptr_default, PhantomData))
            }
        }
    }
}

item_spec_iterator! {
    Body, Geom, Joint, Site, Camera, Light, Frame, Actuator, Sensor, Flex, Pair, Equality, Exclude, Tendon,
    Numeric, Text, Tuple, Key, Mesh, Hfield, Skin, Texture, Material, Plugin
}
/// Iterator methods.
impl MjSpec {
    spec_get_iter! {
        body, geom, joint, site, camera, light, frame, actuator, sensor, flex, pair, equality,
        exclude, tendon, numeric, text, tuple, key, mesh, hfield, skin, texture, material, plugin
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
        with, get, [
            // frame, size
            pos:  &[f64; 3];              "position";
            quat: &[f64; 4];              "orientation";
            alt:  &MjsOrientation;        "alternative orientation";
            fromto: &[f64; 6];            "alternative for capsule, cylinder, box, ellipsoid";
            size: &[f64; 3];              "geom size";

            // visual
            rgba: &[f32; 4];              "rgba when material is omitted";
    ]}

    getter_setter!(with, get, set, [
        type_: MjtGeom;                   "geom type";
        group: i32;                       "group";
    ]);

    userdata_method!(f64);

    string_set_get_with! {
        material; "name of material.";
    }
}

/***************************
** Joint specification
***************************/
mjs_wrapper!(Joint);
impl MjsJoint<'_> {
    getter_setter! {
        with, get, [
            pos:     &[f64; 3];         "joint position.";
            axis:    &[f64; 3];             "joint axis.";
            ref_:    &f64;             "joint reference.";
            range:   &[f64; 2];            "joint range.";
        ]
    }

    getter_setter!(with, get, set, [
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
        with, get, [
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

    getter_setter!(with, get, set, [
        type_: MjtGeom;                "geom type.";
        group: i32;                    "group.";
        contype: i32;                  "contact type.";
        conaffinity: i32;              "contact affinity.";
        condim: i32;                   "contact dimensionality.";
        priority: i32;                 "contact priority.";
        solmix: f64;                   "solver mixing for contact pairs.";
        margin: f64;                   "margin for contact detection.";
        gap: f64;                      "include in solver if dist < margin-gap.";
        mass: f64;                     "used to compute density.";
        density: f64;                  "used to compute mass and inertia from volume or surface.";
        typeinertia: MjtGeomInertia;   "selects between surface and volume inertia.";
        fluid_ellipsoid: MjtNum;       "whether ellipsoid-fluid model is active.";
        fitscale: f64;                 "scale mesh uniformly.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {
        meshname;   "mesh attached to geom.";
        material;   "name of material.";
        hfieldname; "heightfield attached to geom.";
    }

    plugin_wrapper_method!();
}

/***************************
** Camera specification
***************************/
mjs_wrapper!(Camera);
impl MjsCamera<'_> {
    getter_setter! {
        with, get, [
            pos: &[f64; 3];               "camera position.";
            quat: &[f64; 4];              "camera orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
            intrinsic: &[f32; 4];         "intrinsic parameters.";
            sensor_size: &[f32; 2];       "sensor size.";
            resolution: &[f32; 2];        "resolution.";
            focal_length: &[f32; 2];      "focal length (length).";
            focal_pixel: &[f32; 2];       "focal length (pixel).";
            principal_length: &[f32; 2];  "principal point (length).";
            principal_pixel: &[f32; 2];   "principal point (pixel).";
        ]
    }

    getter_setter!(with, get, set, [
        mode: MjtCamLight;             "camera mode.";
        fovy: f64;                    "field of view in y direction.";
        ipd: f64;                     "inter-pupillary distance for stereo.";
    ]);

    getter_setter! {
        with, get, set, [orthographic: bool; "is camera orthographic."]
    }

    userdata_method!(f64);

    string_set_get_with! {
        targetbody; "target body for tracking/targeting.";
    }
}

/***************************
** Light specification
***************************/
mjs_wrapper!(Light);
impl MjsLight<'_> {
    getter_setter! {
        with, get, [
            pos: &[f64; 3];               "light position.";
            dir: &[f64; 3];               "light direction.";
            ambient: &[f32; 3];           "ambient color.";
            diffuse: &[f32; 3];           "diffuse color.";
            specular: &[f32; 3];          "specular color.";
            attenuation: &[f32; 3];       "OpenGL attenuation (quadratic model).";
        ]
    }

    getter_setter!(with, get, set, [
        mode: MjtCamLight;             "light mode.";
        type_: MjtLightType;           "light type.";
        bulbradius: f32;               "bulb radius, for soft shadows.";
        intensity: f32;                "intensity, in candelas.";
        range: f32;                    "range of effectiveness.";
        cutoff: f32;                   "OpenGL cutoff.";
        exponent: f32;                 "OpenGL exponent.";
    ]);

    getter_setter! {
        with, get, set, [
            active: bool;       "active flag.";
            castshadow: bool;   "whether light cast shadows."
        ]
    }

    string_set_get_with! {
        texture; "texture name for image lights.";
        targetbody; "target body for targeting.";
    }
}

/***************************
** Frame specification
***************************/
mjs_wrapper!(Frame);
impl MjsFrame<'_> {
    add_x_method_by_frame! { body, site, joint, geom, camera, light }

    getter_setter! {
        with, get, [
            pos: &[f64; 3];               "frame position.";
            quat: &[f64; 4];              "frame orientation.";
            alt: &MjsOrientation;         "alternative orientation.";
        ]
    }

    string_set_get_with! {
        childclass; "childclass name.";
    }

    /// Adds a child frame.
    pub fn add_frame(&mut self) -> MjsFrame<'_> {
        unsafe {
            let parent_body = mjs_getParent(self.element_mut_pointer());
            let parent_frame = self.element_mut_pointer();
            let frame_ptr = mjs_addFrame(parent_body, parent_frame.cast());
            MjsFrame(frame_ptr, PhantomData)
        }
    }
}

/* Non-tree elements */

/***************************
** Actuator specification
***************************/
mjs_wrapper!(Actuator);
impl MjsActuator<'_> {
    getter_setter! {
        with, get, [
            gear: &[f64; 6];                            "gear parameters.";
            gainprm: &[f64; mjNGAIN as usize];          "gain parameters.";
            biasprm: &[f64; mjNBIAS as usize];          "bias parameters.";
            dynprm: &[f64; mjNDYN as usize];            "dynamic parameters.";
            lengthrange: &[f64; 2];                     "transmission length range.";
            ctrlrange: &[f64; 2];                       "control range.";
            forcerange: &[f64; 2];                      "force range.";
            actrange: &[f64; 2];                        "activation range.";
        ]
    }

    getter_setter!(with, get, set, [
        gaintype: MjtGain;             "gain type.";
        biastype: MjtBias;             "bias type.";
        dyntype: MjtDyn;               "dyn type.";
        group: i32;                    "group.";
        actdim: i32;                   "number of activation variables.";
        trntype: MjtTrn;               "transmission type.";
        cranklength: f64;              "crank length, for slider-crank.";
        inheritrange: f64;             "automatic range setting for position and intvelocity.";
    ]);

    getter_setter! {
        force!, with, get, set, [
            ctrllimited: MjtLimited;        "are control limits defined.";
            forcelimited: MjtLimited;       "are force limits defined.";
            actlimited: MjtLimited;         "are activation limits defined.";
        ]
    }

    getter_setter! {
        with, get, set, [
            actearly: bool;                "apply next activations to qfrc.";
        ]
    }

    userdata_method!(f64);

    string_set_get_with! {
        target;                 "name of transmission target";
        refsite;                "reference site, for site transmission";
        slidersite;             "site defining cylinder, for slider-crank";
    }

    plugin_wrapper_method!();
}

/***************************
** Sensor specification
***************************/
mjs_wrapper!(Sensor);
impl MjsSensor<'_> {
    getter_setter! {
        with, get, [
            intprm: &[i32; mjNSENS as usize];            "integer parameters.";
        ]
    }

    getter_setter!(with, get, set, [
        type_: MjtSensor;              "sensor type.";
        objtype: MjtObj;               "object type the sensor refers to.";
        reftype: MjtObj;               "type of referenced object";
        datatype: MjtDataType;         "data type.";
        cutoff: f64;                   "cutoff for real and positive datatypes.";
        noise: f64;                    "noise stdev.";
        needstage: MjtStage;           "compute stage needed to simulate sensor.";
        dim: i32;                      "number of scalar outputs.";
    ]);

    userdata_method!(f64);

    string_set_get_with! {
        refname; "name of referenced object.";
        objname; "name of sensorized object.";
    }

    plugin_wrapper_method!();
}

/***************************
** Flex specification
***************************/
mjs_wrapper!(Flex);
impl MjsFlex<'_> {
    getter_setter! {
        with, get, [
            rgba: &[f32; 4];                                "rgba when material is omitted.";
            friction: &[f64; 3];                            "contact friction vector.";
            solref: &[MjtNum; mjNREF as usize];             "solref for the pair.";
            solimp: &[MjtNum; mjNIMP as usize];             "solimp for the pair.";
        ]
    }

    getter_setter! {
        with, get, set, [
            young: f64;                    "elastic stiffness.";
            group: i32;                    "group.";
            contype: i32;                  "contact type.";
            conaffinity: i32;              "contact affinity.";
            condim: i32;                   "contact dimensionality.";
            priority: i32;                 "contact priority.";
            solmix: f64;                   "solver mixing for contact pairs.";
            margin: f64;                   "margin for contact detection.";
            gap: f64;                      "include in solver if dist < margin-gap.";

            dim: i32;                "element dimensionality.";
            radius: f64;             "radius around primitive element.";
            activelayers: i32;       "number of active element layers in 3D.";
            edgestiffness: f64;      "edge stiffness.";
            edgedamping: f64;        "edge damping.";
            poisson: f64;            "Poisson's ratio.";
            damping: f64;            "Rayleigh's damping.";
            thickness: f64;          "thickness (2D only).";
            elastic2d: i32;          "2D passive forces; 0: none, 1: bending, 2: stretching, 3: both.";
        ]
    }

    getter_setter! {
        with, get, set, [
            internal: bool;       "enable internal collisions.";
            flatskin: bool;       "render flex skin with flat shading.";
            vertcollide: bool;    "mode for vertex collision.";
        ]        
    }

    getter_setter! {
        force!, with, get, set, [
            selfcollide: MjtFlexSelf;        "mode for flex self collision.";
        ]
    }

    string_set_get_with! {
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
        texcoord: f32;          "vertex texture coordinates";
        elem: i32;              "element vertex ids.";
        elemtexcoord: i32;      "element texture coordinates.";
    }
}

/***************************
** Pair specification
***************************/
mjs_wrapper!(Pair);
impl MjsPair<'_> {
    getter_setter! {
        with, get, [
            friction: &[f64; 5];                            "contact friction vector.";
            solref: &[MjtNum; mjNREF as usize];             "solref for the pair.";
            solimp: &[MjtNum; mjNIMP as usize];             "solimp for the pair.";
            solreffriction: &[MjtNum; mjNREF as usize];     "solver reference, frictional directions.";
        ]
    }

    getter_setter! {
        with, get, set, [
            margin: f64;             "margin for contact detection.";
            gap: f64;        "include in solver if dist<margin-gap.";
            condim: i32;                   "contact dimensionality.";
        ]
    }

    string_set_get_with! {
        geomname1; "name of geom 1.";
        geomname2; "name of geom 2.";
    }
}

/***************************
** Exclude specification
***************************/
mjs_wrapper!(Exclude);
impl MjsExclude<'_> {
    string_set_get_with! {
        bodyname1; "name of body 1.";
        bodyname2; "name of body 2.";
    }
}

/***************************
** Equality specification
***************************/
mjs_wrapper!(Equality);
impl MjsEquality<'_> {
    getter_setter! {
        with, get, [
            data: &[f64; mjNEQDATA as usize];   "data array for equality parameters.";
            solref: &[f64; mjNREF as usize];    "solver reference";         
            solimp: &[f64; mjNIMP as usize];    "solver impedance";
        ]
    }

    getter_setter! {with, get, set, [
        active: bool;   "active flag.";
    ]}

    getter_setter! {with, get, set, [
        type_: MjtEq;   "equality type.";
        objtype: MjtObj; "type of both objects";
    ]}

    string_set_get_with! {
        name1; "name of object 1";
        name2; "name of object 2";
    }
}

/***************************
** Tendon specification
***************************/
mjs_wrapper!(Tendon);
impl MjsTendon<'_> {
    getter_setter! {
        with, get, [
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

    getter_setter! {with, get, set, [
        group: i32;         "group.";
        damping: f64;       "damping coefficient.";
        stiffness: f64;     "stiffness coefficient.";
        frictionloss: f64;  "friction loss.";
        armature: f64;      "inertia associated with tendon velocity.";
        margin: f64;        "margin value for tendon limit detection.";
        width: f64;         "width for rendering.";
    ]}

    getter_setter! {
        with, get, set, [
            limited: bool;       "does tendon have limits (mjtLimited).";
            actfrclimited: bool; "does tendon have actuator force limits."
        ]
    }

    userdata_method!(f64);
    string_set_get_with! {
        material; "name of material for rendering.";
    }

    /// Wrap a site corresponding to `name`, using the tendon.
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn wrap_site(&mut self, name: &str) -> MjsWrap<'_> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapSite(self.ffi_mut(), cname.as_ptr()) };
        MjsWrap(wrap_ptr, PhantomData)
    }

    /// Wrap a geom corresponding to `name`, using the tendon.
    /// # Panics
    /// When the `name` or `sidesite` contain '\0' characters, a panic occurs.
    pub fn wrap_geom(&mut self, name: &str, sidesite: &str) -> MjsWrap<'_> {
        let cname = CString::new(name).unwrap();
        let csidesite = CString::new(sidesite).unwrap();
        let wrap_ptr = unsafe { mjs_wrapGeom(
            self.ffi_mut(),
            cname.as_ptr(), csidesite.as_ptr()
        ) };
        MjsWrap(wrap_ptr, PhantomData)
    }

    /// Wrap a joint corresponding to `name`, using the tendon.
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn wrap_joint(&mut self, name: &str, coef: f64) -> MjsWrap<'_> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapJoint(self.ffi_mut(), cname.as_ptr(), coef) };
        MjsWrap(wrap_ptr, PhantomData)
    }

    /// Wrap a pulley using the tendon.
    pub fn wrap_pulley(&mut self, divisor: f64) -> MjsWrap<'_> {
        let wrap_ptr = unsafe { mjs_wrapPulley(self.ffi_mut(), divisor) };
        MjsWrap(wrap_ptr, PhantomData)
    }
}

/***************************
** Wrap specification
***************************/
mjs_wrapper!(Wrap);
impl MjsWrap<'_> {
    /* Auto-implemented */
}

/***************************
** Numeric specification
***************************/
mjs_wrapper!(Numeric);
impl MjsNumeric<'_> {
    getter_setter! {
        with, get, set, [
            size: i32;                     "size of the numeric array.";
        ]
    }

    vec_set_get! {
        data: f64; "initialization data.";
    }
}

/***************************
** Text specification
***************************/
mjs_wrapper!(Text);
impl MjsText<'_> {
    string_set_get_with! {
        data; "text string.";
    }
}

/***************************
** Tuple specification
***************************/
mjs_wrapper!(Tuple);
impl MjsTuple<'_> {
    vec_set! {
        objtype: i32; "object types.";
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
mjs_wrapper!(Key);
impl MjsKey<'_> {
    getter_setter! {
        with, get, set, [
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
mjs_wrapper!(Plugin);
impl MjsPlugin<'_> {
    string_set_get_with! {
        name; "instance name.";
        plugin_name; "plugin name.";
    }

    getter_setter! {
        with, get, set, [
            active: bool; "is the plugin active.";
        ]
    }
}

/* Assets */

/***************************
** Mesh specification
***************************/
mjs_wrapper!(Mesh);
impl MjsMesh<'_> {
    getter_setter! {
        with, get, [
            refpos: &[f64; 3];            "reference position.";
            refquat: &[f64; 4];           "reference orientation.";
            scale: &[f64; 3];             "scale vector.";
        ]
    }

    getter_setter! {
        with, get, set, [
            inertia: MjtMeshInertia;      "inertia type (convex, legacy, exact, shell).";
            smoothnormal: MjtByte;        "do not exclude large-angle faces from normals.";
            needsdf: MjtByte;             "compute sdf from mesh.";
            maxhullvert: i32;             "maximum vertex count for the convex hull.";
        ]
    }

    string_set_get_with! {
        content_type; "content type of file.";
        file; "mesh file.";
    }

    vec_set! {
        uservert: f32;               "user vertex data.";
        usernormal: f32;             "user normal data.";
        usertexcoord: f32;           "user texcoord data.";
        userface: i32;               "user vertex indices.";
        userfacetexcoord: i32;       "user texcoord indices.";
    }

    plugin_wrapper_method!();
}

/***************************
** Hfield specification
***************************/
mjs_wrapper!(Hfield);
impl MjsHfield<'_> {
    getter_setter! {
        with, get, [
            size: &[f64; 4];              "size of the hfield.";
        ]
    }

    getter_setter! { with, get, set, [
        nrow: i32;  "number of rows.";
        ncol: i32;  "number of columns.";
    ]}

    string_set_get_with! {
        content_type; "content type of file.";
        file; "file: (nrow, ncol, [elevation data]).";
    }

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
        with, get, [
            rgba: &[f32; 4];    "rgba when material is omitted.";
        ]
    }

    getter_setter! {
        with, get, set, [
            inflate: f32;       "inflate in normal direction.";
            group: i32;         "group for visualization.";
        ]
    }

    string_set_get_with! {
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
        face: i32;              "faces.";
    }

    vec_vec_append! {
        vertid: i32;                     "vertex ids.";
        vertweight: f32;                 "vertex weights.";
    }
}

/***************************
** Texture specification
***************************/
mjs_wrapper!(Texture);
impl MjsTexture<'_> {
    getter_setter! {
        with, get, [
            rgb1: &[f64; 3];               "first color for builtin";
            rgb2: &[f64; 3];               "second color for builtin";
            markrgb: &[f64; 3];            "mark color";
            gridsize: &[i32; 2];           "size of grid for composite file; (1,1)-repeat";
            gridlayout: &[i8; 13];         "row-major: L,R,F,B,U,D for faces; . for unused";
        ]
    }

    getter_setter! {
        with, get, set, [
            random: f64;                  "probability of random dots";
            width: i32;                   "image width.";
            height: i32;                  "image height.";
            nchannel: i32;                "number of channels.";
        ]
    }

    getter_setter! {
        force!, with, get, set, [
            type_: MjtTexture;            "texture type.";
            colorspace: MjtColorSpace;    "colorspace.";
            builtin: MjtBuiltin;          "builtin type";
            mark: MjtMark;                "mark type";
        ]
    }

    vec_string_set_append! {
        cubefiles; "different file for each side of the cube.";
    }

    getter_setter! {with, get, set, [
        hflip: bool;    "horizontal flip.";
        vflip: bool;    "vertical flip.";
    ]}

    /// Sets texture `data`.
    pub fn set_data<T>(&mut self, data: &[T]) {
        write_mjs_vec_byte(data, unsafe { self.ffi_mut().data.as_mut().unwrap() });
    }

    string_set_get_with! {
        file; "png file to load; use for all sides of cube.";
        content_type; "content type of file.";
    }
}

/***************************
** Material specification
***************************/
mjs_wrapper!(Material);
impl MjsMaterial<'_> {
    getter_setter! {
        with, get, [
            rgba: &[f32; 4];                               "rgba color.";
            texrepeat: &[f32; 2];    "texture repetition for 2D mapping";
        ]
    }

    getter_setter! {with, get, set, [
        texuniform: bool;       "make texture cube uniform";
    ]}

    getter_setter! {
        with, get, set, [
            emission: f32;                           "emission";
            specular: f32;                           "specular";
            shininess: f32;                         "shininess";
            reflectance: f32;                     "reflectance";
            metallic: f32;                           "metallic";
            roughness: f32;                         "roughness";
        ]
    }

    vec_string_set_append! {
        textures; "names of textures (empty: none).";
    }
}


/***************************
** Body specification
***************************/
mjs_wrapper!(Body);
impl MjsBody<'_> {
    add_x_method! { body, site, joint, geom, camera, light }
    
    // Special case
    /// Add and return a child frame.
    pub fn add_frame(&mut self) -> MjsFrame<'_> {
        let ptr = unsafe { mjs_addFrame(self.ffi_mut(), ptr::null_mut()) };
        MjsFrame(ptr, PhantomData)
    }

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
        with, get, [
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

    // Plain types with normal getters and setters.
    getter_setter! {
        with, get, set, [
            mass: f64;                     "mass.";
            gravcomp: f64;                 "gravity compensation.";
        ]
    }

    getter_setter! {
        with, get, set, [
            mocap: bool;                   "whether this is a mocap body.";
            explicitinertial: bool;        "whether to save the body with explicit inertial clause.";
        ]
    }
    

    // TODO: Test the plugin wrapper.
    plugin_wrapper_method!();

    userdata_method!(f64);
}

item_body_iterator! {
    Body, Joint, Geom, Site, Camera, Light, Frame
}

/// Iterator methods.
impl<'p, 's> MjsBody<'p> {
    body_get_iter! {'s, 'p, [body, joint, geom, site, camera, light, frame] }
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
        let mut world = spec.world_body();
        let mut body = world.add_body();
        assert_eq!(body.name(), "");
        body.set_name(NEW_MODEL_NAME);
        assert_eq!(body.name(), NEW_MODEL_NAME);

        spec.compile().unwrap();
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

        spec.compile().unwrap();
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
        assert!(spec.joint(NEW_NAME).is_none(), "body was not removed fom spec");

        spec.compile().unwrap();
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

        spec.compile().unwrap();
    }

    #[test]
    fn test_body_userdata() {
        const NEW_USERDATA: [f64; 3] = [1.0, 2.0, 3.0];

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");
        let mut world = spec.world_body();

        assert_eq!(world.userdata(), []);

        world.set_userdata(NEW_USERDATA);
        assert_eq!(world.userdata(), NEW_USERDATA);

        spec.compile().unwrap();
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

        spec.compile().unwrap();
    }

    #[test]
    fn test_default() {
        const DEFAULT_NAME: &str = "floor";
        const NOT_DEFAULT_NAME: &str = "floor-not";

        let mut spec = MjSpec::from_xml_string(MODEL).expect("unable to load the spec");

        /* Test search */
        spec.add_default(DEFAULT_NAME, None).unwrap();

        /* Test delete */
        assert!(spec.default(DEFAULT_NAME).is_some());
        assert!(spec.default(NOT_DEFAULT_NAME).is_none());

        let mut world = spec.world_body();
        let mut some_body = world.add_body();
        some_body.add_joint().with_name("test");
        some_body.add_geom().with_size([0.010, 0.0, 0.0]);

        let mut actuator = spec.add_actuator()
            .with_trntype(MjtTrn::mjTRN_JOINT);
        actuator.set_target("test");
        
        assert!(actuator.set_default(DEFAULT_NAME).is_ok());

        spec.compile().unwrap();
    }

    #[test]
    fn test_save() {
        /* TODO: when the bug gets fixed in MuJoCo, switch the angle="radian"to angle="degree" */
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
        let mut world = spec.world_body();
        let mut body = world.add_body();
        body.add_camera();
        body.add_geom().with_size([0.010, 0.0, 0.0]);
        body.add_light();
        body.add_site();

        spec.compile().unwrap();
        assert_eq!(spec.save_xml_string(1000).unwrap(), EXPECTED_XML);

        spec.compile().unwrap();
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
        let mut world = spec.world_body();
        world.add_site()
            .with_name(SITE_NAME);
        let mut site = spec.site(SITE_NAME).unwrap();

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
        let mut world = spec.world_body()
            .with_gravcomp(10.0);

        world.add_frame()
            .with_pos([0.5, 0.5, 0.05])
            .add_body()
            .add_geom()
            .with_size([1.0, 0.0, 0.0]);

        spec.compile().unwrap();
    }

    #[test]
    fn test_wrap() {
        let mut spec = MjSpec::new();
        let mut world = spec.world_body();
        let mut body1= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body1.add_geom().with_size([0.010;3]);
        body1.add_site().with_name("ball1");
        body1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let mut body2= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body2.add_geom().with_size([0.010;3]);
        body2.add_site().with_name("ball2");
        body2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let mut tendon = spec.add_tendon()
            .with_range([0.0, 0.25])
            .with_rgba([1.0, 0.5, 0.0, 1.0]);  // orange
        tendon.wrap_site("ball1");
        tendon.wrap_site("ball2");

        spec.world_body().add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        spec.compile().unwrap();
    }

    #[test]
    fn test_geom() {
        const GEOM_NAME: &str = "test_geom";
        const GEOM_INVALID_NAME: &str = "geom_test";
        let mut spec = MjSpec::new();
        spec.world_body().add_geom()
            .with_name(GEOM_NAME);

        assert!(spec.geom(GEOM_NAME).is_some());
        assert!(spec.geom(GEOM_INVALID_NAME).is_none());
    }

    #[test]
    fn test_camera() {
        const CAMERA_NAME: &str = "test_cam";
        const CAMERA_INVALID_NAME: &str = "cam_test";
        let mut spec = MjSpec::new();
        spec.world_body().add_camera()
            .with_name(CAMERA_NAME);

        assert!(spec.camera(CAMERA_NAME).is_some());
        assert!(spec.camera(CAMERA_INVALID_NAME).is_none());
    }

    #[test]
    fn test_light() {
        const LIGHT_NAME: &str = "test_light";
        const LIGHT_INVALID_NAME: &str = "light_test";
        let mut spec = MjSpec::new();
        spec.world_body().add_light()
            .with_name(LIGHT_NAME);

        assert!(spec.light(LIGHT_NAME).is_some());
        assert!(spec.light(LIGHT_INVALID_NAME).is_none());
    }

    #[test]
    fn test_exclude() {
        const EXCLUDE_NAME: &str = "test_exclude";
        const EXCLUDE_INVALID_NAME: &str = "exclude_test";
        let mut spec = MjSpec::new();

        spec.world_body().add_body().with_name("body1-left");
        spec.world_body().add_body().with_name("body2-right");

        spec.add_exclude()
            .with_name(EXCLUDE_NAME)
            .with_bodyname1("body1-left")
            .with_bodyname2("body2-right");

        assert!(spec.exclude(EXCLUDE_NAME).is_some());
        assert!(spec.exclude(EXCLUDE_INVALID_NAME).is_none());

        assert!(spec.compile().is_ok());
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
        let mut world = spec.world_body();
        let mut body1= world.add_body().with_pos([0.0, 0.0, 0.5]);
        body1.add_geom().with_size([0.010;3]);
        body1.add_site().with_name("ball1");
        body1.add_joint().with_type(MjtJoint::mjJNT_FREE);

        let mut body2= world.add_body().with_pos([0.0, 0.0, 0.5]).with_name(LAST_WORLD_BODY_NAME);
        body2.add_geom().with_size([0.010;3]);
        body2.add_site().with_name("ball2");
        body2.add_joint().with_type(MjtJoint::mjJNT_FREE);

        body2.add_body().with_name(LAST_BODY_NAME);

        let mut tendon = spec.add_tendon()
            .with_range([0.0, 0.25])
            .with_rgba([1.0, 0.5, 0.0, 1.0]);  // orange
        tendon.wrap_site("ball1");
        tendon.wrap_site("ball2");

        spec.world_body().add_geom().with_type(MjtGeom::mjGEOM_PLANE).with_size([1.0; 3]);

        // Iter MjSpec
        assert_eq!(spec.geom_iter().count(), N_GEOM);
        assert_eq!(spec.body_iter().count(), N_BODY);
        assert_eq!(spec.site_iter().count(), N_SITE);
        assert_eq!(spec.tendon_iter().count(), N_TENDON);
        assert_eq!(spec.mesh_iter().count(), N_MESH);
        assert_eq!(spec.body_iter().last().unwrap().name(), LAST_BODY_NAME);

        // Iter MjsBody
        let mut world = spec.world_body();
        assert_eq!(world.geom_iter(true).count(), N_GEOM);
        assert_eq!(world.body_iter(true).count(), N_BODY - 1);  // world must now be excluded
        assert_eq!(world.site_iter(true).count(), N_SITE);
        assert_eq!(world.body_iter(false).last().unwrap().name(), LAST_WORLD_BODY_NAME);
    }
}
