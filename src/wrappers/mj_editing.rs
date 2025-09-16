//! Definitions related to model editing.
use std::ffi::{c_int, CStr, CString};
use std::io::{Error, ErrorKind};
use std::marker::PhantomData;
use std::path::Path;
use std::ptr;
use super::mj_auxiliary::{MjVfs, MjVisual, MjStatistic,};
use super::mj_model::{MjModel, MjtObj, MjtGeom};
use super::mj_option::MjOption;
use crate::mujoco_c::*;

use crate::getter_setter;

// Re-export with lowercase 'f' to fix method generation
use crate::mujoco_c::{mjs_addHField as mjs_addHfield, mjsHField as mjsHfield, mjs_asHField as mjs_asHfield};


/// Represents all the types that [`MjSpec`] supports.
/// This is pre-implemented for all the specification types and is not
/// meant to be implemented by the user.
pub trait SpecItem: Sized {
    /// Returns the internal element struct.
    /// The element struct is the C++ implementation of the
    /// actual item, which is hidden to the user, but is needed
    /// in some functions.
    /// 
    /// # SAFETY
    /// This borrows immutably, but returns a mutable pointer. This is done to overcome MJS's wrong
    /// use of mutable pointers in functions, such as [`mjs_getName`].
    unsafe fn element_pointer(&self) -> *mut mjsElement;

    /// Same as [`SpecItem::element_pointer`], but with a mutable borrow.
    unsafe fn element_mut_pointer(&mut self) -> *mut mjsElement {
        unsafe { self.element_pointer() }
    }

    /// Returns the item's name.
    fn name(&self) -> &str {
        read_mjs_string( unsafe { mjs_getName(self.element_pointer()).as_ref().unwrap() } )
    }

    /// Set a new name.
    fn set_name(&mut self, name: &str) {
        let cstr = CString::new(name).unwrap();  // always has valid UTF-8
        unsafe { mjs_setName(self.element_mut_pointer(), cstr.as_ptr()) };
    }

    /// Returns the used default.
    fn default(&self) -> MjsDefault {
        MjsDefault(unsafe { mjs_getDefault(self.element_pointer()) }, PhantomData)
    }

    /// Make the item inherit properties from `default`.
    /// # Errors
    /// Returns a [`ErrorKind::NotFound`] when the default with the `class_name` doesn't exist.
    fn set_default(&mut self, class_name: &str) -> Result<(), Error> {
        /* Workaround to pass the borrow checker (we use the existing borrow) */
        let cname = CString::new(class_name).unwrap();  // class_name is always valid UTF-8.
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let default = unsafe { mjs_findDefault(spec, cname.as_ptr()) };
        if default.is_null() {
            return Err(Error::new(ErrorKind::NotFound, "class doesn't exist"));
        }

        unsafe { mjs_setDefault(self.element_mut_pointer(), default); }
        Ok(())
    }

    /// Delete the item.
    fn delete(mut self) -> Result<(), Error> {
        let element = unsafe { self.element_mut_pointer() };
        let spec = unsafe { mjs_getSpec(element) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => Err(Error::new(ErrorKind::Other, unsafe { CStr::from_ptr(mjs_getError(spec)).to_string_lossy().into_owned() }))
        }
    }
}

/***************************
** Utility functions
***************************/
/// Reads a MJS string (C++) as a `&str`.
fn read_mjs_string(string: &mjString) -> &str {
    unsafe {
        let ptr = mjs_getString(string);
        CStr::from_ptr(ptr).to_str().unwrap()  // can't be invalid UTF-8.
    }
}

/// Writes to a `destination` MJS string (C++) from a `source` `&str`.
fn write_mjs_string(source: &str, destination: &mut mjString) {
    unsafe {
        let c_source = CString::new(source).unwrap();  // can't be invalid UTF-8.
        mjs_setString(destination, c_source.as_ptr());
    }
}

/// Reads as MJS double vector (C++) as a `&\[f64\]`.
fn read_mjs_vec_f64(array: &mjDoubleVec) -> &[f64] {
    let mut userdata_length = 0;
    unsafe {
        let ptr_arr = mjs_getDouble(array, &mut userdata_length);
        if ptr_arr.is_null() {
            return &[];
        }

        std::slice::from_raw_parts(ptr_arr, userdata_length as usize)
    }
}

/// Writes as MJS double vector (C++) from a `source` to `destination`.
fn write_mjs_vec_f64(source: &[f64], destination: &mut mjDoubleVec) {
    unsafe {
        mjs_setDouble(destination, source.as_ptr(), source.len() as i32);
    }
}

/***************************
** Helper macros
***************************/
/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// Also sets the default as null();
macro_rules! add_x_method {
    ($($name:ident),*) => {paste::paste! {
        $(
            /* Without default */
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0, ptr::null()) };
                [<Mjs $name:camel>](ptr, PhantomData)
            }
        )*
    }};
}

/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// This is for methods that don't accept a default.
macro_rules! add_x_method_no_default {
    ($($name:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Add and return a child ", stringify!($name), ".")]
            pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
                let ptr = unsafe { [<mjs_add $name:camel>](self.0) };
                [<Mjs $name:camel>](ptr, PhantomData)
            }
        )*
    }};
}


/// Creates an `get_ $name` method for finding items in spec / body.
/// Also sets the default as null();
macro_rules! find_x_method {
    ($($item:ident),*) => {paste::paste! {
        $(
            #[doc = concat!("Obtain a reference to the ", stringify!($item), " with the given `name`.")]
            pub fn $item(&self, name: &str) -> Option<[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = mjs_findElement(self.0, MjtObj::[<mjOBJ_ $item:upper>], c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        Some([<Mjs $item:camel>]([<mjs_as $item:camel>](ptr), PhantomData))
                    }
                }
            }
        )*
    }};
}

/// Same as [`find_x_method`], but for types that have corresponding methods (instead of `mjs_findElement`).
macro_rules! find_x_method_direct {
    ($($item:ident),*) => {paste::paste!{
        $(
            #[doc = concat!("Obtain a reference to the ", stringify!($item), " with the given `name`.")]
            pub fn $item(&self, name: &str) -> Option<[<Mjs $item:camel>]> {
                let c_name = CString::new(name).unwrap();
                unsafe {
                    let ptr = [<mjs_find $item:camel>](self.0, c_name.as_ptr());
                    if ptr.is_null() {
                        None
                    }
                    else {
                        Some([<Mjs $item:camel>](ptr, PhantomData))
                    }
                }
            }
        )*
    }};
}


/// Creates a wrapper around a mjs$ffi_name item. It also implements the methods: `ffi()`, `ffi_mut()`
/// and traits: [`SpecItem`], [`Sync`], [`Send`].
macro_rules! mjs_wrapper {
    ($ffi_name:ident) => {paste::paste!{
        #[doc = concat!(stringify!($ffi_name), " specification. This acts as a safe reference to a FFI type [`", stringify!([<mjs $ffi_name>]), "`] internally.")]
        pub struct [<Mjs $ffi_name>]<'s>(*mut [<mjs $ffi_name>], PhantomData<&'s mut ()>);  // the lifetime belongs to the parent

        impl [<Mjs $ffi_name>]<'_> {
            /// Returns an immutable reference to the inner struct.
            pub fn ffi(&self) -> &[<mjs $ffi_name>] {
                unsafe { self.0.as_ref().unwrap() }
            }

            /// Returns a mutable reference to the inner struct.
            pub unsafe fn ffi_mut(&mut self) -> &mut [<mjs $ffi_name>] {
                unsafe { self.0.as_mut().unwrap() }
            }

            /// Returns the message appended to compiler errors.
            pub fn info(&self) {
                read_mjs_string(unsafe { self.ffi().info.as_ref().unwrap() });
            }

            /// Sets the message appended to compiler errors.
            pub fn set_info(&mut self, info: &str) {
                write_mjs_string(info, unsafe { self.ffi_mut().info.as_mut().unwrap() });
            }
        }

        impl SpecItem for [<Mjs $ffi_name>]<'_> {
            unsafe fn element_pointer(&self) -> *mut mjsElement {
                self.ffi().element
            }
        }

        // SAFETY: These are safe to implement, as access to them is available only
        // through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
        unsafe impl Sync for [<Mjs $ffi_name>]<'_> {}
        unsafe impl Send for [<Mjs $ffi_name>]<'_> {}
    }};
}


/// Implements the userdata method.
macro_rules! userdata_method {
    () => {
        /// Returns an immutable slice to userdata.
        pub fn userdata(&self) -> &[f64] {
            read_mjs_vec_f64(unsafe { (*self.0).userdata.as_ref().unwrap() })
        }
        
        /// Sets new userdata.
        pub fn set_userdata<T: AsRef<[f64]>>(&mut self, userdata: T) {
            write_mjs_vec_f64(userdata.as_ref(), unsafe {self.ffi_mut().userdata.as_mut().unwrap() })
        }
    };
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
** template specification
***************************/
mjs_wrapper!(Joint);
mjs_wrapper!(Geom);
mjs_wrapper!(Camera);
mjs_wrapper!(Light);
mjs_wrapper!(Frame);

/* Non-tree elements */
mjs_wrapper!(Actuator);
mjs_wrapper!(Sensor);
mjs_wrapper!(Flex);
mjs_wrapper!(Pair);
mjs_wrapper!(Exclude);
mjs_wrapper!(Equality);
mjs_wrapper!(Tendon);

// Wrap
mjs_wrapper!(Wrap);


mjs_wrapper!(Numeric);
mjs_wrapper!(Text);
mjs_wrapper!(Tuple);
mjs_wrapper!(Key);
mjs_wrapper!(Plugin);


/* Assets */
mjs_wrapper!(Mesh);
mjs_wrapper!(Hfield);
mjs_wrapper!(Skin);
mjs_wrapper!(Texture);
mjs_wrapper!(Material);


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

    userdata_method!();

    /// Returns an immutable slice to name of material.
    pub fn material(&self) -> &str {
        read_mjs_string(unsafe { (*self.0).material.as_ref().unwrap() })
    }

    /// Sets new name of material. Does not check if the material exists.
    pub fn set_material(&mut self, material: &str) {
        write_mjs_string(material, unsafe {self.ffi_mut().material.as_mut().unwrap() })
    }
}

/***************************
** Default specification
***************************/
// This is implemented manually since we can't directly borrow check if something is using the default.
// We also override the delete method to panic instead of deleting.

/// Default specification. This wraps the FFI type [`mjsDefault`] internally.
pub struct MjsDefault<'s>(*mut mjsDefault, PhantomData<&'s mut ()>);  // the lifetime belongs to the parent

impl MjsDefault<'_> {
    /// Returns an immutable reference to the inner struct.
    pub fn ffi(&self) -> &mjsDefault {
        unsafe { self.0.as_ref().unwrap() }
    }

    /// Returns a mutable reference to the inner struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjsDefault {
        unsafe { self.0.as_mut().unwrap() }
    }
}

impl SpecItem for MjsDefault<'_> {
    unsafe fn element_pointer(&self) -> *mut mjsElement {
        self.ffi().element
    }

    /// Defaults can't be deleted.
    fn delete(self) -> Result<(), Error> {
        unimplemented!()
    }
}

// SAFETY: These are safe to implement, as access to them is available only
// through methods or through ffi() and ffi_mut() methods, where the latter is unsafe.
unsafe impl Sync for MjsDefault<'_> {}
unsafe impl Send for MjsDefault<'_> {}



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

    /// Returns an immutable slice to userdata.
    pub fn userdata(&self) -> &[f64] {
        read_mjs_vec_f64(unsafe { (*self.0).userdata.as_ref().unwrap() })
    }
    
    /// Sets new userdata.
    pub fn set_userdata<T: AsRef<[f64]>>(&mut self, userdata: T) {
        write_mjs_vec_f64(userdata.as_ref(), unsafe {self.ffi_mut().userdata.as_mut().unwrap() })
    }
}



/******************************
** Orientation representation
******************************/
/// Alternative orientation specifiers.
pub type MjsOrientation = mjsOrientation;

/// Type of orientation specifier.
pub type MjtOrientation = mjtOrientation;


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
