//! Definitions related to model editing.
use std::ffi::{c_int, CStr, CString};
use std::io::{Error, ErrorKind};
use std::marker::PhantomData;
use std::path::Path;
use std::ptr;

use crate::wrappers::mj_auxiliary::{MjVfs, MjVisual, MjStatistic};
use crate::wrappers::mj_model::{MjModel, MjtObj};
use crate::wrappers::mj_option::MjOption;
use crate::mujoco_c::*;

// Re-export with lowercase 'f' to fix method generation
use crate::mujoco_c::{mjs_addHField as mjs_addHfield, mjsHField as mjsHfield, mjs_asHField as mjs_asHfield};


/// Represents all the types that [`MjSpec`] supports.
pub trait SpecItem: Sized {
    /// Returns the internal element struct.
    /// The element struct is the C++ implementation of the
    /// actual item, which is hidden to the user, but is needed
    /// in some functions.
    unsafe fn element_pointer(&self) -> *mut mjsElement;

    /// Returns the item's name.
    fn name(&self) -> &str {
        read_mjs_string( unsafe { mjs_getName(self.element_pointer()).as_ref().unwrap() } )
    }

    /// Set a new name.
    fn set_name(&mut self, name: &str) {
        let cstr = CString::new(name).unwrap();  // always has valid UTF-8
        unsafe { mjs_setName(self.element_pointer(), cstr.as_ptr()) };
    }

    /// Delete the item.
    fn delete(self) -> Result<(), Error> {
        let element = unsafe { self.element_pointer() };
        let spec = unsafe { dbg!(mjs_getSpec(element)) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => Err(Error::new(ErrorKind::Other, unsafe { CStr::from_ptr(mjs_getError(spec)).to_str().unwrap() }))
        }
    }
}

/***************************
** Utility functions
***************************/
/// Reads a MJS string (C++) as a &str.
fn read_mjs_string(string: &mjString) -> &str {
    unsafe {
        let ptr = mjs_getString(string);
        CStr::from_ptr(ptr).to_str().unwrap()  // can't be invalid utf-8
    }
}

/***************************
** Helper macros
***************************/
/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// Also sets the default as null();
macro_rules! add_x_method {
    ($name:ident) => {paste::paste! {
        #[doc = concat!("Add and return a child ", stringify!($name), ".")]
        pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
            let ptr = unsafe { [<mjs_add $name:camel>](self.0, ptr::null()) };
            [<Mjs $name:camel>](ptr, PhantomData)
        }
    }};
}

/// Creates an `add_ $name` method for adding new elements of $name into the body / spec.
/// This is for methods that don't accept a default.
macro_rules! add_x_method_no_default {
    ($name:ident) => {paste::paste! {
        #[doc = concat!("Add and return a child ", stringify!($name), ".")]
        pub fn [<add_ $name>](&mut self) -> [<Mjs $name:camel>] {
            let ptr = unsafe { [<mjs_add $name:camel>](self.0) };
            [<Mjs $name:camel>](ptr, PhantomData)
        }
    }};
}


/// Creates an `get_ $name` method for finding items in spec / body.
/// Also sets the default as null();
macro_rules! find_x_method {
    ($item:ident) => {paste::paste! {
        #[doc = concat!("Obtain a reference to the ", stringify!($item), " with the given `name`.")]
        pub fn $item(&mut self, name: &str) -> Option<[<Mjs $item:camel>]> {
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
    }};
}


/// Creates a wrapper around a mjs$ffi_name item. It also implements the methods: `ffi()`, `ffi_mut()`
/// and traits: [`SpecItem`], [`Sync`], [`Send`].
macro_rules! mjs_wrapper {
    ($ffi_name:ident) => {paste::paste!{
        #[doc = concat!(stringify!($ffi_name), " specification. This wraps the FFI type [`", stringify!([<mjs $ffi_name>]), "`] internally.")]
        pub struct [<Mjs $ffi_name>]<'s>(*mut [<mjs $ffi_name>], PhantomData<&'s mut ()>);  // the lifetime belongs to the parent

        impl [<Mjs $ffi_name>]<'_> {
            /// Returns an immutable reference to the inner struct.
            pub fn ffi(&self) -> &[<mjs $ffi_name>] {
                unsafe { self.0.as_ref().unwrap() }
            }

            /// Returns a mutable reference to the inner struct.
            pub unsafe fn ffi_mut(&self) -> &mut [<mjs $ffi_name>] {
                unsafe { self.0.as_mut().unwrap() }
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



/***************************
** Model Specification
***************************/
/// Model specification. This wraps the FFI type [`mjSpec`] internally.
pub struct MjSpec(*mut mjSpec);

// SAFETY: The pointer cannot be accessed without borrowing the wrapper.
unsafe impl Sync for MjSpec {}
unsafe impl Send for MjSpec {}

impl MjSpec {
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
        unsafe { MjModel::from_raw( mj_compile(self.0, ptr::null()) ) }
    }
}

/// Children accessor methods.
impl MjSpec {
    find_x_method!(body);
    find_x_method!(joint);
    find_x_method!(actuator);
    find_x_method!(sensor);
    find_x_method!(flex);
    find_x_method!(pair);
    find_x_method!(exclude);
    find_x_method!(equality);
    find_x_method!(tendon);

    // Wrap
    find_x_method!(numeric);
    find_x_method!(text);
    find_x_method!(tuple);
    find_x_method!(key);
    find_x_method!(plugin);
    // Default

    /* Assets */
    find_x_method!(mesh);
    find_x_method!(hfield);
    find_x_method!(skin);
    find_x_method!(texture);
    find_x_method!(material);

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
    add_x_method!(actuator);
    add_x_method_no_default!(sensor);
    add_x_method_no_default!(flex);
    add_x_method!(pair);
    add_x_method_no_default!(exclude);
    add_x_method!(equality);
    add_x_method!(tendon);

    // Wrap
    add_x_method_no_default!(numeric);
    add_x_method_no_default!(text);
    add_x_method_no_default!(tuple);
    add_x_method_no_default!(key);
    add_x_method_no_default!(plugin);
    // Default

    /* Assets */
    add_x_method!(mesh);
    add_x_method_no_default!(hfield);
    add_x_method_no_default!(skin);
    add_x_method_no_default!(texture);
    add_x_method!(material);
}


impl Drop for MjSpec {
    fn drop(&mut self) {
        unsafe { mj_deleteSpec(self.0); }
    }
}

/*******************************
** Other specification wrappers
*******************************/
/* Tree elements */
mjs_wrapper!(Site);
mjs_wrapper!(Body);
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
mjs_wrapper!(Numeric);
mjs_wrapper!(Text);
mjs_wrapper!(Tuple);
mjs_wrapper!(Key);
mjs_wrapper!(Plugin);
// Default

/* Assets */
mjs_wrapper!(Mesh);
mjs_wrapper!(Hfield);
mjs_wrapper!(Skin);
mjs_wrapper!(Texture);
mjs_wrapper!(Material);


/***************************
** Body specification
***************************/
impl MjsBody<'_> {
    add_x_method!(body);
    add_x_method!(site);
    add_x_method!(joint);
    add_x_method!(geom);
    add_x_method!(camera);
    add_x_method!(light);
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
}
