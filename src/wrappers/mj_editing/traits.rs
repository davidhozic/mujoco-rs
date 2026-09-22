//! Trait definitions for model editing.
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::os::raw::c_void;
use std::process::abort;
use std::ffi::CString;
use std::any::Any;

use crate::error::MjEditError;
use crate::mujoco_c::*;

/// Prefix of every user value key that the wrapper stores, which separates the keys of this
/// crate from the keys that another language writes on the same element.
const USER_VALUE_KEY_PREFIX: &str = "mujoco-rs:";

use super::{MjSpec, MjsBody, MjsFrame, MjsSite};
use super::default::MjsDefault;
use super::utility::*;

pub(crate) mod sealed {
    /// Prevents external implementations of [`SpecItem`](super::SpecItem) and
    /// [`AttachTo`](super::AttachTo).
    pub trait Sealed {}
}

/// Every type that [`MjSpec`](super::MjSpec) supports. Sealed.
pub trait SpecItem: Sized + sealed::Sealed {
    /// Returns the `mjsElement` that MuJoCo keeps behind the item.
    ///
    /// The pointer is const. A caller that must satisfy MJS's wrong use of mutable pointers, such
    /// as [`mjs_getName`], casts it at the call site.
    fn element_pointer(&self) -> *const mjsElement;

    /// Same as [`SpecItem::element_pointer`], but with a mutable borrow and a mutable pointer.
    fn element_mut_pointer(&mut self) -> *mut mjsElement {
        self.element_pointer() as *mut _
    }

    /// Returns the item's name.
    ///
    /// # Panics
    /// Panics if the stored MuJoCo string is not valid UTF-8.
    fn name(&self) -> &str {
        // SAFETY: the string belongs to the element and lives as long as it does. mjs_getName
        // takes a mutable pointer but writes nothing.
        unsafe { read_mjs_string(mjs_getName(self.element_pointer() as *mut _)) }
    }

    /// Set a new name.
    /// # Errors
    /// Returns [`MjEditError::AlreadyExists`] when an element with the same name already exists.
    /// # Panics
    /// When the `name` contains '\0' characters mid string, a panic occurs.
    fn set_name(&mut self, name: &str) -> Result<(), MjEditError> {
        let cstr = CString::new(name).unwrap();  // panics on interior NUL bytes; &str guarantees UTF-8
        let result = unsafe { mjs_setName(self.element_mut_pointer(), cstr.as_ptr()) };
        if result != 0 {
            return Err(MjEditError::AlreadyExists);
        }
        Ok(())
    }

    /// Builder style set a new name.
    /// # Panics
    /// Panics when an element with the same name already exists, or when `name` contains '\0'.
    fn with_name(&mut self, name: &str) -> &mut Self {
        self.set_name(name).expect("mjs_setName failed: duplicate name or null byte");
        self
    }

    /// Returns the used default, or `None` when the element carries no default class name.
    ///
    /// Only a body, joint, geom, site, camera, light, actuator, pair, equality, tendon, mesh or
    /// material can carry one. Every other element, a frame included, returns `None`.
    fn default(&self) -> Option<&MjsDefault> {
        let ptr = unsafe { mjs_getDefault(self.element_pointer()) };
        // SAFETY: a non-null return points to the mjsDefault owned by a live mjCDef of the spec.
        unsafe { crate::wrappers::mj_editing::MjsDefault::from_ffi_ptr(ptr) }
    }

    /// Returns the numeric id for this element, or `None` when it has none yet (before
    /// compilation, for example).
    fn id(&self) -> Option<usize> {
        let id = unsafe { mjs_getId(self.element_pointer()) };
        usize::try_from(id).ok()
    }

    /// Assign the item to a default class.
    /// # Errors
    /// Returns [`MjEditError::NotFound`] when the default with the `class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` contains '\0' characters, a panic occurs.
    fn set_default(&mut self, class_name: &str) -> Result<(), MjEditError> {
        /* Workaround to pass the borrow checker (we use the existing borrow) */
        let cname = CString::new(class_name).unwrap();  // panics on interior NUL bytes only.
        let element = self.element_pointer();
        let spec = unsafe { mjs_getSpec(element) };
        let default = unsafe { mjs_findDefault(spec, cname.as_ptr()) };
        if default.is_null() {
            return Err(MjEditError::NotFound);
        }

        unsafe { mjs_setDefault(self.element_mut_pointer(), default); }
        Ok(())
    }

    /// Builder style make the item inherit from a default class.
    /// # Errors
    /// Same as [`SpecItem::set_default`].
    /// # Panics
    /// When the `class_name` contains '\0' characters, a panic occurs.
    fn with_default(&mut self, class_name: &str) -> Result<&mut Self, MjEditError> {
        self.set_default(class_name)?;
        Ok(self)
    }

}

/// A [`SpecItem`] that becomes a concrete object inside
/// [`crate::wrappers::mj_model::MjModel`] once [`super::MjSpec`] compiles. That is every
/// [`SpecItem`] except [`MjsDefault`] and [`MjsWrap`](super::MjsWrap). Only such an object carries
/// [`SpecObject::delete`].
pub trait SpecObject: SpecItem {
    /// The `mjtObj` discriminant passed to `mjs_firstElement` / `mjs_firstChild`.
    const OBJ_TYPE: mjtObj;

    /// Casts a raw `*mut mjsElement` to `*mut Self`.
    ///
    /// # Safety
    /// `ptr` must point to a valid element of type `Self`.
    unsafe fn from_element_as_ptr_mut(ptr: *mut mjsElement) -> *mut Self;

    /// Delete the element from the specification that holds it.
    ///
    /// Deleting a body deletes its subtree, and frees every keyframe, and every actuator, sensor,
    /// tendon, equality, pair and exclude that refers to the subtree.
    ///
    /// # Errors
    /// - [`MjEditError::UnsupportedOperation`] if the element is a frame or the world body.
    /// - [`MjEditError::DeleteFailed`] if MuJoCo refuses the deletion, which it does while another
    ///   specification holds this one.
    ///
    /// # Safety
    /// - Delete each element at most once. MuJoCo keeps the element allocated until the
    ///   specification drops, so a second deletion frees it twice.
    /// - Do not delete an element that the deletion of a body already took out of the
    ///   specification. An iterator collected before that deletion still hands out its handle.
    /// - Do not use the handle of an element that the deletion of a body freed.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut spec = MjSpec::new();
    /// spec.world_body_mut().add_body().with_name("ball");
    ///
    /// // SAFETY: the body is deleted once, and no handle of the spec outlives the call.
    /// unsafe { spec.body_mut("ball").unwrap().delete() }.unwrap();
    /// ```
    ///
    /// A default class is no `SpecObject`, so it carries no `delete`.
    /// ```compile_fail
    /// # use mujoco_rs::prelude::*;
    /// let mut spec = MjSpec::new();
    /// unsafe { spec.add_default("cls", None).delete() }.unwrap();
    /// ```
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        // SAFETY: the handle stands at a live element, which the caller keeps out of a second
        // deletion.
        unsafe { delete_element(self.element_mut_pointer()) }
    }
}

/// Represents the types of spec items that carry some user-set values (key-value map).
/// These values are only available while the [`MjSpec`], to which spec items belong,
/// is alive, and don't get carried over to the compiled [`MjModel`](crate::wrappers::mj_model::MjModel).
///
/// The wrapper prefixes every key that it stores, so a key that another language wrote on the
/// same element stays out of reach.
///
/// Note that the user storage is spec-local, even after copying.
/// Only spec attachments by reference share values.
/// 
/// # Lifetime
/// A stored value is dropped when its key is removed, when another value replaces it, or when
/// MuJoCo deletes the element that holds it. An element survives at most until the belonging
/// [`MjSpec`] is freed.
pub trait UserValued: SpecItem {
    /// Obtains a polymorphic reference to the stored data under `key` contained within this spec item.
    /// If no data is stored under `key`, [`None`] is returned.
    /// Wraps [`mjs_getUserValue`].
    /// 
    /// # Panics
    /// When `key` contains null-bytes.
    /// 
    /// # Note
    /// Plugin-stored values, under plugin-named keys, will always return [`None`], unless the plugin
    /// adds the 'mujoco-rs:' prefix, used internally in MuJoCo-rs as a prefix for keys, in front of
    /// its keys. Doing so, the entire implementation becomes undefined behavior, as non-boxed data
    /// of types that is non-[`Any`] will be cast in our implementation to `&dyn Any`.
    /// 
    /// This function remains a non-`unsafe` function as we consider custom plugins, specifically designed
    /// to crash this crate, outside our safety scope.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// # let mut spec = MjSpec::new();
    /// # let geom = spec.world_body_mut().add_geom();
    /// geom.set_user_value("serial", Box::new(String::from("user-value-1")));
    /// let value = geom.user_value("serial").unwrap();
    /// assert_eq!(value.downcast_ref::<String>().unwrap(), "user-value-1");
    ///
    /// // A downcast to any other type yields `None`.
    /// assert!(value.downcast_ref::<u32>().is_none());
    /// assert!(geom.user_value("absent").is_none());
    /// ```
    fn user_value(&self, key: &str) -> Option<&dyn Any> {
        let c_key = user_value_key(key);
        // SAFETY: the handle stands at a live element, and the key outlives the call. The C
        // parameter is mutable although the function only reads the element.
        let maybe_data = unsafe {
            mjs_getUserValue(self.element_pointer() as *mut _, c_key.as_ptr())
        };

        if maybe_data.is_null() {
            return None;
        }

        // SAFETY: the key prefix keeps foreign writers out, so only set_user_value stores here,
        // and it stores a pointer to a boxed trait object.
        Some(unsafe { &**(maybe_data as *const Box<dyn Any>) })
    }

    /// Obtains a polymorphic mutable reference to the stored data under `key` contained within
    /// this spec item. If no data is stored under `key`, [`None`] is returned.
    /// Wraps [`mjs_getUserValue`].
    ///
    /// # Panics
    /// When `key` contains null-bytes.
    ///
    /// # Note
    /// Plugin-stored values, under plugin-named keys, will always return [`None`], unless under
    /// conditions described in [`UserValued::user_value`].
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// # let mut spec = MjSpec::new();
    /// # let geom = spec.world_body_mut().add_geom();
    /// # geom.set_user_value("trace", Box::new(vec![1u32, 2]));
    /// geom.user_value_mut("trace").unwrap().downcast_mut::<Vec<u32>>().unwrap().push(3);
    /// assert_eq!(geom.user_value("trace").unwrap().downcast_ref::<Vec<u32>>().unwrap(), &[1, 2, 3]);
    /// ```
    fn user_value_mut(&mut self, key: &str) -> Option<&mut dyn Any> {
        let c_key = user_value_key(key);
        // SAFETY: the handle is a live element and the key is valid throughout the call.
        let maybe_data = unsafe {
            mjs_getUserValue(self.element_mut_pointer(), c_key.as_ptr())
        };

        if maybe_data.is_null() {
            return None;
        }

        // SAFETY: same as in user_value. mjs_getUserValue does return `*const c_void`,
        // however, the actual owned data is a mutable Box from the start, thus making
        // the cast from const to mut perfectly sound.
        Some(unsafe { &mut **maybe_data.cast_mut().cast::<Box<dyn Any>>() })
    }

    /// Sets `value` under `key` into this spec item. The value that `key` held before is dropped.
    /// Wraps [`mjs_setUserValueWithCleanup`].
    /// 
    /// # Panics
    /// When `key` contains null-bytes.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// # let mut spec = MjSpec::new();
    /// # let geom = spec.world_body_mut().add_geom();
    /// geom.set_user_value("serial", Box::new(String::from("user-value-1")));
    /// # assert_eq!(geom.user_value("serial").unwrap().downcast_ref::<String>().unwrap(), "user-value-1");
    ///
    /// // The same key takes another type, and drops the value it held.
    /// geom.set_user_value("serial", Box::new(42u32));
    /// assert_eq!(geom.user_value("serial").unwrap().downcast_ref::<u32>(), Some(&42));
    /// ```
    fn set_user_value(&mut self, key: &str, value: Box<dyn Any>) {
        let c_key = user_value_key(key);
        // The outer box keeps the stored pointer thin, because a trait object is a fat pointer.
        let data = Box::into_raw(Box::new(value)).cast();
        // SAFETY: the handle stands at a live element, and MuJoCo hands `data` back to
        // clean_box_any exactly once.
        unsafe {
            mjs_setUserValueWithCleanup(
                self.element_mut_pointer(),
                c_key.as_ptr(), data,
                Some(clean_box_any)
            );
        }
    }

    /// Drops the value that `key` holds. Does nothing when `key` holds no value.
    /// Wraps [`mjs_deleteUserValue`].
    ///
    /// # Panics
    /// When `key` contains null-bytes.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// # let mut spec = MjSpec::new();
    /// # let geom = spec.world_body_mut().add_geom();
    /// # geom.set_user_value("serial", Box::new(String::from("user-value-1")));
    /// geom.remove_user_value("serial");
    /// assert!(geom.user_value("serial").is_none());
    /// ```
    fn remove_user_value(&mut self, key: &str) {
        let c_key = user_value_key(key);
        // SAFETY: the handle stands at a live element, and the key outlives the call.
        unsafe { mjs_deleteUserValue(self.element_mut_pointer(), c_key.as_ptr()) };
    }
}

/// Returns the key under which MuJoCo stores the user value of `key`.
/// 
/// This is needed to avoid accidental clashes with plugin-set keys.
/// The only way a plugin can now clash, is for the plugin itself
/// to prepend the same [`USER_VALUE_KEY_PREFIX`] to the key.
/// 
///
/// # Panics
/// Panics when `key` contains null-bytes.
fn user_value_key(key: &str) -> CString {
    // Allocate and then push.
    // This avoids unnecessary reallocations, as, due to the CString implementation,
    // only one allocation is made (String::with_capacity).
    let mut prefixed = String::with_capacity(USER_VALUE_KEY_PREFIX.len() + key.len() + 1);
    prefixed.push_str(USER_VALUE_KEY_PREFIX);
    prefixed.push_str(key);
    CString::new(prefixed).unwrap()
}

/// Drops the box that [`UserValued::set_user_value`] leaked. MuJoCo calls it when the key takes
/// another value, when the key is removed, and when the element dies.
///
/// # Safety
/// `data` must be a pointer that [`UserValued::set_user_value`] stored, passed back once.
unsafe extern "C" fn clean_box_any(data: *const c_void) {
    // SAFETY: the caller passes back the box that set_user_value leaked, and passes it once.
    let value = unsafe { Box::from_raw(data as *mut Box<dyn Any>) };
    if catch_unwind(AssertUnwindSafe(move || drop(value))).is_err() {
        abort();
    }
}


/// A child that [`mjs_attach`] accepts for a parent of type `P`.
///
/// # Supported attachments
/// | Child | Parent `P` |
/// |---|---|
/// | [`MjsBody`] | [`MjsFrame`], [`MjsSite`] |
/// | [`MjsFrame`] | [`MjsFrame`], [`MjsSite`] |
/// | [`MjSpec`] | [`MjsBody`], [`MjsFrame`], [`MjsSite`] |
///
/// ## Attaching a frame to a body
/// MuJoCo does not copy an [`MjsFrame`] in full when it attaches directly onto an [`MjsBody`].
/// The attachment pair (parent `MjsBody`, child `MjsFrame`) is therefore not permitted,
/// thus [`AttachTo`] for that pair is not implemented.
/// Attach the frame to an [`MjsFrame`] of that body instead.
/// 
/// The following will fail to compile:
/// ```compile_fail
/// # use mujoco_rs::prelude::*;
/// let mut child = MjSpec::new();
/// let mut parent = MjSpec::new();
/// let frame = child.world_body_mut().add_frame();
/// parent.world_body_mut()
///     .attach_by_deep_copy(frame, "c_", "").unwrap();
/// ```
/// 
/// After adding a frame in between, it compiles fine:
/// ```
/// # use mujoco_rs::prelude::*;
/// let mut child = MjSpec::new();
/// let mut parent = MjSpec::new();
/// let frame = child.world_body_mut().add_frame();
/// parent.world_body_mut()
///     .add_frame()
///     .attach_by_deep_copy(frame, "c_", "").unwrap();
/// ```
pub trait AttachTo<P>: sealed::Sealed {
    /// Returns the `mjsElement` that MuJoCo attaches to the parent. The pointer is mutable,
    /// because [`mjs_attach`] renames and reparents the child that it receives.
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement;
}

// A specification is no `SpecItem`, so it carries its own seal for this trait.
impl sealed::Sealed for MjSpec {}

impl AttachTo<MjsFrame> for MjsBody {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.element_mut_pointer()
    }
}

impl AttachTo<MjsSite> for MjsBody {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.element_mut_pointer()
    }
}

impl AttachTo<MjsFrame> for MjsFrame {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.element_mut_pointer()
    }
}

impl AttachTo<MjsSite> for MjsFrame {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.element_mut_pointer()
    }
}

impl AttachTo<MjsBody> for MjSpec {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.ffi().element
    }
}

impl AttachTo<MjsFrame> for MjSpec {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.ffi().element
    }
}

impl AttachTo<MjsSite> for MjSpec {
    fn child_element_mut_pointer(&mut self) -> *mut mjsElement {
        self.ffi().element
    }
}

/// A parent that [`mjs_attach`] accepts.
/// The trait is sealed (cannot be implemented by the user).
///
/// The [`AttachTo`] trait (also sealed) is used for providing
/// supported attachment combinations.
pub trait Attach: SpecItem {
    /// Attaches a **deep-copy** of the `child` to `Self`. Wraps [`mjs_attach`].
    /// For faster attachments, call [`Attach::attach_by_reference`], which is
    /// MuJoCo's default behavior. However, the latter requires `unsafe` due to
    /// possible UBs it allows.
    ///
    /// # Note
    /// MuJoCo mutates the `child` even with deep-copying enabled.
    /// When the child is a [`MjSpec`], it will create a new [`MjsFrame`] in its world body
    /// on every attachment, under which all the sub-elements of `child` are reparented.
    ///
    /// # Errors
    /// Returns [`MjEditError::AttachFailed`] when MuJoCo rejects the attachment.
    ///
    /// # Panics
    /// Panics when `prefix` or `suffix` contain NULL bytes.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut child = MjSpec::new();
    /// child.world_body_mut().add_body().with_name("ball");
    ///
    /// let mut parent = MjSpec::new();
    /// parent.world_body_mut().attach_by_deep_copy(&mut child, "robot_", "").unwrap();
    /// assert!(parent.body("robot_ball").is_some());
    /// ```
    fn attach_by_deep_copy<C>(&mut self, child: &mut C, prefix: &str, suffix: &str)
        -> Result<(), MjEditError>
        where C: AttachTo<Self>
    {
        // SAFETY: all pointers are valid always.
        unsafe {
            attach_element(
                self.element_mut_pointer(), child.child_element_mut_pointer(), prefix, suffix, true
            )
        }
    }

    /// Attaches the `child` to `Self` by reference. Wraps [`mjs_attach`].
    /// Attachment-by-reference is the default behavior in MuJoCo (C library).
    ///
    /// # Safety
    /// This method is safe as long as the following conditions are met:
    /// - no element of the [`MjSpec`] in which the `child` lives is used anymore,
    ///   including the elements outside the attached subtree;
    /// - no further element of that [`MjSpec`] is attached anywhere;
    /// - no existing references to the child (or other tree elements of child's [`MjSpec`])
    ///   can be used further;
    /// - that child [`MjSpec`] is not compiled, because a compilation can free an element to
    ///   which the parent keeps a pointer.
    ///
    /// # Note
    /// An attachment that returns an error still marks the `child` specification as attached, thus
    /// the conditions above hold also for a failed attachment.
    ///
    /// # Errors
    /// Returns [`MjEditError::AttachFailed`] when MuJoCo rejects the attachment.
    ///
    /// # Panics
    /// Panics when `prefix` or `suffix` contain NULL bytes.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut child = MjSpec::new();
    /// child.world_body_mut().add_body().with_name("ball");
    ///
    /// let mut parent = MjSpec::new();
    /// // SAFETY: no element handle of the child is used after the attachment.
    /// unsafe { parent.world_body_mut().attach_by_reference(&mut child, "robot_", "") }.unwrap();
    /// assert!(parent.body("robot_ball").is_some());
    /// ```
    unsafe fn attach_by_reference<C>(&mut self, child: &mut C, prefix: &str, suffix: &str)
        -> Result<(), MjEditError>
        where C: AttachTo<Self>
    {
        // SAFETY: the parent element is live, AttachTo permits the pair, and the caller keeps
        // every handle of the child unused.
        unsafe {
            attach_element(
                self.element_mut_pointer(), child.child_element_mut_pointer(), prefix, suffix, false
            )
        }
    }
}

impl Attach for MjsBody {}
impl Attach for MjsFrame {}
impl Attach for MjsSite {}

/// Attaches the `child` element to the `parent` element. `deep_copy` selects whether the parent
/// deep-copies the elements of the child or "copies" by-reference.
/// Wraps [`mjs_attach`].
///
/// # Errors
/// Returns [`MjEditError::AttachFailed`] when MuJoCo rejects the attachment.
///
/// # Panics
/// Panics when `prefix` or `suffix` contain NULL bytes.
///
/// # Safety
/// Both pointers must stand at a live element of a specification.
/// With `deep_copy` false, the parent shares the elements of the child.
unsafe fn attach_element(
    parent: *mut mjsElement, child: *mut mjsElement,
    prefix: &str, suffix: &str, deep_copy: bool
) -> Result<(), MjEditError>
{
    let c_prefix = CString::new(prefix).unwrap();  // panics on interior NUL bytes only.
    let c_suffix = CString::new(suffix).unwrap();

    // SAFETY: the caller guarantees a live parent element, which belongs to a live specification.
    let spec = unsafe { mjs_getSpec(parent) };
    // MuJoCo keeps the flag on the parent, so every attachment sets the value that it needs.
    unsafe { mjs_setDeepCopy(spec, deep_copy.into()) };

    // The const on the C child parameter is misleading, because mjs_attach renames and reparents
    // the child regardless, so the pointer that reaches here is mutable.
    // SAFETY: both elements stand at a live specification and the two strings outlive the call.
    let element = unsafe { mjs_attach(parent, child, c_prefix.as_ptr(), c_suffix.as_ptr()) };
    if element.is_null() {
        // SAFETY: spec stands at the live specification of the parent.
        return Err(MjEditError::AttachFailed(unsafe { read_spec_error(spec) }));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::rc::Rc;
    use std::cell::Cell;

    use super::*;

    /// Counts its own drops.
    struct DropCounter(Rc<Cell<u32>>);

    impl Drop for DropCounter {
        fn drop(&mut self) {
            self.0.set(self.0.get() + 1);
        }
    }

    #[test]
    fn test_user_value() {
        const VALID_USER_VALUE_KEY: &str = "valid_user_value_key";
        
        enum SetOfUserValueTypes {
            Integer(u32),
        }

        let mut spec = MjSpec::new();
        let geom = spec.world_body_mut().add_geom();

        geom.set_user_value(VALID_USER_VALUE_KEY, Box::new(SetOfUserValueTypes::Integer(14)));
        let value = geom.user_value(VALID_USER_VALUE_KEY).unwrap()
            .downcast_ref::<SetOfUserValueTypes>().unwrap();

        assert!(matches!(value, SetOfUserValueTypes::Integer(14)));
        assert!(geom.user_value(VALID_USER_VALUE_KEY).unwrap().downcast_ref::<u32>().is_none());

        *geom.user_value_mut(VALID_USER_VALUE_KEY).unwrap()
            .downcast_mut::<SetOfUserValueTypes>().unwrap() = SetOfUserValueTypes::Integer(15);
        assert!(matches!(
            geom.user_value(VALID_USER_VALUE_KEY).unwrap()
                .downcast_ref::<SetOfUserValueTypes>().unwrap(),
            SetOfUserValueTypes::Integer(15)
        ));
        assert!(geom.user_value_mut("absent").is_none());

        geom.remove_user_value(VALID_USER_VALUE_KEY);
        assert!(geom.user_value(VALID_USER_VALUE_KEY).is_none());
    }

    #[test]
    fn test_user_value_cleanup() {
        let drops = Rc::new(Cell::new(0));
        {
            let mut spec = MjSpec::new();
            let geom = spec.world_body_mut().add_geom();

            geom.set_user_value("key", Box::new(DropCounter(Rc::clone(&drops))));
            assert_eq!(drops.get(), 0);

            geom.set_user_value("key", Box::new(DropCounter(Rc::clone(&drops))));
            assert_eq!(drops.get(), 1, "overwriting a key must drop the value it held");

            geom.remove_user_value("key");
            assert_eq!(drops.get(), 2, "removing a key must drop the value it held");

            geom.set_user_value("key", Box::new(DropCounter(Rc::clone(&drops))));
        }
        assert_eq!(drops.get(), 3, "dropping the spec must drop the value it holds");
    }

    #[test]
    fn test_user_value_dropped_on_element_delete() {
        let drops = Rc::new(Cell::new(0));
        {
            let mut spec = MjSpec::new();
            let geom = spec.world_body_mut().add_geom();
            geom.set_user_value("key", Box::new(DropCounter(Rc::clone(&drops))));

            // SAFETY: the borrow of the geom ends with the call, so no handle reaches the element
            // after its deletion.
            unsafe { geom.delete() }.unwrap();
            assert_eq!(drops.get(), 0, "memory is supposed to be freed after the spec is dropped");
        }
        assert_eq!(drops.get(), 1, "the spec must drop the value a deleted element held");
    }
}
