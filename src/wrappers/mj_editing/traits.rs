//! Trait definitions for model editing.
use std::ffi::CString;

use crate::error::MjEditError;
use crate::mujoco_c::*;

use super::default::MjsDefault;
use super::utility::*;

pub(crate) mod sealed {
    /// Prevents external implementations of [`SpecItem`](super::SpecItem).
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
