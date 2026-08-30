//! Trait definitions for model editing.
use std::ffi::{CStr, CString};

use crate::error::MjEditError;
use crate::mujoco_c::*;

use super::default::MjsDefault;
use super::utility::*;

pub(crate) mod sealed {
    /// Prevents external implementations of [`SpecItem`](super::SpecItem).
    pub trait Sealed {}
}

/// Represents all the types that [`MjSpec`](super::MjSpec) supports.
/// This is pre-implemented for all the specification types and is not
/// meant to be implemented by the user.
///
pub trait SpecItem: Sized + sealed::Sealed {
    /// Returns the internal element struct.
    /// The element struct is the C++ implementation of the
    /// actual item, which is hidden from the user, but is needed
    /// in some functions.
    /// 
    /// The returned pointer is const. Callers that must satisfy MJS's wrong use of mutable
    /// pointers, such as [`mjs_getName`], cast it at the call site.
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
        // SAFETY: mjs_getName returns a pointer to a null-terminated string owned
        // by the spec element, valid for the element's lifetime.
        // It is safe to convert to *mut _ as it doesn't actually modify anything.
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
    /// Only an element that MuJoCo adds with a default class carries one: body, joint, geom, site,
    /// camera, light, actuator, pair, equality, tendon, mesh and material. Every other element,
    /// a frame included, returns `None`.
    fn default(&self) -> Option<&MjsDefault> {
        // mjs_getDefault looks the classname up in mjCModel::def_map and returns null on a miss;
        // an element added without a default class keeps the empty classname it starts with.
        let ptr = unsafe { mjs_getDefault(self.element_pointer()) };
        // SAFETY: a non-null return points to the mjsDefault owned by a live mjCDef of the spec.
        unsafe { crate::wrappers::mj_editing::MjsDefault::from_ffi_ptr(ptr) }
    }

    /// Returns the numeric id for this element, if assigned.
    ///
    /// MuJoCo returns `-1` when no id exists (for example before compilation);
    /// in that case this returns `None`.
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

    /// Delete the item.
    ///
    /// # Deprecated
    /// This API is deprecated and will be removed in a future release.
    /// Use [`MjSpec::delete_element`](super::MjSpec::delete_element) instead.
    ///
    /// This method is inherently unsound: deleting one element mutates owner/ancestor graph
    /// structures outside the borrowed `&mut self` region, so aliasing assumptions of existing
    /// Rust references can already be violated by the call itself.
    ///
    /// In other words, calling this method is **undefined behavior** and should be avoided.
    /// Use [`MjSpec::delete_element`](super::MjSpec::delete_element) for deletion.
    ///
    /// # Errors
    /// - [`MjEditError::DeleteFailed`] if MuJoCo cannot delete the element.
    /// - [`MjEditError::UnsupportedOperation`] if the element cannot be deleted
    ///   (e.g. the world body or default classes).
    ///
    /// # Safety
    /// This legacy method is not soundly callable; it exists only for backward compatibility.
    #[deprecated(
        since = "5.0.0",
        note = "unsound legacy API; use MjSpec::delete_element(element_mut_pointer())"
    )]
    unsafe fn delete(&mut self) -> Result<(), MjEditError> {
        unsafe { self.__delete_default__() }
    }

    /// Default implementation of the delete method.
    /// Override [`SpecItem::delete`] for custom deletion logic.
    ///
    /// # Errors
    /// Returns [`MjEditError::DeleteFailed`] if MuJoCo's internal deletion fails.
    ///
    /// # Safety
    /// Same contract as [`SpecItem::delete`]: must be called at most once per item; any use
    /// of `self` after a successful call is **use-after-free** undefined behavior. The item must
    /// not be a [`MjsDefault`].
    unsafe fn __delete_default__(&mut self) -> Result<(), MjEditError> {
        // SAFETY: element_mut_pointer() is valid (struct invariant); mjs_getSpec
        // returns the owning spec, also valid.
        let element = self.element_mut_pointer();
        let spec = unsafe { mjs_getSpec(element) };
        let result = unsafe { mjs_delete(spec, element) };
        match result {
            0 => Ok(()),
            _ => {
                let error_msg: String = unsafe {
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
}

/// Represents a [`SpecItem`] that is a concrete object inside [`crate::wrappers::mj_model::MjModel`]
/// after compilation of [`super::MjSpec`]. This includes all the [`SpecItem`]-s except
/// [`MjsDefault`] and [`MjsWrap`](super::MjsWrap).
/// 
/// This trait is used internally by MuJoCo-rs to provide a generic casting interface from
/// *mut mjsElement during iteration. 
pub trait SpecObject: SpecItem {
    /// The `mjtObj` discriminant passed to `mjs_firstElement` / `mjs_firstChild`.
    const OBJ_TYPE: mjtObj;

    /// Casts a raw `*mut mjsElement` to `*mut Self`.
    ///
    /// # Safety
    /// `ptr` must point to a valid element of type `Self`.
    unsafe fn from_element_as_ptr_mut(ptr: *mut mjsElement) -> *mut Self;
}
