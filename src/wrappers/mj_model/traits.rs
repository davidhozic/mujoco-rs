//! Model-specific traits and implementations related to [`MjModel`] but not to it.

use std::sync::{RwLockWriteGuard, RwLockReadGuard, MutexGuard, Arc};
use std::ops::{Deref, DerefMut};
use std::cell::{RefMut, Ref};
use std::rc::Rc;

use crate::wrappers::MjModel;

/// A [`Deref`]-based marker trait used to prevent users from supplying unsafe Deref wrappers around
/// [`MjModel`].
///
/// # Safety
/// Every [`Deref::deref`] call must return the same [`MjModel`], and that model must stay valid and
/// in place for as long as the implementing value lives.
pub unsafe trait ModelType: Deref<Target = MjModel> {}

/// Mutable equivalent of [`ModelType`].
///
/// # Safety
/// [`DerefMut::deref_mut`] must return the same [`MjModel`] as [`Deref::deref`], under the contract
/// of [`ModelType`].
pub unsafe trait ModelTypeMut: ModelType + DerefMut {}

/* References and owning pointers. */
unsafe impl ModelType for &MjModel {}
unsafe impl ModelType for &mut MjModel {}
unsafe impl ModelType for Box<MjModel> {}
unsafe impl ModelType for Rc<MjModel> {}
unsafe impl ModelType for Arc<MjModel> {}

/* Interior mutability and lock guards. */
unsafe impl ModelType for Ref<'_, MjModel> {}
unsafe impl ModelType for RefMut<'_, MjModel> {}
unsafe impl ModelType for MutexGuard<'_, MjModel> {}
unsafe impl ModelType for RwLockReadGuard<'_, MjModel> {}
unsafe impl ModelType for RwLockWriteGuard<'_, MjModel> {}

/* Mutable subset of the types above. */
unsafe impl ModelTypeMut for &mut MjModel {}
unsafe impl ModelTypeMut for Box<MjModel> {}
unsafe impl ModelTypeMut for RefMut<'_, MjModel> {}
unsafe impl ModelTypeMut for MutexGuard<'_, MjModel> {}
unsafe impl ModelTypeMut for RwLockWriteGuard<'_, MjModel> {}
