//! Traits used to facilitate certain behaviors.

use std::{ops::Deref, rc::Rc};

/// Extends Deref so that a type can either clone itself
/// or return a reference to itself.
/// 
/// This facilitates generic code that can either have `&T` or [`Rc<T>`]
pub trait RefOrClone: Deref {
    fn clone_or_ref(&self) -> Self;
}

impl<T> RefOrClone for Rc<T> {
    fn clone_or_ref(&self) -> Self {
        self.clone()
    }
}

impl<'a, T> RefOrClone for &'a T {
    fn clone_or_ref(&self) -> Self {
        *self
    }
}
