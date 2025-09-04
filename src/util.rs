//! Utility related data
use std::{marker::PhantomData, ops::{Deref, DerefMut}};


/// Creates a (start, length) tuple based on
/// lookup variables that define the mapping in MuJoCo's mjModel struct.
/// The tuple is used to create views to correct addresses in corresponding structs.
/// Format: item id, map from item id to index inside the array of all items' values,
///         number of items, maximum number of elements inside the array of all items' values
#[macro_export]
#[doc(hidden)]
macro_rules! mj_view_indices {
    ($id:expr, $addr_map:expr, $njnt:expr, $max_n:expr) => {
        {
            let start_addr = *$addr_map.add($id) as isize;
            if start_addr == -1 {
                (0, 0)
            }
            else
            {
                let end_addr = if $id + 1 < $njnt as usize {*$addr_map.add($id as usize + 1) as usize} else {$max_n as usize};
                let n = end_addr - start_addr as usize;
                (start_addr as usize, n)
            }
        }
    };
}

/// Returns the correct address mapping based on the X in nX (nq, nv, nu, ...).
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_nx_to_mapping {
    ($model_ffi:ident, nq) => {
        $model_ffi.jnt_qposadr
    };

    ($model_ffi:ident, nv) => {
        $model_ffi.jnt_dofadr
    };

    ($model_ffi:ident, nsensordata) => {
        $model_ffi.sensor_adr
    }
}


/// Returns the correct number of items based on the X in nX (nq, nv, nu, ...).
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_nx_to_nitem {
    ($model_ffi:ident, nq) => {
        $model_ffi.njnt
    };

    ($model_ffi:ident, nv) => {
        $model_ffi.njnt
    };

    ($model_ffi:ident, nsensordata) => {
        $model_ffi.nsensor
    };
}

/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper who fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerViewMut<'d, T> {
    ptr: *mut T,
    len: usize,
    phantom: PhantomData<&'d mut ()>
}

impl<'d, T> PointerViewMut<'d, T> {
    pub(crate) fn new(ptr: *mut T, len: usize) -> Self {
        Self {ptr, len, phantom: PhantomData}
    }

    #[allow(unused)]
    pub(crate) unsafe fn set_len(&mut self, len: usize) {
        self.len = len;
    }
}

/// Compares if the two views point to the same data.
impl<T> PartialEq for PointerViewMut<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr  // if the pointer differs, this isn't a view to the same data
    }
}

impl<T> Deref for PointerViewMut<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

impl<T> DerefMut for PointerViewMut<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper who fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerView<'d, T> {
    ptr: *const T,
    len: usize,
    phantom: PhantomData<&'d ()>
}

impl<'d, T> PointerView<'d, T> {
    pub(crate) fn new(ptr: *const T, len: usize) -> Self {
        Self {ptr, len, phantom: PhantomData}
    }
    
    #[allow(unused)]
    pub(crate) unsafe fn set_len(&mut self, len: usize) {
        self.len = len;
    }
}

/// Compares if the two views point to the same data.
impl<T> PartialEq for PointerView<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr  // if the pointer differs, this isn't a view to the same data
    }
}

impl<T> Deref for PointerView<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}
