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


/**************************************************************************************************/
// View creation for MjData and MjModel
/**************************************************************************************************/

/// Creates a $view struct, mapping $field and $opt_field to the same location as in $data.
#[macro_export]
macro_rules! view_creator {
    /* Pointer view */
    ($self:expr, $view:ident, $data:expr, [$($field:ident),*], [$($opt_field:ident),*], $ptr_view:expr) => {
        unsafe {
            $view {
                $(
                    $field: $ptr_view($data.$field.add($self.$field.0), $self.$field.1),
                )*
                $(
                    $opt_field: if $self.$opt_field.1 > 0 {
                        Some($ptr_view($data.$opt_field.add($self.$opt_field.0), $self.$opt_field.1))
                    } else {None},
                )*
            }
        }
    };

    ($self:expr, $view:ident, $data:expr, $prefix:ident, [$($field:ident),*], [$($opt_field:ident),*], $ptr_view:expr) => {
        paste::paste! {
            unsafe {
                $view {
                    $(
                        $field: $ptr_view($data.[<$prefix $field>].add($self.$field.0), $self.$field.1),
                    )*
                    $(
                        $opt_field: if $self.$opt_field.1 > 0 {
                            Some($ptr_view($data.[<$prefix $opt_field>].add($self.$opt_field.0), $self.$opt_field.1))
                        } else {None},
                    )*
                }
            }
        }
    };

    /* Direct reference */
    ($self:expr, $view:ident, $data:expr, [$($field:ident: &mut [$type:ty; $len:literal]),*]) => {
        unsafe {
            $view {
                $(
                    $field: ($data.$field.add($self.id * $len) as *mut [$type; $len]).as_mut().unwrap(),
                )*
            }
        }
    };

    ($self:expr, $view:ident, $data:expr, [$($field:ident: &[$type:ty; $len:literal]),*]) => {
        unsafe {
            $view {
                $(
                    $field: ($data.$field.add($self.id * $len) as *const [$type; $len]).as_ref().unwrap(),
                )*
            }
        }
    };

    /* Direct reference with prefix */
    ($self:expr, $view:ident, $data:expr, $prefix:ident, [$($field:ident: &mut [$type:ty; $len:literal]),*]) => {
        paste::paste! {
            unsafe {
                $view {
                    $(
                        $field: ($data.[<$prefix $field>].add($self.id * $len) as *mut [$type; $len]).as_mut().unwrap(),
                    )*
                }
            }
        }
    };

    ($self:expr, $view:ident, $data:expr, $prefix:ident, [$($field:ident: &[$type:ty; $len:literal]),*]) => {
        paste::paste! {
            unsafe {
                $view {
                    $(
                        $field: ($data.[<$prefix $field>].add($self.id * $len) as *const [$type; $len]).as_ref().unwrap(),
                    )*
                }
            }
        }
    };
}


/// Macro for reducing duplicated code when creating info structs to
/// items that have fixed size arrays in [`MjData`](crate::prelude::MjData).
/// This creates a method `X(self, name; &str) -> XInfo`.
#[macro_export]
macro_rules! fixed_size_info_method {
    ($info_type:ident, $ffi:expr, $type_:ident, [$($attr:ident: $len:expr),*]) => {
        paste::paste! {
            #[doc = concat!(
                "Obtains a [`", stringify!([<Mj $type_:camel $info_type Info>]), "`] struct containing information about the name, id, and ",
                "indices required for obtaining references to the correct locations in [`Mj", stringify!($info_type), "`]. ",
                "The actual view can be obtained via [`", stringify!([<Mj $type_:camel $info_type Info>]), "::view`]."
            )]
            pub fn $type_(&self, name: &str) -> Option<[<Mj $type_:camel $info_type Info>]> {
                let id = unsafe { mj_name2id(self.$ffi, mjtObj::[<mjOBJ_ $type_:upper>] as i32, CString::new(name).unwrap().as_ptr())};
                if id == -1 {  // not found
                    return None;
                }

                let id = id as usize;
                $(
                    let $attr = (id * $len, $len);
                )*

                Some([<Mj $type_:camel $info_type Info>] {name: name.to_string(), id, $($attr),*})
            }
        }
    }
}


/// Creates the xInfo struct along with corresponding xView and xViewMut structs.
#[macro_export]
macro_rules! info_with_view {
    /* PointerView */

    /* name of the view/info, attribute prefix in MjData, [attributes always present], [attributes that can be None] */
    ($info_type:ident, $name:ident, $prefix:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to [`MjData`] arrays corresponding to a " $name "."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type Info>] {
                pub name: String,
                pub id: usize,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel $info_type Info>] {
                /// Returns a mutable view to the correct fields in [`MjData`].
                pub fn view_mut<'d>(&self, data: &'d mut MjData) -> [<Mj $name:camel $info_type ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], data.ffi(), $prefix, [$($attr),*], [$($opt_attr),*], PointerViewMut::new)
                }

                /// Returns a view to the correct fields in [`MjData`].
                pub fn view<'d>(&self, data: &'d MjData) -> [<Mj $name:camel $info_type View>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type View>], data.ffi(), $prefix, [$($attr),*], [$($opt_attr),*], PointerView::new)
                }
            }

            #[doc = "A mutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type ViewMut>]<'d> {
                $(
                    pub $attr: PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel $info_type ViewMut>]<'_> {
                /// Resets the internal variables to 0.0.
                pub fn zero(&mut self) {
                    $(
                        self.$attr.fill(0.0 as $type_);
                    )*
                    $(
                        if let Some(x) = &mut self.$opt_attr {
                            x.fill(0.0 as $type_opt);
                        }
                    )*
                }
            }


            #[doc = "An immutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type View>]<'d> {
                $(
                    pub $attr: PointerView<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerView<'d, $type_opt>>,
                )*
            }
        }
    };

    /* name of the view/info, [attributes always present], [attributes that can be None] */
    ($info_type:ident, $name:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to MjData arrays corresponding to a " $name "."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type Info>] {
                pub name: String,
                pub id: usize,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel $info_type Info>] {
                /// Returns a mutable view to the correct fields in [`MjData`].
                pub fn view_mut<'d>(&self, data: &'d mut MjData) -> [<Mj $name:camel $info_type ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], data.ffi(), [$($attr),*], [$($opt_attr),*], PointerViewMut::new)
                }

                /// Returns a view to the correct fields in [`MjData`].
                pub fn view<'d>(&self, data: &'d MjData) -> [<Mj $name:camel $info_type View>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type View>], data.ffi(), [$($attr),*], [$($opt_attr),*], PointerView::new)
                }
            }


            #[doc = "A mutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type ViewMut>]<'d> {
                $(
                    pub $attr: PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel $info_type ViewMut>]<'_> {
                /// Resets the internal variables to 0.0.
                pub fn zero(&mut self) {
                    $(
                        self.$attr.fill(0.0 as $type_);
                    )*
                    $(
                        if let Some(x) = &mut self.$opt_attr {
                            x.fill(0.0 as $type_opt);
                        }
                    )*
                }
            }


            #[doc = "An immutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type View>]<'d> {
                $(
                    pub $attr: PointerView<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerView<'d, $type_opt>>,
                )*
            }
        }
    };
}
