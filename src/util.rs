//! Utility related data
use std::{marker::PhantomData, ops::{Deref, DerefMut}};
use std::mem;
use std::ptr;


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
#[doc(hidden)]
macro_rules! view_creator {
    /* Pointer view */
    ($self:expr, $view:ident, $data:expr, [$($field:ident),*], [$($opt_field:ident),*], $ptr_view:expr) => {
        unsafe {
            $view {
                $(
                    $field: $ptr_view($data.$field.add($self.$field.0).cast(), $self.$field.1),
                )*
                $(
                    $opt_field: if $self.$opt_field.1 > 0 {
                        Some($ptr_view($data.$opt_field.add($self.$opt_field.0).cast(), $self.$opt_field.1))
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
                        $field: $ptr_view($data.[<$prefix $field>].add($self.$field.0).cast(), $self.$field.1),
                    )*
                    $(
                        $opt_field: if $self.$opt_field.1 > 0 {
                            Some($ptr_view($data.[<$prefix $opt_field>].add($self.$opt_field.0).cast(), $self.$opt_field.1))
                        } else {None},
                    )*
                }
            }
        }
    };
}


/// Macro for reducing duplicated code when creating info structs to
/// items that have fixed size arrays in [`MjData`](crate::prelude::MjData) or [`MjModel`](crate::prelude::MjModel).
/// This creates a method `X(self, name; &str) -> XInfo`.
#[doc(hidden)]
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
                let c_name = CString::new(name).unwrap();
                let id = unsafe { mj_name2id(self.$ffi, mjtObj::[<mjOBJ_ $type_:upper>] as i32, c_name.as_ptr())};
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
#[doc(hidden)]
#[macro_export]
macro_rules! info_with_view {
    /* PointerView */

    /* name of the view/info, attribute prefix in MjData/MjModel, [attributes always present], [attributes that can be None] */
    ($info_type:ident, $name:ident, $prefix:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to [`Mj" $info_type "`] arrays corresponding to a " $name "."]
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
                #[doc = "Returns a mutable view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view_mut<'d>(&self, [<$info_type:lower>]: &'d mut [<Mj $info_type>]) -> [<Mj $name:camel $info_type ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(), $prefix, [$($attr),*], [$($opt_attr),*], crate::util::PointerViewMut::new)
                }

                #[doc = "Returns a view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view<'d>(&self, [<$info_type:lower>]: &'d [<Mj $info_type>]) -> [<Mj $name:camel $info_type View>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type View>], [<$info_type:lower>].ffi(), $prefix, [$($attr),*], [$($opt_attr),*], crate::util::PointerView::new)
                }
            }

            #[doc = "A mutable view to " $name " variables of [`Mj" $info_type "`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type ViewMut>]<'d> {
                $(
                    pub $attr: crate::util::PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<crate::util::PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel $info_type ViewMut>]<'_> {
                /// Resets the internal variables to 0.0.
                pub fn zero(&mut self) {
                    $(
                        self.$attr.fill(unsafe { std::mem::zeroed() });
                    )*
                    $(
                        if let Some(x) = &mut self.$opt_attr {
                            x.fill(unsafe { std::mem::zeroed() });
                        }
                    )*
                }
            }

            #[doc = "An immutable view to " $name " variables of [`Mj" $info_type "`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type View>]<'d> {
                $(
                    pub $attr: crate::util::PointerView<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<crate::util::PointerView<'d, $type_opt>>,
                )*
            }
        }
    };

    /* name of the view/info, [attributes always present], [attributes that can be None] */
    ($info_type:ident, $name:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to [`Mj" $info_type "`] arrays corresponding to a " $name "."]
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
                #[doc = "Returns a mutable view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view_mut<'d>(&self, [<$info_type:lower>]: &'d mut [<Mj $info_type>]) -> [<Mj $name:camel $info_type ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(), [$($attr),*], [$($opt_attr),*], crate::util::PointerViewMut::new)
                }

                #[doc = "Returns a view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view<'d>(&self, [<$info_type:lower>]: &'d [<Mj $info_type>]) -> [<Mj $name:camel $info_type View>]<'d> {
                    view_creator!(self, [<Mj $name:camel $info_type View>], [<$info_type:lower>].ffi(), [$($attr),*], [$($opt_attr),*], crate::util::PointerView::new)
                }
            }

            #[doc = "A mutable view to " $name " variables of [`Mj" $info_type "`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type ViewMut>]<'d> {
                $(
                    pub $attr: crate::util::PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<crate::util::PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel $info_type ViewMut>]<'_> {
                /// Resets the internal variables to 0.0.
                pub fn zero(&mut self) {
                    $(
                        self.$attr.fill(unsafe { std::mem::zeroed() });
                    )*
                    $(
                        if let Some(x) = &mut self.$opt_attr {
                            x.fill(unsafe { std::mem::zeroed() });
                        }
                    )*
                }
            }

            #[doc = "An immutable view to " $name " variables of [`Mj" $info_type "`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type View>]<'d> {
                $(
                    pub $attr: crate::util::PointerView<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<crate::util::PointerView<'d, $type_opt>>,
                )*
            }
        }
    };
}


/// assert_eq!, but with tolerance for floating point rounding.
#[doc(hidden)]
#[macro_export]
macro_rules! assert_relative_eq {
    ($a:expr, $b:expr, epsilon = $eps:expr) => {{
        let (a, b, eps) = ($a as f64, $b as f64, $eps as f64);
        let tol = eps * a.abs().max(b.abs()).max(1.0);
        assert!((a - b).abs() <= tol, "left={:?} right={:?} tol={:?}", a, b, tol);
    }};
}


/// Creates data on the heap, zero initialized.
pub(crate) fn box_zeroed<T>() -> Box<T> {
    let mut b: Box<mem::MaybeUninit<T>> = Box::new_uninit();

    unsafe {
        let ptr = b.as_mut_ptr() as *mut u8;
        ptr::write_bytes(ptr, 0, mem::size_of::<T>());
        b.assume_init()
    }
}
