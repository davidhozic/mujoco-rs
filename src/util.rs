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
    };
    ($model_ffi:ident, ntupledata) => {
        $model_ffi.tuple_adr
    };
    ($model_ffi:ident, ntexdata) => {
        $model_ffi.tex_adr
    };
    ($model_ffi:ident, nnumericdata) => {
        $model_ffi.numeric_adr
    };
    ($model_ffi:ident, nhfielddata) => {
        $model_ffi.hfield_adr
    };
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
    ($model_ffi:ident, ntupledata) => {
        $model_ffi.ntuple
    };
    ($model_ffi:ident, ntexdata) => {
        $model_ffi.ntex
    };
    ($model_ffi:ident, nnumericdata) => {
        $model_ffi.nnumeric
    };
    ($model_ffi:ident, nhfielddata) => {
        $model_ffi.nhfield
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
/// items in [`MjData`](crate::prelude::MjData) or [`MjModel`](crate::prelude::MjModel).
/// This creates a method `X(self, name; &str) -> XInfo`.
/// Compatible entries: (..., [name: fixed number], [name: ffi().attribute (* repeats)], [length of the item's data array (e.g., hfield -> nhfielddata, texture -> ntexdata)])
#[doc(hidden)]
#[macro_export]
macro_rules! info_method {
    ($info_type:ident, $ffi:expr, $type_:ident, [$($attr:ident: $len:expr),*], [$($attr_ffi:ident: $len_ffi:ident $(* $multiplier:expr)?),*], [$($attr_dyn:ident: $ffi_len_dyn:expr),*]) => {
        paste::paste! {
            #[doc = concat!(
                "Obtains a [`", stringify!([<Mj $type_:camel $info_type Info>]), "`] struct containing information about the name, id, and ",
                "indices required for obtaining references to the correct locations in [`Mj", stringify!($info_type), "`]. ",
                "The actual view can be obtained via [`", stringify!([<Mj $type_:camel $info_type Info>]), "::view`].\n",
                "# Panics\n",
                "A panic will occur if `name` contains `\\0` characters."
            )]
            pub fn $type_(&self, name: &str) -> Option<[<Mj $type_:camel $info_type Info>]> {
                let c_name = CString::new(name).unwrap();
                let ffi = self.$ffi;
                let id = unsafe { mj_name2id(ffi, MjtObj::[<mjOBJ_ $type_:upper>] as i32, c_name.as_ptr())};
                if id == -1 {  // not found
                    return None;
                }

                let id = id as usize;
                $(
                    let $attr = (id * $len, $len);
                )*

                $(
                    let $attr_ffi = (id * ffi.$len_ffi as usize $( * $multiplier)*, ffi.$len_ffi as usize $( * $multiplier)*);
                )*

                $(
                    let $attr_dyn = unsafe { mj_view_indices!(
                        id,
                        mj_model_nx_to_mapping!(ffi, $ffi_len_dyn),
                        mj_model_nx_to_nitem!(ffi, $ffi_len_dyn),
                        ffi.$ffi_len_dyn
                    ) };
                )*

                Some([<Mj $type_:camel $info_type Info>] {name: name.to_string(), id, $($attr,)* $($attr_ffi,)* $($attr_dyn),*})
            }
        }
    }
}


/// This creates a method `X(self, name: &str) -> XInfo`.
/// 
/// # Compatible Entry Types
/// The macro supports the following types of entries for struct fields:
/// 
/// - **Fixed number**:  
///   `[field_name: fixed_length]`.  
///   Example: `[id: 1]` (for a field with a fixed length of 1)
/// 
/// - **FFI attribute (possibly with multiplier)**:  
///   `[field_name: ffi_attribute (* multiplier)]`.  
///   Example: `[matid: nmatid * 2]` (where `nmatid` is an attribute from the FFI struct, and the field length is `nmatid * 2`)
/// 
/// - **Dynamic length (from item's data array)**:  
///   `[field_name: data_array_length]`.  
///   Example: `[hfielddata: nhfielddata]` (where `nhfielddata` is the length of the hfield data array).
///   Note: nhfielddata is the major index (i.e., [nhfielddata x something] in the C array.)
///
#[doc(hidden)]
#[macro_export]
macro_rules! info_with_view {
    /* PointerView */

    /* name of the view/info, attribute prefix in MjData/MjModel, [attributes always present], [attributes that can be None] */
    ($info_type:ident, $name:ident, $prefix:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]$(,$generics:ty: $bound:ty)?) => {
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
                pub fn view_mut $(<$generics: $bound>)?(&self, [<$info_type:lower>]: &mut [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type ViewMut>]<'_> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(), $prefix, [$($attr),*], [$($opt_attr),*], crate::util::PointerViewMut::new)
                }

                #[doc = "Returns a view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view $(<$generics: $bound>)?(&self, [<$info_type:lower>]: &[<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type View>]<'_> {
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
    ($info_type:ident, $name:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]$(,$generics:ty: $bound:ty)?) => {
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
                pub fn view_mut $(<$generics: $bound>)?(&self, [<$info_type:lower>]: &mut [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type ViewMut>]<'_> {
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(), [$($attr),*], [$($opt_attr),*], crate::util::PointerViewMut::new)
                }

                #[doc = "Returns a view to the correct fields in [`Mj" $info_type "`]"]
                pub fn view $(<$generics: $bound>)?(&self, [<$info_type:lower>]: &[<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type View>]<'_> {
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


#[doc(hidden)]
#[macro_export]
macro_rules! getter_setter {
    (get, [$($name:ident: bool; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Check ", $comment)]
            pub fn [<$name:lower>](&self) -> bool {
                self.ffi().$name == 1
            }
        )*
    }};

    (get, [$($name:ident: & $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return an immutable reference to ", $comment)]
            pub fn [<$name:lower>](&self) -> &$type {
                &self.ffi().$name
            }

            #[doc = concat!("Return a mutable reference to ", $comment)]
            pub fn [<$name:camel:lower _mut>](&mut self) -> &mut $type {
                unsafe { &mut self.ffi_mut().$name }
            }
        )*
    }};

    (get, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:lower>](&self) -> $type {
                self.ffi().$name.into()
            }
        )*
    }};

    (set, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:lower>](&mut self, value: $type) {
                    unsafe { self.ffi_mut().$name = value.into() };
                }
            )*
        }
    };

    /* Enum conversion */
    (force!, get, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:lower>](&self) -> $type {
                unsafe { std::mem::transmute(self.ffi().$name) }
            }
        )*
    }};

    (force!, set, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:lower>](&mut self, value: $type) {
                    #[allow(unnecessary_transmutes)]
                    unsafe { self.ffi_mut().$name = std::mem::transmute(value) };
                }
            )*
        }
    };

    /* Builder pattern */
    (force!, with, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:lower>](mut self, value: $type) -> Self {
                    #[allow(unnecessary_transmutes)]
                    unsafe { self.ffi_mut().$name = std::mem::transmute(value) };
                    self
                }
            )*
        }
    };

    (with, [$($name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:lower>](mut self, value: $type) -> Self {
                    unsafe { self.ffi_mut().$name = value.into() };
                    self
                }
            )*
        }
    };
    
    /* Handling of optional arguments */
    /* Enum pass */
    (force!, get, set, [ $( $name:ident : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [ $( $name : $type ; $comment );* ]);
    };

    (get, set, [ $( $name:ident : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $( $name : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $( $name : bool ; $comment );* ]);
    };

    (get, set, [ $( $name:ident : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $( $name : $type ; $comment );* ]);
    };

    /* Builder pattern */
    (with, get, set, [ $( $name:ident : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $( $name : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $( $name : bool ; $comment );* ]);
        $crate::getter_setter!(with, [ $( $name : bool ; $comment );* ]);
    };

    (with, get, set, [ $( $name:ident : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(with, [ $( $name : $type ; $comment );* ]);
    };

    (force!, with, get, set, [ $( $name:ident : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [ $( $name : $type ; $comment );* ]);
        $crate::getter_setter!(force!, with, [ $( $name : $type ; $comment );* ]);
    };

    (with, get, [ $( $name:ident : & $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $( $name : & $type ; $comment );* ]);
        $crate::getter_setter!(with, [ $( $name : $type ; $comment );* ]);
    };
}


#[doc(hidden)]
#[macro_export]
/// Constructs builder methods.
macro_rules! builder_setters {
    ($($name:ident: $type:ty; $comment:expr);* $(;)?) => {
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn $name(mut self, value: $type) -> Self {
                self.$name = value;
                self
            }
        )*
    };
}

/// A macro for creating a slice over a raw array of dynamic size (given by some other variable in $len_accessor).
/// Syntax: attribute: <optional pre-transformations (e.g., `as_ptr as_mut_ptr`)> &[datatype; documentation string;
/// code to access the length attribute, appearing after `self.`]
/// Syntax for arrays whose size is a sum from some length array:
///     summed {
///         ...
///         attribute: &[datatype; documentation; [
///                 size multiplier;
///                 (code to access the length array, appearing after self);
///                 (code to access the length array's length, appearing after self)
///             ]
///         ],
///         ...
///     }
///
#[doc(hidden)]
#[macro_export]
macro_rules! array_slice_dyn {
    // Arrays that are of scalar variable size
    ($($name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[$type:ty $([$cast:ident])?; $doc:literal; $($len_accessor:tt)*]),*) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize;
                    if length == 0 {
                        return &[];
                    }
                    unsafe { std::slice::from_raw_parts(self.ffi().$name$(.$as_ptr())?$(.$cast())? as *const _, length) }
               }

                #[doc = concat!("Mutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                    let length = self.$($len_accessor)* as usize;
                    if length == 0 {
                        return &mut [];
                    }
                    unsafe { std::slice::from_raw_parts_mut(self.ffi_mut().$name$(.$as_mut_ptr())?$(.$cast())?, length) }
                }
            )*
        }
    };

    // Arrays that are of summed variable size
    (summed { $($name:ident: &[$type:ty; $doc:literal; [$multiplier:literal ; ($($len_array:tt)*) ; ($($len_array_length:tt)*)]]),* }) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[[$type; $multiplier]] {
                    // Obtain a slice to the length array.
                    let length_array_length = self.$($len_array_length)* as usize;
                    if length_array_length == 0 {
                        return &[];
                    }

                    let length = unsafe { std::slice::from_raw_parts(
                        self.$($len_array)*.cast(),
                        length_array_length
                    ).into_iter().sum::<u32>() as usize };

                    if length == 0 {
                        return &[];
                    }

                    unsafe { std::slice::from_raw_parts(self.ffi().$name.cast(), length) }
               }

                #[doc = concat!("Mutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [[$type; $multiplier]] {
                    let length_array_length = self.$($len_array_length)* as usize;
                    if length_array_length == 0 {
                        return &mut [];
                    }

                    let length = unsafe { std::slice::from_raw_parts(
                        self.$($len_array)*.cast(),
                        length_array_length
                    ).into_iter().sum::<u32>() as usize };

                    if length == 0 {
                        return &mut [];
                    }

                    unsafe { std::slice::from_raw_parts_mut(self.ffi_mut().$name.cast(), length) }
                }
            )*
        }
    };

    // Arrays whose second dimension is dependent on some variable
    (sublen_dep {$($name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[[$type:ty; $($inner_len_accessor:tt)*] $([$cast:ident])?; $doc:literal; $($len_accessor:tt)*]),*}) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize * self.$($inner_len_accessor)* as usize;
                    if length == 0 {
                        return &[];
                    }


                    unsafe { std::slice::from_raw_parts(self.ffi().$name$(.$as_ptr())?$(.$cast())? as *const _, length) }
               }

                #[doc = concat!("Mutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                    let length = self.$($len_accessor)* as usize * self.$($inner_len_accessor)* as usize;
                    if length == 0 {
                        return &mut [];
                    }
                    unsafe { std::slice::from_raw_parts_mut(self.ffi_mut().$name$(.$as_mut_ptr())?$(.$cast())?, length) }
                }
            )*
        }
    };
}

/// assert_eq!, but with tolerance for floating point rounding.
#[doc(hidden)]
#[macro_export]
macro_rules! assert_relative_eq {
    ($a:expr, $b:expr, epsilon = $eps:expr) => {{
        let (a, b, eps) = ($a as f64, $b as f64, $eps as f64);
        assert!((a - b).abs() <= eps, "left={:?} right={:?} eps={:?}", a, b, eps);
    }};
}
