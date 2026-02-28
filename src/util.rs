//! Utility related data
use std::{marker::PhantomData, ops::{Deref, DerefMut}};
use std::sync::{Mutex, MutexGuard};

use crate::mujoco_c::{mj_version, mjVERSION_HEADER};


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
    ($model_ffi:ident, na) => {
        $model_ffi.actuator_actadr
    };
    ($model_ffi:ident, nJten) => {
        $model_ffi.ten_J_rowadr
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
    ($model_ffi:ident, na) => {
        $model_ffi.nu
    };
    ($model_ffi:ident, nJten) => {
        $model_ffi.ntendon
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

/***************************/
//  Evaluation helper macro
/***************************/
/// When @eval is given false, ignore the given contents.
/// In other cases, expand the given contents.
#[macro_export]
#[doc(hidden)]
macro_rules! eval_or_expand {
    (@eval $(true)? { $($data:tt)* } ) => { $($data)* };
    (@eval false { $($data:tt)* } ) => {};
}


/**************************************************************************************************/
// View creation for MjData and MjModel
/**************************************************************************************************/

/// Creates a $view struct, mapping $field and $opt_field to the same location as in $data.
#[macro_export]
#[doc(hidden)]
macro_rules! view_creator {
    ($self:expr, $view:ident, $data:expr, [$($([$prefix_field:ident])? $field:ident $([$cast:ident])? ),*], [$($([$prefix_opt_field:ident])? $opt_field:ident $([$cast_opt:ident])?),*], $ptr_view:expr) => {
        paste::paste! {
            unsafe {
                $view {
                    $(
                        $field: $ptr_view($data.[<$($prefix_field)? $field>].add($self.$field.0) $(.$cast())?, $self.$field.1),
                    )*
                    $(
                        $opt_field: if $self.$opt_field.1 > 0 {
                            Some($ptr_view($data.[<$($prefix_opt_field)? $opt_field>].add($self.$opt_field.0) $(.$cast_opt())?, $self.$opt_field.1))
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
            #[allow(non_snake_case)]
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

                let model_signature = ffi.signature;
                Some([<Mj $type_:camel $info_type Info>] {name: name.to_string(), id, model_signature, $($attr,)* $($attr_ffi,)* $($attr_dyn),*})
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
    ($info_type:ident, $name:ident, [$($([$prefix_attr:ident])? $attr:ident: $type_:ty $([$cast:ident])? ),*], [$($([$prefix_opt_attr:ident])? $opt_attr:ident: $type_opt:ty $([$cast_opt:ident])? ),*]$(,$generics:ty: $bound:ty)?) => {
        paste::paste! {
            #[doc = "Stores information required to create views to [`Mj" $info_type "`] arrays corresponding to a " $name "."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel $info_type Info>] {
                pub name: String,
                pub id: usize,
                pub model_signature: u64,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel $info_type Info>] {
                #[doc = concat!("Returns a mutable view to the correct fields in [`Mj", stringify!($info_type), "`].\n",
                                "\n# Panics\n",
                                "Panics if the `", stringify!($info_type), "` instance was created from a model with a different kinematic tree signature.")]
                pub fn view_mut<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p mut [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type ViewMut>]<'p> {
                    assert_eq!(self.model_signature, [<$info_type:lower>].signature(), "model signature mismatch");
                    view_creator!(self, [<Mj $name:camel $info_type ViewMut>], [<$info_type:lower>].ffi(), [$($([$prefix_attr])? $attr $([$cast])?),*], [$($([$prefix_opt_attr])? $opt_attr $([$cast_opt])?),*], crate::util::PointerViewMut::new)
                }

                #[doc = concat!("Returns a view to the correct fields in [`Mj", stringify!($info_type), "`].\n",
                                "\n# Panics\n",
                                "Panics if the `", stringify!($info_type), "` instance was created from a model with a different kinematic tree signature.")]
                pub fn view<'p $(, $generics: $bound)?>(&self, [<$info_type:lower>]: &'p [<Mj $info_type>]$(<$generics>)?) -> [<Mj $name:camel $info_type View>]<'p> {
                    assert_eq!(self.model_signature, [<$info_type:lower>].signature(), "model signature mismatch");
                    view_creator!(self, [<Mj $name:camel $info_type View>], [<$info_type:lower>].ffi(), [$($([$prefix_attr])? $attr $([$cast])?),*], [$($([$prefix_opt_attr])? $opt_attr $([$cast_opt])?),*], crate::util::PointerView::new)
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
    (get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: bool; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Check ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> bool {
                self$(.$ffi())?.$name == 1
            }
        )*
    }};

    (get, [$($([$ffi:ident $(,$ffi_mut:ident)?])? $((allow_mut = $cfg_mut:literal))? $name:ident $(+ $symbol:tt)?: & $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return an immutable reference to ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> &$type {
                &self$(.$ffi())?.$name
            }

            crate::eval_or_expand! {
                @eval $($cfg_mut)? {
                    #[doc = concat!("Return a mutable reference to ", $comment)]
                    pub fn [<$name:camel:snake _mut>](&mut self) -> &mut $type {
                        #[allow(unused_unsafe)]
                        unsafe { &mut self$(.$($ffi_mut())?)?.$name }
                    }
                }
            }
        )*
    }};

    (get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> $type {
                self$(.$ffi())?.$name.into()
            }
        )*
    }};

    (set, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                }
            )*
        }
    };

    /* Enum conversion */
    (force!, get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)? : $type:ty; $comment:expr);* $(;)?]) => {paste::paste!{
        $(
            #[doc = concat!("Return value of ", $comment)]
            pub fn [<$name:camel:snake $($symbol)?>](&self) -> $type {
                unsafe { std::mem::transmute(self$(.$ffi())?.$name) }
            }
        )*
    }};

    (force!, set, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Set ", $comment)]
                pub fn [<set_ $name:camel:snake>](&mut self, value: $type) {
                    #[allow(unnecessary_transmutes)]
                    unsafe { self$(.$ffi_mut())?.$name = std::mem::transmute(value) };
                }
            )*
        }
    };

    /* Builder pattern */
    (force!, with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    #[allow(unnecessary_transmutes)]
                    unsafe { self$(.$ffi_mut())?.$name = std::mem::transmute(value) };
                    self
                }
            )*
        }
    };

    (force!, [&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    #[allow(unnecessary_transmutes)]
                    unsafe { self$(.$ffi_mut())?.$name = std::mem::transmute(value) };
                    self
                }
            )*
        }
    };

    (with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](mut self, value: $type) -> Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };

    ([&] with, [$($([$ffi_mut:ident])? $name:ident: $type:ty; $comment:expr);* $(;)?]) => {
        paste::paste!{ 
            $(
                #[doc = concat!("Builder method for setting ", $comment)]
                pub fn [<with_ $name:camel:snake>](&mut self, value: $type) -> &mut Self {
                    #[allow(unused_unsafe)]
                    unsafe { self$(.$ffi_mut())?.$name = value.into() };
                    self
                }
            )*
        }
    };
    
    /* Handling of optional arguments */
    /* Enum pass */
    (force!, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    (get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    /* Builder pattern */
    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : bool ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)? : bool ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : bool ; $comment );* ]);
    };

    ($([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi])? $name $(+ $symbol)?: $type ; $comment );* ]);
        $crate::getter_setter!(set, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    (force!, $([$token:tt])? with, get, set, [ $($([$ffi: ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? : $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(force!, get, [$($([$ffi])? $name $(+ $symbol)? : $type ; $comment );* ]);
        $crate::getter_setter!(force!, set, [$($([$ffi_mut])? $name : $type ; $comment );* ]);
        $crate::getter_setter!(force!, $([$token])? with, [$($([$ffi_mut])? $name : $type ; $comment );* ]);
    };

    ($([$token:tt])? with, get, [$( $([$ffi: ident, $ffi_mut:ident])? $((allow_mut = $allow_mut:literal))? $name:ident $(+ $symbol:tt)? : & $type:ty ; $comment:expr );* $(;)?]) => {
        $crate::getter_setter!(get, [ $($([$ffi, $ffi_mut])? $((allow_mut = $allow_mut))? $name $(+ $symbol)? : & $type ; $comment );* ]);
        $crate::getter_setter!($([$token])? with, [ $( $([$ffi_mut])? $name : $type ; $comment );* ]);
    };
}


#[doc(hidden)]
#[macro_export]
/// Constructs builder methods.
macro_rules! builder_setters {
    ($($name:ident: $type:ty $(where $generic_type:ident: $generic:path)?; $comment:expr);* $(;)?) => {
        $(
            #[doc = concat!("Set ", $comment)]
            pub fn $name$(<$generic_type: $generic>)?(mut self, value: $type) -> Self {
                self.$name = value.into();
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
    ($($((allow_mut = $cfg_mut:literal))? $name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[$type:ty $([$cast:ident])?; $doc:literal; $($len_accessor:tt)*]),*) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize;
                    let ptr = self.ffi().$name$(.$as_ptr())?$(.$cast::<$type>())?;
                    if ptr.is_null() || length == 0 {
                        return &[];
                    }
                    unsafe { std::slice::from_raw_parts(ptr, length) }
                }

                crate::eval_or_expand! {
                    @eval $($cfg_mut)? {
                        #[doc = concat!("Mutable slice of the ", $doc," array.")]
                        pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                            let length = self.$($len_accessor)* as usize;
                            let ptr = unsafe { self.ffi_mut().$name$(.$as_mut_ptr())?$(.$cast::<$type>())? };
                            if ptr.is_null() || length == 0 {
                                return &mut [];
                            }
                            unsafe { std::slice::from_raw_parts_mut(ptr, length) }
                        }
                    }
                }
            )*
        }
    };

    // Arrays that are of summed variable size
    (summed { $( $(allow_mut = $cfg_mut:literal)? $name:ident: &[$type:ty; $doc:literal; [$multiplier:literal ; ($($len_array:tt)*) ; ($($len_array_length:tt)*)]]),* }) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[[$type; $multiplier]] {
                    let length_array_length = self.$($len_array_length)* as usize;
                    let data_ptr = self.ffi().$name;
                    let length_ptr = self.$($len_array)*;
                    if data_ptr.is_null() || length_ptr.is_null() || length_array_length == 0 {
                        return &[];
                    }

                    let length = unsafe { std::slice::from_raw_parts(
                        length_ptr,
                        length_array_length
                    ).into_iter().map(|&x| x as u32).sum::<u32>() as usize };

                    if length == 0 {
                        return &[];
                    }

                    unsafe { std::slice::from_raw_parts(data_ptr.cast(), length) }
                }
                
                crate::eval_or_expand! {
                    @eval $($cfg_mut)? {
                        #[doc = concat!("Mutable slice of the ", $doc," array.")]
                        pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [[$type; $multiplier]] {
                            let length_array_length = self.$($len_array_length)* as usize;
                            let data_ptr = unsafe { self.ffi_mut().$name };
                            let length_ptr = self.$($len_array)*;
                            if data_ptr.is_null() || length_ptr.is_null() || length_array_length == 0 {
                                return &mut [];
                            }

                            let length = unsafe { std::slice::from_raw_parts(
                                length_ptr,
                                length_array_length
                            ).into_iter().map(|&x| x as u32).sum::<u32>() as usize };

                            if length == 0 {
                                return &mut [];
                            }

                            unsafe { std::slice::from_raw_parts_mut(data_ptr.cast(), length) }
                        }
                    }
                }
            )*
        }
    };

    // Arrays whose second dimension is dependent on some variable
    (sublen_dep {$( $(allow_mut = $cfg_mut:literal)? $name:ident: $($as_ptr:ident $as_mut_ptr:ident)? &[[$type:ty; $($inner_len_accessor:tt)*] $([$cast:ident])?; $doc:literal; $($len_accessor:tt)*]),*}) => {
        paste::paste! {
            $(
                #[doc = concat!("Immutable slice of the ", $doc," array.")]
                pub fn [<$name:camel:snake>](&self) -> &[$type] {
                    let length = self.$($len_accessor)* as usize * self.$($inner_len_accessor)* as usize;
                    let ptr = self.ffi().$name$(.$as_ptr())?$(.$cast::<$type>())?;
                    if ptr.is_null() || length == 0 {
                        return &[];
                    }

                    unsafe { std::slice::from_raw_parts(ptr, length) }
                }

                crate::eval_or_expand! {
                    @eval $($cfg_mut)? {
                        #[doc = concat!("Mutable slice of the ", $doc," array.")]
                        pub fn [<$name:camel:snake _mut>](&mut self) -> &mut [$type] {
                            let length = self.$($len_accessor)* as usize * self.$($inner_len_accessor)* as usize;
                            let ptr = unsafe { self.ffi_mut().$name$(.$as_mut_ptr())?$(.$cast::<$type>())? };
                            if ptr.is_null() || length == 0 {
                                return &mut [];
                            }
                            unsafe { std::slice::from_raw_parts_mut(ptr, length) }
                        }
                    }
                }
            )*
        }
    };
}

/// Generates getter and setter methods for converting between Rust's &str type and C's char arrays.
/// The macro works by first specifying the methods to create (get = getter, set = setter) --- c_str_as_str_method {get, set, {...}} ---
/// and then providing the rest of the parameters.
/// 
/// The rest of the parameters are recursive and are as follows:
/// - ffi (optional): name of the method that returns some lower-level struct,
///                   which contains the actual attributes we want to read;
/// - name: the attribute name;
/// - sub_index_name: sub_index_type (optional): creates an additional parameter which indexes a the `name` array
///                   in order to get a sub-array
///                   (e.g., `name` could be `[[i8; 100]; 10]` and we wish to get `[i8; 100]`);
/// - comment: the documentation comment to insert as the methods documentation.
/// 
#[doc(hidden)]
#[macro_export]
macro_rules! c_str_as_str_method {
    (get {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {
        $(
            #[doc = concat!("Returns ", $comment, "\n\n# Panics", "\nPanics if the string contains invalid UTF-8.")]
            pub fn $name(&self $(, $sub_index_name: $sub_index_type)? ) -> &str {
                unsafe { 
                    let c_ptr = self$(.$ffi())?.$name$([$sub_index_name])?.as_ptr();
                    std::ffi::CStr::from_ptr(c_ptr).to_str().unwrap()
                }
            }
        )*
    };

    (set {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {paste::paste!{
        $(
            #[doc = concat!("Sets ", $comment, "\n\n# Panics", "\nPanics when `", stringify!($name), "` contains invalid ASCII or is too long.")]
            pub fn [<set_ $name>](&mut self, $($sub_index_name: $sub_index_type,)? $name: &str) {
                assert!($name.is_ascii(), concat!(stringify!($name), " must be valid ASCII."));
                let c_string = std::ffi::CString::new($name).unwrap();
                let bytes = c_string.into_bytes_with_nul();

                // This transmute is safe as long as converting from u8 (bytes) to i8, which is char.
                self$(.$ffi())?.$name$([$sub_index_name])?[..bytes.len()].copy_from_slice(unsafe { std::mem::transmute::<&[u8], &[i8]>(&bytes) });
            }
        )*
    }};

    (with {$($([$ffi:ident])? $name:ident $([$sub_index_name:ident: $sub_index_type:ty])?; $comment:literal; )*}) => {paste::paste!{
        $(
            #[doc = concat!("Builder method for setting", $comment, "\n\n# Panics", "\nPanics when `", stringify!($name), "` contains invalid ASCII or is too long.")]
            pub fn [<with_ $name>](mut self, $($sub_index_name: $sub_index_type,)? $name: &str) -> Self {
                assert!($name.is_ascii(), concat!(stringify!($name), " must be valid ASCII."));
                let c_string = std::ffi::CString::new($name).unwrap();
                let bytes = c_string.into_bytes_with_nul();

                // This transmute is safe as long as converting from u8 (bytes) to i8, which is char.
                self$(.$ffi())?.$name$([$sub_index_name])?[..bytes.len()].copy_from_slice(unsafe { std::mem::transmute::<&[u8], &[i8]>(&bytes) });
                self
            }
        )*
    }};

    // Mixed patterns
    (with, get, set {$($other:tt)*}) => {
        crate::c_str_as_str_method!(get {$($other)*});
        crate::c_str_as_str_method!(set {$($other)*});
        crate::c_str_as_str_method!(with {$($other)*});
    };

    (get, set {$($other:tt)*}) => {
        crate::c_str_as_str_method!(get {$($other)*});
        crate::c_str_as_str_method!(set {$($other)*});
    };

    (with, set {$($other:tt)*}) => {
        crate::c_str_as_str_method!(set {$($other)*});
        crate::c_str_as_str_method!(with {$($other)*});
    };

    (with, get {$($other:tt)*}) => {
        crate::c_str_as_str_method!(get {$($other)*});
        crate::c_str_as_str_method!(with {$($other)*});
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


/// Tries to cast $value into requested type.
/// # Panics
/// Panics if the cast fails.
#[doc(hidden)]
#[macro_export]
macro_rules! cast_mut_info {
    ($value:expr $(, $debug_expr:expr)?) => {
        {
            let evaluated = format!("{:?}", $value);
            bytemuck::checked::try_cast_mut($value)
                .unwrap_or_else(
                    |e| {
                        #[allow(unused)]
                        let mut debug_info = String::new();
                        $(
                            debug_info = format!(" (debug info: '{} = {}')", stringify!($debug_expr), $debug_expr);
                        )?

                        panic!(
                            "failed to cast expression '{}', which evaluates to '{}' into requested type (error: {})\
                             {debug_info} --- \
                             most likely you have a bug in your program.",
                            stringify!($value), evaluated, e
                        );
                    }
                )
        }
    };
}

/// Asserts that the MuJoCo version used matches
/// the one MuJoCo-rs was compiled with.
pub fn assert_mujoco_version() {
    let linked_version = unsafe { mj_version() as u32 };
    let mujoco_rs_version_string = option_env!("CARGO_PKG_VERSION").unwrap_or_else(|| "unknown+mj-unknown");
    assert_eq!(
        linked_version, mjVERSION_HEADER,
        "linked MuJoCo version value ({linked_version}) does not match expected version value ({mjVERSION_HEADER}), \
        with which MuJoCo-rs {mujoco_rs_version_string} FFI bindings were generated.",
    );
}


/* Utility traits */
/// Locks a synchronization primitive and resets its poison status.
/// This is useful on locations that don't need any special handling
/// after a thread panicked while holding a mutex lock.
pub trait LockUnpoison<T> {
    fn lock_unpoison(&self) -> MutexGuard<'_, T>;
}

/// Implements automatic unpoisoning on the [`Mutex`].
impl<T> LockUnpoison<T> for Mutex<T> {
    fn lock_unpoison(&self) -> MutexGuard<'_, T> {
        match self.lock() {
            Ok(lock) => lock,
            Err(e) => {
                self.clear_poison();
                e.into_inner()
            }
        }
    }
}
