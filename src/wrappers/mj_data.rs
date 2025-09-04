use super::mj_auxiliary::MjContact;
use super::mj_model::MjModel;
use crate::mujoco_c::*;
use std::ffi::CString;

use crate::{mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem};
use crate::util::{PointerViewMut, PointerView};


/**************************************************************************************************/
// Macros
/**************************************************************************************************/

/// Creates a $view struct, mapping $field and $opt_field to the same location as in $data.
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
/// items that have fixed size arrays in [`MjData`].
/// This creates a method `X(self, name; &str) -> XInfo`.
macro_rules! fixed_size_info_method {
    ($type_:ident, [$($attr:ident: $len:literal),*]) => {
        paste::paste! {
            #[doc = concat!(
                "Obtains a [`", stringify!([<Mj $type_:camel Info>]), "`] struct containing information about the name, id, and ",
                "indices required for obtaining references to the correct locations in [`MjData`]. ",
                "The actual view can be obtained via [`", stringify!([<Mj $type_:camel Info>]), "::view`]."
            )]
            pub fn $type_(&self, name: &str) -> Option<[<Mj $type_:camel Info>]> {
                let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::[<mjOBJ_ $type_:upper>] as i32, CString::new(name).unwrap().as_ptr())};
                if id == -1 {  // not found
                    return None;
                }

                let id = id as usize;
                $(
                    let $attr = (id * $len, $len);
                )*

                Some([<Mj $type_:camel Info>] {name: name.to_string(), id, $($attr),*})
            }
        }
    }
}


/// Creates the xInfo struct along with corresponding xView and xViewMut structs.
macro_rules! info_with_view {
    /* PointerView */

    /* name of the view/info, attribute prefix in MjData, [attributes always present], [attributes that can be None] */
    ($name:ident, $prefix:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to [`MjData`] arrays corresponding to a " $name "."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel Info>] {
                pub name: String,
                pub id: usize,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel Info>] {
                /// Returns a mutable view to the correct fields in [`MjData`].
                pub fn view_mut<'d>(&self, data: &'d mut MjData) -> [<Mj $name:camel ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel ViewMut>], data.ffi(), $prefix, [$($attr),*], [$($opt_attr),*], PointerViewMut::new)
                }

                /// Returns a view to the correct fields in [`MjData`].
                pub fn view<'d>(&self, data: &'d MjData) -> [<Mj $name:camel View>]<'d> {
                    view_creator!(self, [<Mj $name:camel View>], data.ffi(), $prefix, [$($attr),*], [$($opt_attr),*], PointerView::new)
                }
            }

            #[doc = "A mutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel ViewMut>]<'d> {
                $(
                    pub $attr: PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel ViewMut>]<'_> {
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
            pub struct [<Mj $name:camel View>]<'d> {
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
    ($name:ident, [$($attr:ident: $type_:ty),*], [$($opt_attr:ident: $type_opt:ty),*]) => {
        paste::paste! {
            #[doc = "Stores information required to create views to MjData arrays corresponding to a " $name "."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel Info>] {
                pub name: String,
                pub id: usize,
                $(
                    $attr: (usize, usize),
                )*
                $(
                    $opt_attr: (usize, usize),
                )*
            }

            impl [<Mj $name:camel Info>] {
                /// Returns a mutable view to the correct fields in [`MjData`].
                pub fn view_mut<'d>(&self, data: &'d mut MjData) -> [<Mj $name:camel ViewMut>]<'d> {
                    view_creator!(self, [<Mj $name:camel ViewMut>], data.ffi(), [$($attr),*], [$($opt_attr),*], PointerViewMut::new)
                }

                /// Returns a view to the correct fields in [`MjData`].
                pub fn view<'d>(&self, data: &'d MjData) -> [<Mj $name:camel View>]<'d> {
                    view_creator!(self, [<Mj $name:camel View>], data.ffi(), [$($attr),*], [$($opt_attr),*], PointerView::new)
                }
            }


            #[doc = "A mutable view to " $name " variables of [`MjData`]."]
            #[allow(non_snake_case)]
            pub struct [<Mj $name:camel ViewMut>]<'d> {
                $(
                    pub $attr: PointerViewMut<'d, $type_>,
                )*
                $(
                    pub $opt_attr: Option<PointerViewMut<'d, $type_opt>>,
                )*
            }

            impl [<Mj $name:camel ViewMut>]<'_> {
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
            pub struct [<Mj $name:camel View>]<'d> {
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


/**************************************************************************************************/
// MjData
/**************************************************************************************************/

/// Wrapper around the ``mjData`` struct.
/// Provides lifetime guarantees as well as automatic cleanup.
pub struct MjData<'a> {
    data: *mut mjData,
    model: &'a MjModel
}

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl Send for MjData<'_> {}
unsafe impl Sync for MjData<'_> {}


impl<'a> MjData<'a> {
    /// Constructor for a new MjData. This should is called from MjModel.
    pub fn new(model: &'a MjModel) -> Self {
        unsafe {
            Self {
                data: mj_makeData(model.ffi()),
                model: model,
            }
        }
    }

    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjData {
        unsafe { self.data.as_ref().unwrap() }
    }

    /// Mutable reference to the wrapped FFI struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjData {
        unsafe { self.data.as_mut().unwrap() }
    }

    /// Returns a slice of detected contacts.
    /// To obtain the contact force, call [`MjData::contact_force`].
    pub fn contacts(&self) -> &[MjContact] {
        unsafe {
            std::slice::from_raw_parts((*self.data).contact, (*self.data).ncon as usize)
        }
    }

    /// Obtains a [`MjActuatorInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjActuatorInfo::view`].
    pub fn actuator(&self, name: &str) -> Option<MjActuatorInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_ACTUATOR as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let ctrl;
        let act;
        let model_ffi = self.model.ffi();
        unsafe {
            ctrl = (id as usize, 1);
            act = mj_view_indices!(id as usize, model_ffi.actuator_actadr, model_ffi.nu as usize, model_ffi.na as usize);
        }

        Some(MjActuatorInfo { name: name.to_string(), id: id as usize, ctrl, act})
    }

    fixed_size_info_method! { body, [xfrc_applied: 6, xpos: 3, xquat: 4, xmat: 9, xipos: 3, ximat: 9, subtree_com: 3, cinert: 10, crb: 10, cvel: 6, subtree_linvel: 3, subtree_angmom: 3, cacc: 6, cfrc_int: 6, cfrc_ext: 6] }
    fixed_size_info_method! { camera, [xpos: 3, xmat: 9] }
    fixed_size_info_method! { geom, [xpos: 3, xmat: 9] }
    fixed_size_info_method! { site, [xpos: 3, xmat: 9] }
    fixed_size_info_method! { light, [xpos: 3, xdir: 3] }


    /// Obtains a [`MjJointInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjJointInfo::view`].
    pub fn joint(&self, name: &str) -> Option<MjJointInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_JOINT as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }
        let model_ffi = self.model.ffi();
        let id = id as usize;
        unsafe {
            // $id:expr, $addr_map:expr, $njnt:expr, $max_n:expr
            let qpos = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nq), mj_model_nx_to_nitem!(model_ffi, nq), model_ffi.nq);
            let qvel = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qacc_warmstart = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_applied = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qacc = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let xanchor = (id * 3, 3);
            let xaxis = (id * 3, 3);
            #[allow(non_snake_case)]
            let qLDiagInv = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_bias = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_passive = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_actuator = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_smooth = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qacc_smooth = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_constraint = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            let qfrc_inverse = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);
            
            /* Special case attributes, used for some internal calculation */
            // cdof
            // cdof_dot

            Some(MjJointInfo {name: name.to_string(), id: id as usize,
                qpos, qvel, qacc_warmstart, qfrc_applied, qacc, xanchor, xaxis, qLDiagInv, qfrc_bias,
                qfrc_passive, qfrc_actuator, qfrc_smooth, qacc_smooth, qfrc_constraint, qfrc_inverse
            })
        }
    }

    pub fn sensor(&self, name: &str) -> Option<MjSensorInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_SENSOR as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }
        let model_ffi = self.model.ffi();
        let id = id as usize;

        unsafe {
            let data = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nsensordata), mj_model_nx_to_nitem!(model_ffi, nsensordata), model_ffi.nsensordata);
            Some(MjSensorInfo { id, name: name.to_string(), data })
        }
    }

    #[allow(non_snake_case)]
    pub fn tendon(&self, name: &str) -> Option<MjTendonInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_TENDON as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let model_ffi = self.model.ffi();
        let id = id as usize;
        let ntendon = model_ffi.ntendon as usize;
        let nv = model_ffi.nv as usize;
        let wrapadr = (id, 1);
        let wrapnum = (id, 1);
        let J_rownnz = (id, 1);
        let J_rowadr = (id, 1);
        let J_colind = (id * nv, nv);
        let length = (id, 1);
        let J = (id * nv, nv);
        let velocity = (ntendon, 1);

        Some(MjTendonInfo { id, name: name.to_string(), wrapadr, wrapnum, J_rownnz, J_rowadr, J_colind, length, J, velocity })
    }

    /// Steps the MuJoCo simulation.
    pub fn step(&mut self) {
        unsafe {
            mj_step(self.model.ffi(), self.ffi_mut());
        }
    }

    /// Calculates new dynamics. This is a wrapper around `mj_step1`.
    pub fn step1(&mut self) {
        unsafe {
            mj_step1(self.model.ffi(), self.ffi_mut());
        }
    }

    /// Calculates the rest after dynamics and integrates in time.
    /// This is a wrapper around `mj_step2`.
    pub fn step2(&mut self) {
        unsafe {
            mj_step2(self.model.ffi(), self.ffi_mut());
        }
    }

    /// Forward dynamics: same as mj_step but do not integrate in time.
    /// This is a wrapper around `mj_forward`.
    pub fn forward(&mut self) {
        unsafe { 
            mj_forward(self.model.ffi(), self.ffi_mut());
        }
    }

    /// [`MjData::forward`] dynamics with skip.
    /// This is a wrapper around `mj_forwardSkip`.
    pub fn forward_skip(&mut self, skipstage: mjtStage, skipsensor: bool) {
        unsafe { 
            mj_forwardSkip(self.model.ffi(), self.ffi_mut(), skipstage as i32, skipsensor as i32);
        }
    }

    /// Inverse dynamics: qacc must be set before calling ([`MjData::forward`]).
    /// This is a wrapper around `mj_inverse`.
    pub fn inverse(&mut self) {
        unsafe {
            mj_inverse(self.model.ffi(), self.ffi_mut());
        }
    }

    /// [`MjData::inverse`] dynamics with skip; skipstage is mjtStage.
    /// This is a wrapper around `mj_inverseSkip`.
    pub fn inverse_skip(&mut self, skipstage: mjtStage, skipsensor: bool) {
        unsafe {
            mj_inverseSkip(self.model.ffi(), self.ffi_mut(), skipstage as i32, skipsensor as i32);
        }
    }

    /// Calculates the contact force for the given `contact_id`.
    /// The `contact_id` matches the index of the contact when iterating
    /// via [`MjData::contacts`].
    /// Calls `mj_contactForce` internally.
    pub fn contact_force(&self, contact_id: usize) -> [f64; 6] {
        let mut force = [0.0; 6];
        unsafe {
            mj_contactForce(
                self.model.ffi(), self.data,
                contact_id as i32, force.as_mut_ptr()
            );
        }
        force
    }

    /// Returns a direct pointer to the underlying model.
    /// THIS IS NOT TO BE USED.
    /// It is only meant for the viewer code, which currently still depends
    /// on mutable pointers to model and data. This will be removed in the future.
    pub(crate) unsafe fn __raw(&self) -> *mut mjData {
        self.data
    }

}

impl Drop for MjData<'_> {
    fn drop(&mut self) {
        unsafe {
            mj_deleteData(self.data);
        }
    }
}


/**************************************************************************************************/
// Joint view
/**************************************************************************************************/
info_with_view!(
    joint,
    [
        qpos: f64, qvel: f64, qacc_warmstart: f64, qfrc_applied: f64, qacc: f64, xanchor: f64, xaxis: f64, qLDiagInv: f64, qfrc_bias: f64,
        qfrc_passive: f64, qfrc_actuator: f64, qfrc_smooth: f64, qacc_smooth: f64, qfrc_constraint: f64, qfrc_inverse: f64
    ],
    []
);

/* Backward compatibility */
impl MjJointViewMut<'_> {
    /// Deprecated. Use [`MjJointViewMut::zero`] instead.
    #[deprecated]
    pub fn reset(&mut self) {
        self.zero();
    }
}


/**************************************************************************************************/
// Sensor view
/**************************************************************************************************/
info_with_view!(sensor, sensor, [data: f64], []);

/**************************************************************************************************/
// Geom view
/**************************************************************************************************/
info_with_view!(geom, geom_, [xpos: f64, xmat: f64], []);

/**************************************************************************************************/
// Actuator view
/**************************************************************************************************/
info_with_view!(actuator, [ctrl: f64], [act: f64]);

/**************************************************************************************************/
// Body view
/**************************************************************************************************/
info_with_view!(
    body, [
        xfrc_applied: f64, xpos: f64, xquat: f64, xmat: f64, xipos: f64, ximat: f64,
        subtree_com: f64, cinert: f64, crb: f64, cvel: f64, subtree_linvel: f64,
        subtree_angmom: f64, cacc: f64, cfrc_int: f64, cfrc_ext: f64
    ], []
);

/**************************************************************************************************/
// Camera view
/**************************************************************************************************/
info_with_view!(camera, cam_, [xpos: f64, xmat: f64], []);

/**************************************************************************************************/
// Site view
/**************************************************************************************************/
info_with_view!(site, site_, [xpos: f64, xmat: f64], []);

/**************************************************************************************************/
// Tendon view
/**************************************************************************************************/
info_with_view!(tendon, ten_, [wrapadr: i32, wrapnum: i32, J_rownnz: i32, J_rowadr: i32, J_colind: i32, length: f64, J: f64, velocity: f64], []);

/**************************************************************************************************/
// Site view
/**************************************************************************************************/
info_with_view!(light, light_, [xpos: f64, xdir: f64], []);

/**************************************************************************************************/
// Unit tests
/**************************************************************************************************/

#[cfg(test)]
mod test {
    use crate::prelude::*;

    const MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\">
        <geom name=\"green_sphere\" pos=\".2 .2 .1\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball_joint\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" solref=\"0.004 1.0\"/>
  </worldbody>
</mujoco>
";

    macro_rules! assert_almost_eq {
        ($a:expr, $b:expr) => {
            assert!(($a - $b).abs() < 1e-6)
        }
    }

    #[test]
    fn test_body_view() {
        let body = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = body.make_data();
        let body_info = data.body("ball").unwrap();
        let mut cvel;

        data.step1();

        for _ in 0..10 {
            data.step2();
            data.step1();  // step() and step2() update before integration, thus we need to manually update non-state variables.
        }

        // The ball should start in a still position
        cvel = body_info.view(&data).cvel;
        assert_almost_eq!(cvel[0], 0.0);
        assert_almost_eq!(cvel[1], 0.0);
        assert_almost_eq!(cvel[2], 0.0);
        assert_almost_eq!(cvel[3], 0.0);
        assert_almost_eq!(cvel[4], 0.0);
        // assert_almost_eq!(cvel[5], 0.0);  // Ignore due to slight instability of the model.

        // Give the ball some velocity
        body_info.view_mut(&mut data).xfrc_applied[0] = 5.0;
        data.step2();
        data.step1();

        let view = body_info.view(&data);
        cvel = view.cvel;
        println!("{:?}", cvel);
        assert_almost_eq!(cvel[0], 0.0);
        assert!(cvel[1] > 0.0);  // wy should be positive when rolling with positive vx.
        assert_almost_eq!(cvel[2], 0.0);
        assert!(cvel[3] > 0.0);  // vx should point in the direction of the applied force.
        assert_almost_eq!(cvel[4], 0.0);  // vy should be 0.
        // assert_almost_eq!(cvel[5], 0.0);  // vz should be 0, but we don't test it due to jumpiness (instability) of the ball.

        assert_almost_eq!(view.xfrc_applied[0], 5.0); // the original force should stay applied.

        data.step2();
        data.step1();
    }

    #[test]
    fn test_camera_view() {
        let body = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = body.make_data();
        // let body_info = data.body("ball").unwrap();
    }
}
