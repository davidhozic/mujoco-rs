use std::marker::PhantomData;
use std::ffi::CString;

use super::mj_auxiliary::MjContact;
use super::mj_model::MjModel;
use crate::mujoco_c::*;

use crate::util::{PointerViewMut, PointerView};
use crate::{mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem};


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

    pub fn body(&self, name:&str) -> Option<MjBodyInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_BODY as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let id = id as usize;
        Some(MjBodyInfo {name: name.to_string(), id})
    }


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

    /// Obtains a [`MjGeomInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjGeomInfo::view`].
    pub fn geom(&self, name: &str) -> Option<MjGeomInfo> {
        let cname = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_GEOM as i32, cname.as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let id = id as usize;
        let xpos = (id * 3, 3);
        let xmat = (id * 9, 9);
        Some(MjGeomInfo { name: name.to_string(), id: id as usize, xmat, xpos })
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
}


/**************************************************************************************************/
// Joint view
/**************************************************************************************************/

/// Describes a joint and allows
/// creation of views to joint data in MjData.
#[derive(Debug, PartialEq)]
#[allow(non_snake_case)]
pub struct MjJointInfo{
    pub name: String,
    pub id: usize,
    qpos: (usize, usize),
    qvel: (usize, usize),
    qacc_warmstart: (usize, usize),
    qfrc_applied: (usize, usize),
    qacc: (usize, usize),
    xanchor: (usize, usize),
    xaxis: (usize, usize),
    qLDiagInv: (usize, usize),
    qfrc_bias: (usize, usize),
    qfrc_passive: (usize, usize),
    qfrc_actuator: (usize, usize),
    qfrc_smooth: (usize, usize),
    qacc_smooth: (usize, usize),
    qfrc_constraint: (usize, usize),
    qfrc_inverse: (usize, usize),
}

impl MjJointInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&self, data: &'d mut MjData) -> MjJointViewMut<'d> {
        view_creator!(
            self, MjJointViewMut, data.ffi(),
            [
                qpos, qvel, qacc_warmstart, qfrc_applied, qacc, xanchor, xaxis, qLDiagInv, qfrc_bias,
                qfrc_passive, qfrc_actuator, qfrc_smooth, qacc_smooth, qfrc_constraint, qfrc_inverse
            ],
            [], PointerViewMut::new
        )
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjJointView<'d> {
        view_creator!(
            self, MjJointView, data.ffi(),
            [
                qpos, qvel, qacc_warmstart, qfrc_applied, qacc, xanchor, xaxis, qLDiagInv, qfrc_bias,
                qfrc_passive, qfrc_actuator, qfrc_smooth, qacc_smooth, qfrc_constraint, qfrc_inverse
            ],
            [], PointerView::new
        )
    }
}

/// A mutable view to joint variables of MjData.
#[allow(non_snake_case)]
pub struct MjJointViewMut<'d> {
    pub qpos: PointerViewMut<'d, f64>,
    pub qvel: PointerViewMut<'d, f64>,
    pub qacc_warmstart: PointerViewMut<'d, f64>,
    pub qfrc_applied: PointerViewMut<'d, f64>,
    pub qacc: PointerViewMut<'d, f64>,
    pub xanchor: PointerViewMut<'d, f64>,
    pub xaxis: PointerViewMut<'d, f64>,
    pub qLDiagInv: PointerViewMut<'d, f64>,
    pub qfrc_bias: PointerViewMut<'d, f64>,
    pub qfrc_passive: PointerViewMut<'d, f64>,
    pub qfrc_actuator: PointerViewMut<'d, f64>,
    pub qfrc_smooth: PointerViewMut<'d, f64>,
    pub qacc_smooth: PointerViewMut<'d, f64>,
    pub qfrc_constraint: PointerViewMut<'d, f64>,
    pub qfrc_inverse: PointerViewMut<'d, f64>,
}

impl MjJointViewMut<'_> {
    /// Deprecated. Use [`MjJointViewMut::zero`] instead.
    #[deprecated]
    pub fn reset(&mut self) {
        self.zero();
    }

    /// Resets the internal variables to 0.0.
    pub fn zero(&mut self) {
        self.qpos.fill(0.0);
        self.qvel.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.qfrc_applied.fill(0.0);
        self.qacc.fill(0.0);
        self.xanchor.fill(0.0);
        self.xaxis.fill(0.0);
        self.qLDiagInv.fill(0.0);
        self.qfrc_bias.fill(0.0);
        self.qfrc_passive.fill(0.0);
        self.qfrc_actuator.fill(0.0);
        self.qfrc_smooth.fill(0.0);
        self.qacc_smooth.fill(0.0);
        self.qfrc_constraint.fill(0.0);
        self.qfrc_inverse.fill(0.0);
    }
}


/// An immutable view to joint variables of MjData.
#[allow(non_snake_case)]
pub struct MjJointView<'d> {
    pub qpos: PointerView<'d, f64>,
    pub qvel: PointerView<'d, f64>,
    pub qacc_warmstart: PointerView<'d, f64>,
    pub qfrc_applied: PointerView<'d, f64>,
    pub qacc: PointerView<'d, f64>,
    pub xanchor: PointerView<'d, f64>,
    pub xaxis: PointerView<'d, f64>,
    pub qLDiagInv: PointerView<'d, f64>,
    pub qfrc_bias: PointerView<'d, f64>,
    pub qfrc_passive: PointerView<'d, f64>,
    pub qfrc_actuator: PointerView<'d, f64>,
    pub qfrc_smooth: PointerView<'d, f64>,
    pub qacc_smooth: PointerView<'d, f64>,
    pub qfrc_constraint: PointerView<'d, f64>,
    pub qfrc_inverse: PointerView<'d, f64>,
}


/**************************************************************************************************/
// Geom view
/**************************************************************************************************/

/// Describes a geom and allows
/// creation of views to geom data in MjData.
#[derive(Debug, PartialEq)]
pub struct MjGeomInfo {
    pub name: String,
    pub id: usize,
    xmat: (usize, usize),
    xpos: (usize, usize)
}

impl MjGeomInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&self, data: &'d mut MjData) -> MjGeomViewMut<'d> {
        view_creator!(self, MjGeomViewMut, data.ffi(), geom_, [xmat, xpos], [], PointerViewMut::new)
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjGeomView<'d> {
        view_creator!(self, MjGeomView, data.ffi(), geom_, [xmat, xpos], [], PointerView::new)
    }
}

/// A mutable view to geom variables of MjData.
pub struct MjGeomViewMut<'d> {
    pub xmat: PointerViewMut<'d, f64>,
    pub xpos: PointerViewMut<'d, f64>,
}

impl MjGeomViewMut<'_> {
    /// Resets the internal variables to 0.0.
    pub fn zero(&mut self) {
        self.xmat.fill(0.0);
        self.xpos.fill(0.0);
    }
}

/// An immutable view to geom variables of MjData.
pub struct MjGeomView<'d> {
    pub xmat: PointerView<'d, f64>,
    pub xpos: PointerView<'d, f64>,
}


/**************************************************************************************************/
// Actuator view
/**************************************************************************************************/

/// Describes an actuator and allows
/// creation of views to actuator data in MjData.
#[derive(Debug, PartialEq)]
pub struct MjActuatorInfo {
    pub name: String,
    pub id: usize,
    ctrl: (usize, usize),
    act: (usize, usize),
}

impl MjActuatorInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&self, data: &'d mut MjData) -> MjActuatorViewMut<'d> {
        view_creator!(self, MjActuatorViewMut, data.ffi(), [ctrl], [act], PointerViewMut::new)
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjActuatorView<'d> {
        view_creator!(self, MjActuatorView, data.ffi(), [ctrl], [act], PointerView::new)
    }
}

/// A mutable view to actuator variables of MjData.
pub struct MjActuatorViewMut<'d> {
    pub ctrl: PointerViewMut<'d, f64>,
    pub act: Option<PointerViewMut<'d, f64>>,
}


impl MjActuatorViewMut<'_> {
    /// Resets the internal variables to 0.0.
    pub fn zero(&mut self) {
        self.ctrl.fill(0.0);
        if let Some(a) = &mut self.act {
            a.fill(0.0);
        }
    }
}

/// An immutable view to actuator variables of MjData.
pub struct MjActuatorView<'d> {
    pub ctrl: PointerView<'d, f64>,
    pub act: Option<PointerView<'d, f64>>,
}


/**************************************************************************************************/
// Body view
/**************************************************************************************************/

/// Describes a body and allows
/// creation of views to body data in MjData.
// #[derive(Debug, PartialEq)]
pub struct MjBodyInfo {
    pub name: String,
    pub id: usize,
}

impl MjBodyInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&self, data: &'d mut MjData) -> MjBodyViewMut<'d> {
        view_creator!(
            self, MjBodyViewMut, data.ffi(),
            [
                xfrc_applied: &mut [f64; 6]
            ]
        )
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjBodyView<'d> {
        view_creator!(
            self, MjBodyView, data.ffi(),
            [
                xfrc_applied: &[f64; 6],
                xpos: &[f64; 3],
                xquat: &[f64; 4],
                xmat: &[f64; 9],
                xipos: &[f64; 3],
                ximat: &[f64; 9],
                subtree_com: &[f64; 3],
                cinert: &[f64; 10],
                crb: & [f64; 10],
                cvel: & [f64; 6],
                subtree_linvel: &[f64; 3],
                subtree_angmom: &[f64; 3],
                cacc: &[f64; 6],
                cfrc_int: &[f64; 6],
                cfrc_ext: &[f64; 6]
            ]
        )
    }
}

/// A mutable view to actuator variables of MjData.
pub struct MjBodyViewMut<'d> {
    pub xfrc_applied: &'d mut [f64; 6],
}


impl MjBodyViewMut<'_> {
    /// Resets the internal variables to 0.0.
    pub fn zero(&mut self) {
        self.xfrc_applied.fill(0.0);
    }
}

/// An immutable view to actuator variables of MjData.
pub struct MjBodyView<'d> {
    pub xfrc_applied: &'d [f64; 6],
    pub xpos: &'d [f64; 3],
    pub xquat: &'d [f64; 4],
    pub xmat: &'d [f64; 9],
    pub xipos: &'d [f64; 3],
    pub ximat: &'d [f64; 9],
    pub subtree_com: &'d [f64; 3],
    pub cinert: &'d [f64; 10],
    pub crb: &'d [f64; 10],
    pub cvel: &'d [f64; 6],
    pub subtree_linvel: &'d [f64; 3],
    pub subtree_angmom: &'d [f64; 3],
    pub cacc: &'d [f64; 6],
    pub cfrc_int: &'d [f64; 6],
    pub cfrc_ext: &'d [f64; 6]
}


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
}
