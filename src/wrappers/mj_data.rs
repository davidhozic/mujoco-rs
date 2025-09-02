use std::marker::PhantomData;
use std::ffi::CString;

use super::mj_auxiliary::MjContact;
use super::mj_model::MjModel;
use crate::mujoco_c::*;

use crate::util::{PointerViewMut, PointerView};
use crate::mj_slice_view;


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

    /// Obtains a [`MjJointInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjJointInfo::view`].
    pub fn joint(&self, name: &str) -> Option<MjJointInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_JOINT as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let qpos;
        let qvel;
        let qacc_warmstart;
        let qacc;
        let qfrc_applied;
        let qfrc_bias;
        let model_ffi = self.model.ffi();
        unsafe {
            // ptr:expr, $id:expr, $addr_map:expr, $njnt:expr, $max_n:expr
            qpos = mj_slice_view!(self.qpos, id as usize, model_ffi.jnt_qposadr, model_ffi.njnt as usize, model_ffi.nq as usize);
            qvel = mj_slice_view!(self.qvel, id as usize, model_ffi.jnt_dofadr, model_ffi.njnt as usize, model_ffi.nv as usize);
            qacc_warmstart = mj_slice_view!(self.qacc_warmstart, id as usize, model_ffi.jnt_dofadr, model_ffi.njnt as usize, model_ffi.nv as usize);
            qacc = mj_slice_view!(self.qacc, id as usize, model_ffi.jnt_dofadr, model_ffi.njnt as usize, model_ffi.nv as usize);
            qfrc_applied = mj_slice_view!(self.qfrc_applied, id as usize, model_ffi.jnt_dofadr, model_ffi.njnt as usize, model_ffi.nv as usize);
            qfrc_bias = mj_slice_view!(self.qfrc_bias, id as usize, model_ffi.jnt_dofadr, model_ffi.njnt as usize, model_ffi.nv as usize);
        }

        Some(MjJointInfo {name: name.to_string(), id: id as usize, qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias})
    }

    /// Obtains a [`MjGeomInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjGeomInfo::view`].
    pub fn geom(&self, name: &str) -> Option<MjGeomInfo> {
        const GEOM_XPOS_LEN: usize = 3;
        const GEOM_XMAT_LEN: usize = 9;

        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj::mjOBJ_GEOM as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let xpos = (GEOM_XPOS_LEN * id as usize, 3);
        let xmat = (GEOM_XMAT_LEN * id as usize, 9);
        Some(MjGeomInfo { name: name.to_string(), id: id as usize, xmat, xpos })
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
            act = mj_slice_view!(self.act, id as usize, model_ffi.actuator_actadr, model_ffi.nu as usize, model_ffi.na as usize);
        }

        Some(MjActuatorInfo { name: name.to_string(), id: id as usize, ctrl, act})
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
            mj_deleteData(self.ffi_mut());
        }
    }
}


/// Creates a $view struct, mapping $field and $opt_field to the same location as in $data.
macro_rules! view_creator {
    // Mutable
    ($self:expr, $view:ident, $data:expr, [$($field:ident),*], [$($opt_field:ident),*], true) => {
        unsafe {
            $view {
                $(
                    $field: PointerViewMut::new($data.$field.add($self.$field.0), $self.$field.1),
                )*
                $(
                    $opt_field: if $self.$opt_field.1 > 0 {
                        Some(PointerViewMut::new($data.$opt_field.add($self.$opt_field.0), $self.$opt_field.1))
                    } else {None},
                )*
                phantom: PhantomData,
            }
        }
    };

    // Non-mutable
    ($self:expr, $view:ident, $data:expr, [$($field:ident),*], [$($opt_field:ident),*], false) => {
        unsafe {
            $view {
                $(
                    $field: PointerView::new($data.$field.add($self.$field.0), $self.$field.1),
                )*
                $(
                    $opt_field: if $self.$opt_field.1 > 0 {
                        Some(PointerView::new($data.$opt_field.add($self.$opt_field.0), $self.$opt_field.1))
                    } else {None},
                )*
                phantom: PhantomData,
            }
        }
    };
}


/// A MjJointInfo which shows a slice of the joint.
#[derive(Debug, PartialEq)]
pub struct MjJointInfo{
    pub name: String,
    pub id: usize,
    qpos: (usize, usize),
    qvel: (usize, usize),
    qacc_warmstart: (usize, usize),
    qacc: (usize, usize),
    qfrc_applied: (usize, usize),
    qfrc_bias: (usize, usize),
}

impl MjJointInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjJointViewMut<'d, '_> {
        view_creator!(self, MjJointViewMut, data.ffi_mut(), [qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias], [], true)
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjJointView<'d, '_> {
        view_creator!(self, MjJointView, data.ffi(), [qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias], [], false)
    }
}

pub struct MjJointViewMut<'d, 'm: 'd> {
    pub qpos: PointerViewMut<f64>,
    pub qvel: PointerViewMut<f64>,
    pub qacc_warmstart: PointerViewMut<f64>,
    pub qacc: PointerViewMut<f64>,
    pub qfrc_applied: PointerViewMut<f64>,
    pub qfrc_bias: PointerViewMut<f64>,
    phantom: PhantomData<&'d MjData<'m>>
}

impl MjJointViewMut<'_, '_> {
    pub fn reset(&mut self) {
        self.qpos.fill(0.0);
        self.qvel.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.qacc.fill(0.0);
        self.qfrc_applied.fill(0.0);
        self.qfrc_bias.fill(0.0);
    }
}


pub struct MjJointView<'d, 'm: 'd> {
    pub qpos: PointerView<f64>,
    pub qvel: PointerView<f64>,
    pub qacc_warmstart: PointerView<f64>,
    pub qacc: PointerView<f64>,
    pub qfrc_applied: PointerView<f64>,
    pub qfrc_bias: PointerView<f64>,
    phantom: PhantomData<&'d MjData<'m>>
}


#[derive(Debug, PartialEq)]
pub struct MjGeomInfo {
    pub name: String,
    pub id: usize,
    xmat: (usize, usize),
    xpos: (usize, usize)
}

impl MjGeomInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjGeomViewMut<'d, '_> {
        view_creator!(self, MjGeomViewMut, data.ffi_mut(), [xmat, xpos], [], true)
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjGeomView<'d, '_> {
        view_creator!(self, MjGeomView, data.ffi(), [xmat, xpos], [], false)
    }
}

pub struct MjGeomViewMut<'d, 'm: 'd> {
    pub xmat: PointerViewMut<f64>,
    pub xpos: PointerViewMut<f64>,
    phantom: PhantomData<&'d MjData<'m>>
}

pub struct MjGeomView<'d, 'm: 'd> {
    pub xmat: PointerView<f64>,
    pub xpos: PointerView<f64>,
    phantom: PhantomData<&'d MjData<'m>>
}


/// A MjDataViewX which shows a slice of the actuator.
#[derive(Debug, PartialEq)]
pub struct MjActuatorInfo {
    pub name: String,
    pub id: usize,
    ctrl: (usize, usize),
    act: (usize, usize),
}

impl MjActuatorInfo {
    /// Returns a mutable view to the correct fields in [`MjData`].
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjActuatorViewMut<'d, '_> {
        view_creator!(self, MjActuatorViewMut, data.ffi_mut(), [ctrl], [act], true)
    }

    /// Returns a view to the correct fields in [`MjData`].
    pub fn view<'d>(&self, data: &'d MjData) -> MjActuatorView<'d, '_> {
        view_creator!(self, MjActuatorView, data.ffi(), [ctrl], [act], false)
    }
}

pub struct MjActuatorViewMut<'d, 'm: 'd> {
    pub ctrl: PointerViewMut<f64>,
    pub act: Option<PointerViewMut<f64>>,
    phantom: PhantomData<&'d MjData<'m>>
}

pub struct MjActuatorView<'d, 'm: 'd> {
    pub ctrl: PointerView<f64>,
    pub act: Option<PointerView<f64>>,
    phantom: PhantomData<&'d MjData<'m>>
}
