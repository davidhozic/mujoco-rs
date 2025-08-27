use std::marker::PhantomData;
use std::ffi::CString;

use super::mj_auxilary::MjContact;
use super::mj_model::MjModel;
use crate::mujoco_c::*;

use crate::util::{PointerViewMut, PointerView};
use crate::mj_slice_view;


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
    pub(crate) fn new(model: &'a MjModel) -> Self {
        unsafe {
            Self {
                data: mj_makeData(model.ffi()),
                model: model,
            }
        }
    }

    pub(crate) fn ffi(&self) -> &mjData {
        unsafe { self.data.as_ref().unwrap() }
    }

    pub(crate) fn ffi_mut(&self) -> &mut mjData {
        unsafe { self.data.as_mut().unwrap() }
    }

    /// Returns a slice of detected contacts.
    /// # SAFETY
    /// This is NOT THREAD-SAFE, nor lifetime safe.
    /// The returned slice must ne dropped before [`MjData`].
    pub fn contacts(&self) -> &[MjContact] {
        unsafe {
            std::slice::from_raw_parts((*self.data).contact, (*self.data).ncon as usize)
        }
    }

    pub fn joint(&self, name: &str) -> Option<MjJointInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj__mjOBJ_JOINT as i32, CString::new(name).unwrap().as_ptr())};
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

    pub fn geom(&self, name: &str) -> Option<MjGeomInfo> {
        const GEOM_XPOS_LEN: usize = 3;
        const GEOM_XMAT_LEN: usize = 9;

        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj__mjOBJ_GEOM as i32, CString::new(name).unwrap().as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let xpos = (GEOM_XPOS_LEN * id as usize, 3);
        let xmat = (GEOM_XMAT_LEN * id as usize, 9);
        Some(MjGeomInfo { name: name.to_string(), id: id as usize, xmat, xpos })
    }

    pub fn actuator(&self, name: &str) -> Option<MjActuatorInfo> {
        let id = unsafe { mj_name2id(self.model.ffi(), mjtObj__mjOBJ_ACTUATOR as i32, CString::new(name).unwrap().as_ptr())};
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
    /// # SAFETY
    /// This calls the C binding of mj_step, which is not safe by itself.
    pub fn step(&mut self) {
        unsafe {
            mj_step(self.model.ffi(), self.data);
        }
    }

    /// Calculates new dynamics.
    /// # SAFETY
    /// This calls the C binding of mj_step1, which is not safe by itself.
    pub fn step1(&mut self) {
        unsafe {
            mj_step1(self.model.ffi(), self.data);
        }
    }

    /// Calculates the rest after dynamics and integrates in time.
    /// # SAFETY
    /// This calls the C binding of mj_step2, which is not safe by itself.
    pub fn step2(&mut self) {
        unsafe {
            mj_step2(self.model.ffi(), self.data);
        }
    }

    /// Calculates the contact force for the given `contact_id`.
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


/// A MjDataViewX which shows a slice of the joint.
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
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
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjJointViewMut<'d, '_> {
        view_creator!(self, MjJointViewMut, data.ffi_mut(), [qpos, qvel, qacc_warmstart, qacc, qfrc_applied, qfrc_bias], [], true)
    }

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


/// A MjDataViewX which shows a slice of the geom.
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
#[derive(Debug, PartialEq)]
pub struct MjGeomInfo {
    pub name: String,
    pub id: usize,
    xmat: (usize, usize),
    xpos: (usize, usize)
}

impl MjGeomInfo {
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjGeomViewMut<'d, '_> {
        view_creator!(self, MjGeomViewMut, data.ffi_mut(), [xmat, xpos], [], true)
    }

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
/// # SAFETY
/// This is not thread-safe nor lifetime-safe.
/// The view must be dropped before MjData, which is the
/// RESPONSIBILITY OF THE USER.
#[derive(Debug, PartialEq)]
pub struct MjActuatorInfo {
    pub name: String,
    pub id: usize,
    ctrl: (usize, usize),
    act: (usize, usize),
}

impl MjActuatorInfo {
    pub fn view_mut<'d>(&mut self, data: &'d mut MjData) -> MjActuatorViewMut<'d, '_> {
        view_creator!(self, MjActuatorViewMut, data.ffi_mut(), [ctrl], [act], true)
    }

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
