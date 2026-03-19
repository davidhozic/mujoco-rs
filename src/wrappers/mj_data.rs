//! MjData related.
use super::mj_statistic::{MjWarningStat, MjTimerStat, MjSolverStat};
use super::mj_model::{MjModel, MjtSameFrame, MjtObj, MjtStage};
use super::mj_auxiliary::MjContact;
use super::mj_primitive::*;
use crate::{getter_setter, mujoco_c::*};
use crate::error::MjDataError;

use std::ffi::CString;
use std::path::Path;
use std::borrow::Cow;
use std::ops::Deref;
use std::fmt::Debug;
use std::ptr::{self, NonNull};

use crate::{mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem};
use crate::{view_creator, info_method, info_with_view, array_slice_dyn};

/*******************************************/
// Types
/// State component elements as integer bitflags and several convenient combinations of these flags. Used by
/// `mj_getState`, `mj_setState` and `mj_stateSize`.
pub type MjtState = mjtState;

/// Constraint types. These values are not used in mjModel, but are used in the mjData field `d->efc_type` when the list
/// of active constraints is constructed at each simulation time step.
pub type MjtConstraint = mjtConstraint;

/// These values are used by the solver internally to keep track of the constraint states.
pub type MjtConstraintState = mjtConstraintState;

/// Warning types. The number of warning types is given by `mjNWARNING` which is also the length of the array
/// `mjData.warning`.
pub type MjtWarning = mjtWarning;

/// Timer types. The number of timer types is given by `mjNTIMER` which is also the length of the array
/// `mjData.timer`, as well as the length of the string array `mjTIMERSTRING` with timer names.
pub type MjtTimer = mjtTimer;

/// Sleep state of an object.
pub type MjtSleepState = mjtSleepState;
/*******************************************/


/**************************************************************************************************/
// MjData
/**************************************************************************************************/

/// Wrapper around the `mjData` struct.
/// Provides lifetime guarantees as well as automatic cleanup.
#[derive(Debug)]
pub struct MjData<M: Deref<Target = MjModel>> {
    data: NonNull<mjData>,
    model: M
}

// Allow usage in threaded contexts as long as M itself is Send / Sync
// (e.g. Arc<MjModel>). Non-Send M types such as Rc<MjModel> are correctly
// excluded by the M: Send / M: Sync bounds.
unsafe impl<M: Deref<Target = MjModel> + Send> Send for MjData<M> {}
unsafe impl<M: Deref<Target = MjModel> + Sync> Sync for MjData<M> {}


impl<M: Deref<Target = MjModel>> MjData<M> {
    /// Creates a new [`MjData`] linked to `model`.
    ///
    /// # Panics
    /// Panics if MuJoCo fails to allocate the data structure.
    /// Use [`MjData::try_new`] for a fallible alternative.
    pub fn new(model: M) -> Self {
        Self::try_new(model).expect("allocation of MjData failed")
    }

    /// Fallible version of [`MjData::new`].
    ///
    /// # Errors
    /// Returns `Ok(MjData)` on success, or [`MjDataError::AllocationFailed`] if
    /// MuJoCo returns a null pointer from `mj_makeData`.
    ///
    /// Prefer this method over [`MjData::new`] when you want to handle
    /// allocation failures without a panic.
    pub fn try_new(model: M) -> Result<Self, MjDataError> {
        let data_ptr = unsafe { mj_makeData(model.ffi()) };
        NonNull::new(data_ptr)
            .map(|data| Self { data, model })
            .ok_or(MjDataError::AllocationFailed)
    }

    /// Sets a new [`MjModel`] to be used within the instance. This can be used to modify [`MjModel`]'s
    /// parameters without causing size mismatches or violating borrow checker's requirements.
    /// This can be done by keeping a clone of the model, which is then modified and swapped.
    /// 
    /// # Returns
    /// When successful (the signature matches), [`Ok`] containing the old [`MjModel`] is returned.
    /// 
    /// # Errors
    /// [`MjDataError::SignatureMismatch`] is returned when the signature of `model` does
    /// not match the signature of the model this data belongs to.
    /// 
    /// # Notes
    /// This method only validates model-signature compatibility.
    /// **Not all model parameters are safe (for correct simulation) to change at runtime.**
    /// See [here](https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#mjmodel-changes)
    /// to see what parameters can be changed.
    /// 
    /// If model recompilation speed is not an issue,
    /// it is recommended to use [`MjSpec`](crate::wrappers::mj_editing::MjSpec) instead.
    /// 
    /// # Example
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut model_template = Box::new(MjSpec::new().compile().unwrap());
    /// let model_used = model_template.clone();
    /// let mut data = MjData::new(model_used);
    /// 
    /// model_template.opt_mut().timestep = 0.004;
    /// model_template = data.swap_model(model_template).unwrap();
    /// ```
    pub fn swap_model(&mut self, model: M) -> Result<M, MjDataError> {
        let new_signature = model.signature();
        let current_signature = self.model.signature();
        if new_signature != current_signature {
            return Err(MjDataError::SignatureMismatch { source: new_signature, destination: current_signature });
        }

        Ok(std::mem::replace(&mut self.model, model))
    }

    /// Returns a slice of detected contacts.
    /// To obtain the contact force, call [`MjData::contact_force`].
    pub fn contacts(&self) -> &[MjContact] {
        let ffi = self.ffi();
        let ptr = ffi.contact;
        if ptr.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(ptr, ffi.ncon as usize) }
        }
    }

    info_method! { Data, model.ffi(), body, [
        xfrc_applied: 6, xpos: 3, xquat: 4, xmat: 9, xipos: 3, ximat: 9, subtree_com: 3, cinert: 10,
        crb: 10, cvel: 6, subtree_linvel: 3, subtree_angmom: 3, cacc: 6, cfrc_int: 6, cfrc_ext: 6
    ], [], []}
    info_method! { Data, model.ffi(), camera, [xpos: 3, xmat: 9], [], []}
    info_method! { Data, model.ffi(), geom, [xpos: 3, xmat: 9], [], []}
    info_method! { Data, model.ffi(), site, [xpos: 3, xmat: 9], [], []}
    info_method! { Data, model.ffi(), light, [xpos: 3, xdir: 3], [], []}

    info_method! { Data, model.ffi(), actuator,
        [ctrl: 1, length: 1, velocity: 1, force: 1],
        [],
        [act: na]
    }

    /// Obtains a [`MjJointDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjJointDataInfo::view`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn joint(&self, name: &str) -> Option<MjJointDataInfo> {
        let c_name = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.model.ffi(), MjtObj::mjOBJ_JOINT as i32, c_name.as_ptr())};
        if id == -1 {  // not found
            return None;
        }
        let model_ffi = self.model.ffi();
        let id = id as usize;
        unsafe {
            let nq_range = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nq), mj_model_nx_to_nitem!(model_ffi, nq), model_ffi.nq);
            let nv_range = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nv), mj_model_nx_to_nitem!(model_ffi, nv), model_ffi.nv);

            // $id:expr, $addr_map:expr, $njnt:expr, $max_n:expr
            let qpos = nq_range;
            let qvel = nv_range;
            let qacc_warmstart = nv_range;
            let qfrc_applied = nv_range;
            let qacc = nv_range;
            let xanchor = (id * 3, 3);
            let xaxis = (id * 3, 3);
            #[allow(non_snake_case)]
            let qLDiagInv = nv_range;
            let qfrc_bias = nv_range;
            let qfrc_passive = nv_range;
            let qfrc_actuator = nv_range;
            let qfrc_smooth = nv_range;
            let qacc_smooth = nv_range;
            let qfrc_constraint = nv_range;
            let qfrc_inverse = nv_range;

            let qfrc_spring = nv_range;
            let qfrc_damper = nv_range;
            let qfrc_gravcomp = nv_range;
            let qfrc_fluid = nv_range;
            /* Special case attributes, used for some internal calculation */
            // cdof
            // cdof_dot

            let model_signature = self.model.signature();
            Some(MjJointDataInfo {name: name.to_string(), id: id as usize, model_signature,
                qpos, qvel, qacc_warmstart, qfrc_applied, qacc, xanchor, xaxis, qLDiagInv, qfrc_bias,
                qfrc_spring, qfrc_damper, qfrc_gravcomp, qfrc_fluid, qfrc_passive,
                qfrc_actuator, qfrc_smooth, qacc_smooth, qfrc_constraint, qfrc_inverse
            })
        }
    }

    info_method! { Data, model.ffi(), sensor, [], [], [data: nsensordata] }


    info_method! { Data, model.ffi(), tendon,
        [wrapadr: 1, wrapnum: 1, length: 1, velocity: 1],
        [],
        [J: nJten]
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
    pub fn forward_skip(&mut self, skipstage: MjtStage, skipsensor: bool) {
        unsafe {
            mj_forwardSkip(self.model.ffi(), self.ffi_mut(), skipstage as i32, skipsensor as i32);
        }
    }

    /// Inverse dynamics: qacc must be set before calling this function.
    /// This is a wrapper around `mj_inverse`.
    pub fn inverse(&mut self) {
        unsafe {
            mj_inverse(self.model.ffi(), self.ffi_mut());
        }
    }

    /// [`MjData::inverse`] dynamics with skip; skipstage is [`MjtStage`].
    /// This is a wrapper around `mj_inverseSkip`.
    pub fn inverse_skip(&mut self, skipstage: MjtStage, skipsensor: bool) {
        unsafe {
            mj_inverseSkip(self.model.ffi(), self.ffi_mut(), skipstage as i32, skipsensor as i32);
        }
    }

    /// Calculates the contact force for the given `contact_id`.
    /// The `contact_id` matches the index of the contact when iterating
    /// via [`MjData::contacts`].
    /// Calls `mj_contactForce` internally.
    /// # Note
    /// When `contact_id >= ncon`, `[0; 6]` is returned.
    pub fn contact_force(&self, contact_id: u32) -> [MjtNum; 6] {
        let mut force = [0.0; 6];
        unsafe {
            // Safety: contact_id is validated internally.
            // When contact_id is outside valid range of i32,
            // it overflows to value below 0, which is also checked internally.
            mj_contactForce(
                self.model.ffi(), self.data.as_ptr(),
                contact_id as i32, &mut force
            );
        }
        force
    }

    /* Partially auto-generated */

    /// Reset data to defaults.
    pub fn reset(&mut self) {
        unsafe { mj_resetData(self.model.ffi(), self.ffi_mut()) }
    }

    /// Reset data to defaults, fill everything else with debug_value.
    pub fn reset_debug(&mut self, debug_value: u8) {
        unsafe { mj_resetDataDebug(self.model.ffi(), self.ffi_mut(), debug_value) }
    }

    /// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
    pub fn reset_keyframe(&mut self, key: i32) {
        unsafe { mj_resetDataKeyframe(self.model.ffi(), self.ffi_mut(), key) }
    }

    /// Print mjData to text file, specifying format.
    /// float_format must be a valid printf-style format string for a single float value.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjDataError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When either string contains '\0' characters, a panic occurs.
    pub fn print_formatted<T: AsRef<Path>>(&self, filename: T, float_format: &str) -> Result<(), MjDataError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjDataError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        let c_float_format = CString::new(float_format).unwrap();
        unsafe { mj_printFormattedData(self.model.ffi(), self.ffi(), c_filename.as_ptr(), c_float_format.as_ptr()) }
        Ok(())
    }

    /// Print data to text file.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjDataError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn print<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjDataError> {
        let path_str = filename.as_ref().to_str()
            .ok_or(MjDataError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_printData(self.model.ffi(), self.ffi(), c_filename.as_ptr()) }
        Ok(())
    }

    /// Run position-dependent computations.
    pub fn fwd_position(&mut self) {
        unsafe { mj_fwdPosition(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run velocity-dependent computations.
    pub fn fwd_velocity(&mut self) {
        unsafe { mj_fwdVelocity(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute actuator force qfrc_actuator.
    pub fn fwd_actuation(&mut self) {
        unsafe { mj_fwdActuation(self.model.ffi(), self.ffi_mut()) }
    }

    /// Add up all non-constraint forces, compute qacc_smooth.
    pub fn fwd_acceleration(&mut self) {
        unsafe { mj_fwdAcceleration(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run selected constraint solver.
    pub fn fwd_constraint(&mut self) {
        unsafe { mj_fwdConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Euler integrator, semi-implicit in velocity.
    pub fn euler(&mut self) {
        unsafe { mj_Euler(self.model.ffi(), self.ffi_mut()) }
    }

    /// Runge-Kutta explicit order-N integrator.
    pub fn runge_kutta(&mut self, n: i32) {
        unsafe { mj_RungeKutta(self.model.ffi(), self.ffi_mut(), n) }
    }

    /// Implicit-in-velocity integrators.
    pub fn implicit(&mut self) {
        unsafe { mj_implicit(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run position-dependent computations in inverse dynamics.
    pub fn inv_position(&mut self) {
        unsafe { mj_invPosition(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run velocity-dependent computations in inverse dynamics.
    pub fn inv_velocity(&mut self) {
        unsafe { mj_invVelocity(self.model.ffi(), self.ffi_mut()) }
    }

    /// Apply the analytical formula for inverse constraint dynamics.
    pub fn inv_constraint(&mut self) {
        unsafe { mj_invConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compare forward and inverse dynamics, save results in fwdinv.
    pub fn compare_fwd_inv(&mut self) {
        unsafe { mj_compareFwdInv(self.model.ffi(), self.ffi_mut()) }
    }

    /// Evaluate position-dependent sensors.
    pub fn sensor_pos(&mut self) {
        unsafe { mj_sensorPos(self.model.ffi(), self.ffi_mut()) }
    }

    /// Evaluate velocity-dependent sensors.
    pub fn sensor_vel(&mut self) {
        unsafe { mj_sensorVel(self.model.ffi(), self.ffi_mut()) }
    }

    /// Evaluate acceleration and force-dependent sensors.
    pub fn sensor_acc(&mut self) {
        unsafe { mj_sensorAcc(self.model.ffi(), self.ffi_mut()) }
    }

    /// Evaluate position-dependent energy (potential).
    pub fn energy_pos(&mut self) {
        unsafe { mj_energyPos(self.model.ffi(), self.ffi_mut()) }
    }

    /// Evaluate velocity-dependent energy (kinetic).
    pub fn energy_vel(&mut self) {
        unsafe { mj_energyVel(self.model.ffi(), self.ffi_mut()) }
    }

    /// Check qpos, reset if any element is too big or nan.
    pub fn check_pos(&mut self) {
        unsafe { mj_checkPos(self.model.ffi(), self.ffi_mut()) }
    }

    /// Check qvel, reset if any element is too big or nan.
    pub fn check_vel(&mut self) {
        unsafe { mj_checkVel(self.model.ffi(), self.ffi_mut()) }
    }

    /// Check qacc, reset if any element is too big or nan.
    pub fn check_acc(&mut self) {
        unsafe { mj_checkAcc(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run forward kinematics.
    pub fn kinematics(&mut self) {
        unsafe { mj_kinematics(self.model.ffi(), self.ffi_mut()) }
    }

    /// Map inertias and motion dofs to global frame centered at CoM.
    pub fn com_pos(&mut self) {
        unsafe { mj_comPos(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute camera and light positions and orientations.
    pub fn camlight(&mut self) {
        unsafe { mj_camlight(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute flex-related quantities.
    pub fn flex_comp(&mut self) {
        unsafe { mj_flex(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute tendon lengths, velocities and moment arms.
    pub fn tendon_comp(&mut self) {
        unsafe { mj_tendon(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute actuator transmission lengths and moments.
    pub fn transmission(&mut self) {
        unsafe { mj_transmission(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run composite rigid body inertia algorithm (CRB).
    pub fn crb_comp(&mut self) {
        unsafe { mj_crb(self.model.ffi(), self.ffi_mut()) }
    }

    /// Make inertia matrix.
    pub fn make_m(&mut self) {
        unsafe { mj_makeM(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute sparse L'*D*L factorization of inertia matrix.
    pub fn factor_m(&mut self) {
        unsafe { mj_factorM(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute cvel, cdof_dot.
    pub fn com_vel(&mut self) {
        unsafe { mj_comVel(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces.
    pub fn passive(&mut self) {
        unsafe { mj_passive(self.model.ffi(), self.ffi_mut()) }
    }

    /// Sub-tree linear velocity and angular momentum: compute subtree_linvel, subtree_angmom.
    pub fn subtree_vel(&mut self) {
        unsafe { mj_subtreeVel(self.model.ffi(), self.ffi_mut()) }
    }

    /// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=false removes inertial term.
    pub fn rne(&mut self, flg_acc: bool) -> Vec<MjtNum> {
        let mut out = vec![0.0; self.model.ffi().nv as usize];
        unsafe { mj_rne(self.model.ffi(), self.ffi_mut(), flg_acc as i32, out.as_mut_ptr()) };
        out
    }

    /// RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
    pub fn rne_post_constraint(&mut self) {
        unsafe { mj_rnePostConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Run collision detection.
    pub fn collision(&mut self) {
        unsafe { mj_collision(self.model.ffi(), self.ffi_mut()) }
    }

    /// Construct constraints.
    pub fn make_constraint(&mut self) {
        unsafe { mj_makeConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Find constraint islands.
    pub fn island(&mut self) {
        unsafe { mj_island(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute inverse constraint inertia efc_AR.
    pub fn project_constraint(&mut self) {
        unsafe { mj_projectConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute efc_vel, efc_aref.
    pub fn reference_constraint(&mut self) {
        unsafe { mj_referenceConstraint(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
    /// If cost is not `None`, set `*cost = s(jar)` where `jar = Jac*qacc - aref`.
    /// # Errors
    /// Returns [`MjDataError::LengthMismatch`] if the `jar` length is incorrect.
    pub fn constraint_update(&mut self, jar: &[MjtNum], cost: Option<&mut MjtNum>, flg_cone_hessian: bool) -> Result<(), MjDataError> {
        let nefc = self.ffi().nefc as usize;
        if jar.len() < nefc {
            return Err(MjDataError::LengthMismatch { name: "jar", expected: nefc, got: jar.len() });
        }

        unsafe { mj_constraintUpdate(
            self.model.ffi(), self.ffi_mut(),
            jar.as_ptr(), cost.map_or(ptr::null_mut(), |x| x as *mut MjtNum),
            flg_cone_hessian as i32
        ) };

        Ok(())
    }

    /// Initializes the actuator history buffer for actuator `id` (wraps `mj_initCtrlHistory`).
    /// `times`: optional timestamps slice of length `nsample`; `None` keeps existing timestamps.
    /// `values`: control values slice of length `nsample`.
    /// # Errors
    /// - [`MjDataError::IndexOutOfBounds`] if `id >= nu`.
    /// - [`MjDataError::NoHistoryBuffer`] if the actuator has no history buffer.
    /// - [`MjDataError::LengthMismatch`] if `times` or `values` have the wrong length.
    pub fn init_ctrl_history(&mut self, id: usize, times: Option<&[MjtNum]>, values: &[MjtNum]) -> Result<(), MjDataError> {
        let nu = self.model.ffi().nu as usize;
        if id >= nu {
            return Err(MjDataError::IndexOutOfBounds { kind: "actuator_id", id: id as i64, upper: nu as i64 });
        }

        let nsample = self.model.actuator_history()[id][0];
        if nsample <= 0 {
            return Err(MjDataError::NoHistoryBuffer { kind: "actuator", id });
        }

        let ns = nsample as usize;
        if let Some(t) = times
            && t.len() != ns
        {
            return Err(MjDataError::LengthMismatch { name: "times", expected: ns, got: t.len() });
        }
        if values.len() != ns {
            return Err(MjDataError::LengthMismatch { name: "values", expected: ns, got: values.len() });
        }

        unsafe {
            mj_initCtrlHistory(
                self.model.ffi(), self.ffi_mut(), id as i32,
                times.map_or(ptr::null(), |x| x.as_ptr()),
                values.as_ptr()
            );
        }

        Ok(())
    }

    /// Initializes the sensor history buffer for sensor `id` (wraps `mj_initSensorHistory`).
    /// `times`: optional timestamps slice of length `nsample`; `None` keeps existing timestamps.
    /// `values`: sensor values slice of length `nsample * dim`.
    /// `phase`: time phase offset.
    /// # Errors
    /// - [`MjDataError::IndexOutOfBounds`] if `id >= nsensor`.
    /// - [`MjDataError::NoHistoryBuffer`] if the sensor has no history buffer.
    /// - [`MjDataError::LengthMismatch`] if `times` or `values` have the wrong length.
    pub fn init_sensor_history(&mut self, id: usize, times: Option<&[MjtNum]>, values: &[MjtNum], phase: MjtNum) -> Result<(), MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds { kind: "sensor_id", id: id as i64, upper: nsensor as i64 });
        }

        let nsample = self.model.sensor_history()[id][0];
        if nsample <= 0 {
            return Err(MjDataError::NoHistoryBuffer { kind: "sensor", id });
        }

        let dim = self.model.sensor_dim()[id] as usize;
        let required = (nsample as usize) * dim;

        if let Some(t) = times
            && t.len() != nsample as usize
        {
            return Err(MjDataError::LengthMismatch { name: "times", expected: nsample as usize, got: t.len() });
        }
        if values.len() != required {
            return Err(MjDataError::LengthMismatch { name: "values", expected: required, got: values.len() });
        }

        unsafe {
            mj_initSensorHistory(
                self.model.ffi(), self.ffi_mut(), id as i32,
                times.map_or(ptr::null(), |x| x.as_ptr()),
                values.as_ptr(), phase
            );
        }

        Ok(())
    }

    /// Reads the control history value for actuator `id` at `time`
    /// (`interp`: -1=stored, 0=ZOH, 1=linear, 2=cubic).
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nu`.
    pub fn read_ctrl(&self, id: usize, time: MjtNum, interp: i32) -> Result<MjtNum, MjDataError> {
        let nu = self.model.ffi().nu as usize;
        if id >= nu {
            return Err(MjDataError::IndexOutOfBounds { kind: "actuator_id", id: id as i64, upper: nu as i64 });
        }
        let val = unsafe { mj_readCtrl(self.model.ffi(), self.ffi(), id as i32, time, interp) };
        Ok(val)
    }

    /// Reads sensor `id` at `time` into `dst` (`interp`: -1=stored, 0=ZOH, 1=linear, 2=cubic).
    /// `dst` must be exactly `sensor_dim[id]` elements long.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    /// Returns [`MjDataError::LengthMismatch`] when `dst.len() != sensor_dim[id]`.
    pub fn read_sensor_into(&self, id: usize, time: MjtNum, interp: i32, dst: &mut [MjtNum]) -> Result<(), MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds { kind: "sensor_id", id: id as i64, upper: nsensor as i64 });
        }

        let dim = self.model.sensor_dim()[id] as usize;
        if dst.len() != dim {
            return Err(MjDataError::LengthMismatch { name: "dst", expected: dim, got: dst.len() });
        }
        let ptr = unsafe { mj_readSensor(self.model.ffi(), self.ffi(), id as i32, time, dst.as_mut_ptr(), interp) };
        if !ptr.is_null() {
            // C returned a pointer (no interpolation) - copy into dst.
            dst.copy_from_slice(unsafe { std::slice::from_raw_parts(ptr, dim) });
        }
        Ok(())
    }

    /// Reads sensor `id` at `time` into a stack-allocated `[MjtNum; N]`
    /// (`interp`: -1=stored, 0=ZOH, 1=linear, 2=cubic). `N` must match `sensor_dim[id]`.
    /// See also [`read_sensor`](Self::read_sensor), [`read_sensor_into`](Self::read_sensor_into).
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    /// Returns [`MjDataError::LengthMismatch`] when `N != sensor_dim[id]`.
    pub fn read_sensor_fixed<const N: usize>(&self, id: usize, time: MjtNum, interp: i32) -> Result<[MjtNum; N], MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds { kind: "sensor_id", id: id as i64, upper: nsensor as i64 });
        }

        let dim = self.model.sensor_dim()[id] as usize;
        if N != dim {
            return Err(MjDataError::LengthMismatch { name: "N", expected: dim, got: N });
        }
        let mut out = [0.0 as MjtNum; N];
        let ptr = unsafe { mj_readSensor(self.model.ffi(), self.ffi(), id as i32, time, out.as_mut_ptr(), interp) };
        if !ptr.is_null() {
            // C returned a pointer (no interpolation) - copy into out.
            out.copy_from_slice(unsafe { std::slice::from_raw_parts(ptr, N) });
        }
        Ok(out)
    }

    /// Reads sensor `id` at `time` (`interp`: -1=stored, 0=ZOH, 1=linear, 2=cubic).
    ///
    /// Returns [`Cow::Borrowed`] (zero-copy) for exact matches, ZOH, and extrapolation.
    /// Returns [`Cow::Owned`] for linear/cubic interpolation.
    /// See also [`read_sensor_fixed`](Self::read_sensor_fixed), [`read_sensor_into`](Self::read_sensor_into).
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    pub fn read_sensor(&self, id: usize, time: MjtNum, interp: i32) -> Result<Cow<'_, [MjtNum]>, MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds { kind: "sensor_id", id: id as i64, upper: nsensor as i64 });
        }

        let dim = self.model.sensor_dim()[id] as usize;
        let mut out = vec![0.0 as MjtNum; dim];
        let ptr = unsafe { mj_readSensor(self.model.ffi(), self.ffi(), id as i32, time, out.as_mut_ptr(), interp) };
        if !ptr.is_null() {
            // C returned a pointer (no interpolation) - borrow it directly.
            Ok(Cow::Borrowed(unsafe { std::slice::from_raw_parts(ptr, dim) }))
        } else {
            // C wrote result into out.
            Ok(Cow::Owned(out))
        }
    }

    /// Adds a contact to the contact list.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// Returns [`MjDataError::ContactBufferFull`] if the contact buffer is full.
    pub fn add_contact(&mut self, con: &MjContact) -> Result<(), MjDataError> {
        match unsafe { mj_addContact(self.model.ffi(), self.ffi_mut(), con) } {
            0 => Ok(()),
            _ => Err(MjDataError::ContactBufferFull),
        }
    }

    /// Compute 3/6-by-nv end-effector Jacobian of a global point attached to the given body.
    /// Set `jacp` to `true` to calculate the translational Jacobian and `jacr` to `true` for
    /// the rotational Jacobian. Returns a `(Vec, Vec)` for translation and rotation. Empty `Vec`s
    /// indicate that the corresponding Jacobian was not computed.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is negative or `>= nbody`.
    pub fn jac(&self, jacp: bool, jacr: bool, point: &[MjtNum; 3], body_id: i32) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };
        unsafe {
            mj_jac(
                self.model.ffi(), self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                point, body_id,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }

    /// Compute body frame end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn jac_body(&self, jacp: bool, jacr: bool, body_id: i32) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };
        unsafe {
            mj_jacBody(
                self.model.ffi(), self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                body_id,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }

    /// Compute body center-of-mass end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn jac_body_com(&self, jacp: bool, jacr: bool, body_id: i32) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };
        unsafe {
            mj_jacBodyCom(
                self.model.ffi(), self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                body_id,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }

    /// Compute subtree center-of-mass end-effector Jacobian (translational only).
    /// Returns a `Vec` of length `3 * nv` (row-major 3×nv matrix).
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn jac_subtree_com(&mut self, body_id: i32) -> Result<Vec<MjtNum>, MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = vec![0 as MjtNum; required_len];
        unsafe {
            mj_jacSubtreeCom(
                self.model.ffi(), self.ffi_mut(),
                jacp_vec.as_mut_ptr(),
                body_id,
            )
        };
        Ok(jacp_vec)
    }

    /// Compute geom end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `geom_id` is out of range.
    pub fn jac_geom(&self, jacp: bool, jacr: bool, geom_id: i32) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let ngeom = self.model.ffi().ngeom;
        if geom_id < 0 || (geom_id as i64) >= ngeom {
            return Err(MjDataError::IndexOutOfBounds { kind: "geom_id", id: geom_id as i64, upper: ngeom });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };
        unsafe {
            mj_jacGeom(
                self.model.ffi(), self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                geom_id,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }

    /// Compute site end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `site_id` is out of range.
    pub fn jac_site(&self, jacp: bool, jacr: bool, site_id: i32) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nsite = self.model.ffi().nsite;
        if site_id < 0 || (site_id as i64) >= nsite {
            return Err(MjDataError::IndexOutOfBounds { kind: "site_id", id: site_id as i64, upper: nsite });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };
        unsafe {
            mj_jacSite(
                self.model.ffi(), self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                site_id,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }

    /// Compute subtree angular momentum matrix.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn angmom_mat(&mut self, body_id: i32) -> Result<Vec<MjtNum>, MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let mut mat = vec![0.0; 3 * self.model.ffi().nv as usize];
        unsafe { mj_angmomMat(self.model.ffi(), self.ffi_mut(), mat.as_mut_ptr(), body_id) };
        Ok(mat)
    }

    /// Run all kinematics-like computations (kinematics, comPos, camlight, flex, tendon).
    pub fn forward_kinematics(&mut self) {
        unsafe { mj_fwdKinematics(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
    /// # Errors
    /// Returns:
    /// - [`MjDataError::UnsupportedObjectType`] when `obj_type` is not one of
    ///   `mjOBJ_BODY`, `mjOBJ_XBODY`, `mjOBJ_GEOM`, `mjOBJ_SITE`, `mjOBJ_CAMERA`.
    /// - [`MjDataError::IndexOutOfBounds`] when `obj_id` is out of range for the given type.
    pub fn object_velocity(&self, obj_type: MjtObj, obj_id: i32, flg_local: bool) -> Result<[MjtNum; 6], MjDataError> {
        let max_id = match obj_type {
            MjtObj::mjOBJ_BODY | MjtObj::mjOBJ_XBODY => self.model.ffi().nbody,
            MjtObj::mjOBJ_GEOM => self.model.ffi().ngeom,
            MjtObj::mjOBJ_SITE => self.model.ffi().nsite,
            MjtObj::mjOBJ_CAMERA => self.model.ffi().ncam,
            _ => return Err(MjDataError::UnsupportedObjectType(obj_type as i32)),
        };
        if obj_id < 0 || (obj_id as i64) >= max_id {
            return Err(MjDataError::IndexOutOfBounds { kind: "obj_id", id: obj_id as i64, upper: max_id });
        }
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe {
            mj_objectVelocity(self.model.ffi(), self.ffi(), obj_type as i32, obj_id, &mut result, flg_local as i32)
        };
        Ok(result)
    }

    /// Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.
    /// # Errors
    /// Returns:
    /// - [`MjDataError::UnsupportedObjectType`] when `obj_type` is not supported.
    /// - [`MjDataError::IndexOutOfBounds`] when `obj_id` is out of range for the given type.
    pub fn object_acceleration(&self, obj_type: MjtObj, obj_id: i32, flg_local: bool) -> Result<[MjtNum; 6], MjDataError> {
        let max_id = match obj_type {
            MjtObj::mjOBJ_BODY | MjtObj::mjOBJ_XBODY => self.model.ffi().nbody,
            MjtObj::mjOBJ_GEOM => self.model.ffi().ngeom,
            MjtObj::mjOBJ_SITE => self.model.ffi().nsite,
            MjtObj::mjOBJ_CAMERA => self.model.ffi().ncam,
            _ => return Err(MjDataError::UnsupportedObjectType(obj_type as i32)),
        };
        if obj_id < 0 || (obj_id as i64) >= max_id {
            return Err(MjDataError::IndexOutOfBounds { kind: "obj_id", id: obj_id as i64, upper: max_id });
        }
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe {
            mj_objectAcceleration(self.model.ffi(), self.ffi(), obj_type as i32, obj_id, &mut result, flg_local as i32)
        };
        Ok(result)
    }

    /// Returns smallest signed distance between two geoms and optionally the contact segment.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when either geom id is negative or `>= ngeom`.
    pub fn geom_distance(&self, geom1_id: i32, geom2_id: i32, dist_max: MjtNum, fromto: Option<&mut [MjtNum; 6]>) -> Result<MjtNum, MjDataError> {
        let ngeom = self.model.ffi().ngeom;
        if geom1_id < 0 || (geom1_id as i64) >= ngeom {
            return Err(MjDataError::IndexOutOfBounds { kind: "geom1_id", id: geom1_id as i64, upper: ngeom });
        }
        if geom2_id < 0 || (geom2_id as i64) >= ngeom {
            return Err(MjDataError::IndexOutOfBounds { kind: "geom2_id", id: geom2_id as i64, upper: ngeom });
        }
        Ok(unsafe {
            mj_geomDistance(
                self.model.ffi(), self.ffi(),
                geom1_id, geom2_id, dist_max,
                fromto.map_or(ptr::null_mut(), |x| x),
            )
        })
    }

    /// Map from body local to global Cartesian coordinates. Returns (global position, global orientation matrix).
    /// `sameframe` takes values from [`MjtSameFrame`]. Wraps `mj_local2Global`.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn local_to_global(&mut self, pos: &[MjtNum; 3], quat: &[MjtNum; 4], body_id: i32, sameframe: MjtSameFrame) -> Result<([MjtNum; 3], [MjtNum; 9]), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id < 0 || (body_id as i64) >= nbody {
            return Err(MjDataError::IndexOutOfBounds { kind: "body_id", id: body_id as i64, upper: nbody });
        }
        let mut xpos: [MjtNum; 3] = [0.0; 3];
        let mut xmat: [MjtNum; 9] = [0.0; 9];
        unsafe {
            mj_local2Global(self.ffi_mut(), &mut xpos, &mut xmat, pos, quat, body_id, sameframe as MjtByte)
        };
        Ok((xpos, xmat))
    }

    /// Intersect multiple rays emanating from a single point.
    /// Similar semantics to mj_ray, but `vec` is an array of (nray x 3) directions.
    /// If `normals_out` is `Some`, it must be a slice of `nray` elements filled with surface normals. Use `None` to skip normals.
    /// # Errors
    /// Returns [`MjDataError::LengthMismatch`] if `normals_out` length does not match `vec.len()`.
    #[allow(clippy::too_many_arguments)]
    pub fn multi_ray(
        &mut self, pnt: &[MjtNum; 3], vec: &[[MjtNum; 3]], geomgroup: Option<&[MjtByte; mjNGROUP as usize]>,
        flg_static: bool, bodyexclude: i32, cutoff: MjtNum, normals_out: Option<&mut [[MjtNum; 3]]>
    ) -> Result<(Vec<i32>, Vec<MjtNum>), MjDataError> {
        let nray = vec.len();
        if let Some(buf) = &normals_out
            && buf.len() != nray
        {
            return Err(MjDataError::LengthMismatch { name: "normals_out", expected: nray, got: buf.len() });
        }

        let mut geom_id = vec![0; nray];
        let mut distance = vec![0.0; nray];

        unsafe { mj_multiRay(
            self.model.ffi(), self.ffi_mut(), pnt,
            bytemuck::cast_slice::<[MjtNum; 3], MjtNum>(vec).as_ptr(),
            geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
            flg_static as u8, bodyexclude, geom_id.as_mut_ptr(),
            distance.as_mut_ptr(),
            normals_out.map_or(ptr::null_mut(), |x| bytemuck::cast_slice_mut::<[MjtNum; 3], MjtNum>(x).as_mut_ptr()),
            nray as i32, cutoff
        ) };

        Ok((geom_id, distance))
    }

    /// Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
    /// Returns `(geomid, distance)` where distance is -1 if no intersection.
    /// If `normal_out` is `Some`, it will be filled with the surface normal at the intersection.
    /// `geomgroup` and `flg_static` are as in mjvOption; pass `None` for `geomgroup` to skip group exclusion.
    pub fn ray(
        &self, pnt: &[MjtNum; 3], vec: &[MjtNum; 3],
        geomgroup: Option<&[MjtByte; mjNGROUP as usize]>, flg_static: bool, bodyexclude: i32,
        normal_out: Option<&mut [MjtNum; 3]>
    ) -> (i32, MjtNum) {
        // `normal_out` is a fixed-size array; nothing to validate at runtime here.
        let mut geom_id = -1;
        let dist = unsafe { mj_ray(
            self.model.ffi(), self.ffi(),
            pnt, vec,
            geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
            flg_static as MjtByte, bodyexclude, &mut geom_id,
            normal_out.map_or(ptr::null_mut(), |x| x)
        ) };
        (geom_id, dist)
    }

    /// Intersect ray with visible flexes.
    /// Return distance to nearest surface, or -1 if no intersection.
    /// If `vertid` is `Some`, it will be filled with the id of the nearest vertex.
    /// If `normal_out` is `Some`, it will be filled with the surface normal at the intersection.
    /// `flex_layer`, `flg_vert`, `flg_edge`, `flg_face`, `flg_skin` and `flexid` control what and where to intersect.
    ///
    /// # Panics
    /// Panics if `flexid` is out of bounds (must be `0 <= flexid < nflex`).
    #[allow(clippy::too_many_arguments)]
    pub fn ray_flex(
        &self, flex_layer: i32, flg_vert: bool, flg_edge: bool, flg_face: bool, flg_skin: bool, flexid: i32,
        pnt: &[MjtNum; 3], vec: &[MjtNum; 3],
        vertid: Option<&mut i32>, normal_out: Option<&mut [MjtNum; 3]>
    ) -> mjtNum {
        let nflex = self.model.ffi().nflex as i32;
        assert!(flexid >= 0 && flexid < nflex, "ray_flex: flexid {flexid} out of bounds (nflex = {nflex})");
        unsafe { mj_rayFlex(
            self.model.ffi(), self.ffi(),
            flex_layer, flg_vert as MjtByte, flg_edge as MjtByte, flg_face as MjtByte, flg_skin as MjtByte, flexid,
            pnt, vec,
            vertid.map_or(ptr::null_mut(), |x| x), normal_out.map_or(ptr::null_mut(), |x| x)
        ) }
    }

    /// Copies data state from `src` to `self` based on the specified `spec` combination of `mjtState` flags.
    ///
    /// # Panics
    /// Panics if `src` was created from a different model.
    /// Use [`MjData::try_copy_state_from_data`] for a fallible alternative.
    pub fn copy_state_from_data(&mut self, src: &MjData<M>, spec: u32) {
        self.try_copy_state_from_data(src, spec)
            .expect("copy_state_from_data failed")
    }

    /// Fallible version of [`MjData::copy_state_from_data`].
    ///
    /// # Errors
    /// Returns [`MjDataError::SignatureMismatch`] if `src` was created from
    /// a different model.
    pub fn try_copy_state_from_data(&mut self, src: &MjData<M>, spec: u32) -> Result<(), MjDataError> {
        let src_sig = src.model.signature();
        let dst_sig = self.model.signature();
        if src_sig != dst_sig {
            return Err(MjDataError::SignatureMismatch {
                source: src_sig,
                destination: dst_sig,
            });
        }
        unsafe {
            mj_copyState(self.model.ffi(), src.ffi(), self.ffi_mut(), spec as i32);
        }
        Ok(())
    }

    /// Intersect ray with hfield.
    /// Returns the distance to the intersection, or -1.0 if no intersection.
    ///
    /// # Panics
    /// Panics if `geom_id` is out of bounds (must be `0 <= geom_id < ngeom`).
    pub fn ray_hfield(
        &self, geom_id: i32, pnt: &[MjtNum; 3], vec: &[MjtNum; 3], normal_out: Option<&mut [MjtNum; 3]>
    ) -> mjtNum {
        let ngeom = self.model.ffi().ngeom as i32;
        assert!(geom_id >= 0 && geom_id < ngeom, "ray_hfield: geom_id {geom_id} out of bounds (ngeom = {ngeom})");
        unsafe {
            mj_rayHfield(self.model.ffi(), self.ffi(), geom_id, pnt, vec, normal_out.map_or(ptr::null_mut(), |x| x))
        }
    }

    /// Intersect ray with mesh.
    /// Returns the distance to the intersection, or -1.0 if no intersection.
    ///
    /// # Panics
    /// Panics if `geom_id` is out of bounds (must be `0 <= geom_id < ngeom`).
    pub fn ray_mesh(
        &self, geom_id: i32, pnt: &[MjtNum; 3], vec: &[MjtNum; 3], normal_out: Option<&mut [MjtNum; 3]>
    ) -> mjtNum {
        let ngeom = self.model.ffi().ngeom as i32;
        assert!(geom_id >= 0 && geom_id < ngeom, "ray_mesh: geom_id {geom_id} out of bounds (ngeom = {ngeom})");
        unsafe {
            mj_rayMesh(self.model.ffi(), self.ffi(), geom_id, pnt, vec, normal_out.map_or(ptr::null_mut(), |x| x))
        }
    }

    /// Apply Cartesian force and torque to a point on a body, and add the result to `qfrc_target`.
    ///
    /// # Panics
    /// Panics if `body` is not a valid body index or `qfrc_target` length is less than `nv`.
    /// Use [`MjData::try_apply_ft`] for a fallible alternative.
    pub fn apply_ft(&mut self, force: &[MjtNum; 3], torque: &[MjtNum; 3], point: &[MjtNum; 3], body: i32, qfrc_target: &mut [MjtNum]) {
        self.try_apply_ft(force, torque, point, body, qfrc_target)
            .expect("apply_ft failed")
    }

    /// Fallible version of [`MjData::apply_ft`].
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `body` is not a valid body
    /// index, or [`MjDataError::BufferTooSmall`] if `qfrc_target` is shorter
    /// than `nv`.
    pub fn try_apply_ft(
        &mut self,
        force: &[MjtNum; 3],
        torque: &[MjtNum; 3],
        point: &[MjtNum; 3],
        body: i32,
        qfrc_target: &mut [MjtNum],
    ) -> Result<(), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body < 0 || body as i64 >= nbody {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body",
                id: body as i64,
                upper: nbody,
            });
        }
        let nv = self.model.ffi().nv as usize;
        if qfrc_target.len() < nv {
            return Err(MjDataError::BufferTooSmall {
                name: "qfrc_target",
                got: qfrc_target.len(),
                needed: nv,
            });
        }
        unsafe {
            mj_applyFT(self.model.ffi(), self.ffi_mut(), force, torque, point, body, qfrc_target.as_mut_ptr());
        }
        Ok(())
    }

    /// Reads data's state into `destination`. The `spec` parameter is a bit mask of [`MjtState`] elements,
    /// which controls what state gets copied. The `destination` parameter is a mutable
    /// slice to the location into which the state will be written.
    /// This is a wrapper around [`mj_getState`].
    ///
    /// # Note
    /// The `destination` buffer is allowed to be larger than the
    /// actual state length, and may thus contain old information.
    /// Only the first `state_size` elements of `destination` are updated by this function;
    /// any remaining elements in the buffer are left unchanged. This was done for possible
    /// performance improvements, where one array may hold different parts of simulation state
    /// at different times.
    ///
    /// You can use the returned number of [`MjtNum`] elements written to `destination`
    /// to create a subslice containing only the updated information.
    ///
    /// # Returns
    /// Number of [`MjtNum`] elements written to `destination`.
    ///
    /// # Panics
    /// A panic will occur if `destination` is smaller than [`MjModel::state_size`] with `spec` passed as parameter.
    /// Use [`MjData::try_read_state_into`] for a fallible alternative.
    pub fn read_state_into(&self, spec: u32, destination: &mut [MjtNum]) -> usize {
        self.try_read_state_into(spec, destination)
            .expect("read_state_into failed")
    }

    /// Fallible version of [`MjData::read_state_into`].
    ///
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `destination` is smaller than
    /// the state size required by `spec`.
    ///
    /// On success, returns the number of [`MjtNum`] elements written.
    pub fn try_read_state_into(&self, spec: u32, destination: &mut [MjtNum]) -> Result<usize, MjDataError> {
        let state_size = self.model.state_size(spec);
        if destination.len() < state_size {
            return Err(MjDataError::BufferTooSmall {
                name: "destination",
                got: destination.len(),
                needed: state_size,
            });
        }
        unsafe {
            mj_getState(self.model.ffi(), self.ffi(), destination.as_mut_ptr(), spec as i32);
        }
        Ok(state_size)
    }

    /// Same as [`MjData::read_state_into`], except it allocates
    /// and returns new boxed data containing the state.
    pub fn get_state(&self, spec: u32) -> Box<[MjtNum]> {
        let mut destination = vec![0.0; self.model.state_size(spec)].into_boxed_slice();
        Self::read_state_into(self, spec, &mut destination);
        destination
    }

    /// Sets the `state` to [`MjData`]. This is a wrapper around [`mj_setState`].
    /// The `state` is an array containing the state to write, based on the `spec`
    /// bitmask of elements [`MjtState`].
    ///
    /// # Note
    /// The size of `state` is allowed to be larger. This was done to allow a preallocated
    /// buffer to store any possible state based on `spec`, without having to query the size
    /// every time. This benefits performance in some cases.
    ///
    /// # Panics
    /// A panic will occur if `state`'s length is less than would be copied
    /// based on `spec`.
    /// Use [`MjData::try_set_state`] for a fallible alternative.
    ///
    /// # Safety
    /// When `spec` includes [`MjtState::mjSTATE_EQ_ACTIVE`], MuJoCo writes the raw
    /// `mjtNum` (f64) bytes from `state` directly into the `eq_active` byte array
    /// without booleanization. The caller must ensure that [`MjData::eq_active`] is
    /// not accessed until the `eq_active` values have been re-validated as `0` or `1`
    /// (e.g. by calling `mj_forward` or `mj_step`).
    pub unsafe fn set_state(&mut self, state: &[MjtNum], spec: u32) {
        unsafe { self.try_set_state(state, spec) }
            .expect("set_state failed")
    }

    /// Fallible version of [`MjData::set_state`].
    ///
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `state` is smaller than the
    /// length required by `spec`.
    ///
    /// # Safety
    /// When `spec` includes [`MjtState::mjSTATE_EQ_ACTIVE`], MuJoCo writes the raw
    /// `mjtNum` (f64) bytes from `state` directly into the `eq_active` byte array
    /// without booleanization. The caller must ensure that [`MjData::eq_active`] is
    /// not accessed until the `eq_active` values have been re-validated as `0` or `1`
    /// (e.g. by calling `mj_forward` or `mj_step`).
    pub unsafe fn try_set_state(&mut self, state: &[MjtNum], spec: u32) -> Result<(), MjDataError> {
        let required_len = self.model.state_size(spec);
        if state.len() < required_len {
            return Err(MjDataError::BufferTooSmall {
                name: "state",
                got: state.len(),
                needed: required_len,
            });
        }
        unsafe {
            mj_setState(self.model.ffi(), self.ffi_mut(), state.as_ptr(), spec as i32);
        }
        Ok(())
    }


    /// Copy [`MjData`] to `destination`, skipping large computed arrays not required for
    /// visualization: mass matrices and constraint arrays (`efc_*`, `iefc_*`, including
    /// constraint Jacobians).
    /// This is a wrapper for [`mjv_copyData`].
    ///
    /// # Panics
    /// Panics if `destination` was created from a different model.
    /// Use [`MjData::try_copy_visual_to`] for a fallible alternative.
    pub fn copy_visual_to<N: Deref<Target = MjModel>>(&self, destination: &mut MjData<N>) {
        self.try_copy_visual_to(destination)
            .expect("copy_visual_to failed")
    }

    /// Fallible version of [`MjData::copy_visual_to`].
    ///
    /// # Errors
    /// Returns [`MjDataError::SignatureMismatch`] if `destination` was created
    /// from a different model.
    pub fn try_copy_visual_to<N: Deref<Target = MjModel>>(&self, destination: &mut MjData<N>) -> Result<(), MjDataError> {
        let src_sig = self.model.signature();
        let dst_sig = destination.model.signature();
        if src_sig != dst_sig {
            return Err(MjDataError::SignatureMismatch {
                source: src_sig,
                destination: dst_sig,
            });
        }
        unsafe {
            mjv_copyData(destination.ffi_mut(), self.model.ffi(), self.ffi());
        }
        Ok(())
    }

    /// Copy [`MjData`] to `destination` in full.
    /// This is a wrapper for [`mj_copyData`].
    ///
    /// # Panics
    /// Panics if `destination` was created from a different model.
    /// Use [`MjData::try_copy_to`] for a fallible alternative.
    pub fn copy_to<N: Deref<Target = MjModel>>(&self, destination: &mut MjData<N>) {
        self.try_copy_to(destination)
            .expect("copy_to failed")
    }

    /// Fallible version of [`MjData::copy_to`].
    ///
    /// # Errors
    /// Returns [`MjDataError::SignatureMismatch`] if `destination` was created
    /// from a different model.
    pub fn try_copy_to<N: Deref<Target = MjModel>>(&self, destination: &mut MjData<N>) -> Result<(), MjDataError> {
        let src_sig = self.model.signature();
        let dst_sig = destination.model.signature();
        if src_sig != dst_sig {
            return Err(MjDataError::SignatureMismatch {
                source: src_sig,
                destination: dst_sig,
            });
        }
        unsafe {
            mj_copyData(destination.ffi_mut(), self.model.ffi(), self.ffi());
        }
        Ok(())
    }

    /// Returns a direct pointer to the underlying data.
    /// Returns a direct mutable pointer to the underlying C data struct.
    /// Only for internal use by viewer code that passes the pointer to C++ FFI.
    #[cfg(feature = "cpp-viewer")]
    pub(crate) fn as_raw_ptr(&self) -> *mut mjData {
        self.data.as_ptr()
    }
}


/// Some public attribute methods.
impl<M: Deref<Target = MjModel>> MjData<M> {
    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjData {
        unsafe { self.data.as_ref() }
    }

    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjData {
        unsafe { self.data.as_mut() }
    }

    /// Returns a reference to data's [`MjModel`].
    pub fn model(&self) -> &MjModel {
        &self.model
    }

    /// Returns a clone of the stored model.
    /// Unlike [`model`](Self::model), this returns
    /// the infered `M` type (cloned).
    pub fn model_clone(&self) -> M where M: Clone {
        self.model.clone()
    }

    getter_setter! {get, [
        [ffi] narena: MjtSize; "size of the arena in bytes (inclusive of the stack).";
        [ffi] nbuffer: MjtSize; "size of main buffer in bytes.";
        [ffi] nplugin: i32; "number of plugin instances.";
        [ffi] maxuse_stack: MjtSize; "maximum stack allocation in bytes (mutable).";
        [ffi] maxuse_arena: MjtSize; "maximum arena allocation in bytes.";
        [ffi] maxuse_con: i32; "maximum number of contacts.";
        [ffi] maxuse_efc: i32; "maximum number of scalar constraints.";
        [ffi] ncon: i32; "number of detected contacts.";
        [ffi] ne: i32; "number of equality constraints.";
        [ffi] nf: i32; "number of friction constraints.";
        [ffi] nl: i32; "number of limit constraints.";
        [ffi] nefc: i32; "number of constraints.";
        [ffi] nJ: i32; "number of non-zeros in constraint Jacobian.";
        [ffi] nA: i32; "number of non-zeros in constraint inverse inertia matrix.";
        [ffi] nisland: i32; "number of detected constraint islands.";
        [ffi] nidof: i32; "number of dofs in all islands.";
        [ffi] ntree_awake: i32; "number of awake trees.";
        [ffi] nbody_awake: i32; "number of awake dynamic and static bodies.";
        [ffi] nparent_awake: i32; "number of bodies with awake parents.";
        [ffi] nv_awake: i32; "number of awake dofs.";
        [ffi] signature: u64; "compilation signature.";
    ]}

    getter_setter! {get, [
        [ffi] flg_energypos: bool; "has mj_energyPos been called.";
        [ffi] flg_energyvel: bool; "has mj_energyVel been called.";
        [ffi] flg_subtreevel: bool; "has mj_subtreeVel been called.";
        [ffi] flg_rnepost: bool; "has mj_rnePostConstraint been called.";
    ]}

    getter_setter! {with, get, set, [[ffi, ffi_mut] time: MjtNum; "simulation time.";]}

    getter_setter! {with, get, [
        [ffi, ffi_mut] energy: &[MjtNum; 2]; "potential, kinetic energy.";
    ]}

    getter_setter! {
        get, [
            [ffi] (allow_mut = false) maxuse_threadstack: &[MjtSize; mjMAXTHREAD as usize]; "maximum stack allocation per thread in bytes.";
            [ffi, ffi_mut] solver: &[MjSolverStat; mjNISLAND as usize * mjNSOLVER as usize]; "solver statistics per island, per iteration.";
            [ffi, ffi_mut] solver_niter: &[i32; mjNISLAND as usize]; "number of solver iterations, per island.";
            [ffi, ffi_mut] solver_nnz: &[i32; mjNISLAND as usize]; "number of nonzeros in Hessian or efc_AR, per island.";
            [ffi, ffi_mut] solver_fwdinv: &[MjtNum; 2]; "forward-inverse comparison: qfrc, efc.";
            [ffi, ffi_mut] warning: &[MjWarningStat; MjtWarning::mjNWARNING as usize]; "warning statistics (mutable).";
            [ffi, ffi_mut] timer: &[MjTimerStat; MjtTimer::mjNTIMER as usize]; "timer statistics.";
        ]
    }
}

/// Arrays of dynamic size.
impl<M: Deref<Target = MjModel>> MjData<M> {
    array_slice_dyn! {
        qpos: &[MjtNum; "position"; model.ffi().nq],
        qvel: &[MjtNum; "velocity"; model.ffi().nv],
        act: &[MjtNum; "actuator activation"; model.ffi().na],
        history: &[MjtNum; "history buffer"; model.ffi().nhistory],
        qacc_warmstart: &[MjtNum; "acceleration used for warmstart"; model.ffi().nv],
        plugin_state: &[MjtNum; "plugin state"; model.ffi().npluginstate],
        ctrl: &[MjtNum; "control"; model.ffi().nu],
        qfrc_applied: &[MjtNum; "applied generalized force"; model.ffi().nv],
        xfrc_applied: &[[MjtNum; 6] [force(MjtNum)]; "applied Cartesian force/torque"; model.ffi().nbody],
        eq_active: &[bool [force(u8)]; "enable/disable constraints"; model.ffi().neq],
        mocap_pos: &[[MjtNum; 3] [force(MjtNum)]; "positions of mocap bodies"; model.ffi().nmocap],
        mocap_quat: &[[MjtNum; 4] [force(MjtNum)]; "orientations of mocap bodies"; model.ffi().nmocap],
        qacc: &[MjtNum; "acceleration"; model.ffi().nv],
        act_dot: &[MjtNum; "time-derivative of actuator activation"; model.ffi().na],
        userdata: &[MjtNum; "user data, not touched by engine"; model.ffi().nuserdata],
        sensordata: &[MjtNum; "sensor data array"; model.ffi().nsensordata],
        (unsafe) tree_asleep: &[i32; "<0: awake; >=0: index cycle of sleeping trees"; model.ffi().ntree],
        xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian position of body frame"; model.ffi().nbody],
        xquat: &[[MjtNum; 4] [force(MjtNum)]; "Cartesian orientation of body frame"; model.ffi().nbody],
        xmat: &[[MjtNum; 9] [force(MjtNum)]; "Cartesian orientation of body frame"; model.ffi().nbody],
        xipos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian position of body com"; model.ffi().nbody],
        ximat: &[[MjtNum; 9] [force(MjtNum)]; "Cartesian orientation of body inertia"; model.ffi().nbody],
        xanchor: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian position of joint anchor"; model.ffi().njnt],
        xaxis: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian joint axis"; model.ffi().njnt],
        geom_xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian geom position"; model.ffi().ngeom],
        geom_xmat: &[[MjtNum; 9] [force(MjtNum)]; "Cartesian geom orientation"; model.ffi().ngeom],
        site_xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian site position"; model.ffi().nsite],
        site_xmat: &[[MjtNum; 9] [force(MjtNum)]; "Cartesian site orientation"; model.ffi().nsite],
        cam_xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian camera position"; model.ffi().ncam],
        cam_xmat: &[[MjtNum; 9] [force(MjtNum)]; "Cartesian camera orientation"; model.ffi().ncam],
        light_xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian light position"; model.ffi().nlight],
        light_xdir: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian light direction"; model.ffi().nlight],
        subtree_com: &[[MjtNum; 3] [force(MjtNum)]; "center of mass of each subtree"; model.ffi().nbody],
        cdof: &[[MjtNum; 6] [force(MjtNum)]; "com-based motion axis of each dof (rot:lin)"; model.ffi().nv],
        cinert: &[[MjtNum; 10] [force(MjtNum)]; "com-based body inertia and mass"; model.ffi().nbody],
        flexvert_xpos: &[[MjtNum; 3] [force(MjtNum)]; "Cartesian flex vertex positions"; model.ffi().nflexvert],
        flexelem_aabb: &[[MjtNum; 6] [force(MjtNum)]; "flex element bounding boxes (center, size)"; model.ffi().nflexelem],
        flexedge_J: &[MjtNum; "flex edge Jacobian"; model.ffi().nJfe],
        flexedge_length: &[MjtNum; "flex edge lengths"; model.ffi().nflexedge],
        flexvert_J: &[[MjtNum; 2] [force(MjtNum)]; "flex vertex Jacobian"; model.ffi().nJfv],
        flexvert_length: &[[MjtNum; 2] [force(MjtNum)]; "flex vertex lengths"; model.ffi().nflexvert],
        bvh_aabb_dyn: &[[MjtNum; 6] [force(MjtNum)]; "global bounding box (center, size)"; model.ffi().nbvhdynamic],
        (unsafe) ten_wrapadr: &[i32; "start address of tendon's path"; model.ffi().ntendon],
        (unsafe) ten_wrapnum: &[i32; "number of wrap points in path"; model.ffi().ntendon],
        ten_J: &[MjtNum; "tendon Jacobian"; model.ffi().nJten],
        ten_length: &[MjtNum; "tendon lengths"; model.ffi().ntendon],
        (unsafe) wrap_obj: &[[i32; 2] [force(i32)]; "geom id; -1: site; -2: pulley"; model.ffi().nwrap],
        wrap_xpos: &[[MjtNum; 6] [force(MjtNum)]; "Cartesian 3D points in all paths"; model.ffi().nwrap],
        actuator_length: &[MjtNum; "actuator lengths"; model.ffi().nu],
        (unsafe) moment_rownnz: &[i32; "number of non-zeros in actuator_moment row"; model.ffi().nu],
        (unsafe) moment_rowadr: &[i32; "row start address in colind array"; model.ffi().nu],
        (unsafe) moment_colind: &[i32; "column indices in sparse Jacobian"; model.ffi().nJmom],
        actuator_moment: &[MjtNum; "actuator moments"; model.ffi().nJmom],
        crb: &[[MjtNum; 10] [force(MjtNum)]; "com-based composite inertia and mass"; model.ffi().nbody],
        qM: &[MjtNum; "inertia (sparse)"; model.ffi().nM],
        M: &[MjtNum; "reduced inertia (compressed sparse row)"; model.ffi().nC],
        qLD: &[MjtNum; "L'*D*L factorization of M (sparse)"; model.ffi().nC],
        qLDiagInv: &[MjtNum; "1/diag(D)"; model.ffi().nv],
        bvh_active: &[bool [force(u8)]; "was bounding volume checked for collision"; model.ffi().nbvh],
        tree_awake: &[i32; "is tree awake; 0: asleep; 1: awake"; model.ffi().ntree],
        body_awake: &[MjtSleepState [force(i32)]; "body sleep state"; model.ffi().nbody],
        (unsafe) body_awake_ind: &[i32; "indices of awake and static bodies"; model.ffi().nbody],
        (unsafe) parent_awake_ind: &[i32; "indices of bodies with awake or static parents"; model.ffi().nbody],
        (unsafe) dof_awake_ind: &[i32; "indices of awake dofs"; model.ffi().nv],
        flexedge_velocity: &[MjtNum; "flex edge velocities"; model.ffi().nflexedge],
        ten_velocity: &[MjtNum; "tendon velocities"; model.ffi().ntendon],
        actuator_velocity: &[MjtNum; "actuator velocities"; model.ffi().nu],
        cvel: &[[MjtNum; 6] [force(MjtNum)]; "com-based velocity (rot:lin)"; model.ffi().nbody],
        cdof_dot: &[[MjtNum; 6] [force(MjtNum)]; "time-derivative of cdof (rot:lin)"; model.ffi().nv],
        qfrc_bias: &[MjtNum; "C(qpos,qvel)"; model.ffi().nv],
        qfrc_spring: &[MjtNum; "passive spring force"; model.ffi().nv],
        qfrc_damper: &[MjtNum; "passive damper force"; model.ffi().nv],
        qfrc_gravcomp: &[MjtNum; "passive gravity compensation force"; model.ffi().nv],
        qfrc_fluid: &[MjtNum; "passive fluid force"; model.ffi().nv],
        qfrc_passive: &[MjtNum; "total passive force"; model.ffi().nv],
        subtree_linvel: &[[MjtNum; 3] [force(MjtNum)]; "linear velocity of subtree com"; model.ffi().nbody],
        subtree_angmom: &[[MjtNum; 3] [force(MjtNum)]; "angular momentum about subtree com"; model.ffi().nbody],
        qH: &[MjtNum; "L'*D*L factorization of modified M"; model.ffi().nC],
        qHDiagInv: &[MjtNum; "1/diag(D) of modified M"; model.ffi().nv],
        qDeriv: &[MjtNum; "d (passive + actuator - bias) / d qvel"; model.ffi().nD],
        qLU: &[MjtNum; "sparse LU of (qM - dt*qDeriv)"; model.ffi().nD],
        actuator_force: &[MjtNum; "actuator force in actuation space"; model.ffi().nu],
        qfrc_actuator: &[MjtNum; "actuator force"; model.ffi().nv],
        qfrc_smooth: &[MjtNum; "net unconstrained force"; model.ffi().nv],
        qacc_smooth: &[MjtNum; "unconstrained acceleration"; model.ffi().nv],
        qfrc_constraint: &[MjtNum; "constraint force"; model.ffi().nv],
        qfrc_inverse: &[MjtNum; "net external force; should equal qfrc_applied + J'*xfrc_applied + qfrc_actuator"; model.ffi().nv],
        cacc: &[[MjtNum; 6] [force(MjtNum)]; "com-based acceleration"; model.ffi().nbody],
        cfrc_int: &[[MjtNum; 6] [force(MjtNum)]; "com-based interaction force with parent"; model.ffi().nbody],
        cfrc_ext: &[[MjtNum; 6] [force(MjtNum)]; "com-based external force on body"; model.ffi().nbody],
        (unsafe) contact: &[MjContact; "array of all detected contacts"; ffi().ncon],
        (unsafe) efc_type: &[MjtConstraint [force(i32)]; "constraint type"; ffi().nefc],
        (unsafe) efc_id: &[i32; "id of object of specified type"; ffi().nefc],
        (unsafe) efc_J_rownnz: &[i32; "number of non-zeros in constraint Jacobian row"; ffi().nefc],
        (unsafe) efc_J_rowadr: &[i32; "row start address in colind array"; ffi().nefc],
        (unsafe) efc_J_rowsuper: &[i32; "number of subsequent rows in supernode"; ffi().nefc],
        (unsafe) efc_J_colind: &[i32; "column indices in constraint Jacobian"; ffi().nJ],
        efc_J: &[MjtNum; "constraint Jacobian"; ffi().nJ],
        efc_pos: &[MjtNum; "constraint position (equality, contact)"; ffi().nefc],
        efc_margin: &[MjtNum; "inclusion margin (contact)"; ffi().nefc],
        efc_frictionloss: &[MjtNum; "frictionloss (friction)"; ffi().nefc],
        efc_diagApprox: &[MjtNum; "approximation to diagonal of A"; ffi().nefc],
        efc_KBIP: &[[MjtNum; 4] [force(MjtNum)]; "stiffness, damping, impedance, imp'"; ffi().nefc],
        efc_D: &[MjtNum; "constraint mass"; ffi().nefc],
        efc_R: &[MjtNum; "inverse constraint mass"; ffi().nefc],
        (unsafe) tendon_efcadr: &[i32; "first efc address involving tendon; -1: none"; model.ffi().ntendon],
        (unsafe) tree_island: &[i32; "island id of this tree; -1: none"; model.ffi().ntree],
        (unsafe) island_ntree: &[i32; "number of trees in this island"; ffi().nisland],
        (unsafe) island_itreeadr: &[i32; "island start address in itree vector"; ffi().nisland],
        (unsafe) map_itree2tree: &[i32; "map from itree to tree"; model.ffi().ntree],
        (unsafe) dof_island: &[i32; "island id of this dof; -1: none"; model.ffi().nv],
        (unsafe) island_nv: &[i32; "number of dofs in this island"; ffi().nisland],
        (unsafe) island_idofadr: &[i32; "island start address in idof vector"; ffi().nisland],
        (unsafe) island_dofadr: &[i32; "island start address in dof vector"; ffi().nisland],
        (unsafe) map_dof2idof: &[i32; "map from dof to idof"; model.ffi().nv],
        (unsafe) map_idof2dof: &[i32; "map from idof to dof;  >= nidof: unconstrained"; model.ffi().nv],
        ifrc_smooth: &[MjtNum; "net unconstrained force"; ffi().nidof],
        iacc_smooth: &[MjtNum; "unconstrained acceleration"; ffi().nidof],
        (unsafe) iM_rownnz: &[i32; "inertia: non-zeros in each row"; ffi().nidof],
        (unsafe) iM_rowadr: &[i32; "inertia: address of each row in iM_colind"; ffi().nidof],
        (unsafe) iM_colind: &[i32; "inertia: column indices of non-zeros"; model.ffi().nC],
        iM: &[MjtNum; "total inertia (sparse)"; model.ffi().nC],
        iLD: &[MjtNum; "L'*D*L factorization of M (sparse)"; model.ffi().nC],
        iLDiagInv: &[MjtNum; "1/diag(D)"; ffi().nidof],
        iacc: &[MjtNum; "acceleration"; ffi().nidof],
        (unsafe) efc_island: &[i32; "island id of this constraint"; ffi().nefc],
        (unsafe) island_ne: &[i32; "number of equality constraints in island"; ffi().nisland],
        (unsafe) island_nf: &[i32; "number of friction constraints in island"; ffi().nisland],
        (unsafe) island_nefc: &[i32; "number of constraints in island"; ffi().nisland],
        (unsafe) island_iefcadr: &[i32; "start address in iefc vector"; ffi().nisland],
        (unsafe) map_efc2iefc: &[i32; "map from efc to iefc"; ffi().nefc],
        (unsafe) map_iefc2efc: &[i32; "map from iefc to efc"; ffi().nefc],
        (unsafe) iefc_type: &[MjtConstraint [force(i32)]; "constraint type"; ffi().nefc],
        (unsafe) iefc_id: &[i32; "id of object of specified type"; ffi().nefc],
        (unsafe) iefc_J_rownnz: &[i32; "number of non-zeros in constraint Jacobian row"; ffi().nefc],
        (unsafe) iefc_J_rowadr: &[i32; "row start address in colind array"; ffi().nefc],
        (unsafe) iefc_J_rowsuper: &[i32; "number of subsequent rows in supernode"; ffi().nefc],
        (unsafe) iefc_J_colind: &[i32; "column indices in constraint Jacobian"; ffi().nJ],
        iefc_J: &[MjtNum; "constraint Jacobian"; ffi().nJ],
        iefc_frictionloss: &[MjtNum; "frictionloss (friction)"; ffi().nefc],
        iefc_D: &[MjtNum; "constraint mass"; ffi().nefc],
        iefc_R: &[MjtNum; "inverse constraint mass"; ffi().nefc],
        (unsafe) efc_AR_rownnz: &[i32; "number of non-zeros in AR"; ffi().nefc],
        (unsafe) efc_AR_rowadr: &[i32; "row start address in colind array"; ffi().nefc],
        (unsafe) efc_AR_colind: &[i32; "column indices in sparse AR"; ffi().nA],
        efc_AR: &[MjtNum; "J*inv(M)*J' + R"; ffi().nA],
        efc_vel: &[MjtNum; "velocity in constraint space: J*qvel"; ffi().nefc],
        efc_aref: &[MjtNum; "reference pseudo-acceleration"; ffi().nefc],
        efc_b: &[MjtNum; "linear cost term: J*qacc_smooth - aref"; ffi().nefc],
        iefc_aref: &[MjtNum; "reference pseudo-acceleration"; ffi().nefc],
        iefc_state: &[MjtConstraintState [force(i32)]; "constraint state"; ffi().nefc],
        iefc_force: &[MjtNum; "constraint force in constraint space"; ffi().nefc],
        efc_state: &[MjtConstraintState [force(i32)]; "constraint state"; ffi().nefc],
        efc_force: &[MjtNum; "constraint force in constraint space"; ffi().nefc],
        ifrc_constraint: &[MjtNum; "constraint force"; ffi().nidof]
    }
}

impl<M: Deref<Target = MjModel>> Drop for MjData<M> {
    fn drop(&mut self) {
        unsafe {
            mj_deleteData(self.data.as_ptr());
        }
    }
}

impl<M: Deref<Target = MjModel> + Clone> Clone for MjData<M> {
    /// # Panics
    /// Panics if MuJoCo fails to allocate the cloned data.
    /// Use [`MjData::try_clone`] for a fallible alternative.
    fn clone(&self) -> Self {
        self.try_clone().expect("not enough space to clone data")
    }
}

impl<M: Deref<Target = MjModel> + Clone> MjData<M> {
    /// Fallible version of [`Clone::clone`].
    ///
    /// # Errors
    /// Returns [`MjDataError::AllocationFailed`] if MuJoCo fails to allocate
    /// the copy.
    pub fn try_clone(&self) -> Result<Self, MjDataError> {
        let raw = unsafe { mj_copyData(ptr::null_mut(), self.model.ffi(), self.ffi()) };
        NonNull::new(raw)
            .map(|data| Self { data, model: self.model.clone() })
            .ok_or(MjDataError::AllocationFailed)
    }
}

info_with_view!(Data, actuator,
    [ctrl: MjtNum,
     [actuator_] length: MjtNum,
     [actuator_] velocity: MjtNum,
     [actuator_] force: MjtNum],
    [],
    [act: MjtNum], M: Deref<Target = MjModel>);

info_with_view!(Data, body,
    [xfrc_applied: MjtNum,
     xpos: MjtNum,
     xquat: MjtNum,
     xmat: MjtNum,
     xipos: MjtNum,
     ximat: MjtNum,
     subtree_com: MjtNum,
     cinert: MjtNum,
     crb: MjtNum,
     cvel: MjtNum,
     subtree_linvel: MjtNum,
     subtree_angmom: MjtNum,
     cacc: MjtNum,
     cfrc_int: MjtNum,
     cfrc_ext: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, camera,
    [[cam_] xpos: MjtNum,
     [cam_] xmat: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, geom,
    [[geom_] xpos: MjtNum,
     [geom_] xmat: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, joint,
    [qpos: MjtNum,
     qvel: MjtNum,
     qacc_warmstart: MjtNum,
     qfrc_applied: MjtNum,
     qacc: MjtNum,
     xanchor: MjtNum,
     xaxis: MjtNum,
     qLDiagInv: MjtNum,
     qfrc_bias: MjtNum,
     qfrc_spring: MjtNum,
     qfrc_damper: MjtNum,
     qfrc_gravcomp: MjtNum,
     qfrc_fluid: MjtNum,
     qfrc_passive: MjtNum,
     qfrc_actuator: MjtNum,
     qfrc_smooth: MjtNum,
     qacc_smooth: MjtNum,
     qfrc_constraint: MjtNum,
     qfrc_inverse: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, light,
    [[light_] xpos: MjtNum,
     [light_] xdir: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, sensor,
    [[sensor] data: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, site,
    [[site_] xpos: MjtNum,
     [site_] xmat: MjtNum],
    [],
    [], M: Deref<Target = MjModel>);

info_with_view!(Data, tendon,
    [[ten_] J: MjtNum,
     [ten_] length: MjtNum,
     [ten_] velocity: MjtNum],
    [[ten_] wrapadr: i32,
     [ten_] wrapnum: i32],
    [], M: Deref<Target = MjModel>);

/**************************************************************************************************/
// Unit tests
/**************************************************************************************************/

#[cfg(test)]
mod test {
    use crate::assert_relative_eq;
    use crate::prelude::*;
    use super::*;

    const MODEL: &str = "
<mujoco>
  <asset>
    <mesh name=\"cube\" vertex=\"-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5  0.5 -0.5  0.5  0.5 -0.5  -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5  0.5  0.5  0.5  0.5  0.5\"/>
    <hfield name=\"terrain\" nrow=\"10\" ncol=\"10\" size=\"10 10 .1 .1\"/>
  </asset>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\" pos=\".2 .2 .1\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball\" type=\"free\"/>
    </body>

    <body name=\"ball2\" pos=\".7 .2 .1\">
        <geom name=\"green_sphere2\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball2\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" solref=\"0.004 1.0\"/>
    <geom name=\"mesh_cube\" type=\"mesh\" mesh=\"cube\" pos=\"2 2 0.5\"/>
    <geom name=\"hfield_terrain\" type=\"hfield\" hfield=\"terrain\" pos=\"-2 -2 0\"/>
  </worldbody>
  <actuator>
    <motor name=\"motor_ball\" joint=\"ball\"/>
  </actuator>
</mujoco>";


    #[test]
    fn test_joint_view() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        let joint_info = data.joint("ball").unwrap();
        let body_info = data.body("ball").unwrap();

        for _ in 0..10 {
            data.step();
        }

        /* The ball should start in a still position */
        let mut joint_view = joint_info.view(&data);
        assert_relative_eq!(joint_view.qvel[0], 0.0, epsilon=1e-9);  // vx
        assert_relative_eq!(joint_view.qvel[1], 0.0, epsilon=1e-9);  // vy
        // assert_relative_eq!(view.qvel[2], 0.0);  // vz Ignore due to slight instability of the model.
        assert_relative_eq!(joint_view.qvel[3], 0.0, epsilon=1e-9);  // wx
        assert_relative_eq!(joint_view.qvel[4], 0.0, epsilon=1e-9);  // wy
        assert_relative_eq!(joint_view.qvel[5], 0.0, epsilon=1e-9);  // wz

        /* Give the ball some velocity */
        let mut joint_view_mut = joint_info.view_mut(&mut data);
        joint_view_mut.qvel[0] = 0.5;  // vx = 0.5 m/s
        joint_view_mut.qvel[4] = 0.5 / 0.1;  // wy = 0.5 m/s / 0.1 m

        let initial_qpos: [MjtNum; 3] = joint_view_mut.qpos[..3].try_into().unwrap();  // initial x, y and z.
        data.step();

        /* Test if the ball is moving in the x direction and rotating around y. */
        joint_view = joint_info.view(&data);
        assert_eq!(joint_view.qfrc_spring.len(), joint_view.qvel.len());
        assert_eq!(joint_view.qfrc_damper.len(), joint_view.qvel.len());
        assert_eq!(joint_view.qfrc_gravcomp.len(), joint_view.qvel.len());
        assert_eq!(joint_view.qfrc_fluid.len(), joint_view.qvel.len());
        joint_view = joint_info.view(&data);
        assert_relative_eq!(joint_view.qvel[0], 0.5, epsilon=1e-3);  // vx
        assert_relative_eq!(joint_view.qvel[4], 0.5 / 0.1, epsilon=1e-3);  // wy

        /* Test correct placement */
        let timestep = model.opt().timestep;
        assert_relative_eq!(joint_view.qpos[0], initial_qpos[0] + timestep * joint_view.qvel[0], epsilon=1e-9);  // p = p + dp/dt * dt
        assert_relative_eq!(joint_view.qpos[1], initial_qpos[1] + timestep * joint_view.qvel[1], epsilon=1e-9);
        assert_relative_eq!(joint_view.qpos[2], initial_qpos[2] + timestep * joint_view.qvel[2], epsilon=1e-9);

        /* Test consistency with the body */
        data.step1();  // update derived variables.

        joint_view = joint_info.view(&data);
        let body_view = body_info.view(&data);
        /* Consistency in position */
        assert_relative_eq!(joint_view.qpos[0], body_view.xpos[0], epsilon=1e-9);  // same position.
        assert_relative_eq!(joint_view.qpos[1], body_view.xpos[1], epsilon=1e-9);
        assert_relative_eq!(joint_view.qpos[2], body_view.xpos[2], epsilon=1e-9);

        assert_relative_eq!(joint_view.qpos[3], body_view.xquat[0], epsilon=1e-9);  // same orientation.
        assert_relative_eq!(joint_view.qpos[4], body_view.xquat[1], epsilon=1e-9);
        assert_relative_eq!(joint_view.qpos[5], body_view.xquat[2], epsilon=1e-9);
        assert_relative_eq!(joint_view.qpos[6], body_view.xquat[3], epsilon=1e-9);

        /* Consistency in velocity */
        assert_relative_eq!(joint_view.qvel[0], body_view.cvel[3], epsilon=1e-9);  // same position velocity.
        assert_relative_eq!(joint_view.qvel[1], body_view.cvel[4], epsilon=1e-9);
        assert_relative_eq!(joint_view.qvel[2], body_view.cvel[5], epsilon=1e-9);

        assert_relative_eq!(joint_view.qvel[3], body_view.cvel[0], epsilon=1e-9);  // same rotational velocity.
        assert_relative_eq!(joint_view.qvel[4], body_view.cvel[1], epsilon=1e-9);
        assert_relative_eq!(joint_view.qvel[5], body_view.cvel[2], epsilon=1e-9);
    }

    #[test]
    fn test_actuator_view() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let data = model.make_data();
        let actuator_info = data.actuator("motor_ball").unwrap();

        let actuator_view = actuator_info.view(&data);
        assert_eq!(actuator_view.length.len(), 1);
        assert_eq!(actuator_view.velocity.len(), 1);
        assert_eq!(actuator_view.force.len(), 1);
        assert!(actuator_view.act.is_none());

        // Test if indexing corresponds to exact data structure mapping
        unsafe {
            assert_relative_eq!(actuator_view.length[0], *data.ffi().actuator_length.add(actuator_info.id), epsilon=1e-9);
            assert_relative_eq!(actuator_view.velocity[0], *data.ffi().actuator_velocity.add(actuator_info.id), epsilon=1e-9);
            assert_relative_eq!(actuator_view.force[0], *data.ffi().actuator_force.add(actuator_info.id), epsilon=1e-9);
        }
    }

    #[test]
    fn test_body_view() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        let body_info = data.body("ball2").unwrap();
        let mut cvel;

        data.step1();

        for _ in 0..10 {
            data.step2();
            data.step1();  // step() and step2() update before integration, thus we need to manually update non-state variables.
        }

        // The ball should start in a still position.
        // Use a loose epsilon because physics simulation results can differ by small
        // floating-point amounts across architectures (e.g. x86_64 vs aarch64).
        cvel = body_info.view(&data).cvel;
        assert_relative_eq!(cvel[0], 0.0, epsilon=1e-5);
        assert_relative_eq!(cvel[1], 0.0, epsilon=1e-5);
        assert_relative_eq!(cvel[2], 0.0, epsilon=1e-5);
        assert_relative_eq!(cvel[3], 0.0, epsilon=1e-5);
        assert_relative_eq!(cvel[4], 0.0, epsilon=1e-5);
        // assert_relative_eq!(cvel[5], 0.0);  // Ignore due to slight instability of the model.

        // Give the ball some velocity
        body_info.view_mut(&mut data).xfrc_applied[0] = 5.0;
        data.step2();
        data.step1();

        let view = body_info.view(&data);
        cvel = view.cvel;
        println!("{:?}", cvel);
        assert_relative_eq!(cvel[0], 0.0, epsilon=1e-9);
        assert!(cvel[1] > 0.0);  // wy should be positive when rolling with positive vx.
        assert_relative_eq!(cvel[2], 0.0, epsilon=1e-9);
        assert!(cvel[3] > 0.0);  // vx should point in the direction of the applied force.
        // assert_relative_eq!(cvel[5], 0.0);  // vz should be 0, but we don't test it due to jumpiness (instability) of the ball.

        assert_relative_eq!(view.xfrc_applied[0], 5.0, epsilon=1e-9); // the original force should stay applied.

        data.step2();
        data.step1();
    }

    #[test]
    fn test_copy_reset_variants() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Test reset variants
        data.reset();
        data.reset_debug(7);
        data.reset_keyframe(0);
    }

    #[test]
    fn test_dynamics_and_sensors() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Simulation pipeline components
        data.fwd_position();
        data.fwd_velocity();
        data.fwd_actuation();
        data.fwd_acceleration();
        data.fwd_constraint();

        data.euler();
        data.runge_kutta(4);
        // data.implicit();  // integrator isn't implicit in the model => skip this check

        data.inv_position();
        data.inv_velocity();
        data.inv_constraint();
        data.compare_fwd_inv();

        // Sensors
        data.sensor_pos();
        data.sensor_vel();
        data.sensor_acc();

        data.energy_pos();
        data.energy_vel();

        data.check_pos();
        data.check_vel();
        data.check_acc();

        data.kinematics();
        data.com_pos();
        data.camlight();
        data.flex_comp();
        data.tendon_comp();
        data.transmission();
        data.crb_comp();
        data.make_m();
        data.factor_m();
        data.com_vel();
        data.passive();
        data.subtree_vel();
    }


    #[test]
    fn test_rne_and_collision_pipeline() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        for _ in 0..5 {
            data.step();
        }

        // mj_rne returns a scalar as result
        data.rne(true);

        data.rne_post_constraint();

        // Collision and constraint pipeline
        data.collision();
        data.make_constraint();
        data.island();
        data.project_constraint();
        data.reference_constraint();

        let nefc = data.nefc() as usize;
        assert!(nefc > 0, "expected at least one effective constraint after stepping");
        let jar = vec![0.0; nefc];
        let mut cost = 0.0;
        data.constraint_update(&jar, None, false).unwrap();
        data.constraint_update(&jar, Some(&mut cost), true).unwrap();
    }

    #[test]
    fn test_add_contact() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Add a dummy contact
        let dummy_contact = bytemuck::Zeroable::zeroed();
        data.add_contact(&dummy_contact).unwrap();
    }

    #[test]
    fn test_jacobian() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        let nv = data.model.ffi().nv as usize;
        let expected_len = 3 * nv;

        // Use a small offset point relative to the joint origin
        let point = [0.1, 0.0, 0.0];

        let ball_body_id = model.body("ball").unwrap().id as i32;

        // Test global point Jacobian
        let (jacp, jacr) = data.jac(true, true, &point, ball_body_id).unwrap();
        assert_eq!(jacp.len(), expected_len);
        assert_eq!(jacr.len(), expected_len);

        // Test body frame Jacobian
        let (jacp_body, jacr_body) = data.jac_body(true, true, ball_body_id).unwrap();
        assert_eq!(jacp_body.len(), expected_len);
        assert_eq!(jacr_body.len(), expected_len);

        // Test body COM Jacobian
        let (jacp_com, jacr_com) = data.jac_body_com(true, true, ball_body_id).unwrap();
        assert_eq!(jacp_com.len(), expected_len);
        assert_eq!(jacr_com.len(), expected_len);

        // Test subtree COM Jacobian (translational only)
        let jac_subtree = data.jac_subtree_com(0).unwrap();
        assert_eq!(jac_subtree.len(), expected_len);

        // Test geom Jacobian
        let green_geom_id = model.geom("green_sphere").unwrap().id as i32;
        let (jacp_geom, jacr_geom) = data.jac_geom(true, true, green_geom_id).unwrap();
        assert_eq!(jacp_geom.len(), expected_len);
        assert_eq!(jacr_geom.len(), expected_len);

        // Test site Jacobian - only if sites exist
        if model.ffi().nsite > 0 {
            let site_id = 0i32;
            let (jacp_site, jacr_site) = data.jac_site(true, true, site_id).unwrap();
            assert_eq!(jacp_site.len(), expected_len);
            assert_eq!(jacr_site.len(), expected_len);
        }

        // Test flags set to false produce empty Vec
        let (jacp_none, jacr_none) = data.jac(false, false, &[0.0; 3], ball_body_id).unwrap();
        assert!(jacp_none.is_empty());
        assert!(jacr_none.is_empty());
    }

    #[test]
    fn test_angmom_and_object_dynamics() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        let mat = data.angmom_mat(0).unwrap();
        assert_eq!(mat.len(), (3 * data.model.ffi().nv as usize));

        let vel = data.object_velocity(MjtObj::mjOBJ_BODY, 0, true).unwrap();
        assert_eq!(vel.len(), 6);

        let acc = data.object_acceleration(MjtObj::mjOBJ_BODY, 0, false).unwrap();
        assert_eq!(acc.len(), 6);
    }

    #[test]
    fn test_geom_distance_and_transforms() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        data.step();

        // Test actual distance between two different geoms (green_sphere and green_sphere2).
        // green_sphere is at (.2, .2, .1) and green_sphere2 is at (.7, .2, .1), both with radius 0.1.
        // Expected distance ~= 0.5 - 2*0.1 = 0.3 (center distance minus both radii).
        let geom0_id = model.name_to_id(MjtObj::mjOBJ_GEOM, "green_sphere").unwrap();
        let geom1_id = model.name_to_id(MjtObj::mjOBJ_GEOM, "green_sphere2").unwrap();
        assert!(geom0_id >= 0 && geom1_id >= 0);

        let mut ft = [0.0; 6];
        let dist = data.geom_distance(geom0_id, geom1_id, 1.0, Some(&mut ft)).unwrap();
        assert!(dist > 0.0, "distance between separate geoms should be positive, got {dist}");
        assert!(dist < 1.0, "distance should be less than distmax, got {dist}");
        assert_relative_eq!(dist, 0.3, epsilon=1e-3);
        // fromto should be populated: first 3 = nearest point on geom0, last 3 = nearest point on geom1
        let ft_norm = ft.iter().map(|x| x * x).sum::<MjtNum>().sqrt();
        assert!(ft_norm > 0.0, "fromto should be non-zero for non-overlapping geoms");

        let pos = [0.0; 3];
        let quat = [1.0, 0.0, 0.0, 0.0];
        let (xpos, xmat) = data.local_to_global(&pos, &quat, 0, MjtSameFrame::mjSAMEFRAME_NONE).unwrap();
        assert_eq!(xpos.len(), 3);
        assert_eq!(xmat.len(), 9);

        let ray_vecs = [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let rays = data.multi_ray(&pos, &ray_vecs, None, false, -1, 10.0, None).unwrap();
        assert_eq!(rays.0.len(), 3);
        assert_eq!(rays.1.len(), 3);

        let (geomid, dist) = data.ray(&pos, &[1.0, 0.0, 0.0], None, true, -1, None);
        assert!(dist.is_finite());
        assert!(geomid >= -1);

        // ray API with normal output (optional parameter)
        let mut normal = [0.0; 3];
        let (geomid2, dist2) = data.ray(&pos, &[1.0, 0.0, 0.0], None, true, -1, Some(&mut normal));
        assert!(dist2.is_finite());
        assert!(geomid2 >= -1);
        let norm_len = (normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]).sqrt();
        if dist2 >= 0.0 {
            // hit - normal should be non-zero
            assert!(norm_len > 0.0);
        } else {
            // no hit - normal buffer is unchanged / zeroed
            assert_eq!(normal, [0.0; 3]);
        }

        let mut normals_buf = vec![[0.0; 3]; ray_vecs.len()];
        let (gids, dists) = data.multi_ray(&pos, &ray_vecs, None, false, -1, 10.0, Some(&mut normals_buf)).unwrap();
        assert_eq!(gids.len(), normals_buf.len());
        assert_eq!(dists.len(), normals_buf.len());
        for (d, n) in dists.iter().zip(normals_buf.iter()) {
            let l = (n[0]*n[0] + n[1]*n[1] + n[2]*n[2]).sqrt();
            if *d >= 0.0 {
                assert!(l > 0.0);
            } else {
                assert_eq!(*n, [0.0; 3]);
            }
        }
    }

    #[test]
    fn test_constraint_update_checks_jar_length() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Need to step to generate contact constraints
        // Multiple steps ensure collisions are properly detected and compiled
        for _ in 0..5 {
            data.step();
        }

        // After stepping with a contact-rich model, nefc should be > 0
        let nefc = data.ffi().nefc as usize;
        assert!(nefc > 0, "Expected nefc > 0 after stepping with contact model, got {}", nefc);

        // correct length should not panic
        let jar = vec![0.0; nefc];
        data.constraint_update(&jar, None, false).unwrap();

        // wrong length should return Err
        let bad_jar = vec![0.0; 1];
        let result = data.constraint_update(&bad_jar, None, false);
        assert!(result.is_err());
    }

    #[test]
    fn test_init_history_ctrl_and_sensor() {
        const HIST_MODEL: &str = r#"
<mujoco>
  <option timestep="0.01"/>
  <worldbody>
    <body name="body">
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor0" joint="slide" delay="0.05" nsample="6"/>
  </actuator>
  <sensor>
    <jointpos name="jointpos0" joint="slide" delay="0.025" interp="linear" nsample="4"/>
  </sensor>
</mujoco>
"#;

        let model = MjModel::from_xml_string(HIST_MODEL).unwrap();
        let mut data = model.make_data();

        let times_ctrl: Vec<MjtNum> = (0..6).map(|i| i as MjtNum * 0.01).collect();
        let values_ctrl = vec![1.2345; 6];
        data.init_ctrl_history(0, Some(&times_ctrl), &values_ctrl).unwrap();

        // read back via safe wrapper: exact-match should return provided value
        let val = data.read_ctrl(0, times_ctrl[2], 0).unwrap();
        assert_relative_eq!(val, values_ctrl[2], epsilon=1e-12);

        // sensor history via wrapper
        let times_sens: Vec<MjtNum> = (0..4).map(|i| i as MjtNum * 0.01).collect();
        // sensor dim for jointpos is 1
        let values_sens = vec![2.718; 4 * 1];
        data.init_sensor_history(0, Some(&times_sens), &values_sens, 0.0).unwrap();

        let got = data.read_sensor_fixed::<1>(0, times_sens[1], 0).unwrap();
        assert_eq!(got.len(), 1);
        assert_relative_eq!(got[0], values_sens[1], epsilon=1e-12);

        // also test interpolation path (time between samples)
        let interp_res = data.read_sensor_fixed::<1>(0, 0.005, 1).unwrap();
        assert_eq!(interp_res.len(), 1);
        // values are identical here, but ensure we get a value
        assert_relative_eq!(interp_res[0], values_sens[0], epsilon=1e-12);
    }

    #[test]
    fn test_fwd_kinematics_and_mul_wrappers() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        let joint_info = data.joint("ball").unwrap();
        {
            let mut jv = joint_info.view_mut(&mut data);
            jv.qpos[0] = 0.42;  // set body x
            jv.qpos[1] = 0.43;  // set body y
            jv.qpos[2] = 0.44;  // set body z
            jv.qpos[3] = 1.0;   // unit quaternion
            jv.qpos[4] = 0.0;
            jv.qpos[5] = 0.0;
            jv.qpos[6] = 0.0;
        }
        data.forward_kinematics();
        let body_view = data.body("ball").unwrap().view(&data);
        assert_relative_eq!(body_view.xpos[0], 0.42, epsilon=1e-9);
        assert_relative_eq!(body_view.xpos[1], 0.43, epsilon=1e-9);
        assert_relative_eq!(body_view.xpos[2], 0.44, epsilon=1e-9);
    }

    #[test]
    fn test_qpos_view() {
        const JOINT_BALL_DOF: usize = 7;
        const BALL_INDEX: usize = 1;
        const DOF_TO_MODIFY: usize = 2;
        const MODIFIED_VALUE: f64 = 15.0;

        let model = MjModel::from_xml_string(MODEL).unwrap();
        let name = model.id_to_name(MjtObj::mjOBJ_JOINT, BALL_INDEX as i32).unwrap();

        let mut data = MjData::new(&model);
        let ball2_joint_info = data.joint(name).unwrap();
        ball2_joint_info.view_mut(&mut data).qpos[DOF_TO_MODIFY] = MODIFIED_VALUE;

        assert_eq!(data.qpos()[JOINT_BALL_DOF * BALL_INDEX + DOF_TO_MODIFY], MODIFIED_VALUE);
    }

    #[test]
    fn test_contacts() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = MjData::new(&model);
        let ptr = unsafe { data.ffi_mut().contact };
        assert!(ptr.is_aligned());
        assert!(!ptr.is_null());
        assert_eq!(data.contact().len(), 0);
        data.step();

        assert!(data.contact().len() != 0);
        assert_eq!(data.contact().len(), data.contacts().len());
    }

    #[test]
    fn test_init_ctrl_history_all_combinations() {
        const HIST_MODEL: &str = r#"
<mujoco>
  <worldbody>
    <body name="body">
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor0" joint="slide" delay="0.05" nsample="6"/>
  </actuator>
</mujoco>
"#;

        let model = MjModel::from_xml_string(HIST_MODEL).unwrap();
        let mut data = model.make_data();

        let times: Vec<MjtNum> = (0..6).map(|i| i as MjtNum * 0.01).collect();
        let values = vec![1.2345; 6];

        // success: times Some / None
        assert!(data.init_ctrl_history(0, Some(&times), &values).is_ok());
        assert!(data.init_ctrl_history(0, None, &values).is_ok());

        // times length mismatch -> LengthMismatch
        let bad_times = vec![0.0f64];
        let err = data.init_ctrl_history(0, Some(&bad_times), &values).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "times", .. }));

        // values length mismatch -> LengthMismatch
        let bad_values = vec![1.0; 5];
        let err = data.init_ctrl_history(0, Some(&times), &bad_values).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "values", .. }));

        // invalid actuator id -> IndexOutOfBounds
        let err = data.init_ctrl_history(99, Some(&times), &values).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "actuator_id", .. }));

        // read_ctrl invalid id -> IndexOutOfBounds
        let err = data.read_ctrl(99, times[0], 0).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "actuator_id", .. }));

        // actuator exists but has no history buffer -> NoHistoryBuffer
        const NO_HIST_ACT_MODEL: &str = r#"
<mujoco>
  <worldbody>
    <body>
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor0" joint="slide"/>
  </actuator>
</mujoco>
"#;
        let model2 = MjModel::from_xml_string(NO_HIST_ACT_MODEL).unwrap();
        let mut data2 = model2.make_data();
        let err = data2.init_ctrl_history(0, Some(&times), &values).unwrap_err();
        assert!(matches!(err, MjDataError::NoHistoryBuffer { kind: "actuator", id: 0 }));
    }

    #[test]
    fn test_init_sensor_history_all_combinations() {
        const HIST_SENSOR_MODEL: &str = r#"
<mujoco>
  <worldbody>
    <body name="body">
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="jointpos0" joint="slide" delay="0.025" interp="linear" nsample="4"/>
  </sensor>
</mujoco>
"#;

        let model = MjModel::from_xml_string(HIST_SENSOR_MODEL).unwrap();
        let mut data = model.make_data();

        let times_sens: Vec<MjtNum> = (0..4).map(|i| i as MjtNum * 0.01).collect();
        let values_sens = vec![2.718; 4 * 1]; // dim == 1 for jointpos

        // success: times Some / None
        assert!(data.init_sensor_history(0, Some(&times_sens), &values_sens, 0.0).is_ok());
        assert!(data.init_sensor_history(0, None, &values_sens, 0.0).is_ok());

        // times length mismatch -> LengthMismatch
        let bad_times = vec![0.0f64];
        let err = data.init_sensor_history(0, Some(&bad_times), &values_sens, 0.0).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "times", .. }));

        // values length mismatch -> LengthMismatch
        let bad_values = vec![3.14; 3];
        let err = data.init_sensor_history(0, Some(&times_sens), &bad_values, 0.0).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "values", .. }));

        // invalid sensor id -> IndexOutOfBounds
        let err = data.init_sensor_history(99, Some(&times_sens), &values_sens, 0.0).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "sensor_id", .. }));

        // read_sensor invalid id -> IndexOutOfBounds
        let err: Result<[MjtNum; 1], MjDataError> = data.read_sensor_fixed::<1>(99, times_sens[0], 0);
        assert!(matches!(err.unwrap_err(), MjDataError::IndexOutOfBounds { kind: "sensor_id", .. }));

        // sensor exists but has no history buffer -> NoHistoryBuffer
        const NO_HIST_SENS_MODEL: &str = r#"
<mujoco>
  <worldbody>
    <body>
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="jointpos0" joint="slide"/>
  </sensor>
</mujoco>
"#;
        let model2 = MjModel::from_xml_string(NO_HIST_SENS_MODEL).unwrap();
        let mut data2 = model2.make_data();
        let err = data2.init_sensor_history(0, Some(&times_sens), &values_sens, 0.0).unwrap_err();
        assert!(matches!(err, MjDataError::NoHistoryBuffer { kind: "sensor", id: 0 }));
    }

    #[test]
    fn test_read_sensor_variants() {
        // Model with a delayed sensor (history enabled) so we can test both
        // Cow::Borrowed (exact time match) and Cow::Owned (interpolation) paths.
        const HIST_MODEL: &str = r#"
<mujoco>
  <option timestep="0.01"/>
  <worldbody>
    <body>
      <joint name="slide" type="slide"/>
      <geom size="0.1"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="jp" joint="slide" delay="0.03" interp="linear" nsample="4"/>
  </sensor>
</mujoco>
"#;
        let model = MjModel::from_xml_string(HIST_MODEL).unwrap();
        let mut data = model.make_data();
        let delay = 0.03;

        // Seed the history buffer with known distinct values.
        let hist_times: Vec<_> = (0..4).map(|i| i as MjtNum * 0.01).collect();
        let values = vec![10.0, 20.0, 30.0, 40.0]; // dim==1 for jointpos
        data.init_sensor_history(0, Some(&hist_times), &values, 0.0).unwrap();

        // mj_readSensor internally reads at (time - delay), so to read history
        // entry at hist_times[i] we must pass time = hist_times[i] + delay.

        // exact match -> Cow::Borrowed
        let query_time = hist_times[2] + delay; // 0.02 + 0.03 = 0.05
        let cow = data.read_sensor(0, query_time, 0).unwrap();
        assert_eq!(cow.len(), 1);
        assert_relative_eq!(cow[0], 30.0, epsilon = 1e-12);
        assert!(matches!(cow, std::borrow::Cow::Borrowed(_)),
                "exact match should yield Cow::Borrowed");

        // interpolation -> Cow::Owned
        // midpoint between hist_times[1]=0.01 and hist_times[2]=0.02 -> internal time 0.015
        let interp_query = (hist_times[1] + hist_times[2]) / 2.0 + delay; // 0.045
        let cow_interp = data.read_sensor(0, interp_query, 1).unwrap();
        assert_eq!(cow_interp.len(), 1);
        assert_relative_eq!(cow_interp[0], 25.0, epsilon = 1e-6); // linear interp of 20 and 30
        assert!(matches!(cow_interp, std::borrow::Cow::Owned(_)),
                "interpolation should yield Cow::Owned");

        // read_sensor_fixed<N>: exact match (stack array)
        let arr_exact: [f64; 1] = data.read_sensor_fixed(0, query_time, 0).unwrap();
        assert_relative_eq!(arr_exact[0], 30.0, epsilon = 1e-12);

        // read_sensor_fixed<N>: interpolation (stack array)
        let arr_interp: [f64; 1] = data.read_sensor_fixed(0, interp_query, 1).unwrap();
        assert_relative_eq!(arr_interp[0], 25.0, epsilon = 1e-6);

        // read_sensor_fixed<N>: wrong N -> Err(LengthMismatch)
        let err: Result<[MjtNum; 3], MjDataError> = data.read_sensor_fixed::<3>(0, query_time, 0);
        assert!(matches!(err.unwrap_err(), MjDataError::LengthMismatch { name: "N", .. }));

        // read_sensor_into: exact match
        let mut buf = [0.0; 1];
        data.read_sensor_into(0, hist_times[0] + delay, 0, &mut buf).unwrap();
        assert_relative_eq!(buf[0], 10.0, epsilon = 1e-12);

        // read_sensor_into: interpolation
        let mut buf2 = [0.0; 1];
        data.read_sensor_into(0, interp_query, 1, &mut buf2).unwrap();
        assert_relative_eq!(buf2[0], 25.0, epsilon = 1e-6);

        // read_sensor_into: buffer too small -> Err(LengthMismatch)
        let mut tiny: [MjtNum; 0] = [];
        let err = data.read_sensor_into(0, hist_times[0] + delay, 0, &mut tiny).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "dst", .. }));

        // read_sensor_into: buffer too large -> Err(LengthMismatch)
        let mut big = [0.0; 4];
        let err = data.read_sensor_into(0, hist_times[3] + delay, 0, &mut big).unwrap_err();
        assert!(matches!(err, MjDataError::LengthMismatch { name: "dst", .. }));

        // invalid sensor id -> IndexOutOfBounds for all methods
        let err: Result<[MjtNum; 1], MjDataError> = data.read_sensor_fixed::<1>(99, 0.0, 0);
        assert!(matches!(err.unwrap_err(), MjDataError::IndexOutOfBounds { kind: "sensor_id", .. }));
        let err = data.read_sensor(99, 0.0, 0).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "sensor_id", .. }));
        let err = data.read_sensor_into(99, 0.0, 0, &mut buf).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "sensor_id", .. }));

        // read_sensor_fixed<N>, read_sensor, and read_sensor_into agree for all history times
        for &t in &hist_times {
            let query = t + delay;
            let arr = data.read_sensor_fixed::<1>(0, query, 0).unwrap();
            let cow_val = data.read_sensor(0, query, 0).unwrap();
            let mut into_val = [0.0; 1];
            data.read_sensor_into(0, query, 0, &mut into_val).unwrap();
            assert_relative_eq!(arr[0], cow_val[0], epsilon = 1e-12);
            assert_relative_eq!(arr[0], into_val[0], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_multi_ray_zero_rays() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        let pos = [0.0; 3];
        let ray_vecs: Vec<[MjtNum; 3]> = Vec::new();

        // ensure calling with zero rays returns empty vectors and does not crash
        let (gids, dists) = data.multi_ray(&pos, &ray_vecs, None, false, -1, 10.0, None).unwrap();
        assert!(gids.is_empty());
        assert!(dists.is_empty());
    }

    #[test]
    fn test_multi_ray_normals_length_mismatch() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        let pos = [0.0; 3];
        let ray_vecs = [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let mut bad_normals = vec![[0.0; 3]; 2]; // length 2 != 3

        let res = data.multi_ray(&pos, &ray_vecs, None, false, -1, 10.0, Some(&mut bad_normals));
        assert!(res.is_err());
        assert!(matches!(res.unwrap_err(), MjDataError::LengthMismatch { name: "normals_out", .. }));
    }

    #[test]
    fn test_copy_state_from_data() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data1 = model.make_data();
        let mut data2 = model.make_data();

        data1.set_time(1.23);
        data2.copy_state_from_data(&data1, MjtState::mjSTATE_TIME as u32);

        assert_eq!(data2.time(), 1.23);
    }

    #[test]
    #[should_panic(expected = "ray_flex: flexid")]
    fn test_ray_flex() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let data = model.make_data();
        let pos = [0.0; 3];
        let vec = [1.0, 0.0, 0.0];

        // Model has no flexes, so any flexid is out of bounds.
        data.ray_flex(0, false, false, false, false, 0, &pos, &vec, None, None);
    }

    #[test]
    fn test_ray_mesh_hfield() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        let mesh_id = model.geom("mesh_cube").unwrap().id as i32;
        let hfield_id = model.geom("hfield_terrain").unwrap().id as i32;

        data.forward_kinematics();

        // Ray should hit mesh (centered at 2,2,0.5, size 1x1x1)
        // Top surface of the mesh is at z=0.5 + 0.5 = 1.0.
        // Ray starts at z=5.0 and goes straight down [0, 0, -1].
        // Intersection should be at exactly distance 4.0.
        let mesh_dist = data.ray_mesh(mesh_id, &[2.0, 2.0, 5.0], &[0.0, 0.0, -1.0], None);
        assert_relative_eq!(mesh_dist, 4.0, epsilon=1e-5);

        // Ray should hit hfield (centered at -2,-2,0)
        // Default hfield with no data acts as a plane at z=0.
        // Intersection should be at exactly distance 5.0.
        let hfield_dist = data.ray_hfield(hfield_id, &[-2.0, -2.0, 5.0], &[0.0, 0.0, -1.0], None);
        assert_relative_eq!(hfield_dist, 5.0, epsilon=1e-5);
    }

    #[test]
    fn test_apply_ft() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        let nv = model.nv() as usize;
        let body_id = model.body("ball").unwrap().id as i32;
        let mut qfrc = vec![0.0; nv];

        data.forward();

        // Apply force/torque at the ball's center of mass (pos = [0.2, 0.2, 0.1]) in global frame
        let force = [1.5, 2.5, 3.5];
        let torque = [0.1, 0.2, 0.3];
        let point = [0.2, 0.2, 0.1];

        data.apply_ft(&force, &torque, &point, body_id, &mut qfrc);

        // The "ball" has a free joint.
        // In MuJoCo, for a free joint, the first 3 DOFs are translation (linear),
        // and the next 3 are the rotational DOFs.
        // Since we applied it exactly at the COM, there should be no induced torque from the force position.
        let dof_adr = model.body_dofadr()[body_id as usize] as usize;
        assert!((qfrc[dof_adr] - 1.5).abs() < 1e-5);
        assert!((qfrc[dof_adr + 1] - 2.5).abs() < 1e-5);
        assert!((qfrc[dof_adr + 2] - 3.5).abs() < 1e-5);
        assert!((qfrc[dof_adr + 3] - 0.1).abs() < 1e-5);
        assert!((qfrc[dof_adr + 4] - 0.2).abs() < 1e-5);
        assert!((qfrc[dof_adr + 5] - 0.3).abs() < 1e-5);
    }

    #[test]
    #[should_panic(expected = "model signature mismatch")]
    fn test_signature_mismatch_panics() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body><body name='extra'/></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let joint_info1 = data1.joint("j1").unwrap();

        // This should panic because joint_info1 was created from model1, but we are viewing it with model2/data2
        let data2 = model2.make_data();
        let _view = joint_info1.view(&data2);
    }

    #[test]
    #[should_panic(expected = "model signature mismatch")]
    fn test_signature_mismatch_reversed_joints() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body><body name='b2'><joint name='j2' type='ball'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j2' type='ball'/><geom size='0.1' mass='1'/></body><body name='b2'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let joint_info1 = data1.joint("j1").unwrap();

        // This should panic because the kinematic tree is structurally different
        let data2 = model2.make_data();
        let _view = joint_info1.view(&data2);
    }

    #[test]
    #[should_panic(expected = "model signature mismatch")]
    fn test_signature_mismatch_view_mut_panics() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body><body name='extra'/></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let joint_info1 = data1.joint("j1").unwrap();

        let mut data2 = model2.make_data();
        let _view = joint_info1.view_mut(&mut data2);
    }

    #[test]
    fn test_try_view_signature_mismatch() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body><body name='extra'/></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let joint_info1 = data1.joint("j1").unwrap();
        let mut data2 = model2.make_data();

        let err = joint_info1.try_view(&data2).unwrap_err();
        match err {
            MjDataError::SignatureMismatch { source, destination } => {
                assert_eq!(source, data1.signature());
                assert_eq!(destination, data2.signature());
            }
            other => panic!("expected SignatureMismatch, got {other:?}"),
        }

        let err = joint_info1.try_view_mut(&mut data2).unwrap_err();
        match err {
            MjDataError::SignatureMismatch { source, destination } => {
                assert_eq!(source, data1.signature());
                assert_eq!(destination, data2.signature());
            }
            other => panic!("expected SignatureMismatch, got {other:?}"),
        }
    }

    #[test]
    fn test_signature_match_physics_param_change() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body name='b1'><joint name='j1' type='free'/><geom size='0.1' mass='2'/></body></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let joint_info1 = data1.joint("j1").unwrap();

        // This should NOT panic because only physics parameters changed, the tree is the same
        let data2 = model2.make_data();
        let _view = joint_info1.view(&data2);
    }

    #[test]
    fn test_act_mixed_stateful_stateless() {
        // muscle at id=0 (stateful, actnum=1), motor at id=1 (stateless, actnum=0)
        // This tests mj_view_indices! with na path: actadr[0]=0, actadr[1]=-1
        // If bug exists: end_addr = (-1i32) as usize = usize::MAX -> overflow
        let xml = "<mujoco><option timestep=\"0.002\"/>\
<worldbody><body name=\"b\"><joint name=\"j1\" type=\"slide\" range=\"-1 1\" limited=\"true\"/>\
<joint name=\"j2\" type=\"slide\"/><geom size=\"0.1\" mass=\"1\"/></body></worldbody>\
<actuator><muscle name=\"m1\" joint=\"j1\" lengthrange=\"0 1\"/><motor name=\"m2\" joint=\"j2\"/></actuator></mujoco>";
        let model = MjModel::from_xml_string(xml).unwrap();
        let data = model.make_data();
        let actadr = model.actuator_actadr();
        let actnum = model.actuator_actnum();
        eprintln!("actadr[0]={} actadr[1]={}", actadr[0], actadr[1]);
        eprintln!("actnum[0]={} actnum[1]={}", actnum[0], actnum[1]);
        // muscle (m1) should have an act view with exactly actnum[0] elements
        let info_m1 = data.actuator("m1").unwrap();
        let view_m1 = info_m1.view(&data);
        let act_m1 = view_m1.act.as_ref().expect("muscle must have an act view (bug: overflow sets it to None/garbage)");
        assert_eq!(act_m1.len(), actnum[0] as usize,
            "muscle act len wrong: expected {} got {} (overflow bug?)", actnum[0], act_m1.len());
        // motor (m2) should have no act view
        let info_m2 = data.actuator("m2").unwrap();
        let view_m2 = info_m2.view(&data);
        assert!(view_m2.act.is_none(), "motor must have no act view");
    }

    /// Tests `mj_view_indices!` with mixed joint types (free/ball/slide),
    /// verifying that qpos and qvel view lengths match per-joint DOF counts.
    #[test]
    fn test_view_indices_mixed_joint_types() {
        const MIXED_MODEL: &str = "
<mujoco>
  <worldbody>
    <body name='b_free'>
      <joint name='j_free' type='free'/>
      <geom size='0.1' mass='1'/>
    </body>
    <body name='b_ball'>
      <joint name='j_ball' type='ball'/>
      <geom size='0.1' mass='1'/>
    </body>
    <body name='b_slide'>
      <joint name='j_slide' type='slide'/>
      <geom size='0.1' mass='1'/>
    </body>
  </worldbody>
</mujoco>";

        let model = MjModel::from_xml_string(MIXED_MODEL).unwrap();
        let data = model.make_data();

        // free: 7 qpos, 6 qvel; ball: 4 qpos, 3 qvel; slide: 1 qpos, 1 qvel
        let jfree = data.joint("j_free").unwrap();
        let jball = data.joint("j_ball").unwrap();
        let jslide = data.joint("j_slide").unwrap();

        let vfree = jfree.view(&data);
        let vball = jball.view(&data);
        let vslide = jslide.view(&data);

        assert_eq!(vfree.qpos.len(), 7);
        assert_eq!(vfree.qvel.len(), 6);
        assert_eq!(vball.qpos.len(), 4);
        assert_eq!(vball.qvel.len(), 3);
        assert_eq!(vslide.qpos.len(), 1);
        assert_eq!(vslide.qvel.len(), 1);

        // Total should equal model nq and nv
        assert_eq!(model.ffi().nq as usize, 7 + 4 + 1);
        assert_eq!(model.ffi().nv as usize, 6 + 3 + 1);
    }

    /// Tests `info_method!` stride correctness for body data views:
    /// xpos=3, xmat=9, xquat=4, cinert=10, cvel=6.
    #[test]
    fn test_body_data_view_stride_lengths() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let data = model.make_data();

        let ball = data.body("ball").unwrap();
        let ball2 = data.body("ball2").unwrap();

        let v1 = ball.view(&data);
        let v2 = ball2.view(&data);

        // Stride correctness
        assert_eq!(v1.xpos.len(), 3);
        assert_eq!(v1.xmat.len(), 9);
        assert_eq!(v1.xquat.len(), 4);
        assert_eq!(v1.cinert.len(), 10);
        assert_eq!(v1.cvel.len(), 6);

        // Non-aliasing: different bodies must have distinct slices
        assert_ne!(v1.xpos.as_ptr(), v2.xpos.as_ptr());
        assert_ne!(v1.cvel.as_ptr(), v2.cvel.as_ptr());
    }

    /// Tests `getter_setter!` for MjtNum time: set, get, and builder roundtrip.
    #[test]
    fn test_time_getter_setter_roundtrip() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        assert_relative_eq!(data.time(), 0.0, epsilon = 1e-15);

        data.set_time(3.14);
        assert_relative_eq!(data.time(), 3.14, epsilon = 1e-15);

        let data2 = model.make_data().with_time(2.718);
        assert_relative_eq!(data2.time(), 2.718, epsilon = 1e-15);
    }

    /// Tests that `jac` returns `IndexOutOfBounds` for invalid body IDs.
    #[test]
    fn test_jac_invalid_body_id() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let data = model.make_data();
        let point = [0.0; 3];

        // Negative ID
        let err = data.jac(true, true, &point, -1).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "body_id", .. }));

        // Too-large ID
        let err = data.jac(true, true, &point, 9999).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "body_id", .. }));
    }

    /// Tests that `object_velocity` returns `UnsupportedObjectType` for unsupported types
    /// and `IndexOutOfBounds` for out-of-range IDs.
    #[test]
    fn test_object_velocity_error_paths() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let data = model.make_data();

        // Unsupported type (mjOBJ_JOINT is not in the match arms)
        let err = data.object_velocity(MjtObj::mjOBJ_JOINT, 0, false).unwrap_err();
        assert!(matches!(err, MjDataError::UnsupportedObjectType(_)));

        // Out-of-range body ID
        let err = data.object_velocity(MjtObj::mjOBJ_BODY, 9999, false).unwrap_err();
        assert!(matches!(err, MjDataError::IndexOutOfBounds { kind: "obj_id", .. }));
    }

    /// Tests that state flags select only their subset: setting QPOS must not clobber qvel.
    #[test]
    fn test_state_spec_flags_select_subsets() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Give data some known non-zero qvel
        let jinfo = data.joint("ball").unwrap();
        jinfo.view_mut(&mut data).qvel[0] = 42.0;
        let original_qvel0 = jinfo.view(&data).qvel[0];

        // Now overwrite only QPOS from a fresh data instance
        let fresh = model.make_data();
        data.copy_state_from_data(&fresh, MjtState::mjSTATE_QPOS as u32);

        // qvel should be untouched
        assert_relative_eq!(jinfo.view(&data).qvel[0], original_qvel0, epsilon = 1e-15);
    }

    /// Tests `try_copy_state_from_data` returns `SignatureMismatch` for mismatched models.
    #[test]
    fn test_try_copy_state_signature_mismatch() {
        let model1 = MjModel::from_xml_string("<mujoco><worldbody><body><joint type='free'/><geom size='0.1'/></body></worldbody></mujoco>").unwrap();
        let model2 = MjModel::from_xml_string("<mujoco><worldbody><body><joint type='slide'/><geom size='0.1'/></body></worldbody></mujoco>").unwrap();

        let data1 = model1.make_data();
        let mut data2 = model2.make_data();

        let err = data2.try_copy_state_from_data(&data1, MjtState::mjSTATE_FULLPHYSICS as u32).unwrap_err();
        match err {
            MjDataError::SignatureMismatch { source, destination } => {
                assert_ne!(source, destination);
            }
            other => panic!("expected SignatureMismatch, got {:?}", other),
        }
    }

    /// Tests `copy_state_from_data` with full physics: time, qpos, qvel all match.
    #[test]
    fn test_copy_state_full_physics() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data1 = model.make_data();
        let mut data2 = model.make_data();

        // Evolve data1
        data1.set_time(1.0);
        data1.joint("ball").unwrap().view_mut(&mut data1).qpos[0] = 5.0;
        data1.joint("ball").unwrap().view_mut(&mut data1).qvel[0] = 3.0;

        data2.copy_state_from_data(&data1, MjtState::mjSTATE_FULLPHYSICS as u32);

        assert_relative_eq!(data2.time(), 1.0, epsilon = 1e-15);
        assert_relative_eq!(data2.qpos()[0], 5.0, epsilon = 1e-15);
        assert_relative_eq!(data2.qvel()[0], 3.0, epsilon = 1e-15);
    }

    /**************************************************************************/
    // Force-cast macro correctness tests
    /**************************************************************************/

    /// A richer model for force-cast tests: free joint, slide joint, equalities,
    /// mocap body, tendon, multiple geom types, sensors, contacts.
    const FORCE_MODEL: &str = "
<mujoco>
  <worldbody>
    <body name='b_free' pos='1 2 3'>
        <joint name='j_free' type='free'/>
        <geom name='g_sphere' type='sphere' size='0.1' mass='1'/>
    </body>

    <body name='b_slide' pos='0 0 5'>
        <joint name='j_slide' type='slide' axis='0 0 1' range='-1 1' limited='true'/>
        <geom name='g_box' type='box' size='0.1 0.2 0.3' mass='1'/>
        <site name='s1' pos='0 0 0' size='0.05'/>
    </body>

    <body name='b_hinge' pos='0 5 0'>
        <joint name='j_hinge' type='hinge' axis='0 1 0'/>
        <geom name='g_capsule' type='capsule' size='0.1 0.5' mass='1'/>
        <site name='s2' pos='0 0 0' size='0.05'/>
    </body>

    <body name='mocap_body' mocap='true' pos='10 10 10'>
        <geom name='g_mocap' type='sphere' size='0.01' contype='0' conaffinity='0'/>
    </body>

    <geom name='floor' type='plane' size='50 50 1'/>
  </worldbody>

  <equality>
      <connect name='eq1' body1='b_slide' body2='b_hinge' anchor='0 0 0'/>
      <connect name='eq2' body1='b_hinge' body2='b_slide' anchor='1 2 3'/>
  </equality>

  <tendon>
      <spatial name='ten1'>
          <site site='s1'/>
          <site site='s2'/>
      </spatial>
  </tendon>

  <actuator>
      <motor name='motor_slide' joint='j_slide'/>
  </actuator>

  <sensor>
      <touch name='touch_sensor' site='s1'/>
  </sensor>
</mujoco>";

    /// Verifies [force]-cast array grouping: xpos returns &[[MjtNum; 3]]
    /// with the correct number of elements and matching raw FFI data.
    #[test]
    fn test_force_cast_xpos_array_grouping() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let xpos = data.xpos();

        // The force-cast from *mut f64 -> *mut [MjtNum; 3] must produce
        // exactly nbody elements of [f64; 3].
        assert_eq!(xpos.len(), nbody, "xpos slice len must equal nbody");

        // Cross-validate every element against the raw FFI pointer.
        for i in 0..nbody {
            for j in 0..3 {
                let ffi_val = unsafe { *data.ffi().xpos.add(i * 3 + j) };
                assert_eq!(xpos[i][j], ffi_val,
                    "xpos[{}][{}] mismatch: slice={} ffi={}", i, j, xpos[i][j], ffi_val);
            }
        }
    }

    /// Verifies [force]-cast for xmat (&[[MjtNum; 9]]) and xquat (&[[MjtNum; 4]]).
    #[test]
    fn test_force_cast_xmat_xquat_grouping() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let xmat = data.xmat();
        let xquat = data.xquat();

        assert_eq!(xmat.len(), nbody);
        assert_eq!(xquat.len(), nbody);

        // xmat stride = 9
        for i in 0..nbody {
            for j in 0..9 {
                let ffi_val = unsafe { *data.ffi().xmat.add(i * 9 + j) };
                assert_eq!(xmat[i][j], ffi_val,
                    "xmat[{}][{}] mismatch", i, j);
            }
        }

        // xquat stride = 4
        for i in 0..nbody {
            for j in 0..4 {
                let ffi_val = unsafe { *data.ffi().xquat.add(i * 4 + j) };
                assert_eq!(xquat[i][j], ffi_val,
                    "xquat[{}][{}] mismatch", i, j);
            }
        }
    }

    /// Verifies [force]-cast for cinert (&[[MjtNum; 10]]) - the widest array grouping.
    #[test]
    fn test_force_cast_cinert_10_element_grouping() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let cinert = data.cinert();
        assert_eq!(cinert.len(), nbody);

        for i in 0..nbody {
            for j in 0..10 {
                let ffi_val = unsafe { *data.ffi().cinert.add(i * 10 + j) };
                assert_eq!(cinert[i][j], ffi_val,
                    "cinert[{}][{}] mismatch", i, j);
            }
        }
    }

    /// Verifies [force]-cast for cvel (&[[MjtNum; 6]]) and xfrc_applied (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_6_element_groupings() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;

        // cvel: [MjtNum; 6]
        let cvel = data.cvel();
        assert_eq!(cvel.len(), nbody);
        for i in 0..nbody {
            for j in 0..6 {
                let ffi_val = unsafe { *data.ffi().cvel.add(i * 6 + j) };
                assert_eq!(cvel[i][j], ffi_val, "cvel[{}][{}] mismatch", i, j);
            }
        }

        // xfrc_applied: [MjtNum; 6]
        let xfrc = data.xfrc_applied();
        assert_eq!(xfrc.len(), nbody);
        for i in 0..nbody {
            for j in 0..6 {
                let ffi_val = unsafe { *data.ffi().xfrc_applied.add(i * 6 + j) };
                assert_eq!(xfrc[i][j], ffi_val, "xfrc_applied[{}][{}] mismatch", i, j);
            }
        }
    }

    /// Verifies [force]-cast for mocap_pos (&[[MjtNum; 3]]) and mocap_quat (&[[MjtNum; 4]]).
    #[test]
    fn test_force_cast_mocap_arrays() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let data = model.make_data();

        let nmocap = model.ffi().nmocap as usize;
        assert!(nmocap > 0, "test model must have at least one mocap body");

        let mocap_pos = data.mocap_pos();
        let mocap_quat = data.mocap_quat();

        assert_eq!(mocap_pos.len(), nmocap);
        assert_eq!(mocap_quat.len(), nmocap);

        // The mocap body was placed at pos='10 10 10'
        assert_relative_eq!(mocap_pos[0][0], 10.0, epsilon = 1e-9);
        assert_relative_eq!(mocap_pos[0][1], 10.0, epsilon = 1e-9);
        assert_relative_eq!(mocap_pos[0][2], 10.0, epsilon = 1e-9);

        // Default quaternion is identity [1, 0, 0, 0]
        assert_relative_eq!(mocap_quat[0][0], 1.0, epsilon = 1e-9);
        assert_relative_eq!(mocap_quat[0][1], 0.0, epsilon = 1e-9);
        assert_relative_eq!(mocap_quat[0][2], 0.0, epsilon = 1e-9);
        assert_relative_eq!(mocap_quat[0][3], 0.0, epsilon = 1e-9);

        // Cross-validate with FFI
        for i in 0..nmocap {
            for j in 0..3 {
                assert_eq!(mocap_pos[i][j], unsafe { *data.ffi().mocap_pos.add(i * 3 + j) });
            }
            for j in 0..4 {
                assert_eq!(mocap_quat[i][j], unsafe { *data.ffi().mocap_quat.add(i * 4 + j) });
            }
        }
    }

    /// Verifies [force]-cast bool conversion: eq_active (*mut u8 -> *mut bool).
    #[test]
    fn test_force_cast_eq_active_bool() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let neq = model.ffi().neq as usize;
        assert_eq!(neq, 2, "test model must have exactly 2 equality constraints");

        // Equality constraints are active by default
        let eq_active = data.eq_active();
        assert_eq!(eq_active.len(), neq);
        assert_eq!(eq_active[0], true);
        assert_eq!(eq_active[1], true);

        // Cross-validate with raw u8 FFI pointer
        for i in 0..neq {
            let raw_val = unsafe { *data.ffi().eq_active.add(i) };
            assert_eq!(eq_active[i], raw_val != 0,
                "eq_active[{}]: bool={} raw_u8={}", i, eq_active[i], raw_val);
        }

        // Disable one via mutable force-cast
        data.eq_active_mut()[0] = false;
        assert_eq!(data.eq_active()[0], false);
        assert_eq!(data.eq_active()[1], true);

        // Verify FFI side was actually modified
        let raw_val = unsafe { *data.ffi().eq_active.add(0) };
        assert_eq!(raw_val, 0u8, "disabling eq_active[0] must write 0 to FFI");
    }

    /// Verifies [force]-cast mutable roundtrip for xfrc_applied and mocap_pos.
    #[test]
    fn test_force_cast_mutable_roundtrip() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let nbody = model.ffi().nbody as usize;
        assert!(nbody > 1);

        // Write a known pattern into xfrc_applied via mutable force-cast
        let body_idx = 1; // first non-world body
        data.xfrc_applied_mut()[body_idx] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];

        // Read back via immutable force-cast
        let xfrc = data.xfrc_applied();
        assert_eq!(xfrc[body_idx], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        // Verify all 6 elements individually against raw FFI
        for j in 0..6 {
            let ffi_val = unsafe { *data.ffi().xfrc_applied.add(body_idx * 6 + j) };
            assert_eq!(ffi_val, (j + 1) as f64,
                "xfrc_applied FFI[{}] mismatch", j);
        }

        // Mocap pos write/read roundtrip
        let nmocap = model.ffi().nmocap as usize;
        if nmocap > 0 {
            data.mocap_pos_mut()[0] = [99.0, 88.0, 77.0];
            assert_eq!(data.mocap_pos()[0], [99.0, 88.0, 77.0]);
            for j in 0..3 {
                let ffi_val = unsafe { *data.ffi().mocap_pos.add(j) };
                assert_eq!(ffi_val, [99.0, 88.0, 77.0][j]);
            }
        }
    }

    /// Verifies force-cast for body view fields (xpos, xquat, xmat, cinert, cvel)
    /// have the correct stride and match FFI data.
    #[test]
    fn test_force_cast_body_view_strides_and_values() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let body_info = data.body("b_free").unwrap();
        let view = body_info.view(&data);

        // Stride correctness (all derived from [force] grouping)
        assert_eq!(view.xpos.len(), 3);
        assert_eq!(view.xquat.len(), 4);
        assert_eq!(view.xmat.len(), 9);
        assert_eq!(view.xipos.len(), 3);
        assert_eq!(view.ximat.len(), 9);
        assert_eq!(view.cinert.len(), 10);
        assert_eq!(view.cvel.len(), 6);
        assert_eq!(view.xfrc_applied.len(), 6);
        assert_eq!(view.crb.len(), 10);
        assert_eq!(view.subtree_com.len(), 3);
        assert_eq!(view.subtree_linvel.len(), 3);
        assert_eq!(view.subtree_angmom.len(), 3);
        assert_eq!(view.cacc.len(), 6);
        assert_eq!(view.cfrc_int.len(), 6);
        assert_eq!(view.cfrc_ext.len(), 6);

        // The body was placed at pos='1 2 3' with a free joint;
        // after forward(), xpos should reflect position from qpos
        let body_id = body_info.id;
        for j in 0..3 {
            let ffi_val = unsafe { *data.ffi().xpos.add(body_id * 3 + j) };
            assert_eq!(view.xpos[j], ffi_val,
                "body view xpos[{}] must match FFI xpos", j);
        }
    }

    /// Verifies that the body data view for the world body (id=0) works correctly.
    /// Edge case: world body has special status in MuJoCo.
    #[test]
    fn test_force_cast_world_body_view() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        // The world body (id=0) has name "world" in MuJoCo >= 3.x
        let world_info = data.body("world").unwrap();
        assert_eq!(world_info.id, 0);
        let view = world_info.view(&data);

        // World body xpos = [0, 0, 0]
        assert_eq!(view.xpos[..], [0.0, 0.0, 0.0]);
        // World body xquat = [1, 0, 0, 0] (identity)
        assert_relative_eq!(view.xquat[0], 1.0, epsilon = 1e-9);
        assert_relative_eq!(view.xquat[1], 0.0, epsilon = 1e-9);
        assert_relative_eq!(view.xquat[2], 0.0, epsilon = 1e-9);
        assert_relative_eq!(view.xquat[3], 0.0, epsilon = 1e-9);
        // Stride must still be correct
        assert_eq!(view.xmat.len(), 9);
        assert_eq!(view.cinert.len(), 10);
    }

    /// Verifies mutable force-cast view roundtrip for body xfrc_applied.
    #[test]
    fn test_force_cast_body_view_mut_roundtrip() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let body_info = data.body("b_free").unwrap();
        let body_id = body_info.id;

        // Write via view_mut
        body_info.view_mut(&mut data).xfrc_applied.copy_from_slice(&[10.0, 20.0, 30.0, 40.0, 50.0, 60.0]);

        // Read back via view
        let view = body_info.view(&data);
        assert_eq!(&view.xfrc_applied[..], &[10.0, 20.0, 30.0, 40.0, 50.0, 60.0]);

        // Read back via flat slice
        assert_eq!(data.xfrc_applied()[body_id], [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]);

        // Read back via FFI
        for j in 0..6 {
            let ffi_val = unsafe { *data.ffi().xfrc_applied.add(body_id * 6 + j) };
            assert_eq!(ffi_val, ((j + 1) * 10) as f64);
        }
    }

    /// Verifies [force]-cast camera/light/geom/site xpos+xmat grouping in data.
    #[test]
    fn test_force_cast_geom_site_cam_light_data() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        // Geom xpos: [MjtNum; 3], xmat: [MjtNum; 9]
        let ngeom = model.ffi().ngeom as usize;
        assert_eq!(data.geom_xpos().len(), ngeom);
        assert_eq!(data.geom_xmat().len(), ngeom);
        for i in 0..ngeom {
            for j in 0..3 {
                assert_eq!(data.geom_xpos()[i][j],
                    unsafe { *data.ffi().geom_xpos.add(i * 3 + j) });
            }
            for j in 0..9 {
                assert_eq!(data.geom_xmat()[i][j],
                    unsafe { *data.ffi().geom_xmat.add(i * 9 + j) });
            }
        }

        // Site xpos: [MjtNum; 3], xmat: [MjtNum; 9]
        let nsite = model.ffi().nsite as usize;
        assert_eq!(data.site_xpos().len(), nsite);
        assert_eq!(data.site_xmat().len(), nsite);
        for i in 0..nsite {
            for j in 0..3 {
                assert_eq!(data.site_xpos()[i][j],
                    unsafe { *data.ffi().site_xpos.add(i * 3 + j) });
            }
        }

        // Cam xpos: [MjtNum; 3], xmat: [MjtNum; 9]
        let ncam = model.ffi().ncam as usize;
        assert_eq!(data.cam_xpos().len(), ncam);
        assert_eq!(data.cam_xmat().len(), ncam);

        // Light xpos: [MjtNum; 3], xdir: [MjtNum; 3]
        let nlight = model.ffi().nlight as usize;
        assert_eq!(data.light_xpos().len(), nlight);
        assert_eq!(data.light_xdir().len(), nlight);
    }

    /// Verifies [force]-cast for joint anchor/axis: xanchor (&[[MjtNum; 3]]), xaxis (&[[MjtNum; 3]]).
    #[test]
    fn test_force_cast_joint_anchor_axis() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let njnt = model.ffi().njnt as usize;
        let xanchor = data.xanchor();
        let xaxis = data.xaxis();

        assert_eq!(xanchor.len(), njnt);
        assert_eq!(xaxis.len(), njnt);

        for i in 0..njnt {
            for j in 0..3 {
                assert_eq!(xanchor[i][j], unsafe { *data.ffi().xanchor.add(i * 3 + j) });
                assert_eq!(xaxis[i][j], unsafe { *data.ffi().xaxis.add(i * 3 + j) });
            }
        }
    }

    /// Verifies [force]-cast for cdof (&[[MjtNum; 6]]) and cdof_dot (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_cdof_6_element() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nv = model.ffi().nv as usize;
        let cdof = data.cdof();
        let cdof_dot = data.cdof_dot();

        assert_eq!(cdof.len(), nv);
        assert_eq!(cdof_dot.len(), nv);

        for i in 0..nv {
            for j in 0..6 {
                assert_eq!(cdof[i][j], unsafe { *data.ffi().cdof.add(i * 6 + j) });
                assert_eq!(cdof_dot[i][j], unsafe { *data.ffi().cdof_dot.add(i * 6 + j) });
            }
        }
    }

    /// Verifies [force]-cast for efc_KBIP (&[[MjtNum; 4]]).
    /// Requires contacts to produce constraint data.
    #[test]
    fn test_force_cast_efc_kbip_4_element() {
        // Use the main MODEL which has balls falling onto a floor -> contacts
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Step a few times to generate contacts
        for _ in 0..10 {
            data.step();
        }

        let nefc = data.ffi().nefc as usize;
        if nefc > 0 {
            let efc_kbip = data.efc_kbip();
            assert_eq!(efc_kbip.len(), nefc);
            for i in 0..nefc {
                for j in 0..4 {
                    assert_eq!(efc_kbip[i][j], unsafe { *data.ffi().efc_KBIP.add(i * 4 + j) });
                }
            }
        }
    }

    /// Verifies [force]-cast enum: efc_type (*mut i32 -> *mut MjtConstraint).
    #[test]
    fn test_force_cast_efc_type_enum() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        for _ in 0..10 {
            data.step();
        }

        let nefc = data.ffi().nefc as usize;
        if nefc > 0 {
            let efc_type = data.efc_type();
            assert_eq!(efc_type.len(), nefc);

            for i in 0..nefc {
                let raw_i32 = unsafe { *data.ffi().efc_type.add(i) };
                let expected: MjtConstraint = unsafe { crate::util::force_cast(raw_i32) };
                assert_eq!(efc_type[i], expected,
                    "efc_type[{}]: got {:?}, expected {:?} (raw={})", i, efc_type[i], expected, raw_i32);
            }
        }
    }

    /// Verifies [force]-cast enum: efc_state (*mut i32 -> *mut MjtConstraintState).
    #[test]
    fn test_force_cast_efc_state_enum() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nefc = data.ffi().nefc as usize;
        if nefc > 0 {
            let efc_state = data.efc_state();
            assert_eq!(efc_state.len(), nefc);

            for i in 0..nefc {
                let raw_i32 = unsafe { *data.ffi().efc_state.add(i) };
                let expected: MjtConstraintState = unsafe { crate::util::force_cast(raw_i32) };
                assert_eq!(efc_state[i], expected);
            }
        }
    }

    /// Verifies [force]-cast enum: body_awake (*mut i32 -> *mut MjtSleepState).
    #[test]
    fn test_force_cast_body_awake_enum() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let body_awake = data.body_awake();
        assert_eq!(body_awake.len(), nbody);

        for i in 0..nbody {
            let raw_i32 = unsafe { *data.ffi().body_awake.add(i) };
            let expected: MjtSleepState = unsafe { crate::util::force_cast(raw_i32) };
            assert_eq!(body_awake[i], expected,
                "body_awake[{}] mismatch", i);
        }
    }

    /// Verifies [force]-cast for bvh_active (*mut u8 -> *mut bool) and that
    /// the result is valid (true or false, no garbage).
    #[test]
    fn test_force_cast_bvh_active_bool() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        // Step to populate bvh
        data.step();

        let nbvh = model.ffi().nbvh as usize;
        let bvh_active = data.bvh_active();
        assert_eq!(bvh_active.len(), nbvh);

        for i in 0..nbvh {
            let raw_u8 = unsafe { *data.ffi().bvh_active.add(i) };
            assert!(raw_u8 == 0 || raw_u8 == 1,
                "raw bvh_active[{}]={} must be 0 or 1", i, raw_u8);
            assert_eq!(bvh_active[i], raw_u8 != 0);
        }
    }

    /// Verifies [force]-cast for wrap_obj (&[[i32; 2]]) and wrap_xpos (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_wrap_arrays() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nwrap = model.ffi().nwrap as usize;
        let wrap_obj = data.wrap_obj();
        let wrap_xpos = data.wrap_xpos();

        assert_eq!(wrap_obj.len(), nwrap);
        assert_eq!(wrap_xpos.len(), nwrap);

        for i in 0..nwrap {
            for j in 0..2 {
                assert_eq!(wrap_obj[i][j], unsafe { *data.ffi().wrap_obj.add(i * 2 + j) });
            }
            for j in 0..6 {
                assert_eq!(wrap_xpos[i][j], unsafe { *data.ffi().wrap_xpos.add(i * 6 + j) });
            }
        }
    }

    /// Verifies [force]-cast for flexvert_J (&[[MjtNum; 2]]) and flexvert_length (&[[MjtNum; 2]]).
    /// In a model with no flexes, these should return empty slices.
    #[test]
    fn test_force_cast_flex_empty_slices() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let data = model.make_data();

        // Model has no flexes, so these should be empty
        assert_eq!(data.flexvert_xpos().len(), 0, "no flex -> empty flexvert_xpos");
        assert_eq!(data.flexelem_aabb().len(), 0, "no flex -> empty flexelem_aabb");
        assert_eq!(data.flexvert_j().len(), 0, "no flex -> empty flexvert_J");
        assert_eq!(data.flexvert_length().len(), 0, "no flex -> empty flexvert_length");
        assert_eq!(data.flexedge_j().len(), 0, "no flex -> empty flexedge_J");
        assert_eq!(data.flexedge_length().len(), 0, "no flex -> empty flexedge_length");
    }

    /// Verifies [force]-cast for bvh_aabb_dyn (&[[MjtNum; 6]]).
    #[test]
    fn test_force_cast_bvh_aabb_dyn() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbvhdyn = model.ffi().nbvhdynamic as usize;
        let aabb_dyn = data.bvh_aabb_dyn();
        assert_eq!(aabb_dyn.len(), nbvhdyn);

        for i in 0..nbvhdyn {
            for j in 0..6 {
                assert_eq!(aabb_dyn[i][j], unsafe { *data.ffi().bvh_aabb_dyn.add(i * 6 + j) });
            }
        }
    }

    /// Verifies [force]-cast for subtree arrays: subtree_com, subtree_linvel, subtree_angmom
    /// all with stride 3.
    #[test]
    fn test_force_cast_subtree_3_element() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;

        let subtree_com = data.subtree_com();
        let subtree_linvel = data.subtree_linvel();
        let subtree_angmom = data.subtree_angmom();

        assert_eq!(subtree_com.len(), nbody);
        assert_eq!(subtree_linvel.len(), nbody);
        assert_eq!(subtree_angmom.len(), nbody);

        for i in 0..nbody {
            for j in 0..3 {
                assert_eq!(subtree_com[i][j], unsafe { *data.ffi().subtree_com.add(i * 3 + j) });
            }
        }
    }

    /// Verifies [force]-cast consistency: body view xfrc_applied matches flat slice.
    /// This catches any stride/offset mismatch between view_creator! and array_slice_dyn!.
    #[test]
    fn test_force_cast_view_vs_flat_slice_consistency() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        // Check every named body: view xpos must equal flat xpos slice
        let body_names = ["world", "b_free", "b_slide", "b_hinge", "mocap_body"];
        for name in &body_names {
            let info = data.body(name).unwrap();
            let id = info.id;
            let view = info.view(&data);
            let flat = data.xpos();

            for j in 0..3 {
                assert_eq!(view.xpos[j], flat[id][j],
                    "body {} xpos[{}]: view={} flat={}", id, j, view.xpos[j], flat[id][j]);
            }

            for j in 0..4 {
                assert_eq!(view.xquat[j], data.xquat()[id][j],
                    "body {} xquat[{}] mismatch", id, j);
            }

            for j in 0..10 {
                assert_eq!(view.cinert[j], data.cinert()[id][j],
                    "body {} cinert[{}] mismatch", id, j);
            }
        }
    }

    /// Verifies [force]-cast for the cacc/cfrc_int/cfrc_ext body arrays (stride 6).
    #[test]
    fn test_force_cast_body_cfrc_arrays() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let cacc = data.cacc();
        let cfrc_int = data.cfrc_int();
        let cfrc_ext = data.cfrc_ext();

        assert_eq!(cacc.len(), nbody);
        assert_eq!(cfrc_int.len(), nbody);
        assert_eq!(cfrc_ext.len(), nbody);

        for i in 0..nbody {
            for j in 0..6 {
                assert_eq!(cacc[i][j], unsafe { *data.ffi().cacc.add(i * 6 + j) });
                assert_eq!(cfrc_int[i][j], unsafe { *data.ffi().cfrc_int.add(i * 6 + j) });
                assert_eq!(cfrc_ext[i][j], unsafe { *data.ffi().cfrc_ext.add(i * 6 + j) });
            }
        }
    }

    /// Verifies [force]-cast for crb (&[[MjtNum; 10]]).
    #[test]
    fn test_force_cast_crb_10_element() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let crb = data.crb();
        assert_eq!(crb.len(), nbody);

        for i in 0..nbody {
            for j in 0..10 {
                assert_eq!(crb[i][j], unsafe { *data.ffi().crb.add(i * 10 + j) });
            }
        }
    }

    /// Minimal model test: a model with zero equalities, zero tendons, zero actuators
    /// should produce empty slices for all force-cast fields that depend on those counts.
    #[test]
    fn test_force_cast_empty_model_edge_case() {
        let xml = "<mujoco><worldbody><body><joint type='free'/><geom size='0.1'/></body></worldbody></mujoco>";
        let model = MjModel::from_xml_string(xml).unwrap();
        let data = model.make_data();

        assert_eq!(model.ffi().neq, 0);
        assert_eq!(model.ffi().nmocap, 0);
        assert_eq!(model.ffi().ntendon, 0);

        // Empty force-cast slices
        assert!(data.eq_active().is_empty());
        assert!(data.mocap_pos().is_empty());
        assert!(data.mocap_quat().is_empty());
        assert!(data.wrap_obj().is_empty());
        assert!(data.wrap_xpos().is_empty());
        assert!(data.ten_j().is_empty());
        assert!(model.ten_j_colind().is_empty());
        assert!(model.ten_j_rownnz().is_empty());
        assert!(model.ten_j_rowadr().is_empty());
        assert_eq!(model.ffi().nJten, 0);

        // But body arrays should still work (nbody >= 1 always: world body)
        let nbody = model.ffi().nbody as usize;
        assert!(nbody >= 2); // world + one body
        assert_eq!(data.xpos().len(), nbody);
        assert_eq!(data.xquat().len(), nbody);
        assert_eq!(data.cinert().len(), nbody);
    }

    /// Sparse ten_J and model sparsity fields cross-validated against raw FFI pointers.
    #[test]
    fn test_sparse_ten_j() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let ntendon = model.ffi().ntendon as usize;
        let njten = model.ffi().nJten as usize;
        assert!(ntendon > 0);
        assert!(njten > 0);

        let ten_j = data.ten_j();
        let ten_j_colind = model.ten_j_colind();
        assert_eq!(ten_j.len(), njten);
        assert_eq!(ten_j_colind.len(), njten);
        assert_eq!(model.ten_j_rownnz().len(), ntendon);
        assert_eq!(model.ten_j_rowadr().len(), ntendon);

        for i in 0..njten {
            assert_eq!(ten_j[i], unsafe { *data.ffi().ten_J.add(i) });
            assert_eq!(ten_j_colind[i], unsafe { *model.ffi().ten_J_colind.add(i) });
        }
    }

    /// Checks sparse ten_J against `dir^T * (J_s2 - J_s1)` from jac_site at the
    /// default pose and after 50 gravity steps. Both flat and view APIs are tested.
    #[test]
    fn test_ten_j_vs_jac_site() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        let nv = model.nv() as usize;

        let s1_id = model.name_to_id(MjtObj::mjOBJ_SITE, "s1").unwrap();
        let s2_id = model.name_to_id(MjtObj::mjOBJ_SITE, "s2").unwrap();

        for config in 0..2 {
            if config == 0 {
                data.forward();
            } else {
                for _ in 0..50 { data.step(); }
            }

            let ten_j = data.ten_j();
            let rownnz = model.ten_j_rownnz();
            let rowadr = model.ten_j_rowadr();
            let colind = model.ten_j_colind();

            let (jacp_s1, _) = data.jac_site(true, false, s1_id).unwrap();
            let (jacp_s2, _) = data.jac_site(true, false, s2_id).unwrap();

            let p1 = data.site_xpos()[s1_id as usize];
            let p2 = data.site_xpos()[s2_id as usize];
            let diff = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
            let length = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]).sqrt();
            assert!(length > 1e-10);
            let dir = [diff[0] / length, diff[1] / length, diff[2] / length];

            // J_ten[j] = sum_k dir[k] * (J_s2[k,j] - J_s1[k,j])
            let mut expected = vec![0.0 as MjtNum; nv];
            for j in 0..nv {
                for k in 0..3 {
                    expected[j] += dir[k] * (jacp_s2[k * nv + j] - jacp_s1[k * nv + j]);
                }
            }

            // Sparse -> dense
            let nnz = rownnz[0] as usize;
            let adr = rowadr[0] as usize;
            assert!(nnz > 0);
            let mut actual = vec![0.0 as MjtNum; nv];
            for k in 0..nnz {
                actual[colind[adr + k] as usize] = ten_j[adr + k];
            }

            let max_abs = actual.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
            assert!(max_abs > 0.1, "config {config}: max |J| = {max_abs}");
            for j in 0..nv {
                assert_relative_eq!(actual[j], expected[j], epsilon = 1e-10);
            }

            // Same check via view API
            let ten_view = data.tendon("ten1").unwrap().view(&data);
            let model_view = model.tendon("ten1").unwrap().view(&model);
            let view_nnz = model_view.J_rownnz[0] as usize;
            let mut view_dense = vec![0.0 as MjtNum; nv];
            for k in 0..view_nnz {
                view_dense[model_view.J_colind[k] as usize] = ten_view.J[k];
            }
            for j in 0..nv {
                assert_relative_eq!(view_dense[j], expected[j], epsilon = 1e-10);
            }

            // Tendon length: view == flat == Euclidean distance
            let ten_info = data.tendon("ten1").unwrap();
            assert_relative_eq!(ten_view.length[0], data.ten_length()[ten_info.id], epsilon = 1e-15);
            assert_relative_eq!(ten_view.length[0], length, epsilon = 1e-10);
        }
    }

    /// Tendon data view's J matches the flat ten_j() sparse array; model view's
    /// J_rownnz/J_rowadr/J_colind match their flat counterparts.
    #[test]
    fn test_tendon_view_j_fields() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let flat_j = data.ten_j();
        let flat_rownnz = model.ten_j_rownnz();
        let flat_rowadr = model.ten_j_rowadr();
        let flat_colind = model.ten_j_colind();

        let ten_info = data.tendon("ten1").unwrap();
        let ten_view = ten_info.view(&data);
        let nnz = flat_rownnz[ten_info.id] as usize;
        let adr = flat_rowadr[ten_info.id] as usize;
        assert_eq!(ten_view.J.len(), nnz);

        let mut any_nonzero = false;
        for k in 0..nnz {
            assert_eq!(ten_view.J[k], flat_j[adr + k]);
            any_nonzero |= ten_view.J[k].abs() > 1e-12;
        }
        assert!(any_nonzero);

        let model_info = model.tendon("ten1").unwrap();
        let model_view = model_info.view(&model);
        assert_eq!(model_view.J_rownnz[0], flat_rownnz[model_info.id]);
        assert_eq!(model_view.J_rowadr[0], flat_rowadr[model_info.id]);
        assert_eq!(model_view.J_colind.len(), nnz);
        for k in 0..nnz {
            assert_eq!(model_view.J_colind[k], flat_colind[adr + k]);
        }
    }

    /// Read-only fields in mutable tendon views can only be mutated via explicit unsafe API.
    #[test]
    fn test_tendon_data_view_ro_field_unsafe_mutation_api() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let tendon_info = data.tendon("ten1").unwrap();
        let tendon_id = tendon_info.id;
        let original;

        {
            let tendon_view = tendon_info.view(&data);
            assert!(!tendon_view.wrapadr.is_empty(), "expected non-empty tendon wrapadr for FORCE_MODEL::ten1");
            original = tendon_view.wrapadr[0];
        }

        let temporary = if original == i32::MAX { i32::MIN } else { original + 1 };
        assert_ne!(temporary, original);

        // SAFETY: This intentionally exercises the explicit unsafe mutation
        // entrypoint and validates the write via independent flat accessors.
        {
            let mut tendon_view_mut = tendon_info.view_mut(&mut data);
            unsafe {
                tendon_view_mut.wrapadr.as_mut_slice()[0] = temporary;
            }
        }
        assert_eq!(data.ten_wrapadr()[tendon_id], temporary);

        // SAFETY: Restore original value before any further simulation use.
        {
            let mut tendon_view_mut = tendon_info.view_mut(&mut data);
            unsafe {
                tendon_view_mut.wrapadr.as_mut_slice()[0] = original;
            }
        }

        assert_eq!(data.ten_wrapadr()[tendon_id], original);
    }

    /// Checks `ten_velocity == J_ten @ qvel` with several qvel patterns across
    /// evolving simulation states, via both flat and view APIs.
    #[test]
    fn test_ten_j_velocity_transform() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        let ntendon = model.ntendon() as usize;
        assert!(ntendon > 0);

        // nv=8 in FORCE_MODEL: free(6) + slide(1) + hinge(1)
        let qvel_patterns: &[&[MjtNum]] = &[
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, -0.7],
            &[0.1, -0.2, 0.3, 0.5, -0.1, 0.4, 0.0, 0.0],
            &[0.5, 0.0, -0.3, 0.0, 0.0, 0.0, 2.0, 1.0],
        ];

        for (round, &qv) in qvel_patterns.iter().enumerate() {
            if round == 0 {
                data.forward();
            } else {
                for _ in 0..20 { data.step(); }
            }

            data.qvel_mut().copy_from_slice(qv);
            data.forward();

            let ten_j = data.ten_j();
            let rownnz = model.ten_j_rownnz();
            let rowadr = model.ten_j_rowadr();
            let colind = model.ten_j_colind();
            let qvel = data.qvel();
            let ten_vel = data.ten_velocity();

            for t in 0..ntendon {
                let nnz = rownnz[t] as usize;
                let adr = rowadr[t] as usize;
                let dot: MjtNum = (0..nnz)
                    .map(|k| ten_j[adr + k] * qvel[colind[adr + k] as usize])
                    .sum();
                assert_relative_eq!(dot, ten_vel[t], epsilon = 1e-10);
            }

            // Same identity through the view API
            let ten_view = data.tendon("ten1").unwrap().view(&data);
            let model_view = model.tendon("ten1").unwrap().view(&model);
            let view_nnz = model_view.J_rownnz[0] as usize;
            let view_dot: MjtNum = (0..view_nnz)
                .map(|k| ten_view.J[k] * qvel[model_view.J_colind[k] as usize])
                .sum();
            assert_relative_eq!(view_dot, ten_view.velocity[0], epsilon = 1e-10);

            let ten_info = data.tendon("ten1").unwrap();
            assert_relative_eq!(ten_view.velocity[0], ten_vel[ten_info.id], epsilon = 1e-10);
        }

        let max_vel = data.ten_velocity().iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        assert!(max_vel > 0.1, "max |ten_velocity| = {max_vel}");
    }

    /// Slide (x) + hinge (y) with the hinge site offset from the rotation axis so
    /// both DOFs contribute non-trivially. nv = 2, s1 at origin, s2 at (1,0,3).
    const TENDON_JAC_MODEL: &str = "
<mujoco>
  <worldbody>
    <body name='b1'>
      <joint name='j_slide' type='slide' axis='1 0 0'/>
      <geom type='sphere' size='0.1' mass='1'/>
      <site name='s1' pos='0 0 0'/>
    </body>
    <body name='b2' pos='0 0 3'>
      <joint name='j_hinge' type='hinge' axis='0 1 0'/>
      <geom type='sphere' size='0.1' mass='1'/>
      <site name='s2' pos='1 0 0'/>
    </body>
  </worldbody>
  <tendon>
    <spatial name='ten1'>
      <site site='s1'/>
      <site site='s2'/>
    </spatial>
  </tendon>
</mujoco>";

    /// Analytical tendon Jacobian verification at three joint-space configurations,
    /// testing per-DOF and combined velocity transforms via both flat and view APIs.
    #[test]
    fn test_ten_j_numerical_correctness() {
        let model = MjModel::from_xml_string(TENDON_JAC_MODEL).unwrap();
        let mut data = model.make_data();
        let nv = model.nv() as usize;
        assert_eq!(nv, 2);

        let s1_id = model.name_to_id(MjtObj::mjOBJ_SITE, "s1").unwrap();
        let s2_id = model.name_to_id(MjtObj::mjOBJ_SITE, "s2").unwrap();

        let to_dense = |data: &MjData<_>, model: &MjModel| -> Vec<MjtNum> {
            let ten_j = data.ten_j();
            let nnz = model.ten_j_rownnz()[0] as usize;
            let adr = model.ten_j_rowadr()[0] as usize;
            let colind = model.ten_j_colind();
            let mut dense = vec![0.0 as MjtNum; nv];
            for k in 0..nnz {
                dense[colind[adr + k] as usize] = ten_j[adr + k];
            }
            dense
        };

        let to_dense_view = |data: &MjData<_>, model: &MjModel| -> Vec<MjtNum> {
            let ten_view = data.tendon("ten1").unwrap().view(data);
            let model_view = model.tendon("ten1").unwrap().view(model);
            let view_nnz = model_view.J_rownnz[0] as usize;
            let mut dense = vec![0.0 as MjtNum; nv];
            for k in 0..view_nnz {
                dense[model_view.J_colind[k] as usize] = ten_view.J[k];
            }
            dense
        };

        // J_ten[j] = dir . (J_s2[:,j] - J_s1[:,j])
        let from_jac_site = |data: &MjData<_>| -> Vec<MjtNum> {
            let p1 = data.site_xpos()[s1_id as usize];
            let p2 = data.site_xpos()[s2_id as usize];
            let d = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let dir = [d[0] / len, d[1] / len, d[2] / len];
            let (jp1, _) = data.jac_site(true, false, s1_id).unwrap();
            let (jp2, _) = data.jac_site(true, false, s2_id).unwrap();
            let mut expected = vec![0.0 as MjtNum; nv];
            for j in 0..nv {
                let mut v: MjtNum = 0.0;
                for k in 0..3 {
                    v += dir[k] * (jp2[k * nv + j] - jp1[k * nv + j]);
                }
                expected[j] = v;
            }
            expected
        };

        let check_j = |data: &MjData<_>, expected: &[MjtNum; 2]| {
            let dense = to_dense(data, &model);
            let dense_v = to_dense_view(data, &model);
            let from_jac = from_jac_site(data);
            for j in 0..nv {
                assert_relative_eq!(dense[j], expected[j], epsilon = 1e-10);
                assert_relative_eq!(dense_v[j], expected[j], epsilon = 1e-10);
                assert_relative_eq!(from_jac[j], expected[j], epsilon = 1e-10);
            }
        };

        let check_vel = |data: &MjData<_>, expected: MjtNum| {
            assert_relative_eq!(data.ten_velocity()[0], expected, epsilon = 1e-10);
            let v = data.tendon("ten1").unwrap().view(data);
            assert_relative_eq!(v.velocity[0], expected, epsilon = 1e-10);
        };

        // Default config: s1=(0,0,0), s2=(1,0,3), dir=(1,0,3)/sqrt(10)
        // Slide (x): J[0] = dir.(-1,0,0) = -1/sqrt(10)
        // Hinge (y at pivot (0,0,3)): r=(1,0,0), omega x r = (0,0,-1), J[1] = -3/sqrt(10)
        data.forward();
        let sqrt10 = 10.0f64.sqrt();
        check_j(&data, &[-1.0 / sqrt10, -3.0 / sqrt10]);

        data.qvel_mut().copy_from_slice(&[1.0, 0.0]);
        data.forward();
        check_vel(&data, -1.0 / sqrt10);

        data.qvel_mut().copy_from_slice(&[0.0, 1.0]);
        data.forward();
        check_vel(&data, -3.0 / sqrt10);

        // qvel=(2,-0.5) -> vel = -2/sqrt10 + 1.5/sqrt10 = -0.5/sqrt10
        data.qvel_mut().copy_from_slice(&[2.0, -0.5]);
        data.forward();
        check_vel(&data, -0.5 / sqrt10);

        // Hinge at pi/4: s2 = (cos, 0, 3-sin), r = (cos, 0, -sin)
        // omega x r = (0,1,0) x (cos,0,-sin) = (-sin, 0, -cos)
        let a = std::f64::consts::FRAC_PI_4;
        let (c, s) = (a.cos(), a.sin());
        data.qpos_mut()[1] = a;
        data.qvel_mut().copy_from_slice(&[0.0; 2]);
        data.forward();

        let len2 = (c * c + (3.0 - s) * (3.0 - s)).sqrt();
        let dir2 = [c / len2, 0.0, (3.0 - s) / len2];
        let j_slide = -dir2[0];
        let j_hinge = dir2[0] * (-s) + dir2[2] * (-c);
        check_j(&data, &[j_slide, j_hinge]);
        assert!(j_slide.abs() > 0.05);
        assert!(j_hinge.abs() > 0.05);

        data.qvel_mut().copy_from_slice(&[1.0, 0.0]);
        data.forward();
        check_vel(&data, j_slide);

        data.qvel_mut().copy_from_slice(&[0.0, 1.0]);
        data.forward();
        check_vel(&data, j_hinge);

        data.qvel_mut().copy_from_slice(&[-1.0, 3.0]);
        data.forward();
        check_vel(&data, -j_slide + 3.0 * j_hinge);

        // Hinge at -pi/3 + slide at 0.5: s1=(0.5,0,0),
        // s2=(cos(-pi/3), 0, 3-sin(-pi/3)), r=(c3, 0, -s3)
        let a3 = -std::f64::consts::FRAC_PI_3;
        let (c3, s3) = (a3.cos(), a3.sin());
        data.qpos_mut().copy_from_slice(&[0.5, a3]);
        data.qvel_mut().copy_from_slice(&[0.0; 2]);
        data.forward();

        let dx3 = c3 - 0.5;
        let dz3 = 3.0 - s3;
        let len3 = (dx3 * dx3 + dz3 * dz3).sqrt();
        let dir3 = [dx3 / len3, 0.0, dz3 / len3];
        let j_slide3 = -dir3[0];
        let j_hinge3 = dir3[0] * (-s3) + dir3[2] * (-c3);
        check_j(&data, &[j_slide3, j_hinge3]);

        data.qvel_mut().copy_from_slice(&[1.7, -2.3]);
        data.forward();
        check_vel(&data, j_slide3 * 1.7 + j_hinge3 * (-2.3));
    }

    /// Verifies [force]-cast for iefc_type and iefc_state (island-reordered constraint arrays).
    #[test]
    fn test_force_cast_island_efc_enums() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        for _ in 0..10 {
            data.step();
        }

        let nefc = data.ffi().nefc as usize;
        if nefc > 0 {
            let iefc_type = data.iefc_type();
            let iefc_state = data.iefc_state();

            assert_eq!(iefc_type.len(), nefc);
            assert_eq!(iefc_state.len(), nefc);

            for i in 0..nefc {
                let raw_type = unsafe { *data.ffi().iefc_type.add(i) };
                let raw_state = unsafe { *data.ffi().iefc_state.add(i) };
                let expected_type: MjtConstraint = unsafe { crate::util::force_cast(raw_type) };
                let expected_state: MjtConstraintState = unsafe { crate::util::force_cast(raw_state) };
                assert_eq!(iefc_type[i], expected_type);
                assert_eq!(iefc_state[i], expected_state);
            }
        }
    }

    /// Verifies [force]-cast non-aliasing: adjacent bodies' xpos slices must not overlap.
    #[test]
    fn test_force_cast_non_aliasing_adjacent_bodies() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let b_free = data.body("b_free").unwrap();
        let b_slide = data.body("b_slide").unwrap();

        let v_free = b_free.view(&data);
        let v_slide = b_slide.view(&data);

        // Pointers must differ
        assert_ne!(v_free.xpos.as_ptr(), v_slide.xpos.as_ptr(),
            "adjacent bodies must have non-overlapping xpos");
        assert_ne!(v_free.cinert.as_ptr(), v_slide.cinert.as_ptr(),
            "adjacent bodies must have non-overlapping cinert");
        assert_ne!(v_free.cvel.as_ptr(), v_slide.cvel.as_ptr(),
            "adjacent bodies must have non-overlapping cvel");

        // Pointer difference must equal exactly one stride
        let xpos_diff = unsafe { v_slide.xpos.as_ptr().offset_from(v_free.xpos.as_ptr()) };
        let id_diff = b_slide.id as isize - b_free.id as isize;
        assert_eq!(xpos_diff, id_diff * 3, "xpos pointer gap must be stride*id_diff = 3*{}", id_diff);

        let cinert_diff = unsafe { v_slide.cinert.as_ptr().offset_from(v_free.cinert.as_ptr()) };
        assert_eq!(cinert_diff, id_diff * 10, "cinert pointer gap must be 10*{}", id_diff);
    }

    /**************************************************************************/
    // Multi-timestep force-cast divergence tests
    /**************************************************************************/

    /// Simulates multiple timesteps and verifies that force-cast xpos, qpos, qvel
    /// arrays diverge from initial state while remaining consistent with FFI.
    /// This exercises the force-cast pointer arithmetic across changing data.
    #[test]
    fn test_force_cast_multi_step_xpos_qpos_diverge() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let nq = model.ffi().nq as usize;
        let nv = model.ffi().nv as usize;

        // Snapshot initial values
        let init_xpos: Vec<[MjtNum; 3]> = data.xpos().to_vec();
        let init_qpos: Vec<MjtNum> = data.qpos().to_vec();
        let init_qvel: Vec<MjtNum> = data.qvel().to_vec();

        assert_eq!(init_xpos.len(), nbody);
        assert_eq!(init_qpos.len(), nq);
        assert_eq!(init_qvel.len(), nv);

        // Step 100 times -- gravity should cause free body to fall
        for _ in 0..100 {
            data.step();
        }

        let post_xpos = data.xpos();
        let post_qpos = data.qpos();
        let post_qvel = data.qvel();

        assert_eq!(post_xpos.len(), nbody);
        assert_eq!(post_qpos.len(), nq);
        assert_eq!(post_qvel.len(), nv);

        // b_free should have fallen (z position decreased under gravity)
        let b_free = data.body("b_free").unwrap();
        assert!(post_xpos[b_free.id][2] < init_xpos[b_free.id][2],
            "free body should have fallen: init_z={}, post_z={}",
            init_xpos[b_free.id][2], post_xpos[b_free.id][2]);

        // qpos should differ from initial
        assert_ne!(post_qpos, &init_qpos[..], "qpos must change after 100 steps");

        // qvel should differ from zero (gravity accelerates bodies)
        let any_nonzero_vel = post_qvel.iter().any(|v| v.abs() > 1e-12);
        assert!(any_nonzero_vel, "qvel must have nonzero entries after gravity steps");

        // Cross-validate post-step xpos with FFI
        for i in 0..nbody {
            for j in 0..3 {
                assert_eq!(post_xpos[i][j], unsafe { *data.ffi().xpos.add(i * 3 + j) },
                    "xpos[{}][{}] FFI mismatch after stepping", i, j);
            }
        }

        // Cross-validate post-step qvel with FFI
        for i in 0..nv {
            assert_eq!(post_qvel[i], unsafe { *data.ffi().qvel.add(i) },
                "qvel[{}] FFI mismatch after stepping", i);
        }
    }

    /// Simulates with an actuator active and verifies force-cast arrays (ctrl, qfrc_actuator,
    /// actuator_force) reflect the control input after multiple steps.
    #[test]
    fn test_force_cast_multi_step_actuator_ctrl() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        // Apply control to the slide actuator
        data.ctrl_mut()[0] = 5.0;

        // Step a few times to let forces propagate
        for _ in 0..20 {
            data.step();
        }

        let nu = model.ffi().nu as usize;
        let ctrl = data.ctrl();
        assert_eq!(ctrl.len(), nu);
        assert_eq!(ctrl[0], 5.0);

        // actuator_force should be nonzero
        let act_force = data.actuator_force();
        assert_eq!(act_force.len(), nu);
        assert!(act_force[0].abs() > 1e-12,
            "actuator_force should be nonzero with ctrl=5.0, got {}", act_force[0]);

        // FFI cross-validation
        assert_eq!(act_force[0], unsafe { *data.ffi().actuator_force });

        // qfrc_actuator should be nonzero (force applied to joint)
        let nv = model.ffi().nv as usize;
        let qfrc = data.qfrc_actuator();
        assert_eq!(qfrc.len(), nv);
        let any_nonzero = qfrc.iter().any(|v| v.abs() > 1e-12);
        assert!(any_nonzero, "qfrc_actuator must reflect the actuator force");
    }

    /// Steps multiple times with gravity & contacts, then verifies enum force-casts
    /// (efc_type, efc_state) and array groupings (efc_KBIP, contact xpos/frame)
    /// reflect the evolved simulation state and remain FFI-consistent.
    #[test]
    fn test_force_cast_multi_step_constraints_evolve() {
        // MODEL has many objects that create contacts
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Step enough for contacts to form and constraints to be generated
        for _ in 0..50 {
            data.step();
        }

        let nefc = data.ffi().nefc as usize;
        let ncon = data.ffi().ncon as usize;

        // With a rich model and 50 steps, we expect contacts
        // (not guaranteed in every model, but MODEL has spheres falling on a plane)
        if ncon > 0 {
            // Contact positions (xpos) should have changed
            let contacts = data.contacts();
            assert_eq!(contacts.len(), ncon);
            for c in contacts {
                // Each contact's pos is a [f64; 3]
                let pos_nonzero = c.pos.iter().any(|v| v.abs() > 1e-12);
                assert!(pos_nonzero, "contact pos should be nonzero for an active contact");
            }
        }

        if nefc > 0 {
            let efc_type = data.efc_type();
            let efc_state = data.efc_state();
            assert_eq!(efc_type.len(), nefc);
            assert_eq!(efc_state.len(), nefc);

            // FFI cross-validation post-step
            for i in 0..nefc {
                let raw_type = unsafe { *data.ffi().efc_type.add(i) };
                let raw_state = unsafe { *data.ffi().efc_state.add(i) };
                assert_eq!(efc_type[i], unsafe { crate::util::force_cast::<_, MjtConstraint>(raw_type) });
                assert_eq!(efc_state[i], unsafe { crate::util::force_cast::<_, MjtConstraintState>(raw_state) });
            }

            // efc_KBIP should be populated
            let kbip = data.efc_kbip();
            assert_eq!(kbip.len(), nefc);
            for i in 0..nefc {
                for j in 0..4 {
                    assert_eq!(kbip[i][j], unsafe { *data.ffi().efc_KBIP.add(i * 4 + j) });
                }
            }
        }
    }

    /// Runs many timesteps and verifies that body views (via info_with_view!)
    /// remain consistent with flat slice accessors at each step.
    /// This tests that force-cast pointers track the mutating mjData correctly.
    #[test]
    fn test_force_cast_view_consistency_across_steps() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let body_names = ["world", "b_free", "b_slide", "b_hinge", "mocap_body"];

        for step_idx in 0..30 {
            data.step();

            let xpos_flat = data.xpos();
            let xquat_flat = data.xquat();
            let cvel_flat = data.cvel();
            let cinert_flat = data.cinert();

            for name in &body_names {
                let info = data.body(name).unwrap();
                let view = info.view(&data);
                let id = info.id;

                // xpos from view must match flat slice
                assert_eq!(&view.xpos[..], &xpos_flat[id][..],
                    "xpos mismatch at step {} body '{}'", step_idx, name);

                // xquat from view must match flat slice
                assert_eq!(&view.xquat[..], &xquat_flat[id][..],
                    "xquat mismatch at step {} body '{}'", step_idx, name);

                // cvel from view must match flat slice
                assert_eq!(&view.cvel[..], &cvel_flat[id][..],
                    "cvel mismatch at step {} body '{}'", step_idx, name);

                // cinert from view must match flat slice
                assert_eq!(&view.cinert[..], &cinert_flat[id][..],
                    "cinert mismatch at step {} body '{}'", step_idx, name);
            }
        }
    }

    /// Steps with applied external forces via force-cast mutable array, verifies
    /// that the force affects simulation state across multiple timesteps.
    #[test]
    fn test_force_cast_multi_step_xfrc_applied_effect() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let b_free = data.body("b_free").unwrap();
        let b_free_id = b_free.id;

        // Baseline: step without applied force
        data.forward();
        let baseline_xpos = data.xpos()[b_free_id];

        // Reset and apply upward force to counteract gravity
        data.reset();
        // Apply a strong upward force: [fx, fy, fz, torque_x, torque_y, torque_z]
        data.xfrc_applied_mut()[b_free_id] = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0];

        for _ in 0..50 {
            data.step();
        }

        let forced_xpos = data.xpos()[b_free_id];

        // With an upward force of 100N on a 1kg mass, z should increase
        assert!(forced_xpos[2] > baseline_xpos[2],
            "Upward force should raise body: baseline_z={}, forced_z={}",
            baseline_xpos[2], forced_xpos[2]);

        // FFI cross-validation at this point
        for j in 0..3 {
            assert_eq!(forced_xpos[j], unsafe { *data.ffi().xpos.add(b_free_id * 3 + j) });
        }

        // xfrc_applied should still hold our value
        assert_eq!(data.xfrc_applied()[b_free_id], [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]);
    }

    /// Simulates, mutates mocap position mid-simulation via force-cast mutable array,
    /// and verifies the change is reflected in subsequent forward passes.
    #[test]
    fn test_force_cast_multi_step_mocap_mutation() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nmocap = model.ffi().nmocap as usize;
        assert!(nmocap > 0, "model should have at least one mocap body");

        // Check initial
        let init_pos = data.mocap_pos()[0];
        assert_eq!(init_pos, [10.0, 10.0, 10.0]);

        // Step a few times
        for _ in 0..10 {
            data.step();
        }

        // Mutate mocap position mid-simulation
        data.mocap_pos_mut()[0] = [20.0, 30.0, 40.0];
        data.forward();

        assert_eq!(data.mocap_pos()[0], [20.0, 30.0, 40.0]);
        // FFI cross-validation
        for j in 0..3 {
            assert_eq!(unsafe { *data.ffi().mocap_pos.add(j) }, [20.0, 30.0, 40.0][j]);
        }

        // Mutate again and step further
        data.mocap_pos_mut()[0] = [-5.0, -5.0, -5.0];
        for _ in 0..10 {
            data.step();
        }
        assert_eq!(data.mocap_pos()[0], [-5.0, -5.0, -5.0]);
    }

    /// Verifies that sensor data changes across multiple simulation steps.
    /// Exercises the force-cast sensordata flat array after physics evolves.
    #[test]
    fn test_force_cast_multi_step_sensor_data_evolves() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        data.forward();
        let init_qpos: Vec<MjtNum> = data.qpos().to_vec();

        // Apply a downward force via force-cast xfrc_applied
        let b_slide = data.body("b_slide").unwrap();
        data.xfrc_applied_mut()[b_slide.id] = [0.0, 0.0, -50.0, 0.0, 0.0, 0.0];

        for _ in 0..100 {
            data.step();
        }

        // Positions should have evolved under applied force
        let post_qpos = data.qpos();
        assert_ne!(post_qpos, init_qpos.as_slice(),
            "qpos did not evolve after 100 steps with applied force");

        // Sensor FFI cross-validation
        let nsensordata = model.ffi().nsensordata as usize;
        let post_sensor = data.sensordata();
        assert_eq!(post_sensor.len(), nsensordata);
        for i in 0..nsensordata {
            assert_eq!(post_sensor[i], unsafe { *data.ffi().sensordata.add(i) },
                "sensordata[{}] FFI mismatch", i);
        }
    }

    /// Multi-step test with eq_active force-cast bool: disable an equality
    /// constraint mid-simulation, verify dynamics change.
    #[test]
    fn test_force_cast_multi_step_eq_active_toggle() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        // Step with constraint active
        for _ in 0..20 {
            data.step();
        }
        let pos_with_eq = data.xpos().to_vec();

        // Reset, disable equality constraint, step again
        data.reset();
        data.eq_active_mut()[0] = false;
        data.eq_active_mut()[1] = false;

        for _ in 0..20 {
            data.step();
        }
        let pos_without_eq = data.xpos().to_vec();

        // The positions should differ since the constraints are disabled
        let b_slide = data.body("b_slide").unwrap();
        let diff: MjtNum = (0..3)
            .map(|j| (pos_with_eq[b_slide.id][j] - pos_without_eq[b_slide.id][j]).abs())
            .sum();

        // At least some difference expected from disabling the equality constraint
        // (may be subtle depending on model dynamics, but should not be zero)
        assert!(diff > 1e-15 || {
            // If b_slide didn't move much, check b_hinge (the other constrained body)
            let b_hinge = data.body("b_hinge").unwrap();
            (0..3)
                .map(|j| (pos_with_eq[b_hinge.id][j] - pos_without_eq[b_hinge.id][j]).abs())
                .sum::<MjtNum>() > 1e-15
        }, "disabling equality constraints should change positions");

        // FFI cross-validation for eq_active
        assert_eq!(data.eq_active()[0], false);
        assert_eq!(data.eq_active()[1], false);
        assert_eq!(unsafe { *data.ffi().eq_active }, 0u8);
        assert_eq!(unsafe { *data.ffi().eq_active.add(1) }, 0u8);
    }

    /// Runs step1() + step2() split stepping and verifies force-cast arrays
    /// remain consistent with FFI between sub-steps.
    #[test]
    fn test_force_cast_split_step_consistency() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        let nbody = model.ffi().nbody as usize;

        for _ in 0..15 {
            data.step1();

            // After step1: positions and velocities are computed
            let mid_xpos = data.xpos();
            assert_eq!(mid_xpos.len(), nbody);
            for i in 0..nbody {
                for j in 0..3 {
                    assert_eq!(mid_xpos[i][j], unsafe { *data.ffi().xpos.add(i * 3 + j) },
                        "xpos[{}][{}] FFI mismatch after step1", i, j);
                }
            }

            data.step2();

            // After step2: integration is complete
            let post_xpos = data.xpos();
            for i in 0..nbody {
                for j in 0..3 {
                    assert_eq!(post_xpos[i][j], unsafe { *data.ffi().xpos.add(i * 3 + j) },
                        "xpos[{}][{}] FFI mismatch after step2", i, j);
                }
            }
        }
    }

    /// Steps the simulation, copies state via get_state/set_state, steps further,
    /// and verifies force-cast arrays reflect the correct state at each point.
    #[test]
    fn test_force_cast_multi_step_state_save_restore() {
        use crate::wrappers::mj_data::MjtState;

        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        // Step 30 times
        for _ in 0..30 {
            data.step();
        }
        // Synchronize derived quantities (xpos, etc.) with current qpos
        data.forward();

        // Save state
        let saved_state = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);
        let saved_xpos: Vec<[MjtNum; 3]> = data.xpos().to_vec();
        let saved_qpos: Vec<MjtNum> = data.qpos().to_vec();

        // Step 30 more times (state diverges)
        for _ in 0..30 {
            data.step();
        }
        let diverged_xpos: Vec<[MjtNum; 3]> = data.xpos().to_vec();
        assert_ne!(diverged_xpos, saved_xpos, "state should diverge after more steps");

        // Restore state
        // SAFETY: saved_state was captured via mj_getState; eq_active bytes are valid (0 or 1).
        unsafe { data.set_state(&saved_state, MjtState::mjSTATE_FULLPHYSICS as u32) };
        data.forward();

        // Primary state (qpos) should be exactly restored
        let restored_qpos = data.qpos();
        for i in 0..saved_qpos.len() {
            assert_eq!(restored_qpos[i], saved_qpos[i],
                "qpos[{}] should match saved state after restore", i);
        }

        // Derived quantity (xpos) should be approximately restored
        // (forward() recomputes from scratch, minor floating-point differences possible)
        let restored_xpos = data.xpos();
        for i in 0..saved_xpos.len() {
            for j in 0..3 {
                assert!(
                    (restored_xpos[i][j] - saved_xpos[i][j]).abs() < 1e-10,
                    "xpos[{}][{}] should approximately match saved state: got {} vs {}",
                    i, j, restored_xpos[i][j], saved_xpos[i][j]
                );
            }
        }
    }

    /// Multi-step test that verifies kinematic quantities (xmat, xipos, ximat)
    /// change across steps and stay FFI-consistent via force-cast.
    #[test]
    fn test_force_cast_multi_step_kinematics_evolve() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();

        // Apply an off-center force to induce rotation on the free body.
        let b_free = data.body("b_free").unwrap();
        data.xfrc_applied_mut()[b_free.id] = [1.0, 0.0, 0.0, 0.0, 0.5, 0.0];
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let init_xmat: Vec<[MjtNum; 9]> = data.xmat().to_vec();
        let init_xipos: Vec<[MjtNum; 3]> = data.xipos().to_vec();

        // Step 50 times with the off-center force
        for _ in 0..50 {
            data.step();
        }

        let post_xmat = data.xmat();
        let post_xipos = data.xipos();

        assert_eq!(post_xmat.len(), nbody);
        assert_eq!(post_xipos.len(), nbody);

        // Free body xipos should change (it falls and translates)
        let pos_changed = (0..3).any(|j| (post_xipos[b_free.id][j] - init_xipos[b_free.id][j]).abs() > 1e-6);
        assert!(pos_changed, "free body xipos should change as it moves");

        // Free body xmat should change (off-center force induces rotation)
        let mat_changed = (0..9).any(|j| (post_xmat[b_free.id][j] - init_xmat[b_free.id][j]).abs() > 1e-12);
        assert!(mat_changed, "free body xmat should change with off-center force");

        // FFI cross-validation
        for i in 0..nbody {
            for j in 0..9 {
                assert_eq!(post_xmat[i][j], unsafe { *data.ffi().xmat.add(i * 9 + j) });
            }
            for j in 0..3 {
                assert_eq!(post_xipos[i][j], unsafe { *data.ffi().xipos.add(i * 3 + j) });
            }
        }
    }

    /// Runs simulation and verifies dynamic subtree quantities (subtree_com,
    /// subtree_linvel, subtree_angmom) change and match FFI after stepping.
    #[test]
    fn test_force_cast_multi_step_subtree_dynamics() {
        let model = MjModel::from_xml_string(FORCE_MODEL).unwrap();
        let mut data = model.make_data();
        data.forward();

        let nbody = model.ffi().nbody as usize;
        let init_subtree_com: Vec<[MjtNum; 3]> = data.subtree_com().to_vec();

        for _ in 0..60 {
            data.step();
        }

        let post_subtree_com = data.subtree_com();
        let post_subtree_linvel = data.subtree_linvel();
        let post_subtree_angmom = data.subtree_angmom();

        assert_eq!(post_subtree_com.len(), nbody);
        assert_eq!(post_subtree_linvel.len(), nbody);
        assert_eq!(post_subtree_angmom.len(), nbody);

        // World subtree_com should change (bodies are falling)
        let world_com_diff: MjtNum = (0..3)
            .map(|j| (post_subtree_com[0][j] - init_subtree_com[0][j]).abs())
            .sum();
        assert!(world_com_diff > 1e-6,
            "world subtree_com should shift as bodies fall");

        // FFI cross-validation
        for i in 0..nbody {
            for j in 0..3 {
                assert_eq!(post_subtree_com[i][j], unsafe { *data.ffi().subtree_com.add(i * 3 + j) });
                assert_eq!(post_subtree_linvel[i][j], unsafe { *data.ffi().subtree_linvel.add(i * 3 + j) });
                assert_eq!(post_subtree_angmom[i][j], unsafe { *data.ffi().subtree_angmom.add(i * 3 + j) });
            }
        }
    }

    /// Test swap of [`MjModel`] borrowed by [`MjData`].
    #[test]
    fn test_model_swap() {
        const OLD_TIMESTEP: f64 = 0.002;
        const NEW_TIMESTEP: f64 = 0.1;

        let mut model_template = Box::new(MjSpec::new().compile().unwrap());
        model_template.opt_mut().timestep = OLD_TIMESTEP;

        let model = model_template.clone();
        let mut data = MjData::new(model);

        model_template.opt_mut().timestep = NEW_TIMESTEP;
        model_template = data.swap_model(model_template).unwrap();
        assert_eq!(model_template.opt().timestep, OLD_TIMESTEP);
        assert_eq!(data.model().opt().timestep, NEW_TIMESTEP);
    }
}
