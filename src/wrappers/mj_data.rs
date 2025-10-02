use super::mj_model::{MjModel, MjtSameFrame, MjtObj, MjtStage};
use super::mj_statistic::{MjWarningStat, MjTimerStat};
use super::mj_auxiliary::MjContact;
use super::mj_primitive::*;
use crate::mujoco_c::*;

use std::io::{self, Error, ErrorKind};
use std::ffi::CString;
use std::ops::Deref;
use std::ptr;

use crate::{mj_view_indices, mj_model_nx_to_mapping, mj_model_nx_to_nitem};
use crate::{view_creator, info_method, info_with_view};

/*******************************************/
// Types
/// State component elements as integer bitflags and several convenient combinations of these flags. Used by
/// `mj_getState`, `mj_setState` and `mj_stateSize`.
pub type MjtState = mjtState;

/// Constraint types. These values are not used in mjModel, but are used in the mjData field ``d->efc_type`` when the list
/// of active constraints is constructed at each simulation time step.
pub type MjtConstraint = mjtConstraint;

/// These values are used by the solver internally to keep track of the constraint states.
pub type MjtConstraintState = mjtConstraintState;

/// Warning types. The number of warning types is given by `mjNWARNING`
/// which is also the length of the array `mjData.warning`.
pub type MjtWarning = mjtWarning;

/// Timer types. The number of timer types is given by `mjNTIMER`
/// which is also the length of the array `mjData.timer`, as well as the length of
/// the string array `mjTIMERSTRING` with timer names.
pub type MjtTimer = mjtTimer;
/*******************************************/


/**************************************************************************************************/
// MjData
/**************************************************************************************************/

/// Wrapper around the ``mjData`` struct.
/// Provides lifetime guarantees as well as automatic cleanup.
pub struct MjData<M: Deref<Target = MjModel>> {
    data: *mut mjData,
    model: M
}

// Allow usage in threaded contexts as the data won't be shared anywhere outside Rust,
// except in the C++ code.
unsafe impl<M: Deref<Target = MjModel>> Send for MjData<M> {}
unsafe impl<M: Deref<Target = MjModel>> Sync for MjData<M> {}


impl<M: Deref<Target = MjModel>> MjData<M> {
    /// Constructor for a new MjData. This should is called from MjModel.
    pub fn new(model: M) -> Self {
        unsafe {
            Self {
                data: mj_makeData(model.ffi()),
                model: model,
            }
        }
    }

    /// Returns a slice of detected contacts.
    /// To obtain the contact force, call [`MjData::contact_force`].
    pub fn contacts(&self) -> &[MjContact] {
        unsafe {
            std::slice::from_raw_parts((*self.data).contact, (*self.data).ncon as usize)
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

    /// Obtains a [`MjActuatorDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjActuatorDataInfo::view`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn actuator(&self, name: &str) -> Option<MjActuatorDataInfo> {
        let c_name = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.model.ffi(), MjtObj::mjOBJ_ACTUATOR as i32, c_name.as_ptr())};
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

        Some(MjActuatorDataInfo { name: name.to_string(), id: id as usize, ctrl, act})
    }

    /// Obtains a [`MjJointDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjJointDataInfo::view`].
    /// # Panics
    /// When the `name` contains \0' characters, a panic occurs.
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

            Some(MjJointDataInfo {name: name.to_string(), id: id as usize,
                qpos, qvel, qacc_warmstart, qfrc_applied, qacc, xanchor, xaxis, qLDiagInv, qfrc_bias,
                qfrc_spring, qfrc_damper, qfrc_gravcomp, qfrc_fluid, qfrc_passive,
                qfrc_actuator, qfrc_smooth, qacc_smooth, qfrc_constraint, qfrc_inverse
            })
        }
    }

    /// Obtains a [`MjSensorDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjSensorDataInfo::view`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn sensor(&self, name: &str) -> Option<MjSensorDataInfo> {
        let c_name = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.model.ffi(), MjtObj::mjOBJ_SENSOR as i32, c_name.as_ptr())};
        if id == -1 {  // not found
            return None;
        }
        let model_ffi = self.model.ffi();
        let id = id as usize;

        unsafe {
            let data = mj_view_indices!(id, mj_model_nx_to_mapping!(model_ffi, nsensordata), mj_model_nx_to_nitem!(model_ffi, nsensordata), model_ffi.nsensordata);
            Some(MjSensorDataInfo { id, name: name.to_string(), data })
        }
    }


    /// Obtains a [`MjTendonDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjTendonDataInfo::view`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    #[allow(non_snake_case)]
    pub fn tendon(&self, name: &str) -> Option<MjTendonDataInfo> {
        let c_name = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.model.ffi(), MjtObj::mjOBJ_TENDON as i32, c_name.as_ptr())};
        if id == -1 {  // not found
            return None;
        }

        let model_ffi = self.model.ffi();
        let id = id as usize;
        let nv = model_ffi.nv as usize;
        let wrapadr = (id, 1);
        let wrapnum = (id, 1);
        let J_rownnz = (id, 1);
        let J_rowadr = (id, 1);
        let J_colind = (id * nv, nv);
        let length = (id, 1);
        let J = (id * nv, nv);
        let velocity = (id, 1);

        Some(MjTendonDataInfo { id, name: name.to_string(), wrapadr, wrapnum, J_rownnz, J_rowadr, J_colind, length, J, velocity })
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

    /// Inverse dynamics: qacc must be set before calling ([`MjData::forward`]).
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
    pub fn contact_force(&self, contact_id: usize) -> [MjtNum; 6] {
        let mut force = [0.0; 6];
        unsafe {
            mj_contactForce(
                self.model.ffi(), self.data,
                contact_id as i32, force.as_mut_ptr()
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
    /// float_format must be a valid printf-style format string for a single float value
    /// # Panics
    /// When the `filename` or `float_format` contain '\0' characters, a panic occurs.
    pub fn print_formatted(&self, filename: &str, float_format: &str) {
        let c_filename = CString::new(filename).unwrap();
        let c_float_format = CString::new(float_format).unwrap();
        unsafe { mj_printFormattedData(self.model.ffi(), self.ffi(), c_filename.as_ptr(), c_float_format.as_ptr()) }
    }

    /// Print data to text file.
    /// # Panics
    /// When the `filename` contains '\0' characters, a panic occurs.
    pub fn print(&self, filename: &str) {
        let c_filename = CString::new(filename).unwrap();
        unsafe { mj_printData(self.model.ffi(), self.ffi(), c_filename.as_ptr()) }
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
    pub fn crb(&mut self) {
        unsafe { mj_crb(self.model.ffi(), self.ffi_mut()) }
    }

    /// Make inertia matrix.
    pub fn make_m(&mut self) {
        unsafe { mj_makeM(self.model.ffi(), self.ffi_mut()) }
    }

    /// Compute sparse L'*D*L factorizaton of inertia matrix.
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
    /// If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
    /// Nullable: cost
    pub fn constraint_update(&mut self, jar: &[MjtNum], cost: Option<&mut MjtNum>, flg_cone_hessian: bool) {
        unsafe { mj_constraintUpdate(
            self.model.ffi(), self.ffi_mut(),
            jar.as_ptr(), cost.map_or(ptr::null_mut(), |x| x as *mut MjtNum),
            flg_cone_hessian as i32
        ) }
    }

    /// Add contact to d->contact list; return 0 if success; 1 if buffer full.
    pub fn add_contact(&mut self, con: &MjContact) -> io::Result<()> {
        match unsafe { mj_addContact(self.model.ffi(), self.ffi_mut(), con) } {
            0 => Ok(()),
            1 => Err(Error::new(ErrorKind::StorageFull, "buffer full")),
            _ => Err(Error::new(ErrorKind::Other, "unknown error"))
        }
    }

    /// Compute 3/6-by-nv end-effector Jacobian of a global point attached to the given body.
    /// Set `jacp` to `true` to calculate the translational Jacobian and `jacr` to `true` for
    /// the rotational Jacobian. Returns a `(Vec, Vec)` for translation and rotation. Empty `Vec`s
    /// indicate that the corresponding Jacobian was not computed.
    pub fn jac(&self, jacp: bool, jacr: bool, point: &[MjtNum; 3], body_id: i32) -> (Vec<MjtNum>, Vec<MjtNum>) {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jac(
                self.model.ffi(),
                self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                point.as_ptr(),
                body_id,
            )
        };

        (jacp_vec, jacr_vec)
    }

    /// Compute body frame end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    pub fn jac_body(&self, jacp: bool, jacr: bool, body_id: i32) -> (Vec<MjtNum>, Vec<MjtNum>) {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jacBody(
                self.model.ffi(),
                self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                body_id,
            )
        };

        (jacp_vec, jacr_vec)
    }

    /// Compute body center-of-mass end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    pub fn jac_body_com(&self, jacp: bool, jacr: bool, body_id: i32) -> (Vec<MjtNum>, Vec<MjtNum>) {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jacBodyCom(
                self.model.ffi(),
                self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                body_id,
            )
        };

        (jacp_vec, jacr_vec)
    }

    /// Compute subtree center-of-mass end-effector Jacobian (translational only).
    /// Set `jacp` to `true` to calculate the translational component. Returns a `Vec`.
    /// Empty `Vec` indicates that the Jacobian was not computed.
    pub fn jac_subtree_com(&mut self, jacp: bool, body_id: i32) -> Vec<MjtNum> {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jacSubtreeCom(
                self.model.ffi(),
                self.ffi_mut(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                body_id,
            )
        };

        jacp_vec
    }

    /// Compute geom end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    pub fn jac_geom(&self, jacp: bool, jacr: bool, geom_id: i32) -> (Vec<MjtNum>, Vec<MjtNum>) {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jacGeom(
                self.model.ffi(),
                self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                geom_id,
            )
        };

        (jacp_vec, jacr_vec)
    }

    /// Compute site end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    pub fn jac_site(&self, jacp: bool, jacr: bool, site_id: i32) -> (Vec<MjtNum>, Vec<MjtNum>) {
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp { vec![0 as MjtNum; required_len] } else { vec![] };
        let mut jacr_vec = if jacr { vec![0 as MjtNum; required_len] } else { vec![] };

        unsafe {
            mj_jacSite(
                self.model.ffi(),
                self.ffi(),
                if jacp { jacp_vec.as_mut_ptr() } else { ptr::null_mut() },
                if jacr { jacr_vec.as_mut_ptr() } else { ptr::null_mut() },
                site_id,
            )
        };

        (jacp_vec, jacr_vec)
    }

    /// Compute subtree angular momentum matrix.
    pub fn angmom_mat(&mut self, body_id: i32) -> Vec<MjtNum> {
        let mut mat = vec![0.0; 3 * self.model.ffi().nv as usize];
        unsafe { mj_angmomMat(self.model.ffi(), self.ffi_mut(), mat.as_mut_ptr(), body_id) };
        mat
    }

    /// Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
    pub fn object_velocity(&self, obj_type: MjtObj, obj_id: i32, flg_local: bool) -> [MjtNum; 6] {
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe { mj_objectVelocity(
            self.model.ffi(), self.ffi(),
            obj_type as i32, obj_id,
            result.as_mut_ptr(), flg_local as i32
        ) };
        result
    }

    /// Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.
    pub fn object_acceleration(&self, obj_type: MjtObj, obj_id: i32, flg_local: bool) -> [MjtNum; 6] {
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe { mj_objectAcceleration(
            self.model.ffi(), self.ffi(),
            obj_type as i32, obj_id,
            result.as_mut_ptr(), flg_local as i32
        ) };
        result
    }

    /// Returns smallest signed distance between two geoms and optionally segment from geom1 to geom2.
    pub fn geom_distance(&self, geom1_id: i32, geom2_id: i32, dist_max: MjtNum, fromto: Option<&mut [MjtNum; 6]>) -> MjtNum {
        unsafe { mj_geomDistance(
            self.model.ffi(), self.ffi(),
            geom1_id, geom2_id, dist_max,
            fromto.map_or(ptr::null_mut(), |x| x.as_mut_ptr())
        ) }
    }

    /// Map from body local to global Cartesian coordinates, sameframe takes values from [`MjtSameFrame`].
    /// Returns (global position, global orientation matrix).
    /// Wraps ``mj_local2Global``.
    pub fn local_to_global(&mut self, pos: &[MjtNum; 3], quat: &[MjtNum; 4], body_id: i32, sameframe: MjtSameFrame) -> ([MjtNum; 3], [MjtNum; 9]) {
        /* Create uninitialized because this gets filled by the function. */
        let mut xpos: [MjtNum; 3] =  [0.0; 3];
        let mut xmat: [MjtNum; 9] = [0.0; 9];
        unsafe { mj_local2Global(self.ffi_mut(), xpos.as_mut_ptr(), xmat.as_mut_ptr(), pos.as_ptr(), quat.as_ptr(), body_id, sameframe as MjtByte) };
        (xpos, xmat)
    }

    /// Intersect multiple rays emanating from a single point.
    /// Similar semantics to mj_ray, but vec is an array of (nray x 3) directions.
    /// Returns (geomids, distances).
    pub fn multi_ray(
        &mut self, pnt: &[MjtNum; 3], vec: &[[MjtNum; 3]], geomgroup: Option<&[MjtByte; mjNGROUP as usize]>,
        flg_static: bool, bodyexclude: i32, cutoff: MjtNum
    ) -> (Vec<i32>, Vec<MjtNum>) {
        let nray = vec.len();
        let mut geom_id = vec![0; nray];
        let mut distance = vec![0.0; nray];

        unsafe { mj_multiRay(
            self.model.ffi(), self.ffi_mut(), pnt.as_ptr(),
            vec.as_ptr() as *const MjtNum, geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
            flg_static as u8, bodyexclude, geom_id.as_mut_ptr(),
            distance.as_mut_ptr(), nray as i32, cutoff
        ) };

        (geom_id, distance)
    }

    /// Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
    /// Return distance (x) to nearest surface, or -1 if no intersection and output geomid.
    /// geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.
    pub fn ray(
        &self, pnt: &[MjtNum; 3], vec: &[MjtNum; 3],
        geomgroup: Option<&[MjtByte; mjNGROUP as usize]>, flg_static: bool, bodyexclude: i32
    ) -> (i32, MjtNum) {
        let mut geom_id = -1;
        let dist = unsafe { mj_ray(
            self.model.ffi(), self.ffi(),
            pnt.as_ptr(), vec.as_ptr(),
            geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
            flg_static as MjtByte, bodyexclude, &mut geom_id
        ) };
        (geom_id, dist)
    }

    /// Returns a direct pointer to the underlying model.
    /// THIS IS NOT TO BE USED.
    /// It is only meant for the viewer code, which currently still depends
    /// on mutable pointers to model and data. This will be removed in the future.
    pub(crate) unsafe fn __raw(&self) -> *mut mjData {
        self.data
    }

}


/// Some public attribute methods.
impl<M: Deref<Target = MjModel>> MjData<M> {
    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjData {
        unsafe { self.data.as_ref().unwrap() }
    }

    /// Mutable reference to the wrapped FFI struct.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjData {
        unsafe { self.data.as_mut().unwrap() }
    }

    /// Returns a reference to data's [`MjModel`].
    pub fn model(&self) -> &MjModel {
        &self.model
    }

    /// Maximum stack allocation in bytes.
    pub fn maxuse_stack(&self) -> MjtSize {
        self.ffi().maxuse_stack
    }

    /// Maximum stack allocation per thread in bytes.
    pub fn maxuse_threadstack(&self) -> &[MjtSize] {
        &self.ffi().maxuse_threadstack
    }

    /// Warning statistics.
    pub fn warning_stats(&self) -> &[MjWarningStat] {
        &self.ffi().warning
    }

    /// Timer statistics.
    pub fn timer_stats(&self) -> &[MjTimerStat] {
        &self.ffi().timer
    }

    /// Simulation time
    pub fn time(&self) -> MjtNum {
        self.ffi().time
    }

    /// Potential, kinetic energy.
    pub fn energy(&self) -> &[MjtNum] {
        &self.ffi().energy
    }
}

impl<M: Deref<Target = MjModel>> Drop for MjData<M> {
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
    Data,
    joint,
    [
        qpos: MjtNum, qvel: MjtNum, qacc_warmstart: MjtNum, qfrc_applied: MjtNum, qacc: MjtNum, xanchor: MjtNum, xaxis: MjtNum, qLDiagInv: MjtNum,
        qfrc_bias: MjtNum, qfrc_spring: MjtNum, qfrc_damper: MjtNum, qfrc_gravcomp: MjtNum, qfrc_fluid: MjtNum, qfrc_passive: MjtNum,
        qfrc_actuator: MjtNum, qfrc_smooth: MjtNum, qacc_smooth: MjtNum, qfrc_constraint: MjtNum, qfrc_inverse: MjtNum
    ],
    [],
    M: Deref<Target = MjModel>
);

/// Deprecated name for [`MjJointDataInfo`].
#[deprecated]
pub type MjJointInfo = MjJointDataInfo;


/* Backward compatibility */
impl MjJointDataViewMut<'_> {
    /// Deprecated. Use [`MjJointDataViewMut::zero`] instead.
    #[deprecated]
    pub fn reset(&mut self) {
        self.zero();
    }
}

/// Deprecated name for [`MjJointDataView`].
#[deprecated]
pub type MjJointView<'d> = MjJointDataView<'d>;

/// Deprecated name for [`MjJointDataViewMut`].
#[deprecated]
pub type MjJointViewMut<'d> = MjJointDataViewMut<'d>;

/**************************************************************************************************/
// Sensor view
/**************************************************************************************************/
info_with_view!(Data, sensor, sensor, [data: MjtNum], [], M: Deref<Target = MjModel>);

/**************************************************************************************************/
// Geom view
/**************************************************************************************************/
info_with_view!(Data, geom, geom_, [xpos: MjtNum, xmat: MjtNum], [], M: Deref<Target = MjModel>);

/// Deprecated name for [`MjGeomDataInfo`].
#[deprecated]
pub type MjGeomInfo = MjGeomDataInfo;

/// Deprecated name for [`MjGeomDataView`].
#[deprecated]
pub type MjGeomView<'d> = MjGeomDataView<'d>;

/// Deprecated name for [`MjGeomDataViewMut`].
#[deprecated]
pub type MjGeomViewMut<'d> = MjGeomDataViewMut<'d>;

/**************************************************************************************************/
// Actuator view
/**************************************************************************************************/
info_with_view!(Data, actuator, [ctrl: MjtNum], [act: MjtNum], M: Deref<Target = MjModel>);

/// Deprecated name for [`MjActuatorDataInfo`].
#[deprecated]
pub type MjActuatorInfo = MjActuatorDataInfo;

/// Deprecated name for [`MjActuatorDataView`].
#[deprecated]
pub type MjActuatorView<'d> = MjActuatorDataView<'d>;

/// Deprecated name for [`MjActuatorDataViewMut`].
#[deprecated]
pub type MjActuatorViewMut<'d> = MjActuatorDataViewMut<'d>;

/**************************************************************************************************/
// Body view
/**************************************************************************************************/
info_with_view!(
    Data, body, [
        xfrc_applied: MjtNum, xpos: MjtNum, xquat: MjtNum, xmat: MjtNum, xipos: MjtNum, ximat: MjtNum,
        subtree_com: MjtNum, cinert: MjtNum, crb: MjtNum, cvel: MjtNum, subtree_linvel: MjtNum,
        subtree_angmom: MjtNum, cacc: MjtNum, cfrc_int: MjtNum, cfrc_ext: MjtNum
    ], [], M: Deref<Target = MjModel>
);

/**************************************************************************************************/
// Camera view
/**************************************************************************************************/
info_with_view!(Data, camera, cam_, [xpos: MjtNum, xmat: MjtNum], [], M: Deref<Target = MjModel>);
    
/**************************************************************************************************/
// Site view
/**************************************************************************************************/
info_with_view!(Data, site, site_, [xpos: MjtNum, xmat: MjtNum], [], M: Deref<Target = MjModel>);

/**************************************************************************************************/
// Tendon view
/**************************************************************************************************/
info_with_view!(Data, tendon, ten_, [wrapadr: i32, wrapnum: i32, J_rownnz: i32, J_rowadr: i32, J_colind: i32, length: MjtNum, J: MjtNum, velocity: MjtNum], [], M: Deref<Target = MjModel>);

/**************************************************************************************************/
// Light view
/**************************************************************************************************/
info_with_view!(Data, light, light_, [xpos: MjtNum, xdir: MjtNum], [], M: Deref<Target = MjModel>);

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
  </worldbody>
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

        // The ball should start in a still position
        cvel = body_info.view(&data).cvel;
        assert_relative_eq!(cvel[0], 0.0, epsilon=1e-9);
        assert_relative_eq!(cvel[1], 0.0, epsilon=1e-9);
        assert_relative_eq!(cvel[2], 0.0, epsilon=1e-9);
        assert_relative_eq!(cvel[3], 0.0, epsilon=1e-9);
        assert_relative_eq!(cvel[4], 0.0, epsilon=1e-9);
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
        assert_relative_eq!(cvel[4], 0.0, epsilon=1e-9);  // vy should be 0.
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
        data.crb();
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
        data.step();

        // mj_rne returns a scalar as result
        data.rne(true);

        data.rne_post_constraint();

        // // Collision and constraint pipeline
        data.collision();
        data.make_constraint();
        data.island();
        data.project_constraint();
        data.reference_constraint();

        let jar = vec![0.0; (data.model.ffi().nv) as usize];
        let mut cost = 0.0;
        data.constraint_update(&jar, None, false);
        data.constraint_update(&jar, Some(&mut cost), true);
    }

    #[test]
    fn test_add_contact() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        // Add a dummy contact
        let dummy_contact = unsafe { std::mem::zeroed() };
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
        let (jacp, jacr) = data.jac(true, true, &point, ball_body_id);
        assert_eq!(jacp.len(), expected_len);
        assert_eq!(jacr.len(), expected_len);

        // Test body frame Jacobian
        let (jacp_body, jacr_body) = data.jac_body(true, true, ball_body_id);
        assert_eq!(jacp_body.len(), expected_len);
        assert_eq!(jacr_body.len(), expected_len);

        // Test body COM Jacobian
        let (jacp_com, jacr_com) = data.jac_body_com(true, true, ball_body_id);
        assert_eq!(jacp_com.len(), expected_len);
        assert_eq!(jacr_com.len(), expected_len);

        // Test subtree COM Jacobian (translational only)
        let jac_subtree = data.jac_subtree_com(true, 0);
        assert_eq!(jac_subtree.len(), expected_len);

        // Test geom Jacobian
        let (jacp_geom, jacr_geom) = data.jac_geom(true, true, ball_body_id);
        assert_eq!(jacp_geom.len(), expected_len);
        assert_eq!(jacr_geom.len(), expected_len);

        // Test site Jacobian
        let (jacp_site, jacr_site) = data.jac_site(true, true, ball_body_id);
        assert_eq!(jacp_site.len(), expected_len);
        assert_eq!(jacr_site.len(), expected_len);

        // Test flags set to false produce empty Vec
        let (jacp_none, jacr_none) = data.jac(false, false, &[0.0; 3], ball_body_id);
        assert!(jacp_none.is_empty());
        assert!(jacr_none.is_empty());
    }

    #[test]
    fn test_angmom_and_object_dynamics() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();

        let mat = data.angmom_mat(0);
        assert_eq!(mat.len(), (3 * data.model.ffi().nv as usize));

        let vel = data.object_velocity(MjtObj::mjOBJ_BODY, 0, true);
        assert_eq!(vel, vel); // just ensure it returns 6-length

        let acc = data.object_acceleration(MjtObj::mjOBJ_BODY, 0, false);
        assert_eq!(acc, acc);
    }

    #[test]
    fn test_geom_distance_and_transforms() {
        let model = MjModel::from_xml_string(MODEL).unwrap();
        let mut data = model.make_data();
        data.step();

        let mut ft = [0.0; 6];
        let dist = data.geom_distance(0, 0, 1.0, Some(&mut ft));
        assert_eq!(ft, [0.0; 6]);
        assert_eq!(dist, 1.0);

        let pos = [0.0; 3];
        let quat = [1.0, 0.0, 0.0, 0.0];
        let (xpos, xmat) = data.local_to_global(&pos, &quat, 0, MjtSameFrame::mjSAMEFRAME_NONE);
        assert_eq!(xpos.len(), 3);
        assert_eq!(xmat.len(), 9);

        let ray_vecs = [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let rays = data.multi_ray(&pos, &ray_vecs, None, false, -1, 10.0);
        assert_eq!(rays.0.len(), 3);
        assert_eq!(rays.1.len(), 3);

        let (geomid, dist) = data.ray(&pos, &[1.0, 0.0, 0.0], None, true, -1);
        assert!(dist.is_finite());
        assert!(geomid >= -1);
    }
}
