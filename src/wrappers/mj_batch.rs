//! A set of wrappers around the CPU-based batching API.
//!
//! # Example
//! ```
//! # use mujoco_rs::wrappers::mj_batch::MjBatch;
//! # use mujoco_rs::prelude::*;
//! let model = MjModel::from_xml_string("<mujoco/>").unwrap();
//! // Create a batch of 10 batched simulation instances. Giving `None` for the third parameter
//! // means the default `MjBatchConfig` is used.
//! let mut batch = MjBatch::new(&model, 10, None);
//!
//! // Obtaining state
//! let num_sim = batch.num_sim();  // 10
//! let state_size = batch.state_size();
//! let states = batch.state_flattened();  // `num_sim * state_size` elements
//!
//! let state_0 = batch.state_at(0);
//! let time_3 = batch.state_at(3)[0];  // element 0 of instance 3 (starting count of instances at 0)
//!
//! for state in batch.state_chunks() {
//!     assert_eq!(state.len(), state_size);
//! }
//!
//! // Modifying state
//! // SAFETY: the model has no history buffers, so no write can break a history cursor.
//! unsafe {
//!     batch.state_flattened_mut().fill(0.0);
//!     batch.state_at_mut(0)[0] = 1.0;
//!     for state in batch.state_chunks_mut() {
//!         state[0] = 2.0;
//!     }
//! }
//! ```
use std::slice::{ChunksExact, ChunksExactMut};
use std::ffi::{CStr, CString, c_void};
use std::ptr::{self, NonNull};
use std::mem::MaybeUninit;
use std::sync::Arc;
use std::fmt;

use bytemuck::Pod;

use crate::mujoco_c::{
    mjBatch, mjBatchConfig, mjBatchRecord, mjModel, mj_stateSize, mjb_defaultConfig, mjb_deleteBatch,
    mjb_error, mjb_expand, mjb_expanded, mjb_forward, mjb_getData, mjb_getModel, mjb_model,
    mjb_makeBatch, mjb_nstate, mjb_nthread, mjb_output, mjb_persistent, mjb_reset, mjb_rollout,
    mjb_setConst, mjb_state, mjb_status, mjb_step, mjb_warning
};
use crate::wrappers::mj_model::{MjModel, MjModelLayout, MjtStage};
use crate::util::{ERROR_BUF_LEN, checked_c_len, string_from_c};
use crate::wrappers::{MjData, MjWarningStat, MjtState};
use crate::mujoco_c::mjtWarning::mjNWARNING;
use crate::wrappers::mj_primitive::MjtNum;
use crate::wrappers::traits::ModelType;
use crate::error::MjBatchError;
use crate::builder_setters;

/// The threading configuration of a batch.
pub type MjBatchConfig = mjBatchConfig;

impl Default for MjBatchConfig {
    /// Set the default [`MjBatchConfig`] parameters: one thread per logical CPU, and one
    /// [`MjData`] per thread.
    /// Wraps [`mjb_defaultConfig`].
    fn default() -> Self {
        let mut uninit = MaybeUninit::uninit();
        // SAFETY: `mjb_defaultConfig` writes every field of the config.
        unsafe {
            mjb_defaultConfig(uninit.as_mut_ptr());
            uninit.assume_init()
        }
    }
}

/// A field or state recorded by [`MjBatch::rollout`] after every step.
/// This is a struct used to create [`mjBatchRecord`] inside [`MjBatch::rollout`].
/// It is NOT a wrapper around [`mjBatchRecord`].
pub struct MjBatchRecordBuilder<'b> {
    buffer: &'b mut [MjtNum],
    spec: u32
}

impl<'b> MjBatchRecordBuilder<'b> {
    /// Creates a new [`MjBatchRecordBuilder`] instance, registering the `buffer` as the record
    /// output buffer and `spec` as the bitmask ([`MjtState`]) of what information to include in the
    /// state of the `buffer`.
    ///
    /// # Panics
    /// A panic is triggered when `spec` is not a valid bitmask of [`MjtState`]
    /// (i.e., less than (1 << [`MjtState::mjNSTATE`])).
    pub fn new(buffer: &'b mut [MjtNum], spec: u32) -> Self {
        Self { buffer, spec: 0 }.spec(spec)
    }

    builder_setters! {
        buffer: &'b mut [MjtNum]; "output buffer into which [`MjBatch::rollout`] writes the state after each step.";
        spec:   u32 {
            assert!(*spec < (1 << MjtState::mjNSTATE as u32), "state spec/mask {spec:#x} has an unknown bit");
        };                        "bitmask ([`MjtState`]) of the state that the buffer holds.\n\n# Panics\nPanics if `value` is not less than (1 << [`MjtState::mjNSTATE`]).";
    }
}

impl fmt::Debug for MjBatchRecordBuilder<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MjBatchRecordBuilder")
            .field("spec", &self.spec)
            .field("len", &self.buffer.len())
            .finish_non_exhaustive()
    }
}

impl From<MjBatchRecordBuilder<'_>> for mjBatchRecord {
    /// Converts into the FFI type [`mjBatchRecord`], whose `buf` points into the buffer.
    fn from(record: MjBatchRecordBuilder<'_>) -> Self {
        Self { field: ptr::null(), spec: record.spec as i32, buf: record.buffer.as_mut_ptr().cast() }
    }
}

/// Wrapper around the [`mjBatch`] FFI handle.
/// The handle is an opaque struct to many simulation instances of one model and thread-pool.
pub struct MjBatch {
    handle: NonNull<mjBatch>,
    num_sim: usize,
    layout: Arc<MjModelLayout>,
}

impl MjBatch {
    /// Attempt to make a batched simulation, of `num_sim` instances, out of
    /// [`MjModel`] represented by `model`.
    /// `config` represents the threading configuration (see [`MjBatchConfig`]).
    ///
    /// Wraps [`mjb_makeBatch`].
    ///
    /// # Panics
    /// Panics if MuJoCo cannot create the batch (e.g. `num_sim` is 0, or the allocation fails),
    /// or if `num_sim` exceeds [`i32::MAX`].
    pub fn new(model: &MjModel, num_sim: usize, config: Option<MjBatchConfig>) -> Self {
        Self::try_new(model, num_sim, config).unwrap()
    }

    /// Fallible version of [`MjBatch::new`].
    ///
    /// # Errors
    /// Returns [`MjBatchError::MakeFailed`] when MuJoCo cannot create the batch (e.g. `num_sim`
    /// is 0, or the allocation fails).
    ///
    /// # Panics
    /// Panics if `num_sim` exceeds [`i32::MAX`].
    pub fn try_new(model: &MjModel, num_sim: usize, config: Option<MjBatchConfig>) -> Result<Self, MjBatchError> {
        let mut error = [0; ERROR_BUF_LEN];

        // SAFETY: the model handle is valid while `model` lives, `num_sim` is checked to fit in
        //         an `int`, and MuJoCo writes at most `error.len()` bytes to `error`.
        let ptr = unsafe { mjb_makeBatch(
            model.ffi(), checked_c_len(num_sim),
            config.as_ref().map_or(ptr::null(), |config| config),
            error.as_mut_ptr(), error.len() as i32
        ) };

        // SAFETY: MuJoCo NUL-terminates the message within `error`.
        let handle = NonNull::new(ptr)
            .ok_or_else(|| MjBatchError::MakeFailed(unsafe { string_from_c(error.as_ptr()) }))?;
        Ok(Self { handle, num_sim, layout: Arc::clone(model.layout()) })
    }

    /// Returns number of simulation instances used in the batch.
    pub fn num_sim(&self) -> usize { self.num_sim }

    /// Returns number of threads used in the batch, including the caller's.
    /// Wraps [`mjb_nthread`].
    pub fn num_thread(&self) -> usize {
        // SAFETY: handle is always valid while this MjBatch exists.
        //         The returned value is always >= 1, thus converting to usize is safe as well.
        unsafe { mjb_nthread(self.handle.as_ptr()) as usize }
    }

    /// Returns `true` if the batch keeps one [`MjData`] per
    /// simulation instance, `false` if it keeps one per thread.
    /// Wraps [`mjb_persistent`].
    pub fn persistent(&self) -> bool {
        // SAFETY: handle is always valid while this MjBatch exists.
        unsafe { mjb_persistent(self.handle.as_ptr()) != 0 }
    }

    /// Returns the size of the state vector of a single instance within this batch.
    /// Wraps [`mjb_nstate`].
    pub fn state_size(&self) -> usize {
        // SAFETY: handle is always valid while this MjBatch exists.
        //         The returned value is always >= 0, thus converting to usize is safe as well.
        unsafe { mjb_nstate(self.handle.as_ptr()) as usize }
    }

    /// Returns the flattened matrix of states, sized `num_sim x state_size`.
    /// Wraps [`mjb_state`].
    pub fn state_flattened(&self) -> &[MjtNum] {
        // SAFETY: handle is always valid while this MjBatch exists. MuJoCo allocates
        //         `nsim * nstate` elements at creation and never reallocates them.
        unsafe { std::slice::from_raw_parts(
            mjb_state(self.handle.as_ptr()),
            self.num_sim * self.state_size()
        ) }
    }

    /// Same as [`MjBatch::state_flattened`] but returning as mutable data.
    ///
    /// # Safety
    /// Every history cursor in the state rows must stay an integer in `0..nsample` of its
    /// buffer.
    pub unsafe fn state_flattened_mut(&mut self) -> &mut [MjtNum] {
        // SAFETY: handle is always valid while this MjBatch exists. MuJoCo allocates
        //         `nsim * nstate` elements at creation and never reallocates them.
        unsafe { std::slice::from_raw_parts_mut(
            mjb_state(self.handle.as_ptr()),
            self.num_sim * self.state_size()
        ) }
    }

    /// Returns a [`ChunksExact`] iterator, chunking over [`MjBatch::state_size`] elements of the
    /// flattened [`MjBatch::state_flattened`] slice.
    pub fn state_chunks(&self) -> ChunksExact<'_, MjtNum> {
        self.state_flattened().chunks_exact(self.state_size())
    }

    /// Same as [`MjBatch::state_chunks`] but returning as mutable data.
    ///
    /// # Safety
    /// Every history cursor in the state rows must stay an integer in `0..nsample` of its
    /// buffer.
    pub unsafe fn state_chunks_mut(&mut self) -> ChunksExactMut<'_, MjtNum> {
        let state_size = self.state_size();
        // SAFETY: the caller upholds the contract of `state_flattened_mut`.
        unsafe { self.state_flattened_mut() }.chunks_exact_mut(state_size)
    }

    /// Returns a slice of the state belonging to simulation `index`-th instance.
    /// Wraps [`mjb_state`] (via [`MjBatch::state_flattened`]).
    ///
    /// # Panics
    /// Panics if `index` is not less than [`num_sim`](MjBatch::num_sim).
    pub fn state_at(&self, index: usize) -> &[MjtNum] {
        let state_size = self.state_size();
        &self.state_flattened()[index * state_size..][..state_size]
    }

    /// Same as [`MjBatch::state_at`] but returning as mutable data.
    ///
    /// # Safety
    /// Every history cursor in the state rows must stay an integer in `0..nsample` of its
    /// buffer.
    ///
    /// # Panics
    /// Panics if `index` is not less than [`num_sim`](MjBatch::num_sim).
    pub unsafe fn state_at_mut(&mut self, index: usize) -> &mut [MjtNum] {
        let state_size = self.state_size();
        // SAFETY: the caller upholds the contract of `state_flattened_mut`.
        let states = unsafe { self.state_flattened_mut() };
        &mut states[index * state_size..][..state_size]
    }

    /// Returns the warning statistics of every simulation. Each row belongs to an individual
    /// simulation instance.
    /// Wraps [`mjb_warning`].
    pub fn warnings(&self) -> &[[MjWarningStat; mjNWARNING as usize]] {
        // SAFETY: handle is always valid while this MjBatch exists. MuJoCo allocates the buffer
        //         with `nsim * mjNWARNING` elements at creation and never reallocates it, and
        //         only `&mut self` calls write to it.
        unsafe { std::slice::from_raw_parts(
            mjb_warning(self.handle.as_ptr()).cast(),
            self.num_sim
        ) }
    }

    /// Returns the error message of the last call of `index`-th simulation, or [`None`] if the
    /// call raised no error.
    /// Wraps [`mjb_error`].
    ///
    /// # Panics
    /// Panics if `index` is not less than [`num_sim`](MjBatch::num_sim), or if the message is not
    /// valid UTF-8.
    pub fn error_at(&self, index: usize) -> Option<&str> {
        assert!(index < self.num_sim, "simulation index {index} is out of range [0, {})", self.num_sim);
        // SAFETY: handle is always valid while MjBatch is alive, `index < num_sim` and `num_sim`
        //         fits in an `int` (checked at construction).
        let error = unsafe { CStr::from_ptr(mjb_error(self.handle.as_ptr(), index as i32)) };
        // MuJoCo returns "" if there was no error.
        (!error.is_empty()).then(|| error.to_str().expect("invalid UTF-8 in MuJoCo's error"))
    }

    /// Returns the status of each simulation's last call. A status is non-zero if the call raised
    /// an error.
    /// Wraps [`mjb_status`].
    pub fn statuses(&self) -> &[i32] {
        // SAFETY: handle is always valid while MjBatch is alive, the array has `num_sim` elements.
        unsafe { std::slice::from_raw_parts(mjb_status(self.handle.as_ptr()), self.num_sim) }
    }

    /// Advance each selected simulation by `num_steps` calls of
    /// [`MjData::step`](crate::wrappers::mj_data::MjData::step), setting per-step states and
    /// recording per-step states.
    /// Wraps [`mjb_rollout`].
    ///
    /// # Parameters
    /// - `maybe_ids`: the selected simulations. Giving [`None`] selects all simulations.
    /// - `num_steps`: the number of steps of each selected simulation.
    /// - `states_and_mask`: the states set before each step and their
    ///   [`MjtState`] mask. The slice is a flattened
    ///   `num_ids` x `num_steps` x [`model.state_size(mask)`](MjModel::state_size) array.
    ///   Giving `None` sets no state.
    /// - `batch_records`: the buffers into which the state after each step is written and their
    ///   [`MjtState`] masks. Each buffer is a flattened
    ///   `num_ids` x `num_steps` x [`model.state_size(mask)`](MjModel::state_size) array.
    ///
    /// `num_ids` is the length of the slice in `maybe_ids`, or [`num_sim`](MjBatch::num_sim) for
    /// [`None`].
    ///
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process if `ids` inside `maybe_ids` is not sorted and
    /// unique, if it holds an id not less than [`num_sim`](MjBatch::num_sim), or if `num_steps` is 0.
    ///
    /// # Panics
    /// Panics if `ids.len()`, `num_steps` or `batch_records.len()` exceeds [`i32::MAX`], if the
    /// `states_and_mask` mask is not a subset of [`MjtState::mjSTATE_USER`], or if a buffer
    /// length is not `num_ids * num_steps * state_size`.
    ///
    /// # Example
    /// ```
    /// use mujoco_rs::wrappers::mj_primitive::MjtNum;
    /// use mujoco_rs::wrappers::mj_batch::{MjBatch, MjBatchRecordBuilder};
    /// use mujoco_rs::wrappers::mj_data::MjtState;
    /// use mujoco_rs::prelude::*;
    ///
    /// let model = MjModel::from_xml_string(r#"
    /// <mujoco>
    ///   <worldbody>
    ///     <body>
    ///       <joint name="hinge"/>
    ///       <geom size="0.1"/>
    ///     </body>
    ///   </worldbody>
    ///   <actuator>
    ///     <motor joint="hinge"/>
    ///   </actuator>
    /// </mujoco>
    /// "#).unwrap();
    /// let mut batch = MjBatch::new(&model, 2, None);
    ///
    /// let states: &[MjtNum] = [
    ///   [  // Simulation 0
    ///     // ctrl,  qfrc_applied
    ///     [  0.5,   -1.2],  // Step 0
    ///     [  0.4,   -1.0],  // Step 1
    ///   ],
    ///   [  // Simulation 1
    ///     [  0.0,   2.5],  // Step 0
    ///     [  0.2,   2.4],  // Step 1
    ///   ],
    /// ].as_flattened().as_flattened();
    /// let mask = MjtState::mjSTATE_CTRL as u32 | MjtState::mjSTATE_QFRC_APPLIED as u32;
    ///
    /// // One `qpos` value per simulation and step.
    /// let mut qpos = [0.0; 2 * 2];
    /// let failed = batch.rollout(
    ///     Some(&[0, 1]), 2,
    ///     Some((states, mask)), [MjBatchRecordBuilder::new(&mut qpos, MjtState::mjSTATE_QPOS as u32)]
    /// );
    /// assert_eq!(failed, 0);
    /// ```
    pub fn rollout<const N: usize>(
        &mut self, maybe_ids: Option<&[u32]>, num_steps: u32,
        states_and_mask: Option<(&[MjtNum], u32)>, batch_records: [MjBatchRecordBuilder<'_>; N]
    ) -> u32
    {
        // SAFETY: handle is always valid here.
        let model = unsafe { mjb_model(self.handle.as_ptr()) };
        let num_ids = maybe_ids.map_or(self.num_sim, |x| x.len());
        let num_sims_times_steps = num_ids.strict_mul(num_steps as usize);
        // C reads and writes the buffers without a length check.
        let assert_len = |mask: u32, len: usize| {
            // SAFETY: the batch's model is valid while this MjBatch exists, `mask` is valid.
            let state_size = unsafe { mj_stateSize(model, mask as i32) } as usize;
            assert_eq!(
                len, num_sims_times_steps.strict_mul(state_size),
                "buffer length does not match the state mask {mask:#x}"
            );
        };

        if let Some((states, mask)) = states_and_mask {
            // MuJoCo stops the process for a mask outside mjSTATE_USER.
            assert!(
                mask & !(MjtState::mjSTATE_USER as u32) == 0,
                "state mask {mask:#x} is not a subset of mjSTATE_USER"
            );
            assert_len(mask, states.len());
        }

        // Take `buf` from the borrow, so that C writes through a pointer of the live borrow.
        let c_records = batch_records.map(|batch_record| {
            assert_len(batch_record.spec, batch_record.buffer.len());
            mjBatchRecord::from(batch_record)
        });

        // SAFETY: handle is always valid here, ids, num_steps and batch_records have checked
        //         length, u32 has the same layout as i32, every buffer has the length C accesses,
        //         and each record borrows its buffer mutably for the call.
        unsafe { mjb_rollout(
            self.handle.as_ptr(), maybe_ids.map_or(ptr::null(), |ids| ids.as_ptr().cast()),
            maybe_ids.map_or(0, |ids| checked_c_len(ids.len())), checked_c_len(num_steps),
            states_and_mask.map_or(ptr::null(), |(states, _)| states.as_ptr()),
            states_and_mask.map_or(0, |(_, mask)| mask as i32),
            c_records.as_ptr(), checked_c_len(c_records.len())
        ) as u32 }
    }

    /// Copies the model of `index`-th simulation, with its per-simulation fields, into `model`.
    /// Wraps [`mjb_getModel`].
    ///
    /// # Errors
    /// Returns [`MjBatchError::IncompatibleModel`] if `model` is not compatible with the batch's
    /// model.
    ///
    /// # Panics
    /// Panics if `index` is not less than [`num_sim`](MjBatch::num_sim).
    pub fn copy_model_into(&self, index: usize, model: &mut MjModel) -> Result<(), MjBatchError> {
        assert!(index < self.num_sim, "simulation index {index} is out of range [0, {})", self.num_sim);

        // SAFETY: the batch's model is valid while this MjBatch exists.
        let (source, destination) = (unsafe { &*mjb_model(self.handle.as_ptr()) }, model.ffi());
        // The layout does not include these extra size fields. However, MuJoCo's batching API does
        // compare them and stops the process when they are not equal in value.
        let sizes = |m: &mjModel| [m.nnames, m.npaths, m.noct, m.ngravcomp, m.nbuffer];
        if self.layout != *model.layout() || sizes(source) != sizes(destination) {
            return Err(MjBatchError::IncompatibleModel {
                source: Arc::clone(&self.layout),
                destination: Arc::clone(model.layout()),
            });
        }

        // SAFETY: handle is always valid while MjBatch is alive, `index < num_sim` and `num_sim`
        //         fits in an `int` (checked at construction), and `model` has the batch's layout,
        //         which the copy keeps. `mjb_getModel` only reads the batch, under its internal
        //         mutex.
        unsafe {
            mjb_getModel(self.handle.as_ptr(), index as i32, model.ffi_mut());
        }

        Ok(())
    }

    /// Advance each selected simulation by `num_steps` calls. To step a particular set of simulation instances,
    /// give `maybe_ids` [`Some`] with a slice of simulation instance indices to step, otherwise pass [`None`] to
    /// step all simulation instances.
    ///
    /// Setting `stage` to [`MjtStage::mjSTAGE_NONE`] makes this method give the same results as
    /// if we would be calling [`MjData::step`](crate::wrappers::mj_data::MjData::step) on each
    /// [`MjData`] instance separately. This means the derived
    /// fields of state will lag behind one time-step. Setting `stage` to any other value than
    /// [`MjtStage::mjSTAGE_NONE`], will cause the batching API to compute
    /// [`MjData::forward`](crate::wrappers::mj_data::MjData::forward) with all stages up to
    /// `stage`.
    /// Wraps [`mjb_step`].
    ///
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process if `ids` inside `maybe_ids` is not sorted and unique, if it
    /// holds an id not less than [`num_sim`](MjBatch::num_sim), or if `num_steps` is 0.
    ///
    /// # Panics
    /// Panics if `ids.len()` or `num_steps` exceeds [`i32::MAX`].
    pub fn step(&mut self, maybe_ids: Option<&[u32]>, num_steps: u32, stage: MjtStage) -> u32 {
        // SAFETY: handle is always valid here, ids and num_steps have checked length, u32 has
        //         the same layout as i32, casting stage to i32 can't overflow.
        unsafe { mjb_step(
            self.handle.as_ptr(), maybe_ids.map_or(ptr::null(), |ids| ids.as_ptr().cast()),
            maybe_ids.map_or(0, |ids| checked_c_len(ids.len())),
            checked_c_len(num_steps),
            stage as i32
        ) as u32 }
    }

    /// Compute [`MjData::forward`](crate::wrappers::mj_data::MjData::forward) with all stages up to
    /// `stage` on each selected simulation. Giving `maybe_ids` [`None`] selects all simulations.
    /// Wraps [`mjb_forward`].
    ///
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process if `ids` inside `maybe_ids` is not sorted and
    /// unique, or if it holds an id not less than [`num_sim`](MjBatch::num_sim).
    ///
    /// # Panics
    /// Panics if `ids.len()` exceeds [`i32::MAX`].
    pub fn forward(&mut self, maybe_ids: Option<&[u32]>, stage: MjtStage) -> u32 {
        // SAFETY: handle is always valid here, ids have checked length, u32 has the same layout
        //         as i32, casting stage to i32 can't overflow.
        unsafe { mjb_forward(
            self.handle.as_ptr(), maybe_ids.map_or(ptr::null(), |ids| ids.as_ptr().cast()),
            maybe_ids.map_or(0, |ids| checked_c_len(ids.len())),
            stage as i32
        ) as u32 }
    }

    /// Reset each selected simulation back to a specific keyframe, then compute
    /// [`MjData::forward`](crate::wrappers::mj_data::MjData::forward) with all stages up to `stage`.
    ///
    /// Giving `maybe_ids` [`None`] selects all simulations. `keys` holds one keyframe index per
    /// selected simulation, where -1 resets that simulation to the model defaults. Giving [`None`]
    /// resets to the model defaults.
    /// Wraps [`mjb_reset`].
    ///
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process if `ids` inside `maybe_ids` is not sorted and
    /// unique, if it holds an id not less than [`num_sim`](MjBatch::num_sim), or if a key is not -1
    /// or a keyframe index of the model.
    ///
    /// # Panics
    /// - Panics if `ids.len()` exceeds [`i32::MAX`].
    /// - Panics if `keys` is `Some` and its length differs from the number of selected
    ///   simulations.
    pub fn reset(&mut self, maybe_ids: Option<&[u32]>, keys: Option<&[i32]>, stage: MjtStage) -> u32 {
        // MuJoCo reads one key per selected simulation before it checks the keys.
        if let Some(keys) = keys {
            let num_ids = maybe_ids.map_or(self.num_sim, |x| x.len());
            assert_eq!(keys.len(), num_ids, "keys must have one element per selected simulation");
        }

        // SAFETY: handle is always valid here, ids and keys have checked length, u32 has the
        //         same layout as i32, casting stage to i32 can't overflow.
        unsafe { mjb_reset(
            self.handle.as_ptr(), maybe_ids.map_or(ptr::null(), |ids| ids.as_ptr().cast()),
            maybe_ids.map_or(0, |ids| checked_c_len(ids.len())),
            keys.map_or(ptr::null(), <[i32]>::as_ptr),
            stage as i32
        ) as u32 }
    }

    /// Copies the state of `index`-th simulation into `data`. In a [persistent](MjBatch::persistent)
    /// batch, copies the whole [`MjData`] instead.
    /// Wraps [`mjb_getData`].
    ///
    /// # Errors
    /// Returns [`MjBatchError::IncompatibleModel`] if `data` was created from a model that is not
    /// compatible with the batch's model.
    ///
    /// # Panics
    /// Panics if `index` is not less than [`num_sim`](MjBatch::num_sim).
    pub fn copy_data_into<M: ModelType>(&self, index: usize, data: &mut MjData<M>) -> Result<(), MjBatchError> {
        assert!(index < self.num_sim, "simulation index {index} is out of range [0, {})", self.num_sim);

        // C only compares the total buffer sizes, which a model with a different layout can match.
        if self.layout != *data.model().layout() {
            return Err(MjBatchError::IncompatibleModel {
                source: Arc::clone(&self.layout),
                destination: Arc::clone(data.model().layout()),
            });
        }

        // SAFETY: handle is always valid while MjBatch is alive, `index < num_sim` and `num_sim`
        //         fits in an `int` (checked at construction), and `data` has the batch's layout.
        //         `mjb_getData` only reads the batch, under its internal mutex.
        unsafe {
            mjb_getData(self.handle.as_ptr(), index as i32, data.ffi_mut());
        }

        Ok(())
    }

    /// Reads the output `name` from instances of [`MjData`].
    /// The slice is of dimension `num_sim x num_elements(name)`.
    /// Wraps [`mjb_output`].
    ///
    /// # Note
    /// The output type `T` is only partially enforced, specifically in size and alignment.
    /// These enforcements do not prevent the type from being wrong. For example, you may pass
    /// `T` to be [`f32`] while the real type would be [`i32`] and the call would succeed.
    /// This is not undefined behavior, however, the data itself will be incorrectly interpreted.
    ///
    /// # Errors
    /// - Returns [`MjBatchError::InvalidOutput`] if `name` is not an [`MjData`] field or is a
    ///   state field (read state fields with [`state_at`](MjBatch::state_at)).
    /// - Returns [`MjBatchError::FieldTypeMismatch`] if `T` is not the same byte-size
    ///   of the field named `name` or if the output data is not aligned to `T`.
    ///
    /// # Panics
    /// Panics if `name` contains a NUL byte.
    pub fn output<T: Pod>(&mut self, name: &str) -> Result<&[T], MjBatchError> {
        let field = self.mjb_output(name)?;
        let (ptr, len) = self.typed_field::<T>(name, field)?;
        // SAFETY: pointer and sizes and alignment verified by `typed_field`.
        Ok(unsafe { std::slice::from_raw_parts(ptr.as_ptr(), len) })
    }

    /// Registers the output `name` to be filled at the next batch call with [`MjData`] field data
    /// of every simulation instance in the batch.
    /// Wraps [`mjb_output`].
    ///
    /// # Errors
    /// Returns [`MjBatchError::InvalidOutput`] if `name` is not an [`MjData`] field or is a state
    /// field (read state fields with [`state_at`](MjBatch::state_at)).
    ///
    /// # Panics
    /// Panics if `name` contains a NUL byte.
    pub fn output_register(&mut self, name: &str) -> Result<(), MjBatchError> {
        self.mjb_output(name).map(|_| ())
    }

    /// Registers the output `name` and returns its storage, its element count per simulation and
    /// its element size in bytes.
    fn mjb_output(&mut self, name: &str) -> Result<(NonNull<c_void>, usize, usize), MjBatchError> {
        let (mut len, mut element_size) = (0, 0);
        let c_string = CString::new(name).expect("name contained nul-bytes");

        // SAFETY: handle is always valid while MjBatch is alive and `c_string` is NUL-terminated.
        let ptr = unsafe { mjb_output(
            self.handle.as_ptr(), c_string.as_ptr(),
            &mut len, &mut element_size
        ) };

        // Null for an unknown name and for a state field.
        let ptr = NonNull::new(ptr).ok_or_else(|| MjBatchError::InvalidOutput(name.to_owned()))?;
        Ok((ptr, len as usize, element_size as usize))
    }

    /// Makes the `name` field of [`MjModel`] be separate for each simulation instance.
    /// This means that, before each step, the parameter named `name` will be copied from
    /// this method's returned slice into [`MjModel`] instances inside corresponding threads.
    /// The slice is of dimension `num_sim x num_elements(name)`.
    /// Wraps [`mjb_expand`].
    ///
    /// # Note
    /// The type `T` is enforced the same way as in [`output`](MjBatch::output).
    ///
    /// # Safety
    /// MuJoCo uses some [`MjModel`] fields as indices, addresses or counts (e.g., `body_parentid`,
    /// `jnt_qposadr`). Values written to such a field must stay valid for the model.
    ///
    /// # Errors
    /// - Returns [`MjBatchError::InvalidExpand`] if `name` is not an [`MjModel`] field or is an
    ///   asset field.
    /// - Returns [`MjBatchError::FieldTypeMismatch`] if `T` is not the same byte-size
    ///   of the field named `name` or if the expanded data is not aligned to `T`.
    ///
    /// # Panics
    /// Panics if `name` contains a NUL byte.
    pub unsafe fn expand<T: Pod>(&mut self, name: &str) -> Result<&mut [T], MjBatchError> {
        let (mut len, mut element_size) = (0, 0);
        let c_string = CString::new(name).expect("name contained nul-bytes");

        // SAFETY: handle is always valid while MjBatch is alive and `c_string` is NUL-terminated.
        let ptr = unsafe { mjb_expand(
            self.handle.as_ptr(), c_string.as_ptr(),
            &mut len, &mut element_size
        ) };

        // Null for an unknown name and for an asset field.
        let ptr = NonNull::new(ptr).ok_or_else(|| MjBatchError::InvalidExpand(name.to_owned()))?;
        let (ptr, len) = self.typed_field::<T>(name, (ptr, len as usize, element_size as usize))?;
        // SAFETY: as in `output`; `&mut self` makes the borrow exclusive.
        Ok(unsafe { std::slice::from_raw_parts_mut(ptr.as_ptr(), len) })
    }

    /// Returns the expanded `name` field of [`MjModel`] without expanding it, as a slice.
    /// The slice is of dimension `num_sim x num_elements(name)`.
    /// Wraps [`mjb_expanded`].
    ///
    /// # Note
    /// The type `T` is enforced the same way as in [`output`](MjBatch::output).
    ///
    /// # Errors
    /// - Returns [`MjBatchError::NotExpanded`] if `name` is not an [`MjModel`] field, is an asset
    ///   field or is not expanded.
    /// - Returns [`MjBatchError::FieldTypeMismatch`] if `T` is not the same byte-size
    ///   of the field named `name` or if the expanded data is not aligned to `T`.
    ///
    /// # Panics
    /// Panics if `name` contains a NUL byte.
    pub fn expanded<T: Pod>(&self, name: &str) -> Result<&[T], MjBatchError> {
        let (mut len, mut element_size) = (0, 0);
        let c_string = CString::new(name).expect("name contained nul-bytes");

        // SAFETY: handle is always valid while MjBatch is alive.
        // `c_string` is NUL-terminated.
        let ptr = unsafe { mjb_expanded(
            self.handle.as_ptr(), c_string.as_ptr(),
            &mut len, &mut element_size
        ) };

        // Null for an unknown name, for an asset field and for a field that is not expanded.
        let ptr = NonNull::new(ptr.cast_mut())
            .ok_or_else(|| MjBatchError::NotExpanded(name.to_owned()))?;
        let (ptr, len) = self.typed_field::<T>(name, (ptr, len as usize, element_size as usize))?;
        // SAFETY: as in `output`.
        Ok(unsafe { std::slice::from_raw_parts(ptr.as_ptr(), len) })
    }

    /// Checks that `T` matches the elements of the `num_sim x len` storage at `ptr` and returns
    /// the typed pointer and the total element count.
    fn typed_field<T: Pod>(
        &self, name: &str, (ptr, len, element_size): (NonNull<c_void>, usize, usize)
    ) -> Result<(NonNull<T>, usize), MjBatchError>
    {
        let ptr = ptr.cast::<T>();
        if size_of::<T>() != element_size || !ptr.is_aligned() {
            return Err(MjBatchError::FieldTypeMismatch { name: name.to_owned(), element_size });
        }
        Ok((ptr, self.num_sim * len))
    }

    /// Apply [`MjData::set_const`] to each selected simulation instance inside the batch, on its
    /// own [`MjModel`] values. Giving `maybe_ids` [`None`] selects all simulations.
    /// Read the constants that changed between simulations with [`expanded`](MjBatch::expanded).
    /// Wraps [`mjb_setConst`].
    ///
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process if `ids` inside `maybe_ids` is not sorted and
    /// unique, or if it holds an id not less than [`num_sim`](MjBatch::num_sim).
    ///
    /// # Panics
    /// Panics if `ids.len()` exceeds [`i32::MAX`].
    pub fn set_const(&mut self, maybe_ids: Option<&[u32]>) -> u32 {
        // SAFETY: handle is always valid here, ids have checked length, u32 has the same layout
        //         as i32. The batch never runs on caller models, on which MuJoCo aborts.
        unsafe { mjb_setConst(
            self.handle.as_ptr(), maybe_ids.map_or(ptr::null(), |ids| ids.as_ptr().cast()),
            maybe_ids.map_or(0, |ids| checked_c_len(ids.len()))
        ) as u32 }
    }
}

impl fmt::Debug for MjBatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MjBatch")
            .field("num_sim", &self.num_sim)
            .field("num_thread", &self.num_thread())
            .finish_non_exhaustive()
    }
}

impl Drop for MjBatch {
    fn drop(&mut self) {
        // SAFETY: the handle pointer is fully owned by `self`.
        // The handle is checked on creation and cannot be invalid afterwards.
        unsafe { mjb_deleteBatch(self.handle.as_ptr()); }
    }
}

// SAFETY: the batch owns its model copy, MuJoCo sets its thread-local state only for the duration
//         of a call, and any thread can make calls on the thread pool or delete the batch.
unsafe impl Send for MjBatch {}

// SAFETY: non-mutable references do not mutate internally. Regardless, an internal mutex is present
//         inside MuJoCo.
unsafe impl Sync for MjBatch {}
