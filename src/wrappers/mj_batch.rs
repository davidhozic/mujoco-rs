//! A set of wrappers around the CPU-based batching API.
//! 
//! # Example
//! ```
//! # use mujoco_rs::wrappers::mj_batch::MjBatch;
//! # use mujoco_rs::prelude::*;
//! let model = MjModel::from_xml_string("<mujoco/>").unwrap();
//! // Create a batch of 10 batched simulation instances. Giving `None` for the second parameter
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
//! batch.state_flattened_mut().fill(0.0);
//! batch.state_at_mut(0)[0] = 1.0;
//! for state in batch.state_chunks_mut() {
//!     state[0] = 2.0;
//! }
//! ```
use std::slice::{ChunksExact, ChunksExactMut};
use std::ptr::{self, NonNull};
use std::mem::MaybeUninit;

use crate::mujoco_c::{mjBatch, mjBatchConfig, mjb_defaultConfig, mjb_deleteBatch, mjb_makeBatch, mjb_nstate, mjb_state, mjb_step};
use crate::util::{ERROR_BUF_LEN, checked_c_len, string_from_c};
use crate::wrappers::mj_model::{MjModel, MjtStage};
use crate::wrappers::mj_primitive::MjtNum;
use crate::error::MjBatchError;

/// The threading configuration of a batch.
pub type MjBatchConfig = mjBatchConfig;

impl Default for MjBatchConfig {
    /// Set the default mjBatchConfig parameters: one thread per logical CPU, and one mjData per thread.
    fn default() -> Self {
        let mut uninit = MaybeUninit::uninit();
        // SAFETY:
        unsafe {
            mjb_defaultConfig(uninit.as_mut_ptr());
            uninit.assume_init()
        }
    }
}

/// Wrapper around the [`mjBatch`] FFI handle.
/// The handle is an opaque struct to many simulation instances of one model and thread-pool.
pub struct MjBatch {
    handle: NonNull<mjBatch>,
    num_sim: usize,
}

impl MjBatch {
    /// Attempt to make a batched simulation, of `num_sim` instances, out of
    /// [`MjModel`](crate::wrappers::MjModel) represented by `model`.
    /// `config` represents the threading configuration (see [`MjBatchConfig`]).
    /// 
    /// Wraps [`mjb_makeBatch`].
    ///     
    /// # Panics
    /// Panics on unwraps of errors returned by [`MjBatch::try_new`]
    ///  (e.g. `num_sim` is 0, the allocation fails, etc.).
    pub fn new(model:&MjModel, num_sim: usize, config: Option<MjBatchConfig>) -> Self {
        Self::try_new(model, num_sim, config).unwrap()
    }

    /// Fallible version of [`MjBatch::new`].
    /// 
    /// # Errors
    /// Returns [`MjBatchError::MakeFailed`] when MuJoCo cannot create the batch (e.g. `num_sim`
    /// is 0, or the allocation fails).
    pub fn try_new(model: &MjModel, num_sim: usize, config: Option<MjBatchConfig>) -> Result<Self, MjBatchError> {
        let mut error = [0; ERROR_BUF_LEN];

        // SAFETY: model FFI handle is always valid while `MjModel` exists, `num_sim` is checked for overflows,
        // error buffer cannot overflow.
        let ptr = unsafe { mjb_makeBatch(
            model.ffi(), checked_c_len(num_sim),
            config.as_ref().map_or(ptr::null(), |c| c),
            error.as_mut_ptr(), error.len() as i32
        ) };

        // SAFETY: MuJoCo NUL-terminates the message within `error`.
        let handle = NonNull::new(ptr)
            .ok_or_else(|| MjBatchError::MakeFailed(unsafe { string_from_c(error.as_ptr()) }))?;
        Ok(Self { handle, num_sim })
    }

    /// Returns number of simulation instances used in the batch.
    pub fn num_sim(&self) -> usize { self.num_sim }

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
        // SAFETY: handle is always valid while this MjBatch exists. Lengths are guaranteed by MuJoCo to be correct.
        unsafe { std::slice::from_raw_parts(
            mjb_state(self.handle.as_ptr()),
            self.num_sim * self.state_size()
        ) }
    }

    /// Same as [`MjBatch::state_flattened`] but returning as mutable data.
    pub fn state_flattened_mut(&mut self) -> &mut [MjtNum] {
        // SAFETY: handle is always valid while this MjBatch exists. Lengths are guaranteed by MuJoCo to be correct.
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
    pub fn state_chunks_mut(&mut self) -> ChunksExactMut<'_, MjtNum> {
        let state_size = self.state_size();
        self.state_flattened_mut().chunks_exact_mut(state_size)
    }

    /// Returns a slice of the state belonging to simulation `index`-th instance.
    /// Wraps [`mjb_state`] (via [`MjBatch::state_flattened`])
    pub fn state_at(&self, index: usize) -> &[MjtNum] {
        let state_size = self.state_size();
        &self.state_flattened()[index * state_size..][..state_size]
    }

    /// Same as [`MjBatch::state_at`] but returning as mutable data.
    pub fn state_at_mut(&mut self, index: usize) -> &mut [MjtNum] {
        let state_size = self.state_size();
        &mut self.state_flattened_mut()[index * state_size..][..state_size]
    }

    /// Advance each selected simulation by `num_step` calls.
    /// 
    /// Setting `stage` to [`MjtStage::mjSTAGE_NONE`] makes this method give the same results as if we
    /// would be calling [`MjData::step`](crate::wrappers::mj_data::MjData::step) on each
    /// [`MjData`](crate::wrappers::mj_data::MjData) instance separately. This means the derived fields
    /// of state will lag behind one time-step. Setting `stage` to any other value than [`MjtStage::mjSTAGE_NONE`],
    /// will cause the batching API to compute [`MjData::forward`](crate::wrappers::mj_data::MjData::forward)
    /// with all stages up to `stage`.
    /// 
    /// # Returns
    /// Number of simulations in which MuJoCo raised an error.
    /// 
    /// # Panics
    /// Panics if `ids.len()` or `num_steps` exceeds [`i32::MAX`].
    pub fn step(&mut self, ids: &[i32], num_steps: u32, stage: MjtStage) -> u32 {
        // SAFETY: handle is always valid here, ids are of correct type and have checked length,
        //         num_steps have checked length, casting stage to i32 can't overflow.
        unsafe { mjb_step(
            self.handle.as_ptr(), ids.as_ptr(),
            checked_c_len(ids.len()), checked_c_len(num_steps),
            stage as i32
        ) as u32 }
    }
}

impl Drop for MjBatch {
    fn drop(&mut self) {
        // SAFETY: the handle pointer is fully owned by `self`.
        // The handle is checked on creation and cannot be invalid afterwards.
        unsafe { mjb_deleteBatch(self.handle.as_ptr()); }
    }
}
