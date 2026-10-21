//! Wrapper around MuJoCo's original C++ viewer (also named Simulate).
//!
//! This module exposes [`MjViewerCpp`], which requires static linking against a patched MuJoCo
//! build. It is only available when the `cpp-viewer` Cargo feature is enabled.
//! For most use cases, the Rust-native [`crate::viewer::MjViewer`] is recommended instead.
use crate::wrappers::mj_model::traits::ModelTypeMut;
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_data::MjData;
use crate::util::LockUnpoison;
use crate::mujoco_c::*;

use log::{debug, warn};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

#[repr(C)]
struct mujoco_Simulate { _unused: [u8; 0] }

unsafe extern "C" {
    fn mujoco_cSimulate_create(
        cam: *mut mjvCamera,
        opt: *mut mjvOption,
        pert: *mut mjvPerturb,
        user_scn: *mut mjvScene,
    ) -> *mut mujoco_Simulate;
    fn mujoco_cSimulate_RenderInit(sim: *mut mujoco_Simulate);
    fn mujoco_cSimulate_Load(sim: *mut mujoco_Simulate, m: *mut mjModel_, d: *mut mjData_, displayed_filename: *const std::os::raw::c_char);
    fn mujoco_cSimulate_RenderStep(sim: *mut mujoco_Simulate) -> std::os::raw::c_int;
    fn mujoco_cSimulate_Sync(sim: *mut mujoco_Simulate, state_only: std::os::raw::c_int);
    fn mujoco_cSimulate_ExitRequest(sim: *mut mujoco_Simulate);
    fn mujoco_cSimulate_destroy(sim: *mut mujoco_Simulate);
}


/// Struct used as a shared state, containing the [`MjData`] owned by the viewer.
/// Also used for syncing the simulation state with the C++ viewer.
#[derive(Debug)]
pub struct MjCppViewerSharedState<M: ModelTypeMut> {
    simulate: Option<*mut mujoco_Simulate>,
    data: MjData<M>,
    running: Arc<AtomicBool>,
    user_geoms: Vec<MjvGeom>,
}

impl<M: ModelTypeMut> MjCppViewerSharedState<M> {
    fn new(simulate: *mut mujoco_Simulate, data: MjData<M>, running: Arc<AtomicBool>) -> Self {
        Self { simulate: Some(simulate), data, running, user_geoms: Vec::new() }
    }

    // Called by `MjViewerCpp::drop` before the Simulate instance is destroyed, so later syncs
    // from other threads become no-ops instead of using a dangling pointer.
    fn close(&mut self) {
        self.simulate.take()
            .expect("the viewer is already closed");
        self.running.store(false, Ordering::Relaxed);
    }

    /// Returns a reference to the [`MjData`] that the viewer displays.
    pub fn data(&self) -> &MjData<M> {
        &self.data
    }

    /// Returns a mutable reference to the [`MjData`] that the viewer displays.
    ///
    /// # Safety
    /// The caller must not replace the returned [`MjData`] (by assignment, [`std::mem::swap`],
    /// [`std::mem::replace`] or [`std::mem::take`]). The C++ viewer keeps raw pointers to the
    /// original model and data, thus a replacement would make those pointers dangle.
    pub unsafe fn data_mut(&mut self) -> &mut MjData<M> {
        &mut self.data
    }

    /// Returns `true` when the viewer is still running (is open) and `false` otherwise.
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Returns a slice of additional user geoms that will be drawn into the scene,
    /// in addition to the actual scene elements.
    /// Use [`Self::user_geoms_mut`] for a mutable reference to the actual [`Vec`] of the data.
    pub fn user_geoms(&self) -> &[MjvGeom] {
        &self.user_geoms
    }

    /// Returns a mutable reference to the [`Vec`] holding [`MjvGeom`] objects which
    /// are drawn upon each render call to the C++ (Simulate) viewer.
    ///
    /// # Safety
    /// Every geom must be valid for rendering with the viewer's model: each index field
    /// (`dataid`, `matid`, `texid`, `objid`) is `-1` or in range of its model array.
    pub unsafe fn user_geoms_mut(&mut self) -> &mut Vec<MjvGeom> {
        &mut self.user_geoms
    }

    /// Syncs the simulation state with the C++ viewer.
    /// Returns `true` when the viewer is still open.
    pub fn sync(&mut self) -> bool {
        if let Some(simulate) = self.simulate && self.running() {
            // SAFETY: `simulate` is valid until `close()`, which runs before it is destroyed.
            // Simulate locks its own mutex against the render thread, and `&mut self` ensures
            // no Rust reference into the model or data is alive while C++ writes to them.
            unsafe { mujoco_cSimulate_Sync(simulate, 0) };
            true
        } else { false }
    }
}

// SAFETY: the only non-Send field is the Simulate pointer. The state only uses it for
// `mujoco_cSimulate_Sync`, which locks Simulate's own mutex and is safe to call from any thread.
unsafe impl<M: ModelTypeMut + Send> Send for MjCppViewerSharedState<M> {}

/// Wrapper around the C++ implementation of MuJoCo viewer.
/// If you don't need the side UI, we recommend you use the Rust-native viewer [`crate::viewer::MjViewer`] instead.
///
/// The viewer owns the [`MjData`] and is neither `Send` nor `Sync`, so it stays on the main
/// thread that [`MjViewerCpp::launch_passive`] requires. To step and sync the simulation from
/// another thread, use [`MjViewerCpp::state`] in that thread.
#[derive(Debug)]
pub struct MjViewerCpp<M: ModelTypeMut> {
    sim: *mut mujoco_Simulate,

    state: Arc<Mutex<MjCppViewerSharedState<M>>>,
    running: Arc<AtomicBool>,

    /* Fields used by C++ Simulate. Needed here due to Simulate requiring pointers to them.
       We could potentially have them owned by the C++ viewer, however, that would then be a
       greater refactor. The latter could cause more merge conflicts when upgrading to newer
       MuJoCo versions, thus I recommend you do not change it, to whomever decides to
       attempt this in the future :) .*/
    user_scn: Box<MjvScene>,
    _cam: Box<MjvCamera>,
    _opt: Box<MjvOption>,
    _pert: Box<MjvPerturb>,
}

impl<M: ModelTypeMut> MjViewerCpp<M> {
    /// Returns whether the viewer window is still open.
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Launches a wrapper around MuJoCo's C++ viewer. The `max_user_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    /// Unlike the Rust-native viewer ([`crate::viewer::MjViewer`]), this also accepts a `data` parameter.
    /// Additionally, this just returns a [`MjViewerCpp`] instance directly, without result
    /// as the initialization may fail internally in C++ anyway, which we have no way of checking.
    ///
    /// # Safety
    /// Must be called on the **main** thread. It initializes GLFW and OpenGL, and the returned
    /// viewer renders on the same thread.
    ///
    /// # Panics
    /// Panics if the load thread panics.
    pub unsafe fn launch_passive(data: MjData<M>, max_user_geom: usize) -> Self {
        let model = data.model();

        // Boxed because the C++ viewer keeps pointers to these, so they must not move.
        let mut cam = Box::new(MjvCamera::default());
        let mut opt = Box::new(MjvOption::default());
        let mut pert = Box::new(MjvPerturb::default());
        let mut user_scn = Box::new(MjvScene::new(model, max_user_geom));

        // SAFETY: all pointer arguments are valid and heap-allocated above; the returned viewer
        // owns the boxes, so they outlive the Simulate instance.
        let simulate = unsafe { mujoco_cSimulate_create(&mut *cam, &mut *opt, &mut *pert, user_scn.ffi_mut()) };
        assert!(!simulate.is_null(), "mujoco_cSimulate_create returned a null pointer");
        let sim_usize = simulate as usize;

        let model_usize = model.as_raw_ptr() as usize;
        let data_usize = data.as_raw_ptr() as usize;

        // SAFETY: `simulate` is non-null; the caller guarantees this is the main thread.
        unsafe { mujoco_cSimulate_RenderInit(simulate) };

        // Load on another thread, since the viewer internally blocks until loaded.
        // This is intentional and is the intended way of using the C++ viewer.
        let load_thread = std::thread::spawn(move || {
            let sim = sim_usize as *mut mujoco_Simulate;
            let m = model_usize as *mut mjModel_;
            let d = data_usize as *mut mjData_;
            // SAFETY: m and d point into the heap allocations of `data`, which the returned
            // viewer owns, and `M: ModelTypeMut` gives it exclusive access to the model.
            unsafe { mujoco_cSimulate_Load(sim, m, d, c"file.xml".as_ptr()) };
        });

        while !load_thread.is_finished() {
            // SAFETY: same pointer and thread as `mujoco_cSimulate_RenderInit` above.
            let running = unsafe { mujoco_cSimulate_RenderStep(simulate) };
            if running == 0 {
                // Window closed during model load; stop rendering.
                warn!("the C++ viewer window closed while the model was loading");
                break;
            }
        }
        load_thread.join().unwrap();

        let running = Arc::new(AtomicBool::new(true));
        let state = Arc::new(Mutex::new(MjCppViewerSharedState::new(simulate, data, running.clone())));
        debug!("started the C++ viewer");
        Self {
            sim: simulate,
            state, running,
            user_scn, _cam: cam, _opt: opt, _pert: pert
        }
    }

    /// Returns the shared state that holds the simulation data.
    /// Clone the [`Arc`] to step and sync the simulation from another thread.
    pub fn state(&self) -> &Arc<Mutex<MjCppViewerSharedState<M>>> {
        &self.state
    }

    /// Synchronizes user-added visual-only geometries ([`MjvGeom`]) with the
    /// internal user scene, which MuJoCo's C++ Simulate viewer uses for the
    /// actual additional rendering of user geometries.
    /// 
    /// # Note
    /// This method clears the user-geom buffer contained in the shared state.
    ///
    /// # Panics
    /// Panics when the shared state holds more geoms than the `max_user_geom` given to
    /// [`MjViewerCpp::launch_passive`].
    pub fn sync_user_geoms(&mut self) {
        let geoms = &mut self.state.lock_unpoison().user_geoms;
        let n_geom = geoms.len();

        assert!(n_geom <= self.user_scn.maxgeom() as usize, "too many user geoms");

        // SAFETY: `n_geom <= maxgeom`.
        unsafe {
            self.user_scn.ffi_mut().ngeom = n_geom as i32;
            self.user_scn.geoms_mut()[..n_geom].clone_from_slice(geoms);
            geoms.clear();
        }
    }

    /// Renders the simulation.
    ///
    /// # Errors
    /// Returns `Err` when called after the viewer has already been closed.
    /// The call that detects the close event still returns `Ok(())` and flips
    /// the internal running state to false.
    pub fn render(&mut self) -> Result<(), &'static str> {
        if !self.running() {
            return Err("render called after viewer has been closed!");
        }

        // SAFETY: self.sim is valid until drop. The viewer is not `Send`, so this runs on the
        // main thread that `launch_passive` was called on.
        if unsafe { mujoco_cSimulate_RenderStep(self.sim) } == 0 {
            self.running.store(false, Ordering::Relaxed);
        }
        Ok(())
    }

    /// Syncs the simulation state with the viewer. The transfer runs in both directions.
    /// Returns an [`bool`] indicator of whether the viewer is still running (is open).
    pub fn sync(&mut self) -> bool {
        self.state.lock_unpoison().sync()
    }
}

/// Requests viewer exit and destroys the underlying C++ simulation handle.
impl<M: ModelTypeMut> Drop for MjViewerCpp<M> {
    fn drop(&mut self) {
        // Other threads may still hold the state; stop them from syncing before destruction.
        self.state.lock_unpoison().close();

        // SAFETY: self.sim is a valid non-null pointer and no thread can sync anymore.
        // ExitRequest signals the C++ side to shut down, and destroy frees the allocation.
        unsafe {
            mujoco_cSimulate_ExitRequest(self.sim);
            mujoco_cSimulate_destroy(self.sim);
        }
    }
}
