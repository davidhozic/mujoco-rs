use crate::mujoco_c::*;
use std::ffi::CString;
use std::ops::Deref;

use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;

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
    fn mujoco_cSimulate_Sync(sim: *mut mujoco_Simulate, state_only: bool);
    fn mujoco_cSimulate_destroy(sim: *mut mujoco_Simulate);
}


/// Wrapper around the C++ implementation of MujoCo viewer.
/// If you don't need the side UI, we recommend you use the Rust-native viewer [`crate::viewer::MjViewer`] instead.
/// 
/// # Safety
/// Calls to [`MjViewerCpp::render`] must be done only on the **main** thread!
/// For convenience [`MjViewerCpp`] implements both `Send` and `Sync`, however that is meant only for
/// syncing the viewer. 
/// 
/// Additionally, to allow certain flexibility while still maintaining
/// compatibility with the C++ code, [`MjViewerCpp`] keeps internal pointers to mjModel and mjData,
/// which are wrapped inside [`MjModel`] and [`MjData`], respectively.
/// This technically allows `model` and `data` to be modified
/// while the viewer keeps a pointer to them (their wrapped pointers).
/// Undefined behavior should not occur, however caution is advised as this is a violation
/// of the Rust's borrowing rules.
pub struct MjViewerCpp<M: Deref<Target = MjModel> + Clone + Send + Sync> {
    sim: *mut mujoco_Simulate,
    running: bool,

    user_scn: Box<MjvScene<M>>,
    _cam: Box<MjvCamera>,
    _opt: Box<MjvOption>,
    _pert: Box<MjvPerturb>,
}

impl<M: Deref<Target = MjModel> + Clone + Send + Sync> MjViewerCpp<M> {
    pub fn running(&self) -> bool {
        self.running
    }

    pub fn user_scn_mut(&mut self) -> &mut MjvScene<M> {
        &mut self.user_scn
    }

    /// Launches a wrapper around MuJoCo's C++ viewer. The `max_user_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    /// Unlike the Rust-native viewer ([`crate::viewer::MjViewer`]), this also accepts a `data` parameter.
    /// Additionally, this just returns a [`MjViewerCpp`] instance directly, without result
    /// as the initialization may fail internally in C++ anyway, which we have no way of checking.
    ///
    /// # Safety
    /// See [`MjViewerCpp`]'s note.
    pub fn launch_passive(model: M, data: &MjData<M>, max_user_geom: usize) -> Self {
        // Allocate on the heap as the data must not be moved due to C++ bindings
        let mut cam = Box::new(MjvCamera::default());
        let mut opt: Box<MjvOption> = Box::new(MjvOption::default());
        let mut pert = Box::new(MjvPerturb::default());
        let mut user_scn = Box::new(MjvScene::new(model.clone(), max_user_geom));
        let sim;
        unsafe {
            sim = mujoco_cSimulate_create(&mut *cam, &mut *opt, &mut *pert, user_scn.ffi_mut());
            mujoco_cSimulate_RenderInit(sim);

            let sim_usize = sim as usize;
            let model_raw = model.__raw();
            let data_raw = data.__raw();
            let model_usize = model_raw as usize;
            let data_usize = data_raw as usize;

            // Load on another thread, since the viewer internally blocks until loaded.
            // This is intentional and is the intended way of using the C++ viewer.
            std::thread::spawn(move || {
                let sim = sim_usize as *mut mujoco_Simulate;
                let m = model_usize as *mut mjModel_;
                let d = data_usize as *mut mjData_;
                let c_filename = CString::new("file.xml").unwrap();
                mujoco_cSimulate_Load(sim, m, d, c_filename.as_ptr());
            });
            mujoco_cSimulate_RenderStep(sim);
        }

        Self {sim, running: true, user_scn, _cam: cam, _opt: opt, _pert: pert}
    }

    /// Renders the simulation.
    ///
    /// # SAFETY
    /// This needs to be called periodically from the MAIN thread, otherwise
    /// GLFW stops working.
    pub fn render(&mut self) {
        unsafe {
            assert!(self.running, "render called after viewer has been closed!");
            self.running = mujoco_cSimulate_RenderStep(self.sim) == 1;
        }
    }

    /// Syncs the simulation state with the viewer as well as perform
    /// rendering on the viewer.
    pub fn sync(&mut self) {
        unsafe {
            mujoco_cSimulate_Sync(self.sim, false);
        }
    }
}

impl<M: Deref<Target = MjModel> + Clone + Send + Sync> Drop for MjViewerCpp<M> {
    fn drop(&mut self) {
        unsafe {
            mujoco_cSimulate_destroy(self.sim);
        }
    }
}

unsafe impl<M: Deref<Target = MjModel> + Clone + Send + Sync> Send for MjViewerCpp<M> {}
unsafe impl<M: Deref<Target = MjModel> + Clone + Send + Sync> Sync for MjViewerCpp<M> {}
