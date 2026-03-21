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
    fn mujoco_cSimulate_Sync(sim: *mut mujoco_Simulate, state_only: std::os::raw::c_int);
    fn mujoco_cSimulate_ExitRequest(sim: *mut mujoco_Simulate);
    fn mujoco_cSimulate_destroy(sim: *mut mujoco_Simulate);
}


/// Wrapper around the C++ implementation of MuJoCo viewer.
/// If you don't need the side UI, we recommend you use the Rust-native viewer [`crate::viewer::MjViewer`] instead.
///
/// # Safety
/// Calls to [`MjViewerCpp::render`] must be done only on the **main** thread!
/// For convenience [`MjViewerCpp`] implements both `Send` and `Sync`, however that is meant only for
/// syncing the viewer.
///
/// [`MjViewerCpp::launch_passive`] keeps internal pointers to mjModel and mjData.
/// The caller must ensure both remain alive and at a fixed address for the viewer's lifetime.
/// See [`MjViewerCpp::launch_passive`] for the full safety contract.
#[derive(Debug)]
pub struct MjViewerCpp {
    sim: *mut mujoco_Simulate,
    running: bool,

    user_scn: Box<MjvScene>,
    _cam: Box<MjvCamera>,
    _opt: Box<MjvOption>,
    _pert: Box<MjvPerturb>,
}

impl MjViewerCpp {
    pub fn running(&self) -> bool {
        self.running
    }

    pub fn user_scn_mut(&mut self) -> &mut MjvScene {
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
    /// The caller must ensure that both `model` and `data` remain alive and at a stable memory
    /// address for the entire lifetime of the returned [`MjViewerCpp`]. Dropping or moving the
    /// underlying [`MjModel`] or [`MjData`] while the viewer is alive is undefined behavior.
    /// Calls to [`MjViewerCpp::render`] must be done only on the **main** thread.
    pub unsafe fn launch_passive<M: Deref<Target = MjModel> + Clone + Send + Sync>(model: M, data: &MjData<M>, max_user_geom: usize) -> Self {
        // Allocate on the heap as the data must not be moved due to C++ bindings
        let mut cam = Box::new(MjvCamera::default());
        let mut opt: Box<MjvOption> = Box::new(MjvOption::default());
        let mut pert = Box::new(MjvPerturb::default());
        let mut user_scn = Box::new(MjvScene::new(model.clone(), max_user_geom));

        let sim = unsafe { mujoco_cSimulate_create(&mut *cam, &mut *opt, &mut *pert, user_scn.ffi_mut()) };
        assert!(!sim.is_null(), "mujoco_cSimulate_create returned a null pointer");
        let sim_usize = sim as usize;

        let model_usize = model.as_raw_ptr() as usize;
        let data_usize = data.as_raw_ptr() as usize;

        unsafe { mujoco_cSimulate_RenderInit(sim) };

        // Load on another thread, since the viewer internally blocks until loaded.
        // This is intentional and is the intended way of using the C++ viewer.
        let load_thread = std::thread::spawn(move || {
            let sim = sim_usize as *mut mujoco_Simulate;
            let m = model_usize as *mut mjModel_;
            let d = data_usize as *mut mjData_;
            let c_filename = CString::new("file.xml").unwrap();
            unsafe { mujoco_cSimulate_Load(sim, m, d, c_filename.as_ptr()) };
        });

        while !load_thread.is_finished() {
            let running = unsafe { mujoco_cSimulate_RenderStep(sim) };
            if running == 0 {
                // Window closed during model load; stop rendering.
                break;
            }
        }
        load_thread.join().unwrap();

        Self {sim, running: true, user_scn, _cam: cam, _opt: opt, _pert: pert}
    }

    /// Renders the simulation.
    ///
    /// # Errors
    /// Returns `Err` when called after the viewer has already been closed.
    /// The call that detects the close event still returns `Ok(())` and flips
    /// the internal running state to false.
    ///
    /// # Safety
    /// Must be called from the **main thread**. GLFW requires main-thread access; calling
    /// from any other thread causes undefined behaviour.
    pub unsafe fn render(&mut self) -> Result<(), &'static str> {
        if !self.running {
            return Err("render called after viewer has been closed!");
        }
        unsafe { self.running = mujoco_cSimulate_RenderStep(self.sim) == 1; }
        Ok(())
    }

    /// Syncs the simulation state with the viewer as well as performs
    /// rendering on the viewer.
    pub fn sync(&mut self) {
        if !self.running {
            return;
        }
        unsafe {
            mujoco_cSimulate_Sync(self.sim, 0);
        }
    }
}

impl Drop for MjViewerCpp {
    fn drop(&mut self) {
        unsafe {
            mujoco_cSimulate_ExitRequest(self.sim);
            mujoco_cSimulate_destroy(self.sim);
        }
    }
}

unsafe impl Send for MjViewerCpp {}
unsafe impl Sync for MjViewerCpp {}
