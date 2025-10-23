use crate::mujoco_c::*;
use std::ffi::CString;
use std::ops::Deref;

use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;


/// Wrapper around the C++ implementation of MujoCo viewer.
/// If you don't need the side UI, we recommend you use the Rust-native viewer [`crate::viewer::MjViewer`] instead.
pub struct MjViewerCpp<M: Deref<Target = MjModel> + Clone> {
    sim: *mut mujoco_Simulate,
    running: bool,

    // Store these here since the C++ bindings save references to them.
    // We don't actually need them ourselves, at least not here.
    _cam: Box<MjvCamera>,
    _opt: Box<MjvOption>,
    _pert: Box<MjvPerturb>,
    _user_scn: Box<MjvScene<M>>,
}

impl<M: Deref<Target = MjModel> + Clone> MjViewerCpp<M> {
    #[inline]
    pub fn running(&self) -> bool {
        self.running
    }

    #[inline]
    pub fn user_scn_mut(&mut self) -> &mut MjvScene<M> {
        &mut self._user_scn
    }

    /// Launches a wrapper around MuJoCo's C++ viewer. The `scene_max_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    /// Unlike the Rust-native viewer ([`crate::viewer::MjViewer`]), this also accepts a `data` parameter.
    /// Additionally, this just returns a [`MjViewerCpp`] instance directly, without result
    /// as the initialization may fail internally in C++ anyway, which we have no way of checking.
    ///
    /// # Safety
    /// To allow certain flexibility, while still maintaining
    /// compatibility with the C++ code, [`MjViewerCpp`] keeps internals pointers to mjModel and mjData,
    /// which are wrapped inside [`MjModel`] and [`MjData`], respectively.
    /// This technically allows `model` and `data` to be modified
    /// while the viewer keeps a pointer to them (their wrapped pointers).
    /// Undefined behavior should not occur, however caution is advised as this is a violation
    /// of the Rust's borrowing rules.
    pub fn launch_passive(model: M, data: &MjData<M>, scene_max_geom: usize) -> Self {
        // Allocate on the heap as the data must not be moved due to C++ bindings
        let mut _cam = Box::new(MjvCamera::default());
        let mut _opt: Box<MjvOption> = Box::new(MjvOption::default());
        let mut _pert = Box::new(MjvPerturb::default());
        let mut _user_scn = Box::new(MjvScene::new(model.clone(), scene_max_geom));
        let sim;
        let c_filename = CString::new("file.xml").unwrap();
        unsafe {
            sim = mujoco_cSimulate_create(&mut *_cam, &mut *_opt, &mut *_pert, _user_scn.ffi_mut(), true);
            mujoco_cSimulate_RenderInit(sim);
            mujoco_cSimulate_Load(sim, model.__raw(), data.__raw(), c_filename.as_ptr());
            mujoco_cSimulate_RenderStep(sim, true);
        }

        Self {sim, running: true, _cam, _opt, _pert, _user_scn}
    }

    /// Returns the underlying C++ binding object of the viewer.
    pub fn __raw(&self) -> *mut mujoco_Simulate {
        self.sim
    }

    /// Renders the simulation.
    /// `update_timer` flag specifies whether the time should be updated
    /// inside the viewer (for FPS calculation).
    /// # SAFETY
    /// This needs to be called periodically from the MAIN thread, otherwise
    /// GLFW stops working.
    pub fn render(&mut self, update_timer: bool) {
        unsafe {
            assert!(self.running, "render called after viewer has been closed!");
            self.running = mujoco_cSimulate_RenderStep(self.sim, update_timer) == 1;
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

impl<M: Deref<Target = MjModel> + Clone> Drop for MjViewerCpp<M> {
    fn drop(&mut self) {
        unsafe {
            mujoco_cSimulate_RenderCleanup(self.sim);
            mujoco_cSimulate_destroy(self.sim);
        }
    }
}
