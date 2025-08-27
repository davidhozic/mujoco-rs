use std::ffi::CString;
use glfw;

use crate::mujoco_c::{mujoco_Simulate, new_simulate, free_simulate};
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;

/// Wrapper around the C++ implementation of MujoCo viewer
/// # SAFETY
/// Due to performance reasons and PyO3, this must be destroyed before
/// [`MjData`] and [`MjModel`] instances that are passed in the constructor.
/// Normally, we would include references to them but it's very inconvenient.
pub struct MjViewer<'m> {
    sim: *mut mujoco_Simulate,
    running: bool,

    // Store these here since the C++ bindings save references to them.
    // We don't actually need them ourselves, at least not here.
    _cam: Box<MjvCamera>,
    _opt: Box<MjvOption>,
    _pert: Box<MjvPerturb>,
    _user_scn: Box<MjvScene<'m>>,
    _glfw: glfw::Glfw
}




impl<'m> MjViewer<'m> {
    #[inline]
    pub fn running(&self) -> bool {
        self.running
    }

    #[inline]
    pub fn user_scn_mut(&mut self) -> &mut MjvScene<'m> {
        &mut self._user_scn
    }

    pub fn launch_passive(model: &'m MjModel, data: &MjData, scene_max_ngeom: usize) -> Self {
        let mut _glfw = glfw::init(glfw::fail_on_errors).unwrap();

        // Allocate on the heap as the data must not be moved due to C++ bindings
        let mut _cam = Box::new(MjvCamera::default());
        let mut _opt: Box<MjvOption> = Box::new(MjvOption::default());
        let mut _pert = Box::new(MjvPerturb::default());
        let mut _user_scn = Box::new(MjvScene::new(&model, scene_max_ngeom));
        let sim;
        unsafe {
            sim = new_simulate(&mut *_cam, &mut *_opt, &mut *_pert, _user_scn.ffi_mut(), true);
            (*sim).RenderInit();
            (*sim).Load(model.__raw(), data.ffi_mut(), CString::new("file.xml").unwrap().as_ptr());
            (*sim).RenderStep(true);
        }

        Self {sim, running: true, _cam, _opt, _pert, _glfw, _user_scn}
    }

    /// Returns the underlying C++ binding object of the viewer.
    pub fn raw(&mut self) -> *mut mujoco_Simulate {
        self.sim
    }

    /// Renders the simulation.
    /// `update_timer` flag species whether the time should be updated
    /// inside the viewer (for vsync purposes).
    /// # SAFETY
    /// This needs to be called periodically from the MAIN thread, otherwise
    /// GLFW stops working.
    pub fn render(&mut self, update_timer: bool) {
        unsafe {
            assert!(self.running, "render called after viewer has been closed!");
            self.running = (*self.sim).RenderStep(update_timer);
        }
    }

    pub fn sync(&mut self) {
        unsafe {
            (*self.sim).Sync();
        }
    }
}


impl Drop for MjViewer<'_> {
    fn drop(&mut self) {
        unsafe {
            (*self.sim).RenderCleanup();
            free_simulate(self.sim);
        }
    }
}
