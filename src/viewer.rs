use glfw::{Action, Context, Glfw, GlfwReceiver, Key, PWindow, WindowEvent, MouseButton};

use std::ffi::CString;

use crate::mujoco_c::{free_simulate, mjtNum, mujoco_Simulate, new_simulate};
use crate::prelude::{MjrContext, MjrRectangle};
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;


/****************************************** */
// Rust native viewer
/****************************************** */
const MJ_VIEWER_DEFAULT_SIZE_PX: (u32, u32) = (1280, 720);
const MJ_VIEWER_DEFAULT_TITLE: &str = "MuJoCo Viewer (Rust)";

#[derive(Debug)]
pub enum MjViewerError {
    GlfwInitError (glfw::InitError),
    WindowCreationError
}

#[derive(Debug)]
pub struct MjViewer<'m> {
    /* MuJoCo rendering */
    scene: MjvScene<'m>,
    context: MjrContext,
    camera: MjvCamera,

    /* Other MuJoCo related */
    model: &'m MjModel,

    /* Internal state */
    last_x: mjtNum,
    last_y: mjtNum,

    /* OpenGL */
    glfw: Glfw,
    window: PWindow,
    events: GlfwReceiver<(f64, WindowEvent)>
}

impl<'m> MjViewer<'m> {
    /// Launches the MuJoCo viewer. A [`Result`] struct is returned that either contains
    /// [`MjViewer`] or a [`MjViewerError`].
    pub fn launch_passive(model: &'m MjModel, scene_max_ngeom: usize) -> Result<Self, MjViewerError> {
        let mut glfw = glfw::init_no_callbacks()
            .map_err(|err| MjViewerError::GlfwInitError(err))?;
        let (w, h) = MJ_VIEWER_DEFAULT_SIZE_PX;
        let (mut window, events) = match glfw.create_window(
            w, h, MJ_VIEWER_DEFAULT_TITLE, glfw::WindowMode::Windowed
        ) {
            Some(x) => Ok(x),
            None => Err(MjViewerError::WindowCreationError)
        }?;

        /* Initialize the OpenGL related things */
        window.make_current();
        window.set_all_polling(true);

        let scene = MjvScene::new(model, scene_max_ngeom);
        let context= MjrContext::new(model);
        let camera = MjvCamera::new(0, MjtCamera::mjCAMERA_FREE, model);
        Ok(Self {
            scene,
            context,
            camera,
            model,
            glfw,
            window,
            events,
            last_x: 0.0,
            last_y: 0.0
        })
    }

    /// Checks whether the window is still open.
    pub fn running(&mut self) -> bool {
        !self.window.should_close()
    }

    pub fn sync(&mut self, data: &mut MjData) {
        self.process_events();
        self.update(data);
    }

    /// Processes user input events
    fn process_events(&mut self) {
        self.glfw.poll_events();
        for (_, event) in  glfw::flush_messages(&mut self.events) {
            match event {
                WindowEvent::Key(Key::Q, _, _, _) => self.window.set_should_close(true),
                WindowEvent::Scroll(_, change) => {
                    self.camera.move_(MjtMouse::mjMOUSE_ZOOM, self.model, 0.0, -0.05 * change, &self.scene);
                }
                WindowEvent::CursorPos(x, y) => {
                    /* Calculate the change in mouse position since last call */
                    let dx = x - self.last_x;
                    let dy = y - self.last_y;
                    self.last_x = x;
                    self.last_y = y;

                    /* Check mouse presses and move the camera if any of them is pressed */
                    let action;

                    if self.window.get_mouse_button(MouseButton::Left) == Action::Press {
                        action = MjtMouse::mjMOUSE_ROTATE_H;
                    }
                    else if self.window.get_mouse_button(MouseButton::Right) == Action::Press {
                        action = MjtMouse::mjMOUSE_MOVE_H;
                    }
                    else if self.window.get_mouse_button(MouseButton::Middle) == Action::Press {
                        action = MjtMouse::mjMOUSE_ZOOM;
                    }
                    else {
                        continue;  // If buttons aren't pressed, ignore.
                    }

                    let height = self.window.get_size().1 as mjtNum;
                    self.camera.move_(action, self.model, dx / height, dy / height, &self.scene);
                }
                _ => {}  // ignore other events
            }
        }
    }

    /// Updates the screen state
    fn update(&mut self, data: &mut MjData) {
        /* Read the screen size */
        let mut viewport = MjrRectangle::default();
        let (width, height) = self.window.get_framebuffer_size();
        viewport.width = width;
        viewport.height = height;

        /* Update the scene from the MjData state  */
        self.scene.update(data, &MjvOption::default(), &mut self.camera);
        self.scene.render(&viewport, &self.context);

        /* Display the changes */
        self.window.swap_buffers();
    }
}

/****************************************** */
// C++ viewer wrapper
/****************************************** */
/// Wrapper around the C++ implementation of MujoCo viewer
/// # SAFETY
/// Due to performance reasons and PyO3, this must be destroyed before
/// [`MjData`] and [`MjModel`] instances that are passed in the constructor.
/// Normally, we would include references to them but it's very inconvenient.
pub struct MjViewerCpp<'m> {
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




impl<'m> MjViewerCpp<'m> {
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
            (*sim).Load(model.__raw(), data.__raw(), CString::new("file.xml").unwrap().as_ptr());
            (*sim).RenderStep(true);
        }

        Self {sim, running: true, _cam, _opt, _pert, _glfw, _user_scn}
    }

    /// Returns the underlying C++ binding object of the viewer.
    pub fn __raw(&self) -> *mut mujoco_Simulate {
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
            (*self.sim).Sync(false);
        }
    }
}


impl Drop for MjViewerCpp<'_> {
    fn drop(&mut self) {
        unsafe {
            (*self.sim).RenderCleanup();
            free_simulate(self.sim);
        }
    }
}
