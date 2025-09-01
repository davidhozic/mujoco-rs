use glfw::{Action, Context, Glfw, GlfwReceiver, Key, Modifiers, MouseButton, PWindow, WindowEvent};

use std::time::Instant;

use crate::mujoco_c::*;

#[cfg(feature = "cpp-viewer")]
use std::ffi::CString;

use crate::prelude::{MjrContext, MjrRectangle};
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;

/****************************************** */
// Rust native viewer
/****************************************** */
const MJ_VIEWER_DEFAULT_SIZE_PX: (u32, u32) = (1280, 720);
const MJ_VIEWER_DEFAULT_TITLE: &str = "MuJoCo Viewer (Rust)";
const DOUBLE_CLICK_WINDOW_MS: u128 = 250;


#[derive(Debug)]
pub enum MjViewerError {
    GlfwInitError (glfw::InitError),
    WindowCreationError
}

/// A Rust-native implementation of the MuJoCo viewer. To confirm to rust safety rules,
/// the viewer doesn't store a mutable reference to the [`MjData`] struct, but it instead
/// accepts it as a parameter at its methods.
/// 
/// Currently supported (to be expanded in the future):
/// - Visualization of the 3D scene,
/// - Close via Ctrl + Q or by closing the window,
/// - Body tracking via Ctrl + Alt + double left-click,
/// - Escape from tracked camera via Esc.
/// - Perturbations:
///     - Select the object of interested by double clicking on it,
///     - Hold down Control and start dragging for **rotational** perturbations,
///     - Hold down Control+Alt and start dragging for **translational** perturbations,
///     - To move in the XY plane instead of the default XZ plane, hold Shift.
/// 
/// The [`MjViewer::sync`] method must be called to sync the state of [`MjViewer`] and [`MjData`].
#[derive(Debug)]
pub struct MjViewer<'m> {
    /* MuJoCo rendering */
    scene: MjvScene<'m>,
    context: MjrContext,
    camera: MjvCamera,

    /* Other MuJoCo related */
    model: &'m MjModel,
    pert: MjvPerturb,

    /* Internal state */
    last_x: mjtNum,
    last_y: mjtNum,
    left_click: bool,
    last_bnt_press_time: Instant,
    rect_view: MjrRectangle,
    rect_full: MjrRectangle,

    /* OpenGL */
    glfw: Glfw,
    window: PWindow,
    events: GlfwReceiver<(f64, WindowEvent)>,
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
        glfw.set_swap_interval(glfw::SwapInterval::None);

        let scene = MjvScene::new(model, model.ffi().ngeom as usize + scene_max_ngeom);
        let context= MjrContext::new(model);
        let camera = MjvCamera::new(0, MjtCamera::mjCAMERA_FREE, model);
        let pert = MjvPerturb::default();
        Ok(Self {
            scene,
            context,
            camera,
            model,
            pert,
            glfw,
            window,
            events,
            last_x: 0.0,
            last_y: 0.0,
            left_click: false,
            last_bnt_press_time: Instant::now(),
            rect_view: MjrRectangle::default(),
            rect_full: MjrRectangle::default(),
        })
    }

    /// Checks whether the window is still open.
    pub fn running(&self) -> bool {
        !self.window.should_close()
    }

    pub fn sync(&mut self, data: &mut MjData) {
        /* Process mouse and keyboard events */
        self.process_events(data);

        /* Update the scene from data and render */
        self.update(data);

        /* Apply perturbations */
        self.pert.apply(self.model, data);
    }

    /// Updates the screen state
    fn update(&mut self, data: &mut MjData) {
        /* Read the screen size */
        self.update_rectangles(self.window.get_framebuffer_size());

        /* Update the scene from the MjData state */
        let opt = MjvOption::default();
        self.scene.update(data, &opt, &self.pert, &mut self.camera);
        self.scene.render(self.rect_full, &self.context);

        /* Display the changes */
        self.window.swap_buffers();
    }

    /// Updates the dimensions of the rectangles defining the dimensions of
    /// the user interface, as well as the actual scene viewer.
    fn update_rectangles(&mut self, viewport_size: (i32, i32)) {
        // The scene (middle) rectangle
        self.rect_view.width = viewport_size.0;
        self.rect_view.height = viewport_size.1;

        self.rect_full.width = viewport_size.0;
        self.rect_full.height = viewport_size.1;
    }

    /// Processes user input events
    fn process_events(&mut self, data: &mut MjData) {
        self.glfw.poll_events();
        while let Some((_, event)) = self.events.receive() {
            match event {
                WindowEvent::Key(Key::Q, _, _, modifier) if modifier == Modifiers::Control => {
                    self.window.set_should_close(true);
                    break;  // no use in polling other events
                },
                WindowEvent::Key(Key::Escape, _, _, _) => {
                    self.camera.free();
                },
                WindowEvent::Scroll(_, change) => {
                    self.process_scroll(change);
                }
                WindowEvent::CursorPos(x, y) => {
                    self.process_cursor_pos(x, y, data);
                },

                // Match left button presses
                WindowEvent::MouseButton(MouseButton::Left, action, modifiers) => {
                    self.process_left_click(data, &action, &modifiers);
                }
                _ => {}  // ignore other events
            }
        }
    }

    fn process_scroll(&mut self, change: f64) {
        self.camera.move_(MjtMouse::mjMOUSE_ZOOM, self.model, 0.0, -0.05 * change, &self.scene);
    }

    fn process_cursor_pos(&mut self, x: f64, y: f64, data: &mut MjData) {
        /* Calculate the change in mouse position since last call */
        let dx = x - self.last_x;
        let dy = y - self.last_y;
        self.last_x = x;
        self.last_y = y;

        /* Check mouse presses and move the camera if any of them is pressed */
        let action;
        let height = self.window.get_size().1 as mjtNum;

        let shift = self.window.get_key(Key::LeftShift) == Action::Press;
        
        if self.left_click {
            if self.pert.active == MjtPertBit::mjPERT_TRANSLATE as i32 {
                action = if shift {MjtMouse::mjMOUSE_MOVE_H} else {MjtMouse::mjMOUSE_MOVE_V};
            }
            else {
                action = if shift {MjtMouse::mjMOUSE_ROTATE_H} else {MjtMouse::mjMOUSE_ROTATE_V};
            }
        }
        else if self.window.get_mouse_button(MouseButton::Right) == Action::Press {
            action = if shift {MjtMouse::mjMOUSE_MOVE_H} else {MjtMouse::mjMOUSE_MOVE_V};
        }
        else if self.window.get_mouse_button(MouseButton::Middle) == Action::Press {
            action = MjtMouse::mjMOUSE_ZOOM;
        }
        else {
            return;  // If buttons aren't pressed, ignore.
        }

        /* When the perturbation isn't active, move the camera */
        if self.pert.active == 0 {
            self.camera.move_(action, self.model, dx / height, dy / height, &self.scene);
        }
        else {  // When the perturbation is active, move apply the perturbation.
            self.pert.move_(self.model, data, action, dx / height, dy / height, &self.scene);
        }
    }

    fn process_left_click(&mut self, data: &mut MjData, action: &Action, modifiers: &Modifiers) {
        self.left_click = match action {
            Action::Press => {
                /* Clicking and holding applies perturbation */
                if self.pert.select > 0 && modifiers.contains(Modifiers::Control) {
                    let type_ = if modifiers.contains(Modifiers::Alt) {
                        MjtPertBit::mjPERT_TRANSLATE
                    } else {
                        MjtPertBit::mjPERT_ROTATE
                    };
                    self.pert.start(type_, &self.model, data, &self.scene);
                }

                /* Double click detection */
                if !self.left_click && self.last_bnt_press_time.elapsed().as_millis() < DOUBLE_CLICK_WINDOW_MS {
                    let (mut x, mut y) = self.window.get_cursor_pos();

                    /* Fix the coordinates */
                    let buffer_ratio = self.window.get_framebuffer_size().0 as mjtNum / self.window.get_size().0 as mjtNum;
                    x *= buffer_ratio;
                    y *= buffer_ratio;
                    y = self.rect_full.height as mjtNum - y;  // match OpenGL's coordinate system.

                    /* Obtain the selection */ 
                    let rect: &mjrRect_ = &self.rect_view;
                    let (body_id, _, flex_id, skin_id, xyz) = self.scene.find_selection(
                        data, &MjvOption::default(),
                        rect.width as mjtNum / rect.height as mjtNum,
                        (x - rect.left as mjtNum) / rect.width as mjtNum,
                        (y - rect.bottom as mjtNum) / rect.height as mjtNum
                    );

                    /* Set tracking camera */
                    if modifiers.contains(Modifiers::Alt) {
                        if body_id >= 0 {
                            self.camera.lookat = xyz;
                            if modifiers.contains(Modifiers::Control) {
                                self.camera.track(body_id as u32);
                            }
                        }
                    }
                    else {
                        /* Mark selection */
                        if body_id >= 0 {
                            self.pert.select = body_id;
                            self.pert.flexselect = flex_id;
                            self.pert.skinselect = skin_id;
                            self.pert.active = 0;
                            self.pert.update_local_pos(xyz, data);
                        }
                        else {
                            self.pert.select = 0;
                            self.pert.flexselect = -1;
                            self.pert.skinselect = -1;
                        }
                    }
                }
                self.last_bnt_press_time = Instant::now();
                true
            },
            Action::Release => {
                // Clear perturbation when left click is released.
                self.pert.active = 0;
                false
            },
            Action::Repeat => unreachable!()
        };
    }
}

/****************************************** */
// C++ viewer wrapper
/****************************************** */
/// Wrapper around the C++ implementation of MujoCo viewer
#[cfg(feature = "cpp-viewer")]
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

#[cfg(feature = "cpp-viewer")]
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

#[cfg(feature = "cpp-viewer")]
impl Drop for MjViewerCpp<'_> {
    fn drop(&mut self) {
        unsafe {
            (*self.sim).RenderCleanup();
            free_simulate(self.sim);
        }
    }
}
