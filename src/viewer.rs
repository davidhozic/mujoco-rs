//! Module related to implementation of the [`MjViewer`] and [`MjViewerCpp`].
use glutin::prelude::PossiblyCurrentGlContext;
use glutin::surface::GlSurface;
use winit::event::{ElementState, KeyEvent, Modifiers, MouseButton, MouseScrollDelta, WindowEvent};
use winit::platform::pump_events::EventLoopExtPumpEvents;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::event_loop::EventLoop;
use winit::window::Fullscreen;

use std::time::{Duration, Instant};
use std::error::Error;
use std::fmt::Display;
use std::ops::Deref;

use bitflags::bitflags;


#[cfg(feature = "cpp-viewer")]
use std::ffi::CString;

#[cfg(feature = "cpp-viewer")]
use crate::mujoco_c::*;

use crate::prelude::{MjrContext, MjrRectangle, MjtFont, MjtGridPos};
use crate::wrappers::mj_primitive::MjtNum;
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;
use crate::render_base::{GlState, RenderBase};
use crate::get_mujoco_version;

/****************************************** */
// Rust native viewer
/****************************************** */
const MJ_VIEWER_DEFAULT_SIZE_PX: (u32, u32) = (1280, 720);
const DOUBLE_CLICK_WINDOW_MS: u128 = 250;

const HELP_MENU_TITLES: &str = concat!(
    "Toggle help\n",
    "Toggle full screen\n",
    "Free camera\n",
    "Track camera\n",
    "Camera orbit\n",
    "Camera pan\n",
    "Camera look at\n",
    "Zoom\n",
    "Object select\n",
    "Selection rotate\n",
    "Selection translate\n",
    "Exit\n",
    "Reset simulation\n",
    "Cycle cameras\n",
    "Visualization toggles",
);

const HELP_MENU_VALUES: &str = concat!(
    "F1\n",
    "F5\n",
    "Escape\n",
    "Control + Alt + double-left click\n",
    "Left drag\n",
    "Right [+Shift] drag\n",
    "Alt + double-left click\n",
    "Zoom, middle drag\n",
    "Double-left click\n",
    "Control + [Shift] + drag\n",
    "Control + Alt + [Shift] + drag\n",
    "Control + Q\n",
    "Backspace\n",
    "[ ]\n",
    "See MjViewer docs"
);

#[derive(Debug)]
pub enum MjViewerError {
    WindowCreationError,
    EventLoopError(winit::error::EventLoopError),
    GlutinError(glutin::error::Error)
}

impl Display for MjViewerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WindowCreationError => write!(f, "failed to create window"),
            Self::EventLoopError(e) => write!(f, "failed to initialize event_loop: {}", e),
            Self::GlutinError(e) => write!(f, "glutin raised an error: {}", e)
        }
    }
}

impl Error for MjViewerError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::EventLoopError(e) => Some(e),
            Self::GlutinError(e) => Some(e),
            _ => None
        }
    }
}

/// A Rust-native implementation of the MuJoCo viewer. To confirm to rust safety rules,
/// the viewer doesn't store a mutable reference to the [`MjData`] struct, but it instead
/// accepts it as a parameter at its methods.
/// 
/// The [`MjViewer::sync`] method must be called to sync the state of [`MjViewer`] and [`MjData`].
/// 
/// # Shortcuts
/// Main keyboard and mouse shortcuts can be viewed by pressing ``F1``.
/// Additionally, some visualization toggles are included, but not displayed
/// in the ``F1`` help menu:
/// - C: camera,
/// - U: actuator,
/// - J: joint,
/// - M: center of mass,
/// - H: convex hull,
/// - Z: light,
/// - T: transparent,
/// - I: inertia.
/// 
/// # Safety
/// Due to the nature of OpenGL, this should only be run in the **main thread**.
#[derive(Debug)]
pub struct MjViewer<M: Deref<Target = MjModel> + Clone> {
    /* MuJoCo rendering */
    scene: MjvScene<M>,
    context: MjrContext,
    camera: MjvCamera,

    /* Other MuJoCo related */
    model: M,
    pert: MjvPerturb,
    opt: MjvOption,

    /* Internal state */
    last_x: f64,
    last_y: f64,
    last_bnt_press_time: Instant,
    rect_view: MjrRectangle,
    rect_full: MjrRectangle,
    status_flags: ViewerStatusBits,  // Status flag indicating the state of the menu

    /* OpenGL */
    adapter: RenderBase,
    event_loop: EventLoop<()>,
    modifiers: Modifiers,
    buttons_pressed: ButtonsPressed,
    cursor_position: (u32, u32),

    /* External interaction */
    user_scene: MjvScene<M>,
}

impl<M: Deref<Target = MjModel> + Clone> MjViewer<M> {
    /// Launches the MuJoCo viewer. A [`Result`] struct is returned that either contains
    /// [`MjViewer`] or a [`MjViewerError`]. The `scene_max_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    pub fn launch_passive(model: M, scene_max_geom: usize) -> Result<Self, MjViewerError> {
        let (w, h) = MJ_VIEWER_DEFAULT_SIZE_PX;
        let mut event_loop = EventLoop::new()
            .map_err(|e| MjViewerError::EventLoopError(e))?;

        let adapter = RenderBase::new(
            w, h,
            format!("MuJoCo Rust Viewer (MuJoCo {})", get_mujoco_version()),
            &mut event_loop,
            true  // process events
        );

        /* Initialize the OpenGL related things */
        let GlState {
            gl_context,
            gl_surface,
            ..
        } = adapter.state.as_ref().unwrap();
        gl_context.make_current(gl_surface).map_err(|e| MjViewerError::GlutinError(e))?;
        gl_surface.set_swap_interval(gl_context, glutin::surface::SwapInterval::DontWait).map_err(
            |e| MjViewerError::GlutinError(e)
        )?;
        event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);

        let ngeom = model.ffi().ngeom as usize;
        let scene = MjvScene::new(model.clone(), ngeom + scene_max_geom + EXTRA_SCENE_GEOM_SPACE);
        let user_scene = MjvScene::new(model.clone(), scene_max_geom);
        let context = MjrContext::new(&model);
        let camera  = MjvCamera::new_free(&model);

        Ok(Self {
            model,
            scene,
            context,
            camera,
            pert: MjvPerturb::default(),
            opt: MjvOption::default(),
            user_scene,
            last_x: 0.0,
            last_y: 0.0,
            status_flags: ViewerStatusBits::HELP_MENU,
            last_bnt_press_time: Instant::now(),
            rect_view: MjrRectangle::default(),
            rect_full: MjrRectangle::default(),
            adapter,
            event_loop,
            modifiers: Modifiers::default(),
            buttons_pressed: ButtonsPressed::empty(),
            cursor_position: (0, 0)
        })
    }

    /// Checks whether the window is still open.
    pub fn running(&self) -> bool {
        self.adapter.running
    }

    /// Returns an immutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`MjViewer::sync`].
    pub fn user_scene(&self) -> &MjvScene<M>{
        &self.user_scene
    }

    /// Returns a mutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`MjViewer::sync`].
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<M>{
        &mut self.user_scene
    }

    #[deprecated(since = "1.3.0", note = "use user_scene")]
    pub fn user_scn(&self) -> &MjvScene<M> {
        self.user_scene()
    }

    #[deprecated(since = "1.3.0", note = "use user_scene_mut")]
    pub fn user_scn_mut(&mut self) -> &mut MjvScene<M> {
        self.user_scene_mut()
    }

    /// Syncs the state of `data` with the viewer as well as perform
    /// rendering on the viewer.
    pub fn sync(&mut self, data: &mut MjData<M>) {
        let GlState {
            gl_context,
            gl_surface, ..
        } = self.adapter.state.as_mut().unwrap();

        /* Make sure everything is done on the viewer's window */
        gl_context.make_current(gl_surface).expect("could not make OpenGL context current");

        /* Process mouse and keyboard events */
        self.process_events(data);

        /* Update the scene from data and render */
        self.update_scene(data);

        /* Update the user menu state and overlays */
        self.update_menus();

        /* Swap OpenGL buffers */
        self.render();

        /* Apply perturbations */
        self.pert.apply(&self.model, data);
    }

    /// Renders the drawn content by swapping buffers.
    fn render(&mut self) {
        /* Display the drawn content */
        let GlState {
            gl_context,
            gl_surface, ..
        } = self.adapter.state.as_mut().unwrap();

        /* Make sure everything is done on the viewer's window */
        gl_surface.swap_buffers(gl_context).expect("buffer swap in OpenGL failed");    }

    /// Updates the scene and draws it to the display.
    fn update_scene(&mut self, data: &mut MjData<M>) {
        let model_data_ptr = unsafe {  data.model().__raw() };
        let bound_model_ptr = unsafe { self.model.__raw() };
        assert_eq!(model_data_ptr, bound_model_ptr, "'data' must be created from the same model as the viewer.");

        /* Read the screen size */
        self.update_rectangles(self.adapter.state.as_ref().unwrap().window.inner_size().into());

        /* Update the scene from the MjData state */
        self.scene.update(data, &self.opt, &self.pert, &mut self.camera);

        /* Draw user scene geoms */
        sync_geoms(&self.user_scene, &mut self.scene)
            .expect("could not sync the user scene with the internal scene; this is a bug, please report it.");

        self.scene.render(&self.rect_full, &self.context);
    }

    /// Draws the user menu
    fn update_menus(&mut self) {
        let mut rectangle = self.rect_view;
        rectangle.width = rectangle.width - rectangle.width / 4;

        /* Overlay section */
        if self.status_flags.contains(ViewerStatusBits::HELP_MENU) {  // Help
            self.context.overlay(
                MjtFont::mjFONT_NORMAL, MjtGridPos::mjGRID_TOPLEFT,
                rectangle,
                HELP_MENU_TITLES,
                Some(HELP_MENU_VALUES)
            );
        }
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

    /// Processes user input events.
    fn process_events(&mut self, data: &mut MjData<M>) {
        self.event_loop.pump_app_events(Some(Duration::ZERO), &mut self.adapter);
        while let Some(window_event) = self.adapter.queue.pop_front() {
            match window_event {
                WindowEvent::ModifiersChanged(modifiers) => self.modifiers = modifiers,
                WindowEvent::MouseInput {state, button, .. } => {
                    let index = match button {
                        MouseButton::Left => {
                            self.process_left_click(data, state);
                            ButtonsPressed::LEFT
                        },
                        MouseButton::Middle => ButtonsPressed::MIDDLE,
                        MouseButton::Right => ButtonsPressed::RIGHT,
                        _ => return
                    };

                    self.buttons_pressed.set(index, state == ElementState::Pressed);
                }

                WindowEvent::CursorMoved { position, .. } => {
                    self.process_cursor_pos(position.x, position.y, data);
                    self.cursor_position = position.into();
                }

                // Set the viewer's state to pending exit.
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::KeyQ),
                        state: ElementState::Pressed, ..
                    }, ..
                } if self.modifiers.state().control_key()  => {
                    self.adapter.running = false;
                }

                // Free the camera from tracking.
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::Escape),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.camera.free();
                }
                
                // Toggle help menu
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::F1),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.status_flags.toggle(ViewerStatusBits::HELP_MENU);
                }

                // Full screen
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::F5),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.toggle_full_screen();
                }

                // Reset the simulation (the data).
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::Backspace),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    data.reset();
                    data.forward();
                }

                // Cycle to the next camera
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::BracketRight),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.cycle_camera(1);
                }

                // Cycle to the previous camera
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::BracketLeft),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.cycle_camera(-1);
                }

                // Toggles camera visualization.
                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyC), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_CAMERA),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyU), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_ACTUATOR),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyJ), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_JOINT),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyM), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_COM),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyH), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_CONVEXHULL),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyZ), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_LIGHT),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyT), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_TRANSPARENT),

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyI), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_INERTIA),

                // Zoom in/out
                WindowEvent::MouseWheel {delta, ..} => {
                    let value = match delta {
                        MouseScrollDelta::LineDelta(_, down) => down,
                        _ => 0.0
                    };
                    self.process_scroll(value.into());
                }

                _ => {}  // ignore other events
            }
        }
    }

    /// Toggles visualization options.
    fn toggle_opt_flag(&mut self, flag: MjtVisFlag) {
        let index = flag as usize;
        self.opt.flags[index] = !self.opt.flags[index];
    }

    /// Cycle MJCF defined cameras.
    fn cycle_camera(&mut self, direction: i32) {
        let n_cam = self.model.ffi().ncam;
        if n_cam == 0 {  // No cameras, ignore.
            return;
        }

        self.camera.fix((self.camera.fixedcamid + direction).rem_euclid(n_cam) as u32);
    }

    /// Toggles full screen mode.
    fn toggle_full_screen(&mut self) {
        let window = &self.adapter.state.as_ref().unwrap().window;
        if let Some(_) = window.fullscreen() {
            window.set_fullscreen(None);
        }
        else {
            window.set_fullscreen(Some(Fullscreen::Borderless(None)));
        }
    }

    /// Processes scrolling events.
    fn process_scroll(&mut self, change: f64) {
        self.camera.move_(MjtMouse::mjMOUSE_ZOOM, &self.model, 0.0, -0.05 * change, &self.scene);
    }

    /// Processes camera and perturbation movements.
    fn process_cursor_pos(&mut self, x: f64, y: f64, data: &mut MjData<M>) {
        /* Calculate the change in mouse position since last call */
        let dx = x - self.last_x;
        let dy = y - self.last_y;
        self.last_x = x;
        self.last_y = y;
        let window = &self.adapter.state.as_ref().unwrap().window;
        let modifiers = &self.modifiers.state();
        let buttons = &self.buttons_pressed;
        let shift = modifiers.shift_key();

        /* Check mouse presses and move the camera if any of them is pressed */
        let action;
        let height = window.outer_size().height as f64;
        
        if self.status_flags.contains(ViewerStatusBits::LEFT_CLICK) {
            if self.pert.active == MjtPertBit::mjPERT_TRANSLATE as i32 {
                action = if shift {MjtMouse::mjMOUSE_MOVE_H} else {MjtMouse::mjMOUSE_MOVE_V};
            }
            else {
                action = if shift {MjtMouse::mjMOUSE_ROTATE_H} else {MjtMouse::mjMOUSE_ROTATE_V};
            }
        }
        else if buttons.contains(ButtonsPressed::RIGHT) {
            action = if shift {MjtMouse::mjMOUSE_MOVE_H} else {MjtMouse::mjMOUSE_MOVE_V};
        }
        else if buttons.contains(ButtonsPressed::MIDDLE) {
            action = MjtMouse::mjMOUSE_ZOOM;
        }
        else {
            return;  // If buttons aren't pressed, ignore.
        }

        /* When the perturbation isn't active, move the camera */
        if self.pert.active == 0 {
            self.camera.move_(action, &self.model, dx / height, dy / height, &self.scene);
        }
        else {  // When the perturbation is active, move apply the perturbation.
            self.pert.move_(&self.model, data, action, dx / height, dy / height, &self.scene);
        }
    }

    // /// Processes left clicks and double left clicks.
    fn process_left_click(&mut self, data: &mut MjData<M>, state: ElementState) {
        let modifier_state = self.modifiers.state();
        let window = &self.adapter.state.as_ref().unwrap().window;
        match state {
            ElementState::Pressed => {
                /* Clicking and holding applies perturbation */
                if self.pert.select > 0 && modifier_state.control_key() {
                    let type_ = if modifier_state.alt_key() {
                        MjtPertBit::mjPERT_TRANSLATE
                    } else {
                        MjtPertBit::mjPERT_ROTATE
                    };
                    self.pert.start(type_, &self.model, data, &self.scene);
                }

                /* Double click detection */
                if !self.status_flags.contains(ViewerStatusBits::LEFT_CLICK) && self.last_bnt_press_time.elapsed().as_millis() < DOUBLE_CLICK_WINDOW_MS {
                    let cp = self.cursor_position;
                    let mut x = cp.0 as f64;
                    let mut y = cp.1 as f64;

                    /* Fix the coordinates */
                    let buffer_ratio = window.inner_size().width as f64 / window.outer_size().width as f64;
                    x *= buffer_ratio;
                    y *= buffer_ratio;
                    y = self.rect_full.height as f64 - y;  // match OpenGL's coordinate system.

                    /* Obtain the selection */ 
                    let rect: &MjrRectangle = &self.rect_view;
                    let (body_id, _, flex_id, skin_id, xyz) = self.scene.find_selection(
                        data, &self.opt,
                        rect.width as MjtNum / rect.height as MjtNum,
                        (x - rect.left as MjtNum) / rect.width as MjtNum,
                        (y - rect.bottom as MjtNum) / rect.height as MjtNum
                    );

                    /* Set tracking camera */
                    if modifier_state.alt_key() {
                        if body_id >= 0 {
                            self.camera.lookat = xyz;
                            if modifier_state.control_key() {
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
                self.status_flags.set(ViewerStatusBits::LEFT_CLICK, true);
            },
            ElementState::Released => {
                // Clear perturbation when left click is released.
                self.pert.active = 0;
                self.status_flags.remove(ViewerStatusBits::LEFT_CLICK);
            },
        };
    }
}



bitflags! {
    /// Boolean flags that define some of
    /// the Viewer's internal state.
    #[derive(Debug)]
    struct ViewerStatusBits: u8 {
        const LEFT_CLICK = 1 << 0;
        const HELP_MENU  = 1 << 1;
    }
}

bitflags! {
    /// Boolean flags for tracking button press events.
    #[derive(Debug)]
    struct ButtonsPressed: u8 {
        const LEFT = 0;
        const MIDDLE = 1;
        const RIGHT = 2;
    }
}

/****************************************** */
// C++ viewer wrapper
/****************************************** */
/// Wrapper around the C++ implementation of MujoCo viewer.
/// If you don't need the side UI, we recommend you use the Rust-native viewer [`MjViewer`] instead.
#[cfg(feature = "cpp-viewer")]
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

#[cfg(feature = "cpp-viewer")]
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
    /// Unlike the Rust-native viewer ([`MjViewer`]), this also accepts a `data` parameter.
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
            sim = new_simulate(&mut *_cam, &mut *_opt, &mut *_pert, _user_scn.ffi_mut(), true);
            (*sim).RenderInit();
            (*sim).Load(model.__raw(), data.__raw(), c_filename.as_ptr());
            (*sim).RenderStep(true);
        }

        Self {sim, running: true, _cam, _opt, _pert, _user_scn}
    }

    /// Returns the underlying C++ binding object of the viewer.
    pub fn __raw(&self) -> *mut mujoco_Simulate {
        self.sim
    }

    /// Renders the simulation.
    /// `update_timer` flag species whether the time should be updated
    /// inside the viewer (for FPS calculation).
    /// # SAFETY
    /// This needs to be called periodically from the MAIN thread, otherwise
    /// GLFW stops working.
    pub fn render(&mut self, update_timer: bool) {
        unsafe {
            assert!(self.running, "render called after viewer has been closed!");
            self.running = (*self.sim).RenderStep(update_timer);
        }
    }

    /// Syncs the simulation state with the viewer as well as perform
    /// rendering on the viewer.
    pub fn sync(&mut self) {
        unsafe {
            (*self.sim).Sync(false);
        }
    }
}

#[cfg(feature = "cpp-viewer")]
impl<M: Deref<Target = MjModel> + Clone> Drop for MjViewerCpp<M> {
    fn drop(&mut self) {
        unsafe {
            (*self.sim).RenderCleanup();
            free_simulate(self.sim);
        }
    }
}
