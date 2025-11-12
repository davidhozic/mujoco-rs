//! Module related to implementation of the [`MjViewer`]. For implementation of the C++ wrapper,
//! see [`crate::cpp_viewer::MjViewerCpp`].
#[cfg(feature = "viewer-ui")] use glutin::display::GetGlDisplay;
use glutin::prelude::PossiblyCurrentGlContext;
use glutin::surface::GlSurface;

use winit::event::{ElementState, KeyEvent, Modifiers, MouseButton, MouseScrollDelta, WindowEvent};
use winit::platform::pump_events::EventLoopExtPumpEvents;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::event_loop::EventLoop;
use winit::dpi::PhysicalPosition;
use winit::window::Fullscreen;

use std::time::{Duration, Instant};
use std::error::Error;
use std::fmt::Display;
use std::ops::Deref;

use bitflags::bitflags;

use crate::prelude::{MjrContext, MjrRectangle, MjtFont, MjtGridPos};
use crate::render_base::{GlState, RenderBase, sync_geoms};
use crate::wrappers::mj_primitive::MjtNum;
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::wrappers::mj_data::MjData;
use crate::get_mujoco_version;


#[cfg(feature = "viewer-ui")]
mod ui;


/****************************************** */
// Rust native viewer
/****************************************** */
const MJ_VIEWER_DEFAULT_SIZE_PX: (u32, u32) = (1280, 720);
const DOUBLE_CLICK_WINDOW_MS: u128 = 250;
const TOUCH_BAR_ZOOM_FACTOR: f64 = 0.1;

/// How much extra room to create in the internal [`MjvScene`]. Useful for drawing labels, etc.
pub(crate) const EXTRA_SCENE_GEOM_SPACE: usize = 2000;

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
    EventLoopError(winit::error::EventLoopError),
    GlutinError(glutin::error::Error)
}

impl Display for MjViewerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EventLoopError(e) => write!(f, "failed to initialize event_loop: {}", e),
            Self::GlutinError(e) => write!(f, "glutin raised an error: {}", e)
        }
    }
}

impl Error for MjViewerError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::EventLoopError(e) => Some(e),
            Self::GlutinError(e) => Some(e)
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
/// - I: inertia,
/// - E: constraint.
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

    /* OpenGL */
    adapter: RenderBase,
    event_loop: EventLoop<()>,
    modifiers: Modifiers,
    buttons_pressed: ButtonsPressed,
    raw_cursor_position: (f64, f64),

    /* External interaction */
    user_scene: MjvScene<M>,

    /* User interface */
    #[cfg(feature = "viewer-ui")]
    ui: ui::ViewerUI<M>,

    status: ViewerStatusBit
}

impl<M: Deref<Target = MjModel> + Clone> MjViewer<M> {
    /// Launches the MuJoCo viewer. A [`Result`] struct is returned that either contains
    /// [`MjViewer`] or a [`MjViewerError`]. The `max_user_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    pub fn launch_passive(model: M, max_user_geom: usize) -> Result<Self, MjViewerError> {
        let (w, h) = MJ_VIEWER_DEFAULT_SIZE_PX;
        let mut event_loop = EventLoop::new().map_err(MjViewerError::EventLoopError)?;
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
            #[cfg(feature = "viewer-ui")] window,
            ..
        } = adapter.state.as_ref().unwrap();
        gl_context.make_current(gl_surface).map_err(MjViewerError::GlutinError)?;
        gl_surface.set_swap_interval(gl_context, glutin::surface::SwapInterval::DontWait).map_err(
            |e| MjViewerError::GlutinError(e)
        )?;
        event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);

        let ngeom = model.ffi().ngeom as usize;
        let scene = MjvScene::new(model.clone(), ngeom + max_user_geom + EXTRA_SCENE_GEOM_SPACE);
        let user_scene = MjvScene::new(model.clone(), max_user_geom);
        let context = MjrContext::new(&model);
        let camera  = MjvCamera::new_free(&model);

        #[cfg(feature = "viewer-ui")]
        let ui = ui::ViewerUI::new(model.clone(), &window, &gl_surface.display());

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
            last_bnt_press_time: Instant::now(),
            rect_view: MjrRectangle::default(),
            rect_full: MjrRectangle::default(),
            adapter,
            event_loop,
            modifiers: Modifiers::default(),
            buttons_pressed: ButtonsPressed::empty(),
            raw_cursor_position: (0.0, 0.0),
            #[cfg(feature = "viewer-ui")] ui,
            #[cfg(feature = "viewer-ui")] status: ViewerStatusBit::UI,
            #[cfg(not(feature = "viewer-ui"))] status: ViewerStatusBit::HELP,
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
            gl_surface,
            ..
        } = self.adapter.state.as_mut().unwrap();

        /* Make sure everything is done on the viewer's window */
        gl_context.make_current(gl_surface).expect("could not make OpenGL context current");

        /* Read the screen size */
        self.update_rectangles(self.adapter.state.as_ref().unwrap().window.inner_size().into());

        /* Process mouse and keyboard events */
        self.process_events(data);


        /* Update the scene from data and render */
        self.update_scene(data);

        /* Draw the user menu on top */
        #[cfg(feature = "viewer-ui")]
        self.process_user_ui(data);

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

        gl_surface.swap_buffers(gl_context).expect("buffer swap in OpenGL failed");
    }

    /// Updates the scene and draws it to the display.
    fn update_scene(&mut self, data: &mut MjData<M>) {
        let model_data_ptr = unsafe {  data.model().__raw() };
        let bound_model_ptr = unsafe { self.model.__raw() };
        assert_eq!(model_data_ptr, bound_model_ptr, "'data' must be created from the same model as the viewer.");

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
        if self.status.contains(ViewerStatusBit::HELP) {  // Help
            self.context.overlay(
                MjtFont::mjFONT_NORMAL, MjtGridPos::mjGRID_TOPLEFT,
                rectangle,
                HELP_MENU_TITLES,
                Some(HELP_MENU_VALUES)
            );
        }
    }


    /// Draws the user UI
    #[cfg(feature = "viewer-ui")]
    fn process_user_ui(&mut self, data: &mut MjData<M>) {
        /* Draw the user interface */

        use crate::viewer::ui::UiEvent;
        let GlState { window, .. } = &self.adapter.state.as_ref().unwrap();
        let inner_size = window.inner_size();
        let left = self.ui.process(
            window, &mut self.status,
            &mut self.scene, &mut self.opt,
            &mut self.camera, data
        );

        /* Adjust the viewport so MuJoCo doesn't draw over the UI */
        self.rect_view.left = left as i32;
        self.rect_view.width = inner_size.width as i32;

        /* Reset some OpenGL settings so that MuJoCo can still draw */
        self.ui.reset();

        /* Process events made in the user UI */
        while let Some(event) = self.ui.drain_events() {
            use UiEvent::*;
            match event {
                Close => self.adapter.running = false,
                Fullscreen => self.toggle_full_screen(),
                ResetSimulation => {
                    data.reset();
                    data.forward();
                },
                AlignCamera => {
                    self.camera = MjvCamera::new_free(&self.model);
                }
            }
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

    /// Checks whether the cursor is inside the user interface (i.e., outside of MuJoCo's scene).
    #[cfg(feature = "viewer-ui")]
    fn cursor_outside(&self, x: f64, y: f64) -> bool {
        const LEFT_EVENT_PAD: f64 = 10.0;  // additional safety to prevent glitches
        let mut left = self.rect_view.left as f64;
        if left > 0.0 {  // don't add safety margin if nothing is overlayed
            left += LEFT_EVENT_PAD;
        }
        x < left || y < self.rect_view.bottom as f64
    }

    /// Processes user input events.
    fn process_events(&mut self, data: &mut MjData<M>) {
        self.event_loop.pump_app_events(Some(Duration::ZERO), &mut self.adapter);
        while let Some(window_event) = self.adapter.queue.pop_front() {
            /* Draw the user interface */
            #[cfg(feature = "viewer-ui")]
            {
                let window: &winit::window::Window = &self.adapter.state.as_ref().unwrap().window;
                self.ui.handle_events(window, &window_event);
            }

            match window_event {
                WindowEvent::ModifiersChanged(modifiers) => self.modifiers = modifiers,
                WindowEvent::MouseInput {state, button, .. } => {
                    #[cfg(feature = "viewer-ui")]
                    let (x, y) = self.raw_cursor_position;
                    let is_pressed = state == ElementState::Pressed;
                    
                    #[cfg(feature = "viewer-ui")]
                    if (self.cursor_outside(x, y) || self.ui.covered()) && is_pressed {
                        continue;
                    }

                    let index = match button {
                        MouseButton::Left => {
                            self.process_left_click(data, state);
                            ButtonsPressed::LEFT
                        },
                        MouseButton::Middle => ButtonsPressed::MIDDLE,
                        MouseButton::Right => ButtonsPressed::RIGHT,
                        _ => return
                    };

                    self.buttons_pressed.set(index, is_pressed);
                }

                WindowEvent::CursorMoved { position, .. } => {
                    let PhysicalPosition { x, y } = position;
                    self.process_cursor_pos(x, y, data);
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
                    #[cfg(feature = "viewer-ui")]
                    if self.ui.focused() {
                        continue;
                    }
                    self.camera.free();
                }

                // Toggle help menu
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::F1),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.status.toggle(ViewerStatusBit::HELP);
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
                    #[cfg(feature = "viewer-ui")]
                    if self.ui.focused() {
                        continue;
                    }

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

                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyE), state: ElementState::Pressed, ..},
                    ..
                } => self.toggle_opt_flag(MjtVisFlag::mjVIS_CONSTRAINT),

                #[cfg(feature = "viewer-ui")]
                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::KeyX), state: ElementState::Pressed, ..},
                    ..
                } => self.status.toggle(ViewerStatusBit::UI),

                // Zoom in/out
                WindowEvent::MouseWheel {delta, ..} => {
                    #[cfg(feature = "viewer-ui")]
                    let (x, y) = self.raw_cursor_position;

                    #[cfg(feature = "viewer-ui")]
                    if self.cursor_outside(x, y) || self.ui.covered() {
                        continue;
                    }

                    let value = match delta {
                        MouseScrollDelta::LineDelta(_, down) => down as f64,
                        MouseScrollDelta::PixelDelta(PhysicalPosition {y, ..}) => y * TOUCH_BAR_ZOOM_FACTOR
                    };
                    self.process_scroll(value);
                }

                _ => {}  // ignore other events
            }
        }
    }

    /// Toggles visualization options.
    fn toggle_opt_flag(&mut self, flag: MjtVisFlag) {
        let index = flag as usize;
        self.opt.flags[index] = 1 - self.opt.flags[index];
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
        if window.fullscreen().is_some() {
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
        self.raw_cursor_position = (x, y);
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
        
        if buttons.contains(ButtonsPressed::LEFT) {
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

    /// Processes left clicks and double left clicks.
    fn process_left_click(&mut self, data: &mut MjData<M>, state: ElementState) {
        let modifier_state = self.modifiers.state();
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
                if self.last_bnt_press_time.elapsed().as_millis() < DOUBLE_CLICK_WINDOW_MS {
                    let cp = self.raw_cursor_position;
                    let x = cp.0;
                    let y = self.rect_full.height as f64 - cp.1;

                    /* Obtain the selection */ 
                    let rect = &self.rect_full;
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
            },
            ElementState::Released => {
                // Clear perturbation when left click is released.
                self.pert.active = 0;
            },
        };
    }
}

bitflags! {
    #[derive(Debug)]
    struct ViewerStatusBit: u8 {
        const HELP = 1 << 0;
        #[cfg(feature = "viewer-ui")]
        const UI = 1 << 1;
    }
}

bitflags! {
    /// Boolean flags for tracking button press events.
    #[derive(Debug)]
    struct ButtonsPressed: u8 {
        const LEFT = 1 << 0;
        const MIDDLE = 1 << 1;
        const RIGHT = 1 << 2;
    }
}
