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

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};
use std::ops::{Deref, DerefMut};
use std::marker::PhantomData;
use std::num::NonZero;
use std::error::Error;
use std::fmt::Display;
use std::borrow::Cow;

use bitflags::bitflags;

use crate::prelude::{MjrContext, MjrRectangle, MjtFont, MjtGridPos};
use crate::winit_gl_base::{RenderBaseGlState, RenderBase};
use crate::wrappers::mj_data::{MjData, MjtState};
use crate::{builder_setters, get_mujoco_version};
use crate::wrappers::mj_primitive::MjtNum;
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::vis_common::sync_geoms;


#[cfg(feature = "viewer-ui")]
mod ui;

// Re-export egui for user convenience when using custom UI callbacks
#[cfg(feature = "viewer-ui")]
pub use egui;


/****************************************** */
// Rust native viewer
/****************************************** */
const MJ_VIEWER_DEFAULT_SIZE_PX: (u32, u32) = (1280, 720);
const DOUBLE_CLICK_WINDOW_MS: u128 = 250;
const TOUCH_BAR_ZOOM_FACTOR: f64 = 0.1;
const FPS_SMOOTHING_FACTOR: f64 = 0.1;
const REALTIME_FACTOR_SMOOTHING_FACTOR: f64 = 0.1;
const REALTIME_FACTOR_DISPLAY_THRESHOLD: f64 = 0.02;

/// How much extra room to create in the internal [`MjvScene`]. Useful for drawing labels, etc.
pub(crate) const EXTRA_SCENE_GEOM_SPACE: usize = 2000;

const HELP_MENU_TITLES: &str = concat!(
    "Toggle help\n",
    "Toggle info\n",
    "Toggle v-sync\n",
    "Toggle realtime check\n",
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
    "F2\n",
    "F3\n",
    "F4\n",
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


/// Internal state that is used by [`MjViewer`] to store
/// [`MjData`]-related state. This is separate from [`MjViewer`]
/// to allow use in multi-threaded programs, where the physics part
/// runs in another thread and syncs the state with the viewer
/// running in the main thread.
/// 
/// The state can be obtained through [`MjViewer::state`], which will return an `Arc<Mutex<ViewerSharedState>>`
/// instance. For scoped access, you may also use [`MjViewer::with_state_lock`].
#[derive(Debug)]
pub struct ViewerSharedState<M: Deref<Target = MjModel>>{
    /// This attribute, [`ViewerSharedState::data_passive`] and [`ViewerSharedState::data_passive_state_old`]
    /// are used together to detect changes made to the state within the viewer.
    /// This can happen due to changes made through the UI to joints, equalities, actuators, etc.
    data_passive_state: Box<[MjtNum]>,
    data_passive_state_old: Box<[MjtNum]>,
    data_passive: MjData<M>,
    pert: MjvPerturb,
    running: bool,
    user_scene: MjvScene<M>,

    /* Internals */
    last_sync_time: Instant,
    /// Time factor representing the ratio of viewer syncs with model's selected timestep.
    realtime_factor_smooth: f64,
    /// Preallocated buffer for storing the state of new [`MjData`] state.
    data_state_buffer: Box<[MjtNum]>,
}

impl<M: Deref<Target = MjModel> + Clone> ViewerSharedState<M> {
    fn new(model: M, max_user_geom: usize) -> Self {
        // Tracking of changes made between syncs
        let state_size = model.state_size(MjtState::mjSTATE_INTEGRATION as u32) as usize;
        let data_passive_state = vec![0.0; state_size].into_boxed_slice();
        let data_passive_state_old = data_passive_state.clone();
        let data_passive = MjData::new(model.clone());
        let data_state_buffer = data_passive_state.clone();
        Self {
            data_passive_state,
            data_passive_state_old,
            data_passive,
            pert: MjvPerturb::default(),
            running: true,
            user_scene: MjvScene::new(model, max_user_geom),

            /* Internal */
            last_sync_time: Instant::now(),
            realtime_factor_smooth: 1.0,
            data_state_buffer
        }
    }

    /// Checks whether the viewer is still running or is supposed to run.
    pub fn running(&self) -> bool {
        self.running
    }

    /// Returns an immutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`ViewerSharedState::sync_data`].
    pub fn user_scene(&self) -> &MjvScene<M> {
        &self.user_scene
    }

    /// Returns a mutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`ViewerSharedState::sync_data`].
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<M> {
        &mut self.user_scene
    }

    /// Same as [`ViewerSharedState::sync_data`], except it copies the entire [`MjData`]
    /// struct (including large Jacobian and other arrays), not just the state needed for visualization.
    pub fn sync_data_full(&mut self, data: &mut MjData<M>) {
        self._sync_data(data, true);
    }

    /// Syncs the state of viewer's internal [`MjData`] with `data`.
    /// Synchronization happens in two steps.
    /// First the viewer checks if any changes have been made to the internal [`MjData`]
    /// since the last call to this method (since the last sync). Any changes made are
    /// directly copied to the parameter `data`.
    /// Then the `data`'s state overwrites the internal [`MjData`]'s state.
    /// 
    /// Note that users must afterward call [`MjViewer::render`] for the scene
    /// to be rendered and the UI to be processed.
    /// 
    /// <div class="warning">
    /// Synchronization of data is performed via mjv_copyData, which only copies fields
    /// required for visualization purposes.
    /// 
    /// If you require everything to be synced for use in a UI callback,
    /// you need to call appropriate functions/methods to calculate them (e.g., data.forward()).
    /// Alternatively, you can opt-in into syncing the entire [`MjData`] struct by calling
    /// [`ViewerSharedState::sync_data_full`] instead.
    /// 
    /// The following are **NOT SYNCHRONIZED**:
    /// - Jacobian matrices;
    /// - mass matrices.
    /// </div>
    /// 
    pub fn sync_data(&mut self, data: &mut MjData<M>) {
        self._sync_data(data, false);
    }

    /// Data sync implementation.
    fn _sync_data(&mut self, data: &mut MjData<M>, full_sync: bool) {
        /* Update statistics */
        let passive_time = self.data_passive.time();
        let active_time = data.time();
        if passive_time > 0.0 && active_time > passive_time {  // time = 0 means data was reset
            let time_elapsed_sim = active_time - passive_time;
            let elapsed_sync = self.last_sync_time.elapsed();
            if !elapsed_sync.is_zero() {
                self.realtime_factor_smooth += REALTIME_FACTOR_SMOOTHING_FACTOR * (
                    time_elapsed_sim / elapsed_sync.as_secs_f64()
                    - self.realtime_factor_smooth
                );
            }
        } else {  // simulation was reset
            self.realtime_factor_smooth = 1.0;
        }

        self.last_sync_time = Instant::now();

        /* Sync */
        self.data_passive.read_state_into(
            MjtState::mjSTATE_INTEGRATION as u32,
            &mut self.data_passive_state
        );
        if self.data_passive_state != self.data_passive_state_old {
            data.read_state_into(MjtState::mjSTATE_INTEGRATION as u32, &mut self.data_state_buffer);
            for ((new, passive), passive_old) in self.data_state_buffer.iter_mut()
                .zip(&mut self.data_passive_state)
                .zip(&mut self.data_passive_state_old)
            {
                if *passive_old != *passive {
                    *new = *passive;
                }
            }

            data.set_state(&self.data_state_buffer, MjtState::mjSTATE_INTEGRATION as u32);
        }

        if full_sync {
            // Copy everything.
            data.copy_to(&mut self.data_passive);
        } else {
            // Copy only visually-required information to the internal passive data.
            data.copy_visual_to(&mut self.data_passive);
        }

        // Make both saved states the same.
        // If any modification is made through the viewer
        // between syncs, then the above if block will trigger a transfer.
        self.data_passive.read_state_into(  // read to match the synced passive MjData
            MjtState::mjSTATE_INTEGRATION as u32,
            &mut self.data_passive_state
        );
        self.data_passive_state_old.copy_from_slice(&self.data_passive_state);

        // Apply perturbations
        self.pert.apply(self.data_passive.model(), data);
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
    opt: MjvOption,

    /* Internal state */
    last_x: f64,
    last_y: f64,
    last_bnt_press_time: Instant,
    rect_view: MjrRectangle,
    rect_full: MjrRectangle,
    fps_timer: Instant,
    fps_smooth: f64,

    /* OpenGL */
    adapter: RenderBase,
    event_loop: EventLoop<()>,
    modifiers: Modifiers,
    buttons_pressed: ButtonsPressed,
    raw_cursor_position: (f64, f64),

    /* External interaction */
    user_scene: MjvScene<M>,
    shared_state: Arc<Mutex<ViewerSharedState<M>>>,

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
    /// 
    /// Note that the use of [`MjViewerBuilder`] is preferred, because it is more flexible.
    /// Call [`MjViewer::builder`] to create a [`MjViewerBuilder`] instance.
    pub fn launch_passive(model: M, max_user_geom: usize) -> Result<Self, MjViewerError> {
        MjViewerBuilder::new()
            .max_user_geoms(max_user_geom)
            .build_passive(model)
    }

    /// A shortcut for creating an instance of [`MjViewerBuilder`].
    /// The builder can be used to build the viewer after configuring it.
    /// It allows better configuration than [`MjViewer::launch_passive`], which
    /// is fixed to achieve backward compatibility.
    pub fn builder() -> MjViewerBuilder<M> {
        MjViewerBuilder::new()
    }

    /// Checks whether the window is still open.
    pub fn running(&self) -> bool {
        self.shared_state.lock().unwrap().running()
    }

    /// Returns a reference to the shared state [`ViewerSharedState`].
    /// This struct can be used to sync the state of the viewer with
    /// the simulation, possibly running in another thread.
    pub fn state(&self) -> &Arc<Mutex<ViewerSharedState<M>>> {
        &self.shared_state
    }

    /// Acquires a Mutex lock on the [`MjViewer`]'s shared state ([`MjViewer::state`]).
    /// The acquired lock is passed to the function/closure `fun`.
    /// 
    /// # Errors
    /// Returns [`PoisonError`] if the mutex holding the shared state has panicked, thus poisoning
    /// the lock.
    ///
    /// # Example
    /// ```no_run
    /// # use mujoco_rs::viewer::MjViewer;
    /// # use mujoco_rs::prelude::*;
    /// # let model = MjModel::from_xml_string("<mujoco/>").unwrap();
    /// let mut viewer = MjViewer::builder().max_user_geoms(1)
    ///     .build_passive(&model).unwrap();
    /// viewer.with_state_lock(|mut lock| {
    ///     let scene = lock.user_scene_mut();
    ///     scene.create_geom(MjtGeom::mjGEOM_BOX, Some([1.0, 1.0, 1.0]), Some([0.0, 0.0, 0.0]), None, None);
    /// }).unwrap();
    /// ```
    pub fn with_state_lock<F, R>(&self, fun: F) -> Result<R, PoisonError<MutexGuard<'_, ViewerSharedState<M>>>>
        where F: FnOnce(MutexGuard<ViewerSharedState<M>>) -> R
    {
        Ok(fun(self.shared_state.lock()?))
    }

    /// **DEPRECATED** method for reading the state.
    /// This will be removed in 3.0.0, in favor of [`ViewerSharedState::user_scene`],
    /// which allows usage from multiple threads.
    /// 
    /// [`ViewerSharedState`] can be obtained via [`MjViewer::state`], which returns `Arc<Mutex<ViewerSharedState>>`.
    /// Alternatively, [`MjViewer::with_state_lock`] can be used as follows:
    /// ```no_run
    /// # use mujoco_rs::viewer::MjViewer;
    /// # use mujoco_rs::prelude::*;
    /// # let model = MjModel::from_xml_string("<mujoco/>").unwrap();
    /// let mut viewer = MjViewer::builder().build_passive(&model).unwrap();
    /// viewer.with_state_lock(|lock| {
    ///     let scene = lock.user_scene();
    /// });
    /// ```
    /// 
    /// # Note
    /// There is no way to make a fully compatible proxy method to [`ViewerSharedState::user_scene`]
    /// through the viewer as a reference to the scene is returned, thus this method currently
    /// uses a temporary user scene, part of [`MjViewer`]. The viewer then syncs both
    /// [`MjViewer::user_scene`] and [`ViewerSharedState::user_scene`] to achieve backward compatibility,
    /// however we strongly urge you to use the latter as the **FORMER** will be **REMOVED IN THE FUTURE**.
    #[deprecated(since = "2.2.0", note = "use viewer.with_state_lock(|lock| { lock.user_scene(); ... } )")]
    pub fn user_scene(&self) -> &MjvScene<M>{
        &self.user_scene
    }

    /// **DEPRECATED** method for reading the state.
    /// This will be removed in 3.0.0, in favor of [`ViewerSharedState::user_scene_mut`],
    /// which allows usage from multiple threads.
    /// 
    /// [`ViewerSharedState`] can be obtained via [`MjViewer::state`], which returns `Arc<Mutex<ViewerSharedState>>`.
    /// 
    /// # Note
    /// There is no way to make a fully compatible proxy method to [`ViewerSharedState::user_scene_mut`]
    /// through the viewer as a reference to the scene is returned, thus this method currently
    /// uses a temporary user scene, part of [`MjViewer`]. The viewer then syncs both
    /// [`MjViewer::user_scene_mut`] and [`ViewerSharedState::user_scene_mut`] to achieve backward compatibility,
    /// however we strongly urge you to use the latter as the **FORMER** will be **REMOVED IN THE FUTURE**.
    #[deprecated(since = "2.2.0", note = "use viewer.with_state_lock(|mut lock| { lock.user_scene_mut(); ... } )")]
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<M>{
        &mut self.user_scene
    }

    #[deprecated(since = "1.3.0", note = "use viewer.with_state_lock(|lock| { lock.user_scene(); ... } )")]
    pub fn user_scn(&self) -> &MjvScene<M> {
        self.user_scene()
    }

    #[deprecated(since = "1.3.0", note = "use viewer.with_state_lock(|mut lock| { lock.user_scene_mut(); ... } )")]
    pub fn user_scn_mut(&mut self) -> &mut MjvScene<M> {
        self.user_scene_mut()
    }

    /// Adds a user-defined UI callback for custom widgets in the viewer's UI.
    /// The callback receives an [`egui::Context`] reference and can be used to create
    /// custom windows, panels, or other UI elements.
    /// It also receives a mutable reference to [`MjData`], which can be used to read
    /// and modify simulation state. Note that the model can be accessed through [`MjData::model`].
    ///
    /// This method is only available when the `viewer-ui` feature is enabled.
    ///
    /// # Example
    /// ```no_run
    /// # use mujoco_rs::prelude::*;
    /// # use mujoco_rs::viewer::MjViewer;
    /// # let model = MjModel::from_xml_string("<mujoco/>").unwrap();
    /// # let mut viewer = MjViewer::launch_passive(&model, 0).unwrap();
    /// viewer.add_ui_callback(|ctx, data| {
    ///     use mujoco_rs::viewer::egui;
    ///     egui::Window::new("Custom controls")
    ///         .scroll(true)
    ///         .show(ctx, |ui| {
    ///             ui.label("Custom UI element");
    ///         });
    /// });
    /// ```
    #[cfg(feature = "viewer-ui")]
    pub fn add_ui_callback<F>(&mut self, callback: F)
    where
        F: FnMut(&egui::Context, &mut MjData<M>) + 'static
    {
        self.ui.add_ui_callback(callback);
    }

    /// Same as [`MjViewer::add_ui_callback`], except the `callback` does
    /// not receive the passive [`MjData`] instance of the viewer.
    /// Consequently, the mutex of the viewer's shared state doesn't need to
    /// be locked, yielding better performance.
    #[cfg(feature = "viewer-ui")]
    pub fn add_ui_callback_detached<F>(&mut self, callback: F)
    where
        F: FnMut(&egui::Context) + 'static
    {
        self.ui.add_ui_callback_detached(callback);
    }

    /// Deprecated synchronization and rendering method.
    /// Users should use [`MjViewer::sync_data`] instead, which is a proxy
    /// to [`ViewerSharedState::sync_data`], and afterwards call [`MjViewer::render`].
    /// # Migration to new API
    /// To achieve identical behavior, replace the call of this method with
    /// a call to [`MjViewer::sync_data`] and afterwards [`MjViewer::render`].
    /// 
    /// [`MjViewer::render`] must be called by the user, because syncing no longer
    /// processes the UI and renders the scene. This was changed for the purposes of
    /// allowing multithreading --- i.e., rendering in main thread and everything else in a separate thread.
    #[deprecated(since = "2.2.0", note = "replaced with calls to sync_data and render")]
    pub fn sync(&mut self, data: &mut MjData<M>) {
        self.shared_state.lock().unwrap().sync_data(data);
        self.render();
    }

    /// Same as [`MjViewer::sync_data`], except it copies the entire [`MjData`]
    /// struct (including large Jacobian and other arrays), not just the state needed for visualization.
    /// This is a proxy to [`ViewerSharedState::sync_data_full`].
    pub fn sync_data_full(&mut self, data: &mut MjData<M>) {
        self.shared_state.lock().unwrap().sync_data_full(data);
    }

    /// Syncs the state of viewer's internal [`MjData`] with `data`.
    /// This is a proxy to [`ViewerSharedState::sync_data`].
    /// 
    /// Additionally, any changes made to the internal [`MjData`] in between syncs,
    /// get copied back to `data` before the actual sync.
    /// This includes object perturbations.
    /// 
    /// Note that users must afterward call [`MjViewer::render`] for the scene
    /// to be rendered and the UI to be processed.
    /// 
    /// <div class="warning">
    /// Synchronization of data is performed via mjv_copyData, which only copies fields
    /// required for visualization purposes.
    /// 
    /// If you require everything to be synced for use in a UI callback,
    /// you need to call appropriate functions/methods to calculate them (e.g., data.forward()).
    /// Alternatively, you can opt-in into syncing the entire [`MjData`] struct by calling
    /// [`MjViewer::sync_data_full`] instead.
    /// 
    /// The following are **NOT SYNCHRONIZED**:
    /// - Jacobian matrices;
    /// - mass matrices.
    /// </div>
    /// 
    /// # Example
    /// ```no_run
    /// # use mujoco_rs::prelude::*;
    /// # use mujoco_rs::viewer::MjViewer;
    /// # let model = MjModel::from_xml("/path/scene.xml").unwrap();
    /// # let mut viewer = MjViewer::builder().build_passive(&model).unwrap();
    /// # let mut data = MjData::new(&model);
    /// viewer.sync_data(&mut data);  // sync the data
    /// viewer.render();  // render the scene and process the user interface
    /// ```
    pub fn sync_data(&mut self, data: &mut MjData<M>) {
        self.shared_state.lock().unwrap().sync_data(data);
    }

    /// Processes the UI (when enabled), processes events, draws the scene
    /// and swaps buffers in OpenGL.
    pub fn render(&mut self) {
        let RenderBaseGlState {
            gl_context,
            gl_surface,
            ..
        } = self.adapter.state.as_ref().unwrap();

        /* Make sure everything is done on the viewer's window */
        gl_context.make_current(gl_surface).expect("could not make OpenGL context current");

        /* Read the screen size */
        self.update_rectangles(self.adapter.state.as_ref().unwrap().window.inner_size().into());

        /* Process mouse and keyboard events */
        self.process_events();

        /* Update the scene from data and render */
        self.update_scene();

        /* Draw the user menu on top */
        #[cfg(feature = "viewer-ui")]
        self.process_user_ui();

        /* Update the user menu state and overlays */
        self.update_menus();

        /* Flush to the GPU */
        self.swap_buffers();
    }

    /// Perform OpenGL buffer swap.
    fn swap_buffers(&self) {
        let RenderBaseGlState {
            gl_context,
            gl_surface,
            ..
        } = self.adapter.state.as_ref().unwrap();

        /* Swap OpenGL buffers (render to screen) */
        gl_surface.swap_buffers(gl_context).expect("buffer swap in OpenGL failed");
    }

    fn update_smooth_fps(&mut self) {
        let elapsed = self.fps_timer.elapsed();

        let fps = if elapsed.is_zero() {
            self.fps_smooth
        } else {
            1.0 / elapsed.as_secs_f64()
        };

        self.fps_timer = Instant::now();
        self.fps_smooth += FPS_SMOOTHING_FACTOR * (fps - self.fps_smooth);
    }

    /// Updates the scene and draws it to the display.
    fn update_scene(&mut self) {
        /* Update the scene from the MjData state */
        let lock = &mut self.shared_state.lock().unwrap();
        let ViewerSharedState { data_passive, pert, .. } = lock.deref_mut();
        self.scene.update(data_passive, &self.opt, pert, &mut self.camera);

        // Temporary check until 3.0.0. Geom syncing will fail if the target scene is smaller than
        // the requested number of user scenes.
        let new_user_scene = lock.user_scene();
        let old_user_scene = &self.user_scene;
        if !new_user_scene.geoms().is_empty() && !old_user_scene.geoms().is_empty() {
            panic!(
                "Both the new ViewerSharedState::user_scene and the deprecated MjViewer::user_scene are non-empty. \
                 Please update your code to fully use ViewerSharedState::user_scene."
            );
        }

        // Draw geoms drawn through the user scene.
        sync_geoms(new_user_scene, &mut self.scene)
            .expect("could not sync the user scene with the internal scene; this is a bug, please report it.");

        // Temporary (until MuJoCo-rs 3.0.0) sync. Used only for backward compatibility.
        sync_geoms(old_user_scene, &mut self.scene)
            .expect("could not sync the user scene with the internal scene; this is a bug, please report it.");

        self.scene.render(&self.rect_full, &self.context);
    }

    /// Draws the user menu
    fn update_menus(&mut self) {
        let rectangle_from_ui = self.rect_view;
        let rectangle_full = self.rect_full;

        /* Overlay section */
        if self.status.contains(ViewerStatusBit::HELP) {  // Help
            self.context.overlay(
                MjtFont::mjFONT_NORMAL, MjtGridPos::mjGRID_TOPLEFT,
                rectangle_from_ui,
                HELP_MENU_TITLES,
                Some(HELP_MENU_VALUES)
            );
        }

        // Read later-required information and then drop the mutex lock
        let (
            time,
            memory_pct,
            mut total_memory,
            realtime_factor
        ) = {
            let state_lock = self.shared_state.lock().unwrap();
            let data_lock = &state_lock.data_passive;
            let memory_total = data_lock.narena().max(1) as f64;
            (
                data_lock.time(),
                100.0 * data_lock.maxuse_arena() as f64 / memory_total, memory_total,
                state_lock.realtime_factor_smooth
            )
        };

        if self.status.contains(ViewerStatusBit::INFO) {  // Info
            self.update_smooth_fps();

            // Overlay headers
            let headers = concat!(
                "FPS\n",
                "Time\n",
                "Memory\n",
                "Realtime factor"
            );

            // Calculate the amount of memory used and represent with SI units.
            let mut memory_unit = ' ';
            if total_memory > 1e6 {
                total_memory /= 1e6;
                memory_unit = 'M';
            } else if total_memory > 1e3 {
                total_memory /= 1e3;
                memory_unit = 'k';
            }

            // Format values of the overlay.
            let values = format!(
                concat!(
                    "{:.1}\n",
                    "{:.1}\n",
                    "{:.1} % out of {:.1} {}\n",
                    "{:.1} %"
                ),
                self.fps_smooth,
                time,
                memory_pct, total_memory, memory_unit,
                realtime_factor * 100.0
            );

            self.context.overlay(
                MjtFont::mjFONT_NORMAL,
                MjtGridPos::mjGRID_BOTTOMLEFT,
                rectangle_from_ui,
                &headers,
                Some(&values)
            );
        }

        // Check for slowdowns
        if self.status.contains(ViewerStatusBit::WARN_REALTIME) {
            if (realtime_factor - 1.0).abs() > REALTIME_FACTOR_DISPLAY_THRESHOLD {
                self.context.overlay(
                    MjtFont::mjFONT_BIG,
                    MjtGridPos::mjGRID_BOTTOMRIGHT,
                    rectangle_full,
                    &format!("Realtime factor: {:.1} %", realtime_factor * 100.0),
                    None
                );
            }
        }
    }

    /// Gains scoped access to [`egui::Context`], which is part of the UI,
    /// for dealing with custom initialization (e.g., loading in images).
    #[cfg(feature = "viewer-ui")]
    pub fn with_ui_egui_ctx<F>(&mut self, once_fn: F)
        where F: FnOnce(&mut egui::Context)
    {
        self.ui.with_egui_ctx(once_fn);
    }

    /// Draws the user UI
    #[cfg(feature = "viewer-ui")]
    fn process_user_ui(&mut self) {
        /* Draw the user interface */

        use crate::viewer::ui::UiEvent;
        let RenderBaseGlState {window, ..} = &self.adapter.state.as_ref().unwrap();

        let inner_size = window.inner_size();
        self.ui.init_2d();
        let left = self.ui.process(
            window, &mut self.status,
            &mut self.scene, &mut self.opt,
            &mut self.camera, &self.shared_state
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
                Close => self.shared_state.lock().unwrap().running = false,
                Fullscreen => self.toggle_full_screen(),
                ResetSimulation => {
                    let mut lock = self.shared_state.lock().unwrap();
                    lock.data_passive.reset();
                    lock.data_passive.forward();
                },
                AlignCamera => {
                    self.camera = MjvCamera::new_free(&self.model);
                },
                VSyncToggle => {
                    self.update_vsync();
                }
            }
        }
    }

    /// Reads the state of requested vsync setting and makes appropriate calls to [`glutin`].
    fn update_vsync(&self) {
        let RenderBaseGlState {
            gl_surface, gl_context, ..
        } = &self.adapter.state.as_ref().unwrap();

        if self.status.contains(ViewerStatusBit::VSYNC) {
            gl_surface.set_swap_interval(
                gl_context, glutin::surface::SwapInterval::Wait(NonZero::new(1).unwrap())
            ).expect("failed to enable vsync");
        } else {
            gl_surface.set_swap_interval(
                gl_context, glutin::surface::SwapInterval::DontWait
            ).expect("failed to disable vsync");
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
    fn process_events(&mut self) {
        self.event_loop.pump_app_events(Some(Duration::ZERO), &mut self.adapter);
        while let Some(window_event) = self.adapter.queue.pop_front() {
            #[cfg(feature = "viewer-ui")]
            {
                let window: &winit::window::Window = &self.adapter.state.as_ref().unwrap().window;
                self.ui.handle_events(window, &window_event);
            }

            match window_event {
                WindowEvent::ModifiersChanged(modifiers) => self.modifiers = modifiers,
                WindowEvent::MouseInput {state, button, .. } => {
                    let is_pressed = state == ElementState::Pressed;
                    
                    #[cfg(feature = "viewer-ui")]
                    if self.ui.covered() && is_pressed {
                        continue;
                    }

                    let index = match button {
                        MouseButton::Left => {
                            self.process_left_click(state);
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

                    // The UI might not be detected as covered as dragging can happen slightly outside
                    // of a (popup) window. This might seem like an ad-hoc solution, but is at the time the
                    // shortest and most efficient one.
                    #[cfg(feature = "viewer-ui")]
                    if self.ui.dragged() {
                        continue;
                    }

                    self.process_cursor_pos(x, y);
                }

                // Set the viewer's state to pending exit.
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::KeyQ),
                        state: ElementState::Pressed, ..
                    }, ..
                } if self.modifiers.state().control_key()  => {
                    self.shared_state.lock().unwrap().running = false;
                }

                // Also set the viewer's state to pending exit if the window no longer exists.
                WindowEvent::CloseRequested => { self.shared_state.lock().unwrap().running = false }

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

                 // Toggle info menu
                WindowEvent::KeyboardInput {
                    event: KeyEvent {
                        physical_key: PhysicalKey::Code(KeyCode::F2),
                        state: ElementState::Pressed, ..
                    }, ..
                } => {
                    self.status.toggle(ViewerStatusBit::INFO);
                }

                // Toggle VSync
                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::F3), state: ElementState::Pressed, ..},
                    ..
                } => {
                    self.status.toggle(ViewerStatusBit::VSYNC);
                    self.update_vsync();
                }

                // Non-realtime warnings
                WindowEvent::KeyboardInput {
                    event: KeyEvent {physical_key: PhysicalKey::Code(KeyCode::F4), state: ElementState::Pressed, ..},
                    ..
                } => {
                    self.status.toggle(ViewerStatusBit::WARN_REALTIME);
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
                    let mut lock = self.shared_state.lock().unwrap();
                    lock.data_passive.reset();
                    lock.data_passive.forward();
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
                    if self.ui.covered() {
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
    fn process_cursor_pos(&mut self, x: f64, y: f64) {
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

        let mut lock = self.shared_state.lock().unwrap();
        let ViewerSharedState {data_passive, pert, ..} = lock.deref_mut();
        if buttons.contains(ButtonsPressed::LEFT) {
            if pert.active == MjtPertBit::mjPERT_TRANSLATE as i32 {
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
        if pert.active == 0 {
            self.camera.move_(action, &self.model, dx / height, dy / height, &self.scene);
        }
        else {  // When the perturbation is active, move apply the perturbation.
            pert.move_(&self.model, data_passive, action, dx / height, dy / height, &self.scene);
        }
    }

    /// Processes left clicks and double left clicks.
    fn process_left_click(&mut self, state: ElementState) {
        let modifier_state = self.modifiers.state();
        let mut lock = self.shared_state.lock().unwrap();
        let ViewerSharedState {data_passive, pert, ..} = lock.deref_mut();
        match state {
            ElementState::Pressed => {
                /* Clicking and holding applies perturbation */
                if pert.select > 0 && modifier_state.control_key() {
                    let type_ = if modifier_state.alt_key() {
                        MjtPertBit::mjPERT_TRANSLATE
                    } else {
                        MjtPertBit::mjPERT_ROTATE
                    };
                    pert.start(type_, &self.model, data_passive, &self.scene);
                }

                /* Double click detection */
                if self.last_bnt_press_time.elapsed().as_millis() < DOUBLE_CLICK_WINDOW_MS {
                    let cp = self.raw_cursor_position;
                    let x = cp.0;
                    let y = self.rect_full.height as f64 - cp.1;

                    /* Obtain the selection */ 
                    let rect = &self.rect_full;
                    let (body_id, _, flex_id, skin_id, xyz) = self.scene.find_selection(
                        data_passive, &self.opt,
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
                            pert.select = body_id;
                            pert.flexselect = flex_id;
                            pert.skinselect = skin_id;
                            pert.active = 0;
                            pert.update_local_pos(xyz, data_passive);
                        }
                        else {
                            pert.select = 0;
                            pert.flexselect = -1;
                            pert.skinselect = -1;
                        }
                    }
                }
                self.last_bnt_press_time = Instant::now();
            },
            ElementState::Released => {
                // Clear perturbation when left click is released.
                pert.active = 0;
            },
        };
    }
}


/// Builder for [`MjViewer`].
/// ### Default settings:
/// - `window_name`: MuJoCo Rust Viewer (MuJoCo \<MuJoCo version here\>)
/// - `max_user_geoms`: 0
/// - `vsync`: false
/// - `warn_non_realtime`: false
/// 
pub struct MjViewerBuilder<M: Deref<Target = MjModel> + Clone> {
    /// The name shown on the window decoration.
    window_name: Cow<'static, str>,
    /// Maximum number of geoms that can be given by the user for custom visualization.
    max_user_geoms: usize,
    /// Start the viewer with vertical synchronization. This should be used only if rendering
    /// and simulation are separated by threads and you are ok with [`MjViewer::render`]
    /// blocking to achieve the correct refresh rate (of your monitor).
    vsync: bool,

    /// Start the viewer with warnings enabled for non-realtime synchronization.
    /// When this is enabled and the simulation state isn't synced in realtime, an overlay will be displayed
    /// in the bottom right corner indicating the realtime percentage.
    /// The warning will only be shown if the deviation is 2 % from realtime or more.
    warn_non_realtime: bool,

    /* Miscellaneous */
    /// Used to store the model type only. Useful for type inference.
    model_type: PhantomData<M>,
}

impl<M: Deref<Target = MjModel> + Clone> MjViewerBuilder<M> {
    builder_setters! {
        window_name: S where S: Into<Cow<'static, str>>; "text shown in the title of the window.";
        max_user_geoms: usize; "maximum number of geoms that can be drawn by the user in addition to the regular geoms.";
        vsync: bool; "enable vertical synchronization by default.";
        warn_non_realtime: bool; "enable showing an overlay when the simulation state isn't synced in realtime (deviation larger than 2 %).";
    }
}

impl<M: Deref<Target = MjModel> + Clone> MjViewerBuilder<M> {
    pub fn new() -> Self {
        Self { 
            window_name: Cow::Owned(format!("MuJoCo Rust Viewer (MuJoCo {})", get_mujoco_version())),
            max_user_geoms: 0, vsync: false, warn_non_realtime: false,
            model_type: PhantomData
        }
    }

    pub fn build_passive(&self, model: M) -> Result<MjViewer<M>, MjViewerError> {
        let (w, h) = MJ_VIEWER_DEFAULT_SIZE_PX;
        let mut event_loop = EventLoop::new().map_err(MjViewerError::EventLoopError)?;
        let adapter = RenderBase::new(
            w, h,
            self.window_name.to_string(),
            &mut event_loop,
            true  // process events
        );

        /* Initialize the OpenGL related things */
        let RenderBaseGlState {
            gl_context,
            gl_surface,
            #[cfg(feature = "viewer-ui")] window,
            ..
        } = adapter.state.as_ref().unwrap();
        gl_context.make_current(gl_surface).map_err(MjViewerError::GlutinError)?;

        // Configure vertical synchronization
        if self.vsync {
            gl_surface.set_swap_interval(
                gl_context,
                glutin::surface::SwapInterval::Wait(NonZero::new(1).unwrap())
            ).map_err(|e| MjViewerError::GlutinError(e))?;
        } else {
            gl_surface.set_swap_interval(gl_context, glutin::surface::SwapInterval::DontWait).map_err(
                |e| MjViewerError::GlutinError(e)
            )?;
        }

        event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);

        let ngeom = model.ffi().ngeom as usize;
        let scene = MjvScene::new(model.clone(), ngeom + self.max_user_geoms + EXTRA_SCENE_GEOM_SPACE);
        let context = MjrContext::new(&model);
        let camera  = MjvCamera::new_free(&model);

        // Tracking of changes made between syncs
        let shared_state = Arc::new(Mutex::new(ViewerSharedState::new(model.clone(), self.max_user_geoms)));
        let user_scene = MjvScene::new(model.clone(), self.max_user_geoms);

        // User interface
        #[cfg(feature = "viewer-ui")]
        let ui = ui::ViewerUI::new(model.clone(), &window, &gl_surface.display());
        #[cfg(feature = "viewer-ui")]
        let mut status = ViewerStatusBit::UI;
        #[cfg(not(feature = "viewer-ui"))]
        let mut status = ViewerStatusBit::HELP;

        status.set(ViewerStatusBit::VSYNC, self.vsync);
        status.set(ViewerStatusBit::WARN_REALTIME, self.warn_non_realtime);

        Ok(MjViewer {
            model,
            scene,
            context,
            camera,
            opt: MjvOption::default(),
            user_scene,  // TEMPORARY! TODO: Drop in 3.0.0
            shared_state,
            last_x: 0.0,
            last_y: 0.0,
            last_bnt_press_time: Instant::now(),
            fps_timer: Instant::now(),
            fps_smooth: 60.0,
            rect_view: MjrRectangle::default(),
            rect_full: MjrRectangle::default(),
            adapter,
            event_loop,
            modifiers: Modifiers::default(),
            buttons_pressed: ButtonsPressed::empty(),
            raw_cursor_position: (0.0, 0.0),
            #[cfg(feature = "viewer-ui")] ui,
            status
        })
    }
}

impl<M: Deref<Target = MjModel> + Clone> Default for MjViewerBuilder<M> {
    fn default() -> Self {
        MjViewerBuilder::new()
    }
}

bitflags! {
    #[derive(Debug)]
    struct ViewerStatusBit: u8 {
        const HELP = 1 << 0;
        const VSYNC = 1 << 1;
        const INFO = 1 << 2;
        const WARN_REALTIME = 1 << 3;
        #[cfg(feature = "viewer-ui")] const UI = 1 << 4;
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
