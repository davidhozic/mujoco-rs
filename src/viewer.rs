//! Module related to implementation of the [`MjViewer`]. For implementation of the C++ wrapper,
//! see [`crate::cpp_viewer::MjViewerCpp`] (enabled by the `cpp-viewer` cargo feature).
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
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use std::ops::{Deref, DerefMut};
use std::num::NonZero;
use std::error::Error;
use std::fmt::Display;
use std::borrow::Cow;

use bitflags::bitflags;

use crate::prelude::{MjSpec, MjrContext, MjrRectangle, MjtFont, MjtGridPos};
use crate::vis_common::{sync_geoms, flip_image_vertically, write_png};
use crate::winit_gl_base::{RenderBaseGlState, RenderBase};
use crate::wrappers::mj_primitive::{MjtNum, MjtSize};
use crate::wrappers::mj_data::{MjData, MjtState};
use crate::{builder_setters, mujoco_version};
use crate::wrappers::mj_visualization::*;
use crate::wrappers::mj_model::MjModel;
use crate::util::LockUnpoison;


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

// Compile-time check: HELP_MENU_TITLES and HELP_MENU_VALUES must have the same
// number of newline-delimited entries, otherwise the help overlay misaligns.
const fn count_byte(s: &[u8], target: u8) -> usize {
    let mut count = 0;
    let mut i = 0;
    while i < s.len() {
        if s[i] == target {
            count += 1;
        }
        i += 1;
    }
    count
}
const _: () = assert!(
    count_byte(HELP_MENU_TITLES.as_bytes(), b'\n')
        == count_byte(HELP_MENU_VALUES.as_bytes(), b'\n'),
    "HELP_MENU_TITLES and HELP_MENU_VALUES must have the same number of lines"
);

/// Errors that can occur when initializing or running the MuJoCo viewer.
#[derive(Debug)]
#[non_exhaustive]
pub enum MjViewerError {
    /// The event loop failed to initialize.
    EventLoopError(winit::error::EventLoopError),
    /// A glutin operation failed.
    GlutinError(glutin::error::Error),
    /// Returned when the egui painter (OpenGL UI renderer) fails to initialize.
    PainterInitError(String),
    /// OpenGL / window initialization failed.
    GlInitFailed(crate::error::GlInitError),
    /// A scene operation failed (e.g. user-scene sync overflowed the geom buffer).
    SceneError(crate::error::MjSceneError),
}

/// Formats a human-readable description of the viewer error.
impl Display for MjViewerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EventLoopError(e) => write!(f, "event loop failed to initialize: {e}"),
            Self::GlutinError(e) => write!(f, "glutin error: {e}"),
            Self::PainterInitError(e) => write!(f, "failed to initialize egui painter: {e}"),
            Self::GlInitFailed(e) => write!(f, "GL initialization failed: {e}"),
            Self::SceneError(e) => write!(f, "scene error: {e}"),
        }
    }
}

/// Provides the underlying error source, if any.
impl Error for MjViewerError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::EventLoopError(e) => Some(e),
            Self::GlutinError(e) => Some(e),
            Self::PainterInitError(_) => None,
            Self::GlInitFailed(e) => Some(e),
            Self::SceneError(e) => Some(e),
        }
    }
}

/// Converts an [`MjSceneError`](crate::error::MjSceneError) into [`MjViewerError::SceneError`].
impl From<crate::error::MjSceneError> for MjViewerError {
    fn from(e: crate::error::MjSceneError) -> Self {
        Self::SceneError(e)
    }
}

/// Converts a [`GlInitError`](crate::error::GlInitError) into [`MjViewerError::GlInitFailed`].
impl From<crate::error::GlInitError> for MjViewerError {
    fn from(e: crate::error::GlInitError) -> Self {
        Self::GlInitFailed(e)
    }
}

/// Converts a [`glutin::error::Error`] into [`MjViewerError::GlutinError`].
impl From<glutin::error::Error> for MjViewerError {
    fn from(e: glutin::error::Error) -> Self {
        Self::GlutinError(e)
    }
}

/// Internal state that is used by [`MjViewer`] to store
/// [`MjData`]-related state. This is separate from [`MjViewer`]
/// to allow use in multi-threaded programs, where the physics part
/// runs in another thread and syncs the state with the viewer
/// running in the main thread.
/// 
/// The state can be obtained through [`MjViewer::state`], which will return a reference to an `Arc<Mutex<ViewerSharedState>>`
/// instance. For scoped access, you may also use [`MjViewer::with_state_lock`].
#[derive(Debug)]
pub struct ViewerSharedState {
    /// This attribute, [`ViewerSharedState::data_passive`] and [`ViewerSharedState::data_passive_state_old`]
    /// are used together to detect changes made to the state within the viewer.
    /// This can happen due to changes made through the UI to joints, equalities, actuators, etc.
    data_passive_state: Box<[MjtNum]>,
    data_passive_state_old: Box<[MjtNum]>,
    data_passive: MjData<Box<MjModel>>,
    pert: MjvPerturb,
    running: Arc<AtomicBool>,
    user_scene: MjvScene,

    /* Internals */
    last_sync_time: Instant,
    /// Time factor representing the ratio of viewer syncs with model's selected timestep.
    realtime_factor_smooth: f64,
    /// Preallocated buffer for storing the new [`MjData`] state.
    data_state_buffer: Box<[MjtNum]>,
}

impl ViewerSharedState {
    fn new<M: Deref<Target = MjModel>>(model: M, max_user_geom: usize) -> Self {
        // Empty values to avoid unnecessary double creation
        let empty: Box<[MjtNum]> = vec![].into_boxed_slice();
        let empty_model = Box::new(MjSpec::new().compile().unwrap());
        let empty_data = MjData::new(empty_model);
        let empty_scene = MjvScene::new(empty_data.model(), 0);

        let mut shared_state = Self {
            data_passive: empty_data,
            data_passive_state: empty.clone(),
            data_passive_state_old: empty.clone(),
            data_state_buffer: empty,
            user_scene: empty_scene,
            pert: MjvPerturb::default(),
            running: Arc::new(AtomicBool::new(true)),
            last_sync_time: Instant::now(),
            realtime_factor_smooth: 1.0,
        };

        shared_state.reload_model(model, max_user_geom);
        shared_state
    }

    /// Reinitializes all model-dependent internal state.
    /// Called on construction and whenever [`_sync_data`](Self::_sync_data) detects a model change.
    fn reload_model<M: Deref<Target = MjModel>>(&mut self, model: M, max_user_geom: usize) {
        let model_passive = Box::new(model.clone());
        self.data_passive = MjData::new(model_passive);
        let model_passive = self.data_passive.model();

        self.user_scene = MjvScene::new(model_passive, max_user_geom);
        let state_size = model_passive.state_size(MjtState::mjSTATE_INTEGRATION as u32);
        self.data_passive_state = vec![0.0; state_size].into_boxed_slice();
        // Read the actual initial state (qpos0 may be non-zero) so that data_passive_state_old
        // matches data_passive_state from the start, preventing a spurious write-back of the
        // default pose to the incoming data on the first sync after a model change.
        self.data_passive.read_state_into(
            MjtState::mjSTATE_INTEGRATION as u32,
            &mut self.data_passive_state,
        );
        self.data_passive_state_old = self.data_passive_state.clone();
        self.data_state_buffer = self.data_passive_state.clone();
        self.realtime_factor_smooth = 1.0;
        self.pert = MjvPerturb::default();
    }

    /// Checks whether the viewer is still running or is supposed to run.
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Returns an immutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`ViewerSharedState::sync_data`].
    pub fn user_scene(&self) -> &MjvScene {
        &self.user_scene
    }

    /// Returns a mutable reference to a user scene for drawing custom visual-only geoms.
    /// Geoms in the user scene are preserved between calls to [`ViewerSharedState::sync_data`].
    pub fn user_scene_mut(&mut self) -> &mut MjvScene {
        &mut self.user_scene
    }

    /// Same as [`ViewerSharedState::sync_data`], except it copies the entire [`MjData`]
    /// struct (including large Jacobian and other arrays), not just the state needed for visualization.
    ///
    /// # Panics
    /// Panics if the internal data copy fails due to an inconsistent model state (indicates a bug).
    pub fn sync_data_full<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>) {
        self._sync_data(data, true);
    }

    /// Syncs the viewer's internal passive [`MjData`] with `data`.
    /// Synchronization happens in three steps:
    ///
    /// 1. The viewer checks if any changes have been made to the internal [`MjData`]
    ///    since the last call to this method (since the last sync). Changed elements
    ///    are selectively merged into `data` (elements the viewer did not touch are
    ///    preserved).
    /// 2. `data` is copied into the viewer's internal passive copy
    ///    (visualization fields only; see warning below).
    /// 3. Perturbations are applied to `data` via [`MjvPerturb::apply`], which
    ///    **unconditionally zeroes `xfrc_applied`** before writing any active
    ///    perturbation forces. Any external forces previously set on `data` will be
    ///    cleared.
    ///
    /// Note that users must afterward call [`MjViewer::render`] for the scene
    /// to be rendered and the UI to be processed.
    ///
    /// <div class="warning">
    /// The user's data is copied into the viewer's internal passive copy via ``mjv_copyData``,
    /// which skips large computed arrays not required for visualization.
    /// The viewer's passive copy will therefore **not** contain:
    ///
    /// - mass matrices (``qM``, ``qLD``, ``qLDiagInv``, ``qLU``);
    /// - constraint arrays (``efc_*``, ``iefc_*``, including constraint Jacobians).
    ///
    /// In UI callbacks these fields will be absent unless
    /// [`ViewerSharedState::sync_data_full`] is used or they are recomputed explicitly
    /// (e.g. via `data.forward()`).
    ///
    /// Additionally, because the viewer may write integration state (e.g. ``ctrl``) back
    /// to the user's `data`, any Jacobians or other derived quantities in `data` may be
    /// stale after this call and should be recomputed if needed.
    /// </div>
    ///
    /// # Panics
    /// Panics if the internal data copy or state merge fails due to an inconsistent model
    /// state (indicates a bug).
    pub fn sync_data<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>) {
        self._sync_data(data, false);
    }

    /// Data sync implementation.
    fn _sync_data<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>, full_sync: bool) {
        /* Recreate internal data and user scene when the model changes */
        if data.model().signature() != self.data_passive.model().signature() {
            let max_user_geom = self.user_scene.maxgeom() as usize;
            self.reload_model(data.model(), max_user_geom);
        }

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

            // SAFETY: data_state_buffer was filled by read_state_into using the
            // same state spec on a compatible model, so eq_active bytes (included
            // by mjSTATE_INTEGRATION) come from MuJoCo's own canonical encoding.
            unsafe { data.set_state(&self.data_state_buffer, MjtState::mjSTATE_INTEGRATION as u32) }
                .unwrap();
        }

        if full_sync {
            // Copy everything.
            data.copy_to(&mut self.data_passive)
                .unwrap();
        } else {
            // Copy only visually-required information to the internal passive data.
            data.copy_visual_to(&mut self.data_passive)
                .unwrap();
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
        self.pert.apply(data);
    }
}


/// A Rust-native implementation of the MuJoCo viewer. To conform to Rust's safety rules,
/// the viewer doesn't store a mutable reference to the [`MjData`] struct, but it instead
/// accepts it as a parameter in its methods.
/// 
/// The [`MjViewer::sync_data`] method must be called to sync the state of [`MjViewer`] and [`MjData`].
/// 
/// # Shortcuts
/// Main keyboard and mouse shortcuts can be viewed by pressing `F1`.
/// Additionally, some visualization toggles are included, but not displayed
/// in the `F1` help menu:
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
pub struct MjViewer {
    /* MuJoCo rendering */
    scene: MjvScene,
    context: MjrContext,
    camera: MjvCamera,

    /* Other MuJoCo related */
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
    shared_state: Arc<Mutex<ViewerSharedState>>,
    /// Shared atomic flag cloned from [`ViewerSharedState::running`].
    /// Allows [`MjViewer::running`] to read the flag without locking the mutex on every call
    /// (the user's sim loop polls this on every iteration).
    running_flag: Arc<AtomicBool>,

    /* User interface */
    #[cfg(feature = "viewer-ui")]
    ui: ui::ViewerUI,

    status: ViewerStatusBit,

    /// Cached number of MJCF-defined cameras (`model.ncam`).
    /// Updated in [`update_scene`](Self::update_scene) whenever the model changes.
    /// Avoids locking `shared_state` just to read this integer in [`cycle_camera`](Self::cycle_camera).
    ncam: MjtSize,

    /// Pending screenshot request. [`Some`] with `(viewport_only, depth)` flags when a
    /// screenshot is queued; [`None`] otherwise.
    screenshot_pending: Option<(bool, bool)>
}

impl MjViewer {
    /// Launches the MuJoCo viewer. A [`Result`] struct is returned that either contains
    /// [`MjViewer`] or a [`MjViewerError`]. The `max_user_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    ///
    /// Note that the use of [`MjViewerBuilder`] is preferred, because it is more flexible.
    /// Call [`MjViewer::builder`] to create a [`MjViewerBuilder`] instance.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the [`MjViewer`].
    /// # Errors
    /// - [`MjViewerError::EventLoopError`] if the event loop fails to initialize.
    /// - [`MjViewerError::GlInitFailed`] if OpenGL / window initialization fails.
    /// - [`MjViewerError::GlutinError`] if a glutin operation fails.
    /// - [`MjViewerError::PainterInitError`] if the UI painter fails to initialize
    ///   (feature `viewer-ui`).
    pub fn launch_passive<M: Deref<Target = MjModel>>(model: M, max_user_geom: usize) -> Result<Self, MjViewerError> {
        MjViewerBuilder::new()
            .max_user_geoms(max_user_geom)
            .build_passive(model)
    }

    /// A shortcut for creating an instance of [`MjViewerBuilder`].
    /// The builder can be used to build the viewer after configuring it.
    /// It allows better configuration than [`MjViewer::launch_passive`], which
    /// is fixed to achieve backward compatibility.
    pub fn builder() -> MjViewerBuilder {
        MjViewerBuilder::new()
    }

    /// Checks whether the viewer is still running.
    pub fn running(&self) -> bool {
        self.running_flag.load(Ordering::Relaxed)
    }

    /// Returns a reference to the shared state [`ViewerSharedState`].
    /// This struct can be used to sync the state of the viewer with
    /// the simulation, possibly running in another thread.
    pub fn state(&self) -> &Arc<Mutex<ViewerSharedState>> {
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
    pub fn with_state_lock<F, R>(&self, fun: F) -> Result<R, PoisonError<MutexGuard<'_, ViewerSharedState>>>
        where F: FnOnce(MutexGuard<ViewerSharedState>) -> R
    {
        Ok(fun(self.shared_state.lock()?))
    }

    /// Adds a user-defined UI callback for custom widgets in the viewer's UI.
    /// The callback receives an [`egui::Context`] reference and can be used to create
    /// custom windows, panels, or other UI elements.
    /// It also receives a mutable reference to [`MjData`], which can be used to read
    /// and modify simulation state. Note that the model can be accessed through [`MjData::model`].
    ///
    /// This method is only available when the `viewer-ui` feature is enabled.
    ///
    /// # Note
    /// The viewer's internal shared-state [`Mutex`] is **held for the entire
    /// duration of the callback** (because `data` is a live borrow of the guarded
    /// `data_passive` field). Do **not** attempt to lock the shared state again from
    /// within the callback as that will deadlock the viewer thread:
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
        F: FnMut(&egui::Context, &mut MjData<Box<MjModel>>) + 'static
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

    /// Same as [`MjViewer::sync_data`], except it copies the entire [`MjData`]
    /// struct (including large Jacobian and other arrays), not just the state needed for visualization.
    /// This is a proxy to [`ViewerSharedState::sync_data_full`].
    ///
    /// # Panics
    /// Panics if the internal data copy fails due to an inconsistent model state (indicates a bug).
    pub fn sync_data_full<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>) {
        self.shared_state.lock_unpoison().sync_data_full(data);
    }

    /// Syncs the state of viewer's internal [`MjData`] with `data`.
    /// This is a proxy to [`ViewerSharedState::sync_data`].
    /// 
    /// Any changes made to the internal [`MjData`] in between syncs
    /// get selectively merged back into `data` before the copy.
    /// Perturbations are applied to `data` **after** the sync, which
    /// **unconditionally zeroes `xfrc_applied`** (see
    /// [`ViewerSharedState::sync_data`] for details).
    /// 
    /// Note that users must afterward call [`MjViewer::render`] for the scene
    /// to be rendered and the UI to be processed.
    /// 
    /// <div class="warning">
    /// The user's data is copied into the viewer's internal passive copy via ``mjv_copyData``,
    /// which skips large computed arrays not required for visualization.
    /// The viewer's passive copy will therefore **not** contain:
    ///
    /// - mass matrices (``qM``, ``qLD``, ``qLDiagInv``, ``qLU``);
    /// - constraint arrays (``efc_*``, ``iefc_*``, including constraint Jacobians).
    ///
    /// In UI callbacks these fields will be absent unless
    /// [`MjViewer::sync_data_full`] is used or they are recomputed explicitly
    /// (e.g. via `data.forward()`).
    ///
    /// Additionally, because the viewer may write integration state (e.g. ``ctrl``) back
    /// to the user's `data`, any Jacobians or other derived quantities in `data` may be
    /// stale after this call and should be recomputed if needed.
    /// </div>
    ///
    /// # Panics
    /// Panics if the internal data copy or state merge fails due to an inconsistent model
    /// state (indicates a bug).
    ///
    /// # Example
    /// ```no_run
    /// # use mujoco_rs::prelude::*;
    /// # use mujoco_rs::viewer::MjViewer;
    /// # let model = MjModel::from_xml("/path/scene.xml").unwrap();
    /// # let mut viewer = MjViewer::builder().build_passive(&model).unwrap();
    /// # let mut data = MjData::new(&model);
    /// viewer.sync_data(&mut data);  // sync the data
    /// viewer.render().unwrap();  // render the scene and process the user interface
    /// ```
    pub fn sync_data<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>) {
        self.shared_state.lock_unpoison().sync_data(data);
    }

    /// Processes the UI (when enabled), processes events, draws the scene
    /// and swaps buffers in OpenGL.
    /// # Errors
    /// - [`MjViewerError::GlutinError`] if the OpenGL context cannot be made current or the buffer swap fails.
    /// - [`MjViewerError::SceneError`] if synchronizing user scene geoms fails (e.g. the scene is
    ///   full) or if reading pixels for a pending screenshot fails.
    pub fn render(&mut self) -> Result<(), MjViewerError> {
        let RenderBaseGlState {
            gl_context,
            gl_surface,
            ..
        } = self.adapter.state.as_ref().unwrap();

        /* Make sure everything is done on the viewer's window */
        gl_context.make_current(gl_surface)?;

        /* Read the screen size */
        self.update_rectangles(self.adapter.state.as_ref().unwrap().window.inner_size().into());

        /* Process mouse and keyboard events */
        self.process_events();

        /* Update the scene from data and render */
        self.update_scene()?;

        /* Viewport-only screenshot: capture the centered scene before the UI
         * panel is drawn on top (the scene is rendered to rect_full, so it
         * fills the entire window). */
        if let Some((true, _)) = self.screenshot_pending {
            let (_, depth) = self.screenshot_pending.take().unwrap();
            self.capture_screenshot(depth)?;
        }

        /* Draw the user menu on top */
        #[cfg(feature = "viewer-ui")]
        self.process_user_ui();

        /* Update the user menu state and overlays */
        self.update_menus();

        /* Full-window screenshot: capture after all rendering (UI + overlays). */
        if let Some((false, _)) = self.screenshot_pending {
            let (_, depth) = self.screenshot_pending.take().unwrap();
            self.capture_screenshot(depth)?;
        }

        /* Flush to the GPU */
        self.swap_buffers()
    }

    /// Perform OpenGL buffer swap.
    fn swap_buffers(&self) -> Result<(), MjViewerError> {
        let RenderBaseGlState {
            gl_context,
            gl_surface,
            ..
        } = self.adapter.state.as_ref().unwrap();

        /* Swap OpenGL buffers (render to screen) */
        gl_surface.swap_buffers(gl_context).map_err(MjViewerError::GlutinError)
    }

    /// Captures the current framebuffer contents as a PNG screenshot.
    /// Always reads from [`rect_full`](Self::rect_full) (the entire window).
    /// The caller controls what is visible by choosing *when* to call this
    /// method in the render pipeline. When `depth` is `true`, a 16-bit
    /// grayscale depth image is saved instead of an RGB image.
    ///
    /// # Errors
    /// Returns [`MjViewerError::SceneError`] if reading pixels from the framebuffer fails.
    fn capture_screenshot(&self, depth: bool) -> Result<(), MjViewerError> {
        let rect = &self.rect_full;

        let w = rect.width as usize;
        let h = rect.height as usize;
        if w == 0 || h == 0 {
            return Ok(());
        }

        // Generate a timestamped filename.
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        if depth {
            let mut depth_buf = vec![0.0f32; w * h];
            self.context.read_pixels(None, Some(&mut depth_buf), rect)
                .map_err(MjViewerError::SceneError)?;

            // OpenGL reads bottom-up; flip for top-down PNG row order.
            flip_image_vertically(&mut depth_buf, h, w);

            // Linearize raw OpenGL depth into metric distance.
            let (map, stat) = {
                let lock = self.shared_state.lock_unpoison();
                let model = lock.data_passive.model();
                (model.vis().map.clone(), model.stat().clone())
            };
            let extent = stat.extent as f32;
            let near = map.znear * extent;
            let far = map.zfar * extent;
            for value in &mut depth_buf {
                *value = near / (1.0 - *value * (1.0 - near / far));
            }

            // Normalize to 16-bit range.
            let max = depth_buf.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            let min = depth_buf.iter().cloned().fold(f32::INFINITY, f32::min);
            let scale = u16::MAX as f32;
            let range = max - min;
            let encoded: Box<[u8]> = depth_buf
                .iter()
                .flat_map(|&x| {
                    let norm = if range > 0.0 { (x - min) / range } else { 0.0 };
                    ((norm * scale).clamp(0.0, scale) as u16).to_be_bytes()
                })
                .collect();

            let path = format!("screenshot_{timestamp}_depth.png");
            if let Err(e) = write_png(
                &path, &encoded, w as u32, h as u32,
                png::ColorType::Grayscale, png::BitDepth::Sixteen, png::Compression::High
            ) {
                eprintln!("depth screenshot failed: {e}");
            } else {
                eprintln!("depth screenshot saved to {path} (min={min:.4}, max={max:.4})");
            }
        } else {
            let mut rgb = vec![0u8; w * h * 3];
            self.context.read_pixels(Some(&mut rgb), None, rect)
                .map_err(MjViewerError::SceneError)?;

            // OpenGL reads bottom-up; flip for top-down PNG row order.
            flip_image_vertically(&mut rgb, h, w * 3);

            let path = format!("screenshot_{timestamp}.png");
            if let Err(e) = write_png(
                &path, &rgb, w as u32, h as u32,
                png::ColorType::Rgb, png::BitDepth::Eight, png::Compression::High
            ) {
                eprintln!("screenshot failed: {e}");
            } else {
                eprintln!("screenshot saved to {path}");
            }
        }
        Ok(())
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
    fn update_scene(&mut self) -> Result<(), MjViewerError> {
        {
            let mut lock = self.shared_state.lock_unpoison();
            let ViewerSharedState { data_passive, pert, user_scene, .. } = lock.deref_mut();

            /* Recreate scene when the model changes */
            if data_passive.model().signature() != self.scene.signature() {
                let new_model = data_passive.model();
                let ngeom = new_model.ffi().ngeom as usize;
                let max_user_geom = user_scene.maxgeom() as usize;
                self.scene = MjvScene::new(
                    new_model,
                    ngeom + max_user_geom + EXTRA_SCENE_GEOM_SPACE
                );

                // Reset to a free camera: a tracking or fixed camera may reference a body
                // or camera ID that does not exist in the new model.
                self.camera = MjvCamera::new_free(new_model);
                // Recreate the rendering context so that GPU resources (textures,
                // meshes, heightfields, skins) match the new model.
                // SAFETY: the GL context was made current in render() before this call.
                self.context = unsafe { MjrContext::new(new_model) };
                self.ncam = new_model.ffi().ncam;
                #[cfg(feature = "viewer-ui")]
                self.ui.update_names(new_model);
            }

            /* Update and render the scene from the MjData state */
            self.scene.update(data_passive, &self.opt, pert, &mut self.camera);

            // Draw geoms drawn through the user scene.
            sync_geoms(user_scene, &mut self.scene)?;
        }
        self.scene.render(&self.rect_full, &self.context);
        Ok(())
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
            let state_lock = self.shared_state.lock_unpoison();
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
                headers,
                Some(&values)
            );
        }

        // Check for slowdowns
        if self.status.contains(ViewerStatusBit::WARN_REALTIME)
            && (realtime_factor - 1.0).abs() > REALTIME_FACTOR_DISPLAY_THRESHOLD
        {
            self.context.overlay(
                MjtFont::mjFONT_BIG,
                MjtGridPos::mjGRID_BOTTOMRIGHT,
                rectangle_full,
                &format!("Realtime factor: {:.1} %", realtime_factor * 100.0),
                None
            );
        }
    }

    /// Gains scoped access to [`egui::Context`], which is part of the UI,
    /// for dealing with custom initialization (e.g., loading in images).
    #[cfg(feature = "viewer-ui")]
    pub fn with_ui_egui_ctx<F>(&mut self, once_fn: F)
        where F: FnOnce(&egui::Context)
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
            &mut self.camera, &self.shared_state,
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
                Close => self.running_flag.store(false, Ordering::Relaxed),
                Fullscreen => self.toggle_full_screen(),
                ResetSimulation => {
                    let mut lock = self.shared_state.lock_unpoison();
                    lock.data_passive.reset();
                    lock.data_passive.forward();
                },
                AlignCamera => {
                    self.camera = MjvCamera::new_free(self.shared_state.lock_unpoison().data_passive.model());
                },
                VSyncToggle => {
                    self.update_vsync();
                },
                Screenshot { viewport_only, depth } => {
                    self.screenshot_pending = Some((viewport_only, depth));
                }
            }
        }
    }

    /// Reads the state of requested vsync setting and makes appropriate calls to [`glutin`].
    fn update_vsync(&mut self) {
        let RenderBaseGlState {
            gl_surface, gl_context, ..
        } = &self.adapter.state.as_ref().unwrap();

        if self.status.contains(ViewerStatusBit::VSYNC) {
            if let Err(e) = gl_surface.set_swap_interval(
                gl_context, glutin::surface::SwapInterval::Wait(NonZero::<u32>::MIN)
            ) {
                eprintln!("failed to enable vsync: {e}");
                self.status.set(ViewerStatusBit::VSYNC, false);
            }
        } else {
            if let Err(e) = gl_surface.set_swap_interval(
                gl_context, glutin::surface::SwapInterval::DontWait
            ) {
                eprintln!("failed to disable vsync: {e}");
                self.status.set(ViewerStatusBit::VSYNC, true);
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

    /// Processes user input events.
    fn process_events(&mut self) {
        self.event_loop.pump_app_events(Some(Duration::ZERO), &mut self.adapter);
        while let Some(window_event) = self.adapter.queue.pop_front() {
            #[cfg(feature = "viewer-ui")]
            {
                let window: &winit::window::Window = &self.adapter.state.as_ref().unwrap().window;
                self.ui.handle_events(window, &window_event);

                // if the UI has an active input focus, ignore all keyboard events
                if let WindowEvent::KeyboardInput { .. } = &window_event
                    && self.ui.focused()
                {
                    continue;
                }
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
                        _ => continue
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
                    self.running_flag.store(false, Ordering::Relaxed);
                }

                // Also set the viewer's state to pending exit if the window no longer exists.
                WindowEvent::CloseRequested => { self.running_flag.store(false, Ordering::Relaxed) }

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
                    let mut lock = self.shared_state.lock_unpoison();
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
                } => {
                    if self.modifiers.state().control_key() {  // Control + C is reserved for copy
                        continue;
                    }

                    self.toggle_opt_flag(MjtVisFlag::mjVIS_CAMERA);
                }

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
        let ncam = self.ncam;
        if ncam == 0 {  // No cameras, ignore.
            return;
        }

        self.camera.fix((self.camera.fixedcamid + direction).rem_euclid(ncam as i32) as usize);
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
        self.camera.move_(
            MjtMouse::mjMOUSE_ZOOM,
            self.shared_state.lock_unpoison().data_passive.model(),
            0.0, -0.05 * change, &self.scene
        );
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
        let height = window.inner_size().height as f64;

        let mut lock = self.shared_state.lock_unpoison();
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
            self.camera.move_(
                action,
                data_passive.model(),
                dx / height, dy / height, &self.scene
            );
        }
        else {  // When the perturbation is active, move apply the perturbation.
            pert.move_(data_passive, action, dx / height, dy / height, &self.scene);
        }
    }

    /// Processes left clicks and double left clicks.
    fn process_left_click(&mut self, state: ElementState) {
        let modifier_state = self.modifiers.state();
        let mut lock = self.shared_state.lock_unpoison();
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
                    pert.start(type_, data_passive, &self.scene);
                }

                /* Double click detection */
                if self.last_bnt_press_time.elapsed().as_millis() < DOUBLE_CLICK_WINDOW_MS {
                    let cp = self.raw_cursor_position;
                    let x = cp.0;
                    let y = self.rect_full.height as f64 - cp.1;

                    /* Obtain the selection */ 
                    let rect = &self.rect_full;
                    let sel = self.scene.find_selection(
                        data_passive, &self.opt,
                        rect.width as MjtNum / rect.height as MjtNum,
                        (x - rect.left as MjtNum) / rect.width as MjtNum,
                        (y - rect.bottom as MjtNum) / rect.height as MjtNum
                    );

                    /* Set tracking camera */
                    if modifier_state.alt_key() {
                        if let Some(body_id) = sel.body_id {
                            self.camera.lookat = sel.point;
                            if modifier_state.control_key() {
                                self.camera.track(body_id);
                            }
                        }
                    }
                    else {
                        /* Mark selection */
                        if let Some(body_id) = sel.body_id {
                            pert.select = body_id as i32;
                            pert.flexselect = sel.flex_id.map(|v| v as i32).unwrap_or(-1);
                            pert.skinselect = sel.skin_id.map(|v| v as i32).unwrap_or(-1);
                            pert.active = 0;
                            pert.update_local_pos(&sel.point, data_passive);
                        }
                        else {
                            pert.select = -1;
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

/// Releases OpenGL resources (rendering context and egui painter) while the
/// GL context is still current.
impl Drop for MjViewer {
    fn drop(&mut self) {
        // Ensure the GL context is current before the implicit field drops so that
        // MjrContext::drop (which calls mjr_freeContext) can properly free OpenGL resources.
        if let Some(ref state) = self.adapter.state {
            let _ = state.gl_context.make_current(&state.gl_surface);
        }

        // Release egui painter GL resources while the context is still current.
        #[cfg(feature = "viewer-ui")]
        self.ui.destroy_gl();
    }
}


/// Builder for [`MjViewer`].
/// ### Default settings:
/// - `window_name`: MuJoCo Rust Viewer (MuJoCo \<MuJoCo version here\>)
/// - `max_user_geoms`: 0
/// - `vsync`: false
/// - `warn_non_realtime`: false
/// 
#[derive(Debug, Clone)]
pub struct MjViewerBuilder {
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
}

impl MjViewerBuilder {
    builder_setters! {
        window_name: S where S: Into<Cow<'static, str>>; "text shown in the title of the window.";
        max_user_geoms: usize; "maximum number of geoms that can be drawn by the user in addition to the regular geoms.";
        vsync: bool; "enable vertical synchronization by default.";
        warn_non_realtime: bool; "enable showing an overlay when the simulation state isn't synced in realtime (deviation larger than 2 %).";
    }
}

impl MjViewerBuilder {
    /// Creates a [`MjViewerBuilder`] with default settings.
    pub fn new() -> Self {
        Self {
            window_name: Cow::Owned(format!("MuJoCo Rust Viewer (MuJoCo {})", mujoco_version())),
            max_user_geoms: 0, vsync: false, warn_non_realtime: false,
        }
    }

    /// Builds a [`MjViewer`] with the configured options.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the [`MjViewer`].
    /// # Errors
    /// - [`MjViewerError::EventLoopError`] if the event loop fails to initialize.
    /// - [`MjViewerError::GlInitFailed`] if OpenGL / window initialization fails.
    /// - [`MjViewerError::GlutinError`] if a glutin operation fails.
    /// - [`MjViewerError::PainterInitError`] if the UI painter fails to initialize
    ///   (feature `viewer-ui`).
    pub fn build_passive<M: Deref<Target = MjModel>>(&self, model: M) -> Result<MjViewer, MjViewerError> {
        let (w, h) = MJ_VIEWER_DEFAULT_SIZE_PX;
        let mut event_loop = EventLoop::new().map_err(MjViewerError::EventLoopError)?;
        let adapter = RenderBase::new(
            w, h,
            self.window_name.to_string(),
            &mut event_loop,
            true  // process events
        )?;

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
                glutin::surface::SwapInterval::Wait(NonZero::<u32>::MIN)
            ).map_err(MjViewerError::GlutinError)?;
        } else {
            gl_surface.set_swap_interval(gl_context, glutin::surface::SwapInterval::DontWait).map_err(
                MjViewerError::GlutinError
            )?;
        }

        event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);

        let ngeom = model.ffi().ngeom as usize;
        let scene = MjvScene::new(&*model, ngeom + self.max_user_geoms + EXTRA_SCENE_GEOM_SPACE);
        // SAFETY: The OpenGL context was made current above via gl_surface.
        let context = unsafe { MjrContext::new(&model) };
        let camera  = MjvCamera::new_free(&model);

        // Tracking of changes made between syncs
        let shared_state = Arc::new(Mutex::new(ViewerSharedState::new(&*model, self.max_user_geoms)));
        let running_flag = shared_state.lock_unpoison().running.clone();

        // User interface
        #[cfg(feature = "viewer-ui")]
        let ui = ui::ViewerUI::new(&*model, window, &gl_surface.display())?;
        #[cfg(feature = "viewer-ui")]
        let mut status = ViewerStatusBit::UI;
        #[cfg(not(feature = "viewer-ui"))]
        let mut status = ViewerStatusBit::HELP;

        status.set(ViewerStatusBit::VSYNC, self.vsync);
        status.set(ViewerStatusBit::WARN_REALTIME, self.warn_non_realtime);

        Ok(MjViewer {
            scene,
            context,
            camera,
            opt: MjvOption::default(),
            ncam: model.ffi().ncam,
            shared_state,
            running_flag,
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
            status,
            screenshot_pending: None
        })
    }
}

/// Delegates to [`MjViewerBuilder::new`].
impl Default for MjViewerBuilder {
    fn default() -> Self {
        MjViewerBuilder::new()
    }
}

bitflags! {
    /// Internal bit-flags that track the visibility state of various on-screen overlays.
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
