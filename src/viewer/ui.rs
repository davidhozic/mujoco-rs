//! Implementation of the interface for use in the viewer.
mod widget_extras;
mod windows;
mod panel;
mod modals;

use std::collections::VecDeque;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::fmt::Debug;

use egui::{FontId, Id, LayerId, Order, RichText};
use egui_winit::winit::event::WindowEvent;
use egui_glow::glow::{self, HasContext};
use egui_winit::winit::window::Window;
use egui_winit::egui;

use crate::wrappers::mj_visualization::{MjvOption, MjvCamera, MjvScene};
use crate::viewer::{ViewerSharedState, ViewerStatusBit, MjViewerError};
use crate::wrappers::mj_model::{MjModel, MjtObjBase};

use windows::recorder::state::StateRecorderData;
use windows::sensor::SensorDisplayInfo;
use windows::state::StateDisplayInfo;
use windows::ElementDisplayInfo;
use widget_extras::SearchBar;

/* UI Fonts */
const MAIN_FONT: FontId = FontId::proportional(15.0);
const SECONDARY_FONT: FontId = FontId::proportional(13.0);
const HEADING_FONT: FontId = FontId::proportional(20.0);

/* UI Spacing and Dimensions */
const HEADING_POST_SPACE: f32 = 5.0;
const BUTTON_SPACING_X: f32 = 10.0;
const BUTTON_SPACING_Y: f32 = 5.0;
const BUTTON_ROUNDING: f32 = 50.0;
const TOGGLE_LABEL_HEIGHT_EXTRA_SPACE: f32 = 20.0;
const SIDE_PANEL_PAD: f32 = 10.0;
const MAX_SPAN_WIDTH: f32 = 350.0;

/// Type alias for a user-provided UI callback function.
pub(crate) type UiCallback = Box<dyn FnMut(&mut egui::Ui, &Arc<Mutex<ViewerSharedState>>)>;

/// Processed egui data to be painted.
struct PendingPaint {
    primitives: Vec<egui::ClippedPrimitive>,
    textures_delta: egui::TexturesDelta,
    pixels_per_point: f32,
    screen_size: [u32; 2],
}

/// Viewer user interface context.
pub(crate) struct ViewerUI {
    /* Egui-related */
    egui_ctx: egui::Context,
    state: egui_winit::State,
    painter: egui_glow::Painter,
    gl: Arc<egui_glow::glow::Context>,
    pending_paint: Option<PendingPaint>,

    /* Window toggles */
    // Note that these are bool for easier integration with egui.
    // We assumed they won't take up too much extra space, since
    // running multiple viewers isn't what most people will do
    // and even then this is minor.
    actuator_window: bool,
    joint_window: bool,
    equality_window: bool,
    group_window: bool,
    screenshot_viewport_only: bool,
    screenshot_depth: bool,

    // Camera tracking modal state
    show_tracking_modal: bool,
    tracking_selected_body: Option<usize>,

    // Exiting modal confirmation
    show_exit_modal: bool,

    /* Window's search-bar states */
    actuator_search_bar: SearchBar,
    joint_search_bar: SearchBar,
    equality_search_bar: SearchBar,

    /* Model-related */
    camera_names: Vec<String>,
    actuator_info: Vec<ElementDisplayInfo>,
    joint_info: Vec<ElementDisplayInfo>,
    equality_names: Vec<String>,

    /* Data-related */
    sensor_info: SensorDisplayInfo,
    state_info: StateDisplayInfo,

    /* Recorder-related */
    state_recorder_data: StateRecorderData,

    /* User-related */
    user_ui_callbacks: Vec<UiCallback>,

    /* Event handling */
    events: VecDeque<UiEvent>,
}

impl ViewerUI {
    /// Create a new [`ViewerUI`] instance for the specific winit window.
    pub(crate) fn new(model: &MjModel, window: &Window, gl: &Arc<glow::Context>) -> Result<Self, MjViewerError> {
        let egui_ctx = egui::Context::default();
        let viewport_id = egui_ctx.viewport_id();

        let state = egui_winit::State::new(
            egui_ctx.clone(), viewport_id, &window,
            None, None, None
        );

        let painter = egui_glow::Painter::new(
            gl.clone(),
            "",
            None,
            false
        ).map_err(|e| MjViewerError::PainterInitError(e.to_string()))?;

        let mut viewer_ui = Self {
            egui_ctx, state, painter, gl: gl.clone(), events: VecDeque::new(),
            pending_paint: None,
            camera_names: Vec::new(),
            actuator_info: Vec::new(),
            joint_info: Vec::new(),
            sensor_info: SensorDisplayInfo::new(),
            state_info: StateDisplayInfo::new(),
            state_recorder_data: StateRecorderData::default(),
            equality_names: Vec::new(),
            user_ui_callbacks: Vec::new(),
            actuator_window: false,
            joint_window: false,
            equality_window: false,
            group_window: false,
            screenshot_viewport_only: false,
            screenshot_depth: false,
            show_tracking_modal: false,
            tracking_selected_body: None,
            show_exit_modal: false,
            actuator_search_bar: SearchBar::new("Enter actuator's name"),
            joint_search_bar: SearchBar::new("Enter joint's name"),
            equality_search_bar: SearchBar::new("Enter equality's name"),
        };
        viewer_ui.reset_to_model(model);
        Ok(viewer_ui)
    }

    /// Rebuilds all model-dependent cached state.
    /// Must be called whenever the active model changes.
    pub(crate) fn reset_to_model(&mut self, model: &MjModel) {
        /* Model caches */
        self.camera_names = (0..model.ncam()).map(|i| {
            if let Some(name) = model.id_to_name(MjtObjBase::mjOBJ_CAMERA, i as usize) {
                name.to_string()
            } else { format!("Camera {i}") }
        }).collect();

        self.actuator_info = windows::actuator::display_infos(model).collect();
        self.joint_info = windows::joint::display_infos(model).collect();
        self.equality_names = windows::equality::names(model).collect();
        self.sensor_info.reset_to_model(model);
        self.state_info.reset_to_model(model);

        /* Camera tracking state */
        // The pending selection is a body ID of the previous model, which addresses a different
        // body in the new one.
        self.tracking_selected_body = None;
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(window, event);  // ignore response as it can be obtained later.
    }

    /// Gains scoped access to [`egui::Context`] for dealing with custom initialization
    /// (e.g., loading in images).
    pub(crate) fn with_egui_ctx<F>(&mut self, once_fn: F)
        where F: FnOnce(&egui::Context)
    {
        once_fn(&mut self.egui_ctx)
    }

    /// Opens the exit confirmation modal. Returns `true` if the modal was already open.
    pub(crate) fn request_close(&mut self) -> bool {
        std::mem::replace(&mut self.show_exit_modal, true)
    }

    /// Draws the UI to the viewport. Returns the width in physical pixels that the left side
    /// panel reserves, or `0.0` when the panel is hidden.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn process(
        &mut self,
        window: &Window, status: &mut ViewerStatusBit,
        scene: &mut MjvScene, options: &mut MjvOption,
        camera: &mut MjvCamera,
        shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
        viewer_running: &Arc<AtomicBool>
    ) -> f32 {
        // Viewport reservations, which will be excluded from MuJoCo's viewport.
        // This way MuJoCo won't draw over the UI.
        let mut left = 0.0;

        // Process the UI
        let raw_input = self.state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(raw_input, |ui| {
            let mut is_expanded = status.contains(ViewerStatusBit::UI);
            egui::Panel::left("interface_panel")
                .resizable(true)
                .show_collapsible(ui, &mut is_expanded, |ui|
            {
                // The menu
                egui::ScrollArea::vertical()
                    .max_height(
                        ui.available_height()
                        - (TOGGLE_LABEL_HEIGHT_EXTRA_SPACE + HEADING_POST_SPACE + HEADING_FONT.size)
                    )
                    .scroll_source(egui::scroll_area::ScrollSource::ALL)
                    .show(ui, |ui|
                {
                    // Make buttons have more space in the width
                    let spacing = ui.spacing_mut();
                    spacing.button_padding.x = BUTTON_SPACING_X;
                    spacing.button_padding.y = BUTTON_SPACING_Y;

                    /* Window controls */
                    egui::CollapsingHeader::new(RichText::new("Basic").font(HEADING_FONT))
                        .default_open(true)
                        .show(ui, |ui|
                    {
                        ui.horizontal_wrapped(|ui| {
                            if ui.add(egui::Button::new(
                                RichText::new("Quit").font(MAIN_FONT)
                            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                self.events.push_back(UiEvent::Close);
                            }
                        });
                    });

                    /* UI toggles */
                    panel::ui::show(
                        ui, status, window, &mut self.events, &mut self.actuator_window,
                        &mut self.joint_window, &mut self.equality_window, &mut self.group_window,
                        &mut self.sensor_info, &mut self.state_info,
                    );

                    /* Simulation */
                    panel::simulation::show(ui, &mut self.events);

                    /* Physics options */
                    panel::physics::show(ui, shared_viewer_state);

                    /* Visualization options */
                    panel::rendering::show(
                        ui, &mut self.events, &self.camera_names, &mut self.show_tracking_modal,
                        &mut self.tracking_selected_body, &mut self.screenshot_viewport_only,
                        &mut self.screenshot_depth, scene, options, camera, shared_viewer_state,
                    );

                    /* Visualization */
                    panel::visualization::show(ui, &mut self.events, shared_viewer_state);

                    /* Recorder */
                    panel::recorder::show(ui, &mut self.state_recorder_data);

                    // Make the scroll bars span to the edges of the sidebar.
                    ui.take_available_space();
                });

                // Panel toggle info
                ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                    ui.add_space(HEADING_POST_SPACE);
                    ui.heading("Toggle: X");
                });

                // Make the panel track its width on resize
                left = ui.max_rect().max.x + SIDE_PANEL_PAD;
            });
            status.set(ViewerStatusBit::UI, is_expanded);

            /* Windows */
            // Controls window
            windows::actuator::show(
                ui, &mut self.actuator_window, &self.actuator_info, &mut self.actuator_search_bar,
                shared_viewer_state
            );

            // Joints window
            windows::joint::show(
                ui, &mut self.joint_window, &self.joint_info, &mut self.joint_search_bar,
                shared_viewer_state
            );

            // Sensors window
            windows::sensor::show(ui, &mut self.sensor_info, shared_viewer_state);

            // Equalities window
            windows::equality::show(
                ui, &mut self.equality_window, &self.equality_names, &mut self.equality_search_bar,
                shared_viewer_state
            );

            // Groups window
            windows::group::show(ui, &mut self.group_window, options);

            // States window
            windows::state::show(ui, &mut self.state_info, shared_viewer_state);

            // State recorder window
            windows::recorder::state::show(ui, &mut self.state_recorder_data);

            /* Modals */
            // Camera tracking modal
            modals::camera_tracking::show(
                ui, &mut self.show_tracking_modal, &mut self.tracking_selected_body,
                shared_viewer_state,
            );

            // Exit confirmation
            modals::exit::show(ui, &mut self.show_exit_modal, viewer_running);

            /* User-defined UI callbacks */
            // Callbacks that receive the egui context and MjData passive instance
            for callback in self.user_ui_callbacks.iter_mut() {
                callback(ui, shared_viewer_state);
            }
        });

        // Apply egui's platform output (cursor icon, clipboard, IME).
        self.state.handle_platform_output(window, full_output.platform_output);

        // Tessellate and store result for later drawing.
        let pixels_per_point = full_output.pixels_per_point;
        let mut textures_delta = full_output.textures_delta;

        // A render attempt that ended early leaves its own deltas uncleared. They carry
        // upload and free history that egui expects.
        if let Some(dropped) = self.pending_paint.take() {
            let mut merged = dropped.textures_delta;
            merged.append(textures_delta);
            textures_delta = merged;
        }

        self.pending_paint = Some(PendingPaint {
            primitives: self.egui_ctx.tessellate(full_output.shapes, pixels_per_point),
            textures_delta,
            pixels_per_point,
            screen_size: window.inner_size().into(),
        });

        left * pixels_per_point
    }

    /// Paints the interface.
    pub(crate) fn paint(&mut self) {
        let Some(mut pending) = self.pending_paint.take() else {
            return;
        };

        self.init_2d();
        self.painter.paint_and_update_textures(
            pending.screen_size,
            pending.pixels_per_point,
            &pending.primitives,
            &mut pending.textures_delta
        );
    }

    /// Checks whether the UI is focused (e.g., typing).
    pub(crate) fn is_focused(&self) -> bool {
        self.egui_ctx.memory(|ui| ui.focused().is_some())
    }

    /// Checks whether the mouse is over the UI.
    pub(crate) fn is_covered(&self) -> bool {
        self.egui_ctx.is_pointer_over_egui()
    }

    /// Checks whether the mouse is over one of UI's windows (side panels do not count).
    pub(crate) fn is_window_covered(&self) -> bool {
        let Some(position) = self.egui_ctx.pointer_interact_pos() else {
            return false;
        };

        // The side panels share the background layer with the scene; windows and popups do not.
        self.egui_ctx.layer_id_at(position).is_some_and(|layer| layer.order != Order::Background)
    }

    /// Checks whether the UI is currently being dragged.
    pub(crate) fn is_dragged(&self) -> bool {
        self.egui_ctx.dragged_id().is_some()
    }

    /// Takes the selection away from every UI window.
    pub(crate) fn deselect_windows(&self) {
        let scene_layer = LayerId::new(Order::Middle, Id::new("mujoco_scene"));
        self.egui_ctx.memory_mut(|memory| memory.areas_mut().move_to_top(scene_layer));
    }

    /// Prepares OpenGL for the egui paint pass.
    fn init_2d(&self) {
        let gl = &self.gl;
        // `mjr_render` leaves the polygon mode on GL_LINE when the wireframe flag is set, and
        // egui sets every other state it needs in its own `prepare_painting`.
        // SAFETY: the GL context is current (made current by the render loop before this call).
        unsafe {
            gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
        }
    }

    /// Drains events from queue. If no event is queued, [`None`] is returned.
    pub(crate) fn drain_events(&mut self) -> Option<UiEvent> {
        self.events.pop_front()
    }

    /// Adds a user-defined UI callback that will be invoked during UI rendering.
    /// The callback receives the root [`egui::Ui`] and the viewer's shared state ([`ViewerSharedState`]),
    /// and can be used to create custom windows, panels, or other UI elements.
    pub(crate) fn add_ui_callback<F>(&mut self, callback: F)
    where
        F: FnMut(&mut egui::Ui, &Arc<Mutex<ViewerSharedState>>) + 'static
    {
        self.user_ui_callbacks.push(Box::new(callback));
    }

    /// Release OpenGL resources held by the egui painter.
    /// Must be called while the GL context is still current.
    pub(crate) fn destroy_gl(&mut self) {
        self.painter.destroy();
    }
}

impl Drop for ViewerUI {
    fn drop(&mut self) {
        // A render attempt that ended in between calls leaves texture deltas behind.
        // They must be cleared due to egui panicking otherwise on its own drop.
        if let Some(pending) = &mut self.pending_paint {
            pending.textures_delta.clear();
        }
    }
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl Debug for ViewerUI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}

/// Enum representing UI events raised from the interface.
pub(crate) enum UiEvent {
    Close,
    Fullscreen,
    ResetSimulation,
    AlignCamera,
    VSyncToggle,
    Screenshot { viewport_only: bool, depth: bool }
}
