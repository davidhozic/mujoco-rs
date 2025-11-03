//! Implementation of the interface for use in the viewer.
use egui_winit::winit::event::WindowEvent;
use glutin::display::{Display, GlDisplay};
use egui_winit::winit::window::Window;
use egui_glow::glow::HasContext;
use egui::{FontId, RichText};
use egui_winit::egui;
use egui_winit;

use std::collections::VecDeque;
use std::ffi::CString;
use std::fmt::Debug;
use std::sync::Arc;

use crate::viewer::ViewerStatusBit;

const MAIN_FONT: FontId = FontId::proportional(15.0);
const HEADING_FONT: FontId = FontId::proportional(20.0);
const HEADING_POST_SPACE: f32 = 5.0;
const BUTTON_SPACING_X: f32 = 10.0;
const BUTTON_SPACING_Y: f32 = 5.0;
const BUTTON_ROUNDING: f32 = 50.0;

const SIDE_PANEL_DEFAULT_WIDTH: f32 = 250.0;



/// Viewer user interface context.
pub(crate) struct ViewerUI {
    egui_ctx: egui::Context,
    state: egui_winit::State,
    painter: egui_glow::Painter,
    gl: Arc<egui_glow::glow::Context>,
    events: VecDeque<UiEvent>,
}

impl ViewerUI {
    /// Create a new [`ViewerUI`] instance for the specific winit window.
    pub(crate) fn new(window: &Window, display: &Display) -> Self {
        let egui_ctx = egui::Context::default();
        let viewport_id = egui_ctx.viewport_id();

        let state = egui_winit::State::new(
            egui_ctx.clone(), viewport_id, &window,
            None, None, None
        );

        let get_addr = |s: &str| display.get_proc_address(
            &CString::new(s).unwrap()
        );
        let gl = unsafe { Arc::new(egui_glow::glow::Context::from_loader_function(get_addr)) };

        let painter = egui_glow::Painter::new(
            gl.clone(),
            "",
            None,
            false
        ).unwrap();
        Self { egui_ctx, state, painter, gl, events: VecDeque::new() }
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(&window, event);  // ignore response as it can be obtained later.
    }

    /// Draws the UI to the viewport.
    pub(crate) fn process(&mut self, window: &Window, status: &mut ViewerStatusBit) -> egui::Rect {
        let raw_input = self.state.take_egui_input(&window);
        let mut scene_rect_points = egui::Rect::NOTHING;
        let full_output = self.egui_ctx.run(raw_input, |ui| {
            egui::SidePanel::new(egui::panel::Side::Left, "left-panel")
                .resizable(true)
                .default_width(SIDE_PANEL_DEFAULT_WIDTH)
                .show(ui, |ui|
            {
                /* Basic control */
                ui.heading(RichText::new("Basic control")
                    .font(HEADING_FONT)
                    .underline()
                );

                ui.add_space(HEADING_POST_SPACE);

                // Make buttons have more space in the width
                let spacing = ui.spacing_mut();
                spacing.button_padding.x = BUTTON_SPACING_X;
                spacing.button_padding.y = BUTTON_SPACING_Y;

                // Viewer controls
                if ui.add(egui::Button::new(
                    RichText::new("Quit").font(MAIN_FONT)
                ).corner_radius(BUTTON_ROUNDING)).clicked() {
                    self.events.push_back(UiEvent::Close);
                }

                ui.add_space(HEADING_POST_SPACE);
                ui.separator();

                /* Option */
                ui.heading(RichText::new("Option")
                    .font(HEADING_FONT)
                    .underline()
                );
                ui.add_space(HEADING_POST_SPACE);

                ui.horizontal_wrapped(|ui| {
                    let mut selected = status.contains(ViewerStatusBit::HELP);
                    ui.toggle_value(&mut selected, RichText::new("Help").font(MAIN_FONT));
                    status.set(ViewerStatusBit::HELP, selected);

                    selected = window.fullscreen().is_some();
                    if ui.toggle_value(&mut selected, RichText::new("Fullscreen").font(MAIN_FONT)).clicked() {
                        self.events.push_back(UiEvent::Fullscreen);
                    };
                });

                ui.add_space(HEADING_POST_SPACE);
                ui.separator();

                /* Simulation  */
                ui.heading(RichText::new("Simulation")
                    .font(HEADING_FONT)
                    .underline()
                );
                ui.add_space(HEADING_POST_SPACE);
                ui.horizontal_wrapped(|ui| {
                    // Reset simulation
                    if ui.add(egui::Button::new(
                        RichText::new("Reset").font(MAIN_FONT)
                    ).corner_radius(BUTTON_ROUNDING)).clicked() {
                        self.events.push_back(UiEvent::ResetSimulation);
                    }

                    // Align camera
                    if ui.add(egui::Button::new(
                        RichText::new("Align").font(MAIN_FONT)
                    ).corner_radius(BUTTON_ROUNDING)).clicked() {
                        self.events.push_back(UiEvent::AlignCamera);
                    }
                });


                // Make the panel track its width on resize
                ui.take_available_space();
            });

            egui::SidePanel::new(egui::panel::Side::Left, "left-panel-pad")
                .resizable(false)
                .default_width(10.0)
                .show_separator_line(false)
                .show(ui, |ui|
            {
                scene_rect_points = ui.max_rect();
            });
        });

        self.state.handle_platform_output(&window, full_output.platform_output);

        // Tesselate
        let pixels_per_point = full_output.pixels_per_point;
        let textures_delta = &full_output.textures_delta;
        let clipped_primitives = self.egui_ctx.tessellate(full_output.shapes, pixels_per_point);

        // Paint the menu
        self.painter.paint_and_update_textures(
            window.inner_size().into(),
            pixels_per_point,
            &clipped_primitives,
            textures_delta
        );
        scene_rect_points
    }

    /// Resets OpenGL state. This is needed for MuJoCo's renderer.
    pub(crate) fn reset(&mut self) {
        let gl = &self.gl;
        unsafe {
            gl.use_program(None);
        }
    }

    /// Drains events from queue. If no event is queued, [`None`] is returned.
    pub(crate) fn drain_events(&mut self) -> Option<UiEvent> {
        self.events.pop_front()
    }
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl Debug for ViewerUI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}

pub(crate) enum UiEvent {
    Close,
    Fullscreen,
    ResetSimulation,
    AlignCamera
}
