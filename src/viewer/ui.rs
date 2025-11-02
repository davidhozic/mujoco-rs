//! Implementation of the interface for use in the viewer.
use egui_winit::winit::event::WindowEvent;
use egui_winit::winit::window::Window;
use egui_winit::egui;
use egui_winit;

use std::fmt::Debug;


/// Viewer user interface context.
pub(crate) struct ViewerUI {
    egui_ctx: egui::Context,
    state: egui_winit::State
}

impl ViewerUI {
    /// Create a new [`ViewerUI`] instance for the specific winit window.
    pub(crate) fn new(window: &Window) -> Self {
        let egui_ctx = egui::Context::default();
        let viewport_id = egui_ctx.viewport_id();

        let state = egui_winit::State::new(
            egui_ctx.clone(), viewport_id, &window,
            None, None, None
        );
        Self { egui_ctx: egui_ctx, state: state }
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(&window, event);  // ignore response as it can be obtained later.
    }

    /// Draws the UI to the viewport.
    pub(crate) fn draw(&mut self, window: &Window) {
        let raw_input = self.state.take_egui_input(&window);
        let full_output = self.egui_ctx.run(raw_input, |ui| {
            egui::SidePanel::new(egui::panel::Side::Left, "left-panel")
                .resizable(true)
                .default_width(200.0)
                .show(ui, |ui|
            {
                ui.heading("Left");
            });
        });
        self.state.handle_platform_output(&window, full_output.platform_output);

        // Tesselate
        let pixels_per_point = full_output.pixels_per_point;
        let jobs = self.egui_ctx.tessellate(full_output.shapes, pixels_per_point);

        // Paint the menu

    }
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl Debug for ViewerUI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}
