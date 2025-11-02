//! Implementation of the interface for use in the viewer.
use egui_winit::winit::event::WindowEvent;
use glutin::display::{Display, GlDisplay};
use egui_winit::winit::window::Window;
use egui_glow::glow::HasContext;
use egui_winit::egui;
use egui_winit;

use std::ffi::CString;
use std::fmt::Debug;
use std::sync::Arc;


/// Viewer user interface context.
pub(crate) struct ViewerUI {
    egui_ctx: egui::Context,
    state: egui_winit::State,
    painter: egui_glow::Painter,
    gl: Arc<egui_glow::glow::Context>,
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
        Self { egui_ctx, state, painter, gl }
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(&window, event);  // ignore response as it can be obtained later.
    }

    /// Draws the UI to the viewport.
    pub(crate) fn draw(&mut self, window: &Window) -> egui::Rect {
        let raw_input = self.state.take_egui_input(&window);
        let mut scene_rect_points = egui::Rect::NOTHING;
        let full_output = self.egui_ctx.run(raw_input, |ui| {
            egui::SidePanel::new(egui::panel::Side::Left, "left-panel")
                .resizable(true)
                .default_width(200.0)
                .show(ui, |ui|
            {
                ui.heading("Basic control");
                if ui.button("Hello World").clicked() {
                    println!("Click");
                }
                ui.take_available_space();
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
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl Debug for ViewerUI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}
