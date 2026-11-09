//! A submodule of the viewer's UI representing the state recorder window definitions.
use egui_winit::egui;

/// The UI-related state and data of the state recorder window.
#[derive(Debug, Default)]
pub struct StateRecorderData {
    /// Whether the window is open.
    pub open: bool,
}

/// Shows the state recorder window when `state_recorder_data` marks it as open.
pub fn show(ui: &mut egui::Ui, state_recorder_data: &mut StateRecorderData) {
    egui::Window::new("State recorder")
        .id(egui::Id::new("mujoco_rs_state_recorder_window"))
        .open(&mut state_recorder_data.open)
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .show(ui, |_ui| {});
}
