//! A submodule of the viewer's UI representing the recorder side panel section definitions.
use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::windows::recorder::state::StateRecorderData;
use crate::viewer::ui::{HEADING_FONT, MAIN_FONT};

/// Shows the recorder section of the side panel.
pub fn show(ui: &mut egui::Ui, state_recorder_data: &mut StateRecorderData) {
    egui::CollapsingHeader::new(RichText::new("Recorder").font(HEADING_FONT))
        .show(ui, |ui|
    {
        /* Window toggles */
        ui.horizontal_wrapped(|ui| {
            ui.toggle_value(&mut state_recorder_data.open, RichText::new("State").font(MAIN_FONT));
        });
    });
}
