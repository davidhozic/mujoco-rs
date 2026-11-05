//! A submodule of the viewer's UI representing the ui side panel section definitions.
use std::collections::VecDeque;

use egui_winit::winit::window::Window;
use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::{UiEvent, HEADING_FONT, MAIN_FONT};
use crate::viewer::ViewerStatusBit;

/// Shows the UI section of the side panel.
#[expect(
    clippy::too_many_arguments,
    reason = "each parameter is a separately borrowed field of ViewerUI"
)]
pub fn show(
    ui: &mut egui::Ui, status: &mut ViewerStatusBit, window: &Window,
    events: &mut VecDeque<UiEvent>, actuator_window: &mut bool, joint_window: &mut bool,
    equality_window: &mut bool, group_window: &mut bool, sensor_window: &mut bool,
    state_window: &mut bool,
)
{
    egui::CollapsingHeader::new(RichText::new("UI").font(HEADING_FONT))
        .default_open(true)
        .show(ui, |ui|
    {
        // Normal toggles
        ui.horizontal_wrapped(|ui| {
            let mut selected = status.contains(ViewerStatusBit::HELP);
            ui.toggle_value(&mut selected, RichText::new("Help").font(MAIN_FONT));
            status.set(ViewerStatusBit::HELP, selected);

            selected = window.fullscreen().is_some();
            if ui.toggle_value(&mut selected, RichText::new("Fullscreen").font(MAIN_FONT)).clicked() {
                events.push_back(UiEvent::Fullscreen);
            };

            // VSync
            let mut selected = status.contains(ViewerStatusBit::VSYNC);
            if ui.toggle_value(&mut selected, RichText::new("V-Sync").font(MAIN_FONT)).clicked() {
                events.push_back(UiEvent::VSyncToggle);
            };
            status.set(ViewerStatusBit::VSYNC, selected);

            // Info menu (FPS, time, etc.)
            let mut selected = status.contains(ViewerStatusBit::INFO);
            ui.toggle_value(&mut selected, RichText::new("Info").font(MAIN_FONT));
            status.set(ViewerStatusBit::INFO, selected);
        });

        ui.separator();

        /* Window toggles */
        ui.horizontal_wrapped(|ui| {
            ui.toggle_value(actuator_window, RichText::new("Actuator").font(MAIN_FONT));
            ui.toggle_value(joint_window, RichText::new("Joint").font(MAIN_FONT));
            ui.toggle_value(equality_window, RichText::new("Equality").font(MAIN_FONT));
            ui.toggle_value(group_window, RichText::new("Group").font(MAIN_FONT));
            ui.toggle_value(sensor_window, RichText::new("Sensor").font(MAIN_FONT));
            ui.toggle_value(state_window, RichText::new("State").font(MAIN_FONT));
        });

        ui.separator();

        ui.horizontal_wrapped(|ui| {
            // Warnings
            ui.collapsing(RichText::new("Warnings").font(MAIN_FONT), |ui| {
                // Non-realtime factor warning
                let mut selected = status.contains(ViewerStatusBit::WARN_REALTIME);
                ui.checkbox(&mut selected, RichText::new("Realtime factor").font(MAIN_FONT));
                status.set(ViewerStatusBit::WARN_REALTIME, selected);
            });
        });
    });
}
