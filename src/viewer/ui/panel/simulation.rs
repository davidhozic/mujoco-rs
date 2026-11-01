//! A submodule of the viewer's UI representing the simulation side panel section definitions.
use std::collections::VecDeque;

use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::{UiEvent, HEADING_FONT, MAIN_FONT, BUTTON_ROUNDING};

pub(crate) fn show(ui: &mut egui::Ui, events: &mut VecDeque<UiEvent>) {
    egui::CollapsingHeader::new(RichText::new("Simulation").font(HEADING_FONT))
        .default_open(true)
        .show(ui, |ui|
    {
        ui.horizontal_wrapped(|ui| {
            // Reset simulation
            if ui.add(egui::Button::new(
                RichText::new("Reset").font(MAIN_FONT)
            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                events.push_back(UiEvent::ResetSimulation);
            }

            // Align camera
            if ui.add(egui::Button::new(
                RichText::new("Align").font(MAIN_FONT)
            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                events.push_back(UiEvent::AlignCamera);
            }
        });
    });
}
