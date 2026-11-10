//! A submodule of the viewer's UI representing the exit modal.

use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

use egui::{Color32, Id, RichText};

use crate::viewer::ui::MAIN_FONT;

/// The size of the buttons of the exit modal.
const BUTTON_SIZE: egui::Vec2 = egui::Vec2 { x: 70.0, y: 30.0 };

/// The inner margin between modal's edges and the actual content.
const MODAL_INNER_MARGIN: f32 = 50.0;

/// Shows the exit confirmation modal, asking the user to confirm the exit.
/// The purpose of this is to prevent the user from trying to close the within-viewer windows
/// and thus accidentally closing the window :) (I know, this is my skill issue).
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, viewer_running: &Arc<AtomicBool>
)
{
    if !*open {
        return;
    }

    let modal = egui::Modal::new(Id::new("mujoco_rs_exit_modal"))
        .show(ui, |ui|
    {
        egui::Frame::window(ui.style())
            .inner_margin(MODAL_INNER_MARGIN)
            .show(ui, |ui|
        {
            ui.horizontal_centered(|ui| {
                ui.heading("Are you sure you want to exit?");
                ui.separator();
                // Confirm exit
                if ui.add(
                    egui::Button::new(RichText::new("Ok").font(MAIN_FONT).color(Color32::BLACK))
                    .fill(Color32::from_rgb(51, 153, 204))
                    .min_size(BUTTON_SIZE)
                ).clicked() {
                    viewer_running.store(false, Ordering::Relaxed);  // will exit when possible.
                    ui.close();  // modal can also be closed by ESC or back-click.
                }

                // Cancel exit and return back to the viewer
                if ui.add(
                    egui::Button::new(RichText::new("Cancel").font(MAIN_FONT))
                    .min_size(BUTTON_SIZE)
                ).clicked() {
                    ui.close();
                }
        })});
    });

    if modal.should_close() {
        *open = false;
    }
}
