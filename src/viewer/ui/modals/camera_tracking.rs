//! A submodule of the viewer's UI representing the camera tracking modal definitions.
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::{Id, RichText};

use crate::wrappers::mj_model::MjtObjBase;
use crate::viewer::ViewerSharedState;
use crate::viewer::ui::MAIN_FONT;
use crate::util::LockUnpoison;

/// Maximum height of the body list in the camera tracking modal, in points.
const CAMERA_MODAL_MAX_HEIGHT: f32 = 300.0;
/// Number of body buttons in one row of the camera tracking modal.
const CAMERA_MODAL_BUTTONS_PER_ROW: usize = 5;
/// Width of a body button in the camera tracking modal, in points.
const CAMERA_MODAL_BUTTON_WIDTH: f32 = 100.0;
/// Height of a body button in the camera tracking modal, in points.
const CAMERA_MODAL_BUTTON_HEIGHT: f32 = 35.0;
/// Size of one grid cell (body button) in the camera tracking modal, in points.
const CAMERA_MODAL_CELL_SIZE: egui::Vec2 = egui::Vec2 { x: CAMERA_MODAL_BUTTON_WIDTH, y: CAMERA_MODAL_BUTTON_HEIGHT };
/// Corner radius of a body button in the camera tracking modal, in points.
const CAMERA_MODAL_BUTTON_ROUNDING: f32 = 4.0;
/// Gray level of a body button that the pointer is not over.
const CAMERA_MODAL_BUTTON_COLOR_DEFAULT: u8 = 40;
/// Gray level of a body button that the pointer is over.
const CAMERA_MODAL_BUTTON_COLOR_HOVERED: u8 = 60;

/// Shows the body selection modal for camera tracking when `open` is set. A click on a body
/// writes its ID to `selected_body`.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, selected_body: &mut Option<usize>,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    if !*open {
        return;
    }

    let modal = egui::Modal::new(Id::new("mujoco_rs_select_body_tracking"))
        .show(ui, |ui|
    {
        ui.heading("Select the body to track");
        ui.separator();

        egui::ScrollArea::vertical()
            .max_height(CAMERA_MODAL_MAX_HEIGHT)
            .show(ui, |ui| {
                let lock = shared_viewer_state.lock_unpoison();
                if lock.model_reloaded {
                    return;
                }
                let model = lock.data_passive.model();
                let nbody = model.nbody();

                egui::Grid::new("body_grid")
                    .num_columns(CAMERA_MODAL_BUTTONS_PER_ROW)
                    .show(ui, |ui| {
                        for body_id in 0..nbody as usize {
                            let body_name = if let Some(name) = model.id_to_name(
                                MjtObjBase::mjOBJ_BODY, body_id
                            ) {
                                name.to_string()
                            } else {
                                format!("Body {}", body_id)
                            };

                            let (rect, response) = ui.allocate_exact_size(
                                CAMERA_MODAL_CELL_SIZE, egui::Sense::click()
                            );
                            if response.clicked() {
                                *selected_body = Some(body_id);
                                ui.close();
                            }

                            let button_color = if response.hovered() {
                                egui::Color32::from_gray(CAMERA_MODAL_BUTTON_COLOR_HOVERED)
                            } else {
                                egui::Color32::from_gray(CAMERA_MODAL_BUTTON_COLOR_DEFAULT)
                            };
                            ui.painter().rect_filled(rect, CAMERA_MODAL_BUTTON_ROUNDING, button_color);

                            ui.painter().with_clip_rect(rect).text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                &body_name,
                                MAIN_FONT,
                                egui::Color32::WHITE,
                            );

                            if response.hovered() {
                                response.on_hover_text(&body_name);
                            }

                            if (body_id + 1) % CAMERA_MODAL_BUTTONS_PER_ROW == 0 {
                                ui.end_row();
                            }
                        }
                    });
            });

        ui.separator();
        if ui.button(RichText::new("Cancel").font(MAIN_FONT)).clicked() {
            ui.close();
        }
    });

    if modal.should_close() {
        *open = false;
    }
}
