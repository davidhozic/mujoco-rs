//! A submodule of the viewer's UI representing the visualization side panel section definitions.
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::widget_extras::{
    name_scalar_row, name_array_row, name_rgba_row, name_rgb_row
};
use crate::viewer::ui::{UiEvent, HEADING_FONT, MAIN_FONT, MAX_SPAN_WIDTH};
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::MODEL_SYNC_MESSAGE_TIMEOUT;

/* Precision and Tolerances */
const RANGE_PRECISION_TOLERANCE: f32 = 1e-4;

const MODEL_VIS_SYNC_WARNING: &str = "[WARNING] Visualization options not synced!";
const MODEL_STAT_SYNC_WARNING: &str = "[WARNING] Statistics not synced!";

pub(crate) fn show(
    ui: &mut egui::Ui, events: &mut VecDeque<UiEvent>,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    egui::CollapsingHeader::new(RichText::new("Visualization").font(HEADING_FONT))
        .show(ui, |ui|
    {
        let (stat_is_editable, vis_is_editable, mut vis, mut stat) = {
            let state = shared_viewer_state.lock_unpoison();
            (
                state.last_stat_sync_time().elapsed() < MODEL_SYNC_MESSAGE_TIMEOUT,
                state.last_vis_sync_time().elapsed() < MODEL_SYNC_MESSAGE_TIMEOUT,
                state.data_passive.model_vis().clone(),
                state.data_passive.model_stat().clone(),
            )
        };

        if !vis_is_editable {
            ui.colored_label(egui::Color32::YELLOW, MODEL_VIS_SYNC_WARNING);
        }
        if !stat_is_editable {
            ui.colored_label(egui::Color32::YELLOW, MODEL_STAT_SYNC_WARNING);
        }

        // Headlight
        ui.collapsing(RichText::new("Headlight").font(MAIN_FONT), |ui| {
            egui::Grid::new("headlight_grid").num_columns(2).show(ui, |ui| {
                ui.label("Active");
                ui.horizontal_top(|ui| {
                    let mut active_bool = vis.headlight.active != 0;
                    if ui.checkbox(&mut active_bool, "").changed() {
                        vis.headlight.active = if active_bool { 1 } else { 0 };
                    }
                });
                ui.end_row();

                name_rgb_row(ui, "Ambient", &mut vis.headlight.ambient);
                name_rgb_row(ui, "Diffuse", &mut vis.headlight.diffuse);
                name_rgb_row(ui, "Specular", &mut vis.headlight.specular);
            });
        });

        // Free Camera
        ui.collapsing(RichText::new("Free Camera").font(MAIN_FONT), |ui| {
            egui::Grid::new("camera_grid").num_columns(2).show(ui, |ui| {
                ui.label("Orthographic");
                ui.horizontal_top(|ui| {
                    let mut ortho_bool = vis.global.orthographic != 0;
                    if ui.checkbox(&mut ortho_bool, "").changed() {
                        vis.global.orthographic = if ortho_bool { 1 } else { 0 };
                    }
                });
                ui.end_row();

                name_scalar_row(ui, "Field of view", &mut vis.global.fovy, 1e-2, 5.0..=175.0);
                name_array_row(ui, "Center", &mut stat.center, 1e-3, None);
                name_scalar_row(
                    ui, "Azimuth", &mut vis.global.azimuth, 1e-2,
                    f32::NEG_INFINITY..=f32::INFINITY
                );
                name_scalar_row(ui, "Elevation (deg)", &mut vis.global.elevation, 1e-1, -89.0..=89.0);

                if ui.button("Align").clicked() {
                    events.push_back(UiEvent::AlignCamera);
                }
                ui.end_row();
            });
        });

        // Global
        ui.collapsing(RichText::new("Global").font(MAIN_FONT), |ui| {
            egui::Grid::new("global_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Extent", &mut stat.extent, 1e-3, 0.001..=f64::INFINITY);

                ui.label("Inertia Geom");
                ui.horizontal_top(|ui| {
                    let combo_width = ui.available_width().min(MAX_SPAN_WIDTH);
                    let mut inertia_choice = if vis.global.ellipsoidinertia == 0 { "Box" } else { "Ellipsoid" };
                    egui::ComboBox::from_id_salt("inertia_geom_combo")
                        .selected_text(inertia_choice)
                        .width(combo_width)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut inertia_choice, "Box", "Box");
                            ui.selectable_value(&mut inertia_choice, "Ellipsoid", "Ellipsoid");
                        });
                    vis.global.ellipsoidinertia = if inertia_choice == "Box" { 0 } else { 1 };
                });
                ui.end_row();

                ui.label("BVH active");
                ui.horizontal_top(|ui| {
                    let mut bvh_bool = vis.global.bvactive != 0;
                    if ui.checkbox(&mut bvh_bool, "").changed() {
                        vis.global.bvactive = if bvh_bool { 1 } else { 0 };
                    }
                });
                ui.end_row();
            });
        });

        // Map
        ui.collapsing(RichText::new("Map").font(MAIN_FONT), |ui| {
            egui::Grid::new("map_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Stiffness", &mut vis.map.stiffness, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Rot stiffness", &mut vis.map.stiffnessrot, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Force", &mut vis.map.force, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Torque", &mut vis.map.torque, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Alpha", &mut vis.map.alpha, 1e-3, 0.0..=1.0);
                name_scalar_row(
                    ui, "Fog start", &mut vis.map.fogstart, 1e-3,
                    0.0..=(vis.map.fogend - RANGE_PRECISION_TOLERANCE).max(0.0)
                );
                name_scalar_row(
                    ui, "Fog end", &mut vis.map.fogend, 1e-3,
                    (vis.map.fogstart + RANGE_PRECISION_TOLERANCE)..=f32::INFINITY
                );
                name_scalar_row(
                    ui, "Z near", &mut vis.map.znear, 1e-3,
                    0.001..=(vis.map.zfar - RANGE_PRECISION_TOLERANCE)
                );
                name_scalar_row(
                    ui, "Z far", &mut vis.map.zfar, 1e-3,
                    (vis.map.znear + RANGE_PRECISION_TOLERANCE)..=f32::INFINITY
                );
                name_scalar_row(ui, "Haze", &mut vis.map.haze, 1e-3, 0.0..=1.0);
                name_scalar_row(ui, "Shadow clip", &mut vis.map.shadowclip, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Shadow scale", &mut vis.map.shadowscale, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(
                    ui, "Actuator tendon", &mut vis.map.actuatortendon, 1e-3,
                    0.0..=f32::INFINITY
                );
            });
        });

        // Scale
        ui.collapsing(RichText::new("Scale").font(MAIN_FONT), |ui| {
            egui::Grid::new("scale_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Force width", &mut vis.scale.forcewidth, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(
                    ui, "Contact width", &mut vis.scale.contactwidth, 1e-3,
                    0.0..=f32::INFINITY
                );
                name_scalar_row(
                    ui, "Contact height", &mut vis.scale.contactheight, 1e-3,
                    0.0..=f32::INFINITY
                );
                name_scalar_row(ui, "Connect", &mut vis.scale.connect, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "COM", &mut vis.scale.com, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Camera", &mut vis.scale.camera, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Light", &mut vis.scale.light, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Select point", &mut vis.scale.selectpoint, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Joint length", &mut vis.scale.jointlength, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Joint width", &mut vis.scale.jointwidth, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(
                    ui, "Actuator length", &mut vis.scale.actuatorlength, 1e-3,
                    0.0..=f32::INFINITY
                );
                name_scalar_row(
                    ui, "Actuator width", &mut vis.scale.actuatorwidth, 1e-3,
                    0.0..=f32::INFINITY
                );
                name_scalar_row(ui, "Frame length", &mut vis.scale.framelength, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Frame width", &mut vis.scale.framewidth, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Constraint", &mut vis.scale.constraint, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Slider crank", &mut vis.scale.slidercrank, 1e-3, 0.0..=f32::INFINITY);
                name_scalar_row(ui, "Frustum", &mut vis.scale.frustum, 1e-3, 0.0..=1.0);
            });
        });

        // RGBA
        ui.collapsing(RichText::new("RGBA").font(MAIN_FONT), |ui| {
            egui::Grid::new("rgba_grid").num_columns(2).show(ui, |ui| {
                name_rgba_row(ui, "Fog", &mut vis.rgba.fog);
                name_rgba_row(ui, "Haze", &mut vis.rgba.haze);
                name_rgba_row(ui, "Force", &mut vis.rgba.force);
                name_rgba_row(ui, "Inertia", &mut vis.rgba.inertia);
                name_rgba_row(ui, "Joint", &mut vis.rgba.joint);
                name_rgba_row(ui, "Actuator", &mut vis.rgba.actuator);
                name_rgba_row(ui, "Actuator negative", &mut vis.rgba.actuatornegative);
                name_rgba_row(ui, "Actuator positive", &mut vis.rgba.actuatorpositive);
                name_rgba_row(ui, "COM", &mut vis.rgba.com);
                name_rgba_row(ui, "Camera", &mut vis.rgba.camera);
                name_rgba_row(ui, "Light", &mut vis.rgba.light);
                name_rgba_row(ui, "Select point", &mut vis.rgba.selectpoint);
                name_rgba_row(ui, "Connect", &mut vis.rgba.connect);
                name_rgba_row(ui, "Contact point", &mut vis.rgba.contactpoint);
                name_rgba_row(ui, "Contact force", &mut vis.rgba.contactforce);
                name_rgba_row(ui, "Contact friction", &mut vis.rgba.contactfriction);
                name_rgba_row(ui, "Contact torque", &mut vis.rgba.contacttorque);
                name_rgba_row(ui, "Contact gap", &mut vis.rgba.contactgap);
                name_rgba_row(ui, "Rangefinder", &mut vis.rgba.rangefinder);
                name_rgba_row(ui, "Constraint", &mut vis.rgba.constraint);
                name_rgba_row(ui, "Slider crank", &mut vis.rgba.slidercrank);
                name_rgba_row(ui, "Crank broken", &mut vis.rgba.crankbroken);
                name_rgba_row(ui, "Frustum", &mut vis.rgba.frustum);
                name_rgba_row(ui, "BV", &mut vis.rgba.bv);
                name_rgba_row(ui, "BV active", &mut vis.rgba.bvactive);
            });
        });

        // Write modified vis and stat back to model
        {
            let mut lock = shared_viewer_state.lock_unpoison();
            if lock.model_reloaded {
                return;
            }
            *lock.data_passive.model_vis_mut() = vis;
            *lock.data_passive.model_stat_mut() = stat;
        }
    });
}
