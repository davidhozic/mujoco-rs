//! A submodule of the viewer's UI representing the rendering side panel section definitions.
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;
use log::info;

use crate::viewer::ui::{UiEvent, HEADING_FONT, MAIN_FONT, MAX_SPAN_WIDTH, BUTTON_ROUNDING};
use crate::wrappers::mj_visualization::{MjvOption, MjvCamera, MjtCamera, MjvScene};
use crate::wrappers::mj_model::MjtObjBase;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;
use crate::cast_mut_info;

/// Maps [`MjtRndFlag`](crate::wrappers::mj_visualization::MjtRndFlag) to their string
const GL_EFFECT_MAP: [&str; 11] = [
    "Shadow",
    "Wireframe",
    "Reflection",
    "Additive",
    "Skybox",
    "Fog",
    "Haze",
    "Depth",
    "Segment",
    "ID color",
    "Cull face"
];
const _: () = assert!(GL_EFFECT_MAP.len() == crate::mujoco_c::mjtRndFlag_::mjNRNDFLAG as usize);

/// Maps [`MjtVisFlag`](crate::wrappers::mj_visualization::MjtVisFlag) to their string
const VIS_OPT_MAP: [&str; 31] = [
    "Convex hull",
    "Texture",
    "Joint",
    "Camera",
    "Actuator",
    "Activation",
    "Light",
    "Tendon",
    "Range finder",
    "Constraint",
    "Inertia",
    "Scale inertia",
    "Perturbation force",
    "Perturbation object",
    "Contact point",
    "Island",
    "Contact force",
    "Contact split",
    "Transparent",
    "Auto-connect",
    "Center of mass",
    "Select",
    "Static",
    "Skin",
    "Flex vertex",
    "Flex edge",
    "Flex face",
    "Flex skin",
    "Body BVH",
    "Mesh BVH",
    "SDF iteration"
];
const _: () = assert!(VIS_OPT_MAP.len() == crate::mujoco_c::mjtVisFlag_::mjNVISFLAG as usize);

/// Maps [`MjtLabel`](crate::wrappers::mj_visualization::MjtLabel) to their string
const LABEL_TYPE_MAP: [&str; 17] = [
    "None",
    "Body",
    "Joint",
    "Geom",
    "Site",
    "Camera",
    "Light",
    "Tendon",
    "Actuator",
    "Constraint",
    "Flex",
    "Skin",
    "Selection",
    "Selection point",
    "Contact point",
    "Contact force",
    "Island"
];
const _: () = assert!(LABEL_TYPE_MAP.len() == crate::mujoco_c::mjtLabel_::mjNLABEL as usize);

/// Maps [`MjtFrame`](crate::wrappers::mj_visualization::MjtFrame) to their string
const FRAME_TYPE_MAP: [&str; 8] = [
    "None",
    "Body",
    "Geom",
    "Site",
    "Camera",
    "Light",
    "Contact",
    "World"
];
const _: () = assert!(FRAME_TYPE_MAP.len() == crate::mujoco_c::mjtFrame_::mjNFRAME as usize);

#[expect(
    clippy::too_many_arguments,
    reason = "each parameter is a separately borrowed field of ViewerUI"
)]
pub(crate) fn show(
    ui: &mut egui::Ui, events: &mut VecDeque<UiEvent>, camera_names: &[String],
    show_tracking_modal: &mut bool, tracking_selected_body: &mut Option<usize>,
    screenshot_viewport_only: &mut bool, screenshot_depth: &mut bool,
    scene: &mut MjvScene, options: &mut MjvOption, camera: &mut MjvCamera,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    egui::CollapsingHeader::new(RichText::new("Rendering").font(HEADING_FONT))
        .show(ui, |ui|
    {
        egui::Grid::new("render_select_grid").num_columns(2).show(ui, |ui| {
            // Camera
            ui.label(RichText::new("Camera").font(MAIN_FONT));
            let combo_width = ui.available_width().min(MAX_SPAN_WIDTH);

            let Ok(enumerated) = camera.type_.try_into() else {
                // Unknown camera type - skip the whole selector grid rather than panic.
                ui.label(format!("Unknown camera type {}", camera.type_));
                ui.end_row();
                return;
            };
            let enumerated: MjtCamera = enumerated;
            let lock = shared_viewer_state.lock_unpoison();
            if lock.model_reloaded {
                return;
            }
            let model = lock.data_passive.model();
            let mut camera_choice = match enumerated {
                MjtCamera::mjCAMERA_FIXED => camera_names[camera.fixedcamid as usize].to_string(),
                MjtCamera::mjCAMERA_TRACKING => {
                    let bid = camera.trackbodyid as usize;
                    if let Some(name) = model.id_to_name(MjtObjBase::mjOBJ_BODY, bid) {
                        format!("Tracking: {}", name)
                    } else {
                        format!("Tracking: Body {}", bid)
                    }
                },
                MjtCamera::mjCAMERA_FREE => "Free".to_string(),
                MjtCamera::mjCAMERA_USER => "User".to_string(),
            };
            drop(lock);
            egui::ComboBox::from_id_salt("camera_combo")
                .selected_text(&camera_choice)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    if ui.selectable_value(&mut camera_choice, "Free".to_string(), "Free").clicked() {
                        camera.free();
                    }

                    // Button to open tracking modal
                    if ui.button("Track").clicked() {
                        *show_tracking_modal = true;
                    }

                    // Fixed cameras of the model.
                    for (pos, name) in camera_names.iter().enumerate() {
                        if ui.selectable_value(&mut camera_choice, name.to_string(), name).clicked() {
                            *camera = MjvCamera::new_fixed(pos);
                        }
                    }
                });

            // Apply selected body if one was selected in the modal
            if let Some(body_id) = tracking_selected_body.take() {
                camera.track(body_id);
            }

            ui.end_row();

            // Label
            ui.label(RichText::new("Label").font(MAIN_FONT));
            let mut current_lbl_ty = LABEL_TYPE_MAP[options.label as usize];
            egui::ComboBox::from_id_salt("label_combo")
                .selected_text(current_lbl_ty)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (label_type_i, label_type) in LABEL_TYPE_MAP.iter().enumerate() {
                        if ui.selectable_value(&mut current_lbl_ty, label_type, *label_type).clicked() {
                            options.label = label_type_i as i32;
                        }
                    }
                });
            ui.end_row();

            // Frame
            ui.label(RichText::new("Frame").font(MAIN_FONT));
            let mut current_frm_ty = FRAME_TYPE_MAP[options.frame as usize];
            egui::ComboBox::from_id_salt("frame_combo")
                .selected_text(current_frm_ty)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (frame_type_i, frame_type) in FRAME_TYPE_MAP.iter().enumerate() {
                        if ui.selectable_value(&mut current_frm_ty, frame_type, *frame_type).clicked() {
                            options.frame = frame_type_i as i32;
                        }
                    }
                });
            ui.end_row();
        });

        ui.add_space(5.0);

        // Print the camera state as MJCF to stdout.
        if ui.add(egui::Button::new(
            RichText::new("Print camera").font(MAIN_FONT)
        ).corner_radius(BUTTON_ROUNDING)).clicked() {
            let gl_cams = scene.camera();
            let fwd = gl_cams[0].forward;
            let up = gl_cams[0].up;

            // right = forward x up
            let right = [
                fwd[1] * up[2] - fwd[2] * up[1],
                fwd[2] * up[0] - fwd[0] * up[2],
                fwd[0] * up[1] - fwd[1] * up[0],
            ];

            // Average position of left and right eye cameras
            let pos = [
                (gl_cams[0].pos[0] + gl_cams[1].pos[0]) / 2.0,
                (gl_cams[0].pos[1] + gl_cams[1].pos[1]) / 2.0,
                (gl_cams[0].pos[2] + gl_cams[1].pos[2]) / 2.0,
            ];

            info!(
                "<camera pos=\"{:.3} {:.3} {:.3}\" \
                xyaxes=\"{:.3} {:.3} {:.3} {:.3} {:.3} {:.3}\"/>",
                pos[0], pos[1], pos[2],
                right[0], right[1], right[2],
                up[0], up[1], up[2]
            );
            // TODO in the future, when
            // clipboard copying feature doesn't crash:
            // ctx.copy_text(xml);
            // Until then ^^^.
        }

        // Screenshot
        ui.collapsing(RichText::new("Screenshot").font(MAIN_FONT), |ui| {
            if ui.add(egui::Button::new(
                RichText::new("Screenshot").font(MAIN_FONT)
            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                events.push_back(UiEvent::Screenshot {
                    viewport_only: *screenshot_viewport_only,
                    depth: *screenshot_depth
                });
            }
            ui.checkbox(
                screenshot_viewport_only,
                RichText::new("Viewport only").font(MAIN_FONT)
            );
            ui.checkbox(
                screenshot_depth,
                RichText::new("Depth").font(MAIN_FONT)
            );
        });

        ui.collapsing(RichText::new("Elements").font(MAIN_FONT), |ui| {
            ui.horizontal_wrapped(|ui| {
                for (flag, (enabled, flag_name)) in options.flags.iter_mut().zip(VIS_OPT_MAP).enumerate() {
                    ui.toggle_value(cast_mut_info!(enabled, flag), flag_name);
                }
            });
            ui.separator();
            egui::Grid::new("tree_flex_slide").show(ui, |ui| {
                ui.label("Tree depth");
                ui.add(egui::Slider::new(&mut options.bvh_depth, 0..=20));
                ui.end_row();
                ui.label("Flex layer");
                ui.add(egui::Slider::new(&mut options.flex_layer, 0..=10));
            });
        });

        ui.collapsing(RichText::new("OpenGL effects").font(MAIN_FONT), |ui| {
            ui.horizontal_wrapped(|ui| {
                for (flag, (enabled, flag_name)) in scene.flags_mut().iter_mut().zip(GL_EFFECT_MAP).enumerate() {
                    ui.toggle_value(cast_mut_info!(enabled, flag), flag_name);
                }
            });
        });
    });
}
