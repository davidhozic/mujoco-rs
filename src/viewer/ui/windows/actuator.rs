//! A submodule of the viewer's UI representing the actuator window definitions.
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase, MjtJoint, MjtDyn, MjtGain, MjtBias};
use crate::viewer::ui::{MAIN_FONT, SECONDARY_FONT};
use crate::viewer::ui::widget_extras::SearchBar;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::{
    ComponentDisplayInfo, ElementDisplayInfo, INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y,
    actuator_joint_type
};

/// Returns the display information of each actuator of `model`, in ID order.
pub fn display_infos(model: &MjModel) -> impl Iterator<Item = ElementDisplayInfo> {
    (0..model.nactuator() as usize).map(|actuator_id| {
        let input_limits = model.actuator_ctrlrange();
        let adr_start = model.actuator_ctrladr()[actuator_id] as usize;
        let num_inputs = model.actuator_ctrlnum()[actuator_id] as usize;

        let name = model.id_to_name(MjtObjBase::mjOBJ_ACTUATOR, actuator_id)
            .map(|name| name.to_string())
            .unwrap_or_else(|| format!("Actuator {actuator_id}"));

        let joint_type = actuator_joint_type(model, actuator_id);
        let is_slide = joint_type == Some(MjtJoint::mjJNT_SLIDE);
        let dyntype = model.actuator_dyntype()[actuator_id];
        let is_integrator = dyntype == MjtDyn::mjDYN_INTEGRATOR;
        let is_unit_motor = model.actuator_gaintype()[actuator_id] == MjtGain::mjGAIN_FIXED
            && model.actuator_gainprm()[actuator_id][0].abs() == 1.0
            && model.actuator_biastype()[actuator_id] == MjtBias::mjBIAS_NONE
            && matches!(dyntype, MjtDyn::mjDYN_NONE | MjtDyn::mjDYN_FILTER | MjtDyn::mjDYN_FILTEREXACT)
            && model.actuator_plugin()[actuator_id] < 0;

        let components = (0..num_inputs)
            .map(|input_index| {
                // Most actuator types define no input names. An actuator with more than one
                // input then shows the input index, an actuator with one input shows no label.
                let input_name = model.actuator_input_name(actuator_id, input_index);
                let name = input_name
                    .map(str::to_owned)
                    .or_else(|| (num_inputs > 1).then(|| input_index.to_string()))
                    .unwrap_or_default();

                let [low, high] = input_limits[adr_start + input_index];
                let range = if low < high { low..=high } else { -1.0..=1.0 };
                let unit = match input_name {
                    Some("voltage") => " V",
                    Some("rx" | "ry" | "rz") => if is_integrator { " rad/s" } else { " rad" },
                    _ if joint_type.is_none() => "",
                    Some("pos") => if is_slide { " m" } else { " rad" },
                    Some("vel") => if is_slide { " m/s" } else { " rad/s" },
                    Some("ff") => if is_slide { " N" } else { " N m" },
                    None if is_unit_motor => if is_slide { " N" } else { " N m" },
                    // The gain of a pressure input is an area, so the force must be in N.
                    Some("pressure") if is_slide => " Pa",
                    _ => ""
                };

                ComponentDisplayInfo { name, range: Some(range), unit, sub_component_names: &[""] }
            }).collect();

        ElementDisplayInfo::Full { name, adr_start, components, n_replicate: 1 }
    })
}

/// Shows the actuator window when `open` is set.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, actuator_info: &[ElementDisplayInfo], search_bar: &mut SearchBar,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    // Clear the text inside the window's search bar when the window is closed.
    if !*open {
        search_bar.clear();
    }

    egui::Window::new("Actuator")
        .id(egui::Id::new("mujoco_rs_actuator_window"))
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .open(open)
        .show(ui, |ui|
    {
        let mut lock = shared_viewer_state.lock_unpoison();
        if lock.model_reloaded {
            return;
        }
        let ctrl_mut = lock.data_passive.ctrl_mut();

        /* Search bar */
        search_bar.show(ui);

        /* Actuator widgets */
        for (actuator_id, info) in actuator_info.iter().enumerate() {
            if let Some(query) = search_bar.text() && !info.name().starts_with(query) {
                continue;
            }

            match info {
                ElementDisplayInfo::NameOnly { name } => {
                    egui::CollapsingHeader::new(RichText::new(name).font(MAIN_FONT)).id_salt(actuator_id).show(ui, |ui| {
                        ui.label(RichText::new("unsupported").font(SECONDARY_FONT));
                    });
                }
                ElementDisplayInfo::Full { name, adr_start, components, .. } => {
                    let actuator_ctrl = &mut ctrl_mut[*adr_start..*adr_start + components.len()];
                    egui::CollapsingHeader::new(RichText::new(name).font(MAIN_FONT)).id_salt(actuator_id).show(ui, |ui| {
                        if components.is_empty() {
                            ui.label(RichText::new("no inputs").font(SECONDARY_FONT));
                            return;
                        }

                        egui::Grid::new("actuator_ctrl_ui_grid")
                            .spacing(egui::vec2(INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y))
                            .min_col_width(0.0)
                            .show(ui, |ui|
                        {
                            for (component, ctrl) in components.iter().zip(actuator_ctrl) {
                                if !component.name.is_empty() {
                                    ui.label(RichText::new(&component.name).font(SECONDARY_FONT));
                                }
                                ui.add(
                                    egui::Slider::new(ctrl, component.range.clone().unwrap())
                                    .clamping(egui::SliderClamping::Never)
                                    .update_while_editing(false)
                                    .suffix(component.unit)
                                );
                                ui.add(egui::Separator::default().vertical());
                            }
                            ui.end_row();
                        });
                    });
                }
            }
        }

        if ui.button(RichText::new("Clear").font(MAIN_FONT)).clicked() {
            ctrl_mut.fill(0.0);
        }
    });
}
