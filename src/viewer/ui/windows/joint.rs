//! A submodule of the viewer's UI representing the joint window definitions.
use std::sync::{Arc, Mutex};
use std::f64::consts::PI;
use std::mem;

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase, MjtJoint};
use crate::viewer::ui::{MAIN_FONT, SECONDARY_FONT};
use crate::viewer::ui::widget_extras::SearchBar;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::{
    ComponentDisplayInfo, ElementDisplayInfo, INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y,
    QUAT_COMPONENTS, XYZ_COMPONENTS
};

/// Value change per point of pointer travel in the numeric input of a joint component.
const JOINT_INPUT_DRAG_SPEED: f64 = 0.01;

/// Returns the display information of each joint of `model`, in ID order.
pub fn display_infos(model: &MjModel) -> impl Iterator<Item = ElementDisplayInfo> {
    (0..model.njnt() as usize).map(|joint_id| {
        let name = model.id_to_name(MjtObjBase::mjOBJ_JOINT, joint_id)
            .map(|name| name.to_string())
            .unwrap_or_else(|| format!("Joint {joint_id}"));

        let adr_start = model.jnt_qposadr()[joint_id] as usize;
        let limits = model.jnt_range()[joint_id];

        let components: Box<[_]> = match model.jnt_type()[joint_id] {
            // A free joint holds no limit and no natural range for its position, so each
            // translation component takes a numeric input.
            MjtJoint::mjJNT_FREE => {
                Box::new([
                    ComponentDisplayInfo {
                        name: "position".to_owned(), range: None, unit: " m", sub_component_names: &XYZ_COMPONENTS
                    },
                    ComponentDisplayInfo {
                        name: "orientation".to_owned(), range: Some(-1.0..=1.0), unit: "", sub_component_names: &QUAT_COMPONENTS
                    },
                ])
            }

            // A quaternion component always lies inside the unit sphere.
            MjtJoint::mjJNT_BALL => {
                Box::new([ComponentDisplayInfo {
                    name: "orientation".to_owned(), range: Some(-1.0..=1.0), unit: "", sub_component_names: &QUAT_COMPONENTS
                }])
            }

            // A slide or a hinge joint holds one position component, which the limit applies
            // to. Without a limit the slider spans the same range as MuJoCo's own viewer.
            joint_type => {
                let is_slide = joint_type == MjtJoint::mjJNT_SLIDE;
                let range = if model.jnt_limited()[joint_id] {
                    limits[0]..=limits[1]
                } else if is_slide {
                    -1.0..=1.0
                } else { -PI..=PI };
                let unit = if is_slide { " m" } else { " rad" };
                Box::new([ComponentDisplayInfo {
                    name: "position".to_owned(), range: Some(range), unit, sub_component_names: &[""]
                }])
            }
        };

        ElementDisplayInfo::Full { name, adr_start, components, n_replicate: 1 }
    })
}

/// Shows the joint window when `open` is set.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, joint_info: &[ElementDisplayInfo], search_bar: &mut SearchBar,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    // Clear the text inside the window's search bar when the window is closed.
    if !*open {
        search_bar.clear();
    }

    egui::Window::new("Joint")
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .open(open)
        .show(ui, |ui|
    {
        let mut lock = shared_viewer_state.lock_unpoison();
        if lock.model_reloaded {
            return;
        }
        let qpos = lock.data_passive.qpos_mut();

        /* Search bar */
        search_bar.show(ui);

        /* Joint widgets */
        for info in joint_info {
            if let Some(query) = search_bar.text() && !info.name().starts_with(query) {
                continue;
            }

            match info {
                ElementDisplayInfo::NameOnly { name } => {
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        ui.label(RichText::new("unsupported").font(SECONDARY_FONT));
                    });
                }
                ElementDisplayInfo::Full { name, adr_start, components, .. } => {
                    let mut rest = &mut qpos[*adr_start..];
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        egui::Grid::new("joint_qpos_ui_grid")
                            .spacing(egui::vec2(INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y))
                            .min_col_width(0.0)
                            .show(ui, |ui|
                        {
                            for component in components {
                                let values;
                                (values, rest) = mem::take(&mut rest).split_at_mut(component.sub_component_names.len());

                                // Empty labels also take a cell, so that the values of all rows
                                // share the same columns.
                                ui.label(RichText::new(&component.name).font(SECONDARY_FONT));
                                for (sub_name, value) in component.sub_component_names.iter().zip(values) {
                                    ui.label(RichText::new(*sub_name).font(SECONDARY_FONT));

                                    // A slider must not clamp: the simulation reaches values outside
                                    // the range, for example a hinge that turns past pi.
                                    match &component.range {
                                        Some(range) => ui.add(
                                            egui::Slider::new(value, range.clone())
                                            .clamping(egui::SliderClamping::Never)
                                            .update_while_editing(false)
                                            .suffix(component.unit)
                                        ),

                                        None => ui.add(
                                            egui::DragValue::new(value)
                                            .speed(JOINT_INPUT_DRAG_SPEED)
                                            .update_while_editing(false)
                                            .suffix(component.unit)
                                        )
                                    };
                                    ui.add(egui::Separator::default().vertical());
                                }
                                ui.end_row();
                            }
                        });
                    });
                }
            }
        }
    });
}
