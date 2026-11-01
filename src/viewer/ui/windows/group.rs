//! A submodule of the viewer's UI representing the sensor window definitions.
use egui_winit::egui;

use crate::wrappers::mj_visualization::MjvOption;
use crate::mujoco_c::mjNGROUP;
use crate::cast_mut_info;

pub(crate) fn show(ui: &mut egui::Ui, open: &mut bool, options: &mut MjvOption) {
    egui::Window::new("Group")
        .open(open)
        .resizable(false)
        .show(ui, |ui|
    {
        egui::Grid::new("group_grid").show(ui, |ui| {
            for i in 0..mjNGROUP as usize {
                ui.toggle_value(cast_mut_info!(&mut options.geomgroup[i], i), format!("Geom {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.sitegroup[i], i), format!("Site {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.jointgroup[i], i), format!("Joint {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.tendongroup[i], i), format!("Tendon {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.actuatorgroup[i], i), format!("Actuator {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.flexgroup[i], i), format!("Flex {i}"));
                ui.toggle_value(cast_mut_info!(&mut options.skingroup[i], i), format!("Skin {i}"));
                ui.end_row();
            }
        });
    });
}
