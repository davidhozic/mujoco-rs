//! Example demonstrating custom UI widgets in the MuJoCo viewer.
//! This example shows how to use the add_ui_callback method to create
//! custom windows, panels, and other UI elements using egui.

use std::time::Duration;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;

const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\" pos=\".2 .2 .2\"/>
    <body name=\"ball\" pos=\"0 0 0.5\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>
  </worldbody>
</mujoco>
";

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");

    // Example 1: Simple information window
    #[cfg(feature = "viewer-ui")]
    viewer.add_ui_callback(|ctx| {
        use mujoco_rs::viewer::egui;
        egui::Window::new("Simulation Info")
            .fade_in(false)
            .fade_out(false)
            .default_pos([400.0, 50.0])
            .show(ctx, |ui| {
                ui.heading("Simulation Information");
                ui.separator();
                ui.label("This is a custom information window.");
                ui.label("You can add any egui widgets here!");
            });
    });

    // Example 2: Side panel with controls
    #[cfg(feature = "viewer-ui")]
    viewer.add_ui_callback(|ctx| {
        use mujoco_rs::viewer::egui;
        egui::SidePanel::right("custom_panel")
            .default_width(200.0)
            .show(ctx, |ui| {
                ui.heading("Custom Panel");
                ui.separator();
                ui.label("This is a custom side panel");
                ui.label("Added via add_ui_callback!");
                
                if ui.button("Example Button").clicked() {
                    println!("Custom button clicked!");
                }
            });
    });

    // Example 3: Top panel
    #[cfg(feature = "viewer-ui")]
    viewer.add_ui_callback(|ctx| {
        use mujoco_rs::viewer::egui;
        egui::TopBottomPanel::top("custom_top_panel")
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Custom Top Bar");
                    ui.separator();
                    ui.label("MuJoCo Rust Viewer with Custom Widgets");
                });
            });
    });

    // Run the simulation
    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
