//! Example of the native Rust viewer implementation.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\" pos=\".2 .2 .2\"/>
    <body name=\"ball\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

  </worldbody>
</mujoco>
";

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");

    // Add a custom UI widget to demonstrate the new feature
    #[cfg(feature = "viewer-ui")]
    viewer.add_ui_callback(|ctx| {
        use mujoco_rs::viewer::egui;
        egui::Window::new("Custom controls")
            .fade_in(false)
            .fade_out(false)
            .scroll(true)
            .show(ctx, |ui| {
                ui.heading("Custom Widget Example");
                ui.label("This is a custom widget added via add_ui_callback!");
                ui.separator();
                ui.label("You can add any egui widgets here.");
            });
    });

    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
