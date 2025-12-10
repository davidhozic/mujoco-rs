//! Example of the native Rust viewer implementation.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = stringify! {
<mujoco>
  <worldbody>
    <light ambient="0.2 0.2 0.2" pos=".2 .2 .2"/>
    <body name="ball">
        <geom name="green_sphere" size=".1" rgba="0 1 0 1"/>
        <joint name="sphere_joint" type="free"/>
    </body>

    <geom name="floor" type="plane" size="10 10 1" euler="5 0 0"/>

  </worldbody>
</mujoco>
};

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");

    /* Add a custom UI window */
    viewer.add_ui_callback(|ctx, data| {
        use mujoco_rs::viewer::egui;
        egui::Window::new("Custom controls")
            .scroll(true)
            .show(ctx, |ui| {
                ui.heading("My Custom Widget");
                ui.label("This is a custom UI element!");
                if ui.button("Click me").clicked() {
                    println!("Button clicked!");
                }
            });
    });

    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render();
        std::thread::sleep(Duration::from_millis(2));
    }
}
