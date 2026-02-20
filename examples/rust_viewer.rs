//! Example of the native Rust viewer implementation.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
//! 
//! This example uses the viewer in single-threaded fashion.
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
    let mut viewer = MjViewer::builder()
        .max_user_geoms(0)
        .build_passive(&model).unwrap();

    /* Add a custom UI window */
    let mut opened = true;  // gets moved into the callback
    viewer.add_ui_callback_detached(move |ctx| {
        use mujoco_rs::viewer::egui;
        egui::Window::new("Custom controls")
            .scroll(true)
            .open(&mut opened)
            .show(ctx, |ui| {
                ui.heading("My Custom Widget");
                ui.label("This is a custom UI element!");
                if ui.button("Click me").clicked() {
                    println!("Button clicked!");
                }
            });
    });

    // OR, to gain access to the internal passive data at the same time:
    // viewer.add_ui_callback(move |ctx, _data| {
    //     use mujoco_rs::viewer::egui;
    //     egui::Window::new("Custom controls")
    //         .scroll(true)
    //         .open(&mut opened)
    //         .show(ctx, |ui| {
    //             ui.heading("My Custom Widget");
    //             ui.label("This is a custom UI element!");
    //             if ui.button("Click me").clicked() {
    //                 println!("Button clicked!");
    //             }
    //         });
    // });

    let timestep = model.opt().timestep;
    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render();

        // Sleep for approximately timestep of seconds.
        // Use Instant::now() and Instant::elapsed() in a while loop for a more accurate (but less efficient) wait.
        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}
