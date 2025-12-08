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
    // Create model and data.
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()

    // Create the viewer, bound to the model.
    let mut viewer = MjViewer::builder()
        .max_user_geoms(100)
        .build_passive(&model)
        .expect("could not launch the viewer");

    // Viewer can also be created like so:
    // let mut viewer = MjViewer::launch_passive(&model, 100)
    //     .expect("could not launch the viewer");

    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
