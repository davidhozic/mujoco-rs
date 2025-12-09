//! Example how to use the Rust-native viewer ([`MjViewer`]) in a multi-threaded fashion.
//! By that we mean rendering can be done on the main thread, and the physics on another.
use std::sync::Arc;
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
    let model = Arc::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"));
    let mut data = MjData::new(model.clone());  // or model.make_data()

    // Create the viewer, bound to the model.
    let mut viewer = MjViewer::builder()
        .max_user_geoms(100)
        .build_passive(model.clone())
        .expect("could not launch the viewer");

    let shared_state = viewer.state().clone();
    std::thread::spawn(move || {
        loop {
            data.step();
            shared_state.lock().unwrap().sync(&mut data);
            std::thread::sleep(Duration::from_millis(2));
        }
    });

    while viewer.running() {
        viewer.render();
    }
}
