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
    let model = MjModel::from_xml("/home/davidhozic/Downloads/google-deepmind mujoco_menagerie main boston_dynamics_spot/scene.xml").expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");
    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
