//! Example of using views.
//! The example shows how to obtain a [`MjJointInfo`] struct that can be used
//! to create a (temporary) [`MjJointView`] to corresponding fields in [`MjData`].
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\">
        <geom name=\"green_sphere\" pos=\".2 .2 .1\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball_joint\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" solref=\"0.004 1.0\"/>
  </worldbody>
</mujoco>
";

fn main() {
    /* Load the model and create data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = model.make_data();  // or MjData::new(&model);

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    /* Create the joint info */
    let ball_info = data.joint("ball_joint").unwrap();
    while viewer.running() {
        /* Step the simulation and sync the viewer */
        viewer.sync(&mut data);

        data.step1();
        let xyz = &ball_info.view(&mut data).qpos[..3];
        println!("{xyz:.2?}");
        data.step2();

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
