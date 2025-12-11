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
    <body name=\"ball\" pos=\".2 .2 .2\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball_joint\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" euler=\"15 4 0\" solref=\"0.004 1.0\"/>
    <geom name=\"floor2\" type=\"plane\" pos=\"15 -20 0\" size=\"10 10 1\" euler=\"-15 -4 0\" solref=\"0.004 1.0\"/>

  </worldbody>
</mujoco>
";

fn main() {
    /* Load the model and create data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    /* Create the joint info */
    let ball_info = data.joint("ball_joint").unwrap();
    while viewer.running() {
        /* Step the simulation and sync the viewer */
        data.step();
        viewer.sync_data(&mut data);
        viewer.render();

        /* Obtain the view and access first three variables of `qpos` (x, y, z) */
        let xyz = &ball_info.view(&data).qpos[..3];
        println!("The ball's position is: {xyz:.2?}");

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
