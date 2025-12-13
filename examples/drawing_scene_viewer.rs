//! This example illustrates how to use the MjViewer::user_scn_mut to draw custom
//! visual-only geoms.
//! 
//! This example uses the viewer in single-threaded fashion.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball1\" pos=\".2 .2 .2\">
        <geom size=\".1 .1 .1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball1_joint\" type=\"free\"/>
    </body>

    <body name=\"ball2\" pos=\".5 .2 .2\">
        <geom size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball2_joint\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" euler=\"15 4 0\" solref=\"0.004 1.0\"/>
    <geom name=\"floor2\" type=\"plane\" pos=\"15 -20 0\" size=\"10 10 1\" euler=\"-15 -4 0\" solref=\"0.004 1.0\"/>

  </worldbody>
</mujoco>
";



fn main() {
    /* Create model and data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(
        &model,
        10  // create space for 10 visual-only geoms
    )
        .expect("could not launch the viewer");

    /* Obtain joint info */
    let ball1_joint_info = data.joint("ball1_joint").unwrap();
    let ball2_joint_info = data.joint("ball2_joint").unwrap();

    /* Give the first ball some y-velocity to showcase the visualization */
    ball1_joint_info.view_mut(&mut data).qvel[1] = 2.0;

    let timestep = model.opt().timestep;
    while viewer.running() {
        /* Step the simulation and sync the viewer */
        data.step();
        viewer.with_state_lock(|mut state_lock| {
            state_lock.sync_data(&mut data);

            /* Prepare the scene */
            let scene = state_lock.user_scene_mut();  // obtain a mutable reference to the user scene. The method name mirrors the C++ viewer.
            scene.clear_geom();  // clear existing geoms

            /* Create a line, that (visually) connects the two balls we have in the example model */
            let new_geom = scene.create_geom(
                MjtGeom::mjGEOM_LINE,  // type of geom to draw.
                None,  // size, ignore here as we set it below.
                None,   // position: ignore here as we set it below.
                None,   // rotational matrix: ignore here as we set it below.
                Some([1.0, 1.0, 1.0, 1.0])  // color (rgba): pure white.
            );

            /* Read X, Y and Z coordinates of both balls. */
            let ball1_position = ball1_joint_info.view(&data).qpos[..3]
                .try_into().unwrap();
            let ball2_position = ball2_joint_info.view(&data).qpos[..3]
                .try_into().unwrap();

            /* Modify the visual geom's position, orientation and length, to connect the balls */
            new_geom.connect(
                0.0,            // width
                ball1_position,  // from
                ball2_position     //  to
            );
        });

        viewer.render();
        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}