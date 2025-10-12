//! Example of using using a tendon model.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        
        <body name=\"ball1\" pos=\"-.5 0 0\">
            <geom size=\".1\" rgba=\"0 1 0 1\" mass=\"1\"/>
            <joint type=\"free\"/>
            <site name=\"ball1\" size=\".1 .1 .1\" pos=\"0 0 0\" rgba=\"0 1 0 0.2\" type=\"box\"/>
        </body>

        <body name=\"ball2\"  pos=\".5 0 0\">
            <geom size=\".1\" rgba=\"0 1 1 1\" mass=\"1\"/>
            <joint type=\"free\"/>
            <site name=\"ball2\" size=\".1 .1 .1\" pos=\"0 0 0\" rgba=\"0 1 1 0.2\" type=\"box\"/>
        </body>

        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\"/>
    </worldbody>

    <tendon>
        <spatial limited=\"true\" range=\"0 1\" rgba=\"0 .1 1 1\" width=\".005\">
        <site site=\"ball1\"/>
        <site site=\"ball2\"/>
        </spatial>
    </tendon>
</mujoco>
";

fn main() {
    /* Load the model and create data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    while viewer.running() {
        /* Step the simulation and sync the viewer */
        viewer.sync(&mut data);
        data.step();

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
