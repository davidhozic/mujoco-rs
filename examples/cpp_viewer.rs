//! Example of the native C++ viewer wrapper.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
use std::time::Duration;

use mujoco_rs::cpp_viewer::MjViewerCpp;
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
    let mut viewer = MjViewerCpp::launch_passive(&model, &data, 100);
    let step = model.opt().timestep;
    while viewer.running() {
        viewer.sync();
        viewer.render();
        data.step();
        std::thread::sleep(Duration::from_secs_f64(step));
    }
}
