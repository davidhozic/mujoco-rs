//! Example of the native Rust viewer implementation.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
use mujoco_rs_w::viewer::MjViewer;
use mujoco_rs_w::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <geom name=\"red_box\" type=\"box\" size=\".2 .2 .2\" rgba=\"1 0 0 1\"/>
    <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
  </worldbody>
</mujoco>
";

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = model.make_data();
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");
    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
    }
}
