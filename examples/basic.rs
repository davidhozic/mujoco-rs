//! The most basic example that shows how to load a model from string,
//! create a [`MjData`] instance and step simulation.
use std::time::Duration;

use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\">
        <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

  </worldbody>
</mujoco>
";

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = model.make_data();  // or MjData::new(&model);
    for i in 0..1000 {
        println!("Step {i}");
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
