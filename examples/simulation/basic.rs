//! The most basic example that shows how to load a model from string,
//! create a [`MjData`] instance and step simulation.
use std::time::Duration;

use mujoco_rs::prelude::*;
use env_logger::Env;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\" pos=\".2 .2 .2\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

  </worldbody>
</mujoco>
";

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // The hook sends MuJoCo's messages to the `log` crate, instead of directly to the console.
    install_logging_hook();

    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()
    for i in 0..1000 {
        log::info!("Step {i}");
        data.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
