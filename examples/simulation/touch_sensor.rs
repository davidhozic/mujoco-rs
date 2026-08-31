//! Example of using sensor interaction: touch sensor.
//! 
//! This example uses the viewer in single-threaded fashion.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use env_logger::Env;


const EXAMPLE_MODEL: &str = "
<mujoco>
    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        <body name=\"ball\">
            <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" mass=\"1\"/>
            <joint type=\"free\"/>
            <site name=\"touch\" size=\".1 .1 .1\" pos=\"0 0 0\" rgba=\"0 1 0 0.2\" type=\"box\"/>
        </body>

        <body>
            <geom type=\"box\" size=\"1 1 1\" pos=\"2 1 1\" rgba=\"0 1 1 1\"/>
            <joint type=\"free\"/>
        </body>

        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\"/>
    </worldbody>

    <sensor>
        <touch name=\"touch\" site=\"touch\"/>
    </sensor>
</mujoco>
";

fn main() {
    /* Initialize the log backend and send MuJoCo messages to it */
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // (Optional) The hook sends MuJoCo's messages to the `log` crate, instead of the console.
    // SAFETY: no other thread uses MuJoCo yet.
    unsafe { install_logging_hook() };

    /* Load the model and create data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);  // or model.make_data()

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    let sensor_data_info = data.sensor("touch").unwrap();
    while viewer.running() {
        /* Step the simulation and sync the viewer */
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();

        /* Log the touch sensor output, which is the contact force */
        log::info!("{}", sensor_data_info.view(&data).data[0]);

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
