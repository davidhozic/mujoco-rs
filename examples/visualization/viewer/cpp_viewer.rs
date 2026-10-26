//! Example of the native C++ viewer wrapper.
//! This can only be run in passive mode, which means the user program is the one
//! controlling everything.
use mujoco_rs::cpp_viewer::MjViewerCpp;
use mujoco_rs::util::LockUnpoison;
use mujoco_rs::prelude::*;

use env_logger::Env;

use std::time::Duration;

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
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // (Optional) The hook sends MuJoCo's messages to the `log` crate, instead of the console.
    // SAFETY: no other thread uses MuJoCo yet.
    unsafe { install_logging_hook() };

    let model = Box::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"));
    let step = model.opt().timestep;

    // SAFETY: called from the main thread.
    let mut viewer = unsafe { MjViewerCpp::launch_passive(MjData::new(model), 100) };
    let state = viewer.state().clone();

    while viewer.running() {
        {
            let mut lock = state.lock_unpoison();
            unsafe { lock.data_mut() }.step();
            lock.sync();
            std::thread::sleep(Duration::from_secs_f64(step));
        }

        viewer.render().unwrap();
    }
}
