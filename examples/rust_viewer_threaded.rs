//! Example on how to use the Rust-native viewer ([`MjViewer`]) in a multi-threaded fashion.
//! By that we mean rendering can be done on the main thread, and the physics on another.
use std::time::Instant;
use std::sync::Arc;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = stringify! {
<mujoco>
  <worldbody>
    <light ambient="0.2 0.2 0.2" pos=".2 .2 .2"/>
    <body name="ball">
        <geom name="green_sphere" size=".1" rgba="0 1 0 1"/>
        <joint name="sphere_joint" type="free"/>
    </body>

    <geom name="floor" type="plane" size="10 10 1" euler="5 0 0"/>

  </worldbody>
</mujoco>
};

fn main() {
    // Create model and data.
    let model = Arc::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"));
    let mut data = MjData::new(model.clone());  // or model.make_data()

    // Create the viewer, bound to the model.
    let mut viewer = MjViewer::builder()
        .max_user_geoms(100)
        .vsync(true)  // let the viewer select the appropriate refresh rate.
        .build_passive(model.clone())
        .expect("could not launch the viewer");

    let shared_state = viewer.state().clone();
    let mut viewer_running = shared_state.lock().unwrap().running();  // gets moved into the thread
    let physics_thread = std::thread::spawn(move || {
        while viewer_running {
            let timer = Instant::now();
            data.step();
            {
                let mut lock = shared_state.lock().unwrap();
                lock.sync_data(&mut data);
                viewer_running = lock.running();
            }

            // Use a while loop and polling to wait for accuracy purposes.
            // To increase performance, std::thread::sleep may be used,
            // however that comes at the cost of less accuracy.
            while timer.elapsed().as_secs_f64() < model.opt().timestep {}
        }
    });

    while viewer.running() {
        viewer.render();
    }

    physics_thread.join().unwrap();
}
