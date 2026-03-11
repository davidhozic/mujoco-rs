//! An example on how to access attributes from the MuJoCo structs.
//! While most structs in this library are direct FFI structs to MuJoCo,
//! some have to be wrapped for safety reasons.
//! Wrapped structs expose their attributes through safe accessors such as `model.opt()`.
//! For advanced use cases, `ffi()` and `ffi_mut()` methods provide direct access to
//! the underlying C structs.
//! 
//! This example uses the viewer in single-threaded fashion.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\" pos=\".2 .2 .2\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" mass=\"0.1\" solref=\"0.004 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\" solref=\"0.004 1\"/>

  </worldbody>
</mujoco>
";

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");

    /******************************************/
    /* Access a FFI attribute */
    let timestep = model.opt().timestep;
    /******************************************/

    let mut data = MjData::new(&model);  // or model.make_data()
    let mut viewer = MjViewer::launch_passive(&model, 100)
        .expect("could not launch the viewer");

    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();
        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}
