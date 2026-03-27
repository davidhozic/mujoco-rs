//! Modifying [`MjModel`] physics parameters at runtime.
//!
//! Shows two ways to change the gravity of a model already attached to
//! [`MjData`]: in-place via [`MjData::model_mut`] and by swapping via
//! [`MjData::swap_model`].
//!
//! Not all model parameters are safe to change at runtime.
//! See [MuJoCo's docs](https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#mjmodel-changes)
//! for the full list.

use mujoco_rs::prelude::*;


const MODEL_XML: &str = r#"
<mujoco model="model_parameters">
  <option timestep="0.002"/>
  <worldbody>
    <body name="ball" pos="0 0 1">
      <joint type="free"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <geom type="plane" size="5 5 0.1"/>
  </worldbody>
</mujoco>
"#;

const STEPS: usize = 500;

fn main() {
    let model = Box::new(
        MjModel::from_xml_string(MODEL_XML).expect("could not load the model"),
    );
    let mut data = MjData::new(model);
    let default_gravity = data.model().opt().gravity[2];

    // Baseline: 500 steps under default gravity.
    for _ in 0..STEPS {
        data.step();
    }
    let z_baseline = data.qpos()[2];
    println!("baseline   z={z_baseline:.4}  (gravity={default_gravity:.2})");

    // model_mut: direct in-place mutation (requires M: DerefMut).
    data.reset();
    let half_gravity = default_gravity / 2.0;
    data.model_mut().opt_mut().gravity[2] = half_gravity;
    for _ in 0..STEPS {
        data.step();
    }
    let z_mut = data.qpos()[2];
    println!("model_mut  z={z_mut:.4}  (gravity={half_gravity:.2})");

    // Restore gravity for the next method.
    data.model_mut().opt_mut().gravity[2] = default_gravity;
    data.reset();

    // swap_model: swap in a modified model (works with any M).
    let fresh = Box::new(
        MjModel::from_xml_string(MODEL_XML).expect("could not load the model"),
    );
    let mut swapped_out = data.swap_model(fresh);
    swapped_out.opt_mut().gravity[2] = half_gravity;
    let _prev = data.swap_model(swapped_out);
    data.reset();
    for _ in 0..STEPS {
        data.step();
    }
    let z_swap = data.qpos()[2];
    println!("swap_model z={z_swap:.4}  (gravity={half_gravity:.2})");
}
