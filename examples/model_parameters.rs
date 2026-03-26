//! Example of modifying model parameters at runtime.
//!
//! Demonstrates three approaches for changing physics parameters of an
//! [`MjModel`] that is already attached to an [`MjData`]:
//!
//! 1. **[`MjData::model_mut`]** --- direct mutable access.
//!    Available when the model-handle type `M` implements `DerefMut`
//!    (e.g., `Box<MjModel>`).
//!    This is the simplest and recommended approach.
//!
//! 2. **[`MjData::swap_model`]** --- swap the model for a modified copy.
//!    Works with any `M` (including `Arc<MjModel>`) and validates model
//!    signature compatibility.
//!
//! 3. **`unsafe` [`MjModel::ffi_mut`]** --- direct access to the underlying
//!    C struct. This bypasses all Rust-level safety guarantees and should
//!    only be used when the safe API does not expose the field you need.
//!
//! **Not all model parameters are safe to change at runtime.**
//! See [MuJoCo's documentation](https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#mjmodel-changes)
//! for details on which parameters can be changed.
//!
//! The example loads a simple free-falling ball, runs a few steps with the
//! default gravity, then halves gravity using each of the three methods and
//! shows the effect on the ball's vertical position.

use std::ops::Deref;

use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = r#"
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


/// Run `STEPS` simulation steps and return the final vertical position of the ball.
fn simulate_steps(data: &mut MjData<impl Deref<Target = MjModel>>) -> f64 {
    for _ in 0..STEPS {
        data.step();
    }
    // qpos[2] is the z-component of the free joint (vertical position).
    data.qpos()[2]
}


fn load_model() -> Box<MjModel> {
    Box::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"))
}


fn main() {
    // ---------------------------------------------------------------
    // Baseline --- default gravity (-9.81 m/s^2)
    // ---------------------------------------------------------------
    let model = load_model();
    let mut data = MjData::new(model);

    let default_gravity = data.model().opt().gravity[2];
    let z_baseline = simulate_steps(&mut data);
    println!("=== Baseline ===");
    println!("  gravity[2]   = {default_gravity:.2}");
    println!("  z after {STEPS} steps = {z_baseline:.4}");

    // ---------------------------------------------------------------
    // Method 1 --- model_mut  (requires M: DerefMut)
    // ---------------------------------------------------------------
    // Because MjData holds a Box<MjModel> (which implements DerefMut),
    // we can call model_mut() to modify the physics options in place.
    data.reset();

    let half_gravity = default_gravity / 2.0;
    data.model_mut().opt_mut().gravity[2] = half_gravity;

    let z_model_mut = simulate_steps(&mut data);
    println!("\n=== Method 1: model_mut ===");
    println!("  gravity[2]   = {half_gravity:.2}");
    println!("  z after {STEPS} steps = {z_model_mut:.4}");

    // Restore original gravity for the next method.
    data.model_mut().opt_mut().gravity[2] = default_gravity;
    data.reset();

    // ---------------------------------------------------------------
    // Method 2 --- swap_model  (works with any M)
    // ---------------------------------------------------------------
    // Retrieve the current model, modify it externally, then swap it back.
    // swap_model validates that the model signature matches, so the same
    // model structure is required (only parameter values may differ).
    let mut model_out = data.swap_model(load_model()).unwrap();
    model_out.opt_mut().gravity[2] = half_gravity;

    // Swap the modified model back in. The returned model is the one
    // previously held by data (which still has default gravity).
    let _original = data.swap_model(model_out).unwrap();
    data.reset();

    let z_swap = simulate_steps(&mut data);
    println!("\n=== Method 2: swap_model ===");
    println!("  gravity[2]   = {half_gravity:.2}");
    println!("  z after {STEPS} steps = {z_swap:.4}");

    // Restore for the next method.
    data.model_mut().opt_mut().gravity[2] = default_gravity;
    data.reset();

    // ---------------------------------------------------------------
    // Method 3 --- unsafe ffi_mut  (escape hatch)
    // ---------------------------------------------------------------
    // When the safe wrappers do not expose a particular field, the
    // underlying C struct can be accessed through ffi_mut().
    //
    // SAFETY: We only modify opt.gravity, which is a physics parameter
    // documented by MuJoCo as safe to change at runtime.
    unsafe { data.model_mut().ffi_mut() }.opt.gravity[2] = half_gravity;
    data.reset();

    let z_ffi = simulate_steps(&mut data);
    println!("\n=== Method 3: unsafe ffi_mut ===");
    println!("  gravity[2]   = {half_gravity:.2}");
    println!("  z after {STEPS} steps = {z_ffi:.4}");

    // ---------------------------------------------------------------
    // Verify all methods produce the same result.
    // ---------------------------------------------------------------
    println!("\n=== Summary ===");
    println!("  Baseline z       = {z_baseline:.4}");
    println!("  model_mut z      = {z_model_mut:.4}");
    println!("  swap_model z     = {z_swap:.4}");
    println!("  unsafe ffi_mut z = {z_ffi:.4}");

    let eps = 1e-10;
    assert!(
        (z_model_mut - z_swap).abs() < eps,
        "model_mut and swap_model should produce identical results"
    );
    assert!(
        (z_model_mut - z_ffi).abs() < eps,
        "model_mut and ffi_mut should produce identical results"
    );
    assert!(
        z_model_mut > z_baseline,
        "halved gravity should result in a higher position than full gravity"
    );

    println!("\nAll three methods produce identical results.");
}
