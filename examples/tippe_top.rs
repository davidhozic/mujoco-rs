//! Example of simulating the self-inverting tippe-top.
//!
//! Demonstrates:
//! - Using the RK4 integrator (set via the XML `<option integrator="RK4"/>`).
//! - Loading an initial simulation state from a keyframe
//!   (the top starts with a fast spin of 200 rad/s).
//! - Visualising a free-floating rigid body with a [`MjViewer`].
//!
//! The tippe-top is a classic toy that spontaneously inverts while spinning
//! due to gyroscopic precession combined with frictional dissipation.
//! When the simulation starts the stem points upward; after several seconds
//! it flips 180 degrees so that the stem points downward.
//!
//! Press `[` / `]` inside the viewer to cycle cameras (the "closeup" camera
//! built into the model gives a better view of the inversion).
//!
//! Based on the MuJoCo Python tutorial:
//! <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb>

use std::time::Duration;

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;

// ---------------------------------------------------------------------------
// Model XML
// ---------------------------------------------------------------------------
// The model contains:
//   - A flat plane (the ground) with a checker-board texture.
//   - A "top" body with three geoms:
//       * ball  - the main sphere at the bottom of the top.
//       * stem  - a thin cylinder sticking upward from the ball.
//       * ballast - a flat box just below the ball centre that lowers the
//                   centre of mass below the geometric centre, which is
//                   required for the self-inversion to occur.
//         (contype/conaffinity = 0 means the ballast never collides.)
//   - A "closeup" camera looking at the top from a low angle.
//   - A keyframe named "spinning" that places the top at height 0.02 m and
//     gives it an angular velocity of 200 rad/s around the z-axis.
const EXAMPLE_MODEL: &str = r#"
<mujoco model="tippe top">
  <option integrator="RK4"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
  </asset>

  <worldbody>
    <geom size=".2 .2 .01" type="plane" material="grid"/>
    <light pos="0 0 .6"/>
    <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>

    <body name="top" pos="0 0 .02">
      <freejoint/>
      <geom name="ball"    type="sphere"   size=".02"/>
      <geom name="stem"    type="cylinder" pos="0 0 .02"   size="0.004 .008"/>
      <geom name="ballast" type="box"      size=".023 .023 0.005" pos="0 0 -.015"
            contype="0" conaffinity="0" group="3"/>
    </body>
  </worldbody>

  <!--
    Keyframe that gives the top a high spin rate.
    qpos: x y z qw qx qy qz  (freejoint - position + quaternion)
    qvel: vx vy vz wx wy wz  (freejoint - linear velocity + angular velocity)
    The spin is 200 rad/s around z and a small 1 rad/s wobble around y.
  -->
  <keyframe>
    <key name="spinning" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 200"/>
  </keyframe>
</mujoco>
"#;

// Simulate for up to 20 seconds of simulation time.
const DURATION_SECS: f64 = 20.0;

fn main() {
    // Load the model and create simulation data.
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);

    // Reset to keyframe 0 ("spinning") so the top starts with its initial
    // angular velocity rather than at rest.
    data.reset_keyframe(0);

    // Launch an interactive viewer so we can watch the inversion.
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    println!(
        "Simulating tippe-top with the {} integrator.",
        if model.opt().integrator == MjtIntegrator::mjINT_RK4 as i32 {
            "RK4"
        } else {
            "Euler"
        }
    );
    println!("Press [ / ] to cycle cameras (try the 'closeup' camera).");
    println!("Running for {DURATION_SECS} seconds of simulation time ...");

    while viewer.running() && data.time() < DURATION_SECS {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();

        // Sleep for roughly one timestep to run near real-time.
        std::thread::sleep(Duration::from_secs_f64(model.opt().timestep));
    }

    println!("Simulation finished at t = {:.3} s.", data.time());
}
