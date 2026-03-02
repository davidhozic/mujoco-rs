//! Example of a chaotic pendulum demonstrating the butterfly effect.
//!
//! Demonstrates:
//! - Constructing simulation state by directly setting joint velocities via
//!   the joint view API ([`MjJointDataInfo::view_mut`]).
//! - Computing and printing mechanical energy using [`MjData::energy_pos`] and
//!   [`MjData::energy_vel`] (requires `<flag energy="enable"/>` in the model).
//! - The butterfly effect: two simulations started with nearly identical initial
//!   conditions diverge rapidly.
//!
//! The model is a four-body pendulum represented as a binary tree:
//!   - A central T-shaped body ("0") that can rotate about the y-axis.
//!   - Three pendulums hanging or extending from body "0".
//!
//! The first phase of this example runs two headless simulations with
//! a tiny velocity perturbation (1e-5 rad/s in the root joint) and reports
//! when the trajectories diverge beyond a threshold.
//! The second phase launches an interactive viewer so you can observe the
//! chaotic motion live.
//!
//! Based on the MuJoCo Python tutorial:
//! <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb>

use std::time::Duration;

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;

// ---------------------------------------------------------------------------
// Model XML
// ---------------------------------------------------------------------------
// Global flag `energy="enable"` activates energy bookkeeping in mjData.
// `contact="disable"` removes contact forces so that energy is conserved
// (apart from numerical integration error), making the chaos purely
// geometric (Hamiltonian chaos).
const EXAMPLE_MODEL: &str = r#"
<mujoco model="chaotic pendulum">
  <option timestep=".001">
    <flag energy="enable" contact="disable"/>
  </option>

  <default>
    <joint type="hinge" axis="0 -1 0"/>
    <geom type="capsule" size=".02"/>
  </default>

  <worldbody>
    <light pos="0 -.4 1"/>
    <camera name="fixed" pos="0 -1 0" xyaxes="1 0 0 0 0 1"/>

    <!--
      Body "0" is the central hub.  It has two horizontal arms along x
      and one vertical pendulum pointing downward.
    -->
    <body name="0" pos="0 0 .2">
      <joint name="root"/>
      <!-- horizontal cross-bar -->
      <geom fromto="-.2 0 0 .2 0 0" rgba="1 1 0 1"/>
      <!-- vertical rod pointing down -->
      <geom fromto="0 0 0 0 0 -.25" rgba="1 1 0 1"/>

      <!-- left pendulum -->
      <body name="1" pos="-.2 0 0">
        <joint name="j1"/>
        <geom fromto="0 0 0 0 0 -.2" rgba="1 0 0 1"/>
      </body>

      <!-- right pendulum -->
      <body name="2" pos=".2 0 0">
        <joint name="j2"/>
        <geom fromto="0 0 0 0 0 -.2" rgba="0 1 0 1"/>
      </body>

      <!-- bottom pendulum -->
      <body name="3" pos="0 0 -.25">
        <joint name="j3"/>
        <geom fromto="0 0 0 0 0 -.2" rgba="0 0 1 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// Initial angular velocity of the root joint (rad/s).
const ROOT_VEL: f64 = 10.0;
// Tiny perturbation added to one trajectory to demonstrate chaos.
const PERTURBATION: f64 = 1e-5;
// How many steps the chaos analysis runs for.
const CHAOS_STEPS: usize = 10_000;
// Threshold (radians) above which trajectories are considered to have diverged.
const DIVERGE_THRESHOLD: f64 = 1.0;
// Print energy every this many steps during the viewer phase.
const ENERGY_PRINT_INTERVAL: usize = 100;


fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");

    // ------------------------------------------------------------------
    // Phase 1 - Butterfly Effect
    // Run two simulations that start with nearly identical initial
    // conditions and find the step at which their root angles diverge.
    // ------------------------------------------------------------------
    println!("=== Phase 1: Butterfly Effect ===");

    let mut data_a = MjData::new(&model);
    let mut data_b = MjData::new(&model);

    // Obtain the joint info from one of the data instances (offsets are the same
    // for any MjData created from the same model).
    let root_info = data_a.joint("root").expect("joint 'root' not found in model");

    // Set root velocities: identical except for a tiny perturbation.
    root_info.view_mut(&mut data_a).qvel[0] = ROOT_VEL;
    root_info.view_mut(&mut data_b).qvel[0] = ROOT_VEL + PERTURBATION;

    println!(
        "Trajectory A: root_vel = {:.6} rad/s",
        root_info.view(&data_a).qvel[0]
    );
    println!(
        "Trajectory B: root_vel = {:.6} rad/s  (delta = {:+e})",
        root_info.view(&data_b).qvel[0],
        PERTURBATION
    );

    let mut diverge_step = None;

    for step in 0..CHAOS_STEPS {
        data_a.step();
        data_b.step();

        let angle_a = root_info.view(&data_a).qpos[0];
        let angle_b = root_info.view(&data_b).qpos[0];

        if (angle_a - angle_b).abs() > DIVERGE_THRESHOLD && diverge_step.is_none() {
            diverge_step = Some(step);
            println!(
                "Trajectories diverged at step {} (t = {:.3} s), |dangle| = {:.3} rad",
                step,
                data_a.time(),
                (angle_a - angle_b).abs()
            );
            break;
        }
    }

    if diverge_step.is_none() {
        println!(
            "Trajectories did NOT diverge within {CHAOS_STEPS} steps \
             (final |dangle| = {:.3e} rad).",
            (root_info.view(&data_a).qpos[0] - root_info.view(&data_b).qpos[0]).abs()
        );
    }

    // ------------------------------------------------------------------
    // Phase 2 - Energy Tracking + Viewer
    // Run a fresh simulation, printing total mechanical energy every
    // ENERGY_PRINT_INTERVAL steps to confirm near-conservation.
    // ------------------------------------------------------------------
    println!("\n=== Phase 2: Energy Tracking + Viewer ===");
    println!("Opening interactive viewer. Total energy is printed every {ENERGY_PRINT_INTERVAL} steps.");
    println!("Press Ctrl+Q or close the window to stop.");

    let mut data = MjData::new(&model);
    root_info.view_mut(&mut data).qvel[0] = ROOT_VEL;

    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    let mut step_count: usize = 0;

    while viewer.running() {
        data.step();

        // Compute and print energy at regular intervals.
        if step_count % ENERGY_PRINT_INTERVAL == 0 {
            data.energy_pos();  // compute potential energy -> data.energy()[0]
            data.energy_vel();  // compute kinetic energy   -> data.energy()[1]
            let e_pot = data.energy()[0];
            let e_kin = data.energy()[1];
            println!(
                "t = {:6.3} s | E_pot = {:+.4e} | E_kin = {:+.4e} | E_total = {:+.4e}",
                data.time(),
                e_pot,
                e_kin,
                e_pot + e_kin,
            );
        }

        step_count += 1;
        viewer.sync_data(&mut data);
        viewer.render();

        // The timestep is 0.001 s -> no sleep needed to remain near real-time here;
        // rendering itself takes longer than 1 ms per frame.
        std::thread::sleep(Duration::from_secs_f64(model.opt().timestep));
    }

    println!("Simulation finished at t = {:.3} s.", data.time());
}
