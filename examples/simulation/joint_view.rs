//! Example of using views.
//! The example shows how to obtain a [`MjJointDataInfo`] and a [`MjActuatorDataInfo`] struct that
//! can be used to create a (temporary) [`MjJointDataView`] or [`MjActuatorDataView`] to
//! corresponding fields in [`MjData`].
//!
//! The model is a tray on a ball joint that an orientation servo rocks, and a ball on a free
//! joint that rolls inside the tray.
//! This example uses the viewer in single-threaded fashion.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use env_logger::Env;


const EXAMPLE_MODEL: &str = r#"
<mujoco model="tilting_tray">
  <option timestep="0.002"/>

  <default>
    <geom solref="0.004 1"/>
  </default>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" ambient=".3 .3 .3"/>
    <geom name="floor" type="plane" size="3 3 .1" rgba=".3 .35 .4 1"/>

    <!-- The tray hangs on a ball joint above its pedestal; the servo tilts it. -->
    <geom name="pedestal" type="cylinder" pos="0 0 .245" size=".06 .245" rgba=".5 .5 .55 1"/>
    <body name="tray" pos="0 0 .52">
      <joint name="tray_joint" type="ball" pos="0 0 -.02" damping=".2"/>
      <geom name="tray_floor" type="box" size=".3 .3 .02" rgba=".2 .4 .9 1"/>
      <geom name="tray_rim_left" type="box" pos="-.28 0 .06" size=".02 .3 .04" rgba=".2 .4 .9 1"/>
      <geom name="tray_rim_right" type="box" pos=".28 0 .06" size=".02 .3 .04" rgba=".2 .4 .9 1"/>
      <geom name="tray_rim_front" type="box" pos="0 -.28 .06" size=".26 .02 .04" rgba=".2 .4 .9 1"/>
      <geom name="tray_rim_back" type="box" pos="0 .28 .06" size=".26 .02 .04" rgba=".2 .4 .9 1"/>
    </body>

    <!-- The ball rolls freely inside the tray. -->
    <body name="ball" pos=".1 0 .62">
      <joint name="ball_joint" type="free"/>
      <geom name="ball_body" type="sphere" size=".05" rgba=".1 .8 .3 1"/>
    </body>
  </worldbody>

  <actuator>
    <orientation name="tray_servo" joint="tray_joint" kp="200" dampratio="1" ctrlrange="-.6 .6"/>
  </actuator>
</mujoco>
"#;

/// Amplitude of the tray rotation target, in radians.
const TILT_AMPLITUDE: f64 = 0.15;

/// Rate of the tray rotation target about the x axis, in radians per second.
const TILT_RATE_X: f64 = 1.5;

/// Rate of the tray rotation target about the y axis, in radians per second.
const TILT_RATE_Y: f64 = 0.9;

/// Formats one group of a view slice as an aligned row of the per-step block.
fn format_row(element: &str, field: &str, quantity: &str, unit: &str, values: &[f64]) -> String {
    let numbers = values.iter()
        // A value that rounds to zero prints without a sign, so that no column shows -0.00.
        .map(|value| if value.abs() < 0.005 { 0.0 } else { *value })
        .map(|value| format!("{value:>6.2}"))
        .collect::<Vec<_>>();
    let unit = format!("[{unit}]");
    format!("  {element:<11} {field:<6} {quantity:<12} {unit:<8} [{} ]", numbers.join(","))
}

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

    /* Create the joint and the actuator infos */
    let ball_info = data.joint("ball_joint").unwrap();
    let tray_info = data.joint("tray_joint").unwrap();
    let tray_servo = data.actuator("tray_servo").unwrap();

    while viewer.running() {
        /* Write the whole control block of the actuator through its view */
        // The orientation actuator takes a target rotation in exponential-map form; the
        // oscillation rocks the tray, so that the ball rolls from rim to rim.
        let time = data.time();
        let tilt_x = TILT_AMPLITUDE * (time * TILT_RATE_X).sin();
        let tilt_y = TILT_AMPLITUDE * (time * TILT_RATE_Y).sin();
        tray_servo.view_mut(&mut data).ctrl.copy_from_slice(&[tilt_x, tilt_y, 0.0]);

        /* Step the simulation and sync the viewer */
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();

        /* Each view slice holds the elements of its own joint or actuator */
        // A free joint holds a position and a unit quaternion in `qpos`, a ball joint holds only
        // the quaternion.
        let ball_joint = ball_info.view(&data);
        let tray_joint = tray_info.view(&data);
        let tray_actuator = tray_servo.view(&data);
        let (position, rotation) = ball_joint.qpos.split_at(3);
        let (linear, angular) = ball_joint.qvel.split_at(3);
        let block = [
            format_row("ball_joint", "qpos",  "pos",         "m",     position),
            format_row("ball_joint", "qpos",  "orientation", "quat",  rotation),
            format_row("ball_joint", "qvel",  "linear",      "m/s",   linear),
            format_row("ball_joint", "qvel",  "angular",     "rad/s", angular),
            format_row("tray_joint", "qpos",  "orientation", "quat",  &tray_joint.qpos[..]),
            format_row("tray_joint", "qvel",  "angular",     "rad/s", &tray_joint.qvel[..]),
            format_row("tray_servo", "ctrl",  "target",      "rad",   &tray_actuator.ctrl[..]),
            format_row("tray_servo", "force", "torque",      "N*m",   &tray_actuator.force[..]),
        ].join("\n");
        log::info!("{block}");

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
