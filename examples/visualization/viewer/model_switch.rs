//! Example demonstrating model synchronization in the viewer.
//! Two different models are created; the simulation alternates between
//! stepping and displaying each one every 5 seconds.
//!
//! The viewer automatically re-initializes its internal state whenever
//! [`ViewerSharedState::sync_data`](mujoco_rs::viewer::ViewerSharedState::sync_data)
//! is called with data backed by a new model.
use std::time::{Duration, Instant};
use std::sync::Arc;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


/// A free-falling sphere with a vertical thruster.
const MODEL_BALL: &str = stringify! {
<mujoco>
  <worldbody>
    <light ambient="0.3 0.3 0.3" pos="0 0 3"/>
    <geom name="floor" type="plane" size="10 10 1"/>
    <body name="ball" pos="0 0 1">
      <geom name="sphere" type="sphere" size="0.15" rgba="0.2 0.6 1 1" mass="1"/>
      <joint name="ball_free" type="free" ref="50"/>
      <camera name="Hello" pos="0 0 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="thruster_x" joint="ball_free" gear="1 0 0 0 0 0" ctrlrange="-10 10"/>
    <motor name="thruster_z" joint="ball_free" gear="0 0 1 0 0 0" ctrlrange="-10 10"/>
  </actuator>
</mujoco>
};

/// A double pendulum with motors on both joints.
const MODEL_PENDULUM: &str = stringify! {
<mujoco>
  <worldbody>
    <light ambient="0.3 0.3 0.3" pos="0 0 3"/>
    <body name="pivot" pos="0 0 1.5">
      <geom type="sphere" size="0.05" rgba="0.8 0.8 0.8 1"/>
      <joint name="hinge1" type="hinge" axis="0 1 0"/>
      <body name="arm1" pos="0 0 -0.5">
        <geom type="capsule" size="0.04" fromto="0 0 0.5 0 0 -0.5" rgba="1 0.4 0.2 1" mass="1"/>
        <body name="arm2" pos="0 0 -0.5">
          <geom type="capsule" size="0.04" fromto="0 0 0.5 0 0 -0.5" rgba="0.2 1 0.4 1" mass="1"/>
          <joint name="hinge2" type="hinge" axis="0 1 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" ctrlrange="-5 5" ctrllimited="true"/>
    <motor name="motor2" joint="hinge2" ctrlrange="-5 5" ctrllimited="true"/>
  </actuator>
</mujoco>
};

const SWITCH_INTERVAL: Duration = Duration::from_secs(5);

fn main() {
    let model_ball = Arc::new(
        MjModel::from_xml_string(MODEL_BALL).expect("failed to load ball model"),
    );
    let model_pendulum = Arc::new(
        MjModel::from_xml_string(MODEL_PENDULUM).expect("failed to load pendulum model"),
    );


    let mut data_ball = MjData::new(model_ball.clone());
    let mut data_pendulum = MjData::new(model_pendulum.clone());

    // Give the pendulum a small initial push so it swings chaotically.
    let hinge1_info = data_pendulum.joint("hinge1").unwrap();
    hinge1_info.view_mut(&mut data_pendulum).qvel[0] = 1.5;
    let hinge2_info = data_pendulum.joint("hinge2").unwrap();
    hinge2_info.view_mut(&mut data_pendulum).qvel[0] = 0.5;

    let mut viewer = MjViewer::builder()
        .max_user_geoms(0)
        .vsync(true)
        .build_passive(model_ball.clone())
        .expect("could not launch the viewer");

    let shared_state = viewer.state().clone();

    let physics_thread = std::thread::spawn(move || {
        let mut use_ball = true;
        let mut last_switch = Instant::now();

        loop {
            let frame_start = Instant::now();
            let timestep;

            {
                let mut lock = shared_state.lock().unwrap();
                if !lock.running() {
                    break;
                }

                if use_ball {
                    data_ball.step();
                    lock.sync_data(&mut data_ball);
                    timestep = model_ball.opt().timestep;
                } else {
                    data_pendulum.step();
                    lock.sync_data(&mut data_pendulum);
                    timestep = model_pendulum.opt().timestep;
                }
            }

            if last_switch.elapsed() >= SWITCH_INTERVAL {
                use_ball = !use_ball;
                last_switch = Instant::now();
                println!(
                    "Switched to: {}",
                    if use_ball { "ball" } else { "pendulum" }
                );
            }

            // Busy-wait for the remainder of the timestep for timing accuracy.
            while frame_start.elapsed().as_secs_f64() < timestep {}
        }
    });

    while viewer.running() {
        viewer.render().unwrap();
    }

    physics_thread.join().unwrap();
}
