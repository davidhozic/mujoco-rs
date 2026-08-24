//! Example of the actuator controls in the viewer UI.
//! The model holds one actuator of every input flavour: actuators whose type defines no input
//! names (motor, position), actuators whose control block holds several named inputs (pid,
//! dcmotor, orientation), an actuator without a name, an actuator without control inputs, and
//! control ranges that differ between the inputs of one actuator.
//!
//! Open the "Actuator" window in the viewer. Each actuator holds one slider for each of its
//! control inputs, labelled with the input name that MuJoCo reports for the actuator type.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = stringify! {
<mujoco model="actuator_inputs">
  <option integrator="implicitfast"/>
  <statistic center=".25 0 .3" extent="4"/>

  <default>
    <geom type="capsule" size=".035" rgba=".7 .7 .75 1"/>
    <joint damping=".2"/>
  </default>

  <worldbody>
    <light pos="0 -.6 3" dir="0 .2 -1" diffuse=".8 .8 .8"/>
    <geom name="floor" type="plane" size="0 0 .05" rgba=".3 .35 .4 1"/>

    <body name="slider" pos="-1.8 0 .4">
      <joint name="slide" type="slide" axis="1 0 0" range="-.3 .3"/>
      <geom type="box" size=".05 .05 .05" rgba=".9 .3 .2 1"/>
    </body>

    <body name="pid_arm" pos="-1 0 .4">
      <joint name="pid_hinge" type="hinge" axis="0 1 0"/>
      <geom fromto="0 0 0 0 0 -.25" rgba=".2 .8 .4 1"/>
    </body>

    <body name="pid_ff_arm" pos="-.3 0 .4">
      <joint name="pid_ff_hinge" type="hinge" axis="0 1 0"/>
      <geom fromto="0 0 0 0 0 -.25" rgba=".2 .6 .8 1"/>
    </body>

    <body name="dcmotor_arm" pos=".4 0 .4">
      <joint name="dcmotor_hinge" type="hinge" axis="0 1 0"/>
      <geom fromto="0 0 0 0 0 -.25" rgba=".9 .7 .2 1"/>
    </body>

    <body name="dcmotor_passive_arm" pos="1.1 0 .4">
      <joint name="dcmotor_passive_hinge" type="hinge" axis="0 1 0"/>
      <geom fromto="0 0 0 0 0 -.25" rgba=".6 .4 .8 1"/>
    </body>

    <body name="expmap_box" pos="1.8 0 .4">
      <joint name="expmap_ball" type="ball"/>
      <geom type="box" size=".09 .06 .04" rgba=".8 .5 .3 1"/>
    </body>

    <body name="quat_box" pos="2.3 0 .4">
      <joint name="quat_ball" type="ball"/>
      <geom type="box" size=".09 .06 .04" rgba=".4 .5 .9 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="motor_limited" joint="slide" gear="20" ctrlrange="-3 3"/>
    <motor joint="slide" gear="20"/>
    <position name="position_servo" joint="pid_hinge" kp="8" ctrlrange="-1.2 1.2"/>
    <pid name="pid_pos_vel_ff" joint="pid_hinge" kp="8" kv=".5"
         input="pos vel ff" posrange="-1.2 1.2" velrange="-6 6" ffrange="-2 2"/>
    <pid name="pid_ff_only" joint="pid_ff_hinge" kp="8" input="ff" ffrange="-4 4"/>
    <dcmotor name="dcmotor_all" joint="dcmotor_hinge" motorconst="1.0" resistance="1.0"
             input="pos vel ff voltage" controller="10 0 5" ctrlrange="-5 5"/>
    <dcmotor name="dcmotor_passive" joint="dcmotor_passive_hinge" motorconst="1.0" resistance="1.0"
             input="none"/>
    <orientation name="orientation_expmap" joint="expmap_ball" kp="3" dampratio="1" ctrlrange="-3.1416 3.1416"/>
    <orientation name="orientation_quat" joint="quat_ball" input="quat" kp="3" dampratio="1" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
};

fn main() {
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);
    let mut viewer = MjViewer::builder()
        .max_user_geoms(0)
        .build_passive(&model).unwrap();

    let timestep = model.opt().timestep;
    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();

        // Sleep for approximately timestep of seconds.
        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}
