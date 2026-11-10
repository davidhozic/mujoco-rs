//! Example of different control-info windows inside the Viewer's (egui) UI.
//!
//! Open the "Actuator" window in the viewer. Each actuator holds one slider for each of its
//! control inputs, labelled with the input name that MuJoCo reports for the actuator type.
//!
//! Open the "Joint" window in the viewer. Each slider writes one position component of the
//! joint, and a free joint takes a numeric input for each translation component.
//!
//! Open the "Sensor" window in the viewer. Each sensor shows one value for each of its
//! components, labelled with the axis or the quaternion component and the unit of the quantity.
//! The free box carries a touch sensor, an accelerometer and a gyro on its site.
//!
//! Open the "Equality" window in the viewer. Each equality constraint holds one toggle button,
//! which switches the constraint on or off at runtime. The connect constraint starts inactive;
//! switch it on to attach the passive DC motor arm to the driven DC motor arm.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use env_logger::Env;


const EXAMPLE_MODEL: &str = stringify! {
<mujoco model="control_windows">
  // The Newton solver diverges on the gear tooth contacts; PGS stays stable.
  <option integrator="implicitfast" solver="PGS"/>
  <statistic center=".25 0 .3" extent="4"/>

  <asset>
    <mesh name="gear_tooth" builtin="supersphere" params="10 1 1" scale=".0075 .0075 .0175"/>
  </asset>

  <default>
    <geom type="capsule" size=".035" rgba=".7 .7 .75 1"/>
    <joint damping=".2"/>
    <default class="gear">
      <geom type="ellipsoid" size=".0075 .0075 .0175" euler="90 0 0" friction=".1" rgba=".9 .35 .65 1"/>
    </default>
  </default>

  <worldbody>
    <camera
        name="lidar-array" pos="4.604 0.369 1.329" xyaxes="-0.232 0.973 -0.000 -0.761 -0.181 0.622"
        resolution="10 10" fovy="20"
    />

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
      <site name="dcmotor_arm_site" type="box" size="0.06 0.06 0.06" pos="0 0 -0.25" rgba="0 0 0 0"/>
    </body>

    <body name="dcmotor_passive_arm" pos="1.1 0 .4">
      <joint name="dcmotor_passive_hinge" type="hinge" axis="0 1 0"/>
      <geom fromto="0 0 0 0 0 -.25" rgba=".6 .4 .8 1"/>
      <site name="dcmotor_passive_arm_site" type="box" size="0.06 0.06 0.06" pos="0 0 -0.25" rgba="0 0 0 0"/>
    </body>

    <body name="expmap_box" pos="1.8 0 .4">
      <joint name="expmap_ball" type="ball"/>
      <geom type="box" size=".09 .06 .04" rgba=".8 .5 .3 1"/>
    </body>

    <body name="quat_box" pos="2.3 0 .4">
      <joint name="quat_ball" type="ball"/>
      <geom type="box" size=".09 .06 .04" rgba=".4 .5 .9 1"/>
    </body>

    <body name="limited_ball_box" pos="2.8 0 .4">
      <joint name="limited_ball" type="ball" limited="true" range="0 60"/>
      <geom type="box" size=".09 .06 .04" rgba=".5 .8 .5 1"/>
    </body>

    <body name="free_box" pos="3.3 0 .4">
      <freejoint name="free"/>
      <geom type="box" size=".06 .06 .06" rgba=".8 .8 .4 1"/>
      <site name="free_site" type="box" size=".065 .065 .065" rgba="0 0 0 0"/>
    </body>

    <body name="gear1" pos="3.8 0 .4" childclass="gear">
      <joint name="gear_hinge1" type="hinge" axis="0 1 0"/>
      <replicate count="18" euler="0 20 0">
        <geom name="gear1_geom" pos=".09 0 0"/>
      </replicate>
    </body>

    <body name="gear2" pos="3.98 0 .4" euler="0 10 0" childclass="gear">
      <joint name="gear_hinge2" type="hinge" axis="0 1 0"/>
      <replicate count="18" euler="0 20 0">
        <geom name="gear2_geom" pos=".09 0 0"/>
      </replicate>
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

  <sensor>
    <jointpos name="slide_pos" joint="slide"/>
    <jointvel name="slide_vel" joint="slide"/>
    <jointlimitpos name="slide_limit_pos" joint="slide"/>
    <jointpos name="pid_hinge_pos" joint="pid_hinge"/>
    <jointactuatorfrc name="pid_hinge_actfrc" joint="pid_hinge"/>
    <actuatorfrc name="dcmotor_frc" actuator="dcmotor_all"/>
    <actuatorfrc name="orientation_quat_frc" actuator="orientation_quat"/>
    <ballquat name="quat_ball_quat" joint="quat_ball"/>
    <ballangvel name="quat_ball_angvel" joint="quat_ball"/>
    <jointlimitpos name="limited_ball_limit_pos" joint="limited_ball"/>
    <framepos name="free_pos" objtype="body" objname="free_box"/>
    <framequat name="free_quat" objtype="body" objname="free_box"/>
    <framezaxis name="free_zaxis" objtype="body" objname="free_box"/>
    <touch name="free_touch" site="free_site"/>
    <accelerometer name="free_acc" site="free_site"/>
    <gyro name="free_gyro" site="free_site"/>
    <contact name="free_contact" site="free_site" data="found force" num="5"/>
    <tactile geom="gear1_geom" mesh="gear_tooth"/>
    <rangefinder name="rangefinder_numero1" camera="lidar-array" data="dist normal"/>

    // TODO: Uncomment when MuJoCo fixes the bug. There seems to be some issue with wrong replication when using
    // the above <replicate> command.
    // <clock/>
    // <user name="user_sensor" dim="0"/>
  </sensor>

  <equality>
    <connect site1="dcmotor_arm_site" site2="dcmotor_passive_arm_site" solref="0.25 1" active="false"/>
  </equality>
</mujoco>
};

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // (Optional) The hook sends MuJoCo's messages to the `log` crate, instead of the console.
    // SAFETY: no other thread uses MuJoCo yet.
    unsafe { install_logging_hook() };

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
