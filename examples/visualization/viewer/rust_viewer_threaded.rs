//! Example on how to use the Rust-native viewer ([`MjViewer`]) in a multi-threaded fashion.
//! By that we mean rendering can be done on the main thread, and the physics on another.
use std::time::Instant;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use env_logger::Env;


fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // (Optional) The hook sends MuJoCo's messages to the `log` crate, instead of the console.
    // SAFETY: no other thread uses MuJoCo yet.
    unsafe { install_logging_hook() };

    // Create model and data.
    let model = Box::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"));
    let mut data = MjData::new(model);

    // Create the viewer, bound to the model.
    let mut viewer = MjViewer::builder()
        .max_user_geoms(100)
        .vsync(true)  // let the viewer select the appropriate refresh rate.
        .build_passive(data.model())
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
                lock.sync_model_opt(data.model_opt_mut());
                lock.sync_model_vis(data.model_vis_mut());
                lock.sync_model_stat(data.model_stat_mut());
                // OR
                //  lock.sync_model(unsafe { data.model_mut() });
                viewer_running = lock.running();
            }

            // Use a while loop and polling to wait for accuracy purposes.
            // To increase performance, std::thread::sleep may be used,
            // however that comes at the cost of less accuracy.
            while timer.elapsed().as_secs_f64() < data.model().opt().timestep {}
        }
    });

    while viewer.running() {
        viewer.render().unwrap();
    }

    physics_thread.join().unwrap();
}


const EXAMPLE_MODEL: &str = stringify! {
<mujoco model="ball_trampoline">
  <option solver="CG" integrator="implicitfast"/>

  <statistic center="0 0 0.8" extent="2.2"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <default>
    <default class="rail">
      <geom type="capsule" size="0.035" rgba="0.2 0.2 0.22 1"/>
    </default>
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"/>

    <geom class="rail" fromto="-0.65 -0.65 0.6 0.65 -0.65 0.6"/>
    <geom class="rail" fromto="-0.65 0.65 0.6 0.65 0.65 0.6"/>
    <geom class="rail" fromto="-0.65 -0.65 0.6 -0.65 0.65 0.6"/>
    <geom class="rail" fromto="0.65 -0.65 0.6 0.65 0.65 0.6"/>
    <geom class="rail" fromto="-0.65 -0.65 0.6 -0.65 -0.65 0"/>
    <geom class="rail" fromto="0.65 -0.65 0.6 0.65 -0.65 0"/>
    <geom class="rail" fromto="-0.65 0.65 0.6 -0.65 0.65 0"/>
    <geom class="rail" fromto="0.65 0.65 0.6 0.65 0.65 0"/>

    <flexcomp name="mat" type="grid" count="13 13 1" spacing="0.1 0.1 0.1" pos="0 0 0.6"
              radius="0.015" mass="2" dim="2" rgba="0.15 0.25 0.6 1">
      <edge damping="3"/>
      <elasticity young="1e7" thickness="1e-2" elastic2d="stretch"/>
      <pin gridrange="0 0 12 0 0 12 12 12 0 0 0 12 12 0 12 12"/>
    </flexcomp>

    <body name="ball" pos="0 0 1.6">
      <freejoint/>
      <geom type="sphere" size="0.12" mass="1" rgba="0.9 0.45 0.15 1"/>
    </body>
  </worldbody>
</mujoco>
};
