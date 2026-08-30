//! This is an example on model editing, which shows how to dynamically
//! construct a model via code.
//! 
//! This example uses the viewer in single-threaded fashion.
use std::time::Duration;

use mujoco_rs::wrappers::mj_editing::MjsBody;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use env_logger::Env;

/// Number of pendulum chains.
const N_CHAINS: usize = 3;
/// Number of links in one chain.
const N_LINKS: usize = 4;
/// Length of one link, in meters.
const LINK_LENGTH: f64 = 0.15;


fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("info,mujoco::=off")).init();
    // (Optional) The hook sends MuJoCo's messages to the `log` crate, instead of the console.
    // SAFETY: no other thread uses MuJoCo yet.
    unsafe { install_logging_hook() };

    // Create the model programmatically.
    let model = create_model();

    // Normal simulation with visualization.
    let mut data = MjData::new(&model);  // or model.make_data()
    let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch viewer");
    let timestep = model.opt().timestep;
    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();
        std::thread::sleep(Duration::from_secs_f64(timestep));
    }
}


/// Creates a [`MjModel`] programmatically.
fn create_model() -> MjModel {
    let mut spec = MjSpec::new();

    // Comment at the top of the model
    spec.set_comment("This is an auto-generated MJCF file.");

    // Set the timestep to 5 ms
    spec.option_mut().timestep = 0.005;

    // A pseudo reference to the world body
    let world = spec.world_body_mut();

    // Create a plane
    let plane_geom = world.add_geom()
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([2.0, 2.0, 1.0]);

    plane_geom.alt_mut().set_euler(&[2.0, 0.0, 0.0]);  // two degrees
    // or
    // plane_geom.alt_mut().type_ = MjtOrientation::mjORIENTATION_EULER;
    // plane_geom.alt_mut().euler = [2.0, 0.0, 0.0];

    // Each chain hangs from the world. A link is a child of the link above it, so one chain is a
    // branch of the body tree. Every chain starts horizontal and swings down under gravity.
    for chain in 0..N_CHAINS {
        let mut parent = world.add_body()
            .with_name(&format!("chain_{chain}"))
            .with_pos([0.0, 0.3 * chain as f64 - 0.3, 1.0]);

        for link in 0..N_LINKS {
            let child = parent.add_body()
                .with_name(&format!("link_{chain}_{link}"))
                .with_pos([LINK_LENGTH, 0.0, 0.0]);

            child.add_joint()
                .with_type(MjtJoint::mjJNT_HINGE)
                .with_axis([0.0, 1.0, 0.0]);

            child.add_geom()
                .with_type(MjtGeom::mjGEOM_CAPSULE)
                .with_size([0.015, 0.0, 0.0])
                .with_fromto([0.0, 0.0, 0.0, LINK_LENGTH, 0.0, 0.0]);

            parent = child;
        }
    }

    // A mutable walk down the body tree. The world body keeps its own geoms, so the plane keeps
    // its default colour.
    for chain in spec.world_body_mut().body_iter_mut() {
        shade_by_depth(chain, 0);
    }

    // A read-only walk over every body of the spec, the world body included.
    for body in spec.body_iter() {
        log::info!("Body: {}", body.name());
    }

    // Compile the model (required for saving)
    spec.compile().expect("failed to compile")
}


/// Colours the geoms of `body` and of its whole subtree by how deep each body sits in the tree.
///
/// A child borrows its parent, so the two are never live together and the walk has to recurse.
fn shade_by_depth(body: &mut MjsBody, depth: usize) {
    let shade = depth as f32 / N_LINKS as f32;
    for geom in body.geom_iter_mut(false) {
        *geom.rgba_mut() = [1.0, shade, 1.0 - shade, 1.0];
    }

    for child in body.body_iter_mut() {
        shade_by_depth(child, depth + 1);
    }
}
