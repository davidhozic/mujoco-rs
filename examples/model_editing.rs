//! This is an example on model editing, which shows how to dynamically
//! construct a model via code.
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


fn main() {
    // Create the model programmatically.
    let model = create_model();

    // Normal simulation with visualization.
    let mut data = model.make_data();
    let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch viewer");
    let timestep = model.opt().timestep;
    while viewer.running() {
        viewer.sync(&mut data);
        data.step();
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
    let mut world = spec.world_body();

    // Create a plane
    let mut plane_geom = world.add_geom()
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([1.0, 1.0, 1.0]);

    plane_geom.alt_mut().set_euler(&[2.0, 0.0, 0.0]);  // two degrees
    // or
    // plane_geom.alt_mut().type_ = MjtOrientation::mjORIENTATION_EULER;
    // plane_geom.alt_mut().euler = [2.0, 0.0, 0.0];

    // Create a ball
    let mut ball_body = world.add_body()
        .with_gravcomp(0.981)  // make it like in space.
        .with_name("ball");

    ball_body.add_geom()
        .with_type(MjtGeom::mjGEOM_SPHERE)
        .with_size([0.020, 0.0, 0.0])
        .with_rgba([1.0, 1.0, 0.0, 1.0]);  // make the ball yellow.

    ball_body.add_joint().with_type(MjtJoint::mjJNT_FREE);

    // Iterate all the sub-bodies of the world body recursively.
    for body in world.body_iter(true) {  // body_iter(recurse: true)
        println!("Sub-body of world: {}", body.name());
    }

    // Iterate all bodies recursively through MjSpec.
    // Also prints out the world body.
    for body in spec.body_iter() {
        println!("Body: {}", body.name());
    }

    // Compile the model (required for saving)
    spec.compile().expect("failed to compile")
}
