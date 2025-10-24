//! The example shows how to procedurally generate terrain.
//! It generates a terrain of stair regions.
//! The example is adapted from the MuJoCo Python's model editing tutorial:
//! https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=P2F0ObC2T8w8&line=40&uniqifier=1
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;
use std::time::Instant;


fn main() {   
    const FPS: f64 = 60.0;  // the refresh rate of the viewer
    const SYNC_TIME_MS: u128 = (1000.0 / FPS) as u128;  //  time to wait before updating the viewer (1 / FPS)

    // Create the model programmatically.
    let model = create_model();

    // Normal simulation with visualization.
    let mut data = MjData::new(&model);  // or model.make_data()
    let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch viewer");
    let timestep = model.opt().timestep;
    let mut timer = Instant::now();  // for stepping data
    let mut timer_refresh = Instant::now();  // for syncing viewer

    // Main loop
    while viewer.running() {

        // The timestep is quite small, thus it makes sense to refresh at a lower pace.
        // Syncs are also expensive in this example.
        if timer_refresh.elapsed().as_millis() > SYNC_TIME_MS {
            timer_refresh = Instant::now();
            viewer.sync(&mut data);
        }

        // Step data in realtime
        if timer.elapsed().as_secs_f64() > timestep {  // don't use sleep as it's not accurate enough
            timer = Instant::now();
            data.step();
        }
    }
}


/// Creates a [`MjModel`] programmatically.
fn create_model() -> MjModel {
    let mut spec = MjSpec::new();
    spec.compiler_mut().set_degree(false);

    // Set the timestep to 0.5 ms
    spec.option_mut().timestep = 0.0005;  

    let timeconst = 2.0 * spec.option().timestep;
    // Change the default stiffness parameters and geom type
    let main_class = spec.default_mut("main").unwrap();
    main_class.geom_mut()
        .with_solref([timeconst, 0.5])
        .with_type(MjtGeom::mjGEOM_BOX);


    // Create lights
    let world = spec.world_body_mut();
    for x in (-5..5).step_by(5) {
        for y in (-5..5).step_by(5) {
            world.add_light().with_pos([x as f64, y as f64, 40.0]).with_dir([-x as f64, -y as f64, -15.0]);
        }
    }

    connected_spheres(&mut spec);

    // Create multiple stairs in a grid
    let base_name = "stairs".to_string();
    let mut direction;
    for i in -2..2 {
        for j in -2..2 {
            direction = if rand::random_bool(0.5) { 1 } else { -1 };

            // Generate each step
            stairs(&mut spec, [i as f64 * 2.0 * 2.0, j as f64 * 2.0 * 2.0], 4, direction, &format!("{base_name}{i}_{j}"));
        }
    }

    // Compile the model
    spec.compile().expect("failed to compile")
}



/// Generates a body of stair shape.
/// This is adapted from MuJoCo Python's model editing tutorial:
/// https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=dtFWSSwWSgeD&line=1&uniqifier=1
fn stairs(spec: &mut MjSpec, grid_loc: [f64; 2] , num_stairs: u32, direction: i8, name: &str) {
    const SQUARE_LENGTH: f64 = 2.0;
    const V_SIZE: f64 = 0.076;
    const H_SIZE: f64 = 0.12;
    const H_STEP: f64 = H_SIZE * 2.0;
    const V_STEP: f64 = V_SIZE * 2.0;
    const BROWN: [f32; 4] = [0.460, 0.362, 0.216, 1.0];

    let body_pos = [grid_loc[0], grid_loc[1], 0.0];
    let world = spec.world_body_mut();
    let body = world.add_body().with_pos(body_pos).with_name(name);
    // Offset
    let [x_beginning, y_end] = [-SQUARE_LENGTH + H_SIZE; 2];
    let [x_end, y_beginning] = [SQUARE_LENGTH - H_SIZE; 2];
    // Dimension
    let mut size_one = [H_SIZE, SQUARE_LENGTH, V_SIZE];
    let mut size_two =  [SQUARE_LENGTH, H_SIZE, V_SIZE];
    // Geoms positions
    let mut x_pos_l = [x_beginning, 0.0, direction as f64 * V_SIZE];
    let mut x_pos_r = [x_end, 0.0, direction as f64 * V_SIZE];
    let mut y_pos_up = [0.0, y_beginning, direction as f64 * V_SIZE];
    let mut y_pos_down = [0.0, y_end, direction as f64 * V_SIZE];

    for i in 0..num_stairs {
        size_one[1] = SQUARE_LENGTH - H_STEP * i as f64 - H_STEP;
        size_two[0] = SQUARE_LENGTH - H_STEP * i as f64 ;
        [x_pos_l[2], x_pos_r[2], y_pos_up[2], y_pos_down[2]]  = [direction as f64 * ( V_SIZE + V_STEP * i as f64); 4];
        
        // Left side
        x_pos_l[0] = x_beginning + H_STEP * i as f64;
        body.add_geom().with_rgba(BROWN).with_size(size_one).with_pos(x_pos_l);
        // Right side
        x_pos_r[0] = x_end - H_STEP * i as f64;
        body.add_geom().with_rgba(BROWN).with_size(size_one).with_pos(x_pos_r);
        // Top
        y_pos_up[1] = y_beginning - H_STEP * i as f64;
        body.add_geom().with_rgba(BROWN).with_size(size_two).with_pos(y_pos_up);
        // Bottom
        y_pos_down[1] = y_end + H_STEP * i as f64;
        body.add_geom().with_rgba(BROWN).with_size(size_two).with_pos(y_pos_down);
    }

    // Closing
    let size = [SQUARE_LENGTH - H_STEP * num_stairs as f64, SQUARE_LENGTH - H_STEP * num_stairs as f64, V_SIZE];
    let pos = [0.0, 0.0, direction as f64 * (V_SIZE + V_STEP * num_stairs as f64)];
    body.add_geom().with_pos(pos).with_size(size).with_rgba(BROWN);
}


/// Creates an array of spheres, connected by tendons.
fn connected_spheres(spec: &mut MjSpec) {
    const GRID_LIMIT: i32 = 3;

    // Create some spheres
    // Connect the first ball with the last ball.
    let mut last_site_name = format!("ball_{}_{}", GRID_LIMIT-1, GRID_LIMIT-1);
    for i in -GRID_LIMIT..GRID_LIMIT {
        for j in -GRID_LIMIT..GRID_LIMIT {
            let site_name = format!("ball_{i}_{j}");

            // Connect spheres with a tendon
            let tendon = spec.add_tendon()
                .with_range([0.0, 2.0]);

            tendon.wrap_site(&site_name);  // one part of tendon attached to the new ball
            tendon.wrap_site(&last_site_name);  // second part of tendon attached to the previous ball

            // Create a sphere
            let world = spec.world_body_mut();
            let ball = world.add_body()
                .with_pos([i as f64 * 0.5, j as f64 * 0.5, 5.0]);
            ball.add_geom()
                .with_type(MjtGeom::mjGEOM_SPHERE)
                .with_size([0.1, 0.0, 0.0])
                .with_mass(0.001);

            ball.add_joint().with_type(MjtJoint::mjJNT_FREE);
            ball.add_site().with_name(&site_name);
            last_site_name = site_name;
        }
    }
}
