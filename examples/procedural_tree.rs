//! Procedural tree using recursive model editing.
//!
//! Builds a branching tree programmatically via MjSpec's body/joint/geom API.
//! Ball joints with spring-stiffness make branches sway under an animated wind gust.
//!
//! Adapted from the MuJoCo mjspec tutorial:
//! https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb

use std::time::Duration;
use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_editing::MjsBody;
use mujoco_rs::mujoco_c::mjtDisableBit_;
use mujoco_rs::viewer::MjViewer;

const MAX_DEPTH:    usize = 4;
const SCALE:        f64   = 0.6;
const NUM_BRANCHES: usize = 5;

// Colors matching the Python example.
const BROWN: [f32; 4] = [0.4, 0.24, 0.0, 1.0];
const GREEN: [f32; 4] = [0.0, 0.7,  0.2, 1.0];

// Five branch directions on a ~45 degree cone (phi = pi/4), evenly spaced in theta.
// sin(pi/4) approx 0.7071;  theta = k * 72 degree.
const DIRS: [[f64; 3]; NUM_BRANCHES] = [
    [ 0.7071, 0.0000, 0.7071],
    [ 0.2190, 0.6756, 0.7071],
    [-0.5729, 0.4163, 0.7071],
    [-0.5729,-0.4163, 0.7071],
    [ 0.2190,-0.6756, 0.7071],
];

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let m = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
    [v[0]/m, v[1]/m, v[2]/m]
}

// Grow a branch as a child of `parent`, attached at height `attach_z` along the
// parent capsule.  Child branches are themselves distributed from 60% to 100% of
// their own length, matching the Python linspace distribution.
fn grow(parent: &mut MjsBody, depth: usize, len: f64, r: f64, dir: [f64; 3], attach_z: f64) {
    let d = normalize(dir);

    let body = parent.add_body().with_pos([0.0, 0.0, attach_z]);
    body.alt_mut().set_z_axis(&d);

    body.add_joint().with_type(MjtJoint::mjJNT_BALL);

    body.add_geom()
        .with_type(MjtGeom::mjGEOM_CAPSULE)
        .with_fromto([0.0, 0.0, 0.0, 0.0, 0.0, len])
        .with_size([r, 0.0, 0.0])
        .with_rgba(BROWN);

    if depth >= MAX_DEPTH {
        // Three flat ellipsoid leaves fanned out at 120 degree intervals around the tip.
        for k in 0..3usize {
            let angle_deg = k as f64 * 120.0;
            let angle_rad = angle_deg.to_radians();
            let lx = 0.45 * len * angle_rad.cos();
            let ly = 0.45 * len * angle_rad.sin();
            // Place leaf just beyond the capsule tip: z = len + r (capsule end + cap radius).
            let leaf = body.add_geom()
                .with_type(MjtGeom::mjGEOM_ELLIPSOID)
                .with_pos([lx, ly, len + r])
                .with_size([0.5 * len, 0.15 * len, 0.01 * len])
                .with_rgba(GREEN);
            leaf.alt_mut().set_euler(&[0.0, 0.0, angle_deg]);
        }
    } else {
        // Attach children at heights linearly spaced from 60% to 100% of this branch.
        for (i, &bdir) in DIRS.iter().enumerate() {
            let h = (0.6 + 0.4 * i as f64 / (NUM_BRANCHES - 1) as f64) * len;
            grow(body, depth + 1, len * SCALE, r * SCALE, bdir, h);
        }
    }
}

fn build_tree() -> MjModel {
    // Start from a spec with joint spring defaults.  MjsJoint doesn't yet expose
    // stiffness/damping setters directly, so we embed them in the default class.
    let mut spec = MjSpec::from_xml_string(
        r#"<mujoco>
             <default>
               <joint springdamper="0.003 0.7"/>
             </default>
           </mujoco>"#
    ).expect("failed to parse base spec");

    let opt = spec.option_mut();
    opt.timestep      = 0.002;
    opt.density       = 1.294;  // air density - required for aerodynamic drag
    opt.wind[0]       = 0.0;    // wind in +x
    opt.disableflags |= mjtDisableBit_::mjDSBL_CONSTRAINT as i32;

    // Give the viewer a sensible default camera framing.
    let stat = spec.stat_mut();
    stat.center = [0.0, 0.0, 0.5];
    stat.extent = 1.0;

    let world = spec.world_body_mut();

    world.add_geom()
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([5.0, 5.0, 0.01]);

    world.add_light()
        .with_pos([0.0, 0.0, 3.0])
        .with_dir([0.0, 0.0, -1.0]);

    let trunk_len = 0.5;
    let trunk_r   = 0.04;

    let trunk = world.add_body().with_name("trunk");
    trunk.add_geom()
        .with_type(MjtGeom::mjGEOM_CAPSULE)
        .with_fromto([0.0, 0.0, 0.0, 0.0, 0.0, trunk_len])
        .with_size([trunk_r, 0.0, 0.0])
        .with_rgba(BROWN);

    // First level of branches: distributed along 60-100% of trunk length.
    for (i, &dir) in DIRS.iter().enumerate() {
        let h = (0.6 + 0.4 * i as f64 / (NUM_BRANCHES - 1) as f64) * trunk_len;
        grow(trunk, 1, trunk_len * SCALE, trunk_r * SCALE, dir, h);
    }

    spec.compile().expect("compile failed")
}

fn main() {
    let model = build_tree();
    let mut data = MjData::new(&model);
    println!("tree: {} bodies, {} DOFs", model.ffi().nbody, model.ffi().nv);

    let mut viewer = MjViewer::launch_passive(&model, 0).expect("failed to launch viewer");
    let dt = model.opt().timestep;

    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();
        std::thread::sleep(Duration::from_secs_f64(dt));
    }
}
