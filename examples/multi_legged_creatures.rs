//! Six procedurally generated creatures with 3-8 legs and open-loop sinusoidal control.
//!
//! Each creature is built using MjSpec model editing (bodies, joints, actuators).
//! Legs follow the Python tutorial layout: thigh extends radially outward from the torso,
//! shin hangs downward.  Hip joints rotate about the vertical axis; knee joints rotate
//! about the axis tangent to the torso perimeter.
//!
//! Adapted from the MuJoCo mjspec tutorial:
//! https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb

use std::f64::consts::TAU;
use std::time::Duration;
use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_editing::{MjsBody, MjtBuiltin};
use mujoco_rs::viewer::MjViewer;

const TORSO_R: f64   = 0.1;
const LEG_LEN: f64   = 0.1;   // BODY_RADIUS in Python
const SPAWN_Z: f64   = 0.15;
const KP:      f64   = 10.0;

// (creature index, number of legs, spawn x, spawn y)
// Grid matching Python's meshgrid([-0.5, 0, 0.5], [0, 0.5]).
const CREATURES: [(usize, usize, f64, f64); 6] = [
    (0, 3, -0.5, 0.0),
    (1, 4,  0.0, 0.0),
    (2, 5,  0.5, 0.0),
    (3, 6, -0.5, 0.5),
    (4, 7,  0.0, 0.5),
    (5, 8,  0.5, 0.5),
];

// One distinct torso colour per creature - legs share the same colour.
const COLOURS: [[f32; 4]; 6] = [
    [0.8, 0.2, 0.2, 1.0],
    [0.2, 0.8, 0.2, 1.0],
    [0.2, 0.2, 0.8, 1.0],
    [0.8, 0.8, 0.2, 1.0],
    [0.8, 0.2, 0.8, 1.0],
    [0.2, 0.8, 0.8, 1.0],
];

fn hip_name(n: usize, l: usize)  -> String { format!("c{n}_l{l}_hip") }
fn knee_name(n: usize, l: usize) -> String { format!("c{n}_l{l}_knee") }

fn add_leg(torso: &mut MjsBody, n: usize, l: usize, angle: f64, rgba: [f32; 4]) {
    // Hip attachment point on the torso surface.
    let hx = TORSO_R * angle.cos();
    let hy = TORSO_R * angle.sin();

    // The thigh body is rotated about the world Z-axis by `angle`, so its local
    // +X axis points radially outward - matching the Python attach-site approach.
    let thigh = torso
        .add_body()
        .with_name(&format!("c{n}_l{l}_thigh"))
        .with_pos([hx, hy, 0.0]);
    thigh.alt_mut().set_euler(&[0.0, 0.0, angle.to_degrees()]);

    // Hip: hinge about the body's local Z (= world Z after the Z-rotation above).
    thigh.add_joint()
        .with_name(&hip_name(n, l))
        .with_type(MjtJoint::mjJNT_HINGE)
        .with_axis([0.0, 0.0, 1.0]);

    thigh.add_geom()
        .with_type(MjtGeom::mjGEOM_CAPSULE)
        .with_fromto([0.0, 0.0, 0.0, LEG_LEN, 0.0, 0.0])  // extends in local +X
        .with_size([LEG_LEN / 4.0, 0.0, 0.0])
        .with_rgba(rgba);

    // Shin placed at the outer end of the thigh (local [LEG_LEN, 0, 0]).
    let shin = thigh
        .add_body()
        .with_name(&format!("c{n}_l{l}_shin"))
        .with_pos([LEG_LEN, 0.0, 0.0]);

    // Knee: hinge about child-body Y (= world tangent direction - same as Python).
    shin.add_joint()
        .with_name(&knee_name(n, l))
        .with_type(MjtJoint::mjJNT_HINGE)
        .with_axis([0.0, 1.0, 0.0]);

    shin.add_geom()
        .with_type(MjtGeom::mjGEOM_CAPSULE)
        .with_fromto([0.0, 0.0, 0.0, 0.0, 0.0, -LEG_LEN])  // hangs straight down
        .with_size([LEG_LEN / 5.0, 0.0, 0.0])
        .with_rgba(rgba);
}

fn add_creature(spec: &mut MjSpec, n: usize, n_legs: usize, cx: f64, cy: f64) {
    let rgba = COLOURS[n];
    {
        let world = spec.world_body_mut();
        let torso = world
            .add_body()
            .with_name(&format!("c{n}_torso"))
            .with_pos([cx, cy, SPAWN_Z]);

        torso.add_joint().with_type(MjtJoint::mjJNT_FREE);

        torso.add_geom()
            .with_type(MjtGeom::mjGEOM_ELLIPSOID)
            .with_size([TORSO_R, TORSO_R, TORSO_R * 0.5])
            .with_rgba(rgba);

        for l in 0..n_legs {
            add_leg(torso, n, l, TAU * l as f64 / n_legs as f64, rgba);
        }
    }

    // Position actuators for each leg's hip and knee.
    for l in 0..n_legs {
        let hip_act = spec
            .add_actuator()
            .with_name(&hip_name(n, l))
            .with_trntype(MjtTrn::mjTRN_JOINT)
            .with_gaintype(MjtGain::mjGAIN_FIXED)
            .with_biastype(MjtBias::mjBIAS_AFFINE)
            .with_target(&hip_name(n, l));
        hip_act.gainprm_mut()[0] = KP;
        hip_act.biasprm_mut()[1] = -KP;

        let knee_act = spec
            .add_actuator()
            .with_name(&knee_name(n, l))
            .with_trntype(MjtTrn::mjTRN_JOINT)
            .with_gaintype(MjtGain::mjGAIN_FIXED)
            .with_biastype(MjtBias::mjBIAS_AFFINE)
            .with_target(&knee_name(n, l));
        knee_act.gainprm_mut()[0] = KP;
        knee_act.biasprm_mut()[1] = -KP;
    }
}

fn build_scene() -> MjModel {
    // Use XML default to set joint damping = 2, matching the Python example.
    let mut spec = MjSpec::from_xml_string(
        r#"<mujoco>
             <default>
               <joint damping="2"/>
             </default>
           </mujoco>"#
    ).expect("failed to parse base spec");

    spec.option_mut().timestep = 0.002;

    // Checker floor - matching the Python example.
    spec.add_texture()
        .with_name("floor")
        .with_builtin(MjtBuiltin::mjBUILTIN_CHECKER)
        .with_type(MjtTexture::mjTEXTURE_2D)
        .with_width(300)
        .with_height(300)
        .with_rgb1([0.2_f64, 0.3, 0.4])
        .with_rgb2([0.3_f64, 0.4, 0.5]);
    let mat = spec.add_material().with_name("floor");
    mat.append_textures("floor");
    mat.with_texrepeat([5.0_f32, 5.0_f32]);
    mat.with_texuniform(true);
    mat.with_reflectance(0.2_f32);

    spec.world_body_mut().add_geom()
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([2.0, 2.0, 0.1])
        .with_material("floor");

    // Two directional lights as in the Python example.
    let world = spec.world_body_mut();
    world.add_light()
        .with_pos([-2.0, -1.0, 3.0])
        .with_dir([2.0, 1.0, -2.0]);
    world.add_light()
        .with_pos([2.0, -1.0, 3.0])
        .with_dir([-2.0, 1.0, -2.0]);

    for (n, n_legs, cx, cy) in CREATURES {
        add_creature(&mut spec, n, n_legs, cx, cy);
    }

    spec.compile().expect("compile failed")
}

// Open-loop sinusoidal gait: uniform amplitude for all actuators, per Python.
// Python uses `freq * t` (angular frequency in rad/s, not Hz), so freq=5 -> 5 rad/s.
fn apply_ctrl(data: &mut MjData<&MjModel>, phases: &[f64], freq: f64) {
    let t   = data.time();
    let amp = 0.9;
    let ctrl = data.ctrl_mut();
    for i in 0..ctrl.len().min(phases.len()) {
        ctrl[i] = amp * (freq * t + phases[i]).sin();
    }
}

fn main() {
    let model = build_scene();
    let mut data = MjData::new(&model);
    let n_ctrl = data.ctrl_mut().len();
    println!("scene: {} bodies, {} DOFs, {} actuators",
        model.ffi().nbody, model.ffi().nv, n_ctrl);

    // Systematic phase spread: legs within each creature are evenly spaced;
    // hip and knee of the same leg are offset by 90 deg.
    let mut phases = Vec::with_capacity(n_ctrl);
    for &(_, n_legs, _, _) in &CREATURES {
        for l in 0..n_legs {
            let base = TAU * l as f64 / n_legs as f64;
            phases.push(base);                                    // hip
            phases.push(base + std::f64::consts::FRAC_PI_2);     // knee
        }
    }

    let mut viewer = MjViewer::launch_passive(&model, 0).expect("failed to launch viewer");
    let dt = model.opt().timestep;

    while viewer.running() {
        apply_ctrl(&mut data, &phases, 5.0);
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().unwrap();
        std::thread::sleep(Duration::from_secs_f64(dt));
    }
}
