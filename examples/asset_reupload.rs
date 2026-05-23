//! Demonstrates runtime GPU asset re-upload in the Rust viewer with multi-threaded simulation.
//!
//! Three types of animated assets are rendered in the same scene:
//!
//! - **Heightfield** -- elevation data is rewritten each frame, creating a propagating
//!   ripple wave across the terrain.
//! - **Texture** -- pixel data is rewritten each frame, producing a shifting colour
//!   gradient across the surface of the ball.
//! - **Mesh** -- vertex Z positions are rewritten each frame, making a flat grid mesh
//!   deform into a wave surface.
//!
//! A ball with a free joint is dropped under gravity onto the animated terrain,
//! bouncing and rolling as the heightfield ripples beneath it.
//!
//! The physics runs on a dedicated thread, maintaining real-time simulation rate.
//! The render loop runs on the main thread, as required by OpenGL.
//!
//! Asset updates are queued via [`mujoco_rs::viewer::ViewerSharedState::update_hfields_from`] and friends,
//! then flushed to the GPU on the next [`MjViewer::render`] call.
//!
//! The accessor methods used here are all safe -- no `unsafe` blocks needed.
//! (Only the mutable `_mut` variants of layout fields like `mesh_vertadr` are
//! `unsafe`; the read-only variants used to initialise layout offsets are not.)
use std::time::Instant;
use std::thread;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;

/// Width and height of the 2-D texture in pixels.
const TEX_SIZE: usize = 64;
/// Number of rows and columns in the heightfield.
const HF_N: usize = 32;
/// Number of quads per edge in the wave mesh (vertex grid is (N+1)^2).
const MESH_N: usize = 10;

fn main() {
    let model = Box::new(create_model());
    let mut data = MjData::new(model);
    data.forward();
    // Small initial push so the ball rolls around the bowl.
    data.qvel_mut()[0] = 1.2; // vx
    data.qvel_mut()[1] = 0.6; // vy

    // Read layout values once -- these fields are read-only and safe.
    // Copied to plain integers so no reference is held across later mutable borrows.
    let tex_adr      = data.model().tex_adr()[0] as usize;
    let tex_nchannel = data.model().tex_nchannel()[0] as usize;
    let mesh_adr     = data.model().mesh_vertadr()[0] as usize;
    let mesh_count   = data.model().mesh_vertnum()[0] as usize;
    let hf_adr       = data.model().hfield_adr()[0] as usize;
    let hf_ncol      = data.model().hfield_ncol()[0] as usize;
    let hf_nrow      = data.model().hfield_nrow()[0] as usize;

    let mut viewer = MjViewer::launch_passive(data.model(), 0).unwrap();
    let state = viewer.state().clone();

    let physics_thread = thread::spawn(move || {
        let mut viewer_running = state.lock().unwrap().running();
        while viewer_running {
            let timer = Instant::now();
            let t = data.time() as f32;

            // SAFETY: asset arrays (hfield_data, tex_data, mesh_vert) are
            // only touched here on the physics thread. No other thread reads
            // them until we push the changes via update_*_from below.
            let m = unsafe { data.model_mut() };

            /* Heightfield: propagating ripple wave */
            // Values must be in [0, 1]; 0 = z_bottom, 1 = z_top (from the `size` parameter).
            let hf = m.hfield_data_mut();
            for row in 0..hf_nrow {
                for col in 0..hf_ncol {
                    let x    = col as f32 / (hf_ncol - 1) as f32 * 2.0 - 1.0;
                    let y    = row as f32 / (hf_nrow - 1) as f32 * 2.0 - 1.0;
                    let dist = (x * x + y * y).sqrt();
                    let s    = ((dist - 0.6) / 0.4).clamp(0.0, 1.0);
                    let wall = s * s * (3.0 - 2.0 * s); // Hermite smooth-step
                    let ripple = (dist * 6.0 - t * 2.0).sin() * 0.25 + 0.3;
                    hf[hf_adr + row * hf_ncol + col] =
                        (wall + (1.0 - wall) * ripple).clamp(0.0, 1.0);
                }
            }

            /* Texture: animated UV colour gradient */
            let tex = m.tex_data_mut();
            for py in 0..TEX_SIZE {
                for px in 0..TEX_SIZE {
                    let u = px as f32 / TEX_SIZE as f32;
                    let v = py as f32 / TEX_SIZE as f32;
                    let base = tex_adr + (py * TEX_SIZE + px) * tex_nchannel;
                    tex[base]     = ((u + t * 0.5).sin() * 127.0 + 128.0) as u8;
                    tex[base + 1] = ((v + t * 0.7).cos() * 127.0 + 128.0) as u8;
                    tex[base + 2] = (t * 0.3).sin().abs().mul_add(200.0, 55.0) as u8;
                    if tex_nchannel == 4 {
                        tex[base + 3] = 255; // alpha
                    }
                }
            }

            /* Mesh: sinusoidal Z deformation based on radial distance + time */
            // `mesh_vert_mut()` returns `&mut [[f32; 3]]`; each entry is one vertex [x, y, z].
            let verts = m.mesh_vert_mut();
            for k in 0..mesh_count {
                let x = verts[mesh_adr + k][0];
                let y = verts[mesh_adr + k][1];
                let dist = (x * x + y * y).sqrt();
                verts[mesh_adr + k][2] = 0.2 * (dist * 5.0 - t * 2.5).sin();
            }

            data.step();
            {
                let mut lock = state.lock().unwrap();
                lock.update_hfields_from(data.model());
                lock.update_textures_from(data.model());
                lock.update_meshes_from(data.model());
                lock.sync_data(&mut data);
                viewer_running = lock.running();
            }

            // Busy-wait to maintain real-time simulation rate.
            while timer.elapsed().as_secs_f64() < data.model().opt().timestep {}
        }
    });

    while viewer.running() {
        viewer.render().unwrap();
    }

    physics_thread.join().unwrap();
}


/// Builds an [`MjModel`] containing a heightfield, a textured ball, and a wave mesh.
fn create_model() -> MjModel {
    let mut spec = MjSpec::new();

    /* Heightfield */
    let hf = spec.add_hfield();
    hf.set_name("terrain").unwrap();
    hf.set_nrow(HF_N as i32);
    hf.set_ncol(HF_N as i32);
    // size = [x_half, y_half, z_top_scale, z_bottom_thickness]
    hf.with_size([4.0, 4.0, 1.0, 0.1]);
    // Flat initial elevation (0.5 = mid-height in [0, 1]).
    // Smooth wall: inner 60% is pure ripple; outer 40% transitions to a full-height wall.
    let initial: Vec<f32> = (0..HF_N)
        .flat_map(|row| {
            (0..HF_N).map(move |col| {
                let x    = col as f32 / (HF_N - 1) as f32 * 2.0 - 1.0;
                let y    = row as f32 / (HF_N - 1) as f32 * 2.0 - 1.0;
                let dist = (x * x + y * y).sqrt();
                let s    = ((dist - 0.6) / 0.4).clamp(0.0, 1.0);
                let wall = s * s * (3.0 - 2.0 * s); // Hermite smooth-step
                    let ripple = (dist * 6.0).sin() * 0.25 + 0.3;
                (wall + (1.0 - wall) * ripple).clamp(0.0, 1.0)
            })
        })
        .collect();
    hf.set_userdata(initial);

    /* Texture + material */
    // Use set_data() to ensure MuJoCo allocates exactly TEX_SIZE^2 x nchannel bytes.
    let tex = spec.add_texture();
    tex.with_name("wave_tex")
        .with_type(MjtTexture::mjTEXTURE_2D)
        .with_width(TEX_SIZE as i32)
        .with_height(TEX_SIZE as i32)
        .with_nchannel(3);
    tex.set_data(&vec![128u8; TEX_SIZE * TEX_SIZE * 3]); // grey initially

    let mat = spec.add_material().with_name("tex_mat");
    mat.set_texture(MjtTextureRole::mjTEXROLE_RGB, "wave_tex");

    /* Wave mesh (MESH_N x MESH_N quads) */
    let n = MESH_N + 1; // vertices per edge
    let verts: Vec<f32> = (0..n)
        .flat_map(|i| {
            (0..n).flat_map(move |j| {
                let x = j as f32 / MESH_N as f32 * 2.0 - 1.0;
                let y = i as f32 / MESH_N as f32 * 2.0 - 1.0;
                // Non-zero initial Z so Qhull can build a valid convex hull.
                let dist = (x * x + y * y).sqrt();
                let z = 0.2 * (dist * 5.0_f32).sin();
                [x, y, z]
            })
        })
        .collect();

    let faces: Vec<i32> = (0..MESH_N)
        .flat_map(|i| {
            (0..MESH_N).flat_map(move |j| {
                let tl = (i * n + j) as i32;
                let bl = ((i + 1) * n + j) as i32;
                let tr = tl + 1;
                let br = bl + 1;
                [tl, bl, tr, bl, br, tr]
            })
        })
        .collect();

    let mesh = spec.add_mesh();
    mesh.set_name("wave").unwrap();
    mesh.set_uservert(&verts);
    mesh.set_userface(&faces);

    /* Scene geometry */
    let world = spec.world_body_mut();

    world
        .add_light()
        .with_pos([0.0, 0.0, 5.0])
        .with_dir([0.0, 0.2, -1.0]);

    world
        .add_geom()
        .with_type(MjtGeom::mjGEOM_HFIELD)
        .with_pos([0.0, 0.0, 0.0])
        .with_hfieldname("terrain");

    // Ball with a free joint: falls under gravity and bounces on the terrain.
    let ball = world.add_body();
    ball.with_name("ball").with_pos([0.0, 0.0, 3.5]);
    ball.add_joint().with_type(MjtJoint::mjJNT_FREE);
    ball.add_geom()
        .with_type(MjtGeom::mjGEOM_SPHERE)
        .with_size([0.3, 0.0, 0.0])
        .with_material("tex_mat");

    world
        .add_body()
        .with_name("wave_body")
        .with_pos([3.0, 0.0, 2.0])
        .add_geom()
        .with_type(MjtGeom::mjGEOM_MESH)
        .with_meshname("wave")
        .with_rgba([0.3, 0.7, 1.0, 1.0]);

    spec.compile().unwrap()
}
