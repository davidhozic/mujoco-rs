//! Demonstrates runtime GPU asset re-upload with the [`MjRenderer`].
//!
//! Three types of animated assets are rendered across multiple frames, each saved as a PNG:
//!
//! - **Heightfield** -- elevation data is rewritten each frame, creating a propagating
//!   ripple wave across the terrain.
//! - **Texture** -- pixel data is rewritten each frame, producing a shifting colour
//!   gradient across the surface of the ball.
//! - **Mesh** -- vertex Z positions are rewritten each frame, making a flat grid mesh
//!   deform into a wave surface.
//!
//! After modifying the raw asset arrays directly on the `MjData`'s embedded model, the
//! changes are uploaded to the GPU via [`MjRenderer::update_hfield_from`],
//! [`MjRenderer::update_texture_from`], and [`MjRenderer::update_mesh_from`] before
//! calling [`MjRenderer::render`].  The output PNGs are written to `./output_renderer_asset_reupload/`.
use std::fs;

use mujoco_rs::renderer::MjRenderer;
use mujoco_rs::prelude::*;

/// Width and height of the 2-D texture in pixels.
const TEX_SIZE: usize = 64;
/// Number of rows and columns in the heightfield.
const HF_N: usize = 32;
/// Number of quads per edge in the wave mesh (vertex grid is (N+1)^2).
const MESH_N: usize = 10;
/// Number of frames to render.
const NUM_FRAMES: usize = 60;
/// Number of frames between renders.
const FRAME_SKIP: usize = 100;
/// Output directory for saved images.
const OUTPUT_DIR: &str = "./output_renderer_asset_reupload/";

fn main() {
    fs::create_dir_all(OUTPUT_DIR).unwrap();

    let model = Box::new(create_model());
    let mut data = MjData::new(model);

    // Read layout values once from the model. These are read-only and do not change.
    // Copied to plain integers so no reference is held across later mutable borrows.
    let tex_adr      = data.model().tex_adr()[0] as usize;
    let tex_nchannel = data.model().tex_nchannel()[0] as usize;
    let mesh_adr     = data.model().mesh_vertadr()[0] as usize;
    let mesh_count   = data.model().mesh_vertnum()[0] as usize;
    let hf_adr       = data.model().hfield_adr()[0] as usize;
    let hf_ncol      = data.model().hfield_ncol()[0] as usize;
    let hf_nrow      = data.model().hfield_nrow()[0] as usize;

    let model = data.model();
    let mut renderer = MjRenderer::builder()
        .width(0)   // 0 = inherit offwidth from model's visual.global settings
        .height(0)  // 0 = inherit offheight from model's visual.global settings
        .build(model)
        .expect("failed to initialise renderer");

    let mut camera = MjvCamera::new_free(model);
    camera.move_(MjtMouse::mjMOUSE_ZOOM, model,-10.0, 0.0, renderer.scene());
    renderer.set_camera(camera);

    let mut frames_processed = 0;
    for i in 0.. {
        if frames_processed >= NUM_FRAMES {
            break;
        }

        data.step();
        renderer.sync_data(&mut data).unwrap();

        if i % FRAME_SKIP != 0 {
            continue;
        }

        let t = data.time() as f32;

        // Mutate the asset arrays on the embedded model.
        //
        // SAFETY: only non-structural asset data arrays are modified; the layout
        // (address / count fields) is unchanged, keeping `MjData` internally consistent.
        let m = unsafe { data.model_mut() };

        /* Heightfield: propagating ripple wave */
        let hf = m.hfield_data_mut();
        for row in 0..hf_nrow {
            for col in 0..hf_ncol {
                let x    = col as f32 / (hf_ncol - 1) as f32 * 2.0 - 1.0;
                let y    = row as f32 / (hf_nrow - 1) as f32 * 2.0 - 1.0;
                let dist = (x * x + y * y).sqrt();
                let s    = ((dist - 0.6) / 0.4).clamp(0.0, 1.0);
                let wall = s * s * (3.0 - 2.0 * s);
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
                    tex[base + 3] = 255;
                }
            }
        }

        /* Mesh: sinusoidal Z deformation based on radial distance + time */
        let verts = m.mesh_vert_mut();
        for k in 0..mesh_count {
            let x = verts[mesh_adr + k][0];
            let y = verts[mesh_adr + k][1];
            let dist = (x * x + y * y).sqrt();
            verts[mesh_adr + k][2] = 0.2 * (dist * 5.0 - t * 2.5).sin();
        }

        // Upload the modified assets to the GPU, then render.
        renderer.update_hfield_from(data.model(), 0).unwrap();
        renderer.update_texture_from(data.model(), 0).unwrap();
        renderer.update_mesh_from(data.model(), 0).unwrap();
        // Alternatively upload all assets at once:
        // renderer.update_hfields_from(data.model()).unwrap();
        // renderer.update_textures_from(data.model()).unwrap();
        // renderer.update_meshes_from(data.model()).unwrap();

        renderer.render().unwrap();
        renderer
            .save_rgb(format!("{OUTPUT_DIR}/frame_{frames_processed:04}.png"))
            .unwrap();

        frames_processed += 1;
    }

    println!("Saved {NUM_FRAMES} frames to {OUTPUT_DIR}");
}

/// Builds an [`MjModel`] containing a heightfield, a textured ball, and a wave mesh.
///
/// The offscreen buffer is sized to 1280×720 via the spec's `visual.global` settings so
/// that callers can construct an [`MjRenderer`] with `width(0).height(0)` to inherit the
/// model's configured resolution.
fn create_model() -> MjModel {
    let mut spec = MjSpec::new();

    // Configure offscreen resolution: 1280 × 720.
    let visual = spec.visual_mut();
    visual.global.offwidth  = 1280;
    visual.global.offheight = 720;

    /* Heightfield */
    let hf = spec.add_hfield();
    hf.set_name("terrain").unwrap();
    hf.set_nrow(HF_N as i32);
    hf.set_ncol(HF_N as i32);
    hf.with_size([4.0, 4.0, 1.0, 0.1]);
    let initial: Vec<f32> = (0..HF_N)
        .flat_map(|row| {
            (0..HF_N).map(move |col| {
                let x    = col as f32 / (HF_N - 1) as f32 * 2.0 - 1.0;
                let y    = row as f32 / (HF_N - 1) as f32 * 2.0 - 1.0;
                let dist = (x * x + y * y).sqrt();
                let s    = ((dist - 0.6) / 0.4).clamp(0.0, 1.0);
                let wall = s * s * (3.0 - 2.0 * s);
                let ripple = (dist * 6.0).sin() * 0.25 + 0.3;
                (wall + (1.0 - wall) * ripple).clamp(0.0, 1.0)
            })
        })
        .collect();
    hf.set_userdata(initial);

    /* Texture + material */
    let tex = spec.add_texture();
    // SAFETY: this is a data-backed texture (no builtin), and nchannel=3 matches the 3-channel
    // data uploaded by `set_data` below, so the buffer sizing is consistent.
    unsafe {
        tex.with_name("wave_tex")
            .with_type(MjtTexture::mjTEXTURE_2D)
            .with_width(TEX_SIZE as i32)
            .with_height(TEX_SIZE as i32)
            .with_nchannel(3);
    }
    tex.set_data(&vec![128u8; TEX_SIZE * TEX_SIZE * 3]);

    let mat = spec.add_material().with_name("tex_mat");
    mat.set_texture(MjtTextureRole::mjTEXROLE_RGB, "wave_tex");

    /* Wave mesh */
    let n = MESH_N + 1;
    let verts: Vec<f32> = (0..n)
        .flat_map(|i| {
            (0..n).flat_map(move |j| {
                let x = j as f32 / MESH_N as f32 * 2.0 - 1.0;
                let y = i as f32 / MESH_N as f32 * 2.0 - 1.0;
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
