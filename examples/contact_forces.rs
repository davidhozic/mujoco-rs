//! Example of contact force analysis and visualisation.
//!
//! Demonstrates:
//! - Enabling contact-point and contact-force visualisation on an
//!   [`MjRenderer`] via [`MjRenderer::opts_mut`].
//! - Reading the number of detected contacts from [`MjData::ncon`] and the
//!   contact-force vector from [`MjData::contact_force`].
//! - Saving rendered frames as PNG images for offline inspection.
//!
//! The model consists of a combined box-and-sphere body (free-floating) that
//! falls onto a flat floor.  The simulation runs for a fixed number of steps
//! while the renderer periodically saves frames that show:
//!   - red markers at contact points (`mjVIS_CONTACTPOINT`),
//!   - arrows representing contact forces (`mjVIS_CONTACTFORCE`), and
//!   - a transparent body so that the contact markers inside remain visible
//!     (`mjVIS_TRANSPARENT`).
//!
//! Rendered images are written to `./output_contact_forces/`.
//!
//! Based on the MuJoCo Python tutorial:
//! <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb>

use std::fs;

use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

// ---------------------------------------------------------------------------
// Model XML
// ---------------------------------------------------------------------------
// The body has:
//   - a red box (0.1 m half-size in each direction), and
//   - a green sphere (0.06 m radius) offset from the box centre.
// They share a freejoint so the body falls and bounces freely.
// The floor is a static infinity-plane with a checker-board texture.
//
// <visual><scale> sets the thickness of contact-point and force-arrow glyphs.
const EXAMPLE_MODEL: &str = r#"
<mujoco model="contact forces">
  <visual>
    <global offwidth="1280" offheight="720"/>
    <scale contactwidth="0.05" contactheight="0.05" forcewidth="0.02"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="1 1"
              texuniform="true" reflectance=".2"/>
  </asset>

  <worldbody>
    <light name="top" pos="0 0 2"/>
    <geom name="floor" type="plane" size="0 0 .05" material="grid"/>

    <!--
      The body is rotated 30 deg around z so the box lands on a corner,
      producing multiple simultaneous contact points and making
      the force visualisation more interesting.
    -->
    <body name="box_and_sphere" pos="0 0 0.5" euler="0 0 -30">
      <freejoint/>
      <geom name="red_box"     type="box"    size=".1 .1 .1"  rgba="1 0 0 1"/>
      <geom name="green_sphere" type="sphere" size=".06" pos=".1 .1 .1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Number of simulation steps to run.
const SIM_STEPS: usize = 600;
/// Save a rendered frame every this many steps.
const SAVE_EVERY: usize = 50;
/// Output directory for PNG images.
const OUTPUT_DIR: &str = "./output_contact_forces/";

fn main() {
    // Create output directory.
    fs::create_dir_all(OUTPUT_DIR).expect("failed to create output directory");

    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = MjData::new(&model);

    // Run mj_forward once so that positions and contact detection are
    // initialised before the first render.
    data.forward();

    // Build an off-screen renderer at 1280x720.
    // Width/height 0 picks up the values from `<global offwidth/offheight>`.
    let mut renderer = MjRenderer::builder()
        .width(0)
        .height(0)
        .rgb(true)
        .depth(false)
        .build(&model)
        .expect("failed to build renderer");

    // ------------------------------------------------------------------
    // Enable contact visualisation on the renderer.
    //
    // MjtVisFlag values used:
    //   mjVIS_CONTACTPOINT = 14  -> red dots at contact points
    //   mjVIS_CONTACTFORCE = 16  -> arrows showing contact force direction
    //   mjVIS_TRANSPARENT  = 18  -> make geoms semi-transparent so that
    //                               markers inside them remain visible
    // ------------------------------------------------------------------
    {
        let flags = &mut renderer.opts_mut().flags;
        flags[MjtVisFlag::mjVIS_CONTACTPOINT as usize] = 1;
        flags[MjtVisFlag::mjVIS_CONTACTFORCE  as usize] = 1;
        flags[MjtVisFlag::mjVIS_TRANSPARENT   as usize] = 1;
    }

    // Position the camera so we can see the whole scene.
    let mut camera = MjvCamera::new_free(&model);
    // Zoom out a little and elevate the viewpoint.
    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -0.5, renderer.scene());
    camera.move_(MjtMouse::mjMOUSE_ROTATE_V, &model, 0.0, -0.1, renderer.scene());
    renderer.set_camera(camera);

    println!("Running {SIM_STEPS} simulation steps.");
    println!("Saving a frame every {SAVE_EVERY} steps to '{OUTPUT_DIR}'.");
    println!("{:<8} {:<10} {}", "Step", "ncon", "Contact forces (norm)");

    for step in 0..SIM_STEPS {
        data.step();

        // ---- Print contact information ----------------------------------------
        let ncon = data.ncon() as usize;

        if ncon > 0 {
            let force_strs: Vec<String> = (0..ncon)
                .map(|j| {
                    let force = data.contact_force(j);
                    // The first 3 components are the force (normal + 2 tangential);
                    // the last 3 are the torque.  We report the force magnitude.
                    let norm = (force[0] * force[0]
                        + force[1] * force[1]
                        + force[2] * force[2])
                        .sqrt();
                    format!("{norm:.2}")
                })
                .collect();
            println!(
                "{:<8} {:<10} [{}]",
                step,
                ncon,
                force_strs.join(", ")
            );
        }

        // ---- Render and save frame -------------------------------------------
        if step % SAVE_EVERY == 0 {
            renderer.sync(&mut data);
            let path = format!("{OUTPUT_DIR}/frame_{step:04}.png");
            renderer.save_rgb(&path).expect("failed to save RGB image");
            println!("  -> saved {path}");
        }
    }

    println!("Done. Images are in '{OUTPUT_DIR}'.");
}
