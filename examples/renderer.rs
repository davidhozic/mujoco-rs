//! This example shows how to use the [`MjRenderer`].
//! struct to obtain images of the simulation.
use mujoco_rs::renderer::MjRenderer;
use mujoco_rs::prelude::*;
use std::fs;


const EXAMPLE_MODEL: &str = stringify!(
<mujoco>
    <!--  OpenGL buffer size  -->
    <visual>
    <global offwidth="1920" offheight="1080"/>
    </visual>

    <worldbody>
        <light ambient="0.2 0.2 0.2"/>
        <body name="ball" pos = "0 0 1">
            <geom name="green_sphere" size=".1" rgba="0 1 0 1" mass="1"/>
            <joint name="ball" type="free"/>
            <site name="touch" size=".1 .1 .1" pos="0 0 0" rgba="0 0 0 0.0" type="box"/>
        </body>

        <body pos="2 0 2">
            <geom type="box" size="1 1 1" rgba="1 0 0 1"/>
            <geom type="box" size="0.5 0.5 0.5" rgba="0 1 0 1" pos="0 0 1"/>
            <geom type="box" size="0.25 0.25 0.25" rgba="0 0 1 1" pos="0 0 1.5"/>
            <joint name="box" type="free"/>
        </body>

        <geom name="floor" type="plane" size="10 10 1" euler="10 0 0"/>
    </worldbody>

    <sensor>
        <touch name="touch" site="touch"/>
    </sensor>
</mujoco>
);

const OUTPUT_DIRECTORY: &str = "./output_renderer/";
const SAVE_FREQUENCY: u32 = 50;


fn main() {
    /* Create the output directory for saving png files */
    fs::create_dir_all(OUTPUT_DIRECTORY).unwrap();

    /* Model and data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
    let mut data = MjData::new(&model);  // or model.make_data()

    /* Renderer for rendering at 1280x720 px (width x height) */
    let mut renderer = MjRenderer::builder()
        .width(0).height(0)  // set to width(0) and height(0) to set automatically based on <global offwidth="1920" offheight="1080"/>
        .num_visual_user_geom(5)  // maximum number of visual-only geoms as result of the user
        .num_visual_internal_geom(0)  // maximum number of visual-only geoms not as result of the user
        .font_scale(MjtFontScale::mjFONTSCALE_100)  // scale of the font drawn by OpenGL
        .rgb(true)  // rgb rendering
        .depth(true)  // depth rendering
        .camera(MjvCamera::default())  // default free camera
        .build(&model).expect("failed to initialize the renderer");

    /* Make a camera that follows the ball */
    let ball_body_id = model.body("ball").unwrap().id;
    let mut camera = MjvCamera::new_tracking(ball_body_id as u32);
    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0, renderer.scene());
    renderer.set_camera(camera);  // zoom-out a bit

    for i in 0..1000 {
        data.step();

        /* Save an image every SAVE_FREQUENCY step */
        if i % SAVE_FREQUENCY == 0 {
            renderer.sync(&mut data);
            renderer.save_rgb(format!("{OUTPUT_DIRECTORY}/img_rgb{i}.png")).unwrap();
            let (_min, _max) = renderer.save_depth(format!("{OUTPUT_DIRECTORY}/img_depth{i}.png"), true).unwrap();
        }
    }
}
