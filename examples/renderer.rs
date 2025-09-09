//! This example shows how to use the [`Renderer`](mujoco_rs::renderer::Renderer)
//! struct to obtain images of the simulation.
use mujoco_rs::renderer::MjRenderer;
use mujoco_rs::prelude::*;
use std::fs;


const EXAMPLE_MODEL: &str = "
<mujoco>
    <!--  OpenGL buffer size  -->
    <visual>
    <global offwidth=\"1920\" offheight=\"1080\"/>
    </visual>

    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        <body name=\"ball\">
            <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\" mass=\"1\"/>
            <joint name=\"ball\" type=\"free\"/>
            <site name=\"touch\" size=\".1 .1 .1\" pos=\"0 0 0\" rgba=\"0 0 0 0.0\" type=\"box\"/>
        </body>

        <body pos=\"2 0 2\">
            <geom type=\"box\" size=\"1 1 1\" rgba=\"0 1 1 1\"/>
            <joint name=\"box\" type=\"free\"/>
        </body>

        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"20 0 0\"/>
    </worldbody>

    <sensor>
        <touch name=\"touch\" site=\"touch\"/>
    </sensor>
</mujoco>
";

const OUTPUT_DIRECTORY: &str = "./output_renderer/";
const SAVE_FREQUENCY: u32 = 5;


fn main() {
    /* Create the output directory for saving png files */
    fs::create_dir_all(OUTPUT_DIRECTORY).unwrap();

    /* Model and data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
    let mut data = model.make_data();

    /* Renderer for rendering at 1280x720 px */
    let mut renderer: MjRenderer<1280, 720> = MjRenderer::new(&model, 5).unwrap();

    /* Make a camera that follows the ball */
    let ball_body_id = model.body("ball").unwrap().id;
    let mut camera = MjvCamera::new_tracking(ball_body_id as u32);
    
    /* Zoom out a bit  */
    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0, renderer.scene());

    /* Configure the renderer */
    renderer
        .with_camera(camera);
        // .with_font_scale(MjtFontScale::mjFONTSCALE_100)
        // .with_opts(MjvOption::default())
        // .with_rgb_rendering(true)
        // .with_depth_rendering(true);

    for i in 0..1000 {
        data.step();

        /* Save an image every SAVE_FREQUENCY step */
        if i % SAVE_FREQUENCY == 0 {
            renderer.update_scene(&mut data);
            renderer.render();
            renderer.save_rgb(format!("{OUTPUT_DIRECTORY}/img{i}.png")).unwrap();
        }
    }
}
