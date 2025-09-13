//! This example shows how to draw custom geoms to [`MjRenderer`].
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
const SAVE_FREQUENCY: u32 = 50;


fn main() {
    /* Create the output directory for saving png files */
    fs::create_dir_all(OUTPUT_DIRECTORY).unwrap();

    /* Model and data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
    let mut data = model.make_data();

    /* Renderer for rendering at 1280x720 px (width x height) */
    let mut renderer: MjRenderer = MjRenderer::new(&model, 1280, 720, 5).unwrap()
        .with_font_scale(MjtFontScale::mjFONTSCALE_100)
        .with_opts(MjvOption::default())
        .with_rgb_rendering(true)
        .with_depth_rendering(true);

    /* Make a camera that follows the ball */
    let ball_body_id = model.body("ball").unwrap().id;
    let mut camera = MjvCamera::new_tracking(ball_body_id as u32);
    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0, renderer.scene());
    renderer = renderer.with_camera(camera);  // zoom-out a bit

    /* Obtain joint info */
    let ball_joint_info = data.joint("ball").unwrap();
    let box_joint_info = data.joint("box").unwrap();

    let mut scene;
    for i in 0..1000 {
        data.step();

        /* Save an image every SAVE_FREQUENCY step */
        if i % SAVE_FREQUENCY == 0 {
            /* Obtain coordinates of the ball and the box */
            let from = ball_joint_info.view(&data).qpos[..3].try_into().unwrap();
            let to = box_joint_info.view(&data).qpos[..3].try_into().unwrap();

            /* Draw a line between the ball and the box */
            scene = renderer.user_scene_mut();
            scene.clear_geom();
            scene.create_geom(
                MjtGeom::mjGEOM_LINE,
                None, None, None,
                Some([1.0, 1.0, 1.0, 1.0])
            ).connect(0.05, from, to);  // adjust size, pos and mat to make the line connect the ball and the box.

            renderer.sync(&mut data);
            renderer.save_rgb(format!("{OUTPUT_DIRECTORY}/img_rgb{i}.png")).unwrap();
            renderer.save_depth(format!("{OUTPUT_DIRECTORY}/img_depth{i}.png"), true).unwrap();
        }
    }
}
