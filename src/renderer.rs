//! Module related to implementation of the [`MjRenderer`]
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_rendering::MjrContext;
use crate::prelude::*;

use glfw::{Context, InitError, PWindow, WindowHint};
use png::Encoder;

use std::io::{BufWriter, Write};
use std::fs::File;
use std::path::Path;


pub struct MjRenderer<'m, const WIDTH: usize, const HEIGHT: usize> {
    scene: MjvScene<'m>,
    context: MjrContext,
    model: &'m MjModel,
    camera: MjvCamera,
    option: MjvOption,

    /* Glfw */
    window: PWindow,
}

#[derive(Debug)]
pub enum RendererError {
    GlfwInitError(InitError),
    WindowCreationError
}


impl<'m, const WIDTH: usize, const HEIGHT: usize> MjRenderer<'m, WIDTH, HEIGHT> {
    pub fn new(model: &'m MjModel) -> Result<Self, RendererError> {
        let mut glfw = glfw::init_no_callbacks()
            .map_err(|err| RendererError::GlfwInitError(err))?;

        /* Create window for rendering */
        glfw.window_hint(WindowHint::Visible(false));
        let (mut _window, _) = match glfw.create_window(WIDTH as u32, HEIGHT as u32, "", glfw::WindowMode::Windowed) {
            Some(x) => Ok(x),
            None => Err(RendererError::WindowCreationError)
        }?;

        _window.make_current();
        glfw.set_swap_interval(glfw::SwapInterval::None);

        /* Initialize the rendering context to render to the offscreen buffer. */
        let mut context = MjrContext::new(model);
        context.offscreen();

        /* The 3D scene for visualization */
        let scene = MjvScene::new(model, 1000);

        let camera = MjvCamera::new_free(model);
        let option = MjvOption::default();

        Ok(Self { scene, context, window: _window, model, camera, option })
    }

    /// Returns a reference to the stored scene
    pub fn scene(&self) -> &MjvScene<'m> {
        &self.scene
    }

    /// Returns a mutable reference to the stored scene
    pub fn scene_mut(&mut self) -> &mut MjvScene<'m> {
        &mut self.scene
    }

    /// Sets the font size.
    pub fn with_font_scale(&mut self, font_scale: MjtFontScale) -> &mut Self {
        self.context.change_font(font_scale);
        self
    }

    /// Update the visualization options and return a reference to self.
    pub fn with_opts(&mut self, options: MjvOption) -> &mut Self {
        self.option = options;
        self
    }

    /// Render images using the `camera`.
    pub fn with_camera(&mut self, camera: MjvCamera) -> &mut Self  {
        self.camera = camera;
        self
    }

    /// Update the scene with new data from data.
    pub fn update_scene(&mut self, data: &mut MjData) -> &mut Self {
        let model_data_ptr = unsafe {  data.model().__raw() };
        let bound_model_ptr = unsafe { self.model.__raw() };
        assert_eq!(model_data_ptr, bound_model_ptr, "'data' must be created from the same model as the renderer.");

        self.scene.update(data, &self.option, &MjvPerturb::default(), &mut self.camera);
        self
    }

    /// Draws the scene to an array.
    pub fn render(&mut self) -> &mut Self {
        self.window.make_current();
        let vp = MjrRectangle::new(0, 0, WIDTH as i32, HEIGHT as i32);
        self.scene.render(&vp, &self.context);
        self
    }

    /// Returns an RGB image of the scene.
    pub fn rgb(&self) -> [[[u8; 3]; WIDTH]; HEIGHT] {
        let mut data = [[[0; 3]; WIDTH]; HEIGHT];
        let flat = data.as_flattened_mut().as_flattened_mut();
        let vp = MjrRectangle::new(0, 0, WIDTH as i32, HEIGHT as i32);
        self.context.read_pixels(Some(flat), None, &vp);
        data
    }

    /// Returns a depth image of the scene.
    pub fn depth(&self) -> [[f32; WIDTH]; HEIGHT] {
        let mut data = [[0.0; WIDTH]; HEIGHT];
        let flat = data.as_flattened_mut();
        let vp = MjrRectangle::new(0, 0, WIDTH as i32, HEIGHT as i32);
        self.context.read_pixels(None, Some(flat), &vp);
        data
    }

    /// Save an RGB image of the scene to a path.
    pub fn save_rgb<T: AsRef<Path>>(&self, path: T) {
        let file = File::create(path.as_ref()).unwrap();
        let w = BufWriter::new(file);

        let mut encoder = Encoder::new(w, WIDTH as u32, HEIGHT as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        encoder.set_compression(png::Compression::NoCompression);

        let rgb = self.rgb();
        let mut writer = encoder.write_header().unwrap();
        let slice = unsafe { std::slice::from_raw_parts(rgb.as_ptr() as *const u8, 3 * WIDTH * HEIGHT) };
        writer.write_image_data(slice).unwrap();
    }

    /// Save a depth image of the scene to a path.
    pub fn save_depth<T: AsRef<Path>>(&self, path: T) {
        let file = File::create(path.as_ref()).unwrap();
        let mut writer = BufWriter::new(file);

        let depth = self.depth();
        for item in depth.iter().flatten() {
            writer.write_all(&item.to_le_bytes()).unwrap();
        }
    }

}


/* 
** Don't run any tests as OpenGL hates if anything
** runs outside the main thread.
*/

// #[cfg(test)]
// mod test {
//     use super::*;

//     const MODEL: &str = "
//         <mujoco>
//             <worldbody>
//             </worldbody>
//         </mujoco>
//     ";

//     // #[test]
//     // fn test_update_normal() {
//     //     // let model = MjModel::from_xml_string(MODEL).unwrap();
//     //     // let model2 = MjModel::from_xml_string(MODEL).unwrap();
//     //     // let mut data = model.make_data();
//     //     // let mut renderer = Renderer::new(&model, 720, 1280).unwrap();
        
//     //     // /* Check if scene updates without errors. */
//     //     // renderer.update_scene(&mut data);
//     // }
// }

