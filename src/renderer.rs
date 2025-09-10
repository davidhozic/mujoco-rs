//! Module related to implementation of the [`MjRenderer`]
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_rendering::MjrContext;
use crate::util::box_zeroed;
use crate::prelude::*;

use glfw::{Context, InitError, PWindow, WindowHint};
use bitflags::bitflags;
use png::Encoder;

use std::io::{self, BufWriter, ErrorKind, Write};
use std::fmt::Display;
use std::error::Error;
use std::path::Path;
use std::fs::File;


/// A renderer for rendering 3D scenes.
/// By default, RGB rendering is enabled and depth rendering is disabled.
pub struct MjRenderer<'m, const WIDTH: usize, const HEIGHT: usize> {
    scene: MjvScene<'m>,
    user_scene: MjvScene<'m>,
    context: MjrContext,
    model: &'m MjModel,

    /* Glfw */
    window: PWindow,

    /* Configuration */
    camera: MjvCamera,
    option: MjvOption,
    flags: RendererFlags,

    /* Storage */
    // Use Box to allow less space to be used
    // when rgb or depth rendering is disabled
    rgb: Option<Box<[[[u8; 3]; WIDTH]; HEIGHT]>>,
    depth: Option<Box<[[f32; WIDTH]; HEIGHT]>>,
}

impl<'m, const WIDTH: usize, const HEIGHT: usize> MjRenderer<'m, WIDTH, HEIGHT> {
    /// Construct a new renderer.
    /// `max_geom` represents the maximum number of geoms the [`MjvScene`] can hold, which
    /// includes both the user-drawn geoms and the required from [`MjData`] state.
    pub fn new(model: &'m MjModel, max_geom: usize) -> Result<Self, RendererError> {
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
        let scene = MjvScene::new(model, model.ffi().ngeom as usize + max_geom + EXTRA_SCENE_GEOM_SPACE);
        let user_scene = MjvScene::new(model, max_geom);

        let camera = MjvCamera::new_free(model);
        let option = MjvOption::default();

        let mut s = Self {
            scene, user_scene, context, window: _window, model, camera, option,
            flags: RendererFlags::empty(), rgb: None, depth: None
        };

        s = s.with_rgb_rendering(true);
        Ok(s)
    }

    /// Enables/disables RGB rendering. To be used on construction.
    pub fn with_rgb_rendering(mut self, enable: bool) -> Self {
        // 1. Define the type we want to allocate
        self.flags.set(RendererFlags::RENDER_RGB, enable);
        self.rgb = if enable { Some(box_zeroed()) } else { None } ;
        self
    }

    /// Enables/disables depth rendering. To be used on construction.
    pub fn with_depth_rendering(mut self, enable: bool) -> Self {
        self.flags.set(RendererFlags::RENDER_DEPTH, enable);
        self.depth = if enable { Some(box_zeroed()) } else { None } ;
        self
    }

    /// Returns an immutable reference to the internal scene. This can be used to control the camera
    /// when the camera is returned via [`MjRenderer::camera`].
    pub fn scene(&self) -> &MjvScene<'m>{
        &self.user_scene
    }

    /// Returns an immutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene(&self) -> &MjvScene<'m>{
        &self.user_scene
    }

    /// Returns a mutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<'m>{
        &mut self.user_scene
    }

    /// Sets the font size. To be used on construction.
    pub fn with_font_scale(mut self, font_scale: MjtFontScale) -> Self {
        self.context.change_font(font_scale);
        self
    }

    /// Update the visualization options and return a reference to self. To be used on construction.
    pub fn with_opts(mut self, options: MjvOption) -> Self {
        self.option = options;
        self
    }

    /// Render images using the `camera`. To be used on construction.
    pub fn with_camera(mut self, camera: MjvCamera) -> Self  {
        self.camera = camera;
        self
    }

    /// Update the scene with new data from data.
    pub fn sync(&mut self, data: &mut MjData) {
        let model_data_ptr = unsafe {  data.model().__raw() };
        let bound_model_ptr = unsafe { self.model.__raw() };
        assert_eq!(model_data_ptr, bound_model_ptr, "'data' must be created from the same model as the renderer.");

        self.scene.update(data, &self.option, &MjvPerturb::default(), &mut self.camera);

        /* Draw user scene geoms */
        sync_geoms(&self.user_scene, &mut self.scene)
            .expect("could not sync the user scene with the internal scene; this is a bug, please report it.");

        self.render();
    }

    /// Draws the scene to internal arrays.
    /// Use [`MjRenderer::rgb`] or [`MjRenderer::depth`] to obtain the rendered image image.
    fn render(&mut self) {
        self.window.make_current();
        let vp = MjrRectangle::new(0, 0, WIDTH as i32, HEIGHT as i32);
        self.scene.render(&vp, &self.context);

        /* Fully flatten everything */
        let flat_rgb = self.rgb.as_deref_mut().map(|x| x.as_flattened_mut().as_flattened_mut());
        let flat_depth = self.depth.as_deref_mut().map(|b| b.as_flattened_mut());

        /* Read to whatever is enabled */
        self.context.read_pixels(
            flat_rgb,
            flat_depth,
            &vp
        );
    }

    /// Returns an RGB image of the scene.
    pub fn rgb(&self) -> Option<&[[[u8; 3]; WIDTH]; HEIGHT]> {
        self.rgb.as_deref()
    }

    /// Returns a depth image of the scene.
    pub fn depth(&self) -> Option<&[[f32; WIDTH]; HEIGHT]> {
        self.depth.as_deref()
    }

    /// Save an RGB image of the scene to a path.
    /// # Errors
    /// - [`ErrorKind::InvalidInput`] when RGB rendering is disabled,
    /// - other errors related to write.
    pub fn save_rgb<T: AsRef<Path>>(&self, path: T) -> io::Result<()> {
        if let Some(rgb) = &self.rgb {
            let file = File::create(path.as_ref())?;
            let w = BufWriter::new(file);

            let mut encoder = Encoder::new(w, WIDTH as u32, HEIGHT as u32);
            encoder.set_color(png::ColorType::Rgb);
            encoder.set_depth(png::BitDepth::Eight);
            encoder.set_compression(png::Compression::NoCompression);

            let mut writer = encoder.write_header()?;
            let slice = rgb.as_flattened().as_flattened();
            writer.write_image_data(slice)?;
            Ok(())
        }
        else {
            Err(io::Error::new(ErrorKind::InvalidInput, "RGB rendering is not enabled (render.with_rgb_rendering(true))"))
        }
    }

    /// Save a depth image of the scene to a path.
    /// # Errors
    /// - [`ErrorKind::InvalidInput`] when depth rendering is disabled,
    /// - other errors related to write.
    pub fn save_depth<T: AsRef<Path>>(&self, path: T) -> io::Result<()> {
        if let Some(depth) = &self.depth {
            let file = File::create(path.as_ref())?;
            let mut writer = BufWriter::new(file);
            for item in depth.iter().flatten() {
                writer.write_all(&item.to_le_bytes())?;
            }
            Ok(())
        }
        else {
            Err(io::Error::new(ErrorKind::InvalidInput, "depth rendering is not enabled (render.with_depth_rendering(true))"))
        }
    }
}


#[derive(Debug)]
pub enum RendererError {
    GlfwInitError(InitError),
    WindowCreationError
}

impl Display for RendererError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GlfwInitError(e) => write!(f, "glfw failed to initialize: {}", e),
            Self::WindowCreationError => write!(f, "failed to create window"),
        }
    }
}

impl Error for RendererError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::GlfwInitError(e) => Some(e),
            _ => None
        }
    }
}

bitflags! { 
    /// Flags that enable features of the renderer.
    struct RendererFlags: u8 {
        const RENDER_RGB = 1 << 0;
        const RENDER_DEPTH = 1 << 1;
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

