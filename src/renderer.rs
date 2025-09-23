//! Module related to implementation of the [`MjRenderer`].
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_rendering::MjrContext;
use crate::builder_setters;
use crate::prelude::*;

use glfw::{Context, InitError, PWindow, WindowHint};
use bitflags::bitflags;
use png::Encoder;

use std::io::{self, BufWriter, ErrorKind, Write};
use std::fmt::Display;
use std::error::Error;
use std::path::Path;
use std::fs::File;

const RGB_NOT_FOUND_ERR_STR: &str = "RGB rendering is not enabled (renderer.with_rgb_rendering(true))";
const DEPTH_NOT_FOUND_ERR_STR: &str = "depth rendering is not enabled (renderer.with_depth_rendering(true))";
const INVALID_INPUT_SIZE: &str = "the input width and height don't match the renderer's configuration";
const EXTRA_INTERNAL_VISUAL_GEOMS: usize = 100;


/// A builder for [`MjRenderer`].
#[derive(Debug)]
pub struct MjRendererBuilder {
    width: u32,
    height: u32,
    num_visual_internal_geom: u32,
    num_visual_user_geom: u32,
    rgb: bool,
    depth: bool,
    font_scale: MjtFontScale,
    camera: MjvCamera,
    opts: MjvOption,
}

impl MjRendererBuilder {
    /// Create a builder with default configuration.
    /// Defaults are:
    /// - `width` and `height`: use offwidth and offheight of MuJoCo's visual/global settings from the model,
    /// - `num_visual_internal_geom`: 100,
    /// - `num_visual_user_geom`: 0,
    /// - `rgb`: true,
    /// - `depth`: false.
    pub fn new() -> Self {
        Self {
            width: 0, height: 0,
            num_visual_internal_geom: EXTRA_INTERNAL_VISUAL_GEOMS as u32, num_visual_user_geom: 0,
            rgb: true, depth: false, font_scale: MjtFontScale::mjFONTSCALE_100,
            camera: MjvCamera::default(), opts: MjvOption::default()
        }
    }

    builder_setters! {
        width: u32; "
image width.

<div class=\"warning\">

The width must be less or equal to the offscreen buffer width,
which can be configured at the top of the model's XML like so:

```xml
<visual>
    <global offwidth=\"1920\" .../>
</visual>
```

</div>";

        height: u32; "\
image height.

<div class=\"warning\">

The height must be less or equal to the offscreen buffer height,
which can be configured at the top of the model's XML like so:

```xml
<visual>
    <global offheight=\"1080\" .../>
</visual>
```

</div>";

        num_visual_internal_geom: u32; "\
            maximum number of additional visual-only internal geoms to allocate for.
            Note that the total number of geoms in the internal scene will be
            `num_visual_internal_geom` + `num_visual_user_geom`.";

        num_visual_user_geom: u32;      "maximum number of additional visual-only user geoms (drawn by the user).";
        rgb: bool;                      "RGB rendering enabled (true) or disabled (false).";
        depth: bool;                    "depth rendering enabled (true) or disabled (false).";
        font_scale: MjtFontScale;       "font scale of drawn text (with [MjrContext]).";
        camera: MjvCamera;              "camera used for drawing.";
        opts: MjvOption;                "visualization options.";

    }

    /// Builds a [`MjRenderer`].
    pub fn build(self, model: &MjModel) -> Result<MjRenderer<'_>, RendererError> {
        // Assume model's maximum should be used
        let mut height = self.height;
        let mut width = self.width;
        if width == 0 && height == 0 {
            let global = &model.vis().global;
            height = global.offheight as u32;
            width = global.offwidth as u32;
        }

        let mut glfw = glfw::init_no_callbacks()
            .map_err(|err| RendererError::GlfwInitError(err))?;

        // Create window for rendering
        glfw.window_hint(WindowHint::Visible(false));
        let (mut window, _) = match glfw.create_window(
            width, height,
            "", glfw::WindowMode::Windowed
        ) {
            Some(x) => Ok(x),
            None => Err(RendererError::WindowCreationError)
        }?;

        window.make_current();
        glfw.set_swap_interval(glfw::SwapInterval::None);

        // Initialize the rendering context to render to the offscreen buffer.
        let mut context = MjrContext::new(model);
        context.offscreen();

        // The 3D scene for visualization
        let scene = MjvScene::new(
            model,
            model.ffi().ngeom as usize + self.num_visual_internal_geom as usize
            + self.num_visual_user_geom as usize
        );

        let user_scene = MjvScene::new(
            model,
            self.num_visual_user_geom as usize
        );

        // Construct the renderer and create allocated buffers.
        let renderer = MjRenderer {
            scene, user_scene, context, window, model, camera: self.camera, option: self.opts,
            flags: RendererFlags::empty(), rgb: None, depth: None,
            width: width as usize, height: height as usize
        }   // These require special care
            .with_rgb_rendering(self.rgb)
            .with_depth_rendering(self.depth);

        Ok(renderer)
    }
}


impl Default for MjRendererBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// A renderer for rendering 3D scenes.
/// By default, RGB rendering is enabled and depth rendering is disabled.
pub struct MjRenderer<'m> {
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
    rgb: Option<Box<[u8]>>,
    depth: Option<Box<[f32]>>,

    width: usize,
    height: usize,
}

impl<'m> MjRenderer<'m> {
    /// Construct a new renderer.
    /// The `max_geom` parameter
    /// defines how much space will be allocated for additional, user-defined visual-only geoms.
    /// It can thus be set to 0 if no additional geoms will be drawn by the user.
    /// # Scene allocation
    /// The renderer uses two scenes:
    /// - the internal scene: used by the renderer to draw the model's state.
    /// - the user scene: used by the user to add additional geoms to the internal scene
    /// 
    /// The **internal scene** allocates the amount of space needed to fit every pre-existing
    /// model geom + user visual-only geoms + additional visual-only geoms that aren't from the user (e.g., tendons).
    /// By default, the renderer reserves 100 extra geom slots for drawing the additional visual-only geoms.
    /// If that is not enough or it is too much, you can construct [`MjRenderer`] via its builder
    /// ([`MjRenderer::builder`]), which allows more configuration. 
    /// 
    /// <div class="warning">
    /// 
    /// Parameters `width` and `height` must be less or equal to the offscreen buffer size,
    /// which can be configured at the top of the model's XML like so:
    /// 
    /// ```xml
    /// <visual>
    ///    <global offwidth="1920" offheight="1080"/>
    /// </visual>
    /// ```
    /// 
    /// </div>
    pub fn new(model: &'m MjModel, width: usize, height: usize, max_geom: usize) -> Result<Self, RendererError> {
        let mut glfw = glfw::init_no_callbacks()
            .map_err(|err| RendererError::GlfwInitError(err))?;

        /* Create window for rendering */
        glfw.window_hint(WindowHint::Visible(false));
        let (mut _window, _) = match glfw.create_window(width as u32, height as u32, "", glfw::WindowMode::Windowed) {
            Some(x) => Ok(x),
            None => Err(RendererError::WindowCreationError)
        }?;

        _window.make_current();
        glfw.set_swap_interval(glfw::SwapInterval::None);

        /* Initialize the rendering context to render to the offscreen buffer. */
        let mut context = MjrContext::new(model);
        context.offscreen();

        /* The 3D scene for visualization */
        let scene = MjvScene::new(model, model.ffi().ngeom as usize + max_geom + EXTRA_INTERNAL_VISUAL_GEOMS);
        let user_scene = MjvScene::new(model, max_geom);

        let camera = MjvCamera::new_free(model);
        let option = MjvOption::default();

        let mut s = Self {
            scene, user_scene, context, window: _window, model, camera, option,
            flags: RendererFlags::empty(), rgb: None, depth: None,
            width, height
        };

        s = s.with_rgb_rendering(true);
        Ok(s)
    }

    /// Create a [`MjRendererBuilder`] to configure [`MjRenderer`].
    pub fn builder() -> MjRendererBuilder {
        MjRendererBuilder::new()
    }

    /// Return an immutable reference to the internal scene.
    pub fn scene(&self) -> &MjvScene<'m>{
        &self.scene
    }

    /// Return an immutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene(&self) -> &MjvScene<'m>{
        &self.user_scene
    }

    /// Return a mutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<'m>{
        &mut self.user_scene
    }

    /// Return an immutable reference to visualization options.
    pub fn opts(&self) -> &MjvOption {
        &self.option
    }

    /// Return a mutable reference to visualization options.
    pub fn opts_mut(&mut self) -> &mut MjvOption {
        &mut self.option
    }

    /// Return an immutable reference to the camera.
    pub fn camera(&self) -> &MjvCamera {
        &self.camera
    }

    /// Return a mutable reference to the camera.
    pub fn camera_mut(&mut self) -> &mut MjvCamera {
        &mut self.camera
    }

    /// Check if RGB rendering is enabled.
    pub fn rgb_enabled(&self) -> bool {
        self.flags.contains(RendererFlags::RENDER_RGB)
    }

    /// Check if depth rendering is enabled.
    pub fn depth_enabled(&self) -> bool {
        self.flags.contains(RendererFlags::RENDER_DEPTH)
    }

    /// Sets the font size.
    pub fn set_font_scale(&mut self, font_scale: MjtFontScale) {
        self.context.change_font(font_scale);
    }

    /// Update the visualization options and return a reference to self.
    pub fn set_opts(&mut self, options: MjvOption) {
        self.option = options;
    }

    /// Set the camera used for rendering.
    pub fn set_camera(&mut self, camera: MjvCamera)  {
        self.camera = camera;
    }

    /// Enables/disables RGB rendering.
    pub fn set_rgb_rendering(&mut self, enable: bool) {
        self.flags.set(RendererFlags::RENDER_RGB, enable);
        self.rgb = if enable { Some(vec![0; 3 * self.width * self.height].into_boxed_slice()) } else { None } ;
    }

    /// Enables/disables depth rendering.
    pub fn set_depth_rendering(&mut self, enable: bool) {
        self.flags.set(RendererFlags::RENDER_DEPTH, enable);
        self.depth = if enable { Some(vec![0.0; self.width * self.height].into_boxed_slice()) } else { None } ;
    }

    /// Sets the font size. To be used on construction.
    pub fn with_font_scale(mut self, font_scale: MjtFontScale) -> Self {
        self.set_font_scale(font_scale);
        self
    }

    /// Update the visualization options and return a reference to self. To be used on construction.
    pub fn with_opts(mut self, options: MjvOption) -> Self {
        self.set_opts(options);
        self
    }

    /// Set the camera used for rendering. To be used on construction.
    pub fn with_camera(mut self, camera: MjvCamera) -> Self  {
        self.set_camera(camera);
        self
    }

    /// Enables/disables RGB rendering. To be used on construction.
    pub fn with_rgb_rendering(mut self, enable: bool) -> Self {
        self.set_rgb_rendering(enable);
        self
    }

    /// Enables/disables depth rendering. To be used on construction.
    pub fn with_depth_rendering(mut self, enable: bool) -> Self {
        self.set_depth_rendering(enable);
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

    /// Return a flattened RGB image of the scene.
    pub fn rgb_flat(&self) -> Option<&[u8]> {
        self.rgb.as_deref()
    }

    /// Return an RGB image of the scene. This methods accepts two generic parameters <WIDTH, HEIGHT>
    /// that define the shape of the output slice.
    pub fn rgb<const WIDTH: usize, const HEIGHT: usize>(&self) -> io::Result<&[[[u8; 3]; WIDTH]; HEIGHT]> {
        if let Some(flat) = self.rgb_flat() {
            if flat.len() == WIDTH * HEIGHT * 3 {
                let p_shaped = flat.as_ptr() as *const [[[u8; 3]; WIDTH]; HEIGHT];

                // SAFETY: The alignment of the output is the same as the original.
                // The lifetime also matches  'a in &'a self, which prevents data races.
                // Length (number of elements) matches the output's.
                Ok(unsafe { p_shaped.as_ref().unwrap() })
            }
            else {
                Err(io::Error::new(io::ErrorKind::InvalidInput, INVALID_INPUT_SIZE))
            }
        }
        else {
            Err(io::Error::new(io::ErrorKind::NotFound, RGB_NOT_FOUND_ERR_STR))
        }
    }

    /// Return a flattened depth image of the scene.
    pub fn depth_flat(&self) -> Option<&[f32]> {
        self.depth.as_deref()
    }

    /// Return a depth image of the scene. This methods accepts two generic parameters <WIDTH, HEIGHT>
    /// that define the shape of the output slice.
    pub fn depth<const WIDTH: usize, const HEIGHT: usize>(&self) -> io::Result<&[[f32; WIDTH]; HEIGHT]> {
        if let Some(flat) = self.depth_flat() {
            if flat.len() == WIDTH * HEIGHT {
                let p_shaped = flat.as_ptr() as *const [[f32; WIDTH]; HEIGHT];

                // SAFETY: The alignment of the output is the same as the original.
                // The lifetime matches  'a in &'a self, which prevents data races.
                // Length (number of elements) matches the output's.
                Ok(unsafe { p_shaped.as_ref().unwrap() })
            }
            else {
                Err(io::Error::new(io::ErrorKind::InvalidInput, INVALID_INPUT_SIZE))
            }
        }
        else {
            Err(io::Error::new(io::ErrorKind::NotFound, DEPTH_NOT_FOUND_ERR_STR))
        }
    }

    /// Save an RGB image of the scene to a path.
    /// # Errors
    /// - [`ErrorKind::NotFound`] when RGB rendering is disabled,
    /// - other errors related to write.
    pub fn save_rgb<T: AsRef<Path>>(&self, path: T) -> io::Result<()> {
        if let Some(rgb) = &self.rgb {
            let file = File::create(path.as_ref())?;
            let w = BufWriter::new(file);

            let mut encoder = Encoder::new(w, self.width as u32, self.height as u32);
            encoder.set_color(png::ColorType::Rgb);
            encoder.set_depth(png::BitDepth::Eight);
            encoder.set_compression(png::Compression::NoCompression);

            let mut writer = encoder.write_header()?;
            writer.write_image_data(rgb)?;
            Ok(())
        }
        else {
            Err(io::Error::new(ErrorKind::NotFound, RGB_NOT_FOUND_ERR_STR))
        }
    }

    /// Save a depth image of the scene to a path. The image is 16-bit PNG, which
    /// can be converted into depth (distance) data by dividing the grayscale values by
    /// 65535.0 and applying inverse normalization: `depth = min + (grayscale / 65535.0) * (max - min)`.
    /// If `normalize` is `true`, then the data is normalized with min-max normalization.
    /// Use of [`MjRenderer::save_depth_raw`] is recommended if performance is critical, as
    /// it skips PNG encoding and also saves the true depth values directly.
    /// # Returns
    /// An [`Ok`]`((min, max))` is returned, where min and max represent the normalization parameters.
    /// # Errors
    /// - [`ErrorKind::NotFound`] when depth rendering is disabled,
    /// - other errors related to write.
    pub fn save_depth<T: AsRef<Path>>(&self, path: T, normalize: bool) -> io::Result<(f32, f32)> {
        if let Some(depth) = &self.depth {
            let file = File::create(path.as_ref())?;
            let w = BufWriter::new(file);

            let mut encoder = Encoder::new(w, self.width as u32, self.height as u32);
            encoder.set_color(png::ColorType::Grayscale);
            encoder.set_depth(png::BitDepth::Sixteen);
            encoder.set_compression(png::Compression::NoCompression);

            let (norm, min, max) =
            if normalize {
                let max = depth.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                let min = depth.iter().cloned().fold(f32::INFINITY, f32::min);
                (depth.iter().flat_map(|&x| (((x - min) / (max - min) * 65535.0).min(65535.0) as u16).to_be_bytes()).collect::<Box<_>>(), min, max)
            }
            else {
                (depth.iter().flat_map(|&x| ((x * 65535.0).min(65535.0) as u16).to_be_bytes()).collect::<Box<_>>(), 0.0, 1.0)
            };

            let mut writer = encoder.write_header()?;
            writer.write_image_data(&norm)?;
            Ok((min, max))
        }
        else {
            Err(io::Error::new(ErrorKind::NotFound, DEPTH_NOT_FOUND_ERR_STR))
        }
    }

    /// Save the raw depth data to the `path`. The data is encoded
    /// as a sequence of bytes, where groups of four represent a single f32 value.
    /// The lower bytes of individual f32 appear first (low-endianness).
    /// # Errors
    /// - [`ErrorKind::NotFound`] when depth rendering is disabled,
    /// - other errors related to write.
    pub fn save_depth_raw<T: AsRef<Path>>(&self, path: T) -> io::Result<()> {
        if let Some(depth) = &self.depth {
            let file = File::create(path.as_ref())?;
            let mut writer = BufWriter::new(file);

            /* Fast conversion to a byte slice to prioritize performance */
            let p = unsafe { std::slice::from_raw_parts(
                depth.as_ptr() as *const u8,
                std::mem::size_of::<f32>() * depth.len()
            ) };

            writer.write_all(p)?;
            Ok(())
        }
        else {
            Err(io::Error::new(ErrorKind::NotFound, DEPTH_NOT_FOUND_ERR_STR))
        }
    }

    /// Draws the scene to internal arrays.
    /// Use [`MjRenderer::rgb`] or [`MjRenderer::depth`] to obtain the rendered image.
    fn render(&mut self) {
        self.window.make_current();
        let vp = MjrRectangle::new(0, 0, self.width as i32, self.height as i32);
        self.scene.render(&vp, &self.context);

        /* Fully flatten everything */
        let flat_rgb = self.rgb.as_deref_mut();
        let flat_depth = self.depth.as_deref_mut();

        /* Read to whatever is enabled */
        self.context.read_pixels(
            flat_rgb,
            flat_depth,
            &vp
        );

        /* Make depth values be the actual distance in meters */
        if let Some(depth) = self.depth.as_deref_mut() {
            let map = &self.model.vis().map;
            let near = map.znear;
            let far = map.zfar;
            for value in depth {
                let z_ndc = 2.0 * *value - 1.0;
                *value = 2.0 * near * far / (far + near - z_ndc * (far - near));
            }
        }
    }
}


#[derive(Debug)]
pub enum RendererError {
    GlfwInitError(InitError),
    WindowCreationError,
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

