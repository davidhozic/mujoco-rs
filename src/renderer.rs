//! Module related to implementation of the [`MjRenderer`].
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_rendering::MjrContext;

#[cfg(target_os = "linux")]
use crate::renderer::egl::GlStateEgl;

use crate::vis_common::{sync_geoms, flip_image_vertically, write_png};
use crate::builder_setters;
use crate::prelude::*;

use bitflags::bitflags;

use std::io::{self, BufWriter, Write};
use std::fmt::Display;
use std::error::Error;
use std::marker::PhantomData;
use std::num::NonZero;
use std::ops::Deref;
use std::path::Path;
use std::fs::File;

/// Scale factor for converting normalized [0..1] depth to u16.
const DEPTH_U16_SCALE: f32 = u16::MAX as f32;

#[cfg(feature = "renderer-winit-fallback")]
mod universal;

#[cfg(feature = "renderer-winit-fallback")]
use universal::GlStateWinit;

#[cfg(target_os = "linux")]
mod egl;


const EXTRA_INTERNAL_VISUAL_GEOMS: u32 = 100;


/// GlState enum wrapper. By default, headless implementation will be used
/// when supported. Only on failure will an invisible winit window be used.
#[derive(Debug)]
pub(crate) enum GlState {
    #[cfg(feature = "renderer-winit-fallback")] Winit(GlStateWinit),
    #[cfg(target_os = "linux")] Egl(egl::GlStateEgl),
}

impl GlState {
    /// Creates a new [`GlState`], which by default tries to use
    /// an offscreen implementation. As a fallback, winit will be used.
    pub(crate) fn new(width: NonZero<u32>, height: NonZero<u32>) -> Result<Self, RendererError> {
        #[cfg(target_os = "linux")]
        #[allow(unused_variables)]
        let egl_err = match GlStateEgl::new(width, height) {
            Ok(egl_state) => return Ok(Self::Egl(egl_state)),
            Err(e) => e,
        };

        #[cfg(feature = "renderer-winit-fallback")]
        match GlStateWinit::new(width, height) {
            Ok(winit_state) => return Ok(Self::Winit(winit_state)),
            #[cfg(not(target_os = "linux"))]
            Err(e) => {
                return Err(e);
            },

            #[cfg(target_os = "linux")]
            _ => {}
        }

        #[cfg(target_os = "linux")]
        Err(RendererError::GlutinError(egl_err))
    }

    /// Makes the internal OpenGL context current for the calling thread.
    pub(crate) fn make_current(&self) -> glutin::error::Result<()> {
        match self {
            #[cfg(target_os = "linux")]
            Self::Egl(egl_state) => egl_state.make_current(),
            #[cfg(feature = "renderer-winit-fallback")]
            Self::Winit(winit_state) => winit_state.make_current()
        }
    }
}


/// A builder for [`MjRenderer`].
#[derive(Debug)]
pub struct MjRendererBuilder<M: Deref<Target = MjModel> + Clone> {
    width: u32,
    height: u32,
    num_visual_internal_geom: u32,
    num_visual_user_geom: u32,
    rgb: bool,
    depth: bool,
    font_scale: MjtFontScale,
    camera: MjvCamera,
    opts: MjvOption,
    model_type: PhantomData<M>
}

impl<M: Deref<Target = MjModel> + Clone> MjRendererBuilder<M> {
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
            num_visual_internal_geom: EXTRA_INTERNAL_VISUAL_GEOMS, num_visual_user_geom: 0,
            rgb: true, depth: false, font_scale: MjtFontScale::mjFONTSCALE_100,
            camera: MjvCamera::default(), opts: MjvOption::default(),
            model_type: PhantomData
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
    /// # Returns
    /// On success, returns [`Ok`] variant containing the [`MjRenderer`].
    /// # Errors
    /// - [`RendererError::ZeroDimension`] if the width or height is zero.
    /// - [`RendererError::GlutinError`] if OpenGL initialization fails.
    /// - [`RendererError::EventLoopError`] if the event loop fails to initialize.
    /// - [`RendererError::GlInitFailed`] if the fallback window initialization fails.
    pub fn build(self, model: M) -> Result<MjRenderer<M>, RendererError> {
        // Assume model's maximum should be used
        let mut height = self.height;
        let mut width = self.width;
        if width == 0 && height == 0 {
            let global = &model.vis().global;
            height = global.offheight as u32;
            width = global.offwidth as u32;
        }

        let gl_state = GlState::new(
            NonZero::new(width).ok_or(RendererError::ZeroDimension)?,
            NonZero::new(height).ok_or(RendererError::ZeroDimension)?,
        )?;

        // Initialize the rendering context to render to the offscreen buffer.
        let mut context = MjrContext::new(&model);
        context.offscreen();

        // The 3D scene for visualization
        let scene = MjvScene::new(
            model.clone(),
            model.ffi().ngeom as usize + self.num_visual_internal_geom as usize
            + self.num_visual_user_geom as usize
        );

        let user_scene = MjvScene::new(
            model.clone(),
            self.num_visual_user_geom as usize
        );

        // Construct the renderer and create allocated buffers.
        let renderer = MjRenderer {
            scene, user_scene, context, model, camera: self.camera, option: self.opts,
            flags: RendererFlags::empty(), rgb: None, depth: None,
            width: width as usize, height: height as usize, gl_state
        }   // These require special care
            .with_rgb_rendering(self.rgb)
            .with_depth_rendering(self.depth);

        Ok(renderer)
    }
}


impl<M: Deref<Target = MjModel> + Clone> Default for MjRendererBuilder<M> {
    fn default() -> Self {
        Self::new()
    }
}

/// A renderer for rendering 3D scenes.
/// By default, RGB rendering is enabled and depth rendering is disabled.
#[derive(Debug)]
pub struct MjRenderer<M: Deref<Target = MjModel> + Clone> {
    scene: MjvScene<M>,
    user_scene: MjvScene<M>,
    context: MjrContext,
    model: M,

    /* OpenGL */
    gl_state: GlState,

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

impl<M: Deref<Target = MjModel> + Clone> MjRenderer<M> {
    /// Construct a new renderer.
    /// The `max_user_geom` parameter
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
    /// # Returns
    /// On success, returns [`Ok`] variant containing the [`MjRenderer`].
    /// # Errors
    /// - [`RendererError::ZeroDimension`] if the width or height is zero.
    /// - [`RendererError::GlutinError`] if OpenGL initialization fails.
    /// - [`RendererError::EventLoopError`] if the event loop fails to initialize.
    /// - [`RendererError::GlInitFailed`] if the fallback window initialization fails.
    pub fn new(model: M, width: usize, height: usize, max_user_geom: usize) -> Result<Self, RendererError> {
        let builder = Self::builder()
            .width(width as u32).height(height as u32).num_visual_user_geom(max_user_geom as u32);
        builder.build(model)
    }

    /// Create a [`MjRendererBuilder`] to configure [`MjRenderer`].
    pub fn builder() -> MjRendererBuilder<M> {
        MjRendererBuilder::new()
    }

    /// Return an immutable reference to the internal scene.
    pub fn scene(&self) -> &MjvScene<M>{
        &self.scene
    }

    /// Return an immutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene(&self) -> &MjvScene<M>{
        &self.user_scene
    }

    /// Return a mutable reference to a user scene for drawing custom visual-only geoms.
    pub fn user_scene_mut(&mut self) -> &mut MjvScene<M>{
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
    ///
    /// # Panics
    /// Panics if `data` comes from a different model than the renderer.
    /// Use [`MjRenderer::try_sync`] for a fallible alternative.
    pub fn sync(&mut self, data: &mut MjData<M>) {
        self.try_sync(data).expect("sync failed")
    }

    /// Fallible version of [`MjRenderer::sync`].
    ///
    /// # Errors
    /// - [`RendererError::SignatureMismatch`] if `data` was created from a
    ///   different model than the renderer.
    /// - [`RendererError::GlutinError`] if the OpenGL context could not be made current.
    /// - [`RendererError::SceneError`] if the user-scene sync overflows the geom buffer.
    pub fn try_sync(&mut self, data: &mut MjData<M>) -> Result<(), RendererError> {
        let src_sig = data.model().signature();
        let dst_sig = self.model.signature();
        if src_sig != dst_sig {
            return Err(RendererError::SignatureMismatch {
                source: src_sig,
                destination: dst_sig,
            });
        }

        self.scene.update(data, &self.option, &MjvPerturb::default(), &mut self.camera);

        /* Draw user scene geoms */
        sync_geoms(&self.user_scene, &mut self.scene)?;

        self.render()?;
        Ok(())
    }

    /// Return a flattened RGB image of the scene.
    pub fn rgb_flat(&self) -> Option<&[u8]> {
        self.rgb.as_deref()
    }

    /// Return an RGB image of the scene. This method accepts two generic parameters <WIDTH, HEIGHT>
    /// that define the shape of the output slice.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the rendered RGB image.
    /// # Errors
    /// - [`RendererError::DimensionMismatch`] if the image size doesn't match the required dimensions.
    /// - [`RendererError::RgbDisabled`] if RGB rendering is disabled.
    pub fn rgb<const WIDTH: usize, const HEIGHT: usize>(&self) -> Result<&[[[u8; 3]; WIDTH]; HEIGHT], RendererError> {
        if let Some(flat) = self.rgb_flat() {
            bytemuck::try_from_bytes(flat)
                .map_err(|_| RendererError::DimensionMismatch)
        }
        else {
            Err(RendererError::RgbDisabled)
        }
    }

    /// Return a flattened depth image of the scene.
    pub fn depth_flat(&self) -> Option<&[f32]> {
        self.depth.as_deref()
    }

    /// Return a depth image of the scene. This method accepts two generic parameters <WIDTH, HEIGHT>
    /// that define the shape of the output slice.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the rendered depth image.
    /// # Errors
    /// - [`RendererError::DimensionMismatch`] if the image size doesn't match the required dimensions.
    /// - [`RendererError::DepthDisabled`] if depth rendering is disabled.
    pub fn depth<const WIDTH: usize, const HEIGHT: usize>(&self) -> Result<&[[f32; WIDTH]; HEIGHT], RendererError> {
        if let Some(flat) = self.depth_flat() {
            let bytes: &[u8] = bytemuck::cast_slice(flat);
            bytemuck::try_from_bytes(bytes)
                .map_err(|_| RendererError::DimensionMismatch)
        }
        else {
            Err(RendererError::DepthDisabled)
        }
    }

    /// Save an RGB image of the scene to a path.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`RendererError::RgbDisabled`] when RGB rendering is disabled.
    /// - [`RendererError::IoError`] if a file I/O operation fails.
    pub fn save_rgb<T: AsRef<Path>>(&self, path: T) -> Result<(), RendererError> {
        if let Some(rgb) = &self.rgb {
            write_png(
                path,
                rgb,
                self.width as u32,
                self.height as u32,
                png::ColorType::Rgb,
                png::BitDepth::Eight,
            )?;
            Ok(())
        }
        else {
            Err(RendererError::RgbDisabled)
        }
    }

    /// Save a depth image of the scene to a path. The image is 16-bit PNG, which
    /// can be converted into depth (distance) data by dividing the grayscale values by
    /// 65535.0 and applying inverse normalization: `depth = min + (grayscale / 65535.0) * (max - min)`.
    ///
    /// If `normalize` is `true`, then the data is normalized with per-frame min-max normalization.
    /// When `normalize` is `false`, the depth values are mapped using the model's camera
    /// near/far clip planes as the range, providing a fixed (frame-independent) mapping.
    ///
    /// Use of [`MjRenderer::save_depth_raw`] is recommended if performance is critical, as
    /// it skips PNG encoding and also saves the true depth values directly.
    /// # Returns
    /// An [`Ok`]`((min, max))` is returned, where min and max represent the normalization parameters.
    /// # Errors
    /// - [`RendererError::DepthDisabled`] when depth rendering is disabled.
    /// - [`RendererError::IoError`] if a file I/O operation fails.
    pub fn save_depth<T: AsRef<Path>>(&self, path: T, normalize: bool) -> Result<(f32, f32), RendererError> {
        if let Some(depth) = &self.depth {
            let (norm, min, max) =
            if normalize {
                let max = depth.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                let min = depth.iter().cloned().fold(f32::INFINITY, f32::min);
                let range = max - min;
                if range == 0.0 {
                    (vec![0u8; depth.len() * 2].into_boxed_slice(), min, max)
                } else {
                    (depth.iter().flat_map(|&x| (((x - min) / range * DEPTH_U16_SCALE).clamp(0.0, DEPTH_U16_SCALE) as u16).to_be_bytes()).collect::<Box<_>>(), min, max)
                }
            }
            else {
                // Use model's camera near/far clip planes as the fixed normalization range.
                // After linearization (in render()), depth values are in meters within [near, far].
                let map = &self.model.vis().map;
                let stat = &self.model.stat();
                let extent = stat.extent as f32;
                let near = map.znear * extent;
                let far = map.zfar * extent;
                (depth.iter().flat_map(|&x| (((x - near) / (far - near) * DEPTH_U16_SCALE).clamp(0.0, DEPTH_U16_SCALE) as u16).to_be_bytes()).collect::<Box<_>>(), near, far)
            };

            write_png(
                path,
                &norm,
                self.width as u32,
                self.height as u32,
                png::ColorType::Grayscale,
                png::BitDepth::Sixteen,
            )?;
            Ok((min, max))
        }
        else {
            Err(RendererError::DepthDisabled)
        }
    }

    /// Save the raw depth data to the `path`. The data is encoded
    /// as a sequence of bytes, where groups of four represent a single f32 value.
    /// The lower bytes of individual f32 appear first (little-endianness).
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`RendererError::DepthDisabled`] when depth rendering is disabled.
    /// - [`RendererError::IoError`] if a file I/O operation fails.
    pub fn save_depth_raw<T: AsRef<Path>>(&self, path: T) -> Result<(), RendererError> {
        if let Some(depth) = &self.depth {
            let file = File::create(path.as_ref())?;
            let mut writer = BufWriter::new(file);

            let bytes: &[u8] = bytemuck::cast_slice(depth);
            writer.write_all(bytes)?;
            Ok(())
        }
        else {
            Err(RendererError::DepthDisabled)
        }
    }

    /// Draws the scene to internal arrays.
    /// Use [`MjRenderer::rgb`] or [`MjRenderer::depth`] to obtain the rendered image.
    fn render(&mut self) -> Result<(), RendererError> {
        self.gl_state.make_current().map_err(RendererError::GlutinError)?;
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

        /* Flip the read pixels vertically, as OpenGL reads bottom-up */
        if let Some(rgb) = self.rgb.as_deref_mut() {
            flip_image_vertically(rgb, self.height, self.width * 3);
        }

        /* Make depth values be the actual distance in meters and flip them vertically */
        if let Some(depth) = self.depth.as_deref_mut() {
            flip_image_vertically(depth, self.height, self.width);

            let map = &self.model.vis().map;
            let stat = &self.model.stat();

            let extent = stat.extent as f32;
            let near = map.znear * extent;
            let far = map.zfar * extent;
            for value in depth {
                *value = near / (1.0 - *value * (1.0 - near / far));
            }
        }

        Ok(())
    }
}

/// Errors that can occur during renderer operations.
#[derive(Debug)]
#[non_exhaustive]
pub enum RendererError {
    /// The event loop failed to initialize.
    #[cfg(feature = "renderer-winit-fallback")]
    EventLoopError(winit::error::EventLoopError),
    /// A glutin operation failed.
    GlutinError(glutin::error::Error),
    /// The supplied width or height was zero; MuJoCo requires positive dimensions.
    ZeroDimension,
    /// OpenGL / window initialization failed.
    #[cfg(feature = "renderer-winit-fallback")]
    GlInitFailed(crate::error::GlInitError),
    /// The data and renderer were created from different models.
    SignatureMismatch {
        /// Model signature of the source object.
        source: u64,
        /// Model signature of the destination object.
        destination: u64,
    },
    /// RGB rendering was not enabled.
    RgbDisabled,
    /// Depth rendering was not enabled.
    DepthDisabled,
    /// The requested `WIDTH`/`HEIGHT` do not match the renderer's dimensions.
    DimensionMismatch,
    /// An I/O error occurred (e.g. while saving to a file).
    IoError(io::Error),
    /// A scene operation failed (e.g. user-scene sync overflowed the geom buffer).
    SceneError(crate::error::MjSceneError),
}

impl Display for RendererError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            #[cfg(feature = "renderer-winit-fallback")]
            Self::EventLoopError(_) => write!(f, "event loop failed to initialize"),
            Self::GlutinError(_) => write!(f, "glutin error"),
            Self::ZeroDimension => write!(f, "renderer width and height must both be greater than zero"),
            #[cfg(feature = "renderer-winit-fallback")]
            Self::GlInitFailed(_) => write!(f, "GL initialization failed"),
            Self::SignatureMismatch { source, destination } => {
                write!(f, "model signature mismatch: source {source:#X}, destination {destination:#X}")
            }
            Self::RgbDisabled => write!(f, "RGB rendering is not enabled (renderer.with_rgb_rendering(true))"),
            Self::DepthDisabled => write!(f, "depth rendering is not enabled (renderer.with_depth_rendering(true))"),
            Self::DimensionMismatch => write!(f, "the input width and height don't match the renderer's configuration"),
            Self::IoError(_) => write!(f, "I/O error"),
            Self::SceneError(_) => write!(f, "scene error"),
        }
    }
}

impl Error for RendererError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            #[cfg(feature = "renderer-winit-fallback")]
            Self::EventLoopError(e) => Some(e),
            #[cfg(feature = "renderer-winit-fallback")]
            Self::GlInitFailed(e) => Some(e),
            Self::GlutinError(e) => Some(e),
            Self::IoError(e) => Some(e),
            Self::SceneError(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for RendererError {
    fn from(e: io::Error) -> Self {
        Self::IoError(e)
    }
}

impl From<png::EncodingError> for RendererError {
    fn from(e: png::EncodingError) -> Self {
        Self::IoError(io::Error::from(e))
    }
}

impl From<crate::error::MjSceneError> for RendererError {
    fn from(e: crate::error::MjSceneError) -> Self {
        Self::SceneError(e)
    }
}

#[cfg(feature = "renderer-winit-fallback")]
impl From<crate::error::GlInitError> for RendererError {
    fn from(e: crate::error::GlInitError) -> Self {
        Self::GlInitFailed(e)
    }
}

bitflags! {
    /// Flags that enable features of the renderer.
    #[derive(Debug)]
    struct RendererFlags: u8 {
        const RENDER_RGB = 1 << 0;
        const RENDER_DEPTH = 1 << 1;
    }
}



impl<M: Deref<Target = MjModel> + Clone> Drop for MjRenderer<M> {
    fn drop(&mut self) {
        // Ensure the GL context is current before the implicit field drops
        // (MjrContext's Drop calls mjr_freeContext which requires an active GL context).
        let _ = self.gl_state.make_current();
    }
}

/*
** Don't run any tests as OpenGL hates if anything
** runs outside the main thread.
*/

#[cfg(test)]
mod test {
    use crate::assert_relative_eq;

    use super::*;

    const MODEL: &str = stringify!(
        <mujoco>

        <visual>
            <global offwidth="1280" offheight="720"/>
        </visual>

        <worldbody>
            <geom name="floor" type="plane" size="10 10 1" euler="0 0 0"/>
            <geom type="box" size="1 10 10" pos="-1 0 0" euler="0 0 0"/>

            <camera name="depth_test" euler="90 90 0" pos="2.25 0 1"/>

        </worldbody>
        </mujoco>
    );

    /// Depth calculation test.
    /// This is only run on Linux due to EGL requirements (winit cannot be used on multiple threads).
    #[test]
    #[cfg(target_os = "linux")]
    fn test_depth() {
        let model = MjModel::from_xml_string(MODEL).expect("could not load the model");
        let mut data = MjData::new(&model);
        data.step();

        let mut renderer = MjRenderer::builder()
            .rgb(false)
            .depth(true)
            .camera(MjvCamera::new_fixed(model.name_to_id(MjtObj::mjOBJ_CAMERA, "depth_test").unwrap() as u32))
            .build(&model)
            .unwrap();

        renderer.sync(&mut data);
        let min = renderer.depth_flat().unwrap().iter().fold(f32::INFINITY, |a , &b| a.min(b));
        let max = renderer.depth_flat().unwrap().iter().fold(f32::NEG_INFINITY, |a , &b| a.max(b));

        assert_relative_eq!(min, max, epsilon = 1e-4);
        assert_relative_eq!(min, 2.25, epsilon = 1e-4);
    }
}
