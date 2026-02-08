//! Definitions related to rendering.
use std::{ffi::CString, mem::{zeroed, MaybeUninit}};
use crate::{array_slice_dyn, getter_setter, mujoco_c::*};

use super::mj_model::{MjModel, MjtTexture, MjtTextureRole};
use std::ptr;

/***********************************************************************************************************************
** MjrRectangle
***********************************************************************************************************************/
pub type MjrRectangle = mjrRect;
impl MjrRectangle {
    pub fn new(left: i32, bottom: i32, width: i32, height: i32) -> Self {
        Self {
            left,
            bottom,
            width,
            height,
        }
    }
}

impl Default for MjrRectangle {
    fn default() -> Self {
        unsafe { zeroed() }
    }
}


/// These are the possible grid positions for text overlays. They are used as an argument to the function
/// `mjr_overlay`.
pub type MjtGridPos = mjtGridPos;

/// These are the possible framebuffers. They are used as an argument to the function `mjr_setBuffer`.
pub type MjtFramebuffer = mjtFramebuffer;

/// These are the depth mapping options. They are used as a value for the ``readPixelDepth`` attribute of the
/// `mjrContext` struct, to control how the depth returned by `mjr_readPixels` is mapped from
/// ``znear`` to ``zfar``.
pub type MjtDepthMap = mjtDepthMap;

/// These are the possible font sizes. The fonts are predefined bitmaps stored in the dynamic library at three different
/// sizes.
pub type MjtFontScale = mjtFontScale;

/// These are the possible font types.
pub type MjtFont = mjtFont;

/***********************************************************************************************************************
** MjrContext
***********************************************************************************************************************/
#[derive(Debug)]
pub struct MjrContext {
    ffi: mjrContext
}

impl MjrContext {
    pub fn new(model: &MjModel) -> Self {
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjr_defaultContext(c.as_mut_ptr());
            mjr_makeContext(model.ffi(), c.as_mut_ptr(), MjtFontScale::mjFONTSCALE_100 as i32);
            Self {ffi: c.assume_init()}
        }
    }

    /// Set OpenGL framebuffer for rendering to mjFB_OFFSCREEN.
    pub fn offscreen(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(MjtFramebuffer::mjFB_OFFSCREEN as i32, &mut self.ffi);
        }
        self
    }

    /// Set OpenGL framebuffer for rendering to mjFB_WINDOW.
    pub fn window(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(MjtFramebuffer::mjFB_WINDOW as i32, &mut self.ffi);
        }
        self
    }

    /// Change font of existing context.
    pub fn change_font(&mut self, fontscale: MjtFontScale) {
        unsafe { mjr_changeFont(fontscale as i32, self.ffi_mut()) }
    }

    /// Add Aux buffer with given index to context; free previous Aux buffer.
    pub fn add_aux(&mut self, index: usize, width: u32, height: u32, samples: usize) {
        unsafe { mjr_addAux(index as i32, width as i32, height as i32, samples as i32, self.ffi_mut()); }
    }

    /// Resize offscreen buffers.
    pub fn resize_offscreen(&mut self, width: u32, height: u32) {
        unsafe { mjr_resizeOffscreen(width as i32, height as i32, self.ffi_mut()); }
    }

    /// Upload texture to GPU, overwriting previous upload if any.
    pub fn upload_texture(&mut self, model: &MjModel, texid: u32) {
        unsafe { mjr_uploadTexture(model.ffi(), self.ffi_mut(), texid as i32); }
    }

    /// Make the context's buffer current again.
    pub fn restore_buffer(&mut self) {
        unsafe { mjr_restoreBuffer(self.ffi_mut()); }
    }

    pub fn mjr_set_buffer(&mut self, framebuffer: i32) {
        unsafe { mjr_setBuffer(framebuffer, self.ffi_mut()); }
    }

    /// Read pixels from current OpenGL framebuffer to client buffer.
    /// The ``rgb`` array is of size [width * height * 3], while ``depth`` is of size [width * height].
    /// # Panics
    /// Panics if the provided buffers are not large enough to hold the data for the given `viewport`.
    pub fn read_pixels(&self, rgb: Option<&mut [u8]>, depth: Option<&mut [f32]>, viewport: &MjrRectangle) {
        assert!(viewport.width >= 0);
        assert!(viewport.height >= 0);
        let size = viewport.width as usize * viewport.height as usize;
        if let Some(buf) = rgb.as_ref() {
            assert!(buf.len() >= size * 3, "rgb buffer is too small");
        }
        if let Some(buf) = depth.as_ref() {
            assert!(buf.len() >= size, "depth buffer is too small");
        }

        unsafe {
            mjr_readPixels(
                rgb.map_or(ptr::null_mut(), |x| x.as_mut_ptr()),
                depth.map_or(ptr::null_mut(), |x| x.as_mut_ptr()),
                viewport.clone(), self.ffi()
            )
        }
    }

    /// Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
    pub fn set_aux(&mut self, index: usize) {
        unsafe { mjr_setAux(index as i32, self.ffi_mut()); }
    }

    /// Draws a text overlay. The optional `overlay2` parameter displays additional overlay, next to `overlay`.
    /// # Panics
    /// When the `overlay` or `overlay2` contain '\0' characters, a panic occurs.
    pub fn overlay(&mut self, font: MjtFont, gridpos: MjtGridPos, viewport: MjrRectangle, overlay: &str, overlay2: Option<&str>) {
        let c_overlay = CString::new(overlay).unwrap();
        let c_overlay2 = overlay2.map(|x| CString::new(x).unwrap());

        unsafe { mjr_overlay(
            font as i32, gridpos as i32, viewport,
            c_overlay.as_ptr(),
            c_overlay2.as_ref().map_or(std::ptr::null(), |x| x.as_ptr()),
            self.ffi()
        ); }
    }

    pub fn ffi(&self) -> &mjrContext {
        &self.ffi
    }

    pub unsafe fn ffi_mut(&mut self) -> &mut mjrContext {
        &mut self.ffi
    }
}

/// Array slices.
impl MjrContext {
    array_slice_dyn! {
        (allow_mut = false) textureType: as_ptr as_mut_ptr &[MjtTexture [cast]; "type of texture"; ffi().ntexture],
        (allow_mut = false) skinvertVBO: &[u32; "skin vertex position VBOs"; ffi().nskin],
        (allow_mut = false) skinnormalVBO: &[u32; "skin vertex normal VBOs"; ffi().nskin],
        (allow_mut = false) skintexcoordVBO: &[u32; "skin vertex texture coordinate VBOs"; ffi().nskin],
        (allow_mut = false) skinfaceVBO: &[u32; "skin face index VBOs"; ffi().nskin]
    }
}

impl MjrContext {
    getter_setter! {get, [
        [ffi] lineWidth: f32; "line width for wireframe rendering.";
        [ffi] shadowClip: f32; "clipping radius for directional lights.";
        [ffi] shadowScale: f32; "fraction of light cutoff for spot lights.";
        [ffi] fogStart: f32; "fog start = stat.extent * vis.map.fogstart.";
        [ffi] fogEnd: f32; "fog end = stat.extent * vis.map.fogend.";
        [ffi] shadowSize: i32; "size of shadow map texture.";
        [ffi] offWidth: i32; "width of offscreen buffer.";
        [ffi] offHeight: i32; "height of offscreen buffer.";
        [ffi] offSamples: i32; "number of offscreen buffer multisamples.";
        [ffi] fontScale: i32; "font scale.";
        [ffi] offFBO: u32; "offscreen framebuffer object.";
        [ffi] offFBO_r: u32; "offscreen framebuffer for resolving multisamples.";
        [ffi] offColor: u32; "offscreen color buffer.";
        [ffi] offColor_r: u32; "offscreen color buffer for resolving multisamples.";
        [ffi] offDepthStencil: u32; "offscreen depth and stencil buffer.";
        [ffi] offDepthStencil_r: u32; "offscreen depth and stencil buffer for multisamples.";
        [ffi] shadowFBO: u32; "shadow map framebuffer object.";
        [ffi] shadowTex: u32; "shadow map texture.";
        [ffi] ntexture: i32; "number of allocated textures.";
        [ffi] basePlane: u32; "all planes from model.";
        [ffi] baseMesh: u32; "all meshes from model.";
        [ffi] baseHField: u32; "all height fields from model.";
        [ffi] baseBuiltin: u32; "all builtin geoms, with quality from model.";
        [ffi] baseFontNormal: u32; "normal font.";
        [ffi] baseFontShadow: u32; "shadow font.";
        [ffi] baseFontBig: u32; "big font.";
        [ffi] rangePlane: i32; "all planes from model.";
        [ffi] rangeMesh: i32; "all meshes from model.";
        [ffi] rangeHField: i32; "all hfields from model.";
        [ffi] rangeBuiltin: i32; "all builtin geoms, with quality from model.";
        [ffi] rangeFont: i32; "all characters in font.";
        [ffi] nskin: i32; "number of skins.";
        [ffi] charHeight: i32; "character heights: normal and shadow.";
        [ffi] charHeightBig: i32; "character heights: big.";
        [ffi] glInitialized: i32; "is OpenGL initialized.";
        [ffi] windowAvailable: i32; "is default/window framebuffer available.";
        [ffi] windowSamples: i32; "number of samples for default/window framebuffer.";
        [ffi] windowStereo: i32; "is stereo available for default/window framebuffer.";
        [ffi] windowDoublebuffer: i32; "is default/window framebuffer double buffered.";
        [ffi] currentBuffer: i32; "currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN.";
        [ffi] readPixelFormat: i32; "default color pixel format for mjr_readPixels.";
        [ffi] readDepthMap: i32; "depth mapping: mjDEPTH_ZERONEAR or mjDEPTH_ZEROFAR.";
    ]}

    getter_setter! {get, [
        [ffi] (allow_mut = false) fogRGBA: &[f32; 4]; "fog rgba.";
        [ffi] (allow_mut = false) auxWidth: &[i32; mjNAUX as usize]; "auxiliary buffer width.";
        [ffi] (allow_mut = false) auxHeight: &[i32; mjNAUX as usize]; "auxiliary buffer height.";
        [ffi] (allow_mut = false) auxSamples: &[i32; mjNAUX as usize]; "auxiliary buffer multisamples.";
        [ffi] (allow_mut = false) auxFBO: &[u32; mjNAUX as usize]; "auxiliary framebuffer object.";
        [ffi] (allow_mut = false) auxFBO_r: &[u32; mjNAUX as usize]; "auxiliary framebuffer object for resolving.";
        [ffi] (allow_mut = false) auxColor: &[u32; mjNAUX as usize]; "auxiliary color buffer.";
        [ffi] (allow_mut = false) auxColor_r: &[u32; mjNAUX as usize]; "auxiliary color buffer for resolving.";
        [ffi] (allow_mut = false) mat_texid: &[i32; (mjMAXMATERIAL * MjtTextureRole::mjNTEXROLE as u32) as usize]; "material texture ids (-1: no texture).";
        [ffi] (allow_mut = false) mat_texuniform: &[i32; mjMAXMATERIAL as usize]; "uniform cube mapping.";
        [ffi] (allow_mut = false) mat_texrepeat: &[f32; (mjMAXMATERIAL * 2) as usize]; "texture repetition for 2d mapping.";
        [ffi] (allow_mut = false) texture: &[u32; mjMAXTEXTURE as usize]; "texture names.";
        [ffi] (allow_mut = false) charWidth: &[i32; 127]; "character widths: normal and shadow.";
        [ffi] (allow_mut = false) charWidthBig: &[i32; 127]; "character widths: big.";
    ]}
}

impl Drop for MjrContext {
    fn drop(&mut self) {
        unsafe {
            mjr_freeContext(&mut self.ffi);
        }
    }
}

