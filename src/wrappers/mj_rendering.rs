//! Definitions related to rendering.
use std::mem::{MaybeUninit, zeroed};
use crate::mujoco_c::*;

use super::mj_model::MjModel;
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
            mjr_makeContext(model.ffi(), c.as_mut_ptr(), mjtFontScale::mjFONTSCALE_100 as i32);
            Self {ffi: c.assume_init()}
        }
    }

    /// Set OpenGL framebuffer for rendering to mjFB_OFFSCREEN.
    pub fn offscreen(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(mjtFramebuffer::mjFB_OFFSCREEN as i32, &mut self.ffi);
        }
        self
    }

    /// Set OpenGL framebuffer for rendering to mjFB_WINDOW.
    pub fn window(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(mjtFramebuffer::mjFB_WINDOW as i32, &mut self.ffi);
        }
        self
    }

    /// Change font of existing context.
    pub fn change_font(&mut self, fontscale: mjtFontScale) {
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
    pub fn read_pixels(&self, rgb: Option<&mut [u8]>, depth: Option<&mut [f32]>, viewport: &MjrRectangle) {
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

    pub(crate) fn ffi(&self) -> &mjrContext {
        &self.ffi
    }

    #[allow(unused)]
    pub(crate) fn ffi_mut(&mut self) -> &mut mjrContext {
        &mut self.ffi
    }
}

impl Drop for MjrContext {
    fn drop(&mut self) {
        unsafe {
            mjr_freeContext(&mut self.ffi);
        }
    }
}

