//! Definitions related to rendering.
use std::mem::MaybeUninit;
use crate::mujoco_c::*;

use super::mj_model::MjModel;


/***********************************************************************************************************************
** MjrContext
***********************************************************************************************************************/
pub struct MjrContext {
    ffi: mjrContext
}

impl MjrContext {
    pub fn new(model: &MjModel) -> Self {
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjr_defaultContext(c.as_mut_ptr());
            mjr_makeContext(model.ffi(), c.as_mut_ptr(), mjtFontScale__mjFONTSCALE_100 as i32);
            Self {ffi: c.assume_init()}
        }
    }

    /// Set OpenGL framebuffer for rendering to mjFB_OFFSCREEN.
    pub fn offscreen(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(mjtFramebuffer__mjFB_OFFSCREEN as i32, &mut self.ffi);
        }
        self
    }

    /// Set OpenGL framebuffer for rendering to mjFB_WINDOW.
    pub fn window(&mut self) -> &mut Self {
        unsafe {
            mjr_setBuffer(mjtFramebuffer__mjFB_WINDOW as i32, &mut self.ffi);
        }
        self
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

/***********************************************************************************************************************
** MjrContext
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
