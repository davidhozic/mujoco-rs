//! Universal [`GlState`] dependency, used for controlling the OpenGL context and surface.
//! This wraps [`RendererBase`] and thus renders to an invisible window.
//! This is meant as a fallback when true offscreen rendering fails to initialize.

use crate::winit_gl_base::RenderBase;
use super::RendererError;

use glutin::prelude::PossiblyCurrentGlContext;
use winit::event_loop::EventLoop;

use std::num::NonZero;


pub(crate) struct GlStateWinit {
    inner: RenderBase,
}

impl GlStateWinit{
    pub(crate) fn new(width: NonZero<u32>, height: NonZero<u32>) -> Result<Self, RendererError> {
        let mut _event_loop = EventLoop::new().map_err(
            |e| RendererError::EventLoopError(e)
        )?;

        let inner = RenderBase::new(
            width.into(), height.into(), "".to_string(),
            &mut _event_loop,
            false
        );

        Ok(Self {inner})
    }

    pub(crate) fn make_current(&self) -> Result<(), glutin::error::Error> {
        let inner_state = self.inner.state.as_ref().unwrap();
        inner_state.gl_context.make_current(&inner_state.gl_surface)
    }
}
