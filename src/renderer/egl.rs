use glutin::api::egl::display::Display;
use glutin::api::egl::{context::PossiblyCurrentContext, surface::Surface, device::Device};
use glutin::context::{ContextApi, ContextAttributesBuilder, GlProfile, Version};
use glutin::prelude::{GlDisplay, NotCurrentGlContext, PossiblyCurrentGlContext};
use glutin::surface::{PbufferSurface, SurfaceAttributesBuilder};
use glutin::config::{ConfigSurfaceTypes, ConfigTemplateBuilder};

use std::num::NonZero;


/// GlState implementation, based on EGL for GNU Linux platforms.
pub(crate) struct GlStateEgl {
    pub(crate) context: PossiblyCurrentContext,
    pub(crate) surface: Surface<PbufferSurface>,
}


impl GlStateEgl {
    pub(crate) fn new(width: NonZero<u32>, height: NonZero<u32>) -> glutin::error::Result<Self>{
        let device = Device::query_devices()?.next().unwrap();
        unsafe { 
            let display = Display::with_device(&device, None)?;
            let config_template = ConfigTemplateBuilder::new()
                .with_surface_type(ConfigSurfaceTypes::PBUFFER)
                // Request typical formats; these are hints.
                .with_alpha_size(0)
                .with_depth_size(24)
                .with_stencil_size(8)
                .build();

            let config = display.find_configs(config_template)?.next().unwrap();
            let context_attrs = ContextAttributesBuilder::new()
                .with_profile(GlProfile::Compatibility)
                .with_context_api(ContextApi::OpenGl(Some(Version::new(2, 0))))
                .build(None);

            let context_not_current = display.create_context(&config, &context_attrs)?;

            let surface_attrs = SurfaceAttributesBuilder::<PbufferSurface>::new()
                .build(width, height);

            let surface = display.create_pbuffer_surface(&config, &surface_attrs)?;
            let context = context_not_current.make_current(&surface)?;
            Ok(Self {context, surface})
        }
    }

    pub(crate) fn make_current(&self) -> Result<(), glutin::error::Error> {
        self.context.make_current(&self.surface)
    }
}
