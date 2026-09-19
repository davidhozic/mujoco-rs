use glutin::api::egl::display::Display;
use glutin::api::egl::{context::PossiblyCurrentContext, surface::Surface, device::Device};
use glutin::context::{ContextApi, ContextAttributesBuilder, GlProfile, Version};
use glutin::prelude::{GlDisplay, NotCurrentGlContext, PossiblyCurrentGlContext};
use glutin::surface::{PbufferSurface, SurfaceAttributesBuilder};
use glutin::config::{ConfigSurfaceTypes, ConfigTemplateBuilder};

use log::{debug, warn};

use std::num::NonZero;


/// GlState implementation, based on EGL for GNU Linux platforms.
#[derive(Debug)]
pub(crate) struct GlStateEgl {
    pub(crate) context: PossiblyCurrentContext,
    pub(crate) surface: Surface<PbufferSurface>,
}


impl GlStateEgl {
    /// Opens an offscreen context on the first EGL device that yields one.
    /// 
    /// # Errors
    /// A [`glutin::error::ErrorKind::NotSupported`] error is returned when no device succeeded in initialization.
    pub(crate) fn new(width: NonZero<u32>, height: NonZero<u32>) -> glutin::error::Result<Self> {
        let devices = Device::query_devices().map_err(|e|
            if glutin::error::ErrorKind::NotFound == e.error_kind() {
                glutin::error::ErrorKind::NotSupported("EGL was not found").into()
            } else { e }
        )?;

        let mut last_error = None;
        for (index, device) in devices.enumerate() {
            match Self::on_device(&device, width, height) {
                Ok(state) => {
                    if index > 0 {
                        warn!(
                            "rendering offscreen on EGL device {index} ({}): \
                             every earlier device failed to open a context",
                            describe(&device)
                        );
                    }
                    return Ok(state);
                },
                Err(e) => {
                    debug!("EGL device {index} ({}) opened no context: {e}", describe(&device));
                    last_error = Some(e);
                },
            }
        }

        // `last_error` is None only when the iterator was empty.
        Err(last_error.unwrap_or_else(||
            glutin::error::ErrorKind::NotSupported("could not find any compatible devices").into()
        ))
    }

    /// Tries to opens the offscreen context on `device`.
    fn on_device(
        device: &Device,
        width: NonZero<u32>,
        height: NonZero<u32>,
    ) -> glutin::error::Result<Self> {
        // SAFETY: device is a valid EGL device obtained from query_devices.
        let display = unsafe { Display::with_device(device, None)? };
        let config_template = ConfigTemplateBuilder::new()
            .with_surface_type(ConfigSurfaceTypes::PBUFFER)
            // Request typical formats; these are hints.
            .with_alpha_size(0)
            .with_depth_size(24)
            .with_stencil_size(8)
            .build();

        // SAFETY: display is a valid EGL display and config_template requests supported attributes.
        let config = unsafe { display.find_configs(config_template)?.next().ok_or_else(||
            glutin::error::ErrorKind::NotSupported("could not find any compatible configs")
        )}?;

        let context_attrs = ContextAttributesBuilder::new()
            .with_profile(GlProfile::Compatibility)
            .with_context_api(ContextApi::OpenGl(Some(Version::new(2, 0))))
            .build(None);

        // SAFETY: display and config are valid; context_attrs requests a compatible GL profile.
        let context_not_current = unsafe { display.create_context(&config, &context_attrs)? };

        let surface_attrs = SurfaceAttributesBuilder::<PbufferSurface>::new()
            .build(width, height);

        // SAFETY: display and config are valid; surface_attrs has valid non-zero dimensions.
        let surface = unsafe { display.create_pbuffer_surface(&config, &surface_attrs)? };
        let context = context_not_current.make_current(&surface)?;
        Ok(Self {context, surface})
    }

    pub(crate) fn make_current(&self) -> Result<(), glutin::error::Error> {
        self.context.make_current(&self.surface)
    }
}


/// A device as a log line names it: its DRM node, or that it is Mesa's
/// software device, or whatever name it answers to.
fn describe(device: &Device) -> String {
    if let Some(path) = device.drm_device_node_path() {
        path.display().to_string()
    } else if device.extensions().contains("EGL_MESA_device_software") {
        "software".to_owned()
    } else {
        device.name().unwrap_or("unnamed").to_owned()
    }
}
