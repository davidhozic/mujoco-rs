//! Base definitions needed to support rendering in both `MjViewer` and `MjRenderer`.
use glutin::config::ConfigTemplateBuilder;
use glutin::context::{ContextApi, ContextAttributesBuilder, GlProfile, Version};
use glutin::display::{GetGlDisplay, GlDisplay}; // for get_proc_address
use glutin::prelude::*;         // misc extension traits
use glutin::surface::{GlSurface, Surface, SurfaceAttributesBuilder, SwapInterval, WindowSurface};
use glutin_winit::{ApiPreference, DisplayBuilder, GlWindow};
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::error::EventLoopError;
use winit::event::{WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::platform::pump_events::{EventLoopExtPumpEvents, PumpStatus};
use winit::raw_window_handle::HasWindowHandle;
use winit::window::{Window, WindowId};
use std::mem::MaybeUninit;
use std::sync::mpsc::{self, SendError};
use std::time::Duration;
use glutin::context::PossiblyCurrentContext;


/// Base struct for rendering through Glutin.
/// This is a proxy since Glutin only allows event processing through callbacks, which this implements.
#[derive(Debug)]
pub(crate) struct RenderState {
    pub window: Window,
    pub gl_context: PossiblyCurrentContext,
    pub gl_surface: Surface<WindowSurface>,
}

#[derive(Debug)]
pub(crate) struct RenderBase {
    pub state: Option<RenderState>,
    pub channel: (mpsc::Sender<WindowEvent>, mpsc::Receiver<WindowEvent>),
    size: (u32, u32),
    title: String
}

impl RenderBase {
    pub(crate) fn new(width: u32, height: u32, title: String, event_loop: &mut EventLoop<()>) -> Self {
        let mut s = Self {
            state: None,
            channel: mpsc::channel(),
            size: (width, height),
            title
        };

        // Initialize through app callbacks.
        event_loop.pump_app_events(None, &mut s);
        s
    }

    pub(crate) fn make_current(&self) {
        if let Some(RenderState { window: _, gl_context, gl_surface })
            = &self.state {
            gl_context.make_current(gl_surface);
        }
    }

    pub(crate) fn swap_buffers(&self) {
        if let Some(RenderState { window: _, gl_context, gl_surface })
            = &self.state {
            gl_surface.swap_buffers(gl_context);
        }
    }

    pub(crate) fn set_swap_interval(&self, interval: SwapInterval) {
        if let Some(RenderState { window: _, gl_context, gl_surface })
            = &self.state {
            gl_surface.set_swap_interval(gl_context, interval);
        }
    }
}

impl ApplicationHandler for RenderBase {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        // ---- 1) Pick a GL config and create the window/display ----
        let window_attrs = Window::default_attributes()
            .with_title(&self.title)
            .with_inner_size(PhysicalSize::new(self.size.0, self.size.1));

        let template = ConfigTemplateBuilder::new()
            // Request typical formats; these are hints.
            .with_alpha_size(8)
            .with_depth_size(24)
            .with_stencil_size(8);

        let display_builder = DisplayBuilder::new()
            .with_preference(ApiPreference::FallbackEgl) // or PreferEgl on Linux
            .with_window_attributes(Some(window_attrs));

        // Choose a config. Here we just take the first; you could pick the one with most samples, etc.
        let (maybe_window, gl_config) = display_builder
            .build(event_loop, template, |configs| {
                configs.into_iter().reduce(|current, cfg|
                    if cfg.num_samples() > current.num_samples() {cfg} else { current }
                ).expect("no GL configs found")
            })
            .expect("display build");

        // Create (or finalize) the Window from the config’s requirements.
        let window = maybe_window.expect("failed to create window");

        // ---- 2) Create the GL context + window surface, then make current ----
        let raw_window_handle = Some(window.window_handle().map(|x| x.as_raw()).unwrap()); // required on WGL
        let context_attrs = ContextAttributesBuilder::new()
            .with_profile(GlProfile::Core)
            .with_context_api(ContextApi::OpenGl(Some(Version::new(1, 5))))
            .build(raw_window_handle);

        let gl_display = gl_config.display();
        let not_current = unsafe {
            gl_display
                .create_context(&gl_config, &context_attrs)
                .expect("context create")
        };

        // Build surface attributes from the window’s size/handles.
        let attrs = window
            .build_surface_attributes(SurfaceAttributesBuilder::<WindowSurface>::new().with_srgb(Some(true)))
            .expect("surface attrs");

        let gl_surface = unsafe {
            gl_display
                .create_window_surface(&gl_config, &attrs)
                .expect("surface create")
        };

        let gl_context = not_current.make_current(&gl_surface).expect("make current");

        // Save state
        self.state = Some(RenderState { gl_surface, gl_context, window });
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(_) => {
                if let Some(RenderState { window, gl_context, gl_surface }) = &self.state {
                    window.resize_surface(gl_surface, gl_context);
                }
            }

            // Fill the event buffer for everything else
            _ => self.channel.0.send(event).expect("failed to send event")
        }
    }
}
