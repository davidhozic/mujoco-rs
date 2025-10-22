//! Base definitions needed to support rendering in both `MjViewer` and `MjRenderer`.
use glutin::context::{ContextApi, ContextAttributesBuilder, GlProfile, Version};
use glutin::surface::{Surface, SurfaceAttributesBuilder, WindowSurface};
use glutin::display::{GetGlDisplay, GlDisplay};
use glutin::context::PossiblyCurrentContext;
use glutin::config::ConfigTemplateBuilder;
use glutin::prelude::*;

use winit::platform::pump_events::EventLoopExtPumpEvents;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::raw_window_handle::HasWindowHandle;
use winit::application::ApplicationHandler;
use winit::window::{Window, WindowId};
use winit::event::WindowEvent;
use winit::dpi::PhysicalSize;

use glutin_winit::{ApiPreference, DisplayBuilder, GlWindow};
use std::collections::VecDeque;


/// Base struct for rendering through Glutin.
/// This is a proxy since Glutin only allows event processing through callbacks, which this implements.
#[derive(Debug)]
pub(crate) struct GlState {
    pub(crate) window: Window,
    pub(crate) gl_context: PossiblyCurrentContext,
    pub(crate) gl_surface: Surface<WindowSurface>,
}

#[derive(Debug)]
pub(crate) struct RenderBase {
    pub(crate) state: Option<GlState>,
    pub(crate) running: bool,
    /* Event related */
    pub(crate) queue: VecDeque<WindowEvent>,

    /* Storage */
    size: (u32, u32),
    title: String,
    events: bool
}

impl RenderBase {
    pub(crate) fn new(width: u32, height: u32, title: String, event_loop: &mut EventLoop<()>, events: bool) -> Self {
        let mut s = Self {
            state: None,
            running: false,
            queue: VecDeque::new(),
            size: (width, height),
            title,
            events
        };

        // Initialize through app callbacks.
        event_loop.pump_app_events(None, &mut s);
        s
    }

    fn maybe_forward_event(&mut self, event: WindowEvent) {
        if self.events {
            self.queue.push_back(event);
        }
    }
}

impl ApplicationHandler for RenderBase {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window_attrs = Window::default_attributes()
            .with_title(&self.title)
            .with_inner_size(PhysicalSize::new(self.size.0, self.size.1));

        let template = ConfigTemplateBuilder::new()
            // Request typical formats; these are hints.
            .with_alpha_size(0)
            .with_depth_size(24)
            .with_stencil_size(8);

        let display_builder = DisplayBuilder::new()
            .with_preference(ApiPreference::FallbackEgl) // or PreferEgl on Linux
            .with_window_attributes(Some(window_attrs));

        // Select the config with most samples.
        let (maybe_window, gl_config) = display_builder
            .build(event_loop, template, |configs| {
                configs.into_iter().reduce(|current, cfg|
                    if cfg.num_samples() > current.num_samples() {cfg} else { current }
                ).expect("no GL configs found")
            })
            .expect("display build");

        // Finalize the Window from the config’s requirements
        let window = maybe_window.expect("failed to create window -- this is a bug, please report it");

        // Create the GL context + window surface
        let raw_window_handle = Some(window.window_handle()
            .map(|x| x.as_raw()).unwrap());
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

        // Build surface attributes
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
        self.state = Some(GlState { gl_surface, gl_context, window });
        self.running = true;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
                self.running = false;
            }
            WindowEvent::Resized(_) => {
                if let Some(GlState { window, gl_context, gl_surface }) = &self.state {
                    window.resize_surface(gl_surface, gl_context);
                }
            }

            // Fill the event buffer for everything else
            _ => self.maybe_forward_event(event)
        }
    }
}
