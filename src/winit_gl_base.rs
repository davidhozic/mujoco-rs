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
pub(crate) struct RenderBaseGlState {
    pub(crate) window: Window,
    pub(crate) gl_context: PossiblyCurrentContext,
    pub(crate) gl_surface: Surface<WindowSurface>,
}

#[derive(Debug)]
pub(crate) struct RenderBase {
    pub(crate) state: Option<RenderBaseGlState>,
    /* Event related */
    pub(crate) queue: VecDeque<WindowEvent>,

    /* Storage */
    size: (u32, u32),
    title: String,
    events: bool,
    init_error: Option<String>,
}

impl RenderBase {
    pub(crate) fn new(width: u32, height: u32, title: String, event_loop: &mut EventLoop<()>, events: bool) -> Result<Self, String> {
        let mut s = Self {
            state: None,
            queue: VecDeque::new(),
            size: (width, height),
            title,
            events,
            init_error: None,
        };

        // Initialize through app callbacks.
        event_loop.pump_app_events(None, &mut s);

        if let Some(err) = s.init_error.take() {
            return Err(err);
        }
        Ok(s)
    }

    fn maybe_forward_event(&mut self, event: WindowEvent) {
        if self.events {
            self.queue.push_back(event);
        }
    }
}

impl ApplicationHandler for RenderBase {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let result = (|| -> Result<RenderBaseGlState, String> {
            let window_attrs = Window::default_attributes()
                .with_title(&self.title)
                .with_inner_size(PhysicalSize::new(self.size.0, self.size.1))
                .with_visible(self.events);

            let template = ConfigTemplateBuilder::new()
                // Request typical formats; these are hints.
                .with_alpha_size(0)
                .with_depth_size(24)
                .with_stencil_size(8);

            let display_builder = DisplayBuilder::new()
                .with_preference(ApiPreference::FallbackEgl)
                .with_window_attributes(Some(window_attrs));

            // Select the config with most samples.
            // The config-picker callback must return a Config (not Result), so the
            // expect inside it stays: if build() succeeds, at least one config exists.
            let (maybe_window, gl_config) = display_builder
                .build(event_loop, template, |configs| {
                    configs.into_iter().reduce(|current, cfg|
                        if cfg.num_samples() > current.num_samples() {cfg} else { current }
                    ).expect("display produced no GL configs")
                })
                .map_err(|e| format!("display build failed: {e}"))?;

            // Finalize the Window from the config's requirements
            let window = maybe_window
                .ok_or_else(|| "display builder did not create a window".to_string())?;

            // Create the GL context + window surface
            let raw_window_handle = Some(window.window_handle()
                .map(|x| x.as_raw())
                .map_err(|e| format!("failed to obtain window handle: {e}"))?);
            let context_attrs = ContextAttributesBuilder::new()
                .with_profile(GlProfile::Compatibility)
                .with_context_api(ContextApi::OpenGl(Some(Version::new(2, 0))))
                .build(raw_window_handle);

            let gl_display = gl_config.display();
            let not_current = unsafe {
                gl_display
                    .create_context(&gl_config, &context_attrs)
                    .map_err(|e| format!("failed to create GL context: {e}"))?
            };

            // Build surface attributes
            let attrs = window
                .build_surface_attributes(SurfaceAttributesBuilder::<WindowSurface>::new())
                .map_err(|e| format!("failed to build surface attributes: {e}"))?;

            let gl_surface = unsafe {
                gl_display
                    .create_window_surface(&gl_config, &attrs)
                    .map_err(|e| format!("failed to create window surface: {e}"))?
            };

            let gl_context = not_current.make_current(&gl_surface)
                .map_err(|e| format!("failed to make GL context current: {e}"))?;

            Ok(RenderBaseGlState { gl_surface, gl_context, window })
        })();

        // Store result; errors are checked by RenderBase::new after pump_app_events.
        match result {
            Ok(state) => self.state = Some(state),
            Err(e) => self.init_error = Some(e),
        }
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
                self.maybe_forward_event(WindowEvent::CloseRequested);
            }
            WindowEvent::Resized(_) => {
                if let Some(RenderBaseGlState { window, gl_context, gl_surface }) = &self.state {
                    window.resize_surface(gl_surface, gl_context);
                }
            }

            // Fill the event buffer for everything else
            _ => self.maybe_forward_event(event)
        }
    }
}

