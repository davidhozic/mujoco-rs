//! Implementation of the interface for use in the viewer.
use egui_glow::glow::{FILL, FRONT_AND_BACK, HasContext};
use egui_winit::winit::event::WindowEvent;
use glutin::display::{Display, GlDisplay};
use egui_winit::winit::window::Window;
use egui::{FontId, RichText};
use egui_winit::egui;
use egui_winit;

use std::collections::VecDeque;
use std::ffi::CString;
use std::fmt::Debug;
use std::ops::Deref;
use std::sync::Arc;


use crate::wrappers::mj_visualization::{
    MjvOption, MjvCamera, MjtCamera
};
use crate::wrappers::mj_model::{MjModel, MjtObj};
use crate::viewer::ViewerStatusBit;

const MAIN_FONT: FontId = FontId::proportional(15.0);
const HEADING_FONT: FontId = FontId::proportional(20.0);
const HEADING_POST_SPACE: f32 = 5.0;
const BUTTON_SPACING_X: f32 = 10.0;
const BUTTON_SPACING_Y: f32 = 5.0;
const BUTTON_ROUNDING: f32 = 50.0;

const SIDE_PANEL_DEFAULT_WIDTH: f32 = 250.0;
const TOGGLE_LABEL_HEIGHT_EXTRA_SPACE: f32 = 20.0;
const SIDE_PANEL_SECOND_WIDTH: f32 = 10.0;

/// Maps [`MjtRndFlag`](crate::wrappers::mj_visualization::MjtRndFlag) to their string
const GL_EFFECT_MAP: [&str; 10] = [
    "Shadow",
    "Wireframe",
    "Reflection",
    "Additive",
    "Skybox",
    "Fog",
    "Haze",
    "Segment",
    "ID color",
    "Cull face"
];

/// Maps [`MjtVisFlag`](crate::wrappers::mj_visualization::MjtVisFlag) to their string
const VIS_OPT_MAP: [&str; 31] = [
    "Convex hull",
    "Texture",
    "Joint",
    "Camera",
    "Actuator",
    "Activation",
    "Light",
    "Tendon",
    "Range finder",
    "Constraint",
    "Inertia",
    "Scale inertia",
    "Perturbation force",
    "Perturbation object",
    "Contact point",
    "Island",
    "Contact force",
    "Contact split",
    "Transparent",
    "Auto-connect",
    "Center of mass",
    "Select",
    "Static",
    "Skin",
    "Flex vertex",
    "Flex edge",
    "Flex face",
    "Flex skin",
    "Body BVH",
    "Mesh BVH",
    "SDF iteration"
];

/// Maps [`MjtLabel`](crate::wrappers::mj_visualization::MjtLabel) to their string
const LABEL_TYPE_MAP: [&str; 17] = [
    "None",
    "Body",
    "Joint",
    "Geom",
    "Site",
    "Camera",
    "Light",
    "Tendon",
    "Actuator",
    "Constraint",
    "Flex",
    "Skin",
    "Selection",
    "Selection point",
    "Contact point",
    "Contact force",
    "Island"
];

/// Maps [`MjtFrame`](crate::wrappers::mj_visualization::MjtFrame) to their string
const FRAME_TYPE_MAP: [&str; 8] = [
    "None",
    "Body",
    "Geom",
    "Site",
    "Camera",
    "Light",
    "Contact",
    "World"
];

/// Viewer user interface context.
pub(crate) struct ViewerUI {
    egui_ctx: egui::Context,
    state: egui_winit::State,
    painter: egui_glow::Painter,
    gl: Arc<egui_glow::glow::Context>,
    events: VecDeque<UiEvent>,
    camera_names: Vec<String>,
}

impl ViewerUI {
    /// Create a new [`ViewerUI`] instance for the specific winit window.
    pub(crate) fn new<M: Deref<Target = MjModel>>(model: M, window: &Window, display: &Display) -> Self {
        let egui_ctx = egui::Context::default();
        let viewport_id = egui_ctx.viewport_id();

        let state = egui_winit::State::new(
            egui_ctx.clone(), viewport_id, &window,
            None, None, None
        );

        let get_addr = |s: &str| display.get_proc_address(
            &CString::new(s).unwrap()
        );
        let gl = unsafe { Arc::new(egui_glow::glow::Context::from_loader_function(get_addr)) };

        let cameras = (0..model.ncam()).map(|i| {
            if let Some(name) = model.id_to_name(MjtObj::mjOBJ_CAMERA, i) {
                name.to_string()
            } else { format!("Camera {i}") }
        }).collect();

        let painter = egui_glow::Painter::new(
            gl.clone(),
            "",
            None,
            false
        ).unwrap();
        Self { egui_ctx, state, painter, gl, events: VecDeque::new(), camera_names: cameras }
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(&window, event);  // ignore response as it can be obtained later.
    }

    /// Draws the UI to the viewport.
    pub(crate) fn process(
        &mut self,
        window: &Window, status: &mut ViewerStatusBit,
        scene_flags: &mut [u8], options: &mut MjvOption,
        camera: &mut MjvCamera
    ) -> f32 {
        let raw_input = self.state.take_egui_input(&window);

        // Viewport reservations, which will be excluded from MuJoCo's viewport.
        // This way MuJoCo won't draw over the UI.
        let mut left = 0.0;
        let full_output = self.egui_ctx.run(raw_input, |ctx| {
            if status.contains(ViewerStatusBit::UI) {
                egui::SidePanel::new(egui::panel::Side::Left, "left-panel")
                    .resizable(true)
                    .default_width(SIDE_PANEL_DEFAULT_WIDTH)
                    .show(ctx, |ui|
                {                  
                    // The menu
                    egui::ScrollArea::vertical()
                        .max_height(ui.available_height() - (TOGGLE_LABEL_HEIGHT_EXTRA_SPACE + HEADING_POST_SPACE + HEADING_FONT.size))
                        .show(ui, |ui|
                    {
                        // Make buttons have more space in the width
                        let spacing = ui.spacing_mut();
                        spacing.button_padding.x = BUTTON_SPACING_X;
                        spacing.button_padding.y = BUTTON_SPACING_Y;

                        /* Window controls */
                        egui::CollapsingHeader::new(RichText::new("Window").font(HEADING_FONT))
                            .default_open(true)
                            .show(ui, |ui|
                        {
                            if ui.add(egui::Button::new(
                                RichText::new("Quit").font(MAIN_FONT)
                            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                self.events.push_back(UiEvent::Close);
                            }
                        });
                        
                        /* Option */
                        egui::CollapsingHeader::new(RichText::new("Option").font(HEADING_FONT))
                            .default_open(true)
                            .show(ui, |ui|
                        {
                            ui.horizontal_wrapped(|ui| {
                                let mut selected = status.contains(ViewerStatusBit::HELP);
                                ui.toggle_value(&mut selected, RichText::new("Help").font(MAIN_FONT));
                                status.set(ViewerStatusBit::HELP, selected);

                                selected = window.fullscreen().is_some();
                                if ui.toggle_value(&mut selected, RichText::new("Fullscreen").font(MAIN_FONT)).clicked() {
                                    self.events.push_back(UiEvent::Fullscreen);
                                };
                            });
                        });

                        /* Simulation  */
                        egui::CollapsingHeader::new(RichText::new("Simulation").font(HEADING_FONT))
                            .default_open(true)
                            .show(ui, |ui|
                        {
                            ui.horizontal_wrapped(|ui| {
                                // Reset simulation
                                if ui.add(egui::Button::new(
                                    RichText::new("Reset").font(MAIN_FONT)
                                ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                    self.events.push_back(UiEvent::ResetSimulation);
                                }

                                // Align camera
                                if ui.add(egui::Button::new(
                                    RichText::new("Align").font(MAIN_FONT)
                                ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                    self.events.push_back(UiEvent::AlignCamera);
                                }
                            });
                        });

                        /* Visualization options */
                        egui::CollapsingHeader::new(RichText::new("Rendering").font(HEADING_FONT))
                            .default_open(false)
                            .show(ui, |ui|
                        {
                            egui::Grid::new("render_select_grid").show(ui, |ui| {
                                // Camera
                                ui.label(RichText::new("Camera").font(MAIN_FONT));
                                let mut current_cam_name = match unsafe {std::mem::transmute(camera.type_) } {
                                    MjtCamera::mjCAMERA_FIXED => &self.camera_names[camera.fixedcamid as usize],
                                    MjtCamera::mjCAMERA_TRACKING => "Tracking",
                                    MjtCamera::mjCAMERA_FREE => "Free",
                                    _ => unreachable!()
                                };

                                ui.menu_button(current_cam_name, |ui| {
                                    if ui.selectable_value(&mut current_cam_name, "Free", "Free").clicked() {
                                        camera.free();
                                    }

                                    for (id, name) in self.camera_names.iter().enumerate() {
                                        if ui.selectable_value(&mut current_cam_name, &name, name).clicked() {
                                            *camera = MjvCamera::new_fixed(id as u32);
                                        }
                                    }
                                });
                                ui.end_row();
                                
                                // Label
                                ui.label(RichText::new("Label").font(MAIN_FONT));
                                let mut current_lbl_ty = LABEL_TYPE_MAP[options.label as usize];
                                ui.menu_button(current_lbl_ty, |ui| {
                                    for (label_type_i, label_type) in LABEL_TYPE_MAP.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut current_lbl_ty,
                                            label_type, *label_type
                                        ).clicked() {
                                            options.label = label_type_i as i32;
                                        };
                                    }
                                    
                                });
                                ui.end_row();

                                // Frame
                                ui.label(RichText::new("Frame").font(MAIN_FONT));
                                let mut current_frm_ty = FRAME_TYPE_MAP[options.frame as usize];
                                ui.menu_button(current_frm_ty, |ui| {
                                    for (frame_type_i, frame_type) in FRAME_TYPE_MAP.iter().enumerate() {
                                        if ui.selectable_value(
                                            &mut current_frm_ty,
                                            frame_type, *frame_type
                                        ).clicked() {
                                            options.frame = frame_type_i as i32;
                                        };
                                    }
                                    
                                });
                                ui.end_row();
                            });

                            ui.collapsing("Elements", |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    for (flag, enabled) in &mut options.flags.iter_mut().enumerate() {
                                        if ui.toggle_value(&mut (*enabled == 1), VIS_OPT_MAP[flag]).clicked() {
                                            *enabled = if *enabled == 1 { 0 } else { 1 };
                                        }
                                    }
                                });
                            });

                            ui.collapsing("OpenGL effects", |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    for (flag, enabled) in scene_flags.iter_mut().enumerate() {
                                        if ui.toggle_value(&mut (*enabled == 1), GL_EFFECT_MAP[flag]).clicked() {
                                            *enabled = if *enabled == 1 { 0 } else { 1 };
                                        }
                                    }
                                });
                            });
                        });
                    });

                    // Panel toggle info
                    ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                        ui.add_space(HEADING_POST_SPACE);
                        ui.heading("Toggle: X");
                    });

                    // Make the panel track its width on resize
                    ui.take_available_space();
                });

                egui::SidePanel::new(egui::panel::Side::Left, "left-panel-pad")
                    .resizable(false)
                    .default_width(SIDE_PANEL_SECOND_WIDTH)
                    .show_separator_line(false)
                    .show(ctx, |ui|{
                        left = ui.max_rect().max.x;
                    });
            }
        });

        self.state.handle_platform_output(&window, full_output.platform_output);

        // Tesselate
        let pixels_per_point = full_output.pixels_per_point;
        let textures_delta = &full_output.textures_delta;
        let clipped_primitives = self.egui_ctx.tessellate(full_output.shapes, pixels_per_point);

        // Paint the menu
        unsafe { self.gl.polygon_mode(FRONT_AND_BACK, FILL) };
        self.painter.paint_and_update_textures(
            window.inner_size().into(),
            pixels_per_point,
            &clipped_primitives,
            textures_delta
        );

        left
    }

    /// Resets OpenGL state. This is needed for MuJoCo's renderer.
    pub(crate) fn reset(&mut self) {
        let gl = &self.gl;
        unsafe {
            gl.use_program(None);
        }
    }

    /// Drains events from queue. If no event is queued, [`None`] is returned.
    pub(crate) fn drain_events(&mut self) -> Option<UiEvent> {
        self.events.pop_front()
    }
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl Debug for ViewerUI {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}

pub(crate) enum UiEvent {
    Close,
    Fullscreen,
    ResetSimulation,
    AlignCamera
}
