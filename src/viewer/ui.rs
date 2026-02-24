//! Implementation of the interface for use in the viewer.
use egui_glow::glow::{self, HasContext};
use egui_winit::winit::event::WindowEvent;
use glutin::display::{Display, GlDisplay};
use egui_winit::winit::window::Window;
use egui::{FontId, RichText};
use egui_winit::egui;
use egui_winit;

use crate::util::LockUnpoison;
use crate::cast_mut_info;

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::ffi::CString;
use std::fmt::Debug;
use std::ops::Deref;

use crate::wrappers::mj_visualization::{
    MjvOption, MjvCamera, MjtCamera, MjvScene
};
use crate::wrappers::mj_model::{MjModel, MjtObj, MjtJoint};
use crate::viewer::{ViewerSharedState, ViewerStatusBit};
use crate::wrappers::mj_data::MjData;
use crate::mujoco_c::mjNGROUP;

const MAIN_FONT: FontId = FontId::proportional(15.0);
const HEADING_FONT: FontId = FontId::proportional(20.0);
const HEADING_POST_SPACE: f32 = 5.0;
const BUTTON_SPACING_X: f32 = 10.0;
const BUTTON_SPACING_Y: f32 = 5.0;
const BUTTON_ROUNDING: f32 = 50.0;

const SIDE_PANEL_DEFAULT_WIDTH: f32 = 250.0;
const TOGGLE_LABEL_HEIGHT_EXTRA_SPACE: f32 = 20.0;
const SIDE_PANEL_PAD: f32 = 10.0;

/// Maps [`MjtRndFlag`](crate::wrappers::mj_visualization::MjtRndFlag) to their string
const GL_EFFECT_MAP: [&str; 11] = [
    "Shadow",
    "Wireframe",
    "Reflection",
    "Additive",
    "Skybox",
    "Fog",
    "Haze",
    "Depth",
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

/// Type alias for a user-provided UI callback function.
pub(crate) type UiCallback<M> = Box<dyn FnMut(&egui::Context, &mut MjData<M>)>;

/// Type alias for a detached (from state) user-provided UI callback function.
pub(crate) type UiCallbackDetached = Box<dyn FnMut(&egui::Context)>;

/// Viewer user interface context.
pub(crate) struct ViewerUI<M: Deref<Target = MjModel>> {
    egui_ctx: egui::Context,
    state: egui_winit::State,
    painter: egui_glow::Painter,
    gl: Arc<egui_glow::glow::Context>,
    events: VecDeque<UiEvent>,
    camera_names: Vec<String>,
    actuator_names: Vec<String>,
    joint_name_id: Vec<(String, usize)>,
    equality_names: Vec<String>,
    model: M,
    user_ui_callbacks: Vec<UiCallback<M>>,
    user_ui_callbacks_detached: Vec<UiCallbackDetached>,

    // Window toggles.
    // Note that these are bool for easier integration with egui.
    // We assumed they won't take up too much extra space, since
    // running multiple viewers isn't what most people will do
    // and even then this is minor.
    actuator_window: bool,
    joint_window: bool,
    equality_window: bool,
    group_window: bool
}

impl<M: Deref<Target = MjModel>> ViewerUI<M> {
    /// Create a new [`ViewerUI`] instance for the specific winit window.
    pub(crate) fn new(model: M, window: &Window, display: &Display) -> Self {
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

        let camera_names = (0..model.ncam()).map(|i| {
            if let Some(name) = model.id_to_name(MjtObj::mjOBJ_CAMERA, i as i32) {
                name.to_string()
            } else { format!("Camera {i}") }
        }).collect();

        let actuator_names = (0..model.nu()).map(|i| {
            if let Some(name) = model.id_to_name(MjtObj::mjOBJ_ACTUATOR, i as i32) {
                name.to_string()
            } else { format!("Actuator {i}") }
        }).collect();

        let joint_name_id = (0..model.njnt()).filter_map(|i| {
            match model.jnt_type()[i as usize] {
                MjtJoint::mjJNT_SLIDE | MjtJoint::mjJNT_HINGE => {
                    let name = if let Some(name) = model.id_to_name(MjtObj::mjOBJ_JOINT, i as i32) {
                        name.to_string()
                    } else { format!("Joint {i}") };
                    Some((name, i as usize))
                }
                _ => None
            }
        }).collect();

        let equality_names = (0..model.neq()).map(|i| {
            if let Some(name) = model.id_to_name(MjtObj::mjOBJ_EQUALITY, i as i32) {
                name.to_string()
            } else { format!("Equality {i}") }
        }).collect();

        let painter = egui_glow::Painter::new(
            gl.clone(),
            "",
            None,
            false
        ).unwrap();

        Self {
            egui_ctx, state, painter, gl, events: VecDeque::new(),
            camera_names, actuator_names, joint_name_id, equality_names,
            model,
            user_ui_callbacks: Vec::new(),
            user_ui_callbacks_detached: Vec::new(),
            actuator_window: false,
            joint_window: false,
            equality_window: false,
            group_window: false
        }
    }

    /// Handles winit input events.
    pub(crate) fn handle_events(&mut self, window: &Window, event: &WindowEvent) {
        let _ = self.state.on_window_event(&window, event);  // ignore response as it can be obtained later.
    }

    /// Gains scoped access to [`egui::Context`] for dealing with custom initialization
    /// (e.g., loading in images).
    pub(crate) fn with_egui_ctx<F>(&mut self, once_fn: F)
        where F: FnOnce(&egui::Context)
    {
        once_fn(&mut self.egui_ctx)
    }

    /// Draws the UI to the viewport.
    pub(crate) fn process(
        &mut self,
        window: &Window, status: &mut ViewerStatusBit,
        scene: &mut MjvScene<M>, options: &mut MjvOption,
        camera: &mut MjvCamera,
        shared_viewer_state: &Arc<Mutex<ViewerSharedState<M>>>
    ) -> f32 {
        // Viewport reservations, which will be excluded from MuJoCo's viewport.
        // This way MuJoCo won't draw over the UI.
        let mut left = 0.0;

        // Process the UI
        let raw_input = self.state.take_egui_input(&window);
        let full_output = self.egui_ctx.run(raw_input, |ctx| {
            if status.contains(ViewerStatusBit::UI) {
                egui::SidePanel::new(egui::panel::Side::Left,"interface_panel")
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
                        egui::CollapsingHeader::new(RichText::new("Basic").font(HEADING_FONT))
                            .default_open(true)
                            .show(ui, |ui|
                        {
                            if ui.add(egui::Button::new(
                                RichText::new("Quit").font(MAIN_FONT)
                            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                self.events.push_back(UiEvent::Close);
                            }
                        });
                        
                        /* UI toggles */
                        egui::CollapsingHeader::new(RichText::new("UI").font(HEADING_FONT))
                            .default_open(true)
                            .show(ui, |ui|
                        {
                            // Normal toggles
                            ui.horizontal_wrapped(|ui| {
                                let mut selected = status.contains(ViewerStatusBit::HELP);
                                ui.toggle_value(&mut selected, RichText::new("Help").font(MAIN_FONT));
                                status.set(ViewerStatusBit::HELP, selected);

                                selected = window.fullscreen().is_some();
                                if ui.toggle_value(&mut selected, RichText::new("Fullscreen").font(MAIN_FONT)).clicked() {
                                    self.events.push_back(UiEvent::Fullscreen);
                                };

                                // VSync
                                let mut selected = status.contains(ViewerStatusBit::VSYNC);
                                if ui.toggle_value(&mut selected, RichText::new("V-Sync").font(MAIN_FONT)).clicked() {
                                    self.events.push_back(UiEvent::VSyncToggle);
                                };
                                status.set(ViewerStatusBit::VSYNC, selected);

                                // Info menu (FPS, time, etc.)
                                let mut selected = status.contains(ViewerStatusBit::INFO);
                                ui.toggle_value(&mut selected, RichText::new("Info").font(MAIN_FONT));
                                status.set(ViewerStatusBit::INFO, selected);
                            });

                            ui.separator();

                            // Window toggles
                            ui.horizontal_wrapped(|ui| {
                                ui.toggle_value(&mut self.actuator_window, RichText::new("Actuator").font(MAIN_FONT));
                                ui.toggle_value(&mut self.joint_window, RichText::new("Joint").font(MAIN_FONT));
                                ui.toggle_value(&mut self.equality_window, RichText::new("Equality").font(MAIN_FONT));
                                ui.toggle_value(&mut self.group_window, RichText::new("Group").font(MAIN_FONT));
                            });

                            ui.separator();

                            ui.horizontal_wrapped(|ui| {
                                // Warnings
                                ui.collapsing(RichText::new("Warnings").font(MAIN_FONT), |ui| {
                                    // Non-realtime factor warning
                                    let mut selected = status.contains(ViewerStatusBit::WARN_REALTIME);
                                    ui.checkbox(&mut selected, RichText::new("Realtime factor").font(MAIN_FONT));
                                    status.set(ViewerStatusBit::WARN_REALTIME, selected);
                                });
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
                                let enumerated = camera.type_.try_into().unwrap_or_else(|_| {
                                    panic!("failed to convert {} into MjtCamera", camera.type_)
                                });
                                let mut current_cam_name = match enumerated {
                                    MjtCamera::mjCAMERA_FIXED => &self.camera_names[camera.fixedcamid as usize],
                                    MjtCamera::mjCAMERA_TRACKING => "Tracking",
                                    MjtCamera::mjCAMERA_FREE => "Free",
                                    MjtCamera::mjCAMERA_USER => "User",
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

                            ui.add_space(5.0);

                            // Copy camera state to clipboard as XML
                            if ui.add(egui::Button::new(
                                RichText::new("Print camera").font(MAIN_FONT)
                            ).corner_radius(BUTTON_ROUNDING)).clicked() {
                                let gl_cams = scene.camera();
                                let fwd = gl_cams[0].forward;
                                let up = gl_cams[0].up;

                                // right = forward × up
                                let right = [
                                    fwd[1] * up[2] - fwd[2] * up[1],
                                    fwd[2] * up[0] - fwd[0] * up[2],
                                    fwd[0] * up[1] - fwd[1] * up[0],
                                ];

                                // Average position of left and right eye cameras
                                let pos = [
                                    (gl_cams[0].pos[0] + gl_cams[1].pos[0]) / 2.0,
                                    (gl_cams[0].pos[1] + gl_cams[1].pos[1]) / 2.0,
                                    (gl_cams[0].pos[2] + gl_cams[1].pos[2]) / 2.0,
                                ];

                                println!(
                                    "<camera pos=\"{:.3} {:.3} {:.3}\" \
                                    xyaxes=\"{:.3} {:.3} {:.3} {:.3} {:.3} {:.3}\"/>",
                                    pos[0], pos[1], pos[2],
                                    right[0], right[1], right[2],
                                    up[0], up[1], up[2]
                                );
                                // TODO in the future, when
                                // clipboard copying feature doesn't crash:
                                // ctx.copy_text(xml);
                                // Until then ^^^.
                            }

                            ui.add_space(5.0);

                            ui.collapsing(RichText::new("Elements").font(MAIN_FONT), |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    debug_assert_eq!(
                                        options.flags.len(), VIS_OPT_MAP.len(),
                                        "visualization names don't match options length. This is a bug!"
                                    );
                                    for (flag, (enabled, flag_name)) in options.flags.iter_mut().zip(VIS_OPT_MAP).enumerate() {
                                        ui.toggle_value(cast_mut_info!(enabled, flag), flag_name);
                                    }
                                });
                                ui.separator();
                                egui::Grid::new("tree_flex_slide").show(ui, |ui| {
                                    ui.label("Tree depth");
                                    ui.add(egui::Slider::new(&mut options.bvh_depth, 0..=20));
                                    ui.end_row();
                                    ui.label("Flex layer");
                                    ui.add(egui::Slider::new(&mut options.flex_layer, 0..=10));
                                });
                            });

                            ui.collapsing(RichText::new("OpenGL effects").font(MAIN_FONT), |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    debug_assert_eq!(
                                        scene.flags_mut().len(), GL_EFFECT_MAP.len(),
                                        "OpenGL effect flag names don't match scene flags length. This is a bug!"
                                    );
                                    for (flag, (enabled, flag_name)) in scene.flags_mut().iter_mut().zip(GL_EFFECT_MAP).enumerate() {
                                        ui.toggle_value(
                                            cast_mut_info!(enabled, flag),
                                            flag_name
                                        );
                                    }
                                });
                            });
                        });

                        // Make the scroll bars span to the edges of the sidebar.
                        ui.take_available_space();
                    });

                    // Panel toggle info
                    ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                        ui.add_space(HEADING_POST_SPACE);
                        ui.heading("Toggle: X");
                    });

                    // Make the panel track its width on resize
                    left = ui.max_rect().max.x + SIDE_PANEL_PAD;
                });
            }

            /* Windows */
            // Controls window
            egui::Window::new("Actuator")
                .scroll(true)
                .open(&mut self.actuator_window)
                .show(ctx, |ui|
            {
                let data = &mut shared_viewer_state.lock_unpoison().data_passive;
                let ctrl_mut = data.ctrl_mut();
                egui::Grid::new("ctrl_grid").show(ui, |ui| {
                    debug_assert_eq!(
                        self.actuator_names.len(), ctrl_mut.len(),
                        "actuator names don't match num of actuators in model. This is a bug!"
                    );
                    for (((actuator_name, ctrl), range), limited) in self.actuator_names.iter()
                        .zip(ctrl_mut.iter_mut())
                        .zip(self.model.actuator_ctrlrange())
                        .zip(self.model.actuator_ctrllimited())
                    {
                        ui.label(RichText::new(actuator_name).font(MAIN_FONT));

                        let range_inc = if *limited {
                            range[0]..=range[1]
                        } else { -1.0..=1.0 };

                        ui.add(
                            egui::Slider::new(ctrl, range_inc)
                            .update_while_editing(false)
                        );
                        ui.end_row();
                    }
                });

                // Clear all actuator controls by setting them to 0
                if ui.button(RichText::new("Clear").font(MAIN_FONT)).clicked() {
                    ctrl_mut.fill(0.0);
                }
            });

            // Joints window
            egui::Window::new("Joint")
                .scroll(true)
                .open(&mut self.joint_window)
                .show(ctx, |ui|
            {
                egui::Grid::new("joint_grid").show(ui, |ui| {
                    let limiteds = self.model.jnt_limited();
                    let ranges = self.model.jnt_range();
                    let qpos_addresses = self.model.jnt_qposadr();
                    let data = &mut shared_viewer_state.lock_unpoison().data_passive;
                    let qpos = data.qpos();
                    for (name, index) in &self.joint_name_id
                    {
                        ui.label(RichText::new(name).font(MAIN_FONT));
                        let limited = limiteds[*index];
                        let range = ranges[*index];
                        let mut value = qpos[qpos_addresses[*index] as usize];
                        ui.add_enabled(false, egui::DragValue::new(&mut value));

                        if limited {
                            let [low, high] = range;
                            let value_scaled = ((value - low) / (high - low)).clamp(0.0, 1.0);
                            ui.add(egui::ProgressBar::new(value_scaled as f32));
                        }
                        else {
                            ui.label("no limit");
                        }
                        
                        ui.end_row();
                    }
                });
            });

            // Equalities window
            egui::Window::new("Equality")
                .scroll(true)
                .open(&mut self.equality_window)
                .show(ctx, |ui|
            {
                ui.horizontal_wrapped(|ui| {
                    let data = &mut shared_viewer_state.lock_unpoison().data_passive;
                    debug_assert_eq!(
                        self.equality_names.len(), data.eq_active_mut().len(),
                        "equality names length don't match the number of equalities found in model. This is a bug!"
                    );
                    for (equality_name, active) in self.equality_names.iter()
                        .zip(data.eq_active_mut())
                    {
                        ui.toggle_value(active, RichText::new(equality_name).font(MAIN_FONT));
                    }
                });
            });

            egui::Window::new("Group")
                .open(&mut self.group_window)
                .show(ctx, |ui|
            {
                egui::Grid::new("group_grid").show(ui, |ui| {
                    for i in 0..mjNGROUP as usize {
                        ui.toggle_value(cast_mut_info!(&mut options.geomgroup[i], i), format!("Geom {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.sitegroup[i], i), format!("Site {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.jointgroup[i], i), format!("Joint {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.tendongroup[i], i), format!("Tendon {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.actuatorgroup[i], i), format!("Actuator {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.flexgroup[i], i), format!("Flex {i}"));
                        ui.toggle_value(cast_mut_info!(&mut options.skingroup[i], i), format!("Skin {i}"));
                        ui.end_row();
                    }
                });
            });

            /* User-defined UI callbacks */
            // Callbacks that receive the egui context and MjData passive instance
            for callback in self.user_ui_callbacks.iter_mut() {
                callback(ctx, &mut shared_viewer_state.lock_unpoison().data_passive);
            }
            // Callbacks that only receive the egui context
            for callback in self.user_ui_callbacks_detached.iter_mut() {
                callback(ctx);
            }
        });

        // Prevent window interactions when covering egui widgets
        self.state.handle_platform_output(&window, full_output.platform_output);

        // Tessellate
        let pixels_per_point = full_output.pixels_per_point;
        let textures_delta = &full_output.textures_delta;
        let clipped_primitives = self.egui_ctx.tessellate(full_output.shapes, pixels_per_point);

        // Paint the menu
        self.painter.paint_and_update_textures(
            window.inner_size().into(),
            pixels_per_point,
            &clipped_primitives,
            textures_delta
        );

        left * pixels_per_point
    }

    /// Checks whether the UI is focused (e.g., typing).
    pub(crate) fn focused(&self) -> bool {
        self.egui_ctx.memory(|ui| ui.focused().is_some())
    }

    /// Checks whether the mouse is over the UI.
    pub(crate) fn covered(&self) -> bool {
        self.egui_ctx.is_pointer_over_area()
    }

    /// Checks whether the UI is currently being dragged.
    pub(crate) fn dragged(&self) -> bool {
        self.egui_ctx.dragged_id().is_some()
    }

    /// Prepares OpenGL for drawing 2D overlays.
    pub(crate) fn init_2d(&self) {
        let gl = &self.gl;
        unsafe { 
            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::CULL_FACE);
            gl.disable(glow::BLEND);
            gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
            gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
        }
    }

    /// Resets OpenGL state. This is needed for MuJoCo's renderer.
    pub(crate) fn reset(&mut self) {
        let gl = &self.gl;
        unsafe {
            // Disable shaders
            gl.use_program(None);

            // Unbind buffers
            gl.bind_vertex_array(None);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, None);
            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
        }
    }

    /// Drains events from queue. If no event is queued, [`None`] is returned.
    pub(crate) fn drain_events(&mut self) -> Option<UiEvent> {
        self.events.pop_front()
    }

    /// Adds a user-defined UI callback that will be invoked during UI rendering.
    /// The callback receives the egui context and can be used to create custom windows,
    /// panels, or other UI elements.
    pub(crate) fn add_ui_callback<F>(&mut self, callback: F)
    where
        F: FnMut(&egui::Context, &mut MjData<M>) + 'static
    {
        self.user_ui_callbacks.push(Box::new(callback));
    }

    /// Adds a detached user-defined UI callback that will be invoked during UI rendering.
    /// Unlike [`ViewerUI::add_ui_callback`], this method does not pass in the passive [`MjData`]
    /// instance, located in the shared state, thus avoiding mutex locking when not necessary.
    pub(crate) fn add_ui_callback_detached<F>(&mut self, callback: F)
    where
        F: FnMut(&egui::Context) + 'static
    {
        self.user_ui_callbacks_detached.push(Box::new(callback));
    }
}

/// Implement an empty shell to support use in [`MjViewer`](super::MjViewer).
impl<M: Deref<Target = MjModel>> Debug for ViewerUI<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ViewerUI {{ .. }}")
    }
}

pub(crate) enum UiEvent {
    Close,
    Fullscreen,
    ResetSimulation,
    AlignCamera,
    VSyncToggle
}
