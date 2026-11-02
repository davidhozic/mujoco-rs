//! A submodule of the viewer's UI representing the physics side panel section definitions.
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::widget_extras::{name_scalar_row, name_array_row};
use crate::viewer::ui::{HEADING_FONT, MAIN_FONT, MAX_SPAN_WIDTH};
use crate::mujoco_c::{mjNGROUP, mjMINIMP, mjMAXIMP, mjMINMU};
use crate::wrappers::mj_model::{MjtDisableBit, MjtEnableBit};
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;
use crate::set_flag;

use super::MODEL_SYNC_MESSAGE_TIMEOUT;

const MODEL_OPT_SYNC_WARNING: &str = "[WARNING] Physics options not synced!";

/// Maps integrator modes to their string representations
const INTEGRATOR_MAP: [&str; 5] = [
    "Euler",
    "RK4",
    "implicit",
    "implicitfast",
    "discrete"
];

/// Maps friction cone types to their string representations
const CONE_MAP: [&str; 2] = [
    "Pyramidal",
    "Elliptic"
];

/// Maps Jacobian types to their string representations
const JACOBIAN_MAP: [&str; 3] = [
    "Dense",
    "Sparse",
    "Auto"
];

/// Maps solver algorithms to their string representations
const SOLVER_MAP: [&str; 3] = [
    "PGS",
    "CG",
    "Newton"
];

/// Maps MuJoCo disable flag bits to their string representations
const DISABLE_FLAGS: &[(&str, MjtDisableBit)] = &[
    ("Constraint", MjtDisableBit::mjDSBL_CONSTRAINT),
    ("Equality", MjtDisableBit::mjDSBL_EQUALITY),
    ("Friction Loss", MjtDisableBit::mjDSBL_FRICTIONLOSS),
    ("Limit", MjtDisableBit::mjDSBL_LIMIT),
    ("Contact", MjtDisableBit::mjDSBL_CONTACT),
    ("Spring", MjtDisableBit::mjDSBL_SPRING),
    ("Damper", MjtDisableBit::mjDSBL_DAMPER),
    ("Gravity", MjtDisableBit::mjDSBL_GRAVITY),
    ("Clamp Ctrl", MjtDisableBit::mjDSBL_CLAMPCTRL),
    ("Warm Start", MjtDisableBit::mjDSBL_WARMSTART),
    ("Filter Parent", MjtDisableBit::mjDSBL_FILTERPARENT),
    ("Actuation", MjtDisableBit::mjDSBL_ACTUATION),
    ("Ref Safe", MjtDisableBit::mjDSBL_REFSAFE),
    ("Sensor", MjtDisableBit::mjDSBL_SENSOR),
    ("Mid Phase", MjtDisableBit::mjDSBL_MIDPHASE),
    ("Euler Damp", MjtDisableBit::mjDSBL_EULERDAMP),
    ("Auto Reset", MjtDisableBit::mjDSBL_AUTORESET),
    ("Native CCD", MjtDisableBit::mjDSBL_NATIVECCD),
    ("Island", MjtDisableBit::mjDSBL_ISLAND),
    ("Multi CCD", MjtDisableBit::mjDSBL_MULTICCD),
];
const _: () = assert!(DISABLE_FLAGS.len() == crate::mujoco_c::mjtDisableBit::mjNDISABLE as usize);

/// Maps MuJoCo enable flag bits to their string representations
const ENABLE_FLAGS: &[(&str, MjtEnableBit)] = &[
    ("Override", MjtEnableBit::mjENBL_OVERRIDE),
    ("Energy", MjtEnableBit::mjENBL_ENERGY),
    ("Forward Inverse", MjtEnableBit::mjENBL_FWDINV),
    ("Inverse Discrete", MjtEnableBit::mjENBL_INVDISCRETE),
    ("Sleep", MjtEnableBit::mjENBL_SLEEP),
    ("DiagExact", MjtEnableBit::mjENBL_DIAGEXACT),
    ("IPC", MjtEnableBit::mjENBL_IPC)
];
const _: () = assert!(ENABLE_FLAGS.len() == crate::mujoco_c::mjtEnableBit::mjNENABLE as usize);

/// Shows the physics section of the side panel.
pub fn show(ui: &mut egui::Ui, shared_viewer_state: &Arc<Mutex<ViewerSharedState>>) {
    egui::CollapsingHeader::new(RichText::new("Physics").font(HEADING_FONT))
        .show(ui, |ui|
    {
        let (opt_is_editable, mut options) = {
            let state = shared_viewer_state.lock_unpoison();
            (
                state.last_opt_sync_time().elapsed() < MODEL_SYNC_MESSAGE_TIMEOUT,
                state.data_passive.model().opt().clone(),
            )
        };
        if !opt_is_editable {
            ui.colored_label(egui::Color32::YELLOW, MODEL_OPT_SYNC_WARNING);
        }

        // Physics solver and method selectors (top level)
        egui::Grid::new("physics_enum_grid").num_columns(2).show(ui, |ui| {
            ui.label(RichText::new("Integrator").font(MAIN_FONT));
            let combo_width = ui.available_width().min(MAX_SPAN_WIDTH);

            let current_integrator = INTEGRATOR_MAP.get(options.integrator as usize).copied().unwrap_or("Unknown");
            let mut integrator_idx = options.integrator as usize;
            egui::ComboBox::from_id_salt("integrator_combo")
                .selected_text(current_integrator)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (i, integrator_name) in INTEGRATOR_MAP.iter().enumerate() {
                        ui.selectable_value(&mut integrator_idx, i, *integrator_name);
                    }
                });
            options.integrator = integrator_idx as i32;
            ui.end_row();

            ui.label(RichText::new("Cone").font(MAIN_FONT));
            let current_cone = CONE_MAP.get(options.cone as usize).copied().unwrap_or("Unknown");
            let mut cone_idx = options.cone as usize;
            egui::ComboBox::from_id_salt("cone_combo")
                .selected_text(current_cone)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (i, cone_name) in CONE_MAP.iter().enumerate() {
                        ui.selectable_value(&mut cone_idx, i, *cone_name);
                    }
                });
            options.cone = cone_idx as i32;
            ui.end_row();

            ui.label(RichText::new("Jacobian").font(MAIN_FONT));
            let current_jacobian = JACOBIAN_MAP.get(options.jacobian as usize).copied().unwrap_or("Unknown");
            let mut jacobian_idx = options.jacobian as usize;
            egui::ComboBox::from_id_salt("jacobian_combo")
                .selected_text(current_jacobian)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (i, jacobian_name) in JACOBIAN_MAP.iter().enumerate() {
                        ui.selectable_value(&mut jacobian_idx, i, *jacobian_name);
                    }
                });
            options.jacobian = jacobian_idx as i32;
            ui.end_row();

            ui.label(RichText::new("Solver").font(MAIN_FONT));
            let current_solver = SOLVER_MAP.get(options.solver as usize).copied().unwrap_or("Unknown");
            let mut solver_idx = options.solver as usize;
            egui::ComboBox::from_id_salt("solver_combo")
                .selected_text(current_solver)
                .width(combo_width)
                .show_ui(ui, |ui| {
                    for (i, solver_name) in SOLVER_MAP.iter().enumerate() {
                        ui.selectable_value(&mut solver_idx, i, *solver_name);
                    }
                });
            options.solver = solver_idx as i32;
            ui.end_row();
        });

        ui.collapsing(RichText::new("Algorithm parameters").font(MAIN_FONT), |ui| {
            egui::Grid::new("algo_param_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Timestep", &mut options.timestep, 1e-6, 0.0..=f64::INFINITY);
                name_scalar_row(ui, "Iterations", &mut options.iterations, 1.0, 1..=i32::MAX);
                name_scalar_row(ui, "Tolerance", &mut options.tolerance, 1e-8, 0.0..=f64::INFINITY);
                name_scalar_row(ui, "LS Iter", &mut options.ls_iterations, 1.0, 1..=i32::MAX);
                name_scalar_row(ui, "LS Tol", &mut options.ls_tolerance, 1e-8, 0.0..=f64::INFINITY);
                name_scalar_row(ui, "Noslip Iter", &mut options.noslip_iterations, 1.0, 0..=i32::MAX);
                name_scalar_row(
                    ui, "Noslip Tol", &mut options.noslip_tolerance, 1e-8,
                    0.0..=f64::INFINITY
                );
                name_scalar_row(ui, "CCD Iter", &mut options.ccd_iterations, 1.0, 1..=i32::MAX);
                name_scalar_row(ui, "CCD Tol", &mut options.ccd_tolerance, 1e-8, 0.0..=f64::INFINITY);
                name_scalar_row(
                    ui, "Sleep Tol", &mut options.sleep_tolerance, 1e-8,
                    0.0..=f64::INFINITY
                );
                name_scalar_row(ui, "SDF Iter", &mut options.sdf_iterations, 1.0, 1..=i32::MAX);
                name_scalar_row(ui, "SDF Init", &mut options.sdf_initpoints, 1.0, 0..=i32::MAX);
            });
        });

        ui.collapsing(RichText::new("Physics parameters").font(MAIN_FONT), |ui| {
            egui::Grid::new("phys_param_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Density", &mut options.density, 1e-3, 0.0..=f64::INFINITY);
                name_scalar_row(ui, "Viscosity", &mut options.viscosity, 1e-6, 0.0..=f64::INFINITY);
                name_scalar_row(ui, "Imp Ratio", &mut options.impratio, 1e-3, 0.0..=f64::INFINITY);
                name_array_row(ui, "Gravity", &mut options.gravity, 1e-3, None);
                name_array_row(ui, "Wind", &mut options.wind, 1e-3, None);
                name_array_row(ui, "Magnetic", &mut options.magnetic, 1e-3, None);
            });
        });

        ui.collapsing(RichText::new("Disable Flags").font(MAIN_FONT), |ui| {
            ui.horizontal_wrapped(|ui| {
                for (flag_name, flag_value) in DISABLE_FLAGS {
                    let mut is_enabled = (options.disableflags & (*flag_value as i32)) != 0;
                    if ui.toggle_value(&mut is_enabled, *flag_name).changed() {
                        set_flag!(options.disableflags, *flag_value as i32, is_enabled);
                    }
                }
            });
        });

        ui.collapsing(RichText::new("Enable Flags").font(MAIN_FONT), |ui| {
            ui.horizontal_wrapped(|ui| {
                for (flag_name, flag_value) in ENABLE_FLAGS {
                    let mut is_enabled = (options.enableflags & (*flag_value as i32)) != 0;
                    if ui.toggle_value(&mut is_enabled, *flag_name).changed() {
                        set_flag!(options.enableflags, *flag_value as i32, is_enabled);
                    }
                }
            });
        });

        ui.collapsing(RichText::new("Actuator Group Enable").font(MAIN_FONT), |ui| {
            ui.horizontal_wrapped(|ui| {
                for i in 0..mjNGROUP as usize {
                    let mask = 1 << i;
                    let mut is_enabled = (options.disableactuator & mask) == 0;
                    if ui.toggle_value(&mut is_enabled, format!("Act Group {i}")).changed() {
                        set_flag!(options.disableactuator, mask, !is_enabled);
                    }
                }
            });
        });

        ui.collapsing(RichText::new("Contact Override").font(MAIN_FONT), |ui| {
            egui::Grid::new("contact_override_grid").num_columns(2).show(ui, |ui| {
                name_scalar_row(ui, "Margin", &mut options.o_margin, 1e-5, 0.0..=f64::INFINITY);
                name_array_row(ui, "Sol Ref", &mut options.o_solref, 1e-5, None);
                name_array_row(ui, "Sol Imp", &mut options.o_solimp, 1e-3, Some(&[
                    mjMINIMP..=mjMAXIMP,
                    mjMINIMP..=mjMAXIMP,
                    0.0..=f64::INFINITY,
                    mjMINIMP..=mjMAXIMP,
                    1.0..=f64::INFINITY,
                ]));
                name_array_row(
                    ui, "Friction", &mut options.o_friction, 1e-3,
                    Some(&[const { mjMINMU..=f64::INFINITY }; 5])
                );
            });
        });
        let mut lock = shared_viewer_state.lock_unpoison();
        if lock.model_reloaded {
            return;
        }
        *lock.data_passive.model_opt_mut() = options;
    });
}
