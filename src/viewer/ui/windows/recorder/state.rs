//! A submodule of the viewer's UI representing the state recorder window definitions.
use std::sync::{Arc, Mutex};

use egui_winit::egui;

use crate::viewer::ViewerSharedState;

/// The smallest height of the plot area, in points.
const MIN_PLOT_HEIGHT: f32 = 100.0;

/// The largest height of the plot area, in points.
const MAX_PLOT_HEIGHT: f32 = 200.0;

/// The corner radius of the plot area, in points.
const PLOT_CORNER_RADIUS: f32 = 10.0;

/// The UI-related state and data of the state recorder window.
#[derive(Default)]
pub struct StateRecorderData {
    /// Whether the window is open.
    pub open: bool,
    /// The largest number of samples to record. Unused under [`LimitMode::NoLimit`].
    pub num_timesteps: u32,
    /// The way number of samples are limited.
    pub limit_mode: LimitMode,
}

/// Specifies how samples are limited inside [`StateRecorderData`].
#[derive(Default, PartialEq, Clone, Copy)]
pub enum LimitMode {
    /// Samples are unlimited.
    #[default]
    NoLimit,
    /// Samples are limited such that the recording stops when the limit is reached.
    StopRecording,
    /// Samples are limited such that new samples start overriding the oldest samples when
    /// the limit is reached.
    DropOldest,
}

impl LimitMode {
    const ALL: [Self; 3] = [Self::NoLimit, Self::StopRecording, Self::DropOldest];

    fn label(self) -> &'static str {
        match self {
            Self::NoLimit => "No limit",
            Self::StopRecording => "Stop recording after",
            Self::DropOldest => "Drop oldest after",
        }
    }
}

/// Shows the state recorder window when `state_recorder_data` marks it as open.
pub fn show(
    ui: &mut egui::Ui,
    state_recorder_data: &mut StateRecorderData,
    _viewer_shared_state: &Arc<Mutex<ViewerSharedState>>
) {
    egui::Window::new("State recorder")
        .id(egui::Id::new("mujoco_rs_state_recorder_window"))
        .open(&mut state_recorder_data.open)
        .show(ui, |ui|
    {
        egui::panel::Panel::top("mjrs_sr_tbp").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Save").clicked() {

                    }

                    if ui.button("Load").clicked() {

                    }
                });
            });
        });

        let row_height = ui.spacing().interact_size.y + ui.spacing().item_spacing.y;
        let plot_height = (ui.available_height() - row_height)
            .clamp(MIN_PLOT_HEIGHT, MAX_PLOT_HEIGHT);
        ui.add_space((ui.available_height() - row_height - plot_height).max(0.0));

        ui.horizontal(|ui| {
            ui.label("Number of samples");

            egui::ComboBox::from_id_salt("mjrs_cmb_srd_lm")
                .selected_text(state_recorder_data.limit_mode.label())
                .width(125.0)
                .show_ui(ui, |ui|
            {
                for mode in LimitMode::ALL {
                    ui.selectable_value(
                        &mut state_recorder_data.limit_mode,
                        mode,
                        mode.label()
                    );
                }
            });

            ui.add_enabled(
                state_recorder_data.limit_mode != LimitMode::NoLimit,
                egui::DragValue::new(&mut state_recorder_data.num_timesteps).suffix(" samples")
            );
        });

        let (response, painter) = ui.allocate_painter(
            egui::vec2(ui.available_width(), plot_height), egui::Sense::click_and_drag()
        );
        painter.rect(
            response.rect, PLOT_CORNER_RADIUS, ui.visuals().extreme_bg_color,
            ui.visuals().widgets.noninteractive.bg_stroke, egui::StrokeKind::Inside
        );
    });
}
