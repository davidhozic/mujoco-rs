//! A submodule of the viewer's UI representing the state window definitions.
use std::ops::RangeInclusive;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::widget_extras::WidgetPage;
use crate::wrappers::mj_data::MjtState;
use crate::wrappers::mj_model::MjModel;
use crate::viewer::ui::SECONDARY_FONT;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

/// The default number of digits of precision that state values will be displayed with.
pub const DEFAULT_NUM_DIGITS_STATE_PRECISION: usize = 3;

/// Names of the states inside [`MjtState`], aligned with the masks inside `MjtState`.
const STATE_NAMES: [&str; MjtState::mjNSTATE as usize] = [
    "Time",
    "Position (qpos)",
    "Velocity (qvel)",
    "Actuator activation (act)",
    "History",
    "Warmstart",
    "Control (ctrl)",
    "Generalized force (qfrc_applied)",
    "Cartesian force (xfrc_applied)",
    "Active equalities (eq_active)",
    "Mocap position (mocap_pos)",
    "Mocap orientation (mocap_quat)",
    "User data (userdata)",
    "Plugin",
];

/// Number of columns that the state window has for its checkboxes section.
const NUM_STATE_WINDOW_COLUMNS: usize = 3;

/// The range of digits of precision that state values can be displayed with.
const STATE_PRECISION_RANGE: RangeInclusive<usize> = 0..=15;  // About matches f64's precision.

/// Returns the size of each state element of `model`, in [`MjtState`] bit order.
pub fn sizes(model: &MjModel) -> impl Iterator<Item = usize> {
    (0..MjtState::mjNSTATE as u32).map(|bit| model.state_size(1 << bit))
}

/// Shows the state window when `open` is set.
///
/// # Panics
/// Panics when `state_sizes` does not include correct sizes for `state_window_enables`.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool,
    state_window_enables: &mut [bool; MjtState::mjNSTATE as usize], state_sizes: &[usize],
    state_precision_digits: &mut usize, state_pages: &mut [(usize, usize); MjtState::mjNSTATE as usize],
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    egui::Window::new("State")
        .id(egui::Id::new("mujoco_rs_state_window"))
        .open(open)
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .show(ui, |ui|
    {
        /* States selector */
        egui::Grid::new("mujoco_rs_state_window")
            .num_columns(NUM_STATE_WINDOW_COLUMNS)
            .show(ui, |ui|
        {
            for (i, (state_name, enabled)) in STATE_NAMES.into_iter()
                .zip(state_window_enables.iter_mut()).enumerate()
            {
                ui.checkbox(enabled, RichText::new(state_name).font(SECONDARY_FONT));
                if (i + 1).is_multiple_of(NUM_STATE_WINDOW_COLUMNS) {
                    ui.end_row();
                }
            }
        });

        ui.separator();

        /* Precision selector */
        ui.horizontal(|ui| {
            ui.label(RichText::new("Number of decimal places").font(SECONDARY_FONT));
            ui.add(egui::DragValue::new(state_precision_digits).range(STATE_PRECISION_RANGE));
        });

        /* States display */
        let bitmask = state_window_enables.iter().enumerate()
            .filter(|(_, enabled)| **enabled)
            .fold(0, |mask, (i, _)| mask | (1 << i));

        let state_box = {
            let lock = shared_viewer_state.lock_unpoison();
            if lock.model_reloaded {
                return;
            }
            lock.data_passive.state(bitmask)
        };

        let mut state = &state_box[..];
        egui::Grid::new("mujoco_rs_state_grid")
            .show(ui, |ui|
        {
            let mut local_state;
            for (((size, name), (page, per_page)), _) in state_sizes.iter()
                .zip(STATE_NAMES)
                .zip(state_pages.iter_mut())
                .zip(state_window_enables.iter())
                .filter(|(_, enabled)| **enabled)
            {
                let mut pager = WidgetPage::new(page, per_page);
                (local_state, state) = state.split_at(*size);

                ui.label(RichText::new(name).font(SECONDARY_FONT));

                // Efficiently page by pre-calculating offsets.
                let start = pager.page_local_offset();
                pager.paged_empty(start);
                let limit = pager.available_in_current_page();
                for ls in local_state.iter().skip(start).take(limit) {
                    pager.paged(ui, |ui| {
                        egui::Frame::canvas(ui.style()).show(ui, |ui| {
                            ui.label(
                                RichText::new(format!("{ls:.state_precision_digits$?}"))
                                    .font(SECONDARY_FONT)
                            );
                        });
                    });
                }

                // Fill the rest of the widgets we were supposed to add above. Substracted from start
                // as we already filled that space and from limit since we added that much above.
                // See the command at page_empty use for the sensor window inside sensor.rs
                // for more detailed explanation.
                pager.paged_empty(local_state.len().saturating_sub(start + limit));
                ui.add(pager);
                ui.end_row();
            }
        });
    });
}
