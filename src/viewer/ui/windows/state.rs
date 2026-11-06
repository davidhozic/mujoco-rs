//! A submodule of the viewer's UI representing the state window definitions.
use std::ops::RangeInclusive;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::viewer::ui::widget_extras::{DEFAULT_WIDGETS_PER_ELEMENT, WidgetPage};
use crate::wrappers::mj_data::MjtState;
use crate::wrappers::mj_model::MjModel;
use crate::viewer::ui::SECONDARY_FONT;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

/// The default number of digits of precision that state values will be displayed with.
const DEFAULT_NUM_DIGITS_STATE_PRECISION: usize = 3;

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

/// Cached display information and user settings of the state window.
pub struct StateDisplayInfo {
    /// Whether the window is open.
    pub open: bool,
    /// Information of each state element, in [`MjtState`] bit order.
    elements: [StateElementInfo; MjtState::mjNSTATE as usize],
    /// The number of decimal places to display state values with.
    /// The rounding follows semantics of [`format`], with which the values
    /// are turned into a [`String`].
    precision_digits: usize,
}

impl StateDisplayInfo {
    /// Creates the information of a closed window with no model data.
    pub fn new() -> Self {
        Self {
            open: false,
            elements: std::array::from_fn(|_| StateElementInfo {
                size: 0,
                enabled: false,
                page: 0,
                widgets_per_page: DEFAULT_WIDGETS_PER_ELEMENT,
            }),
            precision_digits: DEFAULT_NUM_DIGITS_STATE_PRECISION,
        }
    }

    /// Rebuilds the model-dependent information from `model`.
    pub fn reset_to_model(&mut self, model: &MjModel) {
        self.elements.iter_mut().enumerate().for_each(|(bit, element)| {
            element.size = model.state_size(1 << bit);
            element.page = 0;
            element.widgets_per_page = DEFAULT_WIDGETS_PER_ELEMENT;
        });
    }
}

/// Display information and user settings of one state element.
struct StateElementInfo {
    /// Number of values in the element.
    size: usize,
    /// Whether the state window shows the values of the element.
    enabled: bool,
    /// Current page of the values.
    page: usize,
    /// Number of values per page.
    widgets_per_page: usize,
}

/// Shows the state window when `info` marks it as open.
///
/// # Panics
/// Panics when the cached sizes do not match the model of `shared_viewer_state`.
pub fn show(
    ui: &mut egui::Ui, info: &mut StateDisplayInfo, shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    let StateDisplayInfo { open, elements, precision_digits } = info;

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
            for (i, (state_name, element)) in STATE_NAMES.into_iter()
                .zip(elements.iter_mut()).enumerate()
            {
                ui.checkbox(&mut element.enabled, RichText::new(state_name).font(SECONDARY_FONT));
                if (i + 1).is_multiple_of(NUM_STATE_WINDOW_COLUMNS) {
                    ui.end_row();
                }
            }
        });

        ui.separator();

        /* Precision selector */
        ui.horizontal(|ui| {
            ui.label(RichText::new("Number of decimal places").font(SECONDARY_FONT));
            ui.add(egui::DragValue::new(precision_digits).range(STATE_PRECISION_RANGE));
        });

        /* States display */
        let bitmask = elements.iter().enumerate()
            .filter(|(_, element)| element.enabled)
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
            for (element, name) in elements.iter_mut()
                .zip(STATE_NAMES)
                .filter(|(element, _)| element.enabled)
            {
                let mut pager = WidgetPage::new(&mut element.page, &mut element.widgets_per_page);
                (local_state, state) = state.split_at(element.size);

                ui.label(RichText::new(name).font(SECONDARY_FONT));

                // Efficiently page by pre-calculating offsets.
                let start = pager.page_local_offset();
                pager.paged_empty(start);
                let limit = pager.available_in_current_page();
                for ls in local_state.iter().skip(start).take(limit) {
                    pager.paged(ui, |ui| {
                        egui::Frame::canvas(ui.style()).show(ui, |ui| {
                            ui.label(
                                RichText::new(format!("{ls:.precision_digits$?}"))
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
