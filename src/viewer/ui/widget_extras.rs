//! A submodule of the viewer's UI holding extra widget creation helpers.
use std::ops::RangeInclusive;

use egui::color_picker::Alpha;
use egui_winit::egui;

use crate::viewer::ui::MAX_SPAN_WIDTH;

/// Adds a grid row with a label and a numeric input bounded to `range`.
pub(super) fn name_scalar_row<Num: egui::emath::Numeric>(
    ui: &mut egui::Ui,
    name: &str,
    value: &mut Num,
    speed: f64,
    range: RangeInclusive<Num>,
)
{
    ui.label(name);
    ui.horizontal_top(|ui| {
        ui.add_sized(
            [ui.available_width().min(MAX_SPAN_WIDTH), ui.spacing().interact_size.y],
            egui::DragValue::new(value).speed(speed).range(range)
        )
    });
    ui.end_row();
}

/// Adds a grid row with a label and a numeric input per value, each bounded to the range at the
/// same index of `ranges`.
pub(super) fn name_array_row<const NUM_VALUES: usize, Num: egui::emath::Numeric>(
    ui: &mut egui::Ui,
    name: &str,
    values: &mut [Num; NUM_VALUES],
    speed: f64,
    ranges: Option<&[RangeInclusive<Num>; NUM_VALUES]>,
)
{
    ui.label(name);

    let interact_height = ui.spacing().interact_size.y;
    let available_width = ui.available_width().min(MAX_SPAN_WIDTH);

    let x_spacing = ui.spacing_mut().item_spacing.y;
    ui.spacing_mut().item_spacing.x = x_spacing;
    let per_element_width = (available_width - x_spacing * (NUM_VALUES - 1) as f32) / NUM_VALUES as f32;
    ui.horizontal_top(|ui| {
        for (i, value) in values.iter_mut().enumerate() {
            let mut drag = egui::DragValue::new(value).speed(speed);
            if let Some(ranges) = ranges {
                drag = drag.range(ranges[i].clone());
            }

            ui.add_sized([per_element_width, interact_height], drag);
        }
    });
    ui.end_row();
}

/// Adds a grid row with a label and an RGBA color picker.
pub(super) fn name_rgba_row(ui: &mut egui::Ui, name: &str, rgba: &mut [f32; 4]) {
    ui.label(name);
    egui::color_picker::color_edit_button_rgba(ui, bytemuck::must_cast_mut(rgba), Alpha::OnlyBlend);
    ui.end_row();
}

/// Adds a grid row with a label and an RGB color picker.
pub(super) fn name_rgb_row(ui: &mut egui::Ui, name: &str, rgb: &mut [f32; 3]) {
    ui.label(name);
    egui::color_picker::color_edit_button_rgb(ui, rgb);
    ui.end_row();
}

/// Single-line search field with a popup that lists the names starting with the typed text.
#[derive(Debug)]
pub(super) struct SearchBar {
    text: String,
    hint_text: String,
}

impl SearchBar {
    /// Creates an empty search bar that shows `hint_text` while the field is empty.
    pub(super) fn new(hint_text: impl Into<String>) -> Self {
        Self { hint_text: hint_text.into(), text: String::new() }
    }

    /// Shows the search field and, while it holds text, the popup of matching `names`.
    /// Returns the name clicked in the popup and then clears the field.
    pub(super) fn show<'a>(
        &mut self,
        ui: &mut egui::Ui,
        names: impl IntoIterator<Item = &'a str>,
    ) -> Option<&'a str>
    {
        let response = ui.add(egui::TextEdit::singleline(&mut self.text)
            .hint_text(&self.hint_text)
        );
        let id = response.id.with("popup");
        let selected = egui::Popup::new(
            id, ui.ctx().clone(), response.rect,
            egui::LayerId::new(egui::Order::Foreground, id)
        )
            .kind(egui::PopupKind::Menu)
            .open(!self.text.is_empty())
            .show(|ui|
        {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            let mut matches = names.into_iter()
                .filter(|name| name.starts_with(self.text.as_str()))
                .peekable();

            if matches.peek().is_none() {
                ui.label("No match");
                return None;
            }
            matches.find(|name| ui.button(*name).clicked())
        })
            .and_then(|response| response.inner);

        if selected.is_some() {
            self.text.clear();
        }
        selected
    }

    /// Empties the search field, which also closes the popup.
    pub(super) fn clear(&mut self) {
        self.text.clear();
    }
}
