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
