//! A submodule of the viewer's UI holding extra widget creation helpers.
use std::ops::RangeInclusive;

use egui::ecolor::{gamma_from_linear, linear_from_gamma};
use egui::color_picker::Alpha;
use egui_winit::egui;

use crate::viewer::ui::MAX_SPAN_WIDTH;

/// Default number of widgets to display within a single page of [`WidgetPage`].
/// The latter is then free to change the default value.
pub const DEFAULT_WIDGETS_PER_ELEMENT: usize = 20;

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
        // Clamp only the user's edits, otherwise egui rewrites a value that was set out of range
        // by the user, even if technically valid (e.g., some floating-point values).
        ui.add_sized(
            [ui.available_width().min(MAX_SPAN_WIDTH), ui.spacing().interact_size.y],
            egui::DragValue::new(value).speed(speed).range(range).clamp_existing_to_range(false)
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
                drag = drag.range(ranges[i].clone()).clamp_existing_to_range(false);
            }

            ui.add_sized([per_element_width, interact_height], drag);
        }
    });
    ui.end_row();
}

/// Adds a grid row with a label and an RGBA color picker.
pub(super) fn name_rgba_row(ui: &mut egui::Ui, name: &str, rgba: &mut [f32; 4]) {
    ui.label(name);
    color_edit_button(ui, rgba, Alpha::OnlyBlend);
    ui.end_row();
}

/// Adds a grid row with a label and an RGB color picker.
pub(super) fn name_rgb_row(ui: &mut egui::Ui, name: &str, rgb: &mut [f32; 3]) {
    ui.label(name);
    let [r, g, b] = *rgb;
    let mut rgba = [r, g, b, 1.0];
    color_edit_button(ui, &mut rgba, Alpha::Opaque);
    rgb.copy_from_slice(&rgba[..3]);
    ui.end_row();
}

/// Shows a color picker button for a MuJoCo color: gamma-space RGB with straight alpha.
fn color_edit_button(ui: &mut egui::Ui, rgba: &mut [f32; 4], alpha: Alpha) {
    // egui's Rgba is linear with premultiplied alpha, while MuJoCo sends its colors to OpenGL as-is.
    let [r, g, b, a] = *rgba;
    let mut color = egui::Rgba::from_rgba_unmultiplied(
        linear_from_gamma(r), linear_from_gamma(g), linear_from_gamma(b), a
    );

    // Write back only an edit, so that the conversion round trip does not change a loaded color.
    if egui::color_picker::color_edit_button_rgba(ui, &mut color, alpha).changed() {
        let [r, g, b, a] = color.to_rgba_unmultiplied();
        *rgba = [gamma_from_linear(r), gamma_from_linear(g), gamma_from_linear(b), a];
    }
}

/// Pager that shows only the widgets of the current page, followed by buttons that change the page.
pub(super) struct WidgetPage<'p> {
    widgets_per_page: &'p mut usize,
    widgets_added: usize,
    page: &'p mut usize,
}

impl<'p> WidgetPage<'p> {
    /// Creates an empty pager that shows `page` with `widgets_per_page` of widgets on each page.
    pub(super) fn new(page: &'p mut usize, widgets_per_page: &'p mut usize) -> Self {
        Self { widgets_added: 0, page, widgets_per_page }
    }

    /// Tries to add a `widget` using `ui` ([`egui::Ui`]). The widget is only added if it belongs
    /// to the currently set page.
    /// Returns [`None`] when the widget is on a different page, otherwise returns
    /// whatever `widget` returned.
    pub(super) fn paged<R>(&mut self, ui: &mut egui::Ui, widget: impl FnOnce(&mut egui::Ui) -> R) -> Option<R> {
        let required_page = self.widgets_added / *self.widgets_per_page;
        self.widgets_added += 1;
        (required_page == *self.page).then(|| {
            widget(ui)
        })
    }

    /// Returns the number of elements that can still fit into the current page.
    ///
    /// # Example
    /// In the following example, the page index is 1 (second page), `x` means free space and `o` means taken.
    /// ```text
    /// |  o
    /// |  o
    /// |  o
    /// |  ----------- current page (page index == 1; second page)
    /// |  o
    /// |  x            <------------- free in this page, thus 2 is returned.
    /// |  x            <-------------
    /// |  -----------
    /// |  x
    /// |  x
    /// |  x
    /// ```
    pub(super) fn available_in_current_page(&self) -> usize {
        // Zero once the added widgets pass the end of the current page.
        ((*self.page + 1) * *self.widgets_per_page).saturating_sub(self.widgets_added)
    }

    /// Returns the start index of the current page.
    pub(super) fn page_local_offset(&self) -> usize {
        *self.page * *self.widgets_per_page
    }

    /// Increases the internal count of the total number of widgets, effectively adding fake widgets.
    pub(super) fn paged_empty(&mut self, num_widgets: usize) {
        self.widgets_added += num_widgets;
    }
}

impl egui::Widget for WidgetPage<'_> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        if self.widgets_added <= *self.widgets_per_page {
            return ui.allocate_response(egui::Vec2::ZERO, egui::Sense::empty());
        }

        ui.horizontal(|ui| {
            // Arrows ⬅ and ➡ are egui's special supported emojis.
            // See https://docs.rs/egui/0.36.1/egui/special_emojis/.
            if ui.button("⬅").clicked() {
                *self.page = self.page.saturating_sub(1);
            }
            if ui.button("➡").clicked() {
                *self.page = (*self.page + 1).min(self.widgets_added.div_ceil(*self.widgets_per_page) - 1);
            }
            if ui.button("Expand").clicked() {
                // Halving the page keeps the page valid and its first widget visible.
                *self.page /= 2;
                *self.widgets_per_page *= 2;
            }
            let page_count = self.widgets_added.div_ceil(*self.widgets_per_page);
            ui.label(format!("(Page {}/{page_count})", *self.page + 1));
        }).response
    }
}

/// Single-line search field that holds a name prefix for filtering a list.
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

    /// Returns the text in the search field, or `None` when the field is empty.
    pub(super) fn text(&self) -> Option<&str> {
        (!self.text.is_empty()).then_some(&self.text)
    }

    /// Shows the search field.
    pub(super) fn show(&mut self, ui: &mut egui::Ui) {
        ui.add(egui::TextEdit::singleline(&mut self.text)
            .hint_text(&self.hint_text)
        );
    }

    /// Empties the search field.
    pub(super) fn clear(&mut self) {
        self.text.clear();
    }
}
