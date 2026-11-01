//! A submodule of the viewer's UI representing the equality window definitions.
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase};
use crate::viewer::ViewerSharedState;
use crate::viewer::ui::MAIN_FONT;
use crate::util::LockUnpoison;

pub(crate) fn names(model: &MjModel) -> impl Iterator<Item = String> {
    (0..model.neq()).map(|i| {
        if let Some(name) = model.id_to_name(MjtObjBase::mjOBJ_EQUALITY, i as usize) {
            name.to_string()
        } else { format!("Equality {i}") }
    })
}

pub(crate) fn show(
    ui: &mut egui::Ui, open: &mut bool, equality_names: &[String],
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    egui::Window::new("Equality")
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .open(open)
        .show(ui, |ui|
    {
        ui.horizontal_wrapped(|ui| {
            let mut lock = shared_viewer_state.lock_unpoison();
            if lock.model_reloaded {
                return;
            }
            let data = &mut lock.data_passive;
            debug_assert_eq!(
                equality_names.len(), data.eq_active_mut().len(),
                "equality names length don't match the number of equalities found in model. This is a bug!"
            );
            for (equality_name, active) in equality_names.iter()
                .zip(data.eq_active_mut())
            {
                ui.toggle_value(active, RichText::new(equality_name).font(MAIN_FONT));
            }
        });
    });
}
