//! A submodule of the viewer's UI representing the sensor window definitions.
use std::ops::RangeInclusive;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase, MjtJoint, MjtBias, MjtWrap};
use crate::wrappers::{MjtConDataField, MjtRayDataField, MjtSensor};
use crate::viewer::ui::widget_extras::{SearchBar, WidgetPage};
use crate::viewer::ui::{MAIN_FONT, SECONDARY_FONT};
use crate::wrappers::mj_primitive::MjtNum;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::{
    ComponentDisplayInfo, ElementDisplayInfo, INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y,
    QUAT_COMPONENTS, XYZ_COMPONENTS, actuator_joint_type
};

/// Width of the bar of a sensor value with a natural range, in points.
const SENSOR_BAR_WIDTH: f32 = 75.0;
/// Corner radius of the bar of a sensor value with a natural range, in points.
const SENSOR_BAR_CORNER_RADIUS: u8 = 3;

/// Initial number of widgets that an element shows on one page.
pub const MAX_WIDGETS_PER_ELEMENT: usize = 20;

/// Ensures, at compile-time, that information inside sensor types with multiple data fields and variants is not stale.
macro_rules! check_sensor_components {
    ($components:expr, $len:expr) => {
        const _: () = const {
            let mut i = 0;
            assert!($components.len() == $len as usize);
            while i < $components.len() {
                let e_type = $components[i].0;
                assert!(e_type as usize == i);
                i += 1;
            }
        };
    };
}

/// Maps MuJoCo contact sensor components to a readable name, the physical unit, sub component
/// axis names and the value range (if any) of the component.
const CONTACT_SENSOR_COMPONENTS: &[(u32, &str, &str, &[&str], Option<RangeInclusive<MjtNum>>)] = &[
    (MjtConDataField::mjCONDATA_FOUND as u32,   "found",   " contacts", &[""],          None),
    (MjtConDataField::mjCONDATA_FORCE as u32,   "force",   " N",        &XYZ_COMPONENTS, None),
    (MjtConDataField::mjCONDATA_TORQUE as u32,  "torque",  " N m",      &XYZ_COMPONENTS, None),
    (MjtConDataField::mjCONDATA_DIST as u32,    "dist",    " m",        &[""],          None),
    (MjtConDataField::mjCONDATA_POS as u32,     "pos",     " m",        &XYZ_COMPONENTS, None),
    (MjtConDataField::mjCONDATA_NORMAL as u32,  "normal",  "",          &XYZ_COMPONENTS, Some(-1.0..=1.0)),
    (MjtConDataField::mjCONDATA_TANGENT as u32, "tangent", "",          &XYZ_COMPONENTS, Some(-1.0..=1.0)),
    (MjtConDataField::mjCONDATA_LINVEL as u32,  "linvel",  " m/s",      &XYZ_COMPONENTS, None),
    (MjtConDataField::mjCONDATA_ANGVEL as u32,  "angvel",  " rad/s",    &XYZ_COMPONENTS, None),
];
check_sensor_components!(CONTACT_SENSOR_COMPONENTS, MjtConDataField::mjNCONDATA);

/// Maps MuJoCo rangefinder sensor components to a readable name, the physical unit, sub component
/// axis names and the value range (if any) of the component.
const RANGEFINDER_SENSOR_COMPONENTS: &[(u32, &str, &str, &[&str], Option<RangeInclusive<MjtNum>>)] = &[
    (MjtRayDataField::mjRAYDATA_DIST as u32,   "distance", " m", &[""],          None),
    (MjtRayDataField::mjRAYDATA_DIR as u32,    "dir",      "",   &XYZ_COMPONENTS, Some(-1.0..=1.0)),
    (MjtRayDataField::mjRAYDATA_ORIGIN as u32, "origin",   " m", &XYZ_COMPONENTS, None),
    (MjtRayDataField::mjRAYDATA_POINT as u32,  "point",    " m", &XYZ_COMPONENTS, None),
    (MjtRayDataField::mjRAYDATA_NORMAL as u32, "normal",   "",   &XYZ_COMPONENTS, Some(-1.0..=1.0)),
    (MjtRayDataField::mjRAYDATA_DEPTH as u32,  "depth",    " m", &[""],          None),
];
check_sensor_components!(RANGEFINDER_SENSOR_COMPONENTS, MjtRayDataField::mjNRAYDATA);

/// Names of the components of a `geomfromto` sensor, in `sensordata` order.
const SENSOR_FROMTO_COMPONENTS: [&str; 6] = ["x1", "y1", "z1", "x2", "y2", "z2"];

/// Names of the components of a `camprojection` sensor, in `sensordata` order.
const SENSOR_CAMPROJECTION_COMPONENTS: [&str; 2] = ["u", "v"];

/// Returns the display information of each sensor of `model`, in ID order.
pub fn display_infos(model: &MjModel) -> impl Iterator<Item = ElementDisplayInfo> {
    (0..model.nsensor() as usize).map(|sensor_id| {
        let name = model.id_to_name(MjtObjBase::mjOBJ_SENSOR, sensor_id)
            .map(|name| name.to_string())
            .unwrap_or_else(|| format!("Sensor {sensor_id}"));

        let adr_start = model.sensor_adr()[sensor_id] as usize;

        let sensor_type = model.sensor_type()[sensor_id];
        let sensor_dim = model.sensor_dim()[sensor_id] as usize;

        // A joint sensor measures a length on a slide joint and an angle on a hinge or ball joint.
        let objid = model.sensor_objid()[sensor_id] as usize;
        let objtype = model.sensor_objtype()[sensor_id];
        let is_slide = objtype == MjtObjBase::mjOBJ_JOINT && model.jnt_type()[objid] == MjtJoint::mjJNT_SLIDE;
        // A fixed tendon combines joint coordinates, so its unit depends on the joints and coefficients.
        let is_spatial_tendon = objtype == MjtObjBase::mjOBJ_TENDON
            && model.wrap_type()[model.tendon_adr()[objid] as usize] != MjtWrap::mjWRAP_JOINT;

        let labelled = |name: &str, labels: &'static [&'static str], range: Option<RangeInclusive<MjtNum>>, unit| -> Box<[ComponentDisplayInfo]> {
            Box::new([ComponentDisplayInfo { name: name.to_owned(), range, unit, sub_component_names: labels }])
        };
        let scalar = |name: &str, (unit, range)| -> Box<[ComponentDisplayInfo]> {
            Box::new([ComponentDisplayInfo { name: name.to_owned(), range, unit, sub_component_names: &[""] }])
        };
        let xyz = |name, unit| labelled(name, &XYZ_COMPONENTS, None, unit);
        let unit_xyz = |name| labelled(name, &XYZ_COMPONENTS, Some(-1.0..=1.0), "");

        let components = match sensor_type {
            MjtSensor::mjSENS_TOUCH => scalar("force", (" N", None)),
            MjtSensor::mjSENS_ACCELEROMETER => xyz("acceleration", " m/s^2"),
            MjtSensor::mjSENS_VELOCIMETER => xyz("velocity", " m/s"),
            MjtSensor::mjSENS_GYRO => xyz("angular velocity", " rad/s"),
            MjtSensor::mjSENS_FORCE => xyz("force", " N"),
            MjtSensor::mjSENS_TORQUE => xyz("torque", " N m"),
            // The unit is the unit of the magnetic option.
            MjtSensor::mjSENS_MAGNETOMETER => xyz("magnetic flux", ""),
            MjtSensor::mjSENS_CAMPROJECTION => labelled("projection", &SENSOR_CAMPROJECTION_COMPONENTS, None, " px"),

            MjtSensor::mjSENS_JOINTPOS => {
                let [low, high] = model.jnt_range()[objid];
                let range = (model.jnt_limited()[objid] && low < high).then_some(low..=high);
                scalar("position", (if is_slide { " m" } else { " rad" }, range))
            }
            // The value is the distance to the limit, not a position inside the joint range.
            MjtSensor::mjSENS_JOINTLIMITPOS => scalar("limit distance", (if is_slide { " m" } else { " rad" }, None)),
            MjtSensor::mjSENS_JOINTVEL => scalar("velocity", (if is_slide { " m/s" } else { " rad/s" }, None)),
            MjtSensor::mjSENS_JOINTLIMITVEL => scalar("limit velocity", (if is_slide { " m/s" } else { " rad/s" }, None)),
            MjtSensor::mjSENS_JOINTACTFRC => {
                let [low, high] = model.jnt_actfrcrange()[objid];
                let range = (model.jnt_actfrclimited()[objid] && low < high).then_some(low..=high);
                scalar("actuator force", (if is_slide { " N" } else { " N m" }, range))
            }
            MjtSensor::mjSENS_JOINTLIMITFRC => scalar("limit force", (if is_slide { " N" } else { " N m" }, None)),

            MjtSensor::mjSENS_TENDONPOS => {
                let [low, high] = model.tendon_range()[objid];
                let range = (model.tendon_limited()[objid] && low < high).then_some(low..=high);
                scalar("length", (if is_spatial_tendon { " m" } else { "" }, range))
            }
            MjtSensor::mjSENS_TENDONLIMITPOS => scalar("limit distance", (if is_spatial_tendon { " m" } else { "" }, None)),
            MjtSensor::mjSENS_TENDONVEL => scalar("velocity", (if is_spatial_tendon { " m/s" } else { "" }, None)),
            MjtSensor::mjSENS_TENDONLIMITVEL => scalar("limit velocity", (if is_spatial_tendon { " m/s" } else { "" }, None)),
            MjtSensor::mjSENS_TENDONACTFRC => scalar("actuator force", (if is_spatial_tendon { " N" } else { "" }, None)),
            MjtSensor::mjSENS_TENDONLIMITFRC => scalar("limit force", (if is_spatial_tendon { " N" } else { "" }, None)),

            // The sensor has one value per actuator output; an orientation actuator has one
            // output per axis.
            MjtSensor::mjSENS_ACTUATORPOS
            | MjtSensor::mjSENS_ACTUATORVEL
            | MjtSensor::mjSENS_ACTUATORFRC => {
                let joint_type = actuator_joint_type(model, objid);
                let is_slide = joint_type == Some(MjtJoint::mjJNT_SLIDE);
                let unit = match sensor_type {
                    _ if joint_type.is_none() => "",
                    MjtSensor::mjSENS_ACTUATORPOS => if is_slide { " m" } else { " rad" },
                    MjtSensor::mjSENS_ACTUATORVEL => if is_slide { " m/s" } else { " rad/s" },
                    _ => if is_slide { " N" } else { " N m" }
                };
                let name = match sensor_type {
                    MjtSensor::mjSENS_ACTUATORPOS => "position",
                    MjtSensor::mjSENS_ACTUATORVEL => "velocity",
                    _ => "force"
                };
                let force_limited = sensor_type == MjtSensor::mjSENS_ACTUATORFRC
                    && model.actuator_forcelimited()[objid]
                    && model.actuator_biastype()[objid] != MjtBias::mjBIAS_DCMOTOR;
                let [low, high] = model.actuator_forcerange()[objid];
                if sensor_dim == 3 {
                    labelled(name, &XYZ_COMPONENTS, (force_limited && high > 0.0).then_some(-high..=high), unit)
                } else {
                    scalar(name, (unit, (force_limited && low < high).then_some(low..=high)))
                }
            }

            MjtSensor::mjSENS_BALLQUAT | MjtSensor::mjSENS_FRAMEQUAT => {
                labelled("orientation", &QUAT_COMPONENTS, Some(-1.0..=1.0), "")
            }
            MjtSensor::mjSENS_BALLANGVEL => xyz("angular velocity", " rad/s"),

            MjtSensor::mjSENS_FRAMEPOS => xyz("position", " m"),
            MjtSensor::mjSENS_SUBTREECOM => xyz("center of mass", " m"),
            MjtSensor::mjSENS_FRAMEXAXIS => unit_xyz("x axis"),
            MjtSensor::mjSENS_FRAMEYAXIS => unit_xyz("y axis"),
            MjtSensor::mjSENS_FRAMEZAXIS => unit_xyz("z axis"),
            MjtSensor::mjSENS_GEOMNORMAL => unit_xyz("normal"),
            MjtSensor::mjSENS_FRAMELINVEL | MjtSensor::mjSENS_SUBTREELINVEL => xyz("linear velocity", " m/s"),
            MjtSensor::mjSENS_FRAMEANGVEL => xyz("angular velocity", " rad/s"),
            MjtSensor::mjSENS_FRAMELINACC => xyz("linear acceleration", " m/s^2"),
            MjtSensor::mjSENS_FRAMEANGACC => xyz("angular acceleration", " rad/s^2"),
            MjtSensor::mjSENS_SUBTREEANGMOM => xyz("angular momentum", " kg m^2/s"),

            // An "enclosed" sensor gives a signed distance to the site boundary.
            MjtSensor::mjSENS_INSIDESITE if model.sensor_intprm()[sensor_id][0] != 0 => scalar("distance", (" m", None)),
            MjtSensor::mjSENS_INSIDESITE => scalar("inside", ("", Some(0.0..=1.0))),
            MjtSensor::mjSENS_GEOMDIST => scalar("distance", (" m", None)),
            MjtSensor::mjSENS_GEOMFROMTO => labelled("segment", &SENSOR_FROMTO_COMPONENTS, None, " m"),
            MjtSensor::mjSENS_E_POTENTIAL => scalar("potential energy", (" J", None)),
            MjtSensor::mjSENS_E_KINETIC => scalar("kinetic energy", (" J", None)),
            MjtSensor::mjSENS_CLOCK => scalar("time", (" s", None)),

            MjtSensor::mjSENS_RANGEFINDER => bitmask_components(RANGEFINDER_SENSOR_COMPONENTS, model.sensor_intprm()[sensor_id][0]),
            MjtSensor::mjSENS_CONTACT => bitmask_components(CONTACT_SENSOR_COMPONENTS, model.sensor_intprm()[sensor_id][0]),

            MjtSensor::mjSENS_TACTILE => return ElementDisplayInfo::NameOnly { name },

            MjtSensor::mjSENS_USER
            | MjtSensor::mjSENS_PLUGIN => {
                (0..sensor_dim).map(|i| {
                    ComponentDisplayInfo {
                        name: if sensor_dim > 1 { format!("value {i}") } else { "value".to_owned() },
                        range: None,
                        unit: "",
                        sub_component_names: &[""],
                    }
                }).collect()
            }
        };

        let n_replicate = match sensor_type {
            MjtSensor::mjSENS_RANGEFINDER => {
                if objtype == MjtObjBase::mjOBJ_SITE {
                    1
                } else {
                    // MuJoCo multiplies the raw resolution (mjs_sensorDim) and checks its sign
                    // only when sensorsize is set.
                    let [width, height] = model.cam_resolution()[objid];
                    (width * height).max(0) as usize
                }
            }
            MjtSensor::mjSENS_CONTACT => model.sensor_intprm()[sensor_id][2] as usize,
            _ => 1
        };
        let info = ElementDisplayInfo::Full { name, adr_start, components, n_replicate };
        // Assert only in development, as this is something that is not left-up to ambiguity.
        // If this doesn't pass, we have a bug.
        // To anyone reading this, please report it if such a bug exists :).
        debug_assert_eq!(info.replica_len() * n_replicate, sensor_dim, "unexpected dimension of sensor type {sensor_type:?}");
        info
    })
}

/// Returns the display information of each component in `table` whose bit is set in `data_bitmask`.
fn bitmask_components(
    table: &[(u32, &str, &'static str, &'static [&'static str], Option<RangeInclusive<MjtNum>>)],
    data_bitmask: i32
)
    -> Box<[ComponentDisplayInfo]>
{
    table.iter()
        .filter(|(bit, ..)| data_bitmask & (1 << *bit) != 0)
        .map(|(_, data_name, unit, subcomponents, range)| {
            ComponentDisplayInfo {
                name: data_name.to_string(),
                unit,
                range: range.clone(),
                sub_component_names: subcomponents,
            }
        }).collect()
}

/// Shows the sensor window when `open` is set.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, sensor_info: &[ElementDisplayInfo], search_bar: &mut SearchBar,
    pages: &mut [(usize, usize)], shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    // Clear the text inside the window's search bar when the window is closed.
    if !*open {
        search_bar.clear();
    }

    egui::Window::new("Sensor")
        .id(egui::Id::new("mujoco_rs_sensor_window"))
        .open(open)
        .scroll(true)
        .drag_to_scroll(egui::scroll_area::DragScroll::Always)
        .show(ui, |ui|
    {
        let lock = shared_viewer_state.lock_unpoison();
        if lock.model_reloaded {
            return;
        }

        /* Search bar */
        search_bar.show(ui);

        /* Sensor widgets */
        let sensordata = lock.data_passive.sensordata();
        for (sensor_id, (info, (page, widgets_per_page))) in sensor_info.iter().zip(pages.iter_mut()).enumerate() {
            if let Some(query) = search_bar.text() && !info.name().starts_with(query) {
                continue;
            }

            match info {
                ElementDisplayInfo::NameOnly { name } => {
                    egui::CollapsingHeader::new(RichText::new(name).font(MAIN_FONT)).id_salt(sensor_id).show(ui, |ui| {
                        ui.label(RichText::new("unsupported").font(SECONDARY_FONT));
                    });
                }
                ElementDisplayInfo::Full { name, adr_start, components, n_replicate } => {
                    let n_components = info.replica_len();
                    let sensor_data = &sensordata[*adr_start..*adr_start + n_components * *n_replicate];
                    egui::CollapsingHeader::new(RichText::new(name).font(MAIN_FONT)).id_salt(sensor_id).show(ui, |ui| {
                        // A user sensor or a camera with a zero resolution gives dim 0.
                        if sensor_data.is_empty() {
                            ui.label(RichText::new("no dimension").font(SECONDARY_FONT));
                            return;
                        }

                        let mut widget_page = WidgetPage::new(
                            page,
                            widgets_per_page
                        );

                        if *n_replicate > 1 {
                            // Optimization 1: add virtual widgets up to the start of the current page, which was
                            // configured in prior UI passes.
                            // This allows us to skip iteration of data replicas assigned to lesser pages, where this
                            // iteration would only cause the counter to increase, without actually adding real widgets.
                            // What would happen is the same as with the `paged_empty` call below, but slower.
                            let available_start = widget_page.page_local_offset();
                            widget_page.paged_empty(available_start);

                            // Optimization 2: Inclusevly iterate only up to data replica assigned to the current page.
                            let available = widget_page.available_in_current_page();

                            for (replica_i, replica_data) in sensor_data
                                .chunks_exact(n_components)
                                .enumerate()
                                .skip(available_start)
                                .take(available)
                            {
                                widget_page.paged(ui, |ui|
                                    ui.collapsing(RichText::new(format!("#{replica_i}")).font(SECONDARY_FONT), |ui| {
                                        egui::Grid::new("sensor_data_ui_grid")
                                            .spacing(egui::vec2(INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y))
                                            .min_col_width(0.0)
                                            .show(ui, |ui|
                                        {
                                            let mut rest = replica_data;
                                            for (row_i, component) in components.iter().enumerate() {
                                                let values;
                                                (values, rest) = rest.split_at(component.sub_component_names.len());
                                                show_sensor_row(ui, component, values, row_i == components.len() - 1);
                                            }
                                        });
                                    })
                                );
                            }

                            // Add fake widgets (increase internal count) such that we simulate above loop iterating
                            // and repopulating all the pages. This final step is needed to trick the UI into thinking
                            // we did actually add more widgets. If we don't do this, there will be no buttons displayed.
                            widget_page.paged_empty(n_replicate.saturating_sub(available_start + available));
                        } else {
                            egui::Grid::new("sensor_data_ui_grid")
                                .spacing(egui::vec2(INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y))
                                .min_col_width(0.0)
                                .show(ui, |ui|
                            {
                                let mut rest = sensor_data;
                                for (row_i, component) in components.iter().enumerate() {
                                    let values;
                                    (values, rest) = rest.split_at(component.sub_component_names.len());
                                    widget_page.paged(ui, |ui| {
                                        show_sensor_row(ui, component, values, row_i == components.len() - 1);
                                    });
                                }
                            });
                        }

                        ui.add(widget_page);
                    });
                }
            }
        }
    });
}

/// Shows one row (i.e., component) of a given sensor.
fn show_sensor_row(ui: &mut egui::Ui, component: &ComponentDisplayInfo, values: &[MjtNum], is_last: bool) {
    // Empty labels also take a cell, so that the values of all rows share the same columns.
    ui.label(RichText::new(&component.name).font(SECONDARY_FONT));
    for (sub_name, value) in component.sub_component_names.iter().zip(values) {
        ui.label(RichText::new(*sub_name).font(SECONDARY_FONT));

        match &component.range {
            Some(range) => {
                ui.add(
                    egui::ProgressBar::new(
                        ((*value - range.start()) /
                            (range.end() - range.start())) as f32
                    )
                    .text(
                        RichText::new(format!("{value:.3}{}", component.unit))
                        .font(SECONDARY_FONT)
                        .color(egui::Color32::GRAY)
                    )
                    .desired_width(SENSOR_BAR_WIDTH)
                    .corner_radius(SENSOR_BAR_CORNER_RADIUS)
                );
            }

            None => {
                egui::Frame::canvas(ui.style()).show(ui, |ui| {
                    ui.label(format!("{value:.3}{}", component.unit));
                });
            }
        }
        ui.add(egui::Separator::default().vertical());
    }

    ui.end_row();

    // Paint a separator ourselves, since grid does not draw one.
    if !is_last {
        let y = ui.min_rect().bottom() + INSPECTOR_GRID_SPACING_Y / 2.0;
        let stroke = ui.visuals().widgets.noninteractive.bg_stroke;
        ui.painter().hline(ui.max_rect().x_range(), y, stroke);
    }
}
