//! A submodule of the viewer's UI representing the sensor window definitions.
use std::ops::RangeInclusive;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase, MjtJoint, MjtWrap};
use crate::viewer::ui::{MAIN_FONT, SECONDARY_FONT};
use crate::viewer::ui::widget_extras::{SearchBar, WidgetPage};
use crate::wrappers::{MjtConDataField, MjtRayDataField, MjtSensor};
use crate::wrappers::mj_primitive::MjtNum;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::{
    ComponentDisplayInfo, ElementDisplayInfo, INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y,
    QUAT_COMPONENTS, XYZ_COMPONENTS, actuator_joint_type
};

const SENSOR_BAR_WIDTH: f32 = 75.0;
const SENSOR_BAR_CORNER_RADIUS: u8 = 3;

/// Maximum number of widgets that an element shows on one page.
pub const MAX_WIDGETS_PER_ELEMENT: usize = 20;

/// Ensures, at compile-time, that information inside sensor types with multiple data fields and variants is not stale.
macro_rules! check_sensor_components {
    ($components:expr, $len:expr) => {
        const _: () = const {
            let mut i = 0;
            assert!($components.len() ==  $len as usize);
            while i < $components.len() {
                let e_type = $components[i].0;
                assert!(e_type as usize == i);
                i += 1;
            }       
        };
    };
}

/// Maps MuJoCo contact sensor components to a readable name, the physical unit and sub component
/// axis names of the component.
const CONTACT_SENSOR_COMPONENTS: &[(u32, &str, &str, &[&str])] = &[
    (MjtConDataField::mjCONDATA_FOUND as u32,   "found",   " contacts", &[""]),
    (MjtConDataField::mjCONDATA_FORCE as u32,   "force",   " N",        &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_TORQUE as u32,  "torque",  " N m",      &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_DIST as u32,    "dist",    " m",        &[""]),
    (MjtConDataField::mjCONDATA_POS as u32,     "pos",     " m",        &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_NORMAL as u32,  "normal",  "",          &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_TANGENT as u32, "tangent", "",          &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_LINVEL as u32,  "linvel",  " m/s",      &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_ANGVEL as u32,  "angvel",  " rad/s",    &XYZ_COMPONENTS),
];
check_sensor_components!(CONTACT_SENSOR_COMPONENTS, MjtConDataField::mjNCONDATA);

/// Maps MuJoCo rangefinder sensor components to a readable name, the physical unit and sub component
/// axis names of the component.
const RANGEFINDER_SENSOR_COMPONENTS: &[(u32, &str, &str, &[&str])] = &[
    (MjtRayDataField::mjRAYDATA_DIST as u32,   "distance", " meters", &[""]),
    (MjtRayDataField::mjRAYDATA_DIR as u32,    "dir",      "",        &XYZ_COMPONENTS),
    (MjtRayDataField::mjRAYDATA_ORIGIN as u32, "origin",   " meters", &XYZ_COMPONENTS),
    (MjtRayDataField::mjRAYDATA_POINT as u32,  "point",    " meters", &XYZ_COMPONENTS),
    (MjtRayDataField::mjRAYDATA_NORMAL as u32, "normal",   "",        &XYZ_COMPONENTS),
    (MjtRayDataField::mjRAYDATA_DEPTH as u32,  "depth",    " meters", &[""]),
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
            Box::new([ComponentDisplayInfo { name: Some(name.to_owned()), range, unit, sub_component_names: labels }])
        };
        let scalar = |name: &str, (unit, range)| -> Box<[ComponentDisplayInfo]> {
            Box::new([ComponentDisplayInfo { name: Some(name.to_owned()), range, unit, sub_component_names: &[""] }])
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
                let jnt_id = model.sensor_objid()[sensor_id] as usize;
                let range = model.jnt_limited()[jnt_id]
                    .then(|| { let [l, r] = model.jnt_range()[jnt_id]; l..=r });
                scalar("position", (if is_slide { " m" } else { " rad" }, range))
            }
            // The value is the distance to the limit, not a position inside the joint range.
            MjtSensor::mjSENS_JOINTLIMITPOS => scalar("limit distance", (if is_slide { " m" } else { " rad" }, None)),
            MjtSensor::mjSENS_JOINTVEL | MjtSensor::mjSENS_JOINTLIMITVEL => {
                let name = if sensor_type == MjtSensor::mjSENS_JOINTVEL { "velocity" } else { "limit velocity" };
                scalar(name, (if is_slide { " m/s" } else { " rad/s" }, None))
            }
            MjtSensor::mjSENS_JOINTACTFRC | MjtSensor::mjSENS_JOINTLIMITFRC => {
                let name = if sensor_type == MjtSensor::mjSENS_JOINTACTFRC { "actuator force" } else { "limit force" };
                scalar(name, (if is_slide { " N" } else { " N m" }, None))
            }

            MjtSensor::mjSENS_TENDONPOS | MjtSensor::mjSENS_TENDONLIMITPOS => {
                let name = if sensor_type == MjtSensor::mjSENS_TENDONPOS { "length" } else { "limit distance" };
                scalar(name, (if is_spatial_tendon { " m" } else { "" }, None))
            }
            MjtSensor::mjSENS_TENDONVEL | MjtSensor::mjSENS_TENDONLIMITVEL => {
                let name = if sensor_type == MjtSensor::mjSENS_TENDONVEL { "velocity" } else { "limit velocity" };
                scalar(name, (if is_spatial_tendon { " m/s" } else { "" }, None))
            }
            MjtSensor::mjSENS_TENDONACTFRC | MjtSensor::mjSENS_TENDONLIMITFRC => {
                let name = if sensor_type == MjtSensor::mjSENS_TENDONACTFRC { "actuator force" } else { "limit force" };
                scalar(name, (if is_spatial_tendon { " N" } else { "" }, None))
            }

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
                if sensor_dim == 3 { xyz(name, unit) } else { scalar(name, (unit, None)) }
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

            MjtSensor::mjSENS_RANGEFINDER => {
                let data_bitmask = model.sensor_intprm()[sensor_id][0];
                let components = RANGEFINDER_SENSOR_COMPONENTS.iter()
                    .filter(|(bit, ..)| data_bitmask & (1 << *bit) != 0);

                // TODO: move ranges into the components array.
                components.map(|(_, data_name, unit, subcomponents)| {
                    ComponentDisplayInfo {
                        name: Some(data_name.to_string()),
                        unit,
                        range: None,
                        sub_component_names: subcomponents,
                    }
                }).collect()
            }
            MjtSensor::mjSENS_CONTACT => {
                let data_bitmask = model.sensor_intprm()[sensor_id][0];
                let components = CONTACT_SENSOR_COMPONENTS.iter()
                    .filter(|(bit, ..)| data_bitmask & (1 << *bit) != 0);

                // TODO: move ranges into the components array.
                components.map(|(_, data_name, unit, subcomponents)| {
                    ComponentDisplayInfo {
                        name: Some(data_name.to_string()),
                        unit,
                        range: None,
                        sub_component_names: subcomponents,
                    }
                }).collect()
            }

            MjtSensor::mjSENS_TACTILE => return ElementDisplayInfo::NameOnly { name },

            MjtSensor::mjSENS_USER
            | MjtSensor::mjSENS_PLUGIN => {
                (0..sensor_dim).map(|i| {
                    ComponentDisplayInfo {
                        name: Some(if sensor_dim > 1 { format!("value {i}") } else { "value".to_owned() }),
                        range: None,
                        unit: "",
                        sub_component_names: &[""],
                    }
                }).collect()
            }
        };

        let n_replicate = match sensor_type {
            MjtSensor::mjSENS_RANGEFINDER => {
                if model.sensor_objtype()[sensor_id] == MjtObjBase::mjOBJ_SITE {
                    1
                } else {  // camera
                    let cam_id = model.sensor_objid()[sensor_id] as usize;
                    let [width, height] = model.cam_resolution()[cam_id];
                    width as usize * height as usize
                }
            }
            MjtSensor::mjSENS_CONTACT => model.sensor_intprm()[sensor_id][2] as usize,
            _ => 1
        };
        let n_components = components.iter().map(|component| component.sub_component_names.len()).sum::<usize>();
        debug_assert_eq!(n_components * n_replicate, sensor_dim, "unexpected dimension of sensor type {sensor_type:?}");
        ElementDisplayInfo::Full { name, adr_start, components, n_replicate }
    })
}

/// Shows the sensor window when `open` is set.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, sensor_info: &[ElementDisplayInfo], search_bar: &mut SearchBar,
    pages: &mut [(usize, usize)], shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    // Clear the text inside the window's search bar when the window is closed.
    if !*open {
        pages.fill((0, MAX_WIDGETS_PER_ELEMENT));
        search_bar.clear();
    }

    egui::Window::new("Sensor")
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
        for (info, (page, widgets_per_page)) in sensor_info.iter().zip(pages.iter_mut()) {
            if let Some(query) = search_bar.text() && !info.name().starts_with(query) {
                continue;
            }

            match info {
                ElementDisplayInfo::NameOnly { name } => {
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        ui.label(RichText::new("unsupported").font(SECONDARY_FONT));
                    });
                }
                ElementDisplayInfo::Full { name, adr_start, components, n_replicate } => {
                    let n_components = components.iter().map(|component| component.sub_component_names.len()).sum::<usize>();
                    let sensor_data = &sensordata[*adr_start..*adr_start + n_components * *n_replicate];
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        // A user sensor can have dim 0; chunks_exact(0) panics,
                        // but the data is then empty.
                        if n_components == 0 {
                            ui.label(RichText::new("no dimension").font(SECONDARY_FONT));
                            return;
                        }

                        let mut widget_page = WidgetPage::new(
                            page,
                            widgets_per_page
                        );

                        if *n_replicate > 1 {
                            for (replica_i, replica_data) in sensor_data.chunks_exact(n_components).enumerate() {
                                widget_page.paged(ui, |ui| {
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
                                    });
                                });
                            }
                        }
                        else {
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
    if let Some(name) = &component.name {
        ui.label(RichText::new(name).font(SECONDARY_FONT));
    }

    for (sub_name, value) in component.sub_component_names.iter().zip(values) {
        if !sub_name.is_empty() {
            ui.label(RichText::new(*sub_name).font(SECONDARY_FONT));
        }

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
