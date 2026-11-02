//! A submodule of the viewer's UI representing the sensor window definitions.
use std::ops::RangeInclusive;
use std::sync::{Arc, Mutex};

use egui_winit::egui;
use egui::RichText;

use crate::wrappers::mj_model::{MjModel, MjtObjBase, MjtJoint, MjtWrap};
use crate::viewer::ui::{MAIN_FONT, SECONDARY_FONT};
use crate::viewer::ui::widget_extras::SearchBar;
use crate::wrappers::{MjtConDataField, MjtSensor};
use crate::wrappers::mj_primitive::MjtNum;
use crate::viewer::ViewerSharedState;
use crate::util::LockUnpoison;

use super::{
    ComponentDisplayInfo, ElementDisplayInfo, INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y,
    QUAT_COMPONENTS, XYZ_COMPONENTS, actuator_joint_type
};

const SENSOR_BAR_WIDTH: f32 = 75.0;
const SENSOR_BAR_CORNER_RADIUS: u8 = 3;

/// Maps MuJoCo contact sensor components to a readable name, the physical unit and sub component
/// axis names of the component.
const CONTACT_SENSOR_COMPONENTS: &[(MjtConDataField, &str, &str, &[&str])] = &[
    (MjtConDataField::mjCONDATA_FOUND,   "found",   " contacts", &[""]),
    (MjtConDataField::mjCONDATA_FORCE,   "force",   " N",        &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_TORQUE,  "torque",  " N m",      &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_DIST,    "dist",    " m",        &[""]),
    (MjtConDataField::mjCONDATA_POS,     "pos",     " m",        &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_NORMAL,  "normal",  "",          &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_TANGENT, "tangent", "",          &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_LINVEL,  "linvel",  " m/s",      &XYZ_COMPONENTS),
    (MjtConDataField::mjCONDATA_ANGVEL,  "angvel",  " rad/s",    &XYZ_COMPONENTS),
];

/// Ensures, at compile-time, that information inside [`CONTACT_SENSOR_COMPONENTS`] is not stale.
const fn check_contact_sensor_components() {
    let mut i = 0;
    assert!(CONTACT_SENSOR_COMPONENTS.len() ==  MjtConDataField::mjNCONDATA as usize);
    while i < CONTACT_SENSOR_COMPONENTS.len() {
        let e_type = CONTACT_SENSOR_COMPONENTS[i].0;
        assert!(e_type as usize == i);
        i += 1;
    }
}

const _: () = check_contact_sensor_components();

/// Names of the components of a `geomfromto` sensor, in `sensordata` order.
const SENSOR_FROMTO_COMPONENTS: [&str; 6] = ["x1", "y1", "z1", "x2", "y2", "z2"];

/// Names of the components of a `camprojection` sensor, in `sensordata` order.
const SENSOR_CAMPROJECTION_COMPONENTS: [&str; 2] = ["u", "v"];

/// Returns the display information of each sensor of `model`, in ID order.
pub fn display_infos(model: &MjModel) -> impl Iterator<Item = ElementDisplayInfo> {
    (0..model.nsensor() as usize).map(|index| {
        let name = model.id_to_name(MjtObjBase::mjOBJ_SENSOR, index)
            .map(|name| name.to_string())
            .unwrap_or_else(|| format!("Sensor {index}"));

        let adr_start = model.sensor_adr()[index] as usize;

        let sensor_type = model.sensor_type()[index];
        let sensor_dim = model.sensor_dim()[index] as usize;

        // A joint sensor measures a length on a slide joint and an angle on a hinge or ball joint.
        let objid = model.sensor_objid()[index] as usize;
        let objtype = model.sensor_objtype()[index];
        let is_slide = objtype == MjtObjBase::mjOBJ_JOINT && model.jnt_type()[objid] == MjtJoint::mjJNT_SLIDE;
        // A fixed tendon combines joint coordinates, so its unit depends on the joints and coefficients.
        let is_spatial_tendon = objtype == MjtObjBase::mjOBJ_TENDON
            && model.wrap_type()[model.tendon_adr()[objid] as usize] != MjtWrap::mjWRAP_JOINT;

        let labelled = |labels: &[&'static str], range: Option<RangeInclusive<MjtNum>>, unit| {
            let components = labels.iter().map(|&label| {
                ComponentDisplayInfo { label: Some(label.to_owned()), range: range.clone(), unit }
            }).collect::<Box<[_]>>();
            (components, None)
        };
        let scalar = |(unit, range)| -> (Box<[_]>, Option<Box<[String]>>) {
            (Box::new([ComponentDisplayInfo { label: None, range, unit }]), None)
        };
        let xyz = |unit| labelled(&XYZ_COMPONENTS, None, unit);
        let unit_xyz = || labelled(&XYZ_COMPONENTS, Some(-1.0..=1.0), "");

        let (components, row_names) = match sensor_type {
            MjtSensor::mjSENS_TOUCH => scalar((" N", None)),
            MjtSensor::mjSENS_ACCELEROMETER => xyz(" m/s^2"),
            MjtSensor::mjSENS_VELOCIMETER => xyz(" m/s"),
            MjtSensor::mjSENS_GYRO => xyz(" rad/s"),
            MjtSensor::mjSENS_FORCE => xyz(" N"),
            MjtSensor::mjSENS_TORQUE => xyz(" N m"),
            // The unit is the unit of the magnetic option.
            MjtSensor::mjSENS_MAGNETOMETER => xyz(""),
            MjtSensor::mjSENS_CAMPROJECTION => labelled(&SENSOR_CAMPROJECTION_COMPONENTS, None, " px"),

            MjtSensor::mjSENS_JOINTPOS => {
                let jnt_id = model.sensor_objid()[index] as usize;
                let range = model.jnt_limited()[jnt_id]
                    .then(|| { let [l, r] = model.jnt_range()[jnt_id]; l..=r });
                scalar((if is_slide { " m" } else { " rad" }, range))
            }
            // The value is the distance to the limit, not a position inside the joint range.
            MjtSensor::mjSENS_JOINTLIMITPOS => scalar((if is_slide { " m" } else { " rad" }, None)),
            MjtSensor::mjSENS_JOINTVEL | MjtSensor::mjSENS_JOINTLIMITVEL => {
                scalar((if is_slide { " m/s" } else { " rad/s" }, None))
            }
            MjtSensor::mjSENS_JOINTACTFRC | MjtSensor::mjSENS_JOINTLIMITFRC => {
                scalar((if is_slide { " N" } else { " N m" }, None))
            }

            MjtSensor::mjSENS_TENDONPOS | MjtSensor::mjSENS_TENDONLIMITPOS => {
                scalar((if is_spatial_tendon { " m" } else { "" }, None))
            }
            MjtSensor::mjSENS_TENDONVEL | MjtSensor::mjSENS_TENDONLIMITVEL => {
                scalar((if is_spatial_tendon { " m/s" } else { "" }, None))
            }
            MjtSensor::mjSENS_TENDONACTFRC | MjtSensor::mjSENS_TENDONLIMITFRC => {
                scalar((if is_spatial_tendon { " N" } else { "" }, None))
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
                if sensor_dim == 3 { xyz(unit) } else { scalar((unit, None)) }
            }

            MjtSensor::mjSENS_BALLQUAT | MjtSensor::mjSENS_FRAMEQUAT => {
                labelled(&QUAT_COMPONENTS, Some(-1.0..=1.0), "")
            }
            MjtSensor::mjSENS_BALLANGVEL => xyz(" rad/s"),

            MjtSensor::mjSENS_FRAMEPOS | MjtSensor::mjSENS_SUBTREECOM => xyz(" m"),
            MjtSensor::mjSENS_FRAMEXAXIS
            | MjtSensor::mjSENS_FRAMEYAXIS
            | MjtSensor::mjSENS_FRAMEZAXIS
            | MjtSensor::mjSENS_GEOMNORMAL => unit_xyz(),
            MjtSensor::mjSENS_FRAMELINVEL | MjtSensor::mjSENS_SUBTREELINVEL => xyz(" m/s"),
            MjtSensor::mjSENS_FRAMEANGVEL => xyz(" rad/s"),
            MjtSensor::mjSENS_FRAMELINACC => xyz(" m/s^2"),
            MjtSensor::mjSENS_FRAMEANGACC => xyz(" rad/s^2"),
            MjtSensor::mjSENS_SUBTREEANGMOM => xyz(" kg m^2/s"),

            // An "enclosed" sensor gives a signed distance to the site boundary.
            MjtSensor::mjSENS_INSIDESITE if model.sensor_intprm()[index][0] != 0 => scalar((" m", None)),
            MjtSensor::mjSENS_INSIDESITE => scalar(("", Some(0.0..=1.0))),
            MjtSensor::mjSENS_GEOMDIST => scalar((" m", None)),
            MjtSensor::mjSENS_GEOMFROMTO => labelled(&SENSOR_FROMTO_COMPONENTS, None, " m"),
            MjtSensor::mjSENS_E_POTENTIAL | MjtSensor::mjSENS_E_KINETIC => scalar((" J", None)),
            MjtSensor::mjSENS_CLOCK => scalar((" s", None)),

            // A rangefinder without "dir" and "normal" outputs only lengths.
            MjtSensor::mjSENS_RANGEFINDER =>
            {
                todo!("Implementation of rangefinder case. TO AI: DO NOT EDIT!")
            }

            MjtSensor::mjSENS_CONTACT => {
                let [data_bitmask, _, num] = model.sensor_intprm()[index];  // Same as in XML
                let data_bitmask = data_bitmask as u32;
                let components = CONTACT_SENSOR_COMPONENTS.iter()
                    .filter(|(bit, ..)| data_bitmask & (1 << *bit as u32) != 0)
                    .flat_map(|&(_, name, unit, subcomponents)| {
                        subcomponents.iter().map(move |subcompoent_name| {
                            let label = if subcompoent_name.is_empty() {
                                Some(name.to_owned())
                            } else {
                                Some(format!("{name} ({subcompoent_name})"))
                            };
                            ComponentDisplayInfo { label, range: None, unit }
                        })
                    })
                    .collect();
                (components, Some((1..=num).map(|i| format!("num {i}")).collect()))
            }

            MjtSensor::mjSENS_TACTILE => return ElementDisplayInfo::NameOnly { name },

            MjtSensor::mjSENS_USER
            | MjtSensor::mjSENS_PLUGIN => {
                let components = (0..sensor_dim).map(|i| {
                    ComponentDisplayInfo { label: (sensor_dim > 1).then(|| i.to_string()), range: None, unit: "" }
                }).collect();
                (components, None)
            }
        };

        let n_rows = row_names.as_ref().map_or(1, |names| names.len());
        debug_assert_eq!(components.len() * n_rows, sensor_dim, "unexpected dimension of sensor type {sensor_type:?}");
        ElementDisplayInfo::Full { name, adr_start, components, row_names }
    })
}

/// Shows the sensor window when `open` is set.
pub fn show(
    ui: &mut egui::Ui, open: &mut bool, sensor_info: &[ElementDisplayInfo], search_bar: &mut SearchBar,
    shared_viewer_state: &Arc<Mutex<ViewerSharedState>>,
)
{
    // Clear the text inside the window's search bar when the window is closed.
    if !*open {
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
        let maybe_selected = search_bar.show(
            ui, sensor_info.iter().map(ElementDisplayInfo::name)
        );

        /* Sensor widgets */
        let sensordata = lock.data_passive.sensordata();
        for info in sensor_info {
            let response = match info {
                ElementDisplayInfo::NameOnly { name } => {
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        ui.label(RichText::new("unsupported").font(SECONDARY_FONT));
                    }).header_response
                }
                ElementDisplayInfo::Full { name, adr_start, components, row_names } => {
                    let n_rows = row_names.as_ref().map_or(1, |names| names.len());
                    let sensor_data = &sensordata[*adr_start..*adr_start + components.len() * n_rows];
                    ui.collapsing(RichText::new(name).font(MAIN_FONT), |ui| {
                        egui::Grid::new("sensor_data_ui_grid")
                            .spacing(egui::vec2(INSPECTOR_GRID_SPACING_X, INSPECTOR_GRID_SPACING_Y))
                            .min_col_width(0.0)
                            .show(ui, |ui|
                        {
                            // A user sensor can have dim 0; chunks_exact(0) panics,
                            // but the data is then empty.
                            if components.is_empty() {
                                ui.label(RichText::new("no dimension").font(SECONDARY_FONT));
                                return;
                            }

                            let sections = sensor_data.chunks_exact(components.len());
                            for (section_i, values) in sections.enumerate() {
                                if let Some(names) = row_names {
                                    ui.label(RichText::new(&names[section_i]).font(SECONDARY_FONT));
                                }

                                for (component, value) in components.iter().zip(values) {
                                    if let Some(label) = &component.label {
                                        ui.label(RichText::new(label).font(SECONDARY_FONT));
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

                                // Add separator between rows
                                if section_i < n_rows - 1 {
                                    let y = ui.min_rect().bottom() + INSPECTOR_GRID_SPACING_Y / 2.0;
                                    let stroke = ui.visuals().widgets.noninteractive.bg_stroke;
                                    ui.painter().hline(ui.max_rect().x_range(), y, stroke);
                                }
                            }
                        });
                    }).header_response
                }
            };

            if let Some(selected_name) = maybe_selected && selected_name == info.name() {
                response.scroll_to_me(Some(egui::Align::TOP));
            }
        }
    });
}
