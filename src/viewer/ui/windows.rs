//! A submodule of the viewer's UI to separate window definitions from all the rest of UI.
pub(super) mod actuator;
pub(super) mod equality;
pub(super) mod sensor;
pub(super) mod group;
pub(super) mod joint;

use std::ops::RangeInclusive;

use crate::wrappers::mj_model::{MjModel, MjtJoint, MjtTrn};
use crate::wrappers::mj_primitive::MjtNum;

const INSPECTOR_GRID_SPACING_X: f32 = 6.0;
const INSPECTOR_GRID_SPACING_Y: f32 = 10.0;

/// Tolerance below which a gear norm is still considered as unit (=1).
const GEAR_NORM_TOLERANCE: MjtNum = 1e-9;

/// Names of the components of a 3D vector, in `qpos` or `sensordata` order.
const XYZ_COMPONENTS: [&str; 3] = ["x", "y", "z"];

/// Names of the components of a quaternion, in `qpos` or `sensordata` order.
const QUAT_COMPONENTS: [&str; 4] = ["qw", "qx", "qy", "qz"];

/// Cached display information about one component of an actuator control (one `ctrl` entry), of a
/// joint position (one `qpos` entry) or of a sensor value (one `sensordata` entry).
pub(super) struct ComponentDisplayInfo {
    /// Name of the component. [`None`] when the component has no name and is the only component
    /// of its owner, which then needs no label.
    label: Option<String>,
    /// Range of the slider (actuator, joint) or the progress bar (sensor) of the component, or
    /// [`None`] when the component has no natural range and shows a plain value instead.
    range: Option<RangeInclusive<MjtNum>>,
    /// Unit suffix of the value.
    unit: &'static str,
}

/// Cached display information about one element (sensor/joint/actuator).
pub(super) enum ElementDisplayInfo {
    /// An element that the viewer cannot display. Only its name and "unsupported" are shown.
    NameOnly {
        /// Name of the element. Auto-generated to be 'Sensor/Joint/Actuator <0->' when not named.
        name: String,
    },
    /// An element whose values are shown, one row per repetition of `components`.
    Full {
        /// Name of the element. Auto-generated to be 'Sensor/Joint/Actuator <0->' when not named.
        name: String,
        /// Address of the first component of the element inside its data array (`ctrl`, `qpos`
        /// or `sensordata`).
        adr_start: usize,
        /// Display information of each component of one row, in data array order.
        /// The subboxes represent rows (name, components).
        components: Box<[
            (Option<String>, Box<[ComponentDisplayInfo]>)
        ]>,
        /// Number of repetitions of the initial elements inside `components`. There will be `n_replicate` of
        /// `components` created.
        n_replicate: usize,
    },
}

impl ElementDisplayInfo {
    /// Returns the display name of the element.
    pub(super) fn name(&self) -> &str {
        match self {
            Self::NameOnly { name } | Self::Full { name, .. } => name,
        }
    }
}

fn actuator_joint_type(model: &MjModel, id: usize) -> Option<MjtJoint> {
    match model.actuator_trntype()[id] {
        MjtTrn::mjTRN_SO3 => Some(MjtJoint::mjJNT_BALL),
        MjtTrn::mjTRN_JOINT | MjtTrn::mjTRN_JOINTINPARENT => {
            let gear = &model.actuator_gear()[model.actuator_outadr()[id] as usize];
            let joint_type = model.jnt_type()[model.actuator_trnid()[id][0] as usize];
            let unit_gear = match joint_type {
                MjtJoint::mjJNT_SLIDE | MjtJoint::mjJNT_HINGE => gear[0].abs() == 1.0,
                MjtJoint::mjJNT_BALL => (gear[0].hypot(gear[1]).hypot(gear[2]) - 1.0).abs() < GEAR_NORM_TOLERANCE,
                _ => false
            };
            unit_gear.then_some(joint_type)
        }
        _ => None
    }
}
