//! Integration tests for [`MjModel::is_compatible_with_model`],
//! [`MjModel::is_asset_compatible_with_model`], [`MjvScene::is_compatible_with_model`],
//! [`MjvScene::is_compatible_with_scene`], [`MjrContext::is_compatible_with_model`] and the `Info`
//! view gate.

use mujoco_rs::wrappers::mj_editing::{
    IntVelocityConfig, PositionConfig, DcMotorConfig, MjsActuator, PidConfig,
};
use mujoco_rs::wrappers::mj_plugin::load_all_plugin_libraries;
use mujoco_rs::prelude::*;

use std::collections::BTreeSet;

/* The base model. */
const BASE_XML: &str = r#"<mujoco model='base'>
<option timestep='0.002' gravity='0 0 -9.81' integrator='Euler'/>
<size nuserdata='16' nuser_body='2' nuser_jnt='1' nuser_geom='3' nuser_site='1' nuser_cam='1'
      nuser_tendon='2' nuser_actuator='1' nuser_sensor='2'/>
<asset>
  <texture name='tx' type='2d' builtin='checker' width='16' height='24' rgb1='1 0 0' rgb2='0 1 0'/>
  <texture name='sky' type='skybox' builtin='gradient' width='8' height='8' rgb1='0 0 1' rgb2='1 1 1'/>
  <material name='mat' texture='tx'/>
  <mesh name='ms' vertex='0 0 0  .1 0 0  0 .1 0  0 0 .1' texcoord='0 0  1 0  0 1  1 1'/>
  <hfield name='hf' nrow='5' ncol='9' size='1 1 .2 .05'/>
  <skin name='sk' vertex='0 0 0  .1 0 0  0 .1 0' face='0 1 2' inflate='.01'>
    <bone body='trunk' bindpos='0 0 0' bindquat='1 0 0 0' vertid='0 1 2' vertweight='1 1 1'/>
  </skin>
</asset>
<worldbody>
  <geom name='floor' type='plane' size='5 5 .1'/>
  <geom name='g_hf' type='hfield' hfield='hf' pos='3 0 0'/>
  <light name='l0' pos='0 0 3'/>
  <camera name='c0' pos='0 0 2'/>
  <body name='mocap' mocap='true' pos='1 0 0'><geom name='g_m' size='.05'/></body>
  <body name='spare' pos='0 -2 0'><geom name='g_sp' size='.03'/></body>
  <body name='v0' pos='0 2 0'><freejoint name='fv0'/><geom name='g_v0' size='.01'/></body>
  <body name='v1' pos='.1 2 0'><freejoint name='fv1'/><geom name='g_v1' size='.01'/></body>
  <body name='v2' pos='.2 2 0'><freejoint name='fv2'/><geom name='g_v2' size='.01'/></body>
  <body name='trunk' pos='0 0 1'>
    <freejoint name='root'/>
    <geom name='g_trunk' type='mesh' mesh='ms' material='mat'/>
    <site name='s_trunk'/>
    <camera name='c_trunk' pos='0 0 .5' resolution='64 64'/>
    <light name='l_trunk' pos='0 0 .5'/>
    <body name='upper' pos='0 0 -.3'>
      <joint name='hip' type='ball'/>
      <geom name='g_upper' type='capsule' pos='0 0 -.1' size='.04 .1'/>
      <body name='lower' pos='0 0 -.2'>
        <joint name='knee' type='hinge' axis='0 1 0'/>
        <joint name='slide' type='slide' axis='1 0 0' limited='true' range='-1 1'/>
        <geom name='g_lower' type='capsule' fromto='0 0 0 0 0 -.2' size='.04'/>
        <site name='s_lower' pos='0 0 -.2'/>
      </body>
    </body>
  </body>
</worldbody>
<deformable>
  <flex name='f0' dim='1' body='v0 v1 v2' vertex='0 0 0 0 0 0 0 0 0' element='0 1 1 2'>
    <edge stiffness='10' damping='.1'/>
  </flex>
</deformable>
<tendon>
  <spatial name='td' limited='true' range='0 1'><site site='s_trunk'/><site site='s_lower'/></spatial>
  <spatial name='t2'><site site='s_trunk'/><site site='s_lower'/></spatial>
  <fixed name='tf'><joint joint='knee' coef='1'/><joint joint='slide' coef='2'/></fixed>
</tendon>
<equality>
  <connect name='eq0' body1='lower' body2='world' anchor='0 0 0'/>
  <joint name='eq1' joint1='knee' joint2='slide' polycoef='0 1 0 0 0'/>
</equality>
<contact>
  <pair name='p0' geom1='g_upper' geom2='g_lower'/>
  <exclude name='x0' body1='trunk' body2='lower'/>
</contact>
<actuator>
  <motor name='a_motor' joint='knee'/>
  <position name='a_pos' joint='slide' kp='3'/>
  <general name='a_int' joint='knee' dyntype='integrator' nsample='3' delay='.004'/>
  <general name='a_filt' joint='slide' dyntype='filter' dynprm='.1'/>
  <general name='a_ten' tendon='tf' dyntype='filterexact' dynprm='.1'/>
  <orientation name='a_so3' site='s_lower' refsite='s_trunk' kp='1' input='expmap'/>
  <pid name='a_pid' joint='knee' kp='1' kv='1' input='pos vel'/>
</actuator>
<sensor>
  <framepos name='se_pos' objtype='site' objname='s_trunk'/>
  <framequat name='se_quat' objtype='site' objname='s_trunk'/>
  <jointpos name='se_jp' joint='knee' nsample='2' delay='.004'/>
  <accelerometer name='se_acc' site='s_lower'/>
  <tendonpos name='se_td' tendon='td'/>
  <actuatorfrc name='se_af' actuator='a_motor'/>
  <user name='se_u1' objtype='site' objname='s_lower' dim='5' needstage='vel'/>
  <user name='se_u2' objtype='body' objname='lower' dim='2' needstage='pos'/>
</sensor>
<custom>
  <numeric name='n0' data='1 2 3'/>
  <numeric name='n1' data='4 5'/>
  <tuple name='t0'><element objtype='body' objname='lower'/></tuple>
  <text name='tx0' data='hello'/>
</custom>
<keyframe><key name='k0' time='0'/><key name='k1' time='1'/></keyframe>
</mujoco>"#;

/// The spec every variant starts from.
fn base_spec() -> MjSpec {
    MjSpec::from_xml_string(BASE_XML).expect("the base model does not parse")
}

fn base() -> MjModel {
    base_spec().compile().expect("the base model does not compile")
}

/* Edits. */

/// One edit to the base spec.
struct Edit {
    label: String,
    kind: Kind,
    apply: Box<dyn Fn(&mut MjSpec)>,
}

impl Edit {
    fn new(label: impl Into<String>, kind: Kind, apply: impl Fn(&mut MjSpec) + 'static) -> Self {
        Self { label: label.into(), kind, apply: Box::new(apply) }
    }
}

/// What an edit is expected to do to the model structure.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    /// Changes a value MuJoCo stores; moves no element and resizes no buffer.
    Parameter,
    /// Moves an element, resizes a buffer, or changes what a slot means.
    Structural,
    /// The effect is not obvious in advance. Only the reference check decides it.
    Open,
}

/// Applies every edit in `edits` to a copy of `spec` and compiles it.
///
/// Returns `None` when the model compilation fails (invalid edit).
fn compile_with(spec: &MjSpec, edits: &[&Edit]) -> Option<MjModel> {
    let mut copy = spec.clone();
    for e in edits {
        (e.apply)(&mut copy);
    }
    copy.compile().ok()
}

/// Invokes `|$item| $body` when the $finder finds the $name. Otherwise, nothing happens.
macro_rules! on_item {
    ($spec:expr, $finder:ident, $name:expr, |$item:ident| $body:expr) => {
        if let Some($item) = $spec.$finder(&$name[..]) {
            $body;
        }
    };
}

/// Proves at compile time that a list of enum values is complete (exhausted).
///
/// The named arm also defines this list into a const array.
/// The `except`-ed variants aren't put into this array.
macro_rules! all_variants {
    ($(#[$doc:meta])* $name:ident: $ty:ty = $($variant:ident),+
     $(; except $($other:ident),+)? $(,)?) => {
        $(#[$doc])*
        const $name: [$ty; [$(stringify!($variant)),+].len()] = [$(<$ty>::$variant),+];

        all_variants!($ty = $($variant),+ $(; except $($other),+)?);
    };

    ($ty:ty = $($variant:ident),+ $(; except $($other:ident),+)? $(,)?) => {
        #[deny(unreachable_patterns)]
        const _: () = match [$(<$ty>::$variant),+][0] {
            $(<$ty>::$variant)|+ => (),
            $($(<$ty>::$other)|+ => (),)?
        };
    };
}

/* Type-field axes: every value of every enum the layout compares. */

all_variants!(
    /// Every [`MjtJoint`] value.
    JOINT_TYPES: MjtJoint = mjJNT_FREE, mjJNT_BALL, mjJNT_SLIDE, mjJNT_HINGE);

/// Every [`MjtJoint`] value, applied to the trunk's root joint.
fn joint_type_edits() -> Vec<Edit> {

    JOINT_TYPES.into_iter()
        .map(|t| {
            // The root is a free joint, so setting it to free again changes nothing.
            let kind = if t == MjtJoint::mjJNT_FREE { Kind::Parameter } else { Kind::Structural };
            Edit::new(format!("joint type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, joint_mut, "root", |j| j.set_type(t));
            })
        })
        .collect()
}

// The values at and above mjNGEOMTYPES are the rendering-only kinds and the "no geom" marker;
// a model geom never carries one.
all_variants!(
    /// Every [`MjtGeom`] value a model geom can carry.
    GEOM_TYPES: MjtGeom =
    mjGEOM_PLANE, mjGEOM_HFIELD, mjGEOM_SPHERE, mjGEOM_CAPSULE, mjGEOM_ELLIPSOID,
    mjGEOM_CYLINDER, mjGEOM_BOX, mjGEOM_MESH, mjGEOM_SDF
    ; except mjNGEOMTYPES, mjGEOM_ARROW, mjGEOM_ARROW1, mjGEOM_ARROW2, mjGEOM_LINE,
             mjGEOM_LINEBOX, mjGEOM_FLEX, mjGEOM_SKIN, mjGEOM_LABEL, mjGEOM_TRIANGLE,
             mjGEOM_NONE);

/// Every [`MjtGeom`] value MuJoCo accepts on a geom, applied to `g_upper`.
fn geom_type_edits() -> Vec<Edit> {
    GEOM_TYPES.into_iter()
        .map(|t| {
            let kind = if t == MjtGeom::mjGEOM_CAPSULE { Kind::Parameter } else { Kind::Structural };
            Edit::new(format!("geom type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, geom_mut, "g_upper", |g| {
                    g.set_type(t);
                    *g.size_mut() = [0.04, 0.04, 0.1];
                    // A mesh geom and a heightfield geom each need their asset named.
                    match t {
                        MjtGeom::mjGEOM_MESH => { g.set_meshname("ms"); }
                        MjtGeom::mjGEOM_HFIELD => { g.set_hfieldname("hf"); }
                        _ => {}
                    }
                });
            })
        })
        .collect()
}

all_variants!(
    /// Every [`MjtTexture`] value.
    TEXTURE_TYPES: MjtTexture = mjTEXTURE_2D, mjTEXTURE_CUBE, mjTEXTURE_SKYBOX);

/// Every [`MjtTexture`] value, applied to the 2D texture.
fn texture_type_edits() -> Vec<Edit> {
    TEXTURE_TYPES.into_iter()
        .map(|t| {
            let kind = if t == MjtTexture::mjTEXTURE_2D { Kind::Parameter } else { Kind::Structural };
            Edit::new(format!("texture type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, texture_mut, "tx", |tex| {
                    tex.set_type(t);
                    // A cube and a skybox hold six square faces, so the height follows the width.
                    if t != MjtTexture::mjTEXTURE_2D {
                        tex.set_width(16);
                        tex.set_height(16 * 6);
                    }
                });
            })
        })
        .collect()
}

all_variants!(
    /// Every [`MjtEq`] value.
    EQUALITY_TYPES: MjtEq =
    mjEQ_CONNECT, mjEQ_WELD, mjEQ_JOINT, mjEQ_TENDON, mjEQ_FLEX, mjEQ_FLEXVERT,
    mjEQ_FLEXSTRAIN, mjEQ_DISTANCE);

/// Every [`MjtEq`] value, applied to the first equality.
fn equality_type_edits() -> Vec<Edit> {
    // The pair of objects an equality names depends on its type. A type whose targets the base
    // model cannot supply fails to compile, and the caller reports it as skipped.
    fn targets(t: MjtEq) -> (MjtObj, &'static str, &'static str) {
        match t {
            MjtEq::mjEQ_CONNECT | MjtEq::mjEQ_WELD => (MjtObj::mjOBJ_BODY, "lower", "world"),
            MjtEq::mjEQ_JOINT => (MjtObj::mjOBJ_JOINT, "knee", "slide"),
            MjtEq::mjEQ_TENDON => (MjtObj::mjOBJ_TENDON, "td", "tf"),
            MjtEq::mjEQ_FLEX | MjtEq::mjEQ_FLEXVERT | MjtEq::mjEQ_FLEXSTRAIN =>
                (MjtObj::mjOBJ_FLEX, "f0", ""),
            MjtEq::mjEQ_DISTANCE => (MjtObj::mjOBJ_GEOM, "g_upper", "g_lower"),
        }
    }

    EQUALITY_TYPES.into_iter()
        .map(|t| {
            let kind = if t == MjtEq::mjEQ_CONNECT { Kind::Parameter } else { Kind::Structural };
            Edit::new(format!("equality type {t:?}"), kind, move |spec: &mut MjSpec| {
                let (objtype, name1, name2) = targets(t);
                on_item!(spec, equality_mut, "eq0", |eq| {
                    eq.set_type(t);
                    eq.set_objtype(objtype);
                    eq.set_name1(name1);
                    eq.set_name2(name2);
                });
            })
        })
        .collect()
}

/// Every actuator kind the wrapper can build, applied to `a_motor`.
///
/// The list mirrors the `set_to_*` methods of [`MjsActuator`].
fn actuator_kind_edits() -> Vec<Edit> {
    // The methods write the dyntype, gaintype and biastype tables; the layout holds the first two.
    let kinds: Vec<ActuatorKind> = vec![
        ("motor",        |a| a.set_to_motor()),
        ("velocity",     |a| a.set_to_velocity(1.0)),
        ("position",     |a| { let _ = a.set_to_position(PositionConfig::default().with_kp(3.0)); }),
        ("int velocity", |a| { let _ = a.set_to_int_velocity(IntVelocityConfig::default().with_kp(3.0)); }),
        ("damper",       |a| { let _ = a.set_to_damper(1.0); *a.ctrlrange_mut() = [0.0, 1.0]; }),
        ("cylinder",     |a| a.set_to_cylinder(0.1, 0.0, 1.0, 0.02)),
        ("dc motor",     |a| { let _ = a.set_to_dc_motor(DcMotorConfig::default().with_resistance(1.0)
                                                       .with_motorconst([1.0, 1.0])); }),
        ("pid",          |a| { let _ = a.set_to_pid(PidConfig::default().with_kp(1.0).with_kv(1.0)); }),
        ("adhesion",     |a| { let _ = a.set_to_adhesion(1.0); *a.ctrlrange_mut() = [0.0, 1.0]; }),
    ];

    kinds.into_iter()
        .map(|(name, set)| {
            // These kinds differ from a motor only in values the layout leaves out.
            let kind = match name {
                "motor" | "velocity" | "position" | "adhesion" => Kind::Parameter,
                _ => Kind::Structural,
            };
            Edit::new(format!("actuator kind {name}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, actuator_mut, "a_motor", |a| set(a));
            })
        })
        .collect()
}

/* The sensor axis: every `MjtSensor` value the base model can host. */

all_variants!(
    /// Every [`MjtSensor`] value.
    SENSOR_TYPES: MjtSensor =
    mjSENS_TOUCH, mjSENS_ACCELEROMETER, mjSENS_VELOCIMETER, mjSENS_GYRO, mjSENS_FORCE,
    mjSENS_TORQUE, mjSENS_MAGNETOMETER, mjSENS_RANGEFINDER, mjSENS_CAMPROJECTION,
    mjSENS_JOINTPOS, mjSENS_JOINTVEL, mjSENS_TENDONPOS, mjSENS_TENDONVEL, mjSENS_ACTUATORPOS,
    mjSENS_ACTUATORVEL, mjSENS_ACTUATORFRC, mjSENS_JOINTACTFRC, mjSENS_TENDONACTFRC,
    mjSENS_BALLQUAT, mjSENS_BALLANGVEL, mjSENS_JOINTLIMITPOS, mjSENS_JOINTLIMITVEL,
    mjSENS_JOINTLIMITFRC, mjSENS_TENDONLIMITPOS, mjSENS_TENDONLIMITVEL, mjSENS_TENDONLIMITFRC,
    mjSENS_FRAMEPOS, mjSENS_FRAMEQUAT, mjSENS_FRAMEXAXIS, mjSENS_FRAMEYAXIS, mjSENS_FRAMEZAXIS,
    mjSENS_FRAMELINVEL, mjSENS_FRAMEANGVEL, mjSENS_FRAMELINACC, mjSENS_FRAMEANGACC,
    mjSENS_SUBTREECOM, mjSENS_SUBTREELINVEL, mjSENS_SUBTREEANGMOM, mjSENS_INSIDESITE,
    mjSENS_GEOMDIST, mjSENS_GEOMNORMAL, mjSENS_GEOMFROMTO, mjSENS_CONTACT, mjSENS_E_POTENTIAL,
    mjSENS_E_KINETIC, mjSENS_CLOCK, mjSENS_TACTILE, mjSENS_PLUGIN, mjSENS_USER);

/// The object a sensor points at: its own type and name, then the referenced type and name.
type SensorTarget = (MjtObj, &'static str, MjtObj, &'static str);

/// One actuator kind: its name and the `set_to_*` method that writes it.
type ActuatorKind = (&'static str, fn(&mut MjsActuator));

/// The objects a generated sensor can point at. The sweep keeps the first one that compiles.
const SENSOR_TARGETS: &[SensorTarget] = &[
    (MjtObj::mjOBJ_SITE,     "s_lower", MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_SITE,     "s_lower", MjtObj::mjOBJ_SITE,    "s_trunk"),
    (MjtObj::mjOBJ_BODY,     "lower",   MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_BODY,     "lower",   MjtObj::mjOBJ_BODY,    "trunk"),
    (MjtObj::mjOBJ_XBODY,    "lower",   MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_JOINT,    "knee",    MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_JOINT,    "slide",   MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_JOINT,    "hip",     MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_TENDON,   "td",      MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_ACTUATOR, "a_motor", MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_GEOM,     "g_upper", MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_GEOM,     "g_upper", MjtObj::mjOBJ_GEOM,    "g_lower"),
    (MjtObj::mjOBJ_CAMERA,   "c_trunk", MjtObj::mjOBJ_UNKNOWN, ""),
    (MjtObj::mjOBJ_MESH,     "ms",      MjtObj::mjOBJ_GEOM,    "g_trunk"),
];

/// Builds the edit that appends one sensor of type `t` pointed at `target`.
fn add_sensor_edit(t: MjtSensor, target: SensorTarget) -> Edit {
    let (objtype, objname, reftype, refname) = target;
    Edit::new(format!("add sensor {t:?} on {objname}"), Kind::Structural, move |spec: &mut MjSpec| {
        let sensor = spec.add_sensor();
        sensor.set_type(t);
        let _ = sensor.set_objtype(objtype);
        sensor.set_objname(objname);
        if reftype != MjtObj::mjOBJ_UNKNOWN {
            let _ = sensor.set_reftype(reftype);
            sensor.set_refname(refname);
        }
        // A rangefinder and a contact sensor both read intprm[0] as a data spec, and both reject
        // a value that is not positive. A contact sensor also rejects a match count intprm[2] that
        // is not positive.
        if matches!(t, MjtSensor::mjSENS_RANGEFINDER | MjtSensor::mjSENS_CONTACT) {
            sensor.intprm_mut()[0] = 1;
        }
        if t == MjtSensor::mjSENS_CONTACT {
            sensor.intprm_mut()[2] = 1;
        }
        // A user sensor has no built-in width, so the edit sets one. Every other type ignores it.
        if t == MjtSensor::mjSENS_USER {
            sensor.set_dim(3);
            sensor.set_needstage(MjtStage::mjSTAGE_VEL);
        }
    })
}

/// The target a sensor type must be given, or `None` when the sweep may try every target.
fn forced_target(t: MjtSensor) -> Option<SensorTarget> {
    match t {
        // A camprojection sensor that names no camera makes the compiler read through a null
        // mjCCamera pointer. The XML parser demands the attribute; the spec API does not.
        MjtSensor::mjSENS_CAMPROJECTION =>
            Some((MjtObj::mjOBJ_SITE, "s_lower", MjtObj::mjOBJ_CAMERA, "c_trunk")),
        _ => None,
    }
}

/// One edit per [`MjtSensor`] value, each pointed at the first target the compiler accepts.
///
/// Returns the edits and the sensor types that no target fitted.
fn sensor_type_edits(spec: &MjSpec) -> (Vec<Edit>, Vec<MjtSensor>) {
    let (mut edits, mut skipped) = (Vec::new(), Vec::new());
    for t in SENSOR_TYPES {
        let forced = forced_target(t);
        let candidates = forced.as_ref().map_or(SENSOR_TARGETS, std::slice::from_ref);
        let fitted = candidates.iter()
            .map(|&target| add_sensor_edit(t, target))
            .find(|candidate| compile_with(spec, &[candidate]).is_some());
        match fitted {
            Some(e) => edits.push(e),
            None => skipped.push(t),
        }
    }
    (edits, skipped)
}

/* Structural axes generated from the base model itself. */

/// One edit per named element of the base model, each deleting that element.
///
/// The sweep walks the spec, so every element the base carries takes part. The caller reports a
/// deletion that leaves an illegal model as skipped.
fn deletion_edits(spec: &MjSpec) -> Vec<Edit> {
    let mut out = Vec::new();
    macro_rules! sweep {
        ($($obj:ident, $kind:literal => $iter:ident, $finder:ident;)+) => {
            // The kinds the sweep leaves out hold no element a spec can delete: markers, a whole
            // model, an xbody, and the dof and plugin kinds that belong to the compiled model.
            all_variants!(MjtObj = $($obj),+ ;
                          except mjOBJ_UNKNOWN, mjOBJ_XBODY, mjOBJ_DOF, mjOBJ_PLUGIN, mjNOBJECT,
                                 mjOBJ_FRAME, mjOBJ_DEFAULT, mjOBJ_MODEL);
            $(
            for name in spec.$iter().map(|item| item.name().to_owned()).collect::<Vec<_>>() {
                // The world body cannot be deleted.
                if name == "world" {
                    continue;
                }
                let target = name.clone();
                out.push(Edit::new(
                    format!("delete {} '{}'", $kind, name), Kind::Open,
                    move |spec: &mut MjSpec| {
                        // SAFETY: the walk runs on the spec that the edits before it left, so it
                        // reaches a live element only, and each edit deletes one name once.
                        on_item!(spec, $finder, target, |item| unsafe { let _ = item.delete(); });
                    },
                ));
            }
        )+
        };
    }
    sweep! {
        mjOBJ_BODY,     "body"     => body_iter, body_mut;
        mjOBJ_JOINT,    "joint"    => joint_iter, joint_mut;
        mjOBJ_GEOM,     "geom"     => geom_iter, geom_mut;
        mjOBJ_SITE,     "site"     => site_iter, site_mut;
        mjOBJ_CAMERA,   "camera"   => camera_iter, camera_mut;
        mjOBJ_LIGHT,    "light"    => light_iter, light_mut;
        mjOBJ_ACTUATOR, "actuator" => actuator_iter, actuator_mut;
        mjOBJ_SENSOR,   "sensor"   => sensor_iter, sensor_mut;
        mjOBJ_TENDON,   "tendon"   => tendon_iter, tendon_mut;
        mjOBJ_EQUALITY, "equality" => equality_iter, equality_mut;
        mjOBJ_PAIR,     "pair"     => pair_iter, pair_mut;
        mjOBJ_EXCLUDE,  "exclude"  => exclude_iter, exclude_mut;
        mjOBJ_FLEX,     "flex"     => flex_iter, flex_mut;
        mjOBJ_MESH,     "mesh"     => mesh_iter, mesh_mut;
        mjOBJ_HFIELD,   "hfield"   => hfield_iter, hfield_mut;
        mjOBJ_SKIN,     "skin"     => skin_iter, skin_mut;
        mjOBJ_TEXTURE,  "texture"  => texture_iter, texture_mut;
        mjOBJ_MATERIAL, "material" => material_iter, material_mut;
        mjOBJ_NUMERIC,  "numeric"  => numeric_iter, numeric_mut;
        mjOBJ_TEXT,     "text"     => text_iter, text_mut;
        mjOBJ_TUPLE,    "tuple"    => tuple_iter, tuple_mut;
        mjOBJ_KEY,      "key"      => key_iter, key_mut;
    }
    out
}

/// One edit per element kind [`MjSpec`] and [`MjsBody`] can add, each appending one element.
fn addition_edits() -> Vec<Edit> {
    vec![
        Edit::new("add mesh", Kind::Open, |spec: &mut MjSpec| {
            let mesh = spec.add_mesh();
            let _ = mesh.set_name("added_mesh");
            mesh.set_uservert(&[0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]);
        }),
        Edit::new("add hfield", Kind::Open, |spec: &mut MjSpec| {
            let hfield = spec.add_hfield();
            let _ = hfield.set_name("added_hfield");
            hfield.set_nrow(2);
            hfield.set_ncol(2);
            hfield.set_userdata([0.0; 4]);
            *hfield.size_mut() = [1.0, 1.0, 0.1, 0.1];
        }),
        // A 1D flex takes 4 scene faces per element, so these two move the vertex and the face
        // counts one at a time.
        Edit::new("add flex vertex", Kind::Open, |spec: &mut MjSpec| {
            on_item!(spec, flex_mut, "f0", |f| {
                f.append_vertbody("spare");
                f.set_vert(&[0.0; 12]);
                f.set_elem(&[0, 1, 2, 3]);
            });
        }),
        Edit::new("close the flex loop", Kind::Open, |spec: &mut MjSpec| {
            on_item!(spec, flex_mut, "f0", |f| f.set_elem(&[0, 1, 1, 2, 2, 0]));
        }),
        // The fourth vertex needs a positive bone weight, so a second bone binds it.
        Edit::new("add skin vertex", Kind::Open, |spec: &mut MjSpec| {
            on_item!(spec, skin_mut, "sk", |s| {
                s.set_vert(&[0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.1, 0.0]);
                s.append_bodyname("upper");
                s.set_bindpos(&[0.0; 6]);
                s.set_bindquat(&[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
                s.append_vertid(&[3]);
                s.append_vertweight(&[1.0]);
            });
        }),
        Edit::new("add body", Kind::Open, |spec: &mut MjSpec| {
            let body = spec.world_body_mut().add_body();
            let _ = body.set_name("added_body");
            let geom = body.add_geom();
            *geom.size_mut() = [0.02, 0.0, 0.0];
            let _ = geom.set_name("added_body_geom");
        }),
        Edit::new("add joint", Kind::Open, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("spare") {
                let joint = body.add_joint();
                let _ = joint.set_name("added_joint");
                joint.set_type(MjtJoint::mjJNT_HINGE);
                *joint.axis_mut() = [1.0, 0.0, 0.0];
            }
        }),
        Edit::new("add geom", Kind::Open, |spec: &mut MjSpec| {
            let geom = spec.world_body_mut().add_geom();
            let _ = geom.set_name("added_geom");
            *geom.size_mut() = [0.02, 0.0, 0.0];
        }),
        Edit::new("add site", Kind::Open, |spec: &mut MjSpec| {
            let site = spec.world_body_mut().add_site();
            let _ = site.set_name("added_site");
        }),
        Edit::new("add camera", Kind::Open, |spec: &mut MjSpec| {
            let camera = spec.world_body_mut().add_camera();
            let _ = camera.set_name("added_camera");
        }),
        Edit::new("add light", Kind::Open, |spec: &mut MjSpec| {
            let light = spec.world_body_mut().add_light();
            let _ = light.set_name("added_light");
        }),
        Edit::new("add actuator", Kind::Open, |spec: &mut MjSpec| {
            let actuator = spec.add_actuator();
            let _ = actuator.set_name("added_actuator");
            actuator.set_target("slide");
            actuator.set_trntype(MjtTrn::mjTRN_JOINT);
        }),
        Edit::new("add pair", Kind::Open, |spec: &mut MjSpec| {
            let pair = spec.add_pair();
            let _ = pair.set_name("added_pair");
            pair.set_geomname1("g_trunk");
            pair.set_geomname2("g_lower");
        }),
        Edit::new("add exclude", Kind::Open, |spec: &mut MjSpec| {
            let exclude = spec.add_exclude();
            let _ = exclude.set_name("added_exclude");
            exclude.set_bodyname1("trunk");
            exclude.set_bodyname2("upper");
        }),
        Edit::new("add equality", Kind::Open, |spec: &mut MjSpec| {
            let equality = spec.add_equality();
            let _ = equality.set_name("added_equality");
            equality.set_type(MjtEq::mjEQ_WELD);
            equality.set_objtype(MjtObj::mjOBJ_BODY);
            equality.set_name1("upper");
            equality.set_name2("world");
        }),
        Edit::new("add numeric", Kind::Open, |spec: &mut MjSpec| {
            let numeric = spec.add_numeric();
            let _ = numeric.set_name("added_numeric");
            numeric.set_data(&[1.0, 2.0]);
        }),
        Edit::new("add text", Kind::Open, |spec: &mut MjSpec| {
            let text = spec.add_text();
            let _ = text.set_name("added_text");
            text.set_data("added");
        }),
        Edit::new("add tuple", Kind::Open, |spec: &mut MjSpec| {
            let tuple = spec.add_tuple();
            let _ = tuple.set_name("added_tuple");
            let _ = tuple.set_objtype(&[MjtObj::mjOBJ_BODY]);
            tuple.append_objname("upper");
            tuple.set_objprm(&[0.0]);
        }),
        Edit::new("add key", Kind::Open, |spec: &mut MjSpec| {
            let key = spec.add_key();
            let _ = key.set_name("added_key");
        }),
        Edit::new("add material", Kind::Structural, |spec: &mut MjSpec| {
            let material = spec.add_material();
            let _ = material.set_name("added_material");
        }),
    ]
}

/// One edit per size field of the spec.
fn size_edits() -> Vec<Edit> {
    macro_rules! sizes {
        ($($field:ident = $value:expr;)+) => {
            vec![$(
                Edit::new(concat!(stringify!($field), " grows"), Kind::Structural,
                     |spec: &mut MjSpec| {
                        // SAFETY: the field is plain data that the compiler reads as a count.
                        unsafe { spec.ffi_mut() }.$field = $value;
                     }),
            )+]
        };
    }
    sizes! {
        nuserdata      = 64;
        nuser_body     = 5;
        nuser_jnt      = 4;
        nuser_geom     = 6;
        nuser_site     = 4;
        nuser_cam      = 3;
        nuser_tendon   = 5;
        nuser_actuator = 4;
        nuser_sensor   = 6;
        memory         = 32 * 1024 * 1024;
    }
}

/// Edits that changes history and also the history interpolation.
fn history_edits() -> Vec<Edit> {
    vec![
        Edit::new("actuator history grows", Kind::Structural, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_int", |a| a.set_nsample(5));
        }),
        Edit::new("history changes to the sensor", Kind::Structural, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_int", |a| a.set_nsample(2));
            on_item!(spec, sensor_mut, "se_jp", |s| s.set_nsample(3));
        }),
        Edit::new("actuator interpolation", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_int", |a| a.set_interp(2));
        }),
        Edit::new("sensor interpolation", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, sensor_mut, "se_jp", |s| s.set_interp(1));
        }),
    ]
}

/* The split axis: one table at a time, against the base or a sibling edit. */

/// The corners of a closed box mesh, 0.1 on each side.
const BOX_VERTICES: [f32; 24] = [
    0.0, 0.0, 0.0,  0.1, 0.0, 0.0,  0.0, 0.1, 0.0,  0.1, 0.1, 0.0,
    0.0, 0.0, 0.1,  0.1, 0.0, 0.1,  0.0, 0.1, 0.1,  0.1, 0.1, 0.1,
];

/// The 12 outward faces of the box in [`BOX_VERTICES`].
const BOX_FACES: [i32; 36] = [
    0, 2, 1,  1, 2, 3,  4, 5, 6,  5, 7, 6,  0, 1, 4,  1, 5, 4,
    2, 6, 3,  3, 6, 7,  0, 4, 2,  2, 4, 6,  1, 3, 5,  3, 7, 5,
];

/// The 4 faces of the tetrahedron that the mesh `ms` holds.
const TETRA_FACES: [i32; 12] = [0, 2, 1,  0, 1, 3,  0, 3, 2,  1, 2, 3];

/// Edits that each change one table, against the base or against a sibling edit.
fn split_edits() -> Vec<Edit> {
    macro_rules! split {
        ($($label:literal: $finder:ident, $name:literal |$item:ident| $body:expr;)+) => {
            vec![$(
                Edit::new($label, Kind::Structural, |spec: &mut MjSpec| {
                    on_item!(spec, $finder, $name, |$item| $body);
                }),
            )+]
        };
    }

    // Rebuilds `ms` with explicit faces, 3 face normals and 4 texture coordinates, plus the given
    // extra vertex and extra texture coordinate, so that each mesh count moves on its own.
    fn tetra(spec: &mut MjSpec, extra_vertex: &[f32], extra_texcoord: &[f32]) {
        on_item!(spec, mesh_mut, "ms", |m| {
            let vertices = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
            let texcoords = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0];
            let last = 3 + extra_texcoord.len() as i32 / 2;
            m.set_uservert(&[&vertices[..], extra_vertex].concat());
            m.set_userface(&TETRA_FACES);
            m.set_usernormal(&[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
            m.set_usertexcoord(&[&texcoords[..], extra_texcoord].concat());
            // SAFETY: each normal index is below the 3 normals, and each texture coordinate index
            // is at most `last`, the final coordinate; both lists hold 3 entries per face.
            unsafe {
                m.set_userfacenormal(&[0, 0, 0, 1, 1, 1, 2, 2, 2, 0, 1, 2]);
                m.set_userfacetexcoord(&[0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, last]);
            }
        });
    }

    // Appends one box mesh per hull limit, each on a world geom. The limit caps the convex hull, so
    // it moves the graph and nothing else.
    fn boxes(spec: &mut MjSpec, limits: &[i32]) {
        for (i, &limit) in limits.iter().enumerate() {
            let name = format!("box{i}");
            let mesh = spec.add_mesh();
            let _ = mesh.set_name(&name);
            mesh.set_uservert(&BOX_VERTICES);
            mesh.set_userface(&BOX_FACES);
            mesh.set_maxhullvert(limit);
            let geom = spec.world_body_mut().add_geom();
            geom.set_type(MjtGeom::mjGEOM_MESH);
            geom.set_meshname(&name);
        }
    }

    // Rebuilds 'f0' as a 1D flex over five vertex bodies. A path and a tee keep every gate count,
    // but differ in the shell and evpair totals.
    fn five_vertices(spec: &mut MjSpec, elements: &[i32]) {
        on_item!(spec, flex_mut, "f0", |f| {
            f.set_vertbody("v0 v1 v2 spare mocap");
            f.set_vert(&[0.0; 15]);
            f.set_elem(elements);
        });
    }

    // Rebuilds 'f0' as one elastic triangle; elastic2d alone sizes the bending table.
    fn triangle(spec: &mut MjSpec, elastic2d: i32) {
        on_item!(spec, flex_mut, "f0", |f| {
            f.set_vertbody("spare mocap spare");
            f.set_dim(2);
            f.set_edgestiffness(0.0);
            f.set_edgedamping(0.0);
            f.set_elem(&[0, 1, 2]);
            f.set_vert(&[0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0]);
            f.set_young(1e3);
            f.set_thickness(0.01);
            f.set_elastic2d(elastic2d);
        });
    }

    // Rebuilds 'f0' on a grid of order + 1 nodes per side, 0.1 wide, every node on 'v0'. Two orders
    // differ only in the node count and the order; every gate count stays.
    fn nodes(spec: &mut MjSpec, order: i32) {
        on_item!(spec, flex_mut, "f0", |f| {
            let k = order + 1;
            let step = 0.1 / f64::from(order);
            let grid: Vec<_> = (0..k.pow(3))
                .flat_map(|n| [n / (k * k), n / k % k, n % k].map(|i| f64::from(i) * step))
                .collect();
            f.set_order(order);
            f.set_nodebody(&["v0"].repeat(grid.len() / 3).join(" "));
            f.set_node(&grid);
            f.set_vert(&[0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.09, 0.09, 0.09]);
            f.set_edgestiffness(0.0);
            f.set_edgedamping(0.0);
            f.set_selfcollide(MjtFlexSelf::mjFLEXSELF_NONE);
        });
    }

    // The flex edits pick vertex bodies by their dof count: 'spare' and 'mocap' carry none, 'v0'
    // and 'trunk' 6, 'upper' 9. The vertex and the edge Jacobians count those dofs.
    let mut edits = split! {
        "texture width":         texture_mut,  "tx"      |t| t.set_width(32);
        "equality on sites":     equality_mut, "eq0"     |q| {
            q.set_objtype(MjtObj::mjOBJ_SITE);
            q.set_name1("s_lower");
            q.set_name2("s_trunk");
        };
        "texture height":        texture_mut,  "tx"      |t| t.set_height(32);
        "texture channels":      texture_mut,  "tx"      |t| t.set_nchannel(4);
        "heightfield rows":      hfield_mut,   "hf"      |h| {
            h.set_nrow(6);
            h.set_userdata(vec![0.0; 6 * 9]);
        };
        "heightfield columns":   hfield_mut,   "hf"      |h| {
            h.set_ncol(10);
            h.set_userdata(vec![0.0; 5 * 10]);
        };
        "user sensor width":     sensor_mut,   "se_u1"   |s| s.set_dim(3);
        "sensor object type":    sensor_mut,   "se_pos"  |s| {
            let _ = s.set_objtype(MjtObj::mjOBJ_BODY);
            s.set_objname("lower");
        };
        "sensor reference type": sensor_mut,   "se_pos"  |s| {
            let _ = s.set_reftype(MjtObj::mjOBJ_SITE);
            s.set_refname("s_lower");
        };
        "transmission type":     actuator_mut, "a_motor" |a| a.set_trntype(MjtTrn::mjTRN_JOINTINPARENT);
        "moment on a ball":      actuator_mut, "a_motor" |a| a.set_target("hip");
        "pid position input":    actuator_mut, "a_pid"   |a| a.set_ctrlspec(MjtCtrlInput::mjINPUT_POS as i32);
        // The two user dynamics differ from each other in the activation count alone.
        "user dynamics":         actuator_mut, "a_filt"  |a| a.set_dyntype(MjtDyn::mjDYN_USER);
        "user dynamics, 2 acts": actuator_mut, "a_filt"  |a| {
            a.set_dyntype(MjtDyn::mjDYN_USER);
            a.set_actdim(2);
        };
        // 'g_m' sits on a mocap body and 'g_v0' on a free body, so the two wraps differ from each
        // other in the Jacobian row alone.
        "tendon wraps a geom":   tendon_mut,   "td"      |t| {
            t.wrap_geom("g_m", "");
            t.wrap_site("s_trunk");
        };
        "tendon wraps a dof":    tendon_mut,   "td"      |t| {
            t.wrap_geom("g_v0", "");
            t.wrap_site("s_trunk");
        };
        "flex vertex dofs":      flex_mut,     "f0"      |f| f.set_vertbody("spare upper v0");
        "flex edge dofs":        flex_mut,     "f0"      |f| f.set_vertbody("upper v0 trunk");
        // The mocap vertex closes the loop without a dof, so the two differ in the edge count alone.
        "flex repeated edge":    flex_mut,     "f0"      |f| {
            f.set_vertbody("spare v0 mocap");
            f.set_elem(&[0, 1, 1, 2, 0, 1]);
        };
        "flex closed edge":      flex_mut,     "f0"      |f| {
            f.set_vertbody("spare v0 mocap");
            f.set_elem(&[0, 1, 1, 2, 2, 0]);
        };
        "flex elasticity":       flex_mut,     "f0"      |f| f.set_young(1e3);
        "geom leaves the bvh":   geom_mut,     "g_sp"    |g| { g.set_contype(0); g.set_conaffinity(0); };
        "mocap leaves the bvh":  geom_mut,     "g_m"     |g| { g.set_contype(0); g.set_conaffinity(0); };
        "spare becomes mocap":   body_mut,     "spare"   |b| b.set_mocap(true);
        // The two tendons run over the same sites, so the pair moves only the wrap split.
        "td gains a site":       tendon_mut,   "td"      |t| t.wrap_site("s_trunk");
        "t2 gains a site":       tendon_mut,   "t2"      |t| t.wrap_site("s_trunk");
        // Against 'tendon wraps a geom', this moves only the wrap types.
        "td wraps two sites":    tendon_mut,   "td"      |t| {
            t.wrap_site("s_trunk");
            t.wrap_site("s_lower");
        };
        "t0 gains an element":   tuple_mut,    "t0"      |t| {
            let _ = t.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_BODY]);
            t.append_objname("upper");
            t.set_objprm(&[0.0, 0.0]);
        };
        // Against 'add site', which puts the site on the world, this moves only the site owner.
        "spare gains a site":    body_mut,     "spare"   |b| { b.add_site(); };
    };
    edits.extend([
        // The explicit normals set the normal count; the other two differ from it in one count.
        Edit::new("mesh normals", Kind::Structural, |spec: &mut MjSpec| tetra(spec, &[], &[])),
        Edit::new("mesh vertex", Kind::Structural, |spec: &mut MjSpec| tetra(spec, &[0.02; 3], &[])),
        Edit::new("mesh texcoord", Kind::Structural, |spec: &mut MjSpec| tetra(spec, &[], &[0.5; 2])),
        // A fifth face adds two nodes to the mesh bvh; the two geoms that leave it take two away.
        Edit::new("mesh face", Kind::Structural, |spec: &mut MjSpec| {
            on_item!(spec, mesh_mut, "ms", |m| m.set_userface(&[&TETRA_FACES[..], &[1, 2, 3]].concat()));
            for name in ["g_sp", "g_m"] {
                on_item!(spec, geom_mut, name, |g| { g.set_contype(0); g.set_conaffinity(0); });
            }
        }),
        // Against the first, the second moves the graph size; against the second, the third moves
        // the graph address.
        Edit::new("box hulls", Kind::Structural, |spec: &mut MjSpec| boxes(spec, &[-1, -1])),
        Edit::new("second hull capped", Kind::Structural, |spec: &mut MjSpec| boxes(spec, &[-1, 5])),
        Edit::new("first hull capped", Kind::Structural, |spec: &mut MjSpec| boxes(spec, &[5, -1])),
        // A raised corner splits a box side into two polygons, and every other count stays.
        Edit::new("box corner moves", Kind::Structural, |spec: &mut MjSpec| {
            boxes(spec, &[-1, -1]);
            let mut vertices = BOX_VERTICES;
            vertices[23] = 0.13;
            spec.mesh_mut("box0").unwrap().set_uservert(&vertices);
        }),
        Edit::new("body moves under the mocap", Kind::Structural, |spec: &mut MjSpec| {
            // SAFETY: 'spare' is live and nothing refers to it.
            on_item!(spec, body_mut, "spare", |b| unsafe { let _ = b.delete(); });
            if let Some(mocap) = spec.world_body_mut().child_mut("mocap") {
                // The new child takes the slot that 'spare' left, so no id moves.
                let body = mocap.add_body();
                let _ = body.set_name("spare");
                let geom = body.add_geom();
                let _ = geom.set_name("g_sp");
                *geom.size_mut() = [0.03, 0.0, 0.0];
            }
        }),
        Edit::new("flex path", Kind::Structural,
                  |spec: &mut MjSpec| five_vertices(spec, &[0, 1, 1, 2, 2, 3, 3, 4])),
        Edit::new("flex tee", Kind::Structural,
                  |spec: &mut MjSpec| five_vertices(spec, &[0, 1, 0, 2, 0, 3, 3, 4])),
        Edit::new("flex triangle", Kind::Structural, |spec: &mut MjSpec| triangle(spec, 0)),
        Edit::new("flex bending triangle", Kind::Structural, |spec: &mut MjSpec| triangle(spec, 1)),
        Edit::new("flex trilinear nodes", Kind::Structural, |spec: &mut MjSpec| nodes(spec, 1)),
        Edit::new("flex quadratic nodes", Kind::Structural, |spec: &mut MjSpec| nodes(spec, 2)),
    ]);
    edits
}

/* The parameter axis: values MuJoCo stores that move no element. */

/// Edits that change a value only. The gate must accept every one of them.
fn parameter_edits() -> Vec<Edit> {
    vec![
        Edit::new("site type", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, site_mut, "s_trunk", |s| s.set_type(MjtGeom::mjGEOM_BOX));
        }),
        Edit::new("pair condim", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, pair_mut, "p0", |p| p.set_condim(1));
        }),
        Edit::new("mocap moves to spare", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, body_mut, "mocap", |b| b.set_mocap(false));
            on_item!(spec, body_mut, "spare", |b| b.set_mocap(true));
        }),
        Edit::new("mesh sdf", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, mesh_mut, "ms", |m| m.set_needsdf(true));
        }),
        Edit::new("flex cell count", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, flex_mut, "f0", |f| f.cellcount_mut()[0] = 2);
        }),
        Edit::new("flex texture coordinates", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, flex_mut, "f0", |f| f.set_texcoord(&[0.0; 6]));
        }),
        Edit::new("actuator armature", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_motor", |a| a.set_armature(0.1));
        }),
        Edit::new("nconmax", Kind::Parameter, |spec: &mut MjSpec| {
            // SAFETY: the field is plain data that the compiler copies into the model.
            unsafe { spec.ffi_mut() }.nconmax = 77;
        }),
        Edit::new("timestep", Kind::Parameter,
                  |spec: &mut MjSpec| spec.option_mut().timestep = 0.01),
        Edit::new("gravity", Kind::Parameter,
                  |spec: &mut MjSpec| spec.option_mut().gravity = [0.0, 0.0, -1.0]),
        Edit::new("solver iterations", Kind::Parameter,
                  |spec: &mut MjSpec| spec.option_mut().iterations = 200),
        Edit::new("solver tolerance", Kind::Parameter,
                  |spec: &mut MjSpec| spec.option_mut().tolerance = 1e-10),
        Edit::new("geom size", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_mut, "g_lower", |g| *g.size_mut() = [0.07, 0.0, 0.2]);
        }),
        Edit::new("geom density", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_mut, "g_lower", |g| g.set_density(2000.0));
        }),
        Edit::new("geom friction", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_mut, "g_lower", |g| *g.friction_mut() = [2.0, 0.01, 0.001]);
        }),
        Edit::new("geom rgba", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_mut, "g_trunk", |g| *g.rgba_mut() = [1.0, 0.0, 0.0, 0.5]);
        }),
        Edit::new("body position", Kind::Parameter, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("trunk") {
                *body.pos_mut() = [0.0, 0.5, 1.0];
            }
        }),
        Edit::new("body mass", Kind::Parameter, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("spare") {
                // Without an explicit inertial clause the compiler takes the mass from the geoms.
                body.set_explicitinertial(true);
                body.set_mass(3.0);
                *body.inertia_mut() = [0.01; 3];
            }
        }),
        Edit::new("joint armature", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_mut, "slide", |j| j.set_armature(0.1));
        }),
        Edit::new("joint range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_mut, "knee", |j| *j.range_mut() = [-2.0, 2.0]);
        }),
        Edit::new("joint axis", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_mut, "knee", |j| *j.axis_mut() = [1.0, 0.0, 0.0]);
        }),
        Edit::new("joint friction loss", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_mut, "slide", |j| j.set_frictionloss(0.2));
        }),
        Edit::new("actuator gear", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_motor", |a| *a.gear_mut() = [7.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        }),
        Edit::new("actuator control range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_mut, "a_pos", |a| *a.ctrlrange_mut() = [-2.0, 2.0]);
        }),
        Edit::new("tendon stiffness", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, tendon_mut, "td", |t| t.stiffness_mut()[0] = 4.0);
        }),
        Edit::new("tendon range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, tendon_mut, "td", |t| *t.range_mut() = [0.0, 2.0]);
        }),
        Edit::new("sensor noise", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, sensor_mut, "se_jp", |s| s.set_noise(0.01));
        }),
        Edit::new("sensor cutoff", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, sensor_mut, "se_jp", |s| s.set_cutoff(3.0));
        }),
        Edit::new("equality solimp", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, equality_mut, "eq0", |e| *e.solimp_mut() = [0.8, 0.9, 0.001, 0.5, 2.0]);
        }),
        Edit::new("pair friction", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, pair_mut, "p0", |p| *p.friction_mut() = [2.0, 2.0, 0.01, 0.001, 0.001]);
        }),
        Edit::new("pair margin", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, pair_mut, "p0", |p| p.set_margin(0.01));
        }),
        Edit::new("numeric values", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, numeric_mut, "n0", |n| n.set_data(&[9.0, 9.0, 9.0]));
        }),
        Edit::new("keyframe time", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, key_mut, "k1", |k| k.set_time(5.0));
        }),
        Edit::new("material rgba", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, material_mut, "mat", |m| *m.rgba_mut() = [0.0, 1.0, 0.0, 1.0]);
        }),
        Edit::new("texture colour", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, texture_mut, "tx", |t| *t.rgb1_mut() = [0.0, 0.0, 1.0]);
        }),
        Edit::new("light diffuse", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, light_mut, "l0", |l| *l.diffuse_mut() = [0.1, 0.2, 0.3]);
        }),
        Edit::new("camera field of view", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, camera_mut, "c_trunk", |c| c.set_fovy(70.0));
        }),
        Edit::new("skin inflate", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, skin_mut, "sk", |s| s.set_inflate(0.03));
        }),
        Edit::new("mesh scale", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, mesh_mut, "ms", |m| *m.scale_mut() = [2.0, 2.0, 2.0]);
        }),
        Edit::new("heightfield extent", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, hfield_mut, "hf", |h| *h.size_mut() = [2.0, 2.0, 0.3, 0.05]);
        }),
        Edit::new("element name", Kind::Parameter, |spec: &mut MjSpec| {
            // 'spare' carries no reference from any other element, so the rename stands alone.
            on_item!(spec, body_mut, "spare", |b| { let _ = b.set_name("spare_renamed_much_longer"); });
        }),
    ]
}

/* Assembly. */

/// Every edit the suite knows, and the sensor types no target fitted.
fn all_edits(spec: &MjSpec) -> (Vec<Edit>, Vec<MjtSensor>) {
    let (sensors, skipped) = sensor_type_edits(spec);
    let edits = joint_type_edits().into_iter()
        .chain(geom_type_edits())
        .chain(texture_type_edits())
        .chain(equality_type_edits())
        .chain(actuator_kind_edits())
        .chain(sensors)
        .chain(deletion_edits(spec))
        .chain(addition_edits())
        .chain(size_edits())
        .chain(history_edits())
        .chain(split_edits())
        .chain(parameter_edits())
        .collect();
    (edits, skipped)
}

/// Compiles the base model with every edit alone. Returns the base model and every model that
/// compiles, keyed by label, and the labels that produce no model.
fn matrix(spec: &MjSpec, edits: &[Edit]) -> (Vec<(String, MjModel)>, Vec<String>) {
    let (mut built, mut skipped) = (vec![("base".to_owned(), base())], Vec::new());
    for e in edits {
        match compile_with(spec, &[e]) {
            Some(model) => built.push((e.label.clone(), model)),
            None => skipped.push(e.label.clone()),
        }
    }
    (built, skipped)
}

/* Reference structure check, written from the C headers and independent of MjModelLayout. */

/// Compares each named accessor and reports the first that differs.
macro_rules! first_difference {
    ($a:ident, $b:ident, $($field:ident),+ $(,)?) => {
        $( if $a.$field() != $b.$field() { return Some(stringify!($field)); } )+
    };
}

/// Returns the name of the first size or table where `a` and `b` keep their elements in different
/// places, or `None` when every one agrees.
///
/// The list comes from `mjmodel.h`, so it is independent of the list `MjModelLayout` holds.
fn first_structural_difference(a: &MjModel, b: &MjModel) -> Option<&'static str> {
    first_difference!(a, b,
        nq, nv, nu, nactuator, nout, na, nbody, nbvh, nbvhstatic, nbvhdynamic, njnt, ntree,
        n_m, n_b, n_c, n_d, ngeom, nsite, ncam, nlight, nflex, nflexvert, nflexedge,
        nflexelem, nflexelemdata, nflexstiffness, nflexelemedge,
        n_jfe, n_jfv, n_jmom, n_jten, nmesh, nmeshvert, nmeshnormal,
        nmeshtexcoord, nmeshface, nmeshgraph, nskin,
        nhfield, nhfielddata, ntex,
        ntexdata, nmat, npair, nexclude, neq, ntendon, nwrap, nsensor, nnumeric, nnumericdata,
        ntuple, ntupledata, nkey, nmocap, nplugin, nuser_body,
        nuser_jnt, nuser_geom, nuser_site, nuser_cam, nuser_tendon, nuser_actuator, nuser_sensor,
        nemax, nuserdata, nsensordata,
        npluginstate, nhistory, narena,
    );
    // Skipped as they have no MjData (or cached range) uses: ngravcomp, nefm0dof, nefm0L, nbuffer,
    // njmax, nconmax, npolygonmax, nmeshdegmax, noct, nmeshpoly, nmeshpolyvert, nmeshpolymap,
    // nflexnode, nflexbending, nflexshelldata, nflexevpair, nflextexcoord, npluginattr, the name,
    // path, text and skin sizes and tables (apart from nskin), body_weldid, body_mocapid,
    // body_bvhadr, body_bvhnum, jnt_actuatorid, dof_simplenum, geom_dataid, site_type, pair_dim,
    // flex_interp, flex_cellnum, flex_nodeadr, flex_nodenum, flex_edgeadr, flex_edgenum,
    // flex_stiffnessadr, flex_bendingadr, flex_shellnum, flex_shelldataadr, flex_evpairadr,
    // flex_evpairnum, flex_texcoordadr, mesh_octadr, mesh_octnum, mesh_polynum, mesh_polyadr,
    // mesh_polyvertadr, mesh_polyvertnum, mesh_polymapadr, mesh_polymapnum, tendon_adr, tendon_num,
    // plugin_attradr.
    first_difference!(a, b,
        body_parentid, body_rootid, body_jntnum, body_jntadr,
        body_dofnum, body_dofadr, body_treeid, body_geomnum, body_geomadr, body_plugin,
        jnt_type, jnt_qposadr, jnt_dofadr, jnt_bodyid,
        dof_bodyid, dof_jntid, dof_parentid, dof_treeid,
        tree_bodyadr, tree_bodynum, tree_dofadr, tree_dofnum,
        geom_type, geom_bodyid, geom_plugin,
        site_bodyid, cam_bodyid, light_bodyid,
        flex_dim, flex_vertadr, flex_vertnum, flex_elemadr, flex_elemnum,
        flex_elemdataadr, flex_elemedgeadr, flex_bvhadr, flex_bvhnum,
        mesh_vertadr, mesh_vertnum, mesh_faceadr, mesh_facenum, mesh_normaladr, mesh_normalnum,
        mesh_texcoordadr, mesh_texcoordnum, mesh_graphadr, mesh_bvhadr, mesh_bvhnum,
        hfield_nrow, hfield_ncol, hfield_adr,
        tex_type, tex_height, tex_width, tex_nchannel, tex_adr,
        eq_type, eq_objtype,
        ten_j_rownnz, ten_j_rowadr, wrap_type,
        actuator_trntype, actuator_dyntype, actuator_gaintype, actuator_ctrladr,
        actuator_ctrlnum, actuator_outadr, actuator_outnum, actuator_actadr, actuator_actnum,
        actuator_plugin, actuator_historyadr,
        sensor_type, sensor_objtype, sensor_reftype, sensor_dim,
        sensor_adr, sensor_plugin, sensor_historyadr,
        plugin, plugin_stateadr, plugin_statenum,
        numeric_adr, numeric_size, tuple_adr, tuple_size,
    );
    // A history keeps its samples in place under every interpolation order, so only the sample
    // count takes part.
    let nsample = |m: &MjModel| m.actuator_history().iter().chain(m.sensor_history()).map(|h| h[0])
        .collect::<Vec<_>>();
    (nsample(a) != nsample(b)).then_some("history nsample")
}

/// Returns each scene quantity, paired with whether it differs between `a` and `b`.
fn scene_quantities(a: (&MjModel, &MjvScene), b: (&MjModel, &MjvScene))
    -> impl Iterator<Item = (&'static str, bool)>
{
    let ((a, scene_a), (b, scene_b)) = (a, b);
    let planes = |m: &MjModel| m.geom_type().iter().filter(|&&t| t == MjtGeom::mjGEOM_PLANE).count();
    [
        ("nmat", a.nmat() != b.nmat()),
        ("ntex", a.ntex() != b.ntex()),
        ("nmesh", a.nmesh() != b.nmesh()),
        ("nhfield", a.nhfield() != b.nhfield()),
        ("planes", planes(a) != planes(b)),
        ("skinvertnum", scene_a.skinvertnum() != scene_b.skinvertnum()),
        ("flexvertnum", scene_a.flexvertnum() != scene_b.flexvertnum()),
        ("flexfacenum", scene_a.flexfacenum() != scene_b.flexfacenum()),
    ].into_iter()
}

/// Returns the first asset field where `a` and `b` keep their mesh, texture or heightfield data in
/// different places, or give a texture a different kind.
///
/// The list comes from `mjmodel.h` and holds the asset arrays and the texture kind, which the
/// viewer's uploads read.
fn first_asset_difference(a: &MjModel, b: &MjModel) -> Option<&'static str> {
    first_difference!(a, b,
        nmesh, nmeshvert, nmeshnormal, nmeshtexcoord, nmeshface, nmeshgraph, ntex, ntexdata,
        nhfield, nhfielddata,
        mesh_vertadr, mesh_vertnum, mesh_normaladr, mesh_normalnum, mesh_texcoordadr,
        mesh_texcoordnum, mesh_faceadr, mesh_facenum, mesh_graphadr,
        tex_adr, tex_width, tex_height, tex_nchannel, tex_type,
        hfield_adr, hfield_nrow, hfield_ncol,
    );
    None
}

/// Asserts that the two views resolved to the same slice of the same buffer. The `opt` form
/// takes a field that the view wraps in an `Option`.
macro_rules! same_slice {
    (opt $label:expr, $kind:expr, $foreign:expr, $own:expr, $field:ident) => {
        let (foreign, own) = ($foreign, $own);
        let foreign = foreign.$field.as_ref().expect("the field is present");
        let own = own.$field.as_ref().expect("the field is present");
        assert_eq!(
            (foreign.as_ptr(), foreign.len()), (own.as_ptr(), own.len()),
            "variant '{}': the {} Info from the base model resolved '{}' elsewhere",
            $label, $kind, stringify!($field),
        );
    };
    ($label:expr, $kind:expr, $foreign:expr, $own:expr, $field:ident) => {
        assert_eq!(
            ($foreign.$field.as_ptr(), $foreign.$field.len()),
            ($own.$field.as_ptr(), $own.$field.len()),
            "variant '{}': the {} Info from the base model resolved '{}' elsewhere",
            $label, $kind, stringify!($field),
        );
    };
}

/* Tests. */

/// Every ordered pair of the matrix, checked against the reference.
#[test]
fn test_the_gate_agrees_with_an_independent_structure_check() {
    let spec = base_spec();
    let (edits, skipped_sensors) = all_edits(&spec);
    let (models, skipped) = matrix(&spec, &edits);
    let built = models.len() - 1;

    // The sweep names every edit that produced no model, so it cannot shrink unnoticed.
    println!("{} edits, {} compiled, {} did not: {:?}", edits.len(), built, skipped.len(), skipped);
    let expected = ["geom type mjGEOM_PLANE", "geom type mjGEOM_SDF", "equality type mjEQ_DISTANCE"];
    let unexpected: Vec<_> = skipped.iter()
        .filter(|label| !label.starts_with("delete ") && !expected.contains(&label.as_str()))
        .collect();
    assert!(unexpected.is_empty(), "these edits do not compile: {unexpected:?}");
    // A plugin sensor needs a plugin instance; the plugin test covers it.
    assert_eq!(skipped_sensors, [MjtSensor::mjSENS_PLUGIN],
               "the set of sensor types that fit no target changed");

    // A model that agrees with the base can still disagree with another variant, so the sweep
    // runs variant against variant too.
    let scenes: Vec<_> = models.iter().map(|(_, model)| MjvScene::new(model, 0)).collect();
    let (mut compatible, mut scene_alone) = (0, BTreeSet::new());
    for ((label_a, a), scene_a) in models.iter().zip(&scenes) {
        for ((label_b, b), scene_b) in models.iter().zip(&scenes) {
            let gate = a.is_compatible_with_model(b);
            let difference = first_structural_difference(a, b);
            assert_eq!(gate, difference.is_none(),
                       "'{label_a}' against '{label_b}': the gate says compatible={gate}, the \
                        reference found {difference:?}");
            compatible += usize::from(gate);

            let differences: Vec<_> = scene_quantities((a, scene_a), (b, scene_b))
                .filter_map(|(name, differs)| differs.then_some(name)).collect();
            for gate in [scene_a.is_compatible_with_model(b), scene_a.is_compatible_with_scene(scene_b)] {
                assert_eq!(gate, differences.is_empty(),
                           "'{label_a}' against '{label_b}': the scene gate says compatible={gate}, \
                            the reference found {differences:?}");
            }
            if let [difference] = differences[..] {
                scene_alone.insert(difference);
            }
        }
    }
    assert!(compatible > models.len(), "too few compatible pairs: {compatible}");
    assert!(compatible < models.len() * models.len(), "every pair passed, the gate accepts all");

    // Each scene quantity must be the only difference of some pair, so that a gate which drops
    // it fails above.
    let (base, scene) = (&models[0].1, &scenes[0]);
    assert_eq!(scene_alone.len(), scene_quantities((base, scene), (base, scene)).count(),
               "a scene quantity is never the only difference: {scene_alone:?}");
}

/// Every pair of edits applied together, checked in both directions against the base and against
/// each single-edit parent.
#[test]
fn test_every_pair_of_edits_agrees_with_the_reference() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);
    let parents: Vec<_> = edits.iter().map(|e| compile_with(&spec, &[e])).collect();

    let (mut checked, mut skipped, mut accepted) = (0, 0, 0);
    let (mut compared, mut absent) = (0, 0);
    for (i, first) in edits.iter().enumerate() {
        for (offset, second) in edits[i + 1..].iter().enumerate() {
            let Some(model) = compile_with(&spec, &[first, second]) else {
                skipped += 1;
                continue;
            };
            let label = format!("{} + {}", first.label, second.label);
            // A parent differs by one edit, so it is the closest neighbour the matrix holds. Two
            // pair models never meet: the reference walks every size and table on each call.
            let others = [("the base", Some(&base)),
                          (first.label.as_str(), parents[i].as_ref()),
                          (second.label.as_str(), parents[i + 1 + offset].as_ref())];

            for (other_label, other) in others {
                // An edit that compiles in a pair may still fail alone, and then it has no parent.
                let Some(other) = other else { absent += 1; continue };
                for (a, b) in [(&model, other), (other, &model)] {
                    let gate = a.is_compatible_with_model(b);
                    let difference = first_structural_difference(a, b);
                    assert_eq!(gate, difference.is_none(),
                               "'{label}' against '{other_label}': the gate says \
                                compatible={gate}, the reference found {difference:?}");
                }
                compared += 1;
            }
            accepted += usize::from(base.is_compatible_with_model(&model));
            checked += 1;
        }
    }
    println!("{checked} pairs checked, {skipped} did not compile, {accepted} accepted by the gate");
    println!("{compared} model pairs compared, {absent} parents missing");
    assert!(accepted > 0, "no pair of edits stays compatible");
}


/// An edit declared as a parameter change must stay compatible, in both directions and through
/// the asset gate.
#[test]
fn test_parameter_edits_stay_compatible() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);
    let declared: Vec<_> = edits.iter().filter(|e| e.kind == Kind::Parameter).collect();
    assert!(!declared.is_empty(), "the parameter axis is empty");
    let bytes = |m: &MjModel| {
        let mut buffer = vec![0; m.size()];
        m.save_to_buffer(&mut buffer).unwrap();
        buffer
    };
    // The type sweeps also visit the value that the base already holds.
    let base_values = ["joint type mjJNT_FREE", "texture type mjTEXTURE_2D", "equality type mjEQ_CONNECT",
                       "actuator kind motor"];

    for e in declared {
        let model = compile_with(&spec, &[e])
            .unwrap_or_else(|| panic!("'{}' produces no model", e.label));
        assert!(base_values.contains(&e.label.as_str()) || bytes(&model) != bytes(&base),
                "'{}' leaves the model unchanged, so it tests nothing", e.label);
        assert!(base.is_compatible_with_model(&model),
                "'{}' changes a value only, so it must stay compatible", e.label);
        assert!(model.is_compatible_with_model(&base),
                "'{}' must be compatible in both directions", e.label);
        assert!(base.is_asset_compatible_with_model(&model), "'{}' moves no asset", e.label);
    }
}

/// The gate must reject an edit declared as a structural change, in both directions.
#[test]
fn test_structural_edits_are_rejected() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);
    let declared: Vec<_> = edits.iter().filter(|e| e.kind == Kind::Structural).collect();
    let mut checked = 0;
    for e in declared {
        let Some(model) = compile_with(&spec, &[e]) else { continue };
        assert!(!base.is_compatible_with_model(&model),
                "'{}' moves an element or resizes a buffer, so it must be rejected", e.label);
        assert!(!model.is_compatible_with_model(&base),
                "'{}' must be rejected in both directions", e.label);
        checked += 1;
    }
    assert!(checked > 0, "no structural edit produced a model");
}

/// Each pair differs in one gate field alone, so the gate must refuse it in both directions.
#[test]
fn test_declared_incompatible_pairs_are_rejected() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let build = |label: &str| {
        let edit = edits.iter().find(|e| e.label == label).unwrap_or_else(|| panic!("no edit '{label}'"));
        compile_with(&spec, &[edit]).unwrap_or_else(|| panic!("'{label}' produces no model"))
    };
    let pairs = [
        ("flex repeated edge", "flex closed edge"),                                         // nflexedge
        ("joint type mjJNT_SLIDE", "joint type mjJNT_HINGE"),                               // jnt_type
        ("add sensor mjSENS_TOUCH on s_lower", "add sensor mjSENS_RANGEFINDER on s_lower"), // sensor_type
        ("texture type mjTEXTURE_CUBE", "texture type mjTEXTURE_SKYBOX"),                   // tex_type
        ("box hulls", "second hull capped"),                                                // nmeshgraph
        ("second hull capped", "first hull capped"),                                        // mesh_graphadr
    ];
    for (label_a, label_b) in pairs {
        let (a, b) = (build(label_a), build(label_b));
        assert!(!a.is_compatible_with_model(&b) && !b.is_compatible_with_model(&a),
                "'{label_a}' against '{label_b}' must be rejected in both directions");
    }
    // tex_type, nmeshgraph and mesh_graphadr are asset tables, so the asset gate must refuse too.
    for &(label_a, label_b) in &pairs[3..] {
        let (a, b) = (build(label_a), build(label_b));
        assert!(!a.is_asset_compatible_with_model(&b) && !b.is_asset_compatible_with_model(&a),
                "'{label_a}' against '{label_b}': the asset gate must refuse in both directions");
    }
}

/// A compatible model must resolve every cached range to the same place.
#[test]
fn test_a_compatible_model_resolves_every_cached_range_identically() {
    let spec = base_spec();
    let base = base();
    let base_data = base.make_data();
    let (edits, _) = all_edits(&spec);

    // The promise covers every model the gate accepts, so the sweep covers that whole set.
    for e in &edits {
        let Some(other) = compile_with(&spec, &[e]) else { continue };
        if !base.is_compatible_with_model(&other) {
            continue;
        }
        let other_data = other.make_data();
        let label = &e.label;

        same_slice!(label, "body", base.body("trunk").unwrap().view(&other),
                    other.body("trunk").unwrap().view(&other), user);
        same_slice!(label, "joint", base.joint("knee").unwrap().view(&other),
                    other.joint("knee").unwrap().view(&other), user);
        same_slice!(label, "geom", base.geom("g_lower").unwrap().view(&other),
                    other.geom("g_lower").unwrap().view(&other), user);
        same_slice!(label, "site", base.site("s_lower").unwrap().view(&other),
                    other.site("s_lower").unwrap().view(&other), user);
        same_slice!(label, "mesh", base.mesh("ms").unwrap().view(&other),
                    other.mesh("ms").unwrap().view(&other), vertadr);
        same_slice!(opt label, "texture", base.texture("tx").unwrap().view(&other),
                    other.texture("tx").unwrap().view(&other), data);
        same_slice!(opt label, "hfield", base.hfield("hf").unwrap().view(&other),
                    other.hfield("hf").unwrap().view(&other), data);
        same_slice!(opt label, "numeric", base.numeric("n1").unwrap().view(&other),
                    other.numeric("n1").unwrap().view(&other), data);
        same_slice!(label, "key", base.key("k1").unwrap().view(&other),
                    other.key("k1").unwrap().view(&other), qpos);
        same_slice!(label, "camera", base.camera("c_trunk").unwrap().view(&other),
                    other.camera("c_trunk").unwrap().view(&other), user);
        same_slice!(label, "light", base.light("l_trunk").unwrap().view(&other),
                    other.light("l_trunk").unwrap().view(&other), pos);
        same_slice!(label, "material", base.material("mat").unwrap().view(&other),
                    other.material("mat").unwrap().view(&other), rgba);
        same_slice!(label, "pair", base.pair("p0").unwrap().view(&other),
                    other.pair("p0").unwrap().view(&other), solref);
        same_slice!(label, "exclude", base.exclude("x0").unwrap().view(&other),
                    other.exclude("x0").unwrap().view(&other), signature);
        same_slice!(label, "equality", base.equality("eq1").unwrap().view(&other),
                    other.equality("eq1").unwrap().view(&other), data);
        same_slice!(label, "actuator", base.actuator("a_int").unwrap().view(&other),
                    other.actuator("a_int").unwrap().view(&other), gear);
        same_slice!(label, "actuator", base.actuator("a_int").unwrap().view(&other),
                    other.actuator("a_int").unwrap().view(&other), user);
        same_slice!(label, "sensor", base.sensor("se_u1").unwrap().view(&other),
                    other.sensor("se_u1").unwrap().view(&other), user);
        same_slice!(label, "tendon", base.tendon("td").unwrap().view(&other),
                    other.tendon("td").unwrap().view(&other), user);
        same_slice!(label, "skin", base.skin("sk").unwrap().view(&other),
                    other.skin("sk").unwrap().view(&other), rgba);
        same_slice!(label, "tuple", base.tuple("t0").unwrap().view(&other),
                    other.tuple("t0").unwrap().view(&other), objprm);
        same_slice!(label, "body data", base_data.body("trunk").unwrap().view(&other_data),
                    other_data.body("trunk").unwrap().view(&other_data), xfrc_applied);
        same_slice!(label, "geom data", base_data.geom("g_lower").unwrap().view(&other_data),
                    other_data.geom("g_lower").unwrap().view(&other_data), xpos);
        same_slice!(label, "site data", base_data.site("s_lower").unwrap().view(&other_data),
                    other_data.site("s_lower").unwrap().view(&other_data), xpos);
        same_slice!(label, "camera data", base_data.camera("c_trunk").unwrap().view(&other_data),
                    other_data.camera("c_trunk").unwrap().view(&other_data), xpos);
        same_slice!(label, "light data", base_data.light("l_trunk").unwrap().view(&other_data),
                    other_data.light("l_trunk").unwrap().view(&other_data), xpos);
        same_slice!(label, "actuator data", base_data.actuator("a_int").unwrap().view(&other_data),
                    other_data.actuator("a_int").unwrap().view(&other_data), ctrl);

        // The dynamic ranges: a joint owns a slice of qpos and of qvel, a sensor owns a slice of
        // sensordata, and a tendon owns one row of the sparse Jacobian.
        same_slice!(label, "joint data", base_data.joint("knee").unwrap().view(&other_data),
                    other_data.joint("knee").unwrap().view(&other_data), qpos);
        same_slice!(label, "joint data", base_data.joint("root").unwrap().view(&other_data),
                    other_data.joint("root").unwrap().view(&other_data), qvel);
        same_slice!(label, "sensor data", base_data.sensor("se_u1").unwrap().view(&other_data),
                    other_data.sensor("se_u1").unwrap().view(&other_data), data);
        same_slice!(label, "tendon data", base_data.tendon("td").unwrap().view(&other_data),
                    other_data.tendon("td").unwrap().view(&other_data), J);
    }
}

/// An `mjData` built for one model must run against any model the gate accepts.
#[test]
fn test_data_runs_against_every_compatible_model() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);

    // The gate makes its promise over every model it accepts, so the sweep covers that whole set.
    let mut swapped = 0;
    for e in &edits {
        let Some(other) = compile_with(&spec, &[e]) else { continue };
        if !base.is_compatible_with_model(&other) {
            continue;
        }
        let mut data = MjData::new(&base);
        for _ in 0..3 {
            data.step();
        }
        // The Info comes from the model the data leaves behind, so the view after the swap
        // resolves a range that the old model cached.
        let sensor = data.sensor("se_u1").unwrap();
        let returned = data.swap_model(&other);
        assert!(std::ptr::eq(returned, &base), "swap_model returns the old model");
        for _ in 0..3 {
            data.step();
        }
        assert_eq!(sensor.view(&data).data.as_ptr(),
                   data.sensor("se_u1").unwrap().view(&data).data.as_ptr(),
                   "'{}': the cached sensor range moved in the swap", e.label);
        swapped += 1;
    }
    assert!(swapped > 0, "no model reached the swap");
}

/// Every path that can hand out a view must refuse an incompatible model with an error.
#[test]
fn test_every_view_path_refuses_an_incompatible_model() {
    let spec = base_spec();
    let base = base();
    let base_data = base.make_data();
    let info = base.body("trunk").unwrap();
    let joint = base_data.joint("knee").unwrap();
    let (edits, _) = all_edits(&spec);

    let mut checked = 0;
    for e in edits.iter().filter(|e| e.kind == Kind::Structural) {
        let Some(mut other) = compile_with(&spec, &[e]) else { continue };
        let label = &e.label;

        assert!(matches!(info.try_view(&other), Err(MjModelError::IncompatibleModel { .. })),
                "'{label}': try_view must refuse");
        assert!(matches!(info.try_view_mut(&mut other), Err(MjModelError::IncompatibleModel { .. })),
                "'{label}': try_view_mut must refuse");
        assert!(matches!(info.clone().update_layout(&other),
                         Err(MjModelError::IncompatibleModel { .. })),
                "'{label}': update_layout must refuse");
        {
            let mut other_data = other.make_data();
            assert!(matches!(joint.try_view(&other_data),
                             Err(MjDataError::IncompatibleModel { .. })),
                    "'{label}': the data view must refuse");
            assert!(matches!(joint.try_view_mut(&mut other_data),
                             Err(MjDataError::IncompatibleModel { .. })),
                    "'{label}': the mutable data view must refuse");
        }
        assert!(MjData::new(&base).try_swap_model(&other).is_err(),
                "'{label}': swap_model must refuse");

        // The view below panics on every model, so the default hook would print one backtrace
        // notice per variant. The empty hook is process-global and must not span an assertion.
        let previous_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = info.view(&other);
        }));
        std::panic::set_hook(previous_hook);

        assert!(panicked.is_err(), "'{label}': the panicking view must panic");
        checked += 1;
    }
    assert!(checked > 0, "no incompatible model reached the view paths");
}

/// The gate must accept a second build of the same source through the other loader, and
/// `update_layout` must move an `Info` to it.
#[test]
fn test_a_model_is_compatible_with_its_twin() {
    let base = base();
    let twin = MjModel::from_xml_string(BASE_XML).unwrap();
    assert!(base.is_compatible_with_model(&twin));
    // The twin takes the XML path and the base takes the spec path, so this check also holds the
    // two build paths to one structure.
    assert_eq!(first_structural_difference(&base, &twin), None);
    assert!(base.is_asset_compatible_with_model(&twin));
    assert_eq!(base.signature(), twin.signature());

    let mut info = base.body("trunk").unwrap();
    info.update_layout(&twin).unwrap();
    assert_eq!(info.view(&twin).pos.as_ptr(), twin.body("trunk").unwrap().view(&twin).pos.as_ptr());
    // The Info now names the twin's layout, and the base still accepts it because they are equal.
    assert!(info.try_view(&base).is_ok());
}

/// Every ordered pair of the matrix through the asset gate, checked against an asset reference.
/// The asset gate authorises the viewer's uploads, so it may accept a pair the full gate refuses.
#[test]
fn test_the_asset_gate_agrees_with_an_independent_asset_check() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let (models, _) = matrix(&spec, &edits);

    let (mut compatible, mut asset_only) = (0, 0);
    for (label_a, a) in &models {
        for (label_b, b) in &models {
            let gate = a.is_asset_compatible_with_model(b);
            let difference = first_asset_difference(a, b);
            assert_eq!(gate, difference.is_none(),
                       "'{label_a}' against '{label_b}': the asset gate says compatible={gate}, \
                        the reference found {difference:?}");
            compatible += usize::from(gate);
            // The asset gate must be the weaker of the two.
            assert!(gate || !a.is_compatible_with_model(b),
                    "'{label_a}' against '{label_b}': the full gate accepted a pair the asset \
                     gate refused");
            asset_only += usize::from(gate && !a.is_compatible_with_model(b));
        }
    }
    assert!(compatible < models.len() * models.len(), "every pair passed the asset gate");
    assert!(asset_only > 0, "no pair separates the two gates, so the weaker one is untested");
}

/// A model that survives `save_to_buffer` and `from_buffer` must still be compatible with the one
/// it came from. `mj_loadModel` takes every size and every table from the file.
#[test]
fn test_a_saved_and_reloaded_model_stays_compatible() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let (models, _) = matrix(&spec, &edits);
    assert!(models.len() > 1, "the matrix holds no variant");

    for (label, model) in models {
        let mut buffer = vec![0u8; model.size()];
        model.save_to_buffer(&mut buffer).unwrap();
        let reloaded = MjModel::from_buffer(&buffer).unwrap();

        assert!(model.is_compatible_with_model(&reloaded),
                "'{label}' is not compatible with its own saved copy");
        assert!(model.is_asset_compatible_with_model(&reloaded),
                "'{label}' loses its assets in a round trip");
        assert_eq!(first_structural_difference(&model, &reloaded), None,
                   "'{label}' changes structure in a round trip");

        // The reload keeps every cached range, so an Info from the original resolves in the copy.
        // Not every variant keeps the knee, so the lookup may find nothing.
        if let Some(info) = model.joint("knee") {
            assert_eq!(info.view(&reloaded).qpos0.as_ptr(),
                       reloaded.joint("knee").unwrap().view(&reloaded).qpos0.as_ptr(),
                       "'{label}': the joint range moved in a round trip");
        }
    }
}

/* Plain-XML models: the shapes a spec edit cannot reach. */

/// An empty model compares empty count tables. An empty table must not make two models alike.
#[test]
fn test_degenerate_models() {
    let empty = MjModel::from_xml_string("<mujoco/>").unwrap();
    let twin = MjModel::from_xml_string("<mujoco/>").unwrap();
    let one_geom = MjModel::from_xml_string(
        "<mujoco><worldbody><geom size='0.1'/></worldbody></mujoco>").unwrap();
    let one_body = MjModel::from_xml_string(
        "<mujoco><worldbody><body><geom size='0.1'/></body></worldbody></mujoco>").unwrap();
    let one_joint = MjModel::from_xml_string(
        "<mujoco><worldbody><body><joint type='hinge'/><geom size='0.1'/></body></worldbody></mujoco>"
    ).unwrap();

    assert_eq!(empty.nbody(), 1, "the world body is always there");
    assert_eq!(empty.njnt(), 0);
    assert!(empty.is_compatible_with_model(&twin));
    assert!(empty.is_asset_compatible_with_model(&twin));

    // Each step below adds one element, so no pair may pass.
    let ladder = [("empty", &empty), ("one geom", &one_geom), ("one body", &one_body),
                  ("one joint", &one_joint), ("base", &base())];
    for (i, (label_a, a)) in ladder.iter().enumerate() {
        for (label_b, b) in ladder.iter().skip(i + 1) {
            assert!(!a.is_compatible_with_model(b),
                    "'{label_a}' and '{label_b}' differ by an element, so they must not match");
            assert_eq!(a.is_compatible_with_model(b),
                       first_structural_difference(a, b).is_none(),
                       "'{label_a}' against '{label_b}'");
        }
    }

    // An empty model still serves an mjData and refuses a foreign Info.
    let mut data = MjData::new(&empty);
    data.step();
    let trunk = base().body("trunk").unwrap();
    assert!(matches!(trunk.try_view(&empty), Err(MjModelError::IncompatibleModel { .. })),
            "the empty model must refuse an Info that another model built");
}

/// A clone keeps every size and every table, so both gates accept it in either direction.
#[test]
fn test_a_cloned_model_is_compatible() {
    let base = base();
    let clone = base.clone();
    assert!(base.is_compatible_with_model(&clone));
    assert!(clone.is_compatible_with_model(&base));
    assert!(base.is_asset_compatible_with_model(&clone));

    let info = base.body("trunk").unwrap();
    assert_eq!(info.view(&clone).pos.as_ptr(), clone.body("trunk").unwrap().view(&clone).pos.as_ptr());
}

/// Swapping two names moves no memory, so both gates accept the pair. The `Info` caches the id it
/// resolved in its own model, so it then reads the element that now carries the other name.
#[test]
fn test_a_name_permutation_is_accepted_and_keeps_the_cached_id() {
    let model = |first: &str, second: &str| MjModel::from_xml_string(&format!(
        "<mujoco><worldbody>\
         <body name='{first}' pos='1 0 0'><joint type='hinge'/><geom size='0.1'/></body>\
         <body name='{second}' pos='2 0 0'><joint type='hinge'/><geom size='0.1'/></body>\
         </worldbody></mujoco>")).unwrap();

    let straight = model("a", "b");
    let swapped = model("b", "a");
    assert!(straight.is_compatible_with_model(&swapped), "a name moves no memory");

    let info = straight.body("a").unwrap();
    assert_eq!(info.id, 1, "'a' is the first body after the world body");
    assert_eq!(info.view(&swapped).pos.as_ptr(),
               swapped.body("b").unwrap().view(&swapped).pos.as_ptr(),
               "the cached id follows the slot, not the name");
    assert_eq!(*info.view(&swapped).pos, [1.0, 0.0, 0.0], "slot 1 keeps its own position");
}

/// The context gate accepts a model exactly when its texture types match the live context.
#[cfg(feature = "renderer")]
#[test]
fn test_the_context_gate_agrees_with_the_texture_table_of_the_context() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let (models, _) = matrix(&spec, &edits);
    // The renderer keeps a GL context current on this thread, which `MjrContext::new` needs.
    let _renderer = mujoco_rs::renderer::MjRenderer::builder().width(8).height(8).build(&models[0].1)
        .unwrap();
    let (mut compatible, mut type_only, mut count) = (0, 0, 0);
    for (label_a, a) in &models {
        // SAFETY: the renderer above keeps a GL context current on this thread.
        let context = unsafe { MjrContext::new(a) };
        let table = &context.ffi().textureType[..context.ffi().ntexture as usize];
        for (label_b, b) in &models {
            let expected = table.iter().copied().eq(b.tex_type().iter().map(|&t| t as i32));
            let gate = context.is_compatible_with_model(b);
            assert_eq!(gate, expected, "'{label_a}' against '{label_b}': the context gate says \
                                        compatible={gate}, its texture table says {expected}");
            compatible += usize::from(gate);
            type_only += usize::from(!gate && a.ntex() == b.ntex());
            count += usize::from(a.ntex() != b.ntex());
        }
    }
    assert!(compatible > models.len(), "too few compatible pairs: {compatible}");
    assert!(type_only > 0 && count > 0, "no pair differs in the type alone or in the count alone");
}

/* Plugin bindings: a small model of its own, because an SDF mesh makes each compile slow. */

/// Two instances of each first-party plugin kind, each bound once.
const PLUGIN_XML: &str = "\
<mujoco>
<extension>
  <plugin plugin='mujoco.elasticity.cable'>
    <instance name='c0'><config key='twist' value='1'/></instance>
    <instance name='c1'><config key='twist' value='2'/></instance>
  </plugin>
  <plugin plugin='mujoco.sdf.torus'>
    <instance name='t0'><config key='radius1' value='.3'/><config key='radius2' value='.1'/></instance>
    <instance name='t1'><config key='radius1' value='.2'/><config key='radius2' value='.05'/></instance>
  </plugin>
  <plugin plugin='mujoco.pid'>
    <instance name='p0'><config key='kp' value='1'/></instance>
    <instance name='p1'><config key='kp' value='2'/></instance>
  </plugin>
  <plugin plugin='mujoco.sensor.touch_grid'>
    <instance name='g0'><config key='size' value='4 4'/><config key='fov' value='90 90'/>
      <config key='gamma' value='0'/><config key='nchannel' value='1'/></instance>
    <instance name='g1'><config key='size' value='2 4'/><config key='fov' value='90 90'/>
      <config key='gamma' value='0'/><config key='nchannel' value='2'/></instance>
  </plugin>
</extension>
<asset>
  <mesh name='m0'><plugin instance='t0'/></mesh>
  <mesh name='m1'><plugin instance='t1'/></mesh>
</asset>
<worldbody>
  <body name='b0'>
    <plugin instance='c0'/><joint name='j0' type='ball'/><site name='s0'/>
    <geom name='g0' type='sdf' mesh='m0'><plugin instance='t0'/></geom>
    <body name='b1' pos='1 0 0'>
      <plugin instance='c1'/><joint name='j1' type='ball'/>
      <geom name='g1' type='sdf' mesh='m1'><plugin instance='t1'/></geom>
    </body>
  </body>
</worldbody>
<actuator>
  <plugin name='a0' joint='j0' instance='p0' actdim='0'/>
  <plugin name='a1' joint='j1' instance='p1' actdim='0'/>
</actuator>
<sensor>
  <plugin name='s0' instance='g0' objtype='site' objname='s0'/>
  <plugin name='s1' instance='g1' objtype='site' objname='s0'/>
</sensor>
</mujoco>";

#[test]
fn test_a_plugin_rebind_is_rejected() {
    let lib = std::env::var("MUJOCO_DYNAMIC_LINK_DIR").expect("the plugins sit beside the library");
    load_all_plugin_libraries(std::path::Path::new(&lib).parent().unwrap().join("bin/mujoco_plugin"), None)
        .unwrap();
    let base = MjModel::from_xml_string(PLUGIN_XML).unwrap();
    macro_rules! rebind {
        ($table:literal, $finder:ident, $first:literal, $second:literal, $instance:literal) => {{
            // A fresh parse, not a clone: `mj_copySpec` binds each reference to its instance, and
            // a later rename of the reference then has no effect.
            let mut spec = MjSpec::from_xml_string(PLUGIN_XML).unwrap();
            spec.$finder($first).unwrap().plugin_mut().set_name(concat!($instance, "1"));
            spec.$finder($second).unwrap().plugin_mut().set_name(concat!($instance, "0"));
            let other = spec.compile().unwrap();
            assert_eq!(first_structural_difference(&base, &other), Some($table));
            assert!(!base.is_compatible_with_model(&other), "a swap of the {} must be rejected", $table);
        }};
    }
    rebind!("body_plugin",     body_mut,     "b0", "b1", "c");
    rebind!("geom_plugin",     geom_mut,     "g0", "g1", "t");
    rebind!("actuator_plugin", actuator_mut, "a0", "a1", "p");
    rebind!("sensor_plugin",   sensor_mut,   "s0", "s1", "g");

    // The attribute text lives in mjModel only, so a longer spelling of the same gain moves nothing.
    let longer = PLUGIN_XML.replacen("key='kp' value='1'", "key='kp' value='1.000'", 1);
    let longer = MjModel::from_xml_string(&longer).unwrap();
    assert_eq!(first_structural_difference(&base, &longer), None);
    assert!(base.is_compatible_with_model(&longer), "a longer attribute text must stay compatible");

    // Two unbound instances at the end of the list swap their kinds, and no binding moves.
    let unbound = |first: &str, second: &str| MjModel::from_xml_string(&PLUGIN_XML.replace(
        "</extension>",
        &format!("<plugin plugin='{first}'><instance name='u0'/></plugin>\
                  <plugin plugin='{second}'><instance name='u1'/></plugin></extension>"),
    )).unwrap();
    let cable_first = unbound("mujoco.elasticity.cable", "mujoco.sdf.torus");
    let torus_first = unbound("mujoco.sdf.torus", "mujoco.elasticity.cable");
    assert_eq!(first_structural_difference(&cable_first, &torus_first), Some("plugin"));
    assert!(!cable_first.is_compatible_with_model(&torus_first), "a swap of the plugin kinds must be rejected");
}
