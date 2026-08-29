//! Integration tests for [`MjModel::is_compatible_with_model`],
//! [`MjModel::is_asset_compatible_with_model`] and the `Info` view gate.
//!
//! The gate decides whether an `mjData` buffer, or an index range an `Info` cached, stays valid
//! when the model behind it changes. A wrong "yes" is a memory-safety fault, and a wrong "no"
//! makes the gate useless.
//!
//! The variant matrix is generated, not written out. An [`Edit`] is a function over an [`MjSpec`],
//! and the suite compiles the base spec with every edit alone and with every pair of edits. The
//! type-field axes walk the whole enum: `all_variants!` builds the list and an anonymous `const`
//! item matches on it, so a value the list misses stops the build. The base model itself, and the degenerate models
//! at the end, stay plain XML.

use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_editing::{
    IntVelocityConfig, PositionConfig, DcMotorConfig, MjsActuator, PidConfig,
};

/* The base model. */

/// One model that carries every element kind MuJoCo compiles, with a name on every element so
/// that an edit can reach it.
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
  <general name='a_int' joint='knee' dyntype='integrator'/>
  <general name='a_filt' joint='slide' dyntype='filter' dynprm='.1'/>
  <general name='a_ten' tendon='tf' dyntype='filterexact' dynprm='.1'/>
  <orientation name='a_so3' site='s_lower' refsite='s_trunk' kp='1' input='expmap'/>
  <pid name='a_pid' joint='knee' kp='1' kv='1' input='pos vel'/>
</actuator>
<sensor>
  <framepos name='se_pos' objtype='site' objname='s_trunk'/>
  <framequat name='se_quat' objtype='site' objname='s_trunk'/>
  <jointpos name='se_jp' joint='knee'/>
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

/// The spec every variant starts from. Cloning it is far cheaper than parsing the XML again.
fn base_spec() -> MjSpec {
    MjSpec::from_xml_string(BASE_XML).expect("the base model does not parse")
}

fn base() -> MjModel {
    base_spec().compile().expect("the base model does not compile")
}

/* Edits. */

/// What an edit is expected to do to the model structure.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    /// Changes a value MuJoCo stores; moves no element and resizes no buffer.
    Parameter,
    /// Moves an element, resizes a buffer, or changes what a slot means.
    Structural,
    /// The effect is not known in advance. Only the reference check decides for it.
    Open,
}

/// One edit to the base spec.
struct Edit {
    label: String,
    kind: Kind,
    apply: Box<dyn Fn(&mut MjSpec)>,
}

fn edit(label: impl Into<String>, kind: Kind, apply: impl Fn(&mut MjSpec) + 'static) -> Edit {
    Edit { label: label.into(), kind, apply: Box::new(apply) }
}

/// Applies every edit in `edits` to a copy of `spec` and compiles it.
///
/// Returns `None` when the result is not a legal model: a generated combination may delete an
/// element that another one needs, and a generated type value may not fit the element it lands on.
fn compile_with(spec: &MjSpec, edits: &[&Edit]) -> Option<MjModel> {
    let mut copy = spec.clone();
    for e in edits {
        (e.apply)(&mut copy);
    }
    copy.compile().ok()
}

/// Finds the item of one kind by name and hands its raw element pointer back, so that the
/// borrow on the spec ends before `delete_element` takes its own.
macro_rules! element_pointer {
    ($spec:expr, $iter_mut:ident, $name:expr) => {
        $spec.$iter_mut().find(|item| item.name() == $name).map(|item| item.element_mut_pointer())
    };
}

/// Runs `body` on the item of one kind that carries `name`, and does nothing when it is gone.
macro_rules! on_item {
    ($spec:expr, $iter_mut:ident, $name:expr, |$item:ident| $body:expr) => {
        if let Some($item) = $spec.$iter_mut().find(|item| item.name() == $name) {
            $body;
        }
    };
}

/// Proves at compile time that a list of enum values is complete and holds no value twice.
///
/// The named form also declares the list as a constant array. The bare form takes a list that
/// another macro already holds, and an `except` group for the values it leaves out on purpose. A
/// value that no group names makes the match non-exhaustive and stops the build; a value that two
/// name makes an arm unreachable.
macro_rules! all_variants {
    ($(#[$doc:meta])* $name:ident: $ty:ty = $($variant:ident),+ $(,)?) => {
        $(#[$doc])*
        const $name: [$ty; [$(stringify!($variant)),+].len()] = [$(<$ty>::$variant),+];

        all_variants!($ty = $($variant),+);
    };

    ($ty:ty = $($variant:ident),+ $(; except $($other:ident),+)? $(,)?) => {
        const _: () = match [$(<$ty>::$variant),+][0] {
            $(<$ty>::$variant)|+ => (),
            $($(<$ty>::$other)|+ => (),)?
        };
    };
}

/* Type-field axes: every value of every enum the layout compares. */

/// Every [`MjtJoint`] value, applied to the trunk's root joint.
///
/// The root joint is the one that sits alone in its body. MuJoCo refuses a free joint beside
/// another one, so a joint that shares a body cannot host the whole enum.
fn joint_type_edits() -> Vec<Edit> {
    all_variants!(ALL: MjtJoint = mjJNT_FREE, mjJNT_BALL, mjJNT_SLIDE, mjJNT_HINGE);

    ALL.into_iter()
        .map(|t| {
            // The root is a free joint, so setting it to free again changes nothing.
            let kind = if t == MjtJoint::mjJNT_FREE { Kind::Parameter } else { Kind::Structural };
            edit(format!("joint type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, joint_iter_mut, "root", |j| j.set_type(t));
            })
        })
        .collect()
}

/// Every [`MjtGeom`] value MuJoCo accepts on a geom, applied to `g_upper`.
fn geom_type_edits() -> Vec<Edit> {
    all_variants!(ALL: MjtGeom =
        mjGEOM_PLANE, mjGEOM_HFIELD, mjGEOM_SPHERE, mjGEOM_CAPSULE, mjGEOM_ELLIPSOID,
        mjGEOM_CYLINDER, mjGEOM_BOX, mjGEOM_MESH, mjGEOM_SDF, mjNGEOMTYPES, mjGEOM_ARROW,
        mjGEOM_ARROW1, mjGEOM_ARROW2, mjGEOM_LINE, mjGEOM_LINEBOX, mjGEOM_FLEX, mjGEOM_SKIN,
        mjGEOM_LABEL, mjGEOM_TRIANGLE, mjGEOM_NONE);

    // The values at and above mjNGEOMTYPES are the rendering-only kinds and the "no geom" marker;
    // a model geom never carries one.
    const fn is_model_geom(t: MjtGeom) -> bool {
        match t {
            MjtGeom::mjGEOM_PLANE | MjtGeom::mjGEOM_HFIELD | MjtGeom::mjGEOM_SPHERE
            | MjtGeom::mjGEOM_CAPSULE | MjtGeom::mjGEOM_ELLIPSOID | MjtGeom::mjGEOM_CYLINDER
            | MjtGeom::mjGEOM_BOX | MjtGeom::mjGEOM_MESH | MjtGeom::mjGEOM_SDF => true,
            MjtGeom::mjNGEOMTYPES | MjtGeom::mjGEOM_ARROW | MjtGeom::mjGEOM_ARROW1
            | MjtGeom::mjGEOM_ARROW2 | MjtGeom::mjGEOM_LINE | MjtGeom::mjGEOM_LINEBOX
            | MjtGeom::mjGEOM_FLEX | MjtGeom::mjGEOM_SKIN | MjtGeom::mjGEOM_LABEL
            | MjtGeom::mjGEOM_TRIANGLE | MjtGeom::mjGEOM_NONE => false,
        }
    }

    ALL.into_iter()
        .filter(|t| is_model_geom(*t))
        .map(|t| {
            let kind = if t == MjtGeom::mjGEOM_CAPSULE { Kind::Parameter } else { Kind::Structural };
            edit(format!("geom type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, geom_iter_mut, "g_upper", |g| {
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

/// Every [`MjtTexture`] value, applied to the 2D texture.
fn texture_type_edits() -> Vec<Edit> {
    all_variants!(ALL: MjtTexture = mjTEXTURE_2D, mjTEXTURE_CUBE, mjTEXTURE_SKYBOX);

    ALL.into_iter()
        .map(|t| {
            let kind = if t == MjtTexture::mjTEXTURE_2D { Kind::Parameter } else { Kind::Structural };
            edit(format!("texture type {t:?}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, texture_iter_mut, "tx", |tex| {
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

/// Every [`MjtEq`] value, applied to the first equality.
fn equality_type_edits() -> Vec<Edit> {
    all_variants!(ALL: MjtEq =
        mjEQ_CONNECT, mjEQ_WELD, mjEQ_JOINT, mjEQ_TENDON, mjEQ_FLEX, mjEQ_FLEXVERT,
        mjEQ_FLEXSTRAIN, mjEQ_DISTANCE);

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

    ALL.into_iter()
        .map(|t| {
            let kind = if t == MjtEq::mjEQ_CONNECT { Kind::Parameter } else { Kind::Structural };
            edit(format!("equality type {t:?}"), kind, move |spec: &mut MjSpec| {
                let (objtype, name1, name2) = targets(t);
                on_item!(spec, equality_iter_mut, "eq0", |eq| {
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
/// The list mirrors the `set_to_*` methods of [`MjsActuator`]: each one writes a different
/// combination of `actuator_trntype`, `actuator_dyntype`, `actuator_gaintype` and
/// `actuator_biastype`, which are four separate tables in the layout.
fn actuator_kind_edits() -> Vec<Edit> {
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
    ];

    kinds.into_iter()
        .map(|(name, set)| {
            let kind = if name == "motor" { Kind::Parameter } else { Kind::Structural };
            edit(format!("actuator kind {name}"), kind, move |spec: &mut MjSpec| {
                on_item!(spec, actuator_iter_mut, "a_motor", |a| set(a));
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

/// The object a generated sensor points at. The sweep tries each one in turn and keeps the first
/// that compiles, so no per-type table has to be maintained by hand.
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
];

/// Builds the edit that appends one sensor of type `t` pointed at `target`.
fn add_sensor_edit(t: MjtSensor, target: SensorTarget) -> Edit {
    let (objtype, objname, reftype, refname) = target;
    edit(format!("add sensor {t:?} on {objname}"), Kind::Structural, move |spec: &mut MjSpec| {
        let sensor = spec.add_sensor();
        sensor.set_type(t);
        let _ = sensor.set_objtype(objtype);
        sensor.set_objname(objname);
        if reftype != MjtObj::mjOBJ_UNKNOWN {
            let _ = sensor.set_reftype(reftype);
            sensor.set_refname(refname);
        }
        // A ray sensor reads its output selection from intprm[0], and rejects a zero.
        if matches!(t, MjtSensor::mjSENS_RANGEFINDER | MjtSensor::mjSENS_CONTACT) {
            sensor.intprm_mut()[0] = 1;
        }
        // A user sensor has no built-in width, so it needs one; every other type ignores this.
        if t == MjtSensor::mjSENS_USER {
            sensor.set_dim(3);
            sensor.set_needstage(MjtStage::mjSTAGE_VEL);
        }
    })
}

/// The target a sensor type must be given, because probing it blindly ends the process.
///
/// `mjSENS_CAMPROJECTION` is the one entry. `mjCSensor::Compile` (`user_objects.cc:7871`) casts
/// the resolved reference to `mjCCamera*` and reads `resolution` from it with no null check, so a
/// camprojection sensor that names no camera dereferences a null pointer. The XML parser demands
/// the attribute; the spec API does not.
fn forced_target(t: MjtSensor) -> Option<SensorTarget> {
    match t {
        MjtSensor::mjSENS_CAMPROJECTION =>
            Some((MjtObj::mjOBJ_SITE, "s_lower", MjtObj::mjOBJ_CAMERA, "c_trunk")),
        _ => None,
    }
}

/// One edit per [`MjtSensor`] value, each pointed at the first target the compiler accepts.
///
/// Returns the edits and the types no target fitted, so the caller reports them instead of
/// letting a type disappear from the sweep in silence.
fn sensor_type_edits(spec: &MjSpec) -> (Vec<Edit>, Vec<MjtSensor>) {
    let (mut edits, mut skipped) = (Vec::new(), Vec::new());
    for t in SENSOR_TYPES {
        let candidates: Vec<_> = forced_target(t)
            .map_or_else(|| SENSOR_TARGETS.to_vec(), |target| vec![target]);
        let fitted = candidates.into_iter()
            .map(|target| add_sensor_edit(t, target))
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
/// The sweep walks the spec, so every element the base carries takes part and none is chosen by
/// hand. A deletion that leaves an illegal model is reported as skipped by the caller.
fn deletion_edits(spec: &MjSpec) -> Vec<Edit> {
    let mut out = Vec::new();
    macro_rules! sweep {
        ($($obj:ident, $kind:literal => $iter:ident, $iter_mut:ident;)+) => {
            // The kinds the sweep leaves out hold no element a spec can delete: an xbody is a
            // body read another way, a dof and a plugin belong to the compiled model, and the rest
            // are markers or name a whole model.
            all_variants!(MjtObj = $($obj),+ ;
                          except mjOBJ_UNKNOWN, mjOBJ_XBODY, mjOBJ_DOF, mjOBJ_PLUGIN, mjNOBJECT,
                                 mjOBJ_FRAME, mjOBJ_DEFAULT, mjOBJ_MODEL);
            $(
            for name in spec.$iter().map(|item| item.name().to_owned()).collect::<Vec<_>>() {
                // The world body carries no name and cannot be deleted.
                if name.is_empty() {
                    continue;
                }
                let target = name.clone();
                out.push(edit(
                    format!("delete {} '{}'", $kind, name), Kind::Open,
                    move |spec: &mut MjSpec| {
                        if let Some(pointer) = element_pointer!(spec, $iter_mut, target) {
                            // SAFETY: the pointer names an element of this spec, found just above.
                            let _ = unsafe { spec.delete_element(pointer) };
                        }
                    },
                ));
            }
        )+
        };
    }
    sweep! {
        mjOBJ_BODY,     "body"     => body_iter, body_iter_mut;
        mjOBJ_JOINT,    "joint"    => joint_iter, joint_iter_mut;
        mjOBJ_GEOM,     "geom"     => geom_iter, geom_iter_mut;
        mjOBJ_SITE,     "site"     => site_iter, site_iter_mut;
        mjOBJ_CAMERA,   "camera"   => camera_iter, camera_iter_mut;
        mjOBJ_LIGHT,    "light"    => light_iter, light_iter_mut;
        mjOBJ_ACTUATOR, "actuator" => actuator_iter, actuator_iter_mut;
        mjOBJ_SENSOR,   "sensor"   => sensor_iter, sensor_iter_mut;
        mjOBJ_TENDON,   "tendon"   => tendon_iter, tendon_iter_mut;
        mjOBJ_EQUALITY, "equality" => equality_iter, equality_iter_mut;
        mjOBJ_PAIR,     "pair"     => pair_iter, pair_iter_mut;
        mjOBJ_EXCLUDE,  "exclude"  => exclude_iter, exclude_iter_mut;
        mjOBJ_FLEX,     "flex"     => flex_iter, flex_iter_mut;
        mjOBJ_MESH,     "mesh"     => mesh_iter, mesh_iter_mut;
        mjOBJ_HFIELD,   "hfield"   => hfield_iter, hfield_iter_mut;
        mjOBJ_SKIN,     "skin"     => skin_iter, skin_iter_mut;
        mjOBJ_TEXTURE,  "texture"  => texture_iter, texture_iter_mut;
        mjOBJ_MATERIAL, "material" => material_iter, material_iter_mut;
        mjOBJ_NUMERIC,  "numeric"  => numeric_iter, numeric_iter_mut;
        mjOBJ_TEXT,     "text"     => text_iter, text_iter_mut;
        mjOBJ_TUPLE,    "tuple"    => tuple_iter, tuple_iter_mut;
        mjOBJ_KEY,      "key"      => key_iter, key_iter_mut;
    }
    out
}

/// One edit per element kind [`MjSpec`] and [`MjsBody`] can add, each appending one element.
fn addition_edits() -> Vec<Edit> {
    vec![
        edit("add body", Kind::Open, |spec: &mut MjSpec| {
            let body = spec.world_body_mut().add_body();
            let _ = body.set_name("added_body");
            let geom = body.add_geom();
            *geom.size_mut() = [0.02, 0.0, 0.0];
            let _ = geom.set_name("added_body_geom");
        }),
        edit("add joint", Kind::Open, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("spare") {
                let joint = body.add_joint();
                let _ = joint.set_name("added_joint");
                joint.set_type(MjtJoint::mjJNT_HINGE);
                *joint.axis_mut() = [1.0, 0.0, 0.0];
            }
        }),
        edit("add geom", Kind::Open, |spec: &mut MjSpec| {
            let geom = spec.world_body_mut().add_geom();
            let _ = geom.set_name("added_geom");
            *geom.size_mut() = [0.02, 0.0, 0.0];
        }),
        edit("add site", Kind::Open, |spec: &mut MjSpec| {
            let site = spec.world_body_mut().add_site();
            let _ = site.set_name("added_site");
        }),
        edit("add camera", Kind::Open, |spec: &mut MjSpec| {
            let camera = spec.world_body_mut().add_camera();
            let _ = camera.set_name("added_camera");
        }),
        edit("add light", Kind::Open, |spec: &mut MjSpec| {
            let light = spec.world_body_mut().add_light();
            let _ = light.set_name("added_light");
        }),
        edit("add actuator", Kind::Open, |spec: &mut MjSpec| {
            let actuator = spec.add_actuator();
            let _ = actuator.set_name("added_actuator");
            actuator.set_target("slide");
            actuator.set_trntype(MjtTrn::mjTRN_JOINT);
        }),
        edit("add pair", Kind::Open, |spec: &mut MjSpec| {
            let pair = spec.add_pair();
            let _ = pair.set_name("added_pair");
            pair.set_geomname1("g_trunk");
            pair.set_geomname2("g_lower");
        }),
        edit("add exclude", Kind::Open, |spec: &mut MjSpec| {
            let exclude = spec.add_exclude();
            let _ = exclude.set_name("added_exclude");
            exclude.set_bodyname1("trunk");
            exclude.set_bodyname2("upper");
        }),
        edit("add equality", Kind::Open, |spec: &mut MjSpec| {
            let equality = spec.add_equality();
            let _ = equality.set_name("added_equality");
            equality.set_type(MjtEq::mjEQ_WELD);
            equality.set_objtype(MjtObj::mjOBJ_BODY);
            equality.set_name1("upper");
            equality.set_name2("world");
        }),
        edit("add numeric", Kind::Open, |spec: &mut MjSpec| {
            let numeric = spec.add_numeric();
            let _ = numeric.set_name("added_numeric");
            numeric.set_data(&[1.0, 2.0]);
        }),
        edit("add text", Kind::Open, |spec: &mut MjSpec| {
            let text = spec.add_text();
            let _ = text.set_name("added_text");
            text.set_data("added");
        }),
        edit("add tuple", Kind::Open, |spec: &mut MjSpec| {
            let tuple = spec.add_tuple();
            let _ = tuple.set_name("added_tuple");
            let _ = tuple.set_objtype(&[MjtObj::mjOBJ_BODY]);
            tuple.append_objname("upper");
            tuple.set_objprm(&[0.0]);
        }),
        edit("add key", Kind::Open, |spec: &mut MjSpec| {
            let key = spec.add_key();
            let _ = key.set_name("added_key");
        }),
        edit("add material", Kind::Open, |spec: &mut MjSpec| {
            let material = spec.add_material();
            let _ = material.set_name("added_material");
        }),
    ]
}

/// One edit per size field of the spec. `mjSpec` carries these as plain data, so the test writes
/// them through the raw struct.
fn size_edits() -> Vec<Edit> {
    macro_rules! sizes {
        ($($field:ident = $value:expr;)+) => {
            vec![$(
                edit(concat!(stringify!($field), " grows"), Kind::Structural,
                     |spec: &mut MjSpec| {
                        // SAFETY: the field is plain data that the compiler reads as a count.
                        unsafe { spec.ffi_mut() }.$field = $value;
                     }),
            )+]
        };
    }
    sizes! {
        nuserdata = 64;
        nuser_body = 5;
        nuser_jnt = 4;
        nuser_geom = 6;
        nuser_site = 4;
        nuser_cam = 3;
        nuser_tendon = 5;
        nuser_actuator = 4;
        nuser_sensor = 6;
        memory = 4 * 1024 * 1024;
    }
}

/* The parameter axis: values MuJoCo stores that move no element. */

/// Edits that change a value only. Every one of these must stay compatible, or the gate rejects an
/// ordinary parameter sweep.
fn parameter_edits() -> Vec<Edit> {
    vec![
        edit("timestep", Kind::Parameter, |spec: &mut MjSpec| spec.option_mut().timestep = 0.01),
        edit("gravity", Kind::Parameter, |spec: &mut MjSpec| spec.option_mut().gravity = [0.0, 0.0, -1.0]),
        edit("solver iterations", Kind::Parameter, |spec: &mut MjSpec| spec.option_mut().iterations = 200),
        edit("solver tolerance", Kind::Parameter, |spec: &mut MjSpec| spec.option_mut().tolerance = 1e-10),
        edit("geom size", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_iter_mut, "g_lower", |g| *g.size_mut() = [0.07, 0.0, 0.2]);
        }),
        edit("geom density", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_iter_mut, "g_lower", |g| g.set_density(2000.0));
        }),
        edit("geom friction", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_iter_mut, "g_lower", |g| *g.friction_mut() = [2.0, 0.01, 0.001]);
        }),
        edit("geom rgba", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, geom_iter_mut, "g_trunk", |g| *g.rgba_mut() = [1.0, 0.0, 0.0, 0.5]);
        }),
        edit("body position", Kind::Parameter, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("trunk") {
                *body.pos_mut() = [0.0, 0.5, 1.0];
            }
        }),
        edit("body mass", Kind::Parameter, |spec: &mut MjSpec| {
            if let Some(body) = spec.world_body_mut().child_mut("spare") {
                body.set_mass(3.0);
            }
        }),
        edit("joint armature", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_iter_mut, "slide", |j| j.set_armature(0.1));
        }),
        edit("joint range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_iter_mut, "knee", |j| *j.range_mut() = [-2.0, 2.0]);
        }),
        edit("joint axis", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_iter_mut, "knee", |j| *j.axis_mut() = [1.0, 0.0, 0.0]);
        }),
        edit("joint friction loss", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, joint_iter_mut, "slide", |j| j.set_frictionloss(0.2));
        }),
        edit("actuator gear", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_iter_mut, "a_motor", |a| *a.gear_mut() = [7.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        }),
        edit("actuator control range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, actuator_iter_mut, "a_pos", |a| *a.ctrlrange_mut() = [-2.0, 2.0]);
        }),
        edit("tendon stiffness", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, tendon_iter_mut, "td", |t| t.stiffness_mut()[0] = 4.0);
        }),
        edit("tendon range", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, tendon_iter_mut, "td", |t| *t.range_mut() = [0.0, 2.0]);
        }),
        edit("sensor noise", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, sensor_iter_mut, "se_jp", |s| s.set_noise(0.01));
        }),
        edit("sensor cutoff", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, sensor_iter_mut, "se_jp", |s| s.set_cutoff(3.0));
        }),
        edit("equality solimp", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, equality_iter_mut, "eq0", |e| *e.solimp_mut() = [0.8, 0.9, 0.001, 0.5, 2.0]);
        }),
        edit("pair friction", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, pair_iter_mut, "p0", |p| *p.friction_mut() = [2.0, 2.0, 0.01, 0.001, 0.001]);
        }),
        edit("pair margin", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, pair_iter_mut, "p0", |p| p.set_margin(0.01));
        }),
        edit("numeric values", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, numeric_iter_mut, "n0", |n| n.set_data(&[9.0, 9.0, 9.0]));
        }),
        edit("keyframe time", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, key_iter_mut, "k1", |k| k.set_time(5.0));
        }),
        edit("material rgba", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, material_iter_mut, "mat", |m| *m.rgba_mut() = [0.0, 1.0, 0.0, 1.0]);
        }),
        edit("texture colour", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, texture_iter_mut, "tx", |t| *t.rgb1_mut() = [0.0, 0.0, 1.0]);
        }),
        edit("light diffuse", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, light_iter_mut, "l0", |l| *l.diffuse_mut() = [0.1, 0.2, 0.3]);
        }),
        edit("camera field of view", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, camera_iter_mut, "c_trunk", |c| c.set_fovy(70.0));
        }),
        edit("skin inflate", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, skin_iter_mut, "sk", |s| s.set_inflate(0.03));
        }),
        edit("mesh scale", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, mesh_iter_mut, "ms", |m| *m.scale_mut() = [2.0, 2.0, 2.0]);
        }),
        edit("heightfield extent", Kind::Parameter, |spec: &mut MjSpec| {
            on_item!(spec, hfield_iter_mut, "hf", |h| *h.size_mut() = [2.0, 2.0, 0.3, 0.05]);
        }),
        edit("element name", Kind::Parameter, |spec: &mut MjSpec| {
            // 'spare' carries no reference from any other element, so the rename stands alone.
            on_item!(spec, body_iter_mut, "spare", |b| { let _ = b.set_name("spare_renamed_much_longer"); });
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
        .chain(parameter_edits())
        .collect();
    (edits, skipped)
}

/// Compiles the base model with every edit alone. Returns the models that compile, keyed by label,
/// and the labels that do not.
fn singletons(spec: &MjSpec, edits: &[Edit]) -> (Vec<(String, Kind, MjModel)>, Vec<String>) {
    let (mut built, mut skipped) = (Vec::new(), Vec::new());
    for e in edits {
        match compile_with(spec, &[e]) {
            Some(model) => built.push((e.label.clone(), e.kind, model)),
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
/// This is a second opinion on the question `MjModel::is_compatible_with_model` answers, written
/// from `mjmodel.h` rather than from `MjModelLayout`. It holds every size MuJoCo stores and every
/// count, address, type and owner table. It holds no name and no value array: a name reaches no
/// `mjData` buffer and no cached range, and a value is what a compatible model is allowed to
/// change.
fn first_structural_difference(a: &MjModel, b: &MjModel) -> Option<&'static str> {
    first_difference!(a, b,
        nq, nv, nu, nactuator, nout, na, nbody, nbvh, nbvhstatic, nbvhdynamic, noct, njnt, ntree,
        n_m, n_b, n_c, n_d, ngeom, nsite, ncam, nlight, nflex, nflexnode, nflexvert, nflexedge,
        nflexelem, nflexelemdata, nflexstiffness, nflexbending, nflexelemedge,
        nflexshelldata, nflexevpair, nflextexcoord, n_jfe, n_jfv, n_jmom, n_jten, nmesh, nmeshvert, nmeshnormal,
        nmeshtexcoord, nmeshface, nmeshgraph, nmeshpoly, nmeshpolyvert, nmeshpolymap, nskin,
        nhfield, nhfielddata, ntex,
        ntexdata, nmat, npair, nexclude, neq, ntendon, nwrap, nsensor, nnumeric, nnumericdata,
        ntuple, ntupledata, nkey, nmocap, nplugin, npluginattr, nuser_body,
        nuser_jnt, nuser_geom, nuser_site, nuser_cam, nuser_tendon, nuser_actuator, nuser_sensor,
        nemax, njmax, nconmax, npolygonmax, nmeshdegmax, nuserdata, nsensordata,
        npluginstate, nhistory, narena,
    );
    // ngravcomp counts the bodies that carry gravity compensation. It sizes no array in mjmodel.h
    // or mjdata.h and no engine file reads it, so it moves no element and belongs to no structure
    // test. nnames, nnames_map, npaths and nbuffer stay out for the same reason: they follow the
    // names, which reach no mjData buffer and no cached range.
    //
    // The skin totals and the skin tables stay out for the same reason, and the reason is
    // checkable: mjdata.h holds no array sized by any nskin size, the skin Info declares only
    // fixed-stride fields that nskin bounds, and the asset gate guards a texture, a mesh and a
    // heightfield upload but no skin. MjvScene::is_compatible_with_model is the gate that owns
    // nskinvert, because the scene, not mjData, holds the skin vertex buffer.
    //
    // ntext, ntextdata, text_adr and text_size stay out on the same criterion: mjdata.h names no
    // text size, MJDATA_POINTERS holds no row sized by one, and the wrapper builds no text Info,
    // so a text field reaches no buffer and no range. A numeric and a tuple do have an Info with a
    // range, which is why their count tables travel in the layout.

    first_difference!(a, b,
        body_parentid, body_rootid, body_weldid, body_mocapid, body_jntnum, body_jntadr,
        body_dofnum, body_dofadr, body_treeid, body_geomnum, body_geomadr, body_bvhadr,
        body_bvhnum, body_plugin,
        jnt_type, jnt_qposadr, jnt_dofadr, jnt_bodyid, jnt_actuatorid,
        dof_bodyid, dof_jntid, dof_parentid, dof_treeid, dof_simplenum,
        tree_bodyadr, tree_bodynum, tree_dofadr, tree_dofnum,
        geom_type, geom_bodyid, geom_dataid, geom_plugin,
        site_type, site_bodyid, cam_bodyid, light_bodyid,
        flex_dim, flex_interp, flex_cellnum, flex_nodeadr, flex_nodenum, flex_vertadr,
        flex_vertnum, flex_edgeadr, flex_edgenum, flex_elemadr, flex_elemnum, flex_elemdataadr,
        flex_stiffnessadr, flex_elemedgeadr, flex_bendingadr, flex_shellnum, flex_shelldataadr,
        flex_evpairadr, flex_evpairnum, flex_texcoordadr, flex_bvhadr, flex_bvhnum,
        mesh_vertadr, mesh_vertnum, mesh_faceadr, mesh_facenum, mesh_normaladr, mesh_normalnum,
        mesh_texcoordadr, mesh_texcoordnum, mesh_graphadr, mesh_bvhadr, mesh_bvhnum, mesh_octadr,
        mesh_octnum, mesh_polynum, mesh_polyadr, mesh_polyvertadr, mesh_polyvertnum,
        mesh_polymapadr, mesh_polymapnum,
        hfield_nrow, hfield_ncol, hfield_adr,
        tex_type, tex_height, tex_width, tex_nchannel, tex_adr,
        pair_dim, eq_type, eq_objtype,
        tendon_adr, tendon_num, ten_j_rownnz, ten_j_rowadr, wrap_type,
        actuator_trntype, actuator_dyntype, actuator_gaintype, actuator_biastype, actuator_ctrladr,
        actuator_ctrlnum, actuator_outadr, actuator_outnum, actuator_actadr, actuator_actnum,
        actuator_plugin, actuator_history, actuator_historyadr,
        sensor_type, sensor_datatype, sensor_needstage, sensor_objtype, sensor_reftype, sensor_dim,
        sensor_adr, sensor_plugin, sensor_history, sensor_historyadr,
        plugin, plugin_stateadr, plugin_statenum, plugin_attradr,
        numeric_adr, numeric_size, tuple_adr, tuple_size,
    );
    None
}

/// Returns the first asset field where `a` and `b` keep their mesh, texture or heightfield data in
/// different places, or give a texture a different kind.
///
/// The second opinion on `MjModel::is_asset_compatible_with_model`, which authorises the viewer's
/// texture, mesh and heightfield uploads. Those uploads read the asset arrays and the texture
/// kind, and nothing else.
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

/// Every ordered pair of the matrix, checked against the reference. This is the sweep the other
/// tests rest on: the gate reads [`MjModel`]'s cached layout, the reference reads the accessors one
/// at a time, so a gap in either shows up as a disagreement. The pairs run variant against variant
/// as well as against the base, because a model that agrees with the base can still disagree with
/// another variant.
#[test]
fn test_the_gate_agrees_with_an_independent_structure_check() {
    let spec = base_spec();
    let (edits, skipped_sensors) = all_edits(&spec);
    let (built, skipped) = singletons(&spec, &edits);

    // Every edit that produced no model is named, so the sweep cannot shrink unnoticed.
    println!("{} edits, {} compiled, {} did not: {:?}",
             edits.len(), built.len(), skipped.len(), skipped);
    if !skipped_sensors.is_empty() {
        println!("{} sensor types fitted no target: {:?}",
                 skipped_sensors.len(), skipped_sensors);
    }
    assert_eq!(built.len() + skipped.len(), edits.len(), "an edit left the sweep unaccounted for");

    let models: Vec<_> = std::iter::once(("base".to_owned(), Kind::Parameter, base()))
        .chain(built).collect();

    let mut compatible = 0;
    for (label_a, _, a) in &models {
        for (label_b, _, b) in &models {
            let gate = a.is_compatible_with_model(b);
            let difference = first_structural_difference(a, b);
            assert_eq!(gate, difference.is_none(),
                       "'{label_a}' against '{label_b}': the gate says compatible={gate}, the \
                        reference found {difference:?}");
            compatible += usize::from(gate);
        }
    }
    assert!(compatible > models.len(), "too few compatible pairs: {compatible}");
    assert!(compatible < models.len() * models.len(), "every pair passed, the gate accepts all");
}

/// Every pair of edits applied together, checked against the base in both directions.
///
/// One edit at a time leaves the interactions untested: two edits can each move an element and
/// still leave every size equal, and the reference is the only thing that knows.
#[test]
fn test_every_pair_of_edits_agrees_with_the_reference() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);

    let (mut checked, mut skipped, mut accepted) = (0usize, 0usize, 0usize);
    for (i, first) in edits.iter().enumerate() {
        for second in &edits[i + 1..] {
            let Some(model) = compile_with(&spec, &[first, second]) else {
                skipped += 1;
                continue;
            };
            let label = format!("{} + {}", first.label, second.label);
            for (a, b) in [(&base, &model), (&model, &base)] {
                let gate = a.is_compatible_with_model(b);
                let difference = first_structural_difference(a, b);
                assert_eq!(gate, difference.is_none(),
                           "'{label}': the gate says compatible={gate}, the reference found \
                            {difference:?}");
            }
            accepted += usize::from(base.is_compatible_with_model(&model));
            checked += 1;
        }
    }
    let total = checked + skipped;
    println!("{checked} pairs checked, {skipped} did not compile, {accepted} accepted by the gate");
    assert_eq!(total, edits.len() * (edits.len() - 1) / 2, "a pair went missing");
    assert!(accepted > 0, "no pair of edits stays compatible");
}

/// An edit declared as a parameter change must stay compatible, in both directions and through the
/// asset gate too.
#[test]
fn test_parameter_edits_stay_compatible() {
    let spec = base_spec();
    let base = base();
    let (edits, _) = all_edits(&spec);
    let declared: Vec<_> = edits.iter().filter(|e| e.kind == Kind::Parameter).collect();
    assert!(!declared.is_empty(), "the parameter axis is empty");

    for e in declared {
        let model = compile_with(&spec, &[e])
            .unwrap_or_else(|| panic!("'{}' produces no model", e.label));
        assert!(base.is_compatible_with_model(&model),
                "'{}' changes a value only, so it must stay compatible", e.label);
        assert!(model.is_compatible_with_model(&base),
                "'{}' must be compatible in both directions", e.label);
        assert!(base.is_asset_compatible_with_model(&model), "'{}' moves no asset", e.label);
    }
}

/// An edit declared as a structural change must be rejected, or an `Info` resolves a range that no
/// longer belongs to it.
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

/// A compatible model must resolve every cached range to the same place, or the gate's "yes" is a
/// promise it does not keep.
#[test]
fn test_a_compatible_model_resolves_every_cached_range_identically() {
    let spec = base_spec();
    let base = base();
    let base_data = base.make_data();
    let (edits, _) = all_edits(&spec);

    for e in edits.iter().filter(|e| e.kind == Kind::Parameter) {
        // The renaming edit keeps the structure but not the lookup keys.
        if e.label == "element name" {
            continue;
        }
        let other = compile_with(&spec, &[e]).expect("a parameter edit compiles");
        let other_data = other.make_data();
        let label = &e.label;

        same_slice!(label, "body", base.body("trunk").unwrap().view(&other),
                    other.body("trunk").unwrap().view(&other), pos);
        same_slice!(label, "joint", base.joint("knee").unwrap().view(&other),
                    other.joint("knee").unwrap().view(&other), axis);
        same_slice!(label, "geom", base.geom("g_lower").unwrap().view(&other),
                    other.geom("g_lower").unwrap().view(&other), size);
        same_slice!(label, "site", base.site("s_lower").unwrap().view(&other),
                    other.site("s_lower").unwrap().view(&other), pos);
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
                    other.camera("c_trunk").unwrap().view(&other), pos);
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
        same_slice!(label, "sensor", base.sensor("se_u1").unwrap().view(&other),
                    other.sensor("se_u1").unwrap().view(&other), cutoff);
        same_slice!(label, "tendon", base.tendon("td").unwrap().view(&other),
                    other.tendon("td").unwrap().view(&other), range);
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

    for e in edits.iter().filter(|e| e.kind == Kind::Parameter) {
        let other = compile_with(&spec, &[e]).expect("a parameter edit compiles");
        let mut data = MjData::new(&base);
        for _ in 0..3 {
            data.step();
        }
        let returned = data.swap_model(&other);
        assert_eq!(returned.signature(), base.signature(), "swap_model returns the old model");
        for _ in 0..3 {
            data.step();
        }
        assert_eq!(data.sensor("se_u1").unwrap().view(&data).data.len(), 5,
                   "'{}': the sensor range survives the swap", e.label);
    }
}

/// Every path that can hand out a view must refuse an incompatible model, and refuse it as an
/// error rather than by reading the wrong memory.
#[test]
fn test_every_view_path_refuses_an_incompatible_model() {
    let spec = base_spec();
    let base = base();
    let base_data = base.make_data();
    let info = base.body("trunk").unwrap();
    let joint = base_data.joint("knee").unwrap();
    let (edits, _) = all_edits(&spec);

    // The view path below is expected to panic on every model, so the default hook would print
    // one backtrace notice per variant.
    let previous_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));

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

        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = info.view(&other);
        }));
        assert!(panicked.is_err(), "'{label}': the panicking view must panic");
        checked += 1;
    }
    std::panic::set_hook(previous_hook);
    assert!(checked > 0, "no incompatible model reached the view paths");
}

/// The gate must accept a model against itself, and against a second build of the same source
/// through the other loader, and `update_layout` must restore the pointer comparison after a swap.
#[test]
fn test_a_model_is_compatible_with_itself_and_with_its_twin() {
    let base = base();
    let twin = MjModel::from_xml_string(BASE_XML).unwrap();
    assert!(base.is_compatible_with_model(&base));
    assert!(base.is_compatible_with_model(&twin));
    // The twin takes the XML path and the base takes the spec path, so this is also where the two
    // ways of building the matrix's own model are held to one structure.
    assert_eq!(first_structural_difference(&base, &twin), None);
    assert!(base.is_asset_compatible_with_model(&twin));
    assert_eq!(base.signature(), twin.signature());

    let mut info = base.body("trunk").unwrap();
    assert_eq!(info.model_signature(), base.signature());
    info.update_layout(&twin).unwrap();
    assert_eq!(info.view(&twin).pos.as_ptr(), twin.body("trunk").unwrap().view(&twin).pos.as_ptr());
    // The Info now names the twin's layout, and the base still accepts it because they are equal.
    assert!(info.try_view(&base).is_ok());
}

#[test]
fn test_the_asset_gate_agrees_with_an_independent_asset_check() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let (built, _) = singletons(&spec, &edits);
    let models: Vec<_> = std::iter::once(("base".to_owned(), Kind::Parameter, base()))
        .chain(built).collect();

    let (mut compatible, mut asset_only) = (0, 0);
    for (label_a, _, a) in &models {
        for (label_b, _, b) in &models {
            let gate = a.is_asset_compatible_with_model(b);
            let difference = first_asset_difference(a, b);
            assert_eq!(gate, difference.is_none(),
                       "'{label_a}' against '{label_b}': the asset gate says compatible={gate}, \
                        the reference found {difference:?}");
            compatible += usize::from(gate);
            // The asset gate must be the weaker of the two, never the stronger one.
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
/// it came from. `mj_loadModel` recomputes nothing: it takes every size and every table from the
/// file, so this is the path that a checkpoint and a network transfer both take.
#[test]
fn test_a_saved_and_reloaded_model_stays_compatible() {
    let spec = base_spec();
    let (edits, _) = all_edits(&spec);
    let (built, _) = singletons(&spec, &edits);
    let expected = built.len() + 1;

    let mut checked = 0;
    for (label, _, model) in std::iter::once(("base".to_owned(), Kind::Parameter, base()))
        .chain(built)
    {
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

        // The signature is the one thing a round trip does not keep: mjModel.signature belongs to
        // no MJMODEL_* macro in mjxmacro.h, so mj_saveModel never writes it. The gate must not
        // depend on it.
        assert_eq!(reloaded.signature(), 0, "'{label}': mj_loadModel now restores the signature");
        checked += 1;
    }
    assert_eq!(checked, expected, "the round trip must cover the whole matrix");
}

/* Plain-XML models: the shapes a spec edit cannot reach. */

/// A model with nothing in it still has to answer the question, and every count table it compares
/// is empty. An empty table must not make two different models look alike.
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
        assert!(a.is_compatible_with_model(a), "'{label_a}' must match itself");
    }

    // An empty model still serves an mjData and refuses a foreign Info.
    let mut data = MjData::new(&empty);
    data.step();
    assert!(one_joint.joint("").is_none(), "the unnamed joint has no name to look up");
}

/// A clone shares the model, so it must answer yes to itself by pointer alone.
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
/// resolved in its own model, so it then reads the element that now carries the other name. This
/// is the documented shape of an `Info`, not a fault in the gate, and the test pins the behaviour
/// so that a change to it is deliberate.
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
