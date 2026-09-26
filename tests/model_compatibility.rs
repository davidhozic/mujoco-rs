//! Integration tests for [`MjModel::is_compatible_with_model`],
//! [`MjModel::is_asset_compatible_with_model`], [`MjvScene::is_compatible_with_model`],
//! [`MjvScene::is_compatible_with_scene`] and the `Info` view gate.
//! 
//! These tests require the mujoco-x-y-z/include/mujoco/mjxmacro.h to be available.

use mujoco_rs::error::{MjDataError, MjModelError};
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

fn base() -> MjModel {
    MjModel::from_xml_string(BASE_XML).expect("the base model does not compile")
}

/// Declares [`WRITABLE_TABLES`] and [`reverse_writable_tables`] over the tables that a safe
/// accessor writes.
macro_rules! writable_tables {
    ($($table:ident),*) => { paste::paste! {
        /// Names of the non-float tables that the layout leaves out, as `mjxmacro.h` spells them.
        const WRITABLE_TABLES: &[&str] = &[$(stringify!($table)),*];

        /// Reverses every table in [`WRITABLE_TABLES`]; the body fails to compile when one of the
        /// accessors turns unsafe.
        fn reverse_writable_tables(model: &mut MjModel) {
            $(model.[<$table _mut>]().reverse();)*
        }
    } };
}

writable_tables! {
    body_simple,           body_sameframe,        body_contype,          body_conaffinity,
    jnt_group,             jnt_limited,           jnt_actfrclimited,     jnt_actgravcomp,
    dof_simplenum,         tree_sleep_policy,     geom_contype,          geom_conaffinity,
    geom_group,            geom_priority,         geom_sameframe,        site_type,
    site_group,            site_sameframe,        cam_mode,              cam_projection,
    cam_output,            light_mode,            light_type,            light_castshadow,
    light_active,          flex_contype,          flex_conaffinity,      flex_priority,
    flex_internal,         flex_selfcollide,      flex_activelayers,     flex_passive,
    flex_group,            flex_edgeequality,     flex_rigid,            flexedge_rigid,
    flex_centered,         flex_flatskin,         skin_group,            tex_colorspace,
    tex_data,              mat_texuniform,        pair_signature,        exclude_signature,
    eq_active0,            tendon_group,          tendon_limited,        tendon_actfrclimited,
    actuator_biastype,     actuator_actlimited,   actuator_actearly,     actuator_group,
    actuator_forcelimited, actuator_ctrllimited,  sensor_datatype,       sensor_needstage,
    bvh_depth,             oct_depth,             tuple_objtype,         paths
}

/// Reports whether `field` is a name field, a path address or count, or a hull or tree table, which
/// the layout leaves out.
fn is_left_out_field(field: &str) -> bool {
    field.starts_with("name_") || field.ends_with("_pathadr")
        || matches!(field, "names" | "names_map" | "nnames" | "npaths")
        || matches!(field, "mesh_extrema" | "mesh_polyvert" | "mesh_polymap")
        || matches!(field, "bvh_child" | "bvh_nodeid" | "oct_child")
}

/// Returns the base model XML with the mesh vertices moved, so the compiler builds another hull and tree.
fn moved_mesh_xml() -> String {
    BASE_XML.replace("vertex='0 0 0  .1 0 0  0 .1 0  0 0 .1'", "vertex='0 0 0  .3 0 0  0 .2 0  0 0 .4'")
}

/// Returns the names of the sizes and of the non-float tables that `mjxmacro.h` declares for
/// `mjModel`.
fn header_fields() -> (BTreeSet<String>, BTreeSet<String>) {
    let version = env!("CARGO_PKG_VERSION").split_once("+mj-").unwrap().1;
    let path = format!("{}/mujoco-{version}/include/mujoco/mjxmacro.h", env!("CARGO_MANIFEST_DIR"));
    let header = std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("cannot read {path}: {err}"));
    let section = |start: &str, end: &str| {
        let from = header.find(start).unwrap();
        &header[from..from + header[from..].find(end).unwrap()]
    };
    let entries = |text: &str| -> Vec<Vec<String>> {
        text.lines()
            .filter_map(|line| line.trim().strip_prefix("XNV").or(line.trim().strip_prefix('X')))
            .filter_map(|rest| rest.trim_start().strip_prefix('('))
            .map(|rest| rest.split(')').next().unwrap().split(',').map(|item| item.trim().to_owned()).collect())
            .collect()
    };

    let sizes = entries(section("#define MJMODEL_SIZES", "\n\n")).into_iter()
        .map(|entry| entry[0].clone()).collect();
    let tables = entries(section("#define MJMODEL_POINTERS_BODY", "#define MJDATA_POINTERS")).into_iter()
        .filter(|entry| !matches!(entry[0].as_str(), "mjtNum" | "float"))
        .map(|entry| entry[1].clone()).collect();
    (sizes, tables)
}

/* Tests. */

/// The layout names every size of the header, and every non-float table that no safe accessor
/// writes, apart from the name, path, hull and tree fields.
#[test]
fn test_the_layout_covers_the_header() {
    let (mut sizes, mut tables) = header_fields();
    sizes.retain(|size| !is_left_out_field(size));
    tables.retain(|table| !is_left_out_field(table));
    let names = |list: &[&str]| -> BTreeSet<String> { list.iter().map(|&name| name.to_owned()).collect() };
    let (compared, writable) = (names(MjModelLayout::TABLES), names(WRITABLE_TABLES));

    assert_eq!(names(MjModelLayout::SIZES), sizes);
    assert_eq!(MjModelLayout::SIZES.len(), sizes.len(), "the layout names a size twice");
    assert_eq!(MjModelLayout::TABLES.len(), compared.len(), "the layout names a table twice");
    assert_eq!(compared.intersection(&writable).count(), 0);
    assert_eq!(&compared | &writable, tables);
}

/// A model that differs from the base only in floats, names, the mesh hull and tree, and safely
/// written tables passes every gate.
#[test]
fn test_every_gate_accepts_a_safe_write() {
    let base = base();
    let mut edited = MjModel::from_xml_string(&moved_mesh_xml().replace("trunk", "torso_link")).unwrap();
    reverse_writable_tables(&mut edited);
    edited.geom_size_mut().iter_mut().flatten().for_each(|size| *size *= 2.0);

    assert!(base.is_compatible_with_model(&edited));
    assert!(edited.is_compatible_with_model(&base));
    assert!(base.is_asset_compatible_with_model(&edited));
    assert!(MjvScene::new(&base, 100).is_compatible_with_model(&edited));
    assert!(MjvScene::new(&base, 100).is_compatible_with_scene(&MjvScene::new(&edited, 10)));
    assert!(base.body("trunk").unwrap().try_view(&edited).is_ok());
}

/// A swap installs a compatible model and returns the old one, and data then steps with it.
#[test]
fn test_a_swap_installs_a_compatible_model() {
    let base = base();
    let mut edited = MjModel::from_xml_string(&moved_mesh_xml()).unwrap();
    edited.opt_mut().timestep = 0.004;

    let mut data = MjData::new(&base);
    data.step();
    assert!(std::ptr::eq(data.swap_model(&edited), &base));
    assert_eq!(data.model().opt().timestep, 0.004);
    data.step();
}

/// A model that differs from the base in one compared table fails every gate but the asset gate.
#[test]
fn test_every_gate_refuses_a_compared_table() {
    let base = base();
    let mut other = MjModel::from_xml_string(BASE_XML).unwrap();
    let njnt = other.jnt_type().len();
    // SAFETY: the model only meets the gates, which never follow a table.
    unsafe { other.jnt_type_mut().swap(njnt - 2, njnt - 1) };
    assert_ne!(base.jnt_type(), other.jnt_type(), "the swap changed no joint type");

    assert!(!base.is_compatible_with_model(&other));
    assert!(!other.is_compatible_with_model(&base));
    assert!(base.is_asset_compatible_with_model(&other));
    assert!(!MjvScene::new(&base, 100).is_compatible_with_model(&other));
    assert!(!MjvScene::new(&base, 100).is_compatible_with_scene(&MjvScene::new(&other, 100)));

    let info = base.body("trunk").unwrap();
    assert!(matches!(info.try_view(&other), Err(MjModelError::IncompatibleModel { .. })));
    assert!(matches!(info.try_view_mut(&mut other), Err(MjModelError::IncompatibleModel { .. })));
    assert!(matches!(info.clone().update_layout(&other), Err(MjModelError::IncompatibleModel { .. })));
}

/// A size difference fails the data gates, and the error names both layouts.
#[test]
fn test_the_data_gates_refuse_a_size() {
    let base = base();
    let other = MjModel::from_xml_string(&BASE_XML.replace("nuserdata='16'", "nuserdata='17'")).unwrap();
    assert!(base.is_asset_compatible_with_model(&other), "a data size must not reach the asset gate");

    let base_data = base.make_data();
    let mut other_data = other.make_data();
    let joint = base_data.joint("knee").unwrap();
    assert!(matches!(joint.try_view(&other_data), Err(MjDataError::IncompatibleModel { .. })));
    assert!(matches!(joint.try_view_mut(&mut other_data), Err(MjDataError::IncompatibleModel { .. })));
    assert!(other_data.copy_state_from_data(&base_data, MjtState::mjSTATE_FULLPHYSICS as u32).is_err());
    assert!(base_data.copy_visual_to(&mut other_data).is_err());

    match MjData::new(&base).try_swap_model(&other) {
        Err(MjDataError::IncompatibleModel { source, destination }) => assert_ne!(source, destination),
        _ => panic!("the swap must refuse a different nuserdata"),
    }
}

/// A different mesh fails the asset gate.
#[test]
fn test_the_asset_gate_refuses_a_mesh() {
    let base = base();
    let other = MjModel::from_xml_string(&BASE_XML.replace("0 0 .1' texcoord", "0 0 .1  .1 .1 .1' texcoord")
        .replace("texcoord='0 0  1 0  0 1  1 1'", "texcoord='0 0  1 0  0 1  1 1  0 0'")).unwrap();
    assert!(!base.is_asset_compatible_with_model(&other));
    assert!(!base.is_compatible_with_model(&other));
}

/// A saved and reloaded model passes every gate against the model it came from.
#[test]
fn test_a_saved_and_reloaded_model_stays_compatible() {
    let base = base();
    let mut buffer = vec![0u8; base.size()];
    base.save_to_buffer(&mut buffer).unwrap();
    let reloaded = MjModel::from_buffer(&buffer).unwrap();

    assert!(base.is_compatible_with_model(&reloaded));
    assert!(MjvScene::new(&base, 100).is_compatible_with_model(&reloaded));
    let mut info = base.body("trunk").unwrap();
    info.update_layout(&reloaded).unwrap();
    assert!(info.try_view(&base).is_ok());
}

/// An unchecked swap installs a model that the gate refuses, and returns the old one.
#[test]
fn test_an_unchecked_swap_installs_a_refused_model() {
    let base = base();
    let hinge = BASE_XML.replace("type='slide' axis='1 0 0'", "type='hinge' axis='1 0 0'");
    let other = MjModel::from_xml_string(&hinge).unwrap();
    assert!(!base.is_compatible_with_model(&other));

    let mut data = MjData::new(&base);
    // SAFETY: a hinge and a slide joint own one qpos and one dof each, so every size and every
    // address table of the two models agrees.
    let old = unsafe { data.swap_model_unchecked(&other) };
    assert!(std::ptr::eq(old, &base));
    assert!(std::ptr::eq(data.model(), &other));
    data.step();
}

/// A geom copy between the scenes of two different models panics.
#[cfg(any(feature = "viewer", feature = "renderer"))]
#[test]
#[should_panic(expected = "models that are not compatible")]
fn test_sync_geoms_refuses_a_different_model() {
    let base = base();
    let other = MjModel::from_xml_string(&BASE_XML.replace("nuserdata='16'", "nuserdata='17'")).unwrap();
    let mut source = MjvScene::new(&base, 8);
    let mut destination = MjvScene::new(&other, 8);
    // SAFETY: the test never writes a material, texture or object id.
    unsafe { source.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) };

    let _ = mujoco_rs::vis_common::sync_geoms(&source, &mut destination);
}
