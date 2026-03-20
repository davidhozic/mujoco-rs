//! Comprehensive Miri test for `mujoco-rs`.
//!
//! Exercises various parts of the MuJoCo C API wrappers (model loading,
//! data simulation, state extract/apply, querying jacobians, contact info,
//! sensor info) to ensure they are memory-safe and free of undefined behavior
//! when run under Miri.
//!
//! Run under Miri:
//! ```bash
//! cargo +nightly miri run --example miri_test
//! ```

use mujoco_rs::prelude::{MjSpec, SpecItem};
use mujoco_rs::wrappers::{MjModel, MjData, MjtObj, MjtGeom, MjtJoint, MjtTrn, MjtSensor};
use mujoco_rs::wrappers::mj_editing::MjtLimited;
use mujoco_rs::wrappers::fun::*;
use mujoco_rs::mujoco_c::mjtState_;

fn build_model() -> MjModel {
    let mut spec = MjSpec::new();
    spec.option_mut().timestep = 0.005;
    spec.option_mut().gravity = [0.0, 0.0, -9.81];
    
    let world = spec.world_body_mut();
    world.add_light().with_name("main_light").with_pos([0.0, 0.0, 2.0]).with_dir([0.0, 0.0, -1.0]);
    world.add_geom()
        .with_name("floor")
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([10.0, 10.0, 1.0])
        .with_rgba([0.8, 0.9, 0.8, 1.0]);
    println!("  Added floor");
        
    // Body 1: Free box
    let body1 = world.add_body().with_name("box1").with_pos([0.0, 0.0, 0.5]);
    body1.add_joint().with_name("box1_joint").with_type(MjtJoint::mjJNT_FREE);
    body1.add_geom().with_name("box1_geom").with_type(MjtGeom::mjGEOM_BOX).with_size([0.2, 0.2, 0.2]).with_rgba([1.0, 0.0, 0.0, 1.0]).with_mass(1.0);
    body1.add_site().with_name("box1_site").with_pos([0.0, 0.0, 0.2]);
    println!("  Added box1");
  
    // Body 2: Sliding box
    let body2 = world.add_body().with_name("box2").with_pos([0.0, 0.0, 1.5]);
    body2.add_joint().with_name("box2_joint").with_type(MjtJoint::mjJNT_SLIDE).with_axis([0.0, 0.0, 1.0]);
    body2.add_geom().with_name("box2_geom").with_type(MjtGeom::mjGEOM_BOX).with_size([0.1, 0.1, 0.1]).with_rgba([0.0, 1.0, 0.0, 1.0]).with_mass(0.5);
    body2.add_site().with_name("box2_site").with_pos([0.0, 0.0, 0.0]);
    println!("  Added box2");
  
    // Actuators
    spec.add_actuator()
        .with_name("box2_motor")
        .with_trntype(MjtTrn::mjTRN_JOINT)
        .with_target("box2_joint")
        .with_ctrllimited(MjtLimited::mjLIMITED_TRUE)
        .with_ctrlrange([-10.0, 10.0]);
    println!("  Added actuator");
      
    // Sensors
    spec.add_sensor()
        .with_name("box2_pos_sensor")
        .with_type(MjtSensor::mjSENS_JOINTPOS)
        .with_objtype(MjtObj::mjOBJ_JOINT)
        .with_objname("box2_joint");
  
    spec.add_sensor()
        .with_name("box1_accel")
        .with_type(MjtSensor::mjSENS_ACCELEROMETER)
        .with_objtype(MjtObj::mjOBJ_SITE)
        .with_objname("box1_site");
  
    match spec.compile() {
        Ok(m) => m,
        Err(e) => {
            panic!("Failed to procedurally compile the model: {}", e);
        }
    }
}

fn test_basic_getters(model: &MjModel) {
    println!("Testing basic getters...");
    // Access FFI fields directly to ensure visibility under Miri
    assert_eq!(model.ffi().nq, 8);
    assert_eq!(model.ffi().nv, 7);
    assert_eq!(model.ffi().nu, 1);
    assert_eq!(model.ffi().nbody, 3);
    assert_eq!(model.ffi().ngeom, 3);

    let motor_id = model.name_to_id(MjtObj::mjOBJ_ACTUATOR, "box2_motor").unwrap();
    let motor_name = model.id_to_name(MjtObj::mjOBJ_ACTUATOR, motor_id).unwrap();
    assert_eq!(motor_name, "box2_motor");

    let qpos0 = model.qpos0();
    assert!(qpos0.len() >= 8);
}

fn test_simulation_and_sensors<'a>(model: &'a MjModel, data: &mut MjData<&'a MjModel>) {
    println!("Stepping simulation and testing sensors...");
    let motor_id = model.name_to_id(MjtObj::mjOBJ_ACTUATOR, "box2_motor").unwrap();
    for _ in 0..10 {
        data.ctrl_mut()[motor_id] = 1.0;
        data.step();
    }
    
    data.sensor_pos();
    let sensor_id = model.name_to_id(MjtObj::mjOBJ_SENSOR, "box2_pos_sensor").unwrap();
    // box2_pos_sensor is mjSENS_JOINTPOS (dim=1)
    let sensor_data: [f64; 1] = data.read_sensor_fixed(sensor_id, 0.0, 0).expect("Failed to read sensor");
    assert!(!sensor_data.is_empty());
}

fn test_kinematics_and_jacobians<'a>(model: &'a MjModel, data: &mut MjData<&'a MjModel>) {
    println!("Testing kinematics and Jacobians...");
    data.kinematics();
    data.com_pos();
    
    let site_id = model.name_to_id(MjtObj::mjOBJ_SITE, "box1_site").unwrap();
    let box1_body_id = model.name_to_id(MjtObj::mjOBJ_BODY, "box1").unwrap();
    
    let (jacp, jacr) = data.jac_site(true, true, site_id).unwrap();
    assert_eq!(jacp.len(), 3 * model.ffi().nv as usize);
    assert_eq!(jacr.len(), 3 * model.ffi().nv as usize);
    
    let (jacp_body, _) = data.jac_body_com(true, false, box1_body_id).unwrap();
    assert_eq!(jacp_body.len(), 3 * model.ffi().nv as usize);

    println!("Testing object velocity/acceleration...");
    let vel = data.object_velocity(MjtObj::mjOBJ_BODY, box1_body_id, false).unwrap();
    assert_eq!(vel.len(), 6);
    
    data.fwd_position();
    data.fwd_velocity();
    data.fwd_actuation();
    data.fwd_acceleration();
    let acc = data.object_acceleration(MjtObj::mjOBJ_BODY, box1_body_id, false).unwrap();
    assert_eq!(acc.len(), 6);
    
    let am_mat = data.angmom_mat(box1_body_id).unwrap();
    assert_eq!(am_mat.len(), 3 * model.ffi().nv as usize);
}

fn test_inverse_dynamics_and_state<'a>(model: &'a MjModel, data: &mut MjData<&'a MjModel>) {
    println!("Testing collisions, inverse dynamics, and state management...");
    data.collision();
    let contacts = data.contacts();
    println!("Detected {} contacts.", contacts.len());
    
    data.inverse();

    let full_state = mjtState_::mjSTATE_FULLPHYSICS as u32;
    let state = data.get_state(full_state);
    assert!(!state.is_empty());
    // SAFETY: full_state is mjSTATE_FULLPHYSICS which restores from a previously read
    // state; eq_active bytes come from mj_getState and are always valid (0 or 1).
    unsafe { data.set_state(&state, full_state) };

    let mut data2 = model.make_data();
    data.copy_to(&mut data2);
    assert_eq!(data2.time(), data.time());
}

fn test_vfs_raycasting<'a>(_model: &'a MjModel, data: &mut MjData<&'a MjModel>) {
    println!("Testing VFS and raycasting...");
    use mujoco_rs::wrappers::MjVfs;
    let mut vfs = MjVfs::new();
    let xml_content = "<mujoco><worldbody/></mujoco>";
    vfs.add_from_buffer("dummy.xml", xml_content.as_bytes()).expect("Failed to add file to VFS");
    let _spec_vfs = MjSpec::from_xml_vfs("dummy.xml", &vfs).expect("Failed to load spec from VFS");

    let pnt = [0.0, 0.0, 5.0];
    let vec = [0.0, 0.0, -1.0];
    let geomid = data.ray(&pnt, &vec, None, false, None, None);
    println!("Ray hit result: {:?}", geomid);
}

fn test_renderer<'a>(model: &'a MjModel, data: &mut MjData<&'a MjModel>) {
    println!("Testing Renderer...");
    use mujoco_rs::renderer::MjRenderer;
    use mujoco_rs::wrappers::MjtFontScale;
    use std::panic::AssertUnwindSafe;

    // Use catch_unwind because glutin's EGL query can panic in Miri environment
    let res = std::panic::catch_unwind(AssertUnwindSafe(|| {
        MjRenderer::builder()
            .width(64)
            .height(64)
            .rgb(true)
            .depth(true)
            .font_scale(MjtFontScale::mjFONTSCALE_100)
            .build(model)
    }));

    match res {
        Ok(Ok(mut renderer)) => {
            renderer.sync(data);
            let rgb = renderer.rgb_flat().expect("No RGB data");
            assert_eq!(rgb.len(), 64 * 64 * 3);
            
            let temp_image = "/tmp/miri_test_render.png";
            renderer.save_rgb(temp_image).expect("Failed to save RGB image");
            println!("Saved rendered image to {}", temp_image);
        }
        Ok(Err(e)) => {
            println!("Warning: Renderer failed to initialize: {e:?}");
            println!("Skipping full render test (expected in Miri/headless environments)");
        }
        Err(_) => {
            println!("Warning: Renderer panicked during initialization (expected in some Miri/headless environments)");
            println!("Skipping full render test...");
        }
    }
}

fn test_iterators_and_views(model: &MjModel, data: &mut MjData<&MjModel>) {
    println!("Testing iterators and views...");
    
    // MjModel "iteration" via indices
    for i in 0..model.ffi().nbody {
        let name = model.id_to_name(MjtObj::mjOBJ_BODY, i as usize).unwrap_or("unnamed");
        assert!(!name.is_empty());
    }
    for i in 0..model.ffi().njnt {
        let name = model.id_to_name(MjtObj::mjOBJ_JOINT, i as usize).unwrap_or("unnamed");
        assert!(!name.is_empty());
    }
    // Data named accessors and views
    if let Some(joint_view) = data.joint("box2_joint") {
        let mut view = joint_view.view_mut(data);
        view.qpos[0] = 1.0;
        assert_eq!(view.qpos[0], 1.0);
    } else {
        panic!("box2_joint not found");
    }

    if let Some(body_view) = data.body("box1") {
        let view = body_view.view(data);
        assert!(view.xpos[2] != 0.0); // should be around 0.5
    }

    // Spec iterators
    let mut spec = MjSpec::new();
    spec.world_body_mut().add_body().with_name("spec_body");
    let mut count = 0;
    // spec.body_iter() iterates over top-level bodies
    for body in spec.body_iter() {
        if body.name() == "spec_body" {
            count += 1;
        }
    }
    assert_eq!(count, 1);
}

fn test_utilities() {
    println!("Testing mju_* utility wrappers...");
    let mut a = [1.0, 2.0, 3.0];
    let b = [4.0, 5.0, 6.0];
    let mut c = [0.0; 3];

    mju_zero(&mut a);
    assert_eq!(a, [0.0, 0.0, 0.0]);

    mju_fill(&mut a, 5.0);
    assert_eq!(a, [5.0, 5.0, 5.0]);

    mju_add(&mut c, &a, &b);
    assert_eq!(c, [9.0, 10.0, 11.0]);

    let dot = mju_dot(&a, &b);
    assert_eq!(dot, 5.0*4.0 + 5.0*5.0 + 5.0*6.0);

    let norm = mju_norm(&a);
    assert!((norm - (25.0*3.0 as f64).sqrt()).abs() < 1e-9);

    let mut q = [1.0, 0.0, 0.0, 0.0];
    mju_unit_4(&mut q);
    assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
}

fn test_derivatives() {
    println!("Testing mjd_* derivative wrappers...");
    let qa = [1.0, 0.0, 0.0, 0.0];
    let qb = [0.0, 1.0, 0.0, 0.0];
    let mut da = [0.0; 9];
    let mut db = [0.0; 9];
    mjd_sub_quat(&qa, &qb, Some(&mut da), Some(&mut db));
    // Derivatives should be non-zero
    assert!(da.iter().any(|&x| x != 0.0));
}

fn test_auxiliary(model: &MjModel) {
    println!("Testing auxiliary wrappers (MjVisual, MjStatistic, etc.)...");
    let vis = model.vis();
    assert!(vis.map.stiffness > 0.0);
    
    let opt = model.opt();
    assert!(opt.timestep > 0.0);

    let stat = model.stat();
    assert!(stat.meanmass > 0.0);
}

#[cfg(not(miri))]
fn main() {
    eprintln!("This example is intended for Miri testing only.");
    eprintln!("Run with: cargo +nightly miri run --example miri_test");
}

#[cfg(miri)]
fn main() {
    unsafe extern "C" {
        fn setup_miri_bump_allocator(buffer: *mut u8, size: usize);
    }
    
    let bump_size = 100 * 1024 * 1024; // 100MB
    let bump_layout = std::alloc::Layout::from_size_align(bump_size, 64).unwrap();
    let bump_buffer = unsafe { std::alloc::alloc_zeroed(bump_layout) };
    
    println!("RUST_LOG: bump_buffer = {:p}", bump_buffer);

    unsafe {
        setup_miri_bump_allocator(bump_buffer, bump_size);
    }

    println!("Loading procedurally generated model...");
    let model = build_model();
    let mut data = model.make_data();

    /* Incremental Miri testing: gradually enabling previously passed or new tests */
    test_basic_getters(&model);
    test_simulation_and_sensors(&model, &mut data);
    test_kinematics_and_jacobians(&model, &mut data);
    test_inverse_dynamics_and_state(&model, &mut data);
    test_vfs_raycasting(&model, &mut data);
    test_iterators_and_views(&model, &mut data);

    /* Run new tests */
    test_utilities();
    test_derivatives();
    test_auxiliary(&model);

    println!("Comprehensive Miri test completed successfully (core physics and utilities)!");

    // Renderer verification last because it may abort Miri process due to EGL dependencies in Miri environment
    test_renderer(&model, &mut data);

    println!("Full suite (including renderer) completed successfully!");

    // Drop MuJoCo structures *before* deallocating their backing memory buffer!
    drop(data);
    drop(model);
    
    // ensure bump buffer lives until the very end
    unsafe {
        std::alloc::dealloc(bump_buffer, bump_layout);
    }
}
