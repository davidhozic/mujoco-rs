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
use mujoco_rs::wrappers::{MjModel, MjtObj, MjtGeom, MjtJoint, MjtTrn, MjtSensor};
use mujoco_rs::wrappers::mj_editing::MjtLimited;

fn build_model() -> MjModel {
    let mut spec = MjSpec::new();
    spec.option_mut().timestep = 0.005;
    spec.option_mut().gravity = [0.0, 0.0, -9.81];
    
    let mut world = spec.world_body_mut();
    world.add_light().with_pos([0.0, 0.0, 1.0]);
    world.add_geom()
        .with_name("floor")
        .with_type(MjtGeom::mjGEOM_PLANE)
        .with_size([10.0, 10.0, 1.0])
        .with_rgba([0.8, 0.9, 0.8, 1.0]);
        
    let body1 = world.add_body().with_name("box1").with_pos([0.0, 0.0, 0.5]);
    body1.add_joint().with_name("box1_joint").with_type(MjtJoint::mjJNT_FREE);
    body1.add_geom().with_name("box1_geom").with_type(MjtGeom::mjGEOM_BOX).with_size([0.2, 0.2, 0.2]).with_rgba([1.0, 0.0, 0.0, 1.0]).with_mass(1.0);
    body1.add_site().with_name("box1_site").with_pos([0.0, 0.0, 0.2]);

    let body2 = world.add_body().with_name("box2").with_pos([0.0, 0.0, 1.5]);
    body2.add_joint().with_name("box2_joint").with_type(MjtJoint::mjJNT_SLIDE).with_axis([0.0, 0.0, 1.0]);
    body2.add_geom().with_name("box2_geom").with_type(MjtGeom::mjGEOM_BOX).with_size([0.1, 0.1, 0.1]).with_rgba([0.0, 1.0, 0.0, 1.0]).with_mass(0.5);

    spec.add_actuator()
        .with_name("box2_motor")
        .with_trntype(MjtTrn::mjTRN_JOINT)
        .with_target("box2_joint")
        .with_ctrllimited(MjtLimited::mjLIMITED_TRUE)
        .with_ctrlrange([-10.0, 10.0]);
    
    spec.add_sensor()
        .with_name("box2_pos_sensor")
        .with_type(MjtSensor::mjSENS_JOINTPOS)
        .with_objtype(MjtObj::mjOBJ_JOINT)
        .with_objname("box2_joint");

    spec.compile().expect("Failed to procedurally compile the model")
}

fn main() {
    // ---------------------------------------------------------------------------
    // NATIVE BUMP ALLOCATOR WORKAROUND
    // Miri's `native-lib` linker forces `libmujoco.so` to resolve `malloc` against 
    // `libc`, bypassing Rust exports (no `no_mangle` overrides).
    // It also explicitly rejects calling auxiliary externally-linked C functions.
    // To solve `[noalloc]` UB, we've injected `setup_miri_bump_allocator` directly
    // into the MuJoCo source tree! We allocate a 100MB buffer in Rust (so Miri tracks it),
    // and pass it natively into MuJoCo. All MuJoCo allocations now have Miri provenance!
    // ---------------------------------------------------------------------------
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
    
    // Quick model getters that read native-allocated memory fields.
    assert_eq!(model.nq(), 8); // Free joint (7) + Slide joint (1)
    assert_eq!(model.nv(), 7); // Free joint (6) + Slide joint (1)
    assert_eq!(model.nu(), 1); // 1 motor
    assert_eq!(model.nbody(), 3); // world + box1 + box2
    assert_eq!(model.ngeom(), 3); // floor + box1_geom + box2_geom

    // Names
    let motor_id = model.name_to_id(MjtObj::mjOBJ_ACTUATOR, "box2_motor");
    assert!(motor_id >= 0);
    
    // `id_to_name` uses `CStr::from_ptr` which reads a native-allocated string
    let motor_name = model.id_to_name(MjtObj::mjOBJ_ACTUATOR, motor_id).unwrap();
    assert_eq!(motor_name, "box2_motor");
    
    // Create data
    println!("Creating data...");
    let mut data = model.make_data();
    
    // Test access to array slices
    let qpos0 = model.qpos0();
    assert_eq!(qpos0.len(), 8);
    
    // Test simulation stepping
    println!("Stepping simulation...");
    for _ in 0..10 {
        // Apply control 
        data.ctrl_mut()[motor_id as usize] = 1.0;
        data.step();
    }
    
    // Test sensors
    println!("Testing sensors...");
    data.sensor_pos();
    let sensor_id = model.name_to_id(MjtObj::mjOBJ_SENSOR, "box2_pos_sensor") as usize;
    
    let sensor_data = data.read_sensor(sensor_id, 0.0, 0).expect("Failed to read sensor");
    assert!(!sensor_data.is_empty());
    
    // Test Jacobians
    println!("Testing Jacobians...");
    data.kinematics();
    data.com_pos();
    
    let site_id = model.name_to_id(MjtObj::mjOBJ_SITE, "box1_site");
    let box1_body_id = model.name_to_id(MjtObj::mjOBJ_BODY, "box1");
    
    let (jacp, jacr) = data.jac_site(true, true, site_id);
    assert_eq!(jacp.len(), 3 * model.nv() as usize);
    assert_eq!(jacr.len(), 3 * model.nv() as usize);
    
    let (jacp_body, _) = data.jac_body_com(true, false, box1_body_id);
    assert_eq!(jacp_body.len(), 3 * model.nv() as usize);

    // Test Object Queries
    println!("Testing object velocity/acceleration...");
    let vel = data.object_velocity(MjtObj::mjOBJ_BODY, box1_body_id, false);
    assert_eq!(vel.len(), 6);
    
    data.fwd_position();
    data.fwd_velocity();
    data.fwd_actuation();
    data.fwd_acceleration();
    let acc = data.object_acceleration(MjtObj::mjOBJ_BODY, box1_body_id, false);
    assert_eq!(acc.len(), 6);
    
    let am_mat = data.angmom_mat(box1_body_id); // Internally reads `nv`
    assert_eq!(am_mat.len(), 3 * model.nv() as usize);
    
    // Collision checking
    println!("Testing collisions...");
    data.collision();
    
    let contacts = data.contacts();
    println!("Detected {} contacts.", contacts.len());
    if !contacts.is_empty() {
        let force = data.contact_force(0);
        assert_eq!(force.len(), 6);
    }
    
    // Test saving an XML to buffer
    println!("Testing XML/Model saving...");
    let _ = model.save_last_xml("test_last.xml"); 
    
    println!("Comprehensive test completed successfully!");

    // Drop MuJoCo structures *before* deallocating their backing memory buffer!
    drop(data);
    drop(model);
    
    // ensure bump buffer lives until the very end
    unsafe {
        std::alloc::dealloc(bump_buffer, bump_layout);
    }
}
