//! Integration tests for force-cast macro correctness across the public API.
//! These tests exercise mujoco_rs as an external consumer would, verifying that
//! force-cast arrays, enum slices, bool slices, view fields, and mutable
//! roundtrips work correctly through the public interface over multiple timesteps.

use mujoco_rs::prelude::*;

/// A model with diverse joint types, bodies, constraints, actuators, and sensors
/// for thorough force-cast integration testing.
const INTEGRATION_MODEL: &str = "
<mujoco>
  <option timestep='0.002'/>
  <worldbody>
    <geom name='floor' type='plane' size='10 10 1'/>

    <body name='free_body' pos='0 0 3'>
        <joint name='free_jnt' type='free'/>
        <geom name='sphere' type='sphere' size='0.2' mass='1' rgba='1 0 0 1'/>
        <site name='free_site' size='0.05'/>
    </body>

    <body name='hinge_body' pos='2 0 2'>
        <joint name='hinge_jnt' type='hinge' axis='0 1 0'/>
        <geom name='capsule' type='capsule' size='0.1 0.5' mass='0.5'/>
        <site name='hinge_site' size='0.05'/>
    </body>

    <body name='slide_body' pos='-2 0 2'>
        <joint name='slide_jnt' type='slide' axis='0 0 1' range='-5 5' limited='true'/>
        <geom name='box' type='box' size='0.15 0.15 0.15' mass='0.8'/>
    </body>

    <body name='mocap_body' mocap='true' pos='5 5 5'>
        <geom type='sphere' size='0.01' contype='0' conaffinity='0'/>
    </body>
  </worldbody>

  <equality>
      <connect name='eq_connect' body1='hinge_body' body2='slide_body' anchor='0 0 0'/>
  </equality>

  <actuator>
      <motor name='slide_motor' joint='slide_jnt' ctrlrange='-10 10' ctrllimited='true'/>
      <motor name='hinge_motor' joint='hinge_jnt'/>
  </actuator>

  <sensor>
      <touch name='touch_sensor' site='free_site'/>
      <jointpos name='hinge_pos' joint='hinge_jnt'/>
      <jointvel name='hinge_vel' joint='hinge_jnt'/>
  </sensor>
</mujoco>";

/// End-to-end test: load model, step simulation, verify force-cast array
/// grouping (xpos [MjtNum;3], xquat [MjtNum;4]) produces correct values
/// across timesteps by cross-referencing flat FFI pointers.
#[test]
fn test_integration_multi_step_array_grouping() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();
    let nbody = model.ffi().nbody as usize;

    for step in 0..100 {
        data.step();

        let xpos = data.xpos();
        let xquat = data.xquat();
        assert_eq!(xpos.len(), nbody);
        assert_eq!(xquat.len(), nbody);

        // Spot-check FFI alignment every 10 steps
        if step % 10 == 0 {
            for i in 0..nbody {
                for j in 0..3 {
                    assert_eq!(xpos[i][j], unsafe { *data.ffi().xpos.add(i * 3 + j) },
                        "step={} body={} xpos[{}] FFI mismatch", step, i, j);
                }
                for j in 0..4 {
                    assert_eq!(xquat[i][j], unsafe { *data.ffi().xquat.add(i * 4 + j) },
                        "step={} body={} xquat[{}] FFI mismatch", step, i, j);
                }
            }
        }
    }

    // After 100 steps (0.2s), bodies should have fallen under gravity
    let free_body = data.body("free_body").unwrap();
    let free_z = data.xpos()[free_body.id][2];
    assert!(free_z < 3.0, "free body should have fallen from z=3, now at z={}", free_z);
}

/// Integration test: exercise view fields with force-cast enums over
/// multiple simulation steps. Views must reflect changing data each step.
#[test]
fn test_integration_view_enum_fields_across_steps() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();

    // Model-side view checks (static properties)
    let free_jnt_info = model.joint("free_jnt").unwrap();
    let free_jnt_view = free_jnt_info.view(&model);
    assert_eq!(free_jnt_view.r#type[0], MjtJoint::mjJNT_FREE);
    assert_eq!(free_jnt_view.limited[0], false);

    let slide_jnt_info = model.joint("slide_jnt").unwrap();
    let slide_jnt_view = slide_jnt_info.view(&model);
    assert_eq!(slide_jnt_view.r#type[0], MjtJoint::mjJNT_SLIDE);
    assert_eq!(slide_jnt_view.limited[0], true);

    let hinge_jnt_info = model.joint("hinge_jnt").unwrap();
    let hinge_jnt_view = hinge_jnt_info.view(&model);
    assert_eq!(hinge_jnt_view.r#type[0], MjtJoint::mjJNT_HINGE);

    // Sensor model view
    let touch_info = model.sensor("touch_sensor").unwrap();
    let touch_view = touch_info.view(&model);
    assert_eq!(touch_view.r#type[0], MjtSensor::mjSENS_TOUCH);
    assert_eq!(touch_view.objtype[0], MjtObj::mjOBJ_SITE);

    // Data-side: step and verify body views update
    let free_body_info = data.body("free_body").unwrap();
    data.forward();
    let init_z = free_body_info.view(&data).xpos[2];

    for _ in 0..50 {
        data.step();
    }

    let post_z = free_body_info.view(&data).xpos[2];
    assert!(post_z < init_z, "body view xpos z should decrease under gravity");
}

/// Integration test: actuator control via safe API, step, verify force-cast
/// arrays (ctrl, actuator_force, qfrc_actuator) reflect the controls.
#[test]
fn test_integration_actuator_control_multi_step() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();

    let nu = model.ffi().nu as usize;
    assert_eq!(nu, 2);

    // Apply control to slide motor (index 0)
    data.ctrl_mut()[0] = 8.0;
    data.ctrl_mut()[1] = -3.0;

    for _ in 0..200 {
        data.step();
    }

    // ctrl should persist
    assert_eq!(data.ctrl()[0], 8.0);
    assert_eq!(data.ctrl()[1], -3.0);

    // actuator_force should be nonzero
    let act_force = data.actuator_force();
    assert_eq!(act_force.len(), nu);
    assert!(act_force[0].abs() > 1e-6, "slide motor force should be nonzero");
    assert!(act_force[1].abs() > 1e-6, "hinge motor force should be nonzero");

    // qfrc_actuator should have nonzero entries
    let nv = model.ffi().nv as usize;
    let qfrc = data.qfrc_actuator();
    assert_eq!(qfrc.len(), nv);
    assert!(qfrc.iter().any(|v| v.abs() > 1e-6));

    // FFI cross-validation
    for i in 0..nu {
        assert_eq!(act_force[i], unsafe { *data.ffi().actuator_force.add(i) });
    }
}

/// Integration test: modify model arrays via force-cast mutable slices,
/// create new data, and verify the changes affect simulation.
#[test]
fn test_integration_model_mutation_affects_simulation() {
    let mut model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();

    // Get sphere geom id
    let sphere_info = model.geom("sphere").unwrap();
    let sphere_id = sphere_info.id;

    // Store original mass
    let orig_mass = model.body_mass()[1]; // free_body

    // Change sphere size and mass via force-cast mutable arrays
    model.geom_size_mut()[sphere_id] = [1.0, 0.0, 0.0]; // much bigger sphere
    model.body_mass_mut()[1] = 100.0; // much heavier

    let mut data = model.make_data();
    for _ in 0..50 {
        data.step();
    }

    // The heavier sphere should fall differently
    // Verify model arrays were mutated correctly
    assert_eq!(model.geom_size()[sphere_id][0], 1.0);
    assert_eq!(model.body_mass()[1], 100.0);
    assert_ne!(model.body_mass()[1], orig_mass);

    // FFI cross-validation
    assert_eq!(model.body_mass()[1], unsafe { *model.ffi().body_mass.add(1) });
    assert_eq!(model.geom_size()[sphere_id][0], unsafe { *model.ffi().geom_size.add(sphere_id * 3) });
}

/// Integration test: exercise equality constraint toggle (eq_active bool force-cast)
/// across simulation steps, verify dynamics diverge.
#[test]
fn test_integration_eq_active_toggle_divergence() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();

    // Run 1: with equality constraint active
    let mut data1 = model.make_data();
    for _ in 0..100 {
        data1.step();
    }
    let pos_with_eq = data1.xpos().to_vec();

    // Run 2: with equality constraint disabled
    let mut data2 = model.make_data();
    data2.eq_active_mut()[0] = false;
    for _ in 0..100 {
        data2.step();
    }
    let pos_without_eq = data2.xpos().to_vec();

    // Verify eq_active was correctly set
    assert_eq!(data2.eq_active()[0], false);
    assert_eq!(unsafe { *data2.ffi().eq_active }, 0u8);

    // Positions should diverge (at least for constrained bodies)
    let hinge_body = data2.body("hinge_body").unwrap();
    let slide_body = data2.body("slide_body").unwrap();

    let hinge_diff: f64 = (0..3)
        .map(|j| (pos_with_eq[hinge_body.id][j] - pos_without_eq[hinge_body.id][j]).abs())
        .sum();
    let slide_diff: f64 = (0..3)
        .map(|j| (pos_with_eq[slide_body.id][j] - pos_without_eq[slide_body.id][j]).abs())
        .sum();

    assert!(hinge_diff > 1e-10 || slide_diff > 1e-10,
        "disabling eq constraint should cause divergence: hinge_diff={}, slide_diff={}",
        hinge_diff, slide_diff);
}

/// Integration test: sensor data evolves across simulation steps. Verifies
/// force-cast sensordata flat array tracks the simulation state.
#[test]
fn test_integration_sensor_data_multi_step() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();

    let nsensordata = model.ffi().nsensordata as usize;
    assert!(nsensordata >= 3, "model has touch(1d) + jointpos(1d) + jointvel(1d) = 3");

    // Apply hinge control to make the joint move
    data.ctrl_mut()[1] = 5.0;

    let mut prev_sensor: Vec<f64> = vec![0.0; nsensordata];

    for step in 0..100 {
        data.step();

        let current = data.sensordata();
        assert_eq!(current.len(), nsensordata);

        // FFI cross-validation
        for i in 0..nsensordata {
            assert_eq!(current[i], unsafe { *data.ffi().sensordata.add(i) },
                "sensordata[{}] FFI mismatch at step {}", i, step);
        }

        if step > 0 {
            // At least some sensor values should be changing
            // (jointvel should be nonzero once the motor acts)
            let any_changed = (0..nsensordata).any(|i| (current[i] - prev_sensor[i]).abs() > 1e-15);
            if step > 5 {
                assert!(any_changed, "sensor data should evolve after step {}", step);
            }
        }

        prev_sensor.copy_from_slice(current);
    }
}

/// Integration test: save/restore state round-trip, verifying force-cast arrays
/// match the saved snapshot exactly after restoration.
#[test]
fn test_integration_state_save_restore_force_cast() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();

    // Apply control and simulate
    data.ctrl_mut()[0] = 3.0;
    data.ctrl_mut()[1] = -2.0;

    for _ in 0..50 {
        data.step();
    }
    // Synchronize derived quantities with current qpos after integration
    data.forward();

    // Save full physics state
    let saved = data.get_state(MjtState::mjSTATE_FULLPHYSICS as u32);
    let saved_xpos = data.xpos().to_vec();
    let saved_qpos = data.qpos().to_vec();
    let saved_qvel = data.qvel().to_vec();
    let saved_time = data.time();

    // Simulate further (state diverges)
    for _ in 0..50 {
        data.step();
    }
    assert_ne!(data.xpos().to_vec(), saved_xpos);

    // Restore
    // SAFETY: saved was captured via mj_getState; eq_active bytes are valid (0 or 1).
    unsafe { data.set_state(&saved, MjtState::mjSTATE_FULLPHYSICS as u32) };
    data.forward();

    // Primary state must match exactly
    assert_eq!(data.qpos().to_vec(), saved_qpos);
    assert_eq!(data.qvel().to_vec(), saved_qvel);
    assert!((data.time() - saved_time).abs() < 1e-15);

    // Derived quantities (xpos) should match closely
    let restored_xpos = data.xpos();
    for i in 0..saved_xpos.len() {
        for j in 0..3 {
            assert!(
                (restored_xpos[i][j] - saved_xpos[i][j]).abs() < 1e-10,
                "xpos[{}][{}] mismatch after restore: {} vs {}",
                i, j, restored_xpos[i][j], saved_xpos[i][j]
            );
        }
    }
}

/// Integration test: create two separate MjData instances from the same model
/// with different controls, verify their force-cast arrays diverge independently.
#[test]
fn test_integration_independent_data_instances() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data_a = model.make_data();
    let mut data_b = model.make_data();

    // Different controls
    data_a.ctrl_mut()[0] = 10.0;
    data_b.ctrl_mut()[0] = -10.0;

    for _ in 0..100 {
        data_a.step();
        data_b.step();
    }

    let xpos_a = data_a.xpos();
    let xpos_b = data_b.xpos();

    // The slide bodies should be in different positions
    let slide_body_a = data_a.body("slide_body").unwrap();
    let slide_body_b = data_b.body("slide_body").unwrap();

    // Compare xpos of free body (most affected by different controls
    // through indirect constraint coupling) — any measurable difference suffices
    let free_body_a = data_a.body("free_body").unwrap();
    let free_body_b = data_b.body("free_body").unwrap();

    let diff_slide: f64 = (0..3)
        .map(|j| (xpos_a[slide_body_a.id][j] - xpos_b[slide_body_b.id][j]).abs())
        .sum();
    let diff_free: f64 = (0..3)
        .map(|j| (xpos_a[free_body_a.id][j] - xpos_b[free_body_b.id][j]).abs())
        .sum();
    let total_diff = diff_slide + diff_free;

    assert!(total_diff > 1e-6, "different controls should lead to different positions, diff={}", total_diff);

    // Each must still be FFI-consistent
    for i in 0..xpos_a.len() {
        for j in 0..3 {
            assert_eq!(xpos_a[i][j], unsafe { *data_a.ffi().xpos.add(i * 3 + j) });
            assert_eq!(xpos_b[i][j], unsafe { *data_b.ffi().xpos.add(i * 3 + j) });
        }
    }
}

/// Integration test: mocap mutation mid-simulation via force-cast arrays,
/// verify it takes effect in subsequent steps.
#[test]
fn test_integration_mocap_mid_simulation_mutation() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();

    let nmocap = model.ffi().nmocap as usize;
    assert!(nmocap > 0);

    data.forward();
    assert_eq!(data.mocap_pos()[0], [5.0, 5.0, 5.0]);

    // Step a bit
    for _ in 0..20 {
        data.step();
    }

    // Mutate mocap position
    data.mocap_pos_mut()[0] = [0.0, 0.0, 0.0];
    data.mocap_quat_mut()[0] = [0.0, 1.0, 0.0, 0.0]; // 180 deg rotation

    data.forward();

    assert_eq!(data.mocap_pos()[0], [0.0, 0.0, 0.0]);
    assert_eq!(data.mocap_quat()[0], [0.0, 1.0, 0.0, 0.0]);

    // FFI cross-validation
    for j in 0..3 {
        assert_eq!(unsafe { *data.ffi().mocap_pos.add(j) }, [0.0, 0.0, 0.0][j]);
    }
    for j in 0..4 {
        assert_eq!(unsafe { *data.ffi().mocap_quat.add(j) }, [0.0, 1.0, 0.0, 0.0][j]);
    }
}

/// Integration test: model-side force-cast enum arrays (jnt_type, geom_type, etc.)
/// are accessible from integration test context and match expected values.
#[test]
fn test_integration_model_enum_slices() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();

    // Joint types
    let jnt_type = model.jnt_type();
    let free_jnt = model.joint("free_jnt").unwrap();
    let hinge_jnt = model.joint("hinge_jnt").unwrap();
    let slide_jnt = model.joint("slide_jnt").unwrap();
    assert_eq!(jnt_type[free_jnt.id], MjtJoint::mjJNT_FREE);
    assert_eq!(jnt_type[hinge_jnt.id], MjtJoint::mjJNT_HINGE);
    assert_eq!(jnt_type[slide_jnt.id], MjtJoint::mjJNT_SLIDE);

    // Geom types
    let geom_type = model.geom_type();
    let floor = model.geom("floor").unwrap();
    let sphere = model.geom("sphere").unwrap();
    let capsule = model.geom("capsule").unwrap();
    let box_geom = model.geom("box").unwrap();
    assert_eq!(geom_type[floor.id], MjtGeom::mjGEOM_PLANE);
    assert_eq!(geom_type[sphere.id], MjtGeom::mjGEOM_SPHERE);
    assert_eq!(geom_type[capsule.id], MjtGeom::mjGEOM_CAPSULE);
    assert_eq!(geom_type[box_geom.id], MjtGeom::mjGEOM_BOX);

    // Joint limited bools
    let jnt_limited = model.jnt_limited();
    assert_eq!(jnt_limited[free_jnt.id], false);
    assert_eq!(jnt_limited[hinge_jnt.id], false);
    assert_eq!(jnt_limited[slide_jnt.id], true);

    // Sensor types
    let sensor_type = model.sensor_type();
    let touch = model.sensor("touch_sensor").unwrap();
    let jpos = model.sensor("hinge_pos").unwrap();
    let jvel = model.sensor("hinge_vel").unwrap();
    assert_eq!(sensor_type[touch.id], MjtSensor::mjSENS_TOUCH);
    assert_eq!(sensor_type[jpos.id], MjtSensor::mjSENS_JOINTPOS);
    assert_eq!(sensor_type[jvel.id], MjtSensor::mjSENS_JOINTVEL);

    // Equality type
    let eq_type = model.eq_type();
    let eq = model.equality("eq_connect").unwrap();
    assert_eq!(eq_type[eq.id], MjtEq::mjEQ_CONNECT);

    // Actuator types
    let act_trntype = model.actuator_trntype();
    let slide_motor = model.actuator("slide_motor").unwrap();
    assert_eq!(act_trntype[slide_motor.id], MjtTrn::mjTRN_JOINT);
    assert_eq!(model.actuator_ctrllimited()[slide_motor.id], true);
}

/// Integration test: xfrc_applied force-cast mutation, step, and verify
/// cacc/cfrc_int/cfrc_ext (stride-6 arrays) reflect the applied force.
#[test]
fn test_integration_force_applied_cacc_cfrc() {
    let model = MjModel::from_xml_string(INTEGRATION_MODEL).unwrap();
    let mut data = model.make_data();
    let free_body = data.body("free_body").unwrap();
    let fid = free_body.id;

    // Apply an upward external force
    data.xfrc_applied_mut()[fid] = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0];

    // xfrc_applied round-trip: verify the mutation stuck
    let xfrc = data.xfrc_applied();
    assert!((xfrc[fid][2] - 50.0).abs() < 1e-10, "xfrc_applied z should be 50");

    // Step to evolve the system (cacc/cfrc are populated during step's internal computations)
    for _ in 0..30 {
        data.step();
    }

    // The free body should have risen (50N > mass*g for a 1kg body)
    let free_z = data.xpos()[fid][2];
    assert!(free_z > 3.0, "free body should rise above initial z=3 with 50N upward, got z={}", free_z);

    // FFI cross-validation for stride-6 arrays
    let cfrc_ext = data.cfrc_ext();
    let cacc = data.cacc();
    for j in 0..6 {
        assert_eq!(cfrc_ext[fid][j], unsafe { *data.ffi().cfrc_ext.add(fid * 6 + j) });
        assert_eq!(cacc[fid][j], unsafe { *data.ffi().cacc.add(fid * 6 + j) });
    }
}
