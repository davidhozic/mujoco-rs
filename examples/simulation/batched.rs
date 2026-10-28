//! Example of using the wrapper around MuJoCo's batch API.
use std::num::NonZero;

use mujoco_rs::{prelude::*, wrappers::MjtNum};

fn main() {
    let model = MjModel::from_xml_string(r#"
        <mujoco>
            <worldbody>
                <geom type="plane" size="0 0 0.1"/>
                <body>
                    <geom type="sphere" size="0.1" pos="0 0 0.2"/>
                    <freejoint/>
                </body>
            </worldbody>
        </mujoco>
    "#).unwrap();

    // Use half of the logical CPUs. Zero (one CPU or no detection) makes MuJoCo use all of them.
    let nthread = (std::thread::available_parallelism().map_or(0, NonZero::get) / 2) as i32;

    // Create a batch of simulations.
    let mut batch = MjBatch::new(
        &model,
        100,  // number of simulations
        Some(MjBatchConfig {
            nthread,
            persistent: false  // one MjData per thread (not per simulation instance)
        })
    );

    /* Batched operations */
    println!("Running {} simulations, {} in parallel", batch.num_sim(), batch.num_thread());

    // Push each simulation along x using different force.
    let num_sim = batch.num_sim() as i32;
    let controls: Vec<_> = (0..num_sim)
        .map(|sim| [
            // Apply the force (vector of 6 scalar forces) at the first step.
            [0.005 * MjtNum::from(sim - num_sim / 2), 0.0, 0.0, 0.0, 0.0, 0.0],
            // Clear the force at the second step.
            [0.0; 6]
        ])
        .collect();

    // Record buffer of num_sim x num_steps x state size (6 for mjSTATE_QFRC_APPLIED).
    let mut rollout = vec![[[0.0; 6]; 2]; batch.num_sim()];

    // Step all simulations with rollout.
    let failed = batch.rollout(
        /* maybe_ids = */ None,
        /* num_steps = */ 2,
        /* states_and_mask = */ Some((
            controls.as_flattened().as_flattened(),
            MjtState::mjSTATE_QFRC_APPLIED as u32
        )),
        /* records_and_mask = */ &mut [  // record both steps
            (rollout.as_flattened_mut().as_flattened_mut(), MjtState::mjSTATE_QFRC_APPLIED as u32)
        ],
    );
    println!("Rollout failed in {failed} simulations");

    // The record contains qfrc_applied, thus it must equal our applied controls.
    assert_eq!(rollout, controls);

    // Step all simulations without rollout.
    let failed = batch.step(
        /* maybe_ids = */ None,
        /* num_steps = */ 100000,
        /* stage = */ MjtStage::mjSTAGE_NONE  // run up to this stage (in this case, run all stages)
    );
    println!("Step failed in {failed} simulations");
}
