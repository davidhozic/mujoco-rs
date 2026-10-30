//! Example of using the wrapper around MuJoCo's batch API.
use std::num::NonZero;

use mujoco_rs::{prelude::*, wrappers::{MjBatchRecordBuilder, MjtNum}};

fn main() {
    let model = MjModel::from_xml_string(r#"
        <mujoco>
            <worldbody>
                <geom type="plane" size="0 0 0.1"/>
                <body pos="0 0 0.2">
                    <geom type="sphere" size="0.1"/>
                    <freejoint/>
                </body>
            </worldbody>
        </mujoco>
    "#).unwrap();

    /* Batch creation */
    // Use half of the logical CPUs. Zero (one CPU or no detection) makes MuJoCo use all of them.
    let nthread = (std::thread::available_parallelism().map_or(0, NonZero::get) / 2) as i32;
    let mut batch = MjBatch::new(
        &model,
        10,  // number of simulations
        Some(MjBatchConfig {
            nthread,
            persistent: false  // one MjData per thread (not per simulation instance)
        })
    );
    println!("Running {} simulations, {} in parallel", batch.num_sim(), batch.num_thread());

    /* Rollout */
    // Inputs of num_sim x num_steps x state size (6 for mjSTATE_QFRC_APPLIED of a free joint).
    // Each simulation is pushed along x by a different force at the first step only.
    let num_sim = batch.num_sim() as i32;
    let controls: Vec<_> = (0..num_sim)
        .map(|sim| [
            [100.0 * MjtNum::from(sim - num_sim / 2), 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0; 6]
        ])
        .collect();

    // Record buffer with the same layout as the inputs.
    let mut recorded = vec![[[0.0; 6]; 2]; batch.num_sim()];

    let failed = batch.rollout(
        /* maybe_ids = */ None,
        /* num_steps = */ 2,
        /* states_and_mask = */ Some((
            controls.as_flattened().as_flattened(),
            MjtState::mjSTATE_QFRC_APPLIED as u32
        )),
        /* batch_records = */ [MjBatchRecordBuilder::new(
            recorded.as_flattened_mut().as_flattened_mut(),
            MjtState::mjSTATE_QFRC_APPLIED as u32
        )],
    );
    println!("Rollout failed in {failed} simulations");

    // The record holds qfrc_applied after each step, which is the input of that step.
    assert_eq!(recorded, controls);

    /* Stepping */
    // Non-state fields only! To read state fields, use `state_at`.
    // Register the outputs first. These are filled in the bottom stepping section.
    let fields = ["xpos", "geom_xpos"];
    for field in fields {
        batch.output_register(field).unwrap();
    }

    let failed = batch.step(
        /* maybe_ids = */ None,
        /* num_steps = */ 100000,
        /* stage = */ MjtStage::mjSTAGE_NONE  // compute no extra stages after the last step
    );
    println!("Step failed in {failed} simulations");

    /* Read MjData's fields */
    for field in fields {
        let output: &[MjtNum] = batch
            .output(field).unwrap();
        let data: Vec<MjtNum> = output
            .chunks_exact(output.len() / num_sim as usize)  // One row per simulation.
            .map(|row| row[3])  // x of body 1, the ball.
            .collect();
        println!("{field}={data:?}");
    }
}
