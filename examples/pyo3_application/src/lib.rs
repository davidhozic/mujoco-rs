//! Example on how to create a MuJoCo-rs application in for Python using PyO3.
use mujoco_rs::{prelude::*, viewer::MjViewer};
use pyo3::prelude::*;
use std::{rc::Rc, time::Instant};


const MODEL_XML: &str = stringify!(
<mujoco>
    <worldbody>
        <light ambient="0.2 0.2 0.2"/>
        <body name="ball" pos=".2 .2 .2">
            <geom name="green_sphere" size=".1" rgba="0 1 0 1"/>
            <joint type="free"/>
        </body>

        <geom name="floor" type="plane" size="10 10 1" euler="5 0 0"/>

    </worldbody>
</mujoco>
);


#[pyclass(unsendable)]
struct Simulation {
    simulation_state: MjData<Rc<MjModel>>,
    viewer: MjViewer<Rc<MjModel>>,
    last_update: Instant
}

#[pymethods]
impl Simulation {
    #[new]
    pub fn new() -> Self {
        let model = Rc::new(MjModel::from_xml_string(MODEL_XML).unwrap());
        let simulation_state = MjData::new(model.clone());
        let viewer = MjViewer::launch_passive(model, 0).unwrap();
        let last_update = Instant::now();
        Self {simulation_state, viewer, last_update}
    }

    pub fn render_loop(&mut self) {
        let timestep = self.simulation_state.model().opt().timestep;
        while self.viewer.running() {
            // Update every timestep seconds
            if self.last_update.elapsed().as_secs_f64() > timestep {
                self.last_update = Instant::now();
                self.simulation_state.step();
                self.viewer.sync_data(&mut self.simulation_state);
                self.viewer.render();
            }
        }
    }
}


#[pymodule]
fn pyo3_application(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Simulation>()?;
    Ok(())
}
