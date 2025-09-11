.. _basic_sim:

======================
Basic simulation
======================

Loading
==========================
To perform basic simulation with MuJoCo, create a :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
struct by calling one of the following methods:

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml` (loads xml from a file),
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_vfs`
  (loads xml from a file on a virtual file system),
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_string`
  (loads xml from a model defined in a string in memory).
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_buffer`
  (loads a compiled model from a buffer)

For example:

.. code-block:: rust

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
    }


:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel` represents the compiled moded from an XML
file and contains everything from basic metadata to physics parameters.

For managing actual simulation state, :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
is used. It can be created either through the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>new`
method or through the :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>make_data` method.
For example:

.. code-block:: rust
    :emphasize-lines: 3

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();  // or MjData::new(&model);
    }


Running
====================

Then to run/step the simulation, just call the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`
method like so:

.. code-block:: rust
    :emphasize-lines: 5

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();  // or MjData::new(&model);
        loop {
            data.step();
        }
    }

The method :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step` is just a wrapper around the
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step` FFI function.
Similarly, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step1` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step2` wrap
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step1` and :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step2`, respectively.

For more information about the specific MuJoCo functions, see the
`MuJoCo documentation <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step>`_.

Realtime
----------------------
The above example will run the simulation as fast as possible.
To run the simulation at slower pace, simply add a ``std::thread::sleep``
line after the step.

To run the code in realtime, the duration in the sleep must match the simulation timestep.
The simulation timestep can be obtained through the simulation options:

.. code-block:: rust
    :emphasize-lines: 7, 10

    use std::time::Duration;
    use std::thread;

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();  // or MjData::new(&model);
        let timestep = model.opt().timestep;
        loop {
            data.step();
            thread::sleep(Duration::from_secs_f64(timestep))
        }
    }
