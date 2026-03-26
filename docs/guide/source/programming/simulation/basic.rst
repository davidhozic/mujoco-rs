.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`

.. _basic_sim:

======================
Basic simulation
======================

Loading
==========================
To perform basic simulation with MuJoCo, create a |mj_model| struct by calling one of the following methods:

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml` (loads XML from a file),
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_vfs`
  (loads XML from a file on a virtual file system),
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_string`
  (loads XML from a model defined in a string in memory),
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_buffer`
  (loads a compiled model from a buffer).

For example:

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
    }


|mj_model| represents the compiled model from an XML file and contains everything from
basic metadata to physics parameters.

For managing actual simulation state, :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
is used. It can be created either through the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>new`
method or through the :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>make_data` method.
For example:

.. code-block:: rust
    :emphasize-lines: 3

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);  // or model.make_data()
    }


.. tip::

    Using the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>new` method is far **more flexible**
    than using the :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>make_data` method.
    The former allows parameters to be of any type as long as they implement
    the `Deref <https://doc.rust-lang.org/std/ops/trait.Deref.html>`_
    trait (e.g., `Box\<MjModel\> <https://doc.rust-lang.org/std/boxed/struct.Box.html>`_). For example:

    .. code-block:: rust
        :emphasize-lines: 2

        fn main() {
            let model = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));
            let mut data = MjData::new(model);  // move model into the data
            let model_ref = data.model();  // obtain a reference to the model
        }

    Note that all related APIs must use the same model-handle type ``M``.
    For example, if you create ``MjData<Box<MjModel>>``, APIs expecting
    ``MjData<&MjModel>`` (or ``Rc``/``Arc`` variants) are not interchangeable.
    For shared ownership, use
    `Rc\<MjModel\> <https://doc.rust-lang.org/std/rc/struct.Rc.html>`_
    (single-threaded) or
    `Arc\<MjModel\> <https://doc.rust-lang.org/std/sync/struct.Arc.html>`_
    when thread-sharing is needed.

    Using ``Box`` or ``Rc`` (instead of direct references) allows usage in environments with lifetime restrictions.
    One such example is **Python bindings** created with **PyO3**.
    The :gh-example:`pyo3_application` example shows how to create a simple MuJoCo-rs based application
    for use from the Python programming language.


Running
====================

Then to run/step the simulation, just call the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`
method like so:

.. code-block:: rust
    :emphasize-lines: 5

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        loop {
            data.step();
        }
    }

The method :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step` is just a wrapper around the
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step` FFI function.
Similarly, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step1` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step2` wrap
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step1` and :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step2`, respectively.

For more information about specific MuJoCo functions, see the
`MuJoCo documentation <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step>`_.

Real-time
----------------------
The example above runs the simulation as fast as possible.
To slow it down, you can either add a call to
`std::thread::sleep <https://doc.rust-lang.org/std/thread/fn.sleep.html>`_
for an approximate delay, or use
`std::time::Instant <https://doc.rust-lang.org/std/time/struct.Instant.html>`_
with `Instant::elapsed <https://doc.rust-lang.org/std/time/struct.Instant.html#method.elapsed>`_
for precise timing.


.. code-block:: rust
    :emphasize-lines: 7, 10, 11

    use std::time::Duration;
    use std::thread;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let timestep = model.opt().timestep;
        loop {
            data.step();
            /* Approximate-time delay */
            thread::sleep(Duration::from_secs_f64(timestep))
        }
    }


Changing model's parameters
================================
For purposes of transfer from simulation to reality in reinforcement learning
it is beneficial to perform domain randomization [Tobin2017]_.

.. danger::

    Not all parameters of |mj_model| are safe to change.
    See `MuJoCo's documentation <https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#mjmodel-changes>`_
    for a list of parameters that are safe to change.

Direct mutation with ``model_mut``
------------------------------------

When the model-handle type ``M`` implements
`DerefMut\<Target = MjModel\> <https://doc.rust-lang.org/std/ops/trait.DerefMut.html>`_
(e.g., owned |mj_model|, ``Box<MjModel>``), the
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_mut` method
gives direct mutable access to the model's physics parameters.

.. code-block:: rust
    :linenos:
    :emphasize-lines: 5, 6

    fn main() {
        let model = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));
        let mut data = MjData::new(model);

        data.model_mut().opt_mut().timestep = 0.004;
        data.model_mut().opt_mut().gravity[2] = -5.0;
    }

.. note::

    ``model_mut`` is **not** available when ``M`` is a shared-ownership type
    (e.g., ``Arc<MjModel>``, ``&MjModel``). In those cases, use ``swap_model`` below.

See the :gh-example:`model_parameters.rs` example for a complete runnable comparison of all three
approaches (``model_mut``, ``swap_model``, and ``unsafe ffi_mut``).

Swapping models with ``swap_model``
--------------------------------------

When ``M`` does not support ``DerefMut`` (e.g., ``Arc<MjModel>``), or when you need to
swap in an entirely different model instance, use the
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model` method,
which swaps the |mj_model| owned by |mj_data| for the |mj_model| given as parameter.

.. code-block:: rust
    :linenos:
    :emphasize-lines: 8

    fn main() {
        let mut model_template = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));

        let model = model_template.clone();
        let mut data = MjData::new(model);

        // Modify simulation timestep
        model_template.opt_mut().timestep = 0.004;
        model_template = data.swap_model(model_template).unwrap();
    }

.. [Tobin2017] J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, and P. Abbeel,
   "Domain randomization for transferring deep neural networks from simulation to
   the real world," in *2017 IEEE/RSJ International Conference on Intelligent
   Robots and Systems (IROS)*, Vancouver, BC, Canada, Sep. 2017, pp. 23--30.
   doi: `10.1109/IROS.2017.8202133 <https://ieeexplore.ieee.org/document/8202133>`_
