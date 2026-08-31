.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`

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
    :emphasize-lines: 5

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);  // or model.make_data()
    }


Using the ``MjData::new`` method is far **more flexible** than using the ``MjModel::make_data``
method.
The former allows parameters to be of any type as long as they implement
:docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelType`
(e.g., `Box\<MjModel\> <https://doc.rust-lang.org/std/boxed/struct.Box.html>`_). For example:

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let model = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));
        let mut data = MjData::new(model);  // move model into the data
        let model_ref = data.model();  // obtain a reference to the model
    }

The model-handle type ``M`` must implement ``ModelType`` --- a MuJoCo-rs trait that is
pre-implemented for most standard ``Deref`` containers, such as
`Rc <https://doc.rust-lang.org/std/rc/struct.Rc.html>`_ and
`Arc <https://doc.rust-lang.org/std/sync/struct.Arc.html>`_.
:docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut` implements the same but for ``DerefMut``
standard containers.
For shared ownership, use ``Rc<MjModel>`` (single-threaded) or ``Arc<MjModel>``
when thread-sharing is needed. For non-shared usage, ``Box<MjModel>`` is the most appropriate,
as it also allows :ref:`changes to model parameters <changing_model_parameters>`.

Using ``Box`` or ``Rc`` (instead of direct references) allows usage in environments with lifetime restrictions.
One such example is **Python bindings** created with **PyO3**.
The :gh-example:`standalone/pyo3_application` example shows how to create a simple MuJoCo-rs based application
for use from the Python programming language.


.. dropdown:: Implementing a custom ``ModelType``
    :color: warning

    .. note::

        This section is optional for most users as the standard containers inside
        the Rust standard library should suffice.

    ``ModelType`` and ``ModelTypeMut`` are ``unsafe`` traits with two rules: every ``deref`` call
    returns the same |mj_model|, and that model stays valid and in place for as long as the
    container lives. Plain ``Deref`` does not guarantee the rules on its own.

    To use a container of your own, implement ``Deref`` and then add the ``unsafe impl``:

    .. code-block:: rust

        use mujoco_rs::wrappers::mj_model::traits::ModelType;
        use mujoco_rs::prelude::*;
        use std::ops::Deref;

        struct MyHandle(Box<MjModel>);

        impl Deref for MyHandle {
            type Target = MjModel;
            fn deref(&self) -> &MjModel { &self.0 }
        }

        // SAFETY: deref returns the model that the handle owns, on every call.
        unsafe impl ModelType for MyHandle {}

    Add ``DerefMut`` and ``ModelTypeMut`` the same way when the container also gives mutable access
    to the model.


Running
====================

Then to run/step the simulation, just call the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`
method like so:

.. code-block:: rust
    :emphasize-lines: 7

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        loop {
            data.step();
        }
    }

The method ``MjData::step`` is just a wrapper around the
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step` FFI function.
Similarly, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step1` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step2` wrap
:docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step1` and :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_step2`, respectively.

For more information about specific MuJoCo functions, see the
`MuJoCo documentation <https://mujoco.readthedocs.io/en/3.12.0/APIreference/APIfunctions.html#mj-step>`_.

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
    :emphasize-lines: 9, 13

    use std::time::Duration;
    use std::thread;

    use mujoco_rs::prelude::*;

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

.. _changing_model_parameters:

Changing model's parameters
================================
For purposes of transfer from simulation to reality in reinforcement learning
it is beneficial to perform domain randomization [Tobin2017]_.
This requires modifying different aspects of the environment,
such as physics parameters --- part of |mj_model|.

.. danger::

    Not all parameters of |mj_model| are safe to change.
    See `MuJoCo's documentation <https://mujoco.readthedocs.io/en/3.12.0/programming/simulation.html#mjmodel-changes>`_
    for a list of parameters that are safe to change.

Direct mutation with ``model_opt_mut`` / ``model_vis_mut`` / ``model_stat_mut``
-------------------------------------------------------------------------------

When the model-handle type ``M`` implements ``ModelTypeMut``
(e.g., ``Box<MjModel>``), the safe accessors
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_opt_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_vis_mut`, and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_stat_mut`
give direct mutable access to the model's physics options, visualization settings,
and statistics respectively.

.. code-block:: rust
    :linenos:
    :emphasize-lines: 6, 7

    use mujoco_rs::prelude::*;

    fn main() {
        let model = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));
        let mut data = MjData::new(model);
        data.model_opt_mut().timestep = 0.004;
        data.model_opt_mut().gravity[2] = -5.0;
    }

.. note::

    For full mutable access to the entire |mj_model| (e.g., to replace the model in-place),
    use the ``unsafe``
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_mut` method.
    It requires ``M: ModelTypeMut`` and is unsafe because swapping to an
    incompatible model can violate internal invariants.

    ``model_opt_mut`` and its siblings are **not** available when ``M`` is a shared-ownership
    type (e.g., ``Arc<MjModel>``, ``&MjModel``). In those cases,
    use :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model`
    or its fallible twin :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_swap_model`
    below. The latter two methods are not unsafe as failure results in a panic
    or in a returned error, respectively.

See the :gh-example:`simulation/model_parameters.rs` example for a complete runnable comparison of both
approaches (``model_opt_mut`` and ``swap_model``).

Swapping models with ``swap_model``
--------------------------------------

When ``M`` does not implement ``ModelTypeMut`` (e.g., ``Arc<MjModel>``), or when you need to
swap in an entirely different model instance, use the ``swap_model`` method
or the ``try_swap_model`` method,
which swap the |mj_model| owned by |mj_data| for the |mj_model| given as parameter.

.. code-block:: rust
    :linenos:
    :emphasize-lines: 11

    use mujoco_rs::prelude::*;

    fn main() {
        let mut model_template = Box::new(MjModel::from_xml("model.xml").expect("could not load the model"));

        let model = model_template.clone();
        let mut data = MjData::new(model);

        // Modify simulation timestep
        model_template.opt_mut().timestep = 0.004;
        model_template = data.swap_model(model_template);
    }

Note that the above example uses |mj_model| as the ``model_template`` only
as an illustration --- usually this would be something like |mj_spec|, however that
is reserved for the :ref:`model_editing` chapter.

.. [Tobin2017] J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, and P. Abbeel,
   "Domain randomization for transferring deep neural networks from simulation to
   the real world," in *2017 IEEE/RSJ International Conference on Intelligent
   Robots and Systems (IROS)*, Vancouver, BC, Canada, Sep. 2017, pp. 23--30.
   doi: `10.1109/IROS.2017.8202133 <https://ieeexplore.ieee.org/document/8202133>`_
