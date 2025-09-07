===================
Attribute views
===================

The MuJoCo library stores information about joints, bodies, etc., in a
contiguous arrays, which can be hard to work with, especially when the
item in question doesn't always have a fixed-size number of e. g. degrees of freedom.

The `Python <https://mujoco.readthedocs.io/en/stable/python.html>`_ MuJoCo bindings solve
this issue by providing views to specific ranges in the corresponding arrays.
MuJoCo-rs follows this and also provides views. Specifically we provide views for
attributes of :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` and attributes of
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`:

A view cannot be created directly, as that would violate Rust's borrow checker rules.
To overcome this, the so called "info" structs must be created first.

Reading
======================

For example, let's say we want to read-only view the position of a free **joint**.
We will first create an info struct by calling :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>joint`
like so:

.. code-block:: rust
    :emphasize-lines: 4

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            ...
        }
    }

To actually view the data, we will now call
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view` and pass it
a reference to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`, like so:

.. code-block:: rust
    :emphasize-lines: 7

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            println!("{:?}", &joint_info.view(&data).qpos[..3]);  // print x, y and z coordinates.
        }
    }

All the attributes inside views, like :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qpos`,
are instances of :docs-rs:`mujoco_rs::util::<struct>PointerView`, which implements the ``Deref`` trait and thus
acts like a slice. This is also true for fields that may actually be scalers, which we still treat
like arrays for consistency and simplicity reasons.

Writing
==================
The above examples show a read-only view. To be able to write to the viewed data,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view_mut` must be called
and passed a mutable reference to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`, like so:

.. code-block:: rust
    :emphasize-lines: 7

    fn main() {
        let model = MjModel::from_xml("model.rs").expect("could not load the model");
        let mut data = model.make_data();
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            joint_info.view_mut(&mut data).qpos[0] = 0.5;
        }
    }


Other views
======================
Views can be created for other types of items too and for :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
too. The process is exactly the same as shown above.
