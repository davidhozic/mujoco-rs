.. _attribute_views:

===================
Attribute views
===================

The MuJoCo library stores data about joints, bodies, and other elements in contiguous arrays.
These can be challenging to work with, particularly when the array's length varies between elements.
For example, different types of joints may have different number of degrees of freedom.
`MuJoCo's Python bindings <https://mujoco.readthedocs.io/en/stable/python.html>`_ solve
this issue by providing views to specific ranges in the corresponding arrays.

Like MuJoCo's Python bindings, MuJoCo-rs also provides views. Specifically, we provide views for
attributes of :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` and
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`.

A view cannot be created directly, as that would require recreating the view after each simulation
step. Allowing preservation of views between simulation steps would violate Rust's borrow checker rules. 
To overcome this, "info" structs exist, which store required information for fast view
creation after each step.

Reading
======================

For example, let's say we want to read the position of a free **joint**.
We will first create an info struct by calling :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>joint`
like so:

.. code-block:: rust
    :emphasize-lines: 4

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
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
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            println!("{:?}", &joint_info.view(&data).qpos[..3]);  // print x, y and z coordinates.
        }
    }

All the attributes inside views, like :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qpos`,
are instances of :docs-rs:`mujoco_rs::util::<struct>PointerView`, which implements the
`Deref <https://doc.rust-lang.org/std/ops/trait.Deref.html>`_ trait and on deref
acts like a slice. While some fields might be scalers, we still treat those as arrays
for implementation simplicity reasons.


Writing
==================
The above example shows a read-only view. For mutability, 
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view_mut` must be called
and passed a mutable reference to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`, like so:

.. code-block:: rust
    :emphasize-lines: 7

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            joint_info.view_mut(&mut data).qpos[0] = 0.5;
        }
    }


Other views
======================
Views can be created for other types of items too, as well as for
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`.
The process is exactly the same as shown above.
