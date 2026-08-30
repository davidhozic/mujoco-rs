.. _attribute_views:

===================
Attribute views
===================

The MuJoCo library stores data about joints, bodies, and other elements in contiguous arrays.
These can be challenging to work with, particularly when the array's length varies between elements.
For example, different types of joints may have a different number of degrees of freedom.
`MuJoCo's Python bindings <https://mujoco.readthedocs.io/en/3.12.0/python.html>`_ solve
this issue by providing views to specific ranges in the corresponding arrays.

Like MuJoCo's Python bindings, MuJoCo-rs also provides views. Specifically, we provide views for
attributes of :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` and
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`.

Views borrow data and cannot be preserved across simulation steps, as that would violate
Rust's borrow checker rules. Re-looking up names each step would also be expensive.
To overcome this, "info" structs exist, which cache the resolved index ranges and the model
layout for fast view creation after each step. The layout holds the sizes of the model and the
number of entries that each element owns inside them.

.. warning::

    Cached info structs are tied to the model layout they were created from.
    Calling ``view()`` / ``view_mut()`` with data from an incompatible model
    will panic. Two models are compatible when they agree on every size and on
    the way each element splits those sizes.
    Use ``try_view()`` / ``try_view_mut()`` if you want to handle mismatches
    as ``Result`` values instead of panicking.


Reading
======================

For example, let's say we want to read the position of a free **joint**.
We will first create an info struct by calling :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>joint`
like so:

.. code-block:: rust
    :emphasize-lines: 6

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            // ...
        }
    }

To actually view the data, we will now call
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view` and pass it
a reference to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`, like so:

.. code-block:: rust
    :emphasize-lines: 9

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            println!("{:?}", &joint_info.view(&data).qpos[..3]);  // print x, y and z coordinates.
        }
    }

If an incompatible model is possible in your workflow, use
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>try_view`
and handle the returned ``Result`` instead of panicking.

View attributes like :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qpos`
use :docs-rs:`mujoco_rs::util::<struct>PointerView` (or ``Option<PointerView>`` for optional
fields). ``PointerView`` implements the
`Deref <https://doc.rust-lang.org/std/ops/trait.Deref.html>`_ trait and on deref
acts like a slice. While some fields might be scalars, we still treat those as arrays
for implementation simplicity reasons.

The same ``view`` / ``view_mut`` pattern applies to most named element types --- bodies,
geoms, sites, actuators, sensors, and so on --- each looked up through its corresponding
finder (e.g. ``MjData::body`` or ``MjData::geom``) and info struct.


Writing
==================
The above example shows a read-only view. For mutability,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view_mut` must be called
and passed a mutable reference to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`, like so:

.. code-block:: rust
    :emphasize-lines: 9

    use mujoco_rs::prelude::*;

    fn main() {
        let model = MjModel::from_xml("model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let joint_info = data.joint("football-ball").expect("name not found");
        loop {
            data.step();
            joint_info.view_mut(&mut data).qpos[0] = 0.5;
        }
    }

For fallible mutable access, use
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>try_view_mut`
to get a ``Result`` instead of panicking on an incompatible model.

In mutable views, regular writable fields use
:docs-rs:`mujoco_rs::util::<struct>PointerViewMut`. Fields marked as read-only in the
generated view still support mutation, but only through
:docs-rs:`mujoco_rs::util::<struct>PointerViewUnsafeMut` and explicit ``unsafe``:

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let mut model = MjModel::from_xml("model.xml").expect("could not load the model");
        let actuator_info = model.actuator("slider").unwrap();
        let mut view = actuator_info.view_mut(&mut model);

        unsafe {
            view.dyntype.as_mut_slice()[0] = MjtDyn::mjDYN_NONE;
            view.actnum.as_mut_slice()[0] = 0;
            view.actadr.as_mut_slice()[0] = -1;
        }
    }


Reusing an info with another model
=====================================

Each view call compares the layout of the info struct against the layout of the given
:docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` or
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`.
While both of them name the same model, that comparison is a single pointer check.
Both models keep their own copy of the layout, so after a swap to a different model
each call compares the entire layout instead. Look the element up again on the new model
to get the pointer check back.
