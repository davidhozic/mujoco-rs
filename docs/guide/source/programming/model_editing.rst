.. _model_editing:

======================
Model editing
======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mjs_body| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody`
.. |mjs_actuator| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsActuator`
.. |mjs_flex| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsFlex`
.. |mjs_frame| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsFrame`
.. |mjs_site| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsSite`

The most general way to create an |mj_model| instance is by loading an XML file
via :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml`.
Due to |mj_model| only allowing (some) changes to parameters and not to the actual
geometry, MuJoCo introduced `Model Editing <https://mujoco.readthedocs.io/en/3.12.0/programming/modeledit.html>`_.

In MuJoCo-rs, we created a high-level wrapper around MuJoCo's C API, which provides
safe wrappers around C structs, as well as methods. Aside from that, we try to stay faithful
to MuJoCo's implementation.

A procedurally generated, not yet compiled, model is represented by its **specification** (|mj_spec|).
A specification can be created **empty** with :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>new`
or **pre-filled** from XML with:

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml` (loads XML from a file),
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml_vfs`
  (loads XML from a file on a virtual file system),
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml_string`
  (loads XML from a model defined in a string in memory).

After creation, we can use |mj_spec| to add items to the model, such as joints, geoms, actuators, etc.
**Non-structured** items can be added through |mj_spec| itself (e.g., actuators, sensors, meshes, etc.).
**Structured** items can be added through |mjs_body| (e.g., bodies, geoms, joints, etc.).

After procedurally creating a specification with |mj_spec|, compile it with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile`. The compiled model
runs in the simulation, and the compiled specification can also be saved to an XML file.

.. attention::

    Model editing stays on one thread. MuJoCo's C++ implementation shares unsynchronized state
    between a specification and its elements, so |mj_spec| is neither ``Send`` nor ``Sync``.


Examples
================
Complete, runnable examples on model editing are available in the repository's examples:

- :gh-example:`Basic model editing <model_editing/model_editing.rs>`
- :gh-example:`Terrain generation <model_editing/terrain_generation.rs>`
- :gh-example:`Procedural tree <model_editing/procedural_tree.rs>`
- :gh-example:`Multi-legged creatures <model_editing/multi_legged_creatures.rs>`


Basic editing
======================
Let's lead with an example. We will create a model, where a ball falls onto a plane.
We start by creating an |mj_spec| instance:

.. code-block:: rust
    :emphasize-lines: 4

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
    }

Now we need to create a spherical body, which will be our ball.
This also includes adding a spherical geom and a free joint.
Since bodies are structured elements, we can't add them to |mj_spec|.
Instead, we will add them to the **world body** (the ``worldbody`` element in a model's XML).

To mutably access the specification's world body, we can use the
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>world_body_mut` method.

.. code-block:: rust
    :emphasize-lines: 5

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body_mut("world").unwrap();
    }

We can now add our ball's body, geom and joint like so:

.. code-block:: rust
    :emphasize-lines: 7-17

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body_mut("world").unwrap();

        // Add the ball
        let ball_body = world.add_body()
            .with_name("ball")                   // name
            .with_pos([0.0, 0.0, 1.0]);          // position

        ball_body.add_geom()
            .with_size([0.010, 0.0, 0.0])        // set the radius to 10 mm.
            .with_type(MjtGeom::mjGEOM_SPHERE);  // make this a spherical geom (default).

        ball_body.add_joint()
            .with_type(MjtJoint::mjJNT_FREE);    // make the ball free to move anywhere.
    }

.. tip::

    In the above block, we used methods that have the ``with_`` prefix.
    These allow method chaining.
    Many fields also carry a ``set_`` method, but not all: an array field such as ``pos`` or
    ``size`` offers ``with_pos`` and ``pos_mut`` only. A plain field setter
    (e.g. ``set_group``, ``set_mass``) returns nothing, while a validated setter returns a
    ``Result`` (``set_name`` fails on a duplicate name, ``set_default`` on an unknown class, and
    ``MjsNumeric::set_size`` on a negative size).
    Setter (``set_``) methods are available for many common fields, including strings and
    several vector/buffer fields. For nested or structured data, use getters that end with
    the ``_mut`` suffix.

Finally, we can now add the base plane, like so:

.. code-block:: rust
    :emphasize-lines: 19-22

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body_mut("world").unwrap();

        // Add the ball
        let ball_body = world.add_body()
            .with_name("ball")                   // name
            .with_pos([0.0, 0.0, 1.0]);          // position

        ball_body.add_geom()
            .with_size([0.010, 0.0, 0.0])        // set the radius to 10 mm.
            .with_type(MjtGeom::mjGEOM_SPHERE);  // make this a spherical geom (default).

        ball_body.add_joint()
            .with_type(MjtJoint::mjJNT_FREE);    // make the ball free to move anywhere.

        // Add the base plane
        world.add_geom()
            .with_type(MjtGeom::mjGEOM_PLANE)
            .with_size([1.0, 1.0, 1.0]);
    }


This concludes the specification's definition.
We can now compile it to a model and save it to an MJB (binary) file.
The specification can also be saved to an MJCF (XML) file. MuJoCo writes MJCF only from a
compiled specification, so ``save_xml`` and ``save_xml_string`` return
:docs-rs:`~mujoco_rs::error::<enum>MjEditError::<variant>SaveFailed` when ``compile`` did not run:

.. code-block:: rust
    :emphasize-lines: 24-28

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body_mut("world").unwrap();

        // Add the ball
        let ball_body = world.add_body()
            .with_name("ball")                   // name
            .with_pos([0.0, 0.0, 1.0]);          // position

        ball_body.add_geom()
            .with_size([0.010, 0.0, 0.0])        // set the radius to 10 mm.
            .with_type(MjtGeom::mjGEOM_SPHERE);  // make this a spherical geom (default).

        ball_body.add_joint()
            .with_type(MjtJoint::mjJNT_FREE);    // make the ball free to move anywhere.

        // Add the base plane
        world.add_geom()
            .with_type(MjtGeom::mjGEOM_PLANE)
            .with_size([1.0, 1.0, 1.0]);

        // Compile and save
        let model = spec.compile().expect("failed to compile");
        spec.save_xml("model.xml").expect("failed to save");     // save XML to a file.
        let xml_str = spec.save_xml_string(8192).expect("failed to save");  // save XML to a string.
        model.save_to_file("model.mjb").expect("failed to save"); // save binary.
    }


The model from the above example, generated by :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile`,
can be used exactly the same as if we were to directly load an XML model (see :ref:`basic_sim`).
The compiled |mj_model| can also be swapped into a running simulation using
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model` or
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_swap_model`,
as described in the :ref:`basic_sim` chapter. The swap keeps the existing |mj_data|, so it accepts
only a compatible model, which can be checked by
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`.
A compatible model keeps the element tree, the ``nq`` of each joint and
the type of each sensor, and every size that fixes an |mj_data| buffer, such as ``nuserdata``,
``na`` and ``nout``. Most attribute changes therefore keep compatibility, while an added or
deleted element, a joint type change that alters ``nq`` (for example ``hinge`` to ``ball``), a
changed sensor type or a changed actuator dynamics type breaks it; ``swap_model`` then panics and
``try_swap_model`` returns
:docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IncompatibleModel`.


Attaching elements
======================
In addition to adding elements to the specification and other elements directly,
existing trees of elements can be *attached* directly onto a new tree.

Elements can be attached to another element via the
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>Attach::<method>attach` method,
available on types implementing ``Attach``. Elements implementing ``Attach`` are |mjs_body|,
|mjs_frame| and |mjs_site|. To attach a whole specification, first
add a frame to the parent's world body with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_frame`
and attach to the newly added frame. Here is the full mapping of what element type can be attached to
what other element type:

.. list-table::
    :header-rows: 1
    :widths: 30 70

    * - Parent
      - Child
    * - |mjs_body|
      - |mjs_frame|, |mj_spec|
    * - |mjs_frame|
      - |mjs_body|, |mjs_frame|, |mj_spec|
    * - |mjs_site|
      - |mjs_body|, |mjs_frame|, |mj_spec|

When attaching elements, a *prefix* and a *suffix* can be given. When no prefix/suffix is desired, just pass ``""`` to
``Attach::attach`` at its respective positions.

The example below attaches a one-body specification under a frame of another specification.

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        // Child specification.
        let mut child = MjSpec::new();
        child.world_body_mut().add_body().with_name("ball")
            .add_geom().with_size([0.01, 0.0, 0.0]);

        // Parent specification.
        let mut parent = MjSpec::new();
        // Add a frame onto which the child will be attached.
        let frame = parent.world_body_mut().add_frame().with_pos([0.0, 0.0, 1.0]);

        // Attach the child to the newly-created frame, giving all sub-elements
        // of the child tree "robot_" as a prefix to all the names.
        // This method is available by importing the `Attach` trait (already in the prelude).
        frame.attach(&mut child, "robot_", "").unwrap();

        // The child is now attached.
        assert!(parent.body("robot_ball").is_some());
        parent.compile().expect("failed to compile");
    }

More on attachment is available in MuJoCo's
`official documentation on attachment <https://mujoco.readthedocs.io/en/stable/programming/modeledit.html#attachment>`__.

Deleting elements
======================
Most elements can be removed from a specification with the ``unsafe``
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecObject::<method>delete`, called on the
handle of the element. A default class and a tendon wrap are no
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecObject`, so they carry no ``delete`` at all.
The world body and a frame do carry it, and it returns
:docs-rs:`~mujoco_rs::error::<enum>MjEditError::<variant>UnsupportedOperation` for them.

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        spec.world_body_mut().add_body().with_name("ball");

        unsafe { spec.body_mut("ball").unwrap().delete() }.expect("failed to delete the body");
    }

``delete`` is ``unsafe`` because MuJoCo cannot answer whether it already deleted an element. It
keeps a deleted element allocated until the specification drops, so a second deletion of the same
element frees it twice. The caller carries three obligations:

- Delete each element at most once.
- Do not delete an element that the deletion of a body already took out of the specification.
- Do not use the handle of an element that the deletion of a body freed.

Deleting a body deletes its whole subtree, and it frees every keyframe, and every actuator, sensor,
tendon, equality, pair and exclude that refers to the subtree. The last two obligations follow from
that: an iterator collected before such a deletion still hands out the handles it took.

The borrow checker covers the ordinary case, because a handle borrows the specification and a
deletion needs it mutably. Look the next element up in each round, as the loop below does, and every
handle the loop holds belongs to a live element.

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn delete_every_geom(spec: &mut MjSpec) {
        // SAFETY: the walk starts again after each deletion, so it reaches a live geom only.
        while let Some(geom) = spec.geom_iter_mut().next() {
            unsafe { geom.delete() }.unwrap();
        }
    }


.. _model_editing_defaults:

Class inheritance (defaults)
==============================
MuJoCo supports `default classes <https://mujoco.readthedocs.io/en/3.12.0/XMLreference.html#default>`_,
which allow shared attribute values to be set in one place and then inherited by multiple elements.
In MuJoCo-rs, default classes can be created with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>add_default`.

A new class is created by providing its ``class_name`` and optionally a ``parent_class_name``
(for class inheritance). If no parent is given, the class inherits from the main default.

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();

        // Create a top-level default class named "red".
        // Use try_add_default() for a fallible variant that returns Result.
        spec.add_default("red", None);

        // Create a child class "small-red" that inherits from "red".
        spec.add_default("small-red", Some("red"));
    }

Elements can then reference the class via their ``childclass`` or ``class`` attribute
in the model XML.
In Rust code, class assignment is done with
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_default` or
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>with_default`.
MuJoCo copies the values of a class into an element when the element is **created**, so set the
class on the parent body before you add its children. A call on an element that already exists
only records the class name: the saved XML then carries ``class="..."``, but the next ``compile``
keeps the old values. A frame also exposes an explicit ``childclass`` setter
(:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsFrame::<method>set_childclass`), with the
same limitation.


Iterators
================
Since MuJoCo-rs 1.5.0, it is possible to also iterate existing :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
items (geoms, joints, etc.). Iterators exist on :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody`.

To iterate over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec` items, call
``[item_type]_iter`` for immutable iteration or ``[item_type]_iter_mut`` for mutable iteration,
with ``[item_type]`` replaced by geom, body, etc.

A body is the exception: |mj_spec| carries ``body_iter`` alone. Every item that a mutable iterator
yields borrows the spec for the same lifetime, so all of them stay live together. A body owns a
subtree, so a flat mutable body iterator would hand out a body and that body's own descendant at
once. Reach a body through
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>world_body_mut` and walk down
the tree.

.. code-block:: rust

    // ...
    for body in spec.body_iter() {  // spec is MjSpec.
        println!("{}", body.name());
    }
    // ...

Iteration over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody` items can be used in a similar way.
The only difference is an additional boolean parameter, which enables recursive iteration when ``true``.
The mutable body iterator
(:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>body_iter_mut`) takes no such
parameter and yields the direct children only, for the reason above.

.. code-block:: rust

    // ...
    // Iterate top level bodies of body.
    for body in body.body_iter(false) {  // body is MjsBody.
        println!("{}", body.name());
    }
    // ...

.. code-block:: rust

    // ...
    // Iterate top level bodies of body + their sub-bodies recursively.
    for body in body.body_iter(true) {  // body is MjsBody.
        println!("{}", body.name());
    }
    // ...


Finding elements
================
Existing elements can be looked up by name. |mj_spec| exposes a finder method (and a matching
``_mut`` variant) per item type, for example
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>body` /
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>body_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>geom`, and
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>frame`. Within a body,
a body of its subtree is found with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>child` /
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>child_mut`.
The search is recursive, and it returns the body itself when its own name matches.
Every finder returns an ``Option`` that is ``None`` when no element with that name exists.

After compilation, an element's numeric id in the resulting |mj_model| can be retrieved with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>id`
(``None`` when the element has no id yet).


.. _model_editing_actuators:

Configuring actuators
======================
An actuator added with :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>add_actuator`
can be configured into one of MuJoCo's predefined actuator types with the ``set_to_*`` family of
methods on |mjs_actuator| (motor, position, integrated velocity, velocity, damper, cylinder,
muscle, adhesion, DC motor, PID, and orientation), mirroring MuJoCo's ``mjs_setToX`` C API.

A method whose parameters are all mandatory takes them positionally (for example
``set_to_velocity(kv)`` or ``set_to_cylinder(timeconst, bias, area, diameter)``). A method whose
underlying C function accepts *nullable* parameters instead takes a dedicated ``Default``-able
config struct --- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PositionConfig`,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>IntVelocityConfig`,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>DcMotorConfig`,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PidConfig`, and
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>OrientationConfig` --- in which those nullable
parameters are ``Option`` fields. A config is built either with struct-update syntax or with the
chainable ``with_*`` builder methods (which take the inner value and wrap optional fields in
``Some``), so only the relevant fields need to be set; everything else stays at its MuJoCo default:

.. code-block:: rust

    use mujoco_rs::prelude::*;
    use mujoco_rs::wrappers::mj_editing::DcMotorConfig;

    fn main() {
        let mut spec = MjSpec::new();
        let body = spec.world_body_mut().add_body();
        body.add_geom().with_size([0.01, 0.0, 0.0]);
        body.add_joint().with_name("hinge");

        let actuator = spec.add_actuator().with_trntype(MjtTrn::mjTRN_JOINT);
        actuator.set_target("hinge");

        // Configure as a DC motor using the chainable builder. Struct-update syntax works too:
        //   DcMotorConfig { motorconst: Some([1.0, 1.0]), resistance: 1.0, ..Default::default() }
        actuator.set_to_dc_motor(
            DcMotorConfig::default().with_motorconst([1.0, 1.0]).with_resistance(1.0)
        ).expect("invalid DC motor parameters");
    }

Methods that MuJoCo can reject (for example a negative gain, or mutually exclusive parameters)
return ``Result<(), MjEditError>``, and the only error variant they ever produce is
:docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>InvalidParameter`; the parameterless or
always-valid ones (``set_to_motor``, ``set_to_velocity``, ``set_to_cylinder``) return ``()``.


.. _model_editing_flexcomp:

Procedural flex generation
==========================
A `flexcomp <https://mujoco.readthedocs.io/en/3.12.0/XMLreference.html#body-flexcomp>`_ procedurally
generates a deformable |mjs_flex| together with its supporting bodies, joints, and optional
equality constraints --- handy for cloth, ropes, and soft volumes. In MuJoCo-rs it is created with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_flexcomp` (or the fallible
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_flexcomp`).

The generation is configured through a ``Default``-able
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjFlexcompConfig`, using the same ``with_*``
builder pattern as the actuator configs above: only the fields you set deviate from MuJoCo's
defaults, and unset optional fields fall back to the compiler's own defaults. ``add_flexcomp``
returns the created |mjs_flex|, which can be tweaked further (for example its self-collision mode).

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();

        // A 5x5 cloth-like 2D grid, held together by edge equality constraints.
        let config = MjFlexcompConfig::default()
            .with_type("grid")
            .with_dim(2)
            .with_count([5, 5, 1])
            .with_spacing([0.1, 0.1, 0.1])
            .with_mass(1.0)
            .with_equality(1);  // 0 none, 1 edge, 2 vertex, 3 strain

        spec.world_body_mut().add_flexcomp("cloth", &config);
        spec.compile().expect("failed to compile");
    }
