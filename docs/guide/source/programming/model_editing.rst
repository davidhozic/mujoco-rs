.. _model_editing:

======================
Model editing
======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mjs_body| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`
.. |mjs_actuator| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator`

The most general way to create an |mj_model| instance is by loading an XML file
via :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml`.
Due to |mj_model| only allowing (some) changes to parameters and not to the actual
geometry, MuJoCo introduced `Model Editing <https://mujoco.readthedocs.io/en/3.9.0/programming/modeledit.html>`_.

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

After procedurally creating a specification with |mj_spec|, the latter
can either be compiled for direct use in the simulation or saved to an XML file.


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
    Alternatively, methods that have the ``set_`` prefix can be used. Field setters
    (e.g. ``set_pos``, ``set_size``) return nothing, while ``set_name`` and ``set_default``
    return a ``Result`` (``set_name`` fails on a duplicate name, ``set_default`` on an unknown class).
    Setter (``set_``) methods are available for many common fields, including strings and
    several vector/buffer fields. For nested or structured data, use getters that end with
    the ``_mut`` suffix.

Finally, we can now add the base plane, like so:

.. code-block:: rust
    :emphasize-lines: 18-22

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
The specification can also be saved directly to an MJCF (XML) file:

.. code-block:: rust
    :emphasize-lines: 23-28

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
as described in the :ref:`basic_sim` chapter.


Deleting elements
======================
Elements can be removed from a specification with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element`,
which takes the element's raw pointer obtained from
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::traits::<trait>SpecItem::<method>element_mut_pointer`:

.. code-block:: rust

    use mujoco_rs::prelude::*;

    fn main() {
        let mut spec = MjSpec::new();
        let body = spec.world_body_mut().add_body().with_name("ball");

        let body_ptr = body.element_mut_pointer();
        // SAFETY: `body_ptr` refers to a live element of `spec` that has not been deleted.
        unsafe { spec.delete_element(body_ptr).expect("failed to delete the body") };
    }

Because ``delete_element`` takes ``&mut MjSpec``, the borrow checker already invalidates any
references obtained earlier (such as ``body`` above) --- you cannot use a reference into the
spec across the call. The method is ``unsafe`` only because the raw element pointer itself
cannot be validated: in particular, the caller must not pass a pointer to an element that has
already been deleted, which would make MuJoCo operate on freed memory.

.. note::

    The older ``SpecItem::delete`` method is **deprecated since 5.0.0** and is unsound
    (it relies on undefined behavior). Use ``delete_element`` instead.


.. _model_editing_defaults:

Class inheritance (defaults)
==============================
MuJoCo supports `default classes <https://mujoco.readthedocs.io/en/3.9.0/XMLreference.html#default>`_,
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

Elements can then reference the class via their ``childclass`` or ``class`` (``dclass``) attribute
in the model XML.
In Rust code, class assignment is typically done with
:docs-rs:`~mujoco_rs::wrappers::mj_editing::traits::<trait>SpecItem::<method>set_default` or
:docs-rs:`~mujoco_rs::wrappers::mj_editing::traits::<trait>SpecItem::<method>with_default`.
Some item-specific wrappers (for example frames) also expose explicit ``childclass`` setters.


Iterators
================
Since MuJoCo-rs 1.5.0, it is possible to also iterate existing :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
items (geoms, joints, etc.). Iterators exist on :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`.

To iterate over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec` items, call
``[item_type]_iter`` for immutable iteration or ``[item_type]_iter_mut`` for mutable iteration,
with ``[item_type]`` replaced by geom, body, etc.

.. code-block:: rust

    // ...
    for body in spec.body_iter() {  // spec is MjSpec.
        println!("{}", body.name());
    }
    // ...

Iteration over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody` items can be used in a similar way.
The only difference is an additional boolean parameter, which enables recursive iteration when ``true``.

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
child bodies are found with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>child` /
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>child_mut`.
Every finder returns an ``Option`` that is ``None`` when no element with that name exists.

After compilation, an element's numeric id in the resulting |mj_model| can be retrieved with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::traits::<trait>SpecItem::<method>id`
(``None`` when the element has no id yet).


.. _model_editing_actuators:

Configuring actuators
======================
An actuator added with :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>add_actuator`
can be configured into one of MuJoCo's predefined actuator types with the ``set_to_*`` family of
methods on |mjs_actuator| (motor, position, integrated velocity, velocity, damper, cylinder,
muscle, adhesion, and DC motor), mirroring MuJoCo's ``mjs_setToX`` C API.

A method whose parameters are all mandatory takes them positionally (for example
``set_to_velocity(kv)`` or ``set_to_cylinder(timeconst, bias, area, diameter)``). A method whose
underlying C function accepts *nullable* parameters instead takes a dedicated ``Default``-able
config struct --- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PositionConfig`,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>IntVelocityConfig`, and
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>DcMotorConfig` --- in which those nullable
parameters are ``Option`` fields. A config is built either with struct-update syntax or with the
chainable ``with_*`` builder methods (which take the inner value and wrap it in ``Some``), so only
the relevant fields need to be set; everything else stays at its MuJoCo default:

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
