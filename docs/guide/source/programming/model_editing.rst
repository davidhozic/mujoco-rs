.. _model_editing:

======================
Model editing
======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mjs_body| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`

The most general way to create an |mj_model| instance is by loading an XML file
via :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml`.
Due to |mj_model| only allowing (some) changes to parameters and not to the actual
geometry, MuJoCo introduced `Model Editing <https://mujoco.readthedocs.io/en/stable/programming/modeledit.html>`_.

In MuJoCo-rs, we created a high-level wrapper around MuJoCo's C API, which provides
safe wrappers around C structs, as well as methods. Aside to that, we try to stay faithful
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

Basic editing
======================
Let's lead with an example. We will create a model, where a ball falls onto a plane.
We start by creating an |mj_spec| instance:

.. code-block:: rust
    :emphasize-lines: 2

    fn main() {
        let mut spec = MjSpec::new();
    }

Now we need to create a spherical body, which will be our ball.
This also includes adding a spherical geom and a free joint.
Since bodies are structured elements, we can't add them to |mj_spec|.
Instead, we will add them to the **world body** (the ``worldbody`` element in a model's XML).

To mutable access the specification's world body, we can use the
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>world_body_mut` method.

.. code-block:: rust
    :emphasize-lines: 3

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body("world").unwrap();
    }

We can now add our ball's body, geom and joint like so:

.. code-block:: rust
    :emphasize-lines: 5-15

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body("world").unwrap();
        
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
    Alternatively, methods that have the ``set_`` prefix can be used, which don't return anything.
    Setter (``set_``) methods exists only for simple types. Anything more complex can be modified
    through getters, which end with the ``_mut`` suffix.

Finally, we can now add the base plane, like so:

.. code-block:: rust
    :emphasize-lines: 18-20

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body("world").unwrap();

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


This concludes specification's definition.
We can now compile it to a model, which can then be saved to either an MJCF (XML) file or
to an MJB (binary) file:

.. code-block:: rust
    :emphasize-lines: 23-25

    fn main() {
        let mut spec = MjSpec::new();
        let world = spec.world_body_mut();       // or spec.body("world").unwrap();

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
        model.save(Some("model.mjb"), None);                      // save binary.
    }


The model from the above example, generated by :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile`,
can be used exactly the same as if we were to directly load an XML model (see :ref:`basic_sim`).


.. _model_editing_defaults:

Class inheritance (defaults)
==============================
MuJoCo supports `default classes <https://mujoco.readthedocs.io/en/stable/XMLreference.html#default>`_,
which allow shared attribute values to be set in one place and then inherited by multiple elements.
In MuJoCo-rs, default classes can be created with
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>add_default`.

A new class is created by providing its ``class_name`` and optionally a ``parent_class_name``
(for class inheritance). If no parent is given, the class inherits from the main default.

.. code-block:: rust

    fn main() {
        let mut spec = MjSpec::new();

        // Create a top-level default class named "red".
        spec.add_default("red", None).expect("failed to create default");

        // Create a child class "small-red" that inherits from "red".
        spec.add_default("small-red", Some("red")).expect("failed to create default");
    }

Elements can then reference the class via their ``childclass`` or ``class`` (``dclass``) attribute
in the model XML, or by setting the ``childclass`` string on the relevant ``Mjs*`` struct.


    
Iterators
================
Since MuJoCo-rs 1.5.0, it is possible to also iterate existing :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
items (geoms, joints, etc.). Iterators exist on :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`.

To iterate over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec` items, call
``[item_type]_iter`` for immutable iteration or ``[item_type]_iter_mut`` for mutable iteration,
with ``[item_type]`` replaced by geom, body, etc.

.. code-block:: rust

    ...
    for body in spec.body_iter() {  // spec is MjSpec.
        println!("{}", body.name());
    }
    ...

Iteration over :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody` items can be used in a similar way.
The only difference is an additional boolean parameter, which enables recursive iteration when ``true``.

.. code-block:: rust

    ...
    // Iterate top level bodies of body.
    for body in body.body_iter(false) {  // body is MjsBody.
        println!("{}", body.name());
    }
    ...

.. code-block:: rust

    ...
    // Iterate top level bodies of body + their sub-bodies recursively.
    for body in body.body_iter(true) {  // body is MjsBody.
        println!("{}", body.name());
    }
    ...


Examples
================
Additional examples on model editing are
available in repository's examples:

- :gh-example:`Basic model editing <model_editing.rs>`
- :gh-example:`Terrain generation <terrain_generation.rs>`
