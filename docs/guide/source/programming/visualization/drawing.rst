
.. _scene_drawing:

=====================
Custom visualization
=====================

.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`


Aside to the true simulation state, MuJoCo provides ways to draw additional 3D geometries (geoms)
onto an existing 3D scene.

In MuJoCo-rs drawing is done through |mjv_scene|.
There are two things that expose a scene for drawing custom visual-only geoms:

- :ref:`mj_rust_viewer` (:docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>user_scene`).
- :ref:`mj_renderer` (:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>user_scene`).


Drawing to a scene
===================
To draw custom geoms to a scene inside a viewer or a renderer, applications need to alternate between calls to two methods:

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>clear_geom`, which will
  clear all the existing geoms.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom`, which will
  add a new geom to the scene.

This :gh-example:`example <drawing_scene_viewer.rs>` shows how to draw a line between two balls.

In the example, we start drawing by first obtaining a mutable reference to the user scene and clearing
its geoms, which are otherwise preserved between syncs:


.. code-block:: rust

    viewer.with_state_lock(|mut state_lock| {
        let scene = state_lock.user_scene_mut();  // obtain a mutable reference to the user scene.
        scene.clear_geom();  // clear existing geoms
    }).unwrap();

We then initialize a new geom. We make it a line (``MjtGeom::mjGEOM_LINE``) and give it a pure white
color (``Some([1.0, 1.0, 1.0, 1.0])``). We leave other fields at ``None``, as they are not needed
at this stage.


.. code-block:: rust
    :emphasize-lines: 4-10

    viewer.with_state_lock(|mut state_lock| {
        let scene = state_lock.user_scene_mut();  // obtain a mutable reference to the user scene.
        scene.clear_geom();  // clear existing geoms

        let new_geom = scene.create_geom(
            MjtGeom::mjGEOM_LINE,  // type of geom to draw.
            None,  // size, ignore here as we set it below.
            None,   // position: ignore here as we set it below.
            None,   // rotational matrix: ignore here as we set it below.
            Some([1.0, 1.0, 1.0, 1.0])  // color (rgba): pure white.
        );
    }).unwrap();


In the above snippet, defining the fields that we've set to None would work, making this the final step.
However, we would need to know their correct values.
We obtain the needed values using :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>connect`,
which calculates the values to result in the geom pointing from one point to another.


.. code-block:: rust
    :emphasize-lines: 19-23

    viewer.with_state_lock(|mut state_lock| {
        let scene = state_lock.user_scene_mut();  // obtain a mutable reference to the user scene.
        scene.clear_geom();  // clear existing geoms

        let new_geom = scene.create_geom(
            MjtGeom::mjGEOM_LINE,  // type of geom to draw.
            None,  // size, ignore here as we set it below.
            None,  // position: ignore here as we set it below.
            None,  // rotational matrix: ignore here as we set it below.
            Some([1.0, 1.0, 1.0, 1.0])  // color (rgba): pure white.
        );

        /* Read X, Y and Z coordinates of both balls. */
        let ball1_position = ball1_joint_info.view(&data).qpos[..3]
            .try_into().unwrap();
        let ball2_position = ball2_joint_info.view(&data).qpos[..3]
            .try_into().unwrap();

        /* Modify the visual geom's position, orientation and length, to connect the balls */
        new_geom.connect(
            0.0,            // width
            ball1_position, // from
            ball2_position  //  to
        );
    }).unwrap();



The following image shows the result of the above :gh-example:`example <drawing_scene_viewer.rs>`.

.. image:: ../../img/visualization-example.png

