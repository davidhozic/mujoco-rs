
.. _scene_drawing:

=====================
Custom visualization
=====================

.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mj_viewer| replace:: :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`
.. |mj_renderer| replace:: :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`


Aside to the true simulation state, MuJoCo provides ways to draw additional 3D geometries (geoms)
onto an existing 3D scene.

In MuJoCo-rs drawing is done through |mjv_scene|.
There are two things that expose a scene for drawing custom visual-only geoms:

- :ref:`mj_rust_viewer` (:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scene`).
- :ref:`mj_renderer` (:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>user_scene`).


Drawing to a scene
===================


:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>add_geom`, which will
add a new geom to the scene.



.. image:: ../../img/visualization-example.png

