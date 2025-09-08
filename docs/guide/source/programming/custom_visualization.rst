.. _custom_visualization:

=====================
Custom visualization
=====================

.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`

Aside to the true simulation state, MuJoCo provides ways to draw additional 3D geometries (geoms)
onto an existing 3D scene.

MuJoCo-rs tries to provide a higher-level API to drawing custom geoms.
All the visualization definitions are available under the :docs-rs:`mujoco_rs::wrappers::mj_visualization` module.
While the module contains many visualization related definitions, the one we're looking for is |mjv_scene|.

The scene can be directly constructed with
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new`, however this is currently
not useful on its own. In the future, MuJoCo-rs wil support a custom renderer, like the MuJoCo Python supports,
where |mjv_scene| will be used as the
drawing reference. Currently, the scene can be directly used to draw custom geometries to the :ref:`mj_rust_viewer`.



Using the scene
===================
The main two methods that will need to be called are
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>clear_geom`, which
will remove all the geoms from the scene, and
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>add_geom`, which will
add a new geom to the scene.

Drawing to the viewer
-----------------------
To draw to the 3D viewer, we will use the :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn_mut`
method, which will give as a mutable reference to the internal user scene.
The user scene has a limited amount of allowed geoms, which can be configured by the user's
application in the :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive` method.

The user scene is empty at beginning. It is also only cleared after calling 
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>clear_geom`,
which allows the viewer to be synced at higher frequencies than the frequency the user scene is
updated at.

To draw to the scene, the method :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>add_geom`
can be used to create the geom and then obtain a mutable reference to it, for purposes of additional modification.

This `example <https://github.com/davidhozic/mujoco-rs/blob/main/examples/drawing_scene.rs>`_
illustrates how to draw a line between two independent balls.
In the example, we also call :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>connect`,
which scales, orients and positions the geom, in order to make it point from point to the other.
The example will draw following white line as shown in the picture:

.. image:: ../img/visualization-example.png
