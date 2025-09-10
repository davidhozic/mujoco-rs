.. _visualization:

========================
Visualization
========================
It's useful to visualize what the simulation is doing.
MuJoCo-rs provides two main ways of visualization:

- :ref:`mj_rust_viewer` for onscreen visualization (window)
- :ref:`mj_renderer` for offscreen rendering (array or a file)

.. _mj_rust_viewer:


3D viewer
=======================

MuJoCo-rs also provides a 3D viewer, written in Rust. It is written to be analog to the C++ viewer (the Simulate UI),
but available without C++ dependencies.

The viewer supports visualization of the 3D scene, as well as interaction via mouse and keyboard.
This also includes object perturbations.
Currently, no UI is provided (buttons, drop-downs, etc.), however it is planned for future development.

A screenshot of the Rust 3D viewer is shown below.

.. figure:: ../../../../img_common/viewer_spot.png

    Rust-native interactive 3D viewer.
    Showing the `Spot <https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot>`_ scene from
    `MuJoCo's menagerie <https://mujoco.readthedocs.io/en/stable/models.html>`_.

The viewer can be launched only in **passive mode**, i. e. it won't run as a separate application,
and needs to be periodically "synced" by the user application.
The user application is the one that needs to run the actual physical simulation, like shown in
:ref:`basic_sim` and also below.

The viewer can be launched with :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive`,
like shown in the following example:

.. code-block:: rust
    :emphasize-lines: 8

    fn main() {
        /* Initiate the physics simulation */
        let model = MjModel::from_xml("path/to/model.xml").expect("could not load the model");
        let mut data = model.make_data();
        let timestep = model.ffi().opt.timestep;

        /* Launch the viewer  */
        let mut viewer = MjViewer::launch_passive(&model, 100).expect("could not launch the viewer");
        while viewer.running() {
            /* Sync the simulation state with the viewer */
            viewer.sync(&mut data);

            /* Update the simulation state */
            data.step();
            std::thread::sleep(Duration::from_secs_f64(timestep));
        }
    }


The method :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive` accepts an immutable reference to
the simulation model (:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel`) and uses it to construct two scenes:

- the internal 3D scene needed for the actual viewer to mirror the simulation state and
- the user 3D scene that can be used to draw custom user objects (obtainable via
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn_mut`).
  This is in more detail described in :ref:`custom_visualization`.


The above example runs until the viewer is closed (:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>running`)
and mirrors/syncs the simulation state with :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync`.

.. code-block:: rust
    :emphasize-lines: 2, 4

    ...
    while viewer.running() {
        /* Sync the simulation state with the viewer */
        viewer.sync(&mut data);
        ...
    }

At the beginning, we also obtained the simulation timestep (time passed in simulation per each call to
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`), which is used to
sleep after the step with ``std::thread::sleep(Duration::from_secs_f64(timestep));``.
This is optional and can be removed or reduced to run the simulation faster than realtime.

Interaction with the viewer is described with a help menu, which is shown on launch of the viewer.
For more, refer to :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` and
`examples <https://github.com/davidhozic/mujoco-rs/tree/main/examples>`_.


.. _mj_renderer:

3D renderer
============

Unlike the :ref:`mj_rust_viewer`, which displays the simulation's 3D scene onto a window,
the renderer exists to provide users the ability to render offscreen. This includes
rendering RGB and depth images to either an array or into a file.


.. _custom_visualization:

Custom visualization
=====================

.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mj_viewer| replace:: :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`
.. |mj_renderer| replace:: :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`


Aside to the true simulation state, MuJoCo provides ways to draw additional 3D geometries (geoms)
onto an existing 3D scene.

MuJoCo-rs tries to provide a higher-level API to drawing custom geoms.
All the visualization definitions are available under the :docs-rs:`mujoco_rs::wrappers::mj_visualization` module.
While the module contains many visualization related definitions, the one we're looking for is |mjv_scene|.
It represents an abstraction to visually represent the 3D world of the simulation and additionally
draw custom visual-only geoms.

The scene can be directly constructed with
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new`, however this is
not useful on its own, unless implementing custom structs. MuJoCo-rs provides two places where the scene
is used and custom visualization can be done:

- the :ref:`mj_rust_viewer` (|mj_viewer|)
- and the renderer (|mj_renderer|).


Using the scene
--------------------
The main method that will need to be called is
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>add_geom`, which will
add a new geom to the scene.

.. _drawing_to_the_viewer:

Drawing to the viewer
~~~~~~~~~~~~~~~~~~~~~~~~
To draw to the scene inside the viewer, we will use the
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scene_mut`
method, which will give as a mutable reference to the internal user scene.
The user scene has a limited amount of allowed geoms, which can be configured by the user
in the :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive` method.

The user scene is empty at beginning. It is also only cleared after calling 
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>clear_geom`,
which allows the viewer to be synced at higher frequencies than the frequency the user scene is
updated at. This method **needs to be called** every time existing geoms are supposed to be re-rendered
or older ones removed, otherwise geoms will just be added on top of the other ones.

To draw to the scene, the method :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>add_geom`
can be used, which will create a geom inside the scene and then obtain a mutable reference to it,
for purposes of additional modification.

This `example <https://github.com/davidhozic/mujoco-rs/blob/main/examples/drawing_scene.rs>`_
illustrates how to draw a line between two independent balls.
In the example, we also call :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>connect`,
which scales, orients and positions the geom, in order to make it point from point to the other.
The example will draw following white line as shown in the picture:

.. image:: ../../img/visualization-example.png


Drawing to the renderer
~~~~~~~~~~~~~~~~~~~~~~~~~~~
Drawing to the renderer is exactly the same as in :ref:`drawing_to_the_viewer`.
The scene can be obtained with :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>user_scene_mut`.
The maximum number of geoms is specified in :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`.
