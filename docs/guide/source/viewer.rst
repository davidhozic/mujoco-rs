.. _mj_rust_viewer:

=======================
3D viewer
=======================

MuJoCo-rs also provides a 3D viewer, written in Rust. It is written to be analog to the C++ viewer (the Simulate UI),
but available without C++ dependencies.

The viewer supports visualization of the 3D scene, as well as interaction via mouse and keyboard.
This also includes object perturbations.
Currently, no UI is provided (buttons, drop-downs, etc.), however it is planned for future development.

A screenshot of the Rust 3D viewer is shown below.

.. figure:: ../../img_common/viewer_spot.png

    Rust-native interactive 3D viewer.
    Showing the `Spot <https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot>`_ scene from
    `MuJoCo's menagerie <https://mujoco.readthedocs.io/en/stable/models.html>`_.

The viewer can be launched only in **passive mode**, i. e. it won't run as a separate application,
and needs to be periodically "synced" by the user application.
The user application is the one that needs to run the actual physical simulation, like shown in
:ref:`basic_sim` and also below.

The viewer can be launched with the :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive`,
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
            std::thread::sleep(Duration::from_secs(timestep));
        }
    }


The method :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive` accepts an immutable reference to
the simulation model (:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel`) and uses it construct two scenes:

- the internal 3D scene needed for the actual viewer to mirror the simulation state and
- the user 3D scene that can be used to draw custom user objects (obtainable via
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn`).


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

At the beginning we also obtained the simulation timestep (time per each call to
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`), which is used to
sleep after the step with ``std::thread::sleep(Duration::from_secs(timestep));``
This is optional and can be removed or reduced to run the simulation time faster than realtime.

Interaction with the viewer is described with the help menu, which is open by default on launch.
For more, refer to :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` and
`examples <https://github.com/davidhozic/mujoco-rs/tree/main/examples>`_.
