.. _mj_rust_viewer:

=======================
3D viewer
=======================
MuJoCo provides an official viewer application, written in C++, which can also be used in MuJoCo's
Python package. To avoid C++ dependencies, MuJoCo-rs provides **its own 3D viewer, written in Rust**.

We also provide the ability to use the official C++ based viewer, however this requires
static linking to modified MuJoCo code, as described in :ref:`static_linking`.

.. _rust_native_viewer:

Rust-native 3D viewer
=======================

Rust-native 3D viewer supports visualization of the 3D scene, as well as interaction via mouse and keyboard.
This also includes object perturbations. Optionally, enabled by the ``viewer-ui`` feature (default), the viewer
also provides a user interface, which tries to replicate the original C++ viewer as best as possible
and thus allows control of constraints, joints, actuators, etc.

A screenshot of the Rust 3D viewer is shown below.

.. figure:: ../../../../img_common/viewer_spot.png

    Rust-native interactive 3D viewer.
    Showing the `Spot <https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot>`_ scene from
    `MuJoCo's menagerie <https://mujoco.readthedocs.io/en/stable/models.html>`_.

The viewer can be launched only in **passive mode**, i.e. it won't run as a separate application,
and needs to be periodically "synced" by the user application.
The user application is the one that needs to run the actual physics simulation, like shown in
:ref:`basic_sim` and also below.

The viewer can be launched with :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive`,
like shown in the following example:

.. code-block:: rust
    :emphasize-lines: 8

    fn main() {
        /* Initiate the physics simulation */
        let model = MjModel::from_xml("path/to/model.xml").expect("could not load the model");
        let mut data = MjData::new(&model);
        let timestep = model.opt().timestep;

        /* Launch the viewer  */
        let mut viewer = MjViewer::launch_passive(&model, 100).expect("could not launch the viewer");
        while viewer.running() {
            /* Sync the simulation state with the viewer */
            viewer.sync_data(&mut data);
            viewer.render();

            /* Update the simulation state */
            data.step();
            std::thread::sleep(Duration::from_secs_f64(timestep));  // wait approximately timestep seconds
        }
    }


The above example runs until the viewer is closed (:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>running`)
and mirrors/syncs the simulation state with :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`.
After or parallel to synchronization, the viewer must also be rendered using the
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` method.

.. code-block:: rust
    :emphasize-lines: 2, 4, 5

    ...
    while viewer.running() {
        /* Sync the simulation state with the viewer */
        viewer.sync_data(&mut data);
        viewer.render();
        ...
    }

At the beginning, we also obtained the simulation timestep (time passed in simulation per each call to
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>step`), which is used to
sleep after the step with ``std::thread::sleep(Duration::from_secs_f64(timestep));``.
This is optional and can be removed or reduced to run the simulation faster than realtime.

.. note::

    The ``sleep()`` function is not accurate. For accurate timing,
    use `std::time::Instant <https://doc.rust-lang.org/std/time/struct.Instant.html>`_ to poll the elapsed time.

For more, refer to :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` and
`examples <https://github.com/davidhozic/mujoco-rs/tree/main/examples>`_.


.. _custom_ui_widgets:

Custom UI widgets
------------------
The Rust-native viewer supports adding custom UI widgets through the
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` method.
This allows you to create custom windows, panels, and other UI elements using
`egui <https://docs.rs/egui/latest/egui/>`_.

The following example demonstrates how to add a custom window to the viewer:

.. code-block:: rust
    :emphasize-lines: 8-19

    fn main() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
        let mut data = MjData::new(&model);
        let mut viewer = MjViewer::launch_passive(&model, 100)
            .expect("could not launch the viewer");

        /* Add a custom UI window */
        viewer.add_ui_callback(|ctx, data| {
            use mujoco_rs::viewer::egui;
            egui::Window::new("Custom controls")
                .scroll(true)
                .show(ctx, |ui| {
                    ui.heading("My Custom Widget");
                    ui.label("This is a custom UI element!");
                    if ui.button("Click me").clicked() {
                        println!("Button clicked!");
                    }
                });
        });

        while viewer.running() {
            viewer.sync_data(&mut data);
            viewer.render();
            data.step();
            std::thread::sleep(Duration::from_millis(2));
        }
    }

Multiple callbacks can be registered by calling ``add_ui_callback`` multiple times.
Each callback will be invoked during the UI rendering phase with access to the egui context.

For a comprehensive example, see the :gh-example:`custom_ui_widgets.rs` example,
which demonstrates various types of UI elements including windows, side panels, and top panels.

.. note::

    Custom UI widgets are only available when the ``viewer-ui`` feature is enabled (default).
    The ``egui`` crate is re-exported from ``mujoco_rs::viewer::egui`` for convenience.


.. attention::

    For performance reasons, when :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`
    is called, the viewer only syncs the state required for visualization --- i.e., it skips
    some large arrays. As a result, the :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` passed to
    the callback (added via :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`)
    may contain outdated information.

    The following are **NOT SYNCHRONIZED** when using :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`:

    - Jacobian matrices;
    - mass matrices.

    If you require those, make sure to call an appropriate method/function on the
    passed :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` instance
    (e.g., :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>forward`).

    Instead of :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`,
    users can call :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data_full`, which will copy
    the entire :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` struct at the expanse of performance.

.. _mj_cpp_viewer:

Wrapper of MuJoCo's C++ 3D viewer
=====================================
MuJoCo-rs also provides a wrapper around a modified MuJoCo's C++ 3D viewer.
Modifications to the C++ viewer are minor with the purpose of preserving future compatibility.
The changes to the viewer are made to allow viewer rendering in a user-controller loop.

.. attention::

    To avoid a major rewrite of the C++ viewer,  
    the latter is given raw, mutable pointers to both :docs-rs:`mujoco_rs::mujoco_c::<type>mjModel`  
    and :docs-rs:`mujoco_rs::mujoco_c::<type>mjData`, which are wrapped inside  
    :docs-rs:`mujoco_rs::wrappers::mj_model::<struct>MjModel`  
    and :docs-rs:`mujoco_rs::wrappers::mj_data::<struct>MjData`, respectively.  
    As a result, Rust's borrow-checker rules are violated. Although incorrect behavior is unlikely,  
    caution is advised.

    It is strongly **recommended** to use the :ref:`rust_native_viewer` when none of the  
    C++ viewer's features are required.

Here is an example of using the C++ wrapper:

.. code-block:: rust

    fn main() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
        let mut data = MjData::new(&model);
        let mut viewer = MjViewerCpp::launch_passive(&model, &data, 100);
        let step = model.opt().timestep;
        while viewer.running() {
            viewer.sync();
            viewer.render(true);  // render on screen   and update the fps timer
            data.step();
            std::thread::sleep(Duration::from_secs_f64(step));
        }
    }


Compared to the Rust-native viewer, the C++ wrapper doesn't take a ``data`` parameter to the :docs-rs:`~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync`
method. Additionally, a call to :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render`
is required.
