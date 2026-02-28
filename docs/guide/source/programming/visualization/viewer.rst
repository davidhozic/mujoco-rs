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

The viewer can be launched in two ways:

- Via its **builder** (:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>builder`), which allows
  full control over the viewer's settings:

  .. code-block:: rust
      :emphasize-lines: 9-14

      use std::time::Duration;

      fn main() {
          /* Initiate the physics simulation */
          let model = MjModel::from_xml("path/to/model.xml").expect("could not load the model");
          let mut data = MjData::new(&model);
          let timestep = model.opt().timestep;

          /* Launch the viewer  */
          let mut viewer = MjViewer::builder()
              .window_name("My Simulation")    // text shown in the window title bar.
              .max_user_geoms(0)               // maximum additional geoms drawn by the user.
              .vsync(false)                    // vertical synchronization (use true when rendering in a separate thread).
              .warn_non_realtime(false)        // show an overlay when the simulation lags behind realtime.
              .build_passive(&model).expect("could not launch the viewer");
          while viewer.running() {
              /* Sync the simulation state with the viewer */
              viewer.sync_data(&mut data);
              viewer.render();

              /* Update the simulation state */
              data.step();
              std::thread::sleep(Duration::from_secs_f64(timestep));  // wait approximately timestep seconds
          }
      }

- Via the :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive` method, a
  convenient shorthand for users who want the **default settings**:

  .. code-block:: rust
      :emphasize-lines: 9

      use std::time::Duration;

      fn main() {
          let model = MjModel::from_xml("path/to/model.xml").expect("could not load the model");
          let mut data = MjData::new(&model);
          let timestep = model.opt().timestep;

          let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch the viewer");
          while viewer.running() {
              viewer.sync_data(&mut data);
              viewer.render();
              data.step();
              std::thread::sleep(Duration::from_secs_f64(timestep));
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

.. note::

    :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data` copies only the subset of
    :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` needed for rendering (positions, velocities, contacts,
    etc.), which is faster.
    :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data_full` copies the **entire**
    ``MjData`` struct — including large Jacobian arrays — and should only be used when those arrays
    are needed inside the viewer (for example, when using
    :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` to access them).

.. admonition:: Performance tip

    Rust viewer contains the so called shared state (:docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`),
    which exists to allow partial multi-threading, without locking the entire viewer.

    Methods that operate on the shared state,
    such as:

    - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`;
    - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>running`;
    - etc.;

    internally acquire a mutex lock to the shared state.
    Sequential calls to more than one of these can consequently
    hurt performance.

    A more optimized way to use these methods is to call their equivalents on the shared
    state directly. The shared state can be accessed mainly through:

    - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>state`, which returns
      an ``Arc<Mutex<ViewerSharedState>>`` --- see :ref:`multithreading-rs-viewer`;
    - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>with_state_lock`,
      which accepts a function/closure to call. The function/closure receives a
      `MutexGuard <https://doc.rust-lang.org/std/sync/struct.MutexGuard.html>`_
      to the shared state.

    Using ``with_state_lock`` groups multiple shared-state operations under a **single** lock
    acquisition, which avoids repeated lock/unlock overhead:

    .. code-block:: rust

        viewer.with_state_lock(|mut lock| {
            lock.sync_data(&mut data);  // both calls share one lock
            viewer_running = lock.running();
        }).unwrap();



.. _multithreading-rs-viewer:

Multi-threading
----------------
Above example shows how to use the viewer synchronously to the simulation loop.
This can slow down the simulation as :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render`
is relatively expensive to call. Additionally, usage synchronous to simulation causes
the refresh rate to be equal to the simulation stepping frequency, which puts strain to the GPU.

To prevent slowdowns and allow V-Sync, the viewer can run in the **main thread**, whilst
the actual physics simulation runs in another.

Here's a cutout from the :gh-example:`example <rust_viewer_threaded.rs>` on how to use the viewer in a multi-threaded way:

.. code-block:: rust
    :emphasize-lines: 1, 11-13, 18-20, 30-34

    let model = Arc::new(MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model"));
    let mut data = MjData::new(model.clone());

    // Create the viewer, bound to the model.
    let mut viewer = MjViewer::builder()
        .window_name("My Threaded Simulation")  // text shown in the window title bar.
        .max_user_geoms(100)
        .vsync(true)  // let the viewer select the appropriate refresh rate.
        .build_passive(model.clone())
        .expect("could not launch the viewer");

    let shared_state = viewer.state().clone();
    let mut viewer_running = shared_state.lock().unwrap().running();  // gets moved into the thread
    let physics_thread = std::thread::spawn(move || {
        while viewer_running {
            let timer = Instant::now();
            data.step();
            {
                let mut lock = shared_state.lock().unwrap();
                lock.sync_data(&mut data);
                viewer_running = lock.running();
            }

            // Use a while loop and polling to wait for accuracy purposes.
            // To increase performance, std::thread::sleep may be used,
            // however that comes at the cost of less accuracy.
            while timer.elapsed().as_secs_f64() < model.opt().timestep {}
        }
    });

    while viewer.running() {
        viewer.render();
    }

    physics_thread.join().unwrap();


The example mainly differs from the synchronous one in the highlighted lines:

- :docs-rs:`mujoco_rs::wrappers::mj_model::<struct>MjModel` is wrapped into
  `Arc <https://doc.rust-lang.org/std/sync/struct.Arc.html>`_,
- Data is synced through :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_data`;
  
  - :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState` is obtained through
    :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>state`, which returns
    ``Arc<Mutex<ViewerSharedState>>``,
  - both calls to ``sync_data`` and ``running`` are grouped inside a single ``lock()`` to
    avoid two separate mutex acquisitions (see the performance tip above).



.. _custom_ui_widgets:

Custom UI widgets
------------------
The Rust-native viewer supports adding custom UI widgets through the
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` method.
This allows you to create custom windows, panels, and other UI elements using
`egui <https://docs.rs/egui/latest/egui/>`_.

.. note::

    Callbacks added via :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`
    receive the passive simulation state (:docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`).
    This requires locking the mutex to the shared state, which may slow down the program.

    To avoid unnecessary locks when the simulation state is not required in the UI,
    :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback_detached`
    can be used instead, which only accepts the
    `egui::Context <https://docs.rs/egui/0.33.0/egui/struct.Context.html>`_ as parameter.


The following example demonstrates how to add a custom window to the viewer:

.. code-block:: rust
    :emphasize-lines: 8-19

    fn main() {
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
        let mut data = MjData::new(&model);
        let mut viewer = MjViewer::builder()
            .max_user_geoms(0)
            .build_passive(&model).expect("could not launch the viewer");

        /* Add a custom UI window */
        // viewer.add_ui_callback(|ctx, data| {...}) or
        viewer.add_ui_callback_detached(|ctx| {
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
    the entire :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` struct at the expense of performance.


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
            viewer.render();
            data.step();
            std::thread::sleep(Duration::from_secs_f64(step));
        }
    }


Compared to the Rust-native viewer, the C++ wrapper doesn't take a ``data`` parameter to the :docs-rs:`~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync`
method. Additionally, a call to :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render`
is required.
