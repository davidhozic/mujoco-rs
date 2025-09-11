

.. _mj_renderer:

3D renderer
============

Unlike the :ref:`mj_rust_viewer`, which displays the simulation's 3D scene onto a window,
the renderer exists to provide users the ability to render offscreen. This includes
rendering RGB and depth images to either an array or to a file.

The renderer can be constructed via the :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`
method. The method accepts a reference to :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
and the two parameters representing the dimensions of rendered images.

After construction, additional options can be set using ``with_`` methods,
like shown below. By default, RGB rendering is enabled, while depth rendering is disabled.

.. code-block:: rust

    let mut renderer: MjRenderer<1280, 720> = MjRenderer::new(&model, 5).unwrap()
        .with_font_scale(MjtFontScale::mjFONTSCALE_100)
        .with_opts(MjvOption::default())
        .with_rgb_rendering(true)  // enabled by default.
        .with_depth_rendering(false)  // disabled by default.
        .with_camera(MjvCamera::new_free(&model));

Much like the viewer, the renderer must also be synced with the simulation state,
using :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync`.

.. code-block:: rust
    :emphasize-lines: 3

    ...
    data.step();  // data is an instance of MjData.
    renderer.sync(&mut data);
    ...


.. note::

    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync` is similar to the MuJoco
    Python's ``Renderer.update_scene`` method. Unlike the MuJoCo Python's method,
    MuJoCo-rs's implementation also performs
    rendering. This was done to make :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` behave
    closer to the :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`.

After syncing, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb` and
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth` can be used to obtain
a reference of the rendered image.

To save rendered images to a file, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`
and :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` can be used.
