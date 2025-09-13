

.. _mj_renderer:

3D renderer
============

Unlike the :ref:`mj_rust_viewer`, which displays the simulation's 3D scene onto a window,
the renderer exists to provide users the ability to render offscreen. This includes
rendering RGB and depth images to either an array or to a file.

The renderer can be constructed via the :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`
method. The method accepts a reference to :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`,
two parameters representing the dimensions of rendered images, and the maximum number of visual-only geoms,
drawn by the user program.

After construction, additional options can be set using ``with_`` methods,
like shown below. These follow the builder pattern.
By default, RGB rendering is enabled, while depth rendering is disabled.

.. code-block:: rust
    
    /* Construct the renderer (model, width, height, max_geom) */.
    let mut renderer = MjRenderer::new(&model, 1280, 720, 5).unwrap()
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
a reference of the rendered image in correct shape, which needs to be specified via method's const generic parameters,
or :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb_flat` and
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth_flat` can be used to obtain
a flattened images.

To save rendered images to a file, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`
and :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` can be used.
These will encode the image into a uncompressed PNG format, where the RGB image will be 8 bits and
the depth will be 16 bits. These are meant **for visualization**.
To save depth data in float-32 bit format, which represents **actual distance values** from the camera,
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw` can be used.
