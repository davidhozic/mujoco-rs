

.. _mj_renderer:

3D renderer
============

Unlike the :ref:`mj_rust_viewer`, which displays the simulation's 3D scene onto a window,
the renderer exists to provide users the ability to render offscreen. This includes
rendering RGB and depth images to either an array or to a file.

The renderer can be constructed by its builder (:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>builder`)
or directly with :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`, however the latter offers less control.
method. By default, RGB rendering is enabled, while depth rendering is disabled.

.. code-block:: rust
    
    /* Build the renderer */.
    let mut renderer = MjRenderer::builder()
        .width(0).height(0)  // set to width(0) and height(0) to set automatically based on <global offwidth="1920" offheight="1080"/>
        .num_visual_user_geom(5)  // maximum number of visual-only geoms as result of the user
        .num_visual_internal_geom(0)  // maximum number of visual-only geoms not as result of the user
        .font_scale(MjtFontScale::mjFONTSCALE_100)  // scale of the font drawn by OpenGL
        .rgb(true)  // rgb rendering
        .depth(true)  // depth rendering
        .camera(MjvCamera::default())  // default free camera
        .build(&model).expect("failed to initialize the renderer");


.. attention::

    Renderer's *width* and *depth* must be equal to or less than the offscreen buffer size,
    configured during MuJoCo's model definition:

    .. code-block:: xml

        <mujoco>
            <visual>
                <global offwidth="1920" offheight="1080"/>
            </visual>
            ...
        </mujoco>


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
