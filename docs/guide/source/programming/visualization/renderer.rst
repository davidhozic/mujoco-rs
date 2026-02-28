

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
    
    /* Build the renderer */
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

    Renderer's *width* and *height* must be equal to or less than the offscreen buffer size,
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
a reference to the rendered image in the correct 2D shape. The shape must be specified via
the method's const generic parameters (``WIDTH`` and ``HEIGHT``), and the methods return
``io::Result`` — an error is returned if the requested dimensions don't match the renderer's
actual resolution.

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb_flat` and
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth_flat` can be used to obtain
a flat 1-D slice of the rendered data instead, returning ``Option<&[u8]>`` and ``Option<&[f32]>``
respectively. These return ``None`` when the corresponding rendering mode (``rgb`` or ``depth``)
was not enabled during construction.

To save rendered images to a file, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`
and :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` can be used.
These encode the image as an uncompressed PNG, where the RGB image is 8 bits per channel and
the depth image is 16 bits. These are meant **for visualization**.

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` accepts a ``normalize: bool``
argument. When ``true``, depth values are linearly normalized from their raw range to the full
16-bit range (0-65535) before saving. When ``false``, raw values are clamped and directly
encoded, which is only meaningful when the depth range happens to fall within 0-1.

To save depth data as raw 32-bit float values representing **actual metric distances** from the
camera, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw` can be used.



End-to-end example
------------------
The following is a complete example that loads a model, runs one step, renders it,
and saves both an RGB and a depth image:

.. code-block:: rust

    use mujoco_rs::prelude::*;
    use mujoco_rs::renderer::MjRenderer;

    const MODEL: &str = r#"
    <mujoco>
      <visual>
        <global offwidth="1920" offheight="1080"/>
      </visual>
      <worldbody>
        <light pos="0 0 3"/>
        <body name="ball" pos="0 0 1">
            <geom type="sphere" size=".1" rgba="0 1 0 1"/>
            <joint type="free"/>
        </body>
        <geom name="floor" type="plane" size="5 5 1"/>
      </worldbody>
    </mujoco>
    "#;

    fn main() {
        let model = MjModel::from_xml_string(MODEL).expect("could not load model");
        let mut data = MjData::new(&model);

        let mut renderer = MjRenderer::builder()
            .width(0).height(0)          // automatically sized from <global offwidth/offheight>
            .rgb(true)
            .depth(true)
            .camera(MjvCamera::default())
            .build(&model).expect("failed to initialize renderer");

        data.step();
        renderer.sync(&mut data);

        renderer.save_rgb("frame.png").expect("failed to save RGB");
        renderer.save_depth("depth.png", true).expect("failed to save depth");  // true = normalize to 0-65535 range.
    }

