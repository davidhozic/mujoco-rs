.. _mj_renderer:

3D renderer
============

.. note::

    The renderer requires the ``renderer`` Cargo feature (not enabled by default).
    Enable it with: ``cargo add mujoco-rs --features renderer``
    (add ``renderer-winit-fallback`` on Windows/macOS or when offscreen rendering is unavailable).

Unlike the :ref:`mj_rust_viewer`, which displays the simulation's 3D scene in a window,
the renderer exists to provide users with the ability to render offscreen. This includes
rendering RGB and depth images to either an array or to a file.

The renderer can be constructed by its builder (:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>builder`)
or directly with :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`; however, the latter offers less control.
By default, RGB rendering is enabled, while depth rendering is disabled.

.. code-block:: rust

    /* Build the renderer */
    let mut renderer = MjRenderer::builder()
        .width(0).height(0)  // set to width(0) and height(0) to set automatically based on <global offwidth="1920" offheight="1080"/>
        .num_visual_user_geom(5)  // maximum number of visual-only geoms as a result of the user
        .num_visual_internal_geom(0)  // maximum number of visual-only geoms not as a result of the user
        .font_scale(MjtFontScale::mjFONTSCALE_100)  // scale of the font drawn by OpenGL
        .rgb(true)  // rgb rendering
        .depth(true)  // depth rendering
        .camera(MjvCamera::default())  // default free camera
        .build(&model).expect("failed to initialize the renderer");


.. attention::

    The renderer's *width* and *height* must be equal to or less than the offscreen buffer size,
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
    renderer.sync(&mut data).unwrap();
    ...

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync` is an infallible convenience method
that panics on failure (it internally delegates to
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` with ``expect``).
Use ``try_sync()`` when you need fallible error handling.


.. note::

    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync` is similar to the MuJoCo
    Python ``Renderer.update_scene`` method. Unlike the MuJoCo Python method,
    MuJoCo-rs's implementation also performs
    rendering. This was done to make :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` behave
    closer to the :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`.

After syncing, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb` and
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth` can be used to obtain
a reference to the rendered image in the correct 2D shape. The shape must be specified via
the method's const generic parameters (``WIDTH`` and ``HEIGHT``), and the methods return
``Result<_, RendererError>`` --- an error is returned if the requested dimensions don't match the renderer's
actual resolution, or when the corresponding rendering mode (RGB/depth) is currently disabled.

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb_flat` and
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth_flat` can be used to obtain
a flat 1-D slice of the rendered data instead, returning ``Option<&[u8]>`` and ``Option<&[f32]>``
respectively. These return ``None`` when the corresponding rendering mode (``rgb`` or ``depth``)
is currently disabled (either disabled at construction time or toggled off later).

To save rendered images to a file, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`
and :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` can be used.
These encode the image as a PNG, where the RGB image is 8 bits per channel and
the depth image is 16 bits. These are meant **for visualization**.

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth` accepts a ``normalize: bool``
argument. When ``true``, depth values are linearly normalized using per-frame min-max
normalization to the full 16-bit range (0-65535) before saving. When ``false``, depth values are
linearly mapped using the model's camera near/far clip planes as a fixed range, providing a
frame-independent mapping to 0-65535. In both cases, the method returns the ``(min, max)`` pair
used for normalization, allowing depth values to be approximately reconstructed
(subject to 16-bit quantization and clamping).

To save depth data as raw 32-bit float values representing **actual metric distances** from the
camera, :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw` can be used.

The PNG compression level used by ``save_rgb`` and ``save_depth`` can be controlled via
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>set_png_compression` or the
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRendererBuilder::<method>png_compression`
builder setter. The default is ``png::Compression::NoCompression`` (fastest encoding, largest files).
Use ``png::Compression::Balanced`` or ``png::Compression::High`` to trade encoding time for
smaller files.


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

