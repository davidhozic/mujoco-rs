.. _migrate_6_0_0:

Migrating to 6.0.0
======================


MuJoCo upgrade
-----------------------

This release links against MuJoCo **3.12.0**. Download the matching release and
update your library path. See :ref:`installation` for details.

The FFI bindings follow the upstream API. This page gives no entry for an accessor that only
mirrors a renamed, a removed or a resized MuJoCo field, and none for a shifted enum discriminant.
For these changes, see MuJoCo's changelog for this release (and prior changelogs since MuJoCo 3.10.0+):
https://mujoco.readthedocs.io/en/3.12.0/changelog.html


``SignatureMismatch`` is renamed to ``IncompatibleModel``
---------------------------------------------------------
The variant is renamed in :docs-rs:`~mujoco_rs::error::<enum>MjDataError`,
:docs-rs:`~mujoco_rs::error::<enum>MjModelError`,
:docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError` and
:docs-rs:`~mujoco_rs::renderer::<enum>RendererError`. It now also fires for two models that share
a signature but need different buffer sizes, so a program that treated an equal signature as
"safe to pair" must handle the error on that path too.

**Before:**

.. code-block:: rust

    match data.copy_to(&mut destination) {
        Err(MjDataError::SignatureMismatch { source, destination }) => { /* ... */ }
        other => other.unwrap(),
    }

**After:**

.. code-block:: rust

    match data.copy_to(&mut destination) {
        Err(MjDataError::IncompatibleModel { source, destination }) => { /* ... */ }
        other => other.unwrap(),
    }

A test of your own that compared signatures directly should call
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`, or
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
for a scene.

**Before:**

.. code-block:: rust

    if model.signature() == other.signature() {
        data.swap_model(other);
    }

**After:**

.. code-block:: rust

    if model.is_compatible_with_model(&other) {
        data.swap_model(other);
    }


``MjvCamera::move_`` no longer takes a scene
--------------------------------------------
The ``scene`` parameter was dropped from |mjv_camera| ``move_``.

**Before:**

.. code-block:: rust

    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0, renderer.scene());

**After:**

.. code-block:: rust

    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0);


MIMO-aware actuator dimensions
------------------------------
Actuators now have separate control and output dimensions.

**Before:**

.. code-block:: rust

    for id in 0..model.nu() as usize {
        let trntype = model.actuator_trntype()[id];
        let gear = model.actuator_gear()[id];
        let range = model.actuator_ctrlrange()[id];
    }

**After:**

.. code-block:: rust

    for id in 0..model.nactuator() as usize {
        let out = model.actuator_outadr()[id] as usize;
        let ctrl = model.actuator_ctrladr()[id] as usize;

        let trntype = model.actuator_trntype()[id];    // actuator-indexed
        let gear = model.actuator_gear()[out];         // output-indexed
        let range = model.actuator_ctrlrange()[ctrl];  // control-indexed
    }

The |mj_data| control-history methods follow the same rule: ``read_ctrl``, ``try_read_ctrl`` and
``init_ctrl_history`` now take an actuator index bounded by ``nactuator``, not a control index
bounded by ``nu``.


Stricter ``MjrContext`` accessor types
--------------------------------------
The |mjr_context| flags ``gl_initialized``, ``window_available``, ``window_stereo`` and
``window_doublebuffer`` now read as ``bool``, and ``font_scale`` and ``read_depth_map`` now read
as ``MjtFontScale`` and ``MjtDepthMap``, instead of ``i32``.

**Before:**

.. code-block:: rust

    if context.gl_initialized() != 0 && context.window_doublebuffer() != 0 {
        let scale: i32 = context.font_scale();
        let depth: i32 = context.read_depth_map();
    }

**After:**

.. code-block:: rust

    if context.gl_initialized() && context.window_doublebuffer() {
        let scale: MjtFontScale = context.font_scale();
        let depth: MjtDepthMap = context.read_depth_map();
    }


``MjrContext::set_buffer`` takes ``MjtFramebuffer``
----------------------------------------------------
``set_buffer`` now takes ``MjtFramebuffer``.

**Before:**

.. code-block:: rust

    context.set_buffer(MjtFramebuffer::mjFB_OFFSCREEN as i32);

**After:**

.. code-block:: rust

    context.set_buffer(MjtFramebuffer::mjFB_OFFSCREEN);


Unsafe mutation of ``MjModel`` ``actuator_gaintype``
----------------------------------------------------
``actuator_gaintype_mut`` is now an ``unsafe fn``; the actuator views are affected the same way,
so a write through a view needs ``as_mut_slice`` inside ``unsafe``. A wrong gain type makes the
engine write more force outputs than the actuator owns.

**Before:**

.. code-block:: rust

    model.actuator_gaintype_mut()[id] = MjtGain::mjGAIN_AFFINE;
    view.gaintype[0] = MjtGain::mjGAIN_AFFINE;

**After:**

.. code-block:: rust

    // SAFETY: mjGAIN_AFFINE writes exactly one force output, which this actuator owns.
    unsafe { model.actuator_gaintype_mut()[id] = MjtGain::mjGAIN_AFFINE };
    unsafe { view.gaintype.as_mut_slice()[0] = MjtGain::mjGAIN_AFFINE };


Unsafe geom creation on ``MjvScene``
------------------------------------
``create_geom`` and ``try_create_geom`` are now ``unsafe fn``. The caller keeps ``matid`` and
``texid`` below the counts of the model that the rendering context was created for, and ``objid``
below the scene's flex or skin count. All three ids start at ``-1``, which means no material, no
texture and no object.

**Before:**

.. code-block:: rust

    let geom = scene.create_geom(MjtGeom::mjGEOM_BOX, None, None, None, None);

**After:**

.. code-block:: rust

    // SAFETY: the geom keeps the ids that create_geom sets to none.
    let geom = unsafe { scene.create_geom(MjtGeom::mjGEOM_BOX, None, None, None, None) };


Renamed ``DcMotorConfig`` input field
---------------------------------------
The public field ``DcMotorConfig::input_mode`` is now ``ctrlspec`` and the builder
``with_input_mode`` is now ``with_ctrlspec``. The value is a bitmask of ``MjtCtrlInput`` values
instead of a single mode selector.

**Before:**

.. code-block:: rust

    let config = DcMotorConfig::default().with_input_mode(0);

**After:**

.. code-block:: rust

    let config = DcMotorConfig::default()
        .with_ctrlspec(MjtCtrlInput::mjINPUT_VOLTAGE as i32);


Relaxed ``MjData`` ``body_awake`` view mutability
-------------------------------------------------
The body views now match the ``body_awake`` array accessor, so a write through a view no longer
needs ``as_mut_slice`` inside ``unsafe``.

**Before:**

.. code-block:: rust

    unsafe { view.awake.as_mut_slice()[0] = MjtSleepState::mjS_AWAKE };

**After:**

.. code-block:: rust

    view.awake[0] = MjtSleepState::mjS_AWAKE;


``SpecItem::default`` returns an ``Option``
---------------------------------------------
MuJoCo's ``mjs_getDefault`` returns null for an element added without a default class. The method
now returns ``Option<&MjsDefault>``.

**Before:**

.. code-block:: rust

    let class = body.default();

**After:**

.. code-block:: rust

    let class = body.default().unwrap();


Deprecated fallible allocation methods
--------------------------------------
|mj_model|'s ``try_clone``, |mj_data|'s ``try_clone``, |mj_spec|'s ``try_new``, ``try_add_frame``
on |mjs_body| and |mjs_frame|, and every macro-generated ``try_add_*`` element constructor on
|mj_spec|, |mjs_body| and |mjs_frame| are now deprecated. Their ``AllocationFailed`` arm is
unreachable. ``try_add_default`` and ``try_add_flexcomp`` keep their ``Result``. Switch to
``clone``, ``new`` and the panicking ``add_*`` methods.

**Before (5.x)**:

.. code-block:: rust

  let mut spec = MjSpec::try_new()?;
  let body = spec.world_body_mut().try_add_body()?;
  let data_copy = data.try_clone()?;

**After (6.0.0)**:

.. code-block:: rust

  let mut spec = MjSpec::new();
  let body = spec.world_body_mut().add_body();
  let data_copy = data.clone();


egui 0.36 and the new UI callback signature
-------------------------------------------
The :ref:`mj_rust_viewer` builds against ``egui`` 0.36, so the closure passed to
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` receives a
`&mut egui::Ui <https://docs.rs/egui/0.36.1/egui/struct.Ui.html>`_ instead of a
`&egui::Context <https://docs.rs/egui/0.36.1/egui/struct.Context.html>`_. The second parameter
changed from the locked passive |mj_data| to the ``&Arc<Mutex<ViewerSharedState>>``
(:docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`), so the closure decides when to take
the lock.

``MjViewer::add_ui_callback_detached`` was removed.

**Before (5.x)**:

.. code-block:: rust

    viewer.add_ui_callback(|ctx, data| {
        egui::SidePanel::right("custom_panel").show(ctx, |ui| {
            ui.heading("Custom Panel");
            if ui.button("Zero the controls").clicked() {
                data.ctrl_mut().fill(0.0);
            }
        });
    });

    viewer.add_ui_callback_detached(|ctx| {
        egui::TopBottomPanel::top("custom_top_panel").show(ctx, |ui| {
            ui.label("Custom Top Bar");
        });
    });

**After (6.0.0)**:

.. code-block:: rust

    viewer.add_ui_callback(|ui, state| {
        egui::Panel::right("custom_panel").show(ui, |ui| {
            ui.heading("Custom Panel");
            ui.label(format!("running: {}", state.lock().unwrap().running()));
        });
    });

    viewer.add_ui_callback(|ui, _| {
        egui::Panel::top("custom_top_panel").show(ui, |ui| {
            ui.label("Custom Top Bar");
        });
    });

Port the widgets themselves against egui's changelog:
https://github.com/emilk/egui/blob/main/CHANGELOG.md


