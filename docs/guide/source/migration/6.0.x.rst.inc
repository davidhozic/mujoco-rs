.. _migrate_6_0_0:

Migrating to 6.0.0
======================

Version 6.0.0 updates MuJoCo to 3.12.0, which removed and renamed several
|mj_data| fields, reworked the actuator dimensions for MIMO actuators, and
changed a few visualization and model signatures.


MuJoCo upgrade
-----------------------

This release links against MuJoCo **3.12.0**. Download the matching release and
update your library path. See :ref:`installation` for details.


Removed ``MjData::q_m`` accessor
---------------------------------
MuJoCo removed the legacy sparse ``mjData.qM`` inertia matrix; the joint-space inertia matrix
is now stored exclusively in the CSR format ``mjData.M``. Use the ``m`` accessor together with
the |mj_model| ``m_rownnz``/``m_rowadr``/``m_colind`` index arrays, or ``full_m`` for a dense copy.

**Before:**

.. code-block:: rust

    let inertia = data.q_m();

**After:**

.. code-block:: rust

    let inertia = data.m();  // CSR values; indices in model.m_rownnz()/m_rowadr()/m_colind()

    // or, for a dense nv x nv matrix:
    let nv = model.nv() as usize;
    let mut dense = vec![0.0; nv * nv];
    data.full_m(&mut dense).unwrap();


``MjvCamera::move_`` no longer takes a scene
--------------------------------------------
The ``mjvScene`` argument was removed from ``mjv_moveCamera`` upstream, so the ``scene``
parameter was dropped from |mjv_camera| ``move_``.

**Before:**

.. code-block:: rust

    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0, renderer.scene());

**After:**

.. code-block:: rust

    camera.move_(MjtMouse::mjMOUSE_ZOOM, &model, 0.0, -1.0);


``MjData::efc_diag_approx`` renamed to ``efc_diag_a``
------------------------------------------------------
MuJoCo renamed the ``efc_diagApprox`` field to ``efc_diagA``, since it can now hold either
the exact or the approximate diagonal of the :math:`A` matrix. The |mj_data| accessor was
renamed to match.

**Before:**

.. code-block:: rust

    let diag = data.efc_diag_approx();

**After:**

.. code-block:: rust

    let diag = data.efc_diag_a();


Removed island matrix ``MjData`` accessors
------------------------------------------
MuJoCo moved the island-specific inertia and Jacobian matrices off the arena onto the stack,
so they are no longer exposed as ``mjData`` fields. The following |mj_data| accessors were
removed with no replacement: ``i_m_rownnz``, ``i_m_rowadr``, ``i_m_colind``, ``i_m``, ``i_ld``,
``i_ldiag_inv``, ``iefc_j_rownnz``, ``iefc_j_rowadr``, ``iefc_j_rowsuper``, ``iefc_j_colind``,
and ``iefc_j``.


Removed ``MjData::maxuse_threadstack`` accessor
-----------------------------------------------
The legacy engine threading API was removed upstream, taking the per-thread stack-usage
statistic with it. The ``maxuse_threadstack`` accessor was removed with no replacement.


MIMO-aware actuator dimensions
------------------------------
Actuators now have separate control and output dimensions: ``nu`` is the total number of
scalar controls, ``nout`` the total number of force outputs, and ``nactuator`` the number of
actuators (previously all three coincided as ``nu``). Actuator-indexed |mj_model| array
accessors are now sized by ``nactuator``, ``actuator_gear``/``actuator_acc0``/
``actuator_length0``/``actuator_lengthrange`` by ``nout``, while the per-control
``actuator_ctrllimited`` and ``actuator_ctrlrange`` remain sized by ``nu``. The |mj_data|
accessors ``actuator_length``, ``moment_rownnz``, ``moment_rowadr``, ``actuator_velocity``
and ``actuator_force`` are now sized by ``nout``. The per-actuator views for control- and
output-indexed fields (``ctrl``, ``length``, ``velocity``, ``force``; ``ctrllimited``,
``ctrlrange``, ``gear``, ``acc0``, ``length0``, ``lengthrange``) changed from fixed-size
views to dynamic ranges based on ``actuator_ctrladr`` and ``actuator_outadr``. For
single-input single-output actuators ``nactuator == nu == nout``, so models without MIMO
actuators are unaffected.

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


Shifted actuator enum discriminants
-------------------------------------
``MjtGain``, ``MjtBias``, ``MjtDyn`` and ``MjtTrn`` gained new variants, so an exhaustive
``match`` on any of them no longer compiles. The raw value of ``mjGAIN_USER`` moved from 4 to 6,
``mjBIAS_USER`` from 4 to 5 and ``mjDYN_USER`` from 6 to 7, so code that stores or compares raw
discriminants (rather than the enum variants) needs the new values.


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
``set_buffer`` took a raw ``i32``, so any integer reached ``mjr_setBuffer``. It now takes
``MjtFramebuffer``.

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
``create_geom`` and ``try_create_geom`` are now ``unsafe fn``. The renderer reads the returned
geom's ``matid``, ``texid`` and, on a flex or a skin geom, ``objid`` as unchecked indices. The
caller keeps ``matid`` and ``texid`` below the counts of the model that the rendering context was
created for, and ``objid`` below the scene's flex or skin count. All three ids start at ``-1``,
which means no material, no texture and no object.

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
``body_awake`` was already safe to mutate through its array accessor, and the body views now
match it, so a write through a view no longer needs ``as_mut_slice`` inside ``unsafe``.

**Before:**

.. code-block:: rust

    unsafe { view.awake.as_mut_slice()[0] = MjtSleepState::mjS_AWAKE };

**After:**

.. code-block:: rust

    view.awake[0] = MjtSleepState::mjS_AWAKE;


``SpecItem::default`` returns an ``Option``
---------------------------------------------
MuJoCo's ``mjs_getDefault`` returns null for an element added without a default class (a frame,
sensor, flex, exclude, numeric, text, tuple, key, plugin, hfield, skin, texture or wrap), so the old
``&MjsDefault`` return dereferenced a null pointer. The method now returns ``Option<&MjsDefault>``.

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
unreachable. ``try_add_default`` and ``try_add_flexcomp`` keep their ``Result``: MuJoCo does report
a duplicate class name and a failed flex build. ``mj_copyModel`` and ``mj_copyData`` raise
``mjERROR``, whose default handler exits the process. ``mj_makeSpec`` and the ``mjs_addX`` functions
allocate with C++ ``new``, which throws instead of returning null; the official MuJoCo binary
bundles its own C++ runtime, so the exception cannot cross the C API and no handler in the calling
program can catch it. Switch to ``clone``, ``new`` and the panicking ``add_*`` methods.

These methods may be undeprecated in the future if MuJoCo's upstream code is changed to report the
failure recoverably.

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
`&egui::Context <https://docs.rs/egui/0.36.1/egui/struct.Context.html>`_. The second parameter changed from the locked passive |mj_data| to the
``&Arc<Mutex<ViewerSharedState>>`` (:docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`), so the closure decides when to take the lock.

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


