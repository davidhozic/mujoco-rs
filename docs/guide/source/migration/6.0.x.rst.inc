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


Minimum Rust version
-----------------------

This release needs Rust **1.95**, up from 1.88. Run ``rustup update stable`` before you build.


Redundant dimension parameters removed
---------------------------------------------------------
Four utility wrappers took a dimension that the slices they already take fix. Drop the argument;
the wrapper derives it and still panics when the slice lengths disagree.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before
     - After
   * - ``mju_eye(&mut mat, n)``
     - ``mju_eye(&mut mat)``
   * - ``mju_symmetrize(&mut res, &mat, n)``
     - ``mju_symmetrize(&mut res, &mat)``
   * - ``mju_sqr_mat_td(&mut res, &mat, diag, nr, nc)``
     - ``mju_sqr_mat_td(&mut res, &mat, diag, nr)``
   * - ``mju_sym2dense(&mut res, &mat, n, &rownnz, &rowadr, &colind)``
     - ``mju_sym2dense(&mut res, &mat, &rownnz, &rowadr, &colind)``

:docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_sqr_mat_td` keeps ``nr``, because a zero
``nc`` leaves ``nr`` free.


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


Unsafe mutation of three more ``MjModel`` tables
------------------------------------------------
``tex_type_mut``, ``sensor_intprm_mut`` and ``flex_elemlayer_mut`` are now ``unsafe fn``, for the
same reason as ``actuator_gaintype_mut`` above. C sizes a buffer or indexes an array from the
values of all three: a texture kind sets how many faces the upload reads, a sensor int parameter
selects the object a sensor stage reads, and an element layer sets how many flex faces the scene
builder emits.

**Before:**

.. code-block:: rust

    model.tex_type_mut()[id] = MjtTexture::mjTEXTURE_CUBE;
    model.sensor_intprm_mut()[id][0] = 1;
    model.flex_elemlayer_mut()[id] = 0;

**After:**

.. code-block:: rust

    // SAFETY: the texture data behind `id` holds the six faces that a cube map reads.
    unsafe { model.tex_type_mut()[id] = MjtTexture::mjTEXTURE_CUBE };
    // SAFETY: 1 selects an object that this sensor's stage can read.
    unsafe { model.sensor_intprm_mut()[id][0] = 1 };
    // SAFETY: 0 keeps the element on the surface, so the face count does not grow.
    unsafe { model.flex_elemlayer_mut()[id] = 0 };


Drawing an ``MjvFigure`` is ``unsafe``
--------------------------------------
``draw`` is now an ``unsafe fn``. ``linepnt`` is a public field, and C reads ``linedata[n][2*i]``
for every ``i < linepnt[n]`` without an upper-bound check, so an entry above ``mjMAXLINEPNT``
read past the row. Validating it would scan all ``mjMAXLINE`` entries on a per-frame path, so the
bound is a caller precondition. ``push`` refuses to grow a plot that is already at capacity, so a
figure filled only through the line-editing methods already satisfies it; a direct write to the
public ``linepnt`` field is what needs the care.

**Before:**

.. code-block:: rust

    figure.draw(viewport, &context);

**After:**

.. code-block:: rust

    // SAFETY: every linepnt entry stays within 0..=mjMAXLINEPNT, as only push wrote them.
    unsafe { figure.draw(viewport, &context) };


Reading an ``MjData`` arena array is ``unsafe``
-----------------------------------------------
MuJoCo allocates the constraint arrays in the ``mjData`` arena and never zeroes it, so an array is
uninitialized until the pipeline stage that computes it has run. Sixteen accessors and their
``_mut`` counterparts are now ``unsafe fn``, grouped by the stage each one needs:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Run first
     - Accessors
   * - ``forward`` or ``step2``
     - ``efc_state``, ``efc_force``, ``efc_b``, ``iefc_state``, ``iefc_force``, ``iefc_aref``,
       ``iacc``, ``iacc_smooth``, ``ifrc_smooth``, ``ifrc_constraint``
   * - ``step1``, ``fwd_velocity`` or later
     - ``efc_vel``, ``efc_aref``
   * - ``fwd_position`` or later, with a sparse Jacobian
     - ``efc_j_rownnz``, ``efc_j_rowadr``, ``efc_j_rowsuper``, ``efc_j_colind``

Two groups need extra care. MuJoCo fills the four Jacobian index arrays only when ``opt.jacobian``
resolves to sparse, so a dense model leaves them uninitialized even after a full ``forward``. The
four island dof arrays stay uninitialized after ``forward`` as well when ``opt.solver`` is ``PGS``,
because only the CG and Newton island paths gather into them.

**Before:**

.. code-block:: rust

    data.step1();
    let state = data.efc_state();   // allocated, but the solver has not run

**After:**

.. code-block:: rust

    data.forward();
    // SAFETY: forward ran the constraint solver, so efc_state holds computed values.
    let state = unsafe { data.efc_state() };
    if state[0] == MjtConstraintState::mjCNSTRSTATE_QUADRATIC {
        // ...
    }


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


Model handles are bound by ``ModelType``
----------------------------------------
The model-handle generic ``M`` is bound by
:docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelType` instead of
``Deref<Target = MjModel>``, and the model-mutating methods by
:docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut` instead of
``DerefMut<Target = MjModel>``. Both markers hold the guarantee that the handle always derefs to
the same model, which the wrappers rely on but ``Deref`` alone does not give.

``&MjModel``, ``&mut MjModel``, ``Box<MjModel>``, ``Rc<MjModel>``, ``Arc<MjModel>``,
``Ref<MjModel>``, ``RefMut<MjModel>``, ``MutexGuard<MjModel>``, ``RwLockReadGuard<MjModel>`` and
``RwLockWriteGuard<MjModel>`` implement the markers. Rename the bound in your own generic code, and
claim the marker for a hand-written handle.

**Before:**

.. code-block:: rust

    use std::ops::Deref;
    use mujoco_rs::prelude::*;

    fn step_all<M: Deref<Target = MjModel>>(datas: &mut [MjData<M>]) {
        for data in datas {
            data.step();
        }
    }

    struct MyHandle(Box<MjModel>);

    impl Deref for MyHandle {
        type Target = MjModel;
        fn deref(&self) -> &MjModel { &self.0 }
    }

**After:**

.. code-block:: rust

    use mujoco_rs::wrappers::mj_model::traits::ModelType;
    use std::ops::Deref;
    use mujoco_rs::prelude::*;

    fn step_all<M: ModelType>(datas: &mut [MjData<M>]) {
        for data in datas {
            data.step();
        }
    }

    struct MyHandle(Box<MjModel>);

    impl Deref for MyHandle {
        type Target = MjModel;
        fn deref(&self) -> &MjModel { &self.0 }
    }

    // SAFETY: deref returns the model that the handle owns, for every call.
    unsafe impl ModelType for MyHandle {}
