6.0.0 (MuJoCo 3.12.0)
======================

.. rubric:: Breaking changes

*Minimum-supported Rust version (MSRV) raised to 1.95*

- `egui <https://docs.rs/egui/0.36.1/egui/>`_ 0.36 requires Rust 1.95, so the MSRV rose from 1.88
  to 1.95. This also applies to any program that extends the :ref:`mj_rust_viewer`'s UI.

*egui upgraded to 0.36*

- The :ref:`mj_rust_viewer` now builds against ``egui`` 0.36, which it re-exports as
  ``mujoco_rs::viewer::egui``. Custom UI code must be ported to the new ``egui`` API,
  see :ref:`migrate_6_0_0`. For the upstream changes, see egui's changelog:
  https://github.com/emilk/egui/blob/0.36.1/CHANGELOG.md

*UI callbacks receive an* `egui::Ui <https://docs.rs/egui/0.36.1/egui/struct.Ui.html>`_ *and the shared state*

- The closure passed to
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` now receives
  ``&mut egui::Ui`` and ``&Arc<Mutex<ViewerSharedState>>``
  (:docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`), instead of ``&egui::Context``
  (`egui::Context <https://docs.rs/egui/0.36.1/egui/struct.Context.html>`_) and
  ``&mut MjData<Box<MjModel>>`` (|mj_data|, |mj_model|).
  ``MjViewer::add_ui_callback_detached`` is removed as ``add_ui_callback`` now handles both cases.

*The signature-mismatch error variants are renamed*

- ``SignatureMismatch`` becomes ``IncompatibleModel`` in
  :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IncompatibleModel`,
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>IncompatibleModel`,
  :docs-rs:`~~mujoco_rs::viewer::<enum>MjViewerError::<variant>IncompatibleModel` and
  :docs-rs:`~~mujoco_rs::renderer::<enum>RendererError::<variant>IncompatibleModel`. The guard
  behind each of them now tests the whole model memory layout instead of the incomplete
  signature only.

*MuJoCo FFI upgraded to 3.12.0*

- MuJoCo-rs now builds against MuJoCo 3.12.0 FFI, which rolls up the upstream 3.10.0,
  3.11.0 and 3.12.0 releases.
  This directly affects all the wrappers, separate changelog will not be provided.
  See MuJoCo's changelog for this release:
  https://mujoco.readthedocs.io/en/3.12.0/changelog.html

``MjvCamera::move_`` *no longer takes a scene*

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>move_` lost its
  ``scene: &MjvScene`` (|mjv_scene|) parameter, following the upstream removal of the unneeded
  :docs-rs:`~mujoco_rs::mujoco_c::<type>mjvScene` argument from
  :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjv_moveCamera`.

*Stricter* |mjr_context| *accessor types*

- :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>gl_initialized`,
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>window_available`,
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>window_stereo` and
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>window_doublebuffer`
  now read as ``bool`` instead of ``i32``; MuJoCo stores only 0 or 1 in them.
- :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>font_scale` and
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_depth_map` now
  read as :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<type>MjtFontScale` and
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<type>MjtDepthMap` instead of ``i32``.

*Unsafe mutation of* |mj_model| ``actuator_gaintype``

- :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_gaintype_mut` is now
  an ``unsafe fn``; the actuator views are affected the same way, so a write through a view needs
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut::<method>as_mut_slice` inside
  ``unsafe``.

*Unsafe geom creation on* |mjv_scene|

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>try_create_geom` are
  now ``unsafe fn``.

*Renamed* ``DcMotorConfig`` *input field*

- The public field ``DcMotorConfig::input_mode`` is now
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>DcMotorConfig::<structfield>ctrlspec` and the
  builder ``with_input_mode`` is now
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>DcMotorConfig::<method>with_ctrlspec`,
  following the renamed :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_setToDCMotor` parameter. The value
  is now a bitmask of :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtCtrlInput` values instead
  of a single mode selector. MuJoCo 3.12.0 also moved the ``dcmotor`` controller gains from
  voltage space to torque space, so the same ``controller`` array now gives a different force.
  The old velocity mode's integrated-velocity term is gone.

*Relaxed* |mj_data| ``body_awake`` *view mutability*

- The body views now hold ``awake`` as a :docs-rs:`~mujoco_rs::util::<struct>PointerViewMut`. Code
  that wrote through ``as_mut_slice`` must drop the call and the surrounding ``unsafe``.

``SpecItem::default`` *returns an* ``Option``

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>default` now returns
  ``Option<&MjsDefault>`` (:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault`). Handle
  the ``None`` case.

*Changed* |mjr_context| ``set_buffer`` *parameter type*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_buffer` now
  accepts ``framebuffer: MjtFramebuffer``
  (:docs-rs:`~mujoco_rs::wrappers::mj_rendering::<type>MjtFramebuffer`) instead of ``i32``.

*Control-history methods take an actuator index*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_read_ctrl` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>init_ctrl_history` now bound
  ``id`` by ``nactuator`` instead of ``nu``.

*Redundant dimension parameters removed*

- :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_eye`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_symmetrize`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_sqr_mat_td` and
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_sym2dense` each dropped a dimension that
  the slices they already take determine. Drop the argument at the call site, see
  :ref:`migrate_6_0_0`.

|mj_data| *and the visual APIs bound the model handle by* ``ModelType``

- The model-handle generic ``M`` is now bound by the ``unsafe`` marker traits
  :docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelType` :sup:`new` and
  :docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut` :sup:`new`, instead of
  ``Deref<Target = MjModel>`` and ``DerefMut<Target = MjModel>``. Every standard
  handle implements the markers, so only code that spells the bound or defines its own handle
  needs an edit, see :ref:`migrate_6_0_0`.

*Three* |mj_model| *setters that drive a C index became* ``unsafe``

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tex_type_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>sensor_intprm_mut` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_elemlayer_mut` are now
  ``unsafe fn``. Wrap the call in an ``unsafe`` block and uphold the C invariant,
  see :ref:`migrate_6_0_0`.

*Drawing an* ``MjvFigure`` *became* ``unsafe``

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>draw` is now an
  ``unsafe fn``. The ``linepnt`` bound is a caller precondition, see :ref:`migrate_6_0_0`.

*Sixteen* |mj_data| *arena accessors became* ``unsafe``

- The accessors below are now ``unsafe fn``, together with their ``_mut`` counterparts:

  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_state`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_force`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_b`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_vel`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_aref`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_state`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_force`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_aref`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iacc`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iacc_smooth`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ifrc_smooth`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ifrc_constraint`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_j_rownnz`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_j_rowadr`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_j_rowsuper`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_j_colind`

  A read taken before the pipeline stage that computes the array returns uninitialized memory.
  Run that stage first, see :ref:`migrate_6_0_0`.

*Two model view fields moved to the read-only list*

- ``intprm`` on :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjSensorModelViewMut` and
  ``type`` on :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTextureModelViewMut` are now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`, matching the array accessors
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>sensor_intprm_mut` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tex_type_mut`. Writing one
  now needs ``as_mut_slice`` and explicit ``unsafe``, see :ref:`migrate_6_0_0`. ``zero`` no longer
  touches either field.

*Element handles are opaque types*

- Every ``Mjs*`` element handle is now an opaque, zero-sized struct instead of an alias for the
  matching ``mujoco_c`` struct; |mjs_body| and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault` are examples. It exposes its FFI
  struct through ``ffi`` for reading and ``unsafe fn ffi_mut`` for writing, so code that hands a
  handle to a ``mujoco_c`` function needs an edit, see :ref:`migrate_6_0_0`. ``std::mem::swap`` on
  two handles used to exchange the contents of two elements; it now exchanges nothing.

|mj_spec| *is no longer* ``Send``

- A model that attaches another model keeps the child specification shared between a spec and every
  clone of it, behind a reference count that MuJoCo does not synchronize.

|mjs_wrap| *lost* ``side_site_mut``

- The shared :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsWrap::<method>side_site` is not removed.
  Mutable references to sites are available through
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>site_mut`, see
  :ref:`migrate_6_0_0`.

|mjs_plugin| *no longer stands for the plugin that an element embeds*

- The ``plugin`` and ``plugin_mut`` accessors of a body, geom, mesh, actuator and sensor now hand
  out :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsPluginReference` :sup:`new`.
  See :ref:`migrate_6_0_0`.

``delete`` *moved from* :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem` *to*
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecObject`

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecObject::<method>delete` :sup:`new` stays
  ``unsafe`` and is no longer deprecated. A default class and a tendon wrap are no ``SpecObject``,
  so a deletion that MuJoCo cannot carry out is now a compile error instead of an error value.
  See :ref:`migrate_6_0_0`.

.. rubric:: New dependencies

- Added `log <https://docs.rs/log/0.4.34/log/>`__ as a mandatory dependency.

.. rubric:: Deprecations

- Deprecated |mj_spec|'s
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element`. Call
  ``SpecObject::delete`` on the element handle instead.
- Deprecated |mj_model|'s
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_clone` --- its
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>AllocationFailed` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_data|'s :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_clone`
  --- its :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>AllocationFailed` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_spec|'s
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>try_new`,
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_frame` on |mjs_body|
  and |mjs_frame|, and every
  macro-generated ``try_add_*`` element constructor on |mj_spec|, |mjs_body| and |mjs_frame| ---
  their :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>AllocationFailed` arm is
  unreachable. Use ``new`` and the
  panicking ``add_*`` methods instead.
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>try_add_default` and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_flexcomp` keep their
  ``Result``: MuJoCo does report a duplicate class name and a failed flex build.

.. rubric:: New features and improvements

*Model and scene compatibility*

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`
  :sup:`new` reports whether another |mj_model| can take the place of this one in an |mj_data| or
  in a cached ``Info``.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_asset_compatible_with_model`
  :sup:`new` is the weaker test the asset-upload methods need: it compares the mesh, texture and
  heightfield memory alone.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
  :sup:`new` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_scene`
  :sup:`new` report whether a model, or a second |mjv_scene|, matches the buffers this scene
  holds. Use them to decide whether a scene needs to be rebuilt before an update or a merge.
- Every ``Info`` gained an ``update_layout`` method, for example
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjBodyDataInfo::<method>update_layout`
  :sup:`new`. Call ``update_layout`` once against the new |mj_data| or |mj_model| after a model
  swap. It reports the same error as a view method when the new model is not compatible.

*Viewer improvements*

- The viewer's UI side-panel now also opens and closes by a drag of its inner edge to and from
  the left screen-edge, so the panel no longer needs the keyboard. The ``X`` key still toggles
  it. The panel now slides with an animation when it opens or closes.
- The "Joint" window now lists every joint, and each joint is a collapsible section like an
  actuator in the "Actuator" window. The window listed a slide joint and a hinge joint only
  before.
- Each component of the "Joint" window now writes ``qpos``, so the window moves the model. The
  window was read-only before. A limited joint gets the joint limit as its slider range. Without
  a limit, a slide joint spans -1 to 1 and a hinge joint spans -pi to pi, as in MuJoCo's own
  viewer. A quaternion component spans -1 to 1, and the window normalizes the quaternion after an
  edit. A free joint holds no natural range, so each of its components takes a numeric input
  instead of a slider.
- A limited ball joint now shows its rotation angle against the limit, because MuJoCo limits the
  angle of a ball joint and not one quaternion component. Every component of a ball joint reported
  "no limit" before, even when the joint was limited.
- The "Actuator" window now holds one collapsible section for each actuator, with one slider for
  each control input of the actuator. A multi-input actuator labels each slider with the input
  name that MuJoCo reports for the actuator type, for example ``pos``, ``vel`` and ``ff`` for a
  ``pid`` actuator. An actuator without control inputs shows "no inputs". The window showed one
  flat slider for each control before, labelled ``<name>[<index>]``.
- A defined ``ctrlrange`` now sets the range of an actuator slider, even when MuJoCo does not
  clamp the control to it. The slider spanned -1 to 1 unless ``actuator_ctrllimited`` was set
  before.
- A press on the scene now takes the selection away from every viewer window.

*Unified logging API*

- Added a new logging chapter. Access it at :ref:`logging` for full information.
- MuJoCo logging wrapper:

  - Added :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjLogConfig` :sup:`new` and
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjLogMessage` :sup:`new` aliases (with
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjtLogLevel` :sup:`new` and
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjtLogTopic` :sup:`new`) for the unified logging
    API of MuJoCo 3.10.0.
  - Added the free logging MuJoCo wrapper functions
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_config` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_getLogConfig`),
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>set_log_config` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_setLogConfig`),
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_message` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_message`),
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_info` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_info`),
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_warning` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_warning`) and
    :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_error` :sup:`new`
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_error`).

- Standard logging integration

  - Added the new top-level :docs-rs:`~mujoco_rs::logging` module, which sends MuJoCo's messages to
    the ``log`` crate, so they reach the same backend as the rest of the application.
    :docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` :sup:`new` installs a hook for
    ``log`` routing.
    :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` :sup:`new` registers a user handler.
  - MuJoCo-rs now reports its own events to the ``log`` crate as well, on the ``Info``, ``Warn``
    and ``Error`` levels, with the emitting module path as the target.

*New compilation-warning accessors*

- |mj_spec| gained two compilation-warning accessors, wrapping
  :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_numWarnings` and
  :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_getWarning`:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>num_warnings` :sup:`new`
    and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>warning` :sup:`new`.

*New* |mj_data| *methods*

- |mj_data|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>full_m` :sup:`new`, converting the
    sparse inertia matrix into a dense ``nv x nv`` matrix; wraps
    :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_fullM`. Returns
    :docs-rs:`Err(MjDataError::BufferTooSmall) <mujoco_rs::error::<enum>MjDataError::<variant>BufferTooSmall>`
    when ``dst`` is shorter than ``nv * nv``.
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_threadpool` :sup:`new`, creating a
    worker thread pool with ``nthread`` threads for parallel simulation; wraps
    :docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_threadpool`.

*RNE into a caller buffer*

- Added :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>rne_into` :sup:`new` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_rne_into` :sup:`new`,
  wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_rne` to write the inverse-dynamics force into a
  caller buffer. ``try_rne_into`` returns ``Err(MjDataError::BufferTooSmall)`` when ``result`` holds
  fewer than ``nv`` elements.

*Cloneable renderer builder*

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` now derives ``Clone``, so a configured
  builder can be duplicated before
  :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder::<method>build`.

*Procedural flex creation*

- |mjs_body|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_flexcomp` :sup:`new` (and the
    fallible ``try_add_flexcomp``), wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_makeFlex` to
    procedurally create a flex with auto-generated bodies, joints, and optional equality constraints
    (the programmatic equivalent of the ``flexcomp`` element). The generation is configured
    through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjFlexcompConfig` :sup:`new` builder.

*New type aliases*

- Added :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjrCamera` :sup:`new` as a type
  alias for :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGLCamera`, matching MuJoCo's
  own :docs-rs:`~mujoco_rs::mujoco_c::<type>mjrCamera` alias convention.

*Actuator input names*

- |mj_model|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_input_name`
    :sup:`new`, wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_actuatorInputName`, gives the name
    of one control input of an actuator. It returns ``None`` when the actuator type defines no input
    names.

*New servo actuator constructors*

- |mjs_actuator|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsActuator::<method>set_to_pid` :sup:`new`,
    wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_setToPID`, makes the actuator a PID controller
    on a single force output. The controller is configured through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PidConfig` :sup:`new` builder.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsActuator::<method>set_to_orientation`
    :sup:`new`, wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_setToOrientation`, makes the
    actuator a geodesic PD orientation servo on a ball joint or a site. It is configured through the
    new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>OrientationConfig` :sup:`new` builder.

*Relaxed mutability on fields that cannot cause a memory fault*

- Mutating :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>paths` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>texture_type` no longer
  needs ``unsafe``.
  A write can at worst give a wrong asset path or wrong rendering, never a memory fault.

.. rubric:: New examples

- :gh-example:`Actuator inputs <visualization/viewer/actuator_inputs.rs>` --- shows the actuator
  controls of the :ref:`mj_rust_viewer` for every input flavour. The model also holds a limited
  ball joint, an unlimited ball joint and a free joint, for the sliders of the "Joint" window.
- :gh-example:`Logging <simulation/logging.rs>` --- shows the three ways to receive a MuJoCo
  message: the default handler with its ``MjLogConfig``, the logging hook next to
  `env_logger <https://docs.rs/env_logger/0.11.11/env_logger/>`_ (with the target that carries the
  topic), and a custom handler that reads the ``MjLogMessage``.

.. rubric:: Bug fixes

*Safety*

- |mj_data|'s :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_state` now
  rejects a history payload whose ring-buffer cursor falls outside ``0..nsample``, and leaves the
  history buffer unchanged. It reports the new
  :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>InvalidHistoryCursor` :sup:`new`.
- The matrix helpers of :docs-rs:`mujoco_rs::wrappers::fun::utility` now reject a dimension above
  ``i32::MAX`` and a size product that overflows, instead of passing a wrapped size to MuJoCo.
- A view of a |mj_data| field that MuJoCo allocates in the arena is now empty until the stage that
  allocates it, for every element id. Only ``efcadr`` on
  :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjTendonDataView` is affected.

*Viewer*

- The viewer runs a forward step on the data it builds for a new model, so the first frame after
  a model load or swap no longer draws every geom collapsed at the origin.
- The viewer no longer clamps ``ctrl`` to the range of its slider. While the "Actuator" window
  was open, it wrote the clamped control back on every frame, so a program that set a control
  outside the slider range lost that value.
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` no longer rebuilds the window
  and the OpenGL context when it is called after the window closed.
- The viewer can no longer panic on a double click that arrives in the same frame as a model swap
  through :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`. The click is dropped
  while the scene still follows the previous model.
- The viewer's side panel and its windows can no longer panic, or write to the wrong actuator,
  joint or equality, when a model swap through ``sync_data`` arrives in the same frame.
- A model swap no longer discards the change that a program makes to the new model's ``opt``,
  ``vis`` or ``stat`` before the next
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_model`.
- The viewer no longer applies a pending camera tracking selection to a body of the previous model
  after a model swap. A selection that the "Track" dialog left open held a body id that addresses
  a different body in the new model, so the camera followed that one instead.

*Model and data pairing*

- Closed a set of out-of-bounds accesses reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_swap_model`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_to`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_visual_to`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_state_from_data` and in
  every ``Info`` view method. All of them compared the model signature alone. The ``MjData``
  methods now test
  ``MjModel::is_compatible_with_model``, which also compares the per-element address tables; an
  ``Info`` view method compares the model layout it cached.
- The viewer and the renderer no longer test the signature either. ``MjViewer::sync_data`` and
  ``MjViewer::sync_model`` test model compatibility, so a model that needs different buffers
  triggers a reload rather than a copy into the wrong sizes. The viewer's asset-upload methods
  test the mesh, texture and heightfield memory alone, so they accept a model whose ``mjData``
  sizes differ.

*Ray casting*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` no longer ends the process
  for a direction shorter than ``mjMINVAL``. It now reports no intersection, which is what
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray` reports for the same
  direction.
- ``multi_ray`` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_multi_ray` no longer report
  geom ``0`` for such a direction; the returned id is now ``None``.

*Visualization*

- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frame` for a tracking
  camera: ``frame`` now validates the id against the model and panics on an out-of-range one.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>start`:
  ``start`` now validates ``flexselect`` against the model and panics on an out-of-range id.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>render`:
  ``render`` now panics when the context holds fewer skins than the scene.
- Closed a heap buffer overflow reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>find_selection`:
  both now test the model with ``MjvScene::is_compatible_with_model`` instead of the signature,
  and panic when the model in the given |mj_data| needs larger scene buffers.
- ``MjvScene::update`` and ``MjvScene::find_selection`` now accept a model that
  :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_loadModel` returned. ``mj_saveModel`` writes no
  compilation signature, so the previous signature test refused the saved copy of the very model
  the scene was created for.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom` now
  writes every field that :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjv_initGeom` leaves unwritten over
  uninitialized memory. ``category`` becomes
  :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjtCatBit::<variant>mjCAT_DECOR`, and a
  ``mjCAT_DECOR`` geom casts no shadow.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new` now clears the
  geom, flex and skin buffers that :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjv_makeScene` allocates but
  never writes. A read before the first render returned uninitialized memory; it now returns
  zeros.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>set_at` now treats a
  negative ``linepnt`` entry as an empty plot, instead of writing outside the plot's data range.
- :docs-rs:`~mujoco_rs::vis_common::<fn>sync_geoms` now panics when the two scenes were created for
  models that are not compatible.

*Rendering*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_pixels` no longer
  wraps when the pixel count or the ``rgb`` byte count overflows ``usize`` on a 32-bit target; it
  reports :docs-rs:`~mujoco_rs::error::<enum>MjrContextError::<variant>InvalidViewport` instead.
- :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new` no longer truncates its
  ``width``, ``height`` and ``max_user_geom`` arguments; they now saturate at ``u32::MAX``.

*Model editing*

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>id` now returns ``None``
  for a default class.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecObject::<method>delete` now reports
  :docs-rs:`~mujoco_rs::error::<enum>MjEditError::<variant>UnsupportedOperation` for a frame and
  for the world body. MuJoCo reported success for both and left the specification corrupted.
- Closed a crash reachable from safe code when naming a tendon wrap:
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name` on a |mjs_wrap|
  now reports ``UnsupportedOperation`` and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>with_name` panics. A wrap
  carries no name of its own:
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>name` reports the wrapped
  object's name.
- Closed a heap over-read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile`. It now rejects a
  texture whose ``nchannel``, ``width`` or ``height`` is negative, and one whose
  ``nchannel * width * height`` product does not fit in an ``i32``.

*Utility functions*

- Closed a division by zero and a non-terminating loop reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_halton`: the wrapper now panics unless
  ``base`` is 2 or more.
- Closed a heap buffer overflow reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_zero`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_copy`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_normalize`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_mul_mat_t_vec`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_eye`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_mul_mat_mat`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_mul_mat_t_mat`,
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_sqr_mat_td` and
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_sym2dense`. They now panic on a
  destination above ``i32::MAX`` elements. The other wrappers in the module are unchanged.
- Closed a heap buffer overflow reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsHfield::<method>set_userdata` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_data`. They now
  panic on a slice above ``i32::MAX`` elements.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new` now checks
  ``max_geom`` in a release build as well. The check was a ``debug_assert!``, so a release build
  passed a truncated capacity to MuJoCo and allocated a smaller scene than the caller asked for.

.. rubric:: Other changes

- Following the reworked FFI struct field-visibility rules, the fields of
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsOrientation` (``type_``, ``axisangle``,
  ``xyaxes``, ``zaxis``, ``euler``) and of the raw
  :docs-rs:`~mujoco_rs::mujoco_c::<type>mjsElement` handle (``elemtype``, ``signature``) are now
  ``pub`` again,
  classifying them as plain-data structs and allowing direct field access.

