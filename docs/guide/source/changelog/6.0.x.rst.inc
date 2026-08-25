6.0.0 (MuJoCo 3.12.0)
======================

.. rubric:: Breaking changes

*Minimum-supported Rust version (MSRV) raised to 1.95*

- ``egui`` 0.36 requires Rust 1.95, so the MSRV rose from 1.88 to 1.95. This also applies to any
  program that extends the :ref:`mj_rust_viewer`'s UI.

*egui upgraded to 0.36*

- The :ref:`mj_rust_viewer` now builds against ``egui`` 0.36, which it re-exports as
  ``mujoco_rs::viewer::egui``. Custom UI code must be ported to the new ``egui`` API,
  see :ref:`migrate_6_0_0`. For the upstream changes, see egui's changelog:
  https://github.com/emilk/egui/blob/main/CHANGELOG.md

*UI callbacks receive a* `egui::Ui <https://docs.rs/egui/0.36.1/egui/struct.Ui.html>`_ *and the shared state*

- The closure passed to
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` now receives
  ``&mut egui::Ui`` and ``&Arc<Mutex<ViewerSharedState>>``, instead of ``&egui::Context`` and
  ``&mut MjData<Box<MjModel>>``.

*Removed* ``MjViewer::add_ui_callback_detached``

- The detached variant existed only to avoid the mutex lock. The remaining callback receives the
  shared state unlocked, so a closure that needs no simulation state simply never locks it. Use
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`.

*MuJoCo FFI upgraded to 3.12.0*

- MuJoCo-rs now builds against MuJoCo 3.12.0 FFI, which rolls up the upstream 3.10.0,
  3.11.0 and 3.12.0 releases. This is a breaking dependency-level change for consumers
  tied to older MuJoCo C ABI details.
  See MuJoCo's changelog for this release:
  https://mujoco.readthedocs.io/en/3.12.0/changelog.html

*Removed the* |mj_data| ``q_m`` *accessor*

- MuJoCo 3.11.0 removed the legacy sparse ancestor-walk inertia matrix ``mjData.qM``; the
  joint-space inertia matrix is now stored exclusively in the compressed sparse row (CSR)
  format ``mjData.M``. The |mj_data| ``q_m()``/``q_m_mut()`` accessors were removed; use
  ``m()`` (with the |mj_model| ``m_rownnz``/``m_rowadr``/``m_colind`` index arrays) or
  :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>full_m` for a dense copy.

``MjvCamera::move_`` *no longer takes a scene*

- |mjv_camera| ``move_`` lost its ``scene: &MjvScene`` parameter, following the upstream
  removal of the unneeded ``mjvScene`` argument from ``mjv_moveCamera``.

*Shifted actuator enum discriminants*

- ``MjtGain``, ``MjtBias``, ``MjtDyn`` and ``MjtTrn`` gained new variants, so an exhaustive
  ``match`` on any of them no longer compiles. The raw value of ``mjGAIN_USER`` moved from 4 to 6,
  ``mjBIAS_USER`` from 4 to 5 and ``mjDYN_USER`` from 6 to 7.

*Removed and renamed* |mj_data| *accessors*

- The |mj_data| accessor ``efc_diag_approx`` was renamed to ``efc_diag_a``,
  following the upstream field rename (it can now hold either the exact or
  approximate diagonal of the :math:`A` matrix).
- Removed the island-specific inertia accessors ``i_m_rownnz``, ``i_m_rowadr``,
  ``i_m_colind``, ``i_m``, ``i_ld``, ``i_ldiag_inv``, and the island Jacobian
  accessors ``iefc_j_rownnz``, ``iefc_j_rowadr``, ``iefc_j_rowsuper``,
  ``iefc_j_colind``, ``iefc_j``. Upstream moved these matrices off the arena
  onto the stack, so they are no longer reachable as ``mjData`` fields.
- Removed the ``maxuse_threadstack`` accessor; the old per-thread stack
  statistic was removed together with the legacy engine threading API.

*MIMO-aware actuator dimensions*

- Actuator-indexed accessors changed length: code that iterates them by ``nu`` still compiles,
  but panics out of bounds or reads the wrong range on MIMO models. Actuator-indexed |mj_model|
  array accessors are now sized by ``nactuator`` (previously ``nu``),
  ``actuator_gear``/``actuator_acc0``/``actuator_length0``/``actuator_lengthrange`` by ``nout``,
  and the per-control ``actuator_ctrllimited``/``actuator_ctrlrange`` remain sized by ``nu``.
  The |mj_data| accessors ``actuator_length``, ``moment_rownnz``, ``moment_rowadr``,
  ``actuator_velocity`` and ``actuator_force`` are now sized by ``nout``. The per-actuator views
  are affected the same way: they now use dynamic ranges based on ``actuator_ctrladr`` and
  ``actuator_outadr``. For single-input single-output actuators ``nactuator == nu == nout`` and
  behavior is unchanged.
- |mj_data| ``read_ctrl``/``try_read_ctrl`` and ``init_ctrl_history`` now bound the actuator index
  by ``nactuator`` instead of ``nu``, because the control history is stored per actuator.

*Stricter* |mjr_context| *accessor types*

- ``gl_initialized``, ``window_available``, ``window_stereo`` and ``window_doublebuffer`` now
  read as ``bool`` instead of ``i32``; MuJoCo stores only 0 or 1 in them.
- ``font_scale`` and ``read_depth_map`` now read as ``MjtFontScale`` and ``MjtDepthMap`` instead
  of ``i32``. ``mjr_defaultContext`` leaves ``fontScale`` zeroed, which is not an
  ``MjtFontScale`` variant, but the safe API cannot reach that state, because ``MjrContext::new``
  and ``ffi_mut`` are ``unsafe`` and ``change_font`` takes the enum.

*Unsafe mutation of* |mj_model| ``actuator_gaintype``

- ``actuator_gaintype_mut`` is now an ``unsafe fn``; the actuator views are affected the same
  way, so a write through a view needs ``as_mut_slice`` inside ``unsafe``. MuJoCo 3.11.0 added
  ``mjGAIN_SO3``, which writes three output elements without consulting ``actuator_outnum``, so a
  safe write could overrun ``actuator_force``.

*Unsafe geom creation on* |mjv_scene|

- |mjv_scene| ``create_geom`` and ``try_create_geom`` are now ``unsafe fn``. The renderer reads the
  returned geom's ``matid``, ``texid`` and, on a flex or a skin geom, ``objid`` as unchecked indices
  into the context and the scene arrays, and no check at creation time can hold, because the caller
  writes the fields afterwards.

*Renamed* ``DcMotorConfig`` *input field*

- The public field ``DcMotorConfig::input_mode`` is now ``ctrlspec`` and the builder
  ``with_input_mode`` is now ``with_ctrlspec``, following the renamed ``mjs_setToDCMotor``
  parameter. The value is now a bitmask of ``MjtCtrlInput`` values instead of a single mode
  selector. MuJoCo 3.12.0 also moved the ``dcmotor`` controller gains from voltage space to torque
  space, so the same ``controller`` array now gives a different force. The old velocity mode's
  integrated-velocity term is gone.

*Relaxed* |mj_data| ``body_awake`` *view mutability*

- The body views now hold ``awake`` as a ``PointerViewMut``, which matches the array accessor,
  already safe to mutate. Code that wrote through ``as_mut_slice`` must drop the call and the
  surrounding ``unsafe``.

``SpecItem::default`` *returns an* ``Option``

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>default` now returns
  ``Option<&MjsDefault>``. MuJoCo's ``mjs_getDefault`` returns null for an element added without a
  default class (a frame, sensor, flex, exclude, numeric, text, tuple, key, plugin, hfield, skin,
  texture or wrap), so the old ``&MjsDefault`` dereferenced a null pointer. Handle the ``None``
  case.

*Changed* |mjr_context| ``set_buffer`` *parameter type*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_buffer` now
  accepts ``framebuffer: MjtFramebuffer`` instead of ``i32``.

.. rubric:: Deprecations

- Deprecated |mj_model|'s ``try_clone`` --- its ``MjModelError::AllocationFailed`` arm is
  unreachable, because ``mj_copyModel`` raises ``mjERROR`` and the default handler exits the process
  instead of returning null. Use ``clone`` instead.

- Deprecated |mj_data|'s ``try_clone`` --- ``mj_copyData`` raises ``mjERROR`` in the same way, so
  its ``MjDataError::AllocationFailed`` arm is unreachable. Use ``clone`` instead.

- Deprecated |mj_spec|'s ``try_new``, ``try_add_frame`` on |mjs_body| and |mjs_frame|, and every
  macro-generated ``try_add_*`` element constructor on |mj_spec|, |mjs_body| and |mjs_frame| ---
  ``mj_makeSpec`` and the ``mjs_addX`` functions allocate with C++ ``new``, which throws instead of
  returning null, so their ``MjEditError::AllocationFailed`` arm is unreachable. An allocation
  failure ends the process, because the exception cannot cross the C API. Use ``new`` and the
  panicking ``add_*`` methods instead. ``try_add_default`` and ``try_add_flexcomp`` keep their
  ``Result``: MuJoCo does report a duplicate class name and a failed flex build.

.. rubric:: New features and improvements

*Viewer improvements*

- The viewer's UI side-panel now also opens and closes by a drag of its inner edge to and from
  the left screen-edge, so the panel no longer needs the keyboard. The ``X`` key still toggles
  it. The panel now slides with an animation when it opens or closes.
- The "Joint" window now lists every joint, and each joint is a collapsible section like an
  actuator in the "Actuator" window. A ball joint and a free joint show all their position
  components in one horizontal row, named ``x``, ``y``, ``z`` and ``qw``, ``qx``, ``qy``, ``qz``.
  The window listed a slide joint and a hinge joint only before.
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
- A press on the scene now takes the selection away from every viewer window, so no window keeps a
  highlighted title bar while the pointer works in the scene.

*New accessors for 3.11.0 fields*

- |mj_model|:

  - ``geom_surfacevel`` :sup:`new`, ``geom_adhesion`` :sup:`new` and ``pair_adhesion`` :sup:`new`
    array accessors for the new surface-velocity and contact-adhesion model fields (the geom and
    pair views gained matching ``surfacevel``/``adhesion`` fields).
  - ``actuator_ctrladr`` :sup:`new`, ``actuator_ctrlnum`` :sup:`new`, ``actuator_ctrlspec`` :sup:`new`,
    ``actuator_outadr`` :sup:`new` and ``actuator_outnum`` :sup:`new` array accessors for the new
    MIMO actuator layout (the actuator views gained matching fields). Mutating any of
    them needs ``unsafe``, because the engine uses their values as unchecked addresses and
    counts into ``ctrl`` and ``actuator_force``.
  - ``nactuator`` :sup:`new`, ``nout`` :sup:`new`, ``npolygonmax`` :sup:`new` and
    ``nmeshdegmax`` :sup:`new` size accessors.
  - ``nefm0dof`` :sup:`new` and ``nefm0_l`` :sup:`new` size accessors, and the
    ``efm0_dofid`` :sup:`new`, ``efm0_l_rownnz`` :sup:`new`, ``efm0_l_rowadr`` :sup:`new`,
    ``efm0_l_colind`` :sup:`new` and ``efm0_l`` :sup:`new` array accessors for the constant
    part of the implicit effective-metric factorization (matching the |mj_data| ``efm_*``
    accessors). Mutating ``efm0_dofid``, ``efm0_l_rownnz``, ``efm0_l_rowadr`` and
    ``efm0_l_colind`` needs ``unsafe``, because the engine uses their values as unchecked
    addresses and counts.
  - ``flg_gravcomp`` :sup:`new`, ``flg_surfacevel`` :sup:`new` and ``flg_adhesion`` :sup:`new`
    flag accessors.

- |mj_data|:

  - ``qfrc_adhesion`` :sup:`new` (passive contact adhesion force; also available in the joint
    views) and ``flexelem_krot`` :sup:`new` array accessors.
  - ``efm_c`` :sup:`new`, ``efm_k_rownnz`` :sup:`new`, ``efm_k_rowadr`` :sup:`new`,
    ``efm_k_colind`` :sup:`new`, ``efm_k_val`` :sup:`new`, ``efm_dofid`` :sup:`new` and
    ``efm_l`` :sup:`new` array accessors, plus the ``efm_active`` :sup:`new`, ``nefm_k`` :sup:`new`,
    ``nefmdof`` :sup:`new` and ``nefm_l`` :sup:`new` scalar accessors, exposing the implicit
    effective-metric (M+K) solver state. Mutating ``efm_k_rownnz``, ``efm_k_rowadr``,
    ``efm_k_colind`` and ``efm_dofid`` needs ``unsafe``, because the engine uses their values as
    unchecked addresses and counts.

- Model-editing (``MjSpec``) accessors: ``MjsBody::simple()`` :sup:`new`,
  ``MjsGeom::surfacevel()`` :sup:`new`, ``MjsGeom::adhesion()`` :sup:`new`,
  ``MjsPair::adhesion()`` :sup:`new` and ``MjsActuator::ctrlspec()`` :sup:`new`
  (with the matching ``set_``/``with_`` methods; ``surfacevel`` is a fixed-size array
  field, so it has ``surfacevel_mut()``/``with_surfacevel()`` instead of ``set_surfacevel()``).
- Added ``MjtCtrlChart`` :sup:`new` as an alias for the new ``mjtCtrlChart`` enum (the
  ``m->actuator_ctrlspec`` value of an so3 actuator, because the field is scoped by ``gaintype``).

*Unified logging API*

- Added ``MjLogConfig`` :sup:`new` and ``MjLogMessage`` :sup:`new` aliases (with ``MjtLogLevel``
  :sup:`new` and ``MjtLogTopic`` :sup:`new`) for the unified logging API of MuJoCo 3.10.0.
- Added the free logging functions
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_config` :sup:`new` (``mju_getLogConfig``),
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>set_log_config` :sup:`new` (``mju_setLogConfig``),
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_message` :sup:`new` (``mju_message``),
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_info` :sup:`new` (``mju_info``),
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_warning` :sup:`new` (``mju_warning``) and
  :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_error` :sup:`new` (``mju_error``).
- Added the new top-level ``logging`` module, which sends MuJoCo's messages to the
  `log <https://docs.rs/log/0.4.34/log/>`_ crate, so they reach the same backend as the rest of the
  application. :docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` :sup:`new`
  installs a hook for ``log`` routing.
  :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` :sup:`new` registers a user handler.

*New accessors for 3.10.0 structs*

- Added ``MjsAuthored`` :sup:`new` and the |mj_spec| ``authored()`` :sup:`new` accessor, exposing the
  authored-field tracking bitmasks introduced in MuJoCo 3.10.0. ``MjsAuthored`` is a
  plain-data struct; its bitmask fields are read directly (e.g. ``authored.disableflags``).
- Added ``MjtConflict`` :sup:`new` and the ``MjsCompiler::conflict()`` :sup:`new` accessor (with the
  matching ``set_``/``with_`` methods), exposing the attach-time conflict-resolution policy
  introduced in MuJoCo 3.10.0.
- Added ``MjsCompiler::authored()`` :sup:`new` (read-only), exposing the bitmask of which
  compiler fields were explicitly set by the user.
- |mj_spec| gained two compilation-warning accessors, wrapping ``mjs_numWarnings`` and
  ``mjs_getWarning``:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>num_warnings` :sup:`new`
    and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>warning` :sup:`new`.

*New* |mj_data| *methods*

- |mj_data|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>full_m` :sup:`new`, converting the
    sparse inertia matrix into a dense ``nv x nv`` matrix; wraps ``mj_fullM``. Returns
    ``Err(MjDataError::BufferTooSmall)`` when ``dst`` is shorter than ``nv * nv``.
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_threadpool` :sup:`new`, creating a
    worker thread pool with ``nthread`` threads for parallel simulation; wraps ``mju_threadpool``.
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>threadlock` :sup:`new` (and
    ``set_threadlock``, ``with_threadlock``), exposing the new ``threadlock`` field that disables
    stack freeing during threaded execution.

*RNE into a caller buffer*

- Added :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>rne_into` :sup:`new` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_rne_into` :sup:`new`,
  wrapping ``mj_rne`` to write the inverse-dynamics force into a caller buffer. ``try_rne_into``
  returns ``Err(MjDataError::BufferTooSmall)`` when ``result`` holds fewer than ``nv`` elements.

*Cloneable renderer builder*

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` now derives ``Clone``,
  so a configured builder can be duplicated before ``build``. It already implemented ``Debug``
  and ``Default``, so all builder types now provide ``Debug`` + ``Clone`` + ``Default``.

*Procedural flex creation*

- |mjs_body|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>add_flexcomp` :sup:`new` (and the
    fallible ``try_add_flexcomp``), wrapping ``mjs_makeFlex`` to procedurally create a flex with
    auto-generated bodies, joints, and optional equality constraints (the programmatic equivalent of
    the ``flexcomp`` element). The generation is configured through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjFlexcompConfig` :sup:`new` builder.

*New type aliases*

- Added :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjrCamera` :sup:`new` as a type
  alias for ``MjvGLCamera``, matching MuJoCo's own ``mjrCamera`` alias convention.

*New accessors for 3.12.0 fields*

- |mj_model|:

  - ``light_softness`` :sup:`new` and ``mesh_extrema`` :sup:`new` array accessors, with
    matching ``softness`` and ``extrema`` fields in the light and mesh info views. Mutating
    ``mesh_extrema`` needs ``unsafe``, in the array accessor and through the mesh view, because
    MuJoCo uses its values as unchecked vertex indices.
  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_input_name`
    :sup:`new`, wrapping ``mj_actuatorInputName``, gives the name of one control input of an
    actuator, for example ``pos`` for a ``pid`` actuator or ``rx`` for an ``orientation``
    actuator on the exponential-map chart. It returns ``None`` when the actuator type defines
    no input names.

- |mjs_light|: ``softness`` :sup:`new`.
- |mjs_actuator|: ``velrange`` :sup:`new` and ``ffrange`` :sup:`new`.
- Added ``MjtCtrlInput`` :sup:`new` as an alias for the new ``mjtCtrlInput`` enum, whose values
  are OR-ed into ``m->actuator_ctrlspec`` to select the inputs of a servo-family actuator.

*New servo actuator constructors*

- |mjs_actuator|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator::<method>set_to_pid` :sup:`new`,
    wrapping ``mjs_setToPID``, makes the actuator a PID controller on a single force output. The
    controller is configured through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PidConfig` :sup:`new` builder.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator::<method>set_to_orientation`
    :sup:`new`, wrapping ``mjs_setToOrientation``, makes the actuator a geodesic PD orientation
    servo on a ball joint or a site. It is configured through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>OrientationConfig` :sup:`new` builder.

*Relaxed mutability on fields that cannot cause a memory fault*

- Mutating |mj_model| ``paths`` and |mjr_context| ``texture_type`` no longer needs ``unsafe``.
  MuJoCo fills ``paths`` during the compile step and never reads it back, and it only compares
  ``textureType`` against ``mjtTexture`` variants, so a write can at worst give a wrong asset
  path or wrong rendering, never a memory fault.

.. rubric:: New examples

- :gh-example:`Actuator inputs <visualization/viewer/actuator_inputs.rs>` --- shows the actuator
  controls of the :ref:`mj_rust_viewer` for every input flavour: actuator types that define no
  input names, the named multi-input control blocks of a ``pid``, a ``dcmotor`` and an
  ``orientation`` actuator, an actuator without a name, and an actuator without control inputs.
  The model also holds a limited ball joint, an unlimited ball joint and a free joint, for the
  sliders of the "Joint" window.
- :gh-example:`Logging <simulation/logging.rs>` --- shows the three ways to receive a MuJoCo
  message: the default handler with its ``MjLogConfig``, the logging hook next to ``env_logger``
  (with the target that carries the topic), and a custom handler that reads the ``MjLogMessage``.

.. rubric:: Bug fixes

- The viewer no longer clamps ``ctrl`` to the range of its slider. While the "Actuator" window
  was open, it wrote the clamped control back on every frame, so a program that set a control
  outside the slider range lost that value.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frame` for a tracking
  camera: MuJoCo indexes ``subtree_com`` by ``trackbodyid`` after testing only that it is not
  negative. ``frame`` now validates the id against the model and panics on an out-of-range one.
- Closed a division by zero and a non-terminating loop reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_halton`: MuJoCo divides by ``base`` without
  checking it, so ``base = 0`` divided by zero and ``base = 1`` never returned. The wrapper now
  panics unless ``base`` is 2 or more.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>id` now returns ``None``
  for a default class. ``mjs_getId`` casts to ``mjCBase``, which ``mjCDef`` does not derive from,
  so the call read an unrelated offset instead of an id.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element` now reports
  ``UnsupportedOperation`` for a default class of this spec, instead of ``DeleteFailed``, and no
  longer panics on an element whose stored name is not valid UTF-8. It also reports
  ``UnsupportedOperation`` for a frame and for a tendon wrap. A frame's element type lies outside
  MuJoCo's element-list array, so the deletion indexed past the end of it, and a wrap's slot in that
  array is empty, so the deletion reported success while the element stayed in its owner's list and
  the spec then released it twice.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>start`: MuJoCo scales
  the local mass by ``flex_edgenum[flexselect]`` after testing only that the id is not negative.
  ``start`` now validates ``flexselect`` against the model and panics on an out-of-range id.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom` now
  writes every field that ``mjv_initGeom`` leaves unwritten over uninitialized memory: ``objtype``
  becomes ``mjOBJ_UNKNOWN``, ``objid`` becomes ``-1``, ``category`` becomes ``mjCAT_DECOR``,
  ``segid`` becomes the geom's own index, and ``label``, ``camdist`` and ``transparent`` are
  cleared. The renderer uses ``objid`` as an unchecked index into the skin and flex arrays, and it
  casts no shadow for a ``mjCAT_DECOR`` geom.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>set_at` now treats a
  negative ``linepnt`` entry as an empty plot. The count is public, and casting a negative value to
  ``usize`` made the range test pass, so the call wrote outside the plot's data range.
- :docs-rs:`~mujoco_rs::vis_common::<fn>sync_geoms` now panics when the two scenes were created for
  models with different signatures. A copied geom keeps its ``objid``, which the renderer uses as
  an unchecked index into the destination scene's flex and skin arrays.
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` no longer rebuilds the window
  and the OpenGL context when it is called after the window closed. winit re-emits ``Resumed`` on
  the first call after the event loop exited, and the rebuild destroyed the context that the live
  ``mjrContext`` holds its objects in.
- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_pixels` no longer
  wraps when the ``rgb`` byte count (the pixel count times 3) overflows ``usize`` on a 32-bit
  target, which let a short ``rgb`` buffer pass the size check; it reports ``InvalidViewport``
  instead.
- Closed a crash reachable from safe code when naming a tendon wrap. ``mjCWrap`` leaves its element
  type at ``mjOBJ_UNKNOWN``, which makes MuJoCo's duplicate-name check index a null list, so
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name` on a |mjs_wrap|
  now reports ``UnsupportedOperation`` and ``with_name`` panics. A wrap carries no name of its own:
  ``name`` reports the wrapped object's name.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>render`: the
  renderer uploads one vertex buffer per scene skin, indexing an array that the context sizes by
  its own model's skin count, which is null when that model has none. ``render`` now panics when
  the context holds fewer skins than the scene.
- :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new` no longer truncates its
  ``width``, ``height`` and ``max_user_geom`` arguments, which turned a value above ``u32::MAX``
  into a small valid one, so the renderer used a size or a geom capacity the caller never asked
  for. The three arguments now saturate at ``u32::MAX``.

.. rubric:: Other changes

- Following the reworked FFI struct field-visibility rules, the fields of
  ``MjsOrientation`` (``type_``, ``axisangle``, ``xyaxes``, ``zaxis``, ``euler``) and of
  the raw ``mjsElement`` handle (``elemtype``, ``signature``) are now ``pub`` again,
  classifying them as plain-data structs and allowing direct field access.

