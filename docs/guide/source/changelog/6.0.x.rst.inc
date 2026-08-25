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

- Use :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`,
  which now receives the ``egui::Ui`` and shared state ``Arc<Mutex<>>`` directly for the user
  to lock.

*MuJoCo FFI upgraded to 3.12.0*

- MuJoCo-rs now builds against MuJoCo 3.12.0 FFI, which rolls up the upstream 3.10.0,
  3.11.0 and 3.12.0 releases. This is a breaking dependency-level change for consumers
  tied to older MuJoCo C ABI details.
  This directly affects all the wrappers, separate changelog will not be provided. 
  See MuJoCo's changelog for this release:
  https://mujoco.readthedocs.io/en/3.12.0/changelog.html

``MjvCamera::move_`` *no longer takes a scene*

- |mjv_camera| ``move_`` lost its ``scene: &MjvScene`` parameter, following the upstream
  removal of the unneeded ``mjvScene`` argument from ``mjv_moveCamera``.

*Stricter* |mjr_context| *accessor types*

- ``gl_initialized``, ``window_available``, ``window_stereo`` and ``window_doublebuffer`` now
  read as ``bool`` instead of ``i32``; MuJoCo stores only 0 or 1 in them.
- ``font_scale`` and ``read_depth_map`` now read as ``MjtFontScale`` and ``MjtDepthMap`` instead
  of ``i32``.

*Unsafe mutation of* |mj_model| ``actuator_gaintype``

- ``actuator_gaintype_mut`` is now an ``unsafe fn``; the actuator views are affected the same
  way, so a write through a view needs ``as_mut_slice`` inside ``unsafe``.

*Unsafe geom creation on* |mjv_scene|

- |mjv_scene| ``create_geom`` and ``try_create_geom`` are now ``unsafe fn``.

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
  ``Option<&MjsDefault>``. Handle the ``None`` case.

*Changed* |mjr_context| ``set_buffer`` *parameter type*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_buffer` now
  accepts ``framebuffer: MjtFramebuffer`` instead of ``i32``.

.. rubric:: Deprecations

- Deprecated |mj_model|'s ``try_clone`` --- its ``MjModelError::AllocationFailed`` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_data|'s ``try_clone`` --- its ``MjDataError::AllocationFailed`` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_spec|'s ``try_new``, ``try_add_frame`` on |mjs_body| and |mjs_frame|, and every
  macro-generated ``try_add_*`` element constructor on |mj_spec|, |mjs_body| and |mjs_frame| ---
  their ``MjEditError::AllocationFailed`` arm is unreachable. Use ``new`` and the
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
- A press on the scene now takes the selection away from every viewer window.

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

*New compilation-warning accessors*

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

*RNE into a caller buffer*

- Added :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>rne_into` :sup:`new` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_rne_into` :sup:`new`,
  wrapping ``mj_rne`` to write the inverse-dynamics force into a caller buffer. ``try_rne_into``
  returns ``Err(MjDataError::BufferTooSmall)`` when ``result`` holds fewer than ``nv`` elements.

*Cloneable renderer builder*

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` now derives ``Clone``,
  so a configured builder can be duplicated before ``build``.

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

*Actuator input names*

- |mj_model|:

  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_input_name`
    :sup:`new`, wrapping ``mj_actuatorInputName``, gives the name of one control input of an
    actuator. It returns ``None`` when the actuator type defines no input names.

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
  A write can at worst give a wrong asset path or wrong rendering, never a memory fault.

.. rubric:: New examples

- :gh-example:`Actuator inputs <visualization/viewer/actuator_inputs.rs>` --- shows the actuator
  controls of the :ref:`mj_rust_viewer` for every input flavour. The model also holds a limited
  ball joint, an unlimited ball joint and a free joint, for the sliders of the "Joint" window.
- :gh-example:`Logging <simulation/logging.rs>` --- shows the three ways to receive a MuJoCo
  message: the default handler with its ``MjLogConfig``, the logging hook next to ``env_logger``
  (with the target that carries the topic), and a custom handler that reads the ``MjLogMessage``.

.. rubric:: Bug fixes

- The viewer no longer clamps ``ctrl`` to the range of its slider. While the "Actuator" window
  was open, it wrote the clamped control back on every frame, so a program that set a control
  outside the slider range lost that value.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frame` for a tracking
  camera: ``frame`` now validates the id against the model and panics on an out-of-range one.
- Closed a division by zero and a non-terminating loop reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_halton`: MuJoCo divides by ``base`` without
  checking it, so ``base = 0`` divided by zero and ``base = 1`` never returned. The wrapper now
  panics unless ``base`` is 2 or more.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>id` now returns ``None``
  for a default class.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element` now reports
  ``UnsupportedOperation`` for a default class of this spec, instead of ``DeleteFailed``, and no
  longer panics on an element whose stored name is not valid UTF-8. It also reports
  ``UnsupportedOperation`` for a frame and for a tendon wrap.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>start`:
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
  models with different signatures.
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` no longer rebuilds the window
  and the OpenGL context when it is called after the window closed.
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
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>render`:
  ``render`` now panics when the context holds fewer skins than the scene.
- :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new` no longer truncates its
  ``width``, ``height`` and ``max_user_geom`` arguments, which turned a value above ``u32::MAX``
  into a small valid one, so the renderer used a size or a geom capacity the caller never asked
  for. The three arguments now saturate at ``u32::MAX``.

.. rubric:: Other changes

- Following the reworked FFI struct field-visibility rules, the fields of
  ``MjsOrientation`` (``type_``, ``axisangle``, ``xyaxes``, ``zaxis``, ``euler``) and of
  the raw ``mjsElement`` handle (``elemtype``, ``signature``) are now ``pub`` again,
  classifying them as plain-data structs and allowing direct field access.

