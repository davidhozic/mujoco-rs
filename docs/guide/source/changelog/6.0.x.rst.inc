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
  https://github.com/emilk/egui/blob/main/CHANGELOG.md

*UI callbacks receive a* `egui::Ui <https://docs.rs/egui/0.36.1/egui/struct.Ui.html>`_ *and the shared state*

- The closure passed to
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` now receives
  ``&mut egui::Ui`` and ``&Arc<Mutex<ViewerSharedState>>``
  (:docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`), instead of ``&egui::Context``
  (`egui::Context <https://docs.rs/egui/0.36.1/egui/struct.Context.html>`_) and
  ``&mut MjData<Box<MjModel>>`` (|mj_data|, |mj_model|).
  ``MjViewer::add_ui_callback_detached`` is removed as ``add_ui_callback`` now handles both cases.

*MuJoCo FFI upgraded to 3.12.0*

- MuJoCo-rs now builds against MuJoCo 3.12.0 FFI, which rolls up the upstream 3.10.0,
  3.11.0 and 3.12.0 releases. This is a breaking dependency-level change for consumers
  tied to older MuJoCo C ABI details.
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
  of a single mode selector. MuJoCo 3.12.0 also moved the ``dcmotor`` controller gains from voltage space to torque
  space, so the same ``controller`` array now gives a different force. The old velocity mode's
  integrated-velocity term is gone.

*Relaxed* |mj_data| ``body_awake`` *view mutability*

- The body views now hold ``awake`` as a :docs-rs:`~mujoco_rs::util::<struct>PointerViewMut`, which
  matches the array accessor, already safe to mutate. Code that wrote through ``as_mut_slice`` must
  drop the call and the surrounding ``unsafe``.

``SpecItem::default`` *returns an* ``Option``

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>default` now returns
  ``Option<&MjsDefault>`` (:docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsDefault`). Handle
  the ``None`` case.

*Changed* |mjr_context| ``set_buffer`` *parameter type*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_buffer` now
  accepts ``framebuffer: MjtFramebuffer``
  (:docs-rs:`~mujoco_rs::wrappers::mj_rendering::<type>MjtFramebuffer`) instead of ``i32``.

.. rubric:: New dependencies

- Added `log <https://docs.rs/log/0.4.34/log/>`__ as a mandatory dependency.

.. rubric:: Deprecations

- Deprecated |mj_model|'s
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_clone` --- its
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>AllocationFailed` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_data|'s :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_clone`
  --- its :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>AllocationFailed` arm is
  unreachable. Use ``clone`` instead.

- Deprecated |mj_spec|'s
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>try_new`,
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>try_add_frame` on |mjs_body|
  and |mjs_frame|, and every
  macro-generated ``try_add_*`` element constructor on |mj_spec|, |mjs_body| and |mjs_frame| ---
  their :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>AllocationFailed` arm is
  unreachable. Use ``new`` and the
  panicking ``add_*`` methods instead.
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>try_add_default` and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>try_add_flexcomp` keep their
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

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody::<method>add_flexcomp` :sup:`new` (and the
    fallible ``try_add_flexcomp``), wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_makeFlex` to
    procedurally create a flex with auto-generated bodies, joints, and optional equality constraints
    (the programmatic equivalent of the ``flexcomp`` element). The generation is configured through the new
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

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator::<method>set_to_pid` :sup:`new`,
    wrapping :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_setToPID`, makes the actuator a PID controller
    on a single force output. The controller is configured through the new
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>PidConfig` :sup:`new` builder.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator::<method>set_to_orientation`
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
  :docs-rs:`~mujoco_rs::error::<enum>MjEditError::<variant>UnsupportedOperation` for a default
  class of this spec, instead of
  :docs-rs:`~mujoco_rs::error::<enum>MjEditError::<variant>DeleteFailed`, and no
  longer panics on an element whose stored name is not valid UTF-8. It also reports
  ``UnsupportedOperation`` for a frame and for a tendon wrap.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>start`:
  ``start`` now validates ``flexselect`` against the model and panics on an out-of-range id.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom` now
  writes every field that :docs-rs:`~mujoco_rs::mujoco_c::<fn>mjv_initGeom` leaves unwritten over
  uninitialized memory: ``objtype`` becomes
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtObj::<variant>mjOBJ_UNKNOWN`, ``objid``
  becomes ``-1``, ``category`` becomes
  :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjtCatBit::<variant>mjCAT_DECOR`,
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
  target, which let a short ``rgb`` buffer pass the size check; it reports
  :docs-rs:`~mujoco_rs::error::<enum>MjrContextError::<variant>InvalidViewport` instead.
- Closed a crash reachable from safe code when naming a tendon wrap. ``mjCWrap`` leaves its element
  type at ``mjOBJ_UNKNOWN``, which makes MuJoCo's duplicate-name check index a null list, so
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name` on a |mjs_wrap|
  now reports ``UnsupportedOperation`` and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>with_name` panics. A wrap
  carries no name of its own:
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>name` reports the wrapped
  object's name.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>render`:
  ``render`` now panics when the context holds fewer skins than the scene.
- :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new` no longer truncates its
  ``width``, ``height`` and ``max_user_geom`` arguments, which turned a value above ``u32::MAX``
  into a small valid one, so the renderer used a size or a geom capacity the caller never asked
  for. The three arguments now saturate at ``u32::MAX``.

.. rubric:: Other changes

- Following the reworked FFI struct field-visibility rules, the fields of
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsOrientation` (``type_``, ``axisangle``,
  ``xyaxes``, ``zaxis``, ``euler``) and of the raw
  :docs-rs:`~mujoco_rs::mujoco_c::<type>mjsElement` handle (``elemtype``, ``signature``) are now
  ``pub`` again,
  classifying them as plain-data structs and allowing direct field access.

