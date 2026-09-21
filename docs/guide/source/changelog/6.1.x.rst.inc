6.1.0 (MuJoCo 3.12.0)
======================

.. rubric:: Breaking changes
.. rubric:: New dependencies
.. rubric:: Deprecations
.. rubric:: Error handling

- Added :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>AttachFailed` :sup:`new`.

.. rubric:: New features and improvements

*Model editing*

- Added traits:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>Attach` :sup:`new` and
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>AttachTo` :sup:`new`.

  The first trait defines
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>Attach::<method>attach_by_deep_copy` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>Attach::<method>attach_by_reference`
  (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_attach`), which allow elements to be attached to
  ``Self`` (implementor of ``Attach``) based on the ``AttachTo`` trait. For example,
  implementations of ``AttachTo<MjsFrame>`` allow the implementor elements to be attached to
  |mjs_frame|.

  ``attach_by_deep_copy`` gives the parent a copy of every element of the child, thus both
  specifications stay usable. ``attach_by_reference`` shares the elements, which is the default
  behavior in MuJoCo, and is therefore ``unsafe``. The safe usage conditions are written in the
  method's docstring.

  The prelude exports both traits.

- Added the :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>UserValued` :sup:`new` trait, which
  stores an arbitrary Rust value on a specification element under a string key:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>UserValued::<method>user_value` :sup:`new`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>UserValued::<method>set_user_value` :sup:`new`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>UserValued::<method>remove_user_value` :sup:`new`.

- Added support for sending |mj_spec| to other threads by wrapping it into
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>SendableSpec` :sup:`new`,
  which implements `Send <https://doc.rust-lang.org/std/marker/trait.Send.html>`__.
  A ``SendableSpec`` can be created from ``MjSpec`` by calling the ``unsafe`` method
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>into_sendable` :sup:`new`,
  or the equivalent
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>SendableSpec::<method>new` :sup:`new`.
  The wrapped specification comes back out with
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>SendableSpec::<method>take` :sup:`new`.

- New methods:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile_with_vfs` :sup:`new`
    for compiling a model with assets loaded on a virtual filesystem;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>encode` :sup:`new`
    for encoding a specification to a file with
    the encoder registered for the content type;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>encode_with_vfs` :sup:`new`
    for reading the encoder's assets through a virtual filesystem.

- Added setters for the |mj_spec| size attributes, which were read-only until now:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_memory` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuserdata` :sup:`new`
    and :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nkey` :sup:`new`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_body` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_jnt` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_geom` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_site` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_cam` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_tendon` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_actuator` :sup:`new`
    and :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>set_nuser_sensor`
    :sup:`new`.

*MjModel*

- New methods:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>encode` :sup:`new`
    for encoding a compiled model to a file with the encoder registered for the content type;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>encode_with_vfs` :sup:`new`
    for reading the encoder's assets through a virtual filesystem.

*MjData*

- New methods:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_const` :sup:`new`
    --- available only when ``M`` in ``MjData<M>``
    implements :docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>save_keyframe` :sup:`new`
    for copying the current state into a model keyframe --- also only available when ``M``
    implements implements ``ModelTypeMut``.

*Asset cache*

- Added wrappers around MuJoCo's asset cache, which keeps assets loaded from a file or a virtual
  file system across recompilations, to prevent reprocessing:

  - :docs-rs:`~mujoco_rs::wrappers::mj_cache::<struct>MjCache` :sup:`new` --- the zero-sized handle,
    with internal mutability (MuJoCo has a global mutex).

.. rubric:: Bug fixes

*Renderer*

- Offscreen rendering using EGL now fallbacks to other supported devices when the first device fails.

.. rubric:: Other changes
