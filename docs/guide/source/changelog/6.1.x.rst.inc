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

  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile_with_vfs` :sup:`new`
  for compiling a model with assets loaded on a virtual filesystem;
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>encode` :sup:`new`
  (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_encode`) for encoding a specification to a file with
  the encoder registered for the content type;
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>encode_with_vfs` :sup:`new`
  for reading the encoder's assets through a virtual filesystem.

*MjModel*

- New methods:

  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>encode` :sup:`new`
  (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_encode`) for encoding a compiled model to a file with
  the encoder registered for the content type, and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>encode_with_vfs` :sup:`new`
  for reading the encoder's assets through a virtual filesystem.

*Asset cache*

- Added wrappers around MuJoCo's asset cache, which keeps assets loaded from a file or a virtual
  file system across recompilations, to prevent reprocessing:

  - :docs-rs:`~mujoco_rs::wrappers::mj_cache::<struct>MjCache` :sup:`new` --- the zero-sized handle,
    with internal mutability (MuJoCo has a global mutex).

.. rubric:: Bug fixes
.. rubric:: Other changes
