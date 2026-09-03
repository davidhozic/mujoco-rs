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

  The first defines :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>Attach::<method>attach`, which
  allows elements to be attached to ``Self`` (implementor of ``Attach``) based on the ``AttachTo`` trait. For example,
  implementations of ``AttachTo<MjsFrame>`` allow the implementor elements to be attached to
  |mjs_frame|.

.. rubric:: Bug fixes
.. rubric:: Other changes
