[Unreleased] 7.1.0 (MuJoCo 3.17.0)
==================================

.. rubric:: Error handling

- New enum :docs-rs:`~mujoco_rs::error::<enum>MjBatchError` :sup:`new`.

.. rubric:: New features and improvements

*Batch API* :sup:`new`

- Added :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatch` :sup:`new` (with its methods) and
  :docs-rs:`~mujoco_rs::wrappers::mj_batch::<type>MjBatchConfig` :sup:`new`. The former can be used
  for batching multiple simulation instances across multiple-threads through a unified API.
- Added :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatchRecordBuilder` :sup:`new`, the
  builder of the output buffer of a state that
  :docs-rs:`~~mujoco_rs::wrappers::mj_batch::<struct>MjBatch::<method>rollout` records after each
  step.

*Viewer UI*

- The RGBA section of the :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` UI now uses color pickers
  instead of numeric fields, so colors can be chosen visually.

.. rubric:: New examples

- :gh-example:`Batched simulation <simulation/batched.rs>` --- runs many simulations in parallel
  using :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatch`.

.. rubric:: Other changes

- The crates now point to the ``LICENSE`` file instead of the ``MIT OR Apache-2.0`` SPDX
  expression. The file adds a reservation of rights for text and data mining and machine learning
  on top of the MIT or Apache license. Read it before you use the crates.
