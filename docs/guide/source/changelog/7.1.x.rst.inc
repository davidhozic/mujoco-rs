7.1.0 (MuJoCo 3.17.0)
======================

.. rubric:: Error handling

- New enum :docs-rs:`~mujoco_rs::error::<enum>MjBatchError` :sup:`new`.

.. rubric:: New features and improvements

*Batch API* :sup:`new`

- Added :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatch` :sup:`new` (with its methods) and
  :docs-rs:`~mujoco_rs::wrappers::mj_batch::<type>MjBatchConfig` :sup:`new`. The former can be used
  for batching multiple simulation instances across multiple-threads through a unified API.

