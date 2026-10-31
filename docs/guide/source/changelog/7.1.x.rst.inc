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

- The RGBA section and the headlight colors of the :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`
  UI now use color pickers instead of numeric fields, so colors can be chosen visually.
- The contact override "Sol Imp" and "Friction" fields and the free camera elevation now accept
  only the values that MuJoCo applies.
- The joint window now shows the unit of each value.
- Added a sensor window that shows the value of each sensor component with its unit. A value with
  a natural range (for example a quaternion component) also shows a bar.
- The joint and actuator windows now align their values in a grid with separators, as the sensor
  window does.

.. rubric:: New examples

- :gh-example:`Batched simulation <simulation/batched.rs>` --- runs many simulations in parallel
  using :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatch`.

.. rubric:: Bug fixes

- The viewer no longer panics on a drag after a ctrl-double-click over the sky. Additionally, the
  selection is now not cleared when double-clicking over the sky whilst ctrl or alt key is pressed.

.. rubric:: Other changes

- The crates now point to the ``LICENSE`` file instead of the ``MIT OR Apache-2.0`` SPDX
  expression. The file adds a reservation of rights for text and data mining and machine learning
  on top of the MIT or Apache license. Read it before you use the crates.
