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
- The joint and actuator windows now show the unit of each value, when the viewer can derive it
  from the model.
- Added a sensor window that shows the value of each sensor component, with its unit when the
  viewer can derive it. A value with a natural range (for example a quaternion component) also
  shows a bar.
- The joint and actuator windows now align their values in a grid with separators, as the sensor
  window does.
- The quaternion components of a free joint now use sliders, as those of a ball joint do. The joint
  window no longer shows the rotation angle of a limited ball joint, and it no longer normalizes the
  quaternion after an edit. MuJoCo normalizes the quaternion in ``qpos`` at the next step.
- The actuator, joint, sensor and equality windows now have a search bar. The window shows only
  the elements whose name starts with the search text.

.. rubric:: New examples

- :gh-example:`Batched simulation <simulation/batched.rs>` --- runs many simulations in parallel
  using :docs-rs:`~mujoco_rs::wrappers::mj_batch::<struct>MjBatch`.

.. rubric:: Bug fixes

- The viewer no longer panics on a drag after a ctrl-double-click over the sky. Additionally, the
  selection is now not cleared when double-clicking over the sky whilst ctrl or alt key is pressed.
- The viewer UI panel no longer changes a model or visualization option that is outside the range
  of its field (for example "LS Iter" set to 0). The range now limits only the values that the user
  enters.
- The "Iterations" field of the viewer UI now accepts 0.
- The "Frustum" field now accepts values above 1.
- The built-in windows and the camera tracking dialog no longer share their state with a user
  `egui <https://docs.rs/egui/0.36.1/egui/>`_ window or dialog that has the same title or ID.
- Two elements whose names are equal in a joint or actuator window (for example a joint named
  ``Joint 3`` and the unnamed joint 3) no longer open and close together.

.. rubric:: Other changes

- The ``actuator_inputs`` example is now
  :gh-example:`Control windows <visualization/viewer/control_windows.rs>`. Now, the example also
  shows the sensor window and the equality window.
- The crates now point to the ``LICENSE`` file instead of the ``MIT OR Apache-2.0`` SPDX
  expression. The file adds a reservation of rights for text and data mining and machine learning
  on top of the MIT or Apache license. Read it before you use the crates.
