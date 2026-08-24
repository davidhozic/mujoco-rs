4.0.1 (MuJoCo 3.8.0)
======================
.. rubric:: Bug fixes

- When using the ``auto-download-mujoco`` feature, the build script now checks for existence
  of the library before downloading it. When the library exists, its download and extraction
  are skipped.


4.0.0 (MuJoCo 3.8.0)
======================

.. rubric:: Breaking changes

*MuJoCo upgraded to 3.8.0*

- MuJoCo-rs 4.0.0 is based on **MuJoCo 3.8.0**, upgraded from 3.6.0.
  MuJoCo 3.8.0 introduces breaking changes to its C API (renamed and removed constants,
  new struct fields). Code relying on specific MuJoCo enum values or internal constants
  may need to be updated.

*MjData::geom_distance now requires mutable access*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>geom_distance` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_geom_distance`
  now take ``&mut self`` instead of ``&self``. This follows MuJoCo 3.7.0, where
  ``mj_geomDistance`` mutates ``mjData`` internally.

*Model-editing wrappers now expose polynomial stiffness and damping coefficients*

- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsJoint`:
  ``stiffness`` and ``damping`` now expose coefficient arrays
  instead of scalar ``f64`` values.
- |mjs_tendon|:
  ``stiffness`` and ``damping`` now expose coefficient arrays
  instead of scalar ``f64`` values.

*MjsFlex::vertcollide removed*

- ``MjsFlex::vertcollide`` is no longer available. MuJoCo 3.7.0 removed the
  upstream ``mjsFlex::vertcollide`` field, so the wrapper no longer exposes it.

*MjData::model_mut is now unsafe*

- Because the :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_mut` allows full
  model replacement in unsafe ways, it is now marked unsafe.

*MjViewer::add_ui_callback closure now receives Box<MjModel>*

- The passive model stored inside the viewer changed from ``Arc<MjModel>`` to
  ``Box<MjModel>``. The closure passed to
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`
  now receives ``&mut MjData<Box<MjModel>>``.

.. rubric:: Error handling

- Added a new variant for handling invalid indices in
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_max_contacts`:
  ``MjModelError::InvalidIndex``.

.. rubric:: New features and improvements

- Updated FFI bindings and wrappers to match MuJoCo 3.8.0 headers.
- |mj_model| now exposes the new joint, dof, tendon, and actuator coefficient
  arrays and actuator-linkage fields added in newer MuJoCo releases.
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsActuator` now exposes its
  ``damping`` coefficient array and ``armature`` field.
- :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtLRMode` is now exported for
  typed access to ``MjLROpt.mode``.
- In light of :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_mut` becoming unsafe,
  six new methods are created to allow safe access to model fields:
  
  Immutable getters:
  
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_opt`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_vis`, and
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_stat`
  
  Mutable accessors:
  
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_opt_mut`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_vis_mut`, and
  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_stat_mut`.

*MjViewer model parameter synchronization*

- :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState` now provides methods to sync model parameters:
  
  - :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_model` syncs all model
    parameters (physics options, visualization, and statistics) bidirectionally, detecting model
    changes via signature comparison.
  - :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_model_opt`,
  - :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_model_vis`, and
  - :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_model_stat`
  
  allow syncing model physics options, visualization parameters, and statistics individually.
  Each performs a bidirectional three-way merge between the viewer's passive model and the
  supplied struct, without requiring ``unsafe`` access.
  
  Corresponding getter methods :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>last_opt_sync_time`,
  :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>last_vis_sync_time`, and
  :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState::<method>last_stat_sync_time` return
  when each parameter type was last synchronized, enabling UI warnings for out-of-sync parameters.

- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` now provides proxy methods
  (:docs-rs:`~mujoco_rs::viewer::<struct>MjViewer::<method>sync_model`,
  :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer::<method>sync_model_opt`,
  :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer::<method>sync_model_vis`,
  :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer::<method>sync_model_stat`) that delegate to the
  corresponding :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState` methods, enabling convenient
  parameter synchronization without manual lock management.

*Viewer camera tracking modal*

- The viewer UI now features an interactive **camera tracking modal** for selecting bodies to track.
  Select "Track" in the camera panel to open a modal with a scrollable grid of body buttons.

.. rubric:: Other changes

- Removed the ``stl_mesh`` example. It existed in MuJoCo-rs 3.0.1 due to MuJoCo 3.6.0 requiring a plugin to load
  STL meshes. MuJoCo 3.7.0+ fixes this, thus the example is no longer required.
- Reduced default width of the viewer's UI side panel to 200.0.
- Physics options (integrator, cone, jacobian, solver) in the viewer UI display a yellow warning when
  model parameters (opt/vis/stat) are out of sync with their initial values, indicating that parameters
  have been modified in the viewer.

