.. _changelog:

==================
Changelog
==================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mj_geomview| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataView`
.. |mj_geomviewmut| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataViewMut`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjs_tendon| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsTendon`
.. |mjs_wrap| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsWrap`
.. |mjv_camera| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`
.. |mjr_context| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`


Versioning
=================
This project uses `semantic versioning <https://semver.org/>`_.
This means that any incompatible changes increase the major version (**Y**.x.x).
This also includes breaking changes that MuJoCo itself introduced, thus even an
update of MuJoCo alone can increase the major version.

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
  
  allow syncing model physics options, visualization parameters, and statistics individually from
  the viewer's passive state to external structures without requiring ``unsafe`` access.
  
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

.. rubric:: Removed examples

- Removed the ``stl_mesh`` example. It existed in MuJoCo-rs 3.0.1 due to MuJoCo 3.6.0 requiring a plugin to load
  STL meshes. MuJoCo 3.7.0+ fixes this, thus the example is no longer required.

.. rubric:: Other changes

- Reduced default width of the viewer's UI side panel to 200.0.
- Physics options (integrator, cone, jacobian, solver) in the viewer UI display a yellow warning when
  model parameters (opt/vis/stat) are out of sync with their initial values, indicating that parameters
  have been modified in the viewer.

3.0.1 (MuJoCo 3.6.0)
======================

.. rubric:: New features and improvements

- ``mujoco_rs::wrappers::mj_plugin``:
  :docs-rs:`~mujoco_rs::wrappers::mj_plugin::<fn>load_plugin_library` :sup:`new` and
  :docs-rs:`~mujoco_rs::wrappers::mj_plugin::<fn>load_all_plugin_libraries` :sup:`new`
  load a single or all MuJoCo plugin shared libraries.
  :docs-rs:`~mujoco_rs::wrappers::mj_plugin::<fn>load_all_plugin_libraries`
  accepts an optional
  :docs-rs:`~mujoco_rs::wrappers::mj_plugin::<type>MjPluginLibraryLoadCallback` :sup:`new`
  function pointer invoked for each loaded library.
- Added ``stl_mesh.rs`` :sup:`new` -- demonstrates loading plugin libraries
  and loading a model with an STL mesh asset.

3.0.0 (MuJoCo 3.6.0)
======================

.. rubric:: Breaking changes

*Methods now return Result instead of panicking*

- The following methods previously returned ``()`` and panicked on error.
  They now return ``Result`` directly:

  - |mj_data|: ``reset_keyframe``, ``set_state``, ``copy_visual_to``, ``copy_to``
  - |mjr_context|: ``read_pixels``
  - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`:
    ``push``, ``set_at``
  - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:
    ``set_font_scale`` now returns ``Result<(), RendererError>``;
    ``with_font_scale`` now returns ``Result<Self, RendererError>``.

  Callers must now handle the ``Result`` (e.g. append ``.unwrap()`` or ``?``).

*Default features changed*

- ``viewer``, ``viewer-ui``, ``renderer``, and ``renderer-winit-fallback`` are **no longer enabled
  by default**. Enable them explicitly when needed
  (e.g. ``cargo add mujoco-rs --features "viewer-ui renderer-winit-fallback"``).

*MjvScene is no longer generic*

- |mjv_scene| is no longer generic over ``M``. It is now a plain ``struct MjvScene``
  without a type parameter.

  - ``MjvScene::new`` now takes ``model: M`` where ``M: Deref<Target = MjModel>``
    (accepts ``Arc<MjModel>``, ``&MjModel``, etc.).
  - ``MjvScene::update``, ``MjvScene::update_with_catmask``, and
    ``MjvScene::find_selection`` now take a generic ``<M>`` parameter on the
    method rather than on the struct. They **panic** if ``data`` was created from
    a different model (signature mismatch).
  - ``MjvScene`` exposes a new ``signature() -> u64`` method for the model
    signature the scene was built for.
  - ``MjvPerturb::start`` / ``move_`` and ``MjvCamera::move_`` now take
    ``&MjvScene`` without a type parameter.
  - ``MjvPerturb::start`` and ``MjvPerturb::apply`` no longer take a
    ``model: &MjModel`` parameter (the model is now obtained internally via
    ``data.model()``).
  - ``vis_common::sync_geoms`` is now non-generic.
  - |mjv_scene| now derives ``Send + Sync`` unconditionally.

*MjRenderer is no longer generic*

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` and :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder`
  are no longer generic over ``M``.
  Remove the ``<M>`` type parameter from all usage sites.

  - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync_data` and
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`
    retain ``<M: Deref<Target = MjModel>>`` as **method-level** generics;
    call sites are unchanged.
  - :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` no longer requires
    ``M: Clone``; any ``M: Deref<Target = MjModel>`` is accepted.
  - ``sync`` is deprecated; use ``sync_data`` + ``render`` instead.

*MjViewer, MjViewerBuilder, ViewerSharedState are no longer generic*

- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`,
  :docs-rs:`~mujoco_rs::viewer::<struct>MjViewerBuilder`, and
  :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`
  are no longer generic over ``M``. Remove the ``<M>`` type parameter from all
  usage sites.

  - The viewer stores a passive ``Arc<MjModel>`` internally (analogous to the
    existing passive ``MjData`` copy). ``sync_data``, ``sync_data_full``, and
    ``build_passive`` retain ``<M: Deref<Target = MjModel>>`` as **method-level**
    generics, so the call sites are unchanged.
  - ``MjViewerBuilder::build_passive`` now accepts any ``M: Deref<Target = MjModel>``
    (e.g. ``&MjModel``, ``Arc<MjModel>``, ``Rc<MjModel>``).
  - The closure passed to ``MjViewer::add_ui_callback`` now receives
    ``&mut MjData<Arc<MjModel>>`` instead of ``&mut MjData<M>``. Update callback
    signatures accordingly.

*Viewer and renderer sync model automatically*

- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data` /
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_data`
  and
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync_data`
  now detect when ``data`` was created from a **different** model than the
  viewer/renderer was initialized with. Instead of panicking or returning an
  error, both automatically recreate the scene(s) for the new model and update
  their internal model reference.

*Separate renderer sync and render*

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`: new
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync_data` method
  updates the scene without rendering. Call
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>render` separately
  afterwards. This replaces the combined ``sync`` method, which is now deprecated.

*MuJoCo upgrade*

- Updated MuJoCo from **3.3.7** to **3.6.0**. C FFI bindings and all generated
  wrappers have been regenerated. See :ref:`installation` for the matching release.

*Error handling: io::Error replaced with typed enums*

All methods that previously returned ``io::Error`` or bare types now return typed
``Result`` variants. Six new error enums have been added to the
:docs-rs:`~mujoco_rs::error` module (all ``#[non_exhaustive]``, re-exported from
the prelude): ``MjDataError``, ``MjSceneError``, ``MjEditError``, ``MjModelError``,
``MjVfsError``, ``GlInitError``. Pre-existing ``RendererError`` and ``MjViewerError``
gained new variants. See `Error handling`_ below for the full method list.

*Type and signature changes*

- :docs-rs:`get_mujoco_version <mujoco_rs::<fn>mujoco_version>` has been renamed to
  :docs-rs:`~~mujoco_rs::<fn>mujoco_version`. The old name is deprecated.

- :docs-rs:`MjData::get_state <mujoco_rs::wrappers::mj_data::<struct>MjData::<method>state>`
  has been renamed to
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>state`.
  The old name is deprecated.

- :docs-rs:`MjvFigure::new <mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>new_boxed>`
  has been renamed to
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>new_boxed`.
  The old name is deprecated.

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_to` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_visual_to`
  now accept a destination of a **different** model type:
  ``destination: &mut MjData<N>`` where ``N: Deref<Target = MjModel>``. Previously
  the source and destination had to share the same ``M``.

- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name`
  now returns ``Result<(), MjEditError>`` instead of ``()``. Append ``?`` to call
  sites.
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>with_name`
  still returns ``&mut Self`` but now panics on duplicate names.

- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem` is now a sealed
  trait. External implementations are no longer permitted.

- ``MjsOrientation::switch_quat`` no longer has a type parameter. Replace
  ``switch_quat::<[f64; 4]>()`` with ``switch_quat()``.

- |mj_model| now implements the standard ``Clone`` trait. The old ``clone()``
  returning ``Option<MjModel>`` has been removed; use ``model.clone()``
  (panics on failure) or the new ``try_clone()`` for a fallible version.

- ``MjModel::save(filename, buffer)`` removed. Use the new ``save_to_file``
  and ``save_to_buffer`` methods.

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>size`
  returns ``usize`` (was ``i32``).

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_last_xml`
  now accepts ``AsRef<Path>`` (was ``&str``) and returns
  ``Err(MjModelError::InvalidUtf8Path)`` for non-UTF-8 paths (previously
  impossible to pass since ``&str`` is always UTF-8).

- ``MjViewerCpp<M>`` (``cpp-viewer`` feature) is no longer generic over ``M``.
  Remove the type parameter from all usage sites. The ``launch_passive``
  associated function retains ``<M: Deref<Target = MjModel>>`` as a
  method-level generic.

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size`
  returns ``usize`` (was ``i32``).

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>name_to_id`
  returns ``Option<usize>`` (was ``i32``; ``-1`` is now ``None``).

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>totalmass`
  renamed from ``get_totalmass``.

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_buffer`
  renamed from ``mjr_set_buffer``.

- |mj_data|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>reset_keyframe`
    now takes ``key: usize`` (was ``i32``) and returns ``Result<(), MjDataError>``.
    Out-of-range keys that previously silently fell back to a plain reset now return
    an error.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>runge_kutta`
    now takes ``n: u32`` (was ``i32``) and panics if ``n < 1``
    (previously passed negative or zero values silently to C).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_threadstack`
    returns ``&[MjtSize; mjMAXTHREAD]`` (was ``&[MjtSize]``).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body_com`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_subtree_com`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_geom`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_site`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>angmom_mat`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_velocity`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_acceleration`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>geom_distance`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>local_to_global`:
    object-id / body-id / geom-id / site-id parameters now take ``usize`` (was ``i32``).
    The ``>= 0`` guard is implicit in the type; pass ``body_id as usize`` at call sites.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray`:
    ``bodyexclude`` changed from ``i32`` to ``Option<usize>``. Replace ``-1`` with
    ``None`` and ``body_id`` with ``Some(body_id as usize)``.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` now returns
    ``(Option<usize>, MjtNum)`` instead of ``(i32, MjtNum)``. The returned geom id is
    ``None`` when the ray misses all geometry (previously ``-1``).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray` now
    returns ``(Vec<Option<usize>>, Vec<MjtNum>)`` instead of
    ``(Vec<i32>, Vec<MjtNum>)`` (use ``try_multi_ray`` for a ``Result``-returning
    alternative). Each element of the geom-id
    vector is ``None`` when the corresponding ray misses all geometry (previously
    ``-1``).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` gained a new
    ``normal_out: Option<&mut [MjtNum; 3]>`` parameter. Pass ``None`` to preserve
    previous behaviour.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray`
    gained ``normals_out`` and now panics on invalid input (use ``try_multi_ray``
    for a fallible alternative). Pass ``None`` for the new parameter.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_subtree_com`
    no longer accepts ``jacp: bool`` (the Jacobian is always computed); now panics
    on invalid input (use ``try_jac_subtree_com`` for a fallible alternative).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print_formatted`
    now accept ``AsRef<Path>`` and return ``Result``.

- |mj_model|:
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>id_to_name`:
  ``id`` takes ``usize`` (was ``i32``).

- :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom` gained
  ``normal_out: Option<&mut [MjtNum; 3]>``.

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`
  takes ``selection_xyz`` by ``&[MjtNum; 3]`` (was by value).

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>move_`
  no longer takes a separate ``model`` parameter; ``data`` changed from
  ``&mut MjData<M>`` to ``&MjData<M>``.

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>find_selection`
  returns :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>SceneSelection`
  (was a 5-tuple ``(i32, i32, i32, i32, [MjtNum; 3])``).
  ``SceneSelection`` fields (``body_id``, ``geom_id``, ``flex_id``, ``skin_id``) are
  ``Option<usize>`` (``None`` = no selection). Use ``if let Some(id) = sel.body_id``
  instead of ``if sel.body_id >= 0``.

- |mjv_camera|: ``new_fixed``, ``new_tracking``, ``track``, ``fix`` now take
  ``usize`` (was ``u32``). Remove ``as u32`` casts at call sites.

- ``TryFrom<i32> for MjtCamera`` now uses ``MjSceneError`` as its error type
  (was ``()``). Replace ``Err(())`` matches with ``Err(MjSceneError::InvalidCameraType(_))``.

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update`:
  the ``pertub`` parameter has been renamed to ``perturb`` (typo fix).

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>set_label`
  now returns ``Result<(), MjSceneError>``.

- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now returns
  ``Result<(), MjViewerError>``.

- |mj_model|: ``print`` / ``print_formatted`` now accept ``AsRef<Path>`` and return
  ``Result<(), MjModelError>``.

- |mj_model|: ``tuple_objtype`` accessor now returns ``&[MjtObj]`` instead of
  ``&[i32]``.

- ``MjCameraModelView`` / ``MjCameraModelViewMut``: camera field ``projection``
  (type ``MjtProjection``) replaces the old boolean ``orthographic`` field,
  matching MuJoCo's ``cam_projection`` rename.

- |mjs_tendon|: ``limited`` and ``actfrclimited`` are now ``MjtLimited``
  (tri-state: ``FALSE`` / ``TRUE`` / ``AUTO``) instead of ``bool``, matching the
  C ``mjtLimited`` semantics.

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsTexture::<method>set_data`
  now requires ``T: bytemuck::NoUninit``.

*New* ``unsafe`` *requirements*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>new`
  is now ``unsafe fn``. A valid OpenGL context must be current on the calling thread.

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_state`
  is now ``unsafe fn`` and returns ``Result<(), MjDataError>``. When ``spec`` includes
  ``mjSTATE_EQ_ACTIVE``, MuJoCo writes raw ``f64`` bytes into the ``eq_active``
  array without booleanization; calling ``eq_active()`` afterwards is UB until the
  state is re-validated (e.g. via ``mj_forward`` / ``mj_step``).

- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` is now
  ``unsafe fn``, must be called from the main thread, no longer accepts
  ``update_timer``, and returns ``Result<(), &'static str>``.

- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive`
  is now ``unsafe fn``. Model and data must remain alive and at a stable address for
  the lifetime of the viewer.

*Thread-safety bound tightening*

- ``MjData<M>``: ``Send`` / ``Sync`` now require ``M: Send`` /
  ``M: Sync``. Previously the bounds were unconditional, making
  ``MjData<Rc<MjModel>>`` incorrectly ``Send + Sync``.
  (``MjvScene`` is now non-generic and unconditionally ``Send + Sync``.)

- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive` now requires
  ``M: Send + Sync`` (previously only ``Deref<Target = MjModel> + Clone``).

*Mutable accessor restrictions*

``*_mut()`` methods on |mj_model|, |mj_data|, and |mjv_scene| array fields are now
``unsafe fn`` where unrestricted writes can corrupt MuJoCo's internal state. Two
categories of fields are affected:

- **Structural invariants** --- topology, address, and engine-computed arrays that
  must not be changed at runtime: **199** fields on |mj_model|, **43** on |mj_data|,
  **12** on |mjv_scene|.

- **Companion-index fields** --- type/mode fields whose values control which array a
  companion index (``*id``, ``*adr``) indexes into; writing inconsistent values
  causes out-of-bounds access: **17** fields on |mj_model| (``jnt_type``,
  ``actuator_trntype``, ``actuator_dyntype``, ``eq_type``, ``eq_objtype``,
  ``wrap_type``, ``wrap_prm``, ``sensor_type``, ``sensor_objtype``,
  ``sensor_reftype``, ``skin_matid``, ``tendon_matid``, ``tendon_treeid``,
  ``body_plugin``, ``actuator_plugin``, ``geom_plugin``, ``sensor_plugin``),
  **4** on |mj_data| (``efc_type``, ``iefc_type``, ``tree_asleep``, ``wrap_obj``).

- **Null-terminated string buffers** --- concatenated ``c_char`` arrays where each
  entry is null-terminated; removing a ``'\\0'`` byte allows MuJoCo's C string
  functions (and ``CStr::from_ptr``) to read past the buffer boundary: **4** fields
  on |mj_model| (``names``, ``plugin_attr``, ``text_data``, ``paths``).

``info_with_view!`` generated ``ViewMut`` types expose the companion-index fields as
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`; mutation requires
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut::<method>as_mut_slice`
inside an ``unsafe`` block. See :ref:`migration <migrate_3_0_0>` for full field
lists and before/after examples.

*Removed APIs*

- ``MjViewerCpp::__raw()`` removed (exposed internal C++ binding pointers).
- :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`: ``figure``
  renamed to ``draw``.
- ``MjViewer::sync`` removed (deprecated in 2.2.0; use ``sync_data`` + ``render``).
- ``MjViewer::user_scn`` / ``user_scn_mut`` removed (deprecated in 1.3.0).
- ``MjViewer::user_scene`` / ``user_scene_mut`` removed (deprecated in 2.2.0;
  access via ``ViewerSharedState::user_scene`` / ``user_scene_mut`` through
  ``viewer.state()``).
- Removed deprecated methods:

  .. list-table::
     :header-rows: 1
     :widths: 30 25 45

     * - Type
       - Removed
       - Replacement
     * - |mj_data|
       - ``warning_stats``
       - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>warning`
     * - |mj_data|
       - ``timer_stats``
       - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>timer`
     * - |mj_data|
       - Type aliases ``MjJointInfo``, ``MjJointView``, ``MjJointViewMut``,
         ``MjGeomInfo``, ``MjGeomView``, ``MjGeomViewMut``,
         ``MjActuatorInfo``, ``MjActuatorView``, ``MjActuatorViewMut``
       - Use the current ``MjJointData*``, ``MjGeomData*``, ``MjActuatorData*`` types.
     * - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjJointDataViewMut`
       - ``reset``
       - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataViewMut::<method>zero`
     * - |mj_model|
       - ``name2id``
       - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>name_to_id`
     * - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`
       - ``new``
       - ``new_free``, ``new_fixed``, ``new_tracking``, or ``new_user``

*MjTendonDataInfo field removal*

- ``MjTendonDataInfo`` no longer exposes ``J_rownnz``, ``J_rowadr``, or
  ``J_colind``; these fields moved upstream to ``mjModel`` in MuJoCo 3.6.0
  and are now accessible via ``MjTendonModelInfo`` (i.e. ``model.tendon()``).

.. _Error handling:

.. rubric:: Error handling

New error types in :docs-rs:`~mujoco_rs::error`
(all ``#[non_exhaustive]``, re-exported from the prelude):

.. list-table:: Methods returning ``Result`` types
   :header-rows: 1
   :widths: 20 55 25

   * - Type
     - Methods
     - Error type
   * - |mj_model|
     - ``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``from_buffer``,
       ``save_last_xml``,
       ``save_to_file`` :sup:`new`,
       ``save_to_buffer`` :sup:`new`,
       ``print``, ``print_formatted``,
       ``try_extract_state`` :sup:`new`,
       ``try_extract_state_into`` :sup:`new`
     - :docs-rs:`~mujoco_rs::error::<enum>MjModelError`
   * - |mj_data|
     - ``add_contact``, ``constraint_update``,
       ``try_jac`` :sup:`new`, ``try_jac_body`` :sup:`new`, ``try_jac_body_com`` :sup:`new`, ``try_jac_subtree_com`` :sup:`new`,
       ``try_jac_geom`` :sup:`new`, ``try_jac_site`` :sup:`new`, ``try_angmom_mat`` :sup:`new`,
       ``try_object_velocity`` :sup:`new`, ``try_object_acceleration`` :sup:`new`, ``try_geom_distance`` :sup:`new`,
       ``try_local_to_global`` :sup:`new`, ``try_multi_ray`` :sup:`new`, ``print``, ``print_formatted``,
       ``init_ctrl_history`` :sup:`new`,
       ``init_sensor_history`` :sup:`new`,
       ``try_read_ctrl`` :sup:`new`,
       ``read_sensor_into`` :sup:`new`,
       ``try_read_sensor_fixed`` :sup:`new`,
       ``try_read_sensor`` :sup:`new`,
       ``try_swap_model`` :sup:`new`,
       ``reset_keyframe``,
       ``copy_state_from_data`` :sup:`new`,
       ``apply_ft`` :sup:`new`,
       ``set_state``,
       ``copy_visual_to``,
       ``copy_to``
     - :docs-rs:`~mujoco_rs::error::<enum>MjDataError`
   * - |mj_vfs|
     - ``add_file`` :sup:`new`, ``add_file_from`` :sup:`new`,
       ``add_from_file`` :sup:`deprecated`, ``add_from_buffer``, ``delete_file``
     - :docs-rs:`~mujoco_rs::error::<enum>MjVfsError`
   * - |mj_spec|
     - ``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``compile``,
       ``save_xml``, ``save_xml_string``, ``try_add_default`` :sup:`new`,
       ``from_parse`` :sup:`new`,
       ``from_parse_vfs`` :sup:`new`
     - :docs-rs:`~mujoco_rs::error::<enum>MjEditError`
   * - |mjv_scene| / :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom` / :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure` / |mjr_context|
     - ``try_create_geom`` :sup:`new`, ``set_label``, ``add_aux``, ``set_aux``,
       ``push``, ``set_at``, ``read_pixels``
     - :docs-rs:`~mujoco_rs::error::<enum>MjSceneError`
   * - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`
     - ``try_rgb`` :sup:`new`, ``try_depth`` :sup:`new`, ``save_rgb``, ``save_depth``, ``save_depth_raw``
     - :docs-rs:`~mujoco_rs::renderer::<enum>RendererError`
   * - :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`
     - ``render``
     - :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`

Methods marked :sup:`new` are new in 3.0.0; the rest existed in 2.x and previously
returned bare types or ``io::Error``.

New error variants in pre-existing enums:

- :docs-rs:`~mujoco_rs::renderer::<enum>RendererError`: ``RgbDisabled``,
  ``DepthDisabled``, ``DimensionMismatch``, ``ZeroDimension``, ``IoError``,
  ``SceneError``, ``GlInitFailed``.
- :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`: ``PainterInitError``,
  ``GlInitFailed``, ``SceneError``.

The six new enums (all ``#[non_exhaustive]``) have the following variants:

- :docs-rs:`~mujoco_rs::error::<enum>MjModelError`: ``InvalidUtf8Path``,
  ``LoadFailed``, ``SaveFailed``, ``AllocationFailed``,
  ``StateSliceLengthMismatch``, ``SpecNotSubset``, ``BufferTooSmall``,
  ``SignatureMismatch``, ``VfsError``.
- :docs-rs:`~mujoco_rs::error::<enum>MjDataError`: ``IndexOutOfBounds``,
  ``UnsupportedObjectType``, ``AllocationFailed``, ``BufferTooSmall``,
  ``LengthMismatch``, ``SignatureMismatch``, ``NoHistoryBuffer``,
  ``ContactBufferFull``, ``InvalidUtf8Path``.
- :docs-rs:`~mujoco_rs::error::<enum>MjVfsError`: ``AlreadyExists``,
  ``LoadFailed``, ``NotFound``, ``InvalidUtf8Path``, ``BufferTooLarge``,
  ``Unknown``.
- :docs-rs:`~mujoco_rs::error::<enum>MjEditError`: ``AllocationFailed``,
  ``InvalidUtf8Path``, ``ParseFailed``, ``CompileFailed``, ``SaveFailed``,
  ``NotFound``, ``AlreadyExists``, ``UnsupportedOperation``, ``DeleteFailed``,
  ``XmlBufferTooSmall`` --- returned by ``save_xml_string`` when the supplied
  buffer is too small; the ``required_size`` field carries the
  ``snprintf``-style byte count (excluding NUL), so retry with
  ``required_size + 1``.
- :docs-rs:`~mujoco_rs::error::<enum>MjSceneError`: ``SceneFull``,
  ``LabelTooLong``, ``InvalidAuxBufferIndex``, ``InvalidViewport``,
  ``BufferTooSmall``, ``FigureBufferFull``, ``FigureIndexOutOfBounds``,
  ``InvalidPlotIndex``, ``NonAsciiLabel``, ``InvalidCameraType``.
- :docs-rs:`~mujoco_rs::error::<enum>GlInitError` (requires ``viewer`` or
  ``renderer-winit-fallback`` feature): ``DisplayBuild``, ``NoWindow``,
  ``WindowHandle``, ``ContextCreation``, ``SurfaceAttributes``,
  ``SurfaceCreation``, ``MakeCurrent``.
- New ``try_`` methods paired with existing infallible counterparts (infallible
  variants now delegate to ``try_``.``expect()``):

  - |mj_data|: ``try_new``, ``try_clone``, ``try_read_state_into``,
    ``try_swap_model``, ``try_read_ctrl``, ``try_read_sensor_fixed``,
    ``try_read_sensor``,
    ``try_jac``, ``try_jac_body``, ``try_jac_body_com``,
    ``try_jac_subtree_com``, ``try_jac_geom``, ``try_jac_site``,
    ``try_angmom_mat``, ``try_object_velocity``, ``try_object_acceleration``,
    ``try_geom_distance``, ``try_local_to_global``, ``try_multi_ray``
  - |mj_model|: ``try_clone``, ``try_make_data``,
    ``try_extract_state``, ``try_extract_state_into``
  - |mj_spec| / |mjs_tendon| / ``MjsBody``: ``try_new``, ``try_clone``, ``try_add_frame``,
    ``try_wrap_site``, ``try_wrap_geom``, ``try_wrap_joint``, ``try_wrap_pulley``,
    ``try_add_default``, macro-generated ``try_add_*``
  - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:
    ``try_rgb``, ``try_depth``
  - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`:
    ``try_create_geom``
  - |mj_data|: ``try_ray_flex`` :sup:`new`, ``try_ray_hfield`` :sup:`new`,
    ``try_ray_mesh`` :sup:`new`
  - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`:
    ``try_full`` :sup:`new`, ``try_empty`` :sup:`new`,
    ``try_pop_front`` :sup:`new`, ``try_pop_back`` :sup:`new`,
    ``cut_front`` :sup:`new`, ``cut_end`` :sup:`new`

- New |mj_data| methods that return ``Result`` directly (no separate ``try_`` variant):
  ``copy_state_from_data``, ``apply_ft``.

.. rubric:: New features and improvements

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`: PNG compression is now
  configurable via
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRendererBuilder::<method>png_compression`
  (builder) and
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>set_png_compression`
  (runtime setter). Defaults to ``png::Compression::NoCompression``. Affects
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb` and
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth`.
  The ``png`` crate is re-exported as ``mujoco_rs::renderer::png`` for naming
  ``png::Compression``.

- |mj_model|: ``extract_state`` / ``extract_state_into``, ``try_make_data``.
- |mj_data|: ``swap_model`` :sup:`new` (validates model signature),
  ``model_mut`` :sup:`new` (``-> &mut MjModel``, available when ``M: DerefMut``),
  ``forward_kinematics``, ``apply_ft``, ``copy_state_from_data``,
  ``ray_flex`` / ``ray_mesh`` / ``ray_hfield``,
  ``init_ctrl_history`` / ``init_sensor_history``,
  ``read_ctrl`` / ``read_sensor`` / ``read_sensor_into`` / ``read_sensor_fixed``,
  ``model_clone`` :sup:`new` (``-> M``, available when ``M: Clone``).
- |mj_spec|: ``from_parse`` / ``from_parse_vfs``.
  ``from_parse``, ``from_parse_vfs``, and ``save_xml`` now accept
  ``AsRef<Path>`` (was ``&str``); returns ``Err(MjEditError::InvalidUtf8Path)``
  for non-UTF-8 paths.
  ``MjSpec`` and ``SpecItem`` are also re-exported from
  :docs-rs:`mujoco_rs::wrappers`.
- |mjs_tendon|: ``wrap`` / ``wrap_mut`` / ``wrap_num``.
- |mjs_wrap|: ``coef``, ``divisor``, ``side_site``, ``side_site_mut``.
- |mj_vfs|: ``add_file`` / ``add_file_from`` replace ``add_from_file``
  (which is now deprecated). ``add_from_buffer`` and ``delete_file`` now accept
  ``AsRef<Path>``; returns ``Err(MjVfsError::InvalidUtf8Path)`` for non-UTF-8
  paths.
- |mjv_scene|: ``update_with_catmask`` (exposes ``catmask`` filter parameter).
- |mjv_camera|: ``frame``, ``frustum``.
- New :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>SceneSelection`
  struct (returned by ``find_selection``); implements ``Default``.
- The :docs-rs:`~mujoco_rs::vis_common` module is now public (requires ``viewer``
  or ``renderer`` feature), exposing ``sync_geoms``, ``write_png``,
  ``flip_image_vertically``.
- |mj_model| and |mj_data| views expose additional fields from MuJoCo 3.6.0.
  New |mj_model| view types added for ``exclude``, ``mesh``, and ``skin``.
  ``dof_bodyid`` and ``dof_treeid`` are now exposed in per-dof joint model view
  types (the fields existed in MuJoCo 3.3.7 but were not previously accessible
  via per-object view types).
- ``MjsMaterial``:
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_texture` /
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>with_texture`
  set a texture name by
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtTextureRole`, correctly targeting the
  pre-sized slot the MuJoCo renderer reads (e.g. ``mjTEXROLE_RGB``).
  The existing ``set_textures`` / ``append_textures`` methods are retained but
  should generally be avoided for role-indexed vectors.
- ``MjsTexture``:
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefile` /
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>with_cubefile`
  set a cube-map face file by
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<enum>MjtCubeFace`.
  The existing ``set_cubefiles`` / ``append_cubefiles`` methods are retained.
- ``info_with_view!`` structs now have ``try_view`` / ``try_view_mut`` methods.
- Trait additions: ``Clone`` for |mj_spec|; ``Default`` for |mj_vfs|, |mj_spec|,
  ``MjOption``, ``MjRendererBuilder``, ``MjViewerBuilder``; ``Send`` + ``Sync``
  for |mj_vfs|; ``FusedIterator`` for all model-editing iterators; ``Eq`` for
  ``PointerView`` / ``PointerViewMut``; ``Drop`` for ``MjRenderer``.
- ``PointerView::new``, ``PointerViewMut::new``, ``MjrRectangle::new`` are now
  ``const fn``.
- |mj_model| and |mj_data| now use ``NonNull<mjModel>`` / ``NonNull<mjData>``
  internally, encoding the non-null invariant at the type level and enabling
  niche optimisation for ``Option<MjModel>`` / ``Option<MjData>``.
- |mj_data|: ``contacts()`` is deprecated; use ``contact()`` instead.

.. rubric:: MjViewer

- Added support for the ``Depth`` rendering flag (``mjRND_DEPTH``).
- Added a "Print camera" button that prints the camera as an MJCF element.
- Added a "Screenshot" panel with: a **Screenshot** button (timestamped PNG),
  a **Viewport only** checkbox (captures before UI is drawn), and a **Depth**
  checkbox (16-bit grayscale depth PNG).

.. rubric:: Bug fixes

- :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera` ``new_fixed`` / ``new_tracking``
  and :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene` ``new``:
  ``debug_assert!`` guards on user-supplied ``camera_id``, ``tracking_id``, and ``max_geom``
  (checking they fit in ``i32``) were silently skipped in release builds. Changed to ``assert!``
  so the precondition is always enforced. The ``# Panics`` documentation now correctly states
  panics occur in all builds, not only debug builds.

- |mj_model|:
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_last_xml`:
  a ``_ => unreachable!()`` arm would panic if MuJoCo ever returned a value other
  than ``0`` or ``1``; changed to ``_ => Err(MjModelError::SaveFailed(...))`` so any
  unexpected return code is surfaced as an error instead of aborting the process.
- :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_is_bad`: now tests
  ``!= 0`` instead of ``== 1``.
- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:

  - ``save_depth``: guards against division by zero when the depth range is zero.
  - ``rgb``, ``rgb_flat``, ``save_rgb``, ``save_depth``, ``save_depth_raw``: images
    were output vertically flipped (OpenGL reads bottom-up); now corrected.
  - ``MjRendererBuilder``: ``font_scale`` setting was silently ignored during
    ``build()``; now applied.

- :docs-rs:`~mujoco_rs::util::<struct>PointerView` and
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewMut` ``PartialEq`` now compares
  both pointer and length (previously pointer only).
- |mj_model|: ``key_mpos`` and ``key_mquat`` array accessors were missing; added.
- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`: mouse normalization now uses
  ``inner_size()`` instead of ``outer_size()``, fixing cursor mapping with window
  decorations.
- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` now implements ``Drop`` to make
  the GL context current before cleanup (prevents resource leaks on some platforms).
- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync`:

  - FFI declaration used ``bool`` for ``state_only`` (should be ``c_int``); fixed.
  - Could be called after the viewer window was closed, causing UB; now returns
    early if not running.

- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive`:
  asserts the C++ simulate handle is non-null after allocation.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`:
  now uses bounds-checked slice indexing instead of raw pointer arithmetic.
- |mjr_context|: ``set_aux`` now has a ``mjNAUX``-based index bounds check.
- Fixed potential UB when parsing null string pointers from MuJoCo C++ wrappers.
- Added strict runtime signature check to view creation (panics with
  ``model signature mismatch`` on mismatch).
- :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`: on model change,
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_data` /
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>sync_data_full`
  no longer spuriously write back the new
  model's default pose (``qpos0``) to the incoming ``data``. Previously, models
  with free joints, ball joints, or joints with a non-zero ``ref`` attribute would
  have their simulation state silently reset to the default pose on the first sync
  after a model switch. Active perturbations are also cleared on model change to
  prevent stale body-index references from the previous model being applied.

.. rubric:: Other changes

- Fixed security issue: ``model_signature`` field on all ``*Info`` structs
  (e.g. ``MjJointDataInfo``) was public, allowing the signature to be
  overwritten to bypass the bounds guard in ``try_view``/``try_view_mut``,
  which could cause out-of-bounds raw-pointer arithmetic (UB). The field is
  now private; use the ``model_signature()`` getter instead.
- Fixed :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` ``Display`` impl:
  the four wrapper variants (``GlutinError``, ``GlInitFailed``, ``IoError``,
  ``SceneError``) were discarding the inner error message, making error
  output non-actionable (e.g. ``"scene error"`` instead of the actual cause).
  They now include the inner error: e.g. ``"scene error: …"``.
  ``EventLoopError`` in both ``RendererError`` and ``MjViewerError`` had the
  same issue and was also fixed.
- Updated enum type aliases to match MuJoCo 3.6.0 definitions.
- Added examples: :gh-example:`tippe_top.rs`, :gh-example:`chaotic_pendulum.rs`,
  :gh-example:`contact_forces.rs`, :gh-example:`multi_legged_creatures.rs`,
  :gh-example:`procedural_tree.rs`, :gh-example:`miri_test.rs`,
  :gh-example:`model_switch.rs`, :gh-example:`model_parameters.rs`.

2.3.5 (MuJoCo 3.3.7)
======================

- Fixed `#161 <https://github.com/davidhozic/mujoco-rs/issues/161>`_: actuator views
  could crash when a non-muscle actuator followed a muscle actuator.
- Internal: minor performance improvement in ``sync_geoms`` (early exit when no
  geoms are added).

2.3.4 (MuJoCo 3.3.7)
=============================
- :ref:`mj_rust_viewer` bug fixes and usability improvements:

  - Keyboard events are now ignored whenever an egui input widget has focus.
  - Fixed a logic error where pressing an unhandled mouse button would drop
    subsequent events.
  - Ctrl+C no longer toggles camera visualization (reserved for copy).

2.3.3 (MuJoCo 3.3.7)
=============================
- Fixed `#144 <https://github.com/davidhozic/mujoco-rs/issues/144>`_: the viewer
  caused high stack usage, potentially problematic on Windows (1M stack limit).

2.3.2 (MuJoCo 3.3.7)
=============================
- Small performance improvement by removing mutex contention during scene render.

2.3.1 (MuJoCo 3.3.7)
=============================
- Bug fixes:

  - Added a NULL pointer check to |mj_data|; allocation failure now panics instead
    of accepting a NULL pointer.
  - Fixed `#139 <https://github.com/davidhozic/mujoco-rs/issues/139>`_:
    ``MjrContext::read_pixels`` no longer allows buffers smaller than the viewport.
  - Fixed potential viewer crashes when the display is disconnected or reconfigured.

2.3.0 (MuJoCo 3.3.7)
=============================
- :ref:`mj_rust_viewer`:

  - Added ``add_ui_callback_detached`` for passive UI widgets.
  - Added ``with_ui_egui_ctx`` for scoped egui context access.
  - Replaced panicking mutex handling with automatic unpoison logic.
  - Performance optimizations.

- |mj_model| and |mj_spec|: version check against the MuJoCo shared library on
  instantiation; panics on mismatch.

2.2.2 (MuJoCo 3.3.7)
================================
- Fixed `incorrect depth value calculation <https://github.com/davidhozic/mujoco-rs/pull/125>`_.

2.2.1 (MuJoCo 3.3.7)
================================
- Fixed `#119 <https://github.com/davidhozic/mujoco-rs/issues/119>`_: borrow tracker tracked the wrong lifetime.
- Reset more OpenGL state after egui draw (figures now render correctly).

2.2.0 (MuJoCo 3.3.7)
================================
- :ref:`mj_renderer` now uses EGL on Linux by default (true offscreen rendering).
  New feature flag ``renderer-winit-fallback`` for compatibility (enabled by default).
- :ref:`mj_rust_viewer`:

  - ``add_ui_callback`` for custom egui UI widgets.
  - Info menu (F2): smoothed FPS, simulation time, used memory.
  - Realtime warning (F4): shows realtime factor when sync time deviates ≥ 2%.
  - Separate sync and render: ``sync_data`` (deprecates ``sync``) + ``render``.
  - Added :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState` for
    multi-threaded viewer/simulation sync.

- New |mj_data| methods: ``copy_visual_to``, ``copy_to``, ``set_state``,
  ``get_state``, ``read_state_into``.
- New example: ``rust_viewer_threaded``.
- Deprecated: ``user_scene()`` / ``user_scene_mut()``
  (replaced by ``ViewerSharedState::user_scene`` / ``user_scene_mut``).

2.1.0 / 2.1.1 (MuJoCo 3.3.7)
================================
- Option to automatically download MuJoCo (``auto-download-mujoco`` feature).
- pkg-config support for Linux and macOS.
- Optional egui UI for :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`
  (enabled by default).

2.0.1 (MuJoCo 3.3.7)
================================
- Fixed the ``renderer`` feature not enabling all needed crates.

2.0.0 (MuJoCo 3.3.7)
================================
- **Breaking changes**:

  - Updated MuJoCo to 3.3.7.
  - :ref:`model_editing`: items no longer wrapped (just aliased types); attributes
    made private; immutable iterators added; ``<item>_mut()`` methods added.
  - Viewer / renderer: switched from GLFW to Winit + Glutin.
  - :ref:`mj_cpp_viewer`: moved to ``mujoco_rs::cpp_viewer``; build target changed
    to ``glfw libmujoco_simulate``.
  - |mj_data| and related types now generic over model handle
    (``Deref<Target = MjModel>``).
  - Various ``c_int`` → ``i32`` / ``bool`` type replacements.
  - Removed ``mujoco_rs::wrappers::mj_interface``.

- Other: additional getters / setters / array slice methods for |mj_data|,
  |mj_model|, |mjv_scene|.

1.5.0 (MuJoCo 3.3.5)
================================
- |mjv_scene|: ``pop_geom`` method added.
- :ref:`model_editing`: iterators added to |mj_spec| and ``MjsBody``.

1.4.2 (MuJoCo 3.3.5)
================================
- Fixed segfault when model specification is invalid
  (`#65 <https://github.com/davidhozic/mujoco-rs/issues/65>`_).

1.4.1 (MuJoCo 3.3.5)
================================
- Added missing |mj_spec| named accessors: ``geom``, ``site``, ``camera``, ``light``.

1.4.0 (MuJoCo 3.3.5)
================================
- |mj_model|: added views for ``key``, ``tuple``, ``texture``, ``site``, ``pair``,
  ``numeric``, ``material``, ``light``, ``hfield``, ``equality``.
- :ref:`model_editing` support added (``mujoco_rs::wrappers::mj_editing`` module).
- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` / ``MjRendererBuilder``
  improvements.
- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`: new key bindings and visualization toggles;
  visual-geom headroom increased from 100 to 2000.
- :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>fix` added.

1.3.0 (MuJoCo 3.3.5)
================================
- Added :docs-rs:`mujoco_rs::renderer` module with ``MjRenderer``.
- Deprecated ``MjvCamera::new()``; replaced by ``new_free``, ``new_fixed``,
  ``new_tracking``, ``new_user``.
- Added ``MjData::maxuse_stack``, ``maxuse_threadstack``, ``warning_stats``,
  ``timer_stats``, ``time``, ``energy``.
- Added ``MjModel::signature``, ``opt`` / ``opt_mut``, ``vis`` / ``vis_mut``,
  ``stat`` / ``stat_mut``.
- Added joint view attributes: ``qfrc_spring``, ``qfrc_damper``,
  ``qfrc_gravcomp``, ``qfrc_fluid``.

1.2.0 (MuJoCo 3.3.5)
================================
- Function wrappers for utility and derivative functions
  (:docs-rs:`mujoco_rs::wrappers::fun`).
- Completed virtual file system wrapper: ``add_from_file``, ``delete_file``,
  ``from_xml_vfs``.

1.1.0 (MuJoCo 3.3.5)
=====================
- Fixed data-race bug `#18 <https://github.com/davidhozic/mujoco-rs/issues/18>`_.
- Fixed |mj_geomview| / |mj_geomviewmut| pointing to wrong address
  (`#17 <https://github.com/davidhozic/mujoco-rs/issues/17>`_).
- Fixed spurious scene-buffer warning on some models
  (`#19 <https://github.com/davidhozic/mujoco-rs/issues/19>`_).
- Added :docs-rs:`mujoco_rs::wrappers::mj_primitive` module.
- More |mj_data| views: actuator, body, camera, geom, joint, light, sensor, site, tendon.
- More |mj_model| views: actuator, body, camera, geom, joint, sensor, tendon.

1.0.1 (MuJoCo 3.3.5)
=====================
- Bug fixes in ``Drop`` implementations.

1.0.0 (MuJoCo 3.3.5)
=====================
- **Breaking**: all ``ffi_mut()`` methods now require ``unsafe``.
- Viewer: help overlay (F1), user scene, mouse perturbation.

0.4.3 (MuJoCo 3.3.5)
=====================
- Removed unnecessary header files (smaller crate size).

0.4.2 (MuJoCo 3.3.5)
=====================
- Renamed env vars: ``MUJOCO_DYNAMIC_LINK_LIB`` → ``MUJOCO_DYNAMIC_LINK_DIR``,
  ``MUJOCO_STATIC_LINK_LIB`` → ``MUJOCO_STATIC_LINK_DIR``.

0.4.1 (MuJoCo 3.3.5)
=====================
- Fixed event handling.

0.4.0 (MuJoCo 3.3.5)
=====================
- Renamed package to ``mujoco-rs``.

0.3.0 (MuJoCo 3.3.5)
=====================
- Initial public release.
