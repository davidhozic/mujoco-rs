.. _migration:

=======================
Migration guide
=======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mjv_camera| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`
.. |mjr_context| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`


This page documents the migration steps for upgrading between major versions of MuJoCo-rs.
For a full list of changes, see the :ref:`changelog`.


.. _migrate_3_0_0:

Migrating to 3.0.0
======================

Version 3.0.0 updates MuJoCo to 3.6.0, introduces typed error enums,
tightens safety requirements, and removes deprecated APIs.


MuJoCo upgrade
-----------------------
MuJoCo-rs 3.0.0 links against MuJoCo **3.6.0**. Download the matching release
and update your library path. See :ref:`installation` for details.


Error handling
-----------------------

The most significant change in 3.0.0 is replacing ``io::Error`` and bare return
types with typed error enums.

New error types
~~~~~~~~~~~~~~~~~~~~~~~~

Six new types in :docs-rs:`~mujoco_rs::error` (all ``#[non_exhaustive]``,
re-exported from the prelude):

- :docs-rs:`~mujoco_rs::error::<enum>MjDataError` --- |mj_data| operations.
- :docs-rs:`~mujoco_rs::error::<enum>MjSceneError` --- |mjv_scene| and rendering.
- :docs-rs:`~mujoco_rs::error::<enum>MjEditError` --- |mj_spec| and model editing.
- :docs-rs:`~mujoco_rs::error::<enum>MjModelError` --- |mj_model| load/save/state.
- :docs-rs:`~mujoco_rs::error::<enum>MjVfsError` --- virtual file system.
- :docs-rs:`~mujoco_rs::error::<enum>GlInitError` --- OpenGL context init
  (requires ``viewer`` or ``renderer-winit-fallback`` feature).

:docs-rs:`~mujoco_rs::renderer::<enum>RendererError` and
:docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError` gained new variants.

Updating error handling code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Replace ``io::ErrorKind`` matches with the specific error enum variants.

**Before (2.x):**

.. code-block:: rust

    use std::io;

    match model.save_last_xml("out.xml") {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::InvalidInput => {
            eprintln!("Invalid path: {e}");
        }
        Err(e) => eprintln!("Save failed: {e}"),
    }

**After (3.0.0):**

.. code-block:: rust

    use mujoco_rs::prelude::*;

    match model.save_last_xml("out.xml") {
        Ok(()) => {}
        Err(MjModelError::SaveFailed(msg)) => eprintln!("Save failed: {msg}"),
        Err(e) => eprintln!("Other error: {e}"),
    }

The table below maps the breaking error-type changes:

.. list-table::
   :header-rows: 1

   * - Type / methods
     - 2.x error type
     - 3.0.0 error type
   * - |mj_model|: ``from_xml``, ``from_xml_vfs``, ``from_xml_string``,
       ``from_buffer``, ``save_last_xml``, ``save_to_file``, ``save_to_buffer``,
       ``print``, ``print_formatted``
     - ``io::Error`` / ``NulError``
     - ``MjModelError``
   * - |mj_vfs|: ``add_from_file``, ``add_from_buffer``, ``delete_file``
     - ``io::Error``
     - ``MjVfsError``
   * - |mj_spec|: ``from_xml``, ``from_xml_vfs``, ``from_xml_string``,
       ``compile``, ``save_xml``, ``save_xml_string``, ``add_default``
     - ``io::Error``
     - ``MjEditError``
   * - |mj_data|: ``add_contact``, Jacobian methods, ``multi_ray``, ``print``,
       ``print_formatted``
     - ``io::Error`` / bare types
     - ``MjDataError``
   * - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:
       ``rgb``, ``depth``, ``save_rgb``, ``save_depth``, ``save_depth_raw``
     - ``io::Error``
     - ``RendererError``
   * - :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`: ``render``
     - ``()``
     - ``Result<(), MjViewerError>``

Newly fallible methods
~~~~~~~~~~~~~~~~~~~~~~~~~~

Several methods that previously returned bare types now return ``Result``.
Append ``?`` or ``.unwrap()`` to call sites.

|mj_data| methods now returning ``Result<_, MjDataError>``:
  ``constraint_update``, ``jac``, ``jac_body``, ``jac_body_com``,
  ``jac_subtree_com``, ``jac_geom``, ``jac_site``, ``angmom_mat``,
  ``object_velocity``, ``object_acceleration``, ``geom_distance``,
  ``local_to_global``, ``multi_ray``.

|mjv_scene| / :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom` / |mjr_context| methods now returning ``Result<_, MjSceneError>``:
  ``create_geom``, ``set_label``, ``add_aux``, ``set_aux``.


Model-editing API changes
----------------------------

:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name`
now returns ``Result<(), MjEditError>`` instead of ``()``.
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>with_name`
still returns ``&mut Self`` but now panics on duplicate names.

``MjsOrientation::switch_quat`` no longer has a generic type parameter. Remove
turbofish syntax and call it directly.

**Before (2.x):**

.. code-block:: rust

  item.set_name("arm_joint");
  item.with_name("arm_joint");
  orientation.switch_quat::<[f64; 4]>();

**After (3.0.0):**

.. code-block:: rust

  item.set_name("arm_joint")?;
  item.with_name("arm_joint");
  orientation.switch_quat();


``MjModel::clone()``
-----------------------

|mj_model| now implements the standard ``Clone`` trait. The previous ``clone()``
returning ``Option<MjModel>`` has been removed.

**Before (2.x):**

.. code-block:: rust

    let model_copy = model.clone().expect("clone failed");

**After (3.0.0):**

.. code-block:: rust

    let model_copy = model.clone();         // panics on failure
    let model_copy = model.try_clone()?;    // fallible


``MjModel::save()`` split
-----------------------------

``MjModel::save(filename: Option<&str>, buffer: Option<&mut [u8]>)`` has been
replaced by two dedicated methods:

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_to_file`
  --- saves to a binary MJB file.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_to_buffer`
  --- saves to a memory buffer. Returns ``MjModelError::BufferTooSmall`` if the
  buffer is too small.

**Before (2.x):**

.. code-block:: rust

    model.save(Some("model.mjb"), None);

    let mut buffer = vec![0u8; model.size() as usize];
    model.save(None, Some(&mut buffer));

**After (3.0.0):**

.. code-block:: rust

    model.save_to_file("model.mjb")?;

    let mut buffer = vec![0u8; model.size() as usize];
    model.save_to_buffer(&mut buffer)?;


Mutable array access restrictions
-----------------------------------------------

Mutable flat-slice accessors (``*_mut`` methods) have been removed for fields that
are managed exclusively by the engine or represent load-time structural invariants.
If your code called ``*_mut()`` on any of these fields, remove those calls. For raw
write access, use ``ffi_mut()`` directly --- but only if you understand the invariants
involved.

On |mj_model|, **all 232 structural array fields** that previously exposed a
``*_mut`` accessor no longer do so. This includes every address array (``*adr``,
``*num``), topology index (``*bodyid``, ``*jntid``, …), physics-invariant
(``body_simple``, ``dof_simplenum``, …), and name/path pointer. These are set once
at load time by ``mj_setConst`` and must not be changed at runtime.

On |mj_data|, **44 engine-computed structural arrays** became read-only:

- *Constraint structure* --- ``efc_type``, ``efc_id``, ``efc_J_rownnz``,
  ``efc_J_rowadr``, ``efc_J_rowsuper``, ``efc_J_colind``,
  ``efc_AR_rownnz``, ``efc_AR_rowadr``, ``efc_AR_colind``
- *Island / constraint mapping* --- ``efc_island``, ``iefc_type``, ``iefc_id``,
  ``iefc_J_rownnz``, ``iefc_J_rowadr``, ``iefc_J_rowsuper``, ``iefc_J_colind``,
  ``tree_island``, ``dof_island``, ``tendon_efcadr``,
  ``island_ntree``, ``island_nv``, ``island_ne``, ``island_nf``, ``island_nefc``,
  ``island_itreeadr``, ``island_idofadr``, ``island_dofadr``, ``island_iefcadr``,
  ``map_itree2tree``, ``map_dof2idof``, ``map_idof2dof``,
  ``map_efc2iefc``, ``map_iefc2efc``
- *Island inertia sparsity* --- ``iM_rownnz``, ``iM_rowadr``, ``iM_colind``
- *Tendon wrap / actuator moment Jacobian / awake-body indices* ---
  ``ten_wrapadr``, ``ten_wrapnum``, ``moment_rownnz``, ``moment_rowadr``,
  ``moment_colind``, ``body_awake_ind``, ``parent_awake_ind``, ``dof_awake_ind``

On |mjv_scene|, **10 flex and skin geometry address arrays** became read-only:
``flexedgeadr``, ``flexedgenum``, ``flexvertadr``, ``flexvertnum``,
``flexfaceadr``, ``flexfacenum``, ``flexfaceused``,
``skinfacenum``, ``skinvertadr``, ``skinvertnum``.


``MjData::print()`` and ``MjModel::print()``
-----------------------------------------------

Both ``print`` and ``print_formatted`` now accept ``AsRef<Path>`` for the filename
and return ``Result`` with the appropriate error type.

**Before (2.x):**

.. code-block:: rust

    data.print("data.txt");
    model.print("model.txt");

**After (3.0.0):**

.. code-block:: rust

    data.print("data.txt")?;
    model.print("model.txt")?;


Type changes
-----------------------------

- |mj_model|: ``size()`` returns ``MjtSize`` (was ``i32``).
- |mj_model|: ``state_size()`` returns ``usize`` (was ``i32``).
- |mj_model|: ``name_to_id()`` returns ``Option<i32>`` (was ``i32``; ``-1`` is now ``None``).
- |mj_model|: ``tuple_objtype()`` returns ``&[MjtObj]`` (was ``&[i32]``).
- |mj_model|: ``maxuse_threadstack()`` returns ``&[MjtSize; mjMAXTHREAD]`` (was ``&[MjtSize]``).
- |mj_data|: ``contact_force()`` takes ``contact_id: u32`` (was ``usize``).
- |mjs_tendon|: ``limited`` and ``actfrclimited`` are now ``MjtLimited`` tri-state (was ``bool``).


Ray-casting parameter changes
------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` gained a new
``normal_out`` parameter. Pass ``None`` to preserve previous behaviour.

**Before (2.x):**

.. code-block:: rust

  let (geom_id, dist) = data.ray(&pnt, &vec, None, true, -1);

**After (3.0.0):**

.. code-block:: rust

  let (geom_id, dist) = data.ray(&pnt, &vec, None, true, -1, None);

Similarly, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray`
gained ``normals_out`` and now returns ``Result``. Pass ``None``.

:docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom` also gained
``normal_out: Option<&mut [MjtNum; 3]>``.

**Before (2.x):**

.. code-block:: rust

  let dist = mju_ray_geom(&pos, &mat, &size, &pnt, &vec, geomtype);

**After (3.0.0):**

.. code-block:: rust

  let dist = mju_ray_geom(&pos, &mat, &size, &pnt, &vec, geomtype, None);


``MjData::jac_subtree_com()`` parameter change
------------------------------------------------

The ``jacp: bool`` parameter has been removed (the Jacobian is always computed).

**Before (2.x):**

.. code-block:: rust

    let jac = data.jac_subtree_com(true, body_id)?;

**After (3.0.0):**

.. code-block:: rust

    let jac = data.jac_subtree_com(body_id)?;


``MjvPerturb::update_local_pos``
------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`
now takes ``selection_xyz`` by reference (``&[MjtNum; 3]``) instead of by value.

**Before (2.x):**

.. code-block:: rust

    perturb.update_local_pos(xyz, &data);

**After (3.0.0):**

.. code-block:: rust

    perturb.update_local_pos(&xyz, &data);


``MjvPerturb::move_``
------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>move_`
no longer takes a separate ``model`` parameter (the model is obtained from
``data.model()``), and ``data`` changed from ``&mut MjData<M>`` to ``&MjData<M>``.

**Before (2.x):**

.. code-block:: rust

    perturb.move_(&model, &mut data, action, dx, dy, &scene);

**After (3.0.0):**

.. code-block:: rust

    perturb.move_(&data, action, dx, dy, &scene);


``find_selection()`` return type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|mjv_scene|'s ``find_selection`` now returns a
:docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>SceneSelection` named
struct instead of a 5-tuple.

**Before (2.x):**

.. code-block:: rust

    let (body_id, geom_id, flex_id, skin_id, point) = scene.find_selection(&data, ...);

**After (3.0.0):**

.. code-block:: rust

    let sel = scene.find_selection(&data, ...);
    println!("{} {} {} {} {:?}", sel.body_id, sel.geom_id, sel.flex_id, sel.skin_id, sel.point);


Core ``Send``/``Sync`` bound tightening
-----------------------------------------

``MjData<M>`` and ``MjvScene<M>`` now require ``M: Send`` / ``M: Sync``.
``MjViewerCpp<M>`` now requires ``M: Send + Sync``.

If you were using ``Rc<MjModel>`` in threaded code, switch to ``Arc<MjModel>``.


New ``unsafe`` requirements
------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>new`
is now ``unsafe fn``. A valid OpenGL context must be current on the calling thread.
In most cases you will not call this directly (``MjRenderer`` and ``MjViewer`` call
it internally). If you construct ``MjrContext`` manually:

**Before (2.x):**

.. code-block:: rust

    let context = MjrContext::new(&model);

**After (3.0.0):**

.. code-block:: rust

    // SAFETY: a valid GL context has been made current above.
    let context = unsafe { MjrContext::new(&model) };


``MjData::set_state`` and ``try_set_state`` are now ``unsafe``
----------------------------------------------------------------

When ``spec`` includes ``mjSTATE_EQ_ACTIVE``, MuJoCo writes raw ``f64`` bytes into
the ``eq_active`` byte array without booleanization, making a subsequent call to
``eq_active()`` undefined behaviour. Re-validate by calling ``mj_forward`` /
``mj_step`` before reading ``eq_active()``.

**Before (2.x):**

.. code-block:: rust

    data.set_state(&saved, MjtState::mjSTATE_FULLPHYSICS as u32);

**After (3.0.0):**

.. code-block:: rust

    // SAFETY: state captured via get_state; bools are valid (0 or 1).
    unsafe { data.set_state(&saved, MjtState::mjSTATE_FULLPHYSICS as u32) };


API renames
-----------------------------

.. list-table::
   :header-rows: 1

   * - Type
     - Old name
     - New name
   * - |mj_model|
     - ``get_totalmass()``
     - ``totalmass()``
   * - |mjr_context|
     - ``mjr_set_buffer()``
     - ``set_buffer()``
   * - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`
     - ``figure()``
     - ``draw()``


``try_sync()``
-----------------------------

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` has been
added as a fallible alternative to ``sync()``. The existing ``sync()`` delegates to
``try_sync().expect()``. Use ``try_sync()`` if you need to handle scene-full errors.


C++ viewer changes
-----------------------

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp` now requires
``M: Send + Sync``. Switch to ``Arc<MjModel>`` if using ``Rc<MjModel>``.

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` is now
``unsafe fn``, must be called from the main thread, no longer accepts
``update_timer``, and returns ``Result<(), &'static str>``.

``MjViewerCpp::__raw()`` has been removed (no replacement).

**Before (2.x):**

.. code-block:: rust

    viewer.render(true);

**After (3.0.0):**

.. code-block:: rust

    // SAFETY: called from the main thread.
    unsafe { viewer.render().unwrap() };


``MjViewerCpp::launch_passive()`` is now ``unsafe``
------------------------------------------------------

Callers must ensure the model and data remain alive and at a stable address.

**Before (2.x):**

.. code-block:: rust

  let viewer = MjViewerCpp::launch_passive(&model, &data, 100);

**After (3.0.0):**

.. code-block:: rust

    // SAFETY: model and data kept alive and at a stable address.
    let viewer = unsafe { MjViewerCpp::launch_passive(&model, &data, 100) };


Removed deprecated methods
----------------------------

.. list-table::
   :header-rows: 1

   * - Removed
     - Replacement
   * - ``MjData::warning_stats``
     - ``MjData::warning``
   * - ``MjData::timer_stats``
     - ``MjData::timer``
   * - Type aliases ``MjJointInfo``, ``MjJointView``, ``MjJointViewMut``,
       ``MjGeomInfo``, ``MjGeomView``, ``MjGeomViewMut``,
       ``MjActuatorInfo``, ``MjActuatorView``, ``MjActuatorViewMut``
     - Use the ``MjJointData*``, ``MjGeomData*``, ``MjActuatorData*`` types.
   * - ``MjJointDataViewMut::reset``
     - ``MjJointDataViewMut::zero``
   * - ``MjModel::name2id``
     - ``MjModel::name_to_id``
   * - ``MjvCamera::new``
     - ``MjvCamera::new_free``, ``new_fixed``, ``new_tracking``, or ``new_user``
   * - ``MjViewer::user_scene``, ``user_scene_mut``, ``user_scn``, ``user_scn_mut``
     - ``ViewerSharedState::user_scene`` / ``user_scene_mut`` via
       :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>state`
   * - ``MjViewer::sync``
     - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data` then
       :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render`
