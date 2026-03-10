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


This page documents the migration steps for upgrading between major versions of MuJoCo-rs.
For a full list of changes, see the :ref:`changelog`.


.. _migrate_3_0_0:

Migrating to 3.0.0
======================

Version 3.0.0 updates the bundled MuJoCo to 3.5.0, introduces typed error enums,
and removes previously deprecated APIs.


MuJoCo upgrade
-----------------------
MuJoCo-rs 3.0.0 links against MuJoCo **3.5.0**. Make sure to download and use the
matching MuJoCo release. See :ref:`installation` for details.


Error handling
-----------------------

The most significant change in 3.0.0 is the introduction of typed error enums in place
of ``std::io::Error``.

New error types
~~~~~~~~~~~~~~~~~~~~~~~~

Six new error types have been added to the :docs-rs:`~mujoco_rs::error` module,
all ``#[non_exhaustive]`` and re-exported from the prelude:

- :docs-rs:`~mujoco_rs::error::<enum>MjDataError` --- errors from |mj_data| operations.
- :docs-rs:`~mujoco_rs::error::<enum>MjSceneError` --- errors from |mjv_scene| and rendering operations.
- :docs-rs:`~mujoco_rs::error::<enum>MjEditError` --- errors from |mj_spec| and model editing operations.
- :docs-rs:`~mujoco_rs::error::<enum>MjModelError` --- errors from |mj_model| loading, saving, and state operations.
- :docs-rs:`~mujoco_rs::error::<enum>MjVfsError` --- errors from |mj_vfs| (virtual file system) operations.
- :docs-rs:`~mujoco_rs::error::<enum>GlInitError` --- OpenGL context initialization errors
  (requires the ``viewer`` or ``renderer-winit-fallback`` feature).

Additionally, :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` and
:docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError` have gained new variants.

Updating error handling code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If your code previously matched on ``io::ErrorKind``, you now match on the specific error enum variants.

**Before (2.x):**

.. code-block:: rust

    use std::io;

    match model.save_last_xml("out.xml") {
        Ok(()) => { /* ... */ }
        Err(e) if e.kind() == io::ErrorKind::InvalidInput => {
            eprintln!("Invalid path: {e}");
        }
        Err(e) => {
            eprintln!("Save failed: {e}");
        }
    }

**After (3.0.0):**

.. code-block:: rust

    use mujoco_rs::prelude::*;

    match model.save_last_xml("out.xml") {
        Ok(()) => { /* ... */ }
        Err(MjModelError::SaveFailed(msg)) => {
            eprintln!("Save failed: {msg}");
        }
        Err(e) => {
            eprintln!("Other error: {e}");
        }
    }

The table below maps the breaking error-type changes:

.. list-table::
   :header-rows: 1

   * - Module / type
     - 2.x error type
     - 3.0.0 error type
   * - |mj_model| (``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``from_buffer``, ``save_last_xml``, ``save_to_file``, ``save_to_buffer``)
     - ``io::Error``
     - ``MjModelError``
   * - |mj_vfs| (``add_from_file``, ``add_from_buffer``, ``delete_file``)
     - ``io::Error``
     - ``MjVfsError``
   * - |mj_spec| (``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``compile``, ``save_xml``, ``save_xml_string``, ``add_default``)
     - ``io::Error``
     - ``MjEditError``
   * - |mj_data| (``add_contact``)
     - ``io::Error``
     - ``MjDataError``
   * - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` (``rgb``, ``depth``, ``save_rgb``, ``save_depth``, ``save_depth_raw``)
     - ``io::Error``
     - ``RendererError``
   * - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` (``try_sync``)
     - (new method)
     - ``Result<(), RendererError>``
   * - :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` / :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`
     - (new variant)
     - ``GlInitFailed(GlInitError)`` added alongside existing ``GlutinError``


``MjModel::clone()``
-----------------------

|mj_model| now implements the standard ``Clone`` trait. The previous ``clone()`` method
that returned ``Option<MjModel>`` has been replaced:

**Before (2.x):**

.. code-block:: rust

    let model_copy = model.clone().expect("clone failed");

**After (3.0.0):**

.. code-block:: rust

    // Panics on failure (Clone trait):
    let model_copy = model.clone();

    // Or use the fallible version:
    let model_copy = model.try_clone()?;


``try_sync()``
-----------------------------

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` has been added as a
fallible alternative to :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync`.
The existing ``sync()`` method still works but now delegates to ``try_sync().expect()``.
If your code needs to handle scene-full errors gracefully, use ``try_sync()`` instead.


``MjModel::save()`` split
-----------------------------

``MjModel::save(filename: Option<&str>, buffer: Option<&mut [u8]>)`` has been replaced by two
dedicated methods:

- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_to_file` --
  saves the model to a binary MJB file. Accepts ``AsRef<Path>`` and returns ``Result<(), MjModelError>``.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_to_buffer` --
  saves the model to a memory buffer and returns ``Result<(), MjModelError>``.
  The buffer must be at least :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>size`
  bytes; otherwise ``MjModelError::BufferTooSmall`` is returned.

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


``MjData::print()`` and ``MjModel::print()``
-----------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print_formatted`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>print`, and
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>print_formatted`
now accept ``AsRef<Path>`` for the filename parameter and return ``Result`` with the
corresponding error type (``MjDataError`` / ``MjModelError``).
The error is returned when the path contains invalid UTF-8.

**Before (2.x):**

.. code-block:: rust

    data.print("data.txt");
    model.print("model.txt");

**After (3.0.0):**

.. code-block:: rust

    data.print("data.txt")?;
    model.print("model.txt")?;

Newly fallible methods
-----------------------------

Several methods that previously returned bare types now return ``Result``.
Code that called these methods directly must now handle the ``Result``,
for example by appending ``.unwrap()`` or using ``?``.

|mj_data| methods now returning ``Result<_, MjDataError>``:
  ``constraint_update``, ``jac``, ``jac_body``, ``jac_body_com``, ``jac_subtree_com``,
  ``jac_geom``, ``jac_site``, ``angmom_mat``, ``object_velocity``, ``object_acceleration``,
  ``geom_distance``, ``local_to_global``, ``multi_ray``.

|mjv_scene| / :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom` / :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext` methods now returning ``Result<_, MjSceneError>``:
  ``create_geom``, ``set_label``, ``add_aux``, ``set_aux``.

:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now returns
``Result<(), MjViewerError>`` instead of ``()``.


``MjData::ray()`` parameter change
----------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` gained a new
``normal_out`` parameter. Pass ``None`` to preserve the previous behavior.

**Before (2.x):**

.. code-block:: rust

    let (geom_id, dist) = data.ray(&pnt, &vec, None, true, -1);

**After (3.0.0):**

.. code-block:: rust

    let (geom_id, dist) = data.ray(&pnt, &vec, None, true, -1, None);


Similarly, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray` gained a
new ``normals_out`` parameter and now returns ``Result``. Pass ``None`` to preserve the
previous behavior.


``mju_ray_geom()`` parameter change
------------------------------------------

:docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom` gained a new ``normal_out``
parameter (``Option<&mut [MjtNum; 3]>``). Pass ``None`` to preserve the previous behavior.

**Before (2.x):**

.. code-block:: rust

    let dist = mju_ray_geom(&pos, &mat, &size, &pnt, &vec, geomtype);

**After (3.0.0):**

.. code-block:: rust

    let dist = mju_ray_geom(&pos, &mat, &size, &pnt, &vec, geomtype, None);


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


API renames
-----------------------------

Several methods have been renamed for consistency with Rust conventions:

.. list-table::
   :header-rows: 1

   * - Type
     - Old name
     - New name
   * - |mj_model|
     - ``get_totalmass()``
     - ``totalmass()``
   * - :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`
     - ``mjr_set_buffer()``
     - ``set_buffer()``


``name_to_id()`` return type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>name_to_id` now returns
``Option<i32>`` instead of ``i32``. It returns ``None`` when the name is not found (the C API
returned ``-1``).

**Before (2.x):**

.. code-block:: rust

    let id = model.name_to_id(MjtObj::mjOBJ_BODY, "my_body");
    if id >= 0 { /* found */ }

**After (3.0.0):**

.. code-block:: rust

    let id = model.name_to_id(MjtObj::mjOBJ_BODY, "my_body").unwrap();
    // or use if let / match for fallible lookups


``find_selection()`` return type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|mjv_scene|'s :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>find_selection`
now returns an :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>SceneSelection` named struct
instead of a 5-tuple.

**Before (2.x):**

.. code-block:: rust

    let (body_id, geom_id, flex_id, skin_id, point) = scene.find_selection(&data, ...);

**After (3.0.0):**

.. code-block:: rust

    let sel = scene.find_selection(&data, ...);
    println!("{} {} {} {} {:?}", sel.body_id, sel.geom_id, sel.flex_id, sel.skin_id, sel.point);


Type changes
-----------------------------

- |mj_model|: ``size()`` now returns ``MjtSize`` instead of ``i32``.
- |mj_model|: ``state_size()`` now returns ``usize`` instead of ``i32``.
- |mj_data|: ``contact_force()`` now takes ``contact_id: u32`` instead of ``usize``.


Removed deprecated methods
----------------------------

The following APIs have been removed. Use the listed replacements:

.. list-table::
   :header-rows: 1

   * - Removed
     - Replacement
   * - ``MjData::warning_stats``
     - ``MjData::warning``
   * - ``MjData::timer_stats``
     - ``MjData::timer``
   * - Type aliases: ``MjJointInfo``, ``MjJointView``, ``MjJointViewMut``, ``MjGeomInfo``,
       ``MjGeomView``, ``MjGeomViewMut``, ``MjActuatorInfo``, ``MjActuatorView``, ``MjActuatorViewMut``
     - Use the ``MjJointData*``, ``MjGeomData*``, ``MjActuatorData*`` types
   * - ``MjJointDataViewMut::reset``
     - ``MjJointDataViewMut::zero``
   * - ``MjModel::name2id``
     - ``MjModel::name_to_id``
   * - ``MjvCamera::new``
     - ``MjvCamera::new_free``, ``new_fixed``, ``new_tracking``, or ``new_user``
   * - ``MjvFigure::figure``
     - ``MjvFigure::draw``
   * - ``MjViewer::user_scene``, ``user_scene_mut``, ``user_scn``, ``user_scn_mut``
     - Use ``ViewerSharedState::user_scene`` / ``user_scene_mut`` via
       :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>state`
   * - ``MjViewer::sync``
     - Use :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data` followed by
       :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render`

Additionally, ``MjData::maxuse_threadstack`` is still available but its return type changed from
``&[MjtSize]`` to ``&[MjtSize; mjMAXTHREAD]``.


C++ viewer changes
-----------------------

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp` now requires the model handle ``M``
to implement ``Send + Sync`` (in addition to ``Deref<Target = MjModel> + Clone``). If you were
passing a non-``Send``/``Sync`` wrapper (e.g., ``Rc<MjModel>``), switch to ``Arc<MjModel>``.

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` no longer accepts the
``update_timer`` boolean parameter. The FPS timer is now always updated.
The return type also changed from ``()`` to ``Result<(), &'static str>``.

``MjViewerCpp::__raw()`` has been removed. There is no direct replacement; the method
exposed internal C++ binding pointers that are no longer part of the public API.

**Before (2.x):**

.. code-block:: rust

    viewer.render(true);

**After (3.0.0):**

.. code-block:: rust

    viewer.render().unwrap();
