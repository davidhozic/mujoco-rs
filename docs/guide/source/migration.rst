.. _migration:

=======================
Migration guide
=======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`


This page documents the migration steps for upgrading between major versions of MuJoCo-rs.
For a full list of changes see the :ref:`changelog`.


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
        Err(MjModelError::InvalidUtf8Path) => {
            eprintln!("Path is not valid UTF-8");
        }
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
   * - |mj_model| (``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``from_buffer``, ``save_last_xml``)
     - ``io::Error``
     - ``MjModelError``
   * - |mj_vfs| (``add_from_file``, ``add_from_buffer``, ``delete_file``)
     - ``io::Error``
     - ``MjVfsError``
   * - |mj_spec| (``from_xml``, ``from_xml_vfs``, ``from_xml_string``, ``compile``, ``save_xml``, ``save_xml_string``)
     - ``io::Error``
     - ``MjEditError``
   * - :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` (``try_sync``)
     - ``MjDataError``
     - ``RendererError``
   * - :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` / :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`
       ``GlInitFailed`` variant
     - ``String``
     - ``GlInitError``


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


``try_sync()`` return type
-----------------------------

:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` now returns
``Result<(), RendererError>`` instead of ``Result<(), MjDataError>``.
If your code handled ``MjDataError`` from ``try_sync``, update the match to use
``RendererError`` variants instead.


``MjvPerturb::update_local_pos``
------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`
now takes ``selection_xyz`` by reference (``&[MjtNum; 3]``) instead of by value,
and the ``model`` parameter has been removed.

**Before (2.x):**

.. code-block:: rust

    perturb.update_local_pos(&model, &data, xyz);

**After (3.0.0):**

.. code-block:: rust

    perturb.update_local_pos(&xyz, &data);


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

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` no longer accepts the
``update_timer`` boolean parameter. The FPS timer is now always updated.

**Before (2.x):**

.. code-block:: rust

    viewer.render(true);

**After (3.0.0):**

.. code-block:: rust

    viewer.render();
