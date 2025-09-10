==================
Changelog
==================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_geomview| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjGeomView`.
.. |mj_geomviewmut| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjGeomViewMut`.


Versioning
=================
This project uses `semantic versioning <https://semver.org/>`_.

Future release (MuJoCo 3.3.5)
================================
- Added a module for offscreen scene rendering: :docs-rs:`mujoco_rs::renderer`:

  - Added the :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` for actual offscreen rendering of the simulation state.

- Deprecated :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new` and replaced it with:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_free`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_tracking` and
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_user`.


1.2.0 (MuJoCo 3.3.5)
================================
- Added function wrappers around some utility functions and both the derivative functions (available under :docs-rs:`mujoco_rs::wrappers::fun`).
- Completed the virtual file system wrapper.

  - Added methods :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>add_from_file` and :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>delete_file`.
  - Added method :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_vfs`.
  - Added a long list of additional methods to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
    and :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`, which wrap corresponding MuJoCo functions.
    See the Git diff on GitHub for more information.

1.1.0 (MuJoCo 3.3.5)
=====================
**Potentially breaking changes:**

- Fixed bug `#18 <https://github.com/davidhozic/mujoco-rs/issues/18>`_ where data races could occur
  under incorrect usage. The major version of MuJoCo-rs is not increased as this safety bugs
  should not be something to rely on.

Other bug fixes:

- Fixed bug `#17 <https://github.com/davidhozic/mujoco-rs/issues/17>`_ where the |mj_geomview| and |mj_geomviewmut|
  pointed to the wrong address, which belonged to the body and not the geom.
- Fixed bug `#19 <https://github.com/davidhozic/mujoco-rs/issues/19>`_ where a warning about the scene buffer
  would be printed when loading some of MuJoCo's example scenes.


Other changes:

- Added new module: :docs-rs:`mujoco_rs::wrappers::mj_primitive`.
- Added more attributes to the view to :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjJointView`
  and :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjJointViewMut`.
- Added more views. All the views available now:
    - |mj_data|: actuator, body, camera, geom, joint, light, sensor, site, tendon.
    - |mj_model|: actuator, body, camera, geom, joint, sensor, tendon.

1.0.1 (MuJoCo 3.3.5)
=====================
Bug fixes:

- Smaller changes inside Drop implementations to make sure there is no undefined behaviors.

1.0.0 (MuJoCo 3.3.5)
=====================
**Breaking changes:**

- Made all ``ffi_mut()`` methods require unsafe blocks.

Viewer:

- Help overlay (F1)
- User scene via :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn` and
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn_mut` for drawing custom visual-only geoms.
- Mouse perturbation of objects:

  - Rotate via Control
  - Translate via Control + Alt

0.4.3 (MuJoCo 3.3.5)
=====================
Build system:

- Removed unnecessary header files, reducing crate's file size.

0.4.2 (MuJoCo 3.3.5)
=====================
Build system:

- Improved clarity of environmental variables:

  - ``MUJOCO_DYNAMIC_LINK_LIB`` -> ``MUJOCO_DYNAMIC_LINK_DIR``
  - ``MUJOCO_STATIC_LINK_LIB`` -> ``MUJOCO_STATIC_LINK_DIR``

- Added some internal cargo features .

0.4.1 (MuJoCo 3.3.5)
=====================
- Fix event handling.

0.4.0 (MuJoCo 3.3.5)
=====================
- Change the package name to `mujoco-rs`.

0.3.0 (MuJoCo 3.3.5)
=====================
- Initial public release (previously private under a different project).
