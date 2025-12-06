==================
Changelog
==================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_geomview| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjGeomView`
.. |mj_geomviewmut| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<type>MjGeomViewMut`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`


Versioning
=================
This project uses `semantic versioning <https://semver.org/>`_.
This means that any incompatible changes increase the major version (**Y**.x.x).
This also includes breaking changes that MuJoCo itself introduced, thus even an
update of MuJoCo alone can increase the major version.

Unreleased (MuJoCo 3.3.7)
================================
- Made :ref:`mj_renderer` use EGL backend by default on Linux for true offscreen rendering (pbuffer).
  
  - New feature added for compatibility purposes: ``renderer-winit-fallback``.
    The feature is enabled by default.


2.1.0 / 2.1.1 (MuJoCo 3.3.7)
================================
- Option to automatically pull MuJoCo.
- pkg-config support for Linux and MacOS. Note that this is not officially supported by MuJoCo.
- Changes to :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`:

  - Added an optional (enabled by default) user interface, made with `egui <https://github.com/emilk/egui>`_.

- New Cargo features:

  - ``auto-download-mujoco``: will automatically download MuJoCo (Windows and Linux).

2.0.1 (MuJoCo 3.3.7)
================================
- Fix the ``renderer`` feature not enabling all the needed crates.

2.0.0 (MuJoCo 3.3.7)
================================
- **Breaking changes**:

  - Updated the MuJoCo version to 3.3.7.
  - :ref:`model_editing`:

    - Items (:docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsJoint`,
      :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsGeom`, etc.) are no longer wrapped and are instead
      just aliased types. Their attributes have been made private to users, so that tools like ``rust-analyzer``
      don't suggest both the getter name and the attribute name at once.
      As a result of this change, methods now return true references instead of the wrapper types.

    - Added immutable iterators.
    - Changed regular named-access methods to be immutable and added corresponding ``<item>_mut()`` methods
      for mutable access.
    - Replaced ``plugin_wrapper`` methods with a normal getter method (``plugin`` and ``plugin_mut``).

  - :ref:`mj_rust_viewer` and :ref:`mj_renderer`:

    - Changed the backend windowing library to Winit (+ Glutin). This is a **potentially** breaking
      change because of possible direct GLFW uses in the user code, which will probably still work
      as expected, but we can't be sure as we did not test GLFW and Winit being used at the same time.
      Change to Winit also means we don't need any C dependencies, unless the C++ viewer wrapper
      is needed, which also contains breaking changes. The latter is described in the next bullet.
    
    - Added and removed variants in :docs-rs:`mujoco_rs::viewer::<enum>MjViewerError` and
      :docs-rs:`mujoco_rs::renderer::<enum>RendererError`.

  - :ref:`mj_cpp_viewer`:

    - Since MuJoCo's build systems downloads GLFW sources anyway, we decided to remove the GLFW
      requirement from the Rust level and instead made it so that the user needs to compile the GLFW
      code during the MuJoCo's viewer (simulate) compilation.
      No change is needed in the user Rust code, users just need to build MuJoCo a bit differently:

      ``cmake --build build --parallel --target glfw libmujoco_simulate --config=Release``.

      The above command, besides the added ``glfw`` part, also contains the ``libmujoco_simulate``
      part in place of the previously ``libsimulate`` part. This change is a consequence
      of MuJoCo upgrade to version 3.3.7.

    - Moved the struct definition from ``mujoco_rs::viewer`` to ``mujoco_rs::cpp_viewer``.

  - Changed |mj_data| and other types to accept a generic type for the model,
    constrained to ``Deref<Target = MjModel>``.
    This enables use in environments such as `PyO3 <https://github.com/PyO3/pyo3>`_.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsMesh`: changed ``smoothnormal`` and ``needsdf`` to be treated as booleans.
  - |mj_data| methods:

    - Renamed ``crb`` to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>crb_comp` due to ``crb``
      now being a method that returns an immutable slice to the ``crb`` attribute of the ffi type,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy` now returns a reference to a 2-element array instead of a slice,

  - |mj_model| methods:

    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>id_to_name` now accepts ``i32`` instead of ``c_int``,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>size` now returns ``i32`` instead of ``c_int``,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size` now accepts ``u32`` instead of ``c_uint``
      and returns ``i32`` instead of ``c_int``,

  - :docs-rs:`mujoco_rs::mujoco_c`:

    - :docs-rs:`~mujoco_rs::mujoco_c::<enum>mjtSameFrame_` is now ``repr(u8)`` instead of ``repr(u32)``
      to fix alignment issues with MuJoCo's structs,

  - :docs-rs:`mujoco_rs::wrappers::fun::utility`:

    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_band_diag`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_eig_3`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_halton`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_is_bad`: replaced ``c_int`` types with ``bool``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_mat_2_rot`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom`: replaced ``c_int`` types with :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtGeom`,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_round`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_transform_spatial`: replaced ``c_int`` types with ``bool``,

  - Removed modules:

    - ``mujoco_rs::wrappers::mj_interface``: this was in early development, but then it became apparent
      that its completion and usage would violate borrow checker rules, resulting in undefined behavior.

- Other changes:

  - Any changes to MuJoCo made in MuJoCo 3.3.6 and MuJoCo 3.3.7 (see https://mujoco.readthedocs.io/en/3.3.7/changelog.html).
  - Added additional getters / setters / array slice methods to:

    - |mj_data|,
    - |mj_model|,
    - |mjv_scene|.

1.5.0 (MuJoCo 3.3.5)
================================
- |mjv_scene|:

  - Added the :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>pop_geom` method
    for popping a single geom from the end of the scene.

- :ref:`model_editing`:

  - Added iterators to :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
    and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`.


1.4.2 (MuJoCo 3.3.5)
================================
Bug fixes:

- Fixed segmentation fault when the model specification is invalid. (`#65 <https://github.com/davidhozic/mujoco-rs/issues/65>`_).

1.4.1 (MuJoCo 3.3.5)
================================
Bug fixes:

- Added missing named accessors to :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>geom`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>site`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>camera`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>light`.

1.4.0 (MuJoCo 3.3.5)
================================
- |mj_model|:

  - Added more views:
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>key` (keyframe),
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tuple`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>texture`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>site`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>pair`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>numeric`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>material`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>light`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>hfield`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>equality`.

- Added extra renames of enum types from the C API.

- :ref:`model_editing` support, which can be used to procedurally generate |mj_model|. It can be used
  for terrain generation, parameter randomization, etc. 

  - Added module :docs-rs:`mujoco_rs::wrappers::mj_editing`.
  - Added two examples. One on basic model editing and one on terrain generation.

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:

  - Added additional getters and setters.
  - Added :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` for purposes of better
    configuration.

- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`:

  - Added events for keys:

    - ``Backspace``: resets the simulation,
    - ``F5``: toggles full screen mode,
    - ``[`` and ``]``: cycles cameras,
    - Visualization toggles:

      - ``C``: camera,
      - ``U``: actuator,
      - ``J``: joint,
      - ``M``: center of mass,
      - ``H``: convex hull,
      - ``Z``: light,
      - ``T``: transparent,
      - ``I``: inertia.

    - Increased the headroom for visual-only geoms, which aren't drawn by the user, from 100 to 2000.

- :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`:

  - Added methods:

    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::fix`:
      changes the camera struct parameters to display a fixed camera.

1.3.0 (MuJoCo 3.3.5)
================================
- Added a module for offscreen scene rendering: :docs-rs:`mujoco_rs::renderer`:

  - Added :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` for actual offscreen rendering of the simulation state.

- Deprecated :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new` and replaced it with:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_free`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_tracking` and
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_user`.

- Deprecated :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn` and
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scn_mut`. They are replaced with
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scene` and
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>user_scene_mut`.

- Added new methods for obtaining public attributes:

  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`:

    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_stack`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_threadstack`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>warning_stats`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>timer_stats`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>time`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy`.

  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`:

    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>signature`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>opt`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>opt_mut`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>vis`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>vis_mut`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>stat`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>stat_mut`.

- Added extra attributes to the :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`'s joint view:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_spring`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_damper`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_gravcomp`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_fluid`;


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
