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

