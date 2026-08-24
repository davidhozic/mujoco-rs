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

