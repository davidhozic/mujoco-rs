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

