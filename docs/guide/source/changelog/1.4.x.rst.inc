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

