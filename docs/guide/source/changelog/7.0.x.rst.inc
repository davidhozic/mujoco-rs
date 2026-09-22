Unreleased [7.0.0] (MuJoCo 3.12.0)
==================================

.. rubric:: Breaking changes

- ``treenum`` is now marked as unsafe to mutate (wrong values can cause out-of-bound reads). Consequently,
  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTendonModelViewMut` is now
    :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tendon_treenum_mut` is now an ``unsafe fn``.
- Removed ``append_cubefiles`` and ``append_textures`` from
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture` and
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial`, respectively.
  ``cubefiles`` and ``textures`` require a fixed number of elements.
  Use :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefile` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_texture`, respectively,
  to set the elements. As described in *Bug fixes* of this release log, these methods do their own
  appends of empty values automatically, in order to reach the required size, thus ``append`` methods
  would have no effect.


.. rubric:: Bug fixes

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_textures` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefiles` now fill
  the remaining needed parts the vector with empty entries. A shorter length of vectors caused
  out-of-bound reads.
