.. _migrate_unreleased:

Unreleased [7.0.0] (MuJoCo 3.12.0)
==================================


Tendon tree count is unsafe to mutate
-------------------------------------

``treenum`` is now marked as unsafe to mutate (wrong values can cause out-of-bound reads). Consequently,

- :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTendonModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`;
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tendon_treenum_mut` is now an ``unsafe fn``.

.. code-block:: rust

    // Before
    model.tendon_treenum_mut()[0] = 1;
    model.tendon("t").unwrap().view_mut(&mut model).treenum[0] = 1;

    // After
    // SAFETY: tendon 0 spans exactly one tree.
    unsafe { model.tendon_treenum_mut()[0] = 1; }
    let mut view = model.tendon("t").unwrap().view_mut(&mut model);
    // SAFETY: tendon "t" spans exactly one tree.
    unsafe { view.treenum.as_mut_slice()[0] = 1; }


Removed ``append_cubefiles`` and ``append_textures``
----------------------------------------------------

Removed ``append_cubefiles`` and ``append_textures`` from
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture` and
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial`, respectively.
``cubefiles`` and ``textures`` require a fixed number of elements.
Use :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefile` and
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_texture`, respectively,
to set the elements.

.. code-block:: rust

    // Before
    texture.append_cubefiles("right.png");
    material.append_textures("wood");

    // After
    texture.set_cubefile(MjtCubeFace::Right, "right.png");
    material.set_texture(MjtTextureRole::mjTEXROLE_RGB, "wood");
