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


Removed the frame and body adders of ``MjsFrame``
-------------------------------------------------

Removed ``add_frame``, ``try_add_frame``, ``add_body``, ``try_add_body`` and
``add_body_with_class`` from |mjs_frame|. Add elements to bodies instead and assign them into
a frame using
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>FrameChild::<method>with_frame` or
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>FrameChild::<method>set_frame`.

Nested frames can also be created using
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_frame_in`.

.. code-block:: rust

    // Before
    let outer = world.add_frame().with_name("outer");
    outer.add_frame().with_name("inner");
    outer.add_body().with_name("ball");
    outer.add_body_with_class("cls").unwrap();

    // After
    world.add_frame().with_name("outer");
    world.add_frame_in("outer").unwrap().with_name("inner");
    world.add_body().with_name("ball").with_frame("outer").unwrap();
    world.add_body_with_class("cls").unwrap().with_frame("outer").unwrap();

The other element adders of |mjs_frame| are deprecated. Replace them in the same way:

.. code-block:: rust

    // Before
    frame.add_geom();
    frame.add_site_with_class("cls").unwrap();

    // After
    body.add_geom().with_frame("outer").unwrap();
    body.add_site_with_class("cls").unwrap().with_frame("outer").unwrap();

We are aware that the new API may be less clean, however, the change was necessary
due to a high chance of possible accidental aliasing (two mutable references with same address)
problems being created. Notably, deprecated-only methods are technically safe, however,
they internally do the same thing as the new API. We deprecated them (slated for removal)
for consistency reasons. We feel it would be annoying trying to figure out how to add
bodies and frames through the new API if the old API was still meant to be used
for other items.


``MjsDefault`` is no longer a ``SpecItem``
-------------------------------------------

:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault` no longer implements
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem`, thus a default class cannot be renamed.
Renaming a default class caused undefined behavior.

It now implements only the new supertrait :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecElement`,
the latter holding ``element_pointer``, ``element_mut_pointer`` and ``name``.

Code that previously relied on (or imported explicitly) ``SpecItem`` directly should
also add an import of ``SpecElement``, in addition to updating the type bounds in generics.

.. code-block:: rust

    // Before
    use mujoco_rs::wrappers::SpecItem;
    fn print_name<T: SpecItem>(item: &T) { println!("{}", item.name()); }
    let class = spec.add_default("a", None);
    class.set_name("b").unwrap();
    print_name(class);

    // After
    use mujoco_rs::wrappers::{SpecElement, SpecItem};
    fn print_name<T: SpecElement>(item: &T) { println!("{}", item.name()); }
    let class = spec.add_default("b", None);
    print_name(class);
