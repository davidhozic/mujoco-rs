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
- Removed ``add_frame``, ``try_add_frame``, ``add_body``, ``try_add_body`` and
  ``add_body_with_class`` from |mjs_frame|. A frame that a mutable frame iterator yielded could add
  another frame. The same iterator would then end up yielding the newly added frame again, resulting in
  active two mutable borrows (aliasing!).
  Use :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_frame_in` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>FrameChild::<method>with_frame` instead.
- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault` no longer implements
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem`, because renaming a default class
  caused undefined behavior. It implements only the new
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecElement` :sup:`new`, which is now the
  supertrait of ``SpecItem``.
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>contact_force`
  now panics when ``contact_id >= ncon``. Before, it returned ``[0; 6]``. If you want to catch
  out-of-bounds errors, use
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_contact_force` :sup:`new`.
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
  and ``is_compatible_with_scene`` now refuse more models, and ``MjvScene::update`` panics on each of
  them. A scene or a user scene that was filled from an accepted, but different, model could read or
  write out of bounds. :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` rebuilds its scenes on its own.
- :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IncompatibleModel` and
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>IncompatibleModel` now carry the
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModelLayout` :sup:`new` of both models instead of
  their signatures.
- Removed ``MjvScene::signature`` and the ``model_signature`` method of every ``Info`` type. Read
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>signature` on the model instead.


.. rubric:: Deprecations

- Deprecated the ``add_*`` and ``add_*_with_class`` element adders of |mjs_frame|.
  Instead, add elements to the same body as the ``MjsFrame`` and call
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>FrameChild::<method>with_frame`.

.. rubric:: Error handling

- New variant :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>FrameParentMismatch` :sup:`new`.

.. rubric:: New features and improvements

*Model editing*

- Added the :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>FrameChild` :sup:`new` trait
  and its methods, for setting the parent frame of body items/elements.

- |mjs_body|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>add_frame_in` :sup:`new`
    for adding a nested frame.

*Rendering*

- |mjr_context|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>is_compatible_with_model` :sup:`new`
    reports whether a context can serve another model.

*Simulation*
- |mj_data|:

  - A fallible method, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_contact_force` :sup:`new`,
    has been created for handling out-of-bound errors.

.. rubric:: Bug fixes

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_textures` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefiles` now fill
  the remaining needed parts the vector with empty entries. A shorter length of vectors caused
  out-of-bound reads.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>contact_force` now checks
  the ID before the call. The cast to ``i32`` could also wrap an ID of ``2^32`` or more into an in-bound,
  but wrong, contact index, thus the method returned the force of another contact.
- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` now also checks its rendering context:
  ``sync_data`` rebuilds the context, and the ``update_*_from`` methods return
  :docs-rs:`~~mujoco_rs::renderer::<enum>RendererError::<variant>IncompatibleModel`, when the context
  cannot serve the model. Before, an upload could read out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`
  now refuses a model whose history buffer divides differently between the actuators and the
  sensors. Before, a swapped model could make a step read out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`
  now refuses a model that binds its plugin instances to other elements. Before, a swapped model
  could make a plugin write out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`
  no longer compares the actuator bias types, the sensor data types and the sensor stages. Safe
  setters can change them, so a cached result could go stale.
