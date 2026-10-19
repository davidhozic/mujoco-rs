Unreleased [7.0.0] (MuJoCo 3.12.0)
==================================

.. rubric:: Breaking changes

- ``treenum`` is now marked as unsafe to mutate (wrong values can cause out-of-bound reads). Consequently,

  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTendonModelViewMut` is now
    :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`;
  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tendon_treenum_mut` is now an ``unsafe fn``.
- The ``plugin`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut` and
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjBodyModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>eq_data_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_stiffness_mut` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_bending_mut` are now ``unsafe fn``,
  and the ``data`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjEqualityModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`. A wrong value in a flex strain equality or an
  interpolated flex shell made a step read out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_biastype_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_dynprm_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_gainprm_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>body_simple_mut`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_rigid_mut` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_centered_mut` are now
  ``unsafe fn``. The ``biastype``, ``dynprm`` and ``gainprm`` fields of
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut` and the ``simple`` field of
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjBodyModelViewMut` are now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.
  A value that disagreed with the rest of the model, made a step access memory out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tuple_objtype_mut` is now an ``unsafe fn``.
  The related ``objtype`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTupleModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.
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
- The member accessors of :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault` (``joint``,
  ``geom``, ``actuator`` and the others, with their ``_mut`` variants) now return handles marked with
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>InDefault` :sup:`new`, for example
  ``&mut MjsJoint<InDefault>``. These handles keep the field accessors, ``info`` and the actuator
  ``set_to_*`` methods, but no longer implement
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecElement`, ``SpecItem``, ``SpecObject``,
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>FrameChild`,
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>Attach` or
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>UserValued`, because their methods crashed on a
  default-class member. The handles of these kinds gain a type parameter that defaults to
  :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>InTree` :sup:`new`, thus writing ``MjsJoint``
  gives the same result as before.
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>contact_force`
  now panics when ``contact_id >= ncon``. Before, it returned ``[0; 6]``. If you want to catch
  out-of-bounds errors, use
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_contact_force` :sup:`new`.
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`,
  ``is_asset_compatible_with_model`` and the ``Info`` views now compare every size and every
  integer table that no safe accessor writes, apart from the names, the paths, the buffer size, the
  collision hulls and trees and the values derived from floats, so they refuse more models, for
  example a model with a different joint type. Before, a swapped model could make a step or a plugin read or write out of bounds.
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
  and ``is_compatible_with_scene`` now refuse more models, and ``MjvScene::update`` panics on each of
  them. A scene or a user scene that was filled from an accepted, but different, model could read or
  write out of bounds. :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` rebuilds its scenes on its own.
- :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IncompatibleModel` and
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>IncompatibleModel` now carry the
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModelLayout` :sup:`new` of both models instead of
  their signatures.
- :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>UnsupportedObjectType` now carries an
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtObjBase` :sup:`new` instead of an ``i32``.
- Removed ``MjvScene::signature`` and the ``model_signature`` method of every ``Info`` type. Read
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>signature` on the model instead.
- Model and specification fields and functions that accept only real geom types, real object
  types or real texture roles now use :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtGeomBase`
  :sup:`new`, :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtObjBase` :sup:`new` and
  :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtTextureRoleBase` :sup:`new`, which hold only
  the variants below ``mjNGEOMTYPES``, ``mjNOBJECT`` and ``mjNTEXROLE``.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>set_objtype`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>set_reftype` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTuple::<method>set_objtype` no longer
  return a ``Result``, and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>with_objtype` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>with_reftype` no longer
  panic, because the type now rejects the invalid object types.
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewerBuilder::<method>build_passive`,
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`,
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRendererBuilder::<method>build` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new` now take the model as
  ``&MjModel`` (|mj_model|) instead of a generic model type, because they only need the reference at construction.
- :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive` no longer takes a
  ``model`` parameter and uses the model of ``data``.


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

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name` on a |mjs_wrap| now makes
  the wrap target the object with the new name. It returned
  :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>UnsupportedOperation` before.

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecObject::<method>delete` now deletes a frame
  and its subtree. It returned ``UnsupportedOperation`` for a frame before.

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
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model_unchecked` :sup:`new`
    swaps in a model that the compatibility check refuses.

- |mj_model|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>layout` :sup:`new`
    returns the layout that the compatibility checks compare.

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
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut::<method>zero` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjBodyModelViewMut::<method>zero` no longer set
  ``plugin`` to ``0``, which bound the actuator or body to plugin instance 0 instead of no plugin (``-1``).
- Fixed potential race condition when using separate :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`
  instances in different threads. |mjr_context| and related wrappers now use a static mutex to prevent
  parallel access do the same data (``context_count`` in MuJoCo C code).
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_visual_to` no longer leaves
  the destination in a state where a stage method (for example ``fwd_velocity``) crashes.
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now makes the viewer's OpenGL
  context current again after the user UI callbacks. A callback that made another context current
  caused the viewer to draw into that context.
- Fixed a hole in :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>overlay`,
  which allowed non-ASCII input to be passed into the function, causing possible out-of-bound reads.
  The function now asserts ASCII input and panics on non-ASCII input.
- The vector and matrix functions of ``mujoco_rs::wrappers::fun::utility`` (for example
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_dot`) now panic when a slice or a matrix
  has more than ``i32::MAX`` elements. Before, the length wrapped and MuJoCo used a wrong range.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>save_xml_string` now clamps
  ``buffer_size`` to ``i32::MAX``. Before, a larger size wrapped, and MuJoCo could write past the
  buffer.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray` now panic for a
  ``bodyexclude`` above ``i32::MAX``, which wrapped into another body ID before. ``multi_ray`` and
  ``try_multi_ray`` also panic for more than ``i32::MAX`` rays.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
  ``new_tracking``, ``fix`` and ``track`` now panic for an ID above ``i32::MAX`` in all builds.
  Before, the ID wrapped into another camera or body ID.
- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>resize_offscreen` now
  panic for a ``width`` or ``height`` above ``i32::MAX``, and ``add_aux`` clamps ``samples``.
  Before, the sizes wrapped into negative values.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile` now returns an error
  for a builtin cube or skybox texture whose size overflows ``i32``. Before, the check used
  ``height``, which such a texture ignores, and MuJoCo wrote out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile` now also returns an
  error for an invalid flex ``order``, ``cellcount`` or ``condim``, a negative actuator or sensor
  ``nsample``, a camera projection sensor that does not reference a camera, a mesh ``maxhullvert``
  in ``0..=3``, a plugin instance without a ``plugin_name``, and a height field whose
  ``nrow * ncol`` overflows ``i32``. Before, MuJoCo accepted these values and then crashed or read
  out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_flexcomp` now
  returns :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>InvalidParameter` for a
  ``count`` with more than ``i32::MAX`` points or a ``cellcount`` with a zero entry. Before,
  MuJoCo wrote out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>move_` now panics
  for a camera type other than free, tracking or fixed. Before, MuJoCo stopped the process.
- The buffer methods of :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`
  (for example :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>push`)
  now panic for a negative ``linepnt``, and ``draw`` panics for a negative ``legendoffset``.
  Before, both read out of bounds.
- |mjv_scene| ``update`` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update_with_catmask`
  now panic for a negative ``vis.quality.numslices``. Before, MuJoCo wrote out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size`, the state
  methods that use it, and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_state_from_data` now panic
  for a ``spec`` with an unknown bit. Before, MuJoCo stopped the process.
