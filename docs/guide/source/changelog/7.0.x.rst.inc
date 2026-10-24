Unreleased [7.0.0] (MuJoCo 3.17.0)
==================================

.. rubric:: Breaking changes

- MuJoCo-rs now builds against MuJoCo 3.17.0 FFI, which rolls up the upstream 3.13.0, 3.14.0,
  3.15.0, 3.16.0 and 3.17.0 releases.
  This directly affects all the wrappers, separate changelog will not be provided.
  See MuJoCo's changelog for this release:
  https://mujoco.readthedocs.io/en/3.17.0/changelog.html

- ``treenum`` is now marked as unsafe to mutate (wrong values can cause out-of-bound reads). Consequently,

  - the ``treenum`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTendonModelViewMut`
    is now :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`;
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
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>hfield_size_mut` is now an ``unsafe fn``,
  and the ``size`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjHfieldModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>site_type_mut` is now an ``unsafe fn``,
  and the ``r#type`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjSiteModelViewMut` is now
  :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.
  A wrong site type made a sensor or the scene update read out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>paths_mut` is now an ``unsafe fn``.
  A path without its terminating zero made MuJoCo read past the end of the buffer.
- (runtime-breaking) The ``zero`` method of a mutable view no longer zeroes the fields that are now
  ``PointerViewUnsafeMut`` (above). Write these fields through the unsafe view if you need them zeroed.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_diag_a`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_kbip`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_d`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_r`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_d`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_r`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_y`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_ar`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efm_c` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efm_l` are now ``unsafe fn``,
  and so are their ``_mut`` variants.
  A read taken before the pipeline stage that computes the array returns uninitialized memory,
  and ``efc_y`` and ``efc_ar`` stay uninitialized under some solver settings.
  Run that stage first, see :ref:`migrate_unreleased`.
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
- (runtime-breaking) :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>is_compatible_with_model`
  and the ``Info`` views now compare every size and every integer table that no safe accessor
  writes, apart from the names, the paths, the buffer size, the mesh faces and hulls, the bounding
  volume and octree nodes and the values derived from floats, so they refuse more models, for
  example a model in which a geom uses a different material. ``is_asset_compatible_with_model``
  compares the mesh, height field and texture part of them. Before, a swapped model could make a
  step or a plugin read or write out of bounds.
- (runtime-breaking)
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
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
  the variants below ``mjNGEOMTYPES``, ``mjNOBJECT`` and ``mjNTEXROLE``, respectively.
  These ``xBase`` enums can be converted into full enums via calls to ``.into()``.
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
- |mj_viewer_cpp| now owns the simulation data, meaning another thread can step it while the main
  thread renders:

  - :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive` takes the
    |mj_data| by value and no longer takes a ``model`` parameter. ``MjViewerCpp`` is now generic
    over a model type that implements
    :docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut`, for example
    ``MjViewerCpp<Box<MjModel>>``.
  - Access the data through
    :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>state` :sup:`new`, which
    returns the shared :docs-rs:`~mujoco_rs::cpp_viewer::<struct>MjCppViewerSharedState`
    :sup:`new`.
  - Removed ``user_scn_mut``. Push the user geoms into
    :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjCppViewerSharedState::<method>user_geoms_mut`
    :sup:`new`, which is an ``unsafe fn``, and call
    :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync_user_geoms` :sup:`new`.
  - :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` is no longer an
    ``unsafe fn``, and :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync` now
    returns ``true`` while the viewer is open.
  - ``MjViewerCpp`` no longer implements ``Send`` and ``Sync``. To step the simulation in another
    thread, send a clone of the shared state to that thread.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_read_ctrl` now return a
  ``Cow<[MjtNum]>`` with one value per control of the actuator, instead of a single ``MjtNum``,
  because an actuator can now have more than one control. Index the result to get a value.


.. rubric:: Deprecations

- Deprecated the ``add_*`` and ``add_*_with_class`` element adders of |mjs_frame|.
  Instead, add elements to the same body as the ``MjsFrame`` and call
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>FrameChild::<method>with_frame`.

.. rubric:: Error handling

- New variant :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>FrameParentMismatch` :sup:`new`.
- New variant :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>OperationFailed` :sup:`new`
  holds MuJoCo's error message when a spec operation other than compilation fails.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_flexcomp` now returns
  :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>InvalidParameter` with MuJoCo's error
  message when MuJoCo fails to create the flex, for example for a missing mesh file. Before, it
  returned ``AllocationFailed`` and lost the message.
- The ``Display`` output of :docs-rs:`~mujoco_rs::error::<enum>MjModelError`,
  :docs-rs:`~mujoco_rs::error::<enum>GlInitError`,
  :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` and
  :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError` no longer includes the inner error, because a
  reporter that walks ``source()`` printed it twice. Walk ``source()`` to show the cause.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>print_formatted`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>print` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>print_formatted` now return
  the new variants :docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IoError` :sup:`new` and
  :docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>IoError` :sup:`new` when the file
  cannot be opened. Before, they returned ``Ok(())`` and wrote nothing.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile` no longer checks
  texture values before it calls MuJoCo. Before, it returned an error for a builtin texture with
  ``nchannel`` below 3, a negative texture size or a texture size that overflows ``i32``. MuJoCo
  does not check every spec value, so give only valid values.

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
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>adopt_inertial` :sup:`new` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>adopt_inertial_with_vfs` :sup:`new`
    compute the inertial of a body from its geoms and write it into the spec.

- |mj_spec|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>fuse_static` :sup:`new` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>fuse_static_with_vfs` :sup:`new`
    fuse the static bodies with their parents
    (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mjs_fuseStatic`).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>discard_visual` :sup:`new` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>discard_visual_with_vfs` :sup:`new`
    discard the elements which are only visual.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>dedup_assets` :sup:`new`
    removes duplicate assets and points their users to the first asset of each group.

*Rendering*

- |mjr_context|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>is_compatible_with_model` :sup:`new`
    reports whether a context can serve another model.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>is_asset_compatible_with_model`
    :sup:`new` reports whether a context can take an asset upload from another model.

*Simulation*

- |mj_data|:

  - A fallible method, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_contact_force` :sup:`new`,
    has been created for handling out-of-bound errors.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model_unchecked` :sup:`new`
    swaps in a model that the compatibility check refuses.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>is_point_inside_site` :sup:`new`
    and :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_is_point_inside_site` :sup:`new`
    report whether a point is inside a site (:docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_insideSite`).
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl_into` :sup:`new`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl_fixed` :sup:`new` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_read_ctrl_fixed` :sup:`new`
    read the controls of an actuator into a buffer or a fixed-size array, without an allocation.

- |mj_model|:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>layout` :sup:`new`
    returns the layout that the compatibility checks compare.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>to_embed_assets` :sup:`new`
    returns a copy of the model that embeds all of its assets.

.. rubric:: New examples

- :gh-example:`Threaded C++ viewer <visualization/viewer/cpp_viewer_threaded.rs>` --- an extended
  example of the original C++ viewer example, which shows multi-threaded use.

.. rubric:: Bug fixes

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsMaterial::<method>set_textures` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTexture::<method>set_cubefiles` now fill
  the remaining needed parts of the vector with empty entries. A shorter length of vectors caused
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
  parallel access to the same data (``context_count`` in MuJoCo C code).
- :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now makes the viewer's OpenGL
  context current again after the user UI callbacks. A callback that made another context current
  caused the viewer to draw into that context.
- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer` and the winit fallback of
  :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` now release their OpenGL context and drop it,
  with its surface, before they close the window. Before, the window closed first, while the
  context still used it.
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
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>id_to_name` now panics for an
  ``id`` above ``i32::MAX``. Before, the ID wrapped, and the method could return the name of another
  element.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
  ``new_tracking``, ``fix`` and ``track`` now panic for an ID above ``i32::MAX`` in all builds.
  Before, the ID wrapped into another camera or body ID.
- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>resize_offscreen` now
  panic for a ``width`` or ``height`` above ``i32::MAX``, and ``add_aux`` clamps ``samples``.
  Before, the sizes wrapped into negative values.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>try_add_flexcomp` now
  returns :docs-rs:`~~mujoco_rs::error::<enum>MjEditError::<variant>InvalidParameter` for a
  ``count`` with more than ``i32::MAX`` points, or for an invalid ``cellcount`` with a trilinear or
  quadratic ``dof``. Before, MuJoCo wrote out of bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>move_` now panics
  for a move or turn action on a camera type other than free, tracking or fixed. Before, MuJoCo
  stopped the process.
- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>push`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>full`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>empty`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>pop_front`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>pop_back` and the
  ``try_`` variants of the last four now panic for a negative ``linepnt``, and ``draw`` panics for a negative
  ``legendoffset`` when ``flg_legend`` is set. Before, ``draw`` read out of bounds.
- |mjv_scene| ``update`` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update_with_catmask`
  now panic for a negative or ``i32::MAX`` ``vis.quality.numslices``. Before, MuJoCo wrote out of
  bounds.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size`, the state
  methods that use it, and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_state_from_data` now panic
  for a ``spec`` with an unknown bit. Before, MuJoCo stopped the process.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>world_body` and
  ``world_body_mut`` now return the world body even after renames. Before, they would panic
  when the world body was renamed.
- :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>max_contacts` and
  ``try_max_contacts`` now return ``0`` for a negative ``sdf_initpoints``. Before, the value
  wrapped due to an overflow.
