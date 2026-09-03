5.0.0 (MuJoCo 3.9.0)
======================

.. rubric:: Breaking changes

*MuJoCo FFI upgraded to 3.9.0*

- MuJoCo-rs now builds against MuJoCo 3.9.0 FFI. This is a breaking
  dependency-level change for consumers tied to older MuJoCo C ABI details.
  See MuJoCo's changelog for this release:
  https://mujoco.readthedocs.io/en/3.9.0/changelog.html

*(Potentially breaking) Changed raw pointer types of trait methods*

- Changed raw pointer types of trait methods. For example, ``element_pointer(&self)``
  now returns ``*const mjsElement``. This is a change that should not affect most users,
  unless they explicitly opted in to using the C model-editing API of MuJoCo directly, or
  implemented/depended on the old trait method signature in advanced extension code.

*MjrContext::upload_texture parameter type changed*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_texture`
  now accepts ``texture_id: usize`` instead of ``texid: u32``.

*MjrContext::upload_texture is now fallible*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_texture`
  now returns ``Result<(), MjrContextError>`` instead of ``()`` and no longer panics.
  Pass an out-of-range ID to receive ``MjrContextError::IndexOutOfBounds { id, len }``
  instead of a panic.

*MjrContext::add_aux / set_aux error type changed*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_aux`
  previously returned ``Result<(), MjSceneError>`` (with ``MjSceneError::InvalidAuxBufferIndex``);
  they now return ``Result<(), MjrContextError>`` (with ``MjrContextError::IndexOutOfBounds``).

*MjSceneError::InvalidAuxBufferIndex / InvalidViewport / BufferTooSmall removed*

- ``MjSceneError::InvalidAuxBufferIndex``, ``MjSceneError::InvalidViewport``, and
  ``MjSceneError::BufferTooSmall`` have been removed from
  :docs-rs:`~mujoco_rs::error::<enum>MjSceneError`. Code matching on these variants will fail
  to compile. Replace them with the corresponding
  :docs-rs:`~mujoco_rs::error::<enum>MjrContextError` variants (see migration guide).

*MjrContext::read_pixels error type changed*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_pixels`
  previously returned ``Result<…, MjSceneError>`` (with ``MjSceneError::InvalidViewport`` and
  ``MjSceneError::BufferTooSmall``); it now returns ``Result<…, MjrContextError>`` (with
  ``MjrContextError::InvalidViewport`` and ``MjrContextError::BufferTooSmall``).

*MjModelError::InvalidIndex removed*

- ``MjModelError::InvalidIndex(usize, usize)`` has been removed from
  :docs-rs:`~mujoco_rs::error::<enum>MjModelError`. Code matching on this variant will fail
  to compile. Replace it with
  :docs-rs:`~mujoco_rs::error::<enum>MjModelError::<variant>IndexOutOfBounds`
  (see migration guide).

*MjsTuple::set_objtype now takes ``&[MjtObj]`` and is fallible*

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTuple::<method>set_objtype` now accepts
  ``&[MjtObj]`` instead of ``&[i32]`` and is a **safe** ``fn`` returning ``Result<(), MjEditError>``
  instead of ``()``. Out-of-range object types are rejected with ``MjEditError::InvalidParameter``.
  See the migration guide.

*MjsSensor::set_objtype / set_reftype are now fallible*

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>set_objtype` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor::<method>set_reftype` now return
  ``Result<(), MjEditError>`` instead of ``()``, rejecting out-of-range object types with
  ``MjEditError::InvalidParameter``. The builder counterparts ``with_objtype`` / ``with_reftype``
  keep their signature but **panic** on a rejected value. See the migration guide.

*MjsNumeric::set_size is now fallible*

- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsNumeric::<method>set_size` now returns
  ``Result<(), MjEditError>`` instead of ``()``, rejecting a negative size with
  ``MjEditError::InvalidParameter`` (a negative size triggers an out-of-bounds write in the model
  compiler). See the migration guide.

*Some index/size vector setters are now ``unsafe``*

- ``MjsFlex::set_elemtexcoord``,
  ``MjsSkin::set_face``, ``MjsMesh::set_userfacetexcoord`` and ``MjsMesh::set_userfacenormal`` are
  now ``unsafe fn``: each writes a
  value the model compiler or renderer later trusts as an unchecked array index, count, or
  ``memcpy`` length, and the correct constraint is cross-field (it cannot be checked from the
  setter alone). Each carries a ``# Safety`` section describing the caller's obligation. See the
  migration guide.

*MjData::history_mut is now an ``unsafe fn``*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>history_mut` is now ``unsafe``.
  The signature is otherwise unchanged; existing calls only need an ``unsafe`` block. The immutable
  ``history`` accessor remains safe. See the migration guide.

*Mjs element handles and MjsDefault are no longer Send/Sync*

- The ``Mjs*`` element handles (|mjs_body|, ``MjsGeom``, ...) and ``MjsDefault`` are now
  ``!Send + !Sync``; they previously carried a blanket ``unsafe impl Send``/``Sync``. Each
  handle is only a raw pointer into one shared ``mjSpec``/``mjCModel`` arena, so moving or
  sharing one across threads was unsound (see Bug fixes). Code that did so will no longer
  compile --- move the owning |mj_spec| instead, which remains ``Send`` (it is no longer
  ``Sync``; see below).

*MjData::add_contact is now an ``unsafe fn``*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>add_contact` is now ``unsafe``.
  It stores the caller-supplied ``MjContact`` verbatim, and MuJoCo later uses it without validation,
  so a malformed contact can cause undefined behavior (see Bug fixes). The signature is otherwise
  unchanged; existing calls only need an ``unsafe`` block.

*MjData::reset_debug is now an ``unsafe fn``*

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>reset_debug` is now ``unsafe``.
  The debug fill writes raw bytes into arrays whose accessors expose types with validity
  invariants (``bool``, fieldless enums), so reading them afterwards can be undefined behavior
  (see Bug fixes). The signature is otherwise unchanged; existing calls only need an ``unsafe``
  block.

*MjData::ray / ray_mesh now take* ``&mut self``

- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray`,
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray_mesh`, and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_ray_mesh` now take
  ``&mut self`` (previously ``&self``). Their bounding-volume traversal mutates ``data.bvh_active``
  (see Bug fixes), so callers holding a shared ``&MjData`` need an exclusive borrow instead.
  ``ray_flex`` / ``ray_hfield`` are unaffected and remain ``&self``; ``multi_ray`` already took
  ``&mut self``.

*MjvScene::find_selection now takes* ``&mut MjData``

- :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>find_selection`
  now takes ``data: &mut MjData<M>`` (previously ``&MjData<M>``). It calls ``mjv_select`` ->
  ``mj_ray``, whose bounding-volume traversal mutates ``data.bvh_active`` (see Bug fixes), so an
  exclusive borrow is required. This mirrors |mjv_scene|'s ``update`` (already ``&mut MjData``) and
  the ``ray`` / ``ray_mesh`` change above.

*MjSpec is no longer* ``Sync``

- |mj_spec| is now ``Send`` but no longer ``Sync`` (see Bug fixes). Several ``&self`` methods --
  notably ``clone`` / ``try_clone`` -- drive C-side writes to the spec, so sharing a ``&MjSpec``
  across threads is unsound. Code that required ``MjSpec: Sync`` (for example placing a ``&MjSpec``
  in a type that demands ``Sync``) will no longer compile; transfer ownership instead, as ``MjSpec``
  remains ``Send``.

*MjsTendon::wrap / wrap_mut no longer return* ``Option``

- |mjs_tendon|'s ``wrap`` and ``wrap_mut`` now return ``&MjsWrap`` / ``&mut MjsWrap`` (previously
  ``Option<&MjsWrap>`` / ``Option<&mut MjsWrap>``) and panic if the index is out of bounds. The old
  signature documented ``None`` on an out-of-range index, but the underlying C ``mjs_getWrap`` aborts
  the process for such an index and never returns null, so the ``None`` arm was unreachable. Drop the
  ``.unwrap()`` on existing calls and ensure the index is in range (``< wrap_num()``), or use the new
  fallible ``try_wrap`` / ``try_wrap_mut``.

.. rubric:: Deprecations

- Deprecated ``SpecItem::delete``
  due to it relying on **undefined behavior**.
  Users are expected to use
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element`
  instead.
- Deprecated |mjs_tendon|'s ``try_wrap_site``, ``try_wrap_geom``, ``try_wrap_joint``, and
  ``try_wrap_pulley`` --- their ``MjEditError::AllocationFailed`` arm is unreachable and an
  allocation failure cannot be recovered soundly (MuJoCo aborts the process, or writes through the
  null pointer before returning). Use the panicking ``wrap_site`` / ``wrap_geom`` / ``wrap_joint`` /
  ``wrap_pulley`` instead. These methods may be undeprecated in the future if MuJoCo's upstream C++
  code is changed to return null recoverably.

.. rubric:: Error handling

New error types:

- :docs-rs:`~mujoco_rs::error::<enum>MjrContextError`: new error enum for
  :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext` operations, with variants:

  - ``IndexOutOfBounds { id, len }`` — an index passed to a rendering-context operation is out of
    range (covers GPU asset uploads and aux buffer index checks).
  - ``InvalidViewport { width, height }`` — the viewport dimensions are invalid for pixel read-back.
  - ``BufferTooSmall { name, got, needed }`` — the caller-supplied buffer is too small to hold the
    read-back data.

New error variants in pre-existing enums:

- :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`: ``SignatureMismatch`` (model structure does
  not match the viewer's passive model), ``IndexOutOfBounds { id, len }`` (asset ID out of range),
  ``ContextError(MjrContextError)`` (wraps a rendering-context error, e.g. from pixel read-back
  during screenshot capture).
- :docs-rs:`~mujoco_rs::renderer::<enum>RendererError`: ``SignatureMismatch`` (model structure does
  not match the renderer's current scene), ``ContextError(MjrContextError)`` (wraps a rendering-context error,
  e.g. an out-of-range asset ID).
- :docs-rs:`~mujoco_rs::error::<enum>MjEditError`: ``InvalidParameter(String)`` (a value was
  rejected; the string describes the rejection). Returned by the new ``set_to_*`` actuator helpers
  (see New features and improvements) and by the now-validating setters
  ``MjsSensor::set_objtype`` / ``set_reftype``, ``MjsTuple::set_objtype`` and
  ``MjsNumeric::set_size`` (see Breaking changes).

.. rubric:: New features and improvements

*Fallible tendon wrap accessors*

- Added :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTendon::<method>try_wrap` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjsTendon::<method>try_wrap_mut`, the fallible
  counterparts of ``wrap`` / ``wrap_mut``. They return
  ``Result<&MjsWrap, MjEditError>`` and yield ``MjEditError::IndexOutOfBounds`` for an out-of-range
  index instead of panicking. A new
  :docs-rs:`~mujoco_rs::error::<enum>MjEditError` ``IndexOutOfBounds`` variant carries the offending
  index and length.

*MjrContext GPU re-upload methods*

- :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_texture`
  now takes ``&self`` instead of ``&mut self``, relaxing the mutability requirement.
- |mjr_context| now provides two additional GPU asset re-upload methods:

  - :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_mesh`
    re-uploads a mesh to the GPU by mesh ID.
  - :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_hfield`
    re-uploads a height field to the GPU by height-field ID.

*Viewer GPU asset re-upload*

The Rust viewer now supports re-uploading textures, meshes, and heightfields to the GPU at
runtime, mirroring the behaviour of MuJoCo's C++ Simulate viewer.

- The singular methods :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_texture_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_mesh_from`, and
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_hfield_from` copy and mark one
  asset at a time. All methods return ``Result<(), MjViewerError>``.
- The plural methods :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_textures_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_meshes_from`, and
  :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>update_hfields_from` copy and mark all
  assets of that type. All methods return ``Result<(), MjViewerError>``.
- For multi-threaded programs the same singular and plural methods are available on the
  shared-state guard:
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_texture_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_mesh_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_hfield_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_textures_from`,
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_meshes_from`, and
  :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>update_hfields_from`.

*Renderer GPU asset re-upload*

The Rust renderer now supports re-uploading textures, meshes, and heightfields to the GPU at
runtime.

- The singular methods :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_texture_from`,
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_mesh_from`, and
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_hfield_from` upload one
  asset immediately. All methods return ``Result<(), RendererError>``.
- The plural methods :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_textures_from`,
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_meshes_from`, and
  :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>update_hfields_from` upload all
  assets of that type immediately. All methods return ``Result<(), RendererError>``.

*Model-editing and utility API additions*

- Added :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_sym2dense` as a wrapper for MuJoCo's
  sparse-to-dense conversion utility. It is an ``unsafe fn``: the C routine indexes ``mat`` /
  ``colind`` / ``res`` from the caller-supplied sparse structure without bounds checks, and the
  only way to validate that structure would be to iterate the index arrays, so the consistency of
  ``rownnz`` / ``rowadr`` / ``colind`` is documented as a ``# Safety`` precondition instead.
- Added model-editing convenience APIs:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>timer`
    for CTIMER values.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>child` and
    :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody::<method>child_mut`
    for named child lookup.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>id`
    for retrieving optional element IDs.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element`
    for deleting a spec element by its FFI pointer --- the sound replacement for the deprecated
    ``SpecItem::delete``.

- Added the missing frame-finding methods to |mj_spec|: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>frame`
  and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>frame_mut`.

- :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsActuator` now exposes the full family of
  actuator-configuration helpers covering MuJoCo's ``mjs_setToX`` C API: ``set_to_motor``,
  ``set_to_position``, ``set_to_int_velocity``, ``set_to_velocity``, ``set_to_damper``,
  ``set_to_cylinder``, ``set_to_muscle``, ``set_to_adhesion``, and ``set_to_dc_motor``. Helpers
  whose C function takes nullable parameters take a dedicated ``Default``-able config struct
  (``PositionConfig``, ``IntVelocityConfig``, ``DcMotorConfig``) in which the nullable parameters
  are ``Option`` fields. Each config can be built either with struct-update syntax or with
  chainable ``with_*`` builder methods (which take the inner value and wrap optionals in ``Some``),
  so callers specify only the fields they need and unset optionals default to ``None``. For example:
  ``actuator.set_to_dc_motor(DcMotorConfig::default().with_motorconst([1.0, 1.0]).with_resistance(1.0))?``.
  Helpers whose parameters are all mandatory keep simple positional arguments (e.g.
  ``set_to_velocity(kv)``, ``set_to_cylinder(timeconst, bias, area, diameter)``,
  ``set_to_muscle(...)``). Helpers that MuJoCo can reject return ``Result<(), MjEditError>`` (e.g.
  ``set_to_damper``, ``set_to_muscle``, ``set_to_dc_motor``); those that cannot fail
  (``set_to_motor``, ``set_to_velocity``, ``set_to_cylinder``) return ``()``.

.. rubric:: New examples

- :gh-example:`Asset re-upload <visualization/viewer/asset_reupload.rs>` --- demonstrates animated heightfield,
  texture, and mesh GPU re-uploads with active physics: a ball falls onto a rippling terrain
  and bounces within a smooth-step wall. Physics and rendering run on separate threads.
- :gh-example:`Renderer asset re-upload <visualization/renderer/renderer_asset_reupload.rs>` --- demonstrates the
  same animated assets using the offscreen renderer: heightfield, texture, and mesh are
  mutated each frame and immediately re-uploaded before rendering to PNG frames.

.. rubric:: Bug fixes

- Fixed |mjs_tendon|'s ``wrap`` / ``wrap_mut``, which documented returning ``None`` for an
  out-of-bounds index but could never do so: the underlying C ``mjs_getWrap`` aborts the process for
  such an index and never returns null, so the ``None`` case was unreachable. The accessors now
  reject an out-of-range index in Rust --- panicking, or returning ``MjEditError::IndexOutOfBounds``
  via the new ``try_wrap`` / ``try_wrap_mut`` (see Breaking changes).
- Fixed a model-editing potential undefined behavior in |mjs_body|, which occurred
  when yielded references of body's items were not destroyed before the next entry
  to the iterator. This was problematic if individual references of yielded items
  were stored and later used --- technically a undefined behavior due to reuse of the ``MjsBody``
  inside the iterator. The ``MjsBody`` parent is now stored as a raw FFI pointer to prevent such
  aliasing.
- Fixed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frame`: a fixed
  camera whose ``fixedcamid`` was outside the model's camera range caused MuJoCo's
  ``mjv_cameraFrame`` to read ``cam_xpos`` / ``cam_xmat`` out of bounds. ``frame`` now validates the
  id against the model and panics on an out-of-range fixed camera id instead.
- Extended that same fixed-camera range check to
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update`: an
  out-of-range ``fixedcamid`` no longer reaches ``mjv_cameraFrame`` (whose ``mjCAMERA_FIXED`` branch
  reads ``cam_xpos`` / ``cam_xmat`` *before* MuJoCo's own range check). This also closes the same
  out-of-bounds read on the offscreen renderer and the viewer, which both update through this method.
- Fixed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update`
  (and ``update_with_catmask``) when an active perturbation carried an out-of-range ``select`` body
  id. MuJoCo's ``mjv_updateScene`` only guards ``select <= 0`` (it has no upper-bound check) before
  indexing ``xpos`` / ``xmat`` / ``body_bvhnum`` by ``select``, so a too-large id read out of bounds.
  ``update`` now validates ``select`` against the model's ``nbody`` and panics on an out-of-range id.
- Fixed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>move_` when the
  public ``select`` field held an out-of-range body id. ``move_`` now validates ``select`` against
  the model's ``nbody`` and panics on an out-of-range id.
- Closed an unsoundness in
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>add_contact`, which was safe but
  copied a caller-built ``MjContact`` into the data arena without validation. MuJoCo later uses the
  stored contact's fields without bound checks (when building constraints and when reading forces),
  so a malformed contact could cause out-of-bounds access from safe code. ``add_contact`` is now an
  ``unsafe fn``: the caller guarantees the contact is valid for the model. See the Breaking changes
  section and the migration guide.
- Closed an unsoundness in
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>reset_debug`, which was safe but
  filled ``mjData`` arrays with raw ``debug_value`` bytes that safe accessors could then read back
  as invalid values for types with validity invariants (e.g. ``bool``, fieldless enums) ---
  undefined behavior for any ``debug_value`` other than 0 or 1. ``reset_debug`` is now an
  ``unsafe fn``: the caller guarantees such accessors are not read before the next reset. See the
  Breaking changes section and the migration guide.
- Fixed a thread-safety hole reachable from safe code in the model-editing handles. Every
  ``Mjs*`` element handle and ``MjsDefault`` carried a blanket ``unsafe impl Send``/``Sync``,
  although each is only a raw pointer into one shared ``mjSpec``/``mjCModel`` arena. Together
  with the lending mutable iterators (which hand out several simultaneously-live ``&mut``
  handles), safe code could move handles to different threads and call mutators such as
  :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>set_name`
  concurrently; ``mjs_setName`` reaches model-global state (``mjCModel::CheckRepeat`` and the
  shared error buffer), so the calls raced on the one arena -- undefined behavior. The handles
  (and ``MjsDefault``) are now ``!Send + !Sync``, so such a mutation can no longer cross a
  thread boundary; the owning |mj_spec| stays ``Send`` (it is now ``!Sync``; see below).
- Closed an unsoundness reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` and
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray_mesh`, which took ``&self``
  although their bounding-volume traversal (``mju_rayTree``) writes ``data.bvh_active`` when the
  model's ``bvactive`` visualization flag is set. The MuJoCo C functions declare ``const mjData*``
  yet mutate through it, so the ``&self`` signature let safe code drive shared-state mutation: with
  ``MjData<M>: Sync`` two threads could race on ``bvh_active``, and a ``&[bool]`` borrowed from the
  safe ``bvh_active`` accessor could be mutated while still live. ``ray``, ``ray_mesh``, and
  ``try_ray_mesh`` now take ``&mut self``. See the Breaking changes section and the migration guide.
- Closed an unsoundness reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>find_selection`, which
  took a shared ``&MjData`` although it calls ``mjv_select`` -> ``mj_ray``, whose bounding-volume
  traversal writes ``data.bvh_active`` (the same ``const mjData*``-but-mutating path as ``ray``
  above). The shared borrow let safe code drive that mutation: with ``MjData<M>: Sync`` two threads
  could race on ``bvh_active``, and a ``&[bool]`` from the safe ``bvh_active`` accessor could be
  mutated while still live. ``find_selection`` now takes ``&mut MjData``. See the Breaking changes
  section and the migration guide.
- Closed an unsoundness reachable from safe code in |mj_spec|, which carried an unconditional
  ``unsafe impl Sync``. ``clone`` / ``try_clone`` produce a faithful, independent copy, but the C++
  copy constructor behind ``mj_copySpec`` is not strictly ``const`` on the source: for each actuator
  it clears two ``std::map`` keyframe-resolution caches (``ForgetKeyframes``). Those maps are empty
  for a normally-built spec, yet ``std::map::clear`` rewrites the tree header unconditionally, so two
  threads sharing a ``&MjSpec`` and cloning concurrently raced on those writes (no spec data is lost).
  |mj_spec| is now ``!Sync`` (it stays ``Send``). See the Breaking changes section and the migration
  guide.
- Closed an out-of-bounds write reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_normalize` when called with an empty slice.
  The wrapper now panics on an empty slice.
- Closed an unsoundness reachable from safe code in ``MjsMesh::set_userfacenormal``, which stored
  unvalidated face-normal indices the renderer later dereferences without a bound check. It is now
  an ``unsafe fn`` with a ``# Safety`` contract. See the Breaking changes section and the migration guide.
- Closed a heap overflow reachable from safe code when compiling an |mj_spec| whose texture has a
  builtin pattern and ``nchannel < 3`` (MuJoCo's builtin generators write more bytes than such a
  buffer holds). :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile` now
  rejects this configuration with ``MjEditError::CompileFailed``.
- Closed an out-of-bounds read reachable from safe code in
  :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>history_mut`, which exposed a
  cursor the sensor/control read path trusts as an array index. It is now an ``unsafe fn``; the
  immutable ``history`` accessor stays safe. See the Breaking changes section and the migration guide.


.. rubric:: Other changes

- Updated bool-typed wrappers to use ``MjtBool`` where MuJoCo uses ``mjtBool``.
  This is not a Rust API breaking change because ``MjtBool`` is a type alias to
  ``mjtBool``, and ``mjtBool`` is ``bool`` by default. Related struct fields and
  function parameters now avoid force-casting and use MuJoCo's bool mapping directly.
  Fields that are ``MjtByte`` in MuJoCo remain byte-backed and are not part of the
  ``MjtBool`` migration.
- Added missing ``mjt*`` type aliases for recently surfaced MuJoCo enums: ``MjtMeshBuiltin``, ``MjtCTimer``.
- Added auxiliary exposed aliases: ``MjPreContact``, ``MjfCollision``, ``MjpResourceProvider``.
- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<tymethod>element_pointer`
  and :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>element_mut_pointer`
  are no longer marked ``unsafe``.
- :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_state`
  is no longer marked ``unsafe``. MuJoCo 3.9.0 changed ``eq_active`` to ``mjtBool``
  (C ``bool``), so writes through ``mjSTATE_EQ_ACTIVE`` are booleanized on assignment
  and reading ``eq_active()`` afterwards can no longer cause undefined behavior.
  Existing ``unsafe { data.set_state(...) }`` call sites keep compiling and only
  produce an ``unused_unsafe`` warning.


