.. _migrate_5_0_0:

Migrating to 5.0.0
======================

Version 5.0.0 updates MuJoCo to 3.9.0, redesigns model-editing element deletion,
migrates several rendering APIs to the |mjr_context| error type, and hardens
model-editing setters against out-of-range object types.


MuJoCo upgrade
-----------------------

MuJoCo-rs 5.0.0 links against MuJoCo **3.9.0**. Download the matching release
and update your library path. See :ref:`installation` for details.


``SpecItem::element_pointer`` pointer type changed (potentially breaking)
--------------------------------------------------------------------------
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<tymethod>element_pointer`
now returns ``*const mjsElement`` (was ``*mut mjsElement``) and is no longer ``unsafe``. This only
affects code that called the model-editing trait methods directly (e.g. to drive MuJoCo's C
model-editing API). When a mutable pointer is required, use
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>element_mut_pointer`
(also no longer ``unsafe``). Any ``unsafe`` block wrapping these calls is now redundant and can be
removed.

**Before (4.x):**

.. code-block:: rust

    let ptr: *mut mjsElement = unsafe { item.element_pointer() };

**After (5.0.0):**

.. code-block:: rust

    // read-only C calls take an immutable pointer:
    let ptr: *const mjsElement = item.element_pointer();
    // when a mutable pointer is required:
    let ptr: *mut mjsElement = item.element_mut_pointer();


Deprecated implementation of removing model-editing elements
--------------------------------------------------------------
Due to the :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecItem::<method>delete`
method relying on undefined behavior, the said method is now deprecated.
It's replacement --- :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>delete_element` ---
now forces the user to drop all existing references of |mj_spec| and its
subsequent elements (geoms, joints, etc.), providing some inconvenience
compared to the previous implementation.

**Before (4.x)**:

.. code-block:: rust

  let mut spec = MjSpec::new();
  let body = spec.world_body_mut().add_body();  // &mut MjsBody
  unsafe { body.delete().unwrap() } ;

**After (5.0.0)**:

.. code-block:: rust

  let mut spec = MjSpec::new();
  let body = spec.world_body_mut().add_body();  // &mut MjsBody
  let body_ptr = body.element_mut_pointer();
  unsafe { spec.delete_element(body_ptr).unwrap() }


``MjsTendon::wrap`` / ``wrap_mut`` no longer return ``Option``
--------------------------------------------------------------
|mjs_tendon|'s ``wrap`` and ``wrap_mut`` now return ``&MjsWrap`` / ``&mut MjsWrap`` (previously
``Option<&MjsWrap>`` / ``Option<&mut MjsWrap>``) and panic if the index is out of bounds. The old
signature documented ``None`` on an out-of-range index, but the underlying C ``mjs_getWrap`` aborts
the process for such an index and never returns null, so the ``None`` arm was unreachable. Drop the
``.unwrap()`` on existing calls and ensure the index is in range (``< wrap_num()``), or use the new
fallible ``try_wrap`` / ``try_wrap_mut``.

**Before (4.x)**:

.. code-block:: rust

  let wrap = tendon.wrap(i).unwrap();

**After (5.0.0)**:

.. code-block:: rust

  let wrap = tendon.wrap(i);


Deprecated fallible tendon-wrap methods
---------------------------------------
|mjs_tendon|'s ``try_wrap_site``, ``try_wrap_geom``, ``try_wrap_joint``, and ``try_wrap_pulley``
are now deprecated. Their ``MjEditError::AllocationFailed`` arm is unreachable: on an allocation
failure MuJoCo aborts the process (or, under a non-default error configuration, writes through the
null pointer before returning), so the failure cannot be recovered soundly. Switch to the panicking
``wrap_site`` / ``wrap_geom`` / ``wrap_joint`` / ``wrap_pulley``, which return ``&mut MjsWrap``
directly.

These methods may be undeprecated in the future if MuJoCo's upstream C++ code is changed to return
null recoverably.

**Before (4.x)**:

.. code-block:: rust

  let wrap = tendon.try_wrap_geom("geom", "sidesite")?;

**After (5.0.0)**:

.. code-block:: rust

  let wrap = tendon.wrap_geom("geom", "sidesite");


``MjrContext::upload_texture`` parameter type changed
-----------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_texture`
now accepts ``texture_id: usize`` instead of ``texid: u32``. Update call sites to cast or
convert the argument accordingly.

**Before (4.x):**

.. code-block:: rust

    context.upload_texture(&model, 0u32);

**After:**

.. code-block:: rust

    context.upload_texture(&model, 0usize);


``MjrContext::upload_texture`` returns ``Result``
-------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>upload_texture`
now returns ``Result<(), MjrContextError>`` instead of ``()``.
Handle or propagate the result at each call site.

**Before (4.x):**

.. code-block:: rust

    context.upload_texture(&model, id);

**After:**

.. code-block:: rust

    context.upload_texture(&model, id)?;


``MjrContext::add_aux`` / ``set_aux`` error type changed
---------------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux` and
:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_aux`
now return ``Result<(), MjrContextError>`` instead of ``Result<(), MjSceneError>``.
The ``MjSceneError::InvalidAuxBufferIndex`` variant has been removed; use
``MjrContextError::IndexOutOfBounds`` instead.

**Before (4.x):**

.. code-block:: rust

    match context.add_aux(index, width, height, samples) {
        Err(MjSceneError::InvalidAuxBufferIndex { index }) => { /* … */ }
        Ok(()) => { /* … */ }
    }

**After:**

.. code-block:: rust

    match context.add_aux(index, width, height, samples) {
        Err(MjrContextError::IndexOutOfBounds { id, len }) => { /* … */ }
        Ok(()) => { /* … */ }
    }


``MjrContext::read_pixels`` error type changed
-----------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_pixels`
now returns ``Result<…, MjrContextError>`` instead of ``Result<…, MjSceneError>``.
``MjSceneError::InvalidViewport`` and ``MjSceneError::BufferTooSmall`` have been removed;
use ``MjrContextError::InvalidViewport`` and ``MjrContextError::BufferTooSmall`` instead.

**Before (4.x):**

.. code-block:: rust

    match context.read_pixels(Some(&mut rgb_buf), None, &viewport) {
        Err(MjSceneError::InvalidViewport { .. }) => { /* … */ }
        Err(MjSceneError::BufferTooSmall { .. }) => { /* … */ }
        Ok(()) => { /* … */ }
    }

**After:**

.. code-block:: rust

    match context.read_pixels(Some(&mut rgb_buf), None, &viewport) {
        Err(MjrContextError::InvalidViewport { .. }) => { /* … */ }
        Err(MjrContextError::BufferTooSmall { .. }) => { /* … */ }
        Ok(()) => { /* … */ }
    }


``MjModelError::InvalidIndex`` removed
---------------------------------------

``MjModelError::InvalidIndex(usize, usize)`` has been removed. Use
:docs-rs:`~mujoco_rs::error::<enum>MjModelError::<variant>IndexOutOfBounds`
with named fields instead.

**Before (4.x):**

.. code-block:: rust

    match model.try_max_contacts(geom1, geom2, None) {
        Err(MjModelError::InvalidIndex(id, len)) => { /* … */ }
        Ok(count) => { /* … */ }
    }

**After:**

.. code-block:: rust

    match model.try_max_contacts(geom1, geom2, None) {
        Err(MjModelError::IndexOutOfBounds { id, len }) => { /* … */ }
        Ok(count) => { /* … */ }
    }


``MjsTuple::set_objtype`` now takes ``&[MjtObj]`` and is fallible
-----------------------------------------------------------------

``MjsTuple::set_objtype`` previously accepted ``&[i32]`` and returned ``()``. It now takes
``&[MjtObj]`` and returns ``Result<(), MjEditError>``: out-of-range object types are rejected with
``MjEditError::InvalidParameter``.

**Before (4.x):**

.. code-block:: rust

    tuple.set_objtype(&[MjtObj::mjOBJ_BODY as i32, MjtObj::mjOBJ_GEOM as i32]);

**After:**

.. code-block:: rust

    tuple.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_GEOM])?;


``MjsSensor::set_objtype`` / ``set_reftype`` are now fallible
-------------------------------------------------------------

``MjsSensor::set_objtype`` and ``set_reftype`` previously returned ``()``; they now return
``Result<(), MjEditError>``, rejecting out-of-range object types with ``MjEditError::InvalidParameter``.
The builder counterparts ``with_objtype`` / ``with_reftype`` keep their signature but **panic** on a
rejected value.

**Before (4.x):**

.. code-block:: rust

    sensor.set_objtype(MjtObj::mjOBJ_SITE);
    sensor.set_reftype(MjtObj::mjOBJ_BODY);

**After:**

.. code-block:: rust

    sensor.set_objtype(MjtObj::mjOBJ_SITE)?;
    sensor.set_reftype(MjtObj::mjOBJ_BODY)?;


MjsNumeric::set_size is now fallible
------------------------------------

``MjsNumeric::set_size`` previously returned ``()``; it now returns ``Result<(), MjEditError>``,
rejecting a negative size with ``MjEditError::InvalidParameter`` (a negative size triggers an
out-of-bounds write in the model compiler).

**Before (4.x):**

.. code-block:: rust

    numeric.set_size(8);

**After:**

.. code-block:: rust

    numeric.set_size(8)?;


Some index/size vector setters are now ``unsafe``
-------------------------------------------------

``MjsFlex::set_elemtexcoord``,
``MjsSkin::set_face``, ``MjsMesh::set_userfacetexcoord`` and ``MjsMesh::set_userfacenormal`` are now
``unsafe fn``. Each writes a
value the model compiler or renderer later trusts as an unchecked index/count/length, and the
correct constraint is cross-field, so it cannot be validated from the setter. Wrap calls in
``unsafe`` after ensuring the obligation in each method's ``# Safety`` section (e.g. for
``set_face``, every index is in ``0..nvert`` and the length is a multiple of 3; for
``set_userfacenormal``, every index is in ``0..N``, where ``N`` is the ``set_usernormal`` slice
length divided by 3).

**Before (4.x):**

.. code-block:: rust

    skin.set_face(&faces);

**After:**

.. code-block:: rust

    // SAFETY: every index is < nvert and faces.len() is a multiple of 3.
    unsafe { skin.set_face(&faces); }


``MjData::add_contact`` is now an ``unsafe fn``
-----------------------------------------------

``add_contact`` wraps ``mj_addContact``, an advanced entry point that stores the caller-supplied
``MjContact`` verbatim. MuJoCo later uses the stored contact without validation, so a malformed
contact can cause undefined behavior. The method is now ``unsafe``; the caller must ensure the
contact is valid for the model. The signature is otherwise unchanged, so existing call sites only
need an ``unsafe`` block.

**Before (4.x):**

.. code-block:: rust

    data.add_contact(&contact)?;

**After:**

.. code-block:: rust

    // SAFETY: `contact` is a valid contact for this model.
    unsafe { data.add_contact(&contact)? };


``MjData::reset_debug`` is now an ``unsafe fn``
-----------------------------------------------

``reset_debug`` wraps ``mj_resetDataDebug``, which fills every buffer-resident array with raw
``debug_value`` bytes. Arrays that MuJoCo does not re-initialize afterwards keep those bytes, and
some safe accessors expose them as types with validity invariants (``bvh_active`` as ``&[bool]``,
``body_awake`` as a fieldless enum slice), so reading them can be undefined behavior. The method
is now ``unsafe``; the caller must not read such accessors before the next reset unless
``debug_value`` produces valid bit patterns for them. The signature is otherwise unchanged, so
existing call sites only need an ``unsafe`` block.

**Before (4.x):**

.. code-block:: rust

    data.reset_debug(7);

**After:**

.. code-block:: rust

    // SAFETY: no invariant-carrying accessor (bvh_active, body_awake) is read
    // before the next reset().
    unsafe { data.reset_debug(7) };


``MjData::history_mut`` is now an ``unsafe fn``
-----------------------------------------------

``history_mut`` exposes the whole history buffer for mutation. Each slot stores a cursor in
``buf[1]`` that ``mj_readSensor`` / ``mj_readCtrl`` trust as an array index without a bound check,
so corrupting it from safe code can cause an out-of-bounds read. The method is now ``unsafe``; the
caller must keep every slot's cursor valid. The signature is otherwise unchanged, so existing call
sites only need an ``unsafe`` block. The immutable ``history`` accessor stays safe.

**Before (4.x):**

.. code-block:: rust

    let buf = data.history_mut();

**After:**

.. code-block:: rust

    // SAFETY: every history slot's cursor (buf[1]) is kept valid.
    let buf = unsafe { data.history_mut() };


``MjData::ray`` / ``ray_mesh`` now take ``&mut self``
-----------------------------------------------------

``ray``, ``ray_mesh``, and ``try_ray_mesh`` now take ``&mut self`` (previously ``&self``). Their
bounding-volume traversal (``mju_rayTree``) writes ``data.bvh_active`` when the model's ``bvactive``
visualization flag is set, even though the underlying MuJoCo C functions are declared
``const mjData*``. The ``&self`` signature therefore let safe code drive shared ``MjData`` mutation
(a data race under ``MjData<M>: Sync``, or mutation of a live ``&[bool]`` obtained from the
``bvh_active`` accessor). Callers that held a shared ``&MjData`` now need an exclusive borrow.
``ray_flex`` and ``ray_hfield`` are unaffected; ``multi_ray`` already took ``&mut self``.

**Before (4.x):**

.. code-block:: rust

    let data = model.make_data();
    let (geomid, dist) = data.ray(&pnt, &vec, None, false, None, None);

**After:**

.. code-block:: rust

    let mut data = model.make_data();
    let (geomid, dist) = data.ray(&pnt, &vec, None, false, None, None);


``MjvScene::find_selection`` now takes ``&mut MjData``
------------------------------------------------------

|mjv_scene|'s ``find_selection`` now takes ``data: &mut MjData<M>`` (previously ``&MjData<M>``).
It calls ``mjv_select`` -> ``mj_ray``, whose bounding-volume traversal writes ``data.bvh_active``
(the same ``const mjData*``-but-mutating path as ``ray`` above), so a shared borrow was unsound.
Pass an exclusive borrow instead. (``MjvScene::update`` already took ``&mut MjData``.)

**Before (4.x):**

.. code-block:: rust

    let data = model.make_data();
    let sel = scene.find_selection(&data, &opt, aspect, relx, rely);

**After:**

.. code-block:: rust

    let mut data = model.make_data();
    let sel = scene.find_selection(&mut data, &opt, aspect, relx, rely);


``MjSpec`` is no longer ``Sync``
--------------------------------

|mj_spec| is now ``Send`` but no longer ``Sync``. ``clone`` / ``try_clone`` make a faithful,
independent copy, but the underlying C++ copy constructor is not strictly ``const`` on the source
(it rewrites transient, normally-empty per-actuator keyframe-cache maps), so two threads cloning a
shared ``&MjSpec`` concurrently is a data race. ``MjSpec`` still moves between threads (``Send``);
only shared cross-thread access is removed. Code that placed a ``&MjSpec`` in a ``Sync``-requiring
context must transfer ownership (or clone per thread) instead.

**Before (4.x):**

.. code-block:: rust

    // relied on &MjSpec being shareable across threads
    std::thread::scope(|s| {
        s.spawn(|| { let _ = spec.clone(); });
        s.spawn(|| { let _ = spec.clone(); });
    });

**After:**

.. code-block:: rust

    // move an owned MjSpec into each thread (clone up front if needed)
    let spec2 = spec.clone();
    std::thread::scope(|s| {
        s.spawn(move || { let _ = spec; });
        s.spawn(move || { let _ = spec2; });
    });


``Mjs*`` element handles are no longer ``Send`` / ``Sync``
------------------------------------------------------------

The ``Mjs*`` element handles (``MjsBody``, ``MjsGeom``, ...) and ``MjsDefault`` previously carried a
blanket ``unsafe impl Send``/``Sync``. Each is only a raw pointer into one shared ``mjSpec`` /
``mjCModel`` arena, so moving or sharing a handle across threads was unsound. They are now
``!Send + !Sync``. Code that sent a handle to another thread will no longer compile --- move the
owning |mj_spec| instead (it stays ``Send``; it is now ``!Sync``, see above) and derive the
per-thread handles from it on the thread that uses them.

**Before (4.x):**

.. code-block:: rust

    // relied on Mjs* handles being Send: a handle was moved into another thread
    let geom = body.add_geom();
    std::thread::scope(|s| {
        s.spawn(move || { let _ = geom.set_name("g"); });
    });

**After:**

.. code-block:: rust

    // move the owning MjSpec; derive handles on the thread that uses them
    std::thread::scope(|s| {
        s.spawn(move || {
            let mut spec = spec;
            let geom = spec.world_body_mut().add_geom();
            let _ = geom.set_name("g");
        });
    });


