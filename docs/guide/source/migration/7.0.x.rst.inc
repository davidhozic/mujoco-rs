.. _migrate_7_0_0:

Migrating to 7.0.0
==================


Tendon tree count is unsafe to mutate
-------------------------------------

``treenum`` is now marked as unsafe to mutate (wrong values can cause out-of-bound reads). Consequently,

- the ``treenum`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTendonModelViewMut` is
  now :docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`;
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


Plugin ids in model views are unsafe to mutate
----------------------------------------------

The ``plugin`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut` and
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjBodyModelViewMut` is now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.actuator("a").unwrap().view_mut(&mut model).plugin[0] = -1;

    // After
    let mut view = model.actuator("a").unwrap().view_mut(&mut model);
    // SAFETY: actuator "a" is not bound to a plugin instance.
    unsafe { view.plugin.as_mut_slice()[0] = -1; }


Equality data, flex stiffness and flex bending are unsafe to mutate
-------------------------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>eq_data_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_stiffness_mut` and
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_bending_mut` are now ``unsafe fn``.
The ``data`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjEqualityModelViewMut` is now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.eq_data_mut()[0][0] = 0.5;
    model.equality("eq").unwrap().view_mut(&mut model).data[0] = 0.5;

    // After
    // SAFETY: equality 0 is not a flex strain equality.
    unsafe { model.eq_data_mut()[0][0] = 0.5; }
    let mut view = model.equality("eq").unwrap().view_mut(&mut model);
    // SAFETY: equality "eq" is not a flex strain equality.
    unsafe { view.data.as_mut_slice()[0] = 0.5; }


Actuator bias type and parameters, body simple flag and flex flags are unsafe to mutate
---------------------------------------------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_biastype_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_dynprm_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>actuator_gainprm_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>body_simple_mut`,
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_rigid_mut` and
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>flex_centered_mut` are now
``unsafe fn``. The ``biastype``, ``dynprm`` and ``gainprm`` fields of
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut` and the ``simple`` field of
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjBodyModelViewMut` are now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.actuator_biastype_mut()[0] = MjtBias::mjBIAS_NONE;
    model.body_simple_mut()[1] = 0;
    let mut view = model.actuator("a").unwrap().view_mut(&mut model);
    view.gainprm[0] = 2.0;

    // After
    // SAFETY: mjBIAS_NONE reads no actuator state, and 0 selects the general mass matrix path.
    unsafe {
        model.actuator_biastype_mut()[0] = MjtBias::mjBIAS_NONE;
        model.body_simple_mut()[1] = 0;
    }
    let mut view = model.actuator("a").unwrap().view_mut(&mut model);
    // SAFETY: actuator "a" is not a PID or a DC motor actuator, so gainprm does not set its number
    // of activation slots.
    unsafe {
        view.gainprm.as_mut_slice()[0] = 2.0;
    }


Tuple object types are unsafe to mutate
---------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tuple_objtype_mut` is now an ``unsafe fn``.
The ``objtype`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjTupleModelViewMut` is now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.tuple_objtype_mut()[0] = MjtObj::mjOBJ_SITE;
    model.tuple("t").unwrap().view_mut(&mut model).objtype[0] = MjtObj::mjOBJ_SITE;

    // After
    // SAFETY: tuple_objid[0] is a valid site id.
    unsafe { model.tuple_objtype_mut()[0] = MjtObjBase::mjOBJ_SITE; }
    let mut view = model.tuple("t").unwrap().view_mut(&mut model);
    // SAFETY: the first object id of tuple "t" is a valid site id.
    unsafe { view.objtype.as_mut_slice()[0] = MjtObjBase::mjOBJ_SITE; }


Height field sizes are unsafe to mutate
---------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>hfield_size_mut` is now an ``unsafe fn``.
The ``size`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjHfieldModelViewMut` is now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.hfield_size_mut()[0][0] = 2.0;
    model.hfield("h").unwrap().view_mut(&mut model).size[0] = 2.0;

    // After
    // SAFETY: every size is positive and finite.
    unsafe { model.hfield_size_mut()[0][0] = 2.0; }
    let mut view = model.hfield("h").unwrap().view_mut(&mut model);
    // SAFETY: the size is positive and finite.
    unsafe { view.size.as_mut_slice()[0] = 2.0; }


Site types and asset paths are unsafe to mutate
-----------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>site_type_mut` and
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>paths_mut` are now ``unsafe fn``.
The ``r#type`` field of :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjSiteModelViewMut` is now
:docs-rs:`~mujoco_rs::util::<struct>PointerViewUnsafeMut`.

.. code-block:: rust

    // Before
    model.site_type_mut()[0] = MjtGeom::mjGEOM_BOX;
    model.site("s").unwrap().view_mut(&mut model).r#type[0] = MjtGeom::mjGEOM_BOX;

    // After
    // SAFETY: a box site needs no mesh data.
    unsafe { model.site_type_mut()[0] = MjtGeomBase::mjGEOM_BOX; }
    let mut view = model.site("s").unwrap().view_mut(&mut model);
    // SAFETY: a box site needs no mesh data.
    unsafe { view.r#type.as_mut_slice()[0] = MjtGeomBase::mjGEOM_BOX; }


``zero`` of a mutable view skips the unsafe fields
--------------------------------------------------

The ``zero`` method of a mutable view (e.g.
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjActuatorModelViewMut::<method>zero`) no longer
zeroes the fields that are now ``PointerViewUnsafeMut`` (above). Write these fields through the
unsafe view if you need them zeroed.

.. code-block:: rust

    // Before
    let mut view = model.actuator("a").unwrap().view_mut(&mut model);
    view.zero();

    // After
    let mut view = model.actuator("a").unwrap().view_mut(&mut model);
    view.zero();
    // SAFETY: zero gain parameters are valid for the actuator type of "a".
    unsafe { view.gainprm.as_mut_slice().fill(0.0); }


More constraint arrays of ``MjData`` are unsafe to read
-------------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_diag_a`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_kbip`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_d`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_r`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_d`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>iefc_r`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_y`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efc_ar`,
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efm_c` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>efm_l` are now
``unsafe fn``, and so are their ``_mut`` variants. MuJoCo does not zero these arrays, so read them
only after the pipeline stage
that computes them. ``efc_y`` and ``efc_ar`` stay uninitialized under some solver settings.

.. code-block:: rust

    // Before
    data.step1();
    let impedance = data.efc_kbip()[0][2];

    // After
    data.forward();
    // SAFETY: forward built the constraints, so efc_kbip holds computed values.
    let impedance = unsafe { data.efc_kbip() }[0][2];


Viewer, renderer and scene constructors take ``&MjModel``
---------------------------------------------------------

:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>launch_passive`,
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewerBuilder::<method>build_passive`,
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>new`,
:docs-rs:`~~mujoco_rs::renderer::<struct>MjRendererBuilder::<method>build` and
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>new` take the model as
``&MjModel`` (|mj_model|) instead of a generic model type.

.. code-block:: rust

    // Before
    let model = Arc::new(MjModel::from_xml_string(XML).unwrap());
    let viewer = MjViewer::launch_passive(model.clone(), 0)?;
    let renderer = MjRenderer::builder().build(model.clone())?;
    let scene = MjvScene::new(model.clone(), 100);

    // After
    let model = Arc::new(MjModel::from_xml_string(XML).unwrap());
    let viewer = MjViewer::launch_passive(&model, 0)?;
    let renderer = MjRenderer::builder().build(&model)?;
    let scene = MjvScene::new(&model, 100);


The C++ viewer owns the simulation data
---------------------------------------

:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>launch_passive` takes the
|mj_data| by value and no longer takes a ``model`` parameter. The model type of the data must
implement :docs-rs:`~mujoco_rs::wrappers::mj_model::traits::<trait>ModelTypeMut`. Step and sync
the data through :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>state`.
:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` is no longer ``unsafe``.
Replace ``user_scn_mut`` with
:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjCppViewerSharedState::<method>user_geoms_mut` and
:docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>sync_user_geoms`.
The former method is unsafe due to the ability of users to write invalid IDs into
:docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom`.

.. code-block:: rust

    // Before
    let model = MjModel::from_xml_string(XML).unwrap();
    let mut data = MjData::new(&model);
    let mut viewer = unsafe { MjViewerCpp::launch_passive(&model, &data, 100) };
    while viewer.running() {
        viewer.sync();
        unsafe { viewer.render().unwrap() };
        data.step();
    }

    // After
    use mujoco_rs::util::LockUnpoison;

    let model = Box::new(MjModel::from_xml_string(XML).unwrap());
    let mut viewer = unsafe { MjViewerCpp::launch_passive(MjData::new(model), 100) };
    let state = viewer.state().clone();
    while viewer.running() {
        {
            let mut lock = state.lock_unpoison();
            // SAFETY: the data is only stepped, never replaced.
            unsafe { lock.data_mut() }.step();
            lock.sync();
        }
        viewer.render().unwrap();
    }


``MjData::read_ctrl`` returns a slice
-------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_read_ctrl` return a
``Cow<[MjtNum]>`` (:docs-rs:`~mujoco_rs::wrappers::mj_primitive::<type>MjtNum`) with one value per
control of the actuator. To avoid heap allocation, use
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl_fixed` or
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl_into`.

.. code-block:: rust

    // Before
    let value: MjtNum = data.read_ctrl(0, time, 1);

    // After
    let value: MjtNum = data.read_ctrl(0, time, 1)[0];
    // or
    let [value] = data.read_ctrl_fixed::<1>(0, time, 1);


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
    material.set_texture(MjtTextureRoleBase::mjTEXROLE_RGB, "wood");


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
the latter holding
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecElement::<tymethod>element_pointer`,
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecElement::<method>element_mut_pointer` and
:docs-rs:`~~mujoco_rs::wrappers::mj_editing::<trait>SpecElement::<method>name`.

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


Default-class members have field accessors only
-----------------------------------------------

The member accessors of :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsDefault` now return
handles marked with :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>InDefault`, for example
``&mut MjsJoint<InDefault>`` (|mjs_joint|). These handles have no methods of
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecElement` or of the traits built on it
(``SpecItem``, :docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>SpecObject`, ``FrameChild``,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>Attach`,
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<trait>UserValued`).

The marker is needed to hide all the (trait) methods that aren't actually supported
and would, prior to the change, cause a segmentation fault.

.. code-block:: rust

    // Before
    let joint: &mut MjsJoint = spec.add_default("cls", None).joint_mut();
    joint.set_armature(0.1);

    // After
    let joint: &mut MjsJoint<InDefault> = spec.add_default("cls", None).joint_mut();
    joint.set_armature(0.1);


``MjData::contact_force`` panics on an invalid ID (runtime-breaking)
---------------------------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>contact_force` now panics when
``contact_id >= ncon``. Use
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_contact_force` to handle an
invalid ID.

.. code-block:: rust

    // Before
    let force = data.contact_force(contact_id); // [0.0; 6] when contact_id >= ncon

    // After
    let force = data.try_contact_force(contact_id).unwrap_or([0.0; 6]);


``MjvScene`` refuses more models (runtime-breaking)
---------------------------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>is_compatible_with_model`
now refuses more models, and
:docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>update`
panics on such a model. If you reuse one scene
for several models, create a new scene when the check fails.

.. code-block:: rust

    // Before
    scene.update(&mut data, &opt, &pert, &mut camera);

    // After
    if !scene.is_compatible_with_model(data.model()) {
        scene = MjvScene::new(data.model(), max_geom);
    }
    scene.update(&mut data, &opt, &pert, &mut camera);


Incompatible-model errors now carry layouts
-------------------------------------------------------

:docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>IncompatibleModel` and
:docs-rs:`~~mujoco_rs::error::<enum>MjModelError::<variant>IncompatibleModel` now hold the
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModelLayout` of both models. ``MjvScene::signature``
and ``model_signature`` of the ``Info`` types are removed. Read the signature from the model.

.. code-block:: rust

    // Before
    if let Err(MjDataError::IncompatibleModel { source, destination }) = data.try_swap_model(other) {
        println!("{source:#X} vs {destination:#X}");
    }
    let signature = scene.signature();
    let signature = body_info.model_signature();

    // After
    if let Err(MjDataError::IncompatibleModel { source, destination }) = data.try_swap_model(other) {
        println!("{source:?} vs {destination:?}");
    }
    let signature = model.signature();


Unsupported-object-type errors now carry the object type
--------------------------------------------------------

:docs-rs:`~~mujoco_rs::error::<enum>MjDataError::<variant>UnsupportedObjectType` now holds an
:docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtObjBase` instead of an ``i32``.

.. code-block:: rust

    // Before
    if let Err(MjDataError::UnsupportedObjectType(raw)) = data.try_object_velocity(MjtObj::mjOBJ_JOINT, 0, false) {
        println!("object type {raw}");
    }

    // After
    if let Err(MjDataError::UnsupportedObjectType(obj_type)) =
        data.try_object_velocity(MjtObjBase::mjOBJ_JOINT, 0, false)
    {
        println!("object type {obj_type:?}");
    }


A model swap refuses more models
--------------------------------

:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model` and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_swap_model`
now refuse a model that differs in a size or in an integer table that no safe accessor writes,
apart from the names, the paths, the buffer size, the mesh faces and hulls, the bounding volume
and octree nodes and the values derived from floats.
If you know that the new model is still safe for the data, use
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>swap_model_unchecked`.
Otherwise, create a new |mj_data|.

.. code-block:: rust

    // Before
    let old = data.swap_model(edited);

    // After
    // SAFETY: `edited` only changes a geom material, so every size and every address table agrees.
    let old = unsafe { data.swap_model_unchecked(edited) };


Real geom types, object types and texture roles use base enums
--------------------------------------------------------------

Model and specification fields and functions that accept only real geom types, real object
types or real texture roles now use :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtGeomBase`,
:docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtObjBase` and
:docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtTextureRoleBase`, respectively.
The object type setters of :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsSensor` and
:docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsTuple` are no longer fallible.

.. code-block:: rust

    // Before
    spec.world_body_mut().add_geom().with_type(MjtGeom::mjGEOM_PLANE);
    sensor.set_objtype(MjtObj::mjOBJ_SITE)?;
    tuple.set_objtype(&[MjtObj::mjOBJ_BODY, MjtObj::mjOBJ_GEOM])?;
    material.set_texture(MjtTextureRole::mjTEXROLE_RGB, "wood");
    let id = model.name_to_id(MjtObj::mjOBJ_BODY, "ball");
    let name = model.id_to_name(model.sensor_objtype()[0], 0);

    // After
    spec.world_body_mut().add_geom().with_type(MjtGeomBase::mjGEOM_PLANE);
    sensor.set_objtype(MjtObjBase::mjOBJ_SITE);
    tuple.set_objtype(&[MjtObjBase::mjOBJ_BODY, MjtObjBase::mjOBJ_GEOM]);
    material.set_texture(MjtTextureRoleBase::mjTEXROLE_RGB, "wood");
    let id = model.name_to_id(MjtObjBase::mjOBJ_BODY, "ball");
    let name = model.id_to_name(model.sensor_objtype()[0], 0);
