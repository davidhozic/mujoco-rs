.. _migrate_4_0_0:

Migrating to 4.0.0
======================

Version 4.0.0 updates MuJoCo to 3.8.0 and follows the upstream API changes in
``mj_geomDistance`` and the model-editing specification structs.


MuJoCo upgrade
-----------------------

MuJoCo-rs 4.0.0 links against MuJoCo **3.8.0**. Download the matching release
and update your library path. See :ref:`installation` for details.


``MjData::geom_distance()`` now requires ``&mut self``
------------------------------------------------------

MuJoCo 3.7.0 changed ``mj_geomDistance`` to mutate ``mjData`` internally. As a
result, both
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>geom_distance`
and
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_geom_distance`
now require mutable access to |mj_data|.

**Before (3.x):**

.. code-block:: rust

    let data = MjData::new(&model);
    let dist = data.geom_distance(geom1, geom2, dist_max, None);

**After (4.0.0):**

.. code-block:: rust

    let mut data = MjData::new(&model);
    let dist = data.geom_distance(geom1, geom2, dist_max, None);


``MjsJoint`` and ``MjsTendon`` stiffness/damping now use coefficient arrays
----------------------------------------------------------------------------

MuJoCo 3.7.0 replaced the scalar ``stiffness`` and ``damping`` fields in the
editing API with polynomial coefficients. MuJoCo-rs mirrors that change:
|mjs_joint| and |mjs_tendon| now use coefficient arrays for ``stiffness`` and
``damping``. Update call sites to pass coefficient arrays instead of scalar
values.

**Before (3.x):**

.. code-block:: rust

    joint.with_stiffness(10.0)
         .with_damping(0.5);

    tendon.with_stiffness(20.0)
          .with_damping(0.2);

**After (4.0.0):**

.. code-block:: rust

    joint.with_stiffness([10.0, 0.0, 0.0])
         .with_damping([0.5, 0.0, 0.0]);

    tendon.with_stiffness([20.0, 0.0, 0.0])
          .with_damping([0.2, 0.0, 0.0]);


``MjsFlex::vertcollide`` was removed
------------------------------------

MuJoCo 3.7.0 removed the upstream ``mjsFlex::vertcollide`` field. The
corresponding ``MjsFlex::vertcollide`` getter/setter methods are therefore no
longer available in MuJoCo-rs 4.0.0.

**Before (3.x):**

.. code-block:: rust

    flex.set_vertcollide(true);

**After (4.0.0):**

.. code-block:: rust

    // Remove vertcollide access -- the upstream field no longer exists.
    let _ = flex;


``MjData::model_mut`` is now unsafe
------------------------------------
To prevent a false sense of correctness, method
:docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>model_mut`
is now marked unsafe to prevent users from direct swapping models to incompatible
ones.

**Before (3.x):**

.. code-block:: rust

    data.model_mut().opt_mut().gravity[2] = half_gravity;
    *data.model_mut() = new_model;


**After (4.0.0):**

.. code-block:: rust

    // For field modification (safe accessors):
    data.model_opt_mut().gravity[2] = half_gravity;

    // For full model replacement:
    let mut old_model = data.swap_model(new_model);


``MjViewer::add_ui_callback`` closure now receives ``Box<MjModel>``
--------------------------------------------------------------------
The passive model stored inside the viewer changed from ``Arc<MjModel>`` to
``Box<MjModel>``. The closure passed to
:docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback`
now receives ``&mut MjData<Box<MjModel>>``.

**Before (3.x):**

.. code-block:: rust

    viewer.add_ui_callback(|ctx, data: &mut MjData<Arc<MjModel>>| { ... });

**After (4.0.0):**

.. code-block:: rust

    viewer.add_ui_callback(|ctx, data: &mut MjData<Box<MjModel>>| { ... });

Type annotations in closure parameters must be updated if present.
Closures without an explicit type annotation require no change.


