.. _interface_c_api:

============================
Interface to C API
============================

When the user requires extra flexibility, that MuJoCo-rs does not offer, direct FFI bindings to the
C language structs and functions can be used.

Direct FFI bindings are available inside the :docs-rs:`mujoco_rs::mujoco_c` module.

Mixing between direct FFI bindings and the rest of MuJoCo-rs is also possible.
Most of MuJoCo-rs's structs are actually just FFI structs aliased to a PascalCase name.
To obtain a reference to the FFI type inside of a wrapped type, call either ``ffi()`` or
``ffi_mut()``.
For example, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ffi_mut`
and :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>ffi` can be used
as parameters inside :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_step`.

.. code-block:: rust

    unsafe { mujoco_rs::mujoco_c::mj_step(model.ffi(), data.ffi_mut()) }

The FFI bindings can also be used to read specific attributes that aren't directly exposed
via wrapped structs:

.. code-block:: rust

    model.ffi().nq

