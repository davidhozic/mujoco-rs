.. _interface_c_api:

============================
Interface to C API
============================

When the user requires extra flexibility that MuJoCo-rs does not offer, direct FFI bindings to the
C language structs and functions can be used. All the structs, except the model-editing-related ones,
have their attributes public.

Direct FFI bindings are available inside the :docs-rs:`mujoco_rs::mujoco_c` module.

Mixing between direct FFI bindings and the rest of MuJoCo-rs is also possible.
Some MuJoCo-rs types are direct FFI aliases, while core simulation/runtime types
(such as ``MjModel`` and ``MjData``) are safe wrapper structs.
To obtain a reference to the FFI type inside a wrapper, call either ``ffi()`` or
``unsafe`` ``ffi_mut()``.
For example, :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ffi_mut`
and :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>ffi` can be used
as arguments to :docs-rs:`~mujoco_rs::mujoco_c::<fn>mj_step`.

.. warning::

    ``ffi_mut()`` bypasses wrapper invariants. Incorrect writes through raw FFI fields can
    violate safety assumptions and may cause undefined behavior.

.. code-block:: rust

    unsafe { mujoco_rs::mujoco_c::mj_step(model.ffi(), data.ffi_mut()) }

FFI bindings can also be used to read raw attributes directly:

.. code-block:: rust

    model.ffi().nq

