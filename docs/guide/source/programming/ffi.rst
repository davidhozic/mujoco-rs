=========================
Raw bindings (FFI)
=========================

When MuJoCo-rs doesn't provide the desired control, the raw Foreign Function Interface (FFI)
can be used to obtain raw bindings to the MuJoCo C code.

The whole set of public types and functions to the MuJoCo C code is available in the 
:docs-rs:`mujoco_rs::mujoco_c` module.

To use the wrapped types in the FFI functions, the methods ``.ffi()`` and ``.ffi_mut()`` can be
called to provide access to the wrapped type. If no such method exists on the type, that means
the type can be used directly (if not, please report a bug). For example,
the method :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>ffi` can be used
to obtain the inner :docs-rs:`mujoco_rs::mujoco_c::<type>mjModel` FFI struct.
For example:

.. code-block:: rust

    unsafe { mujoco_rs::mujoco_c::mj_step(model.ffi(), data.ffi_mut()) }
