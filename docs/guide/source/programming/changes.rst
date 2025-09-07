========================
Changes from MuJoCo
========================

MuJoCo-rs tries to follow the MuJoCo's API with a few exceptions that make it more Rust-idiomatic.

Methods
====================
One of the changes from the MuJoCo C API are methods.
Since most MuJoCo functions directly correspond to some struct, we turned those functions into
methods at appropriate structs, while leaving the more general functions wrapped in idiomatic Rust function wrappers.

For example, the function :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_loadXML` is replaced with the method
:docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml`, while the more general
function :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mju_rayGeom` is replaced with
:docs-rs:`~~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom`.

Structs
===================
Since most MuJoCo's structs are just plain data structs, we avoid wrapping them and mostly
only rename them to follow the PascalCase style. Structs that contain heap allocated data
are however replaced with a safe Rust wrapper that automatically cleans after itself and provides
safe access to some of the attributes. Attributes that aren't directly exposed can be accessed
with the ``.ffi()`` or ``.ffi_mut()`` method. Note that the ``.ffi_mut()`` requires the use ``unsafe``
blocks due to the exposure of raw pointers, which are NOT to be replaced.
