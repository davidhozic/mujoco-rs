========================
Changes from MuJoCo
========================

MuJoCo-rs tries to follow the MuJoCo's API with a few exceptions that make it more Rust-idiomatic,
safer and easier to use.

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
====================
Most of MuJoCo's structs are plain data structs. In Rust, we mostly keep them as-is, renaming them to PascalCase. 
Structs with heap-allocated data are wrapped in safe Rust types that automatically manage memory and provide safe attribute access. 

Attributes not directly exposed can be accessed via the ``.ffi()`` or ``.ffi_mut()`` methods.
For more details, see :ref:`interface_c_api`.

Additionally, :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData` and
:docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel` provide :ref:`attribute_views` to specific
item types (joint, body, geom, etc.).
