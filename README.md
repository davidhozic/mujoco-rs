# mujoco-rs-w
MuJoCo bindings and wrappers for the Rust programming language. Includes a Rust-native viewer and also
bindings to a modified C++ one.

## MuJoCo version
This library uses FFI bindings to MuJoCo **3.3.5**.
The library can either be provided **dynamically** in the form of share library (.so and .dll):
- ``MUJOCO_DYNAMIC_LINK_LIB=/path/mujoco/lib/ cargo build``.

When using the shared library, the **Rust-native MuJoCo viewer** can be used,
but not the original C++ one.

Also note that when the MuJoCo library isn't installed in the standard location,
the path ``/path/mujoco/lib/`` must be added to `LD_LIBRARY_PATH` like so:
- ``LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/mujoco/lib/``

Now regarding **static linking**.
If you do not require the **C++ MuJoCO viewer (the simulate UI)** and have
somehow already obtained statically linkable libraries,
you can just run the following command:
- ``MUJOCO_STATIC_LINK_LIB=/path/mujoco/lib/ cargo build``.

Note that the ``/path/mujoco/lib`` needs to contain all the MuJoCo dependencies.

To build **statically** with the support of the **C++ MuJoCo viewer**,
perform the following:
1. Clone the repository
2. ``git submodule update --init --recursive``
3. ``cd ./mujoco/``
4. ``cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed``
5. ``cmake --build build --parallel --target libsimulate --config=Release``
6. Add the crate normally to your Cargo.toml.


## Features
Optional features can be enabled to add additional features.
These are:
- ``viewer`` (default): enables the Rust-native MuJoCo viewer. This can currently
display everything and respond to mouse/keyboard, however perturbations aren't yet supported. Additionally, no other UI is cpresent than
- ``cpp-viewer``: enables the Rust wrapper around the C++ MuJoCo viewer. This is only available if you build the MuJoCo yourself using the steps above (yes, you need to use the forked repository).


## Examples
Examples can be tested under the ``examples/`` directory.

## RAW FFI bindings
If the current state of the wrappers isn't satisfactory enough, raw FFI bindings to the C MuJoCo
library can be used. These are available under the ``mujoco_rs_w::mujoco_c`` module.
To access the FFI level structs of the wrappers, call the ``ffi()`` and ``ffi_mut()`` methods.

## NOTE
This project is WIP but functional. I accept pull requests about bug fixes
and feature requests. If you have any questions, please open a **discussion**.
