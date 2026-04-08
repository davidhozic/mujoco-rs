.. _webassembly_build:

=============================
WebAssembly (Emscripten)
=============================

MuJoCo-rs can be compiled to `WebAssembly <https://webassembly.org/>`_ via
`Emscripten <https://emscripten.org/>`_, allowing simulations to run under Node.js.

.. note::

    Only the ``wasm32-unknown-emscripten`` Rust target is supported.
    Rendering and visualization features are **not** available for this target.


.. _wasm_prerequisites:

Additional prerequisites for MuJoCo-rs
========================================

In addition to the emsdk prerequisites described in MuJoCo's documentation,
you need:

1. **Rust wasm32-unknown-emscripten target**

   ::

       rustup target add wasm32-unknown-emscripten

2. **MuJoCo-rs repository with the submodule** (if not already done during
   installation):

   ::

       git submodule update --init --recursive


.. _wasm_building:

Building MuJoCo
=============================

Use either the ``mujoco/`` submodule (already initialized in the MuJoCo-rs
repository) or a fresh clone of the official MuJoCo ``3.6.0`` release as the
source directory.  The official unmodified tag is sufficient for WASM -- the
submodule patches are only required for native/C++ features.

To clone the official release:

::

    git clone https://github.com/google-deepmind/mujoco.git --branch 3.6.0 --depth 1

Then build (replace ``mujoco`` with the path to whichever source you chose):

::

    emcmake cmake -S mujoco -B mujoco/build/wasm
    EMCC_CFLAGS="-fwasm-exceptions" cmake --build mujoco/build/wasm --target mujoco --parallel

``EMCC_CFLAGS="-fwasm-exceptions"`` is required because MuJoCo's ``CMakeLists.txt``
injects ``-fexceptions``, which generates JS-only ``emscripten_longjmp`` calls that are
incompatible with the Rust side.  ``-fwasm-exceptions`` overrides it (last flag wins)
and switches to native WebAssembly exception handling, keeping the ABI consistent with
the Rust crate.

The static archive is written to ``mujoco/build/wasm/lib/libmujoco.a``.


.. _wasm_rust_build:

Building the Rust crate
=============================

Point ``MUJOCO_STATIC_LINK_DIR`` at the ``lib/`` directory and build with the
Emscripten target.  ``EMCC_CFLAGS`` must match the flag used when building MuJoCo
(see :ref:`wasm_building` for the explanation):

::

    EMCC_CFLAGS="-fwasm-exceptions" \
        MUJOCO_STATIC_LINK_DIR=$(realpath mujoco/build/wasm/lib) \
        cargo build --example basic --target wasm32-unknown-emscripten

Cargo produces two output artifacts inside
``target/wasm32-unknown-emscripten/debug/examples/``:

- ``basic.js`` -- Emscripten JS glue / loader
- ``basic.wasm`` -- the compiled WebAssembly module


.. _wasm_memory:

Memory growth for complex models
=================================

The Emscripten WASM heap defaults to 16 MB.  Complex models (e.g. many geoms,
tendons, or connected bodies) can exceed this limit during ``mj_compile()`` and
will panic with ``"Could not allocate memory"``.  Add
``-sALLOW_MEMORY_GROWTH=1`` as a linker argument via ``RUSTFLAGS``:

::

    EMCC_CFLAGS="-fwasm-exceptions" \
        MUJOCO_STATIC_LINK_DIR=$(realpath mujoco/build/wasm/lib) \
        RUSTFLAGS="-C link-arg=-sALLOW_MEMORY_GROWTH=1" \
        cargo build --example <your_example> --target wasm32-unknown-emscripten


.. _wasm_running:

Running
=============================

Use the Node.js binary bundled with emsdk, or any compatible Node.js >= 16:

::

    NODE_BIN="$(command -v node 2>/dev/null \
        || echo '/path/to/emsdk/node/<version>/bin/node')"
    "$NODE_BIN" target/wasm32-unknown-emscripten/debug/examples/basic.js

Replace ``<version>`` with the directory name found under
``/path/to/emsdk/node/`` (e.g. ``22.16.0_64bit``).

Expected output: ``Step 0`` through ``Step 999``.

