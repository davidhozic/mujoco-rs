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
source directory.  The official unmodified tag is sufficient for WASM — the
submodule patches are only required for native/C++ features.

To clone the official release:

::

    git clone https://github.com/google-deepmind/mujoco.git --branch 3.6.0 --depth 1

Then build (replace ``mujoco`` with the path to whichever source you chose):

::

    emcmake cmake -S mujoco -B mujoco/build/wasm
    EMCC_CFLAGS="-fwasm-exceptions" cmake --build mujoco/build/wasm --target mujoco --parallel

The static archive is written to ``mujoco/build/wasm/lib/libmujoco.a``.


.. _wasm_rust_build:

Building the Rust crate
=============================

Point ``MUJOCO_STATIC_LINK_DIR`` at the ``lib/`` directory and build with the
Emscripten target:

::

    MUJOCO_STATIC_LINK_DIR=$(realpath mujoco/build/wasm/lib) \
        cargo build --example basic --target wasm32-unknown-emscripten

Cargo produces two output artifacts inside
``target/wasm32-unknown-emscripten/debug/examples/``:

- ``basic.js`` -- Emscripten JS glue / loader
- ``basic.wasm`` -- the compiled WebAssembly module


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

