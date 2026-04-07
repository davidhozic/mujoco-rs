.. _webassembly_build:

=============================
WebAssembly (Emscripten)
=============================

.. versionadded:: 4.0.0

MuJoCo-rs can be compiled to `WebAssembly <https://webassembly.org/>`_ via
`Emscripten <https://emscripten.org/>`_, allowing simulations to run in the
browser or under Node.js.

Follow MuJoCo's
`official WebAssembly build documentation <https://github.com/google-deepmind/mujoco/tree/main/wasm>`_
to set up emsdk and build MuJoCo, but prepend
``EMCC_CFLAGS="-fwasm-exceptions"`` to every ``cmake --build`` command.  For
example:

::

    EMCC_CFLAGS="-fwasm-exceptions" cmake --build build --parallel

.. note::

    ``EMCC_CFLAGS="-fwasm-exceptions"`` is required because MuJoCo's
    ``CMakeLists.txt`` adds ``-fexceptions`` for Emscripten targets, which
    generates ``emscripten_longjmp`` -- a symbol present only in Emscripten's
    JS glue, not in the native wasm sysroot that Rust links against.
    ``EMCC_CFLAGS`` is appended to every ``emcc`` invocation *after* all
    cmake-managed flags, so ``-fwasm-exceptions`` takes precedence (the last
    flag wins in Clang).

.. note::

    Only the ``wasm32-unknown-emscripten`` Rust target is supported.
    Rendering and visualization features are **not** available for this target.

.. seealso::

    :ref:`static_linking` for background on building a static MuJoCo library.


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

Use the ``mujoco/`` submodule as the source directory and place the build
output inside it (excluded from version control by ``mujoco/.gitignore``):

::

    emcmake cmake -S mujoco -B mujoco/build/wasm
    EMCC_CFLAGS="-fwasm-exceptions" cmake --build mujoco/build/wasm --target mujoco --parallel

.. note::

    ``emcmake cmake`` may exit with **status 1** and print a fatal error about
    ``install(EXPORT)`` and the ``lodepng`` target not being in an export set.
    This is a known non-fatal issue; all required build files are still
    generated.  Ignore the non-zero exit code and proceed to the next step.

The static archive is written to ``mujoco/build/wasm/lib/libmujoco.a``.


.. _wasm_rust_build:

Building the Rust crate
=============================

Point ``MUJOCO_STATIC_LINK_DIR`` at the ``lib/`` directory and build with the
Emscripten target:

::

    LIB_DIR=$(realpath mujoco/build/wasm/lib)
    MUJOCO_STATIC_LINK_DIR="$LIB_DIR" \
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


.. _wasm_native_run:

Running natively
=============================

To confirm the example works on the host platform before targeting WebAssembly,
use the prebuilt MuJoCo shared library that ships with MuJoCo-rs:

::

    LIB_PATH=$(realpath mujoco-3.6.0/lib)
    MUJOCO_DYNAMIC_LINK_DIR="$LIB_PATH" LD_LIBRARY_PATH="$LIB_PATH" \
        cargo run --example basic

