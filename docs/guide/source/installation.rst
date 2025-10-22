.. _installation:

=============================
Installation
=============================

.. _mj_download: https://github.com/google-deepmind/mujoco/releases/tag/3.3.7


MuJoCo-rs
====================


MuJoCo-rs can be added to your project like so:

- **With** visualization/rendering support:

  ::

    cargo add mujoco-rs

- **Without** visualization/rendering support:

  ::

    cargo add mujoco-rs --no-default-features


Then additional dependencies need to be installed/configured:

- :ref:`mujoco_dep`: The actual physics engine, which is a C library.




Dependencies
=======================


.. _mujoco_dep:

MuJoCo
---------------
Because MuJoCo-rs doesn't directly bundle MuJoCo,
the latter, in the form of a pre-built library or source code, must be `downloaded <mj_download_>`_
or compiled. Make sure to download or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

Compilation of MuJoCo-rs includes **linking** to the MuJoCo library.
This requires some environmental variables to be set, which tell
MuJoCo-rs where to obtain the MuJoCo library.

The linking process depends on whether you plan to dynamically link (recommended),
statically link or statically link with support for the C++ based viewer (instead of the Rust-native one).

-----------------------------

Dynamic linking
~~~~~~~~~~~~~~~~~~~~~~
Dynamic linking is OS-dependent. To dynamically link, the primary variable
``MUJOCO_DYNAMIC_LINK_DIR`` must be set. 


Linux
++++++++++++
When using Linux (bash), the primary variable can be set like so:
::

   export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/

This is assuming MuJoCo's **.so** file is located inside ``/path/mujoco/lib/``.

Additionally, in the event that the user's program refuses to run and outputs something like:

    "error while loading shared libraries: libmujoco.so"

the path to the MuJoCo library's directory must also be added to ``LD_LIBRARY_PATH``:
::

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib/


Windows
+++++++++++
When using Windows (powershell), the primary variable can be set like so:

::

   $env:MUJOCO_DYNAMIC_LINK_DIR = "/path/mujoco/lib/"

Additionally, the library in DLL form must be added to the **PATH variable**.
For help adding the path ``/path/mujoco/bin/`` to the PATH variable, see
`here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_.
Alternatively, the DLL file can be placed in the working directory.

.. attention::

    Make sure the PATH variable contains the path to the **.dll** file, **NOT .lib**.
    The **.lib** file is used only for compilation, while the **.dll** is used at runtime.
    The **.dll** file is contained in the ``bin/`` directory of the MuJoCo download.


MacOS
++++++++++++++++++
MacOS is untested.


----------------------

.. _static_linking:

Static linking
~~~~~~~~~~~~~~~~~~
**Official MuJoCo builds** include the precompiled MuJoCo library only in its **shared (dynamic) form**.
To statically link, you'll need to **compile** MuJoCo **yourself**.
MuJoCo's build system doesn't (yet) support static linking, which is why
we provide a modified MuJoCo repository with static linking support.

While MuJoCo-rs already provides a :ref:`rust_native_viewer`, we understand that some projects wish
to use the original C++ based 3D viewer (also named Simulate).
To enable this, the modified MuJoCo repository also includes changes that support
a safe interface between Rust and the C++ Simulate code.

To build statically linkable libraries, perform the following steps:

1. Clone the MuJoCo-rs repository,
2. Change your directory to the cloned repository,
3. Run commands:
   ::

       git submodule update --init --recursive
       cd ./mujoco/
       cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF
       cmake --build build --parallel --target glfw libmujoco_simulate --config=Release

   The builds are tested with the ``gcc`` compiler.
   
   Note that ``-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF`` disables link-time optimization which results in slightly
   lower performance. Enabling it causes compatibility problems on the Linux platform. See the attention block below for more info.

4. Set the environmental variable ``MUJOCO_STATIC_LINK_DIR``. Bash example:

   ::

      export MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/

5. Build MuJoCo-rs

   ::

      cargo build

.. attention::

    **Link-time optimization**

    Above CMake configuration command **disables** link-time optimization (LTO). This results in worse performance
    but allows the compiled code to be used with the rust-lld linker, which is the default linker
    since Rust 1.90.0 on the **x86_64-unknown-linux-gnu** target.

    If performance is critical for you, link-time optimization of MuJoCo code can be enabled during configuration:

    ``cmake -B build -S . -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON ...``

    When LTO is enabled, the **system linker** must be used, because rust-lld doesn't know how to read the extra LTO information,
    produced by other linkers:

    ::

    RUSTFLAGS="-C linker-features=-lld" cargo build


    In performance critical cases, it is also recommended to use the official MuJoCo shared (dynamic) libraries,
    which are generally more optimized. Performance gain with LTO enabled on static builds is about 10% more steps per second.

