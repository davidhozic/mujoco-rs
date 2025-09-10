.. _installation:

=============================
Installation
=============================

.. _mj_download: https://github.com/google-deepmind/mujoco/releases


MuJoCo-rs
====================


MuJoCo-rs can be added to your project by running:

::

    cargo add mujoco-rs


Because the MuJoCo-rs doesn't directly bundle MuJoCo,
the latter, in the form, must be `downloaded <mj_download_>`_
or compiled. Make sure to download or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

Linking MuJoCo
====================
To compile your code, you must first set some environmental variables,
telling MuJoCo-rs where to obtain the MuJoCo library.

The linking process depends on whether you plan do dynamically link (recommended),
statically link or statically link with the support of C++ based viewer (instead of the Rust-native one).

-----------------------------

Dynamic linking
--------------------
Dynamic linking is OS-dependent. To dynamically link, the primary variable
``MUJOCO_DYNAMIC_LINK_DIR`` must be set. 


Linux
~~~~~~~~~~~~~~~~~~~~~~
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
~~~~~~~~~~~~~~~~~~~~~~~~~
When using Windows (powershell), the primary variable can be set like so:

::

   $env:MUJOCO_DYNAMIC_LINK_DIR = "/path/mujoco/lib/"

Additionally, the library in DLL form must be added to the **PATH variable**.
For help adding the path ``/path/mujoco/bin/`` to the PATH variable, see
`here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_.

.. attention::

    Make sure the PATH variable contains the path to the **.dll** file, **NOT .lib**.
    The **.lib** file is used only for compilation, while the **.dll** is used at runtime.
    The **.dll** file should be contained in the ``bin/`` directory of the MuJoCo download.


MacOS
~~~~~~~~~~~~~~~~~~~~~~~~~
MacOS is currently untested.


----------------------

.. _static_linking:

Static linking
--------------------
We also provide an option to statically link:
::

   export MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/
   cargo build


Note that the ``/path/mujoco/lib`` needs to contain all the MuJoCo dependencies.

.. _static_link_with_cpp_viewer:

Static linking with C++ viewer
---------------------------------
While MuJoCo-rs already provides **a Rust-native 3D viewer**, we understand that some projects wish
to use the original C++ 3D viewer (also named Simulate).
To enable this, we provide a modified MuJoCo repository, with modifications
enabling static linking and a safe interface between Rust and the C++ Simulate code.

To build statically linkable libs with C++ viewer included, perform the following steps:

1. Clone the MuJoCo-rs repository
2. Run commands:
   ::

       git submodule update --init --recursive
       cd ./mujoco/
       cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed
       cmake --build build --parallel --target libsimulate --config=Release

3. Follow instructions in the :ref:`Static linking <static_linking>` section.

The builds are tested with the ``gcc`` compiler.

-----------------------------


Missing libraries
==================
MuJoCo-rs should work out of the box after you provide it with the MuJoCo library. If the build fails and asks
for additional dependencies, install them via your system package manager.
For example, to install glfw3 on Ubuntu/Debian, this can be done like so: ``apt install libglfw3-dev``.

Note that on Windows, GLFW will be compiled from scratch.
If the build doesn't work, please `report this as a bug <https://github.com/davidhozic/mujoco-rs/issues>`_.


