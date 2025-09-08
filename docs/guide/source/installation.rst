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


Because the library itself doesn't directly bundle MuJoCo's C libraries,
MuJoCo itself, in the form of either a shared or static library, must be `downloaded <mj_download_>`_
or compiled. Make sure to download or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

Linking MuJoCo
====================
To compile your code, you must first set some environmental variables,
telling MuJoCo-rs where to obtain the MuJoCo library. The variables
depend on the operating system and whether you want to dynamically link or statically link.

-----------------------------

Dynamic linking
--------------------
The main variable that needs to be set is ``MUJOCO_DYNAMIC_LINK_DIR``:
::

   export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/

After OS-dependent configuration, described below, projects using MuJoCo-rs can compile normally with ``cargo build``.

Linux
~~~~~~~~~~~~~~~~~~~~~~
When using Linux, the library must be either:

- installed in a discoverable library location (e. g., ``/usr/local/lib``)
- or the path to the library's directory manually added to ``LD_LIBRARY_PATH`` like so:  
  ::

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib/


Windows
~~~~~~~~~~~~~~~~~~~~~~~~~
When using Windows, the ``.dll`` file must be discoverable in the environmental variable: **PATH**.
Unlike the ``.lib`` file, the ``.dll`` file is located in ``/path/mujoco/bin/`` (notice the **/bin/** part).

For help adding the path ``/path/mujoco/bin/`` to the PATH variable, see `here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_.


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


