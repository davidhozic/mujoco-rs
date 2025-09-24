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


Prerequisites
=======================



.. Missing libraries
.. ==================
.. MuJoCo-rs should work out of the box after you provide it with the MuJoCo library. If the build fails and asks
.. for additional dependencies, install them via your system package manager.
.. For example, to install glfw3 on Ubuntu/Debian, this can be done like so: ``apt install libglfw3-dev``.

.. Note that on Windows, GLFW will either be compiled from source or downloaded from GLFW's repository.
.. If the build doesn't work, please `report this as a bug <https://github.com/davidhozic/mujoco-rs/issues>`_.



GLFW3 (optional, rendering only)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. tip::

   If you **do not require** the use of :ref:`mj_renderer` or :ref:`mj_rust_viewer`,
   you can **skip** installation/compilation of GLFW3:

   ::

      # Disables GLFW3 dependency
      cargo add mujoco-rs --no-default-features


Windows
:::::::::::::::::::::
On Windows, a pre-built library will **automatically be downloaded and linked**.
No steps should be required from the user.
If an error occurs, please submit an `issue <https://github.com/davidhozic/mujoco-rs/issues>`_.

You can **optionally** build the library by enabling the ``glfw-build`` feature (only on Windows):

::

    cargo add mujoco-rs --features glfw-build

Linux
:::::::::::::::::::::
Since GLFW 3.4 is relatively new and a lot of distros come with an older version of GLFW,
MuJoCo-rs will always compile GLFW 3.4.

Compilation
___________________
Compilation is automatic through Cargo, so no work is needed from the user, other than the installation
of **CMake** and **build tools**. For example, on Ubuntu we can install those with:

::

    sudo apt install build-essential cmake


In the future, if the ``glfw-rs`` implements better configuration options, pkg-config will also
be allowed.


MacOS
:::::::::::::::::::::
MacOS is untested. It should behave the same as with Linux.


----------------------------------------------------------------------------------------------------


MuJoCo
~~~~~~~~~~~~~~~~~~~~~~~
Because MuJoCo-rs doesn't directly bundle MuJoCo,
the latter, in the form of a pre-built binary or source code, must be `downloaded <mj_download_>`_
or compiled. Make sure to download or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

Compilation of MuJoCo-rs incudes **linking** the MuJoCo library.
This requires some environmental variables to be set, which tell
MuJoCo-rs where to obtain the MuJoCo library.

The linking process depends on whether you plan to dynamically link (recommended),
statically link or statically link with support of C++ based viewer (instead of the Rust-native one).

-----------------------------

Dynamic linking
:::::::::::::::::::
Dynamic linking is OS-dependent. To dynamically link, the primary variable
``MUJOCO_DYNAMIC_LINK_DIR`` must be set. 


Linux
_______________
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
_______________
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
_______________
MacOS is currently untested.


----------------------

.. _static_linking:

Static linking
:::::::::::::::::::
We also provide an option to statically link:
::

   export MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/
   cargo build


Note that the ``/path/mujoco/lib`` needs to contain all the MuJoCo dependencies.

Additionally, official MuJoCo builds include the precompiled MuJoCo library only in its shared (dynamic) form.
To statically link, you'll need to compile the library yourself.
MuJoCo's build system doesn't (yet) support static linking, however
we provide a modified MuJoCo repository, which allow static linking (see :ref:`static_link_with_cpp_viewer`).


.. _static_link_with_cpp_viewer:

Static linking with C++ viewer
::::::::::::::::::::::::::::::::::::::
While MuJoCo-rs already provides a :ref:`rust_native_viewer`, we understand that some projects wish
to use the original C++ based 3D viewer (also named Simulate).
To enable this, we provide a modified MuJoCo repository, with modifications
enabling static linking and a safe interface between Rust and the C++ Simulate code.

To build statically linkable libs with C++ based viewer included, perform the following steps:

1. Clone the MuJoCo-rs repository,
2. Change your directory to the cloned repository,
3. Run commands:
   ::

       git submodule update --init --recursive
       cd ./mujoco/
       cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed
       cmake --build build --parallel --target libsimulate --config=Release

4. Follow instructions in the :ref:`Static linking <static_linking>` section.

The builds are tested with the ``gcc`` compiler.

-----------------------------

