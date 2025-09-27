.. _installation:

=============================
Installation
=============================

.. _mj_download: https://github.com/google-deepmind/mujoco/releases/tag/3.3.5


MuJoCo-rs
====================


MuJoCo-rs can be added to your project by running:

- **With** visualization/rendering support:

  ::

    cargo add mujoco-rs

- **Without** visualization/rendering support:

  ::

    cargo add mujoco-rs --no-default-features


Then additional dependencies need to be installed/configured:

- :ref:`mujoco_dep` (**required**): The actual physics engine, which is a C library.
- :ref:`Other build dependencies<build_deps>`. These are **optional** and only required when **visualization** or **rendering** is enabled.




Prerequisites
=======================


.. _mujoco_dep:

MuJoCo
---------------
Because MuJoCo-rs doesn't directly bundle MuJoCo,
the latter, in the form of a pre-built library or source code, must be `downloaded <mj_download_>`_
or compiled. Make sure to download or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

Compilation of MuJoCo-rs incudes **linking** the MuJoCo library.
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
    The **.dll** file should be contained in the ``bin/`` directory of the MuJoCo download.


MacOS
++++++++++++++++++
MacOS is currently untested.


----------------------

.. _static_linking:

Static linking
~~~~~~~~~~~~~~~~~~
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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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


.. _build_deps:

Build dependencies (visualization/rendering only)
------------------------------------------
MuJoCo-rs may require some additional build-time dependencies, depending whether you want
visualization support. These are needed to build **GLFW** --- a library used for window and OpenGL
context management.

If you **do not require** the use of :ref:`mj_renderer` or :ref:`mj_rust_viewer`,
you can avoid build-time dependencies by disabling the default features:

::

    # Disables viewer and renderer features.
    cargo add mujoco-rs --no-default-features

When there is a need for rendering/visualization support, dependencies are OS-dependent.

Windows
~~~~~~~~~~~~~~~~~~
On Windows, no additional build-time dependencies are required. GLFW is **obtained automatically**.
If you run into problems, please submit an `issue <https://github.com/davidhozic/mujoco-rs>`_.

.. hint::

    Optionally, instead of using the pre-built, GLFW can be compiled from scratch by enabling MuJoCo-rs's ``glfw-build``
    Cargo feature.

Linux
~~~~~~~~~~~~~~~~~~
On Linux, MuJoCo-rs will try to use the GLFW installed in your Linux distribution, but only when
the installed version is **GLFW 3.4**.
Linkage will be auto-configured through pkg-config. In such case,
no additional dependencies are needed. For example, Ubuntu 25.04+ and Fedora 42 use GLFW 3.4. 

If you're distribution uses an **older GLFW** version, GLFW **3.4** will be **compiled**.
This requires **CMake** and common C build tools to be installed.
E.g., for Ubuntu/Debian distributions run:

::

    sudo apt install -y build-essential cmake


Wayland
+++++++++++
By default, only X11 support is enabled. Use MuJoCo-rs's ``glfw-wayland`` Cargo feature
to enable Wayland support.


MacOS
~~~~~~~~~~~~~~~~
We don't test MacOS builds, however the process should be the same as for Linux.
