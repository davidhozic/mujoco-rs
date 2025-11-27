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

    cargo add mujoco-rs --no-default-features --features "auto-download-mujoco use-rpath viewer viewer-ui renderer"

- **Without** visualization/rendering support:

  ::

    cargo add mujoco-rs --no-default-features --features "auto-download-mujoco use-rpath"

See :ref:`opt-cargo-features` for information about available Cargo features.

Then additional dependencies may need to be installed/configured:

- :ref:`mujoco_dep`: The actual physics engine, which is a C library.

  - When the ``auto-download-mujoco`` Cargo feature is enabled, MuJoCo is
    **automatically fetched and configured** on **Linux** and **Windows** platforms.
  
  - When the ``use-rpath`` Cargo feature is enabled, the **RPATH** of the final binary will
    be updated to include the path to the MuJoCo library files. This only affects Linux and MacOS
    and when dynamically linking (including automated downloads). RPATH is a table inside the binary,
    which tells the dynamic loader where shared (dynamic) libraries can be found.


Dependencies
=======================


.. _mujoco_dep:

MuJoCo
---------------

Automatic setup (dynamic linking)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Starting from MuJoCo-rs 2.1.0, the MuJoCo library can be **automatically downloaded**, extracted and configured
for **Linux** and **Windows** platforms by enabling the ``auto-download-mujoco`` Cargo feature:

::

    cargo add mujoco-rs --features "auto-download-mujoco use-rpath"

.. attention::

    Official MuJoCo builds only allow **dynamic linking**. When distributing compiled binaries,
    the downloaded MuJoCo library must be provided to users in addition to the binary.

By default, the library is downloaded and extracted inside the
**package root directory** (where your Cargo.toml is located). If you're happy with this,
nothing further is needed on your part.
To change the download and extraction location, a custom
directory path can be given via the ``MUJOCO_DOWNLOAD_DIR`` environmental variable.
For example: ``MUJOCO_DOWNLOAD_DIR="/home/username/Downloads/" cargo build``.

.. attention::

    On **Linux**, if you run the program and see an error about a missing library file,
    you can either copy the MuJoCo .so files to a standard location (e.g., /usr/lib/)
    or add the path to the .so files into ``LD_LIBRARY_PATH``
    (e.g., ``export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/``).
    If you've enabled the ``use-rpath`` Cargo feature, you can avoid this error,  by placing
    the downloaded ``mujoco-x.x.x`` directory inside the ``MUJOCO_DOWNLOAD_DIR`` directory, which by default is
    the current working directory.

    On **Windows**, the MuJoCo **DLL** file must be located in the directory from which the executable is run
    (i.e., the current working directory at runtime), or its directory
    path must be added to the ``PATH`` environment variable.

.. note::

    Even if the ``auto-download-mujoco`` Cargo feature is enabled, MuJoCo-rs will first try to use pkg-config
    (Linux and MacOS only) and perform a download only after pkg-config fails. Note that pkg-config is not officially supported
    by MuJoCo, so this exists as a courtesy for custom setups.


----------------------


Manual setup
^^^^^^^^^^^^^^^^^^^^

MuJoCo can also be provided manually.
Make sure to `download <mj_download_>`_ or compile MuJoCo version |MUJOCO_VERSION_BOLD|.

The manual setup process depends on whether you plan to dynamically link (recommended)
or statically link (with support for the C++ based viewer wrapper, as an alternative to the Rust-native one).


Dynamic linking
~~~~~~~~~~~~~~~~~~~~~~
Dynamic linking is OS-dependent. To dynamically link, the primary variable
``MUJOCO_DYNAMIC_LINK_DIR`` must be set. 

.. note::

    On Linux and MacOS, if you somehow managed to register MuJoCo with pkg-config, nothing more is needed.
    MuJoCo provides no official way to do this, but we still keep the option open.

.. tabs::

   .. tab:: Linux

        When using Linux (bash), the primary variable can be set like so (assuming MuJoCo's **.so** file is located
        inside ``/path/mujoco/lib/``.):
        ::

            export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/

        .. attention::

            When the ``use-rpath`` Cargo feature is enabled, MuJoCo-rs will also update the **RPATH**.
            To make RPATH work, keep the ``mujoco/`` in the same (relative) directory given to ``MUJOCO_DYNAMIC_LINK_DIR``.

            E.g., if you've set ``MUJOCO_DYNAMIC_LINK_DIR=./mujoco/lib/``, then keep ``mujoco/``
            in the current working directory (or relative to the compiled binary).

        In the event that the user's program refuses to run and outputs something like:

            "error while loading shared libraries: libmujoco.so"

        the path to the MuJoCo library's directory must also be added to ``LD_LIBRARY_PATH``:
        ::

            export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib/

   .. tab:: Windows

        When using Windows (powershell), the primary variable can be set like so:

        ::

        $env:MUJOCO_DYNAMIC_LINK_DIR = "/path/mujoco/lib/"

        Additionally, when running the program, the ``mujoco.dll`` file needs to be placed in the **current working directory**.
        Alternatively, the path to the DLL file can be added to the PATH environmental variable.
        For help adding the path ``/path/mujoco/bin/`` to the PATH variable, see
        `here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_.

        .. attention::

            Make sure the PATH variable contains the path to the directory of the **.dll** file, **NOT** of the **.lib** file.
            The **.lib** file is used only for compilation, while the **.dll** is used at runtime.
            The **.dll** file is contained in the ``bin/`` directory of the MuJoCo download.
            The **.lib** file is contained in the ``lib/`` directory of the MuJoCo download.


   .. tab:: MacOS

        To use MuJoCo-rs on MacOS, follow the following steps:

        1. Open the downloaded .dmg file.
        2. Copy ``mujoco.framework`` to the current working directory.
        3. Create a symbolic link to the copied: ``libmujoco.x.x.x.dylib`` and name it ``libmujoco.dylib``:

        - ``ln -s mujoco.framework/Versions/Current/libmujoco.x.x.x.dylib libmujoco.dylib``.

        4. Set the primary environment variable:

        - ``export MUJOCO_DYNAMIC_LINK_DIR=./``

        5. When compiling, make sure to enable the ``use-rpath`` Cargo feature.

        In the event that the user's program refuses to run and outputs something like:

            "Library not loaded"

        the path to the MuJoCo library's directory must also be added to ``DYLD_LIBRARY_PATH``:
        ::

            export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/path/to/mujoco/lib/



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

1. Clone MuJoCo-rs's repository,
2. Change your directory to the cloned repository,
3. Run commands:
   ::

       git submodule update --init --recursive
       cd ./mujoco/
       cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF
       cmake --build build --parallel --target glfw libmujoco_simulate --config=Release

   This was tested with the ``gcc`` compiler.
   
   Note that ``-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF`` **disables link-time optimization**, thus resulting in slightly
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

    When LTO is enabled, the **system linker** must be used, because rust-lld doesn't know how to read extra LTO information
    produced by other linkers:

    ::

        RUSTFLAGS="-C linker-features=-lld" cargo build


    In performance critical cases, it is also recommended to use the official MuJoCo shared (dynamic) libraries,
    which are generally more optimized. Performance gain with LTO enabled on static builds is about 10% more steps per second.

