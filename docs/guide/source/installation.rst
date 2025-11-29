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

    cargo add mujoco-rs --no-default-features --features "auto-download-mujoco viewer viewer-ui renderer"

- **Without** visualization/rendering support:

  ::

    cargo add mujoco-rs --no-default-features --features "auto-download-mujoco"

See :ref:`opt-cargo-features` for information about available Cargo features.

Then :ref:`mujoco_dep` installation needs to be configured, like described below.
MuJoCo is the actual physics engine that MuJoCo-rs depends on.

Dependencies
=======================


.. _mujoco_dep:

MuJoCo
---------------

Dynamic linking (official MuJoCo build)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. note::

    **MuJoCo** official builds are available only in the form of a **shared/dynamic** library.
    This is the preferred way of providing MuJoCo as it involves the least amount of work.

When the goal is to provide MuJoCo as a dynamic dependency,
you can either tell MuJoCo-rs to download MuJoCo automatically or you can download MuJoCo yourself.

.. tabs::

    .. tab:: Automatic download

        To make MuJoCo-rs automatically download MuJoCo, enable MuJoCo-rs's Cargo feature ``auto-download-mujoco``:

        ::

            cargo add mujoco-rs --no-default-features --features "auto-download-mujoco"

        Then, set the environmental variable ``MUJOCO_DOWNLOAD_DIR`` to the **absolute path**
        into which the MuJoCo library will be extracted. Note that a subdirectory will
        be created automatically in the format ``mujoco-x.y.z``, where ``x.y.z`` is the MuJoCo version.

        .. tabs::

            .. tab:: Linux/MacOS

                ::

                    export MUJOCO_DOWNLOAD_DIR=/home/user/libraries/

            .. tab:: Windows

                ::

                    $env:MUJOCO_DOWNLOAD_DIR="C:\Users\User\Libraries\"

    .. tab:: Manual download

        .. note::

            On Linux and MacOS, if you somehow managed to register MuJoCo with pkg-config, nothing more is needed.
            MuJoCo provides no official way to do this, but we still keep the option open.

        When providing MuJoCo manually, make sure its version is |MUJOCO_VERSION_BOLD|.

        Download MuJoCo from `here <mj_download_>`_.

        After download and extraction of MuJoCo, you can tell MuJoCo-rs where to find MuJoCo, by
        setting the ``MUJOCO_DYNAMIC_LINK_DIR`` environment variable to the **absolute** path of
        the ``lib/`` subdirectory.

        .. tabs::

            .. tab:: Linux

                    When using Linux (bash), the primary variable can be set like so (assuming MuJoCo's **.so** file is located
                    inside ``/path/mujoco/lib/``):
                    ::

                        export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/

            .. tab:: Windows

                    When using Windows (powershell), the primary variable can be set like so:

                    ::

                    $env:MUJOCO_DYNAMIC_LINK_DIR = "/path/mujoco/lib/"

            .. tab:: MacOS

                    One option is to set up your own homebrew file
                    (see `this issue <https://github.com/davidhozic/mujoco-rs/pull/94>`_).

                    Another option is to copy and link some files:

                    1. Open the `downloaded <mj_download_>`_ .dmg file.
                    2. Copy ``mujoco.framework`` to the current working directory.
                    3. Create a symbolic link to the copied: ``libmujoco.x.x.x.dylib`` and name it ``libmujoco.dylib``:

                    - ``ln -s mujoco.framework/Versions/Current/libmujoco.x.x.x.dylib libmujoco.dylib``.

                    4. Set the primary environment variable:

                    - ``export MUJOCO_DYNAMIC_LINK_DIR=./``


You should now be able to compile and run your crate.

Regardless of whether MuJoCo is download by MuJoCo-rs or you provided it manually, you may encounter
**runtime errors** about the library not being found. This can happen if the library is not
located in a **standard location** nor added to the OS-dependent **path environment variable**.
You can fix these kind of errors like so:

.. In the event that the user's program refuses to run and outputs something like:

..     "error while loading shared libraries: libmujoco.so"

.. tabs::

    .. tab:: Linux

        Either copy ``libmujoco.so`` and ``libmujoco.x.y.z.so`` to the standard location (i.e., ``/usr/lib/``)
        
        or,

        Update the ``LD_LIBRARY_PATH`` environment variable.
        Assuming ``libmujoco.so`` is located in ``/path/to/mujoco/lib/``:
        ::
        
            export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib/

    .. tab:: Windows

        Place ``mujoco.dll`` file the **current working directory** (next to the EXE).
        Alternatively, the path to the DLL file's directory can be added to the PATH environmental variable.
        See `here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_
        for a tutorial on configuring PATH.

        .. attention::

            Make sure the PATH variable contains the path to the directory of the **.dll** file, **not** of the **.lib** file.
            The **.lib** file is used only for compilation, while the **.dll** is used at runtime.
            The **.dll** file is contained in the ``bin/`` directory of the MuJoCo download.
            The **.lib** file is contained in the ``lib/`` directory of the MuJoCo download.


    .. tab:: MacOS

        Update the ``DYLD_LIBRARY_PATH`` environment variable.

        Assuming ``libmujoco.dylib`` is located in ``/path/to/mujoco/lib/``:
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

