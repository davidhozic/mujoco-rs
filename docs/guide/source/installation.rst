.. _installation:

=============================
Installation
=============================

.. _mj_download: https://github.com/google-deepmind/mujoco/releases/tag/3.17.0

.. _mj_fork: https://github.com/davidhozic/mujoco-fork/tree/release/3.17.0-static


MuJoCo-rs
====================

.. note::

    MuJoCo-rs requires **Rust 1.95** or newer (MSRV).  Run ``rustup update stable``
    to ensure you are on a supported toolchain.


MuJoCo-rs can be added to your project like so:

- **Without** visualization/rendering support:

  ::

    cargo add mujoco-rs

- **With** visualization/rendering support:

  ::

    cargo add mujoco-rs --features "viewer-ui renderer-winit-fallback"

.. note::

    On Windows and macOS the ``renderer-winit-fallback`` feature is **required** whenever the
    ``renderer`` feature is enabled (the build fails otherwise). It is optional on Linux.

.. _macos-glutin-patch:

.. attention::

    **macOS**: visualization (the viewer and the renderer) does not work on macOS with the
    upstream ``glutin`` crate. To make it work, patch ``glutin`` in your project's
    ``Cargo.toml`` with the following fork:

    .. code-block:: toml

        [patch.crates-io]
        glutin = { git = "https://github.com/davidhozic/glutin", rev = "e95fe97f1857c0498ed2236c0e8d60e5a30a2854" }

See :ref:`opt-cargo-features` for information about available Cargo features.
Visualization and rendering features are opt-in to keep compilation fast for headless use cases.

Then :ref:`mujoco_dep` installation needs to be configured, as described below.
MuJoCo is the actual physics engine that MuJoCo-rs depends on.

Dependencies
=======================


.. _mujoco_dep:

MuJoCo
---------------

The only third-party dependency, not managed by Cargo, is the MuJoCo library itself (which this crate wraps).
You have multiple ways of providing it:


.. list-table::
    :header-rows: 1

    * - Configuration / Operating System
      - Linux
      - Windows
      - macOS
    * - :ref:`pkg_config`
      - |checkmark|
      -
      - |checkmark|
    * - :ref:`dynamic_linking`, automatic download
      - |checkmark|
      - |checkmark|
      -
    * - :ref:`dynamic_linking`, manual download
      - |checkmark|
      - |checkmark|
      - |checkmark|

    * - :ref:`static_linking`, automatic download (unofficial MuJoCo build)
      - |checkmark|
      - |checkmark|
      - |checkmark|

    * - :ref:`static_linking`, manual build (unofficial MuJoCo build)
      - |checkmark|
      - |checkmark|
      - |checkmark|


.. |checkmark| unicode:: U+2713

MuJoCo-rs uses the first available configuration, in this order: ``MUJOCO_STATIC_LINK_DIR``,
``MUJOCO_DYNAMIC_LINK_DIR``, automatic download of static MuJoCo library, pkg-config,
automatic download of dynamic MuJoCo library.

.. note::

    Cross-OS builds (for example a Windows target from a Linux host) support neither the
    pkg-config nor automatic download of the dynamically linked library
    Such a build fails unless ``MUJOCO_DYNAMIC_LINK_DIR`` or ``MUJOCO_STATIC_LINK_DIR`` points to a MuJoCo
    build for the target. Cross-compilation within the same OS (for example Linux x86_64 to
    Linux aarch64) and automatic downloads of statically linked library are not affected.

.. _pkg_config:

pkg-config
~~~~~~~~~~~~~
On **Linux** and **macOS** platforms, pkg-config can be used to streamline the configuration.
You need to set this up yourself. See `this pull request <https://github.com/davidhozic/mujoco-rs/pull/94>`_
for more info. The MuJoCo version that pkg-config reports must be exactly |MUJOCO_VERSION_BOLD|.

Depending on where the library is placed, you may still need to set up the dynamic loader
environment variable, as described in :ref:`here <runtime_library_path>`.

.. attention::

    Currently, **pkg-config precedes the automatic download of the dynamically linked MuJoCo library** (described below).
    This is an old behavior allowing to preserve backward compatibility. It will change in the next
    major (by SemVer standards) release.

    Automatic downloads of statically linked library, as well as manual linking of either
    dynamic or static MuJoCo libraries, precede pkg-config. 

.. _dynamic_linking:

Dynamic linking (official MuJoCo build)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
When the goal is to provide MuJoCo as a dynamic dependency,
you can either tell MuJoCo-rs to download MuJoCo automatically or you can download MuJoCo yourself.

.. tabs::

    .. tab:: Automatic download

        To make MuJoCo-rs automatically download MuJoCo, enable MuJoCo-rs's Cargo feature
        ``download-dynamic-mujoco``:

        ::

            cargo add mujoco-rs --features "download-dynamic-mujoco"

        Then, set the environment variable ``MUJOCO_DOWNLOAD_DIR`` to the **absolute path**
        into which the MuJoCo library will be extracted. Note that a subdirectory will
        be created automatically in the format ``mujoco-x.y.z``, where ``x.y.z`` is the MuJoCo version.

        .. tabs::

            .. tab:: Linux

                ::

                    export MUJOCO_DOWNLOAD_DIR=/home/user/libraries/

            .. tab:: Windows

                ::

                    $env:MUJOCO_DOWNLOAD_DIR="C:\Users\User\Libraries\"

            .. tab:: macOS

                Automatic download for macOS is not supported.

    .. tab:: Manual download

        When providing MuJoCo manually, make sure its version is |MUJOCO_VERSION_BOLD|.

        Download MuJoCo from `here <mj_download_>`_.

        After download and extraction of MuJoCo, you can tell MuJoCo-rs where to find MuJoCo, by
        setting the ``MUJOCO_DYNAMIC_LINK_DIR`` environment variable to the **absolute** path of
        the ``lib/`` subdirectory.

        .. tabs::

            .. tab:: Linux

                When using Linux (bash), the primary variable can be set like so (assuming MuJoCo's
                **.so** file is located inside ``/path/mujoco/lib/``):

                ::

                    export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/

            .. tab:: Windows

                When using Windows (powershell), the primary variable can be set like so:

                ::

                    $env:MUJOCO_DYNAMIC_LINK_DIR="C:\path\mujoco\lib\"

            .. tab:: macOS

                1. Open the `downloaded <mj_download_>`_ .dmg file.
                2. Copy ``mujoco.framework`` to the current working directory.
                3. Create a symbolic link to the copied ``libmujoco.x.x.x.dylib`` and name it ``libmujoco.dylib``:

                   ::

                       ln -s mujoco.framework/Versions/Current/libmujoco.x.x.x.dylib libmujoco.dylib

                4. Set the primary environment variable:

                   ::

                       export MUJOCO_DYNAMIC_LINK_DIR=$(realpath .)


.. _runtime_library_path:

You should now be able to compile and run your crate.

Regardless of whether MuJoCo is downloaded by MuJoCo-rs or you provided it manually, you may encounter
**runtime errors** about the library not being found. This can happen if the library is not
located in a **standard location** nor added to the OS-dependent **path environment variable**.
You can fix these kinds of errors like so:

.. tabs::

    .. tab:: Linux

        Either copy ``libmujoco.so`` and ``libmujoco.so.x.y.z`` to the standard location (i.e., ``/usr/lib/``)

        or,

        Update the ``LD_LIBRARY_PATH`` environment variable.
        Assuming ``libmujoco.so`` is located in ``/path/to/mujoco/lib/``:
        ::

            export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib/

    .. tab:: Windows

        Place the ``mujoco.dll`` file in the **current working directory** (next to the EXE).
        Alternatively, the path to the DLL file's directory can be added to the PATH environment variable.
        See `here <https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/>`_
        for a tutorial on configuring PATH.

        .. attention::

            Make sure the PATH variable contains the path to the directory of the **.dll** file, **not** of the **.lib** file.
            The **.lib** file is used only for compilation, while the **.dll** is used at runtime.
            The **.dll** file is contained in the ``bin/`` directory of the MuJoCo download.
            The **.lib** file is contained in the ``lib/`` directory of the MuJoCo download.


    .. tab:: macOS

        Update the ``DYLD_LIBRARY_PATH`` environment variable.

        Assuming ``libmujoco.dylib`` is located in ``/path/to/mujoco/lib/``:
        ::

            export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/path/to/mujoco/lib/


.. _static_linking:

Static linking
~~~~~~~~~~~~~~~~~~
**Official MuJoCo builds** include the precompiled MuJoCo library only in its **shared (dynamic) form**.

To allow static linking, we maintain an actively synced `MuJoCo fork <https://github.com/davidhozic/mujoco-fork/tree/release/3.17.0-static>`__,
which includes modifications to allow: static linking, wrapper around the C++ Simulate viewer [#cpp_simulate_viewer_fn]_
(this is MuJoCo's official viewer, an alternative to our Rust viewer) and other smaller modifications
to support testing of MuJoCo-rs.

.. [#cpp_simulate_viewer_fn] This requires the Cargo feature ``cpp-viewer``.

Static binaries can either be compiler by users, using CMake, in the fork, or
automatically downloaded by MuJoCo-rs, on certain platforms.

.. tabs:: 

    .. tab:: Automatic download

        While Google Deepmind's MuJoCo team maintains the official dynamically linked release binaries,
        we (MuJoCo-rs authors) maintain unofficial statically linkable builds, uploaded at the
        `releases page <https://github.com/davidhozic/mujoco-fork/releases/tag/unofficial-3.17.0-static>`__
        of our `unofficial MuJoCo fork <mj_fork_>`__. Specifically, we build for the following platforms:

        .. list-table::
            :header-rows: 1

            * - Operating System / Architecture
              - x86_64
              - aarch64
            * - Linux
              - |checkmark|
              - |checkmark|
            * - Windows
              - |checkmark|
              - |checkmark|
            * - macOS
              - |checkmark|
              - |checkmark|

        On **Windows**, each platform has two builds: one for the dynamic C runtime (CRT) and one for
        the static CRT. MuJoCo-rs downloads the static CRT build when the ``crt-static`` target
        feature is enabled (i.e., ``RUSTFLAGS="-C target-feature=+crt-static"``).
        On **Linux** and **macOS**, the CRT is selected when the final binary is linked, thus the
        single prebuilt MuJoCo library (archive) supports both the dynamic and the static CRT.

        These are the details for individual build:

        .. list-table::
            :header-rows: 1

            * - Operating System
              - Compiler
              - Linker
              - Link-time optimization
              - Notes

            * - Linux
              - Clang 15.0.7 (Ubuntu 22.04)
              - LLD 15
              - |checkmark|
              - - Requires glibc 2.32 or newer
                - Embeds the C++ runtime (``libstdc++``)
            * - Windows
              - MSVC 19.44.35230 (Visual Studio 2022, toolset v143)
              - Requires MSVC ``link.exe`` 14.44 or newer (Visual Studio 2022 17.14 or newer)
              -
              - Uses the dynamic (``/MD``) or the static (``/MT``) CRT, depending on the build
            * - macOS
              - Clang 17 (Homebrew ``llvm@17``)
              - Apple ``ld`` 955.13
              - |checkmark|
              - - Requires macOS 11.0 or newer (deployment target)
                - Uses the system C++ runtime (``libc++``)

        For **Windows**, as unmarked above, the library archives are built with link-time optimizations (LTO)
        disabled. Enabling LTO on Windows provides no compatibility guarantees, thus requiring
        linkage to be done to be done using the same MSVC toolchain that generated the MuJoCo
        (or Simulate) library. If you wish to gain a slight performance boost, you'll need to
        compile MuJoCo yourself. See the other static-linking tab for the latter.

    .. tab:: Manual build and linking

        To build manually, go to the `unofficial MuJoCo fork's repository <mj_fork_>`__
        and build it with CMake.

        ::

            cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DMUJOCO_BUILD_TESTS:BOOL=OFF
            cmake --build build --parallel --target <TARGETS> --config=Release

        ``<TARGETS>`` can be, for example, ``mujoco`` to build the MuJoCo library. To build the C++ Simulate
        as a library (for use in MuJoCo-rs), target can be ``libmujoco_simulate``,
        which builds both MuJoCo and Simulate.

        This was tested with the ``gcc`` compiler.

        To actually build MuJoCo-rs, and make it link to the just built MuJoCo library, set the
        ``MUJOCO_STATIC_LINK_DIR`` environment variable to the **absolute** path of where the MuJoCo library lies.
        For example, this is usually in ``mujoco/build/lib`` or ``mujoco/build/lib64``.

        .. tabs::

            .. tab:: Linux / macOS

                ::

                    export MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/

            .. tab:: Windows

                ::

                    $env:MUJOCO_STATIC_LINK_DIR="C:\path\mujoco\lib\"

        Then you may use ``cargo build`` normally.

        .. rubric:: Performance and link-time optimization

        Note that ``-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF`` **disables link-time optimization**,
        thus resulting in slightly lower performance.
        Enabling it causes compatibility problems on **the Linux platform**.
        There should, however, be no problems on Windows and macOS, thus you should be able to safely
        enable LTO on said platforms.

        To obtain better performance by enabling link-time optimization, set
        ``-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON`` and tell Rust to link everything using the
        OS system linker, as opposed to rust-lld:

        ::

            RUSTFLAGS="-C linker-features=-lld" cargo build

        Alternatively, you may also compile MuJoCo using the Clang compiler, which *should* [#clang-rust-comp]_ produce
        artifacts compatible with the rust-lld linker.

        .. [#clang-rust-comp] rust-lld, being an LLVM-compatible linker, should be able to link older clang-built
           artifacts. This is not a guarantee.


        .. seealso::

           See this `Dockerfile <https://github.com/davidhozic/mujoco-rs/blob/v7.1.x/Dockerfile.ubuntu>`_
           for a reproducible build environment which, to our knowledge, matches MuJoCo's official
           build environment.
           The Dockerfile includes commented-out commands for installing the Rust toolchain and
           for using clang-13/clang++-13 as the compiler. Our testing showed that static libraries
           built in the container also work outside of the container.
           The Dockerfile defines a container running Ubuntu 22.04. See the commented-out cmake
           commands in the Dockerfile for the recommended compiler flags.
