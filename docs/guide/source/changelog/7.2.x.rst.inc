7.2.0 (MuJoCo 3.17.0)
=====================

.. rubric:: Deprecations

*Deprecated Cargo features*

- To support both dynamic and static linking, ``auto-download-mujoco`` is deprecated in favor of
  ``download-dynamic-mujoco``.

.. rubric:: New features and improvements

*MuJoCo downloads*

- Added downloading support for **prebuilt statically linkable** libraries
  enabled via the ``download-static-mujoco`` Cargo feature :sup:`new`.
  It also supports cross-OS builds.

.. rubric:: Other changes

*MuJoCo downloads*

- The hash of MuJoCo's downloaded archive is now pre-written inside the build script,
  instead of being downloaded from the server.

*Static linking*

- ``MUJOCO_STATIC_LINK_DIR`` now links only ``libmujoco`` (and ``libsimulate`` with ``cpp-viewer``),
  which bundle their dependencies. Build them from the 3.17.0 static branch of the
  `unofficial MuJoCo fork <https://github.com/davidhozic/mujoco-fork/tree/release/3.17.0-static>`__.
