7.2.0 (MuJoCo 3.17.0)
=====================

.. rubric:: Deprecations

*Deprecated Cargo features*

- To support both dynamic and static linking, ``auto-download-mujoco`` is deprecated in favor of
  ``download-dynamic-mujoco``.

.. rubric:: Other changes

*MuJoCo downloads*

- The hash of MuJoCo's downloaded archive is now pre-written inside the build script,
  instead of being downloaded from the server.
