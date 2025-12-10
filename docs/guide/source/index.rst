=======================
MuJoCo-rs
=======================


.. image:: https://img.shields.io/docsrs/mujoco-rs/latest
    :target: https://docs.rs/mujoco-rs


.. image:: https://img.shields.io/crates/v/mujoco-rs.svg
    :target: https://crates.io/crates/mujoco-rs

.. image:: https://img.shields.io/badge/license-MIT%20or%20Apache--2.0-blue?logo=github
   :target: https://github.com/davidhozic/mujoco-rs
   :alt: License: MIT OR Apache-2.0


MuJoCo bindings and wrappers for the Rust programming language.
Includes a Rust-native :ref:`mj_rust_viewer` and also bindings to a modified C++ one.

`MuJoCo <https://mujoco.org/>`_ is a general purpose physics simulator.

MuJoCo version
=======================
MuJoCo-rs uses FFI bindings to MuJoCo |MUJOCO_VERSION_BOLD|.


Main features
=======================
MuJoCo-rs tries to stay close to the MuJoCo's C API, with a few additional features for ease of use.
The main features on top of MuJoCo include:

- Safe wrappers around structs:
  
  - Automatic allocation and cleanup.
  - Lifetime checks.

- Methods as function wrappers.
- Easy manipulation of simulation data via :ref:`attribute_views` (named access).
- High-level :ref:`model_editing`.
- :ref:`visualization`:

  - :ref:`mj_renderer`: offscreen rendering to array or file.
  - :ref:`mj_rust_viewer`: onscreen visualization.

    .. image:: ../../img_common/viewer_spot.png
        :width: 50%


Installation
=======================
For installation, see :ref:`installation`.


.. _opt-cargo-features:

Optional Cargo features
=======================
Optional Cargo features can be enabled:

- ``viewer``: enables the Rust-native MuJoCo viewer.

  - ``viewer-ui``: enables the (additional) user UI within the viewer.
    This also allows users to add custom `egui <https://docs.rs/egui/0.33.2/egui/>`_ widgets to the viewer.

- ``cpp-viewer``: enables the Rust wrapper around the C++ MuJoCo viewer.
  This requires static linking to a modified fork of MuJoCo, as described in :ref:`installation`.
- ``renderer``: enables offscreen rendering for writing RGB and
  depth data to memory or file.

  - ``renderer-winit-fallback``: enables the invisible window fallback (based on winit) when offscreen
    rendering fails to initialize. Note that true offscreen rendering is only available on Linux platforms
    when the video driver supports it. On Windows and MacOS, this feature must always be
    enabled when the ``renderer`` feature is enabled.

- ``auto-download-mujoco``: MuJoCo dependency will be automatically downloaded to the specified path.

  - This is only available on Linux and Windows.
  - The environmental variable ``MUJOCO_DOWNLOAD_DIR`` must be set to the absolute path of the download location.
  - Downloaded MuJoCo library is still a shared library, see 
    [installation](https://mujoco-rs.readthedocs.io/en/latest/installation.html#mujoco)
    for information on complete configuration.


By default, ``viewer``, ``viewer-ui``, ``renderer``, and ``renderer-winit-fallback`` are enabled.




Table of contents
===================

.. toctree::
   :maxdepth: 2

   installation
   programming/programming
   api
   changelog
