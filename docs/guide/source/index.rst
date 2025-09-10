=======================
MuJoCo-rs documentation
=======================


.. image:: https://img.shields.io/docsrs/mujoco-rs/latest
    :target: https://docs.rs/mujoco-rs


.. image:: https://img.shields.io/crates/v/mujoco-rs.svg
    :target: https://crates.io/crates/mujoco-rs


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
  - Lifetime guarantees.

- Methods as function wrappers.
- Easy manipulation of simulation data via :ref:`attribute_views`.

- :ref:`visualization`:

  - :ref:`mj_renderer`: offscreen rendering to array or file.
  - :ref:`mj_rust_viewer`: onscreen visualization of the 3D simulation.

    .. image:: ../../img_common/viewer_spot.png
        :width: 50%



Installation
=======================
For installation, see :ref:`installation`.

Optional Cargo features
=======================
Optional Cargo features can be enabled:
  - ``viewer``: enables the Rust-native MuJoCo viewer. It can
    display everything and respond to mouse/keyboard events.
    No side-panels (the user menu) currently exists.
  - ``cpp-viewer``: enables the Rust wrapper around the C++ MuJoCo viewer.
    This requires static linking to a modified fork of MuJoCo, as described in :ref:`installation`.
  - ``renderer``: enables offscreen rendering for writing RGB and
    depth data to memory or file.

By default, ``viewer`` and ``renderer`` are enabled.


Table of contents
===================

.. toctree::
   :maxdepth: 2

   installation
   programming/programming
   api
   changelog
