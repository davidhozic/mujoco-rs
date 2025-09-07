=======================
MuJoCo-rs documentation
=======================


.. image:: https://img.shields.io/docsrs/mujoco-rs/latest
    :target: https://docs.rs/mujoco-rs


.. image:: https://img.shields.io/crates/v/mujoco-rs.svg
    :target: https://crates.io/crates/mujoco-rs


MuJoCo bindings and wrappers for the Rust programming language.
Includes a Rust-native viewer and also bindings to a modified C++ one.

`MuJoCo <https://mujoco.org/>`_ is a general purpose physics simulator.

MuJoCo version
=======================
This library uses FFI bindings to MuJoCo |MUJOCO_VERSION_BOLD|.



Installation
=======================
For installation, see :ref:`installation`.


Cargo features
=================
The following Cargo features can be enabled:

- ``viewer`` (default): enables the Rust-native MuJoCo viewer. This can currently
                display everything and respond to mouse/keyboard. No side-panels (the user menu) currently exists.
- ``cpp-viewer``: enables the Rust wrapper around the C++ MuJoCo viewer. This is only available if you build the MuJoCo yourself using the steps above (yes, you need to use the forked repository).



Table of contents
===================

.. toctree::
   :maxdepth: 2

   installation
   programming
   viewer
   api
