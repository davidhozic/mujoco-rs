.. _migration:

=======================
Migration guide
=======================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mjv_camera| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`
.. |mjr_context| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`
.. |mjs_joint| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsJoint`
.. |mjs_body| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody`
.. |mjs_frame| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsFrame`
.. |mjs_tendon| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsTendon`


This page documents the migration steps for upgrading between major versions of MuJoCo-rs.
For a full list of changes, see the :ref:`changelog`.


.. Each release group lives in its own fragment. Add a new release at the top of the list.

.. include:: migration/6.0.x.rst.inc
.. include:: migration/5.0.x.rst.inc
.. include:: migration/4.0.x.rst.inc
.. include:: migration/3.0.x.rst.inc
