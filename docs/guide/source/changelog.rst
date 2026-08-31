.. _changelog:

==================
Changelog
==================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mjs_body| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsBody`
.. |mjs_frame| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsFrame`
.. |mj_geomview| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataView`
.. |mj_geomviewmut| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataViewMut`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjs_tendon| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsTendon`
.. |mjs_wrap| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsWrap`
.. |mjv_camera| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`
.. |mjr_context| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`
.. |mjs_light| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsLight`
.. |mjs_actuator| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsActuator`
.. |mjs_plugin| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjsPlugin`


This page contains the summary of changes made throughout different releases of the MuJoCo-rs crate.

Versioning
=================
This project uses `semantic versioning <https://semver.org/>`_.
This means that any incompatible changes increase the major version (**X**.y.z).
This also includes breaking changes that MuJoCo itself introduced, thus even an
update of MuJoCo alone can increase the major version.

.. Template titles
  .. rubric:: Breaking changes
  .. rubric:: New dependencies
  .. rubric:: Deprecations
  .. rubric:: Error handling
  .. rubric:: New features and improvements
  .. rubric:: Bug fixes
  .. rubric:: Other changes

.. Each release group lives in its own fragment. Add a new release at the top of the list.

.. include:: changelog/6.0.x.rst.inc
.. include:: changelog/5.0.x.rst.inc
.. include:: changelog/4.0.x.rst.inc
.. include:: changelog/3.0.x.rst.inc
.. include:: changelog/2.3.x.rst.inc
.. include:: changelog/2.2.x.rst.inc
.. include:: changelog/2.1.x.rst.inc
.. include:: changelog/2.0.x.rst.inc
.. include:: changelog/1.5.x.rst.inc
.. include:: changelog/1.4.x.rst.inc
.. include:: changelog/1.3.x.rst.inc
.. include:: changelog/1.2.x.rst.inc
.. include:: changelog/1.1.x.rst.inc
.. include:: changelog/1.0.x.rst.inc
.. include:: changelog/0.4.x.rst.inc
.. include:: changelog/0.3.x.rst.inc
