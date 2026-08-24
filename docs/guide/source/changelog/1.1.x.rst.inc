1.1.0 (MuJoCo 3.3.5)
=====================
- Fixed data-race bug `#18 <https://github.com/davidhozic/mujoco-rs/issues/18>`_.
- Fixed |mj_geomview| / |mj_geomviewmut| pointing to wrong address
  (`#17 <https://github.com/davidhozic/mujoco-rs/issues/17>`_).
- Fixed spurious scene-buffer warning on some models
  (`#19 <https://github.com/davidhozic/mujoco-rs/issues/19>`_).
- Added :docs-rs:`mujoco_rs::wrappers::mj_primitive` module.
- More |mj_data| views: actuator, body, camera, geom, joint, light, sensor, site, tendon.
- More |mj_model| views: actuator, body, camera, geom, joint, sensor, tendon.

