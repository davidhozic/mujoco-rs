1.3.0 (MuJoCo 3.3.5)
================================
- Added :docs-rs:`mujoco_rs::renderer` module with ``MjRenderer``.
- Deprecated ``MjvCamera::new()``; replaced by ``new_free``, ``new_fixed``,
  ``new_tracking``, ``new_user``.
- Added ``MjData::maxuse_stack``, ``maxuse_threadstack``, ``warning_stats``,
  ``timer_stats``, ``time``, ``energy``.
- Added ``MjModel::signature``, ``opt`` / ``opt_mut``, ``vis`` / ``vis_mut``,
  ``stat`` / ``stat_mut``.
- Added joint view attributes: ``qfrc_spring``, ``qfrc_damper``,
  ``qfrc_gravcomp``, ``qfrc_fluid``.

