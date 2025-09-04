# Changelog

## 2.0.0 (MuJoCo 3.3.5)
**Breaking changes:**
- Fixed bug [#18](https://github.com/davidhozic/mujoco-rs/issues/18) where data races could occur
  under incorrect usage.

Other bug fixes:
- Fixed bug [#17](https://github.com/davidhozic/mujoco-rs/issues/17) where the `MjGeomView` and `MjGeomViewMut`
  pointed to the wrong address, which belonged to the body and not the geom.

Other changes:
- Added new modules: `wrappers::mj_primitive`.
- Added more attributes to the view to `MjJointView[Mut]`.

## 1.0.1 (MuJoCo 3.3.5)
Bug fixes:
- Smaller changes inside Drop implementations to make sure there is no undefined behaviors.
## Versioning
This project uses [semantic versioning](https://semver.org/):
- Breaking changes: major version bump (e. g. 1.0.0 -> 2.0.0),
- Non-breaking feature changes: minor version bump (e. g. 1.0.0 -> 1.1.0),
- Non-breaking bug fixes: patch version bump (e. g. 1.0.0 -> 1.0.1).


## 1.0.0 (MuJoCo 3.3.5)
**Breaking changes:**
- Made all `ffi_mut()` methods require unsafe blocks.

Viewer:
- Help overlay (F1)
- User scene via `user_scn` and `user_scn_mut` for drawing custom visual-only geoms.
- Mouse perturbation of objects:
    - Rotate via Control
    - Translate via Control + Alt

## 0.4.3 (MuJoCo 3.3.5)
Build system:
- Removed unnecessary header files, reducing crate's file size.

## 0.4.2 (MuJoCo 3.3.5)
Build system:
- Improved clarity of environmental variables:
    - `MUJOCO_DYNAMIC_LINK_LIB` -> `MUJOCO_DYNAMIC_LINK_DIR`
    - `MUJOCO_STATIC_LINK_LIB` -> `MUJOCO_STATIC_LINK_DIR`
- Added some internal cargo features .

## 0.4.1 (MuJoCo 3.3.5)
- Fix event handling.

## 0.4.0 (MuJoCo 3.3.5)
- Change the package name to `mujoco-rs`.

## 0.3.0 (MuJoCo 3.3.5)
- Initial public release (previously private under a different project).
