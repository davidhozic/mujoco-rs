# Changelog

## 1.0.1 (MuJoCo 3.3.5)
Bug fixes:
- Smaller changes inside Drop implementations to make sure there is no undefined behaviors.

## 1.0.0 (MuJoCo 3.3.5)
Breaking changes:
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
