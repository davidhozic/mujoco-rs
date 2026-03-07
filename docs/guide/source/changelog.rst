.. _changelog:

==================
Changelog
==================

.. |mj_data| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
.. |mj_model| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`
.. |mj_spec| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
.. |mj_geomview| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataView`
.. |mj_geomviewmut| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjGeomDataViewMut`
.. |mjv_scene| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene`
.. |mj_vfs| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs`
.. |mjs_tendon| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsTendon`
.. |mjs_wrap| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsWrap`
.. |mjv_camera| replace:: :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`


Versioning
=================
This project uses `semantic versioning <https://semver.org/>`_.
This means that any incompatible changes increase the major version (**Y**.x.x).
This also includes breaking changes that MuJoCo itself introduced, thus even an
update of MuJoCo alone can increase the major version.

3.0.0 (MuJoCo 3.5.0)
======================

- **Breaking changes**:

  - Updated the MuJoCo version to 3.5.0.
  - Regenerated C FFI bindings to match MuJoCo 3.5.0.
  - Updated Rust API wrappers for compatibility with MuJoCo 3.5.0.
  - Many methods that previously returned bare types or ``io::Result`` now return typed
    ``Result`` variants for safer error handling. See the *Error handling* section below
    for the full list.
  - |mj_data|:
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>constraint_update`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body_com`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_subtree_com`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_geom`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_site`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>angmom_mat`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_velocity`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_acceleration`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>geom_distance`,
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>local_to_global`
    now return ``Result<_, MjDataError>`` instead of bare types.
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>add_contact`
    now returns ``Result<(), MjDataError>`` instead of ``io::Result<()>``.
  - |mj_data|: :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray` gained
    a new ``normal_out`` parameter (``Option<&mut [MjtNum; 3]>``).
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray` gained a new
    ``normals_out`` parameter and now returns ``Result<_, MjDataError>``
    instead of a bare tuple.
  - |mj_data|: :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>contact_force`
    now takes ``contact_id: u32`` instead of ``usize``.
  - |mj_model|:
    :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>size` now
    returns ``MjtSize`` instead of ``i32``.
    :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size` now
    returns ``usize`` instead of ``i32``.
  - |mjv_scene|:
    :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom`
    now returns ``Result<&mut MjvGeom, MjSceneError>`` instead of ``&mut MjvGeom``.
    :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>set_label`
    now returns ``Result<(), MjSceneError>``.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`:
    :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_aux`
    now return ``Result<(), MjSceneError>``.
  - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer`:
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb`,
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth`,
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`,
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth`,
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw` now return
    ``Result<_, RendererError>`` instead of ``io::Result``.
  - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now returns
    ``Result<(), MjViewerError>`` instead of ``()``.
  - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` is a new fallible
    alternative to :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>sync`,
    returning ``Result<(), RendererError>``.
    :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` has a new ``SignatureMismatch`` variant
    and now also propagates OpenGL context errors from the internal ``render()`` call.
  - |mj_model| now implements the ``Clone`` trait (panics on failure).
    The previous ``clone()`` that returned ``Option<MjModel>`` is replaced by
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_clone`
    returning ``Result<MjModel, MjDataError>``.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`
    now takes ``selection_xyz`` by reference (``&[MjtNum; 3]``) instead of by value.
  - Removed deprecated methods:

    - |mj_data|: ``warning_stats``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>warning` instead),
      ``timer_stats``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>timer` instead),
      type aliases ``MjJointInfo``, ``MjJointView``, ``MjJointViewMut``,
      ``MjGeomInfo``, ``MjGeomView``, ``MjGeomViewMut``,
      ``MjActuatorInfo``, ``MjActuatorView``, ``MjActuatorViewMut``.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjJointDataViewMut`: ``reset``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataViewMut::<method>zero` instead).
    - |mj_model|: ``name2id``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>name_to_id` instead).
    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`: ``new``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_free`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_tracking`,
      or :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_user` instead).
    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`: ``figure``
      (use :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>draw` instead).
    - :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`: ``user_scene``, ``user_scene_mut``,
      ``user_scn``, ``user_scn_mut``, ``sync``.

- New methods:

  - |mj_spec|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_parse` and
      :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_parse_vfs`, which
      wrap :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_parse`.

  - |mj_data|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>forward_kinematics`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray_flex`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_state_from_data`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>apply_ft`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray_mesh`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>ray_hfield`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>init_ctrl_history`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>init_sensor_history`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_ctrl`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_sensor_into`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_sensor_fixed`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_sensor`.

  - |mj_model|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>extract_state` and
      :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>extract_state_into`.

  - |mj_vfs|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>mount`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>unmount`.

  - |mjs_tendon|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsTendon::<method>get_wrap`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsTendon::<method>get_wrap_mut`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsTendon::<method>get_wrap_num`.

  - |mjs_wrap|:

    - Added getters for
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsWrap::<method>coef`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsWrap::<method>divisor`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsWrap::<method>side_site`, and
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsWrap::<method>side_site_mut`.

  - |mjv_camera|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frame` and
      :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>frustum`.

- Changed methods:

  - |mj_data|:

    - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_threadstack`
      now returns ``&[MjtSize; mjMAXTHREAD]`` instead of ``&[MjtSize]``.

  - |mj_model|:

    - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_buffer`
      now calls :docs-rs:`~~mujoco_rs::mujoco_c::<fn>mj_loadModelBuffer` directly.
      A virtual file-system was previously used to support this.

- Error handling:

  - New error types in the :docs-rs:`~mujoco_rs::error` module:
    :docs-rs:`~mujoco_rs::error::<enum>MjDataError`,
    :docs-rs:`~mujoco_rs::error::<enum>MjSceneError`,
    :docs-rs:`~mujoco_rs::error::<enum>MjEditError`,
    :docs-rs:`~mujoco_rs::error::<enum>MjModelError`,
    :docs-rs:`~mujoco_rs::error::<enum>MjVfsError`,
    :docs-rs:`~mujoco_rs::error::<enum>GlInitError`.
    All are ``#[non_exhaustive]`` and re-exported from the prelude.

  - |mj_model|:

    - Methods
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_vfs`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_string`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_buffer`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_last_xml`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>extract_state`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>extract_state_into`
      now return typed :docs-rs:`~mujoco_rs::error::<enum>MjModelError` variants instead of ``io::Error``.

  - |mj_vfs|:

    - Methods
      :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>add_from_file`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>add_from_buffer`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>delete_file`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>mount`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>unmount`
      now return typed :docs-rs:`~mujoco_rs::error::<enum>MjVfsError` variants instead of ``io::Error``.

  - |mj_data|:

    - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>add_contact`
      now returns :docs-rs:`~mujoco_rs::error::<enum>MjDataError` instead of ``io::Error``.
    - Methods
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>constraint_update`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_body_com`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_subtree_com`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_geom`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>jac_site`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>angmom_mat`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_velocity`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>object_acceleration`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>geom_distance`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>local_to_global`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>multi_ray`
      now return typed :docs-rs:`~mujoco_rs::error::<enum>MjDataError` variants instead of
      bare (panicking) types.

  - |mj_spec|:

    - :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml_vfs`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>from_xml_string`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>compile`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>save_xml`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>save_xml_string`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>add_default`
      now return :docs-rs:`~mujoco_rs::error::<enum>MjEditError` instead of ``io::Error``.

  - |mjv_scene| / :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`:

    - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>create_geom`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>set_label`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>add_aux`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_aux`
      now return typed :docs-rs:`~mujoco_rs::error::<enum>MjSceneError` variants instead of panicking.

  - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer`:

    - Methods
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb`,
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth`,
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb`,
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth`,
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw`
      now return typed :docs-rs:`~mujoco_rs::renderer::<enum>RendererError` variants instead of ``io::Error``.

  - :docs-rs:`~mujoco_rs::renderer::<enum>RendererError`:

    - Gained new variants: ``RgbDisabled``, ``DepthDisabled``, ``DimensionMismatch``,
      ``ZeroDimension``, ``IoError``, ``SceneError``, ``GlInitFailed``, ``SignatureMismatch``.
    - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>try_sync` now propagates
      scene-full errors as ``RendererError::SceneError`` instead of panicking.

  - :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError`:

    - Gained new variants: ``PainterInitError``, ``SwapBuffersError``, ``GlInitFailed``,
      ``SceneError``.
    - :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render` now propagates
      scene-full errors as ``MjViewerError::SceneError`` instead of panicking.

  - :docs-rs:`~mujoco_rs::error::<enum>GlInitError` replaces opaque ``String`` errors
    for OpenGL context initialization failures in the renderer and viewer.
  - New fallible ``try_`` methods:

    - |mj_data|:
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_new`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_clone`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_copy_state_from_data`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_apply_ft`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_read_state_into`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_set_state`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_copy_visual_to`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>try_copy_to`.
    - |mj_model|:
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_clone`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>try_make_data`.
    - |mj_spec| and editing types:
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>try_new`,
      ``try_add_frame``, ``try_wrap_site``,
      ``try_wrap_geom``, ``try_wrap_joint``, ``try_wrap_pulley``, and macro-generated
      ``try_add_*`` methods.
    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure`:
      :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>try_push`,
      :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvFigure::<method>try_set_at`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext`:
      :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>try_read_pixels`.

  - Infallible counterparts now delegate to their ``try_`` variants via ``.expect()``,
    preserving backward compatibility.

- Internal / Safety changes:

  - Fixed potential undefined behavior when parsing string pointers returned from MuJoCo's model editing C++ wrappers
    via ``CStr::from_ptr`` by properly checking for NULL pointers.
  - Added a strict runtime signature check to view creation methods
    (e.g., :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataInfo::<method>view`)
    which will now panic if the |mj_data| was not created from an |mj_model| with the exact same
    kinematic tree structure.
  - Views now use more strict type checks. Specifically, instead of casting all the C types to
    the declared Rust type, only a strict subset of fields are now cast (enums and bools).
    Other fields must match the C type exactly.

  - Module :docs-rs:`~mujoco_rs::mujoco_c` now uses compile-time layout tests to ensure
    declarations match the platform ABI of the linked MuJoCo library.

  - Replaced several ``unsafe`` type-punning blocks with direct pointer casts
    where type compatibility is trivially provable:

    - :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer`:
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb`,
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>depth`, and
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_depth_raw`
      now use direct pointer casts with a runtime length check, replacing the previous
      unsafe casts that lacked length validation.
    - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>save_last_xml` and
      :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>label` now use
      direct ``i8`` to ``u8`` pointer casts (identical layout) instead of unsafe
      ``from_raw_parts`` without documented invariants.
    - ``write_mjs_vec_byte`` and
      :docs-rs:`~~mujoco_rs::wrappers::mj_editing::<type>MjsTexture::<method>set_data`
      now require ``T: bytemuck::NoUninit``
      for compile-time verified byte reinterpretation instead of unsafe pointer casts.

  - Replaced ``force_cast`` union transmutes with ``as *const _ as *mut _`` for
    ``&T`` to ``*mut T`` pointer casts in :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsWrap`
    methods and body-item iterators, making
    the const-to-mut intent explicit without a union-based transmute.

  - Added compile-time ``const`` assertions in the viewer UI to verify that
    ``GL_EFFECT_MAP``, ``VIS_OPT_MAP``, ``LABEL_TYPE_MAP``, and ``FRAME_TYPE_MAP``
    lengths match the corresponding MuJoCo sentinel values (``mjNRNDFLAG``,
    ``mjNVISFLAG``, ``mjNLABEL``, ``mjNFRAME``). Previously these were only
    checked at runtime via ``debug_assert_eq!``.

  - Replaced hardcoded index bound ``10`` with ``mjNAUX as usize`` in
    :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>set_aux`
    so the check stays correct if MuJoCo changes the constant.

  - Added a compile-time assertion that ``HELP_MENU_TITLES`` and ``HELP_MENU_VALUES``
    have the same number of newline-delimited entries, preventing silent misalignment
    of the viewer help overlay.

  - :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel` now uses
    ``NonNull<mjModel>`` instead of ``*mut mjModel`` internally, and
    :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData` uses ``NonNull<mjData>``.
    This encodes the non-null invariant at the type level, eliminating a class of possible
    null-pointer dereferences and enabling niche optimization for ``Option<MjModel>`` etc.

  - Added ``unsafe impl bytemuck::Zeroable`` for all MuJoCo C enum types used in view structs.
    The ``zero()`` method in ``info_with_view!`` now calls ``bytemuck::Zeroable::zeroed()``
    (safe) instead of ``std::mem::zeroed()`` (unsafe). Any future type used in views without
    a ``Zeroable`` impl will produce a compile error.

  - Added a compile-time assertion that ``MjtCamera``'s ``TryFrom<i32>`` discriminant values
    match the actual enum variant values, catching silent drift if MuJoCo renumbers them.

  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>label` and
    :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvGeom::<method>set_label`
    now use ``bytemuck::cast_slice`` / ``bytemuck::cast_slice_mut`` for safe
    ``i8``-to-``u8`` conversion instead of raw pointer casts, eliminating ``unsafe`` blocks.

  - Replaced ``unsafe { CStr::from_ptr(...) }`` with safe ``CStr::from_bytes_until_nul``
    across error-buffer handling in |mj_model|, |mj_spec|, and the ``c_str_as_str_method!``
    macro. This eliminates unbounded NUL scans and removes several ``unsafe`` blocks.
    ``check_raw_model`` and ``check_spec`` are now safe functions.

  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvPerturb::<method>update_local_pos`
    now uses bounds-checked slice indexing into |mj_data| for ``xpos`` and ``xmat`` instead of
    raw pointer arithmetic. An invalid ``select`` index now triggers a ``debug_assert!`` and
    panics via Rust's bounds checker instead of causing undefined behavior.
    In release mode, the slice index will panic instead of the ``debug_assert!``.

- Other changes:

  - Updated enum type aliases.
  - Improved fixed-size array pointer handling.
  - Added support for ``Depth`` rendering flag (``mjRND_DEPTH``) to
    :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`.
  - Added a "Print camera" button to the viewer UI that prints the current camera's position and
    orientation as an MJCF ``<camera>`` XML element to the console.
  - |mj_data| and |mj_model| views now contain extra fields.
  - Fixed a bug where :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` methods like
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>save_rgb` and
    :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>rgb_flat` would output
    vertically flipped (upside down) images.
  - Added new examples based on the `MuJoCo Python tutorial <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb>`_:

    - :gh-example:`Self-inverting tippe-top <tippe_top.rs>`: Simulates the self-inverting tippe-top using the RK4 integrator and
      keyframe initialization; displays the inversion in an interactive :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`.
    - :gh-example:`Chaotic pendulum <chaotic_pendulum.rs>`: Demonstrates the butterfly effect in a four-body pendulum,
      energy tracking via :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy_pos`
      and :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy_vel`, and
      interactive visualization.
    - :gh-example:`Contact forces <contact_forces.rs>`: Shows contact-point and contact-force visualization using
      :docs-rs:`~~mujoco_rs::renderer::<struct>MjRenderer::<method>opts_mut` to enable
      ``mjVIS_CONTACTPOINT``, ``mjVIS_CONTACTFORCE``, and ``mjVIS_TRANSPARENT`` flags;
      saves rendered frames as PNG images.

- :ref:`mj_cpp_viewer`:

  - **Breaking**: :docs-rs:`~~mujoco_rs::cpp_viewer::<struct>MjViewerCpp::<method>render` no longer
    accepts the boolean parameter ``update_timer``. The FPS timer is now always updated.
    It also now returns ``Result<(), &'static str>`` instead of ``()``.

2.3.5 (MuJoCo 3.3.7)
=============================
- Fixed `#161 <https://github.com/davidhozic/mujoco-rs/issues/161>`_, where
  actuator views could crash due to a non-muscle actuator appearing after
  the muscle actuator.
- Internal: Syncing user geoms is now slightly faster when no geoms are added,
  due to an early exit.

2.3.4 (MuJoCo 3.3.7)
=============================
- :ref:`mj_rust_viewer` bug fixes and usability improvements:

  - Keyboard events are now ignored whenever an egui input widget has focus.
  - Fixed a logic error where pressing an unhandled mouse button would return from the
    event-processing loop and drop subsequent events; the handler now continues.
  - The combination of Ctrl+C will no longer toggle camera visualization.
    This combination is reserved for copying.


2.3.3 (MuJoCo 3.3.7)
=============================
- Fixed `#144 <https://github.com/davidhozic/mujoco-rs/issues/144>`_, where the :ref:`mj_rust_viewer`
  caused high stack usage. This was potentially problematic on Windows, where the stack limit is 1M.

2.3.2 (MuJoCo 3.3.7)
=============================
- Small performance improvement by removing mutex contention during MuJoCo's scene render.

2.3.1 (MuJoCo 3.3.7)
=============================
- Bug fixes:

  - Added a NULL pointer check to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`.
    When allocation fails, the code will now panic instead of accepting a NULL pointer.
  - Fixed `#139 <https://github.com/davidhozic/mujoco-rs/issues/139>`_.
    :docs-rs:`~~mujoco_rs::wrappers::mj_rendering::<struct>MjrContext::<method>read_pixels`
    no longer allows buffers that are smaller than requested in the viewport.
  - Fixed potential (Rust) viewer crashes when the display is disconnected or reconfigured by
    ignoring some OpenGL errors. In case of an error at specific places, the errors will be
    ignored and viewer processing will be skipped.

2.3.0 (MuJoCo 3.3.7)
=============================
- Changes to :ref:`mj_rust_viewer`:

  - Added :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback_detached` for custom UI widgets that
    do not require the simulation state (i.e., the passive |mj_data| instance).
  - Added :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>with_ui_egui_ctx`, for gaining scoped access
    to the internal egui's context. This can be used for additional, one-time,
    initialization (e.g., installing image loaders).
  - Replaced code that would panic on poisoned mutexes with code that automatically unpoisons the mutex
    --- nothing fundamentally bad can happen except for a potential glitch in physics, which is still better
    than requiring users to wrap their code in
    `catch_unwind <https://doc.rust-lang.org/std/panic/fn.catch_unwind.html>`_ calls.
  - Performance optimizations in the viewer UI.

- |mj_model| and |mj_spec|:

  - When instantiating a model or a spec, a version check will be made between MuJoCo's shared library and
    the FFI bindings to prevent accidental use of the wrong MuJoCo version. The code will panic on a mismatch.

2.2.2 (MuJoCo 3.3.7)
================================
- Bug fixes:

  - Fixed `incorrect calculation of depth values <https://github.com/davidhozic/mujoco-rs/pull/125>`_.

2.2.1 (MuJoCo 3.3.7)
================================
- Bug fixes:

  - Fixed `#119 <https://github.com/davidhozic/mujoco-rs/issues/119>`_ where the borrow tracker
    tracked the wrong lifetime.
  - Reset more of the OpenGL state after drawing the UI with egui (drawing figures doesn't work without this).

2.2.0 (MuJoCo 3.3.7)
================================
- Made :ref:`mj_renderer` use the EGL backend by default on Linux for true offscreen rendering (pbuffer).

  - New feature added for compatibility purposes: ``renderer-winit-fallback``.
    The feature is enabled by default.

- Changes to :ref:`mj_rust_viewer`:

  - Added :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>add_ui_callback` for custom UI widgets.
    This allows users to create custom windows, panels, and other UI elements using `egui <https://github.com/emilk/egui>`_.
    See :ref:`custom_ui_widgets` for more information.

  - Info menu (F2) displaying smoothed FPS, simulation time, and used memory.
  - Realtime warning (F4) for showing the realtime factor [%] when the simulation's sync time deviates by at least 2 %
    from the model's configured timestep.

  - Separate syncing and rendering logic:

    - Added :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>sync_data`:

      - Deprecates ``sync()``, which performed
        both synchronization and rendering.
      - Rendering must now be done through :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>render`,
        which must be called by user code.

    - Added :docs-rs:`~mujoco_rs::viewer::<struct>ViewerSharedState`:

      - Allows synchronization of the viewer and the simulation state in separate threads.
        Note that the viewer must run in the main thread.
      - Call :docs-rs:`~~mujoco_rs::viewer::<struct>MjViewer::<method>state` to obtain it.


- New methods:

  - |mj_data|:

    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_visual_to`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>copy_to`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>set_state`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>get_state`.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>read_state_into`.

- New examples:

  - :gh-example:`rust_viewer_threaded.rs`.

- Deprecated:

  - ``user_scene()`` and
    ``user_scene_mut()``.
    They are replaced with
    :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>user_scene` and
    :docs-rs:`~~mujoco_rs::viewer::<struct>ViewerSharedState::<method>user_scene_mut`.


2.1.0 / 2.1.1 (MuJoCo 3.3.7)
================================
- Option to automatically pull MuJoCo.
- pkg-config support for Linux and macOS. Note that this is not officially supported by MuJoCo.
- Changes to :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`:

  - Added an optional (enabled by default) user interface, made with `egui <https://github.com/emilk/egui>`_.

- New Cargo features:

  - ``auto-download-mujoco``: will automatically download MuJoCo (Windows and Linux).

2.0.1 (MuJoCo 3.3.7)
================================
- Fixed the ``renderer`` feature not enabling all the needed crates.

2.0.0 (MuJoCo 3.3.7)
================================
- **Breaking changes**:

  - Updated the MuJoCo version to 3.3.7.
  - :ref:`model_editing`:

    - Items (:docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsJoint`,
      :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsGeom`, etc.) are no longer wrapped and are instead
      just aliased types. Their attributes have been made private to users, so that tools like ``rust-analyzer``
      don't suggest both the getter name and the attribute name at once.
      As a result of this change, methods now return true references instead of the wrapper types.

    - Added immutable iterators.
    - Changed regular named-access methods to be immutable and added corresponding ``<item>_mut()`` methods
      for mutable access.
    - Replaced ``plugin_wrapper`` methods with a normal getter method (``plugin`` and ``plugin_mut``).

  - :ref:`mj_rust_viewer` and :ref:`mj_renderer`:

    - Changed the backend windowing library to Winit (+ Glutin). This is a **potentially** breaking
      change because of possible direct GLFW uses in the user code, which will probably still work
      as expected, but we can't be sure as we did not test GLFW and Winit being used at the same time.
      The change to Winit also means we don't need any C dependencies, unless the C++ viewer wrapper
      is needed, which also contains breaking changes. The latter is described in the next bullet.

    - Added and removed variants in :docs-rs:`~mujoco_rs::viewer::<enum>MjViewerError` and
      :docs-rs:`~mujoco_rs::renderer::<enum>RendererError`.

  - :ref:`mj_cpp_viewer`:

    - Since MuJoCo's build system downloads GLFW sources anyway, we decided to remove the GLFW
      requirement from the Rust level and instead made it so that the user needs to compile the GLFW
      code during MuJoCo's viewer (simulate) compilation.
      No change is needed in the user Rust code, users just need to build MuJoCo a bit differently:

      ``cmake --build build --parallel --target glfw libmujoco_simulate --config=Release``.

      The above command, besides the added ``glfw`` part, also contains the ``libmujoco_simulate``
      part in place of the previous ``libsimulate`` part. This change is a consequence
      of the MuJoCo upgrade to version 3.3.7.

    - Moved the struct definition from ``mujoco_rs::viewer`` to ``mujoco_rs::cpp_viewer``.

  - Changed |mj_data| and other types to accept a generic type for the model,
    constrained to ``Deref<Target = MjModel>``.
    This enables use in environments such as `PyO3 <https://github.com/PyO3/pyo3>`_.
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsMesh`: changed ``smoothnormal`` and ``needsdf`` to be treated as booleans.
  - |mj_data| methods:

    - Renamed ``crb`` to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>crb_comp` due to ``crb``
      now being a method that returns an immutable slice to the ``crb`` attribute of the FFI type.
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy` now returns a reference to a 2-element array instead of a slice.

  - |mj_model| methods:

    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>id_to_name` now accepts ``i32`` instead of ``c_int``,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>size` now returns ``i32`` instead of ``c_int``,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>state_size` now accepts ``u32`` instead of ``c_uint``
      and returns ``i32`` instead of ``c_int``.

  - :docs-rs:`mujoco_rs::mujoco_c`:

    - :docs-rs:`~mujoco_rs::mujoco_c::<enum>mjtSameFrame_` is now ``repr(u8)`` instead of ``repr(u32)``
      to fix alignment issues with MuJoCo's structs.

  - :docs-rs:`mujoco_rs::wrappers::fun::utility`:

    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_band_diag`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_eig_3`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_halton`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_is_bad`: replaced ``c_int`` types with ``bool``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_mat_2_rot`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_ray_geom`: replaced ``c_int`` types with :docs-rs:`~mujoco_rs::wrappers::mj_model::<type>MjtGeom`,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_round`: replaced ``c_int`` types with ``i32``,
    - :docs-rs:`~mujoco_rs::wrappers::fun::utility::<fn>mju_transform_spatial`: replaced ``c_int`` types with ``bool``.

  - Removed modules:

    - ``mujoco_rs::wrappers::mj_interface``: this was in early development, but then it became apparent
      that its completion and usage would violate borrow checker rules, resulting in undefined behavior.

- Other changes:

  - Any changes made in MuJoCo 3.3.6 and MuJoCo 3.3.7 (see https://mujoco.readthedocs.io/en/3.3.7/changelog.html).
  - Added additional getters / setters / array slice methods to:

    - |mj_data|,
    - |mj_model|,
    - |mjv_scene|.

1.5.0 (MuJoCo 3.3.5)
================================
- |mjv_scene|:

  - Added the :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<struct>MjvScene::<method>pop_geom` method
    for popping a single geom from the end of the scene.

- :ref:`model_editing`:

  - Added iterators to :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`
    and :docs-rs:`~mujoco_rs::wrappers::mj_editing::<type>MjsBody`.


1.4.2 (MuJoCo 3.3.5)
================================
Bug fixes:

- Fixed segmentation fault when the model specification is invalid. (`#65 <https://github.com/davidhozic/mujoco-rs/issues/65>`_).

1.4.1 (MuJoCo 3.3.5)
================================
Bug fixes:

- Added missing named accessors to :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec`:

  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>geom`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>site`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>camera`,
  - :docs-rs:`~mujoco_rs::wrappers::mj_editing::<struct>MjSpec::<method>light`.

1.4.0 (MuJoCo 3.3.5)
================================
- |mj_model|:

  - Added more views:
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>key` (keyframe),
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>tuple`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>texture`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>site`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>pair`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>numeric`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>material`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>light`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>hfield`,
    :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>equality`.

- Added extra renames of enum types from the C API.

- :ref:`model_editing` support, which can be used to procedurally generate |mj_model|. It can be used
  for terrain generation, parameter randomization, etc.

  - Added module :docs-rs:`mujoco_rs::wrappers::mj_editing`.
  - Added two examples. One on basic model editing and one on terrain generation.

- :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer`:

  - Added additional getters and setters.
  - Added :docs-rs:`~mujoco_rs::renderer::<struct>MjRendererBuilder` for purposes of better
    configuration.

- :docs-rs:`~mujoco_rs::viewer::<struct>MjViewer`:

  - Added events for keys:

    - ``Backspace``: resets the simulation,
    - ``F5``: toggles full screen mode,
    - ``[`` and ``]``: cycles cameras,
    - Visualization toggles:

      - ``C``: camera,
      - ``U``: actuator,
      - ``J``: joint,
      - ``M``: center of mass,
      - ``H``: convex hull,
      - ``Z``: light,
      - ``T``: transparent,
      - ``I``: inertia.

  - Increased the headroom for visual-only geoms, which aren't drawn by the user, from 100 to 2000.

- :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera`:

  - Added methods:

    - :docs-rs:`~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>fix`:
      changes the camera struct parameters to display a fixed camera.

1.3.0 (MuJoCo 3.3.5)
================================
- Added a module for offscreen scene rendering: :docs-rs:`mujoco_rs::renderer`:

  - Added :docs-rs:`~mujoco_rs::renderer::<struct>MjRenderer` for actual offscreen rendering of the simulation state.

- Deprecated ``MjvCamera::new()`` and replaced it with:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_free`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_fixed`,
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_tracking` and
  - :docs-rs:`~~mujoco_rs::wrappers::mj_visualization::<type>MjvCamera::<method>new_user`.

- Deprecated ``user_scn()`` and
  ``user_scn_mut()``. They are replaced with
  ``user_scene()`` and
  ``user_scene_mut()``.

- Added new methods for obtaining public attributes:

  - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`:

    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_stack`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>maxuse_threadstack`,
    - ``warning_stats()``,
    - ``timer_stats()``,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>time`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData::<method>energy`.

  - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`:

    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>signature`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>opt`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>opt_mut`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>vis`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>vis_mut`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>stat`,
    - :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>stat_mut`.

- Added extra attributes to the :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`'s joint view:

  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_spring`.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_damper`.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_gravcomp`.
  - :docs-rs:`~~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView::<structfield>qfrc_fluid`.


1.2.0 (MuJoCo 3.3.5)
================================
- Added function wrappers around some utility functions and both the derivative functions (available under :docs-rs:`mujoco_rs::wrappers::fun`).
- Completed the virtual file system wrapper.

  - Added methods :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>add_from_file` and :docs-rs:`~~mujoco_rs::wrappers::mj_auxiliary::<struct>MjVfs::<method>delete_file`.
  - Added method :docs-rs:`~~mujoco_rs::wrappers::mj_model::<struct>MjModel::<method>from_xml_vfs`.
  - Added a long list of additional methods to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjData`
    and :docs-rs:`~mujoco_rs::wrappers::mj_model::<struct>MjModel`, which wrap corresponding MuJoCo functions.
    See the Git diff on GitHub for more information.

1.1.0 (MuJoCo 3.3.5)
=====================
**Potentially breaking changes:**

- Fixed bug `#18 <https://github.com/davidhozic/mujoco-rs/issues/18>`_ where data races could occur
  under incorrect usage. The major version of MuJoCo-rs is not increased as these safety bugs
  should not be something to rely on.

Other bug fixes:

- Fixed bug `#17 <https://github.com/davidhozic/mujoco-rs/issues/17>`_ where the |mj_geomview| and |mj_geomviewmut|
  pointed to the wrong address, which belonged to the body and not the geom.
- Fixed bug `#19 <https://github.com/davidhozic/mujoco-rs/issues/19>`_ where a warning about the scene buffer
  would be printed when loading some of MuJoCo's example scenes.


Other changes:

- Added new module: :docs-rs:`mujoco_rs::wrappers::mj_primitive`.
- Added more attributes to :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjJointDataView`
  and :docs-rs:`~mujoco_rs::wrappers::mj_data::<struct>MjJointDataViewMut`.
- Added more views. All views now available:
    - |mj_data|: actuator, body, camera, geom, joint, light, sensor, site, tendon.
    - |mj_model|: actuator, body, camera, geom, joint, sensor, tendon.

1.0.1 (MuJoCo 3.3.5)
=====================
Bug fixes:

- Smaller changes inside Drop implementations to make sure there is no undefined behavior.

1.0.0 (MuJoCo 3.3.5)
=====================
**Breaking changes:**

- Made all ``ffi_mut()`` methods require unsafe blocks.

Viewer:

- Help overlay (F1)
- User scene via ``user_scn()`` and
  ``user_scn_mut()`` for drawing custom visual-only geoms.
- Mouse perturbation of objects:

  - Rotate via Control
  - Translate via Control + Alt

0.4.3 (MuJoCo 3.3.5)
=====================
Build system:

- Removed unnecessary header files, reducing crate's file size.

0.4.2 (MuJoCo 3.3.5)
=====================
Build system:

- Improved clarity of environment variables:

  - ``MUJOCO_DYNAMIC_LINK_LIB`` -> ``MUJOCO_DYNAMIC_LINK_DIR``
  - ``MUJOCO_STATIC_LINK_LIB`` -> ``MUJOCO_STATIC_LINK_DIR``

- Added some internal cargo features.

0.4.1 (MuJoCo 3.3.5)
=====================
- Fixed event handling.

0.4.0 (MuJoCo 3.3.5)
=====================
- Changed the package name to ``mujoco-rs``.

0.3.0 (MuJoCo 3.3.5)
=====================
- Initial public release (previously private under a different project).
