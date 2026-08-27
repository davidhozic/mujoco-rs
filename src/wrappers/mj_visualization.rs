//! Definitions related to visualization.
use std::default::Default;
use std::mem::MaybeUninit;
use std::ops::Deref;
use std::ptr;

use super::mj_rendering::{MjrContext, MjrRectangle};
use super::mj_primitive::{MjtNum, MjtByte, MjtSize};
use super::mj_model::{MjModel, MjtGeom, MjtObj};
use super::mj_data::MjData;
use crate::{array_slice_dyn, c_str_as_str_method};
use crate::error::MjSceneError;
use crate::getter_setter;
use crate::mujoco_c::*;

/// Result of a mouse-based selection query via [`MjvScene::find_selection`].
#[derive(Debug, Clone, PartialEq)]
pub struct SceneSelection {
    /// Selected body id, or `None` if nothing was selected.
    pub body_id: Option<usize>,
    /// Selected geom id, or `None` if nothing was selected.
    pub geom_id: Option<usize>,
    /// Selected flex id, or `None` if nothing was selected.
    pub flex_id: Option<usize>,
    /// Selected skin id, or `None` if nothing was selected.
    pub skin_id: Option<usize>,
    /// 3D world coordinates of the selection point; `[0.0; 3]` when nothing was selected.
    pub point: [MjtNum; 3],
}

impl Default for SceneSelection {
    fn default() -> Self {
        Self {
            body_id: None,
            geom_id: None,
            flex_id: None,
            skin_id: None,
            point: [0.0; 3],
        }
    }
}

/* Types */
/// These are the available categories of geoms in the abstract visualizer. The bitmask selects the
/// categories that [`MjvScene::update_with_catmask`] adds to the scene.
pub type MjtCatBit = mjtCatBit;

/// These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
/// and translate them into these actions, as illustrated in MuJoCo's `simulate` application.
pub type MjtMouse = mjtMouse;

/// These bitmasks enable the translational and rotational components of the mouse perturbation. For the regular mouse,
/// only one can be enabled at a time. For the 3D mouse (SpaceNavigator) both can be enabled simultaneously. They are used
/// in `mjvPerturb.active`.
pub type MjtPertBit = mjtPertBit;

/// These are the possible camera types, used in `mjvCamera.type`.
pub type MjtCamera = mjtCamera;

// Compile-time verification that TryFrom discriminant values match the actual enum variants.
const _: () = {
    assert!(MjtCamera::mjCAMERA_FREE as i32 == 0);
    assert!(MjtCamera::mjCAMERA_TRACKING as i32 == 1);
    assert!(MjtCamera::mjCAMERA_FIXED as i32 == 2);
    assert!(MjtCamera::mjCAMERA_USER as i32 == 3);
};

impl TryFrom<i32> for MjtCamera {
    type Error = MjSceneError;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::mjCAMERA_FREE),
            1 => Ok(Self::mjCAMERA_TRACKING),
            2 => Ok(Self::mjCAMERA_FIXED),
            3 => Ok(Self::mjCAMERA_USER),
            _ => Err(MjSceneError::InvalidCameraType(value))
        }
    }
}

/// These are the abstract visualization elements that can have text labels. Used in `mjvOption.label`.
pub type MjtLabel = mjtLabel;

/// These are the MuJoCo objects whose spatial frames can be rendered. Used in `mjvOption.frame`.
pub type MjtFrame = mjtFrame;

/// These are indices in the array `mjvOption.flags`, whose elements enable/disable the visualization of the
/// corresponding model or decoration element.
pub type MjtVisFlag = mjtVisFlag;

/// These are indices in the array `mjvScene.flags`, whose elements enable/disable OpenGL rendering effects.
pub type MjtRndFlag = mjtRndFlag;

/// These are the possible stereo rendering types. They are used in `mjvScene.stereo`.
pub type MjtStereo = mjtStereo;
/**********************************************************************************************************************/

/***********************************************************************************************************************
** MjvPerturb
***********************************************************************************************************************/
/// Mouse perturbation state (selected body/flex/skin, interaction mode, reference position/orientation, local position).
pub type MjvPerturb = mjvPerturb;
impl Default for MjvPerturb {
    fn default() -> Self {
        // SAFETY: mjv_defaultPerturb initializes all fields to valid defaults;
        // the MaybeUninit pointer is valid and aligned.
        unsafe {
            let mut pert = MaybeUninit::uninit();
            mjv_defaultPerturb(pert.as_mut_ptr());
            pert.assume_init()
        }
    }
}

impl MjvPerturb {
    /// Initializes the perturbation state for mouse interaction of the given `type_`.
    /// Must be called before [`MjvPerturb::move_`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `self.select` names a body
    /// (`0 < select < nbody`) and the cameras of `scene` were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `self.flexselect` is greater than or equal to the number of flexes in the model
    /// of `data`.
    pub fn start<M: Deref<Target = MjModel>>(&mut self, type_: MjtPertBit, data: &mut MjData<M>, scene: &MjvScene) {
        // mjv_initPerturb indexes flex_rigid, flex_edgenum and flex_vertnum by flexselect after
        // testing only that it is not negative.
        let nflex = data.model().nflex();
        assert!(
            (self.flexselect as MjtSize) < nflex,
            "selected flex id {} is out of range for a model with {nflex} flexes", self.flexselect
        );
        let model_ffi = data.model().ffi();
        unsafe { mjv_initPerturb(model_ffi, data.ffi_mut(), scene.ffi(), self); }
        self.active = type_ as i32;
    }

    /// Move an object with mouse. Wraps [`mjv_movePerturb`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `action` is not one of
    /// [`MjtMouse::mjMOUSE_MOVE_V`], [`MjtMouse::mjMOUSE_MOVE_H`],
    /// [`MjtMouse::mjMOUSE_MOVE_V_REL`], [`MjtMouse::mjMOUSE_MOVE_H_REL`],
    /// [`MjtMouse::mjMOUSE_ROTATE_V`], [`MjtMouse::mjMOUSE_ROTATE_H`] or
    /// [`MjtMouse::mjMOUSE_ZOOM`], and when the cameras of `scene` were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `self.select` is out of range for the model in `data` (i.e. negative or
    /// `>= nbody`).
    pub fn move_<M: Deref<Target = MjModel>>(&mut self, data: &MjData<M>, action: MjtMouse, dx: MjtNum, dy: MjtNum, scene: &MjvScene) {
        // mjv_movePerturb dereferences d->xmat + 9*select (move-relative actions) and
        // d->xquat + 4*select / m->body_iquat + 4*select (rotate actions) *before* its own
        // sel range check, so an out-of-range id is an out-of-bounds read. Unlike the
        // mjv_updateScene path, the C function has no `select <= 0` early-out either, so the
        // negative case must be rejected here too.
        let nbody = data.model().nbody();
        assert!(
            self.select >= 0 && (self.select as MjtSize) < nbody,
            "selected perturbation body id {} is out of range for a model with {} bodies",
            self.select, nbody
        );
        unsafe { mjv_movePerturb(data.model().ffi(), data.ffi(), action as i32, dx, dy, scene.ffi(), self); }
    }

    /// Apply perturbation pose and force.
    ///
    /// # Note
    /// This method **zeroes `xfrc_applied`** for all bodies before applying the perturbation
    /// force. Any external forces set on `data` before calling this method will be cleared.
    /// If you need to preserve external forces, apply them *after* calling this method.
    pub fn apply<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>) {
        data.xfrc_applied_mut().fill([0.0; 6]);
        let model_ffi = data.model().ffi();
        unsafe { mjv_applyPerturbPose(model_ffi, data.ffi_mut(), self, 0); }
        let model_ffi = data.model().ffi();
        unsafe { mjv_applyPerturbForce(model_ffi, data.ffi_mut(), self); }
    }

    /// Updates the body-local position of the selection point.
    ///
    /// # Panics
    /// Panics if `self.select` is out of range for the `xpos`/`xmat` arrays (i.e., negative or
    /// `>= nbody`). In debug builds, a dedicated assertion fires first for the negative case.
    pub fn update_local_pos<M: Deref<Target = MjModel>>(&mut self, selection_xyz: &[MjtNum; 3], data: &MjData<M>) {
        debug_assert!(self.select >= 0, "invalid selecting when calling update_local_pos");
        let select = self.select as usize;
        let body_xpos = &data.xpos()[select];
        let body_xmat = &data.xmat()[select];
        // Inverse transform into the local frame of the body.
        // mju_sub3 equivalent
        let tmp = [
            selection_xyz[0] - body_xpos[0],
            selection_xyz[1] - body_xpos[1],
            selection_xyz[2] - body_xpos[2],
        ];
        // mju_mulMatTVec3 equivalent (mat is row-major 3x3)
        self.localpos = [
            body_xmat[0] * tmp[0] + body_xmat[3] * tmp[1] + body_xmat[6] * tmp[2],
            body_xmat[1] * tmp[0] + body_xmat[4] * tmp[1] + body_xmat[7] * tmp[2],
            body_xmat[2] * tmp[0] + body_xmat[5] * tmp[1] + body_xmat[8] * tmp[2],
        ];
    }
}



/***********************************************************************************************************************
** MjvCamera
***********************************************************************************************************************/
/// Abstract camera parameters (type, fixed/tracking ids, lookat, distance, azimuth and elevation
/// in degrees, orthographic mode).
pub type MjvCamera = mjvCamera;
impl MjvCamera {
    /// Creates a new free camera.
    /// By default, the camera will look at the center of the model.
    pub fn new_free(model: &MjModel) -> Self {
        let mut camera: mjvCamera_ = Self::default();
        unsafe { mjv_defaultFreeCamera(model.ffi(), &mut camera); }
        camera
    }

    /// Creates a new fixed camera.
    ///
    /// # Panics
    /// In debug builds, panics if `camera_id` exceeds `i32::MAX`.
    pub fn new_fixed(camera_id: usize) -> Self {
        debug_assert!(camera_id <= i32::MAX as usize, "camera_id exceeds i32::MAX");
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_FIXED as i32,
            fixedcamid: camera_id as i32,
            ..Self::default()
        }
    }

    /// Creates a new tracking camera to track a body with the given `tracking_id`.
    ///
    /// # Panics
    /// In debug builds, panics if `tracking_id` exceeds `i32::MAX`.
    pub fn new_tracking(tracking_id: usize) -> Self {
        debug_assert!(tracking_id <= i32::MAX as usize, "tracking_id exceeds i32::MAX");
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_TRACKING as i32,
            trackbodyid: tracking_id as i32,
            ..Self::default()
        }
    }

    /// Creates a new camera of user type.
    pub fn new_user() -> Self {
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_USER as i32,
            ..Self::default()
        }
    }

    /// Sets the camera into tracking mode.
    pub fn track(&mut self, tracking_id: usize) {
        self.type_ = MjtCamera::mjCAMERA_TRACKING as i32;
        self.fixedcamid = -1;
        self.trackbodyid = tracking_id as i32;
    }

    /// Sets the camera free from tracking.
    pub fn free(&mut self) {
        self.trackbodyid = -1;
        self.type_ = MjtCamera::mjCAMERA_FREE as i32;
    }

    /// Sets the camera to a fixed `camera_id`.
    pub fn fix(&mut self, camera_id: usize) {
        self.type_ = MjtCamera::mjCAMERA_FIXED as i32;
        self.fixedcamid = camera_id as i32;
        self.trackbodyid = -1;
    }

    /// Move camera with mouse.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `action` is not one of
    /// [`MjtMouse::mjMOUSE_ROTATE_V`], [`MjtMouse::mjMOUSE_ROTATE_H`],
    /// [`MjtMouse::mjMOUSE_MOVE_V`], [`MjtMouse::mjMOUSE_MOVE_H`], [`MjtMouse::mjMOUSE_TURN_V`],
    /// [`MjtMouse::mjMOUSE_TURN_H`], [`MjtMouse::mjMOUSE_ZOOM`], [`MjtMouse::mjMOUSE_MOVE_V_REL`]
    /// or [`MjtMouse::mjMOUSE_MOVE_H_REL`]. A camera of type [`MjtCamera::mjCAMERA_FIXED`] returns
    /// early, before that point.
    pub fn move_(&mut self, action: MjtMouse, model: &MjModel, dx: MjtNum, dy: MjtNum) {
        unsafe { mjv_moveCamera(model.ffi(), action as i32, dx, dy, self); };
    }

    /// Get the camera coordinate frame (pos, forward, up, right).
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, when `self.type_` is not
    /// [`MjtCamera::mjCAMERA_FREE`], [`MjtCamera::mjCAMERA_TRACKING`] or
    /// [`MjtCamera::mjCAMERA_FIXED`].
    ///
    /// # Panics
    /// Panics if this is a fixed camera (`MjtCamera::mjCAMERA_FIXED`) whose `fixedcamid` is out of
    /// range, or a tracking camera (`MjtCamera::mjCAMERA_TRACKING`) whose `trackbodyid` is greater
    /// than or equal to the number of bodies.
    pub fn frame<M: Deref<Target = MjModel>>(&self, data: &MjData<M>) -> ([MjtNum; 3], [MjtNum; 3], [MjtNum; 3], [MjtNum; 3]) {
        // mjv_cameraFrame indexes cam_xmat/cam_xpos by fixedcamid and subtree_com by trackbodyid
        // without an upper bound; no other branch indexes a model or data array.
        if self.type_ == MjtCamera::mjCAMERA_FIXED as i32 {
            let ncam = data.model().ncam();
            assert!(
                self.fixedcamid >= 0 && (self.fixedcamid as i64) < ncam,
                "fixed camera id {} is out of range for a model with {} cameras",
                self.fixedcamid, ncam
            );
        } else if self.type_ == MjtCamera::mjCAMERA_TRACKING as i32 && self.trackbodyid >= 0 {
            let nbody = data.model().nbody();
            assert!(
                (self.trackbodyid as i64) < nbody,
                "tracked body id {} is out of range for a model with {} bodies",
                self.trackbodyid, nbody
            );
        }

        let mut headpos = [0.0; 3];
        let mut forward = [0.0; 3];
        let mut up = [0.0; 3];
        let mut right = [0.0; 3];
        unsafe {
            mjv_cameraFrame(
                &mut headpos, &mut forward, &mut up, &mut right,
                data.ffi(), self
            );
        }
        (headpos, forward, up, right)
    }

    /// Compute the `frustum` (zver, zhor, zclip) suitable for rendering.
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, when `self.type_` is not
    /// [`MjtCamera::mjCAMERA_FREE`], [`MjtCamera::mjCAMERA_TRACKING`] or
    /// [`MjtCamera::mjCAMERA_FIXED`], or when `self.fixedcamid` is out of range for `model`.
    pub fn frustum(&self, model: &MjModel) -> ([f32; 2], [f32; 2], [f32; 2]) {
        let mut zver = [0.0; 2];
        let mut zhor = [0.0; 2];
        let mut zclip = [0.0; 2];
        unsafe {
            mjv_cameraFrustum(
                &mut zver, &mut zhor, &mut zclip,
                model.ffi(), self
            );
        }
        (zver, zhor, zclip)
    }
}

impl Default for MjvCamera {
    fn default() -> Self {
        // SAFETY: mjv_defaultCamera initializes all fields to valid defaults;
        // the MaybeUninit pointer is valid and aligned.
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjv_defaultCamera(c.as_mut_ptr());
            c.assume_init()
        }
    }
}

/***********************************************************************************************************************
** mjvGLCamera
***********************************************************************************************************************/
/// OpenGL camera parameters (position, forward/up vectors, frustum planes).
pub type MjvGLCamera = mjvGLCamera;

/// OpenGL camera parameters (position, forward/up vectors, frustum planes).
/// This is the same as [`MjvGLCamera`]; this alias follows MuJoCo's internal
/// type alias convention.
pub type MjrCamera = mjrCamera;

impl MjvGLCamera {
    /// Average the current MjvGLCamera with the `other` MjvGLCamera.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `self.orthographic` and
    /// `other.orthographic` differ.
    pub fn average_camera(&self, other: &Self) -> Self {
        unsafe { mjv_averageCamera (self, other) }
    }
}

/***********************************************************************************************************************
** MjvGeom
***********************************************************************************************************************/
/// Visual geometry element (type, size, material, RGBA, label, etc.) used in a scene.
pub type MjvGeom = mjvGeom;

impl MjvGeom {
    /// Sets the geom so that it acts as a connector (line, arrow, etc.) between
    /// two 3D points. `width` is the connector radius in length units, or its width in pixels for
    /// `mjGEOM_LINE`.
    ///
    /// Wraps [`mjv_connector`]. The connector type
    /// is taken from the geom's current [`type_`](MjvGeom::type_) field, so
    /// set it to the desired connector type (e.g. `mjGEOM_LINE`, `mjGEOM_ARROW`)
    /// **before** calling this method, or initialize the geom
    /// with that type via [`MjvScene::create_geom`].
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, for any type other than
    /// `mjGEOM_CAPSULE`, `mjGEOM_CYLINDER`, `mjGEOM_ARROW`, `mjGEOM_ARROW1`, `mjGEOM_ARROW2` or
    /// `mjGEOM_LINE`.
    pub fn connect(&mut self, width: MjtNum, from: [MjtNum; 3], to: [MjtNum; 3]) {
        unsafe {
            mjv_connector(self, self.type_, width, &from, &to);
        }
    }

    /// Compatibility method to convert the `label` attribute into a `String`.
    pub fn label(&self) -> String {
        let len = self.label.iter().position(|&c| c == 0).unwrap_or(self.label.len());
        // SAFETY: i8 and u8 have identical size (1) and alignment (1).
        let bytes: &[u8] = bytemuck::cast_slice(&self.label[..len]);
        String::from_utf8_lossy(bytes).to_string()
    }

    /// Writes `s` into the fixed-size label buffer, NUL-terminating it.
    /// # Errors
    /// Returns [`MjSceneError::NonAsciiLabel`] when `s` contains non-ASCII characters.
    /// Returns [`MjSceneError::LabelTooLong`] when `s` exceeds the buffer capacity
    /// (`self.label.len() - 1` bytes).
    pub fn set_label(&mut self, s: &str) -> Result<(), MjSceneError> {
        if !s.is_ascii() {
            return Err(MjSceneError::NonAsciiLabel);
        }
        let capacity = self.label.len() - 1;
        if s.len() > capacity {
            return Err(MjSceneError::LabelTooLong { len: s.len(), capacity });
        }
        let target: &mut [u8] = bytemuck::cast_slice_mut(&mut self.label[..s.len()]);
        target.copy_from_slice(s.as_bytes());
        self.label[s.len()] = 0;
        Ok(())
    }
}

/***********************************************************************************************************************
** MjvLight
***********************************************************************************************************************/
/// Visual light source parameters (position, direction, ambient/diffuse/specular RGB, etc.).
pub type MjvLight = mjvLight;

/***********************************************************************************************************************
** MjvOption
***********************************************************************************************************************/
/// Visualization rendering options (flags, label types, frame display, etc.).
pub type MjvOption = mjvOption;
impl Default for MjvOption {
    fn default() -> Self {
        let mut opt = MaybeUninit::uninit();
        // SAFETY: mjv_defaultOption initializes all fields to valid defaults;
        // the MaybeUninit pointer is valid and aligned.
        unsafe {
            mjv_defaultOption(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
}

/***********************************************************************************************************************
** MjvFigure
***********************************************************************************************************************/
/// Abstraction for plotting figures.
pub type MjvFigure = mjvFigure;
impl Default for MjvFigure {
    fn default() -> Self {
        *Self::new_boxed()
    }
}

impl MjvFigure {
    /// Instantiates a new figure with default values, allocated on the heap.
    ///
    /// `MjvFigure` is ~800 KB; this constructor avoids placing it on the stack.
    pub fn new_boxed() -> Box<Self> {
        let mut opt = Box::new(MaybeUninit::uninit());
        // SAFETY: mjv_defaultFigure initializes all fields to valid defaults;
        // the MaybeUninit pointer is valid and aligned.
        unsafe {
            mjv_defaultFigure(opt.as_mut_ptr());
            opt.assume_init()
        }
    }

    /// Instantiates a new figure with default values, allocated on the heap.
    #[deprecated(since = "3.0.0", note = "use `new_boxed` instead")]
    pub fn new() -> Box<Self> {
        Self::new_boxed()
    }

    /// Draws the 2D figure to the `viewport` on screen.
    pub fn draw(&mut self, viewport: MjrRectangle, context: &MjrContext) {
        unsafe { mjr_figure(viewport, self, context.ffi()) };
    }
}

/// Figure options.
impl MjvFigure {
    getter_setter! {with, get, set, [
        flg_legend: bool; "whether to show legend.";
        flg_extend: bool; "whether to automatically extend axis ranges to fit data.";
        flg_barplot: bool; "whether to isolate line segments.";
        flg_selection: bool; "whether to show vertical selection line.";
        flg_symmetric: bool; "whether to make y-axis symmetric.";
    ]}

    // style settings
    getter_setter! {with, [
        gridsize: [i32; 2]; "number of grid points in (x, y).";
        gridrgb: [f32; 3]; "grid line RGB color.";
        figurergba: [f32; 4]; "figure RGBA color.";
        panergba: [f32; 4]; "pane RGBA color.";
        legendrgba: [f32; 4]; "legend RGBA color.";
        textrgb: [f32; 3]; "text RGB color.";
        linergb: [[f32; 3]; mjMAXLINE as usize]; "line colors.";
        range: [[f32; 2]; 2]; "axis ranges (min >= max means automatic).";
    ]}

    c_str_as_str_method! {with, get, set {
        xlabel; "the x-axis label.";
        title; "the title.";
        xformat; "the x-axis C's printf format (e.g., `%.1f`).";
        yformat; "the y-axis C's printf format (e.g., `%.1f`).";
        linename [plot_index: usize]; "the line name of plot with `plot_index`.";
    }}
}

/// Plot data manipulation
impl MjvFigure {

    /// Checks if the buffer is full for plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::try_full`] for a fallible alternative.
    pub fn full(&self, plot_index: usize) -> bool {
        self.try_full(plot_index).unwrap()
    }

    /// Checks if the buffer is full for plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::full`] for a panicking alternative.
    pub fn try_full(&self, plot_index: usize) -> Result<bool, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        Ok(self.linepnt[plot_index] >= (self.linedata[plot_index].len() / 2) as i32)
    }

    /// Checks if the buffer is empty for plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::try_empty`] for a fallible alternative.
    pub fn empty(&self, plot_index: usize) -> bool {
        self.try_empty(plot_index).unwrap()
    }

    /// Checks if the buffer is empty for plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::empty`] for a panicking alternative.
    pub fn try_empty(&self, plot_index: usize) -> Result<bool, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        Ok(self.linepnt[plot_index] == 0)
    }

    /// Pushes a new data point to buffer for the specific plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureBufferFull`] if the buffer for
    /// `plot_index` is already at capacity.
    pub fn push(&mut self, plot_index: usize, x: f32, y: f32) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let plot = &mut self.linedata[plot_index];
        let capacity = plot.len() / 2;
        let point_index = self.linepnt[plot_index] as usize;
        if point_index >= capacity {
            return Err(MjSceneError::FigureBufferFull { plot_index, capacity });
        }
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;
        self.linepnt[plot_index] += 1;
        Ok(())
    }

    /// Overrides existing data with a new data point at a specific `point_index` for specific plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `point_index` is at or above the plot capacity of
    /// `linedata[plot_index].len() / 2` points, which `linepnt[plot_index]` allows when it is
    /// itself above the capacity.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureIndexOutOfBounds`] if `point_index` is
    /// not within the current data range for the given plot.
    pub fn set_at(
        &mut self,
        plot_index: usize,
        point_index: usize,
        x: f32,
        y: f32,
    ) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        // Clamped because a negative entry would cast to usize::MAX and pass the test below.
        let current_len = self.linepnt[plot_index].max(0) as usize;
        if point_index >= current_len {
            return Err(MjSceneError::FigureIndexOutOfBounds {
                plot_index,
                point_index,
                current_len,
            });
        }
        let plot = &mut self.linedata[plot_index];
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;
        Ok(())
    }

    /// Clears the plot with `maybe_plot_index`.
    /// If `maybe_plot_index` is [`None`], all plots will be cleared.
    ///
    /// # Panics
    /// Panics if `maybe_plot_index` is `Some(i)` and `i >= mjMAXLINE`.
    pub fn clear(&mut self, maybe_plot_index: Option<usize>) {
        if let Some(plot_index) = maybe_plot_index {
            self.linepnt[plot_index] = 0;
        } else {
            self.linepnt.fill(0);
        }
    }

    /// Pops the first element from the plot data of plot with `plot_index`.
    ///
    /// # Returns
    /// Returns `Some((x, y))` when the plot contains any elements, otherwise `None` is returned.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`, or if `linepnt[plot_index]` is above the plot
    /// capacity of `linedata[plot_index].len() / 2` points.
    ///
    /// Use [`MjvFigure::try_pop_front`] for a fallible alternative.
    pub fn pop_front(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        self.try_pop_front(plot_index).unwrap()
    }

    /// Pops the first element from the plot data of plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Returns `Ok(Some((x, y)))` when the plot contains elements, `Ok(None)` when empty.
    ///
    /// Use [`MjvFigure::pop_front`] for a panicking alternative.
    pub fn try_pop_front(&mut self, plot_index: usize) -> Result<Option<(f32, f32)>, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let len = self.linepnt[plot_index];
        if len <= 0 {
            return Ok(None);
        }
        let plot_data = &mut self.linedata[plot_index];
        let first = (plot_data[0], plot_data[1]);
        plot_data.copy_within(2..len as usize * 2, 0);
        self.linepnt[plot_index] -= 1;
        Ok(Some(first))
    }

    /// Pops the last element from the plot data of plot with `plot_index`.
    ///
    /// # Returns
    /// Returns `Some((x, y))` when the plot contains any elements, otherwise `None` is returned.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`, or if `linepnt[plot_index]` is above the plot
    /// capacity of `linedata[plot_index].len() / 2` points.
    ///
    /// Use [`MjvFigure::try_pop_back`] for a fallible alternative.
    pub fn pop_back(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        self.try_pop_back(plot_index).unwrap()
    }

    /// Pops the last element from the plot data of plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Returns `Ok(Some((x, y)))` when the plot contains elements, `Ok(None)` when empty.
    ///
    /// Use [`MjvFigure::pop_back`] for a panicking alternative.
    pub fn try_pop_back(&mut self, plot_index: usize) -> Result<Option<(f32, f32)>, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let old_len = self.linepnt[plot_index];
        if old_len <= 0 {
            return Ok(None);
        }
        let plot_data = &mut self.linedata[plot_index];
        let new_start = ((old_len - 1) * 2) as usize;
        self.linepnt[plot_index] -= 1;
        Ok(Some((plot_data[new_start], plot_data[new_start + 1])))
    }

    /// Cuts the first `n` elements from the plot data of plot with `plot_index`.
    ///
    /// If `n` exceeds the current length, this is a no-op.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    pub fn cut_front(&mut self, plot_index: usize, n: usize) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return Ok(());
        }

        self.linedata[plot_index].copy_within(2 * n..(len as usize * 2), 0);
        self.linepnt[plot_index] -= n as i32;
        Ok(())
    }

    /// Cuts the last `n` elements from the plot data of plot with `plot_index`.
    ///
    /// If `n` exceeds the current length, this is a no-op.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    pub fn cut_end(&mut self, plot_index: usize, n: usize) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return Ok(());
        }
        self.linepnt[plot_index] -= n as i32;
        Ok(())
    }
}


/***********************************************************************************************************************
** MjvScene
***********************************************************************************************************************/
/// Snapshot of the [`MjModel`] quantities that fix the size of every buffer that
/// [`MjvScene::new`] allocates.
///
/// The flex tables are kept whole: `mjv_makeScene` sizes the flex face buffer from `flex_dim`,
/// `flex_elemnum`, `flex_shellnum` and `flex_elemlayer` together, so no scalar total replaces
/// them.
#[derive(Debug, Clone, PartialEq, Eq)]
struct MjvSceneLayout {
    signature: u64,
    nflexedge: MjtSize,
    nflexvert: MjtSize,
    nskinvert: MjtSize,
    flex_dim: Vec<i32>,
    flex_elemnum: Vec<i32>,
    flex_shellnum: Vec<i32>,
    flex_elemlayer: Vec<i32>,
}

impl From<&MjModel> for MjvSceneLayout {
    fn from(model: &MjModel) -> Self {
        let ffi = model.ffi();
        Self {
            signature: model.signature(),
            nflexedge: ffi.nflexedge,
            nflexvert: ffi.nflexvert,
            nskinvert: ffi.nskinvert,
            flex_dim: model.flex_dim().to_vec(),
            flex_elemnum: model.flex_elemnum().to_vec(),
            flex_shellnum: model.flex_shellnum().to_vec(),
            flex_elemlayer: model.flex_elemlayer().to_vec(),
        }
    }
}


/// 3D scene visualization.
/// This struct provides a way to render visual-only geometry.
///
/// The scene does not hold a reference to the model.
/// Trying to use an existing scene with an incompatible model (see
/// [`MjvScene::is_compatible_with_model`]) will result in a panic.
#[derive(Debug)]
pub struct MjvScene {
    ffi: Box<mjvScene>,
    layout: MjvSceneLayout,
}

impl MjvScene {
    /// Creates a new scene for `model`, allocating space for up to `max_geom` geoms.
    ///
    /// # Panics
    /// In debug builds, panics if `max_geom` exceeds `i32::MAX`.
    pub fn new<M: Deref<Target = MjModel>>(model: M, max_geom: usize) -> Self {
        debug_assert!(max_geom <= i32::MAX as usize, "max_geom exceeds i32::MAX");
        let model_ffi = model.ffi();
        let layout = MjvSceneLayout::from(&*model);

        // SAFETY: The struct memory gets initialized properly before assumed initialized.
        // The uninitialized memory, not part of the struct, gets zeroed below this unsafe block.
        let scene = unsafe {
            let mut t = Box::new_uninit();
            mjv_defaultScene(t.as_mut_ptr());
            mjv_makeScene(model_ffi, t.as_mut_ptr(), max_geom as i32);
            t.assume_init()
        };

        // Zero uninitialized memory that `mjv_makeScene` allocated.
        let nflex = scene.nflex as usize;
        let nface = if scene.flexfacenum.is_null() {
            0
        } else {
            // SAFETY: scene.flexfacenum is non-null and scene.nflex is guaranteed
            // correct by MuJoCo code.
            unsafe { std::slice::from_raw_parts(scene.flexfacenum, nflex) }
            .iter().map(|&n| n as usize).sum()
        };
        let nflexvert = layout.nflexvert as usize;
        let nskinvert = layout.nskinvert as usize;
        let maxgeom = scene.maxgeom as usize;
        let buffers: [(*mut u8, usize); 9] = [
            (scene.geoms.cast(),        maxgeom * size_of::<mjvGeom>()),
            (scene.geomorder.cast(),    maxgeom * size_of::<i32>()),
            (scene.flexfaceused.cast(), nflex * size_of::<i32>()),
            (scene.flexvert.cast(),     3 * nflexvert * size_of::<f32>()),
            (scene.flexface.cast(),     9 * nface * size_of::<f32>()),
            (scene.flexnormal.cast(),   9 * nface * size_of::<f32>()),
            (scene.flextexcoord.cast(), 6 * nface * size_of::<f32>()),
            (scene.skinvert.cast(),     3 * nskinvert * size_of::<f32>()),
            (scene.skinnormal.cast(),   3 * nskinvert * size_of::<f32>()),
        ];
        for (pointer, bytes) in buffers {
            if !pointer.is_null() && bytes != 0 {
                // SAFETY: destination pointer is non-null and bytes > 0.
                unsafe { ptr::write_bytes(pointer, 0, bytes); }
            }
        }

        Self { ffi: scene, layout }
    }

    /// Returns the model signature this scene was created for.
    pub fn signature(&self) -> u64 {
        self.layout.signature
    }

    /// Reports whether `model` can take the place of the model that created this scene.
    ///
    /// The scene buffers hold one entry per flex face, flex vertex, flex edge and skin vertex of
    /// the model that created them, and `mjv_updateScene` refills them with the counts of the
    /// model it receives. The test therefore covers the compilation signature and every count
    /// that sizes such a buffer.
    pub fn is_compatible_with_model(&self, model: &MjModel) -> bool {
        self.layout == MjvSceneLayout::from(model)
    }

    /// Reports whether `other` was created for a model that is compatible with this scene's
    /// model.
    ///
    /// A geom that moves between two scenes keeps its `objid`, which the renderer uses as an
    /// unchecked index into the destination scene's flex and skin arrays.
    pub fn is_compatible_with_scene(&self, other: &MjvScene) -> bool {
        self.layout == other.layout
    }

    /// Panics if `model` is not compatible with the model used to create the scene.
    fn assert_compatible(&self, model: &MjModel) {
        assert!(
            self.is_compatible_with_model(model),
            "the model is not compatible with the scene: scene signature {:#X}, model signature {:#X}",
            self.layout.signature, model.signature()
        );
    }

    /// Updates the scene from the current simulation state in `data`.
    ///
    /// The `catmask` parameter controls which geom categories are included
    /// (e.g., [`MjtCatBit::mjCAT_ALL`] for everything, or a bitwise OR of
    /// [`MjtCatBit::mjCAT_STATIC`], [`MjtCatBit::mjCAT_DYNAMIC`], [`MjtCatBit::mjCAT_DECOR`]).
    ///
    /// The call resets `ngeom` to 0, so it drops every geom that [`MjvScene::create_geom`] added;
    /// create such geoms after the update. Unless `cam` is of type
    /// [`MjtCamera::mjCAMERA_USER`], it also overwrites the scene cameras and clears
    /// `enabletransform`, and it moves `cam.lookat` onto the tracked body for a tracking camera.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `cam` is a tracking camera
    /// ([`MjtCamera::mjCAMERA_TRACKING`]) whose `trackbodyid` is negative or not below the number
    /// of bodies.
    ///
    /// # Panics
    /// - Panics if `data` was created from a model that is not compatible with this scene
    ///   (see [`MjvScene::is_compatible_with_model`]).
    /// - Panics if `cam` is a fixed camera ([`MjtCamera::mjCAMERA_FIXED`]) whose `fixedcamid` is
    ///   out of range for the model in `data`.
    /// - Panics if `perturb.select` is out of range (greater than or equal to the number of
    ///   bodies) for the model in `data`.
    pub fn update_with_catmask<M: Deref<Target = MjModel>>(
        &mut self, data: &mut MjData<M>, opt: &MjvOption, perturb: &MjvPerturb,
        cam: &mut MjvCamera, catmask: i32,
    ) {
        self.assert_compatible(data.model());

        // mjv_updateScene -> mjv_updateCamera calls mjv_cameraFrame, whose mjCAMERA_FIXED branch
        // reads d->cam_xmat + 9*fixedcamid / d->cam_xpos + 3*fixedcamid *before* C performs its own
        // range check, so an out-of-range id is an out-of-bounds read. Guard it here, matching the
        // check already in MjvCamera::frame(). (The free/tracking branches derive the frame from
        // azimuth/elevation, and the tracking body id is range-checked by C before use.)
        if cam.type_ == MjtCamera::mjCAMERA_FIXED as i32 {
            let ncam = data.model().ncam();
            assert!(
                cam.fixedcamid >= 0 && (cam.fixedcamid as i64) < ncam,
                "fixed camera id {} is out of range for a model with {} cameras",
                cam.fixedcamid, ncam
            );
        }

        // MuJoCo does no bound checking for pert->select, thus invalid IDs leads to out-of-bound reads.
        assert!((perturb.select as MjtSize) < data.model().nbody(), "selected perturbated body ID is outside the valid range");

        unsafe {
            mjv_updateScene(
                data.model().ffi(), data.ffi_mut(), opt, perturb,
                cam, catmask, self.ffi.as_mut()
            );
        }
    }

    /// Updates the scene from the current simulation state in `data`, including all geom categories.
    ///
    /// This is equivalent to calling [`update_with_catmask`](Self::update_with_catmask) with
    /// [`MjtCatBit::mjCAT_ALL`].
    ///
    /// # Note
    /// MuJoCo stops the process under the same conditions as
    /// [`update_with_catmask`](Self::update_with_catmask).
    ///
    /// # Panics
    /// Panics under the same conditions as [`update_with_catmask`](Self::update_with_catmask):
    /// a model-signature mismatch, an out-of-range fixed-camera id, or an out-of-range
    /// `perturb.select`.
    pub fn update<M: Deref<Target = MjModel>>(&mut self, data: &mut MjData<M>, opt: &MjvOption, perturb: &MjvPerturb, cam: &mut MjvCamera) {
        self.update_with_catmask(data, opt, perturb, cam, MjtCatBit::mjCAT_ALL as i32);
    }

    /// Creates a new [`MjvGeom`] in this scene, returning a mutable reference to it.
    /// The geom reference is valid for the duration of the `&mut self` borrow.
    ///
    /// # Safety
    /// The caller must keep the returned geom's `matid`, `texid` and, on a flex or a skin geom,
    /// `objid` below the count of the model that the rendering context was created for (`matid`
    /// and `texid` also accept `-1` for none). The renderer reads all three as unchecked indices
    /// into the context and the scene arrays, so a larger value makes [`MjvScene::render`] read
    /// out of bounds. A flex or a skin geom needs its own `objid`: the geom starts at `-1`, which
    /// indexes neither array.
    ///
    /// # Panics
    /// Panics when `ngeom >= maxgeom` (the scene's geom buffer is full).
    ///
    /// Use [`MjvScene::try_create_geom`] for a fallible alternative.
    pub unsafe fn create_geom(
        &mut self, geom_type: MjtGeom, size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>, mat: Option<[MjtNum; 9]>, rgba: Option<[f32; 4]>
    ) -> &mut MjvGeom {
        // SAFETY: the caller upholds the index contract of `try_create_geom`.
        unsafe { self.try_create_geom(geom_type, size, pos, mat, rgba) }
            .expect("create_geom failed: scene full")
    }

    /// Fallible version of [`MjvScene::create_geom`].
    ///
    /// # Safety
    /// The caller upholds the same index contract as in [`MjvScene::create_geom`].
    ///
    /// # Errors
    /// Returns [`MjSceneError::SceneFull`] when `ngeom >= maxgeom`.
    pub unsafe fn try_create_geom(
        &mut self, geom_type: MjtGeom, size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>, mat: Option<[MjtNum; 9]>, rgba: Option<[f32; 4]>
    ) -> Result<&mut MjvGeom, MjSceneError> {
        if self.ffi.ngeom >= self.ffi.maxgeom {
            return Err(MjSceneError::SceneFull { capacity: self.ffi.maxgeom });
        }

        let size_ptr = size.as_ref().map_or(ptr::null(), |x| x);
        let pos_ptr  = pos.as_ref().map_or(ptr::null(), |x| x);
        let mat_ptr  = mat.as_ref().map_or(ptr::null(), |x| x);
        let rgba_ptr = rgba.as_ref().map_or(ptr::null(), |x| x);

        // SAFETY: ngeom < maxgeom guarantees we are within the allocated buffer, and p_geom is
        // non-null because mjv_makeScene allocated it. mjv_initGeom writes every other field, so
        // the assignments below leave no byte of the mju_malloc slot uninitialized.
        unsafe {
            let p_geom = self.ffi.geoms.add(self.ffi.ngeom as usize);
            mjv_initGeom(p_geom, geom_type as i32, size_ptr, pos_ptr, mat_ptr, rgba_ptr);
            // A created geom decorates the scene and belongs to no model element, so it carries
            // the "none" ids. mjv_initGeom writes only the first byte of label, and mjr_render
            // recomputes camdist and transparent before it reads them.
            (*p_geom).objtype     = MjtObj::mjOBJ_UNKNOWN as i32;
            (*p_geom).objid       = -1;
            (*p_geom).category    = MjtCatBit::mjCAT_DECOR as i32;
            (*p_geom).segid       = self.ffi.ngeom;
            (*p_geom).label       = [0; 100];
            (*p_geom).camdist     = 0.0;
            (*p_geom).transparent = 0;
            self.ffi.ngeom += 1;
            Ok(&mut *p_geom)
        }
    }

    /// Clears the created geoms.
    pub fn clear_geom(&mut self) {
        self.ffi.ngeom = 0;
    }

    /// Removes the last geom from the scene.
    /// Does nothing if the scene contains no geoms.
    pub fn pop_geom(&mut self) {
        if self.ffi.ngeom == 0 {
            return;
        }

        self.ffi.ngeom -= 1;
    }

    /// Renders the scene into the buffer that `context` currently targets. This does not
    /// automatically make the OpenGL context current.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when the scene holds geoms but its cameras
    /// were never filled by [`MjvScene::update`], which leaves `frustum_near` at zero. With no
    /// geoms the call returns without drawing.
    ///
    /// # Panics
    /// Panics if `context` holds fewer skins than the scene.
    pub fn render(&mut self, viewport: &MjrRectangle, context: &MjrContext){
        // mjr_render uploads every skin of the scene into context.skinvertVBO[i], an array with
        // one entry per skin of the model the context was built for, and null when it has none.
        assert!(
            self.nskin() <= context.nskin(),
            "the context was created for a model with fewer skins"
        );
        unsafe {
            mjr_render(*viewport, self.ffi_mut(), context.ffi());
        }
    }

    /// Returns the selection point based on a mouse click.
    /// Wraps [`mjv_select`].
    ///
    /// `aspect_ratio` is the viewport width divided by its height. `relx` and `rely` are the cursor
    /// position as fractions of the viewport in `[0, 1]`, measured from the left and bottom edges.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when the scene cameras were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `data` was created from a model that is not compatible with this scene
    /// (see [`MjvScene::is_compatible_with_model`]).
    pub fn find_selection<M: Deref<Target = MjModel>>(
        &self, data: &mut MjData<M>, option: &MjvOption,
        aspect_ratio: MjtNum, relx: MjtNum, rely: MjtNum,
    ) -> SceneSelection {
        self.assert_compatible(data.model());
        let (mut geom_id, mut flex_id, mut skin_id) = (-1 , -1, -1);
        let mut selpnt = [0.0; 3];
        let body_id = unsafe {
            mjv_select(
                data.model().ffi(), data.ffi(), option,
                aspect_ratio, relx, rely, self.ffi(), &mut selpnt,
                &mut geom_id, &mut flex_id, &mut skin_id
            )
        };
        let to_opt = |v| if v >= 0 { Some(v as usize) } else { None };
        SceneSelection { body_id: to_opt(body_id), geom_id: to_opt(geom_id), flex_id: to_opt(flex_id), skin_id: to_opt(skin_id), point: selpnt }
    }

    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjvScene {
        &self.ffi
    }

    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjvScene {
        &mut self.ffi
    }
}

/// Array slices.
impl MjvScene {
    // Scalar length arrays
    array_slice_dyn! {
        (mut = unsafe) flexedge: &[[i32; 2] [force]; "flex edge data"; layout.nflexedge],
        flexvert: &[[f32; 3] [force]; "flex vertices"; layout.nflexvert],
        skinvert: &[[f32; 3] [force]; "skin vertex data"; layout.nskinvert],
        skinnormal: &[[f32; 3] [force]; "skin normal data"; layout.nskinvert],
        (mut = unsafe) geoms: &[MjvGeom; "buffer for geoms"; ffi.ngeom],
        geomorder: &[i32; "buffer for ordering geoms by distance to camera"; ffi.ngeom],
        (mut = unsafe) flexedgeadr: &[i32; "address of flex edges"; ffi.nflex],
        (mut = unsafe) flexedgenum: &[i32; "number of edges in flex"; ffi.nflex],
        (mut = unsafe) flexvertadr: &[i32; "address of flex vertices"; ffi.nflex],
        (mut = unsafe) flexvertnum: &[i32; "number of vertices in flex"; ffi.nflex],
        (mut = unsafe) flexfaceadr: &[i32; "address of flex faces"; ffi.nflex],
        (mut = unsafe) flexfacenum: &[i32; "number of flex faces allocated"; ffi.nflex],
        (mut = unsafe) flexfaceused: &[i32; "number of flex faces currently in use"; ffi.nflex],
        (mut = unsafe) skinfacenum: &[i32; "number of faces in skin"; ffi.nskin],
        (mut = unsafe) skinvertadr: &[i32; "address of skin vertices"; ffi.nskin],
        (mut = unsafe) skinvertnum: &[i32; "number of vertices in skin"; ffi.nskin],
        lights: as_ptr as_mut_ptr &[MjvLight; "buffer for lights"; ffi.nlight]
    }

    // Arrays whose size is obtained via sum:
    // (multiplier; length array; length array length)
    //   => length = multiplier * sum(length_array)
    array_slice_dyn! {
        summed {
            flexface: &[f32; "flex faces vertices"; [9; (ffi.flexfacenum); (ffi.nflex)]],
            flexnormal: &[f32; "flex face normals"; [9; (ffi.flexfacenum); (ffi.nflex)]],
            flextexcoord: &[f32; "flex face texture coordinates"; [6; (ffi.flexfacenum); (ffi.nflex)]]
        }
    }
}


/// Public API getters / setters / builders.
impl MjvScene {
    getter_setter! {get, [
        [ffi] maxgeom: i32; "size of allocated geom buffer.";
        [ffi] ngeom: i32; "number of geoms currently in buffer.";
        [ffi] nflex: i32; "number of flexes.";
        [ffi] nskin: i32; "number of skins.";
        [ffi] nlight: i32; "number of lights currently in buffer.";
        [ffi] status: i32; "status; 0: ok, 1: geoms exhausted.";
    ]}

    getter_setter! {get, [
        [ffi] flexvertopt: bool; "copy of mjVIS_FLEXVERT mjvOption flag.";
        [ffi] flexedgeopt: bool; "copy of mjVIS_FLEXEDGE mjvOption flag.";
        [ffi] flexfaceopt: bool; "copy of mjVIS_FLEXFACE mjvOption flag.";
        [ffi] flexskinopt: bool; "copy of mjVIS_FLEXSKIN mjvOption flag.";
    ]}

    getter_setter! {with, get, set, [
        [ffi, ffi_mut] stereo: MjtStereo [force]; "stereoscopic rendering.";
    ]}

    getter_setter! {with, get, set, [
        [ffi, ffi_mut] scale: f32; "model scaling.";
        [ffi, ffi_mut] framewidth: i32; "frame pixel width; 0: disable framing.";
    ]}

    getter_setter! {with, get, set, [
        [ffi, ffi_mut] enabletransform: bool; "enable model transformation.";
    ]}

    getter_setter! {with, get, [
        [ffi, ffi_mut] camera: &[MjvGLCamera; 2]; "left and right camera.";
        [ffi, ffi_mut] translate: &[f32; 3]; "model translation.";
        [ffi, ffi_mut] rotate: &[f32; 4]; "model quaternion rotation.";
        [ffi, ffi_mut] framergb: &[f32; 3]; "frame color.";
    ]}

    getter_setter! {get, [
        [ffi, ffi_mut] flags: &[MjtByte; MjtRndFlag::mjNRNDFLAG as usize]; "rendering flags (indexed by mjtRndFlag).";
    ]}
}


impl Drop for MjvScene {
    fn drop(&mut self) {
        // SAFETY: self.ffi is a valid initialized MjvScene; called exactly once in Drop.
        unsafe {
            mjv_freeScene(self.ffi.as_mut());
        }
    }
}

// SAFETY: MjvScene exclusively owns the Box<mjvScene> and every buffer that mjv_makeScene
// allocated for it (geoms, geomorder, the flex and skin arrays), all of which mjv_freeScene
// releases in Drop, so no aliasing with other threads is possible. All mutation requires
// &mut self, so &MjvScene sharing across threads is safe.
unsafe impl Send for MjvScene {}
unsafe impl Sync for MjvScene {}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE_MODEL: &str = "
    <mujoco>
    <worldbody>
        <light ambient=\"0.2 0.2 0.2\"/>
        <body name=\"ball\">
            <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\"/>
            <joint type=\"free\"/>
        </body>

        <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>

    </worldbody>
    </mujoco>
    ";

    /* Tests setup */
    fn load_model() -> MjModel {
        MjModel::from_xml_string(EXAMPLE_MODEL).unwrap()
    }


    /// The three bodies are the same in both models, so `mjCModel::Signature` is the same; only
    /// the number of flex vertices differs, and that number sizes `mjvScene::flexvert`.
    fn flex_model(flex: &str) -> MjModel {
        let xml = format!(
            "<mujoco><worldbody>\
<body name='v0' pos='0 0 0'><freejoint/><geom size='0.01'/></body>\
<body name='v1' pos='0.1 0 0'><freejoint/><geom size='0.01'/></body>\
<body name='v2' pos='0.2 0 0'><freejoint/><geom size='0.01'/></body>\
</worldbody><deformable>{flex}</deformable></mujoco>"
        );
        MjModel::from_xml_string(&xml).unwrap()
    }

    /// `mjv_makeScene` allocates the flex vertex and face buffers with `mju_malloc` and never
    /// writes them, so a fresh scene must not expose that memory through its safe accessors.
    #[test]
    fn test_new_scene_zeroes_the_uninitialized_flex_buffers() {
        let model = flex_model(
            "<flex name='f' dim='2' body='v0 v1 v2' vertex='0 0 0 0 0 0 0 0 0' element='0 1 2'/>"
        );
        let scene = MjvScene::new(&model, 100);

        // An all-zero test over an empty slice would pass for the wrong reason.
        assert!(!scene.flexvert().is_empty(), "the model must allocate flex vertices");
        assert!(!scene.flexface().is_empty(), "the model must allocate flex faces");

        assert!(scene.flexvert().iter().all(|vertex| *vertex == [0.0; 3]));
        assert!(scene.flexface().iter().all(|face| *face == [0.0; 9]));
        assert!(scene.flexnormal().iter().all(|normal| *normal == [0.0; 9]));
        assert!(scene.flextexcoord().iter().all(|texcoord| *texcoord == [0.0; 6]));
        assert!(scene.flexfaceused().iter().all(|used| *used == 0));
    }

    /// `mjv_makeScene` allocates `geomorder` with `mju_malloc`, and only `mjr_render` writes it,
    /// and then only the transparent prefix. An updated but unrendered scene must therefore not
    /// expose that memory through its safe accessor.
    #[test]
    fn test_new_scene_zeroes_the_geom_order_buffer() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 100);
        let mut data = model.make_data();
        let (opt, pert) = (MjvOption::default(), MjvPerturb::default());
        let mut camera = MjvCamera::default();
        scene.update(&mut data, &opt, &pert, &mut camera);

        // An all-zero test over an empty slice would pass for the wrong reason.
        assert!(!scene.geomorder().is_empty(), "the update must add geoms");
        assert!(scene.geomorder().iter().all(|order| *order == 0));
    }

    #[test]
    fn test_scene_is_compatible_with_model() {
        let three = flex_model("<flex name='f' dim='1' body='v0 v1 v2' vertex='0 0 0 0 0 0 0 0 0' element='0 1 1 2'/>");
        let two = flex_model("<flex name='f' dim='1' body='v0 v1' vertex='0 0 0 0 0 0' element='0 1'/>");
        assert_eq!(three.signature(), two.signature(), "the pair must share a signature");
        assert_ne!(three.nflexvert(), two.nflexvert());

        let scene = MjvScene::new(&three, 100);
        assert!(scene.is_compatible_with_model(&three));
        assert!(!scene.is_compatible_with_model(&two));
    }

    #[test]
    fn test_scene_is_compatible_with_scene() {
        let three = flex_model("<flex name='f' dim='1' body='v0 v1 v2' vertex='0 0 0 0 0 0 0 0 0' element='0 1 1 2'/>");
        let two = flex_model("<flex name='f' dim='1' body='v0 v1' vertex='0 0 0 0 0 0' element='0 1'/>");

        let scene = MjvScene::new(&three, 100);
        let same = MjvScene::new(&three, 10);
        let other = MjvScene::new(&two, 100);
        assert!(scene.is_compatible_with_scene(&same), "maxgeom must not affect compatibility");
        assert!(!scene.is_compatible_with_scene(&other));
    }
    #[test]
    #[allow(non_snake_case)]
    fn test_MjvGeom() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);
        
        /* Test label handling. Other things are trivial one-liners. */
        // SAFETY: the test never writes a material, texture or object id.
        let geom = unsafe { scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) };
        let label = "Hello World";
        geom.set_label(label).unwrap();
        assert_eq!(geom.label(), label);
    }

    /// A fixed camera whose id is out of range for the model must panic in `frame` instead of
    /// triggering an out-of-bounds read in MuJoCo's `mjv_cameraFrame` (which indexes
    /// `cam_xpos`/`cam_xmat` by `fixedcamid` without bounds checks).
    #[test]
    #[should_panic(expected = "out of range")]
    fn test_camera_frame_fixed_id_out_of_range_panics() {
        let model = load_model();          // EXAMPLE_MODEL defines no <camera>, so ncam == 0
        let data = model.make_data();
        let camera = MjvCamera::new_fixed(0);  // fixedcamid = 0, but ncam = 0 -> out of range
        let _ = camera.frame(&data);
    }

    /// A tracking camera whose body id is out of range must panic in `frame` instead of triggering
    /// an out-of-bounds read in MuJoCo's `mjv_cameraFrame`, which indexes `subtree_com` by
    /// `trackbodyid` after testing only `trackbodyid >= 0`.
    #[test]
    #[should_panic(expected = "out of range")]
    fn test_camera_frame_tracking_id_out_of_range_panics() {
        let model = load_model();          // nbody == 2 (world + "ball")
        let data = model.make_data();
        let camera = MjvCamera::new_tracking(9999);
        let _ = camera.frame(&data);
    }

    /// `MjvScene::update` must apply the same fixed-camera range check as `frame`. Otherwise a fixed
    /// camera with an out-of-range id reaches MuJoCo's `mjv_cameraFrame` (which reads
    /// `cam_xmat`/`cam_xpos` indexed by `fixedcamid`) *before* C runs its own range check.
    #[test]
    #[should_panic(expected = "out of range")]
    fn test_scene_update_fixed_camera_out_of_range_panics() {
        let model = load_model();          // no <camera>, so ncam == 0
        let mut scene = MjvScene::new(&model, 1000);
        let mut data = model.make_data();
        let (opt, pert) = (MjvOption::default(), MjvPerturb::default());
        let mut camera = MjvCamera::new_fixed(9999);
        scene.update(&mut data, &opt, &pert, &mut camera);
    }

    /// A perturbation whose `select` is out of range for the model must panic in `move_` instead of
    /// triggering an out-of-bounds read in MuJoCo's `mjv_movePerturb`, which dereferences
    /// `d->xmat + 9*select` (move-relative) before any range check and has no `select <= 0`
    /// early-out (unlike the `mjv_updateScene` path).
    #[test]
    #[should_panic(expected = "out of range")]
    fn test_perturb_move_select_out_of_range_panics() {
        let model = load_model();          // nbody == 2 (world + "ball")
        let data = model.make_data();
        let scene = MjvScene::new(&model, 100);
        let mut pert = MjvPerturb { select: 1_000_000, ..Default::default() };  // far beyond nbody
        pert.move_(&data, MjtMouse::mjMOUSE_MOVE_H_REL, 0.1, 0.1, &scene);
    }

    /// `mjv_initPerturb` scales `localmass` by `flex_edgenum[flexselect]` after testing only that
    /// the id is not negative, so the wrapper must reject an out-of-range flex id.
    #[test]
    #[should_panic(expected = "out of range for a model with")]
    fn test_perturb_start_flexselect_out_of_range_panics() {
        let model = load_model();
        let mut data = model.make_data();
        let scene = MjvScene::new(&model, 100);
        let mut pert = MjvPerturb { select: 1, flexselect: 1_000_000, ..Default::default() };
        pert.start(MjtPertBit::mjPERT_TRANSLATE, &mut data, &scene);
    }

    /// `mjv_initGeom` leaves `objtype`, `objid`, `category` and `segid` unwritten over the
    /// uninitialized buffer, and the renderer indexes the skin and flex arrays by `objid`, so a
    /// created geom must start with the "none" ids and with `segid` set to its own index.
    #[test]
    fn test_create_geom_fields_initialized() {
        const N_GEOM: usize = 8;

        let model = load_model();
        let mut scene = MjvScene::new(&model, N_GEOM);

        // Dirty every slot first, so that reusing them proves the assignment rather than relying
        // on whatever the allocator happened to leave behind.
        // SAFETY: the scene is never rendered, so the dirty ids below stay unread.
        for _ in 0..N_GEOM {
            let geom = unsafe { scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) };
            geom.objid = 12345;
            geom.objtype = 678;
            geom.category = 9;
        }
        scene.clear_geom();

        for i in 0..N_GEOM {
            let geom = unsafe { scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) };
            assert_eq!(
                (geom.objtype, geom.objid, geom.category),
                (MjtObj::mjOBJ_UNKNOWN as i32, -1, MjtCatBit::mjCAT_DECOR as i32)
            );
            assert_eq!(geom.segid, i as i32);
        }
    }

    #[test]
    fn test_scene_geom_pop() {
        const N_GEOM: usize = 10;
        const N_GEOM_POP: usize = 11;

        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);

        // SAFETY: the test never writes a material, texture or object id.
        for _ in 0..N_GEOM {
            unsafe { scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) };
        }

        assert_eq!(scene.geoms().len(), N_GEOM);

        for _ in 0..N_GEOM_POP {
            scene.pop_geom();
        }

        assert_eq!(scene.geoms().len(), 0);
    }

    #[test]
    fn test_scene_slices() {
        let model = load_model();
        let scene = MjvScene::new(&model, 100);

        assert_eq!(scene.lights().len(), scene.ffi().nlight as usize);
        assert_eq!(scene.geomorder().len(), scene.ffi().ngeom as usize);
        assert_eq!(scene.geoms().len(), scene.ffi().ngeom as usize);
    }

    #[test]
    fn test_figure() {
        let mut fig = MjvFigure::new_boxed();
        let plot = 0;

        // Initially empty
        assert!(fig.empty(plot));
        assert_eq!(fig.pop_front(plot), None);
        assert_eq!(fig.pop_back(plot), None);

        // Push two points
        fig.push(plot, 1.0, 2.0).unwrap();
        fig.push(plot, 3.0, 4.0).unwrap();

        assert!(!fig.empty(plot));

        // Pop from front (FIFO)
        let first = fig.pop_front(plot);
        assert_eq!(first, Some((1.0, 2.0)));

        // Pop from back (LIFO on remaining element)
        let last = fig.pop_back(plot);
        assert_eq!(last, Some((3.0, 4.0)));

        // Now empty again
        assert!(fig.empty(plot));
        assert_eq!(fig.pop_front(plot), None);
        assert_eq!(fig.pop_back(plot), None);
    }

    /// Tests `getter_setter!` bool roundtrip for `enabletransform` on MjvScene.
    #[test]
    fn test_bool_getter_setter_roundtrip() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 100);

        // Default should be false
        assert!(!scene.enabletransform());

        scene.set_enabletransform(true);
        assert!(scene.enabletransform());

        scene.set_enabletransform(false);
        assert!(!scene.enabletransform());
    }

    /// Tests `getter_setter! force!` enum roundtrip for stereo (MjtStereo) on MjvScene.
    #[test]
    fn test_force_enum_getter_setter_roundtrip() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 100);

        // Default stereo mode
        let original = scene.stereo();
        assert_eq!(original, MjtStereo::mjSTEREO_NONE);

        scene.set_stereo(MjtStereo::mjSTEREO_QUADBUFFERED);
        assert_eq!(scene.stereo(), MjtStereo::mjSTEREO_QUADBUFFERED);

        scene.set_stereo(MjtStereo::mjSTEREO_SIDEBYSIDE);
        assert_eq!(scene.stereo(), MjtStereo::mjSTEREO_SIDEBYSIDE);
    }

    /// Tests that `create_geom` returns `SceneFull` when the scene capacity is exhausted.
    #[test]
    fn test_scene_full_error() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 2);

        // SAFETY: the test never writes a material, texture or object id.
        unsafe {
            // Fill to capacity
            scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None);
            scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None);
        }

        // Next should fail
        let err = unsafe { scene.try_create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None) }
            .unwrap_err();
        assert!(matches!(err, MjSceneError::SceneFull { capacity: 2 }));
    }

    /// Tests `set_label` boundary: too-long label returns `LabelTooLong`,
    /// max-length label succeeds with roundtrip.
    #[test]
    fn test_geom_label_boundary() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 10);
        // SAFETY: the test never writes a material, texture or object id.
        let geom = unsafe { scene.create_geom(MjtGeom::mjGEOM_BOX, None, None, None, None) };

        // Determine capacity (buffer len - 1 for NUL)
        let capacity = geom.label.len() - 1;

        // Max-length label should succeed
        let max_label: String = "A".repeat(capacity);
        geom.set_label(&max_label).unwrap();
        assert_eq!(geom.label(), max_label);

        // One byte too long should fail
        let too_long: String = "B".repeat(capacity + 1);
        let err = geom.set_label(&too_long).unwrap_err();
        assert!(matches!(err, MjSceneError::LabelTooLong { .. }));

        // Empty label should work
        geom.set_label("").unwrap();
        assert_eq!(geom.label(), "");
    }

    /// Tests `push` returns `FigureBufferFull` at capacity,
    /// and push succeeds after `pop_back` frees a slot.
    #[test]
    fn test_figure_push_overflow() {
        let mut fig = MjvFigure::new_boxed();
        let plot = 0;
        let capacity = fig.linedata[plot].len() / 2;

        // Fill to capacity
        for i in 0..capacity {
            fig.push(plot, i as f32, i as f32).unwrap();
        }
        assert!(fig.full(plot));

        // Next push should fail
        let err = fig.push(plot, 0.0, 0.0).unwrap_err();
        assert!(matches!(err, MjSceneError::FigureBufferFull { plot_index: 0, .. }));

        // Pop one element, then push should succeed again
        fig.pop_back(plot);
        assert!(!fig.full(plot));
        fig.push(plot, 99.0, 99.0).unwrap();
        assert!(fig.full(plot));
    }

    /// A negative `linepnt`, which safe code can write because the field is public, must read as
    /// an empty plot in `set_at`, exactly as the pop and cut methods already treat it.
    #[test]
    fn test_figure_set_at_negative_linepnt() {
        let mut fig = MjvFigure::new_boxed();
        let plot = 0;
        fig.push(plot, 1.0, 2.0).unwrap();
        fig.linepnt[plot] = -1;

        let err = fig.set_at(plot, 5, 3.0, 4.0).unwrap_err();
        assert!(matches!(err, MjSceneError::FigureIndexOutOfBounds { current_len: 0, .. }), "{err:?}");
        // The rejected call must not have written anything.
        assert_eq!(fig.linedata[plot][10], 0.0);
    }

    /// Verifies the getter and setter generated by `c_str_as_str_method!`
    /// for the `title` field of `MjvFigure`.
    ///
    /// Note: the `with_*` builder takes `self` by value. Because `MjvFigure` is ~800 KB,
    /// `new()` returns `Box<Self>` and calling `with_title` would move the value onto the
    /// stack, causing a stack overflow in a test thread. The setter covers the same code
    /// path, so the builder is not exercised here.
    #[test]
    fn test_c_str_as_str_method_title_roundtrip() {
        let mut fig = MjvFigure::new_boxed();

        // Default title should be empty.
        assert_eq!(fig.title(), "");

        // Setter must store the value and getter must return it.
        fig.set_title("hello");
        assert_eq!(fig.title(), "hello");

        // Overwrite with a different value to confirm the field actually changed.
        fig.set_title("world");
        assert_eq!(fig.title(), "world");
    }

    /// Exercises the `summed { ... }` arm of `array_slice_dyn!` via
    /// `MjvScene::flexface`, `flexnormal`, and `flextexcoord`. A model with no
    /// flex bodies must produce empty slices for all three.
    #[test]
    fn test_array_slice_dyn_summed_flex_empty() {
        let model = load_model();
        let scene = MjvScene::new(&model, 1000);
        assert!(scene.flexface().is_empty(), "flexface must be empty with no flex bodies");
        assert!(scene.flexnormal().is_empty(), "flexnormal must be empty with no flex bodies");
        assert!(scene.flextexcoord().is_empty(), "flextexcoord must be empty with no flex bodies");
    }
}
