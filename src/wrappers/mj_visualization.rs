//! Definitions related to visualization.
use std::default::Default;
use std::mem::MaybeUninit;
use std::ops::Deref;
use std::ptr;

use super::mj_rendering::{MjrContext, MjrRectangle};
use super::mj_primitive::{MjtNum, MjtByte};
use super::mj_model::{MjModel, MjtGeom};
use super::mj_data::MjData;
use crate::{array_slice_dyn, c_str_as_str_method};
use crate::error::MjSceneError;
use crate::getter_setter;
use crate::mujoco_c::*;

/// Result of a mouse-based selection query via [`MjvScene::find_selection`].
#[derive(Debug, Clone, PartialEq)]
pub struct SceneSelection {
    /// Selected body id (-1 if none).
    pub body_id: i32,
    /// Selected geom id (-1 if none).
    pub geom_id: i32,
    /// Selected flex id (-1 if none).
    pub flex_id: i32,
    /// Selected skin id (-1 if none).
    pub skin_id: i32,
    /// 3D world coordinates of the selection point.
    pub point: [MjtNum; 3],
}

impl Default for SceneSelection {
    fn default() -> Self {
        Self {
            body_id: -1,
            geom_id: -1,
            flex_id: -1,
            skin_id: -1,
            point: [0.0; 3],
        }
    }
}

/* Types */
/// These are the available categories of geoms in the abstract visualizer. The bitmask can be used in the function
/// `mjr_render` to specify which categories should be rendered.
pub type MjtCatBit = mjtCatBit;

/// These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
/// and translate them into these actions, as illustrated in `simulate.cc <saSimulate>`.
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
    type Error = ();
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::mjCAMERA_FREE),
            1 => Ok(Self::mjCAMERA_TRACKING),
            2 => Ok(Self::mjCAMERA_FIXED),
            3 => Ok(Self::mjCAMERA_USER),
            _ => Err(())
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
pub type MjvPerturb = mjvPerturb;
impl Default for MjvPerturb {
    fn default() -> Self {
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
    pub fn start<M: Deref<Target = MjModel>>(&mut self, type_: MjtPertBit, model: &MjModel, data: &mut MjData<M>, scene: &MjvScene<M>) {
        unsafe { mjv_initPerturb(model.ffi(), data.ffi_mut(), scene.ffi(), self); }
        self.active = type_ as i32;
    }

    /// Move an object with mouse. This is a wrapper around `mjv_movePerturb`.
    pub fn move_<M: Deref<Target = MjModel>>(&mut self, data: &MjData<M>, action: MjtMouse, dx: MjtNum, dy: MjtNum, scene: &MjvScene<M>) {
        unsafe { mjv_movePerturb(data.model().ffi(), data.ffi(), action as i32, dx, dy, scene.ffi(), self); }
    }

    /// Apply perturbation pose and force.
    ///
    /// # Note
    /// This method **zeroes `xfrc_applied`** for all bodies before applying the perturbation
    /// force. Any external forces set on `data` before calling this method will be cleared.
    /// If you need to preserve external forces, apply them *after* calling this method.
    pub fn apply<M: Deref<Target = MjModel>>(&mut self, model: &MjModel, data: &mut MjData<M>) {
        unsafe {
            data.xfrc_applied_mut().fill([0.0; 6]);
            mjv_applyPerturbPose(model.ffi(), data.ffi_mut(), self, 0);
            mjv_applyPerturbForce(model.ffi(), data.ffi_mut(), self);
        }
    }

    /// Updates the body-local position of the selection point.
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
    pub fn new_fixed(camera_id: u32) -> Self {
        let mut camera: mjvCamera_ = Self::default();
        camera.type_ = MjtCamera::mjCAMERA_FIXED as i32;
        camera.fixedcamid = camera_id as i32;
        camera
    }

    /// Creates a new tracking camera to track a body with the given `tracking_id`.
    pub fn new_tracking(tracking_id: u32) -> Self {
        let mut camera: mjvCamera_ = Self::default();
        camera.type_ = MjtCamera::mjCAMERA_TRACKING as i32;
        camera.trackbodyid = tracking_id as i32;
        camera
    }

    /// Creates a new camera of user type.
    pub fn new_user() -> Self {
        let mut cam = Self::default();
        cam.type_ = MjtCamera::mjCAMERA_USER as i32;
        cam
    }

    /// Sets the camera into tracking mode.
    pub fn track(&mut self, tracking_id: u32) {
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
    pub fn fix(&mut self, camera_id: u32) {
        self.type_ = MjtCamera::mjCAMERA_FIXED as i32;
        self.fixedcamid = camera_id as i32;
        self.trackbodyid = -1;
    }

    /// Move camera with mouse.
    pub fn move_<M: Deref<Target = MjModel>>(&mut self, action: MjtMouse, model: &MjModel, dx: MjtNum, dy: MjtNum, scene: &MjvScene<M>) {
        unsafe { mjv_moveCamera(model.ffi(), action as i32, dx, dy, scene.ffi(), self); };
    }

    /// Get the camera coordinate frame (pos, forward, up, right).
    pub fn frame<M: Deref<Target = MjModel>>(&self, data: &MjData<M>) -> ([MjtNum; 3], [MjtNum; 3], [MjtNum; 3], [MjtNum; 3]) {
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
pub type MjvGLCamera = mjvGLCamera;

impl MjvGLCamera {
    /// Average the current MjvGLCamera with the `other` MjvGLCamera.
    pub fn average_camera(&self, other: &Self) -> Self {
        unsafe { mjv_averageCamera (self, other) }
    }
}

/***********************************************************************************************************************
** MjvGeom
***********************************************************************************************************************/
pub type MjvGeom = mjvGeom;

impl MjvGeom {
    /// Sets the geom so that it acts as a connector (line, arrow, etc.) between
    /// two 3D points.
    ///
    /// This is a wrapper around MuJoCo's `mjv_connector`. The connector type
    /// is taken from the geom's current [`type_`](MjvGeom::type_) field, so
    /// set it to the desired connector type (e.g. `mjGEOM_LINE`, `mjGEOM_ARROW`)
    /// **before** calling this method, or initialize the geom
    /// with that type via [`MjvScene::create_geom`].
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
pub type MjvLight = mjvLight;

/***********************************************************************************************************************
** MjvOption
***********************************************************************************************************************/
pub type MjvOption = mjvOption;
impl Default for MjvOption {
    fn default() -> Self {
        let mut opt = MaybeUninit::uninit();
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
        *Self::new()
    }
}

impl MjvFigure {
    /// Instantiates a new figure with default values.
    pub fn new() -> Box<Self> {
        let mut opt = Box::new(MaybeUninit::uninit());
        unsafe {
            mjv_defaultFigure(opt.as_mut_ptr());
            opt.assume_init()
        }
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
    pub fn full(&self, plot_index: usize) -> bool {
        self.linepnt[plot_index] >= (self.linedata[plot_index].len() / 2) as i32
    }

    /// Checks if the buffer is empty for plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    pub fn empty(&self, plot_index: usize) -> bool {
        self.linepnt[plot_index] == 0
    }

    /// Pushes a new data point to buffer for the specific plot with `plot_index`.
    ///
    /// # Panics
    /// A panic will occur if the buffer is full.
    /// Use [`MjvFigure::try_push`] for a fallible alternative.
    pub fn push(&mut self, plot_index: usize, x: f32, y: f32) {
        self.try_push(plot_index, x, y)
            .expect("figure push failed")
    }

    /// Fallible version of [`MjvFigure::push`].
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureBufferFull`] if the buffer for
    /// `plot_index` is already at capacity.
    pub fn try_push(&mut self, plot_index: usize, x: f32, y: f32) -> Result<(), MjSceneError> {
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
    /// The data must already be present at `point_index`, otherwise an assertion panic will occur.
    /// Use [`MjvFigure::try_set_at`] for a fallible alternative.
    pub fn set_at(&mut self, plot_index: usize, point_index: usize, x: f32, y: f32) {
        self.try_set_at(plot_index, point_index, x, y)
            .expect("figure set_at failed")
    }

    /// Fallible version of [`MjvFigure::set_at`].
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureIndexOutOfBounds`] if `point_index` is
    /// not within the current data range for the given plot.
    pub fn try_set_at(
        &mut self,
        plot_index: usize,
        point_index: usize,
        x: f32,
        y: f32,
    ) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex { plot_index, max_plots: mjMAXLINE as usize });
        }
        let current_len = self.linepnt[plot_index] as usize;
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
    /// # Returns
    /// Returns [`Some(first element)`](Some) when plot contains any elements, otherwise [`None`] is returned.
    /// The return format is (x, y).
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    pub fn pop_front(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        let len = self.linepnt[plot_index];
        if len <= 0 {
            return None;
        }

        let plot_data = &mut self.linedata[plot_index];
        let first = (plot_data[0], plot_data[1]);

        plot_data.copy_within(2..len as usize * 2, 0);

        self.linepnt[plot_index] -= 1;

        Some(first)
    }

    /// Pops the last element from the plot data of plot with `plot_index`.
    /// # Returns
    /// Returns [`Some(last element)`](Some) when plot contains any elements, otherwise [`None`] is returned.
    /// The return format is (x, y).
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    pub fn pop_back(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        let old_len = self.linepnt[plot_index];
        if old_len <= 0 {
            return None;
        }
        let plot_data = &mut self.linedata[plot_index];
        let new_start = ((old_len - 1) * 2) as usize;
        self.linepnt[plot_index] -= 1;

        Some((plot_data[new_start], plot_data[new_start + 1]))  // new len is the previous last index
    }

    /// Cuts the first `n` elements from the plot data of plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    pub fn cut_front(&mut self, plot_index: usize, n: usize) {
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return;
        }

        self.linedata[plot_index].copy_within(2 * n..(len as usize * 2), 0);
        self.linepnt[plot_index] -= n as i32;
    }

    /// Cuts the last `n` elements from the plot data of plot with `plot_index`.
    pub fn cut_end(&mut self, plot_index: usize, n: usize) {
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return;
        }
        self.linepnt[plot_index] -= n as i32;
    }
}


/***********************************************************************************************************************
** MjvScene
***********************************************************************************************************************/
/// 3D scene visualization.
/// This struct provides a way to render visual-only geometry.
/// To prevent changes of array sizes in [`MjModel`], which can lead to overflows,
/// an immutable reference is stored inside this struct.
#[derive(Debug)]
pub struct MjvScene<M: Deref<Target = MjModel>> {
    ffi: Box<mjvScene>,
    model: M,
}

impl<M: Deref<Target = MjModel>> MjvScene<M> {
    /// Creates a new scene for `model`, allocating space for up to `max_geom` geoms.
    pub fn new(model: M, max_geom: usize) -> Self {
        let scn = unsafe {
            let mut t = Box::new_uninit();
            mjv_defaultScene(t.as_mut_ptr());
            mjv_makeScene(model.ffi(), t.as_mut_ptr(), max_geom as i32);
            t.assume_init()
        };

        Self {
            ffi: scn, model,
        }
    }

    /// Updates the scene from the current simulation state in `data`.
    ///
    /// The `catmask` parameter controls which geom categories are included
    /// (e.g., [`MjtCatBit::mjCAT_ALL`] for everything, or a bitwise OR of
    /// [`MjtCatBit::mjCAT_STATIC`], [`MjtCatBit::mjCAT_DYNAMIC`], [`MjtCatBit::mjCAT_DECOR`]).
    pub fn update_with_catmask(
        &mut self, data: &mut MjData<M>, opt: &MjvOption, perturb: &MjvPerturb,
        cam: &mut MjvCamera, catmask: i32,
    ) {
        unsafe {
            mjv_updateScene(
                self.model.ffi(), data.ffi_mut(), opt, perturb,
                cam, catmask, self.ffi.as_mut()
            );
        }
    }

    /// Updates the scene from the current simulation state in `data`, including all geom categories.
    ///
    /// This is equivalent to calling [`update_with_catmask`](Self::update_with_catmask) with
    /// [`MjtCatBit::mjCAT_ALL`].
    pub fn update(&mut self, data: &mut MjData<M>, opt: &MjvOption, perturb: &MjvPerturb, cam: &mut MjvCamera) {
        self.update_with_catmask(data, opt, perturb, cam, MjtCatBit::mjCAT_ALL as i32);
    }

    /// Creates a new [`MjvGeom`] in this scene, returning a mutable reference to it.
    /// The geom reference is bound to the scene's lifetime; it is invalidated when
    /// any code that might reallocate the geoms buffer runs.
    /// # Errors
    /// Returns [`MjSceneError::SceneFull`] when `ngeom >= maxgeom`.
    pub fn create_geom<'s>(
        &'s mut self, geom_type: MjtGeom, size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>, mat: Option<[MjtNum; 9]>, rgba: Option<[f32; 4]>
    ) -> Result<&'s mut MjvGeom, MjSceneError> {
        if self.ffi.ngeom >= self.ffi.maxgeom {
            return Err(MjSceneError::SceneFull { capacity: self.ffi.maxgeom });
        }

        let size_ptr = size.as_ref().map_or(ptr::null(), |x| x);
        let pos_ptr  = pos.as_ref().map_or(ptr::null(), |x| x);
        let mat_ptr  = mat.as_ref().map_or(ptr::null(), |x| x);
        let rgba_ptr = rgba.as_ref().map_or(ptr::null(), |x| x);

        // SAFETY: ngeom < maxgeom guarantees we are within the allocated buffer.
        unsafe {
            let p_geom = self.ffi.geoms.add(self.ffi.ngeom as usize);
            mjv_initGeom(p_geom, geom_type as i32, size_ptr, pos_ptr, mat_ptr, rgba_ptr);
            self.ffi.ngeom += 1;
            // Safety: p_geom is guaranteed non-null (allocated by mjv_makeScene).
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

    /// Renders the scene to the screen. This does not automatically make the OpenGL context current.
    pub fn render(&mut self, viewport: &MjrRectangle, context: &MjrContext){
        unsafe {
            mjr_render(viewport.clone(), self.ffi_mut(), context.ffi());
        }
    }


    /// Returns the selection point based on a mouse click.
    /// This is a wrapper around `mjv_select()`.
    pub fn find_selection(
        &self, data: &MjData<M>, option: &MjvOption,
        aspect_ratio: MjtNum, relx: MjtNum, rely: MjtNum,
    ) -> SceneSelection {
        let (mut geom_id, mut flex_id, mut skin_id) = (-1 , -1, -1);
        let mut selpnt = [0.0; 3];
        let body_id = unsafe {
            mjv_select(
                self.model.ffi(), data.ffi(), option,
                aspect_ratio, relx, rely, self.ffi(), &mut selpnt,
                &mut geom_id, &mut flex_id, &mut skin_id
            )
        };
        SceneSelection { body_id, geom_id, flex_id, skin_id, point: selpnt }
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
impl<M: Deref<Target = MjModel>> MjvScene<M> {
    // Scalar length arrays
    array_slice_dyn! {
        flexedge: &[[i32; 2] [force]; "flex edge data"; model.ffi().nflexedge],
        flexvert: &[[f32; 3] [force]; "flex vertices"; model.ffi().nflexvert],
        skinvert: &[[f32; 3] [force]; "skin vertex data"; model.ffi().nskinvert],
        skinnormal: &[[f32; 3] [force]; "skin normal data"; model.ffi().nskinvert],
        geoms: &[MjvGeom; "buffer for geoms"; ffi.ngeom],
        geomorder: &[i32; "buffer for ordering geoms by distance to camera"; ffi.ngeom],
        (allow_mut = false) flexedgeadr: &[i32; "address of flex edges"; ffi.nflex],
        (allow_mut = false) flexedgenum: &[i32; "number of edges in flex"; ffi.nflex],
        (allow_mut = false) flexvertadr: &[i32; "address of flex vertices"; ffi.nflex],
        (allow_mut = false) flexvertnum: &[i32; "number of vertices in flex"; ffi.nflex],
        (allow_mut = false) flexfaceadr: &[i32; "address of flex faces"; ffi.nflex],
        (allow_mut = false) flexfacenum: &[i32; "number of flex faces allocated"; ffi.nflex],
        (allow_mut = false) flexfaceused: &[i32; "number of flex faces currently in use"; ffi.nflex],
        (allow_mut = false) skinfacenum: &[i32; "number of faces in skin"; ffi.nskin],
        (allow_mut = false) skinvertadr: &[i32; "address of skin vertices"; ffi.nskin],
        (allow_mut = false) skinvertnum: &[i32; "number of vertices in skin"; ffi.nskin],
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
impl<M: Deref<Target = MjModel>> MjvScene<M> {
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

    getter_setter! {force!, with, get, set, [
        [ffi, ffi_mut] stereo: MjtStereo; "stereoscopic rendering.";
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


impl<M: Deref<Target = MjModel>> Drop for MjvScene<M> {
    fn drop(&mut self) {
        unsafe {
            mjv_freeScene(self.ffi.as_mut());
        }
    }
}

// SAFETY: MjvScene<M> owns the FFI scene data exclusively. It is safe to send across
// threads as long as M itself is Send (e.g. Arc<MjModel>). Non-Send M types such as
// Rc<MjModel> are correctly excluded by the M: Send / M: Sync bounds.
unsafe impl<M: Deref<Target = MjModel> + Send> Send for MjvScene<M> {}
unsafe impl<M: Deref<Target = MjModel> + Sync> Sync for MjvScene<M> {}

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
        let model = MjModel::from_xml_string(EXAMPLE_MODEL).unwrap();
        model
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_MjvGeom() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);
        
        /* Test label handling. Other things are trivial one-liners. */
        let geom = scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None).unwrap();
        let label = "Hello World";
        geom.set_label(label).unwrap();
        assert_eq!(geom.label(), label);
    }

    #[test]
    fn test_scene_geom_pop() {
        const N_GEOM: usize = 10;
        const N_GEOM_POP: usize = 11;

        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);

        for _ in 0..N_GEOM {
            scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None).unwrap();
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
        let mut fig = MjvFigure::new();
        let plot = 0;

        // Initially empty
        assert!(fig.empty(plot));
        assert_eq!(fig.pop_front(plot), None);
        assert_eq!(fig.pop_back(plot), None);

        // Push two points
        fig.push(plot, 1.0, 2.0);
        fig.push(plot, 3.0, 4.0);

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

        // Fill to capacity
        scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None).unwrap();
        scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None).unwrap();

        // Next should fail
        let err = scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None).unwrap_err();
        assert!(matches!(err, MjSceneError::SceneFull { capacity: 2 }));
    }

    /// Tests `set_label` boundary: too-long label returns `LabelTooLong`,
    /// max-length label succeeds with roundtrip.
    #[test]
    fn test_geom_label_boundary() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 10);
        let geom = scene.create_geom(MjtGeom::mjGEOM_BOX, None, None, None, None).unwrap();

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

    /// Tests `try_push` returns `FigureBufferFull` at capacity,
    /// and push succeeds after `pop_back` frees a slot.
    #[test]
    fn test_figure_try_push_overflow() {
        let mut fig = MjvFigure::new();
        let plot = 0;
        let capacity = fig.linedata[plot].len() / 2;

        // Fill to capacity
        for i in 0..capacity {
            fig.try_push(plot, i as f32, i as f32).unwrap();
        }
        assert!(fig.full(plot));

        // Next push should fail
        let err = fig.try_push(plot, 0.0, 0.0).unwrap_err();
        assert!(matches!(err, MjSceneError::FigureBufferFull { plot_index: 0, .. }));

        // Pop one element, then push should succeed again
        fig.pop_back(plot);
        assert!(!fig.full(plot));
        fig.try_push(plot, 99.0, 99.0).unwrap();
        assert!(fig.full(plot));
    }
}
