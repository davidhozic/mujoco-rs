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
use crate::getter_setter;
use crate::mujoco_c::*;


/// These are the available categories of geoms in the abstract visualizer. The bitmask can be used in the function
/// `mjr_render` to specify which categories should be rendered.
pub type MjtCatBit = mjtCatBit;

/// These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
/// and translate them into these actions, as illustrated in `simulate.cc <saSimulate>`.
pub type MjtMouse = mjtMouse;

/// These bitmasks enable the translational and rotational components of the mouse perturbation. For the regular mouse,
/// only one can be enabled at a time. For the 3D mouse (SpaceNavigator) both can be enabled simultaneously. They are used
/// in ``mjvPerturb.active``.
pub type MjtPertBit = mjtPertBit;

/// These are the possible camera types, used in ``mjvCamera.type``.
pub type MjtCamera = mjtCamera;
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

/// These are the abstract visualization elements that can have text labels. Used in ``mjvOption.label``.
pub type MjtLabel = mjtLabel;

/// These are the MuJoCo objects whose spatial frames can be rendered. Used in ``mjvOption.frame``.
pub type MjtFrame = mjtFrame;

/// These are indices in the array ``mjvOption.flags``, whose elements enable/disable the visualization of the
/// corresponding model or decoration element.
pub type MjtVisFlag = mjtVisFlag;

/// These are indices in the array ``mjvScene.flags``, whose elements enable/disable OpenGL rendering effects.
pub type MjtRndFlag = mjtRndFlag;

/// These are the possible stereo rendering types. They are used in ``mjvScene.stereo``.
pub type MjtStereo = mjtStereo;

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
    pub fn start<M: Deref<Target = MjModel>>(&mut self, type_: MjtPertBit, model: &MjModel, data: &mut MjData<M>, scene: &MjvScene<M>) {
        unsafe { mjv_initPerturb(model.ffi(), data.ffi_mut(), scene.ffi(), self); }
        self.active = type_ as i32;
    }

    /// Move an object with mouse. This is a wrapper around `mjv_movePerturb`.
    pub fn move_<M: Deref<Target = MjModel>>(&mut self, model: &MjModel, data: &mut MjData<M>, action: MjtMouse, dx: MjtNum, dy: MjtNum, scene: &MjvScene<M>) {
        unsafe { mjv_movePerturb(model.ffi(), data.ffi(), action as i32, dx, dy, scene.ffi(), self); }
    }

    pub fn apply<M: Deref<Target = MjModel>>(&mut self, model: &MjModel, data: &mut MjData<M>) {
        unsafe {
            mju_zero(data.ffi_mut().xfrc_applied, 6 * model.ffi().nbody);
            mjv_applyPerturbPose(model.ffi(), data.ffi_mut(), self, 0);
            mjv_applyPerturbForce(model.ffi(), data.ffi_mut(), self);
        }
    }

    pub fn update_local_pos<M: Deref<Target = MjModel>>(&mut self, selection_xyz: [MjtNum; 3], data: &MjData<M>) {
        let mut tmp = [0.0; 3];
        let data_ffi = data.ffi();
        unsafe { 
            mju_sub3(tmp.as_mut_ptr(), selection_xyz.as_ptr(), data_ffi.xpos.add(3 * self.select as usize));
            mju_mulMatTVec(self.localpos.as_mut_ptr(), data_ffi.xmat.add(9 * self.select as usize), tmp.as_ptr(), 3, 3);
        }
    }
}



/***********************************************************************************************************************
** MjvCamera
***********************************************************************************************************************/
pub type MjvCamera = mjvCamera;
impl MjvCamera {
    /// Deprecated method. Use one of:
    /// - [`MjvCamera::new_free`],
    /// - [`MjvCamera::new_fixed`],
    /// - [`MjvCamera::new_tracking`],
    /// - [`MjvCamera::new_user`].
    #[deprecated]
    pub fn new(camera_id: u32, type_: MjtCamera, model: &MjModel) -> Self {
        match type_ {
            MjtCamera::mjCAMERA_FIXED => Self::new_fixed(camera_id),
            MjtCamera::mjCAMERA_TRACKING => Self::new_tracking(camera_id),
            MjtCamera::mjCAMERA_FREE => Self::new_free(model),
            MjtCamera::mjCAMERA_USER => Self::new_user()
        }
    }

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
    /// Wrapper around the MuJoCo's mjv_connector function.
    /// Calculates the geom attributes so that it points from point `from` to point `to`.
    pub fn connect(&mut self, width: MjtNum, from: [MjtNum; 3], to: [MjtNum; 3]) {
        unsafe {
            mjv_connector(self, self.type_, width, from.as_ptr(), to.as_ptr());
        }
    }

    /// Compatibility method to convert the ``label`` attribute into a ``String``.
    pub fn label(&self) -> String {
        let len = self.label.iter().position(|&c| c == 0).unwrap_or(self.label.len());
        let bytes: &[u8] = unsafe { std::slice::from_raw_parts(self.label.as_ptr() as *const u8, len) };
        String::from_utf8_lossy(bytes).to_string()
    }

    /// Compatibility method to convert the ``s`` parameter into an array that is copied to the ``label`` attribute.
    pub fn set_label(&mut self, s: &str) {
        assert!(s.len() < self.label.len());
        for (i, b) in s.chars().enumerate() {
            self.label[i] = b as i8;
        }
        self.label[s.len()] = 0;
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
        let mut opt = MaybeUninit::uninit();
        unsafe {
            mjv_defaultFigure(opt.as_mut_ptr());
            opt.assume_init()
        }
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

    /// Deprecated alias for [`MjvFigure::draw`].
    #[deprecated(since = "2.3.0", note = "replaced with MjvFigure::draw")]
    pub fn figure(&mut self, viewport: MjrRectangle, context: &MjrContext) {
        unsafe { mjr_figure(viewport,self, context.ffi()) };
    }

    /// Draws the 2D figure to the `viewport` on screen.
    pub fn draw(&mut self, viewport: MjrRectangle, context: &MjrContext) {
        unsafe { mjr_figure(viewport,self, context.ffi()) };
    }
}

/// Figure options.
impl MjvFigure {
    getter_setter! {with, get, set, [
        flg_legend: bool; "whether to show legend.";
        flg_extend: bool; "whether to automatically extend axis ranges to fit data.";
        flg_barplot: bool; "whether to isolate line segments.";
        flg_selection: bool; "whether to show vertical selection line.";
        flg_symmetric: bool; "whether to make y-axis symmetric";
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
        xformat; " the x-axis C's printf format (e.g., `%.1f`).";
        yformat; " the y-axis C's printf format (e.g., `%.1f`).";
        linename [plot_index: usize]; " the line name of plot with `plot_index`.";
    }}
}

/// Plot data manipulation
impl MjvFigure {

    /// Checks if the buffer is full for plot with `plot_index`.
    pub fn full(&self, plot_index: usize) -> bool {
        self.linepnt[plot_index] >= (self.linedata[plot_index].len() / 2) as i32
    }

    /// Checks if the buffer is empty for plot with `plot_index`.
    pub fn empty(&self, plot_index: usize) -> bool {
        self.linepnt[plot_index] == 0
    }

    /// Pushes a new data point to buffer for the specific plot with `plot_index`.
    /// # Panics
    /// A panic will occur if the buffer is overflown. The buffer can hold a maximum of 1001 elements.
    pub fn push(&mut self, plot_index: usize, x: f32, y: f32) {
        let plot = &mut self.linedata[plot_index];
        let point_index = self.linepnt[plot_index] as usize;
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;

        self.linepnt[plot_index] += 1;
    }

    /// Overrides existing data with a new data point at a specific `point_index` for specific plot with `plot_index`.
    /// # Panics
    /// The data must already be present at `point_index`, otherwise an assertion panic will occur.
    pub fn set_at(&mut self, plot_index: usize, point_index: usize, x: f32, y: f32) {
        assert!(
            point_index < self.linepnt[plot_index] as usize,
            "data does not yet exist at index {point_index} for plot {plot_index}"
        );

        let plot = &mut self.linedata[plot_index];
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;
    }

    /// Clears the plot with `maybe_plot_index`.
    /// If `maybe_plot_index` is [`None`], all plots will be cleared.
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
    pub fn cut_front(&mut self, plot_index: usize, n: usize) {
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return;
        }

        self.linedata[plot_index].copy_within(2 * n..(len as usize * 2), 0);
        self.linepnt[plot_index] -= n as i32;
    }

    /// Cuts last first `n` elements from the plot data of plot with `plot_index`.
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
/// a immutable reference is stored inside this struct.
#[derive(Debug)]
pub struct MjvScene<M: Deref<Target = MjModel>> {
    ffi: mjvScene,
    model: M,
}

impl<M: Deref<Target = MjModel>> MjvScene<M> {
    pub fn new(model: M, max_geom: usize) -> Self {
        let scn = unsafe {
            let mut t = MaybeUninit::uninit();
            mjv_defaultScene(t.as_mut_ptr());
            mjv_makeScene(model.ffi(), t.as_mut_ptr(), max_geom as i32);
            t.assume_init()
        };

        Self {
            ffi: scn, model: model,
        }
    }

    pub fn update(&mut self, data: &mut MjData<M>, opt: &MjvOption, pertub: &MjvPerturb, cam: &mut MjvCamera) {
        unsafe {
            mjv_updateScene(
                self.model.ffi(), data.ffi_mut(), opt, pertub,
                cam, MjtCatBit::mjCAT_ALL as i32, &mut self.ffi
            );
        }
    }

    /// Creates a new [`MjvGeom`] inside the scene. A reference is returned for additional modification,
    /// however it must be dropped before any additional calls to this method or any other methods.
    /// The return reference's lifetime is bound to the lifetime of self.
    /// # Panics
    /// When the allocated space for geoms is full.
    pub fn create_geom<'s>(
        &'s mut self, geom_type: MjtGeom, size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>, mat: Option<[MjtNum; 9]>, rgba: Option<[f32; 4]>
    ) -> &'s mut MjvGeom {
        assert!(self.ffi.ngeom < self.ffi.maxgeom, "not enough space is allocated, increase 'max_geom'.");

        /* Gain raw pointers to data inside the Option enum (which is a C union) */
        let size_ptr = size.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let pos_ptr = pos.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let mat_ptr = mat.as_ref().map_or(ptr::null(), |x| x.as_ptr());
        let rgba_ptr = rgba.as_ref().map_or(ptr::null(), |x| x.as_ptr());

        let p_geom;
        unsafe {
            p_geom = self.ffi.geoms.add(self.ffi.ngeom as usize);
            mjv_initGeom(p_geom, geom_type as i32, size_ptr, pos_ptr, mat_ptr, rgba_ptr);
            self.ffi.ngeom += 1;
            p_geom.as_mut().unwrap()
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
    /// The method returns a tuple: (body_id, geom_id, flex_id, skin_id, xyz coordinates of the point)
    pub fn find_selection(
        &self, data: &MjData<M>, option: &MjvOption,
        aspect_ratio: MjtNum, relx: MjtNum, rely: MjtNum,
    ) -> (i32, i32, i32, i32, [MjtNum; 3]) {
        let (mut geom_id, mut flex_id, mut skin_id) = (-1 , -1, -1);
        let mut selpnt = [0.0; 3];
        let body_id = unsafe {
            mjv_select(
                self.model.ffi(), data.ffi(), option,
                aspect_ratio, relx, rely, self.ffi(), selpnt.as_mut_ptr(),
                &mut geom_id, &mut flex_id, &mut skin_id
            )
        };
        (body_id, geom_id, flex_id, skin_id, selpnt)
    }

    pub fn ffi(&self) -> &mjvScene {
        &self.ffi
    }

    pub unsafe fn ffi_mut(&mut self) -> &mut mjvScene {
        &mut self.ffi
    }
}

/// Array slices.
impl<M: Deref<Target = MjModel>> MjvScene<M> {
    // Scalar length arrays
    array_slice_dyn! {
        flexedge: &[[i32; 2] [cast]; "flex edge data"; model.ffi().nflexedge],
        flexvert: &[[f32; 3] [cast]; "flex vertices"; model.ffi().nflexvert],
        skinvert: &[[f32; 3] [cast]; "skin vertex data"; model.ffi().nskinvert],
        skinnormal: &[[f32; 3] [cast]; "skin normal data"; model.ffi().nskinvert],
        geoms: &[MjvGeom; "buffer for geoms"; ffi.ngeom],
        geomorder: &[i32; "buffer for ordering geoms by distance to camera"; ffi.ngeom],
        flexedgeadr: &[i32; "address of flex edges"; ffi.nflex],
        flexedgenum: &[i32; "number of edges in flex"; ffi.nflex],
        flexvertadr: &[i32; "address of flex vertices"; ffi.nflex],
        flexvertnum: &[i32; "number of vertices in flex"; ffi.nflex],
        flexfaceadr: &[i32; "address of flex faces"; ffi.nflex],
        flexfacenum: &[i32; "number of flex faces allocated"; ffi.nflex],
        flexfaceused: &[i32; "number of flex faces currently in use"; ffi.nflex],
        skinfacenum: &[i32; "number of faces in skin"; ffi.nskin],
        skinvertadr: &[i32; "address of skin vertices"; ffi.nskin],
        skinvertnum: &[i32; "number of vertices in skin"; ffi.nskin],
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
            mjv_freeScene(&mut self.ffi);
        }
    }
}

unsafe impl<M: Deref<Target = MjModel>> Send for MjvScene<M> {}
unsafe impl<M: Deref<Target = MjModel>> Sync for MjvScene<M> {}

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
        let geom = scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None);
        let label = "Hello World";
        geom.set_label(label);
        assert_eq!(geom.label(), label);
    }

    #[test]
    fn test_scene_geom_pop() {
        const N_GEOM: usize = 10;
        const N_GEOM_POP: usize = 11;

        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);

        for _ in 0..N_GEOM {
            scene.create_geom(MjtGeom::mjGEOM_SPHERE, None, None, None, None);
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
}
