//! Definitions related to visualization.
use std::default::Default;
use std::mem::MaybeUninit;
use std::ptr;
use std::io;

use super::mj_rendering::{MjrContext, MjrRectangle};
use super::mj_model::{MjModel, MjtGeom};
use super::mj_data::MjData;
use crate::mujoco_c::*;


/// How much extra room to create in the internal [`MjvScene`]. Useful for drawing labels, etc.
pub(crate) const EXTRA_SCENE_GEOM_SPACE: usize = 100;

/***********************************************************************************************************************
** MjtCamera
***********************************************************************************************************************/
pub type MjtCamera = mjtCamera;

/***********************************************************************************************************************
** MjtMouse
***********************************************************************************************************************/
pub type MjtMouse = mjtMouse;

/***********************************************************************************************************************
** MjtPertBit
***********************************************************************************************************************/
pub type MjtPertBit = mjtPertBit;

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
    pub fn start(&mut self, type_: MjtPertBit, model: &MjModel, data: &mut MjData, scene: &MjvScene) {
        unsafe { mjv_initPerturb(model.ffi(), data.ffi_mut(), scene.ffi(), self); }
        self.active = type_ as i32;
    }

    /// Move an object with mouse. This is a wrapper around `mjv_movePerturb`.
    pub fn move_(&mut self, model: &MjModel, data: &mut MjData, action: MjtMouse, dx: mjtNum, dy: mjtNum, scene: &MjvScene) {
        unsafe { mjv_movePerturb(model.ffi(), data.ffi(), action as i32, dx, dy, scene.ffi(), self); }
    }

    pub fn apply(&mut self, model: &MjModel, data: &mut MjData) {
        unsafe {
            mju_zero(data.ffi_mut().xfrc_applied, 6 * model.ffi().nbody);
            mjv_applyPerturbPose(model.ffi(), data.ffi_mut(), self, 0);
            mjv_applyPerturbForce(model.ffi(), data.ffi_mut(), self);
        }
    }

    pub fn update_local_pos(&mut self, selection_xyz: [mjtNum; 3], data: &MjData) {
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

    /// Move camera with mouse.
    pub fn move_(&mut self, action: mjtMouse, model: &MjModel, dx: mjtNum, dy: mjtNum, scene: &MjvScene) {
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
    /// Calculates the geom attributes to result it pointing from point `from` to point `to`.
    pub fn connect(&mut self, width: f64, from: [f64; 3], to: [f64; 3]) {
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
** MjvOption
***********************************************************************************************************************/
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
    /// Draws the 2D figure.
    pub fn figure(&mut self, viewport: MjrRectangle, context: &MjrContext) {
        unsafe { mjr_figure(viewport,self, context.ffi()) };
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
pub struct MjvScene<'m> {
    ffi: mjvScene,
    model: &'m MjModel,
}

impl<'m> MjvScene<'m> {
    pub fn new(model: &'m MjModel, max_geom: usize) -> Self {
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

    pub fn geoms(&self) -> &[MjvGeom] {
        if self.ffi.ngeom == 0 {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.ffi.geoms, self.ffi.ngeom as usize) }
    }

    pub fn geoms_mut(&mut self) -> &mut [MjvGeom] {
        if self.ffi.ngeom == 0 {
            return &mut [];
        }
        unsafe { std::slice::from_raw_parts_mut(self.ffi.geoms, self.ffi.ngeom as usize) }
    }

    pub fn lights(&self) -> &[MjvLight] {
        if self.ffi.nlight == 0 {
            return &[];
        }
        &self.ffi.lights[..self.ffi.nlight as usize]
    }

    pub fn lights_mut(&mut self) -> &mut [MjvLight] {
        if self.ffi.nlight == 0 {
            return &mut [];
        }
        &mut self.ffi.lights[..self.ffi.nlight as usize]
    }

    pub fn update(&mut self, data: &mut MjData, opt: &MjvOption, pertub: &MjvPerturb, cam: &mut MjvCamera) {
        unsafe {
            mjv_updateScene(
                self.model.ffi(), data.ffi_mut(), opt, pertub,
                cam, mjtCatBit::mjCAT_ALL as i32, &mut self.ffi
            );
        }
    }

    /// Creates a new [`MjvGeom`] inside the scene. A reference is returned for additional modification,
    /// however it must be dropped before any additional calls to this method or any other methods.
    /// The return reference's lifetime is bound to the lifetime of self.
    pub fn create_geom<'s>(
        &'s mut self, geom_type: MjtGeom, size: Option<[f64; 3]>,
        pos: Option<[f64; 3]>, mat: Option<[f64; 9]>, rgba: Option<[f32; 4]>
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
        &self, data: &MjData, option: &MjvOption,
        aspect_ratio: mjtNum, relx: mjtNum, rely: mjtNum,
    ) -> (i32, i32, i32, i32, [mjtNum; 3]) {
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


impl Drop for MjvScene<'_> {
    fn drop(&mut self) {
        unsafe {
            mjv_freeScene(&mut self.ffi);
        }
    }
}


/// Copies geometry data (geoms only) from the `src` to `dst`.
/// # Errors
/// Returns an [`io::Error`] of kind [`io::ErrorKind::StorageFull`] if the destination scene does not have
/// enough space to accommodate the additional geoms from the source scene.
pub(crate) fn sync_geoms(src: &MjvScene, dst: &mut MjvScene) -> io::Result<()> {
    let ffi_src = src.ffi();
    let ffi_dst = unsafe { dst.ffi_mut() };
    let new_len = ffi_dst.ngeom + ffi_src.ngeom;

    if new_len > ffi_dst.maxgeom {
        return Err(io::Error::new(io::ErrorKind::StorageFull, "not enough space available in the destination scene"))
    }

    /* Fast copy */
    unsafe { std::ptr::copy_nonoverlapping(
        ffi_src.geoms,
        ffi_dst.geoms.add(ffi_dst.ngeom as usize),
        ffi_src.ngeom as usize
    ) };

    ffi_dst.ngeom = new_len;
    Ok(())
}


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
}
