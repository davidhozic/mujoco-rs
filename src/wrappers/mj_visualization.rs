//! Definitions related to visualization.
use std::default::Default;
use std::mem::MaybeUninit;
use std::ptr;

use glfw::ffi::{glfwGetCurrentContext, glfwGetWindowSize};


use super::mj_rendering::{MjrContext, MjrRectangle};
use super::mj_model::MjModel;
use super::mj_data::MjData;
use crate::mujoco_c::*;


/***********************************************************************************************************************
** MjvScene
***********************************************************************************************************************/
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
        unsafe { std::slice::from_raw_parts(self.ffi.geoms, self.ffi.ngeom as usize) }
    }

    pub fn geoms_mut(&mut self) -> &mut [MjvGeom] {
        unsafe { std::slice::from_raw_parts_mut(self.ffi.geoms, self.ffi.ngeom as usize) }
    }

    pub fn lights(&self) -> &[MjvLight] {
        &self.ffi.lights[..self.ffi.nlight as usize]
    }

    pub fn lights_mut(&mut self) -> &mut [MjvLight] {
        &mut self.ffi.lights[..self.ffi.nlight as usize]
    }

    pub fn update(&mut self, data: &mut MjData, opt: &MjvOption, cam: &mut MjvCamera) {
        unsafe {
            mjv_updateScene(
                self.model.ffi(), data.ffi_mut(), opt, ptr::null(),
                cam, mjtCatBit__mjCAT_ALL as i32, &mut self.ffi
            );
        }
    }

    /// Creates a new [`MjvGeom`] inside the scene. A reference is returned for additional modification,
    /// however it must be dropped before any additional calls to this method or any other methods.
    /// The return reference's lifetime is bound to the lifetime of self.
    pub fn create_geom<'s>(
        &'s mut self, geom_type: mjtGeom, size: Option<[f64; 3]>,
        pos: Option<[f64; 3]>, mat: Option<[f64; 9]>, rgba: Option<[f32; 4]>
    ) -> &'s mut MjvGeom {
        assert!(self.ffi.ngeom < self.ffi.maxgeom);

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
    pub fn render(&mut self, viewport: &MjrRectangle, context: &MjrContext) -> Vec<u8> {
        unsafe {
            /* Read window size */
            let window = glfwGetCurrentContext();
            let mut width  = 0;
            let mut height = 0;
            glfwGetWindowSize(window, &mut width, &mut height);

            let mut output = vec![0; width as usize * height as usize * 3];  // width * height * RGB

            mjr_render(viewport.clone(), self.ffi_mut(), context.ffi());
            self.read_pixels(&mut output, viewport, context);
            output
        }
    }

    /// Reads the render scene and writes the image data to `output`. The size of the `output` must 
    /// be `width * height * 3`, where `width` and `height` are the sizes of the current glfw window
    /// context.
    pub fn read_pixels(&self, output: &mut [u8], viewport: &MjrRectangle, context: &MjrContext) {
        unsafe {
            mjr_readPixels(output.as_mut_ptr(), ptr::null_mut(), viewport.clone(), context.ffi())
        }
    }

    #[allow(unused)]
    pub(crate) fn ffi(&self) -> &mjvScene {
        &self.ffi
    }

    pub(crate) fn ffi_mut(&mut self) -> &mut mjvScene {
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


/***********************************************************************************************************************
** MjvCamera
***********************************************************************************************************************/
pub type MjvCamera = mjvCamera;
impl MjvCamera {
    pub fn new(camera_id: isize, model: &MjModel) -> Self {
        let mut camera = Self::default();

        camera.fixedcamid = camera_id as i32;
        if camera_id == -1 {  // free camera
            camera.type_ = mjtCamera__mjCAMERA_FREE as i32;
            unsafe { mjv_defaultFreeCamera(model.ffi(), &mut camera); }
        }
        else {
            camera.type_ = mjtCamera__mjCAMERA_FIXED as i32;
        }

        camera
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

/***********************************************************************************************************************
** mjvOption
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
** mjvGeom
***********************************************************************************************************************/
pub type MjvGeom = mjvGeom;
impl MjvGeom {
    /// Wrapper around the MuJoCo's mjv_connector function.
    pub fn connect(&mut self, width: f64, from: [f64; 3], to: [f64; 3]) {
        unsafe {
            mjv_connector(self, self.type_, width, from.as_ptr(), to.as_ptr());
        }
    }

    pub fn label(&self) -> String {
        let len = self.label.iter().position(|&c| c == 0).unwrap_or(self.label.len());
        let bytes: &[u8] = unsafe { std::slice::from_raw_parts(self.label.as_ptr() as *const u8, len) };
        String::from_utf8_lossy(bytes).to_string()
    }

    pub fn set_label(&mut self, s: &str) {
        assert!(s.len() < self.label.len());
        for (i, &b) in s.as_bytes().iter().enumerate() {
            self.label[i] = b as i8;
        }
        self.label[s.len()] = 0;
    }
}

/***********************************************************************************************************************
** mjvLight
***********************************************************************************************************************/
pub type MjvLight = mjvLight;

/***********************************************************************************************************************
** mjvGLCamera
***********************************************************************************************************************/
pub type MjvGLCamera = mjvGLCamera;


#[cfg(test)]
mod tests {
    use super::*;

    const MODEL_PATH: &str = "/home/davidhozic/repo/FuzbAI/rust/fuzbai-simulation/models/miza.xml";

    /* Tests setup */
    fn load_model() -> MjModel {
        let model = MjModel::from_xml(MODEL_PATH).unwrap();
        model
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_MjvGeom() {
        let model = load_model();
        let mut scene = MjvScene::new(&model, 1000);
        
        /* Test label handling. Other things are trivial one-liners. */
        let geom = scene.create_geom(mjtGeom__mjGEOM_SPHERE, None, None, None, None);
        let label = "Hello World";
        geom.set_label(label);
        assert_eq!(geom.label(), label);
    }
}