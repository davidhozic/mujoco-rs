//! Definition of MjOption.
use crate::mujoco_c::{mjOption, mj_defaultOption};
pub type MjOption = mjOption;

impl Default for MjOption {
    fn default() -> Self {
        unsafe {
            let mut opt = std::mem::MaybeUninit::uninit();
            mj_defaultOption(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
}
