//! Wrappers or type aliases around primitive types.
use crate::mujoco_c::*;

/***********************************************************************************************************************
** MjtSize
***********************************************************************************************************************/
/// Signed 64-bit integer used for memory sizes in MuJoCo.
pub type MjtSize = mjtSize;

/***********************************************************************************************************************
** MjtNum
***********************************************************************************************************************/
/// Floating-point scalar type used throughout MuJoCo. This binding is compiled against the
/// double-precision build (`f64`); a single-precision build would yield `f32`.
pub type MjtNum = mjtNum;

/***********************************************************************************************************************
** MjtByte
***********************************************************************************************************************/
/// Boolean flag type (`unsigned char`) used for true/false values in MuJoCo.
pub type MjtByte = mjtByte;

