//! Wrappers or type aliases around primitive types.
use crate::mujoco_c::*;

/***********************************************************************************************************************
** MjtSize
***********************************************************************************************************************/
/// Signed 64-bit integer used for sizes, counts, and other integer dimensions in MuJoCo.
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
/// Byte type (`unsigned char`) used for boolean flags and raw byte data (e.g. texture buffers) in MuJoCo.
pub type MjtByte = mjtByte;

