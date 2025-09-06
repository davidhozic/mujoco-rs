//! Module containing safe wrappers around derivative functions.
use crate::wrappers::mj_primitive::*;
use crate::mujoco_c;
use std::ptr;

/// Derivatives of mju_subQuat.
/// Nullable: Da, Db.
/// Db = -Da^T
pub fn mjd_sub_quat(qa: &[MjtNum; 4], qb: &[MjtNum; 4], da: Option<&mut [MjtNum; 9]>, db: Option<&mut [MjtNum; 9]>)  {
    unsafe { mujoco_c::mjd_subQuat(
        qa.as_ptr(), qb.as_ptr(),
        da.map_or(ptr::null_mut(), |d| d.as_mut_ptr()),
        db.map_or(ptr::null_mut(), |d| d.as_mut_ptr())
    ) }
}

/// Derivatives of mju_quatIntegrate.
/// Nullable: Dquat, Dvel, Dscale.
pub fn mjd_quat_integrate(vel: &[MjtNum; 3], scale: MjtNum, dquat: Option<&mut [MjtNum; 9]>, dvel: Option<&mut [MjtNum; 9]>, dscale: Option<&mut [MjtNum; 3]>)  {
    unsafe { mujoco_c::mjd_quatIntegrate(
        vel.as_ptr(), scale,
        dquat.map_or(ptr::null_mut(), |d| d.as_mut_ptr()),
        dvel.map_or(ptr::null_mut(), |d| d.as_mut_ptr()),
        dscale.map_or(ptr::null_mut(), |d| d.as_mut_ptr(),)
    ) }
}



#[cfg(test)]
mod tests {
    use crate::assert_relative_eq;
    use super::*;

    #[test]
    fn test_sub_quat() {
        let qa: [MjtNum; 4] = [1.0, 0.0, 0.0, 0.0]; // identity quaternion
        let qb: [MjtNum; 4] = [0.0, 1.0, 0.0, 0.0]; // 180° rotation about x

        // Case 1: No derivatives (just check it doesn't crash)
        mjd_sub_quat(&qa, &qb, None, None);

        // Case 2: With derivatives
        let mut da = [0.0; 9];
        let mut db = [0.0; 9];
        mjd_sub_quat(&qa, &qb, Some(&mut da), Some(&mut db));

        // Check Db = -Da^T
        for i in 0..3 {
            for j in 0..3 {
                let da_ij = da[3 * i + j];
                let db_ij = db[3 * j + i]; // transpose
                assert_relative_eq!(db_ij, -da_ij, epsilon = 1e-9);
            }
        }
    }

    #[test]
    fn test_quat_integrate() {
        let vel: [MjtNum; 3] = [0.1, -0.2, 0.3];
        let scale: MjtNum = 0.5;

        // Case 1: No derivatives
        mjd_quat_integrate(&vel, scale, None, None, None);

        // Case 2: With all derivatives
        let mut dquat = [0.0; 9];
        let mut dvel = [0.0; 9];
        let mut dscale = [0.0; 3];
        mjd_quat_integrate(&vel, scale, Some(&mut dquat), Some(&mut dvel), Some(&mut dscale));

        // Sanity: Dquat should be close to identity when vel ≈ 0
        let vel_zero = [0.0, 0.0, 0.0];
        let mut dquat_zero = [0.0; 9];
        mjd_quat_integrate(&vel_zero, 0.0, Some(&mut dquat_zero), None, None);

        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(dquat_zero[3 * i + j], expected, epsilon = 1e-9);
            }
        }

        // Sanity: dscale should scale with vel (not all zeros)
        assert!(dscale.iter().any(|&x| x.abs() > 1e-12));
    }
}
