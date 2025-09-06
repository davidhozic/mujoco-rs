//! Module containing safe wrappers around utility functions.
use crate::wrappers::mj_primitive::*;
use crate::mujoco_c;

/* Manually added */
pub fn mju_zero(res: &mut [MjtNum])  {
    unsafe { mujoco_c::mju_zero(res.as_mut_ptr(), res.len() as i32) }
}

pub fn mju_fill(res: &mut [MjtNum], val: MjtNum)  {
    unsafe { mujoco_c::mju_fill(res.as_mut_ptr(), val, res.len() as i32) }
}

pub fn mju_copy(res: &mut [MjtNum], vec: &mut [MjtNum])  {
    assert!(res.len() == vec.len());
    unsafe { mujoco_c::mju_copy(res.as_mut_ptr(), vec.as_mut_ptr(), res.len() as i32) }
}
pub fn mju_sum(vec: &[MjtNum]) -> MjtNum {
    unsafe { mujoco_c::mju_sum(vec.as_ptr(), vec.len() as i32) }
}

pub fn mju_l1(vec: &[MjtNum]) -> MjtNum {
    unsafe { mujoco_c::mju_L1(vec.as_ptr(), vec.len() as i32) }
}

pub fn mju_scl(res: &mut [MjtNum], vec: &[MjtNum], scl: MjtNum) {
    assert!(res.len() == vec.len());
    unsafe { mujoco_c::mju_scl(res.as_mut_ptr(), vec.as_ptr(), scl, vec.len() as i32) }
}

pub fn mju_add(res: &mut [MjtNum], vec1: &[MjtNum], vec2: &[MjtNum]) {
    assert!(res.len() == vec1.len() && vec1.len() == vec2.len());
    unsafe { mujoco_c::mju_add(res.as_mut_ptr(), vec1.as_ptr(), vec2.as_ptr(), vec1.len() as i32) }
}

pub fn mju_sub(res: &mut [MjtNum], vec1: &[MjtNum], vec2: &[MjtNum]) {
    assert!(res.len() == vec1.len() && vec1.len() == vec2.len());
    unsafe { mujoco_c::mju_sub(res.as_mut_ptr(), vec1.as_ptr(), vec2.as_ptr(), vec1.len() as i32) }
}

pub fn mju_add_to(res: &mut [MjtNum], vec: &[MjtNum]) {
    assert!(res.len() == vec.len());
    unsafe { mujoco_c::mju_addTo(res.as_mut_ptr(), vec.as_ptr(), vec.len() as i32) }
}

pub fn mju_sub_from(res: &mut [MjtNum], vec: &[MjtNum]) {
    assert!(res.len() == vec.len());
    unsafe { mujoco_c::mju_subFrom(res.as_mut_ptr(), vec.as_ptr(), vec.len() as i32) }
}

pub fn mju_add_to_scl(res: &mut [MjtNum], vec: &[MjtNum], scl: MjtNum) {
    assert!(res.len() == vec.len());
    unsafe { mujoco_c::mju_addToScl(res.as_mut_ptr(), vec.as_ptr(), scl, vec.len() as i32) }
}

pub fn mju_add_scl(res: &mut [MjtNum], vec1: &[MjtNum], vec2: &[MjtNum], scl: MjtNum) {
    assert!(res.len() == vec1.len() && vec1.len() == vec2.len());
    unsafe { mujoco_c::mju_addScl(res.as_mut_ptr(), vec1.as_ptr(), vec2.as_ptr(), scl, vec1.len() as i32) }
}

pub fn mju_normalize(res: &mut [MjtNum]) -> MjtNum {
    unsafe { mujoco_c::mju_normalize(res.as_mut_ptr(), res.len() as i32) }
}

pub fn mju_norm(vec: &[MjtNum]) -> MjtNum {
    unsafe { mujoco_c::mju_norm(vec.as_ptr(), vec.len() as i32) }
}

pub fn mju_dot(vec1: &[MjtNum], vec2: &[MjtNum]) -> MjtNum {
    assert!(vec1.len() == vec2.len());
    unsafe { mujoco_c::mju_dot(vec1.as_ptr(), vec2.as_ptr(), vec1.len() as i32) }
}

pub fn mju_mul_mat_vec(res: &mut [MjtNum], mat: &[MjtNum], vec: &[MjtNum]) {
    let nr = res.len();
    let nc = vec.len();
    assert!(mat.len() == nr * nc);
    unsafe {
        mujoco_c::mju_mulMatVec(
            res.as_mut_ptr(),
            mat.as_ptr(),
            vec.as_ptr(),
            nr as i32,
            nc as i32,
        )
    }
}

pub fn mju_mul_mat_t_vec(res: &mut [MjtNum], mat: &[MjtNum], vec: &[MjtNum]) {
    let nc = res.len();
    let nr = vec.len();
    assert!(mat.len() == nr * nc);
    unsafe {
        mujoco_c::mju_mulMatTVec(
            res.as_mut_ptr(),
            mat.as_ptr(),
            vec.as_ptr(),
            nr as i32,
            nc as i32,
        )
    }
}

pub fn mju_mul_vec_mat_vec(vec1: &[MjtNum], mat: &[MjtNum], vec2: &[MjtNum]) -> MjtNum {
    let n = vec1.len();
    assert!(vec2.len() == n);
    assert!(mat.len() == n * n);
    unsafe { mujoco_c::mju_mulVecMatVec(vec1.as_ptr(), mat.as_ptr(), vec2.as_ptr(), n as i32) }
}

pub fn mju_transpose(res: &mut [MjtNum], mat: &[MjtNum], nr: usize, nc: usize) {
    assert!(res.len() == nr * nc);
    assert!(mat.len() == nr * nc);
    unsafe { mujoco_c::mju_transpose(res.as_mut_ptr(), mat.as_ptr(), nr as i32, nc as i32) }
}

pub fn mju_symmetrize(res: &mut [MjtNum], mat: &[MjtNum], n: usize) {
    assert!(res.len() == n * n);
    assert!(mat.len() == n * n);
    unsafe { mujoco_c::mju_symmetrize(res.as_mut_ptr(), mat.as_ptr(), n as i32) }
}

pub fn mju_eye(mat: &mut [MjtNum], n: usize) {
    assert!(mat.len() == n * n);
    unsafe { mujoco_c::mju_eye(mat.as_mut_ptr(), n as i32) }
}

pub fn mju_mul_mat_mat(res: &mut [MjtNum], mat1: &[MjtNum], mat2: &[MjtNum], r1: usize, c1: usize, c2: usize) {
    assert!(mat1.len() == r1 * c1);
    assert!(mat2.len() == c1 * c2);
    assert!(res.len() == r1 * c2);
    unsafe {
        mujoco_c::mju_mulMatMat(
            res.as_mut_ptr(),
            mat1.as_ptr(),
            mat2.as_ptr(),
            r1 as i32,
            c1 as i32,
            c2 as i32,
        )
    }
}

pub fn mju_mul_mat_mat_t(res: &mut [MjtNum], mat1: &[MjtNum], mat2: &[MjtNum], r1: usize, c1: usize, r2: usize) {
    assert!(mat1.len() == r1 * c1);
    assert!(mat2.len() == r2 * c1);
    assert!(res.len() == r1 * r2);
    unsafe {
        mujoco_c::mju_mulMatMatT(
            res.as_mut_ptr(),
            mat1.as_ptr(),
            mat2.as_ptr(),
            r1 as i32,
            c1 as i32,
            r2 as i32,
        )
    }
}

pub fn mju_mul_mat_t_mat(res: &mut [MjtNum], mat1: &[MjtNum], mat2: &[MjtNum], r1: usize, c1: usize, c2: usize) {
    assert!(mat1.len() == r1 * c1);
    assert!(mat2.len() == r1 * c2);
    assert!(res.len() == c1 * c2);
    unsafe {
        mujoco_c::mju_mulMatTMat(
            res.as_mut_ptr(),
            mat1.as_ptr(),
            mat2.as_ptr(),
            r1 as i32,
            c1 as i32,
            c2 as i32,
        )
    }
}

pub fn mju_sqr_mat_td(res: &mut [MjtNum], mat: &[MjtNum], diag: Option<&[MjtNum]>, nr: usize, nc: usize) {
    assert!(mat.len() == nr * nc);
    assert!(res.len() == nc * nc);
    if let Some(d) = diag {
        assert!(d.len() == nr);
        unsafe {
            mujoco_c::mju_sqrMatTD(
                res.as_mut_ptr(),
                mat.as_ptr(),
                d.as_ptr(),
                nr as i32,
                nc as i32,
            )
        }
    } else {
        unsafe {
            mujoco_c::mju_sqrMatTD(
                res.as_mut_ptr(),
                mat.as_ptr(),
                std::ptr::null(),
                nr as i32,
                nc as i32,
            )
        }
    }
}


/********************************/
/* Auto generated */
/*******************************/
/// Intersect ray with pure geom, return nearest distance or -1 if no intersection.
pub fn mju_ray_geom(pos: &[MjtNum; 3], mat: &[MjtNum; 9], size: &[MjtNum; 3], pnt: &[MjtNum; 3], vec: &[MjtNum; 3], geomtype: std::ffi::c_int) -> MjtNum  {
    unsafe { mujoco_c::mju_rayGeom(pos.as_ptr(), mat.as_ptr(), size.as_ptr(), pnt.as_ptr(), vec.as_ptr(), geomtype) }
}

/// Set res = 0.
pub fn mju_zero_3(res: &mut [MjtNum; 3])  {
    unsafe { mujoco_c::mju_zero3(res.as_mut_ptr()) }
}

/// Set res = vec.
pub fn mju_copy_3(res: &mut [MjtNum; 3], data: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_copy3(res.as_mut_ptr(), data.as_ptr()) }
}

/// Set res = vec*scl.
pub fn mju_scl_3(res: &mut [MjtNum; 3], vec: &[MjtNum; 3], scl: MjtNum)  {
    unsafe { mujoco_c::mju_scl3(res.as_mut_ptr(), vec.as_ptr(), scl) }
}

/// Set res = vec1 + vec2.
pub fn mju_add_3(res: &mut [MjtNum; 3], vec_1: &[MjtNum; 3], vec_2: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_add3(res.as_mut_ptr(), vec_1.as_ptr(), vec_2.as_ptr()) }
}

/// Set res = vec1 - vec2.
pub fn mju_sub_3(res: &mut [MjtNum; 3], vec_1: &[MjtNum; 3], vec_2: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_sub3(res.as_mut_ptr(), vec_1.as_ptr(), vec_2.as_ptr()) }
}

/// Set res = res + vec.
pub fn mju_add_to_3(res: &mut [MjtNum; 3], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_addTo3(res.as_mut_ptr(), vec.as_ptr()) }
}

/// Set res = res - vec.
pub fn mju_sub_from_3(res: &mut [MjtNum; 3], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_subFrom3(res.as_mut_ptr(), vec.as_ptr()) }
}

/// Set res = res + vec*scl.
pub fn mju_add_to_scl_3(res: &mut [MjtNum; 3], vec: &[MjtNum; 3], scl: MjtNum)  {
    unsafe { mujoco_c::mju_addToScl3(res.as_mut_ptr(), vec.as_ptr(), scl) }
}

/// Set res = vec1 + vec2*scl.
pub fn mju_add_scl_3(res: &mut [MjtNum; 3], vec_1: &[MjtNum; 3], vec_2: &[MjtNum; 3], scl: MjtNum)  {
    unsafe { mujoco_c::mju_addScl3(res.as_mut_ptr(), vec_1.as_ptr(), vec_2.as_ptr(), scl) }
}

/// Normalize vector, return length before normalization.
pub fn mju_normalize_3(vec: &mut [MjtNum; 3]) -> MjtNum  {
    unsafe { mujoco_c::mju_normalize3(vec.as_mut_ptr()) }
}

/// Return vector length (without normalizing the vector).
pub fn mju_norm_3(vec: &[MjtNum; 3]) -> MjtNum  {
    unsafe { mujoco_c::mju_norm3(vec.as_ptr()) }
}

/// Return dot-product of vec1 and vec2.
pub fn mju_dot_3(vec_1: &[MjtNum; 3], vec_2: &[MjtNum; 3]) -> MjtNum  {
    unsafe { mujoco_c::mju_dot3(vec_1.as_ptr(), vec_2.as_ptr()) }
}

/// Return Cartesian distance between 3D vectors pos1 and pos2.
pub fn mju_dist_3(pos_1: &[MjtNum; 3], pos_2: &[MjtNum; 3]) -> MjtNum  {
    unsafe { mujoco_c::mju_dist3(pos_1.as_ptr(), pos_2.as_ptr()) }
}

/// Multiply 3-by-3 matrix by vector: res = mat * vec.
pub fn mju_mul_mat_vec_3(res: &mut [MjtNum; 3], mat: &[MjtNum; 9], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_mulMatVec3(res.as_mut_ptr(), mat.as_ptr(), vec.as_ptr()) }
}

/// Multiply transposed 3-by-3 matrix by vector: res = mat' * vec.
pub fn mju_mul_mat_t_vec_3(res: &mut [MjtNum; 3], mat: &[MjtNum; 9], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_mulMatTVec3(res.as_mut_ptr(), mat.as_ptr(), vec.as_ptr()) }
}

/// Compute cross-product: res = cross(a, b).
pub fn mju_cross(res: &mut [MjtNum; 3], a: &[MjtNum; 3], b: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_cross(res.as_mut_ptr(), a.as_ptr(), b.as_ptr()) }
}

/// Set res = 0.
pub fn mju_zero_4(res: &mut [MjtNum; 4])  {
    unsafe { mujoco_c::mju_zero4(res.as_mut_ptr()) }
}

/// Set res = (1,0,0,0).
pub fn mju_unit_4(res: &mut [MjtNum; 4])  {
    unsafe { mujoco_c::mju_unit4(res.as_mut_ptr()) }
}

/// Set res = vec.
pub fn mju_copy_4(res: &mut [MjtNum; 4], data: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_copy4(res.as_mut_ptr(), data.as_ptr()) }
}

/// Normalize vector, return length before normalization.
pub fn mju_normalize_4(vec: &mut [MjtNum; 4]) -> MjtNum  {
    unsafe { mujoco_c::mju_normalize4(vec.as_mut_ptr()) }
}

/// Coordinate transform of 6D motion or force vector in rotation:translation format.
/// rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type.
/// Nullable: rotnew2old
pub fn mju_transform_spatial(res: &mut [MjtNum; 6], vec: &[MjtNum; 6], flg_force: std::ffi::c_int, newpos: &[MjtNum; 3], oldpos: &[MjtNum; 3], rotnew_2old: &[MjtNum; 9])  {
    unsafe { mujoco_c::mju_transformSpatial(res.as_mut_ptr(), vec.as_ptr(), flg_force, newpos.as_ptr(), oldpos.as_ptr(), rotnew_2old.as_ptr()) }
}

/// Rotate vector by quaternion.
pub fn mju_rot_vec_quat(res: &mut [MjtNum; 3], vec: &[MjtNum; 3], quat: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_rotVecQuat(res.as_mut_ptr(), vec.as_ptr(), quat.as_ptr()) }
}

/// Conjugate quaternion, corresponding to opposite rotation.
pub fn mju_neg_quat(res: &mut [MjtNum; 4], quat: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_negQuat(res.as_mut_ptr(), quat.as_ptr()) }
}

/// Multiply quaternions.
pub fn mju_mul_quat(res: &mut [MjtNum; 4], quat_1: &[MjtNum; 4], quat_2: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_mulQuat(res.as_mut_ptr(), quat_1.as_ptr(), quat_2.as_ptr()) }
}

/// Multiply quaternion and axis.
pub fn mju_mul_quat_axis(res: &mut [MjtNum; 4], quat: &[MjtNum; 4], axis: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_mulQuatAxis(res.as_mut_ptr(), quat.as_ptr(), axis.as_ptr()) }
}

/// Convert axisAngle to quaternion.
pub fn mju_axis_angle_2_quat(res: &mut [MjtNum; 4], axis: &[MjtNum; 3], angle: MjtNum)  {
    unsafe { mujoco_c::mju_axisAngle2Quat(res.as_mut_ptr(), axis.as_ptr(), angle) }
}

/// Convert quaternion (corresponding to orientation difference) to 3D velocity.
pub fn mju_quat_2_vel(res: &mut [MjtNum; 3], quat: &[MjtNum; 4], dt: MjtNum)  {
    unsafe { mujoco_c::mju_quat2Vel(res.as_mut_ptr(), quat.as_ptr(), dt) }
}

/// Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
pub fn mju_sub_quat(res: &mut [MjtNum; 3], qa: &[MjtNum; 4], qb: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_subQuat(res.as_mut_ptr(), qa.as_ptr(), qb.as_ptr()) }
}

/// Convert quaternion to 3D rotation matrix.
pub fn mju_quat_2_mat(res: &mut [MjtNum; 9], quat: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_quat2Mat(res.as_mut_ptr(), quat.as_ptr()) }
}

/// Convert 3D rotation matrix to quaternion.
pub fn mju_mat_2_quat(quat: &mut [MjtNum; 4], mat: &[MjtNum; 9])  {
    unsafe { mujoco_c::mju_mat2Quat(quat.as_mut_ptr(), mat.as_ptr()) }
}

/// Compute time-derivative of quaternion, given 3D rotational velocity.
pub fn mju_deriv_quat(res: &mut [MjtNum; 4], quat: &[MjtNum; 4], vel: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_derivQuat(res.as_mut_ptr(), quat.as_ptr(), vel.as_ptr()) }
}

/// Integrate quaternion given 3D angular velocity.
pub fn mju_quat_integrate(quat: &mut [MjtNum; 4], vel: &[MjtNum; 3], scale: MjtNum)  {
    unsafe { mujoco_c::mju_quatIntegrate(quat.as_mut_ptr(), vel.as_ptr(), scale) }
}

/// Construct quaternion performing rotation from z-axis to given vector.
pub fn mju_quat_z2_vec(quat: &mut [MjtNum; 4], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_quatZ2Vec(quat.as_mut_ptr(), vec.as_ptr()) }
}

/// Extract 3D rotation from an arbitrary 3x3 matrix by refining the input quaternion.
/// Returns the number of iterations required to converge
pub fn mju_mat_2_rot(quat: &mut [MjtNum; 4], mat: &[MjtNum; 9]) -> std::ffi::c_int  {
    unsafe { mujoco_c::mju_mat2Rot(quat.as_mut_ptr(), mat.as_ptr()) }
}

/// Multiply two poses.
pub fn mju_mul_pose(posres: &mut [MjtNum; 3], quatres: &mut [MjtNum; 4], pos_1: &[MjtNum; 3], quat_1: &[MjtNum; 4], pos_2: &[MjtNum; 3], quat_2: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_mulPose(posres.as_mut_ptr(), quatres.as_mut_ptr(), pos_1.as_ptr(), quat_1.as_ptr(), pos_2.as_ptr(), quat_2.as_ptr()) }
}

/// Conjugate pose, corresponding to the opposite spatial transformation.
pub fn mju_neg_pose(posres: &mut [MjtNum; 3], quatres: &mut [MjtNum; 4], pos: &[MjtNum; 3], quat: &[MjtNum; 4])  {
    unsafe { mujoco_c::mju_negPose(posres.as_mut_ptr(), quatres.as_mut_ptr(), pos.as_ptr(), quat.as_ptr()) }
}

/// Transform vector by pose.
pub fn mju_trn_vec_pose(res: &mut [MjtNum; 3], pos: &[MjtNum; 3], quat: &[MjtNum; 4], vec: &[MjtNum; 3])  {
    unsafe { mujoco_c::mju_trnVecPose(res.as_mut_ptr(), pos.as_ptr(), quat.as_ptr(), vec.as_ptr()) }
}

/// Address of diagonal element i in band-dense matrix representation.
pub fn mju_band_diag(i: std::ffi::c_int, ntotal: std::ffi::c_int, nband: std::ffi::c_int, ndense: std::ffi::c_int) -> std::ffi::c_int  {
    unsafe { mujoco_c::mju_bandDiag(i, ntotal, nband, ndense) }
}

/// Eigenvalue decomposition of symmetric 3x3 matrix, mat = eigvec * diag(eigval) * eigvec'.
pub fn mju_eig_3(eigval: &mut [MjtNum; 3], eigvec: &mut [MjtNum; 9], quat: &mut [MjtNum; 4], mat: &[MjtNum; 9]) -> std::ffi::c_int  {
    unsafe { mujoco_c::mju_eig3(eigval.as_mut_ptr(), eigvec.as_mut_ptr(), quat.as_mut_ptr(), mat.as_ptr()) }
}

/// Muscle active force, prm = (range\[2\], force, scale, lmin, lmax, vmax, fpmax, fvmax).
pub fn mju_muscle_gain(len: MjtNum, vel: MjtNum, lengthrange: &[MjtNum; 2], acc_0: MjtNum, prm: &[MjtNum; 9]) -> MjtNum  {
    unsafe { mujoco_c::mju_muscleGain(len, vel, lengthrange.as_ptr(), acc_0, prm.as_ptr()) }
}

/// Muscle passive force, prm = (range\[2\], force, scale, lmin, lmax, vmax, fpmax, fvmax).
pub fn mju_muscle_bias(len: MjtNum, lengthrange: &[MjtNum; 2], acc_0: MjtNum, prm: &[MjtNum; 9]) -> MjtNum  {
    unsafe { mujoco_c::mju_muscleBias(len, lengthrange.as_ptr(), acc_0, prm.as_ptr()) }
}

/// Muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width).
pub fn mju_muscle_dynamics(ctrl: MjtNum, act: MjtNum, prm: &[MjtNum; 3]) -> MjtNum  {
    unsafe { mujoco_c::mju_muscleDynamics(ctrl, act, prm.as_ptr()) }
}

/// Integrate spring-damper analytically, return pos(dt).
pub fn mju_spring_damper(pos_0: MjtNum, vel_0: MjtNum, kp: MjtNum, kv: MjtNum, dt: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_springDamper(pos_0, vel_0, kp, kv, dt) }
}

/// Return min(a,b) with single evaluation of a and b.
pub fn mju_min(a: MjtNum, b: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_min(a, b) }
}

/// Return max(a,b) with single evaluation of a and b.
pub fn mju_max(a: MjtNum, b: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_max(a, b) }
}

/// Clip x to the range [min, max].
pub fn mju_clip(x: MjtNum, min: MjtNum, max: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_clip(x, min, max) }
}

/// Return sign of x: +1, -1 or 0.
pub fn mju_sign(x: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_sign(x) }
}

/// Round x to nearest integer.
pub fn mju_round(x: MjtNum) -> std::ffi::c_int  {
    unsafe { mujoco_c::mju_round(x) }
}

/// Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.
pub fn mju_is_bad(x: MjtNum) -> std::ffi::c_int  {
    unsafe { mujoco_c::mju_isBad(x) }
}

/// Generate Halton sequence.
pub fn mju_halton(index: std::ffi::c_int, base: std::ffi::c_int) -> MjtNum  {
    unsafe { mujoco_c::mju_Halton(index, base) }
}

/// Sigmoid function over 0<=x<=1 using quintic polynomial.
pub fn mju_sigmoid(x: MjtNum) -> MjtNum  {
    unsafe { mujoco_c::mju_sigmoid(x) }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mju_zero_3() {
        let mut a = [1.0, 2.0, 3.0];
        mju_zero_3(&mut a);
        assert_eq!(a, [0.0; 3]);
    }

    #[test]
    fn test_mju_add_3() {
        let mut a = [1.0, 2.0, 3.0];
        let mut b = [5.0, 6.0, 7.0];
        let mut c = [0.0; 3];
        mju_add_3(&mut c, &mut a, &mut b);
        assert_eq!(c, [6.0, 8.0, 10.0]);
    }

    #[test]
    fn test_mju_sub_3() {
        let mut a = [1.0, 2.0, 3.0];
        let mut b = [1.0, 4.0, 2.0];
        let mut c = [0.0; 3];
        mju_sub_3(&mut c, &mut a, &mut b);
        assert_eq!(c, [0.0, -2.0, 1.0]);
    }

    #[test]
    pub fn test_mju_clip() {
        assert_eq!(mju_clip(1.5, -1.0, 1.0), 1.0);
    }

    #[test]
    pub fn test_mju_cross() {
        let mut a = [1.0, 2.0, 3.0];
        let mut b = [1.0, 4.0, 2.0];
        let mut c = [0.0; 3];
        mju_cross(&mut c, &mut a, &mut b);
        assert_eq!(c, [-8.0, 1.0, 2.0]);
    }

    #[test]
    fn test_mju_zero() {
        let mut a = [1.0, 2.0, 3.0];
        mju_zero(&mut a);
        assert_eq!(a, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_mju_fill() {
        let mut a = [0.0; 3];
        mju_fill(&mut a, 5.0);
        assert_eq!(a, [5.0, 5.0, 5.0]);
    }

    #[test]
    fn test_mju_copy() {
        let mut a = [1.0, 2.0, 3.0];
        let mut b = [0.0; 3];
        mju_copy(&mut b, &mut a);
        assert_eq!(b, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_mju_sum() {
        let a = [1.0, -2.0, 3.0];
        assert_eq!(mju_sum(&a), 2.0);
    }

    #[test]
    fn test_mju_l1() {
        let a = [1.0, -2.0, 3.0];
        assert_eq!(mju_l1(&a), 6.0);
    }

    #[test]
    fn test_mju_scl() {
        let a = [1.0, 2.0, 3.0];
        let mut b = [0.0; 3];
        mju_scl(&mut b, &a, 2.0);
        assert_eq!(b, [2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_mju_add() {
        let a = [1.0, 2.0, 3.0];
        let b = [5.0, 6.0, 7.0];
        let mut c = [0.0; 3];
        mju_add(&mut c, &a, &b);
        assert_eq!(c, [6.0, 8.0, 10.0]);
    }

    #[test]
    fn test_mju_sub() {
        let a = [1.0, 2.0, 3.0];
        let b = [1.0, 4.0, 2.0];
        let mut c = [0.0; 3];
        mju_sub(&mut c, &a, &b);
        assert_eq!(c, [0.0, -2.0, 1.0]);
    }

    #[test]
    fn test_mju_add_to() {
        let mut a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        mju_add_to(&mut a, &b);
        assert_eq!(a, [5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_mju_sub_from() {
        let mut a = [5.0, 7.0, 9.0];
        let b = [1.0, 2.0, 3.0];
        mju_sub_from(&mut a, &b);
        assert_eq!(a, [4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_mju_add_to_scl() {
        let mut a = [1.0, 2.0, 3.0];
        let b = [1.0, 1.0, 1.0];
        mju_add_to_scl(&mut a, &b, 2.0);
        assert_eq!(a, [3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_mju_add_scl() {
        let a = [1.0, 2.0, 3.0];
        let b = [1.0, 1.0, 1.0];
        let mut c = [0.0; 3];
        mju_add_scl(&mut c, &a, &b, 2.0);
        assert_eq!(c, [3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_mju_normalize() {
        let mut a = [3.0, 4.0];
        let len = mju_normalize(&mut a);
        assert!((len - 5.0).abs() < 1e-10);
        assert!((mju_norm(&a) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mju_norm() {
        let a = [3.0, 4.0];
        let len = mju_norm(&a);
        assert!((len - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_mju_dot() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, -5.0, 6.0];
        assert_eq!(mju_dot(&a, &b), 12.0);
    }

    #[test]
    fn test_mju_mul_mat_vec() {
        // 2x3 matrix * 3-vector
        let mat = [1.0, 2.0, 3.0,
                   4.0, 5.0, 6.0];
        let vec = [1.0, 2.0, 3.0];
        let mut res = [0.0; 2];
        mju_mul_mat_vec(&mut res, &mat, &vec);
        assert_eq!(res, [14.0, 32.0]);
    }

    #[test]
    fn test_mju_mul_mat_t_vec() {
        // (2x3 matrix)^T * 2-vector
        let mat = [1.0, 2.0, 3.0,
                   4.0, 5.0, 6.0]; // shape (2,3)
        let vec = [1.0, 1.0];       // len = 2
        let mut res = [0.0; 3];     // len = 3
        mju_mul_mat_t_vec(&mut res, &mat, &vec);
        assert_eq!(res, [5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_mju_mul_vec_mat_vec() {
        // v1^T * M * v2
        let v1 = [1.0, 2.0];
        let v2 = [3.0, 4.0];
        let mat = [1.0, 2.0,
                   3.0, 4.0]; // 2x2
        let result = mju_mul_vec_mat_vec(&v1, &mat, &v2);
        // v1^T * M = [7, 10], dot with v2 = 7*3 + 10*4 = 61
        assert_eq!(result, 61.0);
    }

    #[test]
    fn test_mju_transpose() {
        let mat = [1.0, 2.0, 3.0,
                   4.0, 5.0, 6.0]; // 2x3
        let mut res = [0.0; 6];     // 3x2
        mju_transpose(&mut res, &mat, 2, 3);
        assert_eq!(res, [1.0, 4.0,
                         2.0, 5.0,
                         3.0, 6.0]);
    }

    #[test]
    fn test_mju_symmetrize() {
        let mat = [1.0, 2.0,
                   3.0, 4.0]; // 2x2
        let mut res = [0.0; 4];
        mju_symmetrize(&mut res, &mat, 2);
        // (M + M^T)/2 = [[1,2.5],[2.5,4]]
        assert_eq!(res, [1.0, 2.5,
                         2.5, 4.0]);
    }

    #[test]
    fn test_mju_eye() {
        let mut mat = [0.0; 9];
        mju_eye(&mut mat, 3);
        assert_eq!(mat, [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_mju_mul_mat_mat() {
        // (2x3) * (3x2) = (2x2)
        let mat1 = [1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0];
        let mat2 = [7.0,  8.0,
                    9.0, 10.0,
                    11.0, 12.0];
        let mut res = [0.0; 4];
        mju_mul_mat_mat(&mut res, &mat1, &mat2, 2, 3, 2);
        assert_eq!(res, [58.0, 64.0,
                         139.0, 154.0]);
    }

    #[test]
    fn test_mju_mul_mat_mat_t() {
        let mat1 = [1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0];
        let mat2 = [7.0,  8.0,  9.0,
                    10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0,
                    16.0, 17.0, 18.0]; // 4x3
        let mut res = [0.0; 8]; // 2x4
        mju_mul_mat_mat_t(&mut res, &mat1, &mat2, 2, 3, 4);
        assert_eq!(res, [50.0, 68.0, 86.0, 104.0,
                        122.0, 167.0, 212.0, 257.0]);
    }

    #[test]
    fn test_mju_mul_mat_t_mat() {
        // (2x3)^T * (2x2) = (3x2)
        let mat1 = [1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0]; // 2x3
        let mat2 = [7.0, 8.0,
                    9.0, 10.0];    // 2x2
        let mut res = [0.0; 6]; // 3x2
        mju_mul_mat_t_mat(&mut res, &mat1, &mat2, 2, 3, 2);
        assert_eq!(res, [43.0, 48.0,
                         59.0, 66.0,
                         75.0, 84.0]);
    }

    #[test]
    fn test_mju_sqr_mat_td_no_diag() {
        // res = mat^T * mat
        let mat = [1.0, 2.0,
                   3.0, 4.0,
                   5.0, 6.0]; // 3x2
        let mut res = [0.0; 4]; // 2x2
        mju_sqr_mat_td(&mut res, &mat, None, 3, 2);
        assert_eq!(res, [35.0, 44.0,
                         44.0, 56.0]);
    }

    #[test]
    fn test_mju_sqr_mat_td_with_diag() {
        // res = mat^T * diag * mat
        let mat = [1.0, 2.0,
                3.0, 4.0,
                5.0, 6.0]; // 3x2
        let diag = [1.0, 2.0, 3.0];
        let mut res = [0.0; 4]; // 2x2
        mju_sqr_mat_td(&mut res, &mat, Some(&diag), 3, 2);
        assert_eq!(res, [94.0, 116.0,
                        116.0, 144.0]);
    }

}
