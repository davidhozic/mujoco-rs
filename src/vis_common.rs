//! Common visualization utilities.
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_model::MjModel;

use std::ops::Deref;
use std::io;


/// Copies geometry data (geoms only) from the `src` to `dst`.
/// # Errors
/// Returns an [`io::Error`] of kind [`io::ErrorKind::StorageFull`] if the destination scene does not have
/// enough space to accommodate the additional geoms from the source scene.
pub(crate) fn sync_geoms<M: Deref<Target = MjModel>>(src: &MjvScene<M>, dst: &mut MjvScene<M>) -> io::Result<()> {
    let ffi_src = src.ffi();
    let ffi_dst = unsafe { dst.ffi_mut() };

    // Use i64 arithmetic to avoid silent i32 wrapping in release builds when
    // both ngeom values are large (though MuJoCo enforces ngeom <= maxgeom,
    // the sum of two scenes could still wrap an i32).
    let new_len_i64 = (ffi_dst.ngeom as i64) + (ffi_src.ngeom as i64);

    if new_len_i64 > ffi_dst.maxgeom as i64 {
        return Err(io::Error::new(io::ErrorKind::StorageFull, "not enough space available in the destination scene"))
    }

    // new_len_i64 <= maxgeom which is an i32, so the cast is safe.
    let new_len = new_len_i64 as i32;

    // Early-exit when there is nothing to copy so that we never
    // pass a potentially-null `geoms` pointer to copy_nonoverlapping, which
    // would be UB even with a count of 0 (malloc(0) may return null).
    if ffi_src.ngeom == 0 {
        return Ok(());
    }

    // SAFETY: ffi_src.ngeom > 0 guarantees that geoms was allocated by
    // mjv_makeScene (non-null). The overflow guard above ensures that
    // ffi_dst has enough room for ngeom additional elements. The two
    // scenes are distinct allocations so the ranges cannot overlap.
    // ffi_dst.ngeom >= 0 is guaranteed by MuJoCo's invariant, so the
    // cast to usize is safe.
    unsafe { std::ptr::copy_nonoverlapping(
        ffi_src.geoms,
        ffi_dst.geoms.add(ffi_dst.ngeom as usize),
        ffi_src.ngeom as usize
    ) };

    ffi_dst.ngeom = new_len;
    Ok(())
}
