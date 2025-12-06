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
