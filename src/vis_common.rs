//! Common visualization utilities.
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_model::MjModel;
use crate::error::MjSceneError;

use std::fs::File;
use std::io::{self, BufWriter};
use std::ops::Deref;
use std::path::Path;


/// Copies geometry data (geoms only) from the `src` to `dst`.
/// # Errors
/// Returns [`MjSceneError::SceneFull`] if the destination scene does not have
/// enough space to accommodate the additional geoms from the source scene.
pub(crate) fn sync_geoms<M: Deref<Target = MjModel>>(src: &MjvScene<M>, dst: &mut MjvScene<M>) -> Result<(), MjSceneError> {
    let ffi_src = src.ffi();
    let ffi_dst = unsafe { dst.ffi_mut() };

    // Use i64 arithmetic to avoid silent i32 wrapping in release builds when
    // both ngeom values are large (though MuJoCo enforces ngeom <= maxgeom,
    // the sum of two scenes could still wrap an i32).
    let new_len_i64 = (ffi_dst.ngeom as i64) + (ffi_src.ngeom as i64);

    if new_len_i64 > ffi_dst.maxgeom as i64 {
        return Err(MjSceneError::SceneFull { capacity: ffi_dst.maxgeom })
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

/// Flips an image buffer vertically in-place.
pub(crate) fn flip_image_vertically<T>(buffer: &mut [T], height: usize, row_len: usize) {
    for i in 0..(height / 2) {
        let top_idx = i * row_len;
        let bottom_idx = (height - 1 - i) * row_len;
        let (top_split, bottom_split) = buffer.split_at_mut(bottom_idx);
        top_split[top_idx..top_idx + row_len].swap_with_slice(&mut bottom_split[0..row_len]);
    }
}

/// Writes pixel data to a PNG file.
///
/// # Errors
/// Returns [`io::Error`] if file creation or PNG encoding fails.
pub(crate) fn write_png<P: AsRef<Path>>(
    path: P,
    data: &[u8],
    width: u32,
    height: u32,
    color_type: png::ColorType,
    bit_depth: png::BitDepth,
) -> io::Result<()> {
    let file = File::create(path)?;
    let w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(color_type);
    encoder.set_depth(bit_depth);

    let mut writer = encoder
        .write_header()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    writer
        .write_image_data(data)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    Ok(())
}
