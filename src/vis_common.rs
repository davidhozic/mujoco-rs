//! Common visualization utilities shared by the viewer and off-screen renderer.
//!
//! This module provides helpers for geometry synchronisation between
//! [`MjvScene`](crate::wrappers::mj_visualization::MjvScene) instances,
//! in-place image flipping, and PNG export.
use crate::wrappers::mj_visualization::MjvScene;
use crate::wrappers::mj_model::MjModel;
use crate::error::MjSceneError;

use std::fs::File;
use std::io::{self, BufWriter};
use std::ops::Deref;
use std::path::Path;


/// Copies geometry data from one [`MjvScene`] to another.
///
/// All geoms present in `src` are appended to the existing geoms in `dst`.
/// This is primarily used to overlay a secondary scene (e.g., user-drawn
/// decorations) onto the main simulation scene before rendering.
///
/// # Errors
///
/// Returns [`MjSceneError::SceneFull`] if the combined geom count would
/// exceed the destination scene's `maxgeom` capacity.
pub fn sync_geoms<M: Deref<Target = MjModel>>(src: &MjvScene<M>, dst: &mut MjvScene<M>) -> Result<(), MjSceneError> {
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
///
/// OpenGL framebuffers store pixel data bottom-row-first, while most image
/// formats (PNG, BMP, etc.) expect top-row-first order. This function swaps
/// rows symmetrically around the horizontal centre so that the buffer ends up
/// in top-down layout, ready for encoding or display.
///
/// The function is generic over the element type `T`, so it works equally well
/// with `u8` (RGB/RGBA), `u16` (16-bit depth), `f32` (HDR), or any other
/// [`Sized`] type.
///
/// # Arguments
///
/// * `buffer` - Pixel data laid out as `height` consecutive rows, each
///   containing `row_len` elements of type `T`.
/// * `height` - Number of rows in the image.
/// * `row_len` - Number of `T` elements per row (e.g., `width * channels`
///   for interleaved colour data).
///
/// # Panics
///
/// Panics if `buffer.len() < height * row_len`.
pub fn flip_image_vertically<T>(buffer: &mut [T], height: usize, row_len: usize) {
    for i in 0..(height / 2) {
        let top_idx = i * row_len;
        let bottom_idx = (height - 1 - i) * row_len;
        let (top_split, bottom_split) = buffer.split_at_mut(bottom_idx);
        top_split[top_idx..top_idx + row_len].swap_with_slice(&mut bottom_split[0..row_len]);
    }
}

/// Encodes raw pixel data as a PNG image and writes it to a file.
///
/// This is a thin convenience wrapper around the [`png`] crate's encoder.
/// It creates (or overwrites) the file at `path`, writes the PNG header with
/// the given dimensions, colour type, and bit depth, then writes `data` as
/// a single image.
///
/// # Arguments
///
/// * `path`       - Destination file path.
/// * `data`       - Raw pixel bytes in row-major, top-down order.
///                  The caller is responsible for flipping the buffer first if
///                  it came from an OpenGL readback (see [`flip_image_vertically`]).
/// * `width`      - Image width in pixels.
/// * `height`     - Image height in pixels.
/// * `color_type` - PNG colour model (e.g., `png::ColorType::Rgb`).
/// * `bit_depth`  - Bits per channel (e.g., `png::BitDepth::Eight`).
///
/// # Errors
///
/// Returns [`io::Error`] if the file cannot be created or PNG encoding fails.
pub fn write_png<P: AsRef<Path>>(
    path: P,
    data: &[u8],
    width: u32,
    height: u32,
    color_type: png::ColorType,
    bit_depth: png::BitDepth,
    compression: png::Compression
) -> io::Result<()> {
    let file = File::create(path)?;
    let w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(color_type);
    encoder.set_depth(bit_depth);
    encoder.set_compression(compression);

    let mut writer = encoder
        .write_header()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    writer
        .write_image_data(data)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    Ok(())
}
