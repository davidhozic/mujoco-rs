//! Helpers that only the generated sanitizer probes use.

use std::hint::black_box;

/// Reads the first and last element of `slice`.
pub(crate) fn touch_slice_ends<T: Clone>(slice: &[T]) {
    let (Some(first), Some(last)) = (slice.first(), slice.last()) else {
        return;
    };
    black_box(first.clone());
    black_box(last.clone());
}

/// Rewrites the first and last element of `slice` with the value it read.
///
/// The counterpart of [`touch_slice_ends`] for the write path. Each write restores the value it
/// read, so no field changes and every C-side invariant holds.
pub(crate) fn touch_slice_ends_mut<T: Clone>(slice: &mut [T]) {
    if slice.is_empty() {
        return;
    }
    let last = slice.len() - 1;
    for index in [0, last] {
        slice[index] = black_box(slice[index].clone());
    }
}
