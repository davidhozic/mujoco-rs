//! Allocator setup for a run under the Miri interpreter.

use std::alloc::{alloc_zeroed, Layout};
use std::sync::Once;

/// Chunk sizes in bytes and the number of chunks of each size, in ascending size order.
const CHUNK_CLASSES: [(usize, usize); 5] = [
    (256,            8192),
    (4 * 1024,        512),
    (64 * 1024,        64),
    (1024 * 1024,       8),
    (16 * 1024 * 1024,  2),
];

/// The alignment `mju_malloc` promises its callers.
const CHUNK_ALIGN: usize = 64;

unsafe extern "C" {
    fn setup_miri_chunk_allocator(bases: *mut *mut u8, sizes: *const usize, count: usize);
}

unsafe extern "Rust" {
    /// Miri intrinsic. Excuses the allocation from the leak check.
    fn miri_static_root(ptr: *const u8);
}

/// Points MuJoCo at a pool of Rust allocations.
///
/// # Panics
/// When a chunk cannot be allocated.
pub fn install_allocator() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let layouts = CHUNK_CLASSES.map(|(size, _)| Layout::from_size_align(size, CHUNK_ALIGN).unwrap());
        let (mut bases, mut sizes) = (Vec::new(), Vec::new());
        for ((size, count), layout) in CHUNK_CLASSES.into_iter().zip(layouts) {
            for _ in 0..count {
                // SAFETY: every class size is non-zero.
                let base = unsafe { alloc_zeroed(layout) };
                assert!(!base.is_null(), "failed to allocate a {size} byte Miri chunk");
                // SAFETY: `base` is the start of a live allocation.
                unsafe { miri_static_root(base) };
                bases.push(base);
                sizes.push(size);
            }
        }
        // SAFETY: both arrays hold `bases.len()` entries, and the callee copies them before it
        // returns, so dropping the vectors afterwards is sound.
        unsafe { setup_miri_chunk_allocator(bases.as_mut_ptr(), sizes.as_ptr(), bases.len()) };
    });
}
