//! Wrappers around the asset-cache mechanism.
use crate::mujoco_c::{mj_clearCache, mj_getCache, mj_getCacheCapacity, mj_getCacheSize, mj_setCacheCapacity, mjCache};
use crate::util::force_cast;

use std::marker::PhantomData;
use std::fmt;
use std::ptr;


/// The asset cache for caching assets (e.g., textures and meshes)
/// during recompilations of a specification/model.
/// 
/// This is a ZST around [`mjCache`].
/// 
/// Because caching uses a global lock inside MuJoCo, and thus this acting like a type
/// with internal mutability, all of the methods here use non-mutable references.
pub struct MjCache(PhantomData<mjCache>);

// Ensure `MjCache` is in fact a zero-sized type.
const _ : () = assert!(size_of::<MjCache>() == 0);

// SAFETY: the C cache guards every operation with its own internal mutex.
unsafe impl Sync for MjCache {}

impl MjCache {
    /// Returns the global compiler cache. Wraps [`mj_getCache`].
    /// The cache object itself is static (freed upon program exit). However, its cached items
    /// can be cleaned using [`MjCache::clear`].
    pub fn current() -> &'static Self {
        // SAFETY: `mj_getCache` returns a pointer to a C static, sitting behind a
        // Mutex for any modification/read.
        unsafe { force_cast(&*mj_getCache()) }
    }

    /// Returns the current size of the asset cache in bytes. Wraps [`mj_getCacheSize`].
    pub fn size(&self) -> usize {
        unsafe { mj_getCacheSize(self.ffi()) }
    }

    /// Returns `true` if the asset cache holds no assets.
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Returns the current capacity of the asset cache in bytes. Wraps [`mj_getCacheCapacity`].
    pub fn capacity(&self) -> usize {
        unsafe { mj_getCacheCapacity(self.ffi()) }
    }

    /// Set the capacity of the asset cache in bytes (0 to disable).
    /// Returns the new capacity.
    pub fn set_capacity(&self, capacity: usize) -> usize {
        unsafe { mj_setCacheCapacity(self.ffi(), capacity) }
    }

    /// Clear the asset cache.
    pub fn clear(&self) {
        unsafe { mj_clearCache(self.ffi()) };
    }

    // The C side synchronizes internally, so a shared handle may hand out the mutable pointer.
    fn ffi(&self) -> *mut mjCache {
        ptr::from_ref(self).cast_mut().cast()
    }
}

impl fmt::Debug for MjCache {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MjCache")
            .field("size", &self.size())
            .field("capacity", &self.capacity())
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
pub mod tests {
    use crate::wrappers::{MjModel, MjVfs};
    use super::*;

    /// Tests whether cache initializes.
    #[test]
    fn test_cache_initialization() {
        /// The default cache size set inside the MuJoCo's code (1 << 20 gives scaler for MiB to bytes).
        const MJ_DEFAULT_CACHE_CAPACITY: usize = 500 * (1 << 20);
        let cache = MjCache::current();
        // Preallocated by default.
        assert_eq!(cache.capacity(), MJ_DEFAULT_CACHE_CAPACITY);
    }

    /// Tests whether the cache actually caches elements.
    #[test]
    fn test_cache_caching() {
        // Only a mesh loaded from a file enters the cache (`mjCMesh::Compile` in user_mesh.cc).
        const MODEL: &str = "<mujoco>\
            <asset><mesh name='tetrahedron' file='tetrahedron.obj'/></asset>\
            <worldbody><geom type='mesh' mesh='tetrahedron'/></worldbody>\
        </mujoco>";
        const TETRAHEDRON_OBJ: &str = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n\
            f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

        let mut vfs = MjVfs::new();
        vfs.add_from_buffer("model.xml", MODEL.as_bytes()).unwrap();
        vfs.add_from_buffer("tetrahedron.obj", TETRAHEDRON_OBJ.as_bytes()).unwrap();

        let cache = MjCache::current();
        cache.clear();
        assert!(cache.is_empty());
        MjModel::from_xml_vfs("model.xml", &vfs).unwrap();
        assert!(cache.size() > 0);
        cache.clear();
        assert!(cache.is_empty());
    }
}
