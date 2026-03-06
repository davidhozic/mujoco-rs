//! Error types for MuJoCo-rs operations.
//!
//! Three separate enums cover the distinct failure domains of the API:
//!
//! - [`MjDataError`]  – physics data and Jacobian operations (`MjData`).
//! - [`MjSceneError`] – 3-D scene and visualization operations (`MjvScene`, `MjrContext`).
//! - [`MjEditError`]  – model-specification editing operations (`MjSpec`).
//!
//! All three enums are marked `#[non_exhaustive]` so that new variants can be
//! added in future releases without a breaking change.
use std::fmt;


// ── MjDataError ───────────────────────────────────────────────────────────────

/// Errors that can occur in [`MjData`](crate::wrappers::MjData) physics data
/// and Jacobian operations.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjDataError {
    /// A provided object or body index was out of the valid range.
    ///
    /// `kind` names the index parameter (e.g. `"body_id"`, `"geom_id"`),
    /// `id` is the value that was passed, and `upper` is the exclusive upper bound.
    IndexOutOfBounds {
        kind: &'static str,
        id: i64,
        upper: i64,
    },
    /// The provided MuJoCo object type is not supported by this operation.
    ///
    /// The inner value is the raw integer discriminant of the unsupported
    /// [`MjtObj`](crate::wrappers::MjtObj) variant.
    UnsupportedObjectType(i32),
    /// MuJoCo failed to allocate the [`MjData`](crate::wrappers::MjData) structure.
    AllocationFailed,
}

impl fmt::Display for MjDataError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::IndexOutOfBounds { kind, id, upper } => {
                write!(f, "{kind} {id} is out of bounds [0, {upper})")
            }
            Self::UnsupportedObjectType(raw) => {
                write!(
                    f,
                    "object type discriminant {raw} is not supported by this operation"
                )
            }
            Self::AllocationFailed => {
                write!(f, "MuJoCo failed to allocate MjData")
            }
        }
    }
}

impl std::error::Error for MjDataError {}


// ── MjSceneError ──────────────────────────────────────────────────────────────

/// Errors that can occur in 3-D scene and visualization operations
/// ([`MjvScene`](crate::wrappers::MjvScene), [`MjrContext`](crate::wrappers::MjrContext)).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjSceneError {
    /// No more space is available for new geoms in the scene.
    ///
    /// Increase the `max_geom` capacity passed to
    /// [`MjvScene::new`](crate::wrappers::MjvScene::new).
    SceneFull {
        /// The current maximum geom capacity of the scene.
        capacity: i32,
    },
    /// A string label exceeds the fixed-size label buffer of an [`MjvGeom`](crate::wrappers::MjvGeom).
    LabelTooLong {
        /// Length (in bytes) of the string that was rejected.
        len: usize,
        /// Maximum number of bytes (excluding the NUL terminator) the buffer can hold.
        capacity: usize,
    },
    /// An auxiliary buffer index is out of the valid range `[0, mjNAUX)`.
    ///
    /// Returning this error preempts a C-level `mju_error` abort that would
    /// otherwise occur inside MuJoCo's rendering layer. `mjNAUX` is 10.
    InvalidAuxBufferIndex {
        /// The index that was passed.
        index: usize,
    },
}

impl fmt::Display for MjSceneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SceneFull { capacity } => {
                write!(
                    f,
                    "scene geom buffer is full (capacity = {capacity}); \
                     increase max_geom when constructing MjvScene"
                )
            }
            Self::LabelTooLong { len, capacity } => {
                write!(
                    f,
                    "label of {len} bytes exceeds the fixed buffer capacity of {capacity} bytes"
                )
            }
            Self::InvalidAuxBufferIndex { index } => {
                write!(
                    f,
                    "aux buffer index {index} is out of range \
                     [0, mjNAUX = 10)"
                )
            }
        }
    }
}

impl std::error::Error for MjSceneError {}


// ── MjEditError ───────────────────────────────────────────────────────────────

/// Errors that can occur in model-specification editing operations
/// ([`MjSpec`](crate::wrappers::MjSpec) and related types).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjEditError {
    /// MuJoCo failed to allocate the requested model element.
    AllocationFailed,
    /// An index passed to a MuJoCo editing API was out of range.
    ///
    /// Returning this error preempts a C-level `mju_error` abort that would
    /// otherwise occur inside the MuJoCo library.
    IndexOutOfBounds {
        kind: &'static str,
        id: usize,
        upper: usize,
    },
    /// A null parent-element pointer was detected before an `mjs_attach` call.
    ///
    /// Returning this error preempts a C-level `mju_error` abort that would
    /// otherwise occur inside `mjs_attach`.
    NullParentElement,
}

impl fmt::Display for MjEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AllocationFailed => {
                write!(f, "MuJoCo failed to allocate the model element")
            }
            Self::IndexOutOfBounds { kind, id, upper } => {
                write!(f, "{kind} {id} is out of bounds [0, {upper})")
            }
            Self::NullParentElement => {
                write!(
                    f,
                    "parent element pointer is null; cannot attach a child element"
                )
            }
        }
    }
}

impl std::error::Error for MjEditError {}
