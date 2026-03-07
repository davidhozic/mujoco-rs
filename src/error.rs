//! Error types for MuJoCo-rs operations.
//!
//! - [`MjDataError`] - physics data and Jacobian operations (`MjData`).
//! - [`MjSceneError`] - 3-D scene and visualization operations (`MjvScene`, `MjrContext`).
//! - [`MjEditError`] - model-specification editing operations (`MjSpec`).
use std::fmt;

/// Errors that can occur in [`MjData`](crate::wrappers::MjData) physics data
/// and Jacobian operations.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjDataError {
    /// A provided object or body index was out of the valid range.
    IndexOutOfBounds {
        /// Name of the index parameter, e.g. `"body_id"` or `"geom_id"`.
        kind: &'static str,
        /// The index value that was passed.
        id: i64,
        /// Exclusive upper bound of the valid range.
        upper: i64,
    },
    /// The provided MuJoCo object type is not supported by this operation.
    ///
    /// Contains the raw MuJoCo C object-type code (`mjOBJ_*`) that was not recognized.
    UnsupportedObjectType(i32),
    /// MuJoCo failed to allocate the [`MjData`](crate::wrappers::MjData) structure.
    AllocationFailed,
    /// A buffer passed to the operation is too small for the required data.
    BufferTooSmall {
        /// Descriptive name of the buffer (e.g. `"destination"`, `"rgb"`).
        name: &'static str,
        /// Actual length of the buffer that was provided.
        got: usize,
        /// Minimum length the buffer must have.
        needed: usize,
    },
    /// Two [`MjData`](crate::wrappers::MjData) instances (or an `MjData` and a
    /// renderer) were created from different models.
    SignatureMismatch {
        /// Model signature of the source object.
        source: u64,
        /// Model signature of the destination object.
        destination: u64,
    },
}

impl fmt::Display for MjDataError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::IndexOutOfBounds { kind, id, upper } => {
                write!(f, "{kind} {id} is out of bounds [0, {upper})")
            }
            Self::UnsupportedObjectType(raw) => {
                write!(f, "object type {raw} is not supported by this operation")
            }
            Self::AllocationFailed => {
                write!(f, "MuJoCo allocation failed")
            }
            Self::BufferTooSmall { name, got, needed } => {
                write!(
                    f,
                    "{name} buffer is too small: got {got} elements, \
                     but need at least {needed}"
                )
            }
            Self::SignatureMismatch { source, destination } => {
                write!(
                    f,
                    "model signature mismatch: source {source:#X}, \
                     destination {destination:#X}"
                )
            }
        }
    }
}

impl std::error::Error for MjDataError {}


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
        /// Length of the label in bytes.
        len: usize,
        /// Maximum number of bytes (excluding the NUL terminator) the buffer can hold.
        capacity: usize,
    },
    /// An auxiliary buffer index is out of the valid range `[0, mjNAUX)`.
    InvalidAuxBufferIndex {
        /// The out-of-range index that was provided.
        index: usize,
    },
    /// A viewport has invalid (negative) dimensions.
    InvalidViewport {
        /// The viewport width that was provided.
        width: i32,
        /// The viewport height that was provided.
        height: i32,
    },
    /// A pixel buffer passed to a rendering operation is too small.
    BufferTooSmall {
        /// Descriptive name of the buffer (e.g. `"rgb"`, `"depth"`).
        name: &'static str,
        /// Actual length of the buffer that was provided.
        got: usize,
        /// Minimum length the buffer must have.
        needed: usize,
    },
    /// The figure's line-data buffer for a given plot is full.
    FigureBufferFull {
        /// Index of the plot whose buffer is full.
        plot_index: usize,
        /// Maximum number of data points the buffer can hold.
        capacity: usize,
    },
    /// A point index is out of range for the current data in a figure plot.
    FigureIndexOutOfBounds {
        /// Index of the plot.
        plot_index: usize,
        /// The point index that was provided.
        point_index: usize,
        /// Current number of data points in the plot.
        current_len: usize,
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
                write!(f, "aux buffer index {index} is out of range [0, mjNAUX)")
            }
            Self::InvalidViewport { width, height } => {
                write!(
                    f,
                    "viewport dimensions must be non-negative, got {width}x{height}"
                )
            }
            Self::BufferTooSmall { name, got, needed } => {
                write!(
                    f,
                    "{name} buffer is too small: got {got} elements, \
                     but need at least {needed}"
                )
            }
            Self::FigureBufferFull { plot_index, capacity } => {
                write!(
                    f,
                    "figure plot {plot_index} buffer is full \
                     (capacity = {capacity} data points)"
                )
            }
            Self::FigureIndexOutOfBounds { plot_index, point_index, current_len } => {
                write!(
                    f,
                    "point index {point_index} is out of bounds for plot {plot_index} \
                     (current length = {current_len})"
                )
            }
        }
    }
}

impl std::error::Error for MjSceneError {}


/// Errors that can occur in model-specification editing operations
/// ([`MjSpec`](crate::wrappers::MjSpec) and related types).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjEditError {
    /// MuJoCo failed to allocate the requested model element.
    AllocationFailed,
}

impl fmt::Display for MjEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AllocationFailed => write!(f, "MuJoCo failed to allocate the model element"),
        }
    }
}

impl std::error::Error for MjEditError {}
