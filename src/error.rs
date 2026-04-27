//! Error types for MuJoCo-rs operations.
//!
//! - [`MjDataError`] - physics data, view-signature, and Jacobian operations.
//! - [`MjSceneError`] - 3-D scene and visualization operations (`MjvScene`, `MjrContext`).
//! - [`MjEditError`] - model-specification editing operations (`MjSpec`).
//! - [`MjModelError`] - model loading, saving, and state operations (`MjModel`).
//! - [`MjVfsError`] - virtual file system operations (`MjVfs`).
//! - [`MjPluginError`] - plugin library loading operations.
//! - [`GlInitError`] - OpenGL / window initialization (feature-gated).
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
        id: usize,
        /// Exclusive upper bound of the valid range.
        upper: usize,
    },
    /// The provided MuJoCo object type is not supported by this operation.
    ///
    /// Contains the raw MuJoCo C object-type code (`mjOBJ_*`) that was not recognized.
    UnsupportedObjectType(i32),
    /// MuJoCo failed to allocate the requested structure.
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
    /// A slice or array parameter has the wrong length for the operation.
    LengthMismatch {
        /// Descriptive name of the parameter.
        name: &'static str,
        /// Expected length.
        expected: usize,
        /// Actual length that was provided.
        got: usize,
    },
    /// Two model-signature-bound objects were created from different models.
    ///
    /// This is returned by APIs that require matching model signatures,
    /// including data-copy operations and info-view accessors.
    SignatureMismatch {
        /// Model signature of the source object.
        source: u64,
        /// Model signature of the destination object.
        destination: u64,
    },
    /// The specified actuator or sensor has no associated history buffer.
    NoHistoryBuffer {
        /// `"actuator"` or `"sensor"`.
        kind: &'static str,
        /// The zero-based index of the actuator or sensor.
        id: usize,
    },
    /// The contact buffer is full; no more contacts can be added.
    ContactBufferFull,
    /// A filesystem path argument contains invalid UTF-8.
    InvalidUtf8Path,
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
            Self::LengthMismatch { name, expected, got } => {
                write!(
                    f,
                    "{name} has wrong length: expected {expected}, got {got}"
                )
            }
            Self::NoHistoryBuffer { kind, id } => {
                write!(f, "{kind} {id} has no history buffer")
            }
            Self::ContactBufferFull => {
                write!(f, "contact buffer is full")
            }
            Self::InvalidUtf8Path => {
                write!(f, "path contains invalid UTF-8")
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
    /// A plot index is out of the valid range `[0, mjMAXLINE)`.
    InvalidPlotIndex {
        /// The out-of-range plot index that was provided.
        plot_index: usize,
        /// Maximum number of plots (`mjMAXLINE`).
        max_plots: usize,
    },
    /// A geom label string contains non-ASCII bytes.
    ///
    /// MuJoCo's renderer treats the label buffer as ASCII; multi-byte UTF-8
    /// sequences would be rendered as garbage characters.
    NonAsciiLabel,
    /// An integer value does not correspond to any known camera type variant.
    InvalidCameraType(i32),
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
            Self::InvalidPlotIndex { plot_index, max_plots } => {
                write!(
                    f,
                    "plot index {plot_index} is out of range [0, {max_plots})"
                )
            }
            Self::NonAsciiLabel => {
                write!(f, "label contains non-ASCII characters")
            }
            Self::InvalidCameraType(raw) => {
                write!(f, "unknown camera type {raw}")
            }
        }
    }
}

impl std::error::Error for MjSceneError {}


/// Errors that can occur in model-specification editing operations
/// ([`MjSpec`](crate::wrappers::mj_editing::MjSpec) and related types).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjEditError {
    /// MuJoCo failed to allocate the requested model element.
    AllocationFailed,
    /// A filesystem path argument contains invalid UTF-8.
    InvalidUtf8Path,
    /// MuJoCo failed to parse the XML (or other format) input.
    ParseFailed(String),
    /// MuJoCo failed to compile the spec into a model.
    CompileFailed(String),
    /// MuJoCo failed to save the spec to XML.
    SaveFailed(String),
    /// A referenced element (e.g. parent default class) was not found.
    NotFound,
    /// An element with the same name already exists.
    AlreadyExists,
    /// This operation is not supported for the current element.
    UnsupportedOperation,
    /// MuJoCo returned an error while attempting to delete the element.
    DeleteFailed(String),
    /// The output buffer passed to [`MjSpec::save_xml_string`](crate::wrappers::mj_editing::MjSpec::save_xml_string)
    /// was too small to hold the XML.
    ///
    /// `required_size` follows `snprintf`-style semantics: it is the number of bytes MuJoCo would
    /// write, **not** counting the NUL terminator.  To retry successfully, pass a buffer of at
    /// least `required_size + 1` bytes.
    XmlBufferTooSmall {
        /// Number of bytes MuJoCo would write (excluding the NUL terminator).
        /// Pass a buffer of at least `required_size + 1` bytes to retry.
        required_size: usize,
    },
}

impl fmt::Display for MjEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AllocationFailed => write!(f, "MuJoCo failed to allocate the model element"),
            Self::InvalidUtf8Path => write!(f, "path contains invalid UTF-8"),
            Self::ParseFailed(msg) => write!(f, "parse failed: {msg}"),
            Self::CompileFailed(msg) => write!(f, "compilation failed: {msg}"),
            Self::SaveFailed(msg) => write!(f, "save failed: {msg}"),
            Self::NotFound => write!(f, "referenced element not found"),
            Self::AlreadyExists => write!(f, "element with the same name already exists"),
            Self::UnsupportedOperation => write!(f, "this operation is not supported"),
            Self::DeleteFailed(msg) => write!(f, "delete failed: {msg}"),
            Self::XmlBufferTooSmall { required_size } => write!(
                f,
                "XML output buffer too small; retry with at least {} bytes",
                required_size + 1
            ),
        }
    }
}

impl std::error::Error for MjEditError {}


/// Errors that can occur in [`MjModel`](crate::wrappers::MjModel) operations.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjModelError {
    /// A filesystem path argument contains invalid UTF-8.
    InvalidUtf8Path,
    /// MuJoCo failed to load the model from a file, string, or buffer.
    LoadFailed(String),
    /// MuJoCo failed to save the model XML.
    SaveFailed(String),
    /// MuJoCo failed to allocate the requested structure.
    AllocationFailed,
    /// The state source slice has the wrong length for the given spec.
    StateSliceLengthMismatch {
        /// Expected length.
        expected: usize,
        /// Actual length.
        got: usize,
    },
    /// The destination spec is not a subset of the source spec.
    SpecNotSubset,
    /// A destination buffer is too small for the operation.
    BufferTooSmall {
        /// Minimum number of elements required.
        needed: usize,
        /// Actual number of elements available.
        available: usize,
    },
    /// Two model-signature-bound objects were created from different models.
    SignatureMismatch {
        /// Model signature of the source object.
        source: u64,
        /// Model signature of the destination object.
        destination: u64,
    },
    /// A virtual-file-system operation failed while loading a model from a string.
    VfsError(MjVfsError),
    /// The given index is invalid or out of range.
    InvalidIndex(usize, usize),
}

impl fmt::Display for MjModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidUtf8Path => write!(f, "path contains invalid UTF-8"),
            Self::LoadFailed(msg) => write!(f, "model load failed: {msg}"),
            Self::SaveFailed(msg) => write!(f, "model save failed: {msg}"),
            Self::AllocationFailed => write!(f, "MuJoCo failed to allocate the requested structure"),
            Self::StateSliceLengthMismatch { expected, got } => {
                write!(f, "state slice length mismatch: expected {expected}, got {got}")
            }
            Self::SpecNotSubset => {
                write!(f, "dst_spec must be a subset of src_spec")
            }
            Self::BufferTooSmall { needed, available } => {
                write!(
                    f,
                    "buffer is too small: got {available} elements, \
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
            Self::VfsError(e) => write!(f, "VFS error: {e}"),
            Self::InvalidIndex(index, max_index) => write!(
                f,
                "invalid index: {index} is out of bounds (max allowed index is {max_index})"
            ),
        }
    }
}

impl std::error::Error for MjModelError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::VfsError(e) => Some(e),
            _ => None,
        }
    }
}

impl From<MjVfsError> for MjModelError {
    fn from(e: MjVfsError) -> Self {
        Self::VfsError(e)
    }
}


/// Errors that can occur in virtual file system ([`MjVfs`](crate::wrappers::MjVfs)) operations.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjVfsError {
    /// A file or mount with the same name already exists.
    AlreadyExists,
    /// MuJoCo failed to load the file or register the buffer.
    LoadFailed,
    /// The specified file or directory was not found in the VFS.
    NotFound,
    /// The provided path contains invalid UTF-8.
    InvalidUtf8Path,
    /// The buffer length exceeds `i32::MAX` bytes.
    BufferTooLarge,
    /// An unrecognized MuJoCo return code.
    Unknown(i32),
}

impl fmt::Display for MjVfsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlreadyExists => write!(f, "file already exists in VFS"),
            Self::LoadFailed => write!(f, "failed to load file into VFS"),
            Self::NotFound => write!(f, "file not found in VFS"),
            Self::InvalidUtf8Path => write!(f, "path contains invalid UTF-8"),
            Self::BufferTooLarge => write!(f, "buffer length exceeds i32::MAX bytes"),
            Self::Unknown(code) => write!(f, "unknown VFS error (code {code})"),
        }
    }
}

impl std::error::Error for MjVfsError {}


/// Errors that can occur when loading MuJoCo plugin libraries.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MjPluginError {
    /// The provided path contains invalid UTF-8.
    InvalidUtf8Path,
    /// The path string contains an interior null byte.
    NullBytePath,
}

impl fmt::Display for MjPluginError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidUtf8Path => write!(f, "path contains invalid UTF-8"),
            Self::NullBytePath => write!(f, "path contains an interior null byte"),
        }
    }
}

impl std::error::Error for MjPluginError {}

/// Errors that can occur during OpenGL / window initialization.
///
/// Each variant represents a distinct step in the initialization pipeline.
#[cfg(any(feature = "viewer", feature = "renderer-winit-fallback"))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum GlInitError {
    /// The windowing / display builder failed to initialize.
    DisplayBuild(String),
    /// The display builder succeeded but did not produce a window.
    NoWindow,
    /// Failed to obtain the native window handle.
    WindowHandle(String),
    /// OpenGL context creation failed. Wraps [`glutin::error::Error`].
    ContextCreation(glutin::error::Error),
    /// Window surface attributes could not be constructed.
    SurfaceAttributes(String),
    /// Rendering surface creation failed. Wraps [`glutin::error::Error`].
    SurfaceCreation(glutin::error::Error),
    /// Making the GL context current on the surface failed. Wraps [`glutin::error::Error`].
    MakeCurrent(glutin::error::Error),
}

#[cfg(any(feature = "viewer", feature = "renderer-winit-fallback"))]
impl fmt::Display for GlInitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DisplayBuild(e) => write!(f, "display build failed: {e}"),
            Self::NoWindow => write!(f, "display builder did not create a window"),
            Self::WindowHandle(e) => write!(f, "failed to obtain window handle: {e}"),
            Self::ContextCreation(e) => write!(f, "GL context creation failed: {e}"),
            Self::SurfaceAttributes(e) => write!(f, "failed to build surface attributes: {e}"),
            Self::SurfaceCreation(e) => write!(f, "window surface creation failed: {e}"),
            Self::MakeCurrent(e) => write!(f, "failed to make GL context current: {e}"),
        }
    }
}

#[cfg(any(feature = "viewer", feature = "renderer-winit-fallback"))]
impl std::error::Error for GlInitError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::ContextCreation(e) |
            Self::SurfaceCreation(e) |
            Self::MakeCurrent(e) => Some(e),
            _ => None,
        }
    }
}
