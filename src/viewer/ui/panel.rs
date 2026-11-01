//! A submodule of the viewer's UI to separate panel section definitions from all the rest of UI.
pub(super) mod visualization;
pub(super) mod simulation;
pub(super) mod rendering;
pub(super) mod physics;
pub(super) mod ui;

/* Model Synchronization and Status */
const MODEL_SYNC_MESSAGE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
