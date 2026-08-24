//! The bridge between MuJoCo's logging and the [`log`] facade.
//!
//! [`install_logging_hook`] makes MuJoCo send every message to the [`log`] backend, and
//! [`set_log_handler`] registers a handler that receives the message instead.
//! 
//! [`install_logging_hook`] needs to be called regardless for [`set_log_handler`] to work as [`set_log_handler`]
//! does not directly register the logger to MuJoCo.
//! 
//! The wrappers of MuJoCo's own logging API (the message and the configuration types, and the functions that emit
//! a message) live in [`crate::wrappers::mj_logging`].
//!
//! # Example
//! ```
//! use mujoco_rs::prelude::*;
//!
//! fn handler(message: &MjLogMessage) {
//!     println!("{:?}: {}", message.level(), message.subject());
//! }
//!
//! /* Initialization */
//! env_logger::init();  // Any `log` backend works.
//! install_logging_hook();
//!
//! /* Use through the `log` crate */
//! log::warn!(target: "example_program::main", "this is an example warning!");
//!
//! /* Use through MuJoCo wrappers */
//! log_warning("the log backend receives this, with the target 'mujoco'");
//!
//! set_log_handler(handler);
//! log_warning("the handler receives this instead");
//! ```
use std::sync::Mutex;
use std::process;

use log::{Level, Metadata, Record};

use crate::wrappers::mj_logging::{MjLogMessage, MjtLogLevel, MjtLogTopic};
use crate::mujoco_c::mju_setLogHandler;
use crate::util::LockUnpoison;


/// The log handler that [`set_log_handler`] registers.
///
/// The MuJoCo handler is [`logging_hook`], which builds the [`MjLogMessage`] reference and then
/// calls this handler.
static USER_LOG_HANDLER: Mutex<Option<fn(&MjLogMessage)>> = Mutex::new(None);

/// Sets a user-defined log handler that receives every MuJoCo message.
///
/// The handler replaces the built-in [`log`] routing of [`install_logging_hook`].
/// 
/// The [`install_logging_hook`] must be called regardless (once at the start of the program)
/// as the handler is also responsible for converting raw pointes of [`MjLogMessage`] into
/// regular references.
///
/// # Note
/// The handler receives every message, because [`MjLogConfig`](crate::wrappers::mj_logging::MjLogConfig) configures the default MuJoCo
/// handler only.
///
/// <div class="warning">
/// A message with level `mjLOG_ERROR` ends the process with exit code 1 after the handler returns,
/// because MuJoCo must not continue after an error. A panic inside the handler aborts the process,
/// because the handler runs across an FFI boundary.
/// </div>
/// 
pub fn set_log_handler(handler: fn(&MjLogMessage)) {
    USER_LOG_HANDLER.lock_unpoison().replace(handler);
}

/// Installs the internal MuJoCo log handler. Wraps [`mju_setLogHandler`].
///
/// The hook sends every MuJoCo message to the handler of [`set_log_handler`], or, when there is
/// none, to the [`log`] facade. It maps the message like this:
///
/// | MuJoCo field | [`log`] record |
/// |---|---|
/// | `level` | [`Level`] (`mjLOG_DEBUG` -> `Debug`, `mjLOG_WARNING` -> `Warn`, ...) |
/// | `topic` | target: `mujoco` for `mjTOPIC_NONE`, else `mujoco::<topic>` (e.g. `mujoco::sleep`) |
/// | `func`, `subject`, `body` | message text `func: subject\nbody` (empty parts are dropped) |
/// | `file`, `line` | record file and line (only when `line` is not 0) |
///
/// A [`log`] backend matches a target by prefix, so the directives
/// `RUST_LOG=mujoco=warn,mujoco::sleep=debug` enable the sleep topic alone.
///
/// [`MjLogConfig`](crate::wrappers::mj_logging::MjLogConfig) configures the default MuJoCo handler only, so it has no effect on the hook.
pub fn install_logging_hook() {
    // SAFETY: `logging_hook` is a valid handler for the whole program. The user is assumed to not
    // install another handler through the C FFI at the same time.
    unsafe { mju_setLogHandler(Some(logging_hook)) };
}

/// The MuJoCo log handler that [`install_logging_hook`] registers.
unsafe extern "C" fn logging_hook(raw_message: *const MjLogMessage) {
    // SAFETY: MuJoCo never calls the handler with a null pointer, and the message stays valid for
    // the whole call.
    let message = unsafe { &*raw_message };

    // Copy the handler out of the mutex, so that the handler itself can lock the mutex again.
    let user_handler = *USER_LOG_HANDLER.lock_unpoison();
    match user_handler {
        Some(handler) => handler(message),
        None => log_to_facade(message),
    }

    // MuJoCo expects the handler of an error to never return; the C code continues in a broken
    // state if it does.
    if message.level() == MjtLogLevel::mjLOG_ERROR {
        log::logger().flush();
        process::exit(1);
    }
}

/// Sends a MuJoCo message to the [`log`] facade, in the mapping that [`install_logging_hook`]
/// documents.
///
/// # Panics
/// Panics if `subject`, `func`, `file` or `body` contains invalid UTF-8.
fn log_to_facade(message: &MjLogMessage) {
    let level = match message.level() {
        MjtLogLevel::mjLOG_DEBUG   => Level::Debug,
        MjtLogLevel::mjLOG_INFO    => Level::Info,
        MjtLogLevel::mjLOG_WARNING => Level::Warn,
        MjtLogLevel::mjLOG_ERROR   => Level::Error,
    };

    // Both guards are what the log macros do: the cheap global level first, the backend second.
    if level > log::max_level() {
        return;
    }
    let metadata = Metadata::builder().level(level).target(topic_target(message.topic())).build();
    let logger = log::logger();
    if !logger.enabled(&metadata) {
        return;
    }

    // MuJoCo reports an unknown location as line 0, and prints the file only with a line.
    let line = u32::try_from(message.line()).ok().filter(|&line| line != 0);
    let function = message.func().unwrap_or_default();
    let body = message.body().unwrap_or_default();
    let function_separator = if function.is_empty() { "" } else { ": " };
    let body_separator = if body.is_empty() { "" } else { "\n" };
    let subject = message.subject();

    logger.log(
        &Record::builder()
            .metadata(metadata)
            .file(line.and(message.file()))
            .line(line)
            .args(format_args!("{function}{function_separator}{subject}{body_separator}{body}"))
            .build(),
    );
}

/// Returns the [`log`] target of a message topic.
fn topic_target(topic: MjtLogTopic) -> &'static str {
    match topic {
        MjtLogTopic::mjTOPIC_NONE     => "mujoco",
        MjtLogTopic::mjTOPIC_TIME_STP => "mujoco::time_stp",
        MjtLogTopic::mjTOPIC_TIME_CMP => "mujoco::time_cmp",
        MjtLogTopic::mjTOPIC_SLEEP    => "mujoco::sleep",
    }
}
