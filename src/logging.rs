//! The bridge between MuJoCo's logging and the [`log`] facade.
//!
//! [`install_logging_hook`] makes MuJoCo send every message to the [`log`] backend.
//! [`set_log_handler`] installs the same MuJoCo hook, but sends every message to a handler of the
//! program instead. Call one of the two: [`set_log_handler`] when the program needs the structured
//! [`MjLogMessage`], [`install_logging_hook`] otherwise.
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
//! /* A program that needs the structured message calls this instead of the hook above */
//! set_log_handler(handler);
//! log_warning("the handler receives this, and the `log` backend receives nothing");
//! ```
use std::sync::atomic::{AtomicPtr, Ordering};
use std::{mem, process, ptr};
use std::sync::Once;

use log::{Level, Metadata, Record};

use crate::wrappers::mj_logging::{MjLogMessage, MjtLogLevel, MjtLogTopic};
use crate::mujoco_c::mju_setLogHandler;


/// A logging handler function type.
type LoggingHandler = fn(&MjLogMessage);

/// The log handler that [`set_log_handler`] registers, cast to a data pointer, or null when
/// [`install_logging_hook`] routes to the [`log`] facade instead.
///
/// The MuJoCo handler is [`logging_hook`], which builds the [`MjLogMessage`] reference and then
/// calls this handler. A [`LoggingHandler`] is a plain function pointer, so it fits in a pointer
/// and needs no allocation.
static USER_LOG_HANDLER: AtomicPtr<()> = AtomicPtr::new(ptr::null_mut());

/// Guards the single [`mju_setLogHandler`] call, so that repeated installs do not race on the
/// non-atomic global of MuJoCo.
static LOGGING_HOOK_INSTALLED: Once = Once::new();

/// Sets a user-defined log handler that receives every MuJoCo message. Wraps
/// [`mju_setLogHandler`].
///
/// The function installs the same MuJoCo hook as [`install_logging_hook`], but `handler` replaces
/// the built-in [`log`] routing of that hook. A later [`install_logging_hook`] call clears
/// `handler`.
///
/// <div class="warning">
/// A message with level `mjLOG_ERROR` ends the process with exit code 1 after the handler returns,
/// because MuJoCo must not continue after an error. A panic inside the handler aborts the process,
/// because the handler runs across an FFI boundary.
/// </div>
pub fn set_log_handler(handler: LoggingHandler) {
    // Release, and Acquire in the hook: no thread observes the hook without the handler behind it.
    USER_LOG_HANDLER.store(handler as *mut (), Ordering::Release);
    set_mujoco_handler();
}

/// Installs the internal MuJoCo log handler. Wraps [`mju_setLogHandler`].
///
/// The hook sends every MuJoCo message to the [`log`] facade.\
/// Also clears the handler set by [`set_log_handler`].
/// A program that needs the structured
/// [`MjLogMessage`] calls [`set_log_handler`] instead, which installs the same hook. The hook maps
/// the message like this:
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
/// [`MjLogConfig`](crate::wrappers::mj_logging::MjLogConfig) configures the default MuJoCo handler
/// only, so it has no effect on the hook.
pub fn install_logging_hook() {
    USER_LOG_HANDLER.store(ptr::null_mut(), Ordering::Release);
    set_mujoco_handler();
}

/// Registers [`logging_hook`] as the MuJoCo log handler, at most once per program.
///
/// The caller stores [`USER_LOG_HANDLER`] first, so the hook always finds the routing it must use.
fn set_mujoco_handler() {
    // SAFETY: `logging_hook` is a valid handler for the whole program. The user is assumed to not
    // install another handler through the C FFI at the same time.
    // The returned previous handler is dropped: a safe API cannot hand out a callable
    // mjfLogHandler.
    LOGGING_HOOK_INSTALLED.call_once(|| unsafe { mju_setLogHandler(Some(logging_hook)); });
}

/// The MuJoCo log handler that [`install_logging_hook`] registers.
///
/// # Safety
/// `raw_message` must be non-null and must point to an `mjLogMessage` that stays valid for the
/// whole call.
unsafe extern "C" fn logging_hook(raw_message: *const MjLogMessage) {
    // SAFETY: MuJoCo never calls the handler with a null pointer, and the message stays valid for
    // the whole call.
    let message = unsafe { &*raw_message };

    // Read the handler once, so that a concurrent replacement cannot split the two branches.
    let user_handler = USER_LOG_HANDLER.load(Ordering::Acquire);
    if user_handler.is_null() {
        log_to_facade(message);
    } else {
        // SAFETY: a non-null `USER_LOG_HANDLER` holds a `LoggingHandler` that `set_log_handler`
        // cast to a data pointer, so the transmute reproduces that function pointer. The value is
        // not dereferenced: it is the address of the function itself.
        let handler: LoggingHandler = unsafe { mem::transmute(user_handler) };
        handler(message);
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
fn log_to_facade(message: &MjLogMessage) {
    let level = match message.level() {
        MjtLogLevel::mjLOG_DEBUG   => Level::Debug,
        MjtLogLevel::mjLOG_INFO    => Level::Info,
        MjtLogLevel::mjLOG_WARNING => Level::Warn,
        MjtLogLevel::mjLOG_ERROR   => Level::Error,
    };

    // The three guards of the log macros: the compile-time cap, the cheap global level, then the
    // backend.
    if level > log::STATIC_MAX_LEVEL || level > log::max_level() {
        return;
    }
    let metadata = Metadata::builder().level(level).target(topic_target(message.topic())).build();
    let logger = log::logger();
    if !logger.enabled(&metadata) {
        return;
    }

    // MuJoCo reports an unknown location as line 0, and prints the file only with a line.
    let line = u32::try_from(message.line()).ok().filter(|&line| line != 0);
    // The hook runs across an FFI boundary, so it must not panic on a text that MuJoCo
    // truncated in the middle of a UTF-8 sequence.
    let function = message.func().unwrap_or_default();
    let body = message.body().unwrap_or_default();
    let function_separator = if function.is_empty() { "" } else { ": " };
    let body_separator = if body.is_empty() { "" } else { "\n" };
    let subject = message.subject();
    let file = message.file();

    logger.log(
        &Record::builder()
            .metadata(metadata)
            .file(line.and(file))
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
