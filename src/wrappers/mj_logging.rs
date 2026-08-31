//! MuJoCo's unified logging API.
//!
//! [`crate::logging`] holds the bridge that sends a MuJoCo message to the [`log`] crate.
use std::ffi::{CStr, c_char};
use std::ptr;

use crate::{c_str_as_str_method, getter_setter};
use crate::util::printf_safe_cstring;
use crate::mujoco_c::*;


/***********************************************************************************************************************
** MjtLogLevel / MjtLogTopic / MjLogConfig / MjLogMessage
***********************************************************************************************************************/
/// Log message severity.
pub type MjtLogLevel = mjtLogLevel;

/// Log topic identifiers.
pub type MjtLogTopic = mjtLogTopic;

/// Log handler default configuration.
pub type MjLogConfig = mjLogConfig;
impl MjLogConfig {
    getter_setter! {with, get, set, [
        logto_console: bool;    "print to console (default: true).";
        logto_file: bool;       "print to log file (default: true).";
        topics: i32;            "enabled info topic bitmask (default: 0); topic `t` uses bit `1 << (t - 1)`.";
    ]}

    c_str_as_str_method! {with, get, set {
        logfile;    "log file path (default: `MUJOCO_LOG.TXT`).";
    }}
}

/// Structured log message passed to a log handler.
///
/// Build one with [`MjLogMessage::new`] plus the builder methods. The optional `body`, `func`,
/// and `file` fields are stored as raw C string pointers, so they take `&'static CStr`
/// (NUL-terminated and valid for the whole program); pass a `c"..."` literal, or leak a
/// `CString` for a string built at runtime.
pub type MjLogMessage = mjLogMessage;
impl MjLogMessage {
    /// Creates a structured log message with the given `level`, `topic`, and `subject`.
    ///
    /// The optional `body`, `func`, `file`, `line`, and `timestamp` fields start empty/zero and
    /// can be set with the builder methods (e.g. [`with_body`](Self::with_body)).
    ///
    /// # Panics
    /// Panics if `subject` contains invalid ASCII, an interior NUL byte, or is too long for the
    /// 1024-byte buffer.
    pub fn new(level: MjtLogLevel, topic: MjtLogTopic, subject: &str) -> Self {
        let msg = mjLogMessage {
            level: 0,
            topic: 0,
            subject: [0; 1024],
            body: ptr::null(),
            func: ptr::null(),
            file: ptr::null(),
            line: 0,
            timestamp: false,
        };
        msg.with_level(level).with_topic(topic).with_subject(subject)
    }

    getter_setter! {with, get, set, [
        level: MjtLogLevel [force];     "severity level of the message.";
        topic: MjtLogTopic [force];     "topic of the message (`mjTOPIC_NONE` for error/warning/user).";
        line: i32;                      "`__LINE__` or 0.";
        timestamp: bool;                "prepend timestamp to output.";
    ]}

    c_str_as_str_method! {with, get, set {
        subject;    "message subject (one-liner, printf-formatted).";
    }}

    /// Builder method for setting the message body (multi-line detail); `None` clears it.
    pub fn with_body(mut self, body: Option<&'static CStr>) -> Self {
        self.body = Self::cstr_ptr(body);
        self
    }

    /// Sets the message body (multi-line detail); `None` clears it.
    pub fn set_body(&mut self, body: Option<&'static CStr>) {
        self.body = Self::cstr_ptr(body);
    }

    /// Builder method for setting `__func__`; `None` clears it.
    pub fn with_func(mut self, func: Option<&'static CStr>) -> Self {
        self.func = Self::cstr_ptr(func);
        self
    }

    /// Sets `__func__`; `None` clears it.
    pub fn set_func(&mut self, func: Option<&'static CStr>) {
        self.func = Self::cstr_ptr(func);
    }

    /// Builder method for setting `__FILE__`; `None` clears it.
    pub fn with_file(mut self, file: Option<&'static CStr>) -> Self {
        self.file = Self::cstr_ptr(file);
        self
    }

    /// Sets `__FILE__`; `None` clears it.
    pub fn set_file(&mut self, file: Option<&'static CStr>) {
        self.file = Self::cstr_ptr(file);
    }

    /// Maps a nullable caller-owned `&'static CStr` to the raw pointer stored in the message.
    ///
    /// The `'static CStr` bound keeps the stored pointer sound: the data is NUL-terminated and
    /// lives for the whole program.
    fn cstr_ptr(s: Option<&'static CStr>) -> *const c_char {
        s.map_or(ptr::null(), CStr::as_ptr)
    }

    /// Returns the message body (multi-line detail), or `None`.
    ///
    /// # Panics
    /// Panics if the body is not valid UTF-8.
    pub fn body(&self) -> Option<&str> {
        self.opt_cstr(self.body)
    }

    /// Returns `__func__`, or `None` if unavailable.
    ///
    /// # Panics
    /// Panics if the value is not valid UTF-8.
    pub fn func(&self) -> Option<&str> {
        self.opt_cstr(self.func)
    }

    /// Returns `__FILE__`, or `None` if unavailable.
    ///
    /// # Panics
    /// Panics if the value is not valid UTF-8.
    pub fn file(&self) -> Option<&str> {
        self.opt_cstr(self.file)
    }

    /// Borrows a nullable C string field as `&str`, returning `None` for a null pointer.
    ///
    /// # Panics
    /// Panics if the text is not valid UTF-8.
    fn opt_cstr(&self, ptr: *const c_char) -> Option<&str> {
        if ptr.is_null() {
            None
        } else {
            // SAFETY: a non-null pointer comes from a builder method (a `&'static CStr`), or
            // from MuJoCo, which keeps the string alive for the handler call; `self` spans both.
            Some(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap())
        }
    }
}

/***********************************************************************************************************************
** Free logging functions
***********************************************************************************************************************/
/// Get default handler configuration. Wraps [`mju_getLogConfig`].
///
/// The `topics` bitmask starts at the value that the `MUJOCO_LOG_TOPICS` environment variable
/// selects: a comma-separated list of topic names, lowercase and without the `mjTOPIC_` prefix.
pub fn log_config() -> MjLogConfig {
    // SAFETY: mju_getLogConfig returns a plain struct by value; no allocation.
    unsafe { mju_getLogConfig() }
}

/// Set default handler configuration. Wraps [`mju_setLogConfig`].
///
/// # Safety
/// This must not be used in a multi-threaded way. No MuJoCo items can be used
/// while this is called.
pub unsafe fn set_log_config(config: MjLogConfig) {
    // SAFETY: the caller holds every thread that reads the global configuration.
    unsafe { mju_setLogConfig(config) }
}

/// Dispatch a structured log message to the active handler. Wraps [`mju_message`].
///
/// A message with level `mjLOG_ERROR` can end the process; see [`log_error`].
pub fn log_message(msg: &MjLogMessage) {
    // SAFETY: `msg` is a valid reference; mju_message reads it and does not retain the pointer.
    unsafe { mju_message(msg) }
}

/// Log an info message with optional topic filtering. Wraps [`mju_info`].
///
/// The default handler prints the message only when `topic` is `mjTOPIC_NONE`, or when the topic
/// is enabled in the `topics` bitmask of [`log_config`]. It drops every other info message.
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub fn log_info(topic: MjtLogTopic, msg: &str) {
    let c_msg = printf_safe_cstring(msg);
    // SAFETY: `c_msg` holds no unmatched '%' format specifier, so it is well-defined as the sole
    // format argument of mju_info.
    unsafe { mju_info(topic as i32, c_msg.as_ptr()) }
}

/// Main error function; does not return to caller. Wraps [`mju_error`].
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters. Panics if `mju_error` returns: a custom log
/// handler or a legacy `mju_user_error` callback returned, or a log handler called this function
/// and the recursion guard of MuJoCo dropped the message.
pub fn log_error(msg: &str) -> ! {
    let c_msg = printf_safe_cstring(msg);
    // SAFETY: `c_msg` holds no unmatched '%' format specifier.
    unsafe { mju_error(c_msg.as_ptr()) }
    panic!("mju_error returned instead of ending the process")
}

/// Main warning function; returns to caller. Wraps [`mju_warning`].
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub fn log_warning(msg: &str) {
    let c_msg = printf_safe_cstring(msg);
    // SAFETY: `c_msg` holds no unmatched '%' format specifier.
    unsafe { mju_warning(c_msg.as_ptr()) }
}
