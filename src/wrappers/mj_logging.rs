//! MuJoCo's unified logging API.
use std::ffi::{CStr, CString};
use std::ptr;

use crate::{c_str_as_str_method, getter_setter};
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
        topics: i32;            "enabled info topic bitmask (default: 0).";
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
    /// The `'static CStr` bound is what keeps the stored pointer sound: the data is
    /// NUL-terminated (C reads `body`/`func`/`file` as C strings) and lives for the whole
    /// program, so the pointer can never dangle and C never reads out of bounds.
    fn cstr_ptr(s: Option<&'static CStr>) -> *const std::ffi::c_char {
        s.map_or(ptr::null(), CStr::as_ptr)
    }

    /// Returns the message body (multi-line detail), or `None`.
    ///
    /// # Panics
    /// Panics if the body is not valid UTF-8.
    pub fn body(&self) -> Option<&str> {
        Self::opt_cstr(self.body)
    }

    /// Returns `__func__`, or `None` if unavailable.
    ///
    /// # Panics
    /// Panics if the value is not valid UTF-8.
    pub fn func(&self) -> Option<&str> {
        Self::opt_cstr(self.func)
    }

    /// Returns `__FILE__`, or `None` if unavailable.
    ///
    /// # Panics
    /// Panics if the value is not valid UTF-8.
    pub fn file(&self) -> Option<&str> {
        Self::opt_cstr(self.file)
    }

    /// Borrows a nullable C string field as `&str`, returning `None` for a null pointer.
    fn opt_cstr<'a>(ptr: *const std::ffi::c_char) -> Option<&'a str> {
        if ptr.is_null() {
            None
        } else {
            // SAFETY: the pointer is non-null and, per MuJoCo's contract, points to a valid
            // NUL-terminated string that outlives the borrowing message.
            Some(unsafe { CStr::from_ptr(ptr) }.to_str().unwrap())
        }
    }
}

/***********************************************************************************************************************
** Free logging functions
***********************************************************************************************************************/
/// Get default handler configuration. Wraps [`mju_getLogConfig`].
pub fn log_config() -> MjLogConfig {
    // SAFETY: mju_getLogConfig returns a plain struct by value; no allocation.
    unsafe { mju_getLogConfig() }
}

/// Set default handler configuration. Wraps [`mju_setLogConfig`].
pub fn set_log_config(config: MjLogConfig) {
    // SAFETY: mju_setLogConfig copies the struct by value; no aliasing.
    unsafe { mju_setLogConfig(config) }
}

/// Dispatch a structured log message to the active handler. Wraps [`mju_message`].
pub fn log_message(msg: &MjLogMessage) {
    // SAFETY: `msg` is a valid reference; mju_message reads it and does not retain the pointer.
    unsafe { mju_message(msg as *const MjLogMessage) }
}

/// Log an info message with optional topic filtering. Wraps [`mju_info`].
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub fn log_info(topic: MjtLogTopic, msg: &str) {
    // Escape '%' so the message is safe to pass as a printf format string.
    let escaped = msg.replace('%', "%%");
    let c_msg = CString::new(escaped).unwrap();
    // SAFETY: the escaped string contains no unmatched '%' format specifiers,
    // so passing it as the sole format argument to mju_info is well-defined.
    unsafe { mju_info(topic as _, c_msg.as_ptr()) }
}

/// Main error function; does not return to caller. Wraps [`mju_error`].
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub fn log_error(msg: &str) -> ! {
    let escaped = msg.replace('%', "%%");
    let c_msg = CString::new(escaped).unwrap();
    // SAFETY: the escaped string contains no unmatched '%' format specifiers.
    unsafe { mju_error(c_msg.as_ptr()) }
    unreachable!()  // safety net, in case mju_error returns.
}

/// Main warning function; returns to caller. Wraps [`mju_warning`].
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub fn log_warning(msg: &str) {
    let escaped = msg.replace('%', "%%");
    let c_msg = CString::new(escaped).unwrap();
    // SAFETY: the escaped string contains no unmatched '%' format specifiers.
    unsafe { mju_warning(c_msg.as_ptr()) }
}
