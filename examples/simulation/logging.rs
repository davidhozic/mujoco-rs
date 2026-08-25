//! An example showing how to enable and use the logging features of MuJoCo-rs.
//!
//! MuJoCo sends every message to one active handler. The example runs the three handlers that the
//! crate offers, in this order:
//!
//! 1. The default MuJoCo handler, which `MjLogConfig` configures (console and log file).
//! 2. The logging hook, which routes the message to the `log` backend. The topic picks the target:
//!    `mujoco` for `mjTOPIC_NONE`, `mujoco::sleep` for `mjTOPIC_SLEEP`.
//! 3. A custom handler, which receives the structured `MjLogMessage`.
//!
//! A program installs one of them, and `set_log_handler` installs the hook on its own. The example
//! replaces each handler with the next one, to show all three. The crate installs the MuJoCo hook
//! once, so the default handler does not come back.
//!
//! The application keeps logging with the `log` macros. Once the hook is installed, the messages of
//! MuJoCo join them in the same backend. Note that `log_error` ends the process, while
//! `log::error!` returns.

use mujoco_rs::prelude::*;
use log::LevelFilter;

/// A handler that receives the structured message, instead of the `log` backend.
fn custom_handler(message: &MjLogMessage) {
    println!(
        "custom handler: level = {:?}, topic = {:?}, subject = {:?}, body = {:?}",
        message.level(), message.topic(), message.subject(), message.body(),
    );
}

fn main() {
    /* Default handler: MuJoCo writes to the console and to MUJOCO_LOG.TXT */
    // MjLogConfig configures the default handler only; the hook below ignores it.
    set_log_config(log_config().with_logto_file(false));
    log_warning("the default handler prints this to the console");

    /* Logging hook: MuJoCo sends every message to the log backend */
    // The level applies while RUST_LOG is unset; try RUST_LOG=mujoco::sleep=off.
    env_logger::builder().filter_level(LevelFilter::Debug).parse_default_env().init();
    install_logging_hook();
    log::warn!("the application logs as usual");                     // Target "logging".
    log_warning("the model defines no actuator");                    // Target "mujoco".
    log_info(MjtLogTopic::mjTOPIC_SLEEP, "the island fell asleep");   // Target "mujoco::sleep".

    /* Custom handler: replaces the routing above, and needs no `install_logging_hook` call */
    set_log_handler(custom_handler);
    log_message(
        &MjLogMessage::new(MjtLogLevel::mjLOG_WARNING, MjtLogTopic::mjTOPIC_NONE, "a full message")
            .with_func(Some(c"main"))
            .with_body(Some(c"the body holds the multi-line detail")),
    );

}
