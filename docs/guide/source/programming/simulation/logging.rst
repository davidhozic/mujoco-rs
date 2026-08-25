=====================
Logging
=====================

MuJoCo uses its own logging system, more in detail described in
`their documentation (C code guide) <https://mujoco.readthedocs.io/en/stable/programming/simulation.html#errors-warnings-logging>`__.


Pure wrappers
================
We provide safe wrappers around their logging API, available in the :docs-rs:`mujoco_rs::wrappers::mj_logging` module.
The module provides abstractions over the raw logging functions, such as
:docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_info`, :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_message`
and others. All of these wrappers can be configured via
:docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>set_log_config`.


Rust ecosystem integration
===========================
Rust has a de facto standard of doing logging, which is done via the `log <https://docs.rs/log/0.4.34/log/>`__ crate.
We provide a hook for integrating the MuJoCo logging with the ``log`` crate, which can be installed by calling 
:docs-rs:`mujoco_rs::logging::<fn>install_logging_hook` once at the start of the program.


An example (using the `env_logger <https://docs.rs/env_logger/0.11.11/env_logger/>`__ backend):

.. code-block:: rust
   :emphasize-lines: 2

   env_logger::init();  // Any `log` backend works.
   install_logging_hook();

   // log's logging function
   log::warn!(target: "example_program::main", "this is an example warning!");

   // MuJoCo's logging function
   log_warning("the log backend receives this, with the target 'mujoco'");


Custom logging callback
-----------------------

In addition to the default ``log`` integration, the hook also allows users to add safe callback functions
that accept a regular Rust reference to the :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjLogMessage` as its parameter, containing
the requested message to log.

The user callback function can be registered via :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler`.
Note that this is **not** a wrapper around :docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_setLogHandler` but merely a setter
to a global (thread-safe) static. The :docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_setLogHandler` function is actually set
to the same logging hook as with :docs-rs:`mujoco_rs::logging::<fn>install_logging_hook`, except it redirects to the
user callback instead of the ``log`` facade. There is no need to call :docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` because the
:docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` already does it internally.

All ``log_`` methods calls (and internal MuJoCo calls) will now be redirected to the handler:

.. code-block:: rust
   :emphasize-lines: 5

   fn handler(message: &MjLogMessage) {
       println!("{:?}: {}", message.level(), message.subject());
   }

   set_log_handler(handler);
   log_warning("the handler receives this, and the `log` backend receives nothing");
