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
We provide a hook for integrating the MuJoCo logging with the ``log`` crate.
The hook can be installed by calling :docs-rs:`mujoco_rs::logging::<fn>install_logging_hook` once
at the start of the program.

An example (using the `env_logger <https://docs.rs/env_logger/0.11.11/env_logger/>`__ backend):

.. code-block:: rust
   :emphasize-lines: 2

   env_logger::init();  // Any `log` backend works.
   install_logging_hook();

   // log's logging function
   log::warn!("this is an example warning!");

   // MuJoCo's logging function
   log_warning("the log backend receives this, with the target 'mujoco'");


In the above example, the ``log``'s target is set to the Rust-module path when called from the user's code.
The message is displayed in the following format:

   [WARN  <module path>] this is an example warning!

When calling ``log_warning`` from the user's program, or MuJoCo tries to log internally, the message
is displayed in the following format:

   [WARN  mujoco<:: optional sub-target>] the log backend receives this, with the target 'mujoco'

Most MuJoCo logging messages will have no specific sub-topic, thus the format will be something like:

   [WARN  mujoco] ...

The translation from a MuJoCo logging topic (:docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjtLogTopic`)
to the ``log`` target is as follows:

.. list-table::
   :header-rows: 1

   * - Topic
     - ``log`` target
   * - ``mjTOPIC_NONE``
     - ``mujoco``
   * - ``mjTOPIC_TIME_STP``
     - ``mujoco::time_stp``
   * - ``mjTOPIC_TIME_CMP``
     - ``mujoco::time_cmp``
   * - ``mjTOPIC_SLEEP``
     - ``mujoco::sleep``

Custom logging callback
-----------------------

In addition to the default ``log`` integration, the hook also allows users to add safe callback functions
that accept a regular Rust reference to the :docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjLogMessage` as its parameter, containing
the requested message to log.

The user callback function can be registered via :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler`.
Note that this is **not** a wrapper around :docs-rs:`~mujoco_rs::mujoco_c::<fn>mju_setLogHandler` but merely a setter
to a global static inside MuJoCo-rs (not MuJoCo). The actual MuJoCo handler set by ``mju_setLogHandler``
is defined internally inside MuJoCo-rs, and is responsible for both calling the global user-set Rust logging handler
and providing a facade to the ``log`` crate when no user-set Rust logging handler is set.

When setting a custom logging callback inside MuJoCo-rs, there is no need to call
:docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` separately,
because the :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` does it automatically.

All ``log_`` methods calls (and internal MuJoCo calls) will now be redirected to the handler:

.. code-block:: rust
   :emphasize-lines: 5

   fn handler(message: &MjLogMessage) {
       println!("{:?}: {}", message.level(), message.subject());
   }

   set_log_handler(handler);
   log_warning("the handler receives this, and the `log` backend receives nothing");
