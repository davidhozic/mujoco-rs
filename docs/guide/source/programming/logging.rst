.. _logging:

=====================
Logging
=====================

MuJoCo uses its own logging system, more in detail described in
`their documentation (C code guide)
<https://mujoco.readthedocs.io/en/3.12.0/programming/simulation.html#errors-warnings-logging>`__.


Pure wrappers
================
We provide safe wrappers around their logging API, available in the :docs-rs:`mujoco_rs::wrappers::mj_logging` module.
The module provides abstractions over the raw logging functions, such as
:docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_info`, :docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_message`
and others. The default MuJoCo handler is configured via
:docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>set_log_config`, which is an ``unsafe`` function:
MuJoCo keeps the configuration in one non-atomic global that every logging call reads, so the
caller must hold every other thread that uses MuJoCo.

:docs-rs:`~mujoco_rs::wrappers::mj_logging::<fn>log_error` exits the program to
adhere to the MuJoCo's internal error assumptions.

Rust ecosystem integration
===========================
Rust has a de facto standard of doing logging, which is done via the `log <https://docs.rs/log/0.4.34/log/>`__ crate.
We provide a hook for integrating the MuJoCo logging with the ``log`` crate.
The hook can be installed by calling :docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` once
at the start of the program.

.. attention::

    :docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` and
    :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` are ``unsafe``. MuJoCo keeps the log
    handler and the handler configuration in two globals that it writes and reads without
    synchronization, so a thread that emits a message during the installation races with it.
    Call either function at the start of the program, before any thread steps a simulation,
    loads a model, or emits a message.

A message with level :docs-rs:`~~mujoco_rs::wrappers::mj_logging::<type>MjtLogLevel::<variant>mjLOG_ERROR`
still ends the process to adhere to MuJoCo's error assumptions, thus avoiding possible undefined behaviors.

An example (using the `env_logger <https://docs.rs/env_logger/0.11.11/env_logger/>`__ backend):

.. code-block:: rust
    :emphasize-lines: 5-6

    use mujoco_rs::prelude::*;

    fn main() {
        env_logger::init();  // Any `log` backend works.
        // SAFETY: no other thread uses MuJoCo yet.
        unsafe { install_logging_hook() };

        // log's logging function
        log::warn!("this is an example warning!");

        // MuJoCo's logging function
        log_warning("the log backend receives this, with the target 'mujoco'");
    }


In the above example, the ``log``'s target is set to the Rust-module path when called from the user's code.
The message is displayed in a format similar to the following:

   [WARN  <module path>] this is an example warning!

When calling ``log_warning`` from the user's program, or when MuJoCo tries to log internally, the message
is displayed in a format similar to the following:

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

The MuJoCo levels ``mjLOG_DEBUG``, ``mjLOG_INFO``, ``mjLOG_WARNING`` and ``mjLOG_ERROR`` map onto
``log``'s ``Debug``, ``Info``, ``Warn`` and ``Error``.

See :gh-example:`Logging example<simulation/logging.rs>`.

Custom logging callback
-----------------------

In addition to the default ``log`` integration, the hook also accepts a single safe callback function
that takes a regular Rust reference to the
:docs-rs:`~mujoco_rs::wrappers::mj_logging::<type>MjLogMessage`, containing the requested message to
log.

The user callback function can be registered via :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler`.

When setting a custom logging callback inside MuJoCo-rs, there is no need to call
:docs-rs:`~mujoco_rs::logging::<fn>install_logging_hook` separately,
because the :docs-rs:`~mujoco_rs::logging::<fn>set_log_handler` does it automatically.

All ``log_`` function calls (and internal MuJoCo calls) will now be redirected to the handler:

.. code-block:: rust
    :emphasize-lines: 8-9

    use mujoco_rs::prelude::*;

    fn handler(message: &MjLogMessage) {
        println!("{:?}: {}", message.level(), message.subject());
    }

    fn main() {
        // SAFETY: no other thread uses MuJoCo yet.
        unsafe { set_log_handler(handler) };
        log_warning("the handler receives this, and the `log` backend receives nothing");
    }

.. attention::

    Setting a custom handler using only ``set_log_handler`` only redirects **MuJoCo** messages
    to the user handler.

    Any message emitted using ``log::`` invocations bypass the handler. This also applies
    to the internal invocations of MuJoCo-rs as well. For a custom callback of that, `a custom
    logger <https://docs.rs/log/0.4.34/log/#implementing-a-logger>`__ can be defined. Such logger
    can also process the messages from MuJoCo itself, provided ``set_log_handler`` **is not used**.
