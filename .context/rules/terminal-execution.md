---
trigger: always_on
---

# Terminal Execution Policy

This rule applies to VS Code agent terminal tooling only.

When invoking terminal tools in agent mode in VS Code:

1. Use background execution by default.
   - For `run_in_terminal`, set `isBackground: true`.
   - Do this even for short commands, unless interactive foreground input is strictly required.

2. Wait for completion explicitly.
   - After a background run, call `await_terminal` with `timeout: 0`.
   - For long-lived servers, call `get_terminal_output` and keep the process running only when needed.

3. Avoid foreground terminal execution for non-interactive commands.
   - Do not use `isBackground: false` for build/test/run commands.
   - Use foreground only for commands that require direct interactive input from the user.

4. Prefer deterministic command wrappers.
   - Use shell one-liners or scripts that emit clear start/end markers when helpful for debugging.
