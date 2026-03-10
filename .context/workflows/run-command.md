---
description: Run terminal commands in VS Code agent mode using background execution by default
---

# Running a Terminal Command (VS Code Only)

Use this workflow only for VS Code agent terminal tooling.

1. Start the command in background mode.
```bash
run_in_terminal(..., isBackground=true, timeout=0)
```

2. Wait for command completion explicitly.
```bash
await_terminal(<terminal_id>, timeout=0)
```

3. For long-running servers, keep background mode and poll output.
```bash
get_terminal_output(<terminal_id>)
```

4. Use foreground mode only for truly interactive commands that require direct user input.

> [!NOTE]
> This workflow is VS Code-specific and is not a generic shell policy.
