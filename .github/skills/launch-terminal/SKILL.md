---
name: launch-terminal
description: Launch background or detached processes (servers, watchers, long-running commands). Use this when asked to start a background process, run a server, or keep a process alive across tool calls.
---

# Launching Background and Detached Processes

Use this skill whenever a process must run concurrently with other work or must
outlive a single `Bash` tool call.

---

## Running a long-lived process

Use a regular `Bash` call. For servers or daemons that must remain reachable after
the tool call returns (e.g. an HTTP dev server, a test fixture server), capture the
PID so you can stop it later.

```
Bash(command = "python3 -m http.server 8080 --bind 127.0.0.1 --directory dist/ & echo $!")
```

**Important constraints:**

1. **Binding:** Any server must bind exclusively to `127.0.0.1`.  
   Never use `0.0.0.0` or `::` (see `disallowed-commands.md` section Server / Listener Binding).
2. **Stopping:** Save the PID printed by `echo $!` and stop the process with `kill <PID>`
   when the server is no longer needed.

### Finding the PID later

```
Bash(command = "pgrep -f 'http.server 8080'")
```

Or use `lsof` to find the process by port:

```
Bash(command = "lsof -ti :8080")
```

---

## When to ask the user instead

Starting a server that the *user* will manage (e.g. a dev server for interactive
browser testing) is better handed off to the user rather than started by the agent.
Present the command and ask the user to run it in their own terminal:

```
Please run the following in a terminal:

    python3 -m http.server 8080 --bind 127.0.0.1 --directory dist/

Then open http://127.0.0.1:8080/ in your browser.
```

This avoids leaving an orphaned process when the session ends.

---

## Quick reference

```bash
# Start a server and capture its PID
Bash(command="my-server --bind 127.0.0.1 ... & echo $!")

# Stop a process by PID
Bash(command="kill <PID>")
```
