---
name: launch-terminal
description: Launch background or detached processes (servers, watchers, long-running commands). Use this when asked to start a background process, run a server, or keep a process alive across tool calls.
---

# Launching Background and Detached Processes

Use this skill whenever a process must run concurrently with other work or must
outlive a single bash tool call.

---

## Choosing the right mode

| Need | Tool call | Process lifetime |
|---|---|---|
| Run a command while continuing other work (build, test, etc.) | `mode="async"` | Killed when the session ends |
| Start a server/daemon that must stay alive indefinitely | `mode="async", detach: true` | Persists after session ends; must be stopped with `kill <PID>` |

> **Rule constraint:** `disallowed-commands.md` prohibits *shell-level* backgrounding
> (`nohup … &`, `setsid`, etc.). Always use the **bash tool's `detach` parameter**
> instead — never use `nohup`, `&`, or `disown` in command strings.

---

## Background (attached) — `mode="async"`

Use for any command that takes a while but should not outlive the agent session:
builds, tests, watchers that the user will terminate when done.

```
bash(
  command = "cargo watch -x check",
  mode    = "async",
  shellId = "cargo-watch"
)
```

The process is automatically killed when the session shuts down.
Use `read_bash` / `write_bash` / `stop_bash` with the same `shellId` to
interact with it.

---

## Detached (persistent) — `mode="async", detach: true`

Use **only** for servers or daemons that must remain reachable after the current
tool call chain finishes (e.g. an HTTP dev server, a test fixture server).

```
bash(
  command = "python3 -m http.server 8080 --bind 127.0.0.1 --directory dist/",
  mode    = "async",
  detach  = true,
  shellId = "http-server"
)
```

**Important constraints:**

1. **Binding:** Any server must bind exclusively to `127.0.0.1`.  
   Never use `0.0.0.0` or `::` (see `disallowed-commands.md` § Server / Listener Binding).
2. **Stopping:** Detached processes cannot be stopped with `stop_bash`.  
   Save the PID and stop with `kill <PID>` when the server is no longer needed.
3. **No shell-level detach:** Do not use `nohup`, `&`, or `disown` inside the
   command string. The bash tool's `detach: true` parameter handles detachment correctly.

### Getting the PID

```
bash(
  command = "python3 -m http.server 8080 --bind 127.0.0.1 --directory dist/ & echo $!",
  ...
)
```

Or capture it separately after starting:

```
bash(command = "pgrep -f 'http.server 8080'")
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
# Attached background (dies with session)
bash(command="…", mode="async", shellId="my-proc")

# Detached server (persists until killed)
bash(command="my-server --bind 127.0.0.1 …", mode="async", detach=true, shellId="my-server")

# Stop a detached process
bash(command="kill <PID>")
```
