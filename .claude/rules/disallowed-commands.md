# Disallowed Commands

Never execute the commands below. If a request requires one, refuse or propose an alternative.

## Git Operations
- `git commit`: describe changes so the developer commits manually. When a commit **is** explicitly
  authorized, the message must never carry a `Co-Authored-By` or any other AI-attribution trailer.
- `git push` / `git pull` / `git fetch` / `git rebase` / `git merge`.
- `git reset --hard` / `git clean`.
- `git checkout -b` / `git switch -c`: work in the current branch only.
- `git tag`, `git stash pop`.

## Network & Remote Access
- `curl`, `wget`, `scp`, `rsync`, `ssh`, `ftp`, or any tool reaching external hosts.
- Package managers with remote access (`npm install`, `pip install`, `uv pip install`, `uv add`):
  tell the user what to run instead.

## Server / Listener Binding
- Any HTTP server, TCP listener, or WebSocket server must bind to `127.0.0.1` only, never
  `0.0.0.0` or `::`. Correct: `python -m http.server 8080 --bind 127.0.0.1`.

## Privilege Escalation & System Modifications
- `sudo`, `su`; `chmod`, `chown`, `passwd`, `groupadd`, `useradd`.
- `apt-get`, `yum`, `brew`, `pacman`; `mkfs`, `fdisk`, `parted`, `mount`, `umount`.

## File & Data Destruction
- `rm -rf` or any irreversible deletion of large directories or critical files.
- `dd`, `shred`, or other low-level wipe utilities; destructive `find / -exec ... \;`.

## Scratch Files & Temporary Output
`/tmp` is a tmpfs on this host (RAM-backed); confirm with `findmnt -no FSTYPE /tmp` on other machines.

- Small scratch (scripts, cropped images, log excerpts) goes in `/tmp`. Bulk output (saved models,
  `CARGO_TARGET_DIR`, worktrees, render batches) goes to `~/.cache/<project>/` or `target/`.
- A small file written many times is bulk; judge the total left behind.
- Keep results (measurements, patches, probe dumps) apart from regenerable artifacts:
  `~/.cache/mujoco-rs-results/<agent>-<task>/`. Never leave the only copy of a result in a build
  directory.
- Never delete a directory holding an unread result. Delete only what you created; report what you
  removed and what you kept.

## Build & Execution Risks
- No compiling or running untrusted code or binaries.
- No shell-level background daemons (`nohup ... &`, `setsid`, `disown`, `systemctl start`,
  `service`, `docker run`); use the `Bash` tool with `run_in_background: true`.

## Miscellaneous
- No command that sends data or credentials to external systems.
- No key generation or signing (`openssl genrsa`) unless explicitly needed and safe.
- No `echo $SECRET` or reading secret-bearing environment variables.

This list is not exhaustive; when in doubt, describe the action to the user instead of running it.
