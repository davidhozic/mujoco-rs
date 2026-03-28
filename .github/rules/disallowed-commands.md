# Disallowed Commands

This document defines a set of commands that should never be executed
within this development workspace. The purpose is to avoid unwanted side effects,
security risks, or destructive operations. Refer to these rules and refuse or
propose alternatives if a request would require running any prohibited command.

## Git Operations

- `git commit` (and any form of direct commits) - Direct commits should not be made
  automatically. Changes should be described so a developer can perform
  the commit manually.
- `git push` / `git pull` / `git fetch` / `git rebase` / `git merge` - Networked
  repository operations are disallowed. The agent cannot communicate with remote
  servers or change branch state on its own.
- `git reset --hard` / `git clean` - Avoid destructive resets or removals that
  could wipe working files.
- `git checkout -b`, `git switch -c` - Creating or switching branches
  is disallowed. Work in the current branch only.
- `git tag`, `git stash pop` - Do not create tags or restore stashed changes automatically.

## Network & Remote Access

- `curl`, `wget`, `scp`, `rsync`, `ssh`, `ftp`, or any tool for downloading,
  uploading, or connecting to external hosts.
- Package managers with remote access (`npm install`, `pip install`, etc.) when
  they would fetch from external repositories. (Dependency management should be
  described to the user instead.) If you need to install a package, ask the user
  about what to do. Ask by prompting the user via `ask_user` tool (multiple-choice window).
  

## Privilege Escalation & System Modifications

- `sudo`, `su`, or any command that requests elevated privileges.
- `chmod`, `chown`, `passwd`, `groupadd`, `useradd` - altering permissions or
  users on the host system.
- `apt-get`, `yum`, `brew`, `pacman` - installing or removing system packages.
- `mkfs`, `fdisk`, `parted`, `mount`, `umount` - modifying disks or filesystems.

## File & Data Destruction

- `rm -rf` or any command that irreversibly deletes large directories or
  critical files.
- `dd`, `shred`, `mkfs`, or other low-level data wipe utilities.
- `find / -exec ... \;` with destructive actions that could traverse the whole
  filesystem.

## Build & Execution Risks

- Compiling or running arbitrary untrusted code or binaries from unknown
  sources. (Describe how the user can do testing locally instead.)
- Starting background services or daemons that might persist beyond the agent
  session, such as `systemctl start`, `service`, or launching `docker run`.

## Miscellaneous

- Any command that sends data or credentials to external systems.
- Tools for cryptographic key generation or signing unless explicitly needed and
  safe (e.g. `openssl genrsa`).
- `echo $SECRET` or reading environment variables containing secrets.

> **Note:** This list is not exhaustive. When in doubt, err on the side of
> caution: present the user with instructions (`ask_user` tool) rather than executing potentially
> harmful commands.

When in doubt, ask for clarification if a requested action might violate
these rules or suggest alternative, non-destructive approaches.
