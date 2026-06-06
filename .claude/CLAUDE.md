# Claude Code configuration

This project's conventions are shared across all AI assistants and live in `.github/`,
which is the single source of truth. Do not duplicate its content here.

- **Rules** -- read and follow ALL files in `.github/rules/` at all times. Re-read them
  after any context compaction.
- **Skills** -- the workflows in `.github/skills/**/SKILL.md` are exposed as Claude skills
  under `.claude/skills/` (invoke as `/build`, `/test`, `/doc`, etc.).

## Copilot -> Claude Code tool mapping

The rules and skills are authored for GitHub Copilot / VS Code. Apply this mapping wherever
they reference a Copilot-specific tool:

| Copilot wording | Claude Code |
|---|---|
| `task` tool / subagents | `Agent` tool |
| `run_in_terminal isBackground: true`, `mode="async"` (+ `detach: true`) | `Bash` with `run_in_background: true` |
| `await_terminal` / `get_terminal_output` | `BashOutput` |
| `view_file` | `Read` |
| `grep_search` | `Grep` |
