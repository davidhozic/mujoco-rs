# Subagent Policy

When launching subagents (via the `Agent` tool), ensure they also follow this project's
conventions:

1. **Point subagents at rules; don't paste them.** Name the files to read (e.g. "read
   `.claude/rules/macro-system.md` before editing wrappers") and embed only the few critical
   excerpts. Paste full text only for short rules or when the subagent cannot access the repo.
2. **Reference skills by path** (`.claude/skills/`) instead of copying their content.
3. **Minimum rule set for every subagent** (referenced by path):
   - `disallowed-commands.md` (short; may be pasted inline).
   - `coding-conventions.md` when the subagent reads or writes code.
   - `macro-system.md` when it touches wrapper files or `src/util.rs`.
4. **Long-running subagents re-read the embedded rules midway**; say so in the prompt.
5. **Prefer built-in tools over agents** when comparable in time; reserve agents for multi-step
   reasoning, multi-file synthesis, or parallel workloads.
7. **Write precise prompts**: concrete file paths and line numbers, exact commands, expected output
   format.
8. **Challenge every finding from both sides** (audits, reviews, doc verification): argue for the
   fix with exact evidence (file:line, type definitions), then argue against it (by design,
   conventional wording, unreachable path).
