# Subagent Policy

When launching subagents (via the `task` tool or similar), ensure they also follow this project's
conventions:

1. **Include relevant rules in every subagent prompt.** Subagents are stateless and have no access
   to prior context. Before delegating work, read the applicable rules from `.github/rules/` and
   embed the relevant portions (or a faithful summary) in the subagent's prompt so it can follow
   them.

2. **Include relevant skills.** If the subagent's task can benefit from a workflow defined in
   `.github/skills/`, include the skill's content in the prompt so the subagent can use it.

3. **Minimum rule set for every subagent.** At a minimum, every subagent prompt must include:
   - The disallowed commands list (`disallowed-commands.md`).
   - The coding conventions (`coding-conventions.md`) when the subagent will read or write code.
   - The macro system guide (`macro-system.md`) when the subagent will touch wrapper files or
     `src/util.rs`.

4. **Periodic rule re-reads (subagents).**
   - Subagent prompts for long-running tasks should include an explicit instruction to re-read the
     embedded rules midway through multi-step work.
