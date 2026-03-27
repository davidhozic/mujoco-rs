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

5. **Ask for concurrent work after launching background subagents.** When spawning one or more
   background subagents (mode `"background"`), immediately use `ask_user` to ask what to work on
   while waiting for the subagents to finish. Do not sit idle.

6. **Limit parallel high-cost agents.** Do not launch more than 3 parallel agents using premium
   models (e.g. Claude Opus 4.6) to avoid rate-limit errors. For cheaper models (Haiku, Sonnet),
   higher parallelism is acceptable.

7. **Prefer system tools over agents when sufficient.** If a task can be accomplished with built-in
   tools (grep, glob, view, bash, edit, etc.) in comparable time, use those directly instead of
   spawning a subagent. Reserve agents for tasks that require multi-step reasoning, synthesis across
   many files, or parallel workloads that benefit from independent context windows.

8. **Write high-quality subagent prompts.** Include specific file paths and line numbers (not just
   general descriptions), embed the exact commands to run, and state the expected output format.
   Vague prompts produce vague results; precise prompts produce precise results.
