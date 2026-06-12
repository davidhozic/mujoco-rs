# Subagent Policy

When launching subagents (via the `Agent` tool), ensure they also follow this project's
conventions:

1. **Point subagents at rules; don't paste them.** Subagents are stateless, but they can read files
   themselves. Instead of pasting full rule/skill text into the prompt (which duplicates thousands
   of tokens per agent), tell the subagent *which* files to read (e.g. "read
   `.claude/rules/macro-system.md` before editing wrappers") and embed only the few specific
   excerpts critical to the task. Paste full text only for short rules or when the subagent cannot
   access the repo.

2. **Reference relevant skills by path.** If a `.claude/skills/` workflow applies, name the skill
   and its path in the prompt rather than copying its full content.

3. **Minimum rule set for every subagent.** Every subagent prompt must reference (by path, to be
   read by the subagent):
   - The disallowed commands list (`disallowed-commands.md`) -- short; may be pasted inline.
   - The coding conventions (`coding-conventions.md`) when the subagent will read or write code.
   - The macro system guide (`macro-system.md`) when the subagent will touch wrapper files or
     `src/util.rs`.

4. **Periodic rule re-reads (subagents).**
   - Subagent prompts for long-running tasks should include an explicit instruction to re-read the
     embedded rules midway through multi-step work.

5. **Prefer system tools over agents when sufficient.** If a task can be accomplished with built-in
   tools (grep, glob, view, bash, edit, etc.) in comparable time, use those directly instead of
   spawning a subagent. Reserve agents for tasks that require multi-step reasoning, synthesis across
   many files, or parallel workloads that benefit from independent context windows.

7. **Write high-quality subagent prompts.** Include specific file paths and line numbers (not just
   general descriptions), embed the exact commands to run, and state the expected output format.
   Vague prompts produce vague results; precise prompts produce precise results.

8. **Challenge every finding from both sides (audit / verification work).** Whenever subagents
   are used for audits, code review, or documentation verification, each finding must be argued
   from two opposing positions:
   - **For the fix**: cite the exact evidence (line numbers, function names, type definitions)
     that makes the doc or code wrong.
   - **Against the fix**: argue why the finding could be ignored, considered acceptable, or
     is a false positive (e.g. the behaviour is by design, the wording is conventional, the
     affected path is unreachable in practice).

