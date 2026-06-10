# Rule Compliance

Rules for the main agent to stay compliant with this project's conventions throughout long sessions.

1. **Rule re-reads (token-aware).** The rules total ~11k tokens, so re-reading them on a fixed
   cadence is a major, recurring token cost. Read each rule file **once per session, on demand**
   the first time it becomes relevant (e.g. read `macro-system.md` only when touching macros), and
   do **not** re-read a file already in the current context. Re-read a specific file only if you
   notice yourself drifting from its conventions. Do not re-read all rules on a fixed prompt count.

2. **Re-read after compaction.** When context has been compacted (indicated by a summary replacing
   earlier conversation turns), re-read all files in `.github/rules/` before starting new work.

3. **Pre-compaction TODO.** When context compaction appears imminent (e.g., the conversation is very
   long), add a TODO reminder to re-read the rules immediately after compaction occurs.

4. **Minimal-change enforcement.** Do not introduce non-essential edits when existing code already
   compiles and the requested task is complete. Keep changes scoped to required correctness work.
