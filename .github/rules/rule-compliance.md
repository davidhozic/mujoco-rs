# Rule Compliance

Rules for the main agent to stay compliant with this project's conventions throughout long sessions.

1. **Periodic rule re-reads.** Re-read all files in `.github/rules/` every three user prompts or
   `ask_user` responses (whichever comes first), or sooner if responses are getting long. This
   prevents gradual drift from conventions as context grows.

2. **Re-read after compaction.** When context has been compacted (indicated by a summary replacing
   earlier conversation turns), re-read all files in `.github/rules/` before starting new work.

3. **Pre-compaction TODO.** When context compaction appears imminent (e.g., the conversation is very
   long), add a TODO reminder to re-read the rules immediately after compaction occurs.
