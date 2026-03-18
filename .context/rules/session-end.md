---
trigger: always_on
---

# Session End Protocol

When the agent has finished its current task (verification round, bug fix, feature implementation, etc.),
it must NOT simply end the session. Instead:

1. Summarize what was done in 2-3 sentences.
2. Use the `AskUserQuestion` / `ask_questions` tool to prompt the user for continuation instructions.
3. Provide 3-5 options covering likely next steps (e.g., "Run another round", "Review changes",
   "Run tests", "Apply fixes"). Users can always select "Other" to type a custom response.
4. Wait for the user's response before proceeding.

This ensures the user always has control over session flow and can redirect work as needed.
