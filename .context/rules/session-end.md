---
trigger: always_on
---

# Session End Protocol

When the agent has finished its current task (verification round, bug fix, feature implementation, etc.),
it must NOT simply end the session. Instead:

1. Summarize what was done in 2-3 sentences.
2. Use the `ask_questions` tool to prompt the user for continuation instructions.
3. The question MUST use `allowFreeformInput: true` so the user can type custom commands.
4. Provide 3-5 options covering likely next steps (e.g., "Run another round", "Review changes",
   "Run tests", "Apply fixes").
5. Wait for the user's response before proceeding.

Example:
```
ask_questions({
  questions: [{
    header: "Next steps",
    question: "Task X is complete. What would you like to do next?",
    allowFreeformInput: true,
    options: [
      { label: "Run tests" },
      { label: "Review changes" },
      { label: "Continue with next task" }
    ]
  }]
})
```

This ensures the user always has control over session flow and can redirect work as needed.
