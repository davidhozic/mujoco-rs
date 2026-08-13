# Task tracking

Keep a live task list in the harness's own task tool (`TaskCreate` / `TaskUpdate` / `TaskList`),
visible to the user. Not in prose, not in a scratch file.

- Open the list before the work; the first task goes to `in_progress` before the first edit.
- One task, one deliverable. If a task cannot be marked done by pointing at something (a passing
  test, a number, a written file), split it.
- Update the moment a task starts or finishes; exactly one task is `in_progress` at a time.
- Add newly discovered tasks immediately, even if not done this round. Close tasks the user rules
  out.
- Do not close a task the evidence does not close; a blocked task says so.
- Prune finished or obsolete tasks.
