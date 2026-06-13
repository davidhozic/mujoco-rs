# Token Efficiency

Keep token consumption low. These practices apply to the main agent and every subagent.

## Reading & searching
1. **Read targeted ranges, not whole files.** Use `Read` with `offset`/`limit`, or `Bash` with grep to locate
   the relevant lines first, then read only that section. Avoid dumping large files, full logs, or
   whole directories. (`src/util.rs` is large -- read the relevant macro, not all of it.)
2. **Scope every search.** Search for specific symbols/paths instead of broad whole-repo scans.
   Vague tasks trigger expensive exploration; phrase work concretely ("fix `jac()` bounds check in
   `src/wrappers/...`").
3. **Never re-read a file already in context.** The harness tracks file state; re-reading after your
   own edit, or content you already loaded, is wasted tokens.
4. **Cap command output.** Pipe long-running/verbose commands (test runs, `cargo build`, greps over
   generated code) through `tail`/`head`/`grep`, or delegate them to a subagent so only the summary
   returns to the main thread.

## Delegation
5. **Delegate verbose work to subagents.** Test/build runs, doc fetches, and log processing produce
   bulky output -- run them in a subagent (or background `Bash`) so the noise stays out of the main
   context and only a concise summary returns.
6. **Require concise, structured summaries from subagents** (findings + file:line, not raw file
   content). If a subagent must return large content, the task is scoped too broadly -- split it.
7. **Don't spawn a subagent for what a direct tool call does faster** (a single grep/read/edit).
   Reserve agents for multi-file synthesis or parallel workloads.

## Output & session
8. **Keep responses concise.** Don't recap code already shown or narrate options you won't pursue.
9. **Match `/effort` to task difficulty.** Use low effort for mechanical edits; reserve high effort
   for genuinely complex reasoning (thinking bills as output tokens).
10. **Preserve the cached prefix.** Rules and CLAUDE.md form a byte-stable cached prefix. Don't
    churn rule files mid-session or re-read them on a fixed cadence -- both add cost without benefit.
