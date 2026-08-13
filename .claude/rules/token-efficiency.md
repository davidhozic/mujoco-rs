# Token Efficiency

Keep token consumption low. Cost is `context size x number of requests`: each tool call re-sends the
whole context, and every tool result stays in the prefix permanently.

## Reading & searching
1. **Read targeted ranges, not whole files** (`Read` with `offset`/`limit`, grep first). E.g. read
   the relevant macro in `src/util.rs`, not all of it.
2. **Scope every search** to a specific symbol or path; phrase tasks concretely.
3. **Never re-read a file already in context.**
4. **Cap command output** (`tail`/`head`/`grep`); never print a whole log or diff to decide one fact.
5. **Never `Read` a file under `.claude/`**; the rules are already in context.

## Requests
6. **Send independent tool calls in one message.**
7. **Never poll a running job**: a blocking `Bash` call with an explicit `timeout` is the wait.
   Background only when you have other work to interleave.

## Delegation
8. **Delegate verbose work to subagents** (test runs, doc fetches, log scans).
9. **Require concise, structured summaries** (findings + file:line); a subagent returning bulk
   content is scoped too broadly.
10. **Don't spawn a subagent for what one tool call does** (`subagent-policy.md` rule 5).

## Output & session
11. **Keep responses concise**; no recaps of shown code or rejected options.
12. **State a measurement once, as a table**; never raw CSV or sweep logs.
13. **Match `/effort` to task difficulty.**
14. **Preserve the cached prefix**: do not churn rule files mid-session or re-read them on a cadence.
