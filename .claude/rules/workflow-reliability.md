# Workflow Reliability

Applies whenever you author a script for the `Workflow` tool (multi-agent orchestration). The
overriding rule: **a workflow must never silently drop spawned work.** Every agent that is started
must have its outcome accounted for in the returned result -- as a success, an explicit failure, or
an explicit "unverified". Work that vanishes with no trace is the worst failure mode, because a
clean-looking result hides it.

## Why this rule exists

`parallel()` and `pipeline()` coerce a dead agent (terminal API error, mid-run skip) into `null`,
and `agent()` itself returns `null` on those paths. The common idiom `.filter(Boolean)` then
**deletes** those nulls. If a find -> verify pipeline does this, a finding whose verifier died
disappears from **both** the confirmed and the rejected buckets -- the script returns
`{confirmed: [], rejected: [...]}` and the dropped item is simply gone. This has actually happened
in this repo's audits; do not let it happen again.

## Rules for every workflow script

1. **Never `.filter(Boolean)` without reconciling first.** Before discarding nulls, compare the
   spawned count against the returned non-null count. Any difference is dropped work that MUST be
   surfaced -- never silently removed.

2. **Account for every agent.** The returned object must satisfy a reconciliation identity, e.g.
   `total_raised == confirmed + rejected + unverified`. Compute it and `log()` it at the end. For
   find -> verify pipelines specifically: a finding whose verdict is `null`/failed must be routed to
   a distinct **`unverified`** (or `dropped`) bucket in the result -- never into the void, and never
   defaulted into "rejected" (an un-run verifier is not a rejection).

3. **Retry transient failures with a bounded retry.** Wrap each `agent()` in a small retry helper
   (up to ~2 retries) before accepting a `null`. Terminal API errors and skips are frequently
   transient and one re-invocation usually recovers them. Only after retries are exhausted does the
   item move to the `unverified`/`dropped` bucket. Example helper:

   ```js
   async function agentRetry(prompt, opts, tries = 3) {
     for (let i = 0; i < tries; i++) {
       const r = await agent(prompt, opts).catch(() => null)
       if (r != null) return r
       log(`retry ${i + 1}/${tries} for ${opts.label}`)
     }
     return null  // caller routes this to the unverified/dropped bucket
   }
   ```

4. **Log every drop, cap, or sample (no silent caps).** Whenever the workflow drops, truncates,
   samples, or fails to verify an item, `log()` it with the id/label. A silent `.filter(Boolean)`
   reads as "everything passed" when it did not.

## Main-agent duty after a workflow returns

After a `Workflow` run completes, **reconcile spawned vs reported before trusting the result.** If
the result exposes an `unverified`/`dropped` bucket, or the reconciliation counts do not add up
(cross-check against the run journal under the transcript dir if needed), the dropped work is your
responsibility: **re-run the failed agents** (restart them in the main thread or a follow-up
workflow) and fold their verdicts in before reporting the audit as complete. Never present a clean
result while spawned work is unaccounted for.
