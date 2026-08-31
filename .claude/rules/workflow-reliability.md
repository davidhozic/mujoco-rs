# Workflow Reliability

Applies when authoring a `Workflow` tool script (multi-agent orchestration). Overriding rule: **a
workflow must never silently drop spawned work.** `parallel()`/`pipeline()`/`agent()` coerce a dead
agent into `null`, and `.filter(Boolean)` then deletes it: a finding whose verifier died vanishes
from both the confirmed and the rejected buckets.

## Rules for every workflow script

1. **Never `.filter(Boolean)` without reconciling first.** Compare spawned count against returned
   non-null count; surface any difference.
2. **Account for every agent.** The result must satisfy an identity like `total_raised == confirmed
   + rejected + unverified`; compute and `log()` it. A `null`/failed verdict goes to a distinct
   `unverified` bucket, never into the void and never defaulted into "rejected".
3. **Retry transient failures** (up to ~2 retries) before accepting a `null`:

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

4. **Log every drop, cap, or sample** with the item's id/label.

## Main-agent duty after a workflow returns

Reconcile spawned vs reported before trusting the result. Re-run dropped agents (main thread or a
follow-up workflow) and fold their verdicts in before reporting. Never present a clean result while
spawned work is unaccounted for.
