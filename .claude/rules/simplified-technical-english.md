# Simplified Technical English

Write all prose in Simplified Technical English (ASD-STE100 style): chat replies, thinking, README
and `.claude/` prose, commit messages, and subagent prompts (a subagent gets this rule by path).
Rust comments and `///` docs are exempt; they follow `coding-conventions.md`.

## Rules

- One idea per sentence: 20 words for a procedure, 25 for a description. One topic per paragraph,
  at most 6 sentences.
- Active voice ("the solver updates each body"). Simple present or simple past only.
- One word per meaning, one meaning per word: a "checkpoint" is never a "snapshot" or a "save".
- Name a thing by what it is, never by a design pattern or a metaphor: "the `log` crate", not
  "the `log` facade". Keep a term for one referent: "backend" is the sink that receives the
  records.
- Simple verbs, not verbs turned into nouns ("the step advances the simulation").
- Keep articles and relative pronouns ("the state that the buffer holds").
- At most three nouns in a cluster ("the limit value of the contact solver iterations").
- Instructions as commands ("Read the sensor once per step"). Condition before action ("If the step
  diverges, reset the data").
- No idiom, metaphor, irony or humour. Domain terms the code uses ("free" body, "static" geom)
  stay.
- ASCII only (`coding-conventions.md`). No em dash and no `--` substitute: use a comma, colon,
  parentheses, or a full stop. Exception: the Sphinx guide under `docs/`, where RST renders `---`.

## What STE does not change

- Identifiers, type names, paths and numbers stay as the code spells them.
- Quotations from external documents are copied as they stand.
- The other rules keep their force (comment limits, doc requirements, changelog rules). STE governs
  how a sentence is written, not whether it is written.
