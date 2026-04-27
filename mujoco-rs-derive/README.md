# mujoco-rs-derive

Procedural macros used internally by [mujoco-rs](https://crates.io/crates/mujoco-rs).

This crate is an **implementation detail** of `mujoco-rs` and is not intended
for direct use.  It is published separately only because Cargo requires
proc-macro crates to be their own package.

## Derive macros

### `ThreeWayMerge`

Derives a three-way merge implementation for structs with named fields.
Used internally by `mujoco-rs` to merge viewer model-state changes.
