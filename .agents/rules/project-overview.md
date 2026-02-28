---
trigger: always_on
---

# Project Overview

MuJoCo-rs is a high-level Rust wrapper around the [MuJoCo](https://mujoco.org/) C physics simulation library. It provides safe Rust wrappers, a Rust-native 3D viewer, and an offscreen renderer.

## Getting oriented

1. Read `Cargo.toml` to discover the MuJoCo version, available Cargo features, minimum Rust version, and dependencies.
2. Read `README.md` for a high-level project description and main features.
3. Read `src/lib.rs` to see the module structure and which modules are feature-gated.
4. Explore `src/wrappers/` to find the safe Rust wrapper types — infer their purpose from filenames and doc comments.
5. Explore `examples/` to understand usage patterns.

## Documentation

- **MuJoCo-rs guide**: https://mujoco-rs.readthedocs.io
- **MuJoCo overview**: https://mujoco.readthedocs.io/en/stable/overview.html
- **MuJoCo C API types**: https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html
- **MuJoCo C API functions**: https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html

When unsure about a wrapper's behavior or a C struct's fields, consult these docs.

## Build system

The `build.rs` script determines how to link MuJoCo at compile time. Read it together with `Cargo.toml` to understand the linking strategy. The `/build` workflow provides the standard development build command.

## Key files to never edit
- `src/mujoco_c.rs` — auto-generated FFI bindings (see `coding-conventions.md`).
