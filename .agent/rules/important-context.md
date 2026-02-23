# Important Context

## Where to look first
| Task | Read these first |
|---|---|
| Modifying wrappers | `src/util.rs` → the wrapper file → MuJoCo C API docs |
| Build issues | `Cargo.toml` → `build.rs` → `/build` workflow |
| Type questions | `src/wrappers/mj_primitive.rs` |
| Code generation | `../mujoco-rs-utils` (run with `--help`) |

## Key references
- **MuJoCo C API**: https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html
- **MuJoCo version**: check `Cargo.toml`

## Common pitfalls
- MuJoCo C arrays can be dense or sparse (via address arrays). Read the C API docs to determine which applies.
- Fields documented as `-1: none` in MuJoCo should map to `Option` in the Rust wrappers.
- When in doubt, run `/expand-macros` and inspect the generated code.
