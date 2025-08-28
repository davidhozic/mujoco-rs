# mujoco-rs-w
MuJoCo bindings and wrappers for the Rust programming language. Includes the viewer.

## Compilation
This will improve in the future, currently the steps include compilation of the MuJoCo itself as a static library. The version is currently frozen at 3.3.2

Steps:
1. Clone the repository
2. ``git submodule update --init --recursive``
3. ``cd ./mujoco/``
4. ``cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed``
5. ``cmake --build build --parallel --target libsimulate --config=Release``
6. Add the crate normally to your Cargo.toml.

## NOTE
I'm currently using this for a specific project, thus a lot of things are missing. In the future, more things will be added.
