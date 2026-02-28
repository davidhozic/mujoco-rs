---
description: How to run Miri on mujoco-rs with C FFI
---

# Running with Miri

Run the codebase under Miri's experimental FFI native-lib support to detect undefined behavior in Rust code interoperating with MuJoCo C FFI.

// turbo-all

1. **Build MuJoCo with Miri support**:
   Navigate to the MuJoCo source directory (e.g., `mujoco/`) and build with the Miri allocator hooks enabled. We use a configuration similar to the official build environment but with shared libraries enabled for Miri:
   ```bash
   cd mujoco
   cmake -S . -B build \
       -DCMAKE_BUILD_TYPE:STRING=Release \
       -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON \
       -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
       -DMUJOCO_BUILD_TESTS=OFF \
       -DBUILD_SHARED_LIBS:BOOL=ON \
       -G Ninja \
       -DCMAKE_C_COMPILER:STRING=clang \
       -DCMAKE_CXX_COMPILER:STRING=clang++ \
       -DMUJOCO_HARDEN:BOOL=ON \
       -DMUJOCO_MIRI_SUPPORT:BOOL=ON
   cmake --build build --parallel --target mujoco
   ```

2. **Setup environment and Run**:
   Run the codebase under Miri using the built library. Replace `<EXAMPLE_NAME>` as needed:
   ```bash
   # Set paths and Miri flags (using build/lib64/ or build/lib/ depending on your system)
   # Check Cargo.toml for the exact library version (e.g. 3.5.0)
   export MUJOCO_DYNAMIC_LINK_DIR=$(realpath build/lib64/) && \
   export LD_LIBRARY_PATH=$(realpath build/lib64/) && \
   export MIRIFLAGS="-Zmiri-disable-isolation -Zmiri-native-lib=$(realpath build/lib64/libmujoco.so.3.5.0) -Zmiri-permissive-provenance -Zmiri-symbolic-alignment-check" && \
   cd .. && \
   cargo +nightly miri run --example miri_test
   ```

3. **Verify results**:
   Check for any "Undefined Behavior" reports in the output. A clean run ending with "Comprehensive test completed successfully!" indicates success.

> [!NOTE]
> - **Global Allocator**: When `MUJOCO_MIRI_SUPPORT` is enabled, the MuJoCo build uses linker-level wrapping (`--wrap`) to intercept all internal C++ `operator new`/`delete` calls and redirect them to a specialized Miri bump allocator.
> - **Provenance**: `-Zmiri-permissive-provenance` is essential because MuJoCo (a C library) manages its own memory, which Rust then accesses.
> - **Target Limitation**: Miri with native libs can only run on `bin` targets or `examples`. It cannot currently run the standard Rust test harness on `lib` targets due to compiler limitations.