# MuJoCo-rs
[![docs.rs](https://img.shields.io/docsrs/mujoco-rs/latest)](https://docs.rs/mujoco-rs)
[![Crates.io](https://img.shields.io/crates/v/mujoco-rs.svg)](https://crates.io/crates/mujoco-rs)


MuJoCo bindings and wrappers for the Rust programming language. Includes a Rust-native viewer and also
bindings to a modified C++ one.

MuJoCo is a general purpose physics simulator. Link to the MuJoCo website: https://mujoco.org/.


## Rust-native viewer
Screenshot of the built-in Rust viewer. Showing scene from [MuJoCo's menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/boston_dynamics_spot).
![](docs/img_common/viewer_spot.png)


## MuJoCo version
This library uses FFI bindings to MuJoCo **3.3.5**.
### Dynamic linking
The library can either be provided **dynamically** in the form of a shared library (.so and .dll):
- ``MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/ cargo build``
- or ``export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/`` and then ``cargo build``.

When using the shared library, the **Rust-native MuJoCo viewer** can be used,
but not the original C++ one.


#### Linux
Also on **Linux** note that when the MuJoCo library isn't installed in the standard location,
the path ``/path/mujoco/lib/`` must be added to `LD_LIBRARY_PATH` like so:
- ``LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/mujoco/lib/``.

#### Windows notice
If you're on **Windows**, make sure to compile with path to the **mujoco.lib** file:
- ``export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/`` <---- notice the **/lib/**,
- ``cargo build``.

To actually run the compiled code, the path containing the **mujoco.dll** must be given.
This can be set via environmental variables or, if you're using bash, like so:
- ``export PATH="/path/mujoco/bin/:$PATH"`` <---- notice the **/bin/**

All of the above together:
- ``export MUJOCO_DYNAMIC_LINK_DIR=/path/mujoco/lib/``,
- ``export PATH="/path/mujoco/bin/:$PATH``,
- ``cargo run --example touch_sensor``.

### Static linking
Now regarding **static linking**.
If you do not require the **C++ MuJoCo viewer (the simulate UI)** and have
somehow already obtained statically linkable libraries,
you can just run the following command:
- ``MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/ cargo build``.
- or ``export MUJOCO_STATIC_LINK_DIR=/path/mujoco/lib/`` and then ``cargo build``.

Note that the ``/path/mujoco/lib`` needs to contain all the MuJoCo dependencies.

To build **statically** with the support of the **C++ MuJoCo viewer**,
perform the following:
1. Clone the repository
2. ``git submodule update --init --recursive``
3. ``cd ./mujoco/``
4. ``cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed``
5. ``cmake --build build --parallel --target libsimulate --config=Release``
6. Add the crate normally to your Cargo.toml.


## Features
Optional features can be enabled to add additional features.
These are:
- ``viewer`` (default): enables the Rust-native MuJoCo viewer. This can currently
                display everything and respond to mouse/keyboard. No side-panels (the user menu) currently exists.
- ``cpp-viewer``: enables the Rust wrapper around the C++ MuJoCo viewer. This is only available if you build the MuJoCo yourself using the steps above (yes, you need to use the forked repository).


## Missing libraries
The crate should work out of the box after you provide it with the MuJoCo library. If the build fails and asks
for additional dependencies, install them via your system package manager.
For example, to install glfw3 on Ubuntu/Debian, this can be done like so: ``apt install libglfw3-dev``.

## RAW FFI bindings
If the current state of the wrappers isn't satisfactory enough, raw FFI bindings to the C MuJoCo
library can be used. These are available under the ``mujoco_rs::mujoco_c`` module.
To access the FFI level structs of the wrappers, call the ``ffi()`` and ``ffi_mut()`` methods.

## NOTE
This project is WIP but functional. I accept pull requests about bug fixes
and feature requests. If you have any questions, please open a **discussion**.


## Example
This example shows how to launch the viewer and print the coordinates
of a moving ball to the terminal.
Other examples can be found under the ``examples/`` directory.


```rust
//! Example of using views.
//! The example shows how to obtain a [`MjJointInfo`] struct that can be used
//! to create a (temporary) [`MjJointView`] to corresponding fields in [`MjData`].
use std::time::Duration;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;


const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\"/>
    <body name=\"ball\">
        <geom name=\"green_sphere\" pos=\".2 .2 .2\" size=\".1\" rgba=\"0 1 0 1\" solref=\"0.004 1.0\"/>
        <joint name=\"ball_joint\" type=\"free\"/>
    </body>

    <geom name=\"floor1\" type=\"plane\" size=\"10 10 1\" euler=\"15 4 0\" solref=\"0.004 1.0\"/>
    <geom name=\"floor2\" type=\"plane\" pos=\"15 -20 0\" size=\"10 10 1\" euler=\"-15 -4 0\" solref=\"0.004 1.0\"/>

  </worldbody>
</mujoco>
";

fn main() {
    /* Load the model and create data */
    let model = MjModel::from_xml_string(EXAMPLE_MODEL).expect("could not load the model");
    let mut data = model.make_data();  // or MjData::new(&model);

    /* Launch a passive Rust-native viewer */
    let mut viewer = MjViewer::launch_passive(&model, 0)
        .expect("could not launch the viewer");

    /* Create the joint info */
    let ball_info = data.joint("ball_joint").unwrap();
    while viewer.running() {
        /* Step the simulation and sync the viewer */
        viewer.sync(&mut data);
        data.step();

        /* Obtain the view and access first three variables of `qpos` (x, y, z) */
        let xyz = &ball_info.view(&data).qpos[..3];
        println!("The ball's position is: {xyz:.2?}");

        std::thread::sleep(Duration::from_secs_f64(0.002));
    }
}
```
