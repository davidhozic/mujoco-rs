.. _visualization:

========================
Visualization
========================
It's useful to visualize what the simulation is doing.
MuJoCo-rs provides three main ways of visualization (requires enabling the corresponding Cargo features):

- :ref:`mj_rust_viewer` for onscreen visualization (window) --- ``viewer`` feature (or ``viewer-ui`` to enable with UI)
- :ref:`mj_renderer` for offscreen rendering (array or a file) --- ``renderer`` feature
- :ref:`mj_cpp_viewer` for onscreen visualization using MuJoCo's official C++ viewer --- ``cpp-viewer`` feature


.. toctree::
    :caption: Table of contents

    viewer
    renderer
    drawing
