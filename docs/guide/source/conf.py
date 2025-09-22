# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import tomllib


project = 'MuJoCo-rs'
copyright = '2025, David Hožič'
author = 'David Hožič'

# Read the version from the Rust crate
with open("../../../Cargo.toml", "rb") as file:
    crate_meta = tomllib.load(file)


release = crate_meta["package"]["version"].split(".")
release[-1] = 'x'  # bug fixes share the same documentation
release = '.'.join(release)


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


import sys
import os
sys.path.append(os.path.abspath("./_extensions/"))

extensions = [
    "sphinx.ext.extlinks",
    "docs-rs",
    "gh-example"
]

templates_path = ['_templates']
exclude_patterns = []

MUJOCO_VERSION = "3.3.5"


rst_epilog = f"""
.. |MUJOCO_VERSION| replace:: {MUJOCO_VERSION}
.. |MUJOCO_VERSION_BOLD| replace:: **{MUJOCO_VERSION}**
"""


extlinks = {
    "mujoco_rs_docs": (f"https://docs.rs/mujoco-rs/{release}/%s", "%s")
}



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
html_static_path = ['_static']

# -- Options for Latex output -------------------------------------------------
latex_engine = "xelatex"
latex_elements = {
    "preamble": r"""
\usepackage[english,slovene]{babel}
""",
    "babel": ""
}
