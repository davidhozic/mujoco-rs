"""
A custom extension that allows version controlled linkage to the GitHub repository examples.

--------------------------

MIT License

Copyright (c) 2025-today David Hožič

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
from docutils.transforms import Transform
from docutils import nodes

import re

config = {}

def setup(app):
    global config
    app.add_role('gh-example', link_docs_rs)
    app.add_post_transform(StripSpecifiers)
    config = app.config
    return {'version': '0.1'}


def link_docs_rs(name, rawtext, text: str, lineno, inliner, options={}, content=[]):
    link_text = text
    # Input is in form of `display text <link>`
    if link_text.endswith(">"):
        text, link = link_text.split("<", 1)
        link_text = link[:-1]  # -1 to remove '>'
        text = text.strip()

    release = config["release"]
    repository = "mujoco-rs"
    author_gh = "davidhozic"

    url = f"https://github.com/{author_gh}/{repository}/blob/release/{release}/examples/{link_text}"
    node = nodes.reference(rawtext, text, refuri=url, **options)
    return [node], []


class StripSpecifiers(Transform):
    default_priority = 500
    def apply(self):
        for node in self.document.traverse(nodes.Text):
            text = node.astext()
            text = re.sub(r"<(\w+)>", "", text)
            node.parent.replace(node, nodes.Text(text))
