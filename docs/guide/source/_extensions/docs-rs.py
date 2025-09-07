"""
A custom extension that allows linking to docs.rs projects.

Usage:

:docs-rs:`crate_name::module::module::<struct>SomeStruct::<method>some_method`

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
    app.add_role('docs-rs', link_docs_rs)
    app.add_post_transform(StripSpecifiers)
    config = app.config
    return {'version': '0.1'}


def map_specifier(parts: list[str]):
    for part in parts:
        match_ = re.search(r"<(\w+)>", part)
        if match_ is not None:
            type_ = match_.group(1)
            yield part.replace(f"<{type_}>", f"{type_}."), type_
        else:
            yield part, ""

def link_docs_rs(name, rawtext, text, lineno, inliner, options={}, content=[]):
    html_ref = ""
    try:
        root, *parts = text.split("::")
        for i, (mapped, type_) in enumerate(map_specifier(parts)):
            parts[i] = mapped

        if type_ in {"method", "variant"}:  # last iteration result
            parts[-2] += ".html"
            html_ref = parts.pop()
        elif type_:
            parts[-1] += ".html"

    except ValueError:
        root = text
        parts = tuple()

    path = "/".join(parts)
    if html_ref:
        path += f"#{html_ref}"

    release = config["release"]

    url = f"https://docs.rs/{root}/{release}/{root}/{path}"
    node = nodes.reference(rawtext, text, refuri=url, **options)
    return [node], []


class StripSpecifiers(Transform):
    default_priority = 500
    def apply(self):
        for node in self.document.traverse(nodes.Text):
            text = node.astext()
            text = re.sub(r"<(\w+)>", "", text)
            node.parent.replace(node, nodes.Text(text))

