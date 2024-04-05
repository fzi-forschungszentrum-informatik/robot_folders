#
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
"""Small module that helps combining compilation database"""

import pathlib
import json


def find_compilation_db_files(root):
    """Finds all `compilation_database.json` files under `root`"""
    return pathlib.Path(root).rglob("compile_commands.json")


def merge_compile_commands(root, target_file):
    """
    Merges all 'compile_commands.json' files under `root` and saves the result in `target_file`
    """
    commands = []
    for filename in find_compilation_db_files(root):
        with open(filename, "r") as content:
            commands.extend(json.load(content))

    if commands:
        with open(target_file, "w") as out_file:
            json.dump(commands, out_file, indent=4, sort_keys=True)
