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
