#!/usr/bin/env python3
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
"""Small hacky script to get the correct source_zsh command per click version"""

import argparse
import click


def get_click_version():
    click_version = click.__version__
    click_major_version = click_version.split(".")[0]
    return int(click_major_version)


def print_source_command(shell: str):
    click_version = get_click_version()
    if click_version >= 8:
        print(f"{shell}_source")
    else:
        print(f"source_{shell}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s", "--shell", type=str, choices=["bash", "zsh"], default="bash"
    )
    args = parser.parse_args()
    print_source_command(args.shell)


if __name__ == "__main__":
    main()
