#!/usr/bin/env python3
"""Small hacky script to get the correct source_zsh command per click version"""

import click

click_version = click.__version__
click_major_version = click_version.split('.')[0]
if int(click_major_version) >= 8:
    print("zsh_source")
else:
    print("source_zsh")
