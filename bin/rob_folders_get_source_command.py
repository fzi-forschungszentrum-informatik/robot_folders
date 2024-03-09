#!/usr/bin/env python3
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
    parser.add_argument("-s", "--shell", type=str, choices=["bash", "zsh"], default="bash")
    args = parser.parse_args()
    print_source_command(args.shell)


if __name__ == "__main__":
    main()
