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

import click

import pytest

from robot_folders.helpers.option_helpers import SwallowAllOption


def test_swallow_all_option():
    runner = click.testing.CliRunner()

    colcon_args = "--symlink-install --packages-select foobar"

    @click.command()
    @click.option("--colcon_args", cls=SwallowAllOption)
    def test_command(colcon_args):
        click.echo(f"{colcon_args}!")

    result = runner.invoke(test_command, [f"--colcon_args={colcon_args}"])
    assert result.exit_code == 0
    assert result.output == f"('{colcon_args}',)!\n"

    with pytest.raises(ValueError):

        @click.command()
        @click.option("--colcon_args", cls=SwallowAllOption, nargs=2)
        def illegal_test_command():
            click.echo("hello")

        runner.invoke(illegal_test_command)
