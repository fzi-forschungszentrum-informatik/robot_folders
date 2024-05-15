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
""" Robot Folders

    This is the robot_folders main file. If you'd like to change or add
    functionality, modify or add an according file in the 'commands'
    subfolder. Every file in there is automatically added as command to
    robot_folders.
    Make sure that each command file consists at least of the following:
      import click
      @click.command()
      def cli():
        \"\"\"Description of the command\"\"\"

    For further info either have a look at the other commands or at the
    click documentation under http://click.pocoo.org
"""

import click
import os
import traceback

from click.exceptions import UsageError
from robot_folders.helpers.exceptions import ModuleException

plugin_folder = os.path.join(os.path.dirname(__file__), "commands")


class RobotFolders(click.MultiCommand):

    def list_commands(self, ctx):
        rv = []
        for filename in os.listdir(plugin_folder):
            if filename.endswith(".py") and filename != "__init__.py":
                rv.append(filename[:-3])
        rv.sort()
        return rv

    def get_command(self, ctx, name):
        ns = {}
        fn = os.path.join(plugin_folder, name + ".py")
        if os.path.isfile(fn):
            with open(fn) as f:
                code = compile(f.read(), fn, "exec")
                eval(code, ns, ns)
                return ns["cli"]
        else:
            return None

    def invoke(self, ctx):
        try:
            super(RobotFolders, self).invoke(ctx)
        except ModuleException as err:
            click.echo(
                "Execution of module '{}' failed. Error message:\n{}".format(
                    err.module_name, err
                )
            )
            os._exit(err.return_code)
        except UsageError as err:
            click.echo(err.show())
        except SystemExit as err:
            # If a SystemExit comes from inside click, simply execute it.
            pass
            # click.echo("A system exit was triggered from inside robot_folders.")
        except click.exceptions.Exit as err:
            # If an Exit comes from inside click(v7), simply execute it.
            pass
            # click.echo("A exit was triggered from inside robot_folders.")
        except:
            click.echo("Execution of an unknown module failed. Exit with code 1.")
            click.echo("Here's a traceback for debugging purposes:")
            click.echo(traceback.format_exc())
            os._exit(1)


cli = RobotFolders(
    help="This tool helps you managing different robot environments. "
    "Use tab-completion for combining commands or type --help on each level "
    "to get a help message."
)
