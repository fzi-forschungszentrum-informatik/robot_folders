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

from helpers.exceptions import ModuleException

plugin_folder = os.path.join(os.path.dirname(__file__), 'commands')

class RobotFolders(click.MultiCommand):

    def list_commands(self, ctx):
        rv = []
        for filename in os.listdir(plugin_folder):
            if filename.endswith('.py'):
                rv.append(filename[:-3])
        rv.sort()
        return rv

    def get_command(self, ctx, name):
        ns = {}
        fn = os.path.join(plugin_folder, name + '.py')
        if os.path.isfile(fn):
            with open(fn) as f:
                code = compile(f.read(), fn, 'exec')
                eval(code, ns, ns)
                return ns['cli']
        else:
            return None

    def invoke(self, ctx):
        try:
            super(RobotFolders, self).invoke(ctx)
        except ModuleException as err:
            click.echo("Execution of module '{}' failed. Error message:\n{}".format(
                err.module_name, err))

cli = RobotFolders(help='This tool helps you managing different robot environments. '
                        'Use tab-completion for combining commands or type --help on each level '
                        'to get a help message.')

