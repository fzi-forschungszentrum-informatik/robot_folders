"""This command implements the change_environment functionality"""
import os
import click
import subprocess

import robot_folders.helpers.directory_helpers as dir_helpers
from robot_folders.helpers.exceptions import ModuleException

class EnvironmentChoice(click.Command):
    """
    Writes the chosen environment into the cur_env temp file.
    NOTE: The actual sourcing takes place in a a shell script that reads that file's content
    """

    def invoke(self, ctx):
        with open(os.path.join(dir_helpers.get_checkout_dir(), '.cur_env'), 'w') as cur_env_file:
            cur_env_file.write("{}".format(self.name))


class EnvironmentChooser(click.MultiCommand):
    """Class that helps finding and choosing an environment."""

    def list_commands(self, ctx):
        return dir_helpers.list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in dir_helpers.list_environments():
            cmd = EnvironmentChoice(name=name, add_help_option=False)
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            raise ModuleException("unknown environment", "EnvironmentChoice", 1)

    def invoke(self, ctx):
        try:
            super(EnvironmentChooser, self).invoke(ctx)
        except subprocess.CalledProcessError as err:
            raise(ModuleException(str(err), 'change'))


@click.command('change_environment', cls=EnvironmentChooser, short_help='Source an existing environment',
               invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """
    Changes the global environment to the specified one. All environment-specific commands
    are then executed relative to that environment.
    Only one environment can be active at a time.
    """
    if ctx.invoked_subcommand is None:
        env_name = dir_helpers.get_last_activated_env()
        if env_name is not None:
            click.echo(
                "No environment specified. Sourcing the most recent active environment: {}"
                .format(env_name))
    else:
        pass
    # print(env_name)
