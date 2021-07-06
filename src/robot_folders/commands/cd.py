"""This command populates the 'cd' functionality"""
import os
import click

from helpers.workspace_chooser import WorkspaceChooser
import helpers.directory_helpers as dir_helpers


class CdCommand(click.Command):
    """Command to output a cd command"""
    def __init__(self, name=None, target_dir=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.short_help = target_dir
        self.target_dir = target_dir

    def invoke(self, ctx):
        """Prints a cd command to the output"""
        click.echo("cd {}".format(self.target_dir))


class CdChooser(WorkspaceChooser):
    """Class implementing the cd command"""

    def get_command(self, ctx, name):
        env = dir_helpers.get_active_env()
        if env is None:
            click.echo("No active environment found. Using most recently activated \
environment '{}'".format(dir_helpers.get_last_activated_env()))
            env = dir_helpers.get_last_activated_env()

        target_dir = dir_helpers.get_active_env_path()
        if name in self.list_commands(ctx):
            if name == 'ic':
                target_dir = os.path.join(target_dir, 'ic_workspace')
            elif name == 'ros':
                target_dir = dir_helpers.get_catkin_dir()
            elif name == 'colcon':
                target_dir = dir_helpers.get_colcon_dir()
            elif name == 'mca':
                target_dir = os.path.join(target_dir, 'mca_workspace')
        else:
            click.echo('Did not find a workspace with the key < {} > inside '
                       'current environment < {} >.'.format(name, env))
            return self

        return CdCommand(name=name, target_dir=target_dir)


@click.command('cd', cls=CdChooser, invoke_without_command=True,
               short_help='CDs to a workspace inside the active environment')
@click.pass_context
def cli(ctx):
    """ CDs to a workspace inside the active environment
    """

    if ctx.invoked_subcommand is None and \
       ctx.parent.invoked_subcommand == 'cd':
        if dir_helpers.get_active_env() is None:
            click.echo("No active environment found. Using most recently "
                       "activated environment")
        active_env_path = dir_helpers.get_active_env_path()
        if active_env_path is not None:
            click.echo("cd {}".format(active_env_path))
    return
