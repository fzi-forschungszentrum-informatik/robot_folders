import click
import os

from helpers.directory_helpers import get_base_dir

class EnvironmentChoice(click.Command):
    def invoke(self, ctx):
        with open( os.path.join(get_base_dir(), 'checkout', '') + '.cur_env', 'w') as file:
            file.write("{}".format(self.name))


class EnvironmentChooser(click.MultiCommand):
    def get_current_evironments(self):
        checkout_folder = os.path.join(get_base_dir(), 'checkout')
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        return self.get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        # TODO improve logging for invalid environments
        if name in self.get_current_evironments():
            cmd = EnvironmentChoice(name=name, add_help_option=False)
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser, short_help='Source an existing environment', invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Changes the global environment to the specified one. All environment-specific commands are then executed relative to that environment. \
       Only one environment can be active at a time."""
    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one of the available environments!')
    else:
        pass
    # print(env_name)
