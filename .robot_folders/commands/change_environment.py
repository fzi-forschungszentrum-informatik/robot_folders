"""This command implements the change_environment functionality"""
import os
import click

import helpers.config_helpers as config_helpers
from helpers.directory_helpers import get_last_activated_env, get_checkout_dir


def is_fzirob_environment(checkout_folder, env_dir):
    """Checks whether a given directory actually contains an environment"""
    is_environment = False

    environment_folders = ['ic_workspace', 'mca_workspace']
    environment_folders = environment_folders + \
        config_helpers.get_value_safe_default(
            section='directories',
            value='catkin_names',
            default=["catkin_workspace"])
    environment_files = ['setup.bash', 'setup.zsh', 'setup.sh']

    possible_env = os.path.join(checkout_folder, env_dir)
    if os.path.isdir(possible_env):
        # check folders for existence
        for folder in environment_folders:
            if os.path.isdir(os.path.join(possible_env, folder)):
                is_environment = True
                break
        # check if folder was already found
        if not is_environment:
            # check files for existences
            for filename in environment_files:
                if os.path.exists(os.path.join(possible_env, filename)):
                    is_environment = True
                    break

    return is_environment


def get_current_evironments():
    """List all environments"""
    checkout_folder = get_checkout_dir()
    return [env_dir for env_dir in os.listdir(checkout_folder)
            if is_fzirob_environment(checkout_folder, env_dir)]

class EnvironmentChoice(click.Command):
    """
    Writes the chosen environment into the cur_env temp file.
    NOTE: The actual sourcing takes place in a a shell script that reads that file's content
    """

    def invoke(self, ctx):
        with open(os.path.join(get_checkout_dir(), '.cur_env'), 'w') as cur_env_file:
            cur_env_file.write("{}".format(self.name))


class EnvironmentChooser(click.MultiCommand):
    """Class that helps finding and choosing an environment."""


    def list_commands(self, ctx):
        return get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in get_current_evironments():
            cmd = EnvironmentChoice(name=name, add_help_option=False)
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None



@click.command(cls=EnvironmentChooser, short_help='Source an existing environment',
               invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """
    Changes the global environment to the specified one. All environment-specific commands
    are then executed relative to that environment.
    Only one environment can be active at a time.
    """
    if ctx.invoked_subcommand is None:
        env_name = get_last_activated_env()
        if env_name is not None:
            click.echo(
                "No environment specified. Sourcing the most recent active environment: {}"
                .format(env_name))
    else:
        pass
    # print(env_name)
