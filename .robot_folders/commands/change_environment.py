import click
import os
import userconfig

from helpers.directory_helpers import get_last_activated_env, get_checkout_dir, get_catkin_dir

class EnvironmentChoice(click.Command):
    def invoke(self, ctx):
        with open( os.path.join(get_checkout_dir(), '.cur_env'), 'w') as file:
            file.write("{}".format(self.name))


class EnvironmentChooser(click.MultiCommand):
    def get_current_evironments(self):
        checkout_folder = get_checkout_dir()
        return [dir for dir in os.listdir(checkout_folder) if self.is_fzirob_environment(checkout_folder, dir)]

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

    def is_fzirob_environment(self, checkout_folder, dir):
        is_environment = False

        environment_folders = ['ic_workspace', 'mca_workspace']
        environment_folders = environment_folders + userconfig.directories.get('catkin_names', ["catkin_workspace"])
        environment_files = ['setup.bash', 'setup.zsh', 'setup.sh']

        possible_env = os.path.join(checkout_folder, dir)
        if os.path.isdir(possible_env):
            # check folders for exlistance
            for folder in environment_folders:
                if os.path.isdir(os.path.join(possible_env, folder)):
                    is_environment = True
                    break
            # check if folder was already found
            if not is_environment:
                # check files for existances
                for file in environment_files:
                    if os.path.exists(os.path.join(possible_env, file)):
                        is_environment = True
                        break

        return is_environment

@click.command(cls=EnvironmentChooser, short_help='Source an existing environment', invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Changes the global environment to the specified one. All environment-specific commands are then executed relative to that environment. \
       Only one environment can be active at a time."""
    if ctx.invoked_subcommand is None:
        env_name = get_last_activated_env()
        if env_name is not None:
            click.echo("No environment specified. Sourcing the most recent active environment: {}".format(env_name))
    else:
        pass
    # print(env_name)
