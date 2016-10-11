import click
import os

from yaml import load as yaml_load, dump as yaml_dump, safe_dump as yaml_safe_dump

try:
    from yaml import CLoader as Loader, CDumper, Dumper
except ImportError:
    from yaml import Loader, Dumper

from helpers.directory_helpers import get_base_dir, get_checkout_dir
from helpers.repository_helpers import create_rosinstall_entry


class EnvironmentScraper(click.Command):
    def invoke(self, ctx):
        env_dir = os.path.join(get_checkout_dir(), self.name)
        ic_pkg_dir = os.path.join(env_dir, 'ic_workspace', 'packages')
        catkin_src_dir = os.path.join(env_dir, 'catkin_workspace', 'src')
        mca_library_dir = os.path.join(env_dir, 'mca_workspace', 'libraries')
        mca_project_dir = os.path.join(env_dir, 'mca_workspace', 'projects')
        mca_tool_dir = os.path.join(env_dir, 'mca_workspace', 'tools')
        self.yaml_data = dict()

        if os.path.isdir(ic_pkg_dir):
            click.echo("Scraping IC workspace")
            self.rosinstall = list()
            self.parse_folder(ic_pkg_dir)
            self.yaml_data['ic_workspace'] = dict()
            self.yaml_data['ic_workspace']['rosinstall'] = self.rosinstall

        if os.path.isdir(catkin_src_dir):
            click.echo("Scraping catkin workspace")
            self.rosinstall = list()
            self.parse_folder(catkin_src_dir)
            self.yaml_data['catkin_workspace'] = dict()
            self.yaml_data['catkin_workspace']['rosinstall'] = self.rosinstall

        if os.path.isdir(mca_library_dir):
            self.yaml_data['mca_workspace'] = dict()
            click.echo("Scraping mca libraries")
            self.rosinstall = list()
            self.parse_folder(mca_library_dir)
            self.yaml_data['mca_workspace']['libraries'] = self.rosinstall

            if os.path.isdir(mca_project_dir):
                click.echo("Scraping mca projects")
                self.rosinstall = list()
                self.parse_folder(mca_project_dir)
                self.yaml_data['mca_workspace']['projects'] = self.rosinstall

            if os.path.isdir(mca_tool_dir):
                click.echo("Scraping mca tools")
                self.rosinstall = list()
                self.parse_folder(mca_tool_dir)
                self.yaml_data['mca_workspace']['tools'] = self.rosinstall

        yaml_stream = file(ctx.params['out_file'], 'w')
        yaml_safe_dump(self.yaml_data, stream=yaml_stream, encoding='utf-8', allow_unicode=True)

    def parse_folder(self, folder, prefix=''):
        subfolders = os.listdir(folder)
        for subfolder in subfolders:
            subfolder_abs = os.path.join(folder, subfolder)
            if os.path.isdir(subfolder_abs):
                git_dir = os.path.join(subfolder_abs, '.git')
                local_name = os.path.join(prefix, subfolder)
                if os.path.isdir(git_dir):
                    click.echo(local_name)
                    entry = create_rosinstall_entry(subfolder_abs, local_name)
                    self.rosinstall.append(entry)
                self.parse_folder(subfolder_abs, local_name)


class EnvironmentChooser(click.MultiCommand):
    def get_current_evironments(self):
        checkout_folder = get_checkout_dir()
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        return self.get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in self.get_current_evironments():
            cmd = EnvironmentScraper(name=name, params=[click.Argument(param_decls=['out_file'])])
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser,
               short_help='Scrape an environment config to a config file',
               invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Scrapes an environment configuration into a config file,
       so that it can be given to somebody else. \
       This config file can then be used to initialize the environment
       in another robot_folders configuration.
       """
    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
