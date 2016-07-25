import click
import os
import vcstools

from yaml import load as yaml_load, dump as yaml_dump, safe_dump as yaml_safe_dump

try:
    from yaml import CLoader as Loader, CDumper, Dumper
except ImportError:
    from yaml import Loader, Dumper

from helpers.directory_helpers import get_base_dir
#TODO: update imports: delete unused ones, etc.


class EnvironmentAdapter(click.Command):
    def invoke(self, ctx):
        env_dir = os.path.join(get_base_dir(), 'checkout', self.name)
        ic_pkg_dir = os.path.join(env_dir, 'ic_workspace', 'packages')
        catkin_src_dir = os.path.join(env_dir, 'catkin_workspace', 'src')
        mca_library_dir = os.path.join(env_dir, 'mca_workspace', 'libraries')
        mca_project_dir = os.path.join(env_dir, 'mca_workspace', 'projects')
        mca_tool_dir = os.path.join(env_dir, 'mca_workspace', 'tools')
        self.yaml_data = self.parse_yaml_data(ctx.params['in_file'])

        if os.path.isdir(ic_pkg_dir):
            click.echo("Adapting IC workspace")
            self.rosinstall = list()
            self.parse_folder(ic_pkg_dir)
            #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

        if os.path.isdir(catkin_src_dir):
            click.echo("Adapting catkin workspace")
            self.rosinstall = list()
            self.parse_folder(catkin_src_dir)
            #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

        if os.path.isdir(mca_library_dir):
            self.yaml_data['mca_workspace'] = dict()
            click.echo("Adapting mca libraries")
            self.rosinstall = list()
            #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_project_dir):
                click.echo("Adapting mca projects")
                self.rosinstall = list()
                #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_tool_dir):
                click.echo("Adapting mca tools")
                self.rosinstall = list()
                #TODO: compare parsed folder with yaml_data and adapt the ws accordingly


    def parse_yaml_data(self, yaml_config):
        #TODO: read the yaml config file into a dict
        return dict()

    def parse_folder(self, folder, prefix=''):
        #TODO: does this need to get changed for the new task?
        subfolders = os.listdir(folder)
        for subfolder in subfolders:
            subfolder_abs = os.path.join(folder, subfolder)
            if os.path.isdir(subfolder_abs):
                git_dir = os.path.join(subfolder_abs, '.git')
                local_name = os.path.join(prefix, subfolder)
                if os.path.isdir(git_dir):
                    click.echo(local_name)
                    entry = self.create_rosinstall_entry(subfolder_abs, local_name)
                    self.rosinstall.append(entry)
                self.parse_folder(subfolder_abs, local_name)

    def create_rosinstall_entry(self, repo_path, local_name):
        client = vcstools.git.GitClient(repo_path)
        repo = dict()
        repo['git'] = dict()
        repo['git']['local-name'] = local_name
        repo['git']['uri'] = client.get_url()
        repo['git']['version'] = client.get_current_version_label()
        return repo


class EnvironmentChooser(click.MultiCommand):
    def get_current_evironments(self):
        checkout_folder = os.path.join(get_base_dir(), 'checkout')
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if os.path.isdir(os.path.join(checkout_folder, dir))]

    def list_commands(self, ctx):
        return self.get_current_evironments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in self.get_current_evironments():
            cmd = EnvironmentAdapter(name=name, params=[click.Argument(param_decls=['in_file'])])
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser,
               short_help='Adapt an environment to a config file',
               invoke_without_command=True)
@click.pass_context
def cli(ctx):
    """Adapts an environment to a config file.
       New repositories will be added, versions/branches will be changed and \
       deleted repositories will/may be removed.
    """
    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
