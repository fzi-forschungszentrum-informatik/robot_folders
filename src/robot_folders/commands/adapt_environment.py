"""
This implements a command to adapt an environment with a config file.
"""
from __future__ import print_function
import os
import stat
import subprocess
import click

import helpers.directory_helpers as dir_helpers
from helpers.repository_helpers import create_rosinstall_entry
from helpers.ConfigParser import ConfigFileParser
import helpers.environment_helpers as environment_helpers


class EnvironmentAdapter(click.Command):
    """
    Implements a click command interface to adapt an environment.
    """

    def __init__(self, name=None, **attrs):
        click.Command.__init__(self, name, **attrs)

        self.local_delete_policy = 'ask'
        self.rosinstall = dict()

    def invoke(self, ctx):
        """
        This invokes the actual command.
        """
        env_dir = os.path.join(dir_helpers.get_checkout_dir(), self.name)
        ic_dir = os.path.join(env_dir, "ic_workspace")
        mca_dir = os.path.join(env_dir, "mca_workspace")
        catkin_dir = dir_helpers.get_catkin_dir(env_dir)
        ic_pkg_dir = os.path.join(env_dir, 'ic_workspace', 'packages')
        catkin_src_dir = os.path.join(catkin_dir, 'src')
        mca_library_dir = os.path.join(mca_dir, 'libraries')
        mca_project_dir = os.path.join(mca_dir, 'projects')
        mca_tool_dir = os.path.join(mca_dir, 'tools')
        demos_dir = os.path.join(env_dir, 'demos')

        self.local_delete_policy = ctx.parent.params['local_delete_policy']

        config_file_parser = ConfigFileParser(ctx.params['in_file'])
        has_ic, ic_rosinstall, ic_packages, ic_package_versions, ic_flags = \
            config_file_parser.parse_ic_config()
        has_catkin, ros_rosinstall = config_file_parser.parse_ros_config()
        has_mca, mca_additional_repos = config_file_parser.parse_mca_config()
        os.environ['ROB_FOLDERS_ACTIVE_ENV'] = self.name

        if has_ic:
            if os.path.isdir(ic_pkg_dir):
                click.echo("Adapting IC workspace")
                self.parse_folder(ic_pkg_dir)
                if ic_rosinstall:
                    self.adapt_rosinstall(ic_rosinstall,
                                          ic_pkg_dir,
                                          workspace_dir=ic_dir,
                                          is_ic=True,
                                          ic_grab_flags=ic_flags)
                elif ic_packages:
                    # TODO: compare the ic_dir_packages (with their versions)
                    # with the yaml_ic_rosinstall
                    click.echo('Sorry! Currently, the package list format is not supported for'
                               ' adapting environments. Please use the rosinstall notation.')
            else:
                click.echo("Creating IC workspace")
                has_nobackup = dir_helpers.check_nobackup()
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                ic_build_dir = os.path.join(build_base_dir, self.name, 'ic_workspace', 'build')

                environment_helpers.IcCreator(ic_directory=ic_dir,
                                              build_directory=ic_build_dir,
                                              rosinstall=ic_rosinstall,
                                              packages=ic_packages,
                                              package_versions=ic_package_versions,
                                              grab_flags=ic_flags)

        if has_catkin:
            if os.path.isdir(catkin_src_dir):
                click.echo("Adapting catkin workspace")
                self.rosinstall = dict()
                self.parse_folder(catkin_src_dir)
                if ros_rosinstall:
                    self.adapt_rosinstall(ros_rosinstall, catkin_src_dir)
            else:
                click.echo("Creating catkin workspace")
                has_nobackup = dir_helpers.check_nobackup()
                catkin_dir = os.path.join(env_dir, "catkin_ws")
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                catkin_build_dir = os.path.join(build_base_dir, self.name, 'catkin_ws', 'build')

                environment_helpers.CatkinCreator(catkin_directory=catkin_dir,
                                                  build_directory=catkin_build_dir,
                                                  rosinstall=ros_rosinstall)

        if has_mca:
            if os.path.isdir(mca_library_dir):
                click.echo("Adapting mca libraries")
                self.rosinstall = dict()
                self.parse_folder(mca_library_dir)
                if 'libraries' in mca_additional_repos:
                    self.adapt_rosinstall(mca_additional_repos['libraries'],
                                          mca_library_dir)

                if os.path.isdir(mca_project_dir):
                    click.echo("Adapting mca projects")
                    self.rosinstall = dict()
                    self.parse_folder(mca_project_dir)
                    if 'projects' in mca_additional_repos:
                        self.adapt_rosinstall(mca_additional_repos['projects'],
                                              mca_project_dir)

                if os.path.isdir(mca_tool_dir):
                    click.echo("Adapting mca tools")
                    self.rosinstall = dict()
                    self.parse_folder(mca_tool_dir)
                    if 'tools' in mca_additional_repos:
                        self.adapt_rosinstall(mca_additional_repos['tools'],
                                              mca_tool_dir)
            else:
                click.echo("Creating mca workspace")
                has_nobackup = dir_helpers.check_nobackup()
                build_base_dir = dir_helpers.get_build_base_dir(has_nobackup)
                mca_build_dir = os.path.join(build_base_dir, self.name, 'mca_workspace', 'build')

                environment_helpers.MCACreator(mca_directory=mca_dir,
                                               build_directory=mca_build_dir,
                                               mca_additional_repos=mca_additional_repos)

        click.echo('Looking for demo scripts')
        dir_helpers.mkdir_p(demos_dir)
        scripts = config_file_parser.parse_demo_scripts()
        for script in scripts:
            click.echo('Found {} in config. Will be overwritten if file exists'
                       .format(script))
            script_path = os.path.join(demos_dir, script)
            with open(script_path, mode='w') as demo_script:
                demo_script.write(scripts[script])
                demo_script.close()
            os.chmod(script_path,
                     stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)

    def adapt_rosinstall(self,
                         config_rosinstall,
                         packages_dir,
                         workspace_dir="",
                         is_ic=False,
                         ic_grab_flags=None):
        """
        Parses the given config rosinstall and compares it to the locally installed packages
        """
        for repo in config_rosinstall:
            local_version_exists = False
            version_update_required = True
            uri_update_required = True
            local_name = ''
            uri = ''
            version = ''

            if 'local-name' in repo['git']:
                local_name = repo["git"]["local-name"]
            else:
                click.echo("No local-name given for package '{}'. "
                           "Skipping package".format(repo["git"]))
                continue
            package_dir = os.path.join(packages_dir, local_name)

            if 'version' in repo['git']:
                version = repo["git"]["version"]
            else:
                click.echo("WARNING: No version tag given for package '{}'. "
                           "The local version will be kept or the master "
                           "version will be checked out for new package".format(local_name))

            if 'uri' in repo['git']:
                uri = repo["git"]["uri"]
            else:
                click.echo("WARNING: No uri given for package '{}'. "
                           "Skipping package".format(local_name))
                continue

            # compare the repos' versions and uris
            if local_name in self.rosinstall.keys():
                local_version_exists = True
                local_repo = self.rosinstall[local_name]
                if version == '' or version == local_repo["git"]["version"]:
                    version_update_required = False
                else:
                    click.echo("Package '{}' version differs from local version. "
                               .format(local_name))
                    click.echo("1) local version: {}".format(local_repo["git"]["version"]))
                    click.echo("2) config_file version: {}".format(version))
                    version_to_keep = click.prompt("Which version should be used?",
                                                   type=click.Choice(['1', '2']),
                                                   default='1')
                    version_update_required = (version_to_keep == '2')

                if uri == local_repo["git"]["uri"]:
                    uri_update_required = False
                else:
                    click.echo("Package '{}' uri differs from local version. "
                               .format(local_name))
                    click.echo("local version: {}".format(local_repo["git"]["uri"]))
                    click.echo("config_file version: {}".format(uri))
                    version_to_keep = click.prompt("Which uri should be used?",
                                                   type=click.Choice(['1', '2']),
                                                   default='2')
                    uri_update_required = (version_to_keep == '2')

            else:
                click.echo("Package '{}' does not exist in local structure. "
                           "Going to download.".format(local_name))

            # Create repo if it does not exist yet.
            if not local_version_exists:
                if is_ic:
                    # use the IcWorkspace.py to grab the ic-package
                    if workspace_dir != "":
                        grab_command = ["./IcWorkspace.py", "grab", local_name]
                        if ic_grab_flags is not None:
                            grab_command.extend(ic_grab_flags)
                        process = subprocess.Popen(grab_command, cwd=workspace_dir)
                        process.wait()
                else:
                    subprocess.check_call(["git", "clone", uri, package_dir])

            # Change the origin to the uri specified
            if uri_update_required:
                process = subprocess.Popen(["git", "remote", "set-url", "origin", uri],
                                           cwd=package_dir)
                process.wait()

            # Checkout the version specified
            if version_update_required:
                process = subprocess.Popen(["git", "fetch"], cwd=package_dir)
                process.wait()
                process = subprocess.Popen(["git", "checkout", version], cwd=package_dir)
                process.wait()

        config_name_list = [d['git']['local-name'] for d in config_rosinstall]

        for repo in self.rosinstall:
            if repo not in config_name_list:
                click.echo("Package '{}' found locally, but not in config.".format(repo))
                local_name = self.rosinstall[repo]["git"]["local-name"]
                package_dir = os.path.join(packages_dir, local_name)
                if self.local_delete_policy == 'delete_all':
                    dir_helpers.recursive_rmdir(package_dir)
                    click.echo("Deleted '{}'".format(repo))
                elif self.local_delete_policy == 'ask':
                    if click.confirm('Do you want to delete it?'):
                        dir_helpers.recursive_rmdir(package_dir)
                        click.echo("Deleted '{}'".format(repo))
                elif self.local_delete_policy == 'keep_all':
                    click.echo("Keeping repository as all should be kept")

    def parse_folder(self, folder, prefix=""):
        """
        Function to recursively find git repositories
        """
        subfolders = os.listdir(folder)
        for subfolder in subfolders:
            subfolder_abs = os.path.join(folder, subfolder)
            if os.path.isdir(subfolder_abs):
                git_dir = os.path.join(subfolder_abs, '.git')
                local_name = os.path.join(prefix, subfolder)
                if os.path.isdir(git_dir):
                    click.echo("Found '{}' in local folder structure.".format(local_name))
                    entry = create_rosinstall_entry(subfolder_abs, local_name)
                    self.rosinstall[local_name] = entry
                self.parse_folder(subfolder_abs, local_name)


class EnvironmentChooser(click.MultiCommand):
    """Class to select an environment"""

    def list_commands(self, ctx):
        return dir_helpers.list_environments()

    def get_command(self, ctx, name):
        # return empty command with the correct name
        if name in dir_helpers.list_environments():
            cmd = EnvironmentAdapter(name=name, params=[click.Argument(param_decls=['in_file'])])
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser,
               short_help='Adapt an environment to a config file',
               invoke_without_command=True)
@click.option("--local_delete_policy",
              type=click.Choice(['delete_all', 'keep_all', 'ask']),
              default='ask',
              help=('Defines whether repositories existing local, but not '
                    'in the config file should be kept, deleted or the '
                    'user should be asked. Asking is the default behavior.'))
@click.pass_context
def cli(ctx, local_delete_policy):
    """Adapts an environment to a config file.
       New repositories will be added, versions/branches will be changed and
       deleted repositories will/may be removed.
    """
    print("The policy for deleting repos only existing locally is: '{}'"
          .format(local_delete_policy))

    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
