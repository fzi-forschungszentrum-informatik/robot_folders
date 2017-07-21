import click
import os
import stat
import subprocess

from helpers.directory_helpers import get_checkout_dir
from helpers.directory_helpers import recursive_rmdir
from helpers.directory_helpers import mkdir_p
from helpers.directory_helpers import get_catkin_dir
from helpers.repository_helpers import create_rosinstall_entry
from helpers.ConfigParser import ConfigFileParser

local_delete_policy_saved = False


class EnvironmentAdapter(click.Command):
    def invoke(self, ctx):
        env_dir = os.path.join(get_checkout_dir(), self.name)
        ic_dir = os.path.join(env_dir, "ic_workspace")
        mca_dir = os.path.join(env_dir, "mca_workspace")
        catkin_dir = get_catkin_dir(env_dir)
        ic_pkg_dir = os.path.join(env_dir, 'ic_workspace', 'packages')
        catkin_src_dir = os.path.join(catkin_dir, 'src')
        mca_library_dir = os.path.join(mca_dir, 'libraries')
        mca_project_dir = os.path.join(mca_dir, 'projects')
        mca_tool_dir = os.path.join(mca_dir, 'tools')
        demos_dir = os.path.join(env_dir, 'demos')

        self.config_file_parser = ConfigFileParser(ctx.params['in_file'])
        has_ic, ic_rosinstall, ic_packages, ic_package_versions, ic_flags = \
            self.config_file_parser.parse_ic_config()
        has_catkin, ros_rosinstall = self.config_file_parser.parse_ros_config()
        create_mca, mca_additional_repos = self.config_file_parser.parse_mca_config()

        if has_ic:
            if os.path.isdir(ic_pkg_dir):
                click.echo("Adapting IC workspace")
                self.rosinstall = dict()
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
                # TODO: Create Ic workspace here
                # environment_helpers.IcCreator(ic_directory=self.ic_directory,
                                                # build_directory=self.ic_build_directory,
                                                # rosinstall=self.ic_rosinstall,
                                                # packages=self.ic_packages,
                                                # package_versions=self.ic_package_versions,
                                                # grab_flags=self.ic_grab_flags)
                pass

        if has_catkin:
            if os.path.isdir(catkin_src_dir):
                click.echo("Adapting catkin workspace")
                self.rosinstall = dict()
                self.parse_folder(catkin_src_dir)
                if ros_rosinstall:
                    self.adapt_rosinstall(ros_rosinstall, catkin_src_dir)
            else:
                # TODO: create catkin_ws
                pass

        if os.path.isdir(mca_library_dir):
            self.yaml_data['mca_workspace'] = dict()
            click.echo("Adapting mca libraries")
            self.rosinstall = dict()
            # TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_project_dir):
                click.echo("Adapting mca projects")
                self.rosinstall = dict()
                # TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_tool_dir):
                click.echo("Adapting mca tools")
                self.rosinstall = dict()
                # TODO: compare parsed folder with yaml_data and adapt the ws accordingly
        else:
            # TODO: Create mca_workspace if desired
            pass

        click.echo('Looking for demo scripts')
        mkdir_p(demos_dir)
        scripts = self.config_file_parser.parse_demo_scripts()
        for script in scripts:
            click.echo('Found {} in config. Will be overwritten if file exists'
                       .format(script))
            script_path = os.path.join(demos_dir, script)
            with open(script_path, mode='w') as f:
                f.write(scripts[script])
                f.close()
            os.chmod(script_path,
                     stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)

    def adapt_rosinstall(self,
                         config_rosinstall,
                         packages_dir,
                         workspace_dir="",
                         is_ic=False,
                         ic_grab_flags=None):
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
                               "Going to checkout config file version.".format(local_name))
                if uri == local_repo["git"]["uri"]:
                    uri_update_required = False
                else:
                    click.echo("Package '{}' uri differs from local version. "
                               "Going to set config file uri.".format(local_name))

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
                # TODO: somehow decide wether to checkout the version from origin or local...
                process = subprocess.Popen(["git", "checkout", version], cwd=package_dir)
                process.wait()
                # TODO: add further git commands and/or some fancy output, if necessary

        config_name_list = [d['git']['local-name'] for d in config_rosinstall]

        for repo in self.rosinstall:
            if repo not in config_name_list:
                click.echo("Package '{}' found locally, but not in config.".format(repo))
                local_name = self.rosinstall[repo]["git"]["local-name"]
                package_dir = os.path.join(packages_dir, local_name)
                if local_delete_policy_saved == 'delete_all':
                    recursive_rmdir(package_dir)
                    click.echo("Deleted '{}'".format(repo))
                elif local_delete_policy_saved == 'ask':
                    if click.confirm('Do you want to delete it?'):
                        recursive_rmdir(package_dir)
                        click.echo("Deleted '{}'".format(repo))
                elif local_delete_policy_saved == 'keep_all':
                    click.echo("Keeping repository as all should be kept")

    def parse_folder(self, folder, prefix=""):
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
    def get_current_evironments(self):
        checkout_folder = get_checkout_dir()
        # TODO possibly check whether the directory contains actual workspace
        return [dir for dir in os.listdir(checkout_folder) if
                os.path.isdir(os.path.join(checkout_folder, dir))]

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
    # TODO: pass this somehow to the invoked multicommand-object instead of a global variable
    global local_delete_policy_saved
    local_delete_policy_saved = local_delete_policy

    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
