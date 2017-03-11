import click
import os
import subprocess

from yaml import load as yaml_load

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from helpers.directory_helpers import get_base_dir
from helpers.directory_helpers import get_checkout_dir
from helpers.directory_helpers import recursive_rmdir
from helpers.repository_helpers import create_rosinstall_entry
from helpers.directory_helpers import mkdir_p

global_remove_flag = False


class EnvironmentAdapter(click.Command):
    def invoke(self, ctx):
        env_dir = os.path.join(get_checkout_dir(), self.name)
        ic_dir = os.path.join(env_dir, "ic_workspace")
        mca_dir = os.path.join(env_dir, "mca_workspace")
        catkin_dir = os.path.join(env_dir, "catkin_workspace")
        ic_pkg_dir = os.path.join(env_dir, 'ic_workspace', 'packages')
        catkin_src_dir = os.path.join(env_dir, 'catkin_workspace', 'src')
        mca_library_dir = os.path.join(env_dir, 'mca_workspace', 'libraries')
        mca_project_dir = os.path.join(env_dir, 'mca_workspace', 'projects')
        mca_tool_dir = os.path.join(env_dir, 'mca_workspace', 'tools')
        demos_dir = os.path.join(env_dir, 'demos')
        self.yaml_data = self.parse_yaml_data(ctx.params['in_file'])

        if os.path.isdir(ic_pkg_dir):
            click.echo("Adapting IC workspace")
            self.rosinstall = dict()
            self.parse_folder(ic_pkg_dir)
            ic_rosinstall, ic_packages, ic_package_versions, ic_flags = \
                self.parse_ic_workspace_config()
            if ic_rosinstall:
                self.adapt_rosinstall(ic_rosinstall,
                                      ic_pkg_dir,
                                      workspace_dir=ic_dir,
                                      is_ic=True,
                                      ic_grab_flags=ic_flags)
            elif ic_packages:
                #TODO: compare the ic_dir_packages (with their versions) with the yaml_ic_rosinstall
                click.echo('Sorry! Currently, the package list format is not supported for'
                           ' adapting environments. Please use the rosinstall notation.')

        if os.path.isdir(catkin_src_dir):
            click.echo("Adapting catkin workspace")
            self.rosinstall = dict()
            self.parse_folder(catkin_src_dir)
            ros_rosinstall = self.parse_ros_workspace_config()
            if ros_rosinstall:
                self.adapt_rosinstall(ros_rosinstall, catkin_src_dir)

        if os.path.isdir(mca_library_dir):
            self.yaml_data['mca_workspace'] = dict()
            click.echo("Adapting mca libraries")
            self.rosinstall = dict()
            #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_project_dir):
                click.echo("Adapting mca projects")
                self.rosinstall = dict()
                #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

            if os.path.isdir(mca_tool_dir):
                click.echo("Adapting mca tools")
                self.rosinstall = dict()
                #TODO: compare parsed folder with yaml_data and adapt the ws accordingly

        click.echo('Looking for demo scripts')
        mkdir_p(demos_dir)
        scripts = self.parse_demo_scripts()
        for script in scripts:
            click.echo('Found {} in config. Will be overwritten if file exists'
                       .format(script))
            script_path = os.path.join(demos_dir, script)
            with open(script_path, mode='w') as f:
                f.write(scripts[script])
                f.close()
            os.chmod(script_path, 00755)


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
                process = subprocess.Popen(["git", "remote", "set-url", "origin", uri], cwd=package_dir)
                process.wait()

            # Checkout the version specified
            if version_update_required:
                process = subprocess.Popen(["git", "fetch"], cwd=package_dir)
                process.wait()
                #TODO: somehow decide wether to checkout the version from origin or local...
                process = subprocess.Popen(["git", "checkout", version], cwd=package_dir)
                process.wait()
                #TODO: add further git commands and/or some fancy output, if necessary

        config_name_list = [d['git']['local-name'] for d in config_rosinstall]

        for repo in self.rosinstall:
            if repo not in config_name_list:
                click.echo("Package '{}' found locally, but not in config.".format(repo))
                local_name = self.rosinstall[repo]["git"]["local-name"]
                package_dir = os.path.join(packages_dir, local_name)
                if global_remove_flag:
                    recursive_rmdir(package_dir)
                    click.echo("Deleted '{}'".format(repo))
                elif click.confirm('Do you want to delete it?'):

                    recursive_rmdir(package_dir)
                    click.echo("Deleted '{}'".format(repo))

    def parse_yaml_data(self, yaml_config):
        yaml_stream = file(yaml_config, "r")
        data = yaml_load(yaml_stream, Loader=Loader)
        return data

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

    def parse_ic_workspace_config(self):
        ic_rosinstall = None
        ic_packages = None
        ic_package_versions = None
        ic_grab_flags = []
        if 'ic_workspace' in self.yaml_data:
            if 'rosinstall' in self.yaml_data['ic_workspace']:
                ic_rosinstall = self.yaml_data['ic_workspace']['rosinstall']
                ic_packages = None

            elif 'packages' in self.yaml_data['ic_workspace']:
                ic_packages = ' '.join(self.yaml_data['ic_workspace']['packages'])
                ic_rosinstall = None

                if ('package_versions' in self.yaml_data['ic_workspace'] and
                        self.yaml_data['ic_workspace']['package_versions'] is not None):
                    ic_package_versions = self.yaml_data['ic_workspace']['package_versions']

            if ('flags' in self.yaml_data['ic_workspace']
                    and self.yaml_data['ic_workspace']['flags'] is not None):
                for flag in self.yaml_data['ic_workspace']['flags']:
                    ic_grab_flags.append("--" + flag)

        return ic_rosinstall, ic_packages, ic_package_versions, ic_grab_flags

    def parse_ros_workspace_config(self):
        ros_rosinstall = None
        if "catkin_workspace" in self.yaml_data:
            if "rosinstall" in self.yaml_data["catkin_workspace"]:
                ros_rosinstall = self.yaml_data["catkin_workspace"]["rosinstall"]

        return ros_rosinstall

    def parse_demo_scripts(self):
        script_list = dict()
        if 'demos' in self.yaml_data:
            for script in self.yaml_data['demos']:
                script_list[script] = self.yaml_data['demos'][script]
        return script_list


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
            cmd = EnvironmentAdapter(name=name, params=[click.Argument(param_decls=['in_file'])])
            return cmd
        else:
            click.echo('No environment with name < %s > found.' % name)
            return None


@click.command(cls=EnvironmentChooser,
               short_help='Adapt an environment to a config file',
               invoke_without_command=True)
@click.option("--remove_repositories",
              is_flag=True,
              default=False,
              help=("Remove repositories not mentioned by the config file "
                    "without confirmation. If not set, the user has to "
                    "confirm each deletion."))
@click.pass_context
def cli(ctx, remove_repositories):
    """Adapts an environment to a config file.
       New repositories will be added, versions/branches will be changed and
       deleted repositories will/may be removed.
    """
    #TODO: pass this somehow to the invoked multicommand-object instead of a global variable
    global global_remove_flag
    global_remove_flag = remove_repositories

    if ctx.invoked_subcommand is None:
        click.echo('No environment specified. Please choose one '
                   'of the available environments!')
