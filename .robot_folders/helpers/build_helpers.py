import click
import os
from helpers.directory_helpers import get_active_env_path
import subprocess

class Builder(click.Command):
    def get_build_command(self, build_dir):
        # default: make -j1
        build_cmd = "make"

        # ninja
        if os.path.isfile(os.path.join(build_dir, "build.ninja")):
            build_cmd="ninja"

        return build_cmd

class IcBuilder(Builder):
    def invoke(self, ctx):
        build_dir = os.path.join(get_active_env_path(), 'ic_workspace', 'build')
        click.echo("Building ic_workspace in {}".format(build_dir))
        build_cmd = " ".join([self.get_build_command(build_dir), "install"])
        process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=build_dir)
        process.wait()

class CatkinBuilder(Builder):
    def get_build_command(self, catkin_dir):
        # default: make
        build_cmd = "catkin_make"

        # ninja
        if os.path.isfile(os.path.join(catkin_dir, "build", "build.ninja")):
            build_cmd="catkin_make --use-ninja"

        return build_cmd

    def invoke(self, ctx):
        catkin_dir = os.path.join(get_active_env_path(), 'catkin_workspace')
        click.echo("Building catkin_workspace in {}".format(catkin_dir))
        build_cmd = self.get_build_command(catkin_dir)
        process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=catkin_dir)
        process.wait()

# TODO: We could support building of single projects? Unfortunately, I don't know
#       much about mca. (mauch: 20160417)
class McaBuilder(Builder):
    def invoke(self, ctx):
        build_dir = os.path.join(get_active_env_path(), 'mca_workspace', 'build')
        click.echo("Building mca_workspace in {}".format(build_dir))
        build_cmd = " ".join([self.get_build_command(build_dir), "install"])

        process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=build_dir)
        process.wait()
