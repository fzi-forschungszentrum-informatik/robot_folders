import click
import os
from helpers.directory_helpers import get_active_env_path, mkdir_p
import subprocess

class Builder(click.Command):
    build_dir = 'build'
    def get_build_command(self, build_dir):
        # default: make -j1
        build_cmd = "make"

        cmake_cache_file = os.path.join(self.build_dir, 'CMakeCache.txt')
        search_str = 'CMAKE_MAKE_PROGRAM:FILEPATH='
        if os.path.isfile(cmake_cache_file):
            for line in open(cmake_cache_file):
                start = line.find(search_str)
                if start > -1:
                    # remove any trailing chars like newlines
                    build_cmd = line[start+len(search_str):].rstrip()

        return build_cmd

    def check_previous_build(self, base_directory):
        mkdir_p(self.build_dir)
        cmake_cache_file = os.path.join(self.build_dir, 'CMakeCache.txt')
        if not os.path.isfile(cmake_cache_file):
            cmake_cmd = " ".join(["cmake", base_directory])
            process = subprocess.Popen(["bash", "-c", cmake_cmd],
                                       cwd=self.build_dir)
            process.wait()
        return True


class IcBuilder(Builder):
    def invoke(self, ctx):
        self.build_dir = os.path.join(get_active_env_path(), 'ic_workspace', 'build')
        self.build_dir = os.path.realpath(self.build_dir)
        click.echo("Build directory: {}".format(self.build_dir))
        if self.check_previous_build(os.path.join(get_active_env_path(), 'ic_workspace')):
            click.echo("Building ic_workspace in {}".format(self.build_dir))
            build_cmd = " ".join([self.get_build_command(self.build_dir), "install"])
            process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=self.build_dir)
            process.wait()
        else:
            pass

class CatkinBuilder(Builder):
    def get_build_command(self, catkin_dir):
        # default: make
        build_cmd = "catkin_make"
        mkdir_p(self.build_dir)


        cmake_cache_file = os.path.join(catkin_dir, 'build', 'CMakeCache.txt')
        search_str = 'CMAKE_MAKE_PROGRAM:FILEPATH='
        if os.path.isfile(cmake_cache_file):
            for line in open(cmake_cache_file):
                start = line.find(search_str)
                if start > -1:
                    # remove any trailing chars like newlines
                    if "ninja" in line:
                        build_cmd="catkin_make --use-ninja"

        return build_cmd

    def invoke(self, ctx):
        catkin_dir = os.path.join(get_active_env_path(), 'catkin_workspace')
        self.build_dir = os.path.join(catkin_dir, 'build')
        click.echo("Building catkin_workspace in {}".format(catkin_dir))
        build_cmd = self.get_build_command(catkin_dir)
        process = subprocess.Popen(["bash", "-c", build_cmd],
                                               cwd=catkin_dir)
        process.wait()

# TODO: We could support building of single projects? Unfortunately, I don't know
#       much about mca. (mauch: 20160417)
class McaBuilder(Builder):
    def invoke(self, ctx):
        self.build_dir = os.path.join(get_active_env_path(), 'mca_workspace', 'build')
        self.build_dir = os.path.realpath(self.build_dir)
        if self.check_previous_build(os.path.join(get_active_env_path(), 'mca_workspace')):
            click.echo("Building mca_workspace in {}".format(self.build_dir))
            build_cmd = " ".join([self.get_build_command(self.build_dir), "install"])

            process = subprocess.Popen(["bash", "-c", build_cmd],
                                       cwd=self.build_dir)
            process.wait()
