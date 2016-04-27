import click
import os
from helpers.directory_helpers import get_active_env_path, mkdir_p
import userconfig
import subprocess

def get_cmake_flags():
    """Reads the configuration for default cmake flags"""
    generator = userconfig.config.get('generator', 'make')

    cmake_flags = ''

    if generator == 'ninja':
        cmake_flags = " ".join([cmake_flags, "-GNinja"])

    return cmake_flags


class Builder(click.Command):
    build_dir = 'build'
    def get_build_command(self):
        """ Determine whether to use make or ninja. If it cannot be guessed
        from the build directory, use the config value"""

        build_cmd = 'make'

        cmake_cache_file = os.path.join(self.build_dir, 'CMakeCache.txt')
        search_str = 'CMAKE_MAKE_PROGRAM:FILEPATH='
        if os.path.isfile(cmake_cache_file):
            for line in open(cmake_cache_file):
                start = line.find(search_str)
                if start > -1:
                    # remove any trailing chars like newlines
                    build_cmd = line[start+len(search_str):].rstrip()
        else:
            build_cmd = userconfig.config.get('generator', 'make')


        if 'make' in build_cmd:
            if not '-j' in build_cmd:
                num_threads = userconfig.config.get('make_threads', 2)
                build_cmd = " ".join([build_cmd, '-j', str(num_threads)])

        return build_cmd


    def check_previous_build(self, base_directory):
        """ Checks whether the build directory exists and creates it if needed.
        Also performs an initial cmake command, if no CMakeCache.txt exists."""
        mkdir_p(self.build_dir)
        cmake_cache_file = os.path.join(self.build_dir, 'CMakeCache.txt')
        if not os.path.isfile(cmake_cache_file):
            cmake_cmd = " ".join(["cmake", base_directory, get_cmake_flags()])
            process = subprocess.Popen(["bash", "-c", cmake_cmd],
                                       cwd=self.build_dir)
            process.wait()


class IcBuilder(Builder):
    def invoke(self, ctx):
        ic_dir = os.path.realpath(os.path.join(get_active_env_path(), 'ic_workspace'))
        self.build_dir = os.path.realpath(os.path.join(ic_dir, 'build'))
        self.check_previous_build(ic_dir)
        click.echo("Building ic_workspace in {}".format(self.build_dir))
        build_cmd = " ".join([self.get_build_command(), "install"])
        process = subprocess.Popen(["bash", "-c", build_cmd],
                                        cwd=self.build_dir)
        process.wait()

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
        mca_dir = os.path.realpath(os.path.join(get_active_env_path(), 'mca_workspace'))
        self.build_dir = os.path.realpath(os.path.join(mca_dir, 'build'))
        self.check_previous_build(mca_dir)
        click.echo("Building ic_workspace in {}".format(self.build_dir))
        build_cmd = " ".join([self.get_build_command(), "install"])
        process = subprocess.Popen(["bash", "-c", build_cmd],
                                        cwd=self.build_dir)
        process.wait()
