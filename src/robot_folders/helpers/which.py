"""Module for implementing the 'which' command in python"""
import os


def which(program):
    """Defines a 'which' function similar to the linux command"""

    def is_exe(fpath):
        """Checks whether a path points to an executable file"""
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    for os_path in os.environ["PATH"].split(os.pathsep):
        path = os_path + os.sep + program
        if is_exe(path):
          return program

    return None
