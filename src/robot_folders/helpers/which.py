"""Module for implementing the 'which' command in python"""
import os

def which(program):
    """Defines a 'which' function similar to the linux command"""

    def is_exe(fpath):
        """Checks whether a path points to an executable file"""
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None
