"""Defines exceptions used with robot_folders"""

class ModuleException(Exception):
    """Generic exception in one command module"""
    def __init__(self, message, module_name):
        super(ModuleException, self).__init__(message)
        self.message = message
        self.module_name = module_name

    def __str__(self):
        return repr(self.message)
