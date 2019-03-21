"""Module for reading and handling the config."""
from __future__ import print_function
import os
import imp
import shutil
import yaml

try:
    FileNotFoundError
except NameError:
    FileNotFoundError = IOError

class Userconfig(object):
    """Class for managing a userconfig"""
    config = None
    config_fallback = None
    initialized = False
    @classmethod
    def init_class(cls):
        """ Load the distribution config file """
        filename_distribute = os.path.join(Userconfig.get_base_dir(),
                                           'config',
                                           'userconfig_distribute.yaml')
        with open(filename_distribute, 'r') as file_content:
            try:
                Userconfig.config_fallback = yaml.load(file_content, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print("Error in configuration file:", exc)
            except IOError as exc:
                print('ERROR: There was a problem loading the distribution file:', exc)

        # Load the user-modified config file
        filename_userconfig = os.path.join(Userconfig.get_base_dir(),
                                           'config',
                                           'userconfig.yaml')
        # check for migration
        location_old = os.path.join(Userconfig.get_base_dir(),
                                    '.robot_folders',
                                    'userconfig.py')
        if os.path.isfile(location_old):
            print('\n\n!!!!! MIGRATION WARNING !!!!!')
            print('The config file format and location have changed.')
            print('The old config file at {} is deprecated and will be renamed.'
                  .format(location_old))
            print('The new config file is located at {}.'
                  .format(filename_userconfig))
            Userconfig.migrate_userconfig(location_old, filename_userconfig)
            print('!!!!! MIGRATION DONE !!!!!\n\n')

        try:
            with open(filename_userconfig, 'r') as file_content:
                try:
                    Userconfig.config = yaml.load(file_content, Loader=yaml.FullLoader)
                except yaml.YAMLError as exc:
                    print("Error in configuration file:", exc)
        except (IOError, FileNotFoundError) as exc:
            print('Did not find userconfig file. Copying the distribution file.')
            Userconfig.config = Userconfig.config_fallback
            shutil.copy(filename_distribute, filename_userconfig)
        Userconfig.initialized = True

    # We need to define this here, as we need the base dir a lot.
    @classmethod
    def get_base_dir(cls):
        """Get the robot_folders base dir"""
        base_dir = os.environ['ROB_FOLDERS_BASE_DIR']
        return os.path.realpath(base_dir)


    @classmethod
    def migrate_entry(cls, section, value, section_module, yaml_dump):
        """Migrate a config entry from old python config system to yaml """
        if value in section_module:
            yaml_dump[section][value] = section_module[value]
            print('  {}.{}: {}'.format(section, value, section_module[value]))


    @classmethod
    def migrate_userconfig(cls, old_location, new_location):
        """Migration of old python module config to yaml config stucture"""
        legacy_config = imp.load_source('*', old_location)
        build_config = legacy_config.config
        dir_config = legacy_config.directories
        yaml_dump = {'build': dict(), 'directories': dict()}
        print('Migrating entries:')
        Userconfig.migrate_entry('build', 'generator', build_config, yaml_dump)
        Userconfig.migrate_entry('build', 'make_threads', build_config, yaml_dump)
        Userconfig.migrate_entry('build', 'install_ic', build_config, yaml_dump)
        Userconfig.migrate_entry('build', 'install_catkin', build_config, yaml_dump)
        Userconfig.migrate_entry('directories', 'checkout_dir', dir_config, yaml_dump)
        Userconfig.migrate_entry('directories', 'catkin_names', dir_config, yaml_dump)

        with open(new_location, mode='w') as out_stream:
            yaml.dump(yaml_dump, stream=out_stream, default_flow_style=False)
        filename_deprectated = os.path.join(Userconfig.get_base_dir(),
                                            '.robot_folders',
                                            'userconfig_deprecated_safecopy.py')
        os.rename(old_location, filename_deprectated)
        print('Moved old config file {} to backup location {}.'
              .format(old_location, filename_deprectated))
        print('You may safely delete this file, as it is no longer used.')




# This is the internal yaml query. It can be used for the modified
# or the distribution config
def _get_value_safe(dictionary, section, value, debug=True):
    result = None
    try:
        result = dictionary[section][value]
    except KeyError:
        if debug:
            print('Did not find key {}.{}!!!'.format(section, value))
    except TypeError:
        print('There is an illegal config. '
              'Probably something went wrong during initialization. '
              'Check your config file.')
    return result


# Interface funtion to the outside. You might want to consider the
def get_value_safe(section, value, debug=True):
    '''Retrieve an entry from the config backend.

    Will try to find the given entry in the config. If it is not
    found in the user-modified config, the default value from the
    distributed config will be used.
    If it is not found at all, a None object will be returned.

    - **parameters**, **types**, **return** and **return types**::
        :param section: The top level key for the searched entry
        :param value: The second level key for the searched entry
        :param debug: Print debug output when searching for the entry (default True)
        :type section: string
        :type value: string
        :type debug: bool
        :returns: config entry
        :rtype: mixed
    '''
    if not Userconfig.initialized:
        Userconfig.init_class()

    result = _get_value_safe(Userconfig.config, section, value, debug=False)
    if result is None:
        result = _get_value_safe(Userconfig.config_fallback, section, value, debug)
    return result


def get_value_safe_default(section, value, default, debug=True):
    '''Retrieve an entry from the config backend or use default value.

    Will try to find the given entry in the config. If it is not
    found in the user-modified config, the default value from the
    distributed config will be used.
    If it is not found at all, the default value will be returned.

    - **parameters**, **types**, **return** and **return types**::
        :param section: The top level key for the searched entry
        :param value: The second level key for the searched entry
        :param default: The default entry if not found
        :param debug: Print debug output when searching for the entry (default True)
        :type section: string
        :type value: string
        :type default: mixed, the same as return type
        :type debug: bool
        :returns: config entry
        :rtype: mixed
    '''
    result = get_value_safe(section, value, debug)
    if result is None:
        result = default

    return result
