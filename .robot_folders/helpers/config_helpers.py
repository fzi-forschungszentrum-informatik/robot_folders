import os
import shutil
import yaml
import imp


def get_base_dir():
    base_dir = os.environ['ROB_FOLDERS_BASE_DIR']
    return os.path.realpath(base_dir)


# Migrate from old python config system to yaml
def migrate_entry(section, value, section_module, yaml_dump):
    if value in section_module:
        yaml_dump[section][value] = section_module[value]
        print('{}.{}: {}'.format(section, value, section_module[value]))


def migrate_userconfig(old_location, new_location):
    legacy_config = imp.load_source('*', old_location)
    build_config = legacy_config.config
    dir_config = legacy_config.directories
    yaml_dump = {'build': dict(), 'directories': dict()}
    print('Migrating entries:')
    migrate_entry('build', 'generator', build_config, yaml_dump)
    migrate_entry('build', 'make_threads', build_config, yaml_dump)
    migrate_entry('build', 'install_ic', build_config, yaml_dump)
    migrate_entry('build', 'install_catkin', build_config, yaml_dump)
    migrate_entry('directories', 'checkout_dir', dir_config, yaml_dump)
    migrate_entry('directories', 'catkin_names', dir_config, yaml_dump)

    out_stream = file(new_location, mode='w')
    yaml.dump(yaml_dump, stream=out_stream, default_flow_style=False)
    filename_deprectated = os.path.join(get_base_dir(),
                                        '.robot_folders',
                                        'userconfig_deprecated_safecopy.py')
    os.rename(old_location, filename_deprectated)
    print('Moved old config file {} to backup location {}.'
          .format(old_location, filename_deprectated))
    print('You may safely delete this file, as it is no longer used.')


# Load the distribution config file
filename_distribute = os.path.join(get_base_dir(),
                                   'config',
                                   'userconfig_distribute.yaml')
config_fallback = None
try:
    config_fallback = yaml.load(file(filename_distribute, 'r'))
except yaml.YAMLError, exc:
    print "Error in configuration file:", exc
except IOError, exc:
    print 'ERROR: There was a problem loading the distribution file:', exc

# Load the user-modified config file
filename_userconfig = os.path.join(get_base_dir(),
                                   'config',
                                   'userconfig.yaml')
# check for migration
location_old = os.path.join(get_base_dir(),
                            '.robot_folders',
                            'userconfig.py')
if os.path.isfile(location_old):
    print('!!!!! MIGRATION WARNING !!!!!')
    print('The config file format and location have changed.')
    print('The old config file at {} is deprecated and will be renamed.'
          .format(location_old))
    print('The new config file is located at {}.'
          .format(filename_userconfig))
    migrate_userconfig(location_old, filename_userconfig)
    print('!!!!! MIGRATION DONE !!!!!')

config = None
try:
    config = yaml.load(file(filename_userconfig, 'r'))
except yaml.YAMLError, exc:
    print "Error in configuration file:", exc
except IOError, exc:
    print 'Did not find userconfig file. Copying the distribution file'
    config = yaml.load(file(filename_distribute, 'r'))
    shutil.copy(filename_distribute, filename_userconfig)


def _get_value_safe(dictionary, section, value, debug=True):
    result = None
    try:
        result = dictionary[section][value]
    except KeyError:
        if debug:
            print 'Did not find key!!!'
    except TypeError:
        print('There is an illegal config. '
              'Probably something went wrong during initialization. '
              'Check your config file.')
    return result


def get_value_safe(section, value, debug=True):
    result = _get_value_safe(config, section, value, debug=False)
    if result is None:
        result = _get_value_safe(config_fallback, section, value, debug)
    return result


def get_value_safe_default(section, value, default, debug=True):
    result = get_value_safe(section, value, debug)
    if result is None:
        result = default

    return result
