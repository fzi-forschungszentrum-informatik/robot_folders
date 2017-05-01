import os
import shutil
import yaml


def get_base_dir():
    base_dir = os.environ['ROB_FOLDERS_BASE_DIR']
    return os.path.realpath(base_dir)


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
                                   'userconfigs.yaml')
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
