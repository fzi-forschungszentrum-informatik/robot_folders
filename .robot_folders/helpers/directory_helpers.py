import os
import errno
import config_helpers


def get_base_dir():
    base_dir = os.environ['ROB_FOLDERS_BASE_DIR']
    return os.path.realpath(base_dir)


def get_last_activated_env():
    env_file = os.path.join(get_checkout_dir(), '.cur_env')

    if os.path.isfile(env_file):
        with open(env_file, 'r') as f:
            return f.read().rstrip()
    else:
        print "No recently activated environment found. Is this your first run? \
Try to add an environment and then do a change_environment to this."
        return None


def get_active_env():
    try:
        active_env = os.environ['ROB_FOLDERS_ACTIVE_ENV']
        return active_env
    except KeyError:
        # print "Currently, there is no active environment!\n\
        # To get rid of this message, please source the most recent environment or \
        # change to another one."
        return None


def get_active_env_path():
    active_env = get_active_env()
    if active_env is None:
        active_env_fallback = get_last_activated_env()
        if active_env_fallback is None:
            return None
        active_env = active_env_fallback
    return os.path.join(get_checkout_dir(), active_env)

def mkdir_p(path):
    """Checks whether a directory exists, otherwise it will be created."""
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def recursive_rmdir(path):
    for i in os.listdir(path):
        item = os.path.join(path, i)
        if os.path.isdir(item):
            recursive_rmdir(item)
        else:
            os.remove(item)
    os.rmdir(path)


def get_checkout_dir():
    checkout_config = config_helpers.get_value_safe('directories',
                                                    'checkout_dir',
                                                    debug=False)
    if checkout_config == '' or checkout_config is None:
        return os.path.join(get_base_dir(), 'checkout')
    else:
        if not os.path.exists(checkout_config):
            mkdir_p(checkout_config)
        return checkout_config


def get_catkin_dir(env_dir=''):
    """Tries to find the right catkin workspace in the Currently \
    sourced environment."""

    path_found = False
    path = ''
    cur_env_path = env_dir
    if env_dir == '':
        cur_env_path = get_active_env_path()

    valid_names = config_helpers.get_value_safe_default(
        'directories', 'catkin_names', ["catkin_workspace"], debug=False)
    for path_name in valid_names:
        path = os.path.join(cur_env_path, path_name)
        if os.path.exists(path):
            return path
    # print "No catkin workspace was found in the current environment"
    return None
