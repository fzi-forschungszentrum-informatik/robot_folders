import os
import errno
import userconfig


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
        active_env = get_last_activated_env()
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


def get_checkout_dir():
    checkout_config = userconfig.directories.get('checkout_dir', '')
    if checkout_config == '':
        return os.path.join(get_base_dir(), 'checkout')
    else:
        if not os.path.exists(checkout_config):
            mkdir_p(checkout_config)
        return checkout_config
