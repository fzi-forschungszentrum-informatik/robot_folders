import os

def get_base_dir():
    helpers_path = os.path.dirname(os.path.realpath(__file__))
    return os.path.realpath(os.path.join(helpers_path, '..'))

def get_last_activated_env():
    env_file = os.path.join(get_base_dir(), 'checkout', '') + '.cur_env'

    if os.path.isfile(env_file):
        with open(env_file, 'r') as f:
            return f.read().rstrip()
    else:
        print "No recently activated environment found. Is this your first run?\
Try to add an environment and then do a change_environment to this.\n\
To get rid of this message, please source the most recent environment or\
change to another one."
        return None

def get_active_env():
    try:
        active_env = os.environ['ROB_FOLDERS_ACTIVE_ENV']
        return active_env
    except KeyError:
        print "Currently, there is no active environment! Instead, the\
most recently activated environment will be used!"
        return get_last_activated_env()

def get_active_env_path():
    if get_active_env() == None:
        return None
    return os.path.join(get_base_dir(), 'checkout', get_active_env())
