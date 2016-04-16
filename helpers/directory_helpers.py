import os

def get_base_dir():
    helpers_path = os.path.dirname(os.path.realpath(__file__))
    return os.path.realpath(os.path.join(helpers_path, '..'))

def get_active_env():
    env_file = os.path.join(get_base_dir(), 'checkout', '') + '.cur_env'
    if os.path.isfile(env_file):
        with open(env_file, 'r') as f:
            return f.read().rstrip()
    else:
        return None

def get_active_env_path():
    if get_active_env() == None:
        return None
    return os.path.join(get_base_dir(), 'checkout', get_active_env())
