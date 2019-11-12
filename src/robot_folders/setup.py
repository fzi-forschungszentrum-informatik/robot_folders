from setuptools import setup

setup(
    name='robot_folders',
    version='0.1',
    py_modules=['robot_folders'],
    install_requires=[
        'Click',
        'gitpython',
        'PyYaml',
        'wstool'
    ],
    entry_points='''
        [console_scripts]
        rob_folders=robot_folders:cli
    ''',
)
