from setuptools import setup

setup(
    name='robot_folders',
    version='0.2',
    py_modules=['robot_folders'],
    python_requires='>=3',
    install_requires=[
        'Click>=7.0',
        'gitpython',
        'PyYaml',
        'wstool'
    ],
    entry_points='''
        [console_scripts]
        rob_folders=robot_folders:cli
    ''',
)
