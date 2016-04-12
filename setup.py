from setuptools import setup

setup(
    name='robot_folders',
    version='0.1',
    py_modules=['robot_folders'],
    install_requires=[
        'Click',
        'PyYaml'
    ],
    entry_points='''
        [console_scripts]
        fzrob=robot_folders:cli
    ''',
)
