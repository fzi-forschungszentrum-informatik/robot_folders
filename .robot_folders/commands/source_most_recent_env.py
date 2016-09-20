import click
from helpers.directory_helpers import get_last_activated_env

@click.command()
def cli():
    """DEPRECATED, use change_environment without an argument!!! Sources the most recently activated environment."""
    click.echo("DEPRECATED!!! DEPRECATED!!! DEPRECATED!!!\n"
               "This command is deprecated and will be "
               "removed in future versions. Please consider the "
               "change_environment command. If this is used without "
               "an argument it sources the most recend environment.\n"
               "DEPRECATED!!! DEPRECATED!!! DEPRECATED!!!")
    click.echo("Active environment: {}".format(get_last_activated_env()))
