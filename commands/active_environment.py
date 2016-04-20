import click
from helpers.directory_helpers import get_active_env

@click.command()
def cli():
    """Prints out the current environment. If none
is sourced right now, it tells which was the last active
environment, which will be sourced by simply calling the
source command."""

    active_env = get_active_env()
    click.echo("Active environment: {}".format(active_env))
