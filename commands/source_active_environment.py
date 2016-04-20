import click
from helpers.directory_helpers import get_active_env

@click.command()
def cli():
    """Sources the currently active environment."""
    click.echo("Active environment: {}".format(get_active_env()))
