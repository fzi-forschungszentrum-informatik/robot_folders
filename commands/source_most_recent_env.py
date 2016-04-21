import click
from helpers.directory_helpers import get_last_activated_env

@click.command()
def cli():
    """Sources the most recently activated environment."""
    click.echo("Active environment: {}".format(get_last_activated_env()))
