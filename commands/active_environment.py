import click
from global_functions import get_active_env

@click.command()
def cli():
    """Prints out the current environment."""
    click.echo("Active environment: {}".format(get_active_env()))
