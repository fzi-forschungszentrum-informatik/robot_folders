import click
from helpers.directory_helpers import get_checkout_dir


@click.command()
def cli():
    click.echo(get_checkout_dir())
