"""Command to reveive the real checkout base directory"""
import click
from helpers.directory_helpers import get_checkout_dir


@click.command()
def cli():
    """Command implementation to get the checkout directory"""
    click.echo(get_checkout_dir())
