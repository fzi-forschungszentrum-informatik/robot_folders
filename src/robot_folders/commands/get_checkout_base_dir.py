"""Command to reveive the real checkout base directory"""
import click
from robot_folders.helpers.directory_helpers import get_checkout_dir


@click.command("get_checkout_base_dir")
def cli():
    """Command implementation to get the checkout directory"""
    click.echo(get_checkout_dir())
