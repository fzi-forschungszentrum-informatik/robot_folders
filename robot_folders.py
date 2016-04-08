import click

@click.group()
def cli():
    """A simple command line tool."""

@cli.command('add_environment', short_help='Add a new environment')
def add_environment():
    """Adds a new environment and creates the basic needed folders."""

@cli.command('delete', short_help='delete the repo')
def delete():
    """Deletes the repository."""

@cli.command()
@click.option('--count', default=1, help='Number of greetings.')
@click.option('--name', prompt='Your name', default="Nobody",
              help='The person to greet.')
def greet(count, name):
    """Simple program that greets NAME for a total of COUNT times."""
    for x in range(count):
        click.echo('Hello %s!' % name)

