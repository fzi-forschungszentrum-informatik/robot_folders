"""
This module contains helper functions around managing git repositories
"""
import git
import click


def parse_repository(repo_path, use_commit_id):
    """
    Parses a repository path and returns the remote URL and the version (branch/commit)
    """
    repo = git.Repo(repo_path)
    remotes = repo.remotes
    choice = 0
    if len(remotes) > 1:
        click.echo("Found multiple remotes for repo {}.".format(repo_path))
        upstream_branch = repo.active_branch.tracking_branch()
        upstream_remote = upstream_branch.name.split('/')[0]
        default = None
        for index, remote in enumerate(remotes):
            click.echo("{}: {} ({})".format(index, remote.name, remote.url))
            if remote.name == upstream_remote:
                default = index
        valid_choice = -1
        while valid_choice < 0:
            choice = click.prompt("Which one do you want to use?",
                                  type=int,
                                  default=default,
                                  show_default=True)
            if choice >= 0 and choice < len(remotes):
                valid_choice = choice
            else:
                click.echo("Invalid choice: '{}'".format(choice))
        click.echo("Selected remote {} ({})".format(remotes[choice].name, remotes[choice].url))
    url = remotes[choice].url

    detached_head = repo.head.is_detached

    if detached_head or use_commit_id:
        version = repo.head.commit.hexsha
    else:
        version = repo.active_branch.name
    return url, version


def create_rosinstall_entry(repo_path, local_name, use_commit_id=False):
    """
    Creates a rosinstall dict entry for a given repo path and local folder name
    """
    repo = dict()
    repo['git'] = dict()
    repo['git']['local-name'] = local_name

    url, version = parse_repository(repo_path, use_commit_id)
    repo['git']['uri'] = url
    repo['git']['version'] = version
    return repo
