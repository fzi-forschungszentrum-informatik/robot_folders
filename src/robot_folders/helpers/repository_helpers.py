#
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
"""
This module contains helper functions around managing git repositories
"""
import git
import click
from robot_folders.helpers.exceptions import ModuleException


def parse_repository(repo_path, use_commit_id):
    """
    Parses a repository path and returns the remote URL and the version (branch/commit)
    """
    repo = git.Repo(repo_path)
    remotes = repo.remotes
    choice = 0

    detached_head = repo.head.is_detached

    if len(remotes) > 1:
        click.echo("Found multiple remotes for repo {}.".format(repo_path))
        upstream_remote = None
        if not detached_head:
            upstream_branch = repo.active_branch.tracking_branch()
            if upstream_branch == None:
                raise ModuleException(
                    'Branch "{}" from repository "{}" does not have a tracking branch configured. Cannot scrape environment.'.format(
                        repo.active_branch, repo_path
                    ),
                    "repository_helpers",
                    1,
                )

            upstream_remote = upstream_branch.name.split("/")[0]
        default = None
        for index, remote in enumerate(remotes):
            click.echo("{}: {} ({})".format(index, remote.name, remote.url))
            if remote.name == upstream_remote:
                default = index
        valid_choice = -1
        while valid_choice < 0:
            choice = click.prompt(
                "Which one do you want to use?",
                type=int,
                default=default,
                show_default=True,
            )
            if choice >= 0 and choice < len(remotes):
                valid_choice = choice
            else:
                click.echo("Invalid choice: '{}'".format(choice))
        click.echo(
            "Selected remote {} ({})".format(remotes[choice].name, remotes[choice].url)
        )
    url = remotes[choice].url

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
    repo["git"] = dict()
    repo["git"]["local-name"] = local_name

    url, version = parse_repository(repo_path, use_commit_id)
    repo["git"]["uri"] = url
    repo["git"]["version"] = version
    return repo
