"""
This module contains helper functions around managing git repositories
"""
import vcstools


def parse_repository(repo_path):
    """
    Parses a repository path and returns the remote URL and the version (branch/commit)
    """
    client = vcstools.git.GitClient(repo_path)
    url = client.get_url()
    # If a tag is checked out (or the HEAD is detached for other reasons),
    # the current version label returns <detached>.
    # In that case we will use the SHA-ID of the checked out commit.
    version = client.get_current_version_label()
    if version == "<detached>":
        version = client.get_version()
    return url, version


def create_rosinstall_entry(repo_path, local_name):
    """
    Creates a rosinstall dict entry for a given repo path and local folder name
    """
    repo = dict()
    repo['git'] = dict()
    repo['git']['local-name'] = local_name

    url, version = parse_repository(repo_path)
    repo['git']['uri'] = url
    repo['git']['version'] = version
    return repo
