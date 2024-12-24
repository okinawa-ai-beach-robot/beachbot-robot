import base64
import os
from pathlib import Path

from github import Github


def download(
    local_file_path: Path,
    github_repo: str,
    github_path: str,
    branch_name: str,
    token: str = None,
):
    """
    Checks if a file exists locally or in a specific branch of a GitHub repository and downloads it if necessary.
    :param local_file_path: The path to the file on the local system.
    :param github_repo: The GitHub repository in the form "owner/repo".
    :param github_path: The path to the file in the GitHub repo.
    :param branch_name: The branch name to fetch the file from.
    :param token: Optional GitHub token for authentication (if private repo or to avoid rate limits).
    :return: True if the file exists (locally or downloaded), False otherwise.
    """
    # Check if the file exists locally
    if os.path.exists(local_file_path):
        print(f"File found locally: {local_file_path}")
        return True

    # Authenticate with GitHub
    g = Github(token) if token else Github()
    repo = g.get_repo(github_repo)

    try:
        # Get the file content from the specified branch
        file_content = repo.get_contents(github_path, ref=branch_name)
        with open(local_file_path, "wb") as f:
            sha = file_content.sha
            blob = repo.get_git_blob(sha)
            blob_bytes = base64.b64decode(blob.content)
            f.write(blob_bytes)
        print(
            f"File downloaded from branch '{branch_name}' on GitHub to: {local_file_path}"
        )
        return True
    except Exception as e:
        print(
            f"File not found on branch '{branch_name}' in GitHub repo: {github_repo}/{github_path}"
        )
        print(f"Error: {e}")
        return False
