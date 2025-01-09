import base64
import os
from pathlib import Path

from github import Github
from datetime import datetime, timezone
from beachbot.config import logger


def download(
    local_file_path: Path,
    github_repo: str,
    github_path: str,
    branch_name: str,
    token: str = None,
    overwrite: bool = False,
) -> Path:
    """
    Checks if a file exists locally or in a specific branch of a GitHub repository and downloads it if necessary.
    :param local_file_path: The path to the file on the local system.
    :param overwrite: Whether to overwrite the local file if it already exists.
    :param github_repo: The GitHub repository in the form "owner/repo".
    :param github_path: The path to the file in the GitHub repo.
    :param branch_name: The branch name to fetch the file from.
    :param token: Optional GitHub token for authentication (if private repo or to avoid rate limits).
    :return Path: The local path to the downloaded file.
    """
    # Check if the file exists locally
    local_timestamp = None
    if os.path.exists(local_file_path):
        logger.info(f"File found locally: {local_file_path}")
        local_timestamp = datetime.fromtimestamp(os.path.getmtime(local_file_path), tz=timezone.utc)
        if not overwrite:
            logger.info("Skipping download as overwrite is set to False.")
            return local_file_path

    # Authenticate with GitHub
    g = Github(token) if token else Github()
    repo = g.get_repo(github_repo)

    try:
        # Get the file content from the specified branch
        file_content = repo.get_contents(github_path, ref=branch_name)
        # Only download if the file has been modified more recently than local version
        remote_timestamp = file_content.last_modified_datetime
        logger.info(f"remote_timestamp: {remote_timestamp}")
        logger.info("local_timestamp: {local_timestamp}")
        if local_timestamp is None or remote_timestamp > local_timestamp:
            with open(local_file_path, "wb") as f:
                sha = file_content.sha
                blob = repo.get_git_blob(sha)
                blob_bytes = base64.b64decode(blob.content)
                f.write(blob_bytes)
            logger.info(
                f"File downloaded from branch '{branch_name}' on GitHub to: {local_file_path}"
            )
            return local_file_path

        else:
            logger.info(
                f"File already exists locally and is up to date: {local_file_path}"
            )
            return local_file_path
    except Exception as e:
        logger.info(
            f"File not found on branch '{branch_name}' in GitHub repo: {github_repo}/{github_path}"
        )
        logger.error(f"Error: {e}")
        return None
