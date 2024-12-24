from beachbot.config import config
from beachbot.utils.github import download


def test_github_download():
    # Create tmp directory
    tmp_dir = config.BEACHBOT_CACHE / "tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    local_path = tmp_dir / "README.md"
    # OR repo = "okinawa-ai-beach-robot/beach-cleaning-hardware"
    repo = config.BEACHBOT_HARDWARE_REPO
    # File path in the repo
    remote_path = "README.md"
    branch = "pr2"  # Specify the branch name
    github_token = None  # Provide your GitHub token here if needed
    download(local_path, repo, remote_path, branch, github_token)
    # Remove local file
    local_path.unlink()
