import os
from pathlib import Path
import logging
from platformdirs import PlatformDirs
import sys

# Initialize PlatformDirs with proper app name and author
_platform_dirs = PlatformDirs("beachbot", "okinawa-ai-beach-robot")

# Default paths based on PlatformDirs
default_paths = {
    "BEACHBOT_HOME": Path(_platform_dirs.user_data_dir),
    "BEACHBOT_CACHE": Path(_platform_dirs.user_cache_dir),
    "BEACHBOT_CONFIG": Path(_platform_dirs.user_config_dir),
    "BEACHBOT_LOGS": Path(_platform_dirs.user_log_dir),
    "BEACHBOT_MODELS": Path(_platform_dirs.user_cache_dir) / "models",
    "BEACHBOT_DATASETS": Path(_platform_dirs.user_cache_dir) / "datasets",
}


class Config:
    # Directly defining the path attributes as None to allow users to set them
    BEACHBOT_HOME = None
    BEACHBOT_CACHE = None
    BEACHBOT_CONFIG = None
    BEACHBOT_LOGS = None
    BEACHBOT_MODELS = None
    BEACHBOT_DATASETS = None

    # Github Repos
    BEACHBOT_HARDWARE_REPO = "okinawa-ai-beach-robot/beach-cleaning-hardware"

    _instance = None

    def __new__(cls):
        """Ensures only one instance of Config is created."""
        if cls._instance is None:
            cls._instance = super(Config, cls).__new__(cls)
            cls._instance.initialize_paths()
        return cls._instance
    
    def __str__(self):
        return "Beachbot path configuration:\n"  + "\n".join([f"\t{pk}:{getattr(self,pk)}" for pk in default_paths.keys()])

    def initialize_paths(self):
        """Initialize paths with environment variables or defaults."""
        # Use dict comprehension to initialize paths
        for path_key, default in default_paths.items():
            # Use getattr to allow modifying the class attributes later
            value = Path(
                os.getenv(path_key, str(default))
            )  # str(default) to ensure Path
            if not value.exists():
                value.mkdir(parents=True, exist_ok=True)
            setattr(self, path_key, value)


def setup_logger(config: Config = None):
    logger = logging.getLogger("beachbot")
    logger.setLevel(logging.INFO)

    # File handler
    log_path = config.BEACHBOT_LOGS / "beachbot.log"
    # Verify log file exists
    if not log_path.exists():
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.touch()
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.INFO)

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)

    # Formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger


config = Config()
logger = setup_logger(config)
