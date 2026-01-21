"""
FEAGI Configuration Management

Handles default configuration generation and management for FEAGI SDK.
"""

from pathlib import Path
from typing import Optional
import logging

logger = logging.getLogger("feagi.config")


DEFAULT_CONFIG_CONTENT = """# FEAGI Configuration File
# This file is auto-generated. Modify as needed for your deployment.
#
# For more information, see: https://github.com/feagi/feagi/tree/main/docs

[api]
host = "127.0.0.1"  # Localhost only - secure by default, no firewall prompts. Override with FEAGI_API_HOST for network deployments
port = 8000

[websocket]
host = "127.0.0.1"  # Localhost only - secure by default, no firewall prompts. Override with FEAGI_WS_HOST for network deployments
enabled = true
visualization_port = 8080
sensory_port = 5558
motor_port = 5564

[burst_engine]
# Burst duration in seconds (0.01 = 10ms bursts)
burst_duration = 0.01

# Maximum bursts to execute (0 = unlimited)
max_bursts = 0

# Enable GPU acceleration if available
gpu_enabled = false

[performance]
# Number of worker threads for parallel processing
worker_threads = 4

# Enable performance profiling
profiling_enabled = false

[logging]
# Log level: debug, info, warning, error
level = "info"

# Enable file logging
file_logging = true

[connectome]
# Maximum neuron space allocation
neuron_space = 1000000

# Maximum synapse space allocation
synapse_space = 10000000
"""


def generate_default_config(output_path: Optional[Path] = None, force: bool = False) -> Path:
    """
    Generate a default FEAGI configuration file.
    
    Args:
        output_path: Path to write config file (default: ~/.feagi/config/feagi_configuration.toml)
        force: If True, overwrite existing file
    
    Returns:
        Path to created configuration file
    
    Raises:
        FileExistsError: If file exists and force=False
    
    Example:
        from feagi.config import generate_default_config
        
        # Generate in default location
        config_path = generate_default_config()
        
        # Generate in custom location
        config_path = generate_default_config(Path("./my_config.toml"))
        
        # Overwrite existing
        config_path = generate_default_config(force=True)
    """
    from feagi.paths import get_feagi_paths
    
    if output_path is None:
        paths = get_feagi_paths()
        paths.ensure_config_dir()
        output_path = paths.get_default_config()
    else:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    if output_path.exists() and not force:
        raise FileExistsError(
            f"Config file already exists: {output_path}\n"
            f"Use force=True to overwrite."
        )
    
    output_path.write_text(DEFAULT_CONFIG_CONTENT)
    logger.info(f"Generated default config: {output_path}")
    
    return output_path


def ensure_default_config() -> Path:
    """
    Ensure default configuration file exists, creating it if necessary.
    
    Returns:
        Path to configuration file (existing or newly created)
    
    Example:
        from feagi.config import ensure_default_config
        
        # Get or create default config
        config_path = ensure_default_config()
        print(f"Using config: {config_path}")
    """
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    config_path = paths.get_default_config()
    
    if not config_path.exists():
        paths.ensure_config_dir()
        config_path.write_text(DEFAULT_CONFIG_CONTENT)
        logger.info(f"Created default config: {config_path}")
    
    return config_path


def init_feagi_environment() -> dict:
    """
    Initialize FEAGI environment with all necessary directories and default config.
    
    Creates all standard directories and ensures default config exists.
    
    Returns:
        Dictionary with paths to key directories and files:
        {
            "config_dir": Path,
            "config_file": Path,
            "logs_dir": Path,
            "cache_dir": Path,
            "genomes_dir": Path,
            "connectomes_dir": Path
        }
    
    Example:
        from feagi.config import init_feagi_environment
        
        # Initialize everything
        env = init_feagi_environment()
        print(f"Config: {env['config_file']}")
        print(f"Genomes: {env['genomes_dir']}")
    """
    from feagi.paths import get_feagi_paths
    
    paths = get_feagi_paths()
    
    # Ensure all directories exist
    paths.ensure_all()
    
    # Ensure default config exists
    config_file = ensure_default_config()
    
    logger.info("FEAGI environment initialized")
    logger.info(f"  Config: {config_file}")
    logger.info(f"  Genomes: {paths.genomes_dir}")
    logger.info(f"  Connectomes: {paths.connectomes_dir}")
    
    return {
        "config_dir": paths.config_dir,
        "config_file": config_file,
        "logs_dir": paths.logs_dir,
        "cache_dir": paths.cache_dir,
        "genomes_dir": paths.genomes_dir,
        "connectomes_dir": paths.connectomes_dir,
    }
